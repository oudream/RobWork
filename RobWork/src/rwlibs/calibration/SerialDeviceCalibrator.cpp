/*
* SerialDeviceCalibrator.cpp
*
*  Created on: Feb 27, 2012
*      Author: bing
*/

#include "SerialDeviceCalibrator.hpp"

#include "nlls/NLLSNewtonSolver.hpp"
#include <Eigen/Eigenvalues>

namespace rwlibs {
	namespace calibration {

		class SerialDeviceCalibrationSystem: public NLLSSystem {
		public:
			typedef rw::common::Ptr<SerialDeviceCalibrationSystem> Ptr;

			SerialDeviceCalibrationSystem(rw::models::SerialDevice::Ptr device, rw::kinematics::State state, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const std::vector<SerialDevicePoseMeasurement>& measurements, Calibration::Ptr calibration, Jacobian::Ptr jacobian, bool isWeighting) : _device(device), _state(state), _referenceFrame(referenceFrame), _measurementFrame(measurementFrame), _measurements(measurements), _calibration(calibration), _jacobian(jacobian), _isWeighting(isWeighting) {

			}

			virtual ~SerialDeviceCalibrationSystem() {

			}

		private:
			virtual void computeJacobian(Eigen::MatrixXd& stackedJacobians) {
				const int measurementCount = _measurements.size();
				const int rowCount = 6 * measurementCount;
				const int columnCount = _jacobian->getColumnCount();
				stackedJacobians.resize(rowCount, columnCount);
				for (int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
					const SerialDevicePoseMeasurement measurement = _measurements[measurementIndex];

					// Update state according to current measurement.
					const rw::math::Q q = measurement.getQ();
					_device->setQ(q, _state);

					// Compute Jacobian.
					const int rowIndex = 6 * measurementIndex;
					stackedJacobians.block(rowIndex, 0, 6, columnCount) = _jacobian->computeJacobian(_referenceFrame, _measurementFrame, _state);

					// Weight Jacobian according to covariances.
					if (_isWeighting && measurement.hasCovarianceMatrix()) {
						const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(
							measurement.getCovarianceMatrix()).operatorInverseSqrt();
						stackedJacobians.block(rowIndex, 0, 6, columnCount) = weightMatrix
							* stackedJacobians.block(rowIndex, 0, 6, columnCount);
					}
				}
			}

			virtual void computeResiduals(Eigen::VectorXd& stackedResiduals) {
				const int measurementCount = _measurements.size();

				const int rowCount = 6 * measurementCount;
				stackedResiduals.resize(6 * measurementCount);
				for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
					const SerialDevicePoseMeasurement measurement = _measurements[measurementIndex];

					// Update state according to current measurement.
					const rw::math::Q q = measurement.getQ();
					_device->setQ(q, _state);

					// Compute residuals.
					const int rowIndex = 6 * measurementIndex;
					const rw::math::Transform3D<> tfmMeasurement = measurement.getTransform();
					const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(_referenceFrame.get(), _measurementFrame.get(), _state);
					const rw::math::Vector3D<> dP = tfmModel.P() - tfmMeasurement.P();
					stackedResiduals(rowIndex + 0) = dP(0);
					stackedResiduals(rowIndex + 1) = dP(1);
					stackedResiduals(rowIndex + 2) = dP(2);
					const rw::math::Rotation3D<> dR = tfmModel.R() * rw::math::inverse(tfmMeasurement.R());
					stackedResiduals(rowIndex + 3) = (dR(2, 1) - dR(1, 2)) / 2;
					stackedResiduals(rowIndex + 4) = (dR(0, 2) - dR(2, 0)) / 2;
					stackedResiduals(rowIndex + 5) = (dR(1, 0) - dR(0, 1)) / 2;

					// Weight residuals according to covariances.
					if (_isWeighting && measurement.hasCovarianceMatrix()) {
						const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(
							measurement.getCovarianceMatrix()).operatorInverseSqrt();
						stackedResiduals.segment<6>(rowIndex) = weightMatrix * stackedResiduals.segment<6>(rowIndex);
					}
				}
			}

			virtual void takeStep(const Eigen::VectorXd& step) {
				_jacobian->takeStep(step);
			}

		private:
			rw::models::SerialDevice::Ptr _device;
			rw::kinematics::State _state;
			rw::kinematics::Frame::Ptr _referenceFrame;
			rw::kinematics::Frame::Ptr _measurementFrame;
			std::vector<SerialDevicePoseMeasurement> _measurements;
			Calibration::Ptr _calibration;
			Jacobian::Ptr _jacobian;
			bool _isWeighting;
		};

		SerialDeviceCalibrator::SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device,
			rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, Calibration::Ptr calibration, Jacobian::Ptr jacobian) :
		_device(device), _referenceFrame(referenceFrame), _measurementFrame(measurementFrame), _calibration(calibration), _jacobian(jacobian), _isWeighting(true) {

		}

		SerialDeviceCalibrator::~SerialDeviceCalibrator() {

		}

		rw::models::SerialDevice::Ptr SerialDeviceCalibrator::getDevice() const {
			return _device;
		}

		rw::kinematics::Frame::Ptr SerialDeviceCalibrator::getReferenceFrame() const {
			return _referenceFrame;
		}

		void SerialDeviceCalibrator::setReferenceFrame(rw::kinematics::Frame::Ptr referenceFrame) {
			_referenceFrame = referenceFrame;
		}

		rw::kinematics::Frame::Ptr SerialDeviceCalibrator::getMeasurementFrame() const {
			return _measurementFrame;
		}

		void SerialDeviceCalibrator::setMeasurementFrame(rw::kinematics::Frame::Ptr measurementFrame) {
			_measurementFrame = measurementFrame;
		}

		unsigned int SerialDeviceCalibrator::getMinimumMeasurementCount() const {
			return ceil(float(_jacobian->getColumnCount()) / 6);
		}

		int SerialDeviceCalibrator::getMeasurementCount() const {
			return _measurements.size();
		}

		void SerialDeviceCalibrator::addMeasurement(const SerialDevicePoseMeasurement& measurement) {
			_measurements.push_back(measurement);
		}

		void SerialDeviceCalibrator::addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covarianceMatrix) {
			_measurements.push_back(SerialDevicePoseMeasurement(q, transform, covarianceMatrix));
		}

		void SerialDeviceCalibrator::setMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements) {
			_measurements = measurements;
		}

		bool SerialDeviceCalibrator::isWeighting() const {
			return _isWeighting;
		}

		void SerialDeviceCalibrator::setWeighting(bool isWeighting) {
			_isWeighting = isWeighting;
		}

		void SerialDeviceCalibrator::calibrate(const rw::kinematics::State& state) {
			RW_ASSERT(_calibration->isEnabled());

			const int measurementCount = getMeasurementCount();
			const int minimumMeasurementCount = getMinimumMeasurementCount();
			if (measurementCount < minimumMeasurementCount)
				RW_THROW(measurementCount << " measurements was provided but " << minimumMeasurementCount << " is required.");

			// Apply calibration if not applied.
			const bool wasApplied = _calibration->isApplied();
			if (!wasApplied)
				_calibration->apply();

			// Solve non-linear least square system.
			try {
				SerialDeviceCalibrationSystem::Ptr system = rw::common::ownedPtr(new SerialDeviceCalibrationSystem(_device, state, _referenceFrame, _measurementFrame, _measurements, _calibration, _jacobian, _isWeighting));
				_solver = rw::common::ownedPtr(new NLLSNewtonSolver(system));
				_solver->solve();
			} catch (rw::common::Exception& exception) {
				// Revert calibration if it was not applied.
				if (!wasApplied)
					_calibration->revert();

				// Re-trow exception.
				throw exception;
			}

			// Revert calibration if it was not applied.
			if (!wasApplied)
				_calibration->revert();
		}

		NLLSSolver::Ptr SerialDeviceCalibrator::getSolver() const {
			RW_ASSERT(!_solver.isNull());
			return _solver;
		}

		Eigen::MatrixXd SerialDeviceCalibrator::estimateCovarianceMatrix() const {
			RW_ASSERT(!_solver.isNull());
			return _solver->estimateCovarianceMatrix();
		}

	}
}
