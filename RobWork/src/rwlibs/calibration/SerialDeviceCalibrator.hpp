/*
 * SerialDeviceCalibrator.hpp
 *
 *  Created on: Feb 15, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP
#define RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/TransformAddons.hpp"

#include "nlls/NLLSSystem.hpp"
#include "nlls/NLLSSolverLog.hpp"
#include "Calibration.hpp"
#include "SerialDevicePoseMeasurement.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class SerialDeviceCalibrator: public NLLSSystem {
public:
	typedef rw::common::Ptr<SerialDeviceCalibrator> Ptr;

	SerialDeviceCalibrator(rw::models::SerialDevice::Ptr device, rw::kinematics::Frame::Ptr referenceFrame,
			rw::kinematics::Frame::Ptr measurementFrame, Calibration::Ptr calibration);

	virtual ~SerialDeviceCalibrator();

	rw::kinematics::Frame::Ptr getReferenceFrame() const;

	void setReferenceFrame(rw::kinematics::Frame::Ptr referenceFrame);

	rw::kinematics::Frame::Ptr getMeasurementFrame() const;

	void setMeasurementFrame(rw::kinematics::Frame::Ptr measurementFrame);

	Calibration::Ptr getCalibration() const;

	unsigned int getMinimumMeasurementCount() const;

	const std::vector<SerialDevicePoseMeasurement::Ptr>& getMeasurements() const;

	void addMeasurement(SerialDevicePoseMeasurement::Ptr measurement);

	void addMeasurement(const rw::math::Q& q, const Pose6D<double>& pose, const Eigen::Matrix<double, 6, 6>& covarianceMatrix =
			Eigen::Matrix<double, 6, 6>::Identity());

	void setMeasurements(const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements);

	bool isWeightingEnabled() const;

	void setWeightingEnabled(bool isWeightingEnabled);

	NLLSSolverLog::Ptr getSolverLog() const;

	void calibrate(const rw::kinematics::State& state);

	Eigen::MatrixXd getCovarianceMatrix() const;

	virtual void computeJacobian(Eigen::MatrixXd& jacobian);

	void computeJacobian(Eigen::MatrixXd& jacobian, const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements);

	virtual void computeResiduals(Eigen::VectorXd& residuals);

	void computeResiduals(Eigen::VectorXd& residuals, const std::vector<SerialDevicePoseMeasurement::Ptr>& measurements);

	virtual void takeStep(const Eigen::VectorXd& step);

private:
	rw::models::SerialDevice::Ptr _device;
	rw::kinematics::State _state;
	rw::kinematics::Frame::Ptr _referenceFrame;
	rw::kinematics::Frame::Ptr _measurementFrame;
	Calibration::Ptr _calibration;
	std::vector<SerialDevicePoseMeasurement::Ptr> _measurements;
	bool _isWeightingEnabled;
	NLLSSolverLog::Ptr _solverLog;
	Eigen::MatrixXd _covarianceMatrix;
};

}
}

#endif /* RWLIBS_CALIBRATION_SERIALDEVICECALIBRATOR_HPP */
