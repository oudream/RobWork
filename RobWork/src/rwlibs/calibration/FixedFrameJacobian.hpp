/*
 * FixedFrameJacobian.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_

#include "DeviceJacobian.hpp"
#include "FixedFrameCalibration.hpp"

namespace rwlibs {
namespace calibration {

class FixedFrameJacobian: public DeviceJacobian {
public:
	typedef rw::common::Ptr<FixedFrameJacobian> Ptr;

	FixedFrameJacobian(FixedFrameCalibration::Ptr calibration);

	virtual DeviceCalibration::Ptr getCalibration() const;

	virtual int getParameterCount() const;

	virtual Eigen::MatrixXd compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const rw::kinematics::State& state);

	virtual void step(const Eigen::VectorXd& step);

	void setEnabledParameters(bool x, bool y, bool z, bool roll, bool pitch, bool yaw);

private:
	FixedFrameCalibration::Ptr _calibration;
	Eigen::Matrix<int, 6, 1> _enabledParameters;
};

}
}

#endif /* RWLIBS_CALIBRATION_FIXEDFRAMEJACOBIAN_HPP_ */