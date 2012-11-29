/*
 * FixedFrameCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP_

#include "CalibrationBase.hpp"
#include <rw/kinematics.hpp>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class FixedFrameCalibration: public CalibrationBase {
public:
	enum PARAMETER {
		PARAMETER_X = 0,
		PARAMETER_Y = 1,
		PARAMETER_Z = 2,
		PARAMETER_ROLL = 3,
		PARAMETER_PITCH = 4,
		PARAMETER_YAW = 5
	};

	typedef rw::common::Ptr<FixedFrameCalibration> Ptr;

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame);

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection);

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPostCorrection, const rw::math::Transform3D<>& correctionTransform);

	virtual ~FixedFrameCalibration();

	rw::kinematics::FixedFrame::Ptr getFrame() const;

	bool isPostCorrection() const;

	rw::math::Transform3D<> getCorrectionTransform() const;

	void setCorrectionTransform(const rw::math::Transform3D<>& transform);

private:
	virtual void doApply();

	virtual void doRevert();

private:
	rw::kinematics::FixedFrame::Ptr _frame;
	bool _isPostCorrection;
	rw::math::Transform3D<> _originalTransform;
};

}
}

#endif /* RWLIBS_CALIBRATION_FIXEDFRAMECALIBRATION_HPP_ */
