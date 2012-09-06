/*
 * FrameCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformAddons.hpp"

#include "DeviceCalibration.hpp"
#include <Eigen/Geometry>
#include <rw/kinematics.hpp>
#include <rw/models.hpp>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class FixedFrameCalibration: public DeviceCalibration {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef rw::common::Ptr<FixedFrameCalibration> Ptr;

	FixedFrameCalibration(rw::kinematics::FixedFrame::Ptr frame, bool isPreCorrection = true, const Eigen::Affine3d& correction = Eigen::Affine3d::Identity());

	virtual ~FixedFrameCalibration();

	virtual bool isEnabled() const;

	virtual void apply();

	virtual void revert();

	virtual void correct(rw::kinematics::State& state);

	virtual bool isApplied() const;

	rw::kinematics::FixedFrame::Ptr getFrame() const;

	bool isPreCorrection() const;

	Eigen::Affine3d getCorrection() const;

	void correct(const Eigen::Affine3d& correction);

	QDomElement toXml(QDomDocument& document);

	static FixedFrameCalibration::Ptr fromXml(const QDomElement& element, rw::kinematics::StateStructure::Ptr stateStructure);

private:
	bool _isEnabled;
	bool _isApplied;
	rw::kinematics::FixedFrame::Ptr _frame;
	bool _isPreCorrection;
	Eigen::Affine3d _correction;
};

}
}

#endif /* RWLIBS_CALIBRATION_FRAMECALIBRATION_HPP_ */