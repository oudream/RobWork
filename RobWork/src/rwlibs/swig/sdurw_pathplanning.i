%module sdurw_pathplanning

%include <rwlibs/swig/swig_macros.i>
%include <rwlibs/swig/ext_i/rw_vector.i>
%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_proximity.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw_trajectory.i>

%import <rwlibs/swig/ext_i/std.i>

%{

    #include <rw/math/Vector2D.hpp>
    #include <rw/models/CompositeJointDevice.hpp>
    #include <rw/models/CompositeDevice.hpp>
    #include <rw/models/ParallelDevice.hpp>
    #include <rw/models/SerialDevice.hpp>
    #include <rw/models/MobileDevice.hpp>
    #include <rw/models/SE3Device.hpp>
    #include <rw/models/TreeDevice.hpp>


    #include <rw/models/Joint.hpp>
    #include <rw/models/PrismaticSphericalJoint.hpp>
    #include <rw/models/PrismaticUniversalJoint.hpp>
    #include <rw/models/SphericalJoint.hpp>
    #include <rw/models/UniversalJoint.hpp>
    #include <rw/models/VirtualJoint.hpp>
    #include <rw/models/DependentJoint.hpp>
    #include <rw/models/DependentRevoluteJoint.hpp>
    #include <rw/models/DependentPrismaticJoint.hpp>
    #include <rw/models/RevoluteJoint.hpp>
    #include <rw/models/PrismaticJoint.hpp>
    #include <rw/kinematics/MovableFrame.hpp>
    #include <rw/kinematics/FixedFrame.hpp>

    #include <rw/geometry/IndexedTriMesh.hpp>
    #include <rw/trajectory/Trajectory.hpp>
    #include <rw/trajectory/InterpolatorTrajectory.hpp>
	#include <rw/trajectory/TrajectorySequence.hpp>
    #include <rw/trajectory/LloydHaywardBlend.hpp>
    #include <rw/trajectory/ParabolicBlend.hpp>

%}
%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_trajectory.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_trajectory.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_trajectory.*;
%}

%nodefaultctor StopCriteria;
%{
	#include <rw/pathplanning/StopCriteria.hpp>
%}
%include <rw/pathplanning/StopCriteria.hpp>
NAMED_ABSTRACTPTR(StopCriteria,rw::pathplanning::StopCriteria);
%std_vector(VectorStopCriteriaPtr,rw::core::Ptr<rw::pathplanning::StopCriteria> );


%include <rwlibs/swig/typemaps/ConstQConstraintPtr.i>
%{
	#include <rw/pathplanning/QConstraint.hpp>
%}
%include <rw/pathplanning/QConstraint.hpp>
NAMED_OWNEDPTR(QConstraint,rw::pathplanning::QConstraint);
%std_vector(VectorQConstraintPtr,rw::core::Ptr<rw::pathplanning::QConstraint> );

%{
	#include <rw/pathplanning/QEdgeConstraint.hpp>
%}
%include <rw/pathplanning/QEdgeConstraint.hpp>
NAMED_OWNEDPTR(QEdgeConstraint,rw::pathplanning::QEdgeConstraint);
%std_vector(VectorQEdgeConstraintPtr,rw::core::Ptr<rw::pathplanning::QEdgeConstraint> );

%{
	#include <rw/pathplanning/PlannerConstraint.hpp>
%}
%include <rw/pathplanning/PlannerConstraint.hpp>
NAMED_OWNEDPTR(PlannerConstraint,rw::pathplanning::PlannerConstraint);

%{
	#include <rw/pathplanning/QEdgeConstraintIncremental.hpp>
%}
%include <rw/pathplanning/QEdgeConstraintIncremental.hpp>
%template (QEdgeConstraintIncrementalPtr) rw::core::Ptr<rw::pathplanning::QEdgeConstraintIncremental>;

%{
	#include <rw/pathplanning/QSampler.hpp>
%}
%include <rw/pathplanning/QSampler.hpp>
NAMED_OWNEDPTR(QSampler,rw::pathplanning::QSampler);


%nodefaultctor PathPlanner;
%{
	#include <rw/pathplanning/PathPlanner.hpp>
%}
%include <rw/pathplanning/PathPlanner.hpp>

%template(PathPlannerQQ) rw::pathplanning::PathPlanner<rw::math::Q,rw::math::Q const>;
%template(PathPlannerQTransform3D) rw::pathplanning::PathPlanner<rw::math::Q,rw::math::Transform3D<double> const>;
%template(PathPlannerQQSampler) rw::pathplanning::PathPlanner<rw::math::Q, rw::pathplanning::QSampler>;

%nodefaultctor QToQPlanner;
%{
	#include <rw/pathplanning/QToQPlanner.hpp>
%}
%include <rw/pathplanning/QToQPlanner.hpp>
NAMED_ABSTRACTPTR(QToQPlanner,rw::pathplanning::QToQPlanner);

%nodefaultctor QToTPlanner;
%{
	#include <rw/pathplanning/QToTPlanner.hpp>
%}
%include <rw/pathplanning/QToTPlanner.hpp>
NAMED_ABSTRACTPTR(QToTPlanner,rw::pathplanning::QToTPlanner);
%extend rw::pathplanning::QToTPlanner{
    rw::core::Ptr<rw::trajectory::Path< rw::math::Q > > query(rw::math::Q from, rw::math::Transform3D<double> to, rw::core::Ptr<StopCriteria> stop){
        rw::core::Ptr<rw::trajectory::Path< rw::math::Q > > path = rw::core::ownedPtr(new rw::trajectory::Path< rw::math::Q >());
        $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Transform3D<double> >::query(from,to,*path,*stop);
        return path;
    }

    rw::core::Ptr<rw::trajectory::Path< rw::math::Q > > query(rw::math::Q from, rw::math::Transform3D<double> to, double time){
        rw::core::Ptr<rw::trajectory::Path< rw::math::Q > > path = rw::core::ownedPtr(new rw::trajectory::Path< rw::math::Q >());
        $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Transform3D<double> >::query(from,to,*path,time);
        return path;
    }

    rw::core::Ptr<rw::trajectory::Path< rw::math::Q > > query(rw::math::Q from, rw::math::Transform3D<double> to){
        rw::core::Ptr<rw::trajectory::Path< rw::math::Q > > path = rw::core::ownedPtr(new rw::trajectory::Path< rw::math::Q >());
        $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Transform3D<double> >::query(from,to,*path);
        return path;
    }

    rw::core::PropertyMap& getProperties(){
        return $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Transform3D<double> >::getProperties();
    }
}	



%nodefaultctor QToQSamplerPlanner;
%{
	#include <rw/pathplanning/QToQSamplerPlanner.hpp>
%}
%include <rw/pathplanning/QToQSamplerPlanner.hpp>
NAMED_OWNEDPTR(QToQSamplerPlanner,rw::pathplanning::QToQSamplerPlanner);

%{
	#include <rw/pathplanning/PlannerUtil.hpp>
%}
%include <rw/pathplanning/PlannerUtil.hpp>

%{
	#include <rw/pathplanning/QNormalizer.hpp>
%}
%include <rw/pathplanning/QNormalizer.hpp>
NAMED_OWNEDPTR(QNormalizer,rw::pathplanning::QNormalizer);

%{
	#include <rw/pathplanning/QIKSampler.hpp>
%}
%include <rw/pathplanning/QIKSampler.hpp>
NAMED_ABSTRACTPTR(QIKSampler,rw::pathplanning::QIKSampler);

%{
	#include <rw/pathplanning/StateConstraint.hpp>
%}
%include <rw/pathplanning/StateConstraint.hpp>
NAMED_ABSTRACTPTR(StateConstraint,rw::pathplanning::StateConstraint);
%std_vector(VectorStateConstraintPtr,rw::core::Ptr<rw::pathplanning::StateConstraint> );

%{
	#include <rw/pathplanning/PathAnalyzer.hpp>
%}
%include <rw/pathplanning/PathAnalyzer.hpp>
