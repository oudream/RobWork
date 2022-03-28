%module sdurw_trajectory

%include <rwlibs/swig/swig_macros.i>
%include <rwlibs/swig/ext_i/rw_vector.i>
%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/ext_i/std.i>

%exception {
    try {
        //printf("Entering function : $name\n"); // uncomment to get a print out of all function calls
        $action
    }catch(rw::core::Exception& e ){
        SWIG_exception(SWIG_RuntimeError,e.what());
    }catch(...){
        SWIG_exception(SWIG_RuntimeError,"unknown error");
    }
}

%pragma(java) jniclassimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_models.*;
	import org.robwork.sdurw_kinematics.*;
	import org.robwork.sdurw_common.*;
%}
%pragma(java) moduleimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_models.*;
	import org.robwork.sdurw_kinematics.*;
	import org.robwork.sdurw_common.*;
	
%}
%typemap(javaimports) SWIGTYPE %{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_models.*;
	import org.robwork.sdurw_kinematics.*;
	import org.robwork.sdurw_common.*;
%}
%{
	#include <rw/math/Vector2D.hpp>
	#include <rw/math/Vector3D.hpp>
	#include <rw/math/Rotation3D.hpp>
	#include <rw/math/Transform3D.hpp>
	#include <rw/math/Q.hpp>
	#include <rw/math/Transform3DVector.hpp>
	#include <rw/core/Ptr.hpp>

	#include <rw/models/JointDevice.hpp>
	#include <rw/models/SE3Device.hpp>
	#include <rw/models/ParallelDevice.hpp>
	#include <rw/models/SerialDevice.hpp>
	#include <rw/models/CompositeDevice.hpp>
	#include <rw/models/CompositeJointDevice.hpp>
	#include <rw/models/TreeDevice.hpp>
	#include <rw/models/MobileDevice.hpp>

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

	#include <rw/trajectory/TrajectorySequence.hpp>		//Ken addon
	#include <rw/trajectory.hpp>
	#include <vector>
%}


#define NAME_R1(x) x ## R1
#define NAME_R2(x) x ## R2
#define NAME_R3(x) x ## R3
#define NAME_SO3(x) x ## SO3
#define NAME_SE3(x) x ## SE3
#define NAME_Q(x) x ## Q
#define NAME_FLOAT(x) x ## _f
#define NAME_DOUBLE(x) x ## _d
#define NAME_V3D(x) x ## Vector3D
#define NAME_V2D(x) x ## Vector2D
#define NAME_R3D(x) x ## Rotation3D
#define NAME_T3D(x) x ## Transform3D
#define NAME_T3DV(x) x ## Transform3DVector
#define NAME_QUAT(x) x ## Quaternion

%define ADD_TRAJECTORY_STANDARD_TEMPLATE(name,template_type)
	
	%template(NAME_DOUBLE(name)) template_type<double>;
	%template(NAME_V2D(name)) template_type<rw::math::Vector2D<double>>;
	%template(NAME_V3D(name)) template_type<rw::math::Vector3D<double>>;
	%template(NAME_R3D(name)) template_type<rw::math::Rotation3D<double>>;
	%template(NAME_T3D(name)) template_type<rw::math::Transform3D<double>>;
	%template(NAME_Q(name)) template_type<rw::math::Q>;

	%template(NAME_FLOAT(NAME_DOUBLE(name))) template_type<float>;
	%template(NAME_FLOAT(NAME_V2D(name))) template_type<rw::math::Vector2D<float>>;
	%template(NAME_FLOAT(NAME_V3D(name))) template_type<rw::math::Vector3D<float>>;
	%template(NAME_FLOAT(NAME_R3D(name))) template_type<rw::math::Rotation3D<float>>;
	%template(NAME_FLOAT(NAME_T3D(name))) template_type<rw::math::Transform3D<float>>;

	NAMED_OWNEDPTR(NAME_DOUBLE(name),template_type<double> );
	NAMED_OWNEDPTR(NAME_V2D(name),template_type<rw::math::Vector2D<double> > );
	NAMED_OWNEDPTR(NAME_V3D(name),template_type<rw::math::Vector3D<double> > );
	NAMED_OWNEDPTR(NAME_R3D(name),template_type<rw::math::Rotation3D<double> > );
	NAMED_OWNEDPTR(NAME_T3D(name),template_type<rw::math::Transform3D<double> > );
	NAMED_OWNEDPTR(NAME_Q(name),template_type< rw::math::Q > );

	NAMED_OWNEDPTR(NAME_FLOAT(NAME_DOUBLE(name)),template_type<float> );
	NAMED_OWNEDPTR(NAME_FLOAT(NAME_V2D(name)),template_type<rw::math::Vector2D<float> > );
	NAMED_OWNEDPTR(NAME_FLOAT(NAME_V3D(name)),template_type<rw::math::Vector3D<float> > );
	NAMED_OWNEDPTR(NAME_FLOAT(NAME_R3D(name)),template_type<rw::math::Rotation3D<float> > );
	NAMED_OWNEDPTR(NAME_FLOAT(NAME_T3D(name)),template_type<rw::math::Transform3D<float> > );

	ADD_DEFINITION(NAME_DOUBLE(name),NAME_R1(name),sdurw_trejectory);
	ADD_DEFINITION(NAME_V2D(name),NAME_R2(name),sdurw_trejectory);
	ADD_DEFINITION(NAME_V3D(name),NAME_R3(name),sdurw_trejectory);
	ADD_DEFINITION(NAME_R3D(name),NAME_SO3(name),sdurw_trejectory);
	ADD_DEFINITION(NAME_T3D(name),NAME_SE3(name),sdurw_trejectory);
%enddef

%include <rwlibs/swig/typemaps/blendPtr.i>
%{
	#include <rw/trajectory/Blend.hpp>
%}
%include <rw/trajectory/Blend.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(Blend,rw::trajectory::Blend);


%include <rwlibs/swig/typemaps/interpolatorPtr.i>
%{
	#include <rw/trajectory/Interpolator.hpp>
%}
%include <rw/trajectory/Interpolator.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(Interpolator,rw::trajectory::Interpolator);
%template(InterpolatorTransform3DVector) rw::trajectory::Interpolator<rw::math::Transform3DVector<double>>;

%{
	#include <rw/trajectory/Trajectory.hpp>
%}
%include <rw/trajectory/Trajectory.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(Trajectory,rw::trajectory::Trajectory);


%template(NAME_FLOAT(NAME_T3DV(Trajectory))) rw::trajectory::Trajectory<rw::math::Transform3DVector<float>>;
%template(NAME_T3DV(Trajectory)) rw::trajectory::Trajectory<rw::math::Transform3DVector<double>>;
%template(NAME_QUAT(Trajectory)) rw::trajectory::Trajectory<rw::math::Quaternion<double>>;
%template(NAME_FLOAT(NAME_QUAT(Trajectory))) rw::trajectory::Trajectory<rw::math::Quaternion<float>>;

%template (TrajectoryState) rw::trajectory::Trajectory<rw::kinematics::State>;
%template (TrajectoryStatePtr) rw::core::Ptr<rw::trajectory::Trajectory<rw::kinematics::State> >;
OWNEDPTR(rw::trajectory::Trajectory<rw::kinematics::State> );

%{
	#include <rw/trajectory/InterpolatorTrajectory.hpp>
%}
%include <rw/trajectory/InterpolatorTrajectory.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(InterpolatorTrajectory,rw::trajectory::InterpolatorTrajectory);

%template(NAME_FLOAT(NAME_T3DV(InterpolatorTrajectory))) rw::trajectory::InterpolatorTrajectory<rw::math::Transform3DVector<float>>;
%template(NAME_T3DV(InterpolatorTrajectory)) rw::trajectory::InterpolatorTrajectory<rw::math::Transform3DVector<double>>;
%template(NAME_QUAT(InterpolatorTrajectory)) rw::trajectory::InterpolatorTrajectory<rw::math::Quaternion<double>>;
%template(NAME_FLOAT(NAME_QUAT(InterpolatorTrajectory))) rw::trajectory::InterpolatorTrajectory<rw::math::Quaternion<float>>;


NAMED_OWNEDPTR(NAME_FLOAT(NAME_T3DV(InterpolatorTrajectory)),rw::trajectory::InterpolatorTrajectory<rw::math::Transform3DVector<float> > );
NAMED_OWNEDPTR(NAME_T3DV(InterpolatorTrajectory),rw::trajectory::InterpolatorTrajectory<rw::math::Transform3DVector<double> > );
NAMED_OWNEDPTR(NAME_QUAT(InterpolatorTrajectory), rw::trajectory::InterpolatorTrajectory<rw::math::Quaternion<double>>);
NAMED_OWNEDPTR(NAME_FLOAT(NAME_QUAT(InterpolatorTrajectory)), rw::trajectory::InterpolatorTrajectory<rw::math::Quaternion<float>>);

%template(NAME_DOUBLE(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< double > >, rw::core::Ptr< rw::trajectory::Interpolator< double > > >;
%template(NAME_V2D(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Vector2D<double> > >, rw::core::Ptr< rw::trajectory::Interpolator< rw::math::Vector2D<double> > > >;
%template(NAME_V3D(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Vector3D<double> > >, rw::core::Ptr< rw::trajectory::Interpolator< rw::math::Vector3D<double> > > >;
%template(NAME_R3D(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Rotation3D<double> > >, rw::core::Ptr< rw::trajectory::Interpolator< rw::math::Rotation3D<double> > > >;
%template(NAME_T3D(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Transform3D<double> > >, rw::core::Ptr< rw::trajectory::Interpolator< rw::math::Transform3D<double> > > >;
%template(NAME_T3DV(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Transform3DVector<double> > >, rw::core::Ptr< rw::trajectory::Interpolator< rw::math::Transform3DVector<double> > > >;
%template(NAME_QUAT(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Quaternion<double> > >, rw::core::Ptr< rw::trajectory::Interpolator< rw::math::Quaternion<double> > > >;
%template(NAME_Q(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Q > >, rw::core::Ptr< rw::trajectory::Interpolator<  rw::math::Q  > > >;

%template(NAME_FLOAT(Segment)) std::pair< rw::core::Ptr< rw::trajectory::Blend< float > >, rw::core::Ptr< rw::trajectory::Interpolator< float > > >;
%template(NAME_FLOAT(NAME_V2D(Segment))) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Vector2D<float> > >, rw::core::Ptr< rw::trajectory::Interpolator<  rw::math::Vector2D<float> > > >;
%template(NAME_FLOAT(NAME_V3D(Segment))) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Vector3D<float> > >, rw::core::Ptr< rw::trajectory::Interpolator<  rw::math::Vector3D<float> > > >;
%template(NAME_FLOAT(NAME_R3D(Segment))) std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Rotation3D<float> > >, rw::core::Ptr< rw::trajectory::Interpolator<  rw::math::Rotation3D<float> > > >;
%template(NAME_FLOAT(NAME_T3D(Segment)))  std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Transform3D<float> > >, rw::core::Ptr< rw::trajectory::Interpolator<  rw::math::Transform3D<float> > > >;
%template(NAME_FLOAT(NAME_T3DV(Segment)))  std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Transform3DVector<float> > >, rw::core::Ptr< rw::trajectory::Interpolator<  rw::math::Transform3DVector<float> > > >;
%template(NAME_FLOAT(NAME_QUAT(Segment)))  std::pair< rw::core::Ptr< rw::trajectory::Blend< rw::math::Quaternion<float> > >, rw::core::Ptr< rw::trajectory::Interpolator<  rw::math::Quaternion<float> > > >;

%{
    #include <rw/trajectory/LinearInterpolator.hpp>
%}
%include <rw/trajectory/LinearInterpolator.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(LinearInterpolator,rw::trajectory::LinearInterpolator);

%{
    #include <rw/trajectory/RampInterpolator.hpp>
%}
%include <rw/trajectory/RampInterpolator.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(RampInterpolator,rw::trajectory::RampInterpolator);

%{
	#include <rw/trajectory/CircularInterpolator.hpp>
%}
%include <rw/trajectory/CircularInterpolator.hpp>
%template(CircularInterpolatorVector3D) rw::trajectory::CircularInterpolator<rw::math::Vector3D<double>>;
%template(CircularInterpolatorVector3D_f) rw::trajectory::CircularInterpolator<rw::math::Vector3D<float>>;

%{
	#include <rw/trajectory/CubicSplineFactory.hpp>
%}
%include <rw/trajectory/CubicSplineFactory.hpp>
%template(makeNaturalSpline) rw::trajectory::CubicSplineFactory::makeNaturalSpline<rw::math::Vector3D<double>>;
%template(makeNaturalSpline) rw::trajectory::CubicSplineFactory::makeNaturalSpline<rw::math::Transform3DVector<double>>;
%template(makeNaturalSpline) rw::trajectory::CubicSplineFactory::makeNaturalSpline<rw::math::Q>;								// Ken addon
%template(makeNaturalSpline) rw::trajectory::CubicSplineFactory::makeNaturalSpline<rw::math::Quaternion<double>>;

%template(makeClampedSpline) rw::trajectory::CubicSplineFactory::makeClampedSpline<rw::math::Vector3D<double>>;
%template(makeClampedSpline) rw::trajectory::CubicSplineFactory::makeClampedSpline<rw::math::Transform3DVector<double>>;
%template(makeClampedSpline) rw::trajectory::CubicSplineFactory::makeClampedSpline<rw::math::Q>;
%template(makeClampedSpline) rw::trajectory::CubicSplineFactory::makeClampedSpline<rw::math::Quaternion<double>>;

%{
	#include <rw/trajectory/CubicSplineInterpolator.hpp>
%}
%include <rw/trajectory/CubicSplineInterpolator.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(CubicSplineInterpolator,rw::trajectory::CubicSplineInterpolator);
%template(CubicSplineInterpolatorTransform3DVector) rw::trajectory::CubicSplineInterpolator<rw::math::Transform3DVector<double>>;

%{
	#include <rw/trajectory/InterpolatorUtil.hpp>
%}
%include <rw/trajectory/InterpolatorUtil.hpp>
//template(transToVec) rw::trajectory::InterpolatorUtil::transToVec<std::vector<double>,double>;
//template(vecToTrans) rw::trajectory::InterpolatorUtil::vecToTrans<std::vector<double>,double>;

%{
	#include <rw/trajectory/LloydHaywardBlend.hpp>
%}
%include <rw/trajectory/LloydHaywardBlend.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(LloydHaywardBlend,rw::trajectory::LloydHaywardBlend);

%feature("novaluewrapper") rw::core::Ptr< const rw::trajectory::LinearInterpolator< rw::math::Rotation3D< float > > >;
%feature("novaluewrapper") rw::core::Ptr< rw::trajectory::LinearInterpolator< rw::math::Rotation3D< float > > >;
%{
	#include <rw/trajectory/ParabolicBlend.hpp>
%}
%include <rw/trajectory/ParabolicBlend.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(ParabolicBlend,rw::trajectory::ParabolicBlend);




%define SWIG_SET_TIME()
	%extend {
        void setTime(double time){
            $self->rw::trajectory::Timed<T>::getTime() = time;
        }
    };
%enddef
%{
	#include <rw/trajectory/Timed.hpp>
%}
%include <rw/trajectory/Timed.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(Timed,rw::trajectory::Timed);
%template (TimedState) rw::trajectory::Timed<rw::kinematics::State>;

%{
	#include <rw/trajectory/TimedUtil.hpp>
%}
%include <rw/trajectory/TimedUtil.hpp>


#if defined(SWIGPYTHON)
%ignore rw::trajectory::Path::operator=;
#endif

%define SWIG_PATH_FUNCTIONS()
#if (defined (SWIGJAVA) && SWIG_VERSION >= 0x040000)
    %extend {
        int size(){ return int($self->std::vector<T >::size()); }
        T& elem(size_t idx){ return (*$self)[idx]; }
    };
#else
    %extend {
        size_t size(){ return $self->std::vector<T >::size(); }
        T& elem(size_t idx){ return (*$self)[idx]; }
    };
#endif
%enddef

%import(module=rwlibs/swig/sdurw_core) <rwlibs/swig/ext_i/std.i>

#if !defined(SWIGJAVA)
/*%std_vector(SWIGTYPE_internal_Vector_d,double);
%std_vector(SWIGTYPE_internal_Vector2D,rw::math::Vector2D<double>);
%std_vector(SWIGTYPE_internal_Vector3D,rw::math::Vector3D<double>);
%std_vector(SWIGTYPE_internal_Rotation3D,rw::math::Rotation3D<double>);
%std_vector(SWIGTYPE_internal_Transform3D,rw::math::Transform3D<double>);
%std_vector(SWIGTYPE_internal_Q,rw::math::Q);

%std_vector(SWIGTYPE_internal_Vector_f,float);
%std_vector(SWIGTYPE_internal_Vector2D_f,rw::math::Vector2D<float>);
%std_vector(SWIGTYPE_internal_Vector3D_f,rw::math::Vector3D<float>);
%std_vector(SWIGTYPE_internal_Rotation3D_f,rw::math::Rotation3D<float>);
%std_vector(SWIGTYPE_internal_Transform3D_f,rw::math::Transform3D<float>);*/
#endif

%{
	#include <rw/trajectory/Path.hpp>
%}
%include <rw/trajectory/Path.hpp>
#if !defined(SWIGJAVA)
//ADD_TRAJECTORY_STANDARD_TEMPLATE(Path,rw::trajectory::Path);
%std_vector_f (Path_d, double,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (Path_f, float,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathVector2D, rw::math::Vector2D<double>,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathVector2D_f, rw::math::Vector2D<float>,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathVector3D, rw::math::Vector3D<double>,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathVector3D_f, rw::math::Vector3D<float>,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathRotation3D, rw::math::Rotation3D<double>,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathRotation3D_f, rw::math::Rotation3D<float>,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathTransform3D, rw::math::Transform3D<double>,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathTransform3D_d, rw::math::Transform3D<float>,rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathQ, rw::math::Q,rw::trajectory::Path,"generalToFromPy");
#endif

// Transform
%std_vector(VectorTimedTransform3D, rw::trajectory::Timed<rw::math::Transform3D<double>>);
%std_vector_f(PathTimedTransform3D, rw::trajectory::Timed<rw::math::Transform3D<double>>,rw::trajectory::Path,"generalToFromPy");

//Quaternion
%std_vector (VectorQuaternion, rw::math::Quaternion<double>);
%std_vector_f (PathQuaternion, rw::math::Quaternion<double>, rw::trajectory::Path, "generalToFromPy");

//Transform3DVector
%std_vector (VectorTransform3dVector, rw::math::Transform3DVector<double>);
%std_vector_f (PathTransform3DVector, rw::math::Transform3DVector<double>,rw::trajectory::Path, "generalToFromPy");


//State
%std_vector   (VectorState,rw::kinematics::State);
%std_vector   (TimedStateVector,rw::trajectory::Timed<rw::kinematics::State>);
%std_vector_f (PathState, rw::kinematics::State, rw::trajectory::Path,"generalToFromPy");
%std_vector_f (PathTimedState, rw::trajectory::Timed<rw::kinematics::State>,rw::trajectory::Path,"generalToFromPy");
//NAMED_OWNEDPTR(PathState, rw::trajectory::Path<rw::kinematics::State>);

//Q
%std_vector_explicit(VectorPathQ,rw::math::Q,std::vector< rw::trajectory::Path< rw::math::Q > >,"generalToFromPy");
%std_vector (VectorTimedQ,rw::trajectory::Timed< rw::math::Q > );
%std_vector_f (PathState,rw::trajectory::Timed< rw::math::Q >,rw::trajectory::Path,"generalToFromPy");
//NAMED_OWNEDPTR(VectorTimedQ, std::vector<rw::trajectory::Timed< rw::math::Q > >);
//NAMED_OWNEDPTR(PathTimedQ,rw::trajectory::Path<rw::trajectory::Timed< rw::math::Q > >);

// State

//NAMED_OWNEDPTR(TimedStateVector,std::vector<rw::trajectory::Timed<rw::kinematics::State>>);
//NAMED_OWNEDPTR(PathTimedState,rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State> >);

//ADD_DEFINITION(PathTransform3DPtr,PathSE3Ptr,sdurw_trajectory)
//ADD_DEFINITION(PathTransform3DCPtr,PathSE3CPtr,sdurw_trajectory)


%extend rw::trajectory::Path< rw::math::Q > {
    rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed< rw::math::Q > > > toTimedQPath(rw::math::Q speed){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(speed, *$self);
        return rw::core::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed< rw::math::Q > > > toTimedQPath(rw::core::Ptr<rw::models::Device> dev){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(*dev, *$self);
        return rw::core::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State> > > toTimedStatePath(rw::core::Ptr<rw::models::Device> dev,
                                                     const rw::kinematics::State& state){
        rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State>> tpath =
                rw::trajectory::TimedUtil::makeTimedStatePath(*dev, *$self, state);
        return rw::core::ownedPtr( new rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State>>(tpath) );
    }
};

%extend rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State> > {
	
	/*static rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State> > > load(const std::string& filename, rw::core::Ptr<rw::models::WorkCell> wc){
		rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State>>> spath = 
                    rw::core::ownedPtr(new rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State>>);
                *spath = rw::loaders::PathLoader::loadTimedStatePath(*wc, filename);
		return rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State>>>( spath );
	}
	
	void save(const std::string& filename, rw::core::Ptr<rw::models::WorkCell> wc){		 		
		rw::loaders::PathLoader::storeTimedStatePath(*wc,*$self,filename); 
	}*/
	
	void append(rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State> > > spath){
		double startTime = 0;
		if($self->size()>0)
			startTime = (*$self).back().getTime(); 
		
		for(size_t i = 0; i<spath->size(); i++){
			rw::trajectory::Timed<rw::kinematics::State> tstate = (*spath)[i]; 
			tstate.getTime() += startTime;
			(*$self).push_back( tstate );
		}
	}
	
};

%extend rw::trajectory::Path<rw::kinematics::State > {
	
	/*static rw::core::Ptr<rw::trajectory::Path<rw::kinematics::State> > load(const std::string& filename, rw::core::Ptr<rw::models::WorkCell> wc){
            rw::core::Ptr<rw::trajectory::Path<rw::kinematics::State>> spath = rw::core::ownedPtr(new rw::trajectory::StatePath);
            *spath = rw::loaders::PathLoader::loadStatePath(*wc, filename);
		return spath;
	}
	
	void save(const std::string& filename, rw::core::Ptr<rw::models::WorkCell> wc){		 		
		rw::loaders::PathLoader::storeStatePath(*wc,*$self,filename); 
	}*/
	
	void append(rw::core::Ptr<rw::trajectory::Path<rw::kinematics::State> > spath){		
		for(size_t i = 0; i<spath->size(); i++){
			(*$self).push_back( (*spath)[i] );
		}
	}
	
	
	rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State> > > toTimedStatePath(double timeStep){
		rw::core::Ptr<rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State>>> spath = 
			rw::core::ownedPtr( new rw::trajectory::Path<rw::trajectory::Timed<rw::kinematics::State>>() );	
		for(size_t i = 0; i < $self->size(); i++){
			rw::trajectory::Timed<rw::kinematics::State> tstate(timeStep*i, (*$self)[i]); 
			spath->push_back( tstate );
		}	
		return spath;
	}
	
};

%{
	#include <rw/trajectory/TrajectoryFactory.hpp>
%}
%include <rw/trajectory/TrajectoryFactory.hpp>

%rename(next) rw::trajectory::TrajectoryIterator::operator++;
%rename(prev) rw::trajectory::TrajectoryIterator::operator--;
%rename(point) rw::trajectory::TrajectoryIterator::operator*;
%{
	#include <rw/trajectory/TrajectoryIterator.hpp>
%}
%include <rw/trajectory/TrajectoryIterator.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(TrajectoryIterator,rw::trajectory::TrajectoryIterator);

%{
	#include <rw/trajectory/BlendedTrajectory.hpp>
%}
%include <rw/trajectory/BlendedTrajectory.hpp>

%{
	#include <rw/trajectory/TimeMetricUtil.hpp>
%}
%include <rw/trajectory/TimeMetricUtil.hpp>

%{
	#include <rw/trajectory/TrajectorySequence.hpp>
%}
%include <rw/trajectory/TrajectorySequence.hpp>
ADD_TRAJECTORY_STANDARD_TEMPLATE(TrajectorySequence,rw::trajectory::TrajectorySequence);

%std_vector (VectorTrajectoryQPtr,rw::core::Ptr  < rw::trajectory::Trajectory<  rw::math::Q  > > );

