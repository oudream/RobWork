%module sdurw

%{
#include <RobWorkConfig.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
#include <rw/kinematics.hpp>
#include <rw/math.hpp>
#include <rw/models.hpp>

using namespace rwlibs::swig;
using rw::trajectory::Interpolator;
using rw::trajectory::Blend;
using rw::trajectory::Timed;
using rw::trajectory::Trajectory;
using rw::trajectory::InterpolatorTrajectory;
using rw::pathplanning::PathPlanner;

#ifndef WIN32
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif
%}



%pragma(java) jniclassclassmodifiers="class"
#if defined (SWIGJAVA)
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
SWIG_JAVABODY_TYPEWRAPPER(public, public, public, SWIGTYPE)
#endif

#if defined(SWIGPYTHON)
%feature("doxygen:ignore:beginPythonOnly", range="end:endPythonOnly", contents="parse");
%feature("doxygen:ignore:beginJavaOnly", range="end:endJavaOnly");
#elif defined(SWIGJAVA)
%feature("doxygen:ignore:beginPythonOnly", range="end:endPythonOnly");
%feature("doxygen:ignore:beginJavaOnly", range="end:endJavaOnly", contents="parse");
#else
%feature("doxygen:ignore:beginPythonOnly", range="end:endPythonOnly");
%feature("doxygen:ignore:beginJavaOnly", range="end:endJavaOnly");
#endif

%include <std_string.i>
%include <std_vector.i>
%include <std_map.i>

%include <stl.i>
%include <exception.i>

%include <rwlibs/swig/swig_macros.i>

/********************************************
 * General utility functions
 ********************************************/

/* This is called for all functions to handle exceptions disable with %exception; 
 *  
 */
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

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_sensor.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw_invkin.i>
%import <rwlibs/swig/sdurw_proximity.i>
%import <rwlibs/swig/sdurw_graspplanning.i>
%import <rwlibs/swig/sdurw_trajectory.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_graspplanning.*;
import org.robwork.sdurw_invkin.*;
%}

%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_graspplanning.*;
import org.robwork.sdurw_invkin.*;
%}

%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_graspplanning.*;
import org.robwork.sdurw_invkin.*;
%}

/********************************************
 * CORE
 ********************************************/


%extend rw::core::PropertyMap {
            

    rw::math::Q& getQ(const std::string& id){ return $self->get< rw::math::Q >(id); }
    void setQ(const std::string& id, rw::math::Q q){ $self->set< rw::math::Q >(id, q); }
    void set(const std::string& id, rw::math::Q q){ $self->set< rw::math::Q >(id, q); }

    rw::math::Pose6D<double>& getPose6D(const std::string& id){ return $self->get<rw::math::Pose6D<double> >(id); }
    void setPose6D(const std::string& id, rw::math::Pose6D<double> p){  $self->set<rw::math::Pose6D<double> >(id, p); }
    void set(const std::string& id, rw::math::Pose6D<double> p){  $self->set<rw::math::Pose6D<double> >(id, p); }
    
    rw::math::Vector3D<double>& getVector3D(const std::string& id){ return $self->get<rw::math::Vector3D<double> >(id); }
    void setVector3D(const std::string& id, rw::math::Vector3D<double> p){  $self->set<rw::math::Vector3D<double> >(id, p); }
    void set(const std::string& id, rw::math::Vector3D<double> p){  $self->set<rw::math::Vector3D<double> >(id, p); }

    rw::math::Transform3D<double> & getTransform3D(const std::string& id){ return $self->get<rw::math::Transform3D<double> >(id); }
    void setTransform3D(const std::string& id, rw::math::Transform3D<double>  p){  $self->set<rw::math::Transform3D<double> >(id, p); }
    void set(const std::string& id, rw::math::Transform3D<double>  p){  $self->set<rw::math::Transform3D<double> >(id, p); }

    void load(const std::string& filename){ *($self) = rw::loaders::DOMPropertyMapLoader::load(filename); }
    void save(const std::string& filename){ rw::loaders::DOMPropertyMapSaver::save( *($self), filename ); }
}

%import <rwlibs/swig/ext_i/std.i>

//%include <rwlibs/swig/ext_i/boost.i>

%inline %{

    void sleep(double t){
        ::rw::common::TimerUtil::sleepMs( (int) (t*1000) );
    }
    double time(){
        return ::rw::common::TimerUtil::currentTime( );
    }
    long long timeMs(){
        return ::rw::common::TimerUtil::currentTimeMs( );
    }
    void infoLog(const std::string& msg){
        ::rw::core::Log::infoLog() << msg << std::endl;
    }
    void debugLog(const std::string& msg){
        ::rw::core::Log::debugLog() << msg << std::endl;
    }
    void warnLog(const std::string& msg){
        ::rw::core::Log::warningLog() << msg << std::endl;
    }
    void errorLog(const std::string& msg){
        ::rw::core::Log::errorLog() << msg << std::endl;
    }
%}


void writelog(const std::string& msg);


/********************************************
 * LOADERS
 ********************************************/

#if defined(RW_HAVE_XERCES)

class XMLTrajectoryLoader
{
public:
    XMLTrajectoryLoader(const std::string& filename, const std::string& schemaFileName = "");
    XMLTrajectoryLoader(std::istream& instream, const std::string& schemaFileName = "");

    enum Type { QType = 0, Vector3DType, Rotation3DType, Transform3DType};
    Type getType();
    rw::core::Ptr<rw::trajectory::Trajectory< rw::math::Q > > getQTrajectory();
    rw::core::Ptr<rw::trajectory::Trajectory<rw::math::Vector3D<double> > > getVector3DTrajectory();
    rw::core::Ptr<rw::trajectory::Trajectory<rw::math::Rotation3D<double> > > getRotation3DTrajectory();
    rw::core::Ptr<rw::trajectory::Trajectory<rw::math::Transform3D<double> > > getTransform3DTrajectory();
};

class XMLTrajectorySaver
{
public:
    static bool save(const rw::trajectory::Trajectory< rw::math::Q >& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Trajectory<rw::math::Vector3D<double> >& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Trajectory<rw::math::Rotation3D<double> >& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Trajectory<rw::math::Transform3D<double> >& trajectory, const std::string& filename);
    static bool write(const rw::trajectory::Trajectory< rw::math::Q >& trajectory, std::ostream& outstream);
    static bool write(const rw::trajectory::Trajectory<rw::math::Vector3D<double> >& trajectory, std::ostream& outstream);
    static bool write(const rw::trajectory::Trajectory<rw::math::Rotation3D<double> >& trajectory, std::ostream& outstream);
    static bool write(const rw::trajectory::Trajectory<rw::math::Transform3D<double> >& trajectory, std::ostream& outstream);
private:
    XMLTrajectorySaver();
};

#endif

/********************************************
 * PATHPLANNING
 ********************************************/

//%include <rwlibs/swig/rw_i/planning.i>


 
%{
#ifndef WIN32
	#pragma GCC diagnostic pop
#endif
%}