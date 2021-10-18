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

/**
 * @brief Extendible interface for loading of WorkCells from files.
 *
 * By default, the following formats are supported:
 *
 * - all file extensions will be loaded using the standard RobWork
 *   XML format (XMLRWLoader).
 *
 * The Factory defines an extension point "rw.loaders.WorkCellLoader"
 * that makes it possible to add loaders for other file formats than the
 * ones above. Extensions take precedence over the default loaders.
 *
 * The WorkCell loader is chosen based on a case-insensitive file extension
 * name. So "scene.wc.xml" will be loaded by the same loader as
 * "scene.WC.XML"
 *
 * WorkCells are supposed to be loaded using the WorkCellLoaderFactory.load function:
 * @beginPythonOnly
 * ::\n
 *     wc = WorkCellLoaderFactory.load("scene.wc.xml")
 *     if wc.isNull():
 *         raise Exception("WorkCell could not be loaded")
 * @endPythonOnly
 * @beginJavaOnly <pre> <code>
 * WorkCellPtr wc = WorkCellLoaderFactory.load("scene.wc.xml");
 * if (wc.isNull())
 *     throw new Exception("WorkCell could not be loaded.");
 * </code> </pre> @endJavaOnly
 * Alternatively a WorkCell can be loaded in the less convenient way:
 * @beginPythonOnly
 * ::\n
 *    loader = WorkCellLoaderFactory.getWorkCellLoader(".wc.xml");
 *    wc = loader.load("scene.wc.xml")
 *    if wc.isNull():
 *        raise Exception("WorkCell could not be loaded")
 * @endPythonOnly
 * @beginJavaOnly <pre> <code>
 * WorkCellLoaderPtr loader = WorkCellLoaderFactory.getWorkCellLoader(".wc.xml");
 * WorkCellPtr wc = loader.loadWorkCell("scene.wc.xml");
 * if (wc.isNull())
 *     throw new Exception("WorkCell could not be loaded.");
 * </code> </pre> @endJavaOnly
 */
class WorkCellLoader {
public:
	virtual ~WorkCellLoader();
    /**
     * @brief Load a WorkCell from a file.
     *
     * @param filename [in] path to workcell file.
     */
	virtual rw::core::Ptr<rw::models::WorkCell> loadWorkCell(const std::string& filename) = 0;

protected:
	WorkCellLoader();
};

%template (WorkCellLoaderPtr) rw::core::Ptr<WorkCellLoader>;

/**
 * @brief A factory for WorkCellLoader. This factory also defines the
 * "rw.loaders.WorkCellLoader" extension point where new loaders can be
 * registered.
 */
class WorkCellLoaderFactory {
public:
	/**
	 * @brief Get loaders for a specific format.
	 *
	 * @param format [in] the extension (including initial dot).
	 * The extension name is case-insensitive.
	 * @return a suitable loader.
	 */
	static rw::core::Ptr<WorkCellLoader> getWorkCellLoader(const std::string& format);

    /**
     * @brief Loads/imports a WorkCell from a file.
     *
     * An exception is thrown if the file can't be loaded.
     * The RobWork XML format is supported by default.
     *
     * @param filename [in] name of the WorkCell file.
     */
	static rw::core::Ptr<rw::models::WorkCell> load(const std::string& filename);
private:
	WorkCellLoaderFactory();
};

class ImageLoader {
public:
	virtual ~ImageLoader();
	virtual rw::core::Ptr<rw::sensor::Image> loadImage(const std::string& filename) = 0;
	virtual std::vector<std::string> getImageFormats() = 0;
	virtual bool isImageSupported(const std::string& format);
};

%template (ImageLoaderPtr) rw::core::Ptr<ImageLoader>;

class ImageLoaderFactory {
public:
	ImageLoaderFactory();
	static rw::core::Ptr<ImageLoader> getImageLoader(const std::string& format);
	static bool hasImageLoader(const std::string& format);
	static std::vector<std::string> getSupportedFormats();
};

#if defined(RW_HAVE_XERCES)

class XMLTrajectoryLoader
{
public:
    XMLTrajectoryLoader(const std::string& filename, const std::string& schemaFileName = "");
    XMLTrajectoryLoader(std::istream& instream, const std::string& schemaFileName = "");

    enum Type { QType = 0, Vector3DType, Rotation3DType, Transform3DType};
    Type getType();
    rw::core::Ptr<Trajectory< rw::math::Q > > getQTrajectory();
    rw::core::Ptr<Trajectory<rw::math::Vector3D<double> > > getVector3DTrajectory();
    rw::core::Ptr<Trajectory<rw::math::Rotation3D<double> > > getRotation3DTrajectory();
    rw::core::Ptr<Trajectory<rw::math::Transform3D<double> > > getTransform3DTrajectory();
};

class XMLTrajectorySaver
{
public:
    static bool save(const Trajectory< rw::math::Q >& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Vector3D<double> >& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Rotation3D<double> >& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Transform3D<double> >& trajectory, const std::string& filename);
    static bool write(const Trajectory< rw::math::Q >& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<rw::math::Vector3D<double> >& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<rw::math::Rotation3D<double> >& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<rw::math::Transform3D<double> >& trajectory, std::ostream& outstream);
private:
    XMLTrajectorySaver();
};

#endif

/********************************************
 * MATH
 ********************************************/
//%include <rwlibs/swig/rw_i/math.i>

// Utility function within rw::Math


/********************************************
 * MODELS
 ********************************************/
//%include <rwlibs/swig/rw_i/models.i>


/********************************************
 * PATHPLANNING
 ********************************************/

%include <rwlibs/swig/rw_i/planning.i>

/********************************************
 * PLUGIN
 ********************************************/

/********************************************
 * PROXIMITY
 ********************************************/

//%include <rwlibs/swig/rw_i/proximity.i>

/********************************************
 * SENSOR
 ********************************************/

//%include <rwlibs/swig/rw_i/sensor.i>

/********************************************
 * TRAJECTORY
 ********************************************/

template <class T>
class Timed
{
public:
    Timed();
    Timed(double time, const T& value);

    double getTime() const;
    T& getValue();

    %extend {
        void setTime(double time){
            $self->rw::trajectory::Timed<T>::getTime() = time;
        }
    };
};

%template (TimedQ) Timed< rw::math::Q >;
%template (TimedState) Timed<rw::kinematics::State>;

namespace rw { namespace trajectory {
template <class T>
class Path: public std::vector<T>
{
  public:

    Path();
    Path(size_t cnt);
    Path(size_t cnt, const T& value);
    Path(const std::vector<T>& v);

#if (defined (SWIGJAVA) && SWIG_VERSION >= 0x040000)
    %extend {
        int size(){ return boost::numeric_cast<int>($self->std::vector<T >::size()); }
        T& elem(size_t idx){ return (*$self)[idx]; }
    };
#else
    %extend {
        size_t size(){ return $self->std::vector<T >::size(); }
        T& elem(size_t idx){ return (*$self)[idx]; }
    };
#endif
};
}}

%template (VectorState) std::vector<rw::kinematics::State>;
%template (PathState) rw::trajectory::Path<rw::kinematics::State>;
NAMED_OWNEDPTR(PathState, rw::trajectory::Path<rw::kinematics::State>);


// Q
%template (VectorQ) std::vector<rw::math::Q>;
%template (TimedQVector) std::vector<Timed< rw::math::Q > >;
%template (PathQ) rw::trajectory::Path< rw::math::Q >;
%template (PathTimedQ) rw::trajectory::Path<Timed< rw::math::Q > >;
NAMED_OWNEDPTR(TimedQVector, std::vector<Timed< rw::math::Q > >);
NAMED_OWNEDPTR(PathQ, rw::trajectory::Path<rw::math::Q>); 
NAMED_OWNEDPTR(PathTimedQ,rw::trajectory::Path<Timed< rw::math::Q > >);

// State
%template (TimedStateVector) std::vector<Timed<rw::kinematics::State> >;
%template (PathTimedState) rw::trajectory::Path<Timed<rw::kinematics::State> >;
NAMED_OWNEDPTR(TimedStateVector,std::vector<Timed<rw::kinematics::State>>);
NAMED_OWNEDPTR(PathTimedState,rw::trajectory::Path<Timed<rw::kinematics::State> >);

// Transform
%template(VectorTransform3Dd) std::vector<rw::math::Transform3D<double>>;
%template(VectorTimedTransform3Dd) std::vector<rw::trajectory::Timed<rw::math::Transform3D<double>>>;
%template (PathSE3) rw::trajectory::Path<rw::math::Transform3D<double> >;
%template (PathTimedTransform3Dd) rw::trajectory::Path< rw::trajectory::Timed<rw::math::Transform3D<double>>>;
NAMED_OWNEDPTR(PathSE3,rw::trajectory::Path<rw::math::Transform3D<double> > );


%extend rw::trajectory::Path< rw::math::Q > {
    rw::core::Ptr<rw::trajectory::Path<Timed< rw::math::Q > > > toTimedQPath(rw::math::Q speed){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(speed, *$self);
        return rw::core::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::core::Ptr<rw::trajectory::Path<Timed< rw::math::Q > > > toTimedQPath(rw::core::Ptr<rw::models::Device> dev){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(*dev, *$self);
        return rw::core::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::core::Ptr<rw::trajectory::Path<Timed<rw::kinematics::State> > > toTimedStatePath(rw::core::Ptr<rw::models::Device> dev,
                                                     const rw::kinematics::State& state){
        rw::trajectory::Path<Timed<rw::kinematics::State>> tpath =
                rw::trajectory::TimedUtil::makeTimedStatePath(*dev, *$self, state);
        return rw::core::ownedPtr( new rw::trajectory::Path<Timed<rw::kinematics::State>>(tpath) );
    }

};

%extend rw::trajectory::Path<Timed<rw::kinematics::State> > {
	
	static rw::core::Ptr<rw::trajectory::Path<Timed<rw::kinematics::State> > > load(const std::string& filename, rw::core::Ptr<rw::models::WorkCell> wc){
		rw::core::Ptr<rw::trajectory::Path<Timed<rw::kinematics::State>>> spath = 
                    rw::core::ownedPtr(new rw::trajectory::Path<Timed<rw::kinematics::State>>);
                *spath = rw::loaders::PathLoader::loadTimedStatePath(*wc, filename);
		return rw::core::Ptr<rw::trajectory::Path<Timed<rw::kinematics::State>>>( spath );
	}
	
	void save(const std::string& filename, rw::core::Ptr<rw::models::WorkCell> wc){		 		
		rw::loaders::PathLoader::storeTimedStatePath(*wc,*$self,filename); 
	}
	
	void append(rw::core::Ptr<rw::trajectory::Path<Timed<rw::kinematics::State> > > spath){
		double startTime = 0;
		if($self->size()>0)
			startTime = (*$self).back().getTime(); 
		
		for(size_t i = 0; i<spath->size(); i++){
			Timed<rw::kinematics::State> tstate = (*spath)[i]; 
			tstate.getTime() += startTime;
			(*$self).push_back( tstate );
		}
	}
	
};

%extend rw::trajectory::Path<rw::kinematics::State > {
	
	static rw::core::Ptr<rw::trajectory::Path<rw::kinematics::State> > load(const std::string& filename, rw::core::Ptr<rw::models::WorkCell> wc){
            rw::core::Ptr<rw::trajectory::Path<rw::kinematics::State>> spath = rw::core::ownedPtr(new rw::trajectory::StatePath);
            *spath = rw::loaders::PathLoader::loadStatePath(*wc, filename);
		return spath;
	}
	
	void save(const std::string& filename, rw::core::Ptr<rw::models::WorkCell> wc){		 		
		rw::loaders::PathLoader::storeStatePath(*wc,*$self,filename); 
	}
	
	void append(rw::core::Ptr<rw::trajectory::Path<rw::kinematics::State> > spath){		
		for(size_t i = 0; i<spath->size(); i++){
			(*$self).push_back( (*spath)[i] );
		}
	}
	
	
	rw::core::Ptr<rw::trajectory::Path<Timed<rw::kinematics::State> > > toTimedStatePath(double timeStep){
		rw::core::Ptr<rw::trajectory::Path<Timed<rw::kinematics::State>>> spath = 
			rw::core::ownedPtr( new rw::trajectory::Path<Timed<rw::kinematics::State>>() );	
		for(size_t i = 0; i < $self->size(); i++){
			Timed<rw::kinematics::State> tstate(timeStep*i, (*$self)[i]); 
			spath->push_back( tstate );
		}	
		return spath;
	}
	
};

template <class T>
class Blend
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double tau1() const = 0;
    virtual double tau2() const = 0;
};

%template (BlendR1) Blend<double>;
%template (BlendR2) Blend<rw::math::Vector2D<double> >;
%template (BlendR3) Blend<rw::math::Vector3D<double> >;
%template (BlendSO3) Blend<rw::math::Rotation3D<double> >;
%template (BlendSE3) Blend<rw::math::Transform3D<double> >;
%template (BlendQ) Blend< rw::math::Q >;

%template (BlendR1Ptr) rw::core::Ptr<Blend<double> >;
%template (BlendR2Ptr) rw::core::Ptr<Blend<rw::math::Vector2D<double> > >;
%template (BlendR3Ptr) rw::core::Ptr<Blend<rw::math::Vector3D<double> > >;
%template (BlendSO3Ptr) rw::core::Ptr<Blend<rw::math::Rotation3D<double> > >;
%template (BlendSE3Ptr) rw::core::Ptr<Blend<rw::math::Transform3D<double> > >;
%template (BlendQPtr) rw::core::Ptr<Blend< rw::math::Q > >;

OWNEDPTR(Blend<double> )
OWNEDPTR(Blend<rw::math::Vector2D<double> > )
OWNEDPTR(Blend<rw::math::Vector3D<double> > )
OWNEDPTR(Blend<rw::math::Rotation3D<double> > )
OWNEDPTR(Blend<rw::math::Transform3D<double> > )
OWNEDPTR(Blend< rw::math::Q > )

template <class T>
class Interpolator
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double duration() const = 0;
};

%template (InterpolatorR1) Interpolator<double>;
%template (InterpolatorR2) Interpolator<rw::math::Vector2D<double> >;
%template (InterpolatorR3) Interpolator<rw::math::Vector3D<double> >;
%template (InterpolatorSO3) Interpolator<rw::math::Rotation3D<double> >;
%template (InterpolatorSE3) Interpolator<rw::math::Transform3D<double> >;
%template (InterpolatorVector2Df) Interpolator<rw::math::Vector2D<float> >;
%template (InterpolatorVector3Df) Interpolator<rw::math::Vector3D<float> >;
%template (InterpolatorRotation3Df) Interpolator<rw::math::Rotation3D<float> >;
%template (InterpolatorTransform3Df) Interpolator<rw::math::Transform3D<float> >;
%template (InterpolatorQ) Interpolator< rw::math::Q >;

%template (InterpolatorR1Ptr) rw::core::Ptr<Interpolator<double> >;
%template (InterpolatorR2Ptr) rw::core::Ptr<Interpolator<rw::math::Vector2D<double> > >;
%template (InterpolatorR3Ptr) rw::core::Ptr<Interpolator<rw::math::Vector3D<double> > >;
%template (InterpolatorSO3Ptr) rw::core::Ptr<Interpolator<rw::math::Rotation3D<double> > >;
%template (InterpolatorSE3Ptr) rw::core::Ptr<Interpolator<rw::math::Transform3D<double> > >;
%template (InterpolatorQPtr) rw::core::Ptr<Interpolator< rw::math::Q > >;

OWNEDPTR(Interpolator<double> )
OWNEDPTR(Interpolator<rw::math::Vector2D<double> > )
OWNEDPTR(Interpolator<rw::math::Vector3D<double> > )
OWNEDPTR(Interpolator<rw::math::Rotation3D<double> > )
OWNEDPTR(Interpolator<rw::math::Transform3D<double> > )
OWNEDPTR(Interpolator< rw::math::Q > )



%{
    #include <rw/trajectory/LinearInterpolator.hpp>
%}
%include <rw/trajectory/LinearInterpolator.hpp>

%template(LinearInterpolatord) rw::trajectory::LinearInterpolator<double>;
%template(LinearInterpolatorQ) rw::trajectory::LinearInterpolator<rw::math::Q>;
%template(LinearInterpolatorRotation3Dd) rw::trajectory::LinearInterpolator<rw::math::Rotation3D<double>>;
%template(LinearInterpolatorRotation3Df) rw::trajectory::LinearInterpolator<rw::math::Rotation3D<float>>;
%template(LinearInterpolatorTransform3Dd) rw::trajectory::LinearInterpolator<rw::math::Transform3D<double>>;
%template(LinearInterpolatorTransform3Df) rw::trajectory::LinearInterpolator<rw::math::Transform3D<float>>;


//////////// RAMP interpolator

%{
    #include <rw/trajectory/RampInterpolator.hpp>
%}
%include <rw/trajectory/RampInterpolator.hpp>

%extend rw::trajectory::RampInterpolator<rw::math::Rotation3D<double>> {
    
}

%template(RampInterpolatord) rw::trajectory::RampInterpolator<double>;
%template(RampInterpolatorQ) rw::trajectory::RampInterpolator<rw::math::Q>;
%template(RampInterpolatorRotation3Dd) rw::trajectory::RampInterpolator<rw::math::Rotation3D<double>>;
%template(RampInterpolatorRotation3Df) rw::trajectory::RampInterpolator<rw::math::Rotation3D<float>>;
%template(RampInterpolatorTransform3Dd) rw::trajectory::RampInterpolator<rw::math::Transform3D<double>>;
%template(RampInterpolatorTransform3Df) rw::trajectory::RampInterpolator<rw::math::Transform3D<float>>;

template <class T>
class Trajectory
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double duration() const = 0;
    virtual double startTime() const = 0;
    virtual double endTime() const;

    std::vector<T> getPath(double dt, bool uniform = true);
    //virtual typename rw::core::Ptr< TrajectoryIterator<T> > getIterator(double dt = 1) const = 0;

protected:
    /**
     * @brief Construct an empty trajectory
     */
    Trajectory() {};
};

%template (TrajectoryState) Trajectory<rw::kinematics::State>;
%template (TrajectoryR1) Trajectory<double>;
%template (TrajectoryR2) Trajectory<rw::math::Vector2D<double> >;
%template (TrajectoryR3) Trajectory<rw::math::Vector3D<double> >;
%template (TrajectorySO3) Trajectory<rw::math::Rotation3D<double> >;
%template (TrajectorySE3) Trajectory<rw::math::Transform3D<double> >;
%template (TrajectoryQ) Trajectory< rw::math::Q >;

%template (TrajectoryStatePtr) rw::core::Ptr<Trajectory<rw::kinematics::State> >;
%template (TrajectoryR1Ptr) rw::core::Ptr<Trajectory<double> >;
%template (TrajectoryR2Ptr) rw::core::Ptr<Trajectory<rw::math::Vector2D<double> > >;
%template (TrajectoryR3Ptr) rw::core::Ptr<Trajectory<rw::math::Vector3D<double> > >;
%template (TrajectorySO3Ptr) rw::core::Ptr<Trajectory<rw::math::Rotation3D<double> > >;
%template (TrajectorySE3Ptr) rw::core::Ptr<Trajectory<rw::math::Transform3D<double> > >;
%template (TrajectoryQPtr) rw::core::Ptr<Trajectory< rw::math::Q > >;

OWNEDPTR(Trajectory<rw::kinematics::State> )
OWNEDPTR(Trajectory<double> )
OWNEDPTR(Trajectory<rw::math::Vector2D<double> > )
OWNEDPTR(Trajectory<rw::math::Vector3D<double> > )
OWNEDPTR(Trajectory<rw::math::Rotation3D<double> > )
OWNEDPTR(Trajectory<rw::math::Transform3D<double> > )
OWNEDPTR(Trajectory< rw::math::Q > )

template <class T>
class InterpolatorTrajectory: public Trajectory<T> {
public:
    InterpolatorTrajectory(double startTime = 0);
    void add(rw::core::Ptr<Interpolator<T> > interpolator);
    void add(rw::core::Ptr<Blend<T> > blend,
             rw::core::Ptr<Interpolator<T> > interpolator);
    void add(InterpolatorTrajectory<T>* trajectory);
    size_t getSegmentsCount() const;



    //std::pair<rw::core::Ptr<Blend<T> >, rw::core::Ptr<Interpolator<T> > > getSegment(size_t index) const;
};

%template (InterpolatorTrajectoryR1) InterpolatorTrajectory<double>;
%template (InterpolatorTrajectoryR2) InterpolatorTrajectory<rw::math::Vector2D<double> >;
%template (InterpolatorTrajectoryR3) InterpolatorTrajectory<rw::math::Vector3D<double> >;
%template (InterpolatorTrajectorySO3) InterpolatorTrajectory<rw::math::Rotation3D<double> >;
%template (InterpolatorTrajectorySE3) InterpolatorTrajectory<rw::math::Transform3D<double> >;
%template (InterpolatorTrajectoryQ) InterpolatorTrajectory< rw::math::Q >;


/*
class TrajectoryFactory
{
public:
    static rw::core::Ptr<rw::kinematics::StateTrajectory> makeFixedTrajectory(const rw::kinematics::State& state, double duration);
    static rw::core::Ptr<QTrajectory> makeFixedTrajectory(const rw::math::Q& q, double duration);
    static rw::core::Ptr<rw::kinematics::StateTrajectory> makeLinearTrajectory(const Path<Timed<rw::kinematics::State>>& path);
    static rw::core::Ptr<rw::kinematics::StateTrajectory> makeLinearTrajectory(const rw::kinematics::StatePath& path,
        const models::WorkCell& workcell);
    static rw::core::Ptr<rw::kinematics::StateTrajectory> makeLinearTrajectoryUnitStep(const rw::kinematics::StatePath& path);
    static rw::core::Ptr<QTrajectory> makeLinearTrajectory(const TimedQPath& path);
    static rw::core::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, const rw::math::Q& speeds);
    static rw::core::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, const models::Device& device);
    static rw::core::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, rw::core::Ptr<QMetric> metric);
    static rw::core::Ptr<Transform3DTrajectory> makeLinearTrajectory(const Transform3DPath& path, const std::vector<double>& times);
    static rw::core::Ptr<Transform3DTrajectory> makeLinearTrajectory(const Transform3DPath& path, const rw::core::Ptr<Transform3DMetric> metric);
    static rw::core::Ptr<rw::kinematics::StateTrajectory> makeEmptyStateTrajectory();
    static rw::core::Ptr<QTrajectory > makeEmptyQTrajectory();
};

*/
 
%{
#ifndef WIN32
	#pragma GCC diagnostic pop
#endif
%}