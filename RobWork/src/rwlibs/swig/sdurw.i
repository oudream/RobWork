%module sdurw

%{
#include <RobWorkConfig.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>

using namespace rwlibs::swig;
using rw::math::Metric;
using namespace rw::math;
using rw::trajectory::Interpolator;
using rw::trajectory::Blend;
using rw::trajectory::Path;
using rw::trajectory::Timed;
using rw::trajectory::Trajectory;
using rw::trajectory::InterpolatorTrajectory;
using rw::pathplanning::PathPlanner;
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
%include <typemaps.i>

//%include <shared_ptr.i>

#if !defined(SWIGJAVA)
%include "carrays.i"
%array_class(double, doubleArray);
#else
%include "arrays_java.i";
#endif

#if defined(SWIGJAVA)
	%rename(multiply) operator*;
	%rename(divide) operator/;
	%rename(equals) operator==;
	%rename(negate) operator-() const;
	%rename(subtract) operator-;
	%rename(add) operator+;
#endif

#if (defined(SWIGPYTHON) || defined(SWIGLUA))
%feature("flatnested");
#endif

%include <stl.i>

/*
%define COVARIANT(DERIVED, BASE)
%types(rw::common::Ptr<DERIVED> = rw::common::Ptr<BASE>) %{
        *newmemory = SWIG_CAST_NEW_MEMORY;
        return (void*) new rw::common::Ptr<BASE>(*(rw::common::Ptr<DERIVED>*)$from);
%}
%enddef

%COVARIANT(Apple, Fruit)
*/

void writelog(const std::string& msg);

/********************************************
 * General utility functions
 ********************************************/

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
        ::rw::common::Log::infoLog() << msg << std::endl;
    }
    void debugLog(const std::string& msg){
        ::rw::common::Log::debugLog() << msg << std::endl;
    }
    void warnLog(const std::string& msg){
        ::rw::common::Log::warningLog() << msg << std::endl;
    }
    void errorLog(const std::string& msg){
        ::rw::common::Log::errorLog() << msg << std::endl;
    }
%}



/********************************************
 * Constants
 ********************************************/

%constant double Pi = rw::math::Pi;
%constant double Inch2Meter = rw::math::Inch2Meter;
%constant double Meter2Inch = rw::math::Meter2Inch;
%constant double Deg2Rad = rw::math::Deg2Rad;
%constant double Rad2Deg = rw::math::Rad2Deg;

/********************************************
 * STL vectors (primitive types)
 ********************************************/
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
	%extend std::vector<std::string> { char *__str__() { return printCString(*$self); } }
#endif

namespace std {
	%template(StringVector) std::vector<string>;
	%template(DoubleVector) std::vector<double>;
	%template(IntVector) std::vector<int>;
};

/********************************************
 * COMMON
 ********************************************/

namespace rw { namespace common {

/**
  * @brief The Ptr type represents a smart pointer that can take ownership
  * of the underlying object.
  *
  * If the underlying object is owned by the smart pointer, it is destructed
  * when there is no more smart pointers pointing to the object.
  */
template<class T> class Ptr
{
public:
    //! @brief Empty smart pointer (Null).
    Ptr();

    /**
      * @brief Construct new smart pointer that takes ownership of the
      * underlying object.
      *
      * @param ptr The object to take ownership of.
      */
    Ptr(T* ptr);

    /**
      * @brief Construct smart pointer from other smart pointer.
      *
      * @param p the other (compatible) smart pointer.
      */
    template <class S>
    Ptr(const Ptr<S>& p);

    bool isShared();

    /**
      * @brief Check if smart pointer is null.
      *
      * @return true if smart pointer is null.
      */
    bool isNull();

    template<class A>
    bool operator==(const rw::common::Ptr<A>& p) const;
#if defined(SWIGJAVA)
	%rename(dereference) get;
#endif
    T* get() const;

    T *operator->() const;
};

#if defined(SWIGPYTHON)
 %pythonprepend ownedPtr(T*) %{
  args[0].thisown = 0
 %}
#endif
/**
  * @brief Construct a smart pointer that takes ownership over a raw object \b ptr.
  *
  * @param ptr the object to take ownership of.
  */
template <class T>
Ptr<T> ownedPtr(T* ptr);

}}

%define OWNEDPTR(ownedPtr_type)
namespace rw { namespace common {
#if defined(SWIGJAVA)
 %typemap (in) ownedPtr_type* %{
  jclass objcls = jenv->GetObjectClass(jarg1_);
  const jfieldID memField = jenv->GetFieldID(objcls, "swigCMemOwn", "Z");
  jenv->SetBooleanField(jarg1_, memField, (jboolean)false);
  $1 = *(std::remove_const<ownedPtr_type>::type **)&jarg1;
 %}
#elif defined(SWIGLUA)
 %typemap (in,checkfn="SWIG_isptrtype") ownedPtr_type* %{
  if (!SWIG_IsOK(SWIG_ConvertPtr(L,$input,(void**)&$1,$descriptor,SWIG_POINTER_DISOWN))){
    SWIG_fail_ptr("$symname",$input,$descriptor);
  }
 %}
#endif
 %template (ownedPtr) ownedPtr<ownedPtr_type>;
#if (defined(SWIGLUA) || defined(SWIGJAVA))
 %clear ownedPtr_type*;
#endif
}}
%enddef

/** @addtogroup swig */
/* @{ */

//! @copydoc rw::common::PropertyMap
class PropertyMap
{
public: 
	//! @copydoc rw::common::PropertyMap::PropertyMap
	PropertyMap();
	//! @copydoc rw::common::PropertyMap::has
	bool has(const std::string& identifier) const;
    //! @copydoc rw::common::PropertyMap::size
    size_t size() const;
    //! @copydoc rw::common::PropertyMap::empty 
    bool empty() const;
    //! @copydoc rw::common::PropertyMap::erase
    bool erase(const std::string& identifier);
    
	%extend {
		
		bool getBool(const std::string& id){ return $self->get<bool>(id); }
		void setBool(const std::string& id, bool val){  $self->set<bool>(id,val); }
		void set(const std::string& id, bool val){  $self->set<bool>(id,val); }

		std::string getString(const std::string& id){ return $self->get<std::string>(id); }
		void setString(const std::string& id, std::string val){  $self->set<std::string>(id,val); }
		void set(const std::string& id, std::string val){  $self->set<std::string>(id,val); }
		
		std::vector<std::string>& getStringList(const std::string& id){ return $self->get<std::vector<std::string> >(id); }
		void setStringList(const std::string& id, std::vector<std::string> val){ $self->set<std::vector<std::string> >(id,val); }
		void set(const std::string& id, std::vector<std::string> val){ $self->set<std::vector<std::string> >(id,val); }
		
		rw::math::Q& getQ(const std::string& id){ return $self->get<rw::math::Q>(id); }
		void setQ(const std::string& id, rw::math::Q q){ $self->set<rw::math::Q>(id, q); }
		void set(const std::string& id, rw::math::Q q){ $self->set<rw::math::Q>(id, q); }

		rw::math::Pose6D<double>& getPose(const std::string& id){ return $self->get<rw::math::Pose6D<double> >(id); }
		void setPose6D(const std::string& id, rw::math::Pose6D<double> p){  $self->set<rw::math::Pose6D<double> >(id, p); }
		void set(const std::string& id, rw::math::Pose6D<double> p){  $self->set<rw::math::Pose6D<double> >(id, p); }
		
		rw::math::Vector3D<double>& getVector3(const std::string& id){ return $self->get<rw::math::Vector3D<double> >(id); }
		void setVector3(const std::string& id, rw::math::Vector3D<double> p){  $self->set<rw::math::Vector3D<double> >(id, p); }
		void set(const std::string& id, rw::math::Vector3D<double> p){  $self->set<rw::math::Vector3D<double> >(id, p); }

		rw::math::Transform3D<double> & getTransform3(const std::string& id){ return $self->get<rw::math::Transform3D<double> >(id); }
		void setTransform3(const std::string& id, rw::math::Transform3D<double>  p){  $self->set<rw::math::Transform3D<double> >(id, p); }
		void set(const std::string& id, rw::math::Transform3D<double>  p){  $self->set<rw::math::Transform3D<double> >(id, p); }

		PropertyMap& getMap(const std::string& id){ return $self->get<PropertyMap>(id); }
		void setMap(const std::string& id, PropertyMap p){  $self->set<PropertyMap>(id, p); }
		void set(const std::string& id, PropertyMap p){  $self->set<PropertyMap>(id, p); }

		void load(const std::string& filename){ *($self) = rw::loaders::DOMPropertyMapLoader::load(filename); }
		void save(const std::string& filename){ rw::loaders::DOMPropertyMapSaver::save( *($self), filename ); }
		
	}    
 
};
%template (PropertyMapPtr) rw::common::Ptr<PropertyMap>;
OWNEDPTR(PropertyMap)

/**
 * \brief Provides basic log functionality.
 *
 * The Log class owns a number of LogWriters in a static map, which can be accessed
 * using a string identifier. All logs are global.
 *
 * By default the Log class contains a Debug, Info, Warning and Error log. These can be accessed
 * statically as:
 * \code
 * Log::debugLog() <<  "This is an debug message";
 * Log::infoLog() << "This is an info message";
 * Log::warnLog() << "This is an error message";
 * Log::errorLog() << "This is an error message";
 * \endcode
 * or on the log instance
 * \code
 * Log &log = Log::log();
 * log.debug() <<  "This is an debug message";
 * log.info() << "This is an info message";
 * log.warn() << "This is an error message";
 * log.error() << "This is an error message";
 * \endcode
 * or using one one the RW_LOG, RW_LOGLINE or RW_LOG2 macros, e.g.
 * \code
 * RW_LOG_INFO("The value of x is "<<x);
 * RW_LOG_DEBUG("The value of x is "<<x);
 * RW_LOG_ERROR(Log::infoId(), "The value of x is "<<x);
 * \endcode
 *
 * You can control what logs are active both using a loglevel and by using a log mask.
 * The loglevel enables all logs with LogIndex lower or equal to the loglevel. As default
 * loglevel is LogIndex::info which means debug and all user logs are disabled. However,
 * logs can be individually enabled using log masks which will override loglevel setting.
 *
 * Notice that logmasks cannot disable logs that are below or equal to loglevel.
 *
 * change loglevel:
 * \code
 * Log::log().setLevel(Log::Debug);
 * \endcode
 *
 *
 */
class Log
{
public:
    //! @brief loglevel mask
	enum LogIndexMask {
		FatalMask=1, CriticalMask=2,
		ErrorMask=4, WarningMask=8,
		InfoMask=16, DebugMask=32,
		User1Mask=64, User2Mask=128,
		User3Mask=256, User4Mask=512,
		User5Mask=1024, User6Mask=2048,
		User7Mask=4096, User8Mask=8096,
		AllMask = 0xFFFF
	};



	/**
	 * @brief Indices for different logs. The loglevel will be Info as default. Everything below the
	 * loglevel is enabled.
	 */
	enum LogIndex {
		Fatal=0, Critical=1,
		Error=2, Warning=3,
		Info=4, Debug=5,
		User1=6, User2=7,
		User3=8, User4=9,
		User5=10, User6=11,
		User7=12, User8=13
	};

	/**
	 * @brief Convert a LogIndex to a mask.
	 *
	 * @param idx [in] the LogIndex.
	 * @return the mask enabling the given log level.
	 */
    static LogIndexMask toMask(LogIndex idx){
            LogIndexMask toMaskArr[] = {FatalMask, CriticalMask,
                                      ErrorMask, WarningMask,
                                                InfoMask, DebugMask,
                                                User1Mask, User2Mask,
                                                User3Mask, User4Mask,
                                                User5Mask, User6Mask,
                                                User7Mask, User8Mask,
                                                AllMask};
            return toMaskArr[idx];
        }

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the info loglevel
	 *
	 * @return info LogWriter
	 */
    static LogWriter& infoLog();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the warning loglevel
	 *
	 * @return warning LogWriter
	 */
    static LogWriter& warningLog();


	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the error loglevel
	 *
	 * @return error LogWriter
	 */
    static LogWriter& errorLog();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the debug loglevel
	 *
	 * @return debug LogWriter
	 */
    static LogWriter& debugLog();

	/**
	 * @brief returns the global log instance. Global in the sence
	 * of whatever is linked staticly together.
	 *
	 * @return a Log
	 */
    static rw::common::Ptr<Log> getInstance();

    /**
     * @brief convenience function of getInstance
     *
     * @return a Log
     */
    static Log& log();

    /**
     * @brief sets the instance of the log class
     *
     * @param log [in] the log that will be used through the static log methods.
     */
    static void setLog(rw::common::Ptr<Log> log);

    //************************* Here follows the member interface

    /**
     * @brief constructor
     */
    Log();

    /**
     * @brief Destructor
     */
    virtual ~Log();

    /**
     * @brief set the loglevel. Any log with LogIndex equal to or less than
     * loglevel will be enabled. Any log above will be disabled unless an
     * enabled mask is specified for that log
     *
     * @param loglevel [in] the level
     */
    void setLevel(LogIndex loglevel);


    /**
     * @brief gets the log writer associated to logindex \b id
     *
     * @param id [in] logindex
     * @return log writer
     */
    rw::common::Ptr<LogWriter> getWriter(LogIndex id);

    /**
     * @brief Associates a LogWriter with the LogIndex \b id.
     *
     * SetWriter can either be used to redefine an existing log or to create a new
     * custom output.
     *
     * Example:
     * \code
     * Log::SetWriter(Log::User1, new LogStreamWriter(std::cout));
     * RW_LOG(Log::User1, "Message send to User log 1");
     * \endcode
     *
     * @param id [in] the LogIndex that the logwriter is associated with.
     * @param writer [in] LogWriter object to use
     */
    void setWriter(LogIndex id, rw::common::Ptr<LogWriter> writer);

    /**
     * @brief Associates a LogWriter with the logs specified with \b mask.
     *
     * SetWriter can either be used to redefine an existing log or to create a new
     * custom output.
     *
     * Example:
     * \code
     * log.setWriterForMask(Log::InfoMask | Log::DebugMask, new LogStreamWriter(std::cout));
     * RW_LOG(Log::Info, "Message send to User log 1");
     * \endcode
     *
     * @param mask [in] the LogIndexMask that the logwriter is associated with.
     * @param writer [in] LogWriter object to use
     */
	void setWriterForMask(int mask, rw::common::Ptr<LogWriter> writer);

    %extend {
		/**
		 * @brief Returns the LogWriter that is associated with LogIndex \b id
		 *
		 * If the \b id is unknown an exception is thrown.
		 *
		 * @param id [in] loglevel
		 * @return Reference to LogWriter object
		 */
		LogWriter& getLogWriter(LogIndex id) {
			return $self->get(id);
		}
    };

    /**
     * @brief Writes \b message to the log
     *
     * If the \b id cannot be found an exception is thrown
     *
     * @param id [in] Log identifier
     * @param message [in] String message to write
     */
    void write(LogIndex id, const std::string& message);

    /**
     * @brief Writes \b message to the logwriter associated with LogIndex \b id
     *
     * If the \b id cannot be found an exception is thrown

     *
     * @param id [in] Log identifier
     * @param message [in] Message to write
     */
    void write(LogIndex id, const Message& message);

    /**
     * @brief Writes \b message followed by a '\\n' to the log
     *
     * If the \b id cannot be found an exception is thrown
     *
     * @param id [in] Log identifier
     * @param message [in] Message to write
     */
    void writeln(LogIndex id, const std::string& message);

    /**
     * @brief Calls flush on the specified log
     *
     * If the \b id cannot be found an exception is thrown
     *
     * @param id [in] loglevel
     */
    void flush(LogIndex id);


    /**
     * @brief Calls flush on all logs
     */
    void flushAll();


    /**
     * @brief Removes a log
     *
     * If the \b id cannot be found an exception is thrown
     *
     * @param id [in] Log identifier
     */
    void remove(LogIndex id);

	/**
	 * @brief Removes all log writers
	 */
	void removeAll();

	//! @brief Make indentation to make logs easier to read.
	void increaseTabLevel();

	//! @brief Decrease the indentation.
	void decreaseTabLevel();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the info loglevel
	 *
	 * @return info LogWriter
	 */
    LogWriter& info();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the warning loglevel
	 *
	 * @return info LogWriter
	 */
    LogWriter& warning();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the error loglevel
	 *
	 * @return info LogWriter
	 */
    LogWriter& error();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the debug loglevel
	 *
	 * @return info LogWriter
	 */
    LogWriter& debug();

	/**
	 * @brief Enable log(s) given by log mask.
	 *
	 * @param mask [in] the mask for the logs to enable.
	 */
	void setEnable(int mask);

   /**
     * @brief Checks if the given LogIndex is enabled. This can be used to
     * determine if a certain log level will be displayed or not.
     *
     * @param idx [in] the level
     *
     * @return true if enabled, false otherwise.
     */
    bool isEnabled(LogIndex idx);

	/**
	 * @brief Disable log(s) given by log mask.
	 *
	 * @param mask [in] the mask for the logs to disable.
	 */
	void setDisable(int mask);
};

%template (LogPtr) rw::common::Ptr<Log>;
OWNEDPTR(Log)

/**
 * @brief Write interface for Logs
 *
 * LogWriter provides an output strategy for a log.
 */
class LogWriter
{
public:
    /**
     * @brief Descructor
     */
    virtual ~LogWriter();

    /**
     * @brief Flush method
     */
    void flush();

	/**
	 * @brief Set the tab level
	 */
	void setTabLevel(int tabLevel);


    /**
     * @brief Writes \b str to the log
     *
     * @param str [in] message to write
     */
    void write(const std::string& str);

    /**
     * @brief Writes \b msg to the log
     *
     * Default behavior is to use write(const std::string&) for the standard
     * streaming representation of \b msg.
     *
     * @param msg [in] message to write
     */
    void write(const Message& msg);

    /**
     * @brief Writes \b str as a line
     *
     * By default writeln writes \b str followed by a '\\n'. However, logs
     * are free to implement a line change differently.
     */
    void writeln(const std::string& str);

    /**
     * @brief general stream operator
     */
    template< class T>
    LogWriter& operator<<( T t );

#if defined(SWIGPYTHON)
    /**
     * @brief specialized stream operator
     */
    LogWriter& operator<<(const std::string& str);

    /**
     * @brief Write Message to log.
     * @param msg [in] the message.
     * @return a reference to this LogWriter for chaining of stream operators.
     */
    LogWriter& operator<<(const Message& msg);


    /**
     * @brief specialized stream operator
     */
    LogWriter& operator<<(const char* str);

    /**
     * @brief Handle the std::endl and other stream functions.
     */
    LogWriter& operator<<(std::ostream& (*pf)(std::ostream&));
#endif

protected:
    LogWriter();

	virtual void doWrite(const std::string& message) = 0;
	virtual void doSetTabLevel(int tabLevel) = 0;
	virtual void doFlush() = 0;
};

%template (LogWriterPtr) rw::common::Ptr<LogWriter>;

/**
 * @brief Standard type for user messages of RobWork.
 *
 * Messages are used for exception, warnings, and other things that are
 * reported to the user.
 *
 * Message values should contain the source file name and line number so
 * that it is easy to look up the place in the code responsible for the
 * generation of the message.
 *
 * RW_THROW and RW_WARN of macros.hpp have been introduced for the throwing
 * of exceptions and emission of warnings.
 */
class Message
{
public:
    /**
     * @brief Constructor
     *
     * Messages of RobWork are all annotated by the originating file name,
     * the originating line number, and a message text for the user.
     *
     * Supplying all the file, line, and message parameters can be a little
     * painfull, so a utility for creating messages is available from the
     * file macros.hpp.
     *
     * @param file [in] The source file name.
     *
     * @param line [in] The source file line number.
     *
     * @param message [in] A message for a user.
     */
    Message(const std::string& file,
            int line,
            const std::string& message = "");

    /**
     * @brief The name of source file within which the message was
     * constructed.
     *
     * @return The exception file name.
     */
    const std::string& getFile() const;

    /**
     * @brief The line number for the file at where the message was
     * constructed.
     *
     * @return The exception line number.
     */
    int getLine() const;

    /**
     * @brief The message text meant for the user.
     *
     * @return The message text.
     */
    const std::string& getText() const;

    /**
     * @brief Returns a full description of the message containing file, line number and message.
     */
    std::string getFullText() const;

    /**
     * @brief general stream operator
     */
    template< class T>
    Message& operator<<( T t );
};

class ThreadPool { 
public:
    ThreadPool(int threads = -1);
    virtual ~ThreadPool();
    unsigned int getNumberOfThreads() const;
    void stop();
    bool isStopping();
	unsigned int getQueueSize();
	void waitForEmptyQueue();
};

%template (MessagePtr) rw::common::Ptr<Message>;

%template (ThreadPoolPtr) rw::common::Ptr<ThreadPool>;
OWNEDPTR(ThreadPool)

class ThreadTask {
public:
	typedef enum TaskState {
    	INITIALIZATION,
    	IN_QUEUE,
    	EXECUTING,
    	CHILDREN,
    	IDLE,
    	POSTWORK,
    	DONE
    } TaskState;

	ThreadTask(rw::common::Ptr<ThreadTask> parent);
	ThreadTask(rw::common::Ptr<ThreadPool> pool);
	virtual ~ThreadTask();
	bool setThreadPool(rw::common::Ptr<ThreadPool> pool);
	rw::common::Ptr<ThreadPool> getThreadPool();
	//virtual void run();
	//virtual void subTaskDone(ThreadTask* subtask);
	//virtual void idle();
	//virtual void done();
    bool execute();
    TaskState wait(ThreadTask::TaskState previous);
    void waitUntilDone();
    TaskState getState();
    bool addSubTask(rw::common::Ptr<ThreadTask> subtask);
    std::vector<rw::common::Ptr<ThreadTask> > getSubTasks();
    void setKeepAlive(bool keepAlive);
    bool keepAlive();
};

%template (ThreadTaskPtr) rw::common::Ptr<ThreadTask>;
%template (ThreadTaskPtrVector) std::vector<rw::common::Ptr<ThreadTask> >;
OWNEDPTR(ThreadTask)

class Plugin {
protected:
	 Plugin(const std::string& id, const std::string& name, const std::string& version);
	 
public:
	const std::string& getId();
    const std::string& getName();
    const std::string& getVersion();
};

%template (PluginPtr) rw::common::Ptr<Plugin>;
%template (PluginPtrVector) std::vector<rw::common::Ptr<Plugin> >;

struct ExtensionDescriptor {
	ExtensionDescriptor();
	ExtensionDescriptor(const std::string& id_, const std::string& point_);

    std::string id,name,point;
    rw::common::PropertyMap props;

    //rw::common::PropertyMap& getProperties();
    const rw::common::PropertyMap& getProperties() const;
};

class Extension {
public:
	Extension(ExtensionDescriptor desc, Plugin* plugin);
	
	const std::string& getId();
	const std::string& getName();
};

%template (ExtensionPtr) rw::common::Ptr<Extension>;
%template (ExtensionPtrVector) std::vector<rw::common::Ptr<Extension> >;

class ExtensionRegistry {
public:
	ExtensionRegistry();
	static rw::common::Ptr<ExtensionRegistry> getInstance();
	std::vector<rw::common::Ptr<Extension> > getExtensions(const std::string& ext_point_id) const;
	std::vector<rw::common::Ptr<Plugin> > getPlugins() const;
};

%template (ExtensionRegistryPtr) rw::common::Ptr<ExtensionRegistry>;

/**
 * @brief The timer class provides an easy to use platform independent timer
 *
 * In Windows the expected resolution is approx. 16ms.
 * In Linux the expected resolution is approx. 1ms
 */
class Timer
{
public:
    /**
     * @brief Constructor
     *
     * This implicitly starts the timer.
     */
    Timer();

    /**
     * @brief constructor - initialize the timer to a specified value. This does not start the timer.
     * @param timems [in] time in ms
     */
    Timer(long timems);

    /**
     * @brief constructor - initialize the timer to a specified value. This does not start the timer.
     * @param hh [in] hours
     * @param mm [in] minutes
     * @param ss [in] seconds
     * @param ms [in] milli seconds
     */
    Timer(int hh, int mm, int ss = 0, int ms = 0);

    /**
     * @brief Destructor
     */
    virtual ~Timer();

    /** 
     * @brief Returns true if the timer is paused
     * @return True is paused
     */
    bool isPaused();

    /**
     * @brief Reset the timer
     *
     * The timer is set back to zero and starts counting.
     *
     * It is OK to call reset() on a timer that has already been started:
     * The time will just be set back to zero again.
     */
    void reset();


    /**
     * @brief Resets and pauses the timer
     *
     * The timer is set to zero and paused
     *
     * It is OK to call reset() on a timer that has already been started:
     * The time will just be set back to zero again.
     */
    void resetAndPause();

    /**
     * @brief Resets and stats the timer
     *
     * Same as reset()
     *
     * The timer is set to zero and starts counting
     *
     * It is OK to call reset() on a timer that has already been started:
     * The time will just be set back to zero again.
     */
    void resetAndResume();

    /**
     * @brief Pause the timer
     *
     * The timer stops counting. As long as the timer has not been resumed
     * (see resume()) or restarted (see reset()) the timer value (see
     * getTime()) will stay the same.
     *
     * Is is OK to call pause() on a timer that has already been paused: The
     * timer will just stay paused and nothing is changed.
     */
    void pause();

    /**
     * @brief Resume the timer after a pause.
     *
     * The timer starts counting again.
     *
     * It is OK to call resume() on a timer that is already counting: The
     * timer keeps counting and nothing is done.
     */
    void resume();

    /**
     * @brief The time the timer has been running.
     *
     * The time passed while the timer has been paused are not included in
     * the running time. The timer is paused when pause() has been
     * called and counting is resumed when resume() has been called.
     *
     * It is perfectly OK and natural to call getTime() on a running timer,
     * i.e. a timer that has not been paused.
     *
     * A call of reset() resets the running time to zero.
     *
     * \return Time in seconds
     */
    double getTime() const;

    /**
     * @brief The time the timer has been running in hole seconds.
     *
     * see getTime
     *
     * \return Time in hole seconds
     */
    long getTimeSec() const;

    /**
     * @brief The time the timer has been running in mili seconds.
     *
     * see getTime
     *
     * \return Time in mili seconds
     */
    long getTimeMs() const;


    /**
     * @brief returns a string describing the time. The format of the time is described using \b format
     * @param format [in] the format is on the form:
     *  hh:mm:ss --> 05:06:08
     *  h:m:s --> 5:6:8
     * @return a formated time string
     */
    std::string toString(const std::string& format="hh:mm:ss");


    /**
     * @brief Returns system clock in hole seconds
     *
     * \warning The date/time at which this timer counts from is platform-specific, so
     * you should \b not use it for getting the calendar time. It's really only meant for
     * calculating wall time differences.
     */
    static long currentTimeSec();


    /**
     * @brief Returns system clock in milli-seconds
     *
     * \warning The date/time at which this timer counts from is platform-specific, so
     * you should \b not use it for getting the calendar time. It's really only meant for
     * calculating wall time differences.
     */
    static long currentTimeMs();


    /**
     * @brief Returns system clock in micro-seconds.
     *
     * \warning The date/time at which this timer counts from is platform-specific, so
     * you should \b not use it for getting the calendar time. It's really only meant for
     * calculating wall time differences.
     *
     * Notice: The timer cannot hold times longer than approx. 2100second.
     */
    static long currentTimeUs();

    /**
     * @brief Returns system clock in seconds
     *
     * \warning The date/time at which this timer counts from is platform-specific, so
     * you should \b not use it for getting the calendar time. It's really only meant for
     * calculating wall time differences.
     */
    static double currentTime();


    /**
     * @brief Sleeps for a period of time
     *
     * @param period [in] the time in miliseconds to sleep
     */
    static void sleepMs(int period);

    /**
     * @brief Sleeps for a period of time
     *
     * @param period [in] the time in microseconds to sleep
     */
    static void sleepUs(int period);
};

/********************************************
 * ROBWORK CLASS
 ********************************************/ 
 class RobWork {
 public:
	RobWork();
	
	static rw::common::Ptr<RobWork> getInstance();
	
	std::string getVersion() const;
	void initialize();
 };
 
 %template (RobWorkPtr) rw::common::Ptr<RobWork>;

/********************************************
 * GEOMETRY
 ********************************************/

class GeometryData {
public:
    typedef enum {PlainTriMesh,
                  IdxTriMesh,
                  SpherePrim, BoxPrim, OBBPrim, AABBPrim,
                  LinePrim, PointPrim, PyramidPrim, ConePrim,
                  TrianglePrim, CylinderPrim, PlanePrim, RayPrim,
                  UserType} GeometryType;

	/**
	 * @brief the type of this primitive
	 *
	 * @return the type of primitive.
	 */
    virtual GeometryType getType() const = 0;

	/**
	 * @brief gets a trimesh representation of this geometry data.
	 *
	 * The trimesh that is returned is by default a copy, which means
	 * ownership is transfered to the caller. 
	 * @param forceCopy Specifying forceCopy to false will enable copy by reference and 
     * ownership is not necesarilly transfered.
	 * This is more efficient, though pointer is only alive as long as this
	 * GeometryData is alive.
	 * @return TriMesh representation of this GeometryData
	 */
    virtual rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true) = 0;

	/**
	 * @brief format GeometryType to string
	 *
	 * @param type [in] the type of geometry to convert to string.
	 *
	 * @return a string.
	 */
    static std::string toString(GeometryType type);
};

%template (GeometryDataPtr) rw::common::Ptr<GeometryData>;
OWNEDPTR(GeometryData);

class TriMesh: public GeometryData {
public:
    virtual Triangle getTriangle(size_t idx) const = 0;
    virtual void getTriangle(size_t idx, Triangle& dst) const = 0;
    virtual void getTriangle(size_t idx, Trianglef& dst) const = 0;
    virtual size_t getSize() const = 0;
    virtual size_t size() const = 0;
    virtual rw::common::Ptr<TriMesh> clone() const = 0;
    rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);
    //rw::common::Ptr<const TriMesh> getTriMesh(bool forceCopy=true) const;
};

%template (TriMeshPtr) rw::common::Ptr<TriMesh>;

class Primitive: public GeometryData {
public:
    rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);
    virtual rw::common::Ptr<TriMesh> createMesh(int resolution) const = 0;
    virtual rw::math::Q getParameters() const = 0;
};

class Sphere: public Primitive {
public:
    //! constructor
    Sphere(const rw::math::Q& initQ);
    Sphere(double radi):_radius(radi);
    double getRadius();
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    rw::math::Q getParameters() const;
    GeometryData::GeometryType getType() const;
};

class Box: public Primitive {
public:
    Box();
    Box(double x, double y, double z);
    Box(const rw::math::Q& initQ);
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    rw::math::Q getParameters() const;
    GeometryType getType() const;
};

class Cone: public Primitive {
public:
    Cone(const rw::math::Q& initQ);
    Cone(double height, double radiusTop, double radiusBot);
    double getHeight();
    double getTopRadius();
    double getBottomRadius();
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    rw::math::Q getParameters() const;
    GeometryType getType() const;
};

class Plane: public Primitive {
public:
    Plane(const rw::math::Q& q);
    Plane(const rw::math::Vector3D<double>& n, double d);
    Plane(const rw::math::Vector3D<double>& p1,
          const rw::math::Vector3D<double>& p2,
          const rw::math::Vector3D<double>& p3);

    rw::math::Vector3D<double>& normal();
    //const rw::math::Vector3D<double>& normal() const;
#if defined(SWIGJAVA)
	double d() const;
#else
    double& d();
#endif
    double distance(const rw::math::Vector3D<double>& point);
    double refit( std::vector<rw::math::Vector3D<double> >& data );
    rw::common::Ptr<TriMesh> createMesh(int resolution) const ;
    rw::math::Q getParameters() const;
    GeometryType getType() const;
};

/**
 * @brief Cylinder primitive.
 */
class Cylinder: public Primitive {
public:
    //! @brief Default constructor with no parameters.
	Cylinder();

	/**
	  * @brief Cylinder with parameters specified.
	  *
	  * @param radius the radius.
	  * @param height the height.
	  */

	Cylinder(float radius, float height);
	virtual ~Cylinder();
	double getRadius() const;
	double getHeight() const;
	
	/**
	  * @brief Create a mesh representation of the cylinder.
	  *
	  * @param resolution the resolution.
	  * @return the TriMesh.
	  */
	rw::common::Ptr<TriMesh> createMesh(int resolution) const;
	rw::math::Q getParameters() const;
	GeometryType getType() const;
};

class ConvexHull3D {
public:
    virtual void rebuild(const std::vector<rw::math::Vector3D<double> >& vertices) = 0;
    virtual bool isInside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual double getMinDistInside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual double getMinDistOutside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual rw::common::Ptr<PlainTriMeshN1> toTriMesh() = 0;
};


class Geometry {
public:
    Geometry(rw::common::Ptr<GeometryData> data, double scale=1.0);

    Geometry(rw::common::Ptr<GeometryData> data,
             const rw::math::Transform3D<double> & t3d,
             double scale=1.0);

    double getScale() const;
    void setScale(double scale);
    void setTransform(const rw::math::Transform3D<double> & t3d);
    const rw::math::Transform3D<double> & getTransform() const;
    rw::common::Ptr<GeometryData> getGeometryData();
#if !defined(SWIGJAVA)
    const rw::common::Ptr<GeometryData> getGeometryData() const;
#endif
    void setGeometryData(rw::common::Ptr<GeometryData> data);
    const std::string& getName() const;
    const std::string& getId() const;
    void setName(const std::string& name);
    void setId(const std::string& id);
    static rw::common::Ptr<Geometry> makeSphere(double radi);
    static rw::common::Ptr<Geometry> makeBox(double x, double y, double z);
    static rw::common::Ptr<Geometry> makeCone(double height, double radiusTop, double radiusBot);
    static rw::common::Ptr<Geometry> makeCylinder(float radius, float height);
};

%template (GeometryPtr) rw::common::Ptr<Geometry>;
%template (GeometryPtrVector) std::vector<rw::common::Ptr<Geometry> >;
OWNEDPTR(Geometry);

class STLFile {
public:
    static void save(const TriMesh& mesh, const std::string& filename);
    static rw::common::Ptr<PlainTriMeshN1f> load(const std::string& filename);
};

class PlainTriMeshN1
{
};

%template (PlainTriMeshN1Ptr) rw::common::Ptr<PlainTriMeshN1>;

class PlainTriMeshN1f
{
};

%template (PlainTriMeshN1fPtr) rw::common::Ptr<PlainTriMeshN1f>;

%nodefaultctor Triangle;
class Triangle
{
};

%nodefaultctor Trianglef;
class Trianglef
{
};


/**
 * @brief A simple point cloud data structure. Points may be ordered or not. An ordered set is
 * kept as a single array in row major order and with a width and a height. An unordered array
 * must have height==1 and width equal to the number of points.
 */
class PointCloud: public GeometryData {
public:
    /**
     * @brief constructor
     */
    PointCloud();

    /**
     * @brief constructor
     *
     * @param w [in]
     * @param h [in]
     */
    PointCloud(int w, int h);

	/**
	 * @brief destructor
	 */
	virtual ~PointCloud();

	//! @copydoc GeometryData::getType
	 GeometryType getType() const;

	/**
	 * @brief gets the number of points in the point cloud.
	 *
	 * @return the number of points.
	 */
	virtual size_t size() const;

	bool isOrdered();

    /**
     * @brief returns a char pointer to the image data
     *
     * @return const char pointer to the image data
     */
    const std::vector<rw::math::Vector3D<float> >& getData() const;

    /**
     * @brief width of the point cloud data. If the data is unordered then this
     * will be equal to the number of points.
     *
     * @return width of data points
     */
    int getWidth() const;

    int getHeight() const;

    /**
     * @brief set width of point cloud. Data elements are accessed as [x+y*width].
     *
     * If the current data array cannot contain the elements then it will be resized to
     * be able to it.
     *
     * @param w [in] new width
     * @param h [in] new height
     */
    void resize(int w, int h);

	//! @copydoc getTriMesh
	rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);

	const rw::math::Transform3D<float>& getDataTransform() const;

	/**
	 * @brief load point cloud from PCD file
	 *
	 * @param filename [in] name of PCD file
	 * @return a point cloud
	 */
	static rw::common::Ptr<PointCloud> loadPCD( const std::string& filename );

	/**
	 * @brief save point cloud in PCD file format (PCL library format)
	 *
	 * @param cloud [in] the point cloud to save
	 * @param filename [in] the name of the file to save to
	 * @param t3d [in] the transformation of the point cloud
	 */
    static void savePCD(const PointCloud& cloud,
                        const std::string& filename ,
                        const rw::math::Transform3D<float>& t3d =
                        rw::math::Transform3D<float>::identity());
};

%template (PointCloudPtr) rw::common::Ptr<PointCloud>;


/********************************************
 * GRAPHICS
 ********************************************/

%template (WorkCellScenePtr) rw::common::Ptr<WorkCellScene>;
%template (DrawableNodePtr) rw::common::Ptr<DrawableNode>;
%template (DrawableNodePtrVector) std::vector<rw::common::Ptr<DrawableNode> >;

OWNEDPTR(WorkCellScene);

%constant int DNodePhysical = DrawableNode::Physical;
%constant int DNodeVirtual = DrawableNode::Virtual;
%constant int DNodeDrawableObject = DrawableNode::DrawableObject;
%constant int DNodeCollisionObject = DrawableNode::CollisionObject;
%nodefaultctor DrawableNode;
%nodefaultctor WorkCellScene;

class DrawableNode {
public:

    enum DrawType {
        //! Render in solid
        SOLID,
        //! Render in wireframe
        WIRE,
        //! Render both solid and wireframe
        OUTLINE
    };

    virtual void setHighlighted(bool b) = 0;

    virtual bool isHighlighted() const = 0;

    virtual void setDrawType(DrawType drawType) = 0;

    virtual void setTransparency(float alpha) = 0;

    virtual float getTransparency() = 0;

    bool isTransparent();

    virtual void setScale(float scale) = 0;

    virtual float getScale() const = 0;

    virtual void setVisible(bool enable) = 0;

    virtual bool isVisible() = 0;

    virtual const rw::math::Transform3D<double> & getTransform() const  = 0;

    virtual void setTransform(const rw::math::Transform3D<double> & t3d) = 0;

    virtual void setMask(unsigned int mask) = 0;
    virtual unsigned int getMask() const = 0;
};

class Model3D {
public:
    Model3D(const std::string& name);
    virtual ~Model3D();
    //struct Material;
    //struct MaterialFaces;
    //struct MaterialPolys;
    //struct Object3D;
    //typedef enum{
    //    AVERAGED_NORMALS //! vertex normal is determine as an avarage of all adjacent face normals
    //    ,WEIGHTED_NORMALS //! vertex normal is determined as AVARAGED_NORMALS, but with the face normals scaled by the face area
    //    } SmoothMethod;
    //void optimize(double smooth_angle, SmoothMethod method=WEIGHTED_NORMALS);
    //int addObject(Object3D::Ptr obj);
    //void addGeometry(const Material& mat, rw::common::Ptr<Geometry> geom);
    //void addTriMesh(const Material& mat, const rw::geometry::TriMesh& mesh);
    //int addMaterial(const Material& mat);
    //Material* getMaterial(const std::string& matid);
    bool hasMaterial(const std::string& matid);
    void removeObject(const std::string& name);
    //std::vector<Material>& getMaterials();
    //std::vector<Object3D::Ptr>& getObjects();
    const rw::math::Transform3D<double>& getTransform();
    void setTransform(const rw::math::Transform3D<double>& t3d);
    const std::string& getName();
    void setName(const std::string& name);
    int getMask();
    void setMask(int mask);
    rw::common::Ptr<GeometryData> toGeometryData();
    bool isDynamic() const;
    void setDynamic(bool dynamic);
};

%template (Model3DPtr) rw::common::Ptr<Model3D>;
%template (Model3DPtrVector) std::vector<rw::common::Ptr<Model3D> >;
OWNEDPTR(Model3D);

class Render {
public:
    /**
     * @brief draws the object.
     * @param info [in] state and rendering specific info
     * @param type [in] the drawtype which is being used
     * @param alpha [in] the alpha value to render with
     */
    virtual void draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const = 0;
};

%template (RenderPtr) rw::common::Ptr<Render>;

class WorkCellScene {
 public:

     rw::common::Ptr<WorkCell> getWorkCell();

     void setState(const State& state);

     //rw::graphics::GroupNode::Ptr getWorldNode();
     void updateSceneGraph(State& state);
     //void clearCache();

     void setVisible(bool visible, Frame* f);

     bool isVisible(Frame* f);

     void setHighlighted( bool highlighted, Frame* f);
     bool isHighlighted( Frame* f);
     void setFrameAxisVisible( bool visible, Frame* f);
     bool isFrameAxisVisible( Frame* f);
     //void setDrawType( DrawableNode::DrawType type, Frame* f);
     //DrawableNode::DrawType getDrawType( Frame* f );

     void setDrawMask( unsigned int mask, Frame* f);
     unsigned int getDrawMask( Frame* f );
     void setTransparency(double alpha, Frame* f);

     //DrawableGeometryNode::Ptr addLines( const std::string& name, const std::vector<rw::geometry::Line >& lines, Frame* frame, int dmask=DrawableNode::Physical);
     //DrawableGeometryNode::Ptr addGeometry(const std::string& name, rw::common::Ptr<Geometry> geom, Frame* frame, int dmask=DrawableNode::Physical);
     rw::common::Ptr<DrawableNode> addFrameAxis(const std::string& name, double size, Frame* frame, int dmask=DrawableNode::Virtual);
     //rw::common::Ptr<DrawableNode> addModel3D(const std::string& name, rw::common::Ptr<Model3D> model, Frame* frame, int dmask=DrawableNode::Physical);
     //rw::common::Ptr<DrawableNode> addImage(const std::string& name, const rw::sensor::Image& img, Frame* frame, int dmask=DrawableNode::Virtual);
     //rw::common::Ptr<DrawableNode> addScan(const std::string& name, const rw::sensor::Scan2D& scan, Frame* frame, int dmask=DrawableNode::Virtual);
     //rw::common::Ptr<DrawableNode> addScan(const std::string& name, const rw::sensor::Image25D& scan, Frame* frame, int dmask=DrawableNode::Virtual);
     rw::common::Ptr<DrawableNode> addRender(const std::string& name, rw::common::Ptr<Render> render, Frame* frame, int dmask=DrawableNode::Physical);

     rw::common::Ptr<DrawableNode> addDrawable(const std::string& filename, Frame* frame, int dmask);
     void addDrawable(rw::common::Ptr<DrawableNode> drawable, Frame*);

     //std::vector<rw::common::Ptr<DrawableNode> > getDrawables();
     //std::vector<rw::common::Ptr<DrawableNode> > getDrawables(Frame* f);

     //std::vector<rw::common::Ptr<DrawableNode> > getDrawablesRec(Frame* f, State& state);
     rw::common::Ptr<DrawableNode> findDrawable(const std::string& name);

     rw::common::Ptr<DrawableNode> findDrawable(const std::string& name, Frame* frame);

     std::vector<rw::common::Ptr<DrawableNode> > findDrawables(const std::string& name);

     bool removeDrawables(Frame* f);

     bool removeDrawables(const std::string& name);

     bool removeDrawable(rw::common::Ptr<DrawableNode> drawable);

     bool removeDrawable(rw::common::Ptr<DrawableNode> drawable, Frame* f);

     bool removeDrawable(const std::string& name);
     bool removeDrawable(const std::string& name, Frame* f);
     Frame* getFrame(rw::common::Ptr<DrawableNode>  d);

     //rw::graphics::GroupNode::Ptr getNode(Frame* frame);
 };

%nodefaultctor SceneViewer;
class SceneViewer
{
};
 
%template (SceneViewerPtr) rw::common::Ptr<SceneViewer>;

/********************************************
 * GRASPPLANNING
 ********************************************/

/********************************************
 * INVKIN
 ********************************************/
 
 %include <rwlibs/swig/rw_i/invkin.i>

/********************************************
 * KINEMATICS
 ********************************************/

%nodefaultctor State;
class State
{
public:
	std::size_t size() const;
	State clone();
};
%template (StateVector) std::vector<State>;

/**
 * @brief the basic building block for the stateless design using
 * the StateStructure class. A StateData represents a size,
 * a unique id, and a unique name, when inserted into the StateStructure.
 * The size will allocate "size"-doubles in State objects originating from the
 * StateStructure.
 */
class StateData {
protected:
    StateData(int size, const std::string& name);
public:
    /**
     * @brief The name of the state data.
     *
     * @return The name of the state data.
     */
    const std::string& getName() const;

    /**
     * @brief The number of doubles allocated by this StateData in
     * each State object.
     *
     * @return The number of doubles allocated by the StateData
     */
    int size() const;

#if !defined(SWIGJAVA)
    /**
     * @brief An array of length size() containing the values for
     * the state data.
     *
     * It is OK to call this method also for a StateData with zero size.
     *
     * @param state [in] The state containing the StateData values.
     *
     * @return The values for the frame.
     */
    double* getData(State& state);
#endif
#if defined(SWIGJAVA)
%apply double[] {double *};
#endif
    /**
     * @brief Assign for \b state data the size() of values of the array \b
     * vals.
     *
     * The array \b vals must be of length at least size().
     *
     * @param state [inout] The state to which \b vals are written.
     *
     * @param vals [in] The joint values to assign.
     */
    void setData(State& state, const double* vals) const;
};

class StateStructure {
public:
	StateStructure();
	 
	void addFrame(Frame *frame, Frame *parent=NULL);
	const State& getDefaultState() const;
	const std::vector<Frame*>& getFrames() const;
};
%template (StateStructurePtr) rw::common::Ptr<StateStructure>;
OWNEDPTR(StateStructure);

/**
 * @brief The type of node of forward kinematic trees.
 *
 * Types of joints are implemented as subclasses of Frame. The
 * responsibility of a joint is to implement the getTransform() method that
 * returns the transform of the frame relative to whatever parent it is
 * attached to.
 *
 * The getTransform() method takes as parameter the set of joint values
 * State for the tree. Joint values for a particular frame can be accessed
 * via State::getQ(). A frame may contain pointers to other frames so that
 * the transform of a frame may depend on the joint values for other frames
 * also.
 */
class Frame : public StateData
{
public:

    /**
     * @brief Post-multiply the transform of the frame to the parent transform.
     *
     * The transform is calculated for the joint values of \b state.
     *
     * The exact implementation of getTransform() depends on the type of
     * frame. See for example RevoluteJoint and PrismaticJoint.
     *
     * @param parent [in] The world transform of the parent frame.
     * @param state [in] Joint values for the forward kinematics tree.
     * @param result [in] The transform of the frame in the world frame.
     */
    void multiplyTransform(const rw::math::Transform3D<double>& parent,
                           const State& state,
                           rw::math::Transform3D<double>& result) const;

    /**
     * @brief The transform of the frame relative to its parent.
     *
     * The transform is calculated for the joint values of \b state.
     *
     * The exact implementation of getTransform() depends on the type of
     * frame. See for example RevoluteJoint and PrismaticJoint.
     *
     * @param state [in] Joint values for the forward kinematics tree.
     *
     * @return The transform of the frame relative to its parent.
     */
    rw::math::Transform3D<double> getTransform(const State& state) const;

#if !defined(SWIGJAVA) 
    /**
     * @brief Miscellaneous properties of the frame.
     *
     * The property map of the frame is provided to let the user store
     * various settings for the frame. The settings are typically loaded
     * from setup files.
     *
     * The low-level manipulations of the property map can be cumbersome. To
     * ease these manipulations, the PropertyAccessor utility class has been
     * provided. Instances of this class are provided for a number of common
     * settings, however it is undecided if these properties are a public
     * part of RobWork.
     *
     * @return The property map of the frame.
     */
    const PropertyMap& getPropertyMap() const;
#endif

    /**
     * @brief Miscellaneous properties of the frame.
     *
     * The property map of the frame is provided to let the user store
     * various settings for the frame. The settings are typically loaded
     * from setup files.
     *
     * The low-level manipulations of the property map can be cumbersome. To
     * ease these manipulations, the PropertyAccessor utility class has been
     * provided. Instances of this class are provided for a number of common
     * settings, however it is undecided if these properties are a public
     * part of RobWork.
     *
     * @return The property map of the frame.
     */
    PropertyMap& getPropertyMap();


    /**
     * @brief The number of degrees of freedom (dof) of the frame.
     *
     * The dof is the number of joint values that are used for controlling
     * the frame.
     *
     * Given a set joint values of type State, the getDof() number of joint
     * values for the frame can be read and written with State::getQ() and
     * State::setQ().
     *
     * @return The number of degrees of freedom of the frame.
     */
    int getDOF() const;


    // The parents

#if !defined(SWIGJAVA)
    //! @brief The parent of the frame or NULL if the frame is a DAF.
    const Frame* getParent() const;
#endif

    //! @brief The parent of the frame or NULL if the frame is a DAF.
    Frame* getParent();

    /**
     * @brief Returns the parent of the frame
     *
     * If no static parent exists it look for at DAF parent. If such
     * does not exists either it returns NULL.
     *
     * @param state [in] the state to consider
     * @return the parent
     */
    Frame* getParent(const State& state);

#if !defined(SWIGJAVA)
    /**
     * @brief Returns the parent of the frame
     *
     * If no static parent exists it look for at DAF parent. If such
     * does not exists either it returns NULL.
     *
     * @param state [in] the state to consider
     * @return the parent
     */
    const Frame* getParent(const State& state) const;

    /**
     * @brief The dynamically attached parent or NULL if the frame is not a
     * DAF.
     */
    const Frame* getDafParent(const State& state) const;
#endif

    /**
     * @brief The dynamically attached parent or NULL if the frame is not a
     * DAF.
     */
    Frame* getDafParent(const State& state);

    // Iterator stuff left out of script interface for now!

    // Dynamic frame attachments.

    /**
     * @brief Move a frame within the tree.
     *
     * The frame \b frame is detached from its parent and reattached to \b
     * parent. The frames \b frame and \b parent must both belong to the
     * same kinematics tree.
     *
     * Only frames with no static parent (see getParent()) can be moved.
     *
     * @param parent [in] The frame to attach \b frame to.
     * @param state [inout] The state to which the attachment is written.
     */
    void attachTo(Frame* parent, State& state);

    /**
     * @brief Test if this frame is a Dynamically Attachable Frame
     *
     * @return true if this frame is a DAF, false otherwise
     */
    bool isDAF();

    /**
     * @brief Get the transform relative to world.
     *
     * @param state [in] the state.
     * @return transform relative to world.
     */
    rw::math::Transform3D<double> wTf(const State& state) const;

    /**
     * @brief Get the transform of other frame relative to this frame.
     *
     * @param to [in] the other frame
     * @param state [in] the state.
     * @return transform of frame \b to relative to this frame.
     */
    rw::math::Transform3D<double> fTf(const Frame* to, const State& state) const;
    
    %extend {
        /**
         * @brief Iterator pair for all children of the frame.
         */
        std::vector<Frame*> getChildren(const State& state)
        {
            std::vector<Frame*> frames;
        	Frame::iterator_pair pair = $self->getChildren(state);
        	for (Frame::iterator it = pair.first; it != pair.second; it++) {
        	    frames.push_back(&(*it));
        	}
        	return frames;
        }
    }

private:
    // Frames should not be copied.
    Frame(const Frame&);
    Frame& operator=(const Frame&);
};

%template (FramePtr) rw::common::Ptr<Frame>;
%template (FrameCPtr) rw::common::Ptr<const Frame>;
%template (FrameVector) std::vector<Frame*>;
%template (FramePair) std::pair< Frame *, Frame * >;
%template (FramePairVector) std::vector< std::pair< Frame *, Frame * > >;

/**
 * @brief MovableFrame is a frame for which it is possible to freely
 * change the transform relative to the parent.
 *
 * A MovableFrame can for example be used for modelling objects moving in
 * the scene based on e.g. user input.
 */
class MovableFrame: public Frame{
public:

    /**
     * @brief Construct a MovableFrame with Identiy as the initial
     * transform
     *
     * @param name [in] name of the frame
     */
    explicit MovableFrame(const std::string& name);

   /**
     * @brief Sets the transform in the state. The transform is relative to the
     * MovableFrame's parent frame.
     * @param transform [in] transform to set. the transform is described relative to parent frame
     * @param state [out] state into which to set the transform
     */
    void setTransform(const rw::math::Transform3D<double>& transform, State& state);

    /**
     * @brief Changes the transform in the state, such that the movable frame is located in the
     * transform which is described relative to world.
     * @param transform [in] transform to set. transform is described relative to world frame
     * @param state [out] state into which to set the transform
     */
    void moveTo(const rw::math::Transform3D<double>& transform, State& state);

    /**
     * @brief Changes the transform in the state, such that the movable frame is located in the
     * transform which is described relative to refframe
     * @param transform [in] transform to set. transform is described relative to refframe
     * @param refframe [in] the reference frame.
     * @param state [out] state into which to set the transform
     */
    void moveTo(const rw::math::Transform3D<double>& transform, Frame* refframe, State& state);

};

class FixedFrame: public Frame {
public:
    FixedFrame(const std::string& name, const rw::math::Transform3D<double> & transform);
    void setTransform(const rw::math::Transform3D<double> & transform);

    const rw::math::Transform3D<double> & getFixedTransform() const;
};

%inline %{
    rw::math::Transform3D<double>  frameTframe(const Frame* from, const Frame* to, const State& state){
        return ::rw::kinematics::Kinematics::frameTframe(from, to, state );
    }

    rw::math::Transform3D<double>  worldTframe(const Frame* to, const State& state){
        return ::rw::kinematics::Kinematics::worldTframe( to,  state);
    }

    Frame* worldFrame(Frame* frame, const State& state) {
        return ::rw::kinematics::Kinematics::worldFrame( frame, state );
    }

    void gripFrame(Frame* item, Frame* gripper, State& state){
        return ::rw::kinematics::Kinematics::gripFrame( item, gripper, state);
    }

    void gripFrame(MovableFrame* item, Frame* gripper, State& state){
        return ::rw::kinematics::Kinematics::gripFrame( item, gripper, state);
    }

    bool isDAF(const Frame* frame){
        return ::rw::kinematics::Kinematics::isDAF( frame );
    }
%}

namespace rw { namespace kinematics {


    template <class T>
    class FrameMap {
    public:

        /**
         * @brief creates a framemap with an initial size of s
         * @param s [in] nr of elements of the types T with default value "defaultVal"
         * @param defaultVal [in] the default value of new instances of T
         */
        FrameMap(const T& defaultVal, int s = 20);

        /**
         * @brief creates a framemap
         * @param s [in]  nr of elements of the types T
         */
        FrameMap(int s = 20);

        /**
         * @brief inserts a value into the frame map
         * @param frame [in] the frame for which the value is to be associated
         * @param value [in] the value that is to be associated to the frame
         */
        void insert(const Frame& frame, const T& value);

        /**
           @brief True iff a value for \b frame has been inserted in the map (or
           accessed using non-const operator[]).
        */
        bool has(const Frame& frame);

        %extend{
        #if (defined(SWIGLUA) || defined(SWIGPYTHON))
            T __getitem__(const Frame& i)const {return (*$self)[i]; }
            void __setitem__(const Frame& i,T d){ (*$self)[i] = d; }
        #elif defined(SWIGJAVA)
            T get(const Frame& i) const { return (*$self)[i]; }
            void set(const Frame& i,T d){ (*$self)[i] = d; }
        #endif
        }

        /**
         * @brief Erase an element from the map
         */
        void erase( const Frame& frame );

        /**
           @brief Clear the frame map.
        */
        void clear();
    };
}}

%template(FrameMap) rw::kinematics::FrameMap< double >;

class Joint: public Frame
{
};

%template (JointVector) std::vector<Joint*>;

%nodefaultctor Kinematics;
/**
 * @brief Utility functions for the rw::kinematics module.
 */
class Kinematics {
public:
    /**
     * @brief The transform of frame in relative to the world frame.
     *
     * If to=NULL the method returns a 4x4 identify matrix
     *
     * @param to [in] The transform for which to find the world frame.
     *
     * @param state [in] The state of the kinematics tree.
     *
     * @return The transform of the frame relative to the world frame.
     */
    static rw::math::Transform3D<double> worldTframe(const Frame* to, const State& state);


    /**
     * @brief The transform of frame to relative to frame from.
     *
     * FrameTframe() is related to WorldTframe() as follows:
     *
     * frameTframe(from, to, state) == inverse(worldTframe(from, state)) * worldTframe(to, state);
     *
     * @param from [in] The start frame.
     *
     * @param to [in] The end frame.
     *
     * @param state [in] The state of the kinematics tree.
     *
     * @return The transform from the start frame to the end frame.
     */
    static rw::math::Transform3D<double> frameTframe(
        const Frame* from, const Frame* to, const State& state);


    /** 
     * @brief All frames reachable from root for a tree structure of
     * state.
     *
     * This is a tremendously useful utility. An alternative would be to have an
     * iterator interface for trees represented by work cell states.
     *
     * We give no guarantee on the ordering of the frames.
     *
     * @param root [in] The root node from where the frame search is started.
     *
     * @param state [in] The structure of the tree.
     *
     * @return All reachable frames.
     */
    static std::vector<Frame*> findAllFrames(Frame* root, const State& state);

    /** 
     * @brief All frames reachable from root for a tree structure.
     *
     * This is a tremendously useful utility. An alternative would be to have an
     * iterator interface for trees represented by work cell states.
     *
     * We give no guarantee on the ordering of the frames.
     *
     * DAF are not included.
     *
     * @param root [in] The root node from where the frame search is started.
     *
     * @return All reachable frames.
     */
    static std::vector<Frame*> findAllFrames(Frame* root);

    /**
     * @brief Find the world frame of the workcell by traversing the path
     * from frame to the root of the tree.
     *
     * The state state is needed to retrieve the parent frames, but the
     * world frame returned is the same for any (valid) state.
     */
    static Frame* worldFrame(Frame* frame, const State& state);

    /**
     * @brief The chain of frames connecting child to parent.
     *
     * child is included in the chain, but parent is not included. If
     * parent is NULL then the entire path from child to the world
     * frame is returned. If child as well as parent is NULL then the
     * empty chain is gracefully returned.
     *
     * The state gives the connectedness of the tree.
     *
     * If parent is not on the chain from child towards the root, then
     * an exception is thrown.
     */
    static std::vector<Frame*> childToParentChain(Frame* child,
                                                  Frame* parent,
                                                  const State& state);

    /**
     * @brief Like ChildToParentChain() except that the frames are returned
     * in the reverse order.
     */
    static std::vector<Frame*> reverseChildToParentChain(Frame* child,
                                                         Frame* parent,
                                                         const State& state);

    /**
     * @brief The chain of frames connecting parent to child.
     *
     * parent is included in the list, but child is excluded. If
     * parent as well as child is NULL then the empty chain is returned.
     * Otherwise parent is included even if parent is NULL.
     */
    static std::vector<Frame*> parentToChildChain(Frame* parent,
                                                  Frame* child,
                                                  const State& state);

    /**
       @brief True if frame is a DAF and false otherwise.
    */
    static bool isDAF(const Frame* frame);

    /**
     * @brief Check if frame is fixed.
     * @param frame [in] the frame.
     * @return true if fixed, false otherwise.
     */
    static bool isFixedFrame(const Frame* frame);

    /**
     * @brief Grip item with gripper thereby modifying state.
     *
     * item must be a DAF.
     *
     * @param item [in] the frame to grip.
     * @param gripper [in] the grasping frame.
     * @param state [in/out] the state.
     *
     * An exception is thrown if item is not a DAF.
     */
	static void gripFrame(Frame* item, Frame* gripper, State& state);

    /**
     * @brief Grip item with gripper thereby modifying state.
     *
     * item must be a DAF.
     *
     * @param item [in] the frame to grip.
     * @param gripper [in] the grasping frame.
     * @param state [in/out] the state.
     *
     * An exception is thrown if item is not a DAF.
     */
    static void gripFrame(MovableFrame* item, Frame* gripper, State& state);
};

/**
 * @brief Forward kinematics between a pair of frames.
 *
 * FKRange finds the relative transform between a pair of frames. FKRange
 * finds the path connecting the pair of frames and computes the total
 * transform of this path. Following initialization, FKRange assumes that
 * the path does not change structure because of uses of the attachFrame()
 * feature. If the structure of the path has changed, the FKRange will
 * produce wrong results.
 *
 * FKRange is guaranteed to select the shortest path connecting the
 * frames, i.e. the path doesn't go all the way down to the root if it can
 * be helped.
 */
class FKRange
{
public:
    /**
     * @brief Forward kinematics for the path leading from from to to.
     *
     * If a frame of NULL is passed as argument, it is interpreted to mean
     * the WORLD frame.
     *
     * @param from [in] The start frame.
     *
     * @param to [in] The end frame.
     *
     * @param state [in] The path structure.
     */
    FKRange(const Frame* from, const Frame* to, const State& state);

    /**
     * @brief Default constructor
     *
     * Will always return an identity matrix as the transform
     */
    FKRange();

    /**
     * @brief The relative transform between the frames.
     *
     * @param state [in] Configuration values for the frames of the tree.
     */
    rw::math::Transform3D<double> get(const State& state) const;

    /**
     * @brief Returns the last frame in the range.
     *
     * @return The end frame (to).
     */
    rw::common::Ptr< const Frame > getEnd() const;

    /**
     * @brief Returns the first frame in the range.
     *
     * @return The base frame (from).
     */
    rw::common::Ptr< const Frame > getBase() const;
};

/**
 * @brief Forward kinematics for a set of frames.
 *
 * FKTable finds transforms for frames for a given fixed work cell state.
 * The frame transforms are calculated relative to the world frame.
 */
class FKTable
{
public:
    /**
     * @brief Forward kinematics for the work cell state state.
     *
     * @param state [in] The work state for which world transforms are to be
     * calculated.
     */
    FKTable(const State& state);

    /**
     * @brief The world transform for the frame frame.
     *
     * @param frame [in] The frame for which to find the world transform.
     *
     * @return The transform of the frame relative to the world.
     */
    const rw::math::Transform3D<double>& get(const Frame& frame) const;

    /**
     * @brief Returns State associated with the FKTable
     *
     * The State returned is the State used to calculate the forward kinematics.
     *
     * @return State used to calculate the forward kinematics
     */
    const State& getState() const;

    /**
     * @brief resets the FKTable to state
     *
     * @param state
     */
    void reset(const State& state);
};

/********************************************
 * LOADERS
 ********************************************/

/**
 * @brief Extendible interface for loading of WorkCells from files.
 *
 * By default, the following formats are supported:
 *
 * - File extensions ".wu", ".wc", ".tag", ".dev" will be loaded using
 *   the TULLoader.
 * - Remaining file extensions will be loaded using the standard RobWork
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
 * @beginJavaOnly <pre> \code
 * WorkCellPtr wc = WorkCellLoaderFactory.load("scene.wc.xml");
 * if (wc.isNull())
 *     throw new Exception("WorkCell could not be loaded.");
 * \endcode </pre> @endJavaOnly
 * Alternatively a WorkCell can be loaded in the less convenient way:
 * @beginPythonOnly
 * ::\n
 *    loader = WorkCellLoaderFactory.getWorkCellLoader(".wc.xml");
 *    wc = loader.load("scene.wc.xml")
 *    if wc.isNull():
 *        raise Exception("WorkCell could not be loaded")
 * @endPythonOnly
 * @beginJavaOnly <pre> \code
 * WorkCellLoaderPtr loader = WorkCellLoaderFactory.getWorkCellLoader(".wc.xml");
 * WorkCellPtr wc = loader.loadWorkCell("scene.wc.xml");
 * if (wc.isNull())
 *     throw new Exception("WorkCell could not be loaded.");
 * \endcode </pre> @endJavaOnly
 */
class WorkCellLoader {
public:
	virtual ~WorkCellLoader();
    /**
     * @brief Load a WorkCell from a file.
     *
     * @param filename [in] path to workcell file.
     */
	virtual rw::common::Ptr<WorkCell> loadWorkCell(const std::string& filename) = 0;

protected:
	WorkCellLoader();
};

%template (WorkCellLoaderPtr) rw::common::Ptr<WorkCellLoader>;

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
	static rw::common::Ptr<WorkCellLoader> getWorkCellLoader(const std::string& format);

    /**
     * @brief Loads/imports a WorkCell from a file.
     *
     * An exception is thrown if the file can't be loaded.
     * The RobWork XML format is supported by default, as well as
     * TUL WorkCell format.
     *
     * @param filename [in] name of the WorkCell file.
     */
	static rw::common::Ptr<WorkCell> load(const std::string& filename);
private:
	WorkCellLoaderFactory();
};

class ImageLoader {
public:
	virtual ~ImageLoader();
	virtual rw::common::Ptr<Image> loadImage(const std::string& filename) = 0;
	virtual std::vector<std::string> getImageFormats() = 0;
	virtual bool isImageSupported(const std::string& format);
};

%template (ImageLoaderPtr) rw::common::Ptr<ImageLoader>;

class ImageLoaderFactory {
public:
	ImageLoaderFactory();
	static rw::common::Ptr<ImageLoader> getImageLoader(const std::string& format);
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
    rw::common::Ptr<Trajectory<rw::math::Q> > getQTrajectory();
    rw::common::Ptr<Trajectory<rw::math::Vector3D<double> > > getVector3DTrajectory();
    rw::common::Ptr<Trajectory<rw::math::Rotation3D<double> > > getRotation3DTrajectory();
    rw::common::Ptr<Trajectory<rw::math::Transform3D<double> > > getTransform3DTrajectory();
};

class XMLTrajectorySaver
{
public:
    static bool save(const Trajectory<rw::math::Q>& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Vector3D<double> >& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Rotation3D<double> >& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Transform3D<double> >& trajectory, const std::string& filename);
    static bool write(const Trajectory<rw::math::Q>& trajectory, std::ostream& outstream);
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
%include <rwlibs/swig/rw_i/math.i>

// Utility function within rw::Math
rw::math::Rotation3D<double> getRandomRotation3D();
rw::math::Transform3D<double>  getRandomTransform3D(const double translationLength = 1);

namespace rw { namespace math {
    class Math
    {
    public:
        Math() = delete;
        ~Math() = delete;

        /**
         * @brief Quaternion to equivalent angle axis conversion.
         *
         * @param quat [in] the Quaternion object that is to be converted.
         *
         * @return a EAA object that represents the converted quaternion
         */
        template <class A>
        static rw::math::EAA<A> quaternionToEAA(const rw::math::Quaternion<A> &quat);

        /**
         * @brief Equivalent angle axis to quaternion conversion.
         *
         * @param eaa [in] the EAA object that is to be converted
         *
         * @return a Quaternion object that represents the converted EAA
         */
        template <class A>
        static rw::math::Quaternion<A> eaaToQuaternion(const rw::math::EAA<A> &eaa);

        static inline double clamp(double val, double min, double max);

        static rw::math::Q clampQ(const rw::math::Q& q,
                                  const rw::math::Q& min,
                                  const rw::math::Q& max);

        static rw::math::Q clampQ(const rw::math::Q& q,
                                  const std::pair<rw::math::Q, rw::math::Q>& bounds);

        static rw::math::Vector3D<double> clamp(const rw::math::Vector3D<double>& q,
                                          const rw::math::Vector3D<double>& min,
                                          const rw::math::Vector3D<double>& max);

        static double ran();

        static void seed(unsigned seed);

        static void seed();

        static double ran(double from, double to);

        static int ranI(int from, int to);

        static double ranNormalDist(double mean, double sigma);

        static rw::math::Q ranQ(const rw::math::Q& from, const rw::math::Q& to);

        static rw::math::Q ranQ(const std::pair<rw::math::Q,rw::math::Q>& bounds);

        static rw::math::Q ranDir(size_t dim, double length = 1);
        
        static rw::math::Q ranWeightedDir(size_t dim, const rw::math::Q& weights, double length = 1);

        static double round(double d);

        static rw::math::Q sqr(const rw::math::Q& q);

        static rw::math::Q sqrt(const rw::math::Q& q);

        static rw::math::Q abs(const rw::math::Q& v);

        static double min(const rw::math::Q& v);

        static double max(const rw::math::Q& v);

        static double sign(double s);

        static rw::math::Q sign(const rw::math::Q& q);

        static int ceilLog2(int n);
        
        static long long factorial(long long n);

        static bool isNaN(double d);
    };
}} // end namespaces

%template (quaternionToEAA) rw::math::Math::quaternionToEAA<double>;
%template (quaternionToEAA) rw::math::Math::quaternionToEAA<float>;
%template (eaaToQuaternion) rw::math::Math::eaaToQuaternion<double>;
%template (eaaToQuaternion) rw::math::Math::eaaToQuaternion<float>;


/********************************************
 * MODELS
 ********************************************/
 
 %nodefaultctor JacobianCalculator;
//! @brief JacobianCalculator provides an interface for obtaining a Jacobian
class JacobianCalculator
{
public:
    //! @brief Destructor
    virtual ~JacobianCalculator();

    %extend {
		/**
		 * @brief Returns the Jacobian associated to \b state
		 *
		 * @param state [in] State for which to calculate the Jacobian
		 * @return Jacobian for \b state
		 */
		virtual rw::math::Jacobian getJacobian(const State& state) const {
			return $self->get(state);
		}
    };
};

%template (JacobianCalculatorPtr) rw::common::Ptr<JacobianCalculator>;

class WorkCell {
public:
    /**
     * @brief Constructs an empty WorkCell
     *
     * @param name [in] The name of the workcell. A good name for the
     * workcell would be the (eventual) file that the workcell was
     * loaded from.
     */
    WorkCell(const std::string& name);

    /**
     * @brief The name of the workcell or the empty string if no name
     * was provided.
     * @return the name of the workcell
     */
    std::string getName() const;

    /**
     * @brief Returns pointer to the world frame
     *
     * @return Pointer to the world frame
     */
    Frame* getWorldFrame() const;

    /**
     * @brief Adds \b frame with \b parent as parent.
     *
     * If parent == NULL, then \b world is used as parent
     *
     * @param frame [in] Frame to add
     * @param parent [in] Parent frame - uses World is parent == NULL
     * @deprecated Since January 2018.
     * Please use the addFrame method using smart pointers instead.
     */
    void addFrame(Frame* frame, Frame* parent=NULL);

    /**
     * @brief Adds \b frame with \b parent as parent.
     *
     * If parent == NULL, then \b world is used as parent
     *
     * @param frame [in] Frame to add
     * @param parent [in] Parent frame - uses World is parent == NULL
     */
    void addFrame(rw::common::Ptr<Frame> frame,
            rw::common::Ptr<Frame> parent = NULL);


    /**
     * @brief Adds dynamically attachable frame (DAF) \b frame with
     * \b parent as parent.
     *
     * If parent == NULL, then \b world is used as parent
     *
     * @param frame [in] Frame to add
     * @param parent [in] Parent frame - uses World is parent == NULL
     * @deprecated Since January 2018.
     * Please use the addDAF method using smart pointers instead.
     */
    void addDAF(Frame* frame, Frame* parent = NULL);

    /**
     * @brief Adds dynamically attachable frame (DAF) \b frame with
     * \b parent as parent.
     *
     * If parent == NULL, then \b world is used as parent
     *
     * @param frame [in] Frame to add
     * @param parent [in] Parent frame - uses World is parent == NULL
     */
    void addDAF(rw::common::Ptr<Frame> frame,
            rw::common::Ptr<Frame> parent = NULL);

    /**
     * @brief Removes \b frame from work cell
     *
     * @param frame [in] Frame to remove
     * @deprecated Since January 2018.
     * Please use remove(rw::common::Ptr<rw::kinematics::Frame>)
     * instead.
     */
    void remove(Frame* frame);

    /**
     * @brief Removes \b frame from work cell
     *
     * @param frame [in] Frame to remove
     */
    void remove(rw::common::Ptr<Frame> frame);

    /**
     * @brief Adds a Device to the WorkCell.
     *
     * Ownership of \b device is taken.
     *
     * @param device [in] pointer to device.
     */
    void addDevice(rw::common::Ptr<Device> device);

    /**
     * @brief Returns a reference to a vector with pointers to the
     * Device(s) in the WorkCell
     *
     * @return const vector with pointers to Device(s).
     */
    const std::vector<rw::common::Ptr<Device> >& getDevices() const;

    /**
     * @brief Returns frame with the specified name.
     *
     * If multiple frames has the same name, the first frame encountered
     * will be returned. If no frame is found, the method returns NULL.
     *
     * @param name [in] name of Frame.
     *
     * @return The frame with name \b name or NULL if no such frame.
     */
    Frame* findFrame(const std::string& name) const;

    %extend {
        /**
         * @brief Returns MovableFrame with the specified name.
         *
         * If multiple frames has the same name, the first frame encountered
         * will be returned. If no frame is found, the method returns NULL.
         *
         * @param name [in] name of Frame.
         *
         * @return The MovableFrame with name \b name or NULL if no such frame.
         */
        MovableFrame* findMovableFrame(const std::string& name)
        { 
            return $self->WorkCell::findFrame<MovableFrame>(name); 
        }

        /**
         * @brief Returns FixedFrame with the specified name.
         *
         * If multiple frames has the same name, the first frame encountered
         * will be returned. If no frame is found, the method returns NULL.
         *
         * @param name [in] name of Frame.
         *
         * @return The FixedFrame with name \b name or NULL if no such frame.
         */
        FixedFrame* findFixedFrame(const std::string& name)
        { 
            return $self->WorkCell::findFrame<FixedFrame>(name); 
        }

        /**
         * @brief Returns all \b MovableFrames.
         * @return all frames of type \b MovableFrames in the workcell
         */
        std::vector<MovableFrame*> findMovableFrames() const
        { 
            return $self->WorkCell::findFrames<MovableFrame>(); 
        }

        /**
         * @brief Returns all \b FixedFrame.
         * @return all frames of type \b FixedFrame in the workcell
         */
        std::vector<FixedFrame*> findFixedFrames() const
        { 
            return $self->WorkCell::findFrames<FixedFrame>(); 
        }

    };

    /**
     * @brief Returns all frames in workcell
     * @return List of all frames
     */
    std::vector<Frame*> getFrames() const;

    /**
     * @brief The device named \b name of the workcell.
     *
     * NULL is returned if there is no such device.
     *
     * @param name [in] The device name
     *
     * @return The device named \b name or NULL if no such device.
     */
    rw::common::Ptr<Device> findDevice(const std::string& name) const;

    %extend {
        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device.
         *
         * @param name [in] The device name
         *
         * @return The device named \b name or NULL if no such device.
         */
        rw::common::Ptr<JointDevice> findJointDevice(const std::string& name)
        { 
            return $self->WorkCell::findDevice<JointDevice>(name); 
        }

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device.
         *
         * @param name [in] The device name
         *
         * @return The device named \b name or NULL if no such device.
         */
        rw::common::Ptr<SerialDevice> findSerialDevice(const std::string& name)
        { 
            return $self->WorkCell::findDevice<SerialDevice>(name); 
        }

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device.
         *
         * @param name [in] The device name
         *
         * @return The device named \b name or NULL if no such device.
         */
        rw::common::Ptr<TreeDevice> findTreeDevice(const std::string& name)
        { 
            return $self->WorkCell::findDevice<TreeDevice>(name); 
        }

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device.
         *
         * @param name [in] The device name
         *
         * @return The device named \b name or NULL if no such device.
         */
        rw::common::Ptr<ParallelDevice> findParallelDevice(const std::string& name)
        { 
            return $self->WorkCell::findDevice<ParallelDevice>(name); 
        }

        /**
         * @brief Returns a vector with pointers to the Device(s) with a
         * specific type \b JointDevice in the WorkCell
         *
         * @return vector with pointers to Device(s) of type T.
         */
        std::vector < rw::common::Ptr<JointDevice> > findJointDevices()
        { 
            return $self->WorkCell::findDevices<JointDevice>(); 
        }

        /**
         * @brief Returns a vector with pointers to the Device(s) with a
         * specific type \b SerialDevice in the WorkCell
         *
         * @return vector with pointers to Device(s) of type T.
         */
        std::vector < rw::common::Ptr<SerialDevice> > findSerialDevices()
        { 
            return $self->WorkCell::findDevices<SerialDevice>(); 
        }

        /**
         * @brief Returns a vector with pointers to the Device(s) with a
         * specific type \b TreeDevice in the WorkCell
         *
         * @return vector with pointers to Device(s) of type T.
         */
        std::vector < rw::common::Ptr<TreeDevice> > findTreeDevices()
        { 
            return $self->WorkCell::findDevices<TreeDevice>(); 
        }

        /**
         * @brief Returns a vector with pointers to the Device(s) with a
         * specific type \b ParallelDevice in the WorkCell
         *
         * @return vector with pointers to Device(s) of type T.
         */
        std::vector < rw::common::Ptr<ParallelDevice> > findParallelDevices()
        { 
            return $self->WorkCell::findDevices<ParallelDevice>(); 
        }
    };

    /**
     * @brief Returns a default State
     *
     * @return default State
     */
    State getDefaultState() const;

    /**
     * @brief Returns sensor with the specified name.
     *
     * If multiple sensors has the same name, the first sensor
     * encountered will be returned. If no sensor is found, the method
     * returns NULL.
     *
     * @param name [in] name of sensor.
     *
     * @return The sensor with name \b name or NULL if no such sensor.
     */
    rw::common::Ptr<SensorModel> findSensor(const std::string& name) const;

    //TODO(kalor) findSensor<T>(name);
    //TODO(kalor) findSensors<T>();

    /**
     * @brief Returns all frames in workcell
     * @return List of all frames
     */
    std::vector<rw::common::Ptr<SensorModel> > getSensors() const;


    //TODO(kalor) findController<T>(name);
    //TODO(kalor) findControllers<T>();

    /**
     * @brief Returns controller with the specified name.
     *
     * If multiple controlelrs has the same name, the first controller
     * encountered will be returned. If no controller is found, the
     * method returns NULL.
     *
     * @param name [in] name of controller.
     *
     * @return The controller with name \b name or NULL if no such
     * controller.
     */
    rw::common::Ptr<rw::models::ControllerModel> findController(const std::string& name) const;
    
    /**
     * @brief Returns all controllers in workcell
     * @return List of all controllers
     */
    std::vector<rw::common::Ptr<ControllerModel> > getControllers() const;

    /**
     * @brief Returns all object in the work cell
     *
     * @return All object in work cell
     */
    std::vector<rw::common::Ptr<Object> > getObjects() const;

    /**
     * @brief The object named \b name of the workcell.
     *
     * NULL is returned if there is no such object.
     *
     * @param name [in] The object name
     *
     * @return The object named \b name or NULL if no such object.
     */
    rw::common::Ptr<Object> findObject(const std::string& name) const;

    //! @brief Add device to workcell
    void add(rw::common::Ptr<Device> device);
    //! @brief Add object to workcell
    void add(rw::common::Ptr<Object> object);
    //! @brief Add sensormodel to workcell
    void add(rw::common::Ptr<SensorModel> sensor);
    //! @brief Add controllermodel to workcell
    void add(rw::common::Ptr<ControllerModel> controller);

    //! @brief Remove object from workcell
    void remove(rw::common::Ptr<Object> object);
    //! @brief Remove device from workcell
    void remove(rw::common::Ptr<Device> device);
    //! @brief Remove sensormodel from workcell
    void remove(rw::common::Ptr<SensorModel> sensor);
    //! @brief Remove controllermodel from workcell
    void remove(rw::common::Ptr<ControllerModel> controller);

    std::string getFilename () const;
    std::string getFilePath () const;

    
    PropertyMap& getPropertyMap();
private:
    WorkCell(const WorkCell&);
    WorkCell& operator=(const WorkCell&);
};

%template (WorkCellPtr) rw::common::Ptr<WorkCell>;

class Object
{
public:
    //! destructor
    virtual ~Object();
    const std::string& getName();
    Frame* getBase();
    const std::vector<Frame*>& getFrames();
    void addFrame(Frame* frame);
    const std::vector<rw::common::Ptr<Geometry> >& getGeometry() const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels() const;

    // stuff that should be implemented by deriving classes
     const std::vector<rw::common::Ptr<Geometry> >& getGeometry(const State& state) const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels(const State& state) const;
    virtual double getMass(State& state) const = 0;
    virtual rw::math::Vector3D<double> getCOM(State& state) const = 0;
    virtual rw::math::InertiaMatrix<double> getInertia(State& state) const = 0;
};
%template (ObjectPtr) rw::common::Ptr<Object>;
OWNEDPTR(Object);

class RigidObject : public Object {
public:
	RigidObject(Frame* baseframe);
	RigidObject(Frame* baseframe, rw::common::Ptr<Geometry> geom);
	RigidObject(Frame* baseframe, std::vector<rw::common::Ptr<Geometry> > geom);
	RigidObject(std::vector<Frame*> frames);
	RigidObject(std::vector<Frame*> frames, rw::common::Ptr<Geometry> geom);
	RigidObject(std::vector<Frame*> frames, std::vector<rw::common::Ptr<Geometry> > geom);
	void addGeometry(rw::common::Ptr<Geometry> geom);
	void removeGeometry(rw::common::Ptr<Geometry> geom);
	void addModel(rw::common::Ptr<Model3D> model);
	void removeModel(rw::common::Ptr<Model3D> model);
    double getMass() const;
    void setMass(double mass);
    rw::math::InertiaMatrix<double> getInertia() const;
    void setInertia(const rw::math::InertiaMatrix<double>& inertia);
    void setCOM(const rw::math::Vector3D<double>& com);
    void approximateInertia();
    void approximateInertiaCOM();
    const std::vector<rw::common::Ptr<Geometry> >& getGeometry() const ;
    const std::vector<rw::common::Ptr<Model3D> >& getModels() const;
    double getMass(State& state) const;
    rw::math::InertiaMatrix<double> getInertia(State& state) const;
    rw::math::Vector3D<double> getCOM(State& state) const;
};

%template (RigidObjectPtr) rw::common::Ptr<RigidObject>;
OWNEDPTR(RigidObject);

class DeformableObject: public Object
{
public:

    /**
     * @brief constructor - constructs a deformable mesh with a specific number of control nodes
     * and without any faces. Both geometry and model are created based on nodes.
     * @param baseframe [in] base frame of object
     * @param nr_of_nodes [in] the number of controlling nodes in the deformable object
     */
    DeformableObject(Frame* baseframe, int nr_of_nodes);

    //DeformableObject(Frame* baseframe, rw::common::Ptr<Model3D> model);

    //DeformableObject(Frame* baseframe, rw::common::Ptr<Geometry> geom);

    //! destructor
    virtual ~DeformableObject();


    /**
     * @brief get a specific node from the state
     * @param id [in] id of the node to fetch
     * @param state [in] current state
     * @return handle to manipulate a node in the given state.
     */
    rw::math::Vector3D<float>& getNode(int id, State& state) const;

    /**
     * @brief set the value of a specific node in the state.
     * @param id [in] id of the node
     * @param v [in] value to set.
     * @param state [in] state in which to set the value.
     */
    void setNode(int id, const rw::math::Vector3D<float>& v, State& state);

    /**
     * @brief get the number of controlling nodes of this deformable object.
     * @param state [in]
     * @return
     */
    size_t getNrNodes(const rw::kinematics::State& state) const ;
    
    /*
     * @brief get all faces of this soft body
     * @return list of indexed triangles - indeces point to vertices/nodes
     */
    //const std::vector<rw::geometry::IndexedTriangle<> >& getFaces() const;

    /**
     * @brief add a face to three existing nodes
     * @param node1 [in] idx of node 1
     * @param node2 [in] idx of node 2
     * @param node3 [in] idx of node 3
     */
    void addFace(unsigned int node1, unsigned int node2, unsigned int node3);

    /*
     * @brief return a triangle mesh representing the softbody in the current state
     * \b cstate
     * @param cstate
     */
    //rw::geometry::IndexedTriMesh<float>::Ptr getMesh(State& cstate);

    //! @copydoc rw::models::Object::getGeometry
    const std::vector<rw::common::Ptr<Geometry> >& getGeometry(const State& state) const;

    //! @copydoc rw::models::Object::getModels
    const std::vector<rw::common::Ptr<Model3D> >& getModels() const;


    //! @copydoc rw::models::Object::getModels
    const std::vector<rw::common::Ptr<Model3D> >& getModels(const State& state) const;
    
    /**
     * @brief get mass in Kg of this object
     * @param state [in] the state
     * @return mass in kilo grams
     */
    double getMass(State& state) const;

    /**
     * @brief get center of mass of this object
     * @param state [in] the state in which to get center of mass
     * @return Position of COM
     */    
    rw::math::Vector3D<double> getCOM(State& state) const;


    /**
     * @brief returns the inertia matrix of this body calculated around COM with the orientation
     * of the base frame.
     * @param state [in] the state to get the inertia in
     * @return matrix with inertia 
     */
    rw::math::InertiaMatrix<double> getInertia(State& state) const;

    /**
     * @brief updates the model with the current state of the deformable model
     * @param model [in/out] model to be updated
     * @param state
     */
    void update(rw::common::Ptr<Model3D> model, const State& state);
};

%template (DeformableObjectPtr) rw::common::Ptr<DeformableObject>;
OWNEDPTR(DeformableObject);

class Device
{
public:
    Device(const std::string& name);
    //void registerStateData(rw::kinematics::StateStructure::Ptr sstruct);
    virtual void setQ(const rw::math::Q& q, State& state) const = 0;
    virtual rw::math::Q getQ(const State& state) const = 0;
    virtual std::pair<rw::math::Q,rw::math::Q> getBounds() const = 0;
    virtual rw::math::Q getVelocityLimits() const = 0;
    virtual void setVelocityLimits(const rw::math::Q& vellimits) = 0;
    virtual rw::math::Q getAccelerationLimits() const = 0;
    virtual void setAccelerationLimits(const rw::math::Q& acclimits) = 0;
    virtual size_t getDOF() const = 0;
    std::string getName() const;
    void setName(const std::string& name);
    virtual Frame* getBase() = 0;
    virtual Frame* getEnd() = 0;
#if !defined(SWIGJAVA)
    virtual const Frame* getBase() const = 0;
    virtual const Frame* getEnd() const = 0;
#endif
    rw::math::Transform3D<double>  baseTframe(const Frame* f, const State& state) const;
    rw::math::Transform3D<double>  baseTend(const State& state) const;
    rw::math::Transform3D<double>  worldTbase(const State& state) const;
    virtual rw::math::Jacobian baseJend(const State& state) const = 0;
    virtual rw::math::Jacobian baseJframe(const Frame* frame,const State& state) const;
    virtual rw::math::Jacobian baseJframes(const std::vector<Frame*>& frames,const State& state) const;
    //virtual rw::common::Ptr<JacobianCalculator> baseJCend(const State& state) const;
    //virtual JacobianCalculatorPtr baseJCframe(const Frame* frame, const State& state) const;
    //virtual JacobianCalculatorPtr baseJCframes(const std::vector<Frame*>& frames, const State& state) const = 0;
private:
    Device(const Device&);
    Device& operator=(const Device&);
};

%template (DevicePtr) rw::common::Ptr<Device>;
%template (DeviceCPtr) rw::common::Ptr<const Device>;
%template (DevicePtrVector) std::vector<rw::common::Ptr<Device> >;
OWNEDPTR(Device)

%extend rw::common::Ptr<Device> {
    rw::common::Ptr<const Device> asDeviceCPtr() { return *$self; }
}

class JointDevice: public Device
{
public:
    const std::vector<Joint*>& getJoints() const;
    void setQ(const rw::math::Q& q, State& state) const;
    rw::math::Q getQ(const State& state) const;
    size_t getDOF() const;
    std::pair<rw::math::Q, rw::math::Q> getBounds() const;
    void setBounds(const std::pair<rw::math::Q, rw::math::Q>& bounds);
    rw::math::Q getVelocityLimits() const;
    void setVelocityLimits(const rw::math::Q& vellimits);
    rw::math::Q getAccelerationLimits() const;
    void setAccelerationLimits(const rw::math::Q& acclimits);
    rw::math::Jacobian baseJend(const State& state) const;

    //JacobianCalculatorPtr baseJCframes(const std::vector<Frame*>& frames,
    //                                   const State& state) const;

    Frame* getBase();
    virtual Frame* getEnd();

#if !defined(SWIGJAVA)
    const Frame* getBase() const;
    virtual const Frame* getEnd() const;
#endif
};

%template (JointDevicePtr) rw::common::Ptr<JointDevice>;
%template (JointDeviceCPtr) rw::common::Ptr<const JointDevice>;
OWNEDPTR(JointDevice)

class CompositeDevice: public JointDevice
{
public:
    CompositeDevice(
        Frame* base,
		const std::vector<rw::common::Ptr<Device> >& devices,
        Frame* end,
        const std::string& name,
        const State& state);

    CompositeDevice(
        Frame *base,
		const std::vector<rw::common::Ptr<Device> >& devices,
        const std::vector<Frame*>& ends,
        const std::string& name,
        const State& state);
};

%template (CompositeDevicePtr) rw::common::Ptr<CompositeDevice>;
OWNEDPTR(CompositeDevice)

%extend rw::common::Ptr<CompositeDevice> {
    rw::common::Ptr<Device> asDevicePtr() { return *$self; }
    rw::common::Ptr<const Device> asDeviceCPtr() { return *$self; }
    rw::common::Ptr<JointDevice> asJointDevicePtr() { return *$self; }
    rw::common::Ptr<const JointDevice> asJointDeviceCPtr() { return *$self; }
}

class SerialDevice: public JointDevice
{
};
%template (SerialDevicePtr) rw::common::Ptr<SerialDevice>;
%template (SerialDeviceCPtr) rw::common::Ptr<const SerialDevice>;
OWNEDPTR(SerialDevice)

%extend rw::common::Ptr<SerialDevice> {
    rw::common::Ptr<const SerialDevice> asSerialDeviceCPtr() { return *$self; }
}

class ParallelDevice: public JointDevice
{
};
%template (ParallelDevicePtr) rw::common::Ptr<ParallelDevice>;
OWNEDPTR(ParallelDevice)

class TreeDevice: public JointDevice
{
public:
	TreeDevice(
		Frame* base,
		const std::vector<Frame*>& ends,
		const std::string& name,
		const State& state);
};
%template (TreeDevicePtr) rw::common::Ptr<TreeDevice>;
%template (TreeDeviceCPtr) rw::common::Ptr<const TreeDevice>;
OWNEDPTR(TreeDevice)

%nodefaultctor DHParameterSet;
class DHParameterSet
{
};

%template (DHParameterSetVector) std::vector<DHParameterSet>;

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

%include <rwlibs/swig/rw_i/proximity.i>

/********************************************
 * SENSOR
 ********************************************/

%nodefaultctor Sensor;
/**
 * @brief a generel hardware sensor interface. The sensor should interface
 * to a statefull instance of either a real world sensor or a simulated
 * sensor. The sensor interface acts as a realistic handle to controlling
 * some specific instance of a sensor.
 */
class Sensor
{
public:
    //! destructor
    virtual ~Sensor();

    /**
     * @brief returns the name of this sensor
     *
     * @return name of sensor
     */
    const std::string& getName() const;

    /**
     * @brief returns a description of this sensor
     *
     * @return reference to this sensors description
     */
    const std::string& getDescription() const;

    /**
     * @brief The frame to which the sensor is attached.
     *
     * The frame can be NULL.
     * @return pointer to sensor model
     */
    rw::common::Ptr<SensorModel> getSensorModel() const;

    /**
     * @brief Sets the frame to which the sensor should be attached
     *
     * @param smodel set the sensor model
     */
    virtual void setSensorModel(rw::common::Ptr<SensorModel> smodel);

    /**
     * @brief gets the propertymap of this sensor
     */
    PropertyMap& getPropertyMap();
};

%template (SensorPtr) rw::common::Ptr<Sensor>;

/**
 * @brief a general sensormodel interface. The sensormodel describe the model of a sensor
 * and define the data that are part of the State. Much like Device, which describe
 * the kinematic model of a robot. A sensormodel should have a name id and be associated,
 * referenced to some frame in the workcell.
 */
class SensorModel
{
public:
	/**
     * @brief constructor
     *
     * @param name [in] the name of this sensor
     * @param frame [in] the frame that the sensor is referenced to
     */
	SensorModel(const std::string& name, Frame* frame);

    /**
     * @brief constructor
     *
     * @param name [in] the name of this sensor
     * @param frame [in] the frame that the sensor is referenced to
     * @param description [in] description of the sensor
     */
    SensorModel(const std::string& name, Frame* frame, const std::string& description);

    //! destructor
    virtual ~SensorModel();

    /**
     * @brief sets the name of this sensor
     *
     * @param name [in] name of this sensor
     */
    void setName(const std::string& name);

    /**
     * @brief sets the description of this sensor
     *
     * @param description [in] description of this sensor
     */
    void setDescription(const std::string& description);

    /**
     * @brief returns the name of this sensor
     *
     * @return name of sensor
     */
    const std::string& getName() const;

    /**
     * @brief returns a description of this sensor
     *
     * @return reference to this sensors description
     */
    const std::string& getDescription() const;

    /**
     * @brief The frame to which the sensor is attached.
     *
     * The frame can be NULL.
     */
    Frame* getFrame() const;

    /**
     * @brief Sets the frame to which the sensor should be attached
     *
     * @param frame The frame, which can be NULL
     */
    virtual void attachTo(Frame* frame);

    /**
     * @brief gets the propertymap of this sensor
     * @return reference to PropertyMap
     */
    PropertyMap& getPropertyMap();
};

%template (SensorModelPtr) rw::common::Ptr<SensorModel>;
OWNEDPTR(SensorModel)

/**
 * @brief The Camera class defines a generel interface to a camera.
 * A great deal of the interface resembles the DCAM standard since
 * DCAM allready defines a very wide interface.
 *
 * typical usage:
 * Camera c;
 * // setup camera features modes and so on
 * c.initialize();
 * c.start();
 * // acquire images
 * c.stop();
 *
 */
class Camera : public Sensor
{
public:
    /**
     * @brief destructor
     */
    virtual ~Camera();

    /**
     * @brief returns the camera model information (version, type, size, etc.)
     *
     * @return camera model information
     */
    virtual std::string getModelInfo() const;

    /**
     * @brief initializes the camera to the current settings
     * (CaptureMode,ColorMode,etc.)
     *
     * @return true if initialization is succesfully, false otherwise.
     */
    virtual bool initialize() = 0;

    /**
     * @brief returns whether this camera is initialized or not.
     *
     * @return true if intialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief starts this camera, if the camera has not been
     * initialized the initialize function will be called.
     *
     * @return true if camera was successfully started, false
     * otherwise
     */
    virtual bool start() = 0;

    /**
     * @brief returns whether this camera is started or not.
     *
     * @return true if started, false otherwise
     */
    bool isStarted() const;

    /**
     * @brief stops this camera. When the camera is stopped it can be
     * reinitialized using initialize()
     */
    virtual void stop() = 0;

    /**
     * @brief aquires an image from the camera. This method is not blocking.
     * Use  isImageReady to poll for completion of acquire.
     */
    virtual void acquire() = 0;

    /**
     * @brief tests whether a image has been acquired
     *
     * @return true if an image has been acquired, false otherwise.
     */
    virtual bool isImageReady() = 0;

    /**
     * @brief returns the last image acquired from the camera. This method
     * is not blocking, if no image has been acquired yet an empty image
     * is returned. The image returned can for some specific drivers be read
     * only.
     *
     * @return last image captured from camera.
     */
    virtual const Image* getImage() = 0;

    /**
     * @brief returns the framerate that this camera is setup with
     *
     * @return the framerate in frames per second
     */
    virtual double getFrameRate() = 0;

    /**
     * @brief sets the framerate of this camera. If the framerate is not
     * supported the closest supported framerate is choosen.
     *
     * @param framerate [in] the framerate
     */
    virtual void setFrameRate(double framerate) = 0;



    /**
     * @brief get width of the captured images
     *
     * @return width
     */
    virtual unsigned int getWidth() = 0;

    /**
     * @brief get width of the captured images
     *
     * @return width
     */
    virtual unsigned int getHeight() = 0;


    ///// a list of features that most of the time is available

    /**
     *  Check if shutter is available.
     *
     *  @return True if shutter is available
     */
    virtual bool isShutterAvailable() const;

    /**
     * Get actual shutter value.
     * Note: If shutter is not available then a dummy implementation
     * will throw an error message.
     *
     * @return shutter value in micro-seconds.
     */
    virtual double getShutter() const;

    /**
     * Set shutter value. If the given value is not possible the nearest
     * value are choosen.
     * Note: If shutter is not available then a dummy implementation
     * will throw an error message.
     *
     * @param Value New shutter value.
     */
    virtual void setShutter(double Value);

    /**
     * Check if gain is available.
     *
     * @return True if zoom is available
     */
    virtual bool isGainAvailable() const;

    /**
     * Get actual gain value.
     * Note: If gain is not available then a dummy implementation
     * returning -1 is used and an error message is produced.
     *
     * @return Gain value.
     */
    virtual double getGain() const;

    /** Set gain value. If the given value is not possible the nearest
     *  value are choosen.
     *  Note: If gain is not available then a dummy implementation
     *  returning -1 is used and an error message is produced.
     *
     *  @param Value New gain value.
     *  @return New nearest gain value.
     */
    virtual double setGain(double Value);
};

%template (CameraPtr) rw::common::Ptr<Camera>;

/**
 * @brief The CameraModel class defines a generel pinhole camera model where
 * camera parameters and state values are stored.
 */
class CameraModel : public SensorModel
{
public:

	/**
	 * constructor
	 *
	 * @param projection [in] pinhole projection model
	 * @param name [in] name of camera
	 * @param frame [in] frame that camera is attached/referenced to
	 * @param modelInfo [in] text description of the camera
	 */
	CameraModel(
    			const rw::math::ProjectionMatrix& projection,
    			const std::string& name,
    			Frame* frame,
    			const std::string& modelInfo = "");

    /**
     * @brief destructor
     */
    virtual ~CameraModel();

    /**
     * @brief returns the image if it has been saved in the State. Else null is
     * returned.
     * @param state [in] which state the image is taken from.
     * @return last image captured from camera.
     */
    rw::common::Ptr<Image> getImage(const State& state);

    /**
     * @brief set the image in the state
     *
     * @param img [in] image to set in state
     * @param state [in/out] the state in which to set the image.
     */
    void setImage(rw::common::Ptr<Image> img, State& state);

    /**
     * @brief get horisontal field of view.
     * @return  field of view in degrees
     */
    double getFieldOfViewX() const;

    /**
     * @brief get Vertical field of view.
     * @return  field of view in degrees
     */
    double getFieldOfViewY() const;

    ///// a list of features that most of the time is available
    /**
     * @brief get far clipping plane
     * @return distance to far clipping plane in meters.
     */
    double getFarClippingPlane() const;
    /**
     * @brief get near clipping plane
     * @return distance to near clipping plane in meters.
     */
    double getNearClippingPlane() const;
};

%template (CameraModelPtr) rw::common::Ptr<CameraModel>;
%template (CameraModelCPtr) rw::common::Ptr<const CameraModel>;

/**
 * @brief The image class is a simple wrapper around a char data array.
 * This Image wrapper contain information of width, height and encoding.
 *
 * The image class is somewhat inspired by the IplImage of opencv.
 *
 * The coordinate system has its origin located at the top-left position, where from X increases to
 * the left and Y-increases downwards.
 *
 * setting pixel values in an efficient manner has been enabled using some template joggling.
 * It requires that the user know what type of image he/she is working with.
 */
class Image
{
public:
    /**
     * @brief The color encodings that the image can use. This also defines the number
     * channels that an image has.
     */
    typedef enum
    {
        GRAY, //!< Grayscale image 1-channel
        RGB, //!< 3-channel color image (Standard opengl)
        RGBA, //!< 4-channel color image with alpha channel
        BGR, //!< 3-channel color image (Standard OpenCV)
        BGRA, //!< 4-channel color image with alpha channel
        BayerBG,
        Luv,
        Lab,
        HLS,
        User
    } ColorCode;

    /**
     * @brief The pixeldepth determines how many bits that are used per pixel per channel
     */
    typedef enum
    {
        Depth8U, //!< Depth8U
        Depth8S, //!< Depth8S
        Depth16U,//!< Depth16U
        Depth16S,//!< Depth16S
        Depth32S,//!< Depth32S
        Depth32F
    //!< Depth32F
    } PixelDepth;

public:
    /**
     * @brief default constructor
     */
    Image();

    /**
     * @brief constructor
     *
     * @param width [in] width of the image
     * @param height [in] height of the image
     * @param encoding [in] the colorCode of this Image
     * @param depth [in] the pixel depth in bits per channel
     */
    Image(unsigned int width, unsigned int height, ColorCode encoding, PixelDepth depth);

    /**
     * @brief constructor
     *
     * @param imgData [in] char pointer that points to an array of chars with
     * length width*height*(bitsPerPixel/8)
     * @param width [in] width of the image
     * @param height [in] height of the image
     * @param encoding [in] the colorCode of this Image
     * @param depth [in] the pixel depth in bits per channel
     */
    Image(char *imgData, unsigned int width, unsigned int height, ColorCode encoding, PixelDepth depth);

    /**
     * @brief destructor
     */
    virtual ~Image();

    /**
     * @brief resizes the current image.
     *
     * @param width [in] width in pixels
     * @param height [in] height in pixels
     */
    void resize(unsigned int width, unsigned int height);

    /**
     * @brief returns a char pointer to the image data
     *
     * @return const char pointer to the image data
     */
    const char* getImageData() const;

    /**
     * @brief sets the data array of this image. Make sure to
     * change the height and width accordingly.
     */
    void setImageData(char* data);

    /**
     * @brief returns the size of the char data array
     *
     * @return size of char data array
     */
    size_t getDataSize() const;

    /**
     * @brief returns the width of this image
     *
     * @return image width
     */
    unsigned int getWidth() const;

    /**
     * @brief returns the height of this image
     *
     * @return image height
     */
    unsigned int getHeight() const;

    /**
     * @brief returns color encoding/type of this image
     *
     * @return ColorCode of this image
     */
    ColorCode getColorEncoding() const;

    /**
     * @brief returns the number of bits per pixel. This is the number
     * of bits used per pixel per channel.
     *
     * @return number of bits per pixel
     */
    unsigned int getBitsPerPixel() const;

    /**
     * @brief saves this image to a file in the PGM (grayscale) format
     *
     * @param fileName [in] the name of the file that is to be created
     *
     * @return true if save was succesfull, false otherwise
     */
    bool saveAsPGM(const std::string& fileName) const;

    /**
     * @brief saves this image to a file in the ascii PGM (grayscale) format
     *
     * @param fileName [in] the name of the file that is to be created
     * @return true if save was succesfull, false otherwise
     */
    bool saveAsPGMAscii(const std::string& fileName) const;

    /**
     * @brief saves this image to a file in the PPM (color) format
     *
     * @param fileName [in] the name of the file that is to be created
     * @return true if save was succesfull, false otherwise
     */
    bool saveAsPPM(const std::string& fileName) const;

    /**
     * @brief the size of an aligned image row in bytes. This may not be
     * the same as the width if extra bytes are padded to each row for
     * alignment purposes.
     *
     * @return size of aligned image row
     */
    unsigned int getWidthStep() const;

    /**
     * @brief bits per pixel encoded as a PixelDepth type.
     *
     * @return the pixel depth
     */
    inline PixelDepth getPixelDepth() const;

    /**
     * @brief The number of channels that this image has.
     *
     * @return nr of channels
     */
    inline unsigned int getNrOfChannels() const;

    // Here comes all the getPixel operations

    /**
     * @brief generic but inefficient access to a specific channel of
     * a pixel.
     *
     * @param x [in]
     * @param y [in]
     * @param channel documentation missing !
     * @return the pixel value.
     */
    float getPixelValue(size_t x, size_t y, size_t channel) const;

    float getPixelValuef(size_t x, size_t y, size_t channel) const;

    int getPixelValuei(size_t x, size_t y, size_t channel) const;

    /**
     * @brief copies this image and flips it around horizontal or vertical axis or both.
     *
     * @return new image.
     */
    rw::common::Ptr<Image> copyFlip(bool horizontal, bool vertical) const;
};

%template (ImagePtr) rw::common::Ptr<Image>;

class Scanner: public Sensor
{
public:
    virtual ~Scanner();

    /**
     * @brief Opens connection to the scanner
     */
    virtual void open() = 0;

    /**
     * @brief Returns whether the scanner has been opened
     *
     * @return true if scanner is opened
     */
    virtual bool isOpen() = 0;

    /**
     * @brief Closes the connection to the scanner
     */
    virtual void close() = 0;

    /**
     * @brief Acquires data
     */
    virtual void acquire() = 0;

    /**
     * @brief tests whether an image has been acquired
     *
     * @return true if an image has been acquired, false otherwise.
     */
    virtual bool isScanReady() = 0;

    /**
     * @brief returns the framerate that this camera is setup with
     *
     * @return the framerate in frames per second
     */
    virtual double getFrameRate() = 0;
};

/**
 * @brief The Scanner2D sensor encapsulate the basic interface of a
 * 2 dimensional range scanning device such as SICK or Hokyuo laser
 * range scanners.
 *
 * The interface supports any range scanner that measures distance in
 * an arc around the origin of the sensor.
 */
class Scanner2D: public Scanner
{
public:
    /**
     * @brief destructor
     */
    virtual ~Scanner2D();

    /**
     * @brief gets the last acquired scan as a depth image
     * of height 1.
     */
    virtual const PointCloud& getScan() const = 0;

    /**
     * @brief Returns the angular range of the scanner.
     *
     * @return Angular range in radians
     */
    virtual double getAngularRange() const = 0;

    /**
     * @brief Returns the number of scan points
     */
    virtual size_t getMeasurementCount() const = 0;

};

%template (Scanner2DPtr) rw::common::Ptr<Scanner2D>;

/**
 * @brief The Scanner2DModel encapsulate the basic model of a
 * 2 dimensional range scanning device such as SICK or Hokyuo laser
 * range scanners.
 *
 * The model supports any range scanner that measures distance in
 * an arc around the origin of the sensor. The scanner scans in the z-x plane
 * with z-axis being the 0 angle measurement.
 *
 * TODO: enable the selection of internal format, either pointcloud (large) or
 * range-array (compact).
 *
 */
class Scanner2DModel: public SensorModel
{
public:
    /**
     * @brief constructor
     *
     * @param name [in] name of scanner sensor
     * @param angularRangeInRad [in] angular range in rad, with middle scan
     * point pointin along z-axis
     * @param maxDataPoints [in] the number of scan points
     * @param frame [in] the sensor frame
     */
    Scanner2DModel(const std::string& name, double angularRangeInRad, int maxDataPoints, Frame* frame );

    /**
     * @brief Destructor. Closes scanner connection if not already closed.
     */
    virtual ~Scanner2DModel();

    /**
     * @brief get handle to point cloud data in state.
     *
     * @param state [in] the state with point cloud data
     */
    PointCloud& getScan(const State& state);

    /**
     * @brief set point cloud data in state
     *
     * @param data [in] point cloud data to set
     * @param state [in] state in which to set the point cloud
     */
    void setScan(const PointCloud& data, const State& state);

    /**
     * @brief Returns the number of scan points
     */
    size_t getMeasurementCount() const;

    /**
     * @brief set distance range
     *
     * @param min documentation missing !
     * @param max documentation missing !
     */
    void setDistanceRange(double min, double max );
};

%template (Scanner2DModelPtr) rw::common::Ptr<Scanner2DModel>;

/**
 * @brief an interface describing a 3D scanner sensor. The scanner takes
 * pictures in the oposite direction of the z-axis of the frame that it is
 * attached to. The x-y plane forms the image plane such that the xy-origin is
 * located in the bottom left corner of the image.
 *
 */
class Scanner25D: public Scanner
{
public:
    /**
     * @brief Destructor. Closes scanner connection if not already closed.
     */
    virtual ~Scanner25D();

    /**
     * @brief gets the last acquired image
     *
     * @return the image that was last acquired.
     */
    virtual const PointCloud& getScan() = 0;

};

%template (Scanner25DPtr) rw::common::Ptr<Scanner25D>;

/**
 * @brief Model of a 25D (2D with depth information) scanner. The images are
 * essentially point clouds.
 */
class Scanner25DModel: public SensorModel
{
public:
    /**
     * @brief constructor
     *
     * @param name [in] name of scanner sensor
     * @param width [in]
     * @param height [in]
     * @param frame [in] the frame that the scanner is attached to
     */
    Scanner25DModel(const std::string& name, int width, int height, Frame* frame );

    /**
     * @brief Destructor. Closes scanner connection if not already closed.
     */
    virtual ~Scanner25DModel();

    /**
     * @brief get handle to point cloud data in state.
     *
     * @param state [in] the state with point cloud data
     */
    PointCloud& getScan(const State& state);

    /**
     * @brief set point cloud data in state
     *
     * @param data [in] point cloud data to set
     * @param state [in] state in which to set the point cloud
     */
    void setScan(const PointCloud& data, const State& state);

    //! width of images taken with 25 sensor
    int getWidth() const;

    //! height of images taken with 25 sensor
    int getHeight() const;

    //! set the min and maximum depth of this scanner in meters
    void setRange(double min, double max);
};

%template (Scanner25DModelPtr) rw::common::Ptr<Scanner25DModel>;

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

%template (TimedQ) Timed<rw::math::Q>;
%template (TimedState) Timed<State>;

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

%template (TimedQVector) std::vector<Timed<rw::math::Q> >;
%template (TimedStateVector) std::vector<Timed<State> >;
%template (TimedQVectorPtr) rw::common::Ptr<std::vector<Timed<rw::math::Q> > >;
%template (TimedStateVectorPtr) rw::common::Ptr<std::vector<Timed<State> > >;
OWNEDPTR(std::vector<Timed<rw::math::Q> > )
//OWNEDPTR(std::vector<Timed<State> > )

%template (PathSE3) Path<rw::math::Transform3D<double> >;
%template (PathSE3Ptr) rw::common::Ptr<Path<rw::math::Transform3D<double> > >;
%template (PathQ) Path<rw::math::Q>;
%template (PathQPtr) rw::common::Ptr<Path<rw::math::Q> >;
%template (PathTimedQ) Path<Timed<rw::math::Q> >;
%template (PathTimedQPtr) rw::common::Ptr<Path<Timed<rw::math::Q> > >;
%template (PathTimedState) Path<Timed<State> >;
%template (PathTimedStatePtr) rw::common::Ptr<Path<Timed<State> > >;
OWNEDPTR(Path<rw::math::Transform3D<double> > )
OWNEDPTR(Path<rw::math::Q> )
OWNEDPTR(Path<Timed<rw::math::Q> > )
OWNEDPTR(Path<Timed<State> > )

%extend Path<rw::math::Q> {
    rw::common::Ptr<Path<Timed<rw::math::Q> > > toTimedQPath(rw::math::Q speed){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(speed, *$self);
        return rw::common::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::common::Ptr<Path<Timed<rw::math::Q> > > toTimedQPath(rw::common::Ptr<Device> dev){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(*dev, *$self);
        return rw::common::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::common::Ptr<Path<Timed<State> > > toTimedStatePath(rw::common::Ptr<Device> dev,
                                                     const State& state){
        rw::trajectory::TimedStatePath tpath =
                rw::trajectory::TimedUtil::makeTimedStatePath(*dev, *$self, state);
        return rw::common::ownedPtr( new rw::trajectory::TimedStatePath(tpath) );
    }

};

%extend Path<Timed<State> > {
	
	static rw::common::Ptr<Path<Timed<State> > > load(const std::string& filename, rw::common::Ptr<WorkCell> wc){
		rw::common::Ptr<rw::trajectory::TimedStatePath> spath = 
                    rw::common::ownedPtr(new rw::trajectory::TimedStatePath);
                *spath = rw::loaders::PathLoader::loadTimedStatePath(*wc, filename);
		return rw::common::Ptr<rw::trajectory::TimedStatePath>( spath );
	}
	
	void save(const std::string& filename, rw::common::Ptr<WorkCell> wc){		 		
		rw::loaders::PathLoader::storeTimedStatePath(*wc,*$self,filename); 
	}
	
	void append(rw::common::Ptr<Path<Timed<State> > > spath){
		double startTime = 0;
		if($self->size()>0)
			startTime = (*$self).back().getTime(); 
		
		for(size_t i = 0; i<spath->size(); i++){
			Timed<State> tstate = (*spath)[i]; 
			tstate.getTime() += startTime;
			(*$self).push_back( tstate );
		}
	}
	
};

%extend Path<State > {
	
	static rw::common::Ptr<Path<State> > load(const std::string& filename, rw::common::Ptr<WorkCell> wc){
            rw::common::Ptr<rw::trajectory::StatePath> spath = rw::common::ownedPtr(new rw::trajectory::StatePath);
            *spath = rw::loaders::PathLoader::loadStatePath(*wc, filename);
		return rw::common::ownedPtr( spath );
	}
	
	void save(const std::string& filename, rw::common::Ptr<WorkCell> wc){		 		
		rw::loaders::PathLoader::storeStatePath(*wc,*$self,filename); 
	}
	
	void append(rw::common::Ptr<Path<State> > spath){
		double startTime = 0;
		if($self->size()>0)
			startTime = (*$self).front().getTime(); 
		
		for(size_t i = 0; i<spath->size(); i++){
			(*$self).push_back( (*spath)[i] );
		}
	}
	
	
	rw::common::Ptr<Path<Timed<State> > > toTimedStatePath(double timeStep){
		rw::common::Ptr<TimedStatePath> spath = 
			rw::common::ownedPtr( new rw::trajectory::TimedStatePath() );	
		for(size_t i = 0; i<spath->size(); i++){
			Timed<State> tstate(timeStep*i, (*spath)[i]); 
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
%template (BlendQ) Blend<rw::math::Q>;

%template (BlendR1Ptr) rw::common::Ptr<Blend<double> >;
%template (BlendR2Ptr) rw::common::Ptr<Blend<rw::math::Vector2D<double> > >;
%template (BlendR3Ptr) rw::common::Ptr<Blend<rw::math::Vector3D<double> > >;
%template (BlendSO3Ptr) rw::common::Ptr<Blend<rw::math::Rotation3D<double> > >;
%template (BlendSE3Ptr) rw::common::Ptr<Blend<rw::math::Transform3D<double> > >;
%template (BlendQPtr) rw::common::Ptr<Blend<rw::math::Q> >;

OWNEDPTR(Blend<double> )
OWNEDPTR(Blend<rw::math::Vector2D<double> > )
OWNEDPTR(Blend<rw::math::Vector3D<double> > )
OWNEDPTR(Blend<rw::math::Rotation3D<double> > )
OWNEDPTR(Blend<rw::math::Transform3D<double> > )
OWNEDPTR(Blend<rw::math::Q> )

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
%template (InterpolatorQ) Interpolator<rw::math::Q>;

%template (InterpolatorR1Ptr) rw::common::Ptr<Interpolator<double> >;
%template (InterpolatorR2Ptr) rw::common::Ptr<Interpolator<rw::math::Vector2D<double> > >;
%template (InterpolatorR3Ptr) rw::common::Ptr<Interpolator<rw::math::Vector3D<double> > >;
%template (InterpolatorSO3Ptr) rw::common::Ptr<Interpolator<rw::math::Rotation3D<double> > >;
%template (InterpolatorSE3Ptr) rw::common::Ptr<Interpolator<rw::math::Transform3D<double> > >;
%template (InterpolatorQPtr) rw::common::Ptr<Interpolator<rw::math::Q> >;

OWNEDPTR(Interpolator<double> )
OWNEDPTR(Interpolator<rw::math::Vector2D<double> > )
OWNEDPTR(Interpolator<rw::math::Vector3D<double> > )
OWNEDPTR(Interpolator<rw::math::Rotation3D<double> > )
OWNEDPTR(Interpolator<rw::math::Transform3D<double> > )
OWNEDPTR(Interpolator<rw::math::Q> )

class LinearInterpolator: public Interpolator<double> {
public:
    LinearInterpolator(const double& start,
                          const double& end,
                          double duration);

    virtual ~LinearInterpolator();

    double x(double t) const;
    double dx(double t) const;
    double ddx(double t) const;
    double duration() const;
};


class LinearInterpolatorQ: public Interpolator<rw::math::Q> {
public:
    LinearInterpolatorQ(const rw::math::Q& start,
                          const rw::math::Q& end,
                          double duration);

    virtual ~LinearInterpolatorQ();

    rw::math::Q x(double t) const;
    rw::math::Q dx(double t) const;
    rw::math::Q ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorR3: public Interpolator<rw::math::Rotation3D<double> > {
public:
    LinearInterpolatorR3(const rw::math::Rotation3D<double> & start,
                          const rw::math::Rotation3D<double> & end,
                          double duration);

    rw::math::Rotation3D<double>  x(double t) const;
    rw::math::Rotation3D<double>  dx(double t) const;
    rw::math::Rotation3D<double>  ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorSO3: public Interpolator<rw::math::Rotation3D<double> > {
public:
    LinearInterpolatorSO3(const rw::math::Rotation3D<double> & start,
                          const rw::math::Rotation3D<double> & end,
                          double duration);

    rw::math::Rotation3D<double>  x(double t) const;
    rw::math::Rotation3D<double>  dx(double t) const;
    rw::math::Rotation3D<double>  ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorSE3: public Interpolator<rw::math::Transform3D<double> > {
public:
    LinearInterpolatorSE3(const rw::math::Transform3D<double> & start,
                          const rw::math::Transform3D<double> & end,
                          double duration);

    rw::math::Transform3D<double>  x(double t) const;
    rw::math::Transform3D<double>  dx(double t) const;
    rw::math::Transform3D<double>  ddx(double t) const;
    double duration() const;
};


//////////// RAMP interpolator


class RampInterpolatorR3: public Interpolator<rw::math::Vector3D<double> > {
public:
    RampInterpolatorR3(const rw::math::Vector3D<double>& start, const rw::math::Vector3D<double>& end,
                       double vellimit,double acclimit);

    rw::math::Vector3D<double> x(double t) const;
    rw::math::Vector3D<double> dx(double t) const;
    rw::math::Vector3D<double> ddx(double t) const;
    double duration() const;
};

class RampInterpolatorSO3: public Interpolator<rw::math::Rotation3D<double> > {
public:
    RampInterpolatorSO3(const rw::math::Rotation3D<double> & start,
                          const rw::math::Rotation3D<double> & end,
                          double vellimit,double acclimit);

    rw::math::Rotation3D<double>  x(double t) const;
    rw::math::Rotation3D<double>  dx(double t) const;
    rw::math::Rotation3D<double>  ddx(double t) const;
    double duration() const;
};

class RampInterpolatorSE3: public Interpolator<rw::math::Transform3D<double> > {
public:
    RampInterpolatorSE3(const rw::math::Transform3D<double> & start,
                          const rw::math::Transform3D<double> & end,
                          double linvellimit,double linacclimit,
                          double angvellimit,double angacclimit);

    rw::math::Transform3D<double>  x(double t) const;
    rw::math::Transform3D<double>  dx(double t) const;
    rw::math::Transform3D<double>  ddx(double t) const;
    double duration() const;
};

class RampInterpolator: public Interpolator<double> {
public:
    RampInterpolator(const double& start, const double& end, const double& vellimits, const double& acclimits);
    //RampInterpolator(const double& start, const double& end, const double& vellimits, const double& acclimits, double duration);

    double x(double t) const;
    double dx(double t) const;
    double ddx(double t) const;
    double duration() const;
};

class RampInterpolatorQ: public Interpolator<rw::math::Q> {
public:
    RampInterpolatorQ(const rw::math::Q& start, const rw::math::Q& end, const rw::math::Q& vellimits, const rw::math::Q& acclimits);
    //RampInterpolatorQ(const rw::math::Q& start, const rw::math::Q& end, const rw::math::Q& vellimits, const rw::math::Q& acclimits, double duration);

    rw::math::Q x(double t) const;
    rw::math::Q dx(double t) const;
    rw::math::Q ddx(double t) const;
    double duration() const;
};



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
    //virtual typename rw::common::Ptr< TrajectoryIterator<T> > getIterator(double dt = 1) const = 0;

protected:
    /**
     * @brief Construct an empty trajectory
     */
    Trajectory() {};
};

%template (TrajectoryState) Trajectory<State>;
%template (TrajectoryR1) Trajectory<double>;
%template (TrajectoryR2) Trajectory<rw::math::Vector2D<double> >;
%template (TrajectoryR3) Trajectory<rw::math::Vector3D<double> >;
%template (TrajectorySO3) Trajectory<rw::math::Rotation3D<double> >;
%template (TrajectorySE3) Trajectory<rw::math::Transform3D<double> >;
%template (TrajectoryQ) Trajectory<rw::math::Q>;

%template (TrajectoryStatePtr) rw::common::Ptr<Trajectory<State> >;
%template (TrajectoryR1Ptr) rw::common::Ptr<Trajectory<double> >;
%template (TrajectoryR2Ptr) rw::common::Ptr<Trajectory<rw::math::Vector2D<double> > >;
%template (TrajectoryR3Ptr) rw::common::Ptr<Trajectory<rw::math::Vector3D<double> > >;
%template (TrajectorySO3Ptr) rw::common::Ptr<Trajectory<rw::math::Rotation3D<double> > >;
%template (TrajectorySE3Ptr) rw::common::Ptr<Trajectory<rw::math::Transform3D<double> > >;
%template (TrajectoryQPtr) rw::common::Ptr<Trajectory<rw::math::Q> >;

OWNEDPTR(Trajectory<State> )
OWNEDPTR(Trajectory<double> )
OWNEDPTR(Trajectory<rw::math::Vector2D<double> > )
OWNEDPTR(Trajectory<rw::math::Vector3D<double> > )
OWNEDPTR(Trajectory<rw::math::Rotation3D<double> > )
OWNEDPTR(Trajectory<rw::math::Transform3D<double> > )
OWNEDPTR(Trajectory<rw::math::Q> )

template <class T>
class InterpolatorTrajectory: public Trajectory<T> {
public:
    InterpolatorTrajectory(double startTime = 0);
    void add(rw::common::Ptr<Interpolator<T> > interpolator);
    void add(rw::common::Ptr<Blend<T> > blend,
             rw::common::Ptr<Interpolator<T> > interpolator);
    void add(InterpolatorTrajectory<T>* trajectory);
    size_t getSegmentsCount() const;



    //std::pair<rw::common::Ptr<Blend<T> >, rw::common::Ptr<Interpolator<T> > > getSegment(size_t index) const;
};

%template (InterpolatorTrajectoryR1) InterpolatorTrajectory<double>;
%template (InterpolatorTrajectoryR2) InterpolatorTrajectory<rw::math::Vector2D<double> >;
%template (InterpolatorTrajectoryR3) InterpolatorTrajectory<rw::math::Vector3D<double> >;
%template (InterpolatorTrajectorySO3) InterpolatorTrajectory<rw::math::Rotation3D<double> >;
%template (InterpolatorTrajectorySE3) InterpolatorTrajectory<rw::math::Transform3D<double> >;
%template (InterpolatorTrajectoryQ) InterpolatorTrajectory<rw::math::Q>;


/*
class TrajectoryFactory
{
public:
    static rw::common::Ptr<StateTrajectory> makeFixedTrajectory(const State& state, double duration);
    static rw::common::Ptr<QTrajectory> makeFixedTrajectory(const rw::math::Q& q, double duration);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectory(const TimedStatePath& path);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectory(const StatePath& path,
        const models::WorkCell& workcell);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectoryUnitStep(const StatePath& path);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const TimedQPath& path);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, const rw::math::Q& speeds);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, const models::Device& device);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, rw::common::Ptr<QMetric> metric);
    static rw::common::Ptr<Transform3DTrajectory> makeLinearTrajectory(const Transform3DPath& path, const std::vector<double>& times);
    static rw::common::Ptr<Transform3DTrajectory> makeLinearTrajectory(const Transform3DPath& path, const rw::common::Ptr<Transform3DMetric> metric);
    static rw::common::Ptr<StateTrajectory> makeEmptyStateTrajectory();
    static rw::common::Ptr<QTrajectory > makeEmptyQTrajectory();
};

*/
 
/********************************************
 * LUA functions
 ********************************************/


#if defined (SWIGLUA)
%luacode {

-- Group: Lua functions
-- Var: print_to_log
print_to_log = true

-- Var: overrides the global print function
local global_print = print

-- Function: print
--  Forwards the global print functions to the sdurw.print functions
--  whenever print_to_log is defined.
function print(...)
    if print_to_log then
        for i, v in ipairs{...} do
            if i > 1 then rw.writelog("\t") end
            sdurw.writelog(tostring(v))
        end
        sdurw.writelog('\n')
    else
        global_print(...)
    end
end

-- Function:
function reflect( mytableArg )
 local mytable
 if not mytableArg then
  mytable = _G
 else
  mytable = mytableArg
 end
   local a = {} -- all functions
   local b = {} -- all Objects/Tables

 if type(mytable)=="userdata" then
   -- this is a SWIG generated user data, show functions and stuff
   local m = getmetatable( mytable )
   for key,value in pairs( m['.fn'] ) do
      if (key:sub(0, 2)=="__") or (key:sub(0, 1)==".") then
          table.insert(b, key)
      else
          table.insert(a, key)
      end
   end
   table.sort(a)
   table.sort(b)
   print("Object type: \n  " .. m['.type'])

   print("Member Functions:")
   for i,n in ipairs(a) do print("  " .. n .. "(...)") end
   for i,n in ipairs(b) do print("  " .. n .. "(...)") end

 else
   local c = {} -- all constants
   for key,value in pairs( mytable ) do
      -- print(type(value))
      if (type(value)=="function") then
          table.insert(a, key)
      elseif (type(value)=="number") then
          table.insert(c, key)
      else
          table.insert(b, key)
      end
   end
   table.sort(a)
   table.sort(b)
   table.sort(c)
   print("Object type: \n  " .. "Table")

   print("Functions:")
   for i,n in ipairs(a) do print("  " .. n .. "(...)") end
   print("Constants:")
   for i,n in ipairs(c) do print("  " .. n) end
   print("Misc:")
   for i,n in ipairs(b) do print("  " .. n) end


--  print("Metatable:")
--  for key,value in pairs( getmetatable(mytable) ) do
--      print(key);
--      print(value);
--  end

 end
 end

function help( mytable )
   reflect( mytable )
end

local used_ns = {}

function using(ns)
  local ns_found = false
  local ns_name;
  local ns_val;
  for n,v in pairs(_G) do
    if n == ns then
      ns_found = true
      ns_name = n
      ns_val = v
      break
    end
  end
  if not ns_found then
    error("Unknown table: " .. ns)
  else
    if used_ns[ns_name] == nil then
      used_ns[ns_name] = ns_val
      for n,v in pairs(ns_val) do
        if n ~= "string" and n ~= "ownedPtr" then
          if _G[n] ~= nil then
            print("name clash: " .. n .. " is already defined")
          else
            _G[n] = v
          end
        end
      end
    end
  end
end

function ownedPtr(arg)
  local found = false
  for ns_n,ns_v in pairs(used_ns) do
    for n,v in pairs(ns_v) do
      if type(v) ~= "function" and type(v) ~= "number" then
        if string.len(n) >= 4 then
          if string.sub(n, -3) == "Ptr" then
            if getmetatable(arg)[".type"] .. "Ptr" == n then
              return ns_v.ownedPtr(arg)
            end
          end
        end
      end
    end
  end
end

function ownedCPtr(arg)
  local found = false
  for ns_n,ns_v in pairs(used_ns) do
    for n,v in pairs(ns_v) do
      if type(v) ~= "function" and type(v) ~= "number" then
        if string.len(n) >= 5 then
          if string.sub(n, -4) == "CPtr" then
            if getmetatable(arg)[".type"] .. "CPtr" == n then
              return ns_v.ownedCPtr(arg)
            end
          end
        end
      end
    end
  end
end
}
#endif


