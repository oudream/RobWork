//############### Ptr
    namespace rw { namespace core {

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
        bool operator==(const rw::core::Ptr<A>& p) const;


        %rename(deref) get;
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
        namespace rw { namespace core {
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
//############### AnyPtr
//############### BoostXMLParser
//############### DOMCoreBasisTypes
//############### DOMCorePropertyMapLoader
//############### DOMCorePropertyMapSaver
//############### DOMElem
//############### DOMParser
//############### DOMPropertyMapFormat
//############### Event
    namespace rw { namespace core {

        class _n1
        {};
        class _n2
        {};
        class _n3
        {};
        class _n4
        {};
        class _n5
        {};
        class _n6
        {};
        class _n7
        {};
        class _n8
        {};
        template <class CallBackMethod, class T1 = _n1, class T2 = _n1, class T3 = _n1, class T4 = _n1>
        class Event
        {
        public:

        };

    }}
//############### Exception

    %nodefaultctor Exception;
    /**
     * @brief Standard exception type of RobWork.
     *
     * All exception thrown within RobWork are of the type Exception.
     *
     * An exception contains a message (of type Message) for the user and
     * nothing else.
     */
    class Exception : public std::exception
    {
      public:

        /**
         * @brief This constructor creates an empty Exception and should not be used
         */
        Exception();
        /**
         * @brief Constructor
         *
         * @param message [in] A message for a user.
         */
        Exception (const Message& message);

        /**
         * @brief Constructor
         *
         * @param id [in] Integer Id to identify the exception
         * @param message [in] A message for a user.
         */
        Exception (int id, const Message& message);

        virtual ~Exception () throw ();

        /**
         * @brief The message for the user describing the reason for the error.
         *
         * @return  The message for the user.
         */
        const Message& getMessage () const;

        /**
         * @brief get id of this exception message
         * @return id
         */
        int getId () const;

        /**
         * @brief readable description of this esception
         * @return string description
         */
        const char* what () const throw ();

        TOSTRING(Exception)
    };

    %template( VectorException) std::vector<Exception>;

//############### Extension
    struct ExtensionDescriptor {
        ExtensionDescriptor();
        ExtensionDescriptor(const std::string& id_, const std::string& point_);

        std::string id,name,point;
        rw::core::PropertyMap props;

        //rw::core::PropertyMap& getProperties();
        const rw::core::PropertyMap& getProperties() const;
    };

    class Extension {
    public:
        Extension(ExtensionDescriptor desc, Plugin* plugin);
        
        const std::string& getId();
        const std::string& getName();
    };

    %template (ExtensionPtr) rw::core::Ptr<Extension>;
    %template (ExtensionPtrVector) std::vector<rw::core::Ptr<Extension> >;
//############### ExtensionPoint
//############### ExtensionRegister
    class ExtensionRegistry {
    public:
        ExtensionRegistry();
        static rw::core::Ptr<ExtensionRegistry> getInstance();
        std::vector<rw::core::Ptr<Extension> > getExtensions(const std::string& ext_point_id) const;
        std::vector<rw::core::Ptr<Plugin> > getPlugins() const;
    };

    %template (ExtensionRegistryPtr) rw::core::Ptr<ExtensionRegistry>;

//############### IOUtil
//############### Log
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
        static rw::core::Ptr<Log> getInstance();

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
        static void setLog(rw::core::Ptr<Log> log);

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
        rw::core::Ptr<LogWriter> getWriter(LogIndex id);

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
        void setWriter(LogIndex id, rw::core::Ptr<LogWriter> writer);

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
        void setWriterForMask(int mask, rw::core::Ptr<LogWriter> writer);

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

    %template (LogPtr) rw::core::Ptr<Log>;
    OWNEDPTR(Log)

//############### LogStreamWriter
//############### LogWriter
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

    %template (LogWriterPtr) rw::core::Ptr<LogWriter>;

//############### Message

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

    %template (MessagePtr) rw::core::Ptr<Message>;
//############### os
//############### PairIterator
//############### Plugin

    class Plugin {
    protected:
        Plugin(const std::string& id, const std::string& name, const std::string& version);
        
    public:
        const std::string& getId();
        const std::string& getName();
        const std::string& getVersion();
    };

    %template (PluginPtr) rw::core::Ptr<Plugin>;
    %template (PluginPtrVector) std::vector<rw::core::Ptr<Plugin> >;
//############### Property
//############### PropertyBase
//############### PropertyMap
    //! @copydoc rw::core::PropertyMap
    class PropertyMap
    {
    public: 
        //! @copydoc rw::core::PropertyMap::PropertyMap
        PropertyMap();
        //! @copydoc rw::core::PropertyMap::has
        bool has(const std::string& identifier) const;
        //! @copydoc rw::core::PropertyMap::size
        size_t size() const;
        //! @copydoc rw::core::PropertyMap::empty 
        bool empty() const;
        //! @copydoc rw::core::PropertyMap::erase
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
            
            Q& getQ(const std::string& id){ return $self->get<Q>(id); }
            void setQ(const std::string& id, Q q){ $self->set<Q>(id, q); }
            void set(const std::string& id, Q q){ $self->set<Q>(id, q); }

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
    %template (PropertyMapPtr) rw::core::Ptr<PropertyMap>;
    OWNEDPTR(PropertyMap)

//############### PropertyType
//############### RobWork
//############### StringUtil
