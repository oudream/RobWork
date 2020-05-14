// ############### Sensor ############################

    %nodefaultctor Sensor;
    /**
     * @brief a generel hardware sensor interface. The sensor should interface
     * to a statefull instance of either a real world sensor or a simulated
     * sensor. The sensor interface acts as a realistic handle to controlling
     * some specific instance of a sensor.
     */
    class Sensor
    {
      protected:
        /**
         * @brief constructor
         * @param name [in] the name of this sensor
         */
        Sensor (const std::string& name);

        /**
         * @brief constructor
         * @param name [in] the name of this sensor
         * @param description [in] description of the sensor
         */
        Sensor (const std::string& name, const std::string& description);

        /**
         * @brief sets the name of this sensor
         * @param name [in] name of this sensor
         */
        void setName (const std::string& name);

        /**
         * @brief sets the description of this sensor
         * @param description [in] description of this sensor
         */
        void setDescription (const std::string& description);

      public:
        //! @brief destructor
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
        rw::core::Ptr<SensorModel> getSensorModel() const;

        /**
         * @brief Sets the frame to which the sensor should be attached
         *
         * @param smodel set the sensor model
         */
        virtual void setSensorModel(rw::core::Ptr<SensorModel> smodel);

        /**
         * @brief gets the propertymap of this sensor
         */
        PropertyMap& getPropertyMap();
    };

    %template (SensorPtr) rw::core::Ptr<Sensor>;
    OWNEDPTR(Sensor);

// ############### Camera ###########################
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

      protected:
        /**
         * @brief constructor
         * @param name [in] name of sensor
         * @param modelInfo [in] info string
         */
        Camera (const std::string& name, const std::string& modelInfo);

        /**
         * @brief sets the camera model information
         * @param info [in] information of the camera
         */
        void setModelInfo (const std::string info);

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

        /**
         * @brief adds a CameraListener to this camera
         * @param listener [in] the CameraListener that is to be added
         * @return true if listener was added succesfully, false otherwise
         */
        virtual bool addListener (CameraListener& listener);

        /**
         * @brief removes a CameraListener from this cameras listener list.
         * @param listener [in] the listener that is to be removed
         * @return true if listener was removed succesfully, false otherwise.
         */
        virtual bool removeListener (CameraListener& listener);

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
         * gets the shutter bounds.
         * Note: If shutter is not available then a dummy implementation
         * will throw an error message.
         * @return first value is the min bound and second value is the max bound
         */
        virtual std::pair< double, double > getShutterBounds () const;
        
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

    %template (CameraPtr) rw::core::Ptr<Camera>;
    OWNEDPTR(Camera);
// ############### CameraFirewire ###################
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
    class CameraFirewire : public Camera
    {
      public:
        //! @brief Optional features of a camera
        enum CameraFeature {
            SHUTTER,
            ZOOM,
            GAIN,
            FOCUS,
            IRIS,
            HUE,
            WHITEBALANCE,
            SHARPNESS,
            SATURATION,
            GAMMA,
            BRIGHTNESS,
            AUTOEXPOSURE
        };

        //! @brief Optional colormodes available when capturing
        enum ColorCode {
            MONO8,
            YUV411,
            YUV422,
            YUV444,
            RGB8,
            MONO16,
            RGB16,
            MONO16S,
            RGB16S,
            RAW8,
            RAW16,
            RGB24
        };

        /**
         * @brief defines how images are captured.
         * When the SINGLE_SHOT CapturePolicy is used the user has to trigger
         * the camera through acquire. When CONTINUES is used images are captured
         * according to the given frameRate and getImage is more efficient to use.
         * The image returned is allways the newest captured image.
         * When using CONTINUES_BUFFERED images are continuesly captured and put in
         * a buffer.
         */
        enum CapturePolicy { SINGLE_SHOT, CONTINUES, CONTINUES_BUFFERED };

        //! @brief The resolution of the camera capture
        enum CaptureMode {
            M160x120,
            M320x240,
            M640x480,
            M800x600,
            M1024x768,
            M1280x960,
            M1600x1200,
            MFORMAT7
        };

        //! @brief The resolution of the camera capture
        enum Format7Mode { F7MODE0, F7MODE1, F7MODE2, F7MODE3, F7MODE4, F7MODE5, F7MODE6, F7MODE7 };

        //! @brief Modes of the camera, inspired by the DCAM standard modes
        enum TriggerMode { MODE0, MODE1, MODE2, MODE3, MODE4, MODE5, MODE14, MODE15 };

        //! @brief error codes for a camera
        enum ErrorCode {
            SUCCES,
            FAILURE,
            NOT_INITIALIZED,
            NOT_STARTED,
            UNSUPPORTED_CAPTURE_MODE,
            UNSUPPORTED_FEATURE
        };

      protected:
        /**
         * @brief constructor
         * @cond
         * @param frame [in] Frame in which to place the sensor
         * @endcond
         * @param name [in] name of sensor
         * @param modelInfo [in] info string
         */
        CameraFirewire (const std::string& name, const std::string& modelInfo);

      public:
        /**
         * @brief destructor
         */
        virtual ~CameraFirewire ();

        /**
         * @brief returns the CaptureMode of this camera
         * @return the camera capturemode
         */
        virtual CaptureMode getCaptureMode () = 0;

        /**
         * @brief sets the CaptureMode of this camera.
         * @param mode [in] the wanted capture mode
         * @return true if CaptureMode was set successfully, false otherwise
         */
        virtual bool setCaptureMode (CaptureMode mode) = 0;

        /**
         * @brief returns the CaptureMode of this camera
         * @return the camera capturemode
         */
        virtual ColorCode getColorMode () = 0;

        /**
         * @brief sets the CaptureMode of this camera.
         * @param mode [in] the wanted capture mode
         * @return true if CaptureMode was set successfully, false otherwise
         */
        virtual bool setColorMode (ColorCode mode) = 0;

        /**
         * @brief returns the errorcode of the latest error. If no error has occured
         * then SUCCES is returned.
         * @return the error code
         */
        virtual ErrorCode getError ();

        /**
         * @brief tests whether this camera is in an error state.
         * @return true if camera is in error state, false otherwise
         */
        virtual bool isError ();

        /**
         * @brief returns the capture policy of this camera.
         * @return capture policy of the camera
         */
        virtual CapturePolicy getCapturePolicy () = 0;

        /**
         * @brief sets the capture policy of this camera
         * @param policy [in] the capture policy
         * @return true if capture policy was set succesfully, false otherwise
         */
        virtual bool setCapturePolicy (CapturePolicy policy);

        /**
         * @brief returns whether the specified camera option is supported
         * by the camera.
         * @param option [in] the specific CameraOption
         * @return true if the option is available, false otherwise.
         */
        virtual bool isFeatureAvailable (CameraFeature option);

        /**
         * @brief returns the value of the specified camera setting. If the
         * camera is not initialized or the setting is unsupported -1 is returned.
         * @param setting [in] the CameraFeature
         * @return value of the setting if setting is supported and camera is
         * initilized, else -1 is returned.
         */
        virtual double getFeature (CameraFeature setting);

        /**
         * @brief sets the value of the specified camera setting. If the
         * camera is not initialized or the setting is unsupported false is returned.
         * @param setting [in] the CameraFeature
         * @param value [in] the value of the feature
         * @return true if the setting was succesfully changed, false otherwise.
         */
        virtual bool setFeature (CameraFeature setting, double value);

    };
// ############### CameraListener ###################
    /**
     * @brief interface used for listening for camera events
     */
    class CameraListener
    {
      protected:
        /**
         * @brief constructor
         */
        CameraListener ();

      public:
        /**
         * @brief destructor
         */
        virtual ~CameraListener ();

        /**
         * @brief called when the camera wish to signal a change.
         */
        virtual void notifyChanged () = 0;
    };

// ############### SensorModel #######################
    /**
     * @brief a general sensormodel interface. The sensormodel describe the model of a sensor
     * and define the data that are part of the State. Much like Device, which describe
     * the kinematic model of a robot. A sensormodel should have a name id and be associated,
     * referenced to some frame in the workcell.
     */
    class SensorModel: public Stateless
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

        //! @brief destructor
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

    %template (SensorModelPtr) rw::core::Ptr<SensorModel>;
    %template (VectorSensorModelPtr) std::vector<rw::core::Ptr<SensorModel>>;
    OWNEDPTR(SensorModel)
// ############### CameraModel ######################
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
        CameraModel (const ProjectionMatrix& projection, const std::string& name,
                     Frame* frame, const std::string& modelInfo = "");


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
        rw::core::Ptr<Image> getImage(const State& state);

        /**
         * @brief set the image in the state
         *
         * @param img [in] image to set in state
         * @param state [in/out] the state in which to set the image.
         */
        void setImage(rw::core::Ptr<Image> img, State& state);
        
        //! @brief get the camera projection matrix
        ProjectionMatrix getProjectionMatrix () const;

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

    %template (CameraModelPtr) rw::core::Ptr<CameraModel>;
    %template (CameraModelCPtr) rw::core::Ptr<const CameraModel>;

// ############### Contact2D ########################
    /**
     * @brief data structure for describing a contact in 2D
     */
    class Contact2D
    {
      public:
        //! @brief Contact position
        rw::math::Vector2D<double> p;    

        //! @brief Surface contact normal
        rw::math::Vector2D<double> n;    

        //! @brief surface curvature
        double curvature;          

         //! @brief double moving average of the curvature
        double avgCurvature;      

        //! @brief coulomb friction coefficient
        double mu;                 
    };

    %template( Contact2DPtr ) rw::core::Ptr<Contact2D>;
    OWNEDPTR(Contact2D);
// ############### Contact3D ########################
    /**
     * @brief data structure for describing a contact in 3D
     */
    class Contact3D
    {
      public:
        //! @brief constructor
        Contact3D ();

        /**
         * @brief constructor
         * @param tp [in] point contact
         * @param tn [in] contact normal
         * @param normalf [in] normal force in the contact
         */
        Contact3D (rw::math::Vector3D<double> tp, rw::math::Vector3D<double> tn, double normalf);

        /**
         * @brief constructor
         * @param tp [in] point contact
         * @param tn [in] contact normal
         * @param tf [in] force in the contact
         */
        Contact3D (rw::math::Vector3D<double> tp, rw::math::Vector3D<double> tn, rw::math::Vector3D<double> tf);

        //! @brief Contact position
        rw::math::Vector3D<double> p;    

        //! @brief Surface contact normal
        rw::math::Vector3D<double> n;    

        //! @brief the actual force
        rw::math::Vector3D<double> f;    

        //! @brief normal force
        double normalForce;        

        //! @brief index to the geometric primitive on which the contact is located
        unsigned int _faceIdx, _faceIdx2;

        //! @brief surface curvature
        double curvature;

         //! @brief coulomb friction coefficient    
        double mu;          

    };

    %template(Contact3DPtr ) rw::core::Ptr<Contact3D>;
    %template(VectorContact3D) std::vector<Contact3D>;
    OWNEDPTR(Contact3D)
// ############### FTSensor #########################
       /**
     * @brief Interface of a N-axis Force Torque sensor
     */
    class FTSensor : public Sensor
    {
      public:
        /**
         * @param name documentation missing !
         * @param desc documentation missing !
         * @return
         */
        FTSensor (const std::string& name, const std::string& desc = "");

        /**
         * @brief destructor
         * @return
         */
        virtual ~FTSensor ();

        /**
         * @brief acquires force data from the tactile cells
         */
        virtual void acquire () = 0;

        /**
         * @brief gets the maximum force in Newton that this sensor can measure on any of its
         * axis.
         * @return max force in Newton.
         */
        virtual double getMaxForce () = 0;

        /**
         * @brief gets the maximum torque in Newton Meter (N m)that this sensor can measure on any
         * of its axis.
         * @return max torque in Newton Meter(N m).
         */
        virtual double getMaxTorque () = 0;

        /**
         * @brief gets the force in N that is acting on the origin. The
         * force is described in relation to the origin.
         * @return force acting on origin.
         */
        virtual rw::math::Vector3D<double> getForce () = 0;

        /**
         * @brief gets the torgue in Nm that is acting on the origin. The
         * torque is described in relation to the origin.
         * @return torque acting on origin.
         */
        virtual rw::math::Vector3D<double> getTorque () = 0;

        /**
         * @brief the transform from the sensor frame to the point of origin.
         * @return transform from sensor frame to point of origin.
         */
        virtual rw::math::Transform3D<double> getTransform () = 0;
    };
    %template(FTSensorPtr) rw::core::Ptr<FTSensor>;

// ############### FTSensorModel ####################

    /**
     * @brief N-axis Force Torque sensor model
     */
    class FTSensorModel : public SensorModel
    {
      public:
        /**
         * Constructor
         * @param name [in] name of FT sensor
         * @param frame [in] the frame to which this sensor is attached
         * @param desc [in] optional description of sensor
         */
        FTSensorModel (const std::string& name, rw::kinematics::Frame* frame,
                       const std::string& desc = "");

        /**
         * @brief destructor
         * @return
         */
        virtual ~FTSensorModel ();

        /**
         * @brief get maximum wrench (force and torque) characteristics
         * @return
         */
        rw::math::Wrench6D<double> getMaxWrench () const;

        /**
         * @brief gets the maximum force in Newton that this sensor can measure on any of its
         * axis.
         * @return max force in Newton.
         */
        rw::math::Vector3D<double> getMaxForce () const;

        /**
         * @brief gets the maximum torque in Newton Meter (N m)that this sensor can measure on any
         * of its axis.
         * @return max torque in Newton Meter(N m).
         */
        rw::math::Vector3D<double> getMaxTorque () const;

        /**
         * @brief set the maximum wrench of this FTSensor
         * @param max [in] maximum allowed wrench
         */
        void setMaxWrench (const rw::math::Wrench6D<double>& max);

        /**
         * @brief gets the force in N that is acting on the origin. The
         * force is described in relation to the origin.
         * @return force acting on origin.
         */
        rw::math::Wrench6D<double> getWrench (const State& state) const;

        //! @brief set the wrench that is acting on the origin of this FTsensor
        void setWrench (const rw::math::Wrench6D<double>& wrench, const State& state);

        /**
         * @brief gets the force in N that is acting on the origin. The
         * force is described in relation to the origin.
         * @return force acting on origin.
         */
        rw::math::Vector3D<double> getForce (const State& state) const;

        //! @brief set the force that is acting on the origin of this FTsensor
        void setForce (const rw::math::Vector3D<double>& force, const State& state);

        /**
         * @brief gets the torgue in Nm that is acting on the origin. The
         * torque is described in relation to the origin.
         * @return torque acting on origin.
         */
        rw::math::Vector3D<double> getTorque (const State& state) const;

        //! @brief set the torque that is acting on the origin of this FTsensor
        void setTorque (const rw::math::Vector3D<double>& force, const State& state);

        /**
         * @brief the transform from the sensor frame to the point of origin.
         * @return transform from sensor frame to point of origin.
         */
        rw::math::Transform3D<double> getTransform () const;

        /**
         * @brief set the transform between frame and origin. The origin of the
         * sensor is the frame where sensor data is described.
         * @param t3d [in] transformation from frame to origin
         */
        void setTransform (const rw::math::Transform3D<double>& t3d);
    };
    %template(FTSensorModelPtr) rw::core::Ptr<FTSensorModel>;
    OWNEDPTR(FTSensorModel)

// ############### image ############################

    struct Pixel4i
    {
        Pixel4i (int v0, int v1, int v2, int v3);

        //! @brief up to four channels
        int ch[4];    
    };

    struct Pixel4f
    {
        Pixel4f (float v0, float v1, float v2, float v3);

        //! @brief up to four channels
        float ch[4];    
    };

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
            //! @brief Grayscale image 1-channel
            GRAY, 
            //! @brief 3-channel color image (Standard opengl)
            RGB,
            //! @brief 4-channel  color image with alpha channel
            RGBA, 
            //! @brief 3-channel color image (Standard OpenCV)
            BGR,
            //! @brief 4-channel color image with alpha channel
            BGRA,
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
            Depth8U,
            Depth8S,
            Depth16U,
            Depth16S,
            Depth32S,
            Depth32F
        } PixelDepth;

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
         * @brief returns the dimensions (width and height) of this image
         * @return a pair of integers where first is the width and second
         * is the height
         */
        std::pair< unsigned int, unsigned int > getImageDimension ();

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
         * @brief generic but inefficient access to pixel information. The float
         * value is between [0;1] which means non float images are scaled according to
         * their pixel depth (bits per pixel).
         * @param x [in] x coordinate
         * @param y [in] y coordinate
         * @return up to 4 pixels (depends on nr of channels) in a float format
         */
        Pixel4f getPixel (size_t x, size_t y) const;
        Pixel4f getPixelf (size_t x, size_t y) const;

        /**
         * @brief generic but inefficient access to pixel information. The float
         * value is between [0;1] which means non float images are scaled according to
         * their pixel depth (bits per pixel).
         * @param x [in] x coordinate
         * @param y [in] y coordinate
         * @param dst [out] up to 4 pixels (depends on nr of channels) in a float format
         */
        void getPixel (size_t x, size_t y, Pixel4f& dst) const;

        /**
         * @brief generic access to pixel information, however user must take care of the pixel
         * depth himself. If image is a Depth8U then the maximum value is 254. Also float images
         * are scaled accordingly.
         * @param x [in] x coordinate
         * @param y [in] y coordinate
         * @return up to 4 pixels (depends on nr of channels) as ints
         */
        Pixel4i getPixeli (size_t x, size_t y) const;

        /**
         * @brief generic access to pixel information, however user must take care of the pixel
         * depth himself. If image is a Depth8U then the maximum value is 254. Also float images
         * are scaled accordingly.
         * @param x [in] x coordinate
         * @param y [in] y coordinate
         * @param dst [out] up to 4 pixels (depends on nr of channels) in a float format
         */
        void getPixel (size_t x, size_t y, Pixel4i& dst) const;

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
        rw::core::Ptr<Image> copyFlip(bool horizontal, bool vertical) const;
    };

    %template (ImagePtr) rw::core::Ptr<Image>;
// ############### ImageUtil ########################
    %nodefaultctor ImageUtil;
    /**
     * @brief a collection of simple image utility functions
     */
    class ImageUtil
    {
      public:
        /**
         * @brief converts an image of RGB type into an image of
         * GRAY type.
         */
        static void RGB2GRAY (const Image& src, Image& dst);

        /*
         * @brief converts an image of type GRAY into an image of type RGB
         * @param src
         * @param dst
         */
        // static void GRAY2RGB(const Image& src, Image& dst);

        /**
         * @brief sets the value of all channels of an image to
         * \b color.
         */
        static void reset (Image& img, int color = 0);

        /**
         * @brief flips the image around the x-axis (horizontal)
         * @param img
         */
        static void flipX (Image& img);

        /**
         * @brief flips the image around the y-axis (vertical)
         * @param img
         */
        static void flipY (Image& img);

        /**
         * @cond
         * @param img
         * @endcond
         */
        // static void flipY(const Image& srcimg, Image& dstimg);

        /**
         * convert pointcloud to a depth image. Colors are scaled to min and ax distance of
         * points.
         * @param cloud [in] cloud to convert to image
         * @return image showing the pointcloud as a depth image
         */
        static rw::core::Ptr< Image >
        makeDepthImage (const PointCloud& cloud);

        /**
         * convert pointcloud to a depth image. Colors are scaled to min and max distance
         * specified by user
         * @param cloud [in] cloud to convert to image
         * @param min [in] the minimum distance corresponding to black
         * @param max [in] the maximum distance corresponding to white
         * @return image showing the pointcloud as a depth image
         */
        static rw::core::Ptr< Image >
        makeDepthImage (const PointCloud& cloud, float min, float max);
    };
// ############### RGBDCameraModel ##################

    class RGBDCameraModel : public SensorModel
    {
      public:

        /**
         * @brief constructor
         * @param name [in] name of sensor
         * @param frame documentation missing !
         * @param modelInfo [in] info string
         */
        RGBDCameraModel (const std::string& name, Frame* frame,
                         const std::string& modelInfo);

        /**
         * @brief destructor
         */
        virtual ~RGBDCameraModel ();
    };
    %template(RGBDCameraModelPtr) rw::core::Ptr<RGBDCameraModel>;

// ############### Scanner ##########################
    /**
     * @brief this interface describe a generic range scanning class.
     *
     */
    class Scanner: public Sensor
    {
      protected:
        /**
         * @brief constructor
         * @param name [in] name of sensor
         * @param desc [in] description of sensor
         */
        Scanner (const std::string& name, const std::string& desc) : Sensor (name, desc) {}

        /**
         * @brief constructor
         * @param name [in] name of sensor
         */
        Scanner (const std::string& name) : Sensor (name) {}

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
         * @brief Returns the min and max range of this Scanner
         * @return min and max range
         */
        virtual std::pair< double, double > getRange () = 0;

        /**
         * @brief returns the framerate that this camera is setup with
         *
         * @return the framerate in frames per second
         */
        virtual double getFrameRate() = 0;
    };
    %template(ScannerPtr) rw::core::Ptr<Scanner>;
    OWNEDPTR(Scanner);

// ############### Scanner1D ########################
    /**
     * @brief a one dimensional range scanner.
     */

    class Scanner1D : public Scanner
    {
      public:
        virtual ~Scanner1D ();
    };

// ############### Scanner2D ########################

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
      protected:
        /**
         * @brief constructor
         * @param name [in] name of scanner sensor
         * @param description [in] description of scanner sensor
         */
        Scanner2D (const std::string& name, const std::string& description = "");
        
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

    %template (Scanner2DPtr) rw::core::Ptr<Scanner2D>;
    OWNEDPTR(Scanner2D);


// ############### Scanner2DModel ###################

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
         * @brief Returns the min and max angular range of the scanner, where
         * the angles represent the beginning and end of scanning in the z-x plane.
         * Hence, angles represent rotation of z-axis around the y-axis. Normally range would
         * be something like -170 to 170 degree for a Hokyo or Sick scanner
         *
         * @return Angular range in radians
         */
        std::pair< double, double > getAngularRange () const ;

        /**
         * @brief Returns the number of scan points
         */
        size_t getMeasurementCount() const;

        /**
         * @brief get the min an max range in meters that is scannable by the 2D scanner
         * @return range in meters
         */
        std::pair< double, double > getDistanceRange () const;

        /**
         * @brief set distance range
         * @param range
         */
        void setDistanceRange (const std::pair< double, double >& range);

        /**
         * @brief set distance range
         *
         * @param min documentation missing !
         * @param max documentation missing !
         */
        void setDistanceRange(double min, double max );
    };

    %template (Scanner2DModelPtr) rw::core::Ptr<Scanner2DModel>;

// ############### Scanner25D #######################
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

    %template (Scanner25DPtr) rw::core::Ptr<Scanner25D>;
    OWNEDPTR(Scanner25D);

// ############### Scanner25DModel ###################
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

        //! @brief width of images taken with 25 sensor
        int getWidth() const;

        //! @brief height of images taken with 25 sensor
        int getHeight() const;

        //! @brief get the min and maximum depth of this scanner in meters
        std::pair< double, double > getRange () const;

        //! @brief set the min and maximum depth of this scanner in meters
        void setRange(double min, double max);

        //! @brief set the min and maximum depth of this scanner in meters
        void setRange (const std::pair< double, double >& range);
    };

    %template (Scanner25DModelPtr) rw::core::Ptr<Scanner25DModel>;
    OWNEDPTR(Scanner25DModel);

// ############### SensorData ########################
    /**
     * @brief toplevel class for sensor data. Basicly describes interface for
     * setting and getting timestamps.
     */
    class SensorData
    {
      public:
        /**
         * @brief constructor
         * @param timeStamp
         */
        SensorData (long timeStamp = 0);

        /**
         * @brief get timestamp of this sensor data
         * @return timestamp in ms
         */
        virtual long getTimeStamp ();

        /**
         * @brief set timestamp of this sensor data
         * @param timestamp [in] time in ms
         */
        virtual void setTimeStamp (long timestamp);
    };

// ############### StereoCameraModel #################
    /**
     * @brief The Camera class defines a general interface to a stereo camera.
     * A stereo camera consists of two cameras with the same intrinsic parameters,
     * but with different extrinsic parameters.
     *
     * Since ideal cameras are assumed, the intrinsics are given as a horizontal/vertical
     * pixel resolution as well as a horizontal field of view (FOV).
     *
     * The extrinsic parameters are given simply as two transformation matrices,
     * which give the pose of the cameras relative some external frame.
     *
     */
    class StereoCameraModel : public SensorModel
    {
      public:

        //! @brief output calibration file format for SaveCalibration()
        enum CalibrationFormat { OPENCV };

        //! @brief FOV direction
        enum FOVDirection { HORIZONTAL, VERTICAL };

        /**
         * @brief constructor
         * @param name [in] name of sensor
         * @param fov [in] horizontal field of view
         * @param width [in] width of image
         * @param height [in] height of image
         * @param TL [in] transform from sensor frame to left camera frame
         * @param TR [in] transform from sensor frame to right camera frame
         * @param frame [in] sensor frame
         * @param modelInfo [in] info string
         */
        StereoCameraModel (const std::string& name, double fov, double width, double height,
                           const rw::math::Transform3D<double>& TL, const rw::math::Transform3D<double>& TR,
                           Frame* frame, const std::string& modelInfo = "");
        /**
         * @brief destructor
         */
        virtual ~StereoCameraModel ();

        //! @brief get left image
        rw::core::Ptr<Image> getLeftImage (const State& state);

        //! @brief set left image
        void setLeftImage (rw::core::Ptr<Image> img, State& state);

        //! @brief get right image
        rw::core::Ptr<Image> getRightImage (const State& state);

        //! @brief set right image
        void setRightImage (rw::core::Ptr<Image> img, State& state);

        /**
         * @brief utility function for saving a stereo calibration to a file
         *
         * @param filename [in] file to save to
         * @param fov [in] field of view (FOV) [rad]
         * @param wx [in] horizontal pixels
         * @param wy [in] vertical pixels
         * @param TL [in] transformation of left camera frame
         * @param TR [in] transformation of right camera frame
         * @param direction [in] the direction of the specified FOV
         * @param format [in] calibration file format to use
         * @return true if the file was successfully saved, false otherwise
         */
        static bool SaveCalibration (const std::string& filename, double fov, double wx, double wy,
                                     const rw::math::Transform3D<double>& TL,
                                     const rw::math::Transform3D<double>& TR,
                                     FOVDirection direction   = HORIZONTAL,
                                     CalibrationFormat format = OPENCV);

        /**
         * @brief utility function for writing a camera calibration to a stream
         *
         * @param os the stream to write to
         * @param fov field of view (FOV) [rad]
         * @param wx horizontal pixels
         * @param wy vertical pixels
         * @param T [in] transformation of the camera frame
         * @param dist [in] distortion parameters
         * @param direction [in] the direction of the specified FOV
         * @param format [in] calibration file format to use
         */
        static void
        WriteCalibration (std::ostream& os, double fov, double wx, double wy,
                          const rw::math::Transform3D<double>& T,
                          const std::vector< double >& dist = std::vector< double > (4, 0.0),
                          FOVDirection direction = HORIZONTAL, CalibrationFormat format = OPENCV);

        
    };
    %template(StereoCameraModelPtr) rw::core::Ptr< StereoCameraModel >;

// ############### TactileArray ######################
    class TactileArray : public Sensor
    {
      public:

        /**
         * @brief constructor
         * @param name [in] name of sensor
         */
        TactileArray (const std::string& name) : Sensor (name) {}

        /**
         * @brief destructor
         */
        virtual ~TactileArray () {}

        Frame* getFrame () const;

        /**
         * @brief gets the size of an individual tactile cell with coordinates (x,y)
         * @param x
         * @param y
         * @return the dimensions of the tactile cell in meters
         */
        virtual rw::math::Vector2D<double> getTexelSize (int x, int y) const = 0;

        /**
         * @brief get the minimum and maximum pressure capability of any tactile
         * cell in the TactileArray
         * @return
         */
        virtual std::pair< double, double > getPressureLimit () const = 0;

        /**
         * @brief gets the 3d geometry of this tactilearray. The vertexes are expressed
         * realtive to the transform.
         * @return
         */
        virtual const boost::multi_array< rw::math::Vector3D<double>, 2 >& getVertexGrid () const = 0;

        /**
         * @brief a transformation from the sensor frame to the geometric data of
         * the tactile array.
         * @return
         */
        virtual const rw::math::Transform3D<double>& getTransform () const = 0;

        /**
         * @brief a matrix with position of each tactile cell center. The coordinates
         * are described relative to the TactileArray transform (see getTransform())
         * @return a matrix describing the center of each tactile cell.
         */
        virtual const boost::multi_array< rw::math::Vector3D<double>, 2 >& getCenters () const = 0;

        /**
         * @brief a matrix of normals that are described relative to each tactile
         * cell center.
         * @return
         */
        virtual const boost::multi_array< rw::math::Vector3D<double>, 2 >& getNormals () const = 0;

        virtual int getWidth () const = 0;

        virtual int getHeight () const = 0;

        //************** the statefull interface (dynamic states) ***************

        /**
         * @brief acquires force data from the tactile cells
         */
        virtual void acquire () = 0;

        /**
         * @brief returns the pressure on each texel of the TactileArray in
         * the unit N/m^2.
         * @return matrix of texel pressure values
         */
        virtual const Eigen::MatrixXf& getTexelData () const = 0;
    };
    %template(TactileArrayPtr) rw::core::Ptr< TactileArray > ;
    OWNEDPTR(TactileArray);

// ############### TactileArrayModel #################

    /**
     * @brief the TactileArrayModel describes tactile sensor consisting of
     * arrays of tactile cells that can be placed on a defined shape. The shape is described
     * with a matrix of 3d vertices. Such that tactil (0,0) maps to the quad
     * defined by the four vertices {(0,0),(0,1),(1,1),(1,0)}. Notice that the
     * normal is defined by sequence of the vertices and that the normal defines
     * the direction of tactile sensing.
     */
    class TactileArrayModel : public SensorModel
    {
      public:
        /**
         * @brief constructor
         * @param name [in] name of sensor
         * @param sensorframe [in] the frame to which the sensor is attached
         * @param fThmap [in] transformation from sensor frame to the heightmap definition
         * @param heightMap [in] a height map defining the height of each corner in the tactile
         * array
         * @param cell_width [in] width of cell
         * @param cell_height [in] height of cell
         */
        TactileArrayModel (const std::string& name, Frame* sensorframe,
                           const rw::math::Transform3D<double>& fThmap, const Eigen::MatrixXf& heightMap,
                           double cell_width, double cell_height);

        /**
         * @brief destructor
         */
        virtual ~TactileArrayModel ();

        /**
         * @brief gets the size of an individual tactile cell with coordinates (x,y)
         * @param x
         * @param y
         * @return the dimensions of the tactile cell in meters
         */
        rw::math::Vector2D<double> getTexelSize (int x, int y) const;

        /**
         * @brief get the minimum and maximum pressure capability of any tactile
         * cell in the TactileArray
         * @return min and max pressure in Pa
         */
        std::pair< double, double > getPressureLimit () const;

        /**
         * @brief set pressure limits. should define min max of any tactile cell in array
         * @param min [in] min pressure in Pa
         * @param max [in] max pressure in Pa
         */
        void setPressureLimit (double min, double max);
        void setPressureLimit (std::pair< double, double > range);
        /**
         * @brief gets the 3d geometry of this tactilearray. The vertexes are expressed
         * realtive to the transform.
         * @return
         */
        const boost::multi_array< rw::math::Vector3D<double>, 2 >& getVertexGrid () const;

        /**
         * @brief a transformation from the sensor frame to the geometric data of
         * the tactile array.
         * @return
         */
        const rw::math::Transform3D<double>& getTransform () const;

        /**
         * @brief a matrix with position of each tactile cell center. The coordinates
         * are described relative to the TactileArray transform (see getTransform())
         * @return a matrix describing the center of each tactile cell.
         */
        const boost::multi_array< rw::math::Vector3D<double>, 2 >& getCenters () const;

        /**
         * @brief a matrix of normals that are described relative to each tactile
         * cell center.
         * @return
         */
        const boost::multi_array< rw::math::Vector3D<double>, 2 >& getNormals () const;

        //! @brief get width of tactile array
        int getWidth () const;

        //! @brief get height of tactile array
        int getHeight () const;

        //************** the statefull interface (dynamic states) ***************
        /**
         * @brief returns the pressure on each texel of the TactileArray in
         * the unit Pa (N/m^2).
         * @param state [in] state to get the values from
         * @return matrix of texel pressure values
         */
        Eigen::MatrixXf& getTexelData (State& state) const;

        /**
         * @brief set the pressure on each texel of the TactileArray in
         * the unit Pa (N/m^2).
         * @param data [in] pressure values
         * @param state [in] state to set the values in
         */
        void setTexelData (const Eigen::MatrixXf& data, State& state) const;

    };
    %template(TactileArrayModelPtr) rw::core::Ptr< TactileArrayModel > ;
    %template(TactileVertexMatrix) boost::multi_array< rw::math::Vector3D<double>, 2 >;
    OWNEDPTR(TactileArrayModel);

// ############### TactileArrayUtil ##################

    /**
     * @brief Utillity class for general computations on a tactile array
     */
    class TactileArrayUtil
    {
      public:
        /**
         * @brief Estimate the contacts on the tactile array sensor.
         * @param arraySensor [in] the array sensor that describe the tactile array
         * @param state [in] the current state of the system
         * @param minContactForce [in] A threshold value that determines when a force is a contact
         * force and not just noise.
         * @return All estimated contacts
         */
        static std::vector< Contact3D >
        estimateContacts (const TactileArrayModel& arraySensor,
                          const State& state, double minContactForce);
    };