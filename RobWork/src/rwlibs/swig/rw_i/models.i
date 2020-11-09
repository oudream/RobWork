// ################# ControllerModel

    /**
     * @brief Interface to allow modelling of different types of controllers.
     * A controller is an instance that takes an input manipulates it to an output
     * that in effect controls something. As such controllers vary greatly and have
     * only little in common.
     */
    class ControllerModel : public rw::kinematics::Stateless
    {
      public:

        /**
         * @brief constructor
         * @param name [in] the name of this controllermodel
         * @param frame [in] the frame to which this controller is attached/associated.
         */
        ControllerModel (const std::string& name, rw::kinematics::Frame* frame);

        /**
         * @brief constructor
         * @param name [in] the name of this controllermodel
         * @param frame [in] the frame to which this controller is attached/associated.
         * @param description [in] description of the controller
         */
        ControllerModel (const std::string& name, rw::kinematics::Frame* frame,
                         const std::string& description);

        //! destructor
        virtual ~ControllerModel () {}

        /**
         * @brief sets the name of this controllermodel
         * @param name [in] name of this controllermodel
         */
        void setName (const std::string& name);

        /**
         * @brief sets the description of this controllermodel
         * @param description [in] description of this controllermodel
         */
        void setDescription (const std::string& description);

        /**
         * @brief returns the name of this controllermodel
         * @return name of controllermodel
         */
        const std::string& getName () const;

        /**
         * @brief returns a description of this controllermodel
         * @return reference to this controllermodels description
         */
        const std::string& getDescription () const;

        /**
         * @brief The frame to which the controllermodel is attached.
         *
         * The frame can be NULL.
         */
        rw::kinematics::Frame* getFrame () const;

        /**
         * @brief Sets the frame to which the controllermodel should be attached
         *
         * @param frame The frame, which can be NULL
         */
        virtual void attachTo (rw::kinematics::Frame* frame);

        /**
         * @brief gets the propertymap of this controllermodel
         */
        rw::core::PropertyMap& getPropertyMap ();

        #if !defined(SWIGJAVA)
            /**
             * @brief gets the propertymap of this controllermodel
             */
            const rw::core::PropertyMap& getPropertyMap () const;
        #endif
    };
    %template(ControllerModelPtr) rw::core::Ptr<ControllerModel>;
    %template(ControllerModelPtrVector) std::vector<rw::core::Ptr<ControllerModel> >;

// ################# Objects

    /**
     * @brief The object class represents a physical thing in the scene which has geometry.
     * An object has a base frame (similar to a Device) and may have a number of associated frames.
     */
    class Object: public rw::kinematics::Stateless
    {
      public:

        //! @brief destructor
        virtual ~Object();

        /**
         * @brief get name of this object. Name is always the same as the name of the
         * base frame.
         * @return name of object.
         */
        const std::string& getName();

        /**
         * @brief get base frame of this object
         * @return base frame of object
         */
        rw::kinematics::Frame* getBase();

        /**
         * @brief get all associated frames of this object
         * @return a vector of frames
         */
        const std::vector<rw::kinematics::Frame*>& getFrames();

        /**
         * @brief associate a frame to this Object.
         * @param frame [in] frame to associate to object
         */       
        void addFrame(rw::kinematics::Frame* frame);

        /**
         * @brief get default geometries
         * @return geometry for collision detection
         */
        const std::vector<rw::core::Ptr<Geometry> >& getGeometry() const;

        /**
         * @brief get the default models
         * @return models for vizualization
         */
        const std::vector<rw::core::Ptr<Model3D> >& getModels() const;


         /**
         * @brief get geometry of this object
         * @return geometry for collision detection.
         */
        const std::vector<rw::core::Ptr< Geometry >>& getGeometry(const rw::kinematics::State& state) const;
        
        /**
         * @brief get visualization models of this object
         * @return models for visualization
         */
        const std::vector<rw::core::Ptr<Model3D> >& getModels(const rw::kinematics::State& state) const;

        // stuff that should be implemented by deriving classes

        /**
	     * @brief get mass in Kg of this object
	     * @return mass in kilo grams
	     */
        virtual double getMass(rw::kinematics::State& state) const = 0;

        /**
	     * @brief get center of mass of this object
	     * @param state [in] the state in which to get center of mass
	     * @return
	     */
        virtual rw::math::Vector3D<double> getCOM(rw::kinematics::State& state) const = 0;

        /**
	     * @brief returns the inertia matrix of this body calculated around COM with the orientation
	     * of the base frame.
	     */
        virtual rw::math::InertiaMatrix<double> getInertia(rw::kinematics::State& state) const = 0;

      protected:
        //! constructor
        Object(rw::kinematics::Frame* baseframe);
        //! constructor - first frame is base
        Object(std::vector<rw::kinematics::Frame*> frames);
    };
    %template (ObjectPtr) rw::core::Ptr<Object>;
    %template (ObjectPtrVector) std::vector< rw::core::Ptr< Object > >;
    OWNEDPTR(Object);


// ################# DeformableObject

    /**
     @brief The deformable object is an object that contain a deformable mesh. Deformations
    are part of the state object and they are modeled/controlled through control nodes.
    each control node correspond to a vertice in the mesh. All vertices are described relative to the base
    frame of the object.
    */
    class DeformableObject: public Object
    {
      public:

        /**
         * @brief constructor - constructs a deformable mesh with a specific number of control nodes
         * and without any faces. Both geometry and model are created based on nodes.
         * @param baseframe [in] base frame of object
         * @param nr_of_nodes [in] the number of controlling nodes in the deformable object
         */
        DeformableObject(rw::kinematics::Frame* baseframe, int nr_of_nodes);


        /**
         * @brief constructor - control nodes are taken as vertices in the Model3D. Vertices that
         * are equal are merged into the same control node. All faces of the model are used to
         * define faces of the deformable object.
         *
         * geometry will be created based on model information
         *
         * @note only triangle faces are currently supported.
         *
         * @param baseframe [in] base frame of object
         * @param model [in]
         */
        DeformableObject(rw::kinematics::Frame* baseframe, rw::core::Ptr<Model3D> model);

        /**
         * @brief constructor - control nodes are taken from a triangle mesh generated from triangulating the
         * geometry. Vertices that
         * are equal are merged into the same control node. All faces of the geometry are used to
         * define faces of the deformable object.
         *
         * model will be created based on geometry information
         *
         * @param baseframe [in] base frame of object
         * @param geom [in] geometry to define the faces and nodes
         */
        DeformableObject(rw::kinematics::Frame* baseframe, rw::core::Ptr<Geometry> geom);

        //! @brief destructor
        virtual ~DeformableObject();


        /**
         * @brief get a specific node from the state
         * @param id [in] id of the node to fetch
         * @param state [in] current state
         * @return handle to manipulate a node in the given state.
         */
        rw::math::Vector3D<float>& getNode(int id, rw::kinematics::State& state) const;

        /**
         * @brief set the value of a specific node in the state.
         * @param id [in] id of the node
         * @param v [in] value to set.
         * @param state [in] state in which to set the value.
         */
        void setNode(int id, const rw::math::Vector3D<float>& v, rw::kinematics::State& state);

        /**
         * @brief get the number of controlling nodes of this deformable object.
         * @param state [in]
         * @return
         */
        size_t getNrNodes(const rw::kinematics::State& state) const ;

        /**
         * @brief get the number of controlling nodes of this deformable object.
         * @return Number of Nodes
         */
        size_t getNrNodes() const;
        
        /**
         * @brief get all faces of this soft body
         * @return list of indexed triangles - indeces point to vertices/nodes
         */
        const std::vector<rw::geometry::IndexedTriangle<uint16_t> >& getFaces() const;

        /**
         * @brief add a face to three existing nodes
         * @param node1 [in] idx of node 1
         * @param node2 [in] idx of node 2
         * @param node3 [in] idx of node 3
         */
        void addFace(unsigned int node1, unsigned int node2, unsigned int node3);

        /**
         * @brief return a triangle mesh representing the softbody in the current state \b cstate
         * @param cstate
         */
        rw::core::Ptr< rw::geometry::IndexedTriMesh<float> > getMesh(rw::kinematics::State& cstate);

        /**
         * @brief get mass in Kg of this object
         * @param state [in] the state
         * @return mass in kilo grams
         */
        double getMass(rw::kinematics::State& state) const;

        /**
         * @brief get center of mass of this object
         * @param state [in] the state in which to get center of mass
         * @return Position of COM
         */    
        rw::math::Vector3D<double> getCOM(rw::kinematics::State& state) const;


        /**
         * @brief returns the inertia matrix of this body calculated around COM with the orientation
         * of the base frame.
         * @param state [in] the state to get the inertia in
         * @return matrix with inertia 
         */
        rw::math::InertiaMatrix<double> getInertia(rw::kinematics::State& state) const;

        /**
         * @brief updates the model with the current state of the deformable model
         * @param model [in/out] model to be updated
         * @param state
         */
        void update(rw::core::Ptr<Model3D> model, const rw::kinematics::State& state);
    };

    %template (DeformableObjectPtr) rw::core::Ptr<DeformableObject>;
    %template (DeformableObjectPtrVector) std::vector<rw::core::Ptr<DeformableObject>>;
    OWNEDPTR(DeformableObject);





// ################# Device

    /**
     * @brief An abstract device class
     *
     * The Device class is the basis for all other devices. It is assumed that all devices
	 * have a configuration which can be encoded by a Q, that all have a base frame
	 * representing where in the work cell they are located and a primary end frame. Notice that
	 * some devices may have multiple end-frames.
     */
    class Device
    {
      public:

        /**
         * Constructs a device with a name
         *
         * @param name [in] name of the device
         */
        Device(const std::string& name);
        
        /**
         * @brief Sets configuration vector @f$ \mathbf{q} \in \mathbb{R}^n @f$
         *
         * @param q [in] configuration vector @f$ \mathbf{q} @f$
         * @param state [in] state into which to set @f$ \mathbf{q} @f$
         *
         * @pre q.size() == getDOF()
         */
        virtual void setQ(const rw::math::Q& q, rw::kinematics::State& state) const = 0;

        /**
         * @brief Gets configuration vector @f$ \mathbf{q}\in \mathbb{R}^n @f$
         *
         * @param state [in] state from which which to get @f$ \mathbf{q} @f$
         * @return configuration vector @f$ \mathbf{q} @f$
         */
        virtual rw::math::Q getQ(const rw::kinematics::State& state) const = 0;

        /**
         * @brief Returns the upper @f$ \mathbf{q}_{min} \in \mathbb{R}^n @f$ and
         * lower @f$ \mathbf{q}_{max} \in \mathbb{R}^n @f$ bounds of the joint space
         *
         * @return std::pair containing @f$ (\mathbf{q}_{min}, \mathbf{q}_{max}) @f$
         */
        virtual std::pair<rw::math::Q,rw::math::Q> getBounds() const = 0;

        /**
         * @brief Sets the upper @f$ \mathbf{q}_{min} \in \mathbb{R}^n @f$ and
         * lower @f$ \mathbf{q}_{max} \in \mathbb{R}^n @f$ bounds of the joint space
         *
         * @param bounds [in] std::pair containing
         * @f$ (\mathbf{q}_{min}, \mathbf{q}_{max}) @f$
         */
        virtual void setBounds (const std::pair<rw::math::Q,rw::math::Q>& bounds) = 0;

        /**
         * @brief Returns the maximal velocity of the joints
         * \f$\mathbf{\dot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \dot{\mathbf{q}}_{min}=-\dot{\mathbf{q}}_{max}\f$
         *
         * @return the maximal velocity
         */
        virtual rw::math::Q getVelocityLimits() const = 0;

        /**
         * @brief Sets the maximal velocity of the joints
         * \f$\mathbf{\dot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \dot{\mathbf{q}}_{min}=-\dot{\mathbf{q}}_{max}\f$
         *
         * @param vellimits [in] rw::math::Q with the maximal velocity
         */
        virtual void setVelocityLimits(const rw::math::Q& vellimits) = 0;

        /**
         * @brief Returns the maximal acceleration of the joints
         * \f$\mathbf{\ddot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \ddot{\mathbf{q}}_{min}=-\ddot{\mathbf{q}}_{max}\f$
         *
         * @return the maximal acceleration
         */
        virtual rw::math::Q getAccelerationLimits() const = 0;

        /**
         * @brief Sets the maximal acceleration of the joints
         * \f$\mathbf{\ddot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \ddot{\mathbf{q}}_{min}=-\ddot{\mathbf{q}}_{max}\f$
         *
         * @param  acclimits [in] the maximal acceleration
         */
        virtual void setAccelerationLimits(const rw::math::Q& acclimits) = 0;

        /**
         * @brief Returns number of active joints
         * @return number of active joints @f$ n @f$
         */
        virtual size_t getDOF() const = 0;

        /**
         * @brief Returns the name of the device
         * @return name of the device
         */
        std::string getName() const;

        /**
         * @brief Sets the name of the Device
         * @param name [in] the new name of the frame
         */
        void setName(const std::string& name);

        /**
         * @brief a method to return the frame of the base of the device.
         * @return the base frame
         */
        virtual rw::kinematics::Frame* getBase() = 0;

        /**
         * @brief a method to return the frame of the end of the device
         * @return the end frame
         */
        virtual rw::kinematics::Frame* getEnd() = 0;

    #if !defined(SWIGJAVA)
        virtual const rw::kinematics::Frame* getBase() const = 0;
        virtual const rw::kinematics::Frame* getEnd() const = 0;
    #endif

        /**
         * @brief Calculates the homogeneous transform from base to a frame f
         * @f$ \robabx{b}{f}{\mathbf{T}} @f$
         * @return the homogeneous transform @f$ \robabx{b}{f}{\mathbf{T}} @f$
         */
        rw::math::Transform3D<double>  baseTframe(const rw::kinematics::Frame* f, const rw::kinematics::State& state) const;

        /**
         * @brief Calculates the homogeneous transform from base to the end frame
         * @f$ \robabx{base}{end}{\mathbf{T}} @f$
         * @return the homogeneous transform @f$ \robabx{base}{end}{\mathbf{T}} @f$
         */
        rw::math::Transform3D<double>  baseTend(const rw::kinematics::State& state) const;

        /**
         * @brief Calculates the homogeneous transform from world to base @f$
         * \robabx{w}{b}{\mathbf{T}} @f$
         *
         * @return the homogeneous transform @f$ \robabx{w}{b}{\mathbf{T}} @f$
         */
        rw::math::Transform3D<double>  worldTbase(const rw::kinematics::State& state) const;

        /**
         * @brief Calculates the jacobian matrix of the end-effector described
         * in the robot base frame @f$ ^{base}_{end}\mathbf{J}_{\mathbf{q}}(\mathbf{q})
         * @f$
         *
         * @param state [in] State for which to calculate the Jacobian
         *
         * @return the @f$ 6*ndof @f$ jacobian matrix: @f$
         * {^{base}_{end}}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * This method calculates the jacobian relating joint velocities (@f$
         * \mathbf{\dot{q}} @f$) to the end-effector velocity seen from
         * base-frame (@f$ \nu^{ase}_{end} @f$)
         *
         * \f[
         * \nu^{base}_{end} =
         * {^{base}_{end}}\mathbf{J}_\mathbf{q}(\mathbf{q})\mathbf{\dot{q}}
         * \f]
         *
         *
         * The jacobian matrix \f[ {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) \f]
         * is defined as:
         *
         * \f[
         * {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         * \frac{\partial ^{base}\mathbf{x}_n}{\partial \mathbf{q}}
         * \f]
         *
         * Where:
         * \f[
         *  {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         *  \left[
         *    \begin{array}{cccc}
         *      {^{base}_1}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) 
         *      {^{base}_2}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) 
         *      \cdots
         *      {^b_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) \\
         *    \end{array}
         *  \right]
         * \f]
         * where \f$ {^{base}_i}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) \f$ is defined by
         * \f[
         *  {^{base}_i}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         *  \begin{array}{cc}
         *    \left[
         *      \begin{array}{c}
         *        {^{base}}\mathbf{z}_i \times {^{i}\mathbf{p}_n} \\
         *        {^{base}}\mathbf{z}_i \\
         *      \end{array}
         *    \right] \textrm{revolute joint}
         *  \end{array}
         * \f]
         * \f[
         *  {^{base}_i}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         *  \begin{array}{cc}
         *    \left[
         *      \begin{array}{c}
         *        {^{base}}\mathbf{z}_i \\
         *        \mathbf{0} \\
         *    \end{array}
         *    \right] \textrm{prismatic joint} \\
         *  \end{array}
         * \f]
         *
         * By default the method forwards to baseJframe().
         */
        virtual rw::math::Jacobian baseJend(const rw::kinematics::State& state) const = 0;

        /**
         * @brief Calculates the jacobian matrix of a frame f described in the
         * robot base frame @f$ ^{base}_{frame}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * @param frame [in] Frame for which to calculate the Jacobian
         * @param state [in] State for which to calculate the Jacobian
         *
         * @return the @f$ 6*ndof @f$ jacobian matrix: @f$
         * {^{base}_{frame}}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * This method calculates the jacobian relating joint velocities (@f$
         * \mathbf{\dot{q}} @f$) to the frame f velocity seen from base-frame
         * (@f$ \nu^{base}_{frame} @f$)
         *
         * \f[
         * \nu^{base}_{frame} =
         * {^{base}_{frame}}\mathbf{J}_\mathbf{q}(\mathbf{q})\mathbf{\dot{q}}
         * \f]
         *
         *
         * The jacobian matrix \f[ {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) \f]
         * is defined as:
         *
         * \f[
         * {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         * \frac{\partial ^{base}\mathbf{x}_n}{\partial \mathbf{q}}
         * \f]
         *
         * By default the method forwards to baseJframes().
         */
        virtual rw::math::Jacobian baseJframe(const rw::kinematics::Frame* frame,const rw::kinematics::State& state) const;

        /**
         * @brief The Jacobian for a sequence of frames.
         *  
         * A Jacobian is computed for each of the frames and the Jacobians are
         * stacked on top of eachother.
         * @param frames [in] the frames to calculate the frames from
         * @param state [in] the state to calculate in
         * @return the jacobian
         */
        virtual rw::math::Jacobian baseJframes(const std::vector<rw::kinematics::Frame*>& frames,const rw::kinematics::State& state) const;

        /**
         * @brief Miscellaneous properties of the device.
         *
         * The property map of the device is provided to let the user store
         * various settings for the device. The settings are typically loaded
         * from setup files.
         *
         * The low-level manipulations of the property map can be cumbersome. To
         * ease these manipulations, the PropertyAccessor utility class has been
         * provided. Instances of this class are provided for a number of common
         * settings, however it is undecided if these properties are a public
         * part of RobWork.
         *
         * @return The property map of the device.
         */
        rw::core::PropertyMap& getPropertyMap ();

      private:
        Device(const Device&);
        Device& operator=(const Device&);
    };

    %template (DevicePtr) rw::core::Ptr<Device>;
    %template (DeviceCPtr) rw::core::Ptr<const Device>;
    %template (DevicePtrVector) std::vector<rw::core::Ptr<Device> >;
    OWNEDPTR(Device)

    %extend rw::core::Ptr<Device> {
        rw::core::Ptr<const Device> asDeviceCPtr() { return *$self; }
    }


// ################# DHParameterSet
    %nodefaultctor DHParameterSet;

    /**
     * @brief Simple class to help represent a set of Denavit-Hartenberg
     * parameters
     */
    class DHParameterSet
    {
      public:
        /**
         * @brief Constructor for DHParameters initialized to zero.
         */
        DHParameterSet();

        /**
         * @brief Constructor
         * @param alpha [in] \f$\alpha_{i-1}\f$
         * @param a [in] \f$ a_{i-1}\f$
         * @param d [in] \f$ d_{i}\f$
         * @param theta [in] \f$\theta_{i-1}\f$
         */
        DHParameterSet(double alpha, double a, double d, double theta);

        /**
         * @brief Constructor
         * @param alpha [in] \f$\alpha_{i-1}\f$
         * @param a [in] \f$ a_{i-1}\f$
         * @param d [in] \f$ d_{i}\f$
         * @param theta [in] \f$\theta_{i-1}\f$
         * @param type documentation missing !
         */
        DHParameterSet(double alpha, double a, double d, double theta, const std::string& type);

        /**
         * @brief Constructor
         * @param alpha [in] \f$\alpha_{i-1}\f$
         * @param a [in] \f$ a_{i-1}\f$
         * @param beta [in] documentation missing !
         * @param b [in] documentation missing !
         * @param parallel [in] documentation missing !
         */
        DHParameterSet(double alpha, double a, double beta, double b, bool parallel);


        /** @brief \f$\alpha_{i-1}\f$ **/
        double alpha() const;

        /** @brief \f$ a_{i-1}\f$ **/
        double a() const;

        /** @brief \f$ d_{i} \f$ **/
        double d() const;

        /** @brief \f$ \theta_{i} \f$ **/
        double theta() const;

        double b() const;

        double beta() const;

        bool isParallel() const;

        /**
         * @brief the DH-convention type
         */
        std::string getType() const;

        /**
         * @brief Returns the DH-Parameters for a SerialDevice. 
         *
         * If no or only a partial DH representation exists only the list will be empty or non-complete.
         *
         * @param device [in] SerialDevice for which to get the DH parameters
         * @return The set of DH parameters
         */
        static std::vector<DHParameterSet> getDHParameters(rw::core::Ptr<SerialDevice> device);

        static const DHParameterSet* get(const rw::core::PropertyMap& pmap);

        static const DHParameterSet* get(const Joint* joint);

        static void set(const DHParameterSet& dhset, rw::core::PropertyMap& pmap);

        static void set(const DHParameterSet& dhset, rw::kinematics::Frame* joint);

    };

    %template (DHParameterSetVector) std::vector<DHParameterSet>;

// ################# JacobianCalculator
    %nodefaultctor JacobianCalculator;
    //! @brief JacobianCalculator provides an interface for obtaining a Jacobian
    class JacobianCalculator
    {
      public:
        //! @brief Destructor
        virtual ~JacobianCalculator();

        /**
         * @brief Returns the Jacobian associated to \b state
         * @param state [in] State for which to calculate the Jacobian
         * @return Jacobian for \b state
         */
        virtual rw::math::Jacobian get(const rw::kinematics::State& state) const = 0;

        %extend {
            /**
             * @brief Returns the Jacobian associated to \b state
             *
             * @param state [in] State for which to calculate the Jacobian
             * @return Jacobian for \b state
             */
            virtual rw::math::Jacobian getJacobian(const rw::kinematics::State& state) const {
                return $self->get(state);
            }
        };
    };

    %template (JacobianCalculatorPtr) rw::core::Ptr<JacobianCalculator>;
    OWNEDPTR(JacobianCalculator);

// ################# DeviceJacobianCalculator
    /**
     * @brief Calculator for Jacobians of one or several Devices.
     *
     * Implements Jacobian calculations for one or several Devices.
     *
     * If more than one end-effector is given a "stacked" Jacobian is returned.
     *
     */
    class DeviceJacobianCalculator : public JacobianCalculator
    {
      public:
        /**
         * @brief Constructs JacobianCalculator.
         *
         * The dimension of the jacobian wil be (tcps.size() * 6, device.getDOF()).
         *
         * @param devices [in] The device to calculate for
         * @param base [in] Reference base of the Jacobian. Does not have to be the same as the base
         * of the device
         * @param tcps [in] List of tool end-effectors for which to calculate the Jacobian.
         * @param state [in] State giving how frame are connected
         */
        DeviceJacobianCalculator (std::vector< rw::core::Ptr<Device> > devices, const rw::kinematics::Frame* base,
                                  const std::vector< rw::kinematics::Frame* >& tcps,
                                  const rw::kinematics::State& state);

        /**
         * @brief Destructor
         */
        virtual ~DeviceJacobianCalculator ();

        /**
         * @copydoc JacobianCalculator::get(const State&) const
         */
        virtual rw::math::Jacobian get (const rw::kinematics::State& state) const;
    };

// ################# JacobianUtil
    /**
       @brief Primitive utilities for computing jacobians for joints of various
       types.
    */
    class JacobianUtil
    {
      public:
        /**
           @brief Add to column \b col of \b jacobian the Jacobian of a revolute
           joint with transform \b joint for a tool position of \b tcp.

           The Jacobian is given relative to the common world frame of \b joint
           and \b tcp.
        */
        static void addRevoluteJacobianCol (rw::math::Jacobian& jacobian, int row, int col,
                                            const rw::math::Transform3D<double>& joint,
                                            const rw::math::Transform3D<double>& tcp);

        /**
           @brief Add to column \b col of \b jacobian the Jacobian of a
           prismatic joint with transform \b joint for a tool position of \b
           tcp.

           The Jacobian is given relative to the common world frame of \b joint
           and \b tcp.
        */
        static void addPrismaticJacobianCol (rw::math::Jacobian& jacobian, int row, int col,
                                             const rw::math::Transform3D<double>& joint,
                                             const rw::math::Transform3D<double>& tcp);

        /**
           @brief Add to column \b col of \b jacobian the Jacobian for a passive
           revolute joint at position \b passive that controls the tool at
           position \b tcp. The joint scaling factor of the passive joint is \b
           scale.

           The Jacobian is given relative to the common world frame of \b joint
           and \b tcp.
        */
        static void addPassiveRevoluteJacobianCol (rw::math::Jacobian& jacobian, int row, int col,
                                                   const rw::math::Transform3D<double>& passive,
                                                   const rw::math::Transform3D<double>& tcp, double scale);

        /**
           @brief True iff \b child is in the subtree of \b parent for a tree
           structure of \b state.

           <code>isInSubTree(frame, frame, state)</code> is true always.

           This utility function is used for checking if a given joint does
           affect some tcp frame or if the Jacobian column for that joint should
           be considered zero.

           isInSubTree() runs in time proportional to the size of the subtree.
        */
        static bool isInSubTree (const rw::kinematics::Frame& parent, const rw::kinematics::Frame& child,
                                 const rw::kinematics::State& state);
    };


// ################# Joint
    /**
     * @brief A Joint is a Frame with assignable values for
     * position, velocity limits and acceleration limits.
     *
     */
    class Joint : public rw::kinematics::Frame
    {
      protected:
        /**
         * @brief Default constructor for the joint interface.
         *
         * @param name [in] The name of the frame.
         * @param dof [in] the degrees of freedom of this joint
         */

        Joint (const std::string& name, size_t dof);

        /**
         * @brief constructor - with the possiblity of adding additional
         * states than the dofs.
         * @param name [in] The name of the joint frame.
         * @param dof [in] degree of freedom of the joint
         * @param stateSize [in] additional doubles to allocate space for in the state
         */
        Joint (const std::string& name, size_t dof, size_t stateSize);

      public:
        /**
         * @brief Virtual destructor
         */
        virtual ~Joint () {}


        /**
         * @brief Sets joint bounds
         * @param bounds [in] the lower and upper bounds of this joint
         */
        void setBounds (const std::pair< const rw::math::Q, const rw::math::Q >& bounds);

        /**
         * @brief Gets joint bounds
         * @return the lower and upper bound of this joint
         */
        const std::pair< rw::math::Q, rw::math::Q >& getBounds () const;

        /**
         * @brief Sets max velocity of joint
         * @param maxVelocity [in] the new maximum velocity of the joint
         */
        void setMaxVelocity (const rw::math::Q& maxVelocity);

        /**
         * @brief Gets max velocity of joint
         * @return the maximum velocity of the joint
         */
        const rw::math::Q& getMaxVelocity () const;

        /**
         * @brief Sets max acceleration of joint
         * @param maxAcceleration [in] the new maximum acceleration of the joint
         */
        void setMaxAcceleration (const rw::math::Q& maxAcceleration);

        /**
         * @brief Gets max acceleration of joint
         * @return the maximum acceleration of the joint
         */
        const rw::math::Q& getMaxAcceleration () const;

        /**
         * @brief Finds the Jacobian of the joints and adds it in \b jacobian.
         *
         * Calculates the Jacobian contribution to the device Jacobian when controlling a frame \b
         * tcp and given a current joint pose \b joint.
         *
         * The values are stored from row \b row to \b row+5 and column \b col to
         * col+(joint.getDOF()-1).
         *
         * @param row [in] Row where values should be stored
         * @param col [in] Column where values should be stored
         * @param joint [in] Transform of the joint
         * @param tcp [in] Transformation of the point to control
         * @param state
         * @param jacobian [in] Jacobian to which to add the results.
         */
        virtual void getJacobian (size_t row, size_t col, const rw::math::Transform3D<double>& joint,
                                  const rw::math::Transform3D<double>& tcp, const rw::kinematics::State& state,
                                  rw::math::Jacobian& jacobian) const = 0;

        /**
         * @brief get the fixed transform from parent to this joint
         *
         * Notice that this does not include the actual rotation of the joint (its state)
         * only its fixed transform.
         *
         * @return fixed part of transform from paretn to joint
         */
        virtual rw::math::Transform3D<double> getFixedTransform () const = 0;

        /**
         * @brief change the transform from parent to joint base.
         * @param t3d [in] the new transform.
         */
        virtual void setFixedTransform (const rw::math::Transform3D<double>& t3d) = 0;

        /**
         * @brief get the isolated joint transformation which is purely dependent on
         * q.
         * @param state [in] the state from which to extract q
         * @return the joint transformation
         */
        virtual rw::math::Transform3D<double>
        getJointTransform (const rw::kinematics::State& state) const = 0;

        /**
         * @brief set the active state of the joint
         * @param isActive [in] true to enable control/motorization of joint, false otherwise
         */
        void setActive (bool isActive);

        /**
         * @brief a joint is active if its motorized/controlled in some
         * fasion. passive or non-active joints are typically used in parrallel robots.
         * @return
         */
        bool isActive () const;

        /**
         * @brief set the function to be used in transforming from the state q to the actual q
         * needed.
         *
         * This function can be used e.g. by a calibration.
         * @param function [in] function with first order derivative.
         */
        virtual void setJointMapping (rw::core::Ptr<rw::math::Function1Diff<>> function) = 0;

        /**
         * @brief removes mapping of joint values
         */
        virtual void removeJointMapping () = 0;
    };
    %template(JointPtr) rw::core::Ptr< Joint >;
    %template(JointPointerVector) std::vector<Joint*>;

// ################# JointDevice

    /**
     @brief A device for a sequence of joints.

     Contrary to for example SerialDevice and TreeDevice, the joints need not
     have any particular ordering within the kinematic tree.

     A JointDevice is a joint for which the values of the configuration Q each
     correspond to a frame of type Joint.

     To implement a Device it is common to derive from JointDevice and just
     add implement methods where your device differs from the standard
     behaviour. Subclasses typically differ in their implementation of setQ()
     and the Jacobian computation.
     */
    class JointDevice: public Device
    {
    public:

        /**
         * @brief Construct the device for a sequence of joints.
         * @param name [in] name of device
         * @param base [in] the base of the device
         * @param end [in] the end (or tool) of the device
         * @param joints [in] the joints of the device
         * @param state [in] the state that shows how frames are connected as
                needed for the computation of Jacobians.
         */
        JointDevice (const std::string& name, rw::kinematics::Frame* base, rw::kinematics::Frame* end,
                     const std::vector< Joint* >& joints, const rw::kinematics::State& state);

        /**
         * @brief Get all joints of this device
         * @return all joints
         */
        const std::vector<Joint*>& getJoints() const;

        /** @copydoc Device::setQ */
        void setQ(const rw::math::Q& q, rw::kinematics::State& state) const;

        /** @copydoc Device::getQ */
        rw::math::Q getQ(const rw::kinematics::State& state) const;

        /** @copydoc Device::getDOF */
        size_t getDOF() const;

        /** @copydoc Device::getBounds */
        std::pair<rw::math::Q, rw::math::Q> getBounds() const;

        /** @copydoc Device::setBounds */
        void setBounds(const std::pair<rw::math::Q, rw::math::Q>& bounds);

        /** @copydoc Device::getVelocityLimits */
        rw::math::Q getVelocityLimits() const;

        /** @copydoc Device::setVelocityLimits */
        void setVelocityLimits(const rw::math::Q& vellimits);

        /** @copydoc Device::getAccelerationLimits */
        rw::math::Q getAccelerationLimits() const;

        /** @copydoc Device::setAccelerationLimits */
        void setAccelerationLimits(const rw::math::Q& acclimits);

        /** @copydoc Device::baseJend */
        rw::math::Jacobian baseJend(const rw::kinematics::State& state) const;

        /** @copydoc Device::baseJCframes */
        rw::core::Ptr< JacobianCalculator > baseJCframes (const std::vector< rw::kinematics::Frame* >& frames,
                                              const rw::kinematics::State& state) const;

        /** @copydoc Device::getBase */
        rw::kinematics::Frame* getBase();

        /** @copydoc Device::getEnd() */
        virtual rw::kinematics::Frame* getEnd();

    #if !defined(SWIGJAVA)
        /** @copydoc Device::getBase() */
        const rw::kinematics::Frame* getBase() const;

        /** @copydoc Device::getEnd() */
        virtual const rw::kinematics::Frame* getEnd() const;
    #endif
    };

    %template (JointDevicePtr) rw::core::Ptr<JointDevice>;
    %template (JointDeviceCPtr) rw::core::Ptr<const JointDevice>;
    %template (JointDevicePtrVector) std::vector< rw::core::Ptr< JointDevice>>;
    OWNEDPTR(JointDevice)


// ################# CompositeDevice

    /**
       @brief A device constructed from a sequence of devices.

       The configuration of a composite device is equal to the concatenation of
       the configurations of the sequence of devices.

       The devices that make up the CompositeDevice may not share joints, but
       the implementation does not check if this is actually the case.

       A composite device implements its operations of Device by querying each
       Joint in the straight-forward way of JointDevice. The notable
       exception is Device::setQ() which is implemented by forwarding the
       Device::setQ() calls to the sequence of devices. This means that
       CompositeDevice works also for example for devices of type ParallelDevice
       that have an overwritten implementation of Device::setQ().

       The devices from which the composite device is constructed must all be of
       type JointDevice. An exception is thrown by the constructor if one of
       the devices is not of this subtype.

       The computation of Jacobians of CompositeDevice is not correct in
       general, but is correct only for devices for which the standard technique
       of JointDevice is correct. We cannot in general in RobWork do any
       better currently. The implementation does not check if the requirements
       for the computation of Jacobians are indeed satisfied.

       CompositeDevice is related to TreeDevice in the sense that
       CompositeDevice has also multiple end-effectors (one end-effector for
       each device). CompositeDevice differs from TreeDevice by not requiring
       that the child-to-parent paths of the end-effectors connect to a common
       base.
     */
    class CompositeDevice: public JointDevice
    {
    public:
        /**
           @brief Constructor
           @param base [in] the base of the device
           @param devices [in] the sequence of subdevices
           @param end [in] the end (or tool) of the device
           @param name [in] the name of the device
           @param state [in] the kinematic structure assumed for Jacobian computations
        */
        CompositeDevice (rw::kinematics::Frame* base, const std::vector< rw::core::Ptr < Device > >& devices,
                         rw::kinematics::Frame* end, const std::string& name,
                         const rw::kinematics::State& state);

        /**
           @brief Constructor
           @param base [in] the base of the device
           @param devices [in] the sequence of subdevices
           @param ends [in] the end frames (or tools) of the device
           @param name [in] the name of the device
           @param state [in] the kinematic structure assumed for Jacobian computations
        */
        CompositeDevice (rw::kinematics::Frame* base, const std::vector< rw::core::Ptr<Device> >& devices,
                         const std::vector< rw::kinematics::Frame* >& ends, const std::string& name,
                         const rw::kinematics::State& state);

        /**
           @copydoc Device::setQ

           The method is implemented via forwarding to the Device::setQ()
           methods of the subdevices.
        */
        void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;

        // Methods specific to CompositeDevice follow here.

        /**
           @brief like Device::baseJend() but with a Jacobian calculated for all
           end-effectors (see getEnds()).
        */
        rw::math::Jacobian baseJends (const rw::kinematics::State& state) const;

        /**
           @brief The end-effectors of the composite device.

           The end-effectors of the composite device are the end-effectors of
           the devices from which the composite device was constructed.

           This sequence of end-effectors may or may not include the default
           end-effector returned by getEnd().
        */
        const std::vector< rw::kinematics::Frame* >& getEnds () const { return _ends; }

    };

    %template (CompositeDevicePtr) rw::core::Ptr<CompositeDevice>;
    OWNEDPTR(CompositeDevice)

    %extend rw::core::Ptr<CompositeDevice> {
        rw::core::Ptr<Device> asDevicePtr() { return *$self; }
        rw::core::Ptr<const Device> asDeviceCPtr() { return *$self; }
        rw::core::Ptr<JointDevice> asJointDevicePtr() { return *$self; }
        rw::core::Ptr<const JointDevice> asJointDeviceCPtr() { return *$self; }
    }


// ################# CompositeJointDevice
        /**
       @brief A device constructed from a sequence of devices.

       The configuration of a composite device is equal to the concatenation of
       the configurations of the sequence of devices.

       The devices that make up the CompositeJointDevice may not share joints, but
       the implementation does not check if this is actually the case.

       A composite device implements its operations of Device by querying each
       Joint in the straight-forward way of JointDevice. The notable
       exception is Device::setQ() which is implemented by forwarding the
       Device::setQ() calls to the sequence of devices. This means that
       CompositeJointDevice works also for example for devices of type ParallelDevice
       that have an overwritten implementation of Device::setQ().

       The devices from which the composite device is constructed must all be of
       type JointDevice. An exception is thrown by the constructor if one of
       the devices is not of this subtype.

       The computation of Jacobians of CompositeJointDevice is not correct in
       general, but is correct only for devices for which the standard technique
       of JointDevice is correct. We cannot in general in RobWork do any
       better currently. The implementation does not check if the requirements
       for the computation of Jacobians are indeed satisfied.

       CompositeJointDevice is related to TreeDevice in the sense that
       CompositeJointDevice has also multiple end-effectors (one end-effector for
       each device). CompositeJointDevice differs from TreeDevice by not requiring
       that the child-to-parent paths of the end-effectors connect to a common
       base.
    */
    class CompositeJointDevice : public JointDevice
    {
      public:
        /**
           @brief Constructor

           @param base [in] the base of the device
           @param devices [in] the sequence of subdevices
           @param end [in] the end (or tool) of the device
           @param name [in] the name of the device
           @param state [in] the kinematic structure assumed for Jacobian computations
        */
        CompositeJointDevice (rw::kinematics::Frame* base,
                              const std::vector< rw::core::Ptr<Device> >& devices, rw::kinematics::Frame* end,
                              const std::string& name, const rw::kinematics::State& state);

        /**
           @brief Constructor

           @param base [in] the base of the device
           @param devices [in] the sequence of subdevices
           @param ends [in] the end frames (or tools) of the device
           @param name [in] the name of the device
           @param state [in] the kinematic structure assumed for Jacobian computations
        */
        CompositeJointDevice (rw::kinematics::Frame* base,
                              const std::vector< rw::core::Ptr<Device> >& devices,
                              const std::vector< rw::kinematics::Frame* >& ends,
                              const std::string& name, const rw::kinematics::State& state);

        //! @brief destructor
        virtual ~CompositeJointDevice () {}

        /**
           @copydoc Device::setQ

           The method is implemented via forwarding to the Device::setQ()
           methods of the subdevices.
        */
        void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;

        // Methods specific to CompositeJointDevice follow here.

        /**
           @brief like Device::baseJend() but with a Jacobian calculated for all
           end-effectors (see getEnds()).
        */
        rw::math::Jacobian baseJends (const rw::kinematics::State& state) const;

        /**
           @brief The end-effectors of the composite device.

           The end-effectors of the composite device are the end-effectors of
           the devices from which the composite device was constructed.

           This sequence of end-effectors may or may not include the default
           end-effector returned by getEnd().
        */
        const std::vector< rw::kinematics::Frame* >& getEnds () const;
    };
// ################# DependentJoint
    /**
     * @brief Dependent joints are 0-dof joints for which the actual joints transformation depends
     * on one of more other joints.
     *
     * DependentJoint is an abstract class from which all dependent joints should inherit.
     */
    class DependentJoint : public Joint
    {
      public:
        /**
         * @brief Destructor
         */
        virtual ~DependentJoint ();

        /**
         * @brief Returns true if the DependentJoint is controlled by \b joint.
         *
         * A DependentJoint may depend on more than one joints.
         *
         * @param joint [in] Joints to test with
         * @return True if this is controlled by \b joint
         *
         */
        virtual bool isControlledBy (const Joint* joint) const = 0;

      protected:
        /**
         * @brief Constructs DependentJoint
         * @param name [in] Name of the joints
         */
        DependentJoint (const std::string& name);
    };

// ################# DependentPrismaticJoint
    /**
     * @brief Dependent prismatic joint.
     *
     * DependentPrismaticJoint implements a prismatic joint for which the displacement
     * along the z-axis are linearly dependent on another joint
     */
    class DependentPrismaticJoint : public DependentJoint
    {
      public:
        /**
         * @brief A revolute joint with a displacement transform of \b transform.
         *
         * @param name [in] The name of the frame.
         *
         * @param transform [in] The displacement transform of the joint.
         *
         * @param owner [in] The joint controlling the dependent joint.
         *
         * @param scale [in] Scaling factor for the controlling joint value.
         *
         * @param offset [in] Offset for the controlling joint value.
         */
        DependentPrismaticJoint (const std::string& name, const rw::math::Transform3D<double>& transform,
                                 Joint* owner, double scale, double offset);

        /**
         * @brief The parent to frame transform for a revolute joint.
         *
         * The parent to frame transform is T * Tz(q) where:
         *
         * - T is the displacement transform of the joint;
         *
         * - q = q_owner * scale + offset is the joint value of the joint;
         *
         * - Tz(q) is the transform that translates a point an distance q in the
         * direction of the z-axis.
         *
         * @copydoc kinematics::Frame::getTransform
         */
        rw::math::Transform3D<double> getTransform (const rw::kinematics::State& state) const;

        #if !defined(SWIGJAVA)
        /**
           @brief The joint controlling the passive revolute frame.
        */
        const Joint& getOwner () const;
        #endif
        /**
           @brief The joint controlling the passive revolute frame.
        */
        Joint& getOwner ();

        /**
           @brief The scaling factor for the joint value of the controlling joint.
         */
        double getScale () const;

        /**
         * @brief get offset of this joint value in relation to controlling joint
         */
        double getOffset () const;

        //! @copydoc DependentJoint::isControlledBy
        bool isControlledBy (const Joint* joint) const;

        //! @copydoc Joint::getJacobian
        void getJacobian (size_t row, size_t col, const rw::math::Transform3D<double>& joint,
                          const rw::math::Transform3D<double>& tcp, const rw::kinematics::State& state,
                          rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<double> getFixedTransform () const;

        //! @copydoc Joint::setFixedTransform()
        void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform()
        rw::math::Transform3D<double> getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc Joint::setJointMapping()
        virtual void setJointMapping (rw::core::Ptr<rw::math::Function1Diff<>> function);

        //! @copydoc Joint::removeJointMapping()
        virtual void removeJointMapping () {}
    };


// ################# DependentRevoluteJoint
    /**
     * @brief Dependent revolute joints.
     *
     * DependentRevoluteJoint implements a revolute joint for which the rotation about the
     * z-axis are linearly dependent on another joint.
     *
     *  The parent to frame transform is T * Rz(q) where:
     *
     * - T is the displacement transform of the joint;
     *
     * - q = q_owner * scale + offset is the joint value of the joint;
     *
     * - Rz(q) is the transform that rotates a point an angle q about the
     * z-axis.
     */
    class DependentRevoluteJoint : public DependentJoint
    {
      public:
        /**
         * @brief A revolute joint with a displacement transform of \b transform.
         *
         * @param name [in] The name of the frame.
         *
         * @param transform [in] The displacement transform of the joint.
         *
         * @param owner [in] The joint controlling the passive joint.
         *
         * @param scale [in] Scaling factor for the controlling joint value.
         *
         * @param offset [in] Offset for the controlling joint value.
         */
        DependentRevoluteJoint (const std::string& name, const rw::math::Transform3D<double>& transform,
                                Joint* owner, double scale, double offset);

        #if !defined(SWIGJAVA)
        /**
           @brief The joint controlling the passive revolute frame.
        */
        const Joint& getOwner () const;
        #endif 

        /**
           @brief The joint controlling the passive revolute frame.
        */
        Joint& getOwner ();

        /**
           @brief The scaling factor for the joint value of the controlling joint.
         */
        double getScale () const;

        /**
         * @brief get offset of this joint value in relation to controlling joint
         */
        double getOffset () const;

        /**
         * @copydoc DependentJoint::isControlledBy
         */
        bool isControlledBy (const Joint* joint) const;

        /**
         * @brief calculate the current q of this joint
         * @param state
         * @return
         */
        double calcQ (const rw::kinematics::State& state);

        //! @copydoc Joint::getJacobian
        void getJacobian (size_t row, size_t col, const rw::math::Transform3D<double>& joint,
                          const rw::math::Transform3D<double>& tcp, const rw::kinematics::State& state,
                          rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<double> getFixedTransform ();

        //! @copydoc Joint::setFixedTransform()
        void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform()
        rw::math::Transform3D<double> getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc Joint::setJointMapping()
        virtual void setJointMapping (rw::core::Ptr<rw::math::Function1Diff<>> function);

        //! @copydoc Joint::removeJointMapping()
        virtual void removeJointMapping ();
    };

// ################# JointDeviceJacobianCalculator
    /**
     * @brief Calculator for Jacobians of a JointDevice
     *
     * Implements Jacobian calculations for a JointDevice. Users should generally not construct a
     * JointDeviceJacobianCalculator themselves by obtain one directly from a JointDevice.
     *
     * If more than one end-effector is given a "stacked" Jacobian is returned.
     *
     */
    class JointDeviceJacobianCalculator : public JacobianCalculator
    {
      public:
        /**
         * @brief Constructs JacobianCalculator.
         *
         * The dimension of the jacobian wil be (tcps.size() * 6, device.getDOF()).
         *
         * @param device [in] The device to calculate for
         * @param base [in] Reference base of the Jacobian. Does not have to be the same as the base
         * of the device
         * @param tcps [in] List of tool end-effectors for which to calculate the Jacobian.
         * @param state [in] State giving how frame are connected
         */
        JointDeviceJacobianCalculator (rw::core::Ptr< JointDevice > device,
                                       const rw::kinematics::Frame* base,
                                       const std::vector< rw::kinematics::Frame* >& tcps,
                                       const rw::kinematics::State& state);

        /**
         * @brief Destructor
         */
        virtual ~JointDeviceJacobianCalculator ();

        /**
         * @copydoc JacobianCalculator::get(const rw::kinematics::State& state) const
         */
        virtual rw::math::Jacobian get (const rw::kinematics::State& state) const;
    };

// ################# MobileDevice

    /**
     * @brief Provides a differential controlled mobile device
     *
     * The MobileDevice class provides a differential controlled mobile device
     * with non-holonomic constraints. The \f$ x\f$ direction is defined as
     * straight forward and \f$ z\f$ vertically up. The wheels are assumed to be
     * positioned summetrically around the base and have \f$ 0\f$ \f$ x\f$
     * component.
     *
     * When using setQ it takes 2 parameters, which corresponds to the distances
     * travelled by the wheels. Based on this input and the current pose of the
     * device it calcualtes a new pose as.
     */
    class MobileDevice : public Device
    {
      public:
        /**
         * @brief Constructs a mobile device
         * @param base [in] the base of the device
         * @param wheel1 [in] the left wheel
         * @param wheel2 [in] the right wheel
         * @param state [in] the state of the device
         * @param name [in] name of device
         */
        MobileDevice (rw::kinematics::MovableFrame* base, RevoluteJoint* wheel1,
                      RevoluteJoint* wheel2, rw::kinematics::State& state, const std::string& name);

        /**
         * @brief Destructor
         */
        virtual ~MobileDevice ();

        /**
         * @brief Sets the position and orientation of the base
         *
         * This operation moves the base of the robot, without considering
         * the non-holonomic constraints of the device
         * @param transform [in] new base transform
         * @param state [in] state to write change to
         */
        void setDevicePose (const rw::math::Transform3D<double>& transform, rw::kinematics::State& state);

        /**
         * @copydoc Device::setQ
         */
        virtual void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;

        /**
         * @copydoc Device::getQ
         */
        virtual rw::math::Q getQ (const rw::kinematics::State& state) const;

        /**
         * @copydoc Device::getBounds
         */
        virtual std::pair< rw::math::Q, rw::math::Q > getBounds () const;

        /**
         * @copydoc Device::setBounds
         */
        virtual void setBounds (const std::pair< rw::math::Q, rw::math::Q >& bounds);

        /**
         * @copydoc Device::getVelocityLimits
         */
        virtual rw::math::Q getVelocityLimits () const;

        /**
         * @copydoc Device::setVelocityLimits
         */
        virtual void setVelocityLimits (const rw::math::Q& vellimits);

        /**
         * @copydoc Device::getAccelerationLimits
         */
        virtual rw::math::Q getAccelerationLimits () const;

        /**
         * @copydoc Device::setAccelerationLimits
         */
        virtual void setAccelerationLimits (const rw::math::Q& acclimits);

        /**
         * @copydoc Device::getDOF
         */
        virtual size_t getDOF () const;

        /**
         * @copydoc Device::getBase()
         */
        virtual rw::kinematics::Frame* getBase ();

        #if !defined(SWIGJAVA)
            /**
             * @copydoc Device::getBase() const
             */
            virtual const rw::kinematics::Frame* getBase () const;
        #endif 

        /**
         * @copydoc Device::getEnd()
         */
        virtual rw::kinematics::Frame* getEnd ();

        #if !defined(SWIGJAVA)
            /**
             * @copydoc Device::getEnd() const
             */
            virtual const rw::kinematics::Frame* getEnd () const;
        #endif

        /**
         * @copydoc Device::baseJend
         */
        virtual rw::math::Jacobian baseJend (const rw::kinematics::State& state) const;

        /**
           @copydoc Device::baseJframe
           Not implemented.
        */
        virtual rw::math::Jacobian baseJframe (const rw::kinematics::Frame* frame,
                                           const rw::kinematics::State& state) const;

        /**
           @copydoc Device::baseJframes
           Not implemented.
        */
        virtual rw::math::Jacobian baseJframes (const std::vector< rw::kinematics::Frame* >& frames,
                                            const rw::kinematics::State& state) const;

        /**
           @copydoc Device::baseJCframes
           Not implemented.
        */
        virtual rw::core::Ptr<JacobianCalculator>
        baseJCframes (const std::vector< rw::kinematics::Frame* >& frames,
                      const rw::kinematics::State& state) const;
    };


// ################# Models
    /**
       @brief Utility functions for the rw::models module.
    */
    class Models
    {
      public:
        // Frames and workcells.

        /**
           @brief All frames of the workcell.
        */
        static std::vector< rw::kinematics::Frame* > findAllFrames (const WorkCell& workcell);

        /**
           @brief The frame named \b name of workcell \b workcell.

           An exception is thrown if the frame can not be found in the workcell.

           See WorkCell::findFrame() for a non-throwing version.
        */
        static rw::kinematics::Frame& getFrame (const WorkCell& workcell, const std::string& name);

        /**
           @brief The device named \b name of workcell \b workcell.

           An exception is thrown if the device can not be found in the workcell.

           See WorkCell::findDevice() for a non-throwing version.
        */
        static rw::core::Ptr<Device> getDevice (const WorkCell& workcell, const std::string& name);

        // Bounds checking

        /**
           @brief True iff the configuration \b q is within the box with lower and
           upper corners given by \b bounds. Each value of \b q is allowed to be
           outside of the box by the amount \b tolerance.
        */
        static bool inBounds (const rw::math::Q& q, const std::pair<rw::math::Q,rw::math::Q>& bounds,
                              double tolerance = 0);

        /**
           @brief True iff the configuration \b q is within the joint limits of the
           device \b device.
        */
        static bool inBounds (const rw::math::Q& q, const Device& device, double tolerance = 0);

        /**
           @brief True iff the joint value \b val is within the joint limits of the
           joint \b joint with a tolerance of \b tolerance.
        */
        static bool inBounds (const rw::math::Q& val, const Joint& joint, double tolerance = 0);

        /**
           @brief True iff the joint values of \b state are within the joint limits
           of the joints of \b workcell with a tolerance of \b tolerance.
        */
        static bool inBounds (const rw::kinematics::State& state, const WorkCell& workcell,
                              double tolerance = 0);

        // Q path to state path conversion.

        /**
           @brief Convert a sequence of configurations to a sequence of states.

           The device configurations are assumed to belong to a common device
           and state.

           @param device [in] The device for the configurations.
           @param path [in] The sequence of device configurations.
           @param common_state [in] State to share for all configurations.
           @return Sequence of states - one state for each configuration.
        */
        static rw::trajectory::Path<rw::kinematics::State> getStatePath (const Device& device,
                                                       const rw::trajectory::Path< rw::math::Q >& path,
                                                       const rw::kinematics::State& common_state);

        /**
           @brief Convert a sequence of configurations to a sequence of states.

           The device configurations are assumed to belong to a common device
           and state.

           @param device [in] The device for the configurations.
           @param path [in] The sequence of device configurations.
           @param common_state [in] State to share for all configurations.
           @param result [out] Sequence of states - one state for each configuration.
        */
        static void getStatePath (const Device& device, const rw::trajectory::Path< rw::math::Q >& path,
                                  const rw::kinematics::State& common_state,
                                  rw::trajectory::Path<rw::kinematics::State>& result);

        /**
           @brief Construct a new device for which the base of the device equals
           \b base and the end of the device equals \b end.

           For changes in the configuration of \b device, \b base should be
           fixed relative to device->getBase() and \b end should be fixed
           relative to device->getEnd().

           If \b base is NULL, then device->getBase() is used as the default
           value.

           If \b end is NULL, then device->getEnd() is used as the default
           value.

           If \b base and \b end equal base and end for the device, then the
           original \b device is returned.

           @param device [in] Original device.

           @param state [in] The kinematic structure assumed for Jacobian
           computations.

           @param base [in] Base frame for the new device.

           @param end [in] End frame for the new device.
        */
        static rw::core::Ptr<Device> makeDevice (rw::core::Ptr<Device> device,
                                                   const rw::kinematics::State& state,
                                                   rw::kinematics::Frame* base = NULL,
                                                   rw::kinematics::Frame* end  = NULL);
    };

// ################# ParallelDevice

    /**
     * @brief This class defines the interface for Parallel devices.
     */
    class ParallelDevice: public JointDevice
    {
      public:
        /**
         * @brief Constructor
         *
         * @param legs [in] the serial legs connecting the endplate to the base.
         * The base of each serial Leg must be the same frame. Likewise, the endeffector
         * (last frame) of each Leg must transform to the same transform as each of the
         * other legs
         * @param name [in] name of device
         * @param state [in] the state for the assembly mode
         */
        ParallelDevice (const std::vector< ParallelLeg* > & legs, const std::string name, const rw::kinematics::State& state);

        /**
         * @brief Constructor for parallel device with multiple junctions.
         * @param name [in] name of the device.
         * @param base [in] the base frame.
         * @param end [in] the end frame.
         * @param joints [in] a list of joints. Each joint can be included in multiple legs.
         * @param state [in] the state used to construct a JointDevice.
         * @param junctions [in] a list of junctions.
         * Each junction is given by a list of legs that must begin and end in the same frame.
         */
        ParallelDevice (const std::string name, rw::kinematics::Frame* base,
                        rw::kinematics::Frame* end, const std::vector< Joint* >& joints,
                        const rw::kinematics::State& state, const std::vector< std::vector< ParallelLeg* >  >& junctions);

        /** @brief Destructor */
        ~ParallelDevice ();

        /**
         * @copydoc Device::setQ
         *
         * The configuration \b q is the configuration for the actuated joints
         * of the parallel device. Based on the value of \b q the setQ() method
         * automatically computes the values for the unactuated (passive)
         * joints.
         */
        virtual void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;

        /**
         * @brief Set only some of the actuated joints.
         *
         * This version of setQ will only set a subset of the actuated joints.
         * Based on the value of \n q, the function will compute the values for the
         * unactuated (passive) joints, and the remaining actuated joints.
         *
         * This is mainly useful for parallel devices that have more controlled joints
         * than strictly required to make the kinematics determined.
         *
         * @param q [in] the configuration of the actuated joints
         * (the only considered elements are the ones where the corresponding elements of \b enabled
         * is true).
         * @param enabled [in] vector of same size as \b q, specifying which values to solve for.
         * @param state [in/out] the state with all active and passive joint values.
         * The input state is expected to contain a valid and consistent configuration of the
         * device.
         */
        virtual void setQ (const rw::math::Q& q, const std::vector< bool >& enabled,
                           rw::kinematics::State& state) const;

        /** @copydoc Device::baseJframe */
        rw::math::Jacobian baseJframe (const rw::kinematics::Frame* frame,
                                   const rw::kinematics::State& state) const;

        /** @copydoc Device::baseJend */
        rw::math::Jacobian baseJend (const rw::kinematics::State& state) const;

        /**
         * @brief The legs of the parallel device.
         */
        virtual std::vector< ParallelLeg* > getLegs () const;

        /**
         * @brief Get the junctions of the device.
         * @return a vector of junctions. Each junction is given by a two or more legs.
         */
        virtual std::vector< std::vector< ParallelLeg* >  > getJunctions () const;

        /**
         * @brief The active joints of the parallel device.
         */
        virtual std::vector< Joint* > getActiveJoints () const;

        /**
         * @brief Get all joints (both active and passive).
         * @return a vector of all the joints.
         */
        virtual std::vector< Joint* > getAllJoints () const;

        /**
         * @brief Get the total degrees of freedom for all (active and passive) joints in the
         * device.
         * @return the total degrees of freedom.
         */
        std::size_t getFullDOF () const;

        /**
         * @brief Get bounds for all joints (includes both active and passive joints).
         * @return a pair with the lower and upper limits.
         */
        std::pair< rw::math::Q, rw::math::Q > getAllBounds () const;

        /**
         * @brief Get the full configuration vector of the device. This gives the complete state of
         * the parallel device.
         * @param state [in] the state that contains the full configuration.
         * @return the configuration vector with the joint values for both active and passive
         * joints.
         */
        rw::math::Q getFullQ (const rw::kinematics::State& state) const;

        /**
         * @brief Set the full configuration of the device.
         * This sets the joint values directly, and there is no checks or guarantees that the device
         * will be in a valid connected configuration afterwards.
         * @param q [in] the configuration vector to set.
         * @param state [in/out] the state to update with a new configuration.
         */
        void setFullQ (const rw::math::Q& q, rw::kinematics::State& state) const;

        
    };
    %template (ParallelDevicePtr) rw::core::Ptr<ParallelDevice>;
    %template (ParallelDevicePtrVector) std::vector<rw::core::Ptr<ParallelDevice>>;
    OWNEDPTR(ParallelDevice);


// ################# ParallelLeg

     /**
     * @brief Class representing a single leg in a ParallelDevice
     */
    class ParallelLeg
    {
      public:

        /**
         * @brief Constructs leg from frames
         * @param frames [in] list of Frame's
         */
        ParallelLeg (std::vector< rw::kinematics::Frame* > frames);

        /**
         * @brief Destructor
         */
        virtual ~ParallelLeg ();

        /**
         * @brief Returns the base to end Jacobian
         * @param state [in] State for which to calculate the Jacobian
         * @return the Jacobian
         */
        const rw::math::Jacobian& baseJend (const rw::kinematics::State& state);

        /**
         * @brief Returns the Jacobian of \b frame relative to base frame.
         * @param frame [in] the frame to find Jacobian for.
         * @param state [in] State for which to calculate the Jacobian
         * @return the Jacobian
         */
        rw::math::Jacobian baseJframe (const rw::kinematics::Frame* frame,
                                       const rw::kinematics::State& state) const;

        /**
         * @brief Returns the base to end transformation
         * @param state [in] State for which to calculate the transform
         * @return the transform
         */
        rw::math::Transform3D< double > baseTend (const rw::kinematics::State& state) const;

        /**
         * @brief Returns the transformation of a \b frame relative to the base.
         * @param frame [in] the frame to find transformation to.
         * @param state [in] State for which to calculate the transform
         * @return the transform
         */
        rw::math::Transform3D< double > baseTframe (const rw::kinematics::Frame* frame,
                                                    const rw::kinematics::State& state) const;

        /**
         * @brief Returns the kinematic chain of the leg
         * @return list of frames
         */
        const std::vector< rw::kinematics::Frame* >& getKinematicChain () const;

        /**
         * @brief the base of the leg
         * @return the frame
         */
        rw::kinematics::Frame* getBase ();

        /**
         * @brief the end of the leg
         * @return the frame
         */
        rw::kinematics::Frame* getEnd ();

        /**
         * @brief Number of active joints
         * @return number of active joints
         */
        size_t nrOfActiveJoints ();

        /**
         * @brief Number of passive joints
         * @return number of passive joints
         */
        size_t nrOfPassiveJoints ();

        /**
         * @brief Number of joints (both active and passive)
         * @return number of joints
         */
        size_t nrOfJoints ();

        /**
         * @brief Returns list of the actuated (active) joints
         * @return list of joints
         */
        const std::vector< Joint* >& getActuatedJoints ();

        /**
         * @brief Returns list of unactuated (passive) joints
         * @return list of joints
         */
        const std::vector< Joint* >& getUnactuatedJoints ();

        /**
         * @brief Get the total degrees of freedom (includes both active and passive joints).
         * @return the total degrees of freedom.
         */
        size_t getJointDOFs () const;

        /**
         * @brief Get configuration of the leg.
         * @param state [in] the state with the configuration values.
         * @return the configuration.
         */
        rw::math::Q getQ (const rw::kinematics::State& state) const;

        /**
         * @brief Sets q for the leg in the state
         * @param q [in] q to set
         * @param state [out] the State to modify
         */
        void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;
    };
    %template(ParallelLegPtr) rw::core::Ptr<ParallelLeg>;
    %template(ParallelLegPtrVector) std::vector<rw::core::Ptr<ParallelLeg>>;
    %template(ParallelLegPointerVector) std::vector<ParallelLeg*>;
    %template(ParallelLegPointerVectorVector) std::vector<std::vector<ParallelLeg*>>;

// ################# PrismaticJoint
    /**
     * @brief Prismatic joints.
     *
     * PrismaticJoint implements a prismatic joint for the displacement in the
     * direction of the z-axis of an arbitrary displacement transform.
     */
    class PrismaticJoint : public Joint
    {
      public:
        /**
         * @brief Constructs PrismaticJoint
         *
         * @param name [in] Name of the joints
         * @param transform [in] Static transform of the joint
         */
        PrismaticJoint (const std::string& name, const rw::math::Transform3D<double>& transform);

        //! destructor
        virtual ~PrismaticJoint ();

        /**
         * @brief Post-multiply the transform of the joint to the parent transform.
         *
         * The transform is calculated for the joint values of \b q.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         * @param parent [in] The world transform of the parent frame.
         * @param q [in] Joint values for the joint
         * @param result [in] The transform of the frame in the world frame.
         */
        void multiplyJointTransform (const rw::math::Transform3D<double>& parent, const rw::math::Q& q,
                                     rw::math::Transform3D<double>& result) const;

        /**
         * @brief The transform of the joint relative to its parent.
         *
         * The transform is calculated for the joint values of \b state.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         *
         * @param q [in] Joint values for the joint
         *
         * @return The transform of the frame relative to its displacement transform.
         */
        rw::math::Transform3D<double> getJointTransform (double q) const;

        /**
         * @brief The transform of the joint relative to its parent.
         *
         * The transform is calculated for the joint values of \b state.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         *
         * @param q [in] Joint values for the joint
         *
         * @return The transform of the frame relative to its parent transform.
         */
        rw::math::Transform3D<double> getTransform (double q) const;
        // we need to declare the getTransform again because its shadowed by the getTransform(q)
        using rw::kinematics::Frame::getTransform;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<double> getFixedTransform () const;

        //! @copydoc Joint::setFixedTransform()
        void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform()
        rw::math::Transform3D<double> getJointTransform (const rw::kinematics::State& state) const;

        /**
         * @copydoc Joint::getJacobian()
         */
        void getJacobian (size_t row, size_t col, const rw::math::Transform3D<double>& joint,
                          const rw::math::Transform3D<double>& tcp, const rw::kinematics::State& state,
                          rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::setJointMapping()
        virtual void setJointMapping (rw::core::Ptr<rw::math::Function1Diff<>> function);

        //! @copydoc Joint::removeJointMapping()
        virtual void removeJointMapping ();

      protected:
        /**
         * @copydoc rw::kinematics::Frame::doMultiplyTransform
         */
        void doMultiplyTransform (const rw::math::Transform3D<double>& parent, const rw::kinematics::State& state,
                                  rw::math::Transform3D<double>& result) const;

        /**
         * @copydoc rw::kinematics::Frame::doGetTransform
         */
        rw::math::Transform3D<double> doGetTransform (const rw::kinematics::State& state) const;
    };
    %template(PrismaticJointPtr) rw::core::Ptr<PrismaticJoint>;

// ################# PrismaticSphericalJoint
    /**
     * @brief A prismatic spherical joint that allows rotations in all directions and translation
     * along one direction.
     *
     * Rotation is allowed around all axes. The xy-position is fixed, while the z-axis is
     * translational.
     *
     * This joint is equivalent to a spherical joint followed by a translational joint.
     */
    class PrismaticSphericalJoint : public Joint
    {
      public:

        /**
         * @brief Construct a prismatic spherical joint.
         * @param name [in] name of the joint.
         * @param transform [in] static transform of the joint.
         */
        PrismaticSphericalJoint (const std::string& name, const rw::math::Transform3D<double>& transform);

        //! @brief Destructor.
        virtual ~PrismaticSphericalJoint ();

        // From Frame
        //! @brief Frame::doMultiplyTransform
        virtual void doMultiplyTransform (const rw::math::Transform3D<double>& parent,
                                          const rw::kinematics::State& state,
                                          rw::math::Transform3D<double>& result) const;

        //! @brief Frame::doGetTransform
        virtual rw::math::Transform3D<double> doGetTransform (const rw::kinematics::State& state) const;

        // From Joint
        //! @copydoc Joint::getJacobian
        virtual void getJacobian (std::size_t row, std::size_t col,
                                  const rw::math::Transform3D<double>& joint,
                                  const rw::math::Transform3D<double>& tcp,
                                  const rw::kinematics::State& state,
                                  rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::getFixedTransform
        virtual rw::math::Transform3D<double> getFixedTransform () const;

        //! @copydoc Joint::setFixedTransform
        virtual void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform
        virtual rw::math::Transform3D<double>
        getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc Joint::setJointMapping
        virtual void setJointMapping (rw::core::Ptr <rw::math::Function1Diff<>> function);

        //! @copydoc Joint::removeJointMapping
        virtual void removeJointMapping ();

    };
    %template(PrismaticSphericalJointPtr) rw::core::Ptr<PrismaticSphericalJoint>;

// ################# PrismaticUniversalJoint
    /**
     * @brief A prismatic universal joint that allows rotations in two directions and translation
     * along the third.
     *
     * Rotation is allowed around the x and y axes. The xy-position is fixed, while the z-axis is
     * translational.
     *
     * This joint is equivalent to a universal joint followed by a translational joint.
     */
    class PrismaticUniversalJoint : public Joint
    {
      public:

        /**
         * @brief Construct a prismatic universal joint.
         * @param name [in] name of the joint.
         * @param transform [in] static transform of the joint.
         */
        PrismaticUniversalJoint (const std::string& name, const rw::math::Transform3D<double>& transform);

        //! @brief Destructor.
        virtual ~PrismaticUniversalJoint ();

        // From Frame
        //! @brief Frame::doMultiplyTransform
        virtual void doMultiplyTransform (const rw::math::Transform3D<double>& parent,
                                          const rw::kinematics::State& state,
                                          rw::math::Transform3D<double>& result) const;

        //! @brief Frame::doGetTransform
        virtual rw::math::Transform3D<double> doGetTransform (const rw::kinematics::State& state) const;

        // From Joint
        //! @copydoc Joint::getJacobian
        virtual void getJacobian (std::size_t row, std::size_t col,
                                  const rw::math::Transform3D<double>& joint,
                                  const rw::math::Transform3D<double>& tcp,
                                  const rw::kinematics::State& state,
                                  rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::getFixedTransform
        virtual rw::math::Transform3D<double> getFixedTransform () const;

        //! @copydoc Joint::setFixedTransform
        virtual void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform
        virtual rw::math::Transform3D<double>
        getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc Joint::setJointMapping
        virtual void setJointMapping (rw::core::Ptr<rw::math::Function1Diff<>> function);

        //! @copydoc Joint::removeJointMapping
        virtual void removeJointMapping ();
    };
    %template(PrismaticUniversalJointPtr) rw::core::Ptr<PrismaticUniversalJoint>;
// ################# RevoluteJoint

        class RevoluteJoint : public Joint
    {
      public:
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr< RevoluteJoint > Ptr;

        /**
         * @brief Constructs RevoluteJoint
         *
         * @param name [in] Name of the joints
         * @param transform [in] Static transform of the joint
         */
        RevoluteJoint (const std::string& name, const rw::math::Transform3D<double>& transform);

        //! @brief destructor
        virtual ~RevoluteJoint ();

        /**
         * @brief Post-multiply the transform of the joint to the parent transform.
         *
         * The transform is calculated for the joint values of \b q.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         * @param parent [in] The world transform of the parent frame.
         * @param q [in] Joint values for the joint
         * @param result [in] The transform of the frame in the world frame.
         */
        void multiplyJointTransform (const rw::math::Transform3D<double>& parent, const rw::math::Q& q,
                                     rw::math::Transform3D<double>& result) const;

        /**
         * @brief The transform of the joint relative to its parent.
         *
         * The transform is calculated for the joint values of \b state.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         *
         * @param q [in] Joint values for the joint
         *
         * @return The transform of the frame relative to its displacement transform.
         */
        rw::math::Transform3D<double> getJointTransform (double q) const;

        /**
         * @brief The transform of the joint relative to its parent.
         *
         * The transform is calculated for the joint values of \b state.
         *
         * This method is equivalent to Frame::multiplyTransform except that is operates
         * directly on a joint vector instead of a State.
         *
         *
         * @param q [in] Joint values for the joint
         *
         * @return The transform of the frame relative to its parent frame.
         */
        rw::math::Transform3D<double> getTransform (double q) const;
        // we need to declare the getTransform again because its shadowed by the getTransform(q)
        using rw::kinematics::Frame::getTransform;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<double> getFixedTransform () const;

        //! @copydoc Joint::getJacobian
        void getJacobian (size_t row, size_t col, const rw::math::Transform3D<double>& joint,
                          const rw::math::Transform3D<double>& tcp, const rw::kinematics::State& state,
                          rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::setFixedTransform()
        void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform()
        rw::math::Transform3D<double> getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc Joint::setJointMapping()
        virtual void setJointMapping (rw::core::Ptr< rw::math::Function1Diff<> > function);

        //! @copydoc Joint::removeJointMapping()
        virtual void removeJointMapping ();

      protected:
        //! @copydoc rw::kinematics::Frame::doMultiplyTransform
        void doMultiplyTransform (const rw::math::Transform3D<double>& parent, const kinematics::State& state,
                                  rw::math::Transform3D<double>& result) const;

        //! @copydoc rw::kinematics::Frame::doGetTransform
        rw::math::Transform3D<double> doGetTransform (const kinematics::State& state) const;
    };
    %template(RevoluteJointPtr) rw::core::Ptr<RevoluteJoint>;

// ################# RigidBodyInfo
    
    /**
     * @brief A class to wrap rigid body information.
     */
    class RigidBodyInfo
    {
      public:
        /**
         * @brief constructs a RigidBodyInfo with a mass, inertia matrix, initial
         * pose and velocity.
         */
        RigidBodyInfo (double mass, const rw::math::InertiaMatrix<double>& Ibody);

        /**
         * @brief destructor
         */
        virtual ~RigidBodyInfo ();

        /**
         * @brief returns the mass of this RigidBodyInfo
         * @return the mass
         */
        double getMass ();

        /**
         * @brief returns the inertia matrix of this rigid body
         */
        rw::math::InertiaMatrix<double> getInertia ();
    };

// ################# RigidObject

    /**
     * @brief the RigidObject defines a physical object in the workcell that is rigid in the sence that
     * the geometry does not change. The rigid object also have basic properties such as Inertia and mass.
     * These are default 1.0 kg and inertia of solid sphere with mass 1.0kg and radius of 10cm. The center
     * of mass defaults to origin of the base frame.
     */

    class RigidObject : public Object {
    public:
        /**
         * @brief constructor
         * @param baseFrame [in] base frame of the object
         */
        RigidObject(rw::kinematics::Frame* baseframe);
        
        /**
         * @brief constructor
         * @param baseFrame [in] base frame of the object
         * @param geom [in] the Geometry Forming the object
         */
        RigidObject(rw::kinematics::Frame* baseframe, rw::core::Ptr< Geometry > geom);
        
        /**
         * @brief constructor
         * @param baseFrame [in] base frame of the object
         * @param geom [in] a list of geometries to form the object
         */
        RigidObject(rw::kinematics::Frame* baseframe, std::vector<rw::core::Ptr< Geometry >> geom);
        
        /**
         * @brief constructor
         * @param frames [in] first frame is base frame of the object
         */
        RigidObject(std::vector<rw::kinematics::Frame*> frames);
        
        /**
         * @brief constructor
         * @param frames [in] first frame is base frame of the object
         * @param geom [in] the Geometry Forming the object
         */
        RigidObject(std::vector<rw::kinematics::Frame*> frames, rw::core::Ptr< Geometry > geom);

        /**
         * @brief constructor
         * @param frames [in] first frame is base frame of the object
         * @param geom [in] a list of geometries to form the object
         */
        RigidObject(std::vector<rw::kinematics::Frame*> frames, std::vector<rw::core::Ptr< Geometry >> geom);

        /**
         * @brief add collision geometry from this object
         * @param geom the geometry to add
         */
        void addGeometry(rw::core::Ptr<Geometry> geom);

        /**
         *  @brief remove collision geometry from this object
         *  @param geom [in] the geometry to remove
         */
        void removeGeometry(rw::core::Ptr<Geometry> geom);

        /**
         * @brief add visualization model to this object
         * @param model [in] the model to be added
         */
        void addModel(rw::core::Ptr<Model3D> model);

        /**
         * @brief remove visualization model to this rigid object
         * @param model [in] the model to be removed
         */
        void removeModel(rw::core::Ptr<Model3D> model);

        /**
         * @brief returns the mass of this RigidObject
         * @return mass of the Object
         */
        double getMass() const;

        /**
         * @brief set mass of this RigidObject
         * @param mass [in] the mass of this object
         */
        void setMass(double mass);

        /** 
         * @brief get the inertia matrix of this rigid body seen in the base frame
         * @return IntertiaMatrix
         */
        rw::math::InertiaMatrix<double> getInertia() const;

        /**
         *  @brief set inertia of this rigid object
         * @param initia [in] the inertia of this object
         */
        void setInertia(const rw::math::InertiaMatrix<double>& inertia);

        /**
         * @brief set the center of mass of this rigid body seen in the base frame
         */
        void setCOM(const rw::math::Vector3D<double>& com);

        /**
         * @brief approximates inertia based on geometry, mass and center of mass properties
         */
        void approximateInertia();

        /** 
         * @brief approximates inertia and center of mass based on geometry and mass properties
	       */
        void approximateInertiaCOM();

        /**
         * @brief get geometry of this rigid object
         * @return a list of all Geometries
         */
        const std::vector<rw::core::Ptr<Geometry> >& getGeometry() const;

        /**
         * @brief get visualization models for this rigid object
         * @return a list of all models
         */
        const std::vector<rw::core::Ptr<Model3D> >& getModels() const;

        //! @copydoc Object::getMass
        double getMass(rw::kinematics::State& state) const;

        //! @copydoc Object::getInertia
        rw::math::InertiaMatrix<double> getInertia(rw::kinematics::State& state) const;

        //! @copydoc Object::getCOM
        rw::math::Vector3D<double> getCOM(rw::kinematics::State& state) const;
    };

    %template (RigidObjectPtr) rw::core::Ptr<RigidObject>;
    %template (RigidObjectPtrVector) std::vector<rw::core::Ptr < RigidObject > >;
    OWNEDPTR(RigidObject);

// ################# SE3Device
     /**
     * @brief A Cartesian 6-Dof device
     *
     * The SE3Device is a 6-dof device with 6 independent inputs that
     * enables the device to place its end-effector anywhere in the workspace.
     *
     * The @f$ \mathbf{q}\in \mathbb{R}^6 @f$ input vector maps directly to the
     * end-effector pose @f$ \robabx{b}{e}{\mathbf{x}} @f$, thus:
     *
     * @f[ \robabx{b}{e}{\mathbf{x}} =
     * \left[
     * \begin{array}{c}
     * x\\
     * y\\
     * z\\
     * \theta k_x\\
     * \theta k_y\\
     * \theta k_z
     * \end{array}
     * \right] =
     * \left[
     * \begin{array}{c}
     * q_1\\
     * q_2\\
     * q_3\\
     * q_4\\
     * q_5\\
     * q_6
     * \end{array}
     * \right] =
     * \mathbf{q} @f]
     *
     * It is easily seen that the jacobian @f$
     * {^b_6}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) = \frac{\partial
     * ^b\mathbf{x}_6}{\partial \mathbf{q}} @f$ equals the @f$ 6\times 6 @f$
     * identity matrix @f$ \mathbf{I}^{6\times 6} @f$
     *
     * The device can be seen as a "perfect" robot, it has no singularities
     * anywhere in the task space, no kinematic or dynamic limits (it can
     * instantaneous move anywhere at any time). The device is interesting in
     * simulations where performance and stability of closed-loop control
     * systems (for instance visual-servoing systems) must be evaluated - if a
     * closed-loop control system does not perform well with a "perfect" robot
     * it will probably not perform well with a real robot either.
     */
    class SE3Device : public Device
    {
      public:
        /**
         * @brief Constructor
         *
         * @param name [in] device name
         * @param base documentation missing !
         * @param mframe documentation missing !
         */
        SE3Device (const std::string& name, rw::kinematics::Frame* base,
                   rw::kinematics::MovableFrame* mframe);

        virtual ~SE3Device () {}

        /**
         * @copydoc Device::setQ
         *
         * @pre q.size() == 6
         */
        void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;

        /**
         * @copydoc Device::getQ
         */
        rw::math::Q getQ (const rw::kinematics::State& state) const;

        /**
         * @copydoc Device::getBounds
         *
         * Since the SE3Device robot is unconstrained and can move anywhere
         * whithin the taskspace each of the 6 input's are unbounded (@f$
         * [-\inf, \inf] @f$) in practice the inputs are limited to the
         * numerical limits of the real datatype, thus this method returns the
         * range ([DBL_MIN, DBL_MAX]) for each of the 6 inputs
         */
        std::pair< rw::math::Q, rw::math::Q > getBounds () const;

        /**
         * @brief get base of the device
         * @return base Frame
         */
        rw::kinematics::Frame* getBase ();

        #if !defined(SWIGJAVA)
            /**
             * @brief get base of the device
             * @return base Frame
             */
            const rw::kinematics::Frame* getBase () const;
        #endif

        /**
         * @brief get end of the device
         * @return end Frame
         */
        rw::kinematics::Frame* getEnd ();

        #if !defined(SWIGJAVA)
            /**
             * @brief get end of the device
             * @return end Frame
             */
            const rw::kinematics::Frame* getEnd () const;
        #endif

        /**
         * @brief Calculates the jacobian matrix of the end-effector described
         * in the robot base frame @f$ ^b_e\mathbf{J}_{\mathbf{q}}(\mathbf{q})
         * @f$
         *
         * @return the @f$ 6*ndof @f$ jacobian matrix: @f$
         * {^b_e}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * Where:
         *
         * \f[
         *  {^b_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) = \mathbf{I}^{6\times 6} =
         *  \left[
         *    \begin{array}{cccccc}
         *    1 0 0 0 0 0\\
         *    0 1 0 0 0 0\\
         *    0 0 1 0 0 0\\
         *    0 0 0 1 0 0\\
         *    0 0 0 0 1 0\\
         *    0 0 0 0 0 1\\
         *    \end{array}
         *  \right]
         * \f]
         *
         */
        rw::math::Jacobian baseJend (const rw::kinematics::State& state) const;

        rw::core::Ptr<JacobianCalculator> baseJCframes (const std::vector< rw::kinematics::Frame* >& frames,
                                              const rw::kinematics::State& state) const;
        /**
         * @copydoc Device::getDOF
         *
         * This method always returns the value 6
         */
        size_t getDOF () const;

        /**
         * @brief set outer bound of the device
         * @param bounds [in] the minimum Q and the maximum Q
         */
        virtual void setBounds (const std::pair<rw::math::Q, rw::math::Q>& bounds);

        /**
         * @brief get the Joint velocity limit
         * @return the velocity limit as Q
         */
        virtual rw::math::Q getVelocityLimits () const;

        /**
         * @brief set the Joint velocity limit
         * @param vellimits [in] the velocity limit as Q
         */
        virtual void setVelocityLimits (const rw::math::Q& vellimits);

        /**
         * @brief get the Joint Acceleration limit
         * @return the Acceleration limit as Q
         */
        rw::math::Q getAccelerationLimits () const;

        /**
         * @brief set the Joint Acceleration limit
         * @param acclimit [in] the acceleration limit as Q
         */
        void setAccelerationLimits (const rw::math::Q& acclimits);
    };

// ################# SerialDevice

    /**
     * @brief The device for a serial chain.
     *
     * SerialChain is like JointDevice except that SerialChain has the
     * additional guarantee that the joints lie on a single parent to child
     * path of the kinematic tree.
     */
    class SerialDevice: public JointDevice
    {
      public:
        /**
         * @brief Constructor
         *
         * @param first [in] the base frame of the robot
         * @param last [in] the end-effector of the robot
         * @param name [in] name of device
         * @param state [in] the connectedness of the frames
         */
        SerialDevice (rw::kinematics::Frame* first, rw::kinematics::Frame* last, const std::string& name,
                      const rw::kinematics::State& state);

        /**
         * @brief Frames of the device.
         *
         * This method is being used when displaying the kinematic
         * structure of devices in RobWorkStudio. The method really
         * isn't of much use for everyday programming.
         *
         * @return list of raw Frame pointers.
         */
        const std::vector< rw::kinematics::Frame* >& frames () const;   


        /**
         * @brief Creates object
         *
         * @param serialChain [in] a vector of connected frames. The
         * first frame in \b serialChain is the base of the device and
         * the last frame of \b serialChain is the end of the device.
         * The joints of the device are the active joints of
         * \b serialChain.
         *
         * @param name [in] name of device
         *
         * @param state [in] the initial state of everything
         */
        SerialDevice (const std::vector< rw::kinematics::Frame* >& serialChain, const std::string& name,
                      const rw::kinematics::State& state);         


    };

    %template (SerialDevicePtr) rw::core::Ptr<SerialDevice>;
    %template (SerialDevicePtrVector) std::vector<rw::core::Ptr<SerialDevice>>;
    %template (SerialDeviceCPtr) rw::core::Ptr<const SerialDevice>;
    OWNEDPTR(SerialDevice)

    %extend rw::core::Ptr<SerialDevice> {
        rw::core::Ptr<const SerialDevice> asSerialDeviceCPtr() { return *$self; }
    }


// ################# SphericalJoint
    /**
     * @brief A spherical joint that allows rotations in all directions.
     *
     * Rotation is allowed around the x-, y- and z-axes. The position is fixed.
     */
    class SphericalJoint : public Joint
    {
      public:

        /**
         * @brief Construct a spherical joint.
         * @param name [in] name of the joint.
         * @param transform [in] static transform of the joint.
         */
        SphericalJoint (const std::string& name, const rw::math::Transform3D<double>& transform);

        //! @brief Destructor.
        virtual ~SphericalJoint ();

        // From Frame
        //! @brief Frame::doMultiplyTransform
        virtual void doMultiplyTransform (const rw::math::Transform3D<double>& parent,
                                          const rw::kinematics::State& state,
                                          rw::math::Transform3D<double>& result) const;

        //! @brief Frame::doGetTransform
        virtual rw::math::Transform3D<double> doGetTransform (const rw::kinematics::State& state) const;

        // From Joint
        //! @copydoc Joint::getJacobian
        virtual void getJacobian (std::size_t row, std::size_t col,
                                  const rw::math::Transform3D<double>& joint,
                                  const rw::math::Transform3D<double>& tcp,
                                  const rw::kinematics::State& state,
                                  rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::getFixedTransform
        virtual rw::math::Transform3D<double> getFixedTransform () const;

        //! @copydoc Joint::setFixedTransform
        virtual void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform
        virtual rw::math::Transform3D<double>
        getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc Joint::setJointMapping
        virtual void setJointMapping (rw::core::Ptr<rw::math::Function1Diff<>> function);

        //! @copydoc Joint::removeJointMapping
        virtual void removeJointMapping ();
    };
    %template(SphericalJointPtr) rw::core::Ptr<SphericalJoint>;

// ################# TreeDevice
    /**
     * @brief A tree structured device
     *
     * This device type defines devices that are tree-structured, with multiple end effectors.
     * Typical for dexterous hands, and multi-armed robots.
     *
     * @dot
     * digraph TreeDevice {
     *  node [shape=record, fontname=Helvetica, fontsize=10, style=filled];
     *  Base [ fillcolor="red"];
     *  Link1 [ label="Link1\n<Link>", fillcolor="red"];
     *  Axis1 [ label="Axis1\n<Joint>", fillcolor="red"];
     *  Link2 [ label="Link2\n<Link>",fillcolor="red"];
     *  Axis2 [ label="Axis2\n<Joint>",fillcolor="red"];
     *  Link3 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis3 [ label="Axis1\n<Joint>",fillcolor="red"];
     *  EndEffector1 [ fillcolor="red"];
     *  Link4 [ label="Link2\n<Link>",fillcolor="red"];
     *  Axis4 [ label="Axis2\n<Joint>",fillcolor="red"];
     *  Link5 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis5 [ label="Axis1\n<Joint>",fillcolor="red"];
     *  EndEffector2 [ fillcolor="red"];
     *  Link6 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis6 [ label="Axis1\n<Joint>",fillcolor="red"];
     *  EndEffector3 [ fillcolor="red"];
     *
     * world -> object1;
     * world -> table;
     * table -> object2;
     * world -> Base;
     * Base -> Link1;
     * Link1 -> Axis1;
     * Axis1 -> Link2;
     * Link2 -> Axis2;
     * Axis2 -> Link3;
     * Link3 -> Axis3;
     * Axis3 -> EndEffector1;
     * Axis1 -> Link4
     * Link4 -> Axis4
     * Axis4 -> Link5
     * Link5 -> Axis5
     * Axis5 -> EndEffector2
     * Axis2 -> Link6
     * Axis6 -> EndEffector3
     * }
     * @enddot
     *
     */
    class TreeDevice: public JointDevice
    {
      public:
        /**
         * @brief Constructor
         *
         * @param base [in] the base frame of the robot
         * @param ends [in] the set of end-effectors of the robot
         * @param name [in] name of device
         * @param state [in] the initial state of everything
         */
        TreeDevice (rw::kinematics::Frame* base, const std::vector< rw::kinematics::Frame* >& ends,
                    const std::string& name, const rw::kinematics::State& state);

        //! @brief destructor
        virtual ~TreeDevice ();

        /**
         * @brief like Device::baseJend() but with a Jacobian calculated for all
         * end effectors.
         */
        rw::math::Jacobian baseJends (const rw::kinematics::State& state) const;

        /**
           @brief The end-effectors of the tree device.
         */
        const std::vector< rw::kinematics::Frame* >& getEnds () const;

        /**
         * @brief Frames of the device.
         *
         * This method is being used when displaying the kinematic structure of
         * devices in RobWorkStudio. The method really isn't of much use for
         * everyday programming.
         */
        const std::vector< rw::kinematics::Frame* >& frames () const;

    };
    %template (TreeDevicePtr) rw::core::Ptr<TreeDevice>;
    %template (TreeDeviceCPtr) rw::core::Ptr<const TreeDevice>;
    %template (TreeDevicePtrVector) std::vector<rw::core::Ptr<TreeDevice>>;
    OWNEDPTR(TreeDevice);

// ################# UniverSalJoint
    /**
     * @brief A universal joint that allows rotations in two directions.
     *
     * Rotation is allowed around the x and y axes. The position and rotation around the z axis is
     * fixed.
     */
    class UniversalJoint : public Joint
    {
      public:

        /**
         * @brief Construct a universal joint.
         * @param name [in] name of the joint.
         * @param transform [in] static transform of the joint.
         */
        UniversalJoint (const std::string& name, const rw::math::Transform3D<double>& transform);

        //! @brief Destructor.
        virtual ~UniversalJoint ();

        // From Frame
        //! @brief Frame::doMultiplyTransform
        virtual void doMultiplyTransform (const rw::math::Transform3D<double>& parent,
                                          const rw::kinematics::State& state,
                                          rw::math::Transform3D<double>& result) const;

        //! @brief Frame::doGetTransform
        virtual rw::math::Transform3D<double> doGetTransform (const rw::kinematics::State& state) const;

        // From Joint
        //! @copydoc Joint::getJacobian
        virtual void getJacobian (std::size_t row, std::size_t col,
                                  const rw::math::Transform3D<double>& joint,
                                  const rw::math::Transform3D<double>& tcp,
                                  const rw::kinematics::State& state,
                                  rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::getFixedTransform
        virtual rw::math::Transform3D<double> getFixedTransform () const;

        //! @copydoc Joint::setFixedTransform
        virtual void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform
        virtual rw::math::Transform3D<double>
        getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc Joint::setJointMapping
        virtual void setJointMapping (rw::core::Ptr<rw::math::Function1Diff<>> function);

        //! @copydoc Joint::removeJointMapping
        virtual void removeJointMapping ();
    };
    %template(UniversalJointPtr) rw::core::Ptr<UniversalJoint>;

// ################# VirtualJoint
    /**
       @brief Virtuals joints.

       VirtualJoint is a joint with a role similar to a rw::kinematics::FixedFrame with
           an optional number of dof allocated in the state.

       Virtual joints are useful when you want a store joint values of e.g.
           a number of passive joints.
     */
    class VirtualJoint : public Joint
    {
      public:
        /**
         * @brief A virtual joint with a displacement transform of \b transform.
         * @param name [in] The name of the frame.
         * @param transform [in] The displacement transform of the joint.
         * @param dof [in] Number of degrees of freedom of the joint
         */
        VirtualJoint (const std::string& name, const rw::math::Transform3D<double>& transform, size_t dof);

        //! @copydoc Joint::getJacobian
        void getJacobian (size_t row, size_t col, const rw::math::Transform3D<double>& joint,
                          const rw::math::Transform3D<double>& tcp, const rw::kinematics::State& state,
                          rw::math::Jacobian& jacobian) const;

        //! @copydoc Joint::getFixedTransform()
        rw::math::Transform3D<double> getFixedTransform () const;

        //! @copydoc Joint::getFixedTransform()
        void setFixedTransform (const rw::math::Transform3D<double>& t3d);

        //! @copydoc Joint::getJointTransform()
        rw::math::Transform3D<double> getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc Joint::setJointMapping()
        virtual void setJointMapping (rw::core::Ptr<rw::math::Function1Diff<>> function);

        //! @copydoc Joint::removeJointMapping()
        virtual void removeJointMapping ();

      protected:
        rw::math::Transform3D<double> doGetTransform (const rw::kinematics::State& state) const;

        void doMultiplyTransform (const rw::math::Transform3D<double>& parent, const rw::kinematics::State& state,
                                  rw::math::Transform3D<double>& result) const;
    };
    %template(VirtualJointPtr) rw::core::Ptr<VirtualJoint>;

// ################# WorkCell

    /**
     * @brief WorkCell keeps track of devices, obstacles and objects in the
     * scene.
     *
     * WorkCell is a pretty dumb container to which you can add your devices and
     * the frames you your GUI to show as objects or camera views.
     *
     * WorkCell is responsible for keeping track of everything including all
     * devices, object and obstacles in the environment. WorkCell contains the
     * World Frame, which represents the root and the only frame without a
     * parent.
     */
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
         * @brief Constructs a WorkCell
         *
         * @param tree [in] The (initial) tree structure of the WorkCell
         *
         * @param name [in] The name of the workcell. A good name for the
         * workcell would be the (eventual) file that the workcell was
         * loaded from.
         *
         * @param filename [in] The filename from which the workcell is
         * loaded.
         */
        WorkCell(
                rw::core::Ptr<rw::kinematics::StateStructure> tree,
                const std::string& name = "",
                const std::string& filename = "");

        /**
         * Destroys a work cell including the devices that have been added.
         *
         * Management of the frames is done by a tree of which the work cell
         * knows nothing. Therefore if this kinematics tree is still in
         * existence (which it probably is), then the frames that used to be
         * accessible via this work cell will still be valid.
         */
        ~WorkCell();

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
        rw::kinematics::Frame* getWorldFrame() const;

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
        void addFrame(rw::kinematics::Frame* frame, rw::kinematics::Frame* parent=NULL);

        /**
         * @brief Adds \b frame with \b parent as parent.
         *
         * If parent == NULL, then \b world is used as parent
         *
         * @param frame [in] Frame to add
         * @param parent [in] Parent frame - uses World is parent == NULL
         */
        void addFrame(rw::core::Ptr<rw::kinematics::Frame> frame,
                rw::core::Ptr<rw::kinematics::Frame> parent = NULL);


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
        void addDAF(rw::kinematics::Frame* frame, rw::kinematics::Frame* parent = NULL);

        /**
         * @brief Adds dynamically attachable frame (DAF) \b frame with
         * \b parent as parent.
         *
         * If parent == NULL, then \b world is used as parent
         *
         * @param frame [in] Frame to add
         * @param parent [in] Parent frame - uses World is parent == NULL
         */
        void addDAF(rw::core::Ptr<rw::kinematics::Frame> frame,
                rw::core::Ptr<rw::kinematics::Frame> parent = NULL);

        /**
         * @brief Removes \b frame from work cell
         * @param frame [in] Frame to remove
         * @deprecated Since January 2018.
         * Please use remove(rw::core::Ptr<rw::kinematics::Frame>)
         * instead.
         */
        void remove(rw::kinematics::Frame* frame);

        /**
         * @brief Removes \b frame from work cell
         * @param frame [in] Frame to remove
         */
        void remove(rw::core::Ptr<rw::kinematics::Frame> frame);

        /**
         * @brief Removes \b object from workcell
         * @param object [in] Object to remove
         */
        void removeObject(Object* object);

        /**
         * @brief Adds a Device to the WorkCell.
         * Ownership of \b device is taken.
         * @param device [in] pointer to device.
         */
        void addDevice(rw::core::Ptr<Device> device);

        /**
         * @brief Returns a reference to a vector with pointers to the
         * Device(s) in the WorkCell
         * @return const vector with pointers to Device(s).
         */
        const std::vector<rw::core::Ptr<Device> >& getDevices() const;

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
        rw::kinematics::Frame* findFrame(const std::string& name) const;

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
            rw::kinematics::MovableFrame* findMovableFrame(const std::string& name)
            { 
                return $self->WorkCell::findFrame<rw::kinematics::MovableFrame>(name); 
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
            rw::kinematics::FixedFrame* findFixedFrame(const std::string& name)
            { 
                return $self->WorkCell::findFrame<rw::kinematics::FixedFrame>(name); 
            }

            /**
             * @brief Returns all \b MovableFrames.
             * @return all frames of type \b MovableFrames in the workcell
             */
            std::vector<rw::kinematics::MovableFrame*> findMovableFrames() const
            { 
                return $self->WorkCell::findFrames<rw::kinematics::MovableFrame>(); 
            }

            /**
             * @brief Returns all \b FixedFrame.
             * @return all frames of type \b FixedFrame in the workcell
             */
            std::vector<rw::kinematics::FixedFrame*> findFixedFrames() const
            { 
                return $self->WorkCell::findFrames<rw::kinematics::FixedFrame>(); 
            }

        };

        /**
         * @brief Returns all frames in workcell
         * @return List of all frames
         */
        std::vector<rw::kinematics::Frame*> getFrames() const;

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device.
         *
         * @param name [in] The device name
         *
         * @return The device named \b name or NULL if no such device.
         */
        rw::core::Ptr<Device> findDevice(const std::string& name) const;

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
            rw::core::Ptr<JointDevice> findJointDevice(const std::string& name)
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
            rw::core::Ptr<SerialDevice> findSerialDevice(const std::string& name)
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
            rw::core::Ptr<TreeDevice> findTreeDevice(const std::string& name)
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
            rw::core::Ptr<ParallelDevice> findParallelDevice(const std::string& name)
            { 
                return $self->WorkCell::findDevice<ParallelDevice>(name); 
            }

            /**
             * @brief Returns a vector with pointers to the Device(s) with a
             * specific type \b JointDevice in the WorkCell
             *
             * @return vector with pointers to Device(s) of type T.
             */
            std::vector < rw::core::Ptr<JointDevice> > findJointDevices()
            { 
                return $self->WorkCell::findDevices<JointDevice>(); 
            }

            /**
             * @brief Returns a vector with pointers to the Device(s) with a
             * specific type \b SerialDevice in the WorkCell
             *
             * @return vector with pointers to Device(s) of type T.
             */
            std::vector < rw::core::Ptr<SerialDevice> > findSerialDevices()
            { 
                return $self->WorkCell::findDevices<SerialDevice>(); 
            }

            /**
             * @brief Returns a vector with pointers to the Device(s) with a
             * specific type \b TreeDevice in the WorkCell
             *
             * @return vector with pointers to Device(s) of type T.
             */
            std::vector < rw::core::Ptr<TreeDevice> > findTreeDevices()
            { 
                return $self->WorkCell::findDevices<TreeDevice>(); 
            }

            /**
             * @brief Returns a vector with pointers to the Device(s) with a
             * specific type \b ParallelDevice in the WorkCell
             *
             * @return vector with pointers to Device(s) of type T.
             */
            std::vector < rw::core::Ptr<ParallelDevice> > findParallelDevices()
            { 
                return $self->WorkCell::findDevices<ParallelDevice>(); 
            }
        };

        /**
         * @brief Returns a default State
         *
         * @return default State
         */
        rw::kinematics::State getDefaultState() const;

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
        rw::core::Ptr<SensorModel> findSensor(const std::string& name) const;

        //TODO(kalor) findSensor<T>(name);
        //TODO(kalor) findSensors<T>();

        /**
         * @brief Returns all frames in workcell
         * @return List of all frames
         */
        std::vector<rw::core::Ptr<SensorModel> > getSensors() const;

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
        rw::core::Ptr<ControllerModel> findController(const std::string& name) const;
        

        //TODO(kalor) findController<T>(name);
        //TODO(kalor) findControllers<T>();


        /**
         * @brief Returns all controllers in workcell
         * @return List of all controllers
         */
        std::vector<rw::core::Ptr<ControllerModel> > getControllers() const;

        /**
         * @brief Returns all object in the work cell
         *
         * @return All object in work cell
         */
        std::vector<rw::core::Ptr<Object> > getObjects() const;

        /**
         * @brief The object named \b name of the workcell.
         *
         * NULL is returned if there is no such object.
         *
         * @param name [in] The object name
         *
         * @return The object named \b name or NULL if no such object.
         */
        rw::core::Ptr<Object> findObject(const std::string& name) const;

        //! @brief Add device to workcell
        void add(rw::core::Ptr<Device> device);
        //! @brief Add object to workcell
        void add(rw::core::Ptr<Object> object);
        //! @brief Add sensormodel to workcell
        void add(rw::core::Ptr<SensorModel> sensor);
        //! @brief Add controllermodel to workcell
        void add(rw::core::Ptr<ControllerModel> controller);

        //! @brief Remove object from workcell
        void remove(rw::core::Ptr<Object> object);
        //! @brief Remove device from workcell
        void remove(rw::core::Ptr<Device> device);
        //! @brief Remove sensormodel from workcell
        void remove(rw::core::Ptr<SensorModel> sensor);
        //! @brief Remove controllermodel from workcell
        void remove(rw::core::Ptr<ControllerModel> controller);

        /**
         * @brief gets the complete state structure of the workcell.
         * @return the state structure of the workcell.
         */
        rw::core::Ptr< rw::kinematics::StateStructure > getStateStructure ();

        /**
         * @brief Returns the work cell changed event
         * @return
         */
        rw::core::Event< WorkCellChangedListener, int >& workCellChangedEvent ();

        /**
         * @brief Properties of this workcell
         */
        rw::core::PropertyMap& getPropertyMap ();

        /**
         * @brief Returns collision setup associated to work cell
         *
         * @return Collision setup
         */
        CollisionSetup getCollisionSetup ();

        /**
         * @brief Get the scene descriptor.
         * @return the scene descriptor.
         */
        rw::core::Ptr< SceneDescriptor > getSceneDescriptor ();

        /**
         * @brief Set the scene descriptor.
         * @param scene [in] the scene descriptor.
         */
        void setSceneDescriptor (rw::core::Ptr< SceneDescriptor > scene);

        /**
         * @brief Returns the full path and filename of the workcell.
         *
         * If the workcell is loaded from file, then this method returns the
         * full filename. Otherwise it returns an empty string.
         */
        std::string getFilename () const;

        /**
         * @brief Returns the path of where the work cell is located
         *
         * If the workcell is not loaded from file, it returns an empty
         * string
         */
        std::string getFilePath () const;

        /**
         * @brief Returns the filename of the calibration associated to the
         * work cell.
         *
         * Returns an empty string in case no calibration is associated.
         *
         * To load the file use the getFilePath()+getCalibrationFilename()
         * to get the absolute location
         */
        const std::string& getCalibrationFilename () const;


        /**
         * @brief Sets the filename of the calibration file
         *
         * @param calibrationFilename [in] Filename of calibration file with
         * path relative to the work cell path.
         */
        void setCalibrationFilename (const std::string& calibrationFilename);

      private:
        WorkCell(const WorkCell&);
        WorkCell& operator=(const WorkCell&);
    };

    %template (WorkCellPtr) rw::core::Ptr<WorkCell>;
    %template (WorkCellCPtr) rw::core::Ptr<const WorkCell>;
    %template (WorkCellChangedEvent) rw::core::Event< WorkCellChangedListener, int >;
