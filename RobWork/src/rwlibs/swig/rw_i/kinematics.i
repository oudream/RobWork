//################ StateData
    /**
     * @brief the basic building block for the stateless design using
     * the StateStructure class. A StateData represents a size,
     * a unique id, and a unique name, when inserted into the StateStructure.
     * The size will allocate "size"-doubles in State objects originating from the
     * StateStructure.
     */
    class StateData {
      public: 

        /**
         * @brief An integer ID for the StateData.
         *
         * IDs are assigned to the state data upon insertion State.
         * StateData that are not in a State have an ID of -1.
         *
         * StateData present in different trees may have identical IDs.
         *
         * IDs are used for the efficient implementation of State. Normally,
         * you should not make use of StateData IDs yourself.
         *
         * @return An integer ID for the frame.
         */
        inline int getID () const;
        
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

        #if defined(SWIGJAVA)
            %apply double[] {double *};
        #endif

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

        %extend {
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
            std::vector<double> getDataVector(State& state){
                double* data = $self->getData(state);
                std::vector<double> ret($self->size());
                for(int i = 0; i < $self->size(); i++){
                    ret.push_back(data[i]);
                }
                return ret;
            }
        }
        /**
         * @brief Assign for \b state data the size() of values of the array \b
         * vals.
         *
         * The array \b vals must be of length at least size().
         *
         * @param state [inout] The state to which \b vals are written.
         *
         * @param vals [in] The joint values to assign.
         *
         * setData() and getData() are related as follows:
         * \code
         * data.setData(state, q_in);
         * const double* q_out = data.getData(state);
         * for (int i = 0; i < data.getDOF(); i++)
         *   q_in[i] == q_out[i];
         * \endcode
         */
        inline void setData (State& state, const double* vals) const;

        /**
         * @brief Check is state data includes a cache.
         * @return true if cache, false otherwise.
         */
        inline bool hasCache () const;

        /**
         * @brief Get the cache.
         * @param state [in] the state.
         * @return the cache.
         */
        rw::core::Ptr< StateCache > getCache (State& state);

        /**
         * @brief Get default cache.
         * @return the cache.
         */
        rw::core::Ptr< StateCache > getDefaultCache ();

        /**
         * @brief Set the cache values.
         * @param cache [in] the cache.
         * @param state [in/out] state updated with new cache.
         */
        void setCache (rw::core::Ptr< StateCache > cache, State& state);

        /**
         * @brief Get the state structure.
         * @return the state structure.
         */
        StateStructure* getStateStructure ();

        /**
         * @brief A state with \b size number of doubles in the State vector.
         *
         * \b size must be non-negative.
         *
         * The newly created state data can be added to a structure with
         * StateStructure::addData().
         *
         * The size of the state data in nr of doubles of the state data
         * is constant throughout
         * the lifetime of the state data.
         *
         * @param size [in] The number of degrees of freedom of the frame.
         *
         * @param name [in] The name of the frame.
         */
        StateData (int size, const std::string& name);

        /**
         * @copydoc StateData(int, const std::string&)
         * @param cache [in] a cache.
         */
        StateData (int size, const std::string& name, rw::core::Ptr< StateCache > cache);

    };

    %template(StateDataPtr) rw::core::Ptr<StateData>;
    %template(VectorStateDataPtr) std::vector<rw::core::Ptr<StateData>>;

//################ Frame

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

    %template (FramePtr) rw::core::Ptr<Frame>;
    %template (FrameCPtr) rw::core::Ptr<const Frame>;
    %template (FrameVector) std::vector<Frame*>;
    %template (FramePair) std::pair< Frame *, Frame * >;
    %template (FramePairVector) std::vector< std::pair< Frame *, Frame * > >;
    %template (VectorVectorFrame) std::vector<std::vector<Frame*>>;
    %template (MapStringFrame) std::map<std::string,Frame*>;


//################ FixedFrame
    
    /**
     * @brief FixedFrame is a frame for which the transform relative to the
     * parent is constant.
     *
     * A fixed frame can for example be used for attaching a camera, say, with a
     * fixed offset relative to the tool.
     */
    class FixedFrame: public Frame {
    public:
        /**
         * @brief A frame fixed to its parent with a constant relative transform
         * of \b transform.
         *
         * @param name [in] The name of the frame.
         * @param transform [in] The transform with which to attach the frame.
         */
        FixedFrame(const std::string& name, const rw::math::Transform3D<double> & transform);
        
        //! @brief destructor
        virtual ~FixedFrame();

        /**
         * @brief Sets the fixed transform of this frame.
         * @param transform [in] the new transformation of this frame
         * @note THIS IS NOT THREAD SAFE. If you need thread safety then use
         * MovableFrame instead or make sure multiple threads are not using this
         * frame when changing the transformation.
         */
        void setTransform(const rw::math::Transform3D<double> & transform);

		/**
		 * @brief Move the frame such that it is located with a relative transform \b refTtarget relative to \b refframe.
		 * @param refTtarget [in] the transform relative to \b refframe .
		 * @param refframe [in] the reference frame.
		 * @param state [in] the state giving the current poses.
		 */
		void moveTo(const rw::math::Transform3D<double>& refTtarget, Frame* refframe, State& state);

        /**
         * @brief get the fixed transform of this frame.
         */
        const rw::math::Transform3D<double> & getFixedTransform() const;
    };
    %template(FixedFramePtr) rw::core::Ptr<FixedFrame>;
    %template(VectorFixedFrame) std::vector<FixedFrame*>;
//################ FKRange
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
        rw::core::Ptr< const Frame > getEnd() const;

        /**
         * @brief Returns the first frame in the range.
         *
         * @return The base frame (from).
         */
        rw::core::Ptr< const Frame > getBase() const;
    };


//################ FKTable

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
         * @brief Forward kinematics for the work cell state \b state.
         * @param state [in] The work state for which world transforms are to be
         * calculated.
         */
        FKTable (const State* state);

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
//################ FrameMap

    namespace rw { namespace kinematics {


        template <class T>
        class FrameMap 
        {
          public:

            /**
             * @brief creates a framemap with an initial size of s
             * @param s [in] nr of elements of the types T with default value "defaultVal"
             * @param defaultVal [in] the default value of new instances of T
             */
            FrameMap(const T& defaultVal, int s = 20);

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
    %template(FrameMapd) rw::kinematics::FrameMap< double >;

//################ FramePairMap

    namespace rw { namespace kinematics {


        //! @brief A map from an unordered pair of frames to some value.
        template <class T>
        class FramePairMap: public PairMap<const Frame*, T>
        {
        public:
            //! @copydoc rw::common::PairMap::PairMap()
            FramePairMap();

            //! @copydoc rw::common::PairMap::PairMap(const T2&)
            FramePairMap(const T& defaultVal);
        };

    }}
//################ FrameTypes
    /**
     * @brief Enumeration of all concrete frame types of RobWork.
     *
     * FrameType::Type is an enumeration of all frame types defined within
     * RobWork. For every implementation X of Frame, FrameType has an
     * enumeration value named FrameType::X.
     *
     * The type of a frame can be accessed via frameTypeAccessor().
     *
     * It is the responsibility of the work cell loaders to properly initialize
     * the frame type values.
     *
     * The use of FrameType is a hack introduced due to the lack of a working
     * dynamic_cast<>.
     */
    class FrameType
    {
      public:
        /**
         * @brief FrameType enumeration
         */
        enum Type {
            RevoluteJoint,
            PrismaticJoint,
            FixedFrame,
            MovableFrame,
            DependentJoint,
            Unknown
        };

        /**
         * @brief Identifier for a frame of type \b type.
         *
         * @param type [in] The type of frame.
         */
        FrameType (const Type& type);

        /**
         * @brief The frame type.
         *
         * @return The frame type.
         */
        Type get () const;
    };

//################ Kinematics
    %nodefaultctor Kinematics;
    /**
     * @brief Utility functions for the rw::kinematics module.
    */
    class Kinematics
    {
      public:
        /**
         * @brief The transform of \b frame in relative to the world frame.
         *
         * If to=NULL the method returns a \f$4\times 4\f$ identify matrix
         *
         * @param to [in] The transform for which to find the world frame.
         *
         * @param state [in] The state of the kinematics tree.
         *
         * @return The transform of the frame relative to the world frame.
         */
        static rw::math::Transform3D<double> worldTframe (const Frame* to, const State& state);

        /**
         * @brief The transform of frame \b to relative to frame \b from.
         *
         * FrameTframe() is related to WorldTframe() as follows:
         \code
         frameTframe(from, to, state) ==
         inverse(worldTframe(from, state)) *
         worldTframe(to, state);
         \endcode
         *
         * @param from [in] The start frame.
         *
         * @param to [in] The end frame.
         *
         * @param state [in] The state of the kinematics tree.
         *
         * @return The transform from the start frame to the end frame.
         */
        static rw::math::Transform3D<double> frameTframe (const Frame* from, const Frame* to,
                                                const State& state);

        /** @brief All frames reachable from \b root for a tree structure of \b
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
        static std::vector< Frame* > findAllFrames (Frame* root, const State& state);

        /** @brief All frames reachable from \b root for a tree structure.
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
        static std::vector< Frame* > findAllFrames (Frame* root);

        /**
           @brief Find the world frame of the workcell by traversing the path
           from \b frame to the root of the tree.

           The state \b state is needed to retrieve the parent frames, but the
           world frame returned is the same for any (valid) state.
        */
        static Frame* worldFrame (Frame* frame, const State& state);

        /**
           @brief The chain of frames connecting \b child to \b parent.

           \b child is included in the chain, but \b parent is not included. If
           \b parent is NULL then the entire path from \b child to the world
           frame is returned. If \b child as well as \b parent is NULL then the
           empty chain is gracefully returned.

           The \b state gives the connectedness of the tree.

           If \b parent is not on the chain from \b child towards the root, then
           an exception is thrown.
        */
        static std::vector< Frame* > childToParentChain (Frame* child, Frame* parent,
                                                         const State& state);

        /**
           @brief Like ChildToParentChain() except that the frames are returned
           in the reverse order.
        */
        static std::vector< Frame* > reverseChildToParentChain (Frame* child, Frame* parent,
                                                                const State& state);

        /**
           @brief The chain of frames connecting \b parent to \b child.

           \b parent is included in the list, but \b child is excluded. If \b
           parent as well as \b child is NULL then the empty chain is returned.
           Otherwise \b parent is included even if \b parent is NULL.
         */
        static std::vector< Frame* > parentToChildChain (Frame* parent, Frame* child,
                                                         const State& state);

        /**
         * @brief A map linking frame names to frames.
         *
         * The map contains an entry for every frame below \b root in the tree with
         * structure described by \b state.
         *
         * @param root [in] Root of the kinematics tree to search.
         *
         * @param state [in] The kinematics tree structure.
         */
        static std::map< std::string, Frame* >
        buildFrameMap (Frame* root, const State& state);

        /**
           @brief True if \b frame is a DAF and false otherwise.
        */
        static bool isDAF (const Frame* frame);

        /**
         * @brief Check if frame is fixed.
         * @param frame [in] the frame.
         * @return true if fixed, false otherwise.
         */
        static bool isFixedFrame (const Frame* frame);

        /**
         * @brief Grip \b item with \b gripper thereby modifying \b state.
         *
         * \b item must be a DAF.
         *
         * @param item [in] the frame to grip.
         * @param gripper [in] the grasping frame.
         * @param state [in/out] the state.
         * @exception An exception is thrown if \b item is not a DAF.
         * @see See also gripFrame(MovableFrame*, Frame*, State&).
         */
        static void gripFrame (Frame* item, Frame* gripper, State& state);

        /**
         * @brief Grip \b item with \b gripper thereby modifying \b state.
         *
         * \b item must be a DAF.
         *
         * @param item [in] the frame to grip.
         * @param gripper [in] the grasping frame.
         * @param state [in/out] the state.
         * @exception An exception is thrown if \b item is not a DAF.
         * @see See also gripFrame(Frame*, Frame*, State&).
         */
        static void gripFrame (MovableFrame* item, Frame* gripper, State& state);

        /**
         * @brief Get static frame groups.
         *
         * A static frame group consist of frames that are fixed with respect to the other frames in
         * the group. A Dynamically Attachable Frame (DAF) or a MovableFrame will divide a static
         * group.
         * @param root [in] the root frame of the tree to search.
         * @param state [in] containing information about the current tree state and the Dynamically
         * Attachable Frames (DAF).
         * @return vector with the frame groups.
         */
        static std::vector< std::vector<Frame*> > getStaticFrameGroups (Frame* root, const State& state);
    };

//################ MovableFrame
    /**
     * @brief MovableFrame is a frame for which it is possible to freely
     * change the transform relative to the parent.
     *
     * A MovableFrame can for example be used for modelling objects moving in
     * the scene based on e.g. user input.
     */
    class MovableFrame: public Frame
    {
      public:

        /**
         * @brief Construct a MovableFrame with Identiy as the initial
         * transform
         *
         * @param name [in] name of the frame
         */
        explicit MovableFrame(const std::string& name);

        //! destructor
        virtual ~MovableFrame ();

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
    %template (MovableFrameVector) std::vector<MovableFrame *> ;
    %template (MovableFramePtr) rw::core::Ptr<MovableFrame> ;
    OWNEDPTR(MovableFrame);

//################ QState
    /**
     * @brief The configuration state of a work cell.
     *
     * The configuration state contains state data values for all
     * valid StateData in a StateStructure. The validity is defined by the
     * StateSetup.
     *
     * See Frame::getTransform() for the calculation of the relative transform
     * of a frame for a given configuration state.
     *
     * Configuration states can be freely copied and assigned.
     *
     * The configuration state is a part of the StateStructure state (see
     * State).
     */
    class QState
    {
      public:
        /**
         * @brief Constructs an empty QState
         */
        QState ();

        /**
         * @brief A configuration state.
         *
         * This constructor is not available for use outside of RobWork. Instead
         * your configuration states should be constructed via the copy
         * constructor.
         *
         * @param setup [in] The shared setup for configuration states.
         */
        explicit QState (rw::core::Ptr< StateSetup > setup);

        //! destructor
        virtual ~QState ();

        #if !defined(SWIGJAVA)
            /** TODO(kalor) extend function to return array
             * @brief An array of length frame.getDOF() containing the joint values
             * for \b frame.
             *
             * It is OK to call this method also for frames with zero degrees of
             * freedom.
             *
             * @return The joint values for the frame.
             */
            const double* getQ (const StateData& data) const;
        
            /**
             * @brief non const version of getQ.
             */
            double* getQ (const StateData& data);
        #endif
        
        /**
         * @brief Assign for \b frame the frame.getDOF() joint values of the
         * array \b vals.
         *
         * The array \b vals must be of length at least frame.getDOF().
         *
         * @param data [in] The StateData for which the joint values are assigned.
         *
         * @param vals [in] The joint values to assign.
         *
         * setQ() and getQ() are related as follows:
         * \code
         * q_state.setQ(frame, q_in);
         * const double* q_out = q_state.getQ(frame);
         * for (int i = 0; i < frame.getDOF(); i++)
         *   q_in[i] == q_out[i];
         * \endcode
         */
        void setQ (const StateData& data, const double* vals);

        /** TODO(kalor) implement
         * @brief streaming operator
         *
         * @param os [in] output stream
         *
         * @param state [in] state to stream out
         *
         * @return the stream
         */
        //friend std::ostream& operator<< (std::ostream& os, const QState& state);

        /**
         * @brief Scaling of a configuration state by a scalar.
         */
        friend QState operator* (const QState& q, double scale);

        /**
         * @brief Scaling of a configuration state by division
         */
        friend QState operator/ (const QState& q, double scale);

        /**
         * @brief Scaling of a configuration state by a scalar.
         */
        friend QState operator* (double scale, const QState& q);

        /**
         * @brief Addition of configuration states.
         */
        friend QState operator+ (const QState& a, const QState& b);

        /**
         * @brief Subtraction of configuration states.
         */
        friend QState operator- (const QState& a, const QState& b);

        /**
         * @brief Unary minus operator.
         */
        QState operator- () const;

        /**
         * @brief returns the StateSetup
         */
        rw::core::Ptr< StateSetup > getStateSetup () const;

        // void copy(const QState& qstate);

        /**
           @brief The dimension of the state vector.
         */
        size_t size () const;

        /*TODO(kalor) implement
         * @brief Get element of state.
         * @param index [in] the index.
         * @return the value at given index.
         */
        //double& operator() (size_t index);

        // @copydoc operator()(size_t)
        //const double& operator() (size_t index) const;
    };
//################ State
    %nodefaultctor State;
    /**
     * @brief The state of a work cell (or kinematics tree).
     *
     * You need a work cell state in order to calculate forward kinematics for
     * trees of frames.
     *
     * Work cell states can be copied and assigned freely.
     *
     * The work cell state consists of a part for the tree structure and a part
     * for the configuration values. You are encouraged to use the getParent(),
     * getChildren(), getQ() and setQ() utility functions rather than explicitly
     * type, say, state.getQState().getQ(). That way you will have a much easier
     * time of updating the code if we decide to change the way the kinematics
     * data structures are stored (!). Also getQ(state, frame) is shorter to
     * type also.
     *
     * The operation of a work cell state is undefined valid if the tree used
     * for its initialization is modified. (The implementation takes some care
     * to check for this and crashes the program in a controlled way if it takes
     * place.)
     */

    class State : public Serializable
    {
      public:
        //! @brief Smart pointer type to State.
        typedef rw::core::Ptr< State > Ptr;
        //! Value type.
        typedef double value_type;

        /**
         * @brief Default constructor giving an empty state.
         * Beware that the state is not initialized and that passing this state
         * to a procedure will typically cause a program crash.
         */
        State ();

        //! destructor
        virtual ~State ();

        /**
         * @brief Assign to a state the configuration state of this state.
         *
         * The State can be thought of as consisting of a tree state
         * (TreeState) (for the structure of the tree) and a configuration state
         * (QState) (containing joint values, for example). The setQStateInState()
         * method copies into this state the QState part of another state.
         *
         * @param to [out] The state to which the configuration state is written.
         */
        void setQStateInState (State& to) const;

        /**
         * @brief Assign to a state the tree state of this state.
         *
         * The State can be thought of as consisting of a tree state
         * (TreeState) (for the structure of the tree) and a configuration state
         * (QState) (containing joint values, for example). The setTreeState()
         * method copies into this state the TreeState part of another state.
         *
         * Implementation note: setTreeStateInState() is currently a lot faster than
         * setQStateInState() (even though they are both fast), so if you have the
         * choice then use the former.
         *
         * @param to [out] The state to which the tree state is written.
         */
        void setTreeStateInState (State& to) const;

        /**
         * @brief Scaling of the configuration state by a scalar.
         *
         * The tree state remains the same.
         */
        State operator* (double scale) const;

        /**
         * @brief Scaling of the configuration state by division.
         *
         * The tree state remains the same.
         */
        friend State operator/ (const State& state, double scale);

        /**
         * @brief Scaling of the configuration state by a scalar.
         *
         * The tree state remains the same.
         */
        friend State operator* (double scale, const State& state);

        /**
         * @brief Addition of configuration states.
         *
         * It is \e undefined whether it is the tree state of \b a or \b b that
         * is used for the resulting state. We say that it is undefined to force
         * you to use setTreeStateInState() to make you explicitly choose the
         * tree state.
         */
        friend State operator+ (const State& a, const State& b);

        /**
         * @brief Subtraction of configuration states.
         *
         * It is \e undefined whether it is the tree state of \b a or \b b that
         * is used for the resulting state. We say that it is undefined to force
         * you to use setTreeStateInState() to make you explicitly choose the
         * tree state.
         */
        friend State operator- (const State& a, const State& b);

        /**
         * @brief Unary minus operator.
         *
         * The tree state remains the same.
         */
        State operator- () const;

        /**
         * @brief copies data from a state into this state. The version
         * of the state is allowed to be different from this state. Only
         * state data that is valid for both states will be copied.
         * @param src [in] the state that is to be copied
         */
        void copy (const State& src);

        /**
         * @brief performs a deep copy of this state and returns the clone. Both
         * QState and TreeState are (deep) copied as normal however the cachestates will
         * be copied using their clone method.
         * @return a deep copy of this state (clone)
         */
        State clone ();

        /**
         * @brief performs a deep copy of \b src into this state.
         * @param src [in] the state that is to be cloned
         */
        void clone (const State& src);

        /**
         * @brief this function upgrades the current version of this
         * State to the newest state. It will not override data values that
         * is set in the current state.
         */
        void upgrade ();

        /**
         * @brief this function upgrades the current version of this
         * State with the given state. It will not override data values that
         * is set in the current state.
         */
        void upgradeTo (const State& state);

        /**
         * @brief The dimension of the configuration state vector.
         *
         * Knowing the size of the state is useful for example in error
         * messages, so that you can report if two states seem to belong to
         * different workcells.
         */
        size_t size () const;

        /*TODO(kalor) implement
         * @brief Provides direct access to the configurations stored in the state
         *
         * Notice that modifying a state directly may result in the state being inconsistent
         *
         * @param index [in] Index of element to access
         */
        //double& operator() (size_t index);

        /*TODO(kalor) implement
         * @brief Provides direct read access to the configurations stored in the state
         *
         * @param index [in] Index of element to access
         */
        //const double& operator() (size_t index) const;

        /*TODO(kalor) implement
           @brief Same as operator().
         */
        //double& operator[] (size_t index);

        /* TODO(kalor) implement
           @brief Same as operator().
         */
        //const double& operator[] (size_t index) const;

        /**
         * @brief gets the frame with id \b id. If a frame with id \b id does not exist
         * NULL is returned
         */
        Frame* getFrame (int id);

        /**
         * @brief get the state id. Represents the static structure of the StateStructure that
         * this state relates to.
         */
        int getUniqueId () const;

        /**
         * @brief Returns pointer to the state structure (the structure of Frame's and StateData)
         * @return Pointer to the StateStructure matching the frame
         */
        rw::core::Ptr< StateStructure > getStateStructure () const;

        // void add(Stateless& obj);

        //! @copydoc rw::common::Serializable::read
        void read (class InputArchive& iarchive, const std::string& id);

        //! @copydoc rw::common::Serializable::write
        void write (class OutputArchive& oarchive, const std::string& id) const;

        /**
         * @brief Get default.
         * @param data [in] the state data.
         * @return default state.
         */
        static const State& getDefault (StateData* data);

    };
    %template (StateVector) std::vector<State>;

//################ StateCache
    /**
     * @brief the basic building block for the stateless desing using
     * the StateStructure class. A StateCache represents a size,
     * a unique id, and a unique name, when inserted into the StateStructure.
     * The size will allocate "size"-doubles in State objects originating from the
     * StateStructure.
     */
    class StateCache
    {
      public:

        /**
         * @brief destructor
         */
        virtual ~StateCache (){};

        /*
         * @brief An integer ID for the StateCache.
         *
         * IDs are assigned to the state data upon insertion State.
         * StateCache that are not in a State have an ID of -1.
         *
         * StateCache present in different trees may have identical IDs.
         *
         * IDs are used for the efficient implementation of State. Normally,
         * you should not make use of frame IDs yourself.
         *
         * @return An integer ID for the frame.
         */
        // inline int getID() const { return _id; }

        /**
         * @brief The number of doubles allocated by this StateCache in
         * each State object.
         *
         * @return The number of doubles allocated by the StateCache
         */
        virtual size_t size () const = 0;

        /**
         * @brief this creates a deep copy of this cache
         */
        virtual rw::core::Ptr< StateCache > clone () const = 0;

      protected:
        StateCache ();

    };
    %template(StateCachePtr) rw::core::Ptr<StateCache>;
    OWNEDPTR(StateCache)
//################ Stateless
    /**
     * @brief interface for a stateless or typically a part stateless class.
     */
    class Stateless
    {
      public:

        //! destructor
        virtual ~Stateless () {}

        /**
         * @brief initialize this stateless data to a specific state
         * @param state [in] the state in which to register the data.
         *
         * @note the data will be registered in the state structure of the \b state
         * and any copies or other instances of the \b state will therefore also
         * contain the added states.
         */
        virtual void registerIn (State& state);

        //! register this stateless object in a statestructure.
        virtual void registerIn (rw::core::Ptr<StateStructure> state);

        //! unregisters all state data of this stateless object
        virtual void unregister ();

        /**
         * @brief Get the state structure.
         * @return the state structure.
         */
        rw::core::Ptr<StateStructure> getStateStructure ();

        #if !defined(SWIGJAVA)
            //! @copydoc getStateStructure
            const rw::core::Ptr<StateStructure> getStateStructure () const;
        #endif

        /**
         * @brief Check if object has registered its state.
         * @return true if registered, false otherwise.
         */
        bool isRegistered ();

      protected:

        /**
         * @brief constructor
         */
        Stateless ();

        //! implementations of sensor should add all their stateless data on initialization
        template< class T > void add (StatelessData< T >& data);

        /**
         * @brief Add data.
         * @param data [in] data to add.
         */
        void add (StateData* data);

        //! implementations of sensor should add all their state data on initialization
        void add (rw::core::Ptr< StateData > data);
    };
    %template(StatelessPtr) rw::core::Ptr<Stateless>;
//################ StatelessData
    namespace rw { namespace kinematics {

        /**
         * @brief class for enabling statelessness in classes that are data containers
         */
        template< class DATA > class StatelessData
        {
        public:
            /**
             * @brief constructor
             * @param dN [in] the number of elements of type DATA that should be allocated in the state.
             */
            StatelessData (int dN = 1);

            /**
             * @copydoc StatelessData(int)
             * @param cache [in] data cache.
             */
            StatelessData (int dN, rw::core::Ptr< StateCache > cache);

            //! destructor
            virtual ~StatelessData ();

            /**
             * @brief initialize this stateless data to a specific state
             * @param state [in] the state in which to register the data.
             *
             * @note the data will be registered in the state structure of the \b state
             * and any copies or other instances of the \b state will therefore also
             * contain the added states.
             */
            void init (State& state);

            /**
             * @brief get the data from the \b state
             * @param state [in] the state in which the data is saved
             * @return reference to data
             */
            DATA* getArray (const rw::kinematics::State& state);

            /**
             * @brief get the data from the \b state
             * @param state [in] the state in which the data is saved
             * @return reference to data
             */
            DATA& get (const rw::kinematics::State& state);

            /**
             * @brief get the data from the \b state
             * @param state [in] the state in which the data is saved
             * @return reference to data
             */
            const DATA& get (const rw::kinematics::State& state) const;

            /**
             * @brief get the data from the \b state
             * @param i [in] the index of the data.
             * @param state [in] the state in which the data is saved
             * @return reference to data
             */
            DATA& get (int i, const rw::kinematics::State& state);

            /**
             * @brief get the data from the \b state
             * @param i [in] the index of the data.
             * @param state [in] the state in which the data is saved
             * @return reference to data
             */
            const DATA& get (int i, const rw::kinematics::State& state) const;

            /**
             * @brief set data element in state
             * @param data [in] data to copy into state
             * @param state [in] the state in which to change data
             */
            void set (const DATA& data, rw::kinematics::State& state);

            /**
             * @brief set data element in state
             * @param data [in] data to copy into state
             * @param i [in] the index of the data.
             * @param state [in] the state in which to change data
             */
            void set (const DATA& data, int i, rw::kinematics::State& state);

            /**
             * @brief number of array elements
             * @return number of elements in array
             */
            int getN () const;

            /**
             * @brief get the cache of this statedata object. If it has no cache then
             * the returned pointer will be NULL.
             * @param state [in] state in which to get cache from.
             * @return
             */
            template< class CACHE_TYPE > CACHE_TYPE* getStateCache (rw::kinematics::State& state);

            //! @copydoc getStateCache(rw::kinematics::State&) const
            template< class CACHE_TYPE >
            CACHE_TYPE* getStateCache (const rw::kinematics::State& state) const;

            /**
             * @brief Get the state data.
             * @return state data.
             */
            rw::core::Ptr< StateData > getStateData ();
        };

    }}     // namespace rw::kinematics

//################ StateSetup
     /**
     * @brief Utility class to help construct a State
     *
     * StateSetup contains the data to share among QState objects and
     * TreeState objects, namely the assignment of offsets to frames,
     * the mapping of frame indexes to indexes in the QState,
     * the mapping of frame indexes to daf and dafparent index in
     * the TreeState,
     */
    class StateSetup
    {
      public:
        /**
         * @brief Creates an empty StateSetup
         */
        StateSetup ();

        /**
         * @brief Creates a StateSetup from a StateStructure and a number of
         * valid statedata.
         * @param version [in] the version of the StateSetup
         * @param tree [in]
         * @param stateDatas [in] a list of valid statedatas for this version
         */
        explicit StateSetup (int version, StateStructure& tree,
                             const std::vector< rw::core::Ptr< StateData > >& stateDatas);

        //! @brief destructor
        ~StateSetup ();

        /**
         * @brief The position in QState at which the configuration for \b frame
         * is stored.
         */
        inline int getOffset (const StateData& data) const;

        /**
         * @brief Get the position in cache list where \b data is stored.
         * @param data [in] the data to look for.
         * @return the id or a negative value if not found.
         */
        inline int getCacheIdx (const StateData& data) const;

        /**
         * @brief The total number of doubles allocated by all valid
         * state data in the StateSetup.
         * @return the total number of allocated doubles
         * @note This number equals the length of the QState array.
         */
        inline int size () const;

        /**
         * @brief gets the version of the StateSetup
         * @return the version of the state setup
         */
        inline int getVersion () const;

        /**
         * @brief gets the frame with index idx
         * @param id [in] the unique id of the frame
         * @return the frame with id id, else NULL
         */
        inline const Frame* getFrame (int id) const;

        #if !defined(SWIGJAVA)
            /**
             * @brief gets the frame with index idx
             * @param id [in] the unique id of the frame
             * @return the frame with id id, else NULL
             */
            inline Frame* getFrame (int id);
        #endif 

        /**
         * @brief gets the index that maps a frame parent into
         * all its daf children.
         * @param parent [in] the parent to the children list
         * @return index into the childlist array in tree state
         */
        int getChildListIdx (const Frame* parent) const;

        /**
         * @brief gets the number of valid frames in the state setup
         */
        int getMaxChildListIdx () const;

        /**
         * @brief gets the list of DAFs that are valid in this state setup
         * @return list of DAFs
         */
        const std::vector< Frame* >& getDafs () const;

        /**
         * @brief gets the index that maps a DAF into its
         * position in the TreeState daf list
         * @param daf [in] the daf frame
         * @return index into the TreeState daf list
         */
        int getDAFIdx (const Frame* daf) const;

        /**
         * @brief gets the nr of valid DAFs in the state setup
         * @return nr of valid DAFs
         */
        int getMaxDAFIdx () const;

        /**
         * @brief gets the state structure that the state setup is part
         * of.
         * @return state structure
         */
        const StateStructure* getTree () const;

        #if !defined(SWIGJAVA)
            /**
             * @brief gets the state structure that the state setup is part
             * of.
             * @return state structure
             */
            StateStructure* getTree ();
        #endif

        /**
         * @brief gets all valid state data of the state setup.
         * @return list of valid state datas
         * @note elements in the list is invalid if they are NULL
         */
        const std::vector< rw::core::Ptr< StateData > >& getStateData () const;

        /**
         * @brief Get the position in cache list where state data with \b id is stored.
         * @param id [in] state data id.
         * @return the id or a negative value if not found.
         */
        inline int getCacheIdx (int id) const;

        /**
         * @brief Get the maximum number of caches possible.
         * @return number of caches.
         */
        inline int getMaxCacheIdx () const;
    };
    %template(StateSetupPtr) rw::core::Ptr<StateSetup>;
    OWNEDPTR(StateSetup)

//################ StateStructure

    /**
     * @brief the StateStructure is responsible for handling a
     * structure of StateData and Frames
     */
    class StateStructure
    {
      public:
        /**
         * @brief constructs a frame tree with a default root frame
         * with the name "WORLD".
         */
        StateStructure ();

        /**
         * @brief destructor
         */
        virtual ~StateStructure ();

        /**
         * @brief tests if StateData data exist in this StateStructure
         *
         * @return true if the data was found, false otherwise
         *
         * @note the search includes the union of StateData in all
         * StateSetup's that belong to the StateStructure
         */
        bool has (const StateData* data);

        /**
         * @brief gets the max ID of any StateData/Frame currently in the tree.
         *
         * All frame/data IDs (see StateData::getID()) for the data of the tree are
         * lower than this number (and greater than or equal to zero).
         *
         */
        int getMaxID () const;

        /**
         * @brief adds a statedata to the frame tree and allocates memory
         * for its states. This method updates the default
         * state.
         *
         * @note Ownership is taken, the data object may not have been added to
         * any StateStructure before.
         */
        void addData (StateData* data);

        /**
         * @brief adds a statedata to the frame tree and allocates memory
         * for its states. This method updates the default
         * state.
         *
         * @note Ownership is not taken, the data object may not have been added to
         * any StateStructure before.
         */
        void addData (rw::core::Ptr< StateData > data);

        /**
         * @brief adds a frame to the frame tree and statically associates
         * the frame with the parent frame. This method updates the default
         * state.
         *
         * If parent frame is null then the frame will be attached to the world
         * frame.
         *
         */
        void addFrame (rw::core::Ptr< Frame > frame, rw::core::Ptr< Frame > parent = NULL);

        /**
         * @brief adds a DAF to the frame tree and dynamicly associates
         * the frame with a parent frame.
         *
         * @note the parent frame must exist in the frame tree and cannot be
         * NULL.
         */
        void addDAF (rw::core::Ptr< Frame > frame, rw::core::Ptr< Frame > parent);

        /**
         * @brief removes a StateData object from the tree. The actual
         * deletion of the object will happen when no States depend on
         * the StateData anymore.
         * @param data [in] pointer to object that is to be removed
         *
         * @note if the data object is a frame and it has staticly connected
         * children then the remove operation is illigal.
         *
         * @note if the data object is a frame and it has dynamicly attached
         * children then all of these will change parent relation ship such that
         * world will become their parent.
         */
        void remove (StateData* data);

        /**
         * @brief upgrades the state to the default state, but without
         * clearing the values of the state.
         * @param oldState [in] the state that should be upgraded
         * @return the upgraded state
         */
        State upgradeState (const State& oldState);

        /**
         * @brief get the default state of the frame tree
         * @return the default tree state
         */
        const State& getDefaultState () const;

        /**
         * @brief set the default state of the dynamic frame tree
         * if the given state is an older state then states valid in both
         * new and old version will be copied to the default state.
         */
        void setDefaultState (const State& state);

        /**
         * @brief All state data in the tree.
         * @return All state data in the tree
         */
        const std::vector< rw::core::Ptr< StateData > >& getStateData () const;
        /**
         * @brief All frames of the tree. Notice that elements in
         * this vector can be NULL
         *
         * @return All frames of the tree.
         */
        const std::vector< Frame* >& getFrames () const;

        /**
         * @brief All DAFs of the tree.
         *
         * @return All DAFs of the tree.
         */
        const std::vector< Frame* >& getDAFs () const;

        #if !defined(SWIGJAVA)
            /**
             * @brief get root of state structure
             * @return the root frame of the StateStructure
             */
            const Frame* getRoot () const;
        #endif

        /**
         * @brief get root of state structure
         * @return the root frame of the StateStructure
         */
        Frame* getRoot ();

        /**
         * @brief destructs all frames and statedata that is not used any more.
         * @param ID [in] used to include a specific StateData ID for destruction, defualt -1 to
         * ignore this option.
         */
        void cleanup (int ID = -1);

        /*
         * @brief test if the state structure has a specific frame
         * @param frame [in]
         * @return
         */
        // bool hasFrame(kinematics::Frame *frame);

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
        Frame* findFrame (const std::string& name) const;

        /**
         * @brief Find data from name.
         * @param name [in] the name.
         * @return the data if found.
         */
        rw::core::Ptr< StateData > findData (const std::string& name) const;


        /**
         * @brief Returns StateDataAddedEvent object needed for subscription to and firing of event
         * @return Reference to the StateDataAddedEvent
         */
        rw::core::Event< boost::function< void (const StateData*) >, const StateData* >& stateDataAddedEvent ();

        /**
         * @brief Returns StateDataRemovedEvent object needed for subscription to and firing of
         * event
         * @return Reference to the StateDataRemovedEvent
         */
        rw::core::Event< boost::function< void (const StateData*) >, const StateData* >& stateDataRemovedEvent ();

    };
    %template (StateStructurePtr) rw::core::Ptr<StateStructure>;
    %template (StateDataRemovedEvent) rw::core::Event< boost::function< void (const StateData*) >, const StateData* >;
    %template (StateDataAddedEvent) rw::core::Event< boost::function< void (const StateData*) >, const StateData* >;
    OWNEDPTR(StateStructure);
//################ TreeState
    /**
     * @brief The tree structure state of a work cell.
     *
     * The tree structure state gives access to the parent and child frames of a
     * frame.
     *
     * Currently modification of the tree structure is not supported. (This
     * implementation simply forwards to the non-public Tree data structure.)
     *
     * Tree structure states can be copied and assigned freely.
     */
    class TreeState
    {
      public:
        /**
         * @brief Construct an empty TreeState
         */
        TreeState ();

        /**
         * @brief Construct an empty TreeState
         */
        explicit TreeState (rw::core::Ptr< StateSetup > setup);

        /**
         * @brief Copy constructor.
         * @param other [in] other TreeState to copy.
         */
        explicit TreeState (const TreeState& other);

        /**
         * @brief destructor
         */
        virtual ~TreeState ();

        /**
         * @brief The parent frame of \b frame.
         *
         * If the frame has no parent, then NULL is returned.
         *
         * (We should query the modifiable part of the tree here, but that is
         * not implemented yet.)
         *
         * @param frame [in] The frame for which to retrieve the parent.
         *
         * @return The parent of the frame or NULL if the frame has no parent.
         */
        const Frame* getParent (Frame* frame) const;


        /**
         * @brief The child frames of \b frame.
         *
         * (We should query the modifiable part of the tree here, but that is
         * not implemented yet.)
         *
         * Note that we break const-correctness. We treat TreeState as an
         * implementation detail upon which an iterator interface in Frame is
         * then built.
         *
         * @param frame [in] The frame for which to retrieve the children.
         *
         * @return The children of the frame if any children exist, else NULL.
         */
        const std::vector<Frame*>& getChildren (const Frame* frame) const;

        /**
         * @brief Move a frame within the tree.
         *
         * The frame \b frame is detached from its parent and reattached to \b
         * parent. The frames \b frame and \b parent must both belong to the
         * same tree.
         *
         * We may want to later restrict this method so that only frames of
         * certain types can be moved.
         *
         * @param frame [in] The frame to move.
         * @param parent [in] The frame to attach \b frame to.
         */
        void attachFrame (Frame* frame, Frame* parent);

        /**
         * @brief gets the StateSetup used to create the TreeState
         * @return the StateSetup
         */
        rw::core::Ptr< StateSetup > getStateSetup () const;
    };
//################ Extra 

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