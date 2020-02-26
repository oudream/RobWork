// ################# ProximityStrategy 
    %nodefaultctor ProximityStrategy;
    /**
     * @brief The ProximityStrategy interface is a clean interface
     * for defining methods that are common for different proximity
     * strategy classes. Specifically adding of geometric models and
     * relating them to frames.
     */
    class ProximityStrategy {
      public:
        /**
         * @brief Adds a Proximity model of a frame to this strategy.
         *
         * The Proximity model is the one specified in the frames property
         *
         * @param object [in] the frame on which the Proximity model is to be
         * created.
         *
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual bool addModel(rw::common::Ptr<Object> object);

        /**
         * @brief Adds a Proximity model to a frame where the geometry is copied
         * in the underlying proximity strategy.
         *
         * The Proximity model is constructed from the list of faces
         *
         * @param frame [in] the frame to which the Proximity model should associate
         * @param faces [in] list of faces from which to construct the Proximity model
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual bool addModel(
            const Frame* frame,
            const Geometry& faces
            );

        /**
         * @brief Adds a Proximity model to a frame.
         *
         * The Proximity model is constructed from the list of faces
         *
         * @param frame [in] the frame to which the Proximity model should associate
         * @param faces [in] list of faces from which to construct the Proximity model
         * @param forceCopy [in] force the strategy to copy the geometry data, if false the
         * strategy may choose to store the geometry reference or not.
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual bool addModel(
            const Frame* frame,
            rw::common::Ptr<Geometry> faces,
            bool forceCopy = false
            );

        /**
         * @brief Tells whether the frame has a proximity model in the strategy
         *
         * To have a proximity model does not means that it is loaded. If a \b GeoID string from
         * which a model can be loaded it returns true as well
         *
         * @param frame [in] the frame to check for1.0/
         * @return true if a model exists or can be created
         */
        virtual bool hasModel(const Frame* frame);

        /**
           @brief Clear (remove all) model information for frame \b frame.
         */
        virtual void clearFrame(const Frame* frame);

        /**
           @brief Clear (remove all) model information for all frames.
         */
        virtual void clearFrames();

        //// new functions added to support old interface

        /**
         * @brief get the proximitymodel associated to \b frame. If no model
         * has been associated to frame then NULL is returned.
         * @param frame [in] frame for which an proximitymodel is associated
         */
        rw::common::Ptr<ProximityModel> getModel(const Frame* frame);

        //// this is the new interface based on CollisionModelInfo
        /**
         * @brief creates an empty ProximityModel
         */
		virtual rw::common::Ptr<ProximityModel> createModel() = 0;

        /**
         * @brief deallocates the memory used for \b model
         * @param model
         */
        virtual void destroyModel(ProximityModel* model) = 0;

        /**
         * @brief adds geometry to a specific proximity model. The proximity strategy copies all
         * data of the geometry.
         * @param model [in] the proximity model to add data to
         * @param geom [in] the geometry that is to be added
         */
        virtual bool addGeometry(ProximityModel* model, const Geometry& geom) = 0;

        /**
         * @brief adds geometry to a specific model. Depending on the option \b forceCopy the proximity
         * strategy may choose to copy the geometry data or use it directly.
         * @param model
         * @param geom
         * @param forceCopy
         * @return
         */
        virtual bool addGeometry(ProximityModel* model, rw::common::Ptr<Geometry> geom, bool forceCopy=false) = 0;

        /**
         * @brief removes a geometry from a specific proximity model
         */
        virtual bool removeGeometry(ProximityModel* model, const std::string& geomId) = 0;

        /**
         * @brief the list of all geometry ids that are associated to
         * the proximity model \b model is returned
         * @param model [in] the model containing the geometries
         * @return all geometry ids associated to the proximity model
         */
        virtual std::vector<std::string> getGeometryIDs(ProximityModel* model) = 0;

        /**
         * @brief Clears any stored model information
         */
        virtual void clear() = 0;

      protected:
        /**
         * @brief Creates object
         */
        ProximityStrategy();
    };
    %template(ProximityStrategyPtr) rw::common::Ptr<ProximityStrategy>;
    OWNEDPTR(ProximityStrategy);

// ################# CollisionDetector 
    /**
     * @brief result of a collision query
     */
    struct CollisionDetectorQueryResult
    {

        %extend {
            std::vector< std::pair< Frame* , Frame* > > getFramePairVector() {
                return std::vector< std::pair< Frame* , Frame* > >($self->collidingFrames.begin(), $self->collidingFrames.end());
            }
        }
        std::vector< ProximityStrategyData > _fullInfo;
    };
    %inline %{
        //! @brief types of collision query
        enum class CollisionDetectorQueryType: int{
            AllContactsFullInfo = 0,
            AllContactsNoInfo = 1,
            FirstContactFullInfo = 2,
            FirstContactNoInfo = 3
        };
    %}
    %nodefaultctor CollisionDetector;
    /**
     @brief The CollisionDetector implements an efficient way of checking a
    complete frame tree for collisions.

    It relies on a BroadPhaseDetector to do initial filtering which removes obviously not
    colliding frame pairs.

    After the filtering the remaining frame pairs are tested for collision using an
    CollisionStrategy which is a narrow phase collision detector.

    The collision detector does not dictate a specific detection
    strategy or algorithm, instead it relies on the CollisionStrategy interface for
    the actual collision checking between two frames.

    @note The collision detector is not thread safe and as such should not be used by multiple
    threads at a time.
    */
    class CollisionDetector
    {
    public:
        

        /**
         * @brief Collision detector for a workcell with only broad-phase collision checking.
         *
         * The default collision setup stored in the workcell is used for
         * broad phase collision filtering as a static filter list.
         *
         * Notice that no narrow phase checking is performed.
         * If broad-phase filter returns any frame-pairs, this will be taken as a collision.
         *
         * @param workcell [in] the workcell.
         */
        CollisionDetector(rw::common::Ptr<WorkCell> workcell);

        /**
         * @brief Collision detector for a workcell.
         *
         * The collision detector is initialized with the \b strategy .
         * Notice that the collision detector will create and store models inside the \b strategy .
         *
         * The default collision setup stored in the workcell is used for
         * broad phase collision filtering as a static filter list.
         *
         * @param workcell [in] the workcell.
         * @param strategy [in/out] the strategy for narrow-phase checking. The strategy will have models added to it.
         */
        CollisionDetector(rw::common::Ptr<WorkCell> workcell, rw::common::Ptr<CollisionStrategy> strategy);

        /**
         * @brief Collision detector for a workcell.
         * Collision checking is done for the provided collision setup alone.
         *
         * @param workcell [in] the workcell.
         * @param strategy [in/out] the strategy for narrow-phase checking. The strategy will have models added to it.
         * @param filter [in] proximity filter used to cull or filter frame-pairs that are obviously not colliding
         */
        CollisionDetector(rw::common::Ptr<WorkCell> workcell,
            rw::common::Ptr<CollisionStrategy> strategy,
            rw::common::Ptr<ProximityFilterStrategy> filter);

        /**
         * @brief Check the workcell for collisions.
         *
         * @param state [in] The state for which to check for collisions.
         * @param data [in/out] Defines parameters for the collision check, the results and also
         * enables caching inbetween calls to incollision
         * @return true if a collision is detected; false otherwise.
         */
        bool inCollision(const State& state, class ProximityData &data) const;

        /**
         * @brief Check the workcell for collisions.
         * 
         * @param state [in] The state for which to check for collisions.
         * @param result [out] If non-NULL, the pairs of colliding frames are
         * inserted in \b result.
         * @param stopAtFirstContact [in] If \b result is non-NULL and \b
         * stopAtFirstContact is true, then only the first colliding pair is
         * inserted in \b result. By default all colliding pairs are inserted.
         * 
         * @return true if a collision is detected; false otherwise.
         */
        bool inCollision(const State& state, CollisionDetectorQueryResult* result = 0, bool stopAtFirstContact = false) const;

        /**
         * @brief The broad phase collision strategy of the collision checker.
         */
        rw::common::Ptr<ProximityFilterStrategy> getProximityFilterStrategy() const;

        /**
         * @brief Get the narrow-phase collision strategy.
         * @return the strategy if set, otherwise NULL.
         */
        rw::common::Ptr<CollisionStrategy> getCollisionStrategy() const;

        /**
         * @brief Add Geometry associated to \b frame
         * 
         * The current shape of the geometry is copied, hence later changes to \b geometry has no effect
         *
         * @param frame [in] Frame to associate geometry to
         * @param geometry [in] Geometry to add
         */
        void addGeometry(Frame* frame, const rw::common::Ptr<Geometry> geometry);

        /**
         * @brief Removes geometry from CollisionDetector
         * 
         * The id of the geometry is used to match the collision model to the geometry.
         *
         * @param frame [in] The frame which has the geometry associated
         * @param geometry [in] Geometry with the id to be removed
         */
        void removeGeometry(Frame* frame, const rw::common::Ptr<Geometry> geometry);

        /**
         * @brief Removes geometry from CollisionDetector
         * 
         * The \b geometryId is used to match the collision model to the geometry.
         *
         * @param frame [in] The frame which has the geometry associated
         * @param geometryId [in] Id of geometry to be removed
         */
        void removeGeometry(Frame* frame, const std::string geometryId);
        
        //! @brief Adds rule specifying inclusion/exclusion of frame pairs in collision detection
        void addRule(const ProximitySetupRule& rule);

        //! @brief Removes rule specifying inclusion/exclusion of frame pairs in collision detection
        void removeRule(const ProximitySetupRule& rule);

        /**
         * @brief Get the computation time used in the inCollision functions.
         * @return the total computation time.
         */
        double getComputationTime() const;

        /**
         * @brief Get the number of times the inCollision functions have been called.
         * @return number of calls to inCollision functions.
         */
        int getNoOfCalls() const;

        /**
         * @brief Reset the counter for inCollision invocations and the computation timer.
         */
        void resetComputationTimeAndCount();

        /**
         * @brief return the ids of all the geometries of this frames.
         */
        std::vector<std::string> getGeometryIDs(Frame *frame);

        /**
         * @brief Returns whether frame has an associated geometry with \b geometryId.
         * @param frame [in] Frame in question
         * @param geometryId [in] Id of the geometry
         */
        bool hasGeometry(Frame* frame, const std::string& geometryId);

        %extend {
            /**
             * @brief Check the workcell for collisions.
             * 
             * @param state [in] The state for which to check for collisions.
             * @param result [out] Where to store pairs of colliding frames.
             * @param stopAtFirstContact [in] If \b result is non-NULL and \b
             * stopAtFirstContact is true, then only the first colliding pair is
             * inserted in \b result. By default all colliding pairs are inserted.
             * 
             * @return true if a collision is detected; false otherwise.
             */
            bool inCollision(const State& state, std::vector< std::pair< Frame* , Frame* > > &result, bool stopAtFirstContact = false){
                CollisionDetector::QueryResult data;
                bool success;
                success = $self->inCollision(state, &data,stopAtFirstContact);

                result = std::vector< std::pair< Frame* , Frame* > >(data.collidingFrames.begin(), data.collidingFrames.end());

                return success;
            }
        }
        %extend {
            /*
            static rw::common::Ptr<CollisionDetector> make(rw::common::Ptr<WorkCell> workcell){
                return rw::common::ownedPtr( new CollisionDetector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()) );
            }
            */

            static rw::common::Ptr<CollisionDetector> make(rw::common::Ptr<WorkCell> workcell, rw::common::Ptr<CollisionStrategy> strategy){
                return rw::common::ownedPtr( new CollisionDetector(workcell, strategy) );
            }
        }
    };
    %template (CollisionDetectorPtr) rw::common::Ptr<CollisionDetector>;
    OWNEDPTR(CollisionDetector);

// ################# CollisionSetup
    /**
     * @brief Setup for the collision checker
     *
     * The CollisionSetup contains information about
     * which frames, not be checked against each other
     */
    class CollisionSetup
    {
    public:
        /**
         * @brief Default constructor for when no excludes are described
         */
        CollisionSetup();

        /**
         @brief Constructs CollisionSetup with list of exclusions

        @param exclude [in] pairs to be excluded
        */
        explicit CollisionSetup(const std::vector<std::pair<std::string,std::string> >& exclude);

        %extend {
            /**
             @brief CollisionSetup for a list of pairs to exclude and a sequence
            of volatile frames.

            @param exclude [in] pairs to be excluded

            @param volatileFrames [in] names of frames to treat as volatile.

            @param excludeStaticPairs [in] if true exclude statically related pairs.
            */
            CollisionSetup(const std::vector<std::pair<std::string,std::string> >& exclude,
                        const std::vector<std::string>& volatileFrames,
                        bool excludeStaticPairs)
            {
                std::set < std::string> s;
                std::copy(volatileFrames.begin(),volatileFrames.end(),std::inserter(s,s.end()));
                return new CollisionSetup(exclude,s,excludeStaticPairs);
            }
        }

        void addExcludePair(std::pair< std::string, std::string >& pair);

        void removeExcludePair(std::pair< std::string, std::string >& pair);

        /**
         * @brief Returns the exclude list
         * @return the exclude list
         */
        const std::vector< std::pair< std::string, std::string > >& getExcludeList() const;

        /**
         @brief True iff the collision setup for the frame can change over
        time.
        */
        bool isVolatile(const Frame& frame) const;

        /**
         @brief True iff all statically related pairs of frames should be
        excluded.

        Note that this will exclude also statically related pairs of frames
        for which one or both of the pairs are volatile.
        */
        bool excludeStaticPairs() const;

        /**
         * @brief Combine setup of this and setup of \b b into this collision setup.
         */
        void merge(const CollisionSetup& b);

        /* TODO(kalor) implement outside of scope
         * @brief Combine setup \b a and setup \b b into a single collision setup.
         */
        //static CollisionSetup merge(const CollisionSetup& a, const CollisionSetup& b);

        static CollisionSetup get(const WorkCell& wc);
        static CollisionSetup get(rw::common::Ptr<WorkCell> wc);

        static CollisionSetup get(const rw::common::PropertyMap& map);

        static void set(const CollisionSetup& setup, rw::common::Ptr<WorkCell> wc);

        static void set(const CollisionSetup& setup, rw::common::PropertyMap& map);
    };
    %template(StringPairVector) std::vector < std::pair <std::string, std::string> >; 
    %template(StringPair) std::pair <std::string, std::string>; 
// ################# CollisionStrategy 
    //! @brief a collision pair of
    struct CollisionStrategyCollisionPair {
        //! @brief geometry index
        int geoIdxA, geoIdxB;
        /**
         *  @brief indices into the geomPrimIds array, which means that inidicies [_geomPrimIds[startIdx];_geomPrimIds[startIdx+size]]
         *  are the colliding primitives between geometries geoIdxA and geoIdxB
         */
        int startIdx, size;
    };
    %template(CollisionStrategyCollisionPairVector) std::vector<CollisionStrategyCollisionPair>;
    /**
     * @brief result of a single collision pair
     *
     * A collision result is one or all colliding triangles between two objects which may have
     * several geometries attached.
     * The collision result does not have access to the actual triangle meshes of the geometries
     * so to extract the actual contact location the user has to supply the triangles meshes of
     * the geometries himself.
     *
     */
    struct CollisionStrategyResult
        {
        //! @brief reference to the first model
        rw::common::Ptr<ProximityModel> a;

        //! @brief reference to the second model
        rw::common::Ptr<ProximityModel> b;

        //! @brief transformation from a to b
        rw::math::Transform3D<double> _aTb;

        //! @brief the collision pairs
        std::vector<CollisionStrategyCollisionPair> _collisionPairs;

        /**
         * @brief indices of triangles/primitives in geometry a and b that are colliding
         * all colliding triangle indices are in this array also those that are from different geometries
         */
        std::vector<std::pair<int, int> > _geomPrimIds;

        int _nrBVTests, _nrPrimTests;

        int getNrPrimTests();
        int getNrBVTests();

        /**
         * @brief clear all result values
         */
        void clear();
    };
    %template(IntPairVector) std::vector<std::pair<int, int> >;
    %template(IntPair) std::pair<int, int>;

    %inline %{
        //! @brief types of collision query
        enum class CollisionStrategyQueryType: int{
            FirstContact, 
            AllContacts
        };
    %}

    %nodefaultctor CollisionStrategy;
    /**
     * @brief An interface that defines methods to test collision between
     * two objects.
     */
    class CollisionStrategy : public virtual ProximityStrategy {
    public:
        %extend{
            /**
             * @brief Checks to see if two given frames @f$ \mathcal{F}_a @f$ and
             * @f$ \mathcal{F}_b @f$ are in collision
             * @param a [in] @f$ \mathcal{F}_a @f$
             * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
             * @param b [in] @f$ \mathcal{F}_b @f$
             * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
             * @param type [in] collision query type
             * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
             * colliding, false otherwise.
             */
            bool inCollision(
                const Frame* a,
                const rw::math::Transform3D<double>& wTa,
                const Frame *b,
                const rw::math::Transform3D<double>& wTb,
                CollisionStrategyQueryType type = CollisionStrategyQueryType::FirstContact)
            {
                return $self->inCollision(a,wTa,b,wTb,CollisionStrategy::QueryType(int(type)));
            }

            /**
             * @brief Checks to see if two given frames @f$ \mathcal{F}_a @f$ and
             * @f$ \mathcal{F}_b @f$ are in collision
             * @param a [in] @f$ \mathcal{F}_a @f$
             * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
             * @param b [in] @f$ \mathcal{F}_b @f$
             * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
             * @param data [in/out] caching and result container
             * @param type [in] collision query type
             * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
             * colliding, false otherwise.
             */
            bool inCollision(
                const Frame* a,
                const rw::math::Transform3D<double>& wTa,
                const Frame *b,
                const rw::math::Transform3D<double>& wTb,
                ProximityStrategyData& data,
                CollisionStrategyQueryType type = CollisionStrategyQueryType::FirstContact)
            {
                return $self->inCollision(a,wTa,b,wTb,data,CollisionStrategy::QueryType(int(type)));
            }
        }
        /**
         * @brief Checks to see if two proximity models are in collision
         * @param a [in] model 1
         * @param wTa [in] transform of model a
         * @param b [in] model 2
         * @param wTb [in] transform of model b
         * @param data [in/out] caching and result container
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        bool inCollision(
            rw::common::Ptr<ProximityModel> a,
            const rw::math::Transform3D<double>& wTa,
            rw::common::Ptr<ProximityModel> b,
            const rw::math::Transform3D<double>& wTb,
            ProximityStrategyData& data);

        struct Contact {

            // point described in object A frame
            rw::math::Vector3D<> point;

            // surface normal on object B described in object A coordinates
            rw::math::Vector3D<> normalA;
            rw::math::Vector3D<> normalB;
        };

        /**
         * @brief A collision strategy constructed from a collision tolerance
         * strategy and a resolution.
         * The constructed collision strategy considers a pair of geometries to
         * be in collision if \b strategy claim they are in collision for a
         * tolerance of \b tolerance.
        */
        static rw::common::Ptr<CollisionStrategy> make(rw::common::Ptr<CollisionToleranceStrategy> strategy,
                            double tolerance);

        /**
         * @brief A collision strategy constructed from a collision tolerance
         * strategy and a resolution.
         * The constructed collision strategy considers a pair of geometries to
         * be in collision if \b strategy claim they are in collision for a
         * tolerance of \b tolerance.
         */
        static rw::common::Ptr<CollisionStrategy> make(rw::common::Ptr<CollisionToleranceStrategy> strategy,
                            const rw::kinematics::FrameMap< double >& frameToTolerance,
                            double defaultTolerance);
    protected:

        /**
         * @brief Checks to see if two proximity models are in collision
         * @param a [in] model 1
         * @param wTa [in] transform of model a
         * @param b [in] model 2
         * @param wTb [in] transform of model b
         * @param data [in/out] caching and result container
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool doInCollision(
            rw::common::Ptr<ProximityModel> a,
            const rw::math::Transform3D<double>& wTa,
            rw::common::Ptr<ProximityModel> b,
            const rw::math::Transform3D<double>& wTb,
            ProximityStrategyData& data) = 0;

        /**
         * @brief Creates object
         */
        CollisionStrategy();
    };

    %template (CollisionStrategyPtr) rw::common::Ptr<CollisionStrategy>;
    OWNEDPTR(CollisionStrategy);
// ################# CollisionToleranceStrategy

    %nodefaultctor CollisionToleranceStrategy;
    /**
     * @brief This is a collision strategy that detects collisions between objects
     * that are closer than a specified tolerance.
     */
    class CollisionToleranceStrategy: public virtual ProximityStrategy {
    public:
        /**
         * @brief Destroys object
         */
        virtual ~CollisionToleranceStrategy();

        /**
         * @brief Checks to see if the geometry attached to two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are closer than the specified tolerance.
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        bool isWithinDistance(
            const Frame* a,
            const rw::math::Transform3D<double>& wTa,
            const Frame *b,
            const rw::math::Transform3D<double>& wTb,
            double tolerance);

        /**
         * @brief Checks to see if the geometry attached to two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are closer than the specified tolerance. Result is cached in data
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param data
         * @param distance
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        bool isWithinDistance(
            const Frame* a,
            const rw::math::Transform3D<double>& wTa,
            const Frame *b,
            const rw::math::Transform3D<double>& wTb,
            double distance,
            class ProximityStrategyData& data);

        /**
         * @brief Checks to see if two proximity models @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are closer than the specified tolerance. Result is cached in data
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision
         * @param data
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        bool isWithinDistance(
			rw::common::Ptr<ProximityModel> a,
            const rw::math::Transform3D<double>& wTa,
			rw::common::Ptr<ProximityModel> b,
            const rw::math::Transform3D<double>& wTb,
            double tolerance,
            ProximityStrategyData& data);
    	
    protected:

        /**
         * @brief Checks to see if two proximity models @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are closer than the specified tolerance. Result is cached in data
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision
         * @param data
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool doIsWithinDistance(
                    rw::common::Ptr<ProximityModel> a,
                    const math::Transform3D<>& wTa,
                    rw::common::Ptr<ProximityModel> b,
                    const math::Transform3D<>& wTb,
                    double tolerance,
                    class ProximityStrategyData& data) = 0;

    protected:
        /**
         * @brief Creates object
         */
        CollisionToleranceStrategy();
    };

    %template(CollisionToleranceStrategyPtr) rw::common::Ptr<CollisionToleranceStrategy>;
    OWNEDPTR(CollisionToleranceStrategy);
// ################# DistanceCalculator 

    /**
     * @brief The DistanceCalculator implements an efficient way of calculating
     * different distances between two objects, each represented by a frame
     *
     * A list of frame pairs is contained within the distance calculater,
     * that specifies which frames are to be checked against each other.
     * The method of used for distance calculation relies on the DistanceStrategy
     * chosen.
     *
     * The DistanceCalculator supports switching between multiple strategies
     */
    class DistanceCalculator {
      public:

        /**
         * @brief Distance calculations for a given tree, collision setup and
         * primitive distance calculator. Uses proximity strategy given by the workcell.
         *
         * \b strategy must be non-NULL.
         * \b root must be non-NULL.
         * Ownership of \b root is not taken.
         *
         * @param root [in] - the root of the Frame tree.
         * @param workcell [in] - the workcell to do the distance calculations in.
         * @param strategy [in] - the primitive strategy of distance calculations.
         * @param initial_state [in] - the work cell state to use for the initial traversal of the tree.
         */
        DistanceCalculator(Frame *root,
                            rw::common::Ptr<WorkCell> workcell,
                            rw::common::Ptr<DistanceStrategy> strategy,
                            const State& initial_state);
        /**
         * @brief Construct distance calculator for a WorkCell with an associated
         * distance calculator strategy.
         *
         * The DistanceCalculator extracts information about the tree and the
         * CollisionSetup from workcell.
         *
         * @param workcell [in] the workcell to check
         * @param strategy [in] the distance calculation strategy to use
         */
        DistanceCalculator(rw::common::Ptr<WorkCell> workcell,
            rw::common::Ptr<DistanceStrategy> strategy);

        /**
         * @brief Constructs distance calculator for a selected set of frames
         *
         * The list \b pairs specifies which frame-pairs to be used for distance checking.
         *
         * \b strategy must be non-NULL.
         *
         * Ownership of \b root is not taken.
         *
         * @param pairs [in] Pairs of frame to check
         * @param strategy [in] the distance calculation strategy to use
         */
        DistanceCalculator(const std::vector< std::pair< Frame *, Frame * > >& pairs,
            rw::common::Ptr<DistanceStrategy> strategy);

        /**
         * @brief Destructor
         */
        virtual ~DistanceCalculator();

        /**
         * @brief Calculates the distances between frames in the tree
         *
         * @param state [in] The state for which to calculate distances.
         *
         * @param result [out] If non-NULL, the distance results are written
         * to \b result.
         *
         * @return the shortest distance between frame and frame tree
         */
        DistanceStrategyResult distance(const State& state,
                                std::vector<DistanceStrategyResult>* result = 0) const;

        DistanceStrategyResult distanceOMP(const State& state,
                                    std::vector<DistanceStrategyResult>* result = 0) const;

        /**
         * @brief Calculates the distance between frame and the rest of the tree
         *
         * @param state [in] The state for which to calculate distances.
         *
         * @param frame [in] The frame for which distances are to be calculated
         *
         * @param result [out] If non-NULL, the distance results are written
         * to \b result.
         *
         * @return the shortest distance between frame and frame tree
         */
        DistanceStrategyResult distance(const State& state,
                                const Frame* frame,
                                std::vector<DistanceStrategyResult>* result = 0) const;

        /**
         * @brief Set the primitive distance calculator to \b strategy.
         *
         * \b strategy must be non-NULL.
         *
         * Ownership of the strategy is not taken.
         *
         * @param strategy [in] - the primitive distance calculator to use.
         */
        void setDistanceStrategy(rw::common::Ptr<DistanceStrategy> strategy);

        /**
         * @brief Adds distance model to frame
         *
         * The distance model is constructed based on the list of faces given.
         *
         * @param frame [in] frame to which the distance model should associate
         * @param faces [in] list of faces from which to construct the model
         * @return true if a distance model was succesfully created and linked
         * with the frame; false otherwise.
         */
        bool addDistanceModel(const Frame* frame, const Geometry& faces);

        /**
         * @brief Clears the cache of the distance models
         */
        void clearCache();

        double getComputationTime();

        int getCount();

        void resetComputationTimeAndCount();

        //void setDistanceThresholdStrategy(rw::common::Ptr<DistanceStrategy> strategy);

    };
    %template (DistanceCalculatorPtr) rw::common::Ptr<DistanceCalculator>;
    OWNEDPTR(DistanceCalculator);

// ################# DistanceStrategy 
    struct DistanceStrategyResult {
        DistanceStrategyResult();

        //! @brief reference to the first frame
        const Frame* f1;

        //! @brief reference to the second frame
        const Frame* f2;
        
        /**
         * @brief pointer to the ProximityModel containing the geometries for the first frame
         **/
        rw::common::Ptr<ProximityModel> a;
        
        /**
         * @brief pointer to the ProximityModel containing the geometries for the second frame
         **/
        rw::common::Ptr<ProximityModel> b;

        //! Closest point on f1 to f2, described in f1 reference frame
        rw::math::Vector3D<double> p1;

        //! Closest point on f2 to f1, described in >>>> \b f1 <<<<< reference frame
        rw::math::Vector3D<double> p2;

        //! @brief distance between frame f1 and frame f1
        double distance;

        //! @brief geometry index to triangle mesh A
        int geoIdxA;
        
        //! @brief geometry index to triangle mesh B
        int geoIdxB;

        //! @brief index to the first face/triangle that is the closest feature
        unsigned int idx1;
        
        //! @brief index to the second face/triangle that is the closest feature
        unsigned int idx2;

        void clear();

        //TODO(kalor) add printout
    };
    %template(DistanceStrategyResultVector) std::vector<DistanceStrategyResult>;

    %nodefaultctor DistanceStrategy;
    /**
     * @brief This is an interface that defines methods for computing the minimum distance
     * between geometric objects. If geometry objects has been related to frames (see ProximityStrategy)
     * then distance functions computing the distance between the geometry attached to frames can also be used.
     */
    class DistanceStrategy : public virtual ProximityStrategy {
      public:
        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @cond
         * @param result [out] DistanceResult to copy result into
         * @endcond
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceStrategyResult distance(const Frame* a,
                                const rw::math::Transform3D<double>& wTa,
                                const Frame* b,
                                const rw::math::Transform3D<double>& wTb);

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @cond
         * @param result [out] DistanceResult to copy result into
         * @endcond
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param data
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceStrategyResult& distance(
                                const Frame* a,
                                const rw::math::Transform3D<double>& wTa,
                                const Frame* b,
                                const rw::math::Transform3D<double>& wTb,
                                class ProximityStrategyData& data);

        /**
         * @brief Calculates the distance between two proximity models @f$ \mathcal{a} @f$ and
         * @f$ \mathcal{b} @f$
         * @cond
         * @param result [out] DistanceResult to copy result into
         * @endcond
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param data
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceStrategyResult& distance(
            rw::common::Ptr<ProximityModel> a,
            const rw::math::Transform3D<double>& wTa,
            rw::common::Ptr<ProximityModel> b,
            const rw::math::Transform3D<double>& wTb,
            class ProximityStrategyData& data);


        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ if the distance are within threshold. If the distance
         * between the frames are larger than the threshold, the result will be inaccurate.
         * @cond
         * @param result [out] DistanceResult to copy result into
         * @endcond
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param threshold [in] threshold for distance calculations
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceStrategyResult distance(const Frame* a,
                                const rw::math::Transform3D<double>& wTa,
                                const Frame* b,
                                const rw::math::Transform3D<double>& wTb,
                                double threshold);

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ if the distance are within threshold. If the distance
         * between the frames are larger than the threshold, the result will be inaccurate.
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param threshold [in] threshold for distance calculations
         * @param data
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceStrategyResult& distance(const Frame* a,
                                const rw::math::Transform3D<double>& wTa,
                                const Frame* b,
                                const rw::math::Transform3D<double>& wTb,
                                double threshold,
                                ProximityStrategyData& data);

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ if the distance are within threshold. If the distance
         * between the frames are larger than the threshold, the result will be inaccurate.
         * @cond
         * @param result [out] DistanceResult to copy result into
         * @endcond
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param threshold [in] threshold for distance calculations
         * @param data
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceStrategyResult& distance(
                rw::common::Ptr<ProximityModel> a,
                const rw::math::Transform3D<double>& wTa,
                rw::common::Ptr<ProximityModel> b,
                const rw::math::Transform3D<double>& wTb,
                double threshold,
                ProximityStrategyData& data);
    };

    %template (DistanceStrategyPtr) rw::common::Ptr<DistanceStrategy>;
    OWNEDPTR(DistanceStrategy);
// ################# DistanceMultiStrategy
    /**
     * @brief DistanceResult contains basic information about the distance
     * result between two frames.
     */
    struct DistanceMultiStrategyResult {
        //! @brief reference to the first proximity model
        rw::common::Ptr<ProximityModel> a;

        //! @brief reference to the second proximity model
        rw::common::Ptr<ProximityModel> b;

        //! Closest point on f1 to f2, described in f1 reference frame
        rw::math::Vector3D<double> p1;

        //! Closest point on f2 to f1, described in f2 reference frame
        rw::math::Vector3D<double> p2;

        //! @brief distance between frame f1 and frame f2
        double distance;

        //! Closest points on f1 to f2, described in f1 reference frame
        std::vector< rw::math::Vector3D<double> > p1s;

        /**
         * @brief Closest point on f2 to f1, IMPORTANT! NOTICE! described in
         * >>>> \b f1 <<<<< reference frame
         */
        std::vector< rw::math::Vector3D<double> > p2s;
        
        /**
         * @brief indices to the primitives which are the closest points on the first proximity model
         **/
        std::vector< int > p1prims;

        /**
         * @brief indices to the primitives which are the closest points on the second proximity model
         **/
        std::vector< int > p2prims;

        //! distances between contact points
        std::vector< double > distances;

        void clear(){
            p1s.clear();
            p2s.clear();
            p1prims.clear();
            p2prims.clear();
            distances.clear();
        }
    };
    %template(DistanceMultiStrategyResultVector) std::vector<DistanceMultiStrategyResult>;

    %nodefaultctor DistanceMultiStrategy;
    class DistanceMultiStrategy: public virtual ProximityStrategy {
    public:

        /**
         * @brief Destroys object
         */
        virtual ~DistanceMultiStrategy();

        /**
         * @brief Calculates all distances between geometry of two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] point pairs that are closer than tolerance will
         * be included in the result.
         *
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceMultiStrategyResult distances(
            const Frame* a,
            const rw::math::Transform3D<double>& wTa,
            const Frame* b,
            const rw::math::Transform3D<double>& wTb,
            double tolerance);

        /**
         * @brief Calculates all distances between geometry of two  given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] point pairs that are closer than tolerance will be included in the result.
         * @param data
         *
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceMultiStrategyResult& distances(
            const Frame* a,
            const rw::math::Transform3D<double>& wTa,
            const Frame* b,
            const rw::math::Transform3D<double>& wTb,
            double tolerance,
            ProximityStrategyData& data);

        /**
         * @copydoc doDistances
         */
        DistanceMultiStrategyResult& distances(
            rw::common::Ptr<ProximityModel> a,
            const rw::math::Transform3D<double>& wTa,
            rw::common::Ptr<ProximityModel> b,
            const rw::math::Transform3D<double>& wTb,
            double tolerance,
            ProximityStrategyData& data);
    };
    %template(DistanceMultiStrategyPtr) rw::common::Ptr<DistanceMultiStrategy>;
    OWNEDPTR(DistanceMultiStrategy);

// ################# ProximityCache

    %typemap(in) (void *value) {
            $1 = (void *) $input;
    };
    class ProximityCache{
      public:
        /**
         * @brief Constructor
         */
        ProximityCache(void *owner); //TODO(kalor) don't work in java

        /**
         * @brief Destructor
         */
        virtual ~ProximityCache();

        /**
         * @brief Returns size of cache
         * @return size
         */
        virtual size_t size() const = 0;

        /** 
         * @brief Clears cache
         */ 
        virtual void clear() = 0;
    };
    %template(ProximityCachePtr) rw::common::Ptr<ProximityCache>;
    OWNEDPTR(ProximityCache);

// ################# ProximityData

    class ProximityData {
    public:
        /**
         * @brief Default constructor.
         *
         * By default, the collision detector returns on first contact
         * with no detailed information about the collision.
         *
         * Use setCollisionQueryType to change this behaviour.
         */
        ProximityData();
        
        %extend {
            /**
             * @brief Set the type of collision query.
             *
             * The detection can perform faster if it is allowed to return
             * after detecting the first collision. Alternatively, it is
             * possible to detect all collisions if required.
             *
             * @param qtype [in] the query type.
             * @see CollisionDetector::QueryType
             */
            void setCollisionQueryType(CollisionDetectorQueryType qtype){
                $self->setCollisionQueryType(rw::proximity::CollisionDetector::QueryType(int(qtype)));
            }

                /**
             * @brief Get the collision query type.
             * @return the query type.
             * @see CollisionDetector::QueryType
             */
            CollisionDetectorQueryType getCollisionQueryType() const{
                return CollisionDetectorQueryType(int($self->getCollisionQueryType()));
            }

        }
        /**
         * @brief Detailed information about the collision.
         * @note This data is only available for some collision query types.
         * @see CollisionDetector::QueryResult
         */
        CollisionDetectorQueryResult _collisionData;

        /**
         * @brief Cached data used by the collision detector to speed up
         * consecutive queries.
         */
        rw::common::Ptr<ProximityCache> _cache;

    };

    %template (ProximityDataPtr) rw::common::Ptr<ProximityData>;

// ################# ProximityFilter
	/**
	 * @brief this class is used for fetching frame pairs using some proximity filtering strategy.
	 *
	 * The proximity filter is statefull and in the simplest case its an iterator over a set of frame pairs.
	 *
	 * The filter implementations should support early existing, to reduce computations.
	 */
	class ProximityFilter {
	public:
		/**
		 * @brief returns the next possibly colliding framepair.
		 * @return a frame pair
		 */
		virtual void pop() = 0;

		/**
		 * @brief returns the current front and pops it afterwards
		 * @return the current front element
		 */
		virtual std::pair<Frame*, Frame*> frontAndPop() = 0;

		/**
		 * @brief if there are any more possibly colliding framepairs since last
		 * call to update then this will return true, else false will be returned.
		 */
		virtual std::pair<Frame*, Frame*> front() = 0;


		/**
		 * @brief if there are any more possibly colliding framepairs since last
		 * call to update then this will return true, else false will be returned.
		 */
		virtual bool isEmpty() = 0;

        virtual ~ProximityFilter();

	};
    %template(ProximityFilterPtr) rw::common::Ptr<ProximityFilter>;

// ################# ProximityFilterStrategy
    %nodefaultctor ProximityFilterStrategy;
    /**
     * @brief describe the interface of a broad phase proximity strategy or proximity culler.
     *
     * A broadphase strategy implement heuristics or rules for finding frame pairs that
     * are possibly overlapping and excluding framepairs that are definitely not overlapping.
     *
     * The interface supports early exiting by returning frame-pairs in an iterative manor. This
     * enables efficient collision filtering at the cost of ease of use. Before acquiring sets of
     * framepairs the update function need be called. Thereafter multiple calls to next
     * will return possibly colliding frame pairs.
     *
     * \code
     *
     * Filter f = bpstrategy->update(state)
     * while(f->hasNext()){
     *  FramePair fpair = f->next();
     * 	// do collision with narrowphase strategy
     *  ...
     * }
     * \endcode
     *
     */
    class ProximityFilterStrategy {
    public:
        //! @brief Destructor
        virtual ~ProximityFilterStrategy() {};

        /**
         * @brief Reset
         * @param state [in] the state.
         */
        virtual void reset(const State& state) = 0;

        /**
         * @brief creates a FilterData object. This is used for caching relavant data between calls to update
         *
         * @return
         */
        virtual rw::common::Ptr<ProximityCache> createProximityCache() = 0;

        /**
         * @brief Do an update
         * @param state [in] the state.
         * @return
         */
        virtual rw::common::Ptr<ProximityFilter> update(const State& state) = 0;

        /**
         * @brief called once before acquirering all possibly colliding
         * frame pairs in the workcell
         * @param state [in] the state for which collision detection is performed.
         * @param data
         */
        virtual rw::common::Ptr<ProximityFilter> update(const State& state, rw::common::Ptr<ProximityCache> data) = 0;

        /**
         * @brief get the proximity setup that describe the include/exclude rules of this
         * BroadPhaseStrategy
         * @return a reference to the ProximitySetup
         */
        virtual ProximitySetup& getProximitySetup() = 0;

        /** 
         * @brief Adds geometry associated to frame
         * @param frame [in] Frame which has the geometry associated
         * @param geo [in] Geometry
         */ 
        virtual void addGeometry(Frame* frame, const rw::common::Ptr<Geometry> geo) = 0;

        /** 
         * @brief Removes the geometric model \b geo associated with
         * Frame \b frame from this strategy.
         *
         * @param frame [in] Frame which has the geometry associated
         * @param geo [in] Geometry
         */ 
        virtual void removeGeometry(Frame* frame, const rw::common::Ptr<Geometry> geo) = 0;

        /** 
         * @brief Removes the geometric model with name \b geoName and which is associated with
         * \b frame.
         *
         * @param frame [in] Frame which has the geometry associated
         * @param geoName [in] Name of geometry
         */ 
        virtual void removeGeometry(Frame* frame, const std::string& geoName) = 0;

        /**
         * @brief Adds a ProximitySetupRule
         * @param rule [in] the rule to add.
         */
        virtual void addRule(const ProximitySetupRule& rule) = 0;


        /**
         * @brief Removes a ProximitySetupRule
         * If the rule cannot be found, then noting happens.
         * @param rule [in] the rule to remove.
         */
        virtual void removeRule(const ProximitySetupRule& rule) = 0;

    };

    //! @brief smart pointer type to this class
    %template(ProximityFilterStrategyPtr) rw::common::Ptr<ProximityFilterStrategy>;
    //! @brief smart pointer type to this const class
    %template(ProximityFilterStrategyCPtr) rw::common::Ptr<const ProximityFilterStrategy>;

// ################# ProximityModel 
    class ProximityModel {
      public:
        ProximityModel(ProximityStrategy* pOwner);
        /**
         * @brief return vector of names for the geometries added to this ProximityModel
         *
         **/
        std::vector<std::string> getGeometryIDs();
        /**
         * @brief adds geometry 
         *
         * @param geom the geometry to add
         **/
        bool addGeometry(const Geometry& geom);
        
        /**
         * @brief removes a geometry from the ProximityModel
         *
         * @param geoid name of geometry to remove
         * @return bool
         **/       
        bool removeGeometry(const std::string& geoid);

        /**
         * @brief return pointer to the associated frame
         *
         **/
        Frame* getFrame();

        /**
         * @brief sets the associated frame
         *
         * @param frame frame to set
         **/     
        void setFrame(Frame* frame);

        ProximityStrategy* owner;
        
    };
    %template(ProximityModelPtr) rw::common::Ptr<ProximityModel>;
    OWNEDPTR(ProximityModel);

// ################# ProximitySetup
    class ProximitySetup
    {
    public:
        /**
         * @brief Default constructor for when no excludes are described
         */
        ProximitySetup();

        ProximitySetup(const CollisionSetup& csetup);

        /**
         @brief Constructs ProximitySetup with list of exclusions
        @param rules documentation missing !
        @cond
        @param exclude [in] pairs to be excluded
        @endcond
        */
        ProximitySetup(const std::vector<ProximitySetupRule>& rules);


        void addProximitySetupRule(const ProximitySetupRule& rule);

        void removeProximitySetupRule(const ProximitySetupRule& rule);

        /**
         * @brief Returns the exclude list
         * @return the exclude list
         */
        const std::vector<ProximitySetupRule>& getProximitySetupRules() const;

        /**
         * @brief Combine setup of this and setup of \b b into this collision setup.
         */
        void merge(const ProximitySetup& setup, const std::string& prefix);

        bool useExcludeStaticPairs() const;

        void setUseExcludeStaticPairs(bool exclude);

        bool useIncludeAll() const;

        void setUseIncludeAll(bool includeAll);

        void setLoadedFromFile(bool loaded_from_file);

        bool getLoadedFromFile() const;

        void setFileName(const std::string& file_name);

        std::string getFileName() const;

        static ProximitySetup get(const WorkCell& wc);
        static ProximitySetup get(rw::common::Ptr<WorkCell> wc);
        static ProximitySetup get(const rw::common::PropertyMap& map);

        static void set(const ProximitySetup& setup, rw::common::Ptr<WorkCell> wc);
        static void set(const ProximitySetup& setup, rw::common::PropertyMap& map);
    };

// ################# ProximitySetupRule

    /*%inline %{
        // @brief Include and Exclude identifiers 
        enum class ProximitySetupRuleType: int{INCLUDE_RULE = 1, EXCLUDE_RULE}; 
    %}*/
    %nodefaultctor ProximitySetupRule;
    /**
     * @brief Rule specifying include/exclude of frame pairs
     *
     * The rule has two patterns, pattern A and pattern B, to which frames can be matched. A pattern
     * could contain a fully specified frame name such as "Table". It can also include 
     * wild card characters such a "Robot.*" or regular expressions.
     */
    class ProximitySetupRule {
    public:
        /** @brief Include and Exclude identifiers */
		enum RuleType{INCLUDE_RULE = 1, EXCLUDE_RULE};
        
        //! @brief Constuct empty rule
        ProximitySetupRule();


        /** 
		 * @brief Constructs rule with patternA and patternB and type
		 *
		 * @param patternA [in] Pattern identifying first frame in rule
		 * @param patternB [in] Pattern identifying second frame in rule
		 * @param type documentation missing !
		 */
		ProximitySetupRule(const std::string& patternA, const std::string& patternB, RuleType type);
        
        RuleType type() const;

        /**
         * @brief Returns the string patterns used to match
         */
        std::pair<std::string, std::string> getPatterns();


        /**
         * @brief Check whether \b str1 and \b str2 matches the pattern.
         *
         * Success is defined if the first pattern matches \b str1 and the second
         * matches \b str2 or the first matches \b str2 and the second \b str1.
         */
        bool match(const std::string& str1, const std::string& str2) const;

        /**
         * @brief Check whether \b pair matches the patterns.
         *
         * Success is defined if the first pattern matches \b pair.first and the second
         * matches \b pair.second or the first matches \b pair.second and the second \b pair.first.
         */
        bool match(std::pair<std::string,std::string>& pair) const;

        /**
         * @brief Check whether \b name matches one of the patterns
         */
        bool matchOne(const std::string& name) const;

        /**
         * @brief Check whether \b str matches pattern A
         */
        bool matchPatternA(const std::string& str) const;

        /**
         * @brief Check whether \b name matches pattern B
         */
        bool matchPatternB(const std::string& str) const;
            
        //TODO(kalor) Print function



        /**
         * @brief Compares if two rules are the same
         */
        bool operator==(const ProximitySetupRule& p) const;

        /**
         * @brief Make an exclude rule for the patterns
         */
        static ProximitySetupRule makeExclude(const std::pair<std::string, std::string>& patterns);

        /**
         * @brief Make an exclude rule for the patterns
         */
        static ProximitySetupRule makeExclude(const std::string& patternA, const std::string& patternB);

        /**
         * @brief Make an include rule for the patterns
         */
        static ProximitySetupRule makeInclude(const std::pair<std::string, std::string>& patterns);

        /**
         * @brief Make an include rule for the patterns
         */
        static ProximitySetupRule makeInclude(const std::string& patternA, const std::string& patternB);
    };

    %template(ProximitySetupRuleVector) std::vector<ProximitySetupRule>;

// ################# ProximityStrategyData    
    /**
     * @brief A generic object for containing data that is essential in
     * proximity queries between two ProximityModels.
     *
     * The ProximityData object is used for Collision queries, tolerance and distance queries between
     * two ProximityModels.
     * example: collision result, cached variables for faster collision detection,
     *
     */
    %nodefaultctor ProximityStrategyData;
    class ProximityStrategyData {
    public:
        ProximityStrategyData();
        CollisionStrategyResult& getCollisionData();
        bool inCollision();

        %extend{
            void setCollisionQueryType(CollisionStrategyQueryType qtype){
                $self->setCollisionQueryType(rw::proximity::CollisionStrategy::QueryType(int(qtype)));
            }
            
            CollisionStrategyQueryType getCollisionQueryType() const{
                return CollisionStrategyQueryType(int($self->getCollisionQueryType()));
            }
        }
        
        DistanceStrategyResult& getDistanceData();
        DistanceMultiStrategyResult& getMultiDistanceData();

        //! @brief relative acceptable error
        double rel_err;
        //! @brief absolute acceptable error
        double abs_err;
    };
    OWNEDPTR(ProximityStrategyData);
    %template (ProximityStrategyDataVector) std::vector<ProximityStrategyData>;
    %template (ProximityStrategyDataPtr) rw::common::Ptr<ProximityStrategyData>;

//