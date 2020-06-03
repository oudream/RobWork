
//############### Archive
    /**
     * @brief archive interface for serializaion classes.
     */
    class Archive
    {
      public:
        //! destructor
        virtual ~Archive ();

        /**
         * @brief open file for reading and writing
         * @param filename
         */
        void open (const std::string& filename);

        /**
         * @brief initialize archive for reading and/or writing to a stream
         * @param stream [in] the stream
         */
        void open (std::iostream& stream);

        /**
         * @brief open an output stream for writing
         */
        void open (std::ostream& ofs);

        //! @brief open an inputstream for reading
        void open (std::istream& ifs);

        /**
         * @brief test if this archive is openned for manipulation. If this is false then
         * no storage will be performed.
         * @return true if Archive is ready for streaming
         */
        virtual bool isOpen () = 0;

        /**
         * @brief close the archive.
         */
        virtual void close () = 0;

        /**
         * @brief flush the archive. Anything stored in buffers will be flushed to the
         * actual media that has been openned.
         */
        virtual void flush () = 0;

        // TODO: make extension point for archives
      protected:
        //! @copydoc open(const std::string&)
        virtual void doOpenArchive (const std::string& filename) = 0;
        //! @copydoc open(std::iostream&)
        virtual void doOpenArchive (std::iostream& stream) = 0;
        //! @copydoc open(std::istream&)
        virtual void doOpenInput (std::istream& ifs) = 0;
        //! @copydoc open(std::ostream&)
        virtual void doOpenOutput (std::ostream& ofs) = 0;
    };

//############### OutputArchive
    /**
     * @brief serializable objects can be written to an output archive.
     *
     * This class define an interface for serializing data.
     *
     */
    class OutputArchive : public virtual Archive
    {
      public:
        //! @brief destructor
        virtual ~OutputArchive ();

        // utils to handle arrays
        /**
         * @brief create a serialized scope in which objects can be written
         * @param id [in] id of the scope
         * @param idDefault [in] (optional) default id to use if \b id is an empty string.
         */
        void writeEnterScope (const std::string& id, const std::string& idDefault = "");

        /**
         * @brief leave the current scope
         * @param id [in] id of the scope
         * @param idDefault [in] (optional) default id to use if \b id is an empty string.
         */
        void writeLeaveScope (const std::string& id, const std::string& idDefault = "");

        // now for the complex types, these needs to implement save/load functionality
        /**
         * @brief generic write method. This method will write any objects that are either derived
         * from the rw::common::Serializable class or where overloaded read() write() methods
         * exists.
         * @param object
         * @param id
         */
        template< class T > void write (const T& object, const std::string& id);

        /**
         * @brief Same as write(object,id) however an additional parameter is given which is the
         * default identifier to use in case id is empty.
         * @param object [in] object to serialize
         * @param id [in] identifier
         * @param id_default [in] default id
         */
        template< class T >
        void write (const T& object, const std::string& id, const std::string& id_default);

        /**
         * @brief Output stream operator
         */
        template< class T > OutputArchive& operator<< (T& dst);

      protected:
        // writing primitives to archive
        /**
         * @brief Write value \b val to archive with identifier \b id.
         * @param val [in] value to write.
         * @param id [in] identifier for the value.
         */
        virtual void doWrite (bool val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (boost::int8_t val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (boost::uint8_t val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (boost::int16_t val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (boost::uint16_t val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (boost::int32_t val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (boost::uint32_t val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (boost::int64_t val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (boost::uint64_t val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (float val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (double val, const std::string& id) = 0;
        //! @copydoc doWrite(bool,const std::string&)
        virtual void doWrite (const std::string& val, const std::string& id) = 0;

        /**
         * @brief Write vector \b val to archive with identifier \b id.
         * @param val [in] vector to write.
         * @param id [in] identifier for the vector.
         */
        virtual void doWrite (const std::vector< bool >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< boost::int8_t >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< boost::uint8_t >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< boost::int16_t >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< boost::uint16_t >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< boost::int32_t >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< boost::uint32_t >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< boost::int64_t >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< boost::uint64_t >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< float >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< double >& val, const std::string& id) = 0;
        //! @copydoc doWrite(const std::vector<bool>&,const std::string&)
        virtual void doWrite (const std::vector< std::string >& val, const std::string& id) = 0;

        /**
         * @brief Write an Eigen matrix to output.
         * @param val [in] the matrix to output.
         * @param id [in] identifier for the matrix.
         */
        virtual void doWrite (const Eigen::MatrixXd& val, const std::string& id) = 0;

        /**
         * @brief Write an Eigen vector to output.
         * @param val [in] the vector to output.
         * @param id [in] identifier for the matrix.
         */
        virtual void doWrite (const Eigen::VectorXd& val, const std::string& id) = 0;

        /**
         * @brief handles serialization of an object. The object must either be a primitive type,
         * inherit from Serializable or there must exist an overloaded method
         * rw::common::serialization::write(const T& data, class OutputArchive& oarchive, const
         * std::string& id)
         *
         * @param object [in] object to be serialized
         * @param id [in] potential id associated to the object
         */
        template< class T > void doWrite (const T& object, const std::string& id);

        /**
         * @brief Enter a scope.
         * @param id [in] identifier for the scope.
         */
        virtual void doWriteEnterScope (const std::string& id) = 0;

        /**
         * @brief Leave a scope.
         * @param id [in] identifier for the scope.
         */
        virtual void doWriteLeaveScope (const std::string& id) = 0;
    };
//############### InputArchive

    /**
     * @brief an archive interface for reading from a serialized class.
     */
    class InputArchive : public virtual Archive
    {
      public:
        //! @brief constructor
        InputArchive ();

        /**
         * @brief enter specific scope with id when reading.
         * @param id [in] id of the scope to enter
         * @param idDefault [in] the default scope id
         */
        void readEnterScope (const std::string& id, const std::string& idDefault = "");

        /**
         * @brief leave specific scope with id when reading.
         * @param id [in] id of the scope to leave
         * @param idDefault [in] the default scope id
         */
        void readLeaveScope (const std::string& id, const std::string& idDefault = "");

        // convienience wrappers for reading primitives
        //! read boolean
        bool readBool (const std::string& id);
        //! read integer
        int readInt (const std::string& id);
        //! read unsigned integer
        unsigned int readUInt (const std::string& id);
        //! read 8 bit integer
        boost::int8_t readInt8 (const std::string& id);
        //! read 8 bit unsigned integer
        boost::uint8_t readUInt8 (const std::string& id);
        //! read 64 bit integer
        boost::int64_t readInt64 (const std::string& id);
        //! read 64 bit unsigned integer
        boost::uint64_t readUInt64 (const std::string& id);
        //! read double floating point
        double readDouble (const std::string& id);
        //! read string
        std::string readString (const std::string& id);

        /**
         * @brief Read \b object with identifier \b id from archive.
         * @param object [out] the object from archive.
         * @param id [in] the identifier.
         */
        template< class T > void read (T& object, const std::string& id);

        /**
         * @copydoc read
         * @param idDefault [in] (optional) default identifier used if \b id is an empty string.
         */
        template< class T >
        void read (T& object, const std::string& id, const std::string& idDefault);

        /**
         * @brief Stream operator for streaming from archive to object.
         * @param dst [out] the object from archive.
         * @return a reference to the archive for chaining.
         */
        template< class T > InputArchive& operator>> (T& dst);

      protected:
        //! @copydoc read
        template< class T > void doRead (T& object, const std::string& id);

        // reading primitives to archive
        /**
         * @brief Read value \b val with identifier \b id from archive.
         * @param val [out] the value from archive.
         * @param id [in] the identifier.
         */
        virtual void doRead (bool& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (boost::int8_t& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (boost::uint8_t& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (boost::int16_t& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (boost::uint16_t& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (boost::int32_t& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (boost::uint32_t& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (boost::int64_t& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (boost::uint64_t& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (float& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (double& val, const std::string& id) = 0;
        //! @copydoc doRead(bool&,const std::string&)
        virtual void doRead (std::string& val, const std::string& id) = 0;

        /**
         * @brief Read vector \b val with identifier \b id from archive.
         * @param val [out] the vector from archive.
         * @param id [in] the identifier.
         */
        virtual void doRead (std::vector< bool >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< boost::int8_t >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< boost::uint8_t >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< boost::int16_t >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< boost::uint16_t >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< boost::int32_t >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< boost::uint32_t >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< boost::int64_t >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< boost::uint64_t >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< float >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< double >& val, const std::string& id) = 0;
        //! @copydoc doRead(std::vector<bool>&,const std::string&)
        virtual void doRead (std::vector< std::string >& val, const std::string& id) = 0;

        /**
         * @brief Read an Eigen matrix from input.
         * @param val [out] the result.
         * @param id [in] identifier for the matrix.
         */
        virtual void doRead (Eigen::MatrixXd& val, const std::string& id) = 0;

        /**
         * @brief Read an Eigen vector from input.
         * @param val [out] the result.
         * @param id [in] identifier for the matrix.
         */
        virtual void doRead (Eigen::VectorXd& val, const std::string& id) = 0;

        /**
         * @brief Enter a scope.
         * @param id [in] identifier for the scope.
         */
        virtual void doReadEnterScope (const std::string& id) = 0;

        /**
         * @brief Leave a scope.
         * @param id [in] identifier for the scope.
         */
        virtual void doReadLeaveScope (const std::string& id) = 0;
    };
//############### BINArchive
    /**
     * @brief archive for loading and saving serializable classes.
     */
    #if !defined(SWIGJAVA)
        class BINArchive : public InputArchive, public virtual OutputArchive
    #else
        class BINArchive : public InputArchive
    #endif
    {
      public:
        //! @brief constructor
        BINArchive ();

        /**
         * @brief Constructor.
         * @param ofs [out] output stream to write to.
         */
        BINArchive (std::ostream& ofs);

        //! destructor
        virtual ~BINArchive ();

        //! close streaming to archive
        void close ();

        //! \copydoc rw::common::Archive::flush
        void flush ();

        //! \copydoc rw::common::Archive::isOpen
        bool isOpen ();

      protected:
        void doOpenArchive (const std::string& filename);

        void doOpenArchive (std::iostream& stream);

        void doOpenOutput (std::ostream& ofs);

        void doOpenInput (std::istream& ifs);

        //////////////////// SCOPE
        // utils to handle arrays
        //! \copydoc OutputArchive::doWriteEnterScope
        void doWriteEnterScope (const std::string& id);

        //! \copydoc OutputArchive::doWriteLeaveScope
        void doWriteLeaveScope (const std::string& id);

        //! \copydoc InputArchive::doReadEnterScope
        void doReadEnterScope (const std::string& id);

        //! \copydoc InputArchive::doReadLeaveScope
        void doReadLeaveScope (const std::string& id);

        ///////////////////////// WRITING

        void doWrite (bool val, const std::string& id);
        void doWrite (boost::int8_t val, const std::string& id);
        void doWrite (boost::uint8_t val, const std::string& id);
        void doWrite (boost::int16_t val, const std::string& id);
        void doWrite (boost::uint16_t val, const std::string& id);
        void doWrite (boost::int32_t val, const std::string& id);
        void doWrite (boost::uint32_t val, const std::string& id);
        void doWrite (boost::int64_t val, const std::string& id);
        void doWrite (boost::uint64_t val, const std::string& id);
        void doWrite (float val, const std::string& id);
        void doWrite (double val, const std::string& id);
        void doWrite (const std::string& val, const std::string& id);

        void doWrite (const std::vector< bool >& val, const std::string& id);
        void doWrite (const std::vector< boost::int8_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::uint8_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::int16_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::uint16_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::int32_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::uint32_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::int64_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::uint64_t >& val, const std::string& id);
        void doWrite (const std::vector< float >& val, const std::string& id);
        void doWrite (const std::vector< double >& val, const std::string& id);
        void doWrite (const std::vector< std::string >& val, const std::string& id);

        //! @copydoc OutputArchive::doWrite(const Eigen::MatrixXd&, const std::string&)
        void doWrite (const Eigen::MatrixXd& val, const std::string& id);

        //! @copydoc OutputArchive::doWrite(const Eigen::VectorXd&, const std::string&)
        void doWrite (const Eigen::VectorXd& val, const std::string& id);


        //! @copydoc OutputArchive::doWrite(const std::vector<bool>&,const std::string&)
        template< class T > void writeValue (const std::vector< T >& val, const std::string& id);

        //! @copydoc OutputArchive::doWrite(bool,const std::string&)
        template< class T > void writeValue (const T& val, const std::string& id);

        /**
         * @brief Write a generic Eigen matrix to output.
         * @param val [in] the matrix to output.
         * @param id [in] (not used)
         */
        template< class Derived >
        void writeMatrix (const Eigen::DenseCoeffsBase< Derived, Eigen::ReadOnlyAccessors >& val,
                          const std::string& id);

        virtual void doRead (bool& val, const std::string& id);
        virtual void doRead (boost::int8_t& val, const std::string& id);
        virtual void doRead (boost::uint8_t& val, const std::string& id);
        virtual void doRead (boost::int16_t& val, const std::string& id);
        virtual void doRead (boost::uint16_t& val, const std::string& id);
        virtual void doRead (boost::int32_t& val, const std::string& id);
        virtual void doRead (boost::uint32_t& val, const std::string& id);
        virtual void doRead (boost::int64_t& val, const std::string& id);
        virtual void doRead (boost::uint64_t& val, const std::string& id);
        virtual void doRead (float& val, const std::string& id);
        virtual void doRead (double& val, const std::string& id);
        virtual void doRead (std::string& val, const std::string& id);

        virtual void doRead (std::vector< bool >& val, const std::string& id);
        virtual void doRead (std::vector< boost::int8_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::uint8_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::int16_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::uint16_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::int32_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::uint32_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::int64_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::uint64_t >& val, const std::string& id);
        virtual void doRead (std::vector< float >& val, const std::string& id);
        virtual void doRead (std::vector< double >& val, const std::string& id);
        virtual void doRead (std::vector< std::string >& val, const std::string& id);

        //! @copydoc InputArchive::doRead(Eigen::MatrixXd&, const std::string&)
        virtual void doRead (Eigen::MatrixXd& val, const std::string& id);

        //! @copydoc InputArchive::doRead(Eigen::VectorXd&, const std::string&)
        virtual void doRead (Eigen::VectorXd& val, const std::string& id);

        // template<class T>
        // void read(T& object, const std::string& id){
        //    ((InputArchive*)this)->read<T>(object, id);
        //}

        //! @copydoc InputArchive::doRead(std::vector<bool>&,const std::string&)
        template< class T > void readValue (std::vector< T >& val, const std::string& id);

        //! @copydoc InputArchive::doRead(bool&,const std::string&)
        template< class T > void readValue (T& val, const std::string& id);

        /**
         * @brief Read a generic Eigen matrix.
         * @param val [out] the result.
         * @param id [in] (not used)
         */
        template< class Derived >
        void readMatrix (Eigen::PlainObjectBase< Derived >& val, const std::string& id);
    };
//############### Cache

    /**
     * @brief This class is a template for caching
     */
    template< class KEY, class VAL > class Cache
    {
      public:
        /**
         * @brief default constructor
         */
        Cache ();

        /**
         * @brief default destructor
         */
        virtual ~Cache ();

        /**
         * @brief Tests whether a key is present in the cache
         */
        bool isInCache (const KEY& id);

        /**
         * @brief tests if the key id is in the cache
         */
        bool has (const KEY& id);

        /**
         * @brief gets the value that is associated with the key
         */
        rw::core::Ptr< VAL > get (const KEY& key);

        /**
         * @brief Ads a value to a key that was aquired at some specific
         * time. The rights to val is taken ower by this class.
         */
        void add (const KEY& key, VAL* val);

        /**
         * @brief Ads a value to a key that was aquired at some specific
         * time. The rights to value is not changed.
         */
        void add (const KEY& key, rw::core::Ptr< VAL >& val);

        /**
         * @brief remove all values-key pairs that match key
         */
        void remove (const KEY& key);

        /**
         * @brief clear all value-key pairs from this Cache
         */
        void clear ();
    };
//############### ConcatVectorIterator

    namespace rw {namespace common {
        /**
         * @brief Forward iterator for the concatenation of a pair of vectors of
         * pointers to T
         */
        template< typename T > class ConcatVectorIterator
        {
        public:

            /**
             @brief Constructor

            The iterator \b pos points into \b curr. When the iterator reaches
            the end of \b curr it then switches to the start of \b next.

            Given a pair of vectors \b curr and \b next, the start and end
            iterator for the concatenated sequence are given by
                \code
                const ConcatVectorIterator<T> begin(curr, curr->begin(), next);
                const ConcatVectorIterator<T> end(next, next->end(), 0);
                \endcode

            You can use ConcatVectorIterator for iterating through a single
            sequence by letting \b next be NULL.
            */
            ConcatVectorIterator (const std::vector< T* >* curr, std::vector< T* >::const_iterator pos, const std::vector< T* >* next);

            /**
             * @brief Reference to the T element
             */
            T& operator* () const ;

            /**
             * @brief Pointer to the T element
             */
            T* operator-> () const ;

            INCREMENT(ConcatVectorIterator&);

            /*
             * @brief Increments the position of the iterator
             * @return the ConcatVectorIterator with the value before the incrementation
             */
            //ConcatVectorIterator operator++ (int);

            /**
             * @brief Tests whether the positions of two iterators are equal
             * @param other [in] ConcatVectorIterator to compare with
             * @return true if equal
             */
            bool operator== (const ConcatVectorIterator& other) const;

            /**
             * @brief Tests whether the positions of two iterators are unequal
             * @param other [in] ConcatVectorIterator to compare with
             * @return true if unequal
             */
            bool operator!= (const ConcatVectorIterator& other) const;
        };

        /**
         * @brief Forward iterator for the concatenation of a pair of vectors of
         * pointers to T
         */
        template< typename T > class ConstConcatVectorIterator
        {
        public:
            /**
             * @brief Constructor and implicit conversion from iterators.
             *
             * All ConstConcatVectorIterator are constructed via
             * ConcatVectorIterator values or copy construction.
             */
            ConstConcatVectorIterator (ConcatVectorIterator< T > pos);

            /**
             * @brief Reference to the T element
             */
            const T& operator* () const;

            /**
             * @brief Pointer to the T element
             */
            const T* operator-> () const;

            INCREMENT(ConstConcatVectorIterator&);

            /*
             * @brief Increments the position of the iterator
             * @return the ConstConcatVectorIterator with the value before the incrementation
             */
            //ConstConcatVectorIterator operator++ (int);

            /**
             * @brief Tests whether the positions of two iterators are equal
             * @param other [in] ConstConcatVectorIterator to compare with
             * @return true if equal
             */
            bool operator== (const ConstConcatVectorIterator& other) const;

            /**
             * @brief Tests whether the positions of two iterators are unequal
             * @param other [in] ConstConcatVectorIterator to compare with
             * @return true if unequal
             */
            bool operator!= (const ConstConcatVectorIterator& other) const;
        };
    }}
//############### FileCache
    /**
     * @brief a cache that use a timestamp in combination with a key to determine the uniqueness
     * of an item in the cache.
     */
    template< class KEY, class VAL, class STAMP_T > class FileCache
    {
      public:
        /**
         * @brief default constructor
         */
        FileCache ();

        /**
         * @brief default destructor
         */
        virtual ~FileCache ();

        /**
         * @brief Tests whether a key is present in the cache
         */
        bool isInCache (const KEY& id, const STAMP_T& stamp);

        /**
         * @brief tests if the key id is in the cache
         */
        bool has (const KEY& id, const STAMP_T& stamp) ;

        /**
         * @brief gets the value that is associated with the key
         */
        rw::core::Ptr< VAL > get (const KEY& key);

        /**
         * @brief Ads a value to a key that was aquired at some specific
         * time.
         */
        void add (const KEY& key, VAL* val, const STAMP_T& stamp);

        /**
         * @brief Ads a value to a key that was aquired at some specific
         * time.
         */
        void add (const KEY& key, rw::core::Ptr< VAL > val, const STAMP_T& stamp);

        /**
         * @brief remove all values-key pairs that match key
         */
        void remove (const KEY& key);

        /**
         * @brief clear all value-key pairs from this Cache
         */
        void clear ();
    };
//############### INIArchive
    /**
     * @brief archive for loading and saving serializable classes to an ini-file format.
     */
    #if !defined(SWIGJAVA)
        class INIArchive : public InputArchive, public virtual OutputArchive
    #else
        class INIArchive : public InputArchive
    #endif
    {
      public:
        /**
         * @brief constructor
         */
        INIArchive ();

        INIArchive (const std::string& filename);

        INIArchive (std::ostream& ofs);

        //! destructor
        virtual ~INIArchive ();

        //! @brief close this archive for streaming
        void close ();

        //! @copydoc Archive::isOpen
        bool isOpen ();

        //! @copydoc Archive::flush
        void flush ();

      protected:
        //! @brief Maximum number of characters in one line.
        static const int MAX_LINE_WIDTH = 1000;

        //////////////////// SCOPE
        //! @copydoc OutputArchive::doWriteEnterScope
        void doWriteEnterScope (const std::string& id);

        //! @copydoc OutputArchive::doWriteLeaveScope
        void doWriteLeaveScope (const std::string& id);

        //! @copydoc InputArchive::doReadEnterScope
        void doReadEnterScope (const std::string& id);

        //! @copydoc InputArchive::doReadLeaveScope
        void doReadLeaveScope (const std::string& id);

        void doOpenArchive (const std::string& filename);
        void doOpenArchive (std::iostream& stream);
        void doOpenOutput (std::ostream& ofs);
        void doOpenInput (std::istream& ifs);

        ///////////////////////// WRITING
        virtual void doWrite (bool val, const std::string& id);
        void doWrite (boost::int8_t val, const std::string& id);
        void doWrite (boost::uint8_t val, const std::string& id);

        void doWrite (boost::int16_t val, const std::string& id);
        void doWrite (boost::uint16_t val, const std::string& id);
        void doWrite (boost::int32_t val, const std::string& id);
        void doWrite (boost::uint32_t val, const std::string& id);
        void doWrite (boost::int64_t val, const std::string& id);
        void doWrite (boost::uint64_t val, const std::string& id);
        void doWrite (float val, const std::string& id);
        void doWrite (double val, const std::string& id);
        void doWrite (const std::string& val, const std::string& id);

        void doWrite (const std::vector< bool >& val, const std::string& id);
        void doWrite (const std::vector< boost::int8_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::uint8_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::int16_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::uint16_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::int32_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::uint32_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::int64_t >& val, const std::string& id);
        void doWrite (const std::vector< boost::uint64_t >& val, const std::string& id);
        void doWrite (const std::vector< float >& val, const std::string& id);
        void doWrite (const std::vector< double >& val, const std::string& id);
        void doWrite (const std::vector< std::string >& val, const std::string& id);

        //! @copydoc OutputArchive::doWrite(const Eigen::MatrixXd&, const std::string&)
        void doWrite (const Eigen::MatrixXd& val, const std::string& id);

        //! @copydoc OutputArchive::doWrite(const Eigen::VectorXd&, const std::string&)
        void doWrite (const Eigen::VectorXd& val, const std::string& id);

        // template<class T>
        // void doWrite(const T& data, const std::string& id){ OutputArchive::write<T>(data,id); }

        //! @copydoc OutputArchive::doWrite(const std::vector<bool>&,const std::string&)
        template< class T > void writeValue (const std::vector< T >& val, const std::string& id);

        //! @copydoc OutputArchive::doWrite(bool,const std::string&)
        template< class T > void writeValue (const T& val, const std::string& id);

        /**
         * @brief Write a generic Eigen matrix to output.
         * @param val [in] the matrix to output.
         * @param id [in] identifier for the matrix.
         */
        template< class Derived >
        void writeMatrix (const Eigen::DenseCoeffsBase< Derived, Eigen::ReadOnlyAccessors >& val,
                          const std::string& id);


        ///////////////// READING

        /**
         * @brief Split a line in a name and a value.
         * @return a pair of strings. First is name, second is the value.
         */
        std::pair< std::string, std::string > getNameValue ();

        virtual void doRead (bool& val, const std::string& id);
        virtual void doRead (boost::int8_t& val, const std::string& id);
        virtual void doRead (boost::uint8_t& val, const std::string& id);
        virtual void doRead (boost::int16_t& val, const std::string& id);
        virtual void doRead (boost::uint16_t& val, const std::string& id);
        virtual void doRead (boost::int32_t& val, const std::string& id);
        virtual void doRead (boost::uint32_t& val, const std::string& id);
        virtual void doRead (boost::int64_t& val, const std::string& id);
        virtual void doRead (boost::uint64_t& val, const std::string& id);
        virtual void doRead (float& val, const std::string& id);
        virtual void doRead (double& val, const std::string& id);
        virtual void doRead (std::string& val, const std::string& id);
        virtual void doRead (std::vector< bool >& val, const std::string& id);
        virtual void doRead (std::vector< boost::int8_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::uint8_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::int16_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::uint16_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::int32_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::uint32_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::int64_t >& val, const std::string& id);
        virtual void doRead (std::vector< boost::uint64_t >& val, const std::string& id);
        virtual void doRead (std::vector< float >& val, const std::string& id);
        virtual void doRead (std::vector< double >& val, const std::string& id);
        virtual void doRead (std::vector< std::string >& val, const std::string& id);

        //! @copydoc InputArchive::doRead(Eigen::MatrixXd&, const std::string&)
        virtual void doRead (Eigen::MatrixXd& val, const std::string& id);

        //! @copydoc InputArchive::doRead(Eigen::VectorXd&, const std::string&)
        virtual void doRead (Eigen::VectorXd& val, const std::string& id);

        //! @copydoc InputArchive::doRead(std::vector<bool>&,const std::string&)
        template< class T > void readValue (std::vector< T >& val, const std::string& id);

        //! @copydoc InputArchive::doRead(bool&,const std::string&)
        template< class T > void readValue (T& val, const std::string& id);

        /**
         * @brief Read a generic Eigen matrix.
         * @param val [out] the result.
         * @param id [in] identifier for the matrix - gives a warning if it does not match the id in
         * the file.
         */
        template< class Derived >
        void readMatrix (Eigen::PlainObjectBase< Derived >& val, const std::string& id);

        /**
         * @brief Read one line from input.
         *
         * Line length is maximum MAX_LINE_WIDTH.
         *
         * @return true if success.
         */
        bool getLine ();
    };
//############### LogBufferedChar
    /**
     * @brief Buffers characters before writing them to the output stream.
     *
     * The size of the buffer is fixed. On overflow the behavior depends on the
     * specified OverFlowPolicy. If a single message is larger than
     * the entire content of the buffer it is truncated.
     *
     */
    class LogBufferedChar : public LogWriter
    {
      public:
        /**
         * @brief Behaviors for the OverflowPolicy
         */
        enum OverflowPolicy {
            //! Remove the first added content (circular queue)
            REMOVE_FIRST,
            //! Skip the content which does not fit input the buffer
            REMOVE_LAST,
            //! Automatically calls flush to write the buffer and the new message to the output
            //! stream. Using AUTO_FLUSH it is possible to write messages larger than the buffer
            //! size.
            AUTO_FLUSH
        };

        /**
         * @brief Constructs a LogBufferedChar
         *
         * The LogBufferedMsg keeps a reference to the stream object. Destroying
         * the stream object while the LogBufferedChar has a reference to it
         * results in undefined behavior.
         *
         * @param size [in] Size of buffer (in characters)
         * @param stream [in] Stream to write to
         * @param policy [in] Overflow policy. Default is REMOVE_FIRST
         */
        LogBufferedChar (size_t size, std::ostream* stream, OverflowPolicy policy = REMOVE_FIRST);

        /**
         * @brief Destructor
         *
         * Calls flush before destroying the object
         */
        virtual ~LogBufferedChar ();

      protected:
        /**
         * @brief Writes str to the buffer
         * @param str [in] String to write
         */
        void doWrite (const std::string& str);

        /**
         * @brief Flushes the buffer to the output stream
         */
        void doFlush ();

        /**
         * @copydoc LogWriter::setTabLevel
         */
        void doSetTabLevel (int tablevel);
    };
//############### LogBufferedMsg
    /**
     * @brief Buffers messages before writing them to the output stream.
     *
     * The size of the buffer is not fixed and will grow until flush is called.
     * To have a fixed size buffer use LogBufferedChar instead.
     */
    class LogBufferedMsg : public LogWriter
    {
      public:
        /**
         * @brief Constructs LogBufferedMsg with a target ostream
         *
         * The LogBufferedMsg keeps a reference to the stream object. Destroying
         * the stream object while the LogBufferedMsg has a reference to it
         * results in undefined behavior.
         *
         * @param stream [in] Stream to write to
         */
        LogBufferedMsg (std::ostream* stream);

        /**
         * @brief Destructor
         *
         * Calls flush before destruction
         */
        virtual ~LogBufferedMsg ();

      protected:
        /**
         * @brief Writes str to the buffer
         * @param str [in] str to write
         */
        virtual void doWrite (const std::string& str);

        /**
         * @brief Write content of buffer to output stream and flush it
         */
        virtual void doFlush ();

        /**
         * @copydoc LogWriter::setTabLevel
         */
        virtual void doSetTabLevel (int tablevel);
    };
//############### LogFileWriter
     
     /**
     * @brief Writes log output to a file
     */
    class LogFileWriter : public LogWriter
    {
      public:
        /**
         * @brief Constructs LogFileWriter writing to a file named \b filename
         *
         * Throws exception if failing to open file
         *
         * @param filename [in] Name of file
         */
        LogFileWriter (const std::string& filename);

        /**
         * @brief Destructor
         */
        ~LogFileWriter ();

      protected:
        /**
         * @copydoc LogWriter::write(const std::string&)
         */
        void doWrite (const std::string& str);

        /**
         * @brief Calls flush on the ostream
         */
        void doFlush ();

        /**
         * @copydoc LogWriter::setTabLevel(int)
         */
        void doSetTabLevel (int tabLevel);
    };
//############### LogMultiWriter
    /**
     * @brief Writes log output to multiple LogWriters
     */
    class LogMultiWriter : public LogWriter
    {
      public:
        /**
         * @brief Constructs empty LogMultiWriter
         */
        LogMultiWriter ();

        /**
         * @brief Destructor
         */
        ~LogMultiWriter ();

        /**
         * @brief Adds a LogWriter to be written to
         */
        void addWriter (rw::core::Ptr<LogWriter> writer);

      protected:
        /**
         * @copydoc LogWriter::write(const std::string&)
         */
        void doWrite (const std::string& str);

        /**
         * @brief Calls flush on the individual writers
         */
        void doFlush ();

        /**
         * @copydoc LogWriter::setTabLevel(int)
         */
        void doSetTabLevel (int tabLevel);
    };
//############### PairMap

    namespace rw { namespace common {
        /**
         * @brief a specialized mapping implementation for pairs. It uses the internal
         * structure of template T1 to provide fast O(1) lookup for mappings from a Pair
         * to anything. The order of the Pairs does not matter.
         *
         * @note A requirement is that all pairs must be registered in the same StateStructure.
         */
        template< class T1, class T2 > class PairMap
        {
            typedef std::pair< T1, T1 > Pair;

        public:
            /**
             * @brief creates a map
             */
            PairMap ();

            /**
             * @brief creates a map with an initial size of s
             * @param defaultVal [in] the default value of new instances of T
             */
            PairMap (const T2& defaultVal);

            /**
             * @brief inserts a value into the map
             * @param pair [in] the pair for which the value is to be associated
             * @param value [in] the value that is to be associated to the pair
             */
            void insert (const Pair& pair, const T2& value);

            /**
             @brief True iff a value for \b frame has been inserted in the map (or
            accessed using non-const operator[]).
            */
            bool has (const Pair& pair) const;
            
            MAPOPERATOR(T2,Pair); //T2& operator[] (const Pair& pair);

            MAP2OPERATOR(T2, T1, T1); //T2& operator() (T1 f1, T1 f2);

            /**
             * @brief Erase a pair from the map
             * @param pair [in] the pair for which to erase from the map.
             */
            void erase (const Pair& pair);

            /**
             * @brief Erase a pair from the map
             * @param f1 [in] the first frame in the pair for which to erase from the map.
             * @param f2 [in] the second frame in the pair for which to erase from the map.
             */
            void erase (T1 f1, T1 f2);

            /**
             * @brief Clear the map.
             */
            void clear ();

            /**
             * @brief Return the map size.
             * @return the number of elements in the map.
             */
            std::size_t size () const;

            /**
             * @brief Return maximum size.
             * @return the maximum number of elements that the map object can hold.
             */
            std::size_t max_size () const;

            /**
             * @brief Test whether map is empty.
             * @return whether the map container is empty, i.e. whether its size is 0.
             */
            bool empty () const;
        };
    }}
//############### ProgramOptions
    /**
     * @brief a class for parsing program command line into a PropertyMap
     */
    class ProgramOptions
    {
      public:
        /**
         * @brief Construct new set of program options.
         * @param applicationName [in] the name of the application.
         * @param version [in] the version of the application.
         */
        ProgramOptions (const std::string& applicationName, const std::string& version);
        /**
         * @brief this initialize default options that can add simple properties to the propertymap.
         */
        void initOptions ();

        /**
         * @brief add a string option that is only allowed to occur once on the command line
         * @param name [in] name of option
         * @param defval [in] the default string value if any
         * @param desc [in] description of commandline option
         */
        void addStringOption (const std::string& name, const std::string& defval,
                              const std::string& desc);

        /**
         * @brief Set \b name of option number \b i.
         * @param name [in] the name.
         * @param i [in] index of the option.
         */
        void setPositionalOption (const std::string& name, int i);

        /**
         * @brief parses input, if
         * @param argc
         * @param argv
         * @return if 0 is returned then help or an error
         */
        int parse (int argc, char** argv);

        /**
         * @brief Parses input from a string.
         * @param string [in] input line.
         * @return 0 if success.
         */
        int parse (const std::string& string);

        /**
         * @brief Get the underlying program options description from boost.
         * @return reference to options_description.
         */
        boost::program_options::options_description& getOptionDescription ();

        /**
         * @brief Get the underlying positional program options description from boost.
         * @return reference to positional_options_description.
         */
        boost::program_options::positional_options_description& getPosOptionDescription ();

        /**
         * @brief Get parsed properties in RobWork format in the form of a PropertyMap.
         * @return the property map with parsed options.
         */
        rw::core::PropertyMap getPropertyMap () { return _pmap; }

    };
//############### ScopedTime
    /**
     * @brief Times what is executed in a scope.
     *
     * Automatically calls resume on the timer given in construction and pause when destroyed.
     *
     * @note usage
     * \code
     * ...
     * long time;
     * {
     *   // put code here that is not to be timed
     *   ScopedTimer timer(time);
     *   // put code here that is to be timed
     * }
     * std::cout << "Time: " << time << std::endl;
     * \endcode
     */
    class ScopedTimer
    {
      public:
        /**
         * @brief constructor. Starts the timer
         * @param timer
         */
        ScopedTimer (Timer& timer);

        /**
         * @brief destructor, stops the timer
         */
        virtual ~ScopedTimer ();

        /**
         * @brief Returns the time wrapped in the ScopedTimer
         * @return Timer wrapped
         */
        Timer& getTimer ();
    };
//############### Serializable
    /**
     * @brief interface for defining serialization of classes. If a class cannot inherit
     * the Serializable because of non-access to code then one can instead provide
     * overloaded read/write methods to perform the serialization.
     */
    class Serializable
    {
      public:
        //! destructor
        virtual ~Serializable ();

        /**
         * Enable read-serialization of inherited class by implementing this method. Data is read
         * from iarchive and filled into this object.
         * @param iarchive [in] the InputArchive from which to read data.
         * @param id [in] The id of the serialized sobject.
         *
         * @note the id can be empty in which case the overloaded method should provide
         * a default identifier. E.g. the Vector3D class defined "Vector3D" as its default
         * id.
         */
        virtual void read (class InputArchive& iarchive, const std::string& id) = 0;

        /**
         * Enable write-serialization of inherited class by implementing this method. Data is
         * written to oarchive from this object.
         * @param oarchive [out] the OutputArchive in which data should be written.
         * @param id [in] The id of the serialized sobject.
         *
         * @note the id can be empty in which case the overloaded method should provide
         * a default identifier. E.g. the Vector3D class defined "Vector3D" as its default
         * id.
         */
        virtual void write (class OutputArchive& oarchive, const std::string& id) const = 0;
    };

//############### ThreadPool

    /**
     * @brief A thread pool that can be assigned work.
     *
     * Work is handled in a FIFO manner, and the pool is intended to be very simple and basic.
     *
     * For more complex behaviour please look at the ThreadTask type,
     * which has a higher abstraction for adding tasks to a ThreadPool.
     */
    class ThreadPool 
    { 
      public:

        /**
         * @brief Create new thread pool using the given number of threads.
         *
         * If no argument is given, the maximum number of hardware threads on the system is used
         * (number of CPUs, cores or hyperthreading units).
         *
         * If number of threads is set to 0, the addWork function will be blocking (work is executed
         * in thread that invokes addWork).
         *
         * @param threads number of threads to use - default is the number of hardware threads
         * available on the system.
         */
        ThreadPool(int threads = -1);

        //! @brief Destruct the pool and all threads (this calls the stop function first).
        virtual ~ThreadPool();
        
        //! @brief Get number of threads in the pool.
        unsigned int getNumberOfThreads() const;

        /**
         * @brief Stop processing more work in the queue, and try to stop running work if possible.
         *
         * Long-running work should check if the isStopping function returns true and shut down
         * gracefully.
         *
         * Interrupts are issued, so if there is interruption points in the WorkFunction,
         * the work should check for boost::thread_interrupted exceptions and shut down gracefully.
         */
        void stop();

        /**
         * @brief Check if work tasks are supposed to shut itself down.
         *
         * This function should be called from long-running worker functions to let them shut down
         * gracefully.
         *
         * @return true if thread should shut down.
         */
        bool isStopping();
        
        //! @brief Add work to the thread pool.
        void addWork (boost::function< void (ThreadPool*) > work);

        /**
         * @brief Get the number of current tasks in the queue (tasks are removed from queue when
         * done).
         * @return the number of current tasks.
         */
        unsigned int getQueueSize();
        
        //! @brief Wait until the task queue becomes empty.
        void waitForEmptyQueue();
    };
    %template (ThreadPoolPtr) rw::core::Ptr<ThreadPool>;
    OWNEDPTR(ThreadPool)
//############### ThreadSafeQueue
    /**
     * @brief Queue class which is thread safe, eg. multiple threads may
     * use it at the same time.
     */
    template< class T > class ThreadSafeQueue
    {
      public:
        //! constructor
        ThreadSafeQueue ();

        /**
         * @brief test if the queue is empty
         * @return true if queue is empty, false otherwise
         */
        inline bool empty ();

        /**
         * @brief add data to the queue
         * @param wp [in] data to add to queue
         */
        inline void push (T wp);

        /**
         * @brief try to pop data from the queue. If no data is available then false is returned
         * if data is available then true is returned and wp is set.
         * @param wp [out]
         * @return true is wp set, false otherwise
         */
        inline bool try_pop (T* wp);

        /**
         * @brief pop data from the queue in blocking manner. That is it will wait until
         * data is added to the queue if it is initially empty.
         * @param wp [out] data that is popped from the queue
         * @return
         */
        inline bool pop (T* wp);

        /**
         * @brief test if the queue contain a specific data value. This is slow O(N)
         * so keep that in mind when using it.
         * @param value [in] the value to compare with.
         * @return
         */
        bool has (T value);

        /**
         * @brief Pop data from the queue in blocking manner and print the element to standard
         * output.
         * @param wp [out] data that is popped from the queue.
         * @return true.
         */
        inline bool popAndPrint (T* wp);

        /**
         * @brief Get the size of the queue.
         * @return the size.
         */
        inline size_t size ();
    };
//############### ThreadSafeStack
    namespace rw { namespace common {

        /**
         * @brief Concurrent queue of WorkPiles
         *
         */
        template< class T > class ThreadSafeStack
        {
          public:
            ThreadSafeStack ();

            /**
             * @brief Check if stack is empty.
             * @return true if empty.
             */
            inline bool empty ();

            /**
             * @brief Push a new element to the stack.
             * @param wp [in] the element to add to stack.
             */
            inline void push (T wp);

            /**
             * @brief Pop element from stack, if there is any.
             * @param wp [out] the element.
             * @return true.
             */
            inline bool try_pop (T* wp);

            /**
             * @brief Pop element from stack. If empty, wait for an element to be pushed.
             * @param wp [out] the element.
             * @return true.
             */
            inline bool pop (T* wp);

            /**
             * @brief Check if given value is in stack.
             * @param value [in] the value to look for.
             * @return true if found, false otherwise.
             */
            bool has (T value);

            /**
             * @brief Get size of stack.
             * @return the current size.
             */
            inline size_t size ();
        };

    }}
//############### ThreadSafeVariable
    namespace rw { namespace common {

        /**
         * @brief A thread safe protected variable.
         *
         * This is very useful for making simple thread-safe variables, but also for synchronization
         * between threads.
         */
        template< typename T > class ThreadSafeVariable
        {
          public:
            /**
             * @brief Create new protected variable.
             * @param var [in] the initial value.
             */
            ThreadSafeVariable (const T var);

            //! @brief Destructor
            virtual ~ThreadSafeVariable ();

            /**
             * @brief Get the value.
             * @return the value.
             */
            T getVariable ();

            /**
             * @brief Change the value.
             *
             * @note If some are still waiting for the last update of the value (by using the
             * waitForUpdate() function), this function will block until they have received the previous
             * update first.
             *
             * @param var [in] the new value.
             */
            void setVariable (const T var);

            /**
             * @brief Wait for a change of the value (blocking).
             * @param previous [in] the previous value to compare with.
             *
             * @note The type, T, should implement the operator== function for comparison.
             *
             * @return the new value that is not equal to the previous value.
             */
            T waitForUpdate (T previous);

            /**
             * @brief Use the () operator to access the value.
             * @return the value.
             */
            T operator() ();

            #if !defined(SWIGPYTHON)
              /**
               * @brief Set the value using the assignment operator (same as using setVariable()).
               * @param var [in] the new value.
               */
              void operator= (const T var);
            #endif

        };
    }}
//############### ThreadTask
    /**
     * @brief A task that facilitates the use of a hierarchic tree of tasks and subtasks.
     *
     * Often parallel processing can be done at multiple levels. Typically it is not known
     * beforehand if some task can be split into multiple smaller subtasks or not. The ThreadTask
     * keeps track of the state of all its subtasks - only when all subtasks have been processed,
     * the parent task will be able to finish. Instead of finishing, it can also choose to add new
     * subtasks that depends on the result of the previously run subtasks.
     *
     * The ThreadTask can utilize a ThreadPool of arbitrary size (down to 0 threads).
     * When 0 threads are used, the addSubTask() function will be blocking and execute the work
     * immediately. If more than 0 threads are used, the addSubTask function will return
     * immediately, and the task is instead added to the work queue for processing when a thread
     * becomes available.
     *
     * There are two ways to use the ThreadTask:
     *
     *  - Use it as a grouping mechanism: here one or more subtasks can be added for parallel
     * processing. One can use the ThreadPool size to adjust the number of threads one wants to use
     * for a specific application.
     *
     *  - Subclass it and override the four standard functions to make more complex
     * "branch-and-combine-results" type of tasks.
     *
     *  The four standard functions are as follows:
     *
     *  run() is the main work unit of the ThreadTask.
     *
     *  subTaskDone() is called each time a subtask has ended.
     *
     *  idle() is called when the task has finished its run() function and all subtasks has ended.
     *
     *  done() is the final function called before the ThreadTask is ended completely.
     *
     * Please remember that the four functions can in general not be expected to be run in the same
     * thread! In the first three functions it is allowed to add new subtasks to keep the thread
     * running. In the done() function this is not allowed (it is simply ignored as the task will
     * end immediately after this function has been called).
     */
    class ThreadTask 
    {
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

        /**
         * @brief Create task that will inherit parents thread pool.
         * @param parent [in] the parent task to take the thread pool from.
         */
        ThreadTask(rw::core::Ptr<ThreadTask> parent);

        /**
         * @brief Create task that will use a specific thread pool.
         * If no thread pool is given, the task will not use parallelization.
         * @param pool [in] (optional) a pointer to the ThreadPool to use.
         */
        ThreadTask(rw::core::Ptr<ThreadPool> pool);

        /**
         * @brief Destruct this task.
         */
        virtual ~ThreadTask();

        /**
         * @brief Set which ThreadPool to use to do the actual execution of work.
         * When execution is started the pool can not be changed anymore.
         * @param pool [in] pointer to the pool
         * @return true if change was successful, false otherwise.
         */
        bool setThreadPool(rw::core::Ptr<ThreadPool> pool);

        /**
         * @brief Get the ThreadPool that is used by this task currently.
         *
         * @return pointer to the ThreadPool.
         */
        rw::core::Ptr<ThreadPool> getThreadPool();

        /**
         *  @brief Function is the first function executed to do the actual work (new subtasks can
         * be added in this function).
         */
        virtual void run ();

        /**
         * @brief Function is executed each time a subtask has finished (new subtasks can be added
           in this function). If #registerFailure is used to register failures in subtasks, this function should handle
           or propagate the failures. The default implementation of this function is as follows:
         * \verbatim
                for(const Exception& e : subtask->getExceptions()) {
                        registerFailure(e);
                }
                \endverbatim
         *
         * @param subtask [in] the subtask that just finished.
         */
        virtual void subTaskDone (ThreadTask* subtask);

        /**
         * @brief Function is executed when the task becomes idle (new subtasks can be added in
         * this function).
         */
        virtual void idle ();

        /**
         * @brief Function is executed when work is finished (at this point new subtasks can NOT be
         * added).
         */
        virtual void done ();

        /**
         * @brief Start executing the work in this task and all subtasks already added, by using the
         * assigned ThreadPool.
         * @note There is no guarantee that the parent run() function starts executing before the
         * childrens run() functions.
         * @return true if execution started successfully, false if already running or thread has
         * finished.
         */
        bool execute();

        /**
         * @brief Wait until state of task changes (blocking).
         *
         * Remember to check if the new State is the desired (for instance DONE).
         *
         * @param previous [in] the previous state (wait for changes from this)
         * @return the new TaskState
         */
        TaskState wait(ThreadTask::TaskState previous);

        //! @brief Wait until state of task changes to DONE (blocking).
        void waitUntilDone();

        /**
         * @brief Get the current state of the task (non-blocking).
         * @return the current TaskState.
         */
        TaskState getState();

        /**
         * @brief Add a child task to this task.
         *
         * This task will not end before all child tasks has ended.
         * Never call execute on the child tasks - they will be executed by the parent task.
         *
         * @param subtask the ThreadTask to add as child.
         * @return true if child was added successfully (only if task has not already ended)
         */ 
        bool addSubTask(rw::core::Ptr<ThreadTask> subtask);

        /**
         * @brief Get the subtasks currently added to the ThreadTask.
         * @return a vector of subtasks.
         */
        std::vector<rw::core::Ptr<ThreadTask> > getSubTasks();

        /**
         * @brief Choose if the thread should exit automatically when all work and children has
         * finished (this is the default).
         *
         * Remember to set this to false when there is no more work to be done by the task, to allow
         * the task to stop.
         *
         * @param enable [in] true if the thread should NOT finish automatically.
         */
        void setKeepAlive(bool keepAlive);

        /**
         * @brief Check is the task has keep alive option enabled.
         * @return true if task should be kept alive, false otherwise.
         */
        bool keepAlive();

         /**
         * @brief Mark the task as a failure by registering an exception.
         *
         * The user should override the #subTaskDone function in the parent task to
         * either handle the exceptions appropriately or propagate them further on.
         *
         * @param e [in] an exception describing the problem.
         */
        void registerFailure (const Exception& e);

        /*
         * @brief Get a list of exceptions registered in task and subtasks.
         * @return a list of exceptions.
         */
        %extend {
            std::vector<Exception> getException() const{
              std::list<Exception> init = $self->getExceptions();
              return std::vector<Exception> (init.begin(),init.end());
            }
        }
    };

    %template (ThreadTaskPtr) rw::core::Ptr<ThreadTask>;
    %template (ThreadTaskPtrVector) std::vector<rw::core::Ptr<ThreadTask> >;
    OWNEDPTR(ThreadTask)

//############### Timer
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

//############### TimerUtil
    /**
     * @brief Access of the system clock so called wall time.
     */
    class TimerUtil
    {
      public:
        /**
         * @brief Returns system clock in milli-seconds
         *
         * \warning The date/time at which this timer counts from is platform-specific, so
         * you should \b not use it for getting the calendar time. It's really only meant for
         * calculating wall time differences.
         */
        static long long currentTimeMs ();

        /**
         * @brief Returns system clock in micro-seconds.
         *
         * \warning The date/time at which this timer counts from is platform-specific, so
         * you should \b not use it for getting the calendar time. It's really only meant for
         * calculating wall time differences.
         *
         * Notice: The timer cannot hold times longer than approx. 2100second.
         */
        static long long currentTimeUs ();

        /**
         * @brief Returns system clock in seconds
         *
         * \warning The date/time at which this timer counts from is platform-specific, so
         * you should \b not use it for getting the calendar time. It's really only meant for
         * calculating wall time differences.
         */
        static double currentTime ();

        /**
         * @brief Sleeps for a period of time
         *
         * @param period [in] the time in miliseconds to sleep
         */
        static void sleepMs (int period);

        /**
         * @brief Sleeps for a period of time
         *
         * @param period [in] the time in microseconds to sleep
         */
        static void sleepUs (int period);
    };

//############### VectorIterator
   namespace rw { namespace common {
        /**
         * @brief Forward iterator for vectors of pointers to T
         */
        template< typename T > class VectorIterator
        {
        public:
            
            /**
             @brief Iterator for the element at \b pos.
            */
            explicit VectorIterator (std::vector< T* >::const_iterator pos);

            /**
             * @brief Reference to the T element
             */
            T& operator* () const;

            /**
             * @brief Pointer to the T element
             */
            T* operator-> () const;

            INCREMENT(VectorIterator&);

            /*
             * @brief Increments the position of the iterator
             * @return the VectorIterator with the value before the incrementation
             */
            //VectorIterator operator++ (int);

            /**
             * @brief Tests whether the positions of two iterators are equal
             *
             * @param other [in] VectorIterator to compare with
             *
             * @return true if equal
             */
            bool operator== (const VectorIterator& other) const;

            /**
             * @brief Tests whether the positions of two iterators are unequal
             *
             * @param other [in] VectorIterator to compare with
             *
             * @return true if unequal
             */
            bool operator!= (const VectorIterator& other) const;
        };

        /**
         * @brief Forward iterator for vectors of pointers to const T
         */
        template< typename T > class ConstVectorIterator
        {
          public:
            /**
             @brief Iterator for the element at \b pos.
            */
            explicit ConstVectorIterator (std::vector< T* >::const_iterator pos) : pos (pos) {}

            /**
             * @brief Reference to the T element
             */
            const T& operator* () const;

            /**
             * @brief Pointer to the T element
             */
            const T* operator-> () const;

            INCREMENT(ConstVectorIterator&);

            /*
             * @brief Increments the position of the iterator
             * @return the VectorIterator with the value before the incrementation
             */
            //ConstVectorIterator operator++ (int);

            /**
             * @brief Tests whether the positions of two iterators are equal
             *
             * @param other [in] VectorIterator to compare with
             *
             * @return true if equal
             */
            bool operator== (const ConstVectorIterator& other) const;

            /**
             * @brief Tests whether the positions of two iterators are unequal
             *
             * @param other [in] VectorIterator to compare with
             *
             * @return true if unequal
             */
            bool operator!= (const ConstVectorIterator& other) const;

            /**
             * @brief Implicit conversion from iterators.
             */
            ConstVectorIterator (VectorIterator< T > pos);
        };
    }} 