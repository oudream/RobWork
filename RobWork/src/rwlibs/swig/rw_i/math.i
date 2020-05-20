// ######################### Q

    /**
     * @brief Configuration vector
     */
    class Q
    {
      public:
        /**
         * @brief A configuration of vector of length \b dim.
         */
        Q(size_t dim);

        /**
         * @brief Default constructor.
         *
         * The vector will be of dimension zero.
         */
        Q();

        #if defined(SWIGJAVA)
            %apply double[] {double *};
        #endif
        /**
         * @brief Creates a Q of length \b n and initialized with values from \b values
         *
         * The method reads n values from \b values and do not check whether reading out of bounds.
         *
         * @param n [in] Length of q.
         * @param values [in] Array of values to initialize with
         */
        Q(size_t n, const double* values); 

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to \b value
         *
         * @param n [in] Length of q.
         * @param value [in] Value to initialize
         */
        Q(int n, double a0);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param ... [in] Values to initialize [q(2);q(n-1)]
         *
         */
        Q(int n, double a0, double a1);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param a2 [in] Values to initialize q(2)
         *
         */
        Q(int n, double a0, double a1, double a2);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param a2 [in] Values to initialize q(2)
         * @param a3 [in] Values to initialize q(3)
         *
         */
        Q(int n, double a0, double a1, double a2, double a3);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param a2 [in] Values to initialize q(2)
         * @param a3 [in] Values to initialize q(3)
         * @param a4 [in] Values to initialize q(4)
         * 
         */
        Q(int n, double a0, double a1, double a2, double a3, double a4);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param a2 [in] Values to initialize q(2)
         * @param a3 [in] Values to initialize q(3)
         * @param a4 [in] Values to initialize q(4)
         * @param a5 [in] Values to initialize q(5)
         * 
         */
        Q(int n, double a0, double a1, double a2, double a3, double a4, double a5);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param a2 [in] Values to initialize q(2)
         * @param a3 [in] Values to initialize q(3)
         * @param a4 [in] Values to initialize q(4)
         * @param a5 [in] Values to initialize q(5)
         * @param a6 [in] Values to initialize q(6)
         * 
         */
        Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param a2 [in] Values to initialize q(2)
         * @param a3 [in] Values to initialize q(3)
         * @param a4 [in] Values to initialize q(4)
         * @param a5 [in] Values to initialize q(5)
         * @param a6 [in] Values to initialize q(6)
         * @param a7 [in] Values to initialize q(7)
         * 
         */
        Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7);

        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param a2 [in] Values to initialize q(2)
         * @param a3 [in] Values to initialize q(3)
         * @param a4 [in] Values to initialize q(4)
         * @param a5 [in] Values to initialize q(5)
         * @param a6 [in] Values to initialize q(6)
         * @param a7 [in] Values to initialize q(7)
         * @param a8 [in] Values to initialize q(8)
         * 
         */
        Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8);
        
        /**
         * @brief Creates a Q of length \b n and initialize all values in Q to the values specified after \b n
         *
         * The number of arguments after \b n must match the number n.
         *
         * @param n [in] Length of q.
         * @param a0 [in] Value to initialize q(0)
         * @param a1 [in] Value to initialize q(1)
         * @param a2 [in] Values to initialize q(2)
         * @param a3 [in] Values to initialize q(3)
         * @param a4 [in] Values to initialize q(4)
         * @param a5 [in] Values to initialize q(5)
         * @param a6 [in] Values to initialize q(6)
         * @param a7 [in] Values to initialize q(7)
         * @param a8 [in] Values to initialize q(8)
         * @param a9 [in] Values to initialize q(9)
         * 
         */
        Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9);

        /**
         * @brief Construct a configuration vector from a std::vector
         * expression.
         *
         * @param r [in] An expression for a vector of doubles
         */
        Q(const std::vector<double>& r);

        /**
         * @brief Construct from Eigen base.
         * @param q [in] Eigen base.
         */
        Q(const Eigen::Matrix<double, -1, 1>& q);

        /**
         * @brief Returns Q of length \b n initialized with 0's
         */
        static Q zero(std::size_t n);

		//! @brief Destructor.
		virtual ~Q();

        /**
         * @brief The dimension of the configuration vector.
         */
        int size() const;

        /**
         * @brief True if the configuration is of dimension zero.
         */
        bool empty() const;

        /**
         * @brief Accessor for the internal Eigen vector state.
         */
        Eigen::Matrix<double, -1, 1>& e ();

        /**
         * @brief Extracts a sub part (range) of this Q.
         * @param start [in] Start index
         * @param cnt [in] the number of elements to include
         * @return
         */
        const Q getSubPart (size_t start, size_t cnt) const;

        /**
         * @brief Set subpart of vector.
         * @param index [in] the initial index.
         * @param part [in] the part to insert beginning from \b index.
         */
        void setSubPart (size_t index, const Q& part);

        /**
         * @brief Returns the Euclidean norm (2-norm) of the configuration
         * @return the norm
         */
        double norm2 () const;

        /**
         * @brief Returns the Manhatten norm (1-norm) of the configuration
         * @return the norm
         */
        double norm1 () const;

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the configuration
         * @return the norm
         */
        double normInf () const;

        const Q operator-() const;

        /**
         * @brief Vector subtraction.
         */
        Q operator-(const Q& b) const;
        
        /**
         * @brief Vector addition.
         */
        Q operator+(const Q& b) const;

        /**
         * @brief Scalar multiplication.
         */
        Q operator*(double s) const;

        /**
           @brief Scalar division.
         */
        Q operator/(double s) const;

        /**
         * @brief Compares whether this is less than \b q
         *
         * The less operator is defined such that the first index is the most significant. That is
         * if (*this)[0] < q[0] then true is returned. If (*this)[0] > q[0] false is returned and
         * only if (*this)[0] == q[0] is the next index considered.
         */
        bool operator< (const Q& q);

        /**
         * @brief Convert to a standard vector.
         * @param v [out] the result.
         */
        void toStdVector (std::vector< double >& v) const;

        /**
         * @brief Convert to a standard vector.
         * @return the result.
         */
        std::vector< double > toStdVector () const;

        TOSTRING(Q)
        ARRAYOPERATOR(double)
    };

    %template (QVector) std::vector<Q>;
    %template(QPair) std::pair<Q, Q>;
    %template(PairConstQConstQ) std::pair< const Q, const Q>;



    namespace rw { namespace math {
        /**
         * @brief Compares \b q1 and \b q2 for equality.
         *
         * \b q1 and \b q2 are considered equal if and only if they have equal
         * length and if q1(i) == q2(i) for all i.
         *
         * @relates Q
         *
         * @param q1 [in]
         * @param q2 [in]
         * @return True if q1 equals q2, false otherwise.
         */
        bool operator== (const Q& q1, const Q& q2);

        /**
         * @brief Inequality operator.The inverse of operator==().
         */
        inline bool operator!= (const Q& q1, const Q& q2);

        /**
         * @brief The dot product (inner product) of \b a and \b b.
         * @relates Q
         */
        double dot (const Q& a, const Q& b);

        /**
         * @brief concatenates q1 onto q2 such that the returned q has
         * the configurations of q1 in [0;q1.size()[ and has q2 in
         * [q1.size();q1.size()+q2.size()[
         * @param q1 [in] the first Q
         * @param q2 [in] the second Q
         * @return the concatenation of q1 and q2
         */
        Q concat (const Q& q1, const Q& q2);
    }}

// ######################### Vector2D
    namespace rw { namespace math {
            /**
         * @brief A 2D vector @f$ \mathbf{v}\in \mathbb{R}^2 @f$
         *
         * @f$ \robabx{i}{j}{\mathbf{v}} = \left[
         *  \begin{array}{c}
         *  v_x \\
         *  v_y
         *  \end{array}
         *  \right]
         *  @f$
         *
         *  In addition, Vector2D supports the cross product operator:
         *  v3 = cross(v1, v2)
         *
         *  Usage example:
         *  @code
         *  using namespace rw::math;
         *
         *  Vector2D<double> v1(1.0, 2.0);
         *  Vector2D<double> v2(6.0, 7.0);
         *  Vector2D<double> v3 = cross( v1, v2 );
         *  Vector2D<double> v4 = v2 - v1;
         *  @endcode
        */
        template<class T> class Vector2D
        {
          public:
            /**
             * @brief Creates a 2D vector initialized with 0's
             */
            Vector2D();

            /**
             * @brief Creates a 2D vector
             *
             * @param x [in] @f$ x @f$
             *
             * @param y [in] @f$ y @f$
             */
            Vector2D(T x, T y);

            /*TODO(kalor) find out how to implement this
             * @brief Creates a 2D vector from Eigen Vector
             * @param r [in] an Eigen Vector
             */
            /*template <class R>
            Vector2D(const Eigen::MatrixBase<R>& r);*/

            /**
             @brief Returns Eigen vector equivalent to *this.
            */
            Eigen::Matrix<T, 2, 1> e() const;

            /**
             @brief The dimension of the vector (i.e. 2).

            This method is provided to help support generic algorithms using
            size() and operator[].
            */
            size_t size() const;

            /**
             * @brief returns the counter clock-wise angle between
             * this vector and the x-axis vector (1,0). The angle
             * returned will be in the interval [-Pi,Pi]
             */
            double angle();

            /**
             * @brief Returns the Euclidean norm (2-norm) of the vector
             * @return the norm
             */
            T norm2();

            /**
             * @brief Returns the Manhatten norm (1-norm) of the vector
             * @return the norm
             */
            T norm1();

            /**
             * @brief Returns the infinte norm (\f$\inf\f$-norm) of the vector
             * @return the norm
             */         
            T normInf();

            TOSTRING(rw::math::Vector2D<T>)
            ARRAYOPERATOR(T)

        };

    }}

    %template (Vector2Df) rw::math::Vector2D<float>;
    %template (Vector2Dd) rw::math::Vector2D<double>;
    %template (Vector2DdVector) std::vector<rw::math::Vector2D<double> >;

// ######################### Vector3D

    namespace rw { namespace math {
            /**
         * @brief A 3D vector @f$ \mathbf{v}\in \mathbb{R}^3 @f$
         *
         * @f$ \robabx{i}{j}{\mathbf{v}} = \left[
         *  \begin{array}{c}
         *  v_x \\
         *  v_y \\
         *  v_z
         *  \end{array}
         *  \right]
         *  @f$
         *
         *  Usage example:
         *
         *  \code
         *  const Vector3D<double> v1(1.0, 2.0, 3.0);
         *  const Vector3D<double> v2(6.0, 7.0, 8.0);
         *  const Vector3D<double> v3 = cross(v1, v2);
         *  const double d = dot(v1, v2);
         *  const Vector3D<double> v4 = v2 - v1;
         *  \endcode
         */
        template<class T> class Vector3D
        {
          public:
            /**
             * @brief Creates a 3D vector initialized with 0's
             */
            Vector3D();

            /**
             * @brief Creates a 3D vector
             * @param x [in] @f$ x @f$
             * @param y [in] @f$ y @f$
             * @param z [in] @f$ z @f$
             */
            Vector3D(T x, T y, T z);

            /*TODO(kalor) find out how to implement this
             * @brief Creates a 3D vector from vector_expression
             *
             * @param r [in] an Eigen Vector
             */
            //template< class R > explicit Vector3D (const Eigen::MatrixBase< R >& r);

            /**
             * @brief Returns Eigen vector with the values of *this
             */
            const Eigen::Matrix< T, 3, 1 > e () const;

            /**
             * @brief The dimension of the vector (i.e. 3).
             * This method is provided to help support generic algorithms using
             * size() and operator[].
            */
            size_t size() const;

            /**
             * @brief Scalar division.
             */
            const Vector3D< T > operator/ (T s) const;

            /**
             * @brief Scalar multiplication.
             */
            const Vector3D operator*(T scale) const;

            /**
             * @brief Vector addition.
             */
            Vector3D operator+(const Vector3D& other) const;

            /**
             * @brief Vector subtraction.
             */
            Vector3D operator-(const Vector3D& other) const;

            /**
             * @brief Unary minus.
             */
            const Vector3D< T > operator- () const;

            /**
             * @brief Compare with \b b for equality.
             * @param b [in] other vector.
             * @return True if a equals b, false otherwise.
             */
            bool operator==(const Vector3D& q);
            
            /**
             * @brief Returns the Euclidean norm (2-norm) of the vector
             * @return the norm
             */
            T norm2();

            /**
             * @brief Returns the Manhatten norm (1-norm) of the vector
             * @return the norm
             */
            T norm1();

            /**
             * @brief Returns the infinte norm (\f$\inf\f$-norm) of the vector
             * @return the norm
             */
            T normInf(); 

            /**
             * @brief Get zero vector.
             * @return vector.
             */
            static Vector3D<T> zero();

            /**
             * @brief Get x vector (1,0,0)
             * @return vector.
             */
            static Vector3D<T> x();

            /**
             * @brief Get y vector (0,1,0)
             * @return vector.
             */
            static Vector3D<T> y();

            /**
             * @brief Get z vector (0,0,1)
             * @return vector.
             */
            static Vector3D<T> z();
            
            TOSTRING(rw::math::Vector3D<T>)
            ARRAYOPERATOR(T)

            %extend {
                #if defined(SWIGPYTHON)
                        Wrench6D<T> multiply(const Wrench6D<T>& bV) { return (*$self)*bV; }
                #endif
            };
        };
    }}
    %template (Vector3Dd) rw::math::Vector3D<double>;
    %template (Vector3Df) rw::math::Vector3D<float>;
    %template (Vector3DdVector) std::vector< rw::math::Vector3D<double> >;
    %template (Vector3DfVector) std::vector< rw::math::Vector3D<float> >;

// ######################### Rotation3D
    namespace rw { namespace math {
        /**
         * @brief A 3x3 rotation matrix \f$ \mathbf{R}\in SO(3) \f$
         *
         * @f$
         *  \mathbf{R}=
         *  \left[
         *  \begin{array}{ccc}
         *  {}^A\hat{X}_B & {}^A\hat{Y}_B & {}^A\hat{Z}_B
         *  \end{array}
         *  \right]
         *  =
         *  \left[
         *  \begin{array}{ccc}
         *  r_{11} & r_{12} & r_{13} \\
         *  r_{21} & r_{22} & r_{23} \\
         *  r_{31} & r_{32} & r_{33}
         *  \end{array}
         *  \right]
         * @f$
         */
        template<class T> class Rotation3D
        {
        public:
            /**
             * @brief A rotation matrix with uninitialized storage.
             */
            Rotation3D();

            /**
             * @brief Constructs an initialized 3x3 rotation matrix
             *
             * @param r11 \f$ r_{11} \f$
             * @param r12 \f$ r_{12} \f$
             * @param r13 \f$ r_{13} \f$
             * @param r21 \f$ r_{21} \f$
             * @param r22 \f$ r_{22} \f$
             * @param r23 \f$ r_{23} \f$
             * @param r31 \f$ r_{31} \f$
             * @param r32 \f$ r_{32} \f$
             * @param r33 \f$ r_{33} \f$
             *
             * @f$
             *  \mathbf{R} =
             *  \left[
             *  \begin{array}{ccc}
             *  r_{11} & r_{12} & r_{13} \\
             *  r_{21} & r_{22} & r_{23} \\
             *  r_{31} & r_{32} & r_{33}
             *  \end{array}
             *  \right]
             * @f$
             */
            Rotation3D(T v0,T v1,T v2,
                        T v3,T v4,T v5,
                        T v6,T v7,T v8);

            /**
             * @brief Constructs an initialized 3x3 rotation matrix
             * @f$ \robabx{a}{b}{\mathbf{R}} =
             * \left[
             *  \begin{array}{ccc}
             *   \robabx{a}{b}{\mathbf{i}} & \robabx{a}{b}{\mathbf{j}} & \robabx{a}{b}{\mathbf{k}}
             *  \end{array}
             * \right]
             * @f$
             *
             * @param i @f$ \robabx{a}{b}{\mathbf{i}} @f$
             * @param j @f$ \robabx{a}{b}{\mathbf{j}} @f$
             * @param k @f$ \robabx{a}{b}{\mathbf{k}} @f$
             */
            Rotation3D(
                const rw::math::Vector3D<T>& i,
                const rw::math::Vector3D<T>& j,
                const rw::math::Vector3D<T>& k);
                        

            explicit Rotation3D(const rw::math::Rotation3D<T>& R);

            /**
             * @brief Constructs a 3x3 rotation matrix set to identity
             * @return a 3x3 identity rotation matrix
             *
             * @f$
             * \mathbf{R} =
             * \left[
             * \begin{array}{ccc}
             * 1 & 0 & 0 \\
             * 0 & 1 & 0 \\
             * 0 & 0 & 1
             * \end{array}
             * \right]
             * @f$
             */
            static const rw::math::Rotation3D<T>& identity();
            
            /**
             * @brief Normalizes the rotation matrix to satisfy SO(3).
             *
             * Makes a normalization of the rotation matrix such that the columns
             * are normalized and othogonal s.t. it belongs to SO(3).
             */
            void normalize();
                
            /**
             * @brief Returns the i'th row of the rotation matrix
             *
             * @param i [in] Index of the row to return. Only valid indices are 0, 1 and 2.
             */
            rw::math::Vector3D<T> getRow(size_t i) const;

            /**
             * @brief Returns the i'th column of the rotation matrix
             * @param i [in] Index of the column to return. Only valid indices are 0, 1 and 2.
             */
            rw::math::Vector3D<T> getCol(size_t i) const;

            /**
             * @brief Comparison operator.
             *
             * The comparison operator makes a element wise comparison.
             * Returns true only if all elements are equal.
             *
             * @param rhs [in] Rotation to compare with
             * @return True if equal.
             */
            bool operator==(const rw::math::Rotation3D<T> &rhs) const;

            /**
             * @brief Compares rotations with a given precision
             *
             * Performs an element wise comparison. Two elements are considered equal if the difference
             * are less than precision.
             *
             * @param rot [in] Rotation to compare with
             * @param precision [in] The precision to use for testing
             *
             * @return True if all elements are less than precision apart.
             */
            bool equal(const rw::math::Rotation3D<T>& rot, T precision) const;

            /**
             * @brief Verify that this rotation is a proper rotation
             *
             * @return True if this rotation is considered a proper rotation
             */
            bool isProperRotation() const;

            /**
             * @brief Verify that this rotation is a proper rotation
             *
             * @return True if this rotation is considered a proper rotation
             */
            bool isProperRotation(T precision) const;

            /**
             * @brief Returns a Eigen 3x3 matrix @f$ \mathbf{M}\in SO(3)
             * @f$ that represents this rotation
             *
             * @return @f$ \mathbf{M}\in SO(3) @f$
             */
            Eigen::Matrix<T,3,3> e() const;

            #if !defined(SWIGJAVA)
                /**
                 * @brief Calculates \f$ \robabx{a}{c}{\mathbf{R}} =
                 * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{R}} \f$
                 *
                 * @param bRc [in] \f$ \robabx{b}{c}{\mathbf{R}} \f$
                 *
                 * @return \f$ \robabx{a}{c}{\mathbf{R}} \f$
                 */
                rw::math::Rotation3D<T> operator*(const rw::math::Rotation3D<T>& bRc) const;
            
                /**
                 * @brief Calculates \f$ \robabx{a}{c}{\mathbf{v}} =
                 * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{v}} \f$
                 *
                 * @param bVc [in] \f$ \robabx{b}{c}{\mathbf{v}} \f$
                 * @return \f$ \robabx{a}{c}{\mathbf{v}} \f$
                 */
                rw::math::Vector3D<T> operator*(const rw::math::Vector3D<T>& bVc) const;

            #endif
            /**
             * @brief Creates a skew symmetric matrix from a Vector3D. Also
             * known as the cross product matrix of v.
             *
             * @relates Rotation3D
             *
             * @param v [in] vector to create Skew matrix from
             */
            static rw::math::Rotation3D<T> skew(const rw::math::Vector3D<T>& v);

            
            /**
             *  @brief Write to \b result the product \b a * \b b.
             */
            static void multiply (const rw::math::Rotation3D< T >& a, const rw::math::Rotation3D< T >& b,
                                rw::math::Rotation3D< T >& result);

            /**
             *  @brief Write to \b result the product \b a * \b b.
             */
            static void multiply (const rw::math::Rotation3D< T >& a, const Vector3D< T >& b,
                                Vector3D< T >& result);

            /**
             * @brief Calculates \f$ \robabx{a}{c}{\mathbf{R}} =
             * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{R}} \f$
             *
             * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
             *
             * @param bRc [in] \f$ \robabx{b}{c}{\mathbf{R}} \f$
             *
             * @return \f$ \robabx{a}{c}{\mathbf{R}} \f$
             */
            static const rw::math::Rotation3D< T > multiply (const rw::math::Rotation3D< T >& aRb,
                                                const rw::math::Rotation3D< T >& bRc);

            /**
             * @brief Calculates \f$ \robabx{a}{c}{\mathbf{v}} =
             * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{v}} \f$
             *
             * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
             * @param bVc [in] \f$ \robabx{b}{c}{\mathbf{v}} \f$
             * @return \f$ \robabx{a}{c}{\mathbf{v}} \f$
             */
            static const Vector3D< T > multiply (const rw::math::Rotation3D< T >& aRb, const Vector3D< T >& bVc);

            /**
             * @brief Calculate the inverse.
             * @note This function changes the object that it is invoked on, but this is about x5 faster
             * than rot = inverse( rot )
             * @see inverse(const rw::math::Rotation3D< T > &) for the (slower) version that does not change the
             * rotation object itself.
             * @return the inverse rotation.
             */
            inline rw::math::Rotation3D< T >& inverse ();

            %extend {
                const rw::math::EAA<T> operator*(const rw::math::EAA<T>& bTKc){
                    return *((rw::math::Rotation3D<T>*)$self) * bTKc;
                }

                #if defined(SWIGPYTHON)
                        Wrench6D<T> multiply(const Wrench6D<T>& bV) { return (*$self)*bV; }
                #endif
            };  

            TOSTRING(rw::math::Rotation3D<T>)
            MATRIXOPERATOR(T)

            #if !defined(SWIGLUA)    
                %extend {
                    rw::math::InertiaMatrix<T> multiply(const rw::math::InertiaMatrix<T>& bRc) { return (*$self)*bRc; }
                };
            #endif
        };
    }}

    %template (Rotation3Dd) rw::math::Rotation3D<double>;
    %template (Rotation3Df) rw::math::Rotation3D<float>;
    %template (VectorRotation3D) std::vector< rw::math::Rotation3D<double> >;

    namespace rw { namespace math {

        /**
         * @brief Casts rw::math::Rotation3D<T> to rw::math::Rotation3D<Q>
         *
         * @relates Rotation3D
         *
         * @param rot [in] Rotation3D with type T
         * @return Rotation3D with type Q
         */
        template< class Q, class T > const rw::math::Rotation3D< Q > cast (const rw::math::Rotation3D< T >& rot);
        
        /**
         * @brief Calculates the inverse @f$ \robabx{b}{a}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}}^{-1} @f$ of a rotation matrix
         *
         * @relates Rotation3D
         *
         * @see Rotation3D::inverse() for a faster version that modifies the existing rotation object
         * instead of allocating a new one.
         *
         * @param aRb [in] the rotation matrix @f$ \robabx{a}{b}{\mathbf{R}} @f$
         *
         * @return the matrix inverse @f$ \robabx{b}{a}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}}^{-1} @f$
         *
         * @f$ \robabx{b}{a}{\mathbf{R}} = \robabx{a}{b}{\mathbf{R}}^{-1} =
         * \robabx{a}{b}{\mathbf{R}}^T @f$
         */
        template <class T>
        const rw::math::Rotation3D<T> inverse(const rw::math::Rotation3D<T>& aRb);
    }}

    %template (inverse) rw::math::inverse<double>;
    %template (inverse) rw::math::inverse<float>;
    %template (cast) rw::math::cast<float,double>;
    %template (cast) rw::math::cast<double,float>;

// ######################### Rotation3DVector
    namespace rw { namespace math {
    /**
     * @brief An abstract base class for Rotation3D parameterisations
     *
     * Classes that represents a parametrisation of a 3D rotation may inherit
     * from this class
     */
    template<class T>
    class Rotation3DVector 
    {
      public:
        /**
         * @brief Virtual destructor
         */
        virtual ~Rotation3DVector();

        /**
         * @brief Returns the corresponding 3x3 Rotation matrix
         * @return The rotation matrix
         */
        virtual const rw::math::Rotation3D<T> toRotation3D() const = 0;

      protected:
        /**
         * @brief Copy Constructor
         *
         * We allow subclasses of this class to be copied.
         */
        Rotation3DVector(const Rotation3DVector&);

        /**
         * @brief Default Constructor
         */
        Rotation3DVector();
    };
    }}

    %template (Rotation3DVectord) rw::math::Rotation3DVector<double>;
    %template (Rotation3DVectorf) rw::math::Rotation3DVector<float>;

// ######################### EAA

    namespace rw { namespace math {
        /**
         * @brief A class for representing an equivalent angle-axis rotation
         *
         * This class defines an equivalent-axis-angle orientation vector also known
         * as an @f$ \thetak @f$ vector or "axis+angle" vector
         *
         * The equivalent-axis-angle vector is the product of a unit vector @f$
         * \hat{\mathbf{k}} @f$ and an angle of rotation around that axis @f$ \theta
         * @f$
         *
         * @note given two EAA vectors @f$ \theta_1\mathbf{\hat{k}}_1 @f$ and @f$
         * \theta_2\mathbf{\hat{k}}_2 @f$ it is generally not possible to subtract
         * or add these vectors, except for the special case when @f$
         * \mathbf{\hat{k}}_1 == \mathbf{\hat{k}}_2 @f$ this is why this class does
         * not have any subtraction or addition operators
         */
        template<class T> class EAA: public Rotation3DVector<T>
        {
        public:
            /**
             * @brief Extracts Equivalent axis-angle vector from Rotation matrix
             *
             * @param R [in] A 3x3 rotation matrix @f$ \mathbf{R} @f$
             *
             * @f$
             * \theta = arccos(\frac{1}{2}(Trace(\mathbf{R})-1)=arccos(\frac{r_{11}+r_{22}+r_{33}-1}{2})
             * @f$
             *
             * @f$
             * \thetak=log(\mathbf{R})=\frac{\theta}{2 sin \theta}(\mathbf{R}-\mathbf{R}^T) =
             * \frac{\theta}{2 sin \theta}
             * \left[
             * \begin{array}{c}
             * r_{32}-r_{23}\\
             * r_{13}-r_{31}\\
             * r_{21}-r_{12}
             * \end{array}
             * \right]
             * @f$
             *
             * @f$
             * \thetak=
             * \left[
             * \begin{array}{c}
             * 0\\
             * 0\\
             * 0
             * \end{array}
             * \right]
             * @f$ if @f$ \theta = 0 @f$
             *
             * @f$
             * \thetak=\pi
             * \left[
             * \begin{array}{c}
             * \sqrt{(R(0,0)+1.0)/2.0}\\
             * \sqrt{(R(1,1)+1.0)/2.0}\\
             * \sqrt{(R(2,2)+1.0)/2.0}
             * \end{array}
             * \right]
             * @f$ if @f$ \theta = \pi @f$
             *
             */
            EAA(const rw::math::Rotation3D<T>& rot);

            /**
             * @brief Constructs an EAA vector initialized to \f$\{0,0,0\}\f$
             */
            EAA();

            /**
             * @brief Constructs an initialized EAA vector
             * @param axis [in] \f$ \mathbf{\hat{k}} \f$
             * @param angle [in] \f$ \theta \f$
             * @pre norm_2(axis) = 1
             */
            EAA(const rw::math::Vector3D<T>& axis, T angle);

            /**
             * @brief Constructs an initialized EAA vector
             * @f$ \thetak =
             * \left[\begin{array}{c}
             *    \theta k_x\\
             *    \theta k_y\\
             *    \theta k_z
             * \end{array}\right]
             * @f$
             * @param thetakx [in] @f$ \theta k_x @f$
             * @param thetaky [in] @f$ \theta k_y @f$
             * @param thetakz [in] @f$ \theta k_z @f$
             */
            EAA(T thetakx, T thetaky, T thetakz);

            /**
             * @brief Constructs an EAA vector that will rotate v1 into
             * v2. Where v1 and v2 are normalized and described in the same reference frame.
             * @param v1 [in] normalized vector
             * @param v2 [in] normalized vector
             */
            EAA(const rw::math::Vector3D<T>& v1, const rw::math::Vector3D<T>& v2);

            /**
             * @brief Constructs an initialized EAA vector
             *
             * The angle of the EAA are \f$\|eaa\|\f$ and the axis is \f$\frac{eaa}{\|eaa\|}\f$
             * @param eaa [in] Values to initialize the EAA
             */
            explicit EAA (Vector3D< T > eaa);

            //! @brief destructor
            virtual ~EAA ();

            EAA(const EAA<T>& eaa);

            /**
             * @brief Extracts the angle of rotation @f$ \theta @f$
             * @return @f$ \theta @f$
             */
            T angle () const;
        
            /**
             * @copydoc Rotation3DVector::toRotation3D()
             *
             * @f$
             * \mathbf{R} = e^{[\mathbf{\hat{k}}],\theta}=\mathbf{I}^{3x3}+[\mathbf{\hat{k}}]
             * sin\theta+[{\mathbf{\hat{k}}}]^2(1-cos\theta) = \left[ \begin{array}{ccc}
             *      k_xk_xv\theta + c\theta & k_xk_yv\theta - k_zs\theta & k_xk_zv\theta + k_ys\theta \\
             *      k_xk_yv\theta + k_zs\theta & k_yk_yv\theta + c\theta & k_yk_zv\theta - k_xs\theta\\
             *      k_xk_zv\theta - k_ys\theta & k_yk_zv\theta + k_xs\theta & k_zk_zv\theta + c\theta
             *    \end{array}
             *  \right]
             * @f$
             *
             * where:
             * - @f$ c\theta = cos \theta @f$
             * - @f$ s\theta = sin \theta @f$
             * - @f$ v\theta = 1-cos \theta @f$
             */
            virtual const rw::math::Rotation3D< T > toRotation3D () const;

            /**
             * @brief Extracts the axis of rotation vector @f$ \mathbf{\hat{\mathbf{k}}} @f$
             * @return @f$ \mathbf{\hat{\mathbf{k}}} @f$
             */
            rw::math::Vector3D<T> axis() const;

            /**
             * @brief Comparison operator.
             *
             * The comparison operator makes a element wise comparison.
             * Returns true only if all elements are equal.
             *
             * @param rhs [in] EAA to compare with
             * @return True if equal.
             */
            bool operator== (const EAA< T >& rhs);

            /**
             * @brief Get the size of the EAA.
             * @return the size (always 3).
             */
            size_t size () const;

            %extend {
                #if (defined(SWIGLUA) || defined(SWIGPYTHON))
                    T& x() { return (*$self)[0]; }
                    T& y() { return (*$self)[1]; }
                    T& z() { return (*$self)[2]; }
                #elif defined(SWIGJAVA)
                    T x() const { return (*$self)[0]; }
                    T y() const { return (*$self)[1]; }
                    T z() const { return (*$self)[2]; }
                #endif
            };

            TOSTRING(rw::math::EAA<T>)
            ARRAYOPERATOR(T)
        };
    }}

    %template (EAAd) rw::math::EAA<double>;
    %template (EAAf) rw::math::EAA<float>;
    %template (VectorEAAd) std::vector< rw::math::EAA<double> >;
    
    namespace rw { namespace math {
        /**
         * @brief Calculates the cross product
         * @param v [in] a 3D vector
         * @param eaa [in] a 3D eaa vector
         * @return the resulting 3D vector
         */
        template< class T > const Vector3D< T > cross (const Vector3D< T >& v, const EAA< T >& eaa);

        /**
         * @brief Casts EAA<T> to EAA<Q>
         * @param eaa [in] EAA with type T
         * @return EAA with type Q
         */
        template< class Q, class T > const EAA< Q > cast (const EAA< T >& eaa);
    }}

    %template(cross) rw::math::cross<double>;
    %template(cross) rw::math::cross<float>;

// ######################### RPY
    namespace rw { namespace math {
        /**
         * @brief A class for representing Roll-Pitch-Yaw Euler angle rotations.
         */
        template<class T> class RPY: public Rotation3DVector<T>
        {
        public:
            /**
             * @brief Constructs rotation in which all elements are initialized to 0
             */
            RPY();

            /**
             * @brief Constructs an initialized roll-pitch-yaw euler angle vector
             * @param roll Rotation around z
             * @param pitch Rotation around y
             * @param yaw Rotation around x
             */
            RPY(T roll, T pitch, T yaw);


            RPY(const RPY& rpy);

            /**
             * @brief Constructs an RPY object initialized according to the specified Rotation3D
             *
             * \f$ \beta = arctan2(-r_{31},\sqrt{r_{11}^2+r_{21}^2}) \f$
             * \f$ \alpha = arctan2(r_{21}/cos(\beta), r_{11}/cos(\beta)) \f$
             * \f$ \beta = arctan2(r_{32}/cos(\beta), r_{33}/cos(\beta))) \f$
             *
             * @param R [in] A 3x3 rotation matrix \f$ \mathbf{R} \f$
             *
             * @param epsilon [in] Value specifying the value for which \f$
             * cos(\beta)\f$ is assumed 0 and the special case solution assuming
             * \f$\alpha=0, \beta=\pi/2 and \gamma = arctan2(r_{21}, r_{22})\f$ is
             * to be used.
             */
            RPY(const rw::math::Rotation3D<T>& rot, T epsilon = 1e-5);
            
            /**
             * @copydoc Rotation3DVector::toRotation3D
             */
            rw::math::Rotation3D<T> toRotation3D() const;
            
            /**
             * @brief size of this RPY.
             * @return the value 3
             */
            size_t size () const;

            TOSTRING(rw::math::RPY<T>)
            ARRAYOPERATOR(T)
        };
    }}

    %template (RPYd) rw::math::RPY<double>;
    %template (RPYf) rw::math::RPY<float>;
    %template (RPYdVector) std::vector< rw::math::RPY<double> >;

    namespace rw { namespace math {
        /**
         * @brief Casts RPY<T> to RPY<Q>
         *
         * @param rpy [in] RPY with type T
         *
         * @return RPY with type Q
         */
        template< class Q, class T > const RPY< Q > cast (const RPY< T >& rpy);
    }}

// ######################### Quaternion


    namespace rw { namespace math {
        /**
         * @brief A Quaternion @f$ \mathbf{q}\in \mathbb{R}^4 @f$ a complex
         * number used to describe rotations in 3-dimensional space.
         * @f$ q_w+{\bf i}\ q_x+ {\bf j} q_y+ {\bf k}\ q_z @f$
         *
         * Quaternions can be added and multiplied in a similar way as usual
         * algebraic numbers. Though there are differences. Quaternion
         * multiplication is not commutative which means
         * @f$Q\cdot P \neq P\cdot Q @f$
         */
        template<class T> class Quaternion: public Rotation3DVector<T>
        {
        public:

            /**
             * @brief constuct Quaterinion of {0,0,0,1}
             */
            Quaternion();

            /**
             * @brief Creates a Quaternion
             * @param qx [in] @f$ q_x @f$
             * @param qy [in] @f$ q_y @f$
             * @param qz [in] @f$ q_z @f$
             * @param qw  [in] @f$ q_w @f$
             */
            Quaternion(T qx, T qy, T qz, T qw);

            /**
             * @brief Creates a Quaternion from another Quaternion
             * @param quat [in] Quaternion
             */
            Quaternion(const Quaternion& quat);

            /**
             * @brief Extracts a Quaternion from Rotation matrix using
             * setRotation(const rw::math::Rotation3D<R>& rot)
             * @param rot [in] A 3x3 rotation matrix @f$ \mathbf{rot} @f$
             *
             */
            Quaternion(const rw::math::Rotation3D<T>& rot);

            



            /**
             * @brief get method for the x component
             * @return the x component of the quaternion
             */
            T getQx() const;

            /**
             * @brief get method for the y component
             * @return the y component of the quaternion
             */
            T getQy() const;

            /**
             * @brief get method for the z component
             * @return the z component of the quaternion
             */
            T getQz() const;


            /**
             * @brief get method for the w component
             * @return the w component of the quaternion
             */
            T getQw() const;

            /**
             * @brief get length of quaternion
             * @f$ \sqrt{q_x^2+q_y^2+q_z^2+q_w^2} @f$
             * @return the length og this quaternion
             */
            T getLength () const;

            /**
             * @brief get squared length of quaternion
             * @f$ q_x^2+q_y^2+q_z^2+q_w^2 @f$
             * @return the length og this quaternion
             */
            T getLengthSquared () const;

            /**
             * @brief normalizes this quaternion so that
             * @f$ normalze(Q)=\frac{Q}{\sqrt{q_x^2+q_y^2+q_z^2+q_w^2}} @f$
             */
            void normalize();

            /**
             * @brief Calculates the @f$ 3\times 3 @f$ Rotation matrix
             *
             * @return A 3x3 rotation matrix @f$ \mathbf{rot} @f$
             * @f$
             * \mathbf{rot} =
             *  \left[
             *   \begin{array}{ccc}
             *      1-2(q_y^2-q_z^2) & 2(q_x\ q_y+q_z\ q_w)& 2(q_x\ q_z-q_y\ q_w) \\
             *      2(q_x\ q_y-q_z\ q_w) & 1-2(q_x^2-q_z^2) & 2(q_y\ q_z+q_x\ q_w)\\
             *      2(q_x\ q_z+q_y\ q_w) & 2(q_y\ q_z-q_x\ q_z) & 1-2(q_x^2-q_y^2)
             *    \end{array}
             *  \right]
             * @f$
             *
             */
            rw::math::Rotation3D<T> toRotation3D() const;

            /**
             * @brief The dimension of the quaternion (i.e. 4).
             * This method is provided to help support generic algorithms using
             size() and operator[].
            */
            size_t size () const;


            /**
             * @brief Calculates a slerp interpolation between \b this and \b v.
             *
             * The slerp interpolation ensures a constant velocity across the interpolation.
             * For \f$t=0\f$ the result is \b this and for \f$t=1\f$ it is \b v.
             *
             * @note Algorithm and implementation is thanks to euclideanspace.com
             */
            Quaternion<T> slerp(const Quaternion<T>& v, T t) const;

            /**
             @brief Scalar multiplication.
            */
            Quaternion operator*(T s);

            /**
             * @brief Unary minus.
             */
            Quaternion< T > operator- () const;

            /**
             * @brief Unary plus.
             */
            Quaternion< T > operator+ () const;

            /**
             * @brief Comparison (equals) operator
             */
            inline bool operator== (const Quaternion< T >& r) const;

            /**
             * @brief Convert to an Eigen Quaternion.
             * @return Eigen Quaternion representation.
             */
            Eigen::Quaternion< T >& e ();

            TOSTRING(rw::math::Quaternion<T>)
            ARRAYOPERATOR(T)
        };
    }}

    %template (Quaterniond) rw::math::Quaternion<double>;
    %template (Quaternionf) rw::math::Quaternion<float>;
    %template (QuaterniondVector) std::vector< rw::math::Quaternion<double> >;

    namespace rw { namespace math {
        /**
         * @brief Casts Quaternion<T> to Quaternion<Q>
         * @param quaternion [in] Quarternion with type T
         * @return Quaternion with type Q
         */
        template< class Q, class T >
        inline const Quaternion< Q > cast (const Quaternion< T >& quaternion);
    }}

// ######################### Transform3D

    namespace rw { namespace math {
        /**
         * @brief A 4x4 homogeneous transform matrix @f$ \mathbf{T}\in SE(3) @f$
         *
         * @f$
         * \mathbf{T} =
         * \left[
         *  \begin{array}{cc}
         *  \mathbf{R} & \mathbf{d} \\
         *  \begin{array}{ccc}0 & 0 & 0\end{array} & 1
         *  \end{array}
         * \right]
         * @f$
         *
         */
        template<class T> class Transform3D 
        {
          public:

            /**
             * @brief Default Constructor.
             *
             * Initializes with 0 translation and Identity matrix as rotation
             */
            Transform3D();
            Transform3D(const rw::math::Transform3D<T>& t3d);

            /**
             * @brief Constructs a homogeneous transform
             * @param d [in] @f$ \mathbf{d} @f$ A 3x1 translation vector
             * @param R [in] @f$ \mathbf{R} @f$ A 3x3 rotation matrix
             */
            Transform3D(const rw::math::Vector3D<T>& d, const rw::math::Rotation3D<T>& R);

            /**
             * @brief A homogeneous transform with a rotation of \b R and a
             * translation of zero.
             */
            explicit Transform3D(const rw::math::Rotation3D<T>& R);

            /**
             * @brief A homogeneous transform with a rotation of zero and a
             translation of \b d.
            */
            explicit Transform3D(const rw::math::Vector3D<T>& d);

            /**
             * @brief Constructs a homogeneous transform
             *
             * Calling this constructor is equivalent to the transform
             * Transform3D(d, r.toRotation3D()).
             *
             * @param d [in] A 3x1 translation vector
             * @param r [in] A 3x1 rotation vector
             */
            Transform3D(const rw::math::Vector3D<T>& d, const rw::math::Rotation3DVector<T>& r);

            /**
             * @brief Constructs a homogeneous transform using the original
             * Denavit-Hartenberg notation
             *
             * @param alpha [in] @f$ \alpha_i @f$
             * @param a [in] @f$ a_i @f$
             * @param d [in] @f$ d_i @f$
             * @param theta [in] @f$ \theta_i @f$
             * @return @f$ ^{i-1}\mathbf{T}_i @f$
             *
             * @f$
             *  \robabx{i-1}{i}{\mathbf{T}}=
             *  \left[
             *    \begin{array}{cccc}
             *      c\theta_i & -s\theta_i c\alpha_i &  s\theta_i s\alpha_i & a_i c\theta_i \\
             *      s\theta_i &  c\theta_i c\alpha_i & -c\theta_i s\alpha_i & a_i s\theta_i \\
             *      0         &  s\alpha_i           &  c\alpha_i           & d_i \\
             *      0         &  0                   & 0                    & 1
             *    \end{array}
             *  \right]
             * @f$
             */
            static const Transform3D DH (T alpha, T a, T d, T theta);

            /**
             * @brief Constructs a homogeneous transform using the Craig (modified)
             * Denavit-Hartenberg notation
             *
             * @param alpha [in] @f$ \alpha_{i-1} @f$
             * @param a [in] \f$ a_{i-1} \f$
             * @param d [in] \f$ d_i \f$
             * @param theta [in] \f$ \theta_i \f$
             * @return @f$ \robabx{i-1}{i}{\mathbf{T}} @f$
             *
             * @note The Craig (modified) Denavit-Hartenberg notation differs from
             * the original Denavit-Hartenberg notation and is given as
             *
             * @f$
             * \robabx{i-1}{i}{\mathbf{T}} =
             * \left[
             * \begin{array}{cccc}
             * c\theta_i & -s\theta_i & 0 & a_{i-1} \\
             * s\theta_i c\alpha_{i-1} & c\theta_i c\alpha_{i-1} & -s\alpha_{i-1} & -s\alpha_{i-1}d_i \\
             * s\theta_i s\alpha_{i-1} & c\theta_i s\alpha_{i-1} &  c\alpha_{i-1} &  c\alpha_{i-1}d_i \\
             * 0 & 0 & 0 & 1
             * \end{array}
             * \right]
             * @f$
             *
             */
            static const Transform3D craigDH (T alpha, T a, T d, T theta);

                    /**
             * @brief Constructs a homogeneous transform using the Gordon (modified)
             * Denavit-Hartenberg notation
             *
             * @param alpha [in] @f$ \alpha_i @f$
             * @param a [in] @f$ a_i @f$
             * @param beta [in] @f$ \beta_i @f$
             * @param b [in] @f$ b_i @f$
             * @return @f$ ^{i-1}\mathbf{T}_i @f$
             *
             * @note The Gordon (modified) Denavit-Hartenberg differs from
             * the original Denavit-Hartenberg as it branches between parallel
             * and non-parallel z-axes.
             *
             * @f$ z_{i-1} @f$ is close to parallel to @f$ z_i @f$
             * @f$
             *  \robabx{i-1}{i}{\mathbf{T}}=
             *  \left[
             *    \begin{array}{cccc}
             *       c\beta_i & s\alpha_i s\beta_i &  c\alpha_i s\beta_i &  a_i c\beta_i \\
             *       0        & c\alpha_i          & -s\alpha_i          &  b_i \\
             *      -s\beta_i & s\alpha_i c\beta_i &  c\alpha_i c\beta_i & -a_i s\beta \\
             *      0         & 0                  & 0                    & 1
             *    \end{array}
             *  \right]
             * @f$
             */
            static const Transform3D DHHGP (T alpha, T a, T beta, T b);

            /**
             * @brief Constructs the identity transform
             * @return the identity transform
             *
             * @f$
             * \mathbf{T} =
             * \left[
             * \begin{array}{cccc}
             * 1 & 0 & 0 & 0\\
             * 0 & 1 & 0 & 0\\
             * 0 & 0 & 1 & 0\\
             * 0 & 0 & 0 & 1
             * \end{array}
             * \right]
             * @f$
             */
            static const Transform3D& identity ();

            /**
             * @brief Comparison operator.
             *
             * The comparison operator makes a element wise comparison.
             * Returns true only if all elements are equal.
             *
             * @param rhs [in] Transform to compare with
             * @return True if equal.
             */
            bool operator== (const rw::math::Transform3D< T >& rhs) const;

            /**
             * @brief Compares the transformations with a given precision
             *
             * Performs an element wise comparison. Two elements are considered equal if the difference
             * are less than \b precision.
             *
             * @param t3d [in] Transform to compare with
             * @param precision [in] The precision to use for testing
             * @return True if all elements are less than \b precision apart.
             */
            bool equal (const rw::math::Transform3D< T >& t3d,
                        const T precision = std::numeric_limits< T >::epsilon ()) const;

            /**
             * @brief Calculates @f$ \robabx{a}{c}{\mathbf{T}} = \robabx{a}{b}{\mathbf{T}}
             * \robabx{b}{c}{\mathbf{T}} @f$
             * @param bTc [in] @f$ \robabx{b}{c}{\mathbf{T}} @f$
             * @return @f$ \robabx{a}{c}{\mathbf{T}} @f$
             *
             * @f$
             * \robabx{a}{c}{\mathbf{T}} =
             * \left[
             *  \begin{array}{cc}
             *  \robabx{a}{b}{\mathbf{R}}\robabx{b}{c}{\mathbf{R}} & \robabx{a}{b}{\mathbf{d}} +
             * \robabx{a}{b}{\mathbf{R}}\robabx{b}{c}{\mathbf{d}} \\ \begin{array}{ccc}0 & 0 &
             * 0\end{array} & 1 \end{array} \right]
             * @f$
             */
            const Transform3D operator* (const Transform3D& bTc) const;

            /**
             * @brief Calculates @f$ \robax{a}{\mathbf{p}} = \robabx{a}{b}{\mathbf{T}}
             * \robax{b}{\mathbf{p}} \f$ thus transforming point @f$ \mathbf{p} @f$ from frame @f$ b @f$
             * to frame @f$ a @f$
             * @param bP [in] @f$ \robax{b}{\mathbf{p}} @f$
             * @return @f$ \robax{a}{\mathbf{p}} @f$
             */
            const Vector3D< T > operator* (const Vector3D< T >& bP) const;

            /**
             * @brief Gets the rotation part @f$ \mathbf{R} @f$ from @f$ \mathbf{T} @f$
             * @return @f$ \mathbf{R} @f$
             */
            rw::math::Rotation3D< T >& R ();

            /**
             * \brief Gets the position part @f$ \mathbf{d} @f$ from @f$ \mathbf{T} @f$
             * \return @f$ \mathbf{d} @f$
             */
            Vector3D< T >& P ();

            /**
             * @brief Write to \b result the product \b a * \b b.
             */
            static inline void multiply (const rw::math::Transform3D< T >& a, const rw::math::Transform3D< T >& b,
                                        rw::math::Transform3D< T >& result);

            /**
             * @brief computes the inverse of t1 and multiplies it with t2.
             * The result is saved in t1. t1 = inv(t1) * t2
             */
            static inline rw::math::Transform3D< T >& invMult (rw::math::Transform3D< T >& t1, const rw::math::Transform3D< T >& t2);

            /**
             * @brief creates a transformation that is positioned in eye and looking toward
             * center along -z where up indicates the upward direction along which the y-axis
             * is placed. Same convention as for gluLookAt
             * and is handy for placing a cameraview.
             *
             * @param eye [in] position of view
             * @param center [in] point to look toward
             * @param up [in] the upward direction (the
             *
             * @return Transformation
             */
            static rw::math::Transform3D<T> makeLookAt(const Vector3D<T>& eye, const Vector3D<T>& center, const Vector3D<T>& up);

            /**
             * @brief Returns a Eigen 4x4 matrix @f$ \mathbf{M}\in SE(3)
             * @f$ that represents this homogeneous transformation
             *
             * @return @f$ \mathbf{M}\in SE(3) @f$
             */
            Eigen::Matrix<T,4,4> e () const;

            %extend {
                #if defined(SWIGPYTHON)
                    Wrench6D<T> multiply(const Wrench6D<T>& bV) { return (*$self)*bV; }
                #endif
            };
            TOSTRING(rw::math::Transform3D<T>)
        };
    }}

    %template (Transform3Dd) rw::math::Transform3D<double>;
    %template (Transform3Df) rw::math::Transform3D<float>;
    %template (Transform3DdVector) std::vector<rw::math::Transform3D<double> >;

    namespace rw { namespace math {
        /**
         * @brief Calculates the inverse of the transformation matrix.
         *
         * @param aTb [in] the transform matrix.
         *
         * @return the inverse matrix.
         */
        template <class T>
        const rw::math::Transform3D<T> inverse(const rw::math::Transform3D<T>& aTb);

        /**
         * @brief Cast rw::math::Transform3D<T> to rw::math::Transform3D<Q>
         * @param trans [in] Transform3D with type T
         * @return Transform3D with type Q
         */
        template< class Q, class T > const rw::math::Transform3D< Q > cast (const rw::math::Transform3D< T >& trans);
    }}

    %template (inverse) rw::math::inverse<double>;
    %template (inverse) rw::math::inverse<float>;

// ######################### Pose6D
    namespace rw { namespace math {
        /**
         * @brief A Pose6D @f$ \mathbf{x}\in \mathbb{R}^6 @f$ describes a position
         * and orientation in 3-dimensions.
         *
         * @f$ {\mathbf{x}} = \left[
         *  \begin{array}{c}
         *  x \\
         *  y \\
         *  z \\
         *  \theta k_x \\
         *  \theta k_y \\
         *  \theta k_z
         *  \end{array}
         *  \right]
         *  @f$
         *
         * where @f$ (x,y,z)@f$ is the 3d position and @f$ (\theta k_x, \theta k_y,
         * \theta k_z)@f$ describes the orientation in equal angle axis (EAA)
         * format.
         */

        template<class T> class Pose6D {
        public:
            /**
             * @brief Creates an "identity" Pose6D. Position is zero vector and orientation
             * is zero vector
             */
            Pose6D ();
            Pose6D(const Pose6D<T>& p6d);

            /**
             * @brief Creates a Pose6D from 6 parameters. 3 defining the
             * position and 3 defining the EAA orientation.
             * @param x [in] The position in the @f$ x @f$ axis
             * @param y [in] The position in the @f$ y @f$ axis
             * @param z [in] The position in the @f$ z @f$ axis
             * @param kx [in] @f$ \theta k_x @f$
             * @param ky [in] @f$ \theta k_y @f$
             * @param kz [in] @f$ \theta k_z @f$
             */
            Pose6D (T x, T y, T z, T kx, T ky, T kz); 
            
            /**
             * @brief Creates a Pose6D from a Vector3D and a EAA
             * @param v3d [in] Vector3D describing the 3D position of the Pose6D
             * @param eaa [in] EAA describing the rotational component of the Pose6D.
             */
            Pose6D(const Vector3D<T>& position,const EAA<T>& rotation);

            /**
             * @brief Creates a Pose6D from a Transform3D
             *
             * @param t3d [in] A Transform3D
             */
            Pose6D(const rw::math::Transform3D<T>& t3d);

            /**
             * @brief Returns the \f$i\f$'th element in the pose.
             *
             * \f$i\in\{0,1,2\} \f$ corresponds to \f$\{x,y,z\}\f$ respectively.
             * \f$i\in\{3,4,5\}\f$ corresponds to the equivalent angle axis.
             *
             * @param i [in] index to return
             * @return the \f$i\f$'th index of the pose.
             */
            T get (size_t i) const;

            /**
             * @brief Get the position.
             * @return reference to position vector.
             */
            Vector3D< T >& getPos ();

            /**
             * @brief Get the orientation.
             * @return reference to orientation rotation vector.
             */
            EAA< T >& getEAA ();
            
            /**
             * @brief Converts the Pose6D into the corresponding Transform3D
             * @return the corresponding Transform3D
             */
            rw::math::Transform3D<T> toTransform3D();

            TOSTRING(rw::math::Pose6D<T>)
        };
    }}
    %template (Pose6Dd) rw::math::Pose6D<double>;
    %template (Pose6Df) rw::math::Pose6D<float>;
    %template (VectorPose6Dd) std::vector<rw::math::Pose6D<double> >;

    namespace rw { namespace math {
        /**
         * @brief Casts Pose6D<T> to Pose6D<Q>
         * @param pose [in] Pose6D with type T
         * @return Pose6D with type Q
         */
        template< class Q, class T > const Pose6D< Q > cast (const Pose6D< T >& pose);
    
    }}

// ######################### VelocityScrew6D
    namespace rw { namespace math {

        /**
         * @brief Class for representing 6 degrees of freedom velocity screws.
         *
         * \f[
         * \mathbf{\nu} =
         * \left[
         *  \begin{array}{c}
         *  v_x\\
         *  v_y\\
         *  v_z\\
         *  \omega_x\\
         *  \omega_y\\
         *  \omega_z
         *  \end{array}
         * \right]
         * \f]
         *
         * A VelocityScrew is the description of a frames linear and rotational velocity
         * with respect to some reference frame.
         *
         */
        template<class T> class VelocityScrew6D
        {
        public:
            /**
             * @brief Constructs a 6 degrees of freedom velocity screw
             *
             * @param vx [in] @f$ v_x @f$
             * @param vy [in] @f$ v_y @f$
             * @param vz [in] @f$ v_z @f$
             * @param wx [in] @f$ \omega_x @f$
             * @param wy [in] @f$ \omega_y @f$
             * @param wz [in] @f$ \omega_z @f$
             */
            VelocityScrew6D (T vx, T vy, T vz, T wx, T wy, T wz);

            /**
             * @brief Default Constructor. Initialized the velocity to 0
             */
            VelocityScrew6D();
            VelocityScrew6D(const VelocityScrew6D<T>& p6d);

            /**
             * @brief Constructs a velocity screw in frame @f$ a @f$ from a
             * transform @f$\robabx{a}{b}{\mathbf{T}} @f$.
             *
             * @param transform [in] the corresponding transform.
             */
            VelocityScrew6D(const rw::math::Transform3D<T>& t3d);


            /**
             * @brief Constructs a velocity screw from a linear and angular velocity
             *
             * @param linear [in] linear velocity
             * @param angular [in] angular velocity
             */
            VelocityScrew6D (const Vector3D< T >& linear, const EAA< T >& angular);

            /**
             * @brief Extracts the linear velocity
             *
             * @return the linear velocity
             */
            const Vector3D< T > linear () const;

            /**
             * @brief Extracts the angular velocity and represents it using an
             * equivalent-angle-axis as @f$ \dot{\Theta}\mathbf{k} @f$
             *
             * @return the angular velocity
             */
            const EAA< T > angular () const;

            /**
             * @brief Comparison operator.
             *
             * The comparison operator makes a element wise comparison.
             * Returns true only if all elements are equal.
             *
             * @param rhs [in] VelocityScrew6D to compare with
             * @return True if equal.
             */
            bool operator== (const VelocityScrew6D< T >& rhs) const;

            /**
             * @brief Scales velocity screw and returns scaled version
             * @param s [in] scaling value
             * @return Scales screw
             */
            VelocityScrew6D<T> operator*(T scale) const;
            
            /**
             * @brief Adds two velocity screws together @f$
             * \mathbf{\nu}_{12}=\mathbf{\nu}_1+\mathbf{\nu}_2 @f$
             *
             * @param screw2 [in] @f$ \mathbf{\nu}_2 @f$
             *
             * @return the velocity screw @f$ \mathbf{\nu}_{12} @f$
             */
            VelocityScrew6D<T> operator+(const VelocityScrew6D<T>& other) const;

            /**
             * @brief Subtracts two velocity screws
             * \f$\mathbf{\nu}_{12}=\mathbf{\nu}_1-\mathbf{\nu}_2\f$
             *
             * \param screw2 [in] \f$\mathbf{\nu}_2\f$
             * \return the velocity screw \f$\mathbf{\nu}_{12} \f$
             */
            VelocityScrew6D<T> operator-(const VelocityScrew6D<T>& other) const;

            /**
             * @brief Takes the 1-norm of the velocity screw. All elements both
             * angular and linear are given the same weight.
             *
             * @return the 1-norm
             */
            double norm1();

            /**
             * @brief Takes the 2-norm of the velocity screw. All elements both
             * angular and linear are given the same weight
             *
             * @return the 2-norm
             */
            double norm2();

            /**
             * @brief Takes the infinite norm of the velocity screw. All elements
             * both angular and linear are given the same weight.
             *
             * @return the infinite norm
             */
            double normInf();

            /**
             @brief Converter to Eigen vector
            */
            Eigen::Matrix< T, 6, 1 > e () const;

            TOSTRING(rw::math::VelocityScrew6D<T>)
            ARRAYOPERATOR(T)
        };
    }}
    %template (VelocityScrew6Dd) rw::math::VelocityScrew6D<double>;
    %template (VelocityScrew6Df) rw::math::VelocityScrew6D<float>;
    %template (VectorVelocityScrew6d) std::vector<rw::math::VelocityScrew6D<double> >;

    namespace rw { namespace math {
        /**
         * @brief Takes the 1-norm of the velocity screw. All elements both
         * angular and linear are given the same weight.
         *
         * @param screw [in] the velocity screw
         * @return the 1-norm
         */
        template< class T > T norm1 (const VelocityScrew6D< T >& screw) { return screw.norm1 (); }

        /**
         * @brief Takes the 2-norm of the velocity screw. All elements both
         * angular and linear are given the same weight
         *
         * @param screw [in] the velocity screw
         * @return the 2-norm
         */
        template< class T > T norm2 (const VelocityScrew6D< T >& screw) { return screw.norm2 (); }

        /**
         * @brief Takes the infinite norm of the velocity screw. All elements
         * both angular and linear are given the same weight.
         *
         * @param screw [in] the velocity screw
         *
         * @return the infinite norm
         */
        template< class T > T normInf (const VelocityScrew6D< T >& screw) { return screw.normInf (); }

        /**
         * @brief Casts VelocityScrew6D<T> to VelocityScrew6D<Q>
         *
         * @param vs [in] VelocityScrew6D with type T
         *
         * @return VelocityScrew6D with type Q
         */
        template< class Q, class T > const VelocityScrew6D< Q > cast (const VelocityScrew6D< T >& vs);
    }}

// ######################### Wrench6D
    
    namespace rw { namespace math {
        /**
         * @brief Class for representing 6 degrees of freedom wrenches.
         *
         * \f[
         * \mathbf{\nu} =
         * \left[
         *  \begin{array}{c}
         *  f_x\\
         *  f_y\\
         *  f_z\\
         *  \tau_x\\
         *  \tau_y\\
         *  \tau_z
         *  \end{array}
         * \right]
         * \f]
         *
         * A Wrench is the description of a frames linear force and rotational torque
         * with respect to some reference frame.
         *
         */
        
        template<class T> class Wrench6D
        {
        public:		

            /**
             * @brief Constructs a 6 degrees of freedom velocity screw
             *
             * @param fx [in] @f$ f_x @f$
             * @param fy [in] @f$ f_y @f$
             * @param fz [in] @f$ f_z @f$
             * @param tx [in] @f$ \tau_x @f$
             * @param ty [in] @f$ \tau_y @f$
             * @param tz [in] @f$ \tau_z @f$
             */
            Wrench6D(T fx, T fy, T fz, T tx, T ty, T tz);

            /**
             * @brief Default Constructor. Initialized the wrench to 0
             */
            Wrench6D();

            /**
             * @brief Constructs a wrench from a force and torque
             *
             * @param force [in] linear force
             * @param torque [in] angular torque
             */
            Wrench6D (const rw::math::Vector3D< T >& force, const rw::math::Vector3D< T >& torque);

            /**
             * @brief Sets the force component
             *
             * @param force [in] linear force
             */
            void setForce (const Vector3D< T >& force);

            /**
             * @brief Sets the torque component
             *
             * @param torque [in] angular torque
             */
            void setTorque (const Vector3D< T >& torque);
            
            /**
             * @brief Extracts the force
             *
             * @return the force
             */
            const rw::math::Vector3D<T> force() const;

            /**
             * @brief Extracts the torque and represents it using an Vector3D<T>
             *
             * @return the torque
             */
            const rw::math::Vector3D<T> torque() const;

            /**
             * @brief Scales wrench and returns scaled version
             * @param s [in] scaling value
             * @return Scaled wrench
             */
            const Wrench6D<T> operator*(T s) const;


            /**
             * @brief Changes frame of reference and referencepoint of
             * wrench: @f$ \robabx{b}{b}{\mathbf{w}}\to
             * \robabx{a}{a}{\mathbf{w}} @f$
             *
             * The frames @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
             * rigidly connected.
             *
             * @param aTb [in] the location of frame @f$ \mathcal{F}_b @f$ wrt.
             * frame @f$ \mathcal{F}_a @f$: @f$ \robabx{a}{b}{\mathbf{T}} @f$
             *
             * @param bV [in] wrench wrt. frame @f$ \mathcal{F}_b @f$: @f$
             * \robabx{b}{b}{\mathbf{\nu}} @f$
             *
             * @return the wrench wrt. frame @f$ \mathcal{F}_a @f$: @f$
             * \robabx{a}{a}{\mathbf{\nu}} @f$
             *
             * Transformation of both the wrench reference point and of the base to
             * which the wrench is expressed
             *
             * \f[
             * \robabx{a}{a}{\mathbf{w}} =
             * \left[
             *  \begin{array}{c}
             *  \robabx{a}{a}{\mathbf{force}} \\
             *  \robabx{a}{a}{\mathbf{torque}}
             *  \end{array}
             * \right] =
             * \left[
             *  \begin{array}{cc}
             *    \robabx{a}{b}{\mathbf{R}} & S(\robabx{a}{b}{\mathbf{p}})
             *    \robabx{a}{b}{\mathbf{R}} \\
             *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
             *  \end{array}
             * \right]
             * \robabx{b}{b}{\mathbf{\nu}} =
             * \left[
             *  \begin{array}{c}
             *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{b}{\mathbf{v}} +
             *    \robabx{a}{b}{\mathbf{p}} \times \robabx{a}{b}{\mathbf{R}}
             *    \robabx{b}{b}{\mathbf{\omega}}\\
             *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{b}{\mathbf{\omega}}
             *  \end{array}
             * \right]
             * \f]
             *
             */
            friend const Wrench6D<T> operator*(const rw::math::Transform3D<T>& aTb,
                                                    const Wrench6D<T>& bV);
            
            /**
             * @brief Changes wrench referencepoint of
             * wrench: @f$ \robabx{b}{b}{\mathbf{w}}\to
             * \robabx{a}{a}{\mathbf{w}} @f$
             *
             * The frames @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
             * rigidly connected.
             *
             * @param aPb [in] the location of frame @f$ \mathcal{F}_b @f$ wrt.
             * frame @f$ \mathcal{F}_a @f$: @f$ \robabx{a}{b}{\mathbf{T}} @f$
             *
             * @param bV [in] wrench wrt. frame @f$ \mathcal{F}_b @f$: @f$
             * \robabx{b}{b}{\mathbf{\nu}} @f$
             *
             * @return the wrench wrt. frame @f$ \mathcal{F}_a @f$: @f$
             * \robabx{a}{a}{\mathbf{\nu}} @f$
             *
             * Transformation of both the velocity reference point and of the base to
             * which the wrench is expressed
             *
             * \f[
             * \robabx{a}{a}{\mathbf{w}} =
             * \left[
             *  \begin{array}{c}
             *  \robabx{a}{a}{\mathbf{force}} \\
             *  \robabx{a}{a}{\mathbf{torque}}
             *  \end{array}
             * \right] =
             * \left[
             *  \begin{array}{cc}
             *    \robabx{a}{b}{\mathbf{R}} & S(\robabx{a}{b}{\mathbf{p}})
             *    \robabx{a}{b}{\mathbf{R}} \\
             *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
             *  \end{array}
             * \right]
             * \robabx{b}{b}{\mathbf{\nu}} =
             * \left[
             *  \begin{array}{c}
             *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{b}{\mathbf{v}} +
             *    \robabx{a}{b}{\mathbf{p}} \times \robabx{a}{b}{\mathbf{R}}
             *    \robabx{b}{b}{\mathbf{\omega}}\\
             *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{b}{\mathbf{\omega}}
             *  \end{array}
             * \right]
             * \f]
             *
             */
            friend const Wrench6D<T> operator*(const rw::math::Vector3D<T>& aPb, const Wrench6D<T>& bV);

            /**
             * @brief Changes frame of reference for wrench: @f$
             * \robabx{b}{i}{\mathbf{w}}\to \robabx{a}{i}{\mathbf{w}}
             * @f$
             *
             * @param aRb [in] the change in orientation between frame
             * @f$ \mathcal{F}_a @f$ and frame
             * @f$ \mathcal{F}_b @f$: @f$ \robabx{a}{b}{\mathbf{R}} @f$
             *
             * @param bV [in] velocity screw wrt. frame
             * @f$ \mathcal{F}_b @f$: @f$ \robabx{b}{i}{\mathbf{\nu}} @f$
             *
             * @return the wrench wrt. frame @f$ \mathcal{F}_a @f$:
             * @f$ \robabx{a}{i}{\mathbf{w}} @f$
             *
             * Transformation of the base to which the wrench is expressed. The wrench
             * reference point is left intact
             *
             * \f[
             * \robabx{a}{i}{\mathbf{w}} =
             * \left[
             *  \begin{array}{c}
             *  \robabx{a}{i}{\mathbf{force}} \\
             *  \robabx{a}{i}{\mathbf{torque}}
             *  \end{array}
             * \right] =
             * \left[
             *  \begin{array}{cc}
             *    \robabx{a}{b}{\mathbf{R}} & \mathbf{0}^{3x3} \\
             *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
             *  \end{array}
             * \right]
             * \robabx{b}{i}{\mathbf{\nu}} =
             * \left[
             *  \begin{array}{c}
             *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{i}{\mathbf{v}} \\
             *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{i}{\mathbf{\omega}}
             *  \end{array}
             * \right]
             * \f]
             */
            friend const Wrench6D<T> operator*(const rw::math::Rotation3D<T>& aRb, const Wrench6D<T>& bV);

            /**
             * @brief Adds two wrenches together @f$
             * \mathbf{w}_{12}=\mathbf{w}_1+\mathbf{w}_2 @f$
             *
             * @param rhs [in] @f$ \mathbf{\nu}_1 @f$
             *
             * @return the wrench @f$ \mathbf{w}_{12} @f$
             */
            const Wrench6D< T > operator+ (const Wrench6D< T >& rhs) const;

            /**
             * @brief Subtracts two velocity screws
             * \f$\mathbf{\nu}_{12}=\mathbf{\nu}_1-\mathbf{\nu}_2\f$
             *
             * \param rhs [in] \f$\mathbf{w}_1\f$
             * \return the wrench \f$\mathbf{w}_{12} \f$
             */
            const Wrench6D<T> operator-(const Wrench6D<T>& wrench) const;

            /**
             * @brief Takes the 1-norm of the wrench. All elements both
             * force and torque are given the same weight.
             * @return the 1-norm
             */
            T norm1() const;
            
            /**
             * @brief Takes the 2-norm of the wrench. All elements both
             * force and torque are given the same weight
             * @return the 2-norm
             */
            T norm2() const ;

            /**
             * @brief Takes the infinite norm of the wrench. All elements
             * both force and torque are given the same weight.
             *
             * @return the infinite norm
             */
            T normInf() const ;

            /**
             * @brief Converter to Eigen data type
             */
            Eigen::Matrix< T, 6, 1 > e () const;

            /**
             * @brief Compares \b a and \b b for equality.
             * @param b [in] other wrench to compare with.
             * @return True if a equals b, false otherwise.
             */
            bool operator== (const Wrench6D< T >& b) const;

            TOSTRING(rw::math::Wrench6D<T>)
            ARRAYOPERATOR(T)
        };
    }}

    %template (Wrench6Dd) rw::math::Wrench6D<double>;
    %template (Wrench6Df) rw::math::Wrench6D<float>;
    %template (VectorWrench6Dd) std::vector<rw::math::Wrench6D<double> >;

    namespace rw { namespace math {
        /**
         * @brief Takes the 1-norm of the wrench. All elements both
         * force and torque are given the same weight.
         *
         * @param wrench [in] the wrench
         * @return the 1-norm
         */
        template< class T > T norm1 (const Wrench6D< T >& wrench);

        /**
         * @brief Takes the 2-norm of the wrench. All elements both
         * force and tporque are given the same weight
         *
         * @param wrench [in] the wrench
         * @return the 2-norm
         */
        template< class T > T norm2 (const Wrench6D< T >& wrench);

        /**
         * @brief Takes the infinite norm of the wrench. All elements
         * both force and torque are given the same weight.
         *
         * @param wrench [in] the wrench
         *
         * @return the infinite norm
         */
        template< class T > T normInf (const Wrench6D< T >& wrench);

        /**
         * @brief Casts Wrench6D<T> to Wrench6D<Q>
         *
         * @param vs [in] Wrench6D with type T
         *
         * @return Wrench6D with type Q
         */
        template< class Q, class T > const Wrench6D< Q > cast (const Wrench6D< T >& vs);
    }}


// ######################### InertiaMatrix
    namespace rw { namespace math {

        /**
         * @brief A 3x3 inertia matrix
         */
        template<class T> class InertiaMatrix
        {
        public:

            /**
             * @brief Constructs an initialized 3x3 rotation matrix
             *
             * @param r11 \f$ r_{11} \f$
             * @param r12 \f$ r_{12} \f$
             * @param r13 \f$ r_{13} \f$
             * @param r21 \f$ r_{21} \f$
             * @param r22 \f$ r_{22} \f$
             * @param r23 \f$ r_{23} \f$
             * @param r31 \f$ r_{31} \f$
             * @param r32 \f$ r_{32} \f$
             * @param r33 \f$ r_{33} \f$
             *
             * @f$
             *  \mathbf{R} =
             *  \left[
             *  \begin{array}{ccc}
             *  r_{11} & r_{12} & r_{13} \\
             *  r_{21} & r_{22} & r_{23} \\
             *  r_{31} & r_{32} & r_{33}
             *  \end{array}
             *  \right]
             * @f$
             */
            InertiaMatrix(
                T r11, T r12, T r13,
                T r21, T r22, T r23,
                T r31, T r32, T r33);

            /**
             * @brief Constructs an initialized 3x3 rotation matrix
             * @f$ \robabx{a}{b}{\mathbf{R}} =
             * \left[
             *  \begin{array}{ccc}
             *   \robabx{a}{b}{\mathbf{i}} & \robabx{a}{b}{\mathbf{j}} & \robabx{a}{b}{\mathbf{k}}
             *  \end{array}
             * \right]
             * @f$
             *
             * @param i @f$ \robabx{a}{b}{\mathbf{i}} @f$
             * @param j @f$ \robabx{a}{b}{\mathbf{j}} @f$
             * @param k @f$ \robabx{a}{b}{\mathbf{k}} @f$
             */
            InertiaMatrix(
                const rw::math::Vector3D<T>& i,
                const rw::math::Vector3D<T>& j,
                const rw::math::Vector3D<T>& k);

            /**
             * @brief constructor - where only the diagonal is set
             * @param i [in] m(0,0)
             * @param j [in] m(1,1)
             * @param k [in] m(2,2)
             */
            InertiaMatrix(
                T i = 0.0,
                T j = 0.0,
                T k = 0.0);

            /**
             * @brief Construct an internal matrix from a Eigen::MatrixBase
             * It is the responsibility of the user that 3x3 matrix is indeed an
                    inertia matrix.
            */
            explicit InertiaMatrix (const Eigen::Matrix<T,3,3>& r);

            /**
             * @brief Returns reference to the internal 3x3 matrix
             */
            Eigen::Matrix<T,3,3>& e ();

            /**
             * @brief Calculates \f$ \robabx{a}{c}{\mathbf{R}} =
             * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{R}} \f$
             *
             * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
             *
             * @param bRc [in] \f$ \robabx{b}{c}{\mathbf{R}} \f$
             *
             * @return \f$ \robabx{a}{c}{\mathbf{R}} \f$
             */
            friend InertiaMatrix<T> operator*(const rw::math::Rotation3D<T>& aRb, const InertiaMatrix<T>& bRc);

            /**
             * @brief Calculates \f$ \robabx{a}{c}{\mathbf{R}} =
             * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{R}} \f$
             *
             * @param bRc [in] \f$ \robabx{b}{c}{\mathbf{R}} \f$
             *
             * @return \f$ \robabx{a}{c}{\mathbf{R}} \f$
             */
            InertiaMatrix<T> operator*(const rw::math::Rotation3D<T>& bRc);

            /**
             * @brief Calculates the addition between the two InertiaMatrices
             */
            InertiaMatrix<T> operator+(const InertiaMatrix<T>& I2);

            /**
             * @brief Calculates \f$ \robabx{a}{c}{\mathbf{v}} =
             * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{v}} \f$
             * @param bVc [in] \f$ \robabx{b}{c}{\mathbf{v}} \f$
             * @return \f$ \robabx{a}{c}{\mathbf{v}} \f$
             */
            Vector3D<T> operator*(const Vector3D<T>& bVc);

            /**
             * @brief Make inertia matrix for a solid sphere.
             * @param mass [in] mass of solid sphere.
             * @param radi [in] radius of sphere.
             * @return the inertia matrix.
             */
            static InertiaMatrix<T> makeSolidSphereInertia(T mass, T radi);

            /**
             * @brief Make inertia matrix for a hollow sphere.
             * @param mass [in] mass of hollow sphere.
             * @param radi [in] radius of sphere.
             * @return the inertia matrix.
             */
            static InertiaMatrix<T> makeHollowSphereInertia(T mass, T radi);

            /**
             * @brief calculates the inertia of a cuboid where the reference frame is in the
             * center of the cuboid with
             * @param mass
             * @param x
             * @param y
             * @param z
             * @return
             */
            static InertiaMatrix<T> makeCuboidInertia(T mass, T x, T y, T z);

            TOSTRING(rw::math::InertiaMatrix<T>)
            MATRIXOPERATOR(T)
        };
    }}
    %template (InertiaMatrixd) rw::math::InertiaMatrix<double>;
    %template (InertiaMatrixf) rw::math::InertiaMatrix<float>;
    %template (InertiaMatrixdVector) std::vector<rw::math::InertiaMatrix<double> >;

    namespace rw { namespace math {
        /**
         * @brief Calculates the inverse @f$ \robabx{b}{a}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}}^{-1} @f$ of a rotation matrix
         *
         * @param aRb [in] the rotation matrix @f$ \robabx{a}{b}{\mathbf{R}} @f$
         *
         * @return the matrix inverse @f$ \robabx{b}{a}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}}^{-1} @f$
         *
         * @f$ \robabx{b}{a}{\mathbf{R}} = \robabx{a}{b}{\mathbf{R}}^{-1} =
         * \robabx{a}{b}{\mathbf{R}}^T @f$
         */
        template< class Q > InertiaMatrix< Q > inverse (const InertiaMatrix< Q >& aRb);

        /**
         * @brief Casts InertiaMatrix<T> to InertiaMatrix<Q>
         * @param rot [in] InertiaMatrix with type T
         * @return InertiaMatrix with type Q
         */
        template< class Q, class T > InertiaMatrix< Q > cast (const InertiaMatrix< T >& rot);
    }}


// ######################### ProjectionMatrix

    /**
     * @brief projection matrix
     */
    class ProjectionMatrix {
    public:
        //! @brief constructor
        ProjectionMatrix();

        //! @brief get the boost matrix corresponding to this projection
        const Eigen::Matrix< double, 4, 4 > e () const;

        //! @brief Destructor
        ~ProjectionMatrix();

        /** 
         * @brief test if this is a perspective projection
         * 
         * @return true if perspective projection, false otherwise.
         */
        bool isPerspectiveProjection();

        /** 
         * @brief test if this is a ortographic projection
         * 
         * @return true if ortographic projection, false otherwise.
         */
        bool isOrtographicProjection();

        /**
         * @brief set the projection matrix to an ortographic projection by defining
         * the box with length to all sides (left, right, bottom, top, near and far)
         *
         * @param left [in] length in m to left edge of image
         * @param right [in] length in m to right edge of image
         * @param bottom [in] length in m to bottom edge of image
         * @param top [in] length in m to top edge of image
         * @param zNear [in] length in m to near clipping plane
         * @param zFar [in] length in m to far clipping plane
         */
        void setOrtho(double left, double right,
                        double bottom, double top,
                        double zNear, double zFar);

        //! get ortographic projection. Onli valid if isOrtographicProjection is true
        bool getOrtho(double left, double right,
                        double bottom, double top,
                        double zNear, double zFar) const;

        /**
         * @brief set the projection matrix to the viewing frustum
         *
         * @param left [in] distance in m near cutting plane from center to left edge
         * @param right [in] distance in m near cutting plane from center to right edge
         * @param bottom [in] distance in m near cutting plane from center to bottom edge
         * @param top [in] distance in m near cutting plane from center to top edge
         * @param zNear [in] distance in m along z-axis to near cutting plane
         * @param zFar [in] distance in m along z-axis to far cutting plane
         */
        void setFrustum(double left, double right,
                        double bottom, double top,
                        double zNear, double zFar);

        /**
         * @brief get the projection matrix to the viewing frustum
         *
         * @param left [out] distance in m near cutting plane from center to left edge
         * @param right [out] distance in m near cutting plane from center to right edge
         * @param bottom [out] distance in m near cutting plane from center to bottom edge
         * @param top [out] distance in m near cutting plane from center to top edge
         * @param zNear [out] distance in m along z-axis to near cutting plane
         * @param zFar [out] distance in m along z-axis to far cutting plane
         */
        bool getFrustum(double left, double right,
                        double bottom, double top,
                        double zNear, double zFar) const;

        /**
         * @brief set the projection matrix to perspective projection
         *
         * @param fovy [in] vertical field of view [degrees]
         * @param aspectRatio [in] aspect ratio between width and height of image
         * @param zNear [in] distance to near cutting plane
         * @param zFar [in] distance to far cutting plane
         */
        void setPerspective(double fovy, double aspectRatio, double zNear, double zFar);

        /**
         * @brief set the projection matrix to perspective projection
         *
         * @param fovy [in] vertical field of view [degrees]
         * @param width [in] width of image
         * @param height [in] height of image
         * @param zNear [in] distance to near cutting plane
         * @param zFar [in] distance to far cutting plane
         */
        void setPerspective(double fovy, double width, double height, double zNear, double zFar);

        /**
         * @brief set the projection matrix to perspective projection
         *
         * @param fovy [in] vertical field of view [degrees]
         * @param aspectRatio [in] aspect ratio between width and height of image
         * @param zNear [in] distance to near cutting plane
         * @param zFar [in] distance to far cutting plane
         */
        bool getPerspective(double fovy, double aspectRatio, double zNear, double zFar) const;

        /**
         * @brief creates a projection matrix with a perspective projection
         *
         * @param fovy [in]
         * @param aspectRatio [in]
         * @param zNear [in]
         * @param zFar [in]
         * @return new ProjectionMatrix.
         */
        static ProjectionMatrix makePerspective(double fovy, double aspectRatio, double zNear, double zFar);

        /**
         * @brief creates a projection matrix with a perspective projection
         *
         * @param fovy [in]
         * @param width [in] of image
         * @param height [in] of image
         * @param zNear [in]
         * @param zFar [in]
         * @return new ProjectionMatrix.
         */
        static ProjectionMatrix makePerspective(double fovy, double width, double height, double zNear, double zFar);


        /**
         * @brief creates a projection matrix with a orthographic projection
         *
         * @param left [in]
         * @param right [in]
         * @param bottom [in]
         * @param top [in]
         * @param zNear [in]
         * @param zFar [in]
         * @return new ProjectionMatrix.
         */
        static ProjectionMatrix makeOrtho(double left, double right,
                                        double bottom, double top,
                                        double zNear, double zFar);

        /**
         * get near and far clipping plane
         * @return
         */
        std::pair< double, double > getClipPlanes () const;
    };


// ######################### Jacobian

    class Jacobian
    {
    public:

        /**
         * @brief Creates an empty @f$ m\times n @f$ (uninitialized) Jacobian matrix
         *
         * @param m [in] number of rows
         *
         * @param n [in] number of columns
         */
        Jacobian (size_t m, size_t n);
        
        size_t size1() const ;
        size_t size2() const ;

        /**
         * @brief Creates an empty @f$ 6\times n @f$ (uninitialized) Jacobian matrix
         *
         * @param n [in] number of columns
         */
        explicit Jacobian (size_t n);

        /**
         * @brief Construct zero initialized Jacobian.
         * @param size1 [in] number of rows.
         * @param size2 [in] number of columns.
         * @return zero-initialized jacobian.
         */
        static Jacobian zero (size_t size1, size_t size2);

        /**
           @brief Accessor for the internal Eigen matrix state.
         */
        Eigen::Matrix<double, -1, -1> e();

        /**
         * @brief Get an element of the jacobian.
         * @param row [in] the row.
         * @param col [in] the column.
         * @return reference to the element.
         */
        double& elem(int i, int j);

        /**
         * @brief Creates the velocity transform jacobian
         * @f$ \robabcdx{a}{b}{a}{b}{\bf{J_v}} @f$
         * for transforming both the reference frame and the velocity
         * reference point from one frame \b b to another frame \b a
         *
         * @param aTb [in] @f$ \robabx{a}{b}{\bf{T}} @f$
         *
         * @return @f$ \robabcdx{a}{b}{a}{b}{\bf{J_v}} @f$
         *
         * \f[
         * \robabcdx{a}{b}{a}{b}{\bf{J_v}} =
         * \left[
         *  \begin{array}{cc}
         *    \robabx{a}{b}{\mathbf{R}} & S(\robabx{a}{b}{\mathbf{d}})\robabx{a}{b}{\mathbf{R}} \\
         *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
         *  \end{array}
         * \right]
         * \f]
         *
         * Change the frame of reference from \b b to frame \b a and reference point
         * from frame \b a to frame \b b:
         * @f$ \robabx{a}{b}{\bf{J}} =  \robabcdx{a}{b}{a}{b}{\bf{J}_v} \cdot \robabx{b}{a}{\bf{J}}
         * @f$
         */
        explicit Jacobian (const rw::math::Transform3D<double>& aTb);

        /**
         * @brief Creates the velocity transform jacobian
         * @f$ \robabcdx{a}{b}{i}{i}{\bf{J_v}} @f$
         * for transforming a velocity screw from one frame of reference \b b to
         * another frame \b a
         *
         * @param aRb [in] @f$ \robabx{a}{b}{\bf{R}} @f$
         *
         * @return @f$ \robabcdx{a}{b}{i}{i}{\bf{J}_v} @f$
         *
         * \f[
         * \robabcdx{a}{b}{i}{i}{\bf{J_v}} =
         * \left[
         *  \begin{array}{cc}
         *    \robabx{a}{b}{\mathbf{R}} & \mathbf{0}^{3x3} \\
         *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
         *  \end{array}
         * \right]
         * \f]
         *
         * Change the frame of reference from \b b to frame \b a :
         * @f$ \robabx{a}{c}{\bf{J}} =  \robabcdx{a}{b}{c}{c}{\bf{J}_v} \cdot \robabx{b}{c}{\bf{J}}
         * @f$
         *
         */
        explicit Jacobian (const rw::math::Rotation3D<double>& aRb);

        /**
         * @brief Creates the velocity transform jacobian
         * @f$ \robabcdx{i}{i}{b}{a}{\bf{J}_v} @f$
         * for transforming the reference point of a velocity screw from one
         * frame \b b to another frame \b a
         *
         * @param aPb [in] @f$ \robabx{a}{b}{\bf{P}} @f$
         *
         * @return @f$ \robabcdx{i}{i}{b}{a}{\bf{J}_v} @f$
         *
         * \f[
         * \robabcdx{i}{i}{b}{a}{\bf{J}_v} =
         * \left[
         *  \begin{array}{cc}
         *    \bf{I}^{3x3} & S(\robabx{a}{b}{\bf{P}}) \\
         *    \bf{0}^{3x3} & \bf{I}^{3x3}
         *  \end{array}
         * \right]
         * \f]
         *
         *  transforming the reference point of a Jacobian from
         * frame \b c to frame \b d :
         * @f$ \robabx{a}{d}{\mathbf{J}} =  \robabcdx{a}{a}{c}{d}{\mathbf{J_v}} \cdot
         * \robabx{a}{c}{\mathbf{J}} @f$
         */
        explicit Jacobian (const rw::math::Vector3D<double>& aPb);

        /**
         * @brief add rotation jacobian to a specific row and column in this jacobian
         * @param part
         * @param row
         * @param col
         */
        void addRotation (const rw::math::Vector3D<double>& part, size_t row, size_t col);

        /**
         * @brief add position jacobian to a specific row and column in this jacobian
         * @param part
         * @param row
         * @param col
         */
        void addPosition (const rw::math::Vector3D<double>& part, size_t row, size_t col);

        TOSTRING(Jacobian)
        MATRIXOPERATOR(double)
    };

    namespace rw { namespace math {
        /**
         * @brief Calculates velocity vector
         * @param Jq [in] the jacobian @f$ \mathbf{J}_{\mathbf{q}} @f$
         * @param dq [in] the joint velocity vector @f$ \dot{\mathbf{q}} @f$
         * @return the velocity vector @f$ \mathbf{\nu} @f$
         * @relates Jacobian
         */
        inline const VelocityScrew6D<double> operator* (const Jacobian& Jq, const Q& dq);

        /**
         * @brief Calculates joint velocities
         *
         * @param JqInv [in] the inverse jacobian @f$ \mathbf{J}_{\mathbf{q}}^{-1} @f$
         *
         * @param v [in] the velocity vector @f$ \mathbf{\nu} @f$
         *
         * @return the joint velocity vector @f$ \dot{\mathbf{q}} @f$
         *
         * @relates Jacobian
         */
        inline const Q operator* (const Jacobian& JqInv, const VelocityScrew6D<double>& v);

        /**
         * @brief Multiplies jacobians @f$ \mathbf{J} = \mathbf{J}_1 *
         * \mathbf{J}_2 @f$
         *
         * @param j1 [in] @f$ \mathbf{J}_1 @f$
         *
         * @param j2 [in] @f$ \mathbf{J}_2 @f$
         *
         * @return @f$ \mathbf{J} @f$
         *
         * @relates Jacobian
         */
        inline const Jacobian operator* (const Jacobian& j1, const Jacobian& j2);

        /**
         * @brief Rotates each column of \b v by \b r.
         * The Jacobian must be of height 6.
         * @relates Jacobian
        */
        const Jacobian operator* (const rw::math::Rotation3D<double>& r, const Jacobian& v);

    }}


// ######################### Metric

    namespace rw { namespace math {
        %nodefaultctor Metric;

        /**
         @brief Template interface for metrics on type T.

        A metric is a function that defines a scalar distance between elements.
        */
        template <class T>
        class Metric
        {
          public:

            /**
             * @brief The distance from the zero element to q
             */
            double distance(const T& q) const;

            /**
             @brief The distance from element a to b.

            @param a [in] first element
            @param b [in] second element
            @return the distance
            */
            double distance(const T& a, const T& b) const;

            /**
             @brief The dimension of elements on which this metric operates.

            The returns -1 if the elements don't have a measure of dimension or
            if the metric works for elements of all dimensions.
            */
            int size() const;

          protected:
            // Here we have the methods implemented in the subclasses.

            /**
             @brief Subclass implementation of the distance() method.
            */
            virtual scalar_type doDistance (const value_type& q) const = 0;

            /**
             @brief Subclass implementation of the distance() method.
            */
            virtual scalar_type doDistance (const value_type& a, const value_type& b) const = 0;

            /**
             @brief Subclass implementation of the size() method.

            By default the methods returns -1, i.e. valid for all elements.
            */
            virtual int doSize () const;

          protected:
            //! Protected constructor called by subclassed.
            Metric ();

            //! Disable copying of superclass.
            Metric (const rw::math::Metric&);
        };
    }}

    %template (MetricQ) rw::math::Metric<Q>;
    %template (MetricQPtr) rw::core::Ptr<rw::math::Metric<Q> >;
    %template (MetricQCPtr) rw::core::Ptr<const rw::math::Metric<Q> >;
    %template (MetricTransform3D) rw::math::Metric<rw::math::Transform3D<double> >;
    %template (MetricPtrTransform3D) rw::core::Ptr<rw::math::Metric<rw::math::Transform3D<double> > >;
    OWNEDPTR(rw::math::Metric<Q> );
    OWNEDPTR(rw::math::Metric<rw::math::Transform3D<double> > );

// ######################### MetricFactory
    
    class MetricFactory
    {
    public:
        /**
         * @brief Euclidean configuration metric.
         *  See class EuclideanMetric for details.
         */
        template< class VectorType >
        static rw::core::Ptr< rw::math::Metric< VectorType >> makeEuclidean ();

        /**
           @brief Weighted Euclidean configuration metric.

           See class WeightedEuclideanMetric for details.

           @param weights [in] Weights for the metric.
           @return Weighted Euclidean metric.
        */
        template< class VectorType >
        static rw::core::Ptr< rw::math::Metric< VectorType >>
        makeWeightedEuclidean (const VectorType& weights);

    private:
        MetricFactory();
        MetricFactory(const MetricFactory&);
        MetricFactory& operator=(const MetricFactory&);

    };

    %template(makeEuclideanQ) MetricFactory::makeEuclidean<Q>;
    %template(makeWeightedEuclideanQ) MetricFactory::makeWeightedEuclidean<Q>;


// ######################### CameraMatrix
    namespace rw { namespace math { 
        /**
         * @brief The PerspectiveTransform2D is a perspective transform in 2D.
         * The homographic transform can be used to map one arbitrary 2D quadrilateral
         * into another.
         */

        template< class T> class CameraMatrix
        {
        public:
            //! @brief The type of the underlying Eigen data type.
            typedef Eigen::Matrix< T, 4, 4 > Base;

            //! A pair of Vector3D
            typedef std::pair< rw::math::Vector3D< T >, rw::math::Vector3D< T > > Vector3DPair;

            /**
             * @brief constructor
             */
            CameraMatrix (T r11, T r12, T r13, T r21, T r22, T r23, T r31, T r32, T r33);

            /**
             * @brief destructor
             */
            virtual ~CameraMatrix ();

            /**
             * @brief transform a point using this perspective transform
             */
            Vector3D< T > operator* (const Vector3D< T >& v2d);

            /**
             * @brief Returns reference to the internal camera matrix
             *
             * @return @f$ \mathbf{M}\in SO(3) @f$
             */
            Eigen::Matrix< T, 4, 4 >& e ();

            MATRIXOPERATOR(T)
        };
    }}

    %template(CameraMatrixd) rw::math::CameraMatrix<double>;
// ######################### Constants
// ######################### EigenDeecomposition

    namespace rw { namespace math {
        //! @brief Type representing a set of eigen values and eigen vectors.
        template< class T = double > struct EigenDecomposition
        {
         public:
            /**
             * @brief Construct new decomposition.
             * @param vectors [in] the eigen vectors as columns in a matrix.
             * @param values [in] the corresponding eigen values.
             */
            EigenDecomposition (Eigen::Matrix< T, -1, -1 > vectors,
                                Eigen::Matrix< T, -1, 1 > values);

            /**
             * @brief returns all eigenvectors as columns in a matrix
             * @return reference to the matrix.
             */
            const Eigen::Matrix< T, -1, -1 >& getEigenVectors ();

            /**
             * @brief returns the i'th eigenvector
             * @return the eigen vector.
             */
            Eigen::Matrix< T, -1, 1 > getEigenVector (size_t i);

            /**
             * @brief return all eigenvalues
             * @return the eigen values.
             */
            const Eigen::Matrix< T, -1, 1 >& getEigenValues ();

            /**
             * @brief returns the i'th eigenvalue
             * @return the eigenvalue.
             */
            T getEigenValue (size_t i);

            /**
             * @brief sorts the eigen vectors according to their eigen value. The vector with smallest
             * eigen value has index 0
             */
            void sort ();
        };
    }}    // namespace rw::math

// ######################### Function
    namespace rw { namespace math {

        /**
         *  @brief Interface for functions
         */
        template< class RES_T = double, class ARG_T = double > class Function
        {
          public:

            /**
             * @brief Returns function value for arguments q.
             */
            virtual RES_T f (ARG_T q) = 0;

            /**
             * @brief Wraps the evaluation of x() with operator().
             */
            RES_T operator() (ARG_T q);
        };


        /**
         * @brief Interface for functions which are 1 time differentiable
         */
        template< class RES_T = double, class ARG_T = double, class GRAD_T = double >
        class Function1Diff : virtual public Function< RES_T, ARG_T >
        {
          public:
            /**
             * @brief Returns gradient(derivative) of the function
             */
            virtual GRAD_T df (ARG_T q) = 0;
        };
    }}    // namespace rw::math

    %template(Functiondd) rw::math::Function<double,double>;
    %template(Function1Diffddd) rw::math::Function1Diff<double,double,double>;
    %template(Function1DiffdddPtr) rw::core::Ptr<rw::math::Function1Diff<double,double,double>>;
    
    #define FUNCTUIN1DIFFDDD_TYPE rw::math::Function1Diff<double,double,double>
    OWNEDPTR(FUNCTUIN1DIFFDDD_TYPE);


// ######################### Line2D
    /**
     * @brief Describes a line segment in 2D.
     *
     */
    class Line2D
    {
      public:
        /**
         * @brief definition of intersection result values for the intersection test
         * between two lines.
         */
        enum IntersectResult {
            PARALLEL,      //! Two lines are parallel
            COINCIDENT,    //! Two lines are parallel and coinciding
            INTERSECTS     //! Two lines intersects at one point
        };

        /**
         * @brief Constructor
         */
        Line2D ();

        /**
         * @brief Creates a line between that intersect the two points p1 and p2.
         * @param p1 [in] point
         * @param p2 [in] point
         */
        Line2D (const rw::math::Vector2D<double>& p1, const rw::math::Vector2D<double>& p2);

        /**
         * @brief Creates a line between that intersect the two points (x1,y1) and (x2,y2).
         * @param x1 [in] x coordinate of point 1
         * @param y1 [in] y coordinate of point 1
         * @param x2 [in] x coordinate of point 2
         * @param y2 [in] y coordinate of point 2
         */
        Line2D (double x1, double y1, double x2, double y2);

        /**
         * @brief Destructor
         */
        virtual ~Line2D ();

        /**
         * @brief calculates the intersection between two lines. A intersection
         * point is only saved in res if the two lines are not parallel or coincident.
         * The intersection test does not take the segments into acount.
         * @param line [in] the line two test against
         * @param res [out] the point of intersection
         * @return the intersection type
         */
        IntersectResult getIntersect (const Line2D& line, rw::math::Vector2D<double>& res) const;

        /**
         * @brief calculates the angle between this line and \b line
         * @param line [in] a line
         * @return the angle from this line to \b line
         */
        double calcAngle (const Line2D& line) const;

        /**
         * @brief calculates the angle between the projection of this line onto
         * yz-plane and the x-axis
         * @return the angle
         */
        double calcAngle () const;

        /**
         * @brief calculates the shortest distance between point v and the infinite
         * line.
         * @param v [in] Point to which to calculate distance
         */
        double calcDist (const rw::math::Vector2D<double>& v) const;

        /**
         * @brief gets the length of the line segment.
         * @return line segment length
         */
        double getLength () const;

        /**
         * @brief first point on the line
         */
        rw::math::Vector2D<double>& p1 ();

        /**
         * @brief second point on the line
         */
        rw::math::Vector2D<double>& p2 ();
        /**
         * @brief calculates the unit normal of the line
         */
        rw::math::Vector2D<double> calcUnitNormal () const;
    };
// ######################### Line2DPolar
    /**
     * @brief Describes a line in 2D in polar coordinates.
     */
    class Line2DPolar
    {
      public:
        /**
         * @brief constructor
         *
         * rho * (cos(theta), sin(theta)) is the point on the line nearest to origo.
         *
         * @param rho [in] distance to the point on line which is closest to origo
         * @param theta [in] angle from x-axis up to the line that connects the origo and the
         * point on the line that is closest to origo.
         */
        Line2DPolar (double rho = 0, double theta = 0);

        /**
         * @brief constructor
         *
         * @param pnt [in] is any point on the line
         * @param theta [in]  angle in radians from x-axis up to the line that connects the origo
         * and the point on the line that is closest to origo.
         */
        Line2DPolar (const rw::math::Vector2D<double>& pnt, double theta);

        /**
         * @brief constructor - The line moving through the segment from 'start' to 'end'.
         * @param start [in] point on line
         * @param end [in] point on line
         */
        Line2DPolar (const rw::math::Vector2D<double>& start, const rw::math::Vector2D<double>& end);

        /**
         * @brief constructor - The line moving through the line segment.
         * @param line [in] the line described as a segment
         */
        Line2DPolar (const Line2D& line);

        /**
         * @brief the shortest distance from origo the line
         * @return
         */
        double getRho () const;

        /**
         * @brief angle in radians from x-axis up to the line that connects the origo and the
         * point on the line that is closest to origo.
         * @return
         */
        double getTheta () const;

        //! @brief get normal of line
        rw::math::Vector2D<double> calcNormal () const;

        //! @brief The L_2 distance from 'pnt' to the line.
        double dist2 (const rw::math::Vector2D<double>& pnt);

        //! The point for the projection of 'pnt' onto 'line'.
        static rw::math::Vector2D<double> projectionPoint (const Line2DPolar& line, const rw::math::Vector2D<double>& pnt);

        //! A supporting point on the line (equal to rho * normal).
        static rw::math::Vector2D<double> linePoint (const Line2DPolar& line);

        /**
         * @brief The vector for the projection of \b pnt onto the normal of \b line.
         * @param line [in] a line.
         * @param pnt [in] a point.
         * @return the projection vector.
         */
        static rw::math::Vector2D<double> normalProjectionVector (const Line2DPolar& line, const rw::math::Vector2D<double>& pnt);

        /**
         * @brief \b line given relative to the coordinate frame of \b pose.
         * @param pose [in] the pose.
         * @param line [in] the line.
         * @return a Line2DPolar.
         */
        static Line2DPolar lineToLocal (const rw::math::Pose2D<double>& pose, const Line2DPolar& line);

    };
// ######################### LinearAlgebra
    /*TODO(kalor) find a way to implement this
     * @brief Collection of Linear Algebra functions
     */
// ######################### MetricUtil
    /*TODO(kalor) find a way to implement this
     *  @brief Various metrics and other distance measures.
     */
// ######################### PerspectiveTransform2D
    namespace rw { namespace math {

        /**
         * @brief The PerspectiveTransform2D is a perspective transform in 2D.
         *
         * The homographic transform can be used to map one arbitrary 2D
         * quadrilateral into another.
         */
        template< class T > class PerspectiveTransform2D
        {
          public:
            /**
             * @brief constructor
             */
            PerspectiveTransform2D ();

            /**
             * @brief constructor
             */
            PerspectiveTransform2D (T r11, T r12, T r13, T r21, T r22, T r23, T r31, T r32,
                                    T r33);

            /**
             * @brief constructor
             * @param r
             * @return
             */
            explicit PerspectiveTransform2D (const Eigen::Matrix< T, 3, 3 >& r);

            /**
             * @brief calculates a PerspectiveTransform2D that maps points from point
             * set pts1 to point set pts2
             * @param pts1 [in] point set one
             * @param pts2 [in] point set two
             */
            static PerspectiveTransform2D< T > calcTransform (std::vector< Vector2D< T > > pts1,
                                                            std::vector< Vector2D< T > > pts2);

            /**
             * @brief Returns the inverse of the PerspectiveTransform
             */
            PerspectiveTransform2D< T > inverse ();

            /**
             * @brief Returns matrix element reference
             * @param row [in] row, row must be @f$ < 3 @f$
             * @param col [in] col, col must be @f$ < 3 @f$
             * @return reference to matrix element
             */
            T& operator() (std::size_t row, std::size_t col);

            /**
             * @brief transform a point using this perspective transform
             */
            Vector2D< T > operator* (const Vector2D< T >& v);

            /**
             * @brief transform a 2d point into a 3d point with this
             * perspective transform
             * @param hT
             * @param v
             * @return
             */
            Vector3D< T > calc3dVec (const PerspectiveTransform2D< T >& hT, const Vector2D< T >& v);


            /**
             * @brief Returns reference to the 3x3 matrix @f$ \mathbf{M}\in SO(3)
             * @f$ that represents this rotation
             *
             * @return @f$ \mathbf{M}\in SO(3) @f$
             */
            Eigen::Matrix<T,-1,-1>& e ();
        };

        /**
         * @brief Take the inverse of a PerspectiveTransform2D.
         * @param aRb [in] a PerspectiveTransform2D.
         * @return the inverse of \b aRb .
         * @relates rw::math::PerspectiveTransform2D
         */
        template< class T > PerspectiveTransform2D< T > inverse (const PerspectiveTransform2D< T >& aRb);
    }}

// ######################### PolynomialND 
    namespace rw { namespace math {

        /**
         * @brief Representation of a polynomial that can have non-scalar coefficients (polynomial
         * matrix).
         *
         * Representation of a polynomial of the following form:
         *
         * @f$
         *  f(x) = C_n x^n + C_(n-1) x^(n-1) + C_2 x^2 + C_1 x + C_0
         * @f$
         *
         * The polynomial is represented as a list of coefficients ordered from lowest-order term to
         * highest-order term, \f${c_0,c_1,...,c_n}\f$.
         */
        template< typename Coef, typename Scalar> class PolynomialND
        {
         public:
            /**
             * @brief Create polynomial with uninitialized coefficients.
             * @param order [in] the order of the polynomial.
             */
            explicit PolynomialND (std::size_t order) ;

            /**
             * @brief Create polynomial from vector.
             * @param coefficients [in] the coefficients ordered from lowest-order term to highest-order
             * term.
             */
            PolynomialND (const std::vector< Coef >& coefficients);

            /**
             * @brief Create polynomial from other polynomial.
             * @param p [in] the polynomial to copy.
             */
            PolynomialND (const rw::math::PolynomialND< Coef, Scalar >& p);

            /**
             * @brief Destructor
             */
            virtual ~PolynomialND ();

            /**
             * @brief Get the order of the polynomial (the highest power).
             * @return the order.
             */
            std::size_t order () const;

            /**
             * @brief Increase the order of this polynomial.
             * @param increase [in] how much to increase the order (default is 1).
             * @see increaseOrder(std::size_t,const Coef&) for a version that initializes the new
             * coefficients to a certain value.
             */
            void increaseOrder (std::size_t increase = 1);

            /**
             * @brief Increase the order of this polynomial.
             * @param increase [in] how much to increase the order (default is 1).
             * @param value [in] initialize new coefficients to this value.
             */
            void increaseOrder (std::size_t increase, const Coef& value);

            /**
             * @brief Evaluate the polynomial using Horner's Method.
             * @param x [in] the input parameter.
             * @return the value \f$f(x)\f$.
             */
            Coef evaluate (const Scalar& x) const;

            /**
             * @brief Evaluate the first \b n derivatives of the polynomial using Horner's Method.
             * @param x [in] the input parameter.
             * @param n [in] the number of derivatives to find (default is the first derivative only)
             * @return a vector of values \f${f(x),\dot{f}(x),\ddot{f}(x),\cdots}\f$.
             */
            std::vector< Coef > evaluateDerivatives (const Scalar& x, std::size_t n = 1) const;

            /**
             * @brief Perform deflation of polynomial.
             * @param x [in] a root of the polynomial.
             * @return a new polynomial of same order minus one.
             * @note There is no check that the given root is in fact a root of the polynomial.
             */
            rw::math::PolynomialND< Coef, Scalar > deflate (const Scalar& x) const;

            /**
             * @brief Get the derivative polynomial.
             * @param n [in] gives the n'th derivative (default is n=1).
             * @return a new polynomial of same order minus one.
             * @note To evaluate derivatives use the evaluate derivative method which is more precise.
             */
            rw::math::PolynomialND< Coef, Scalar > derivative (std::size_t n = 1) const;

            /**
             * @brief Get specific coefficient.
             * @param i [in] the power of the term to get coefficient for.
             * @return the coefficient.
             */
            Coef& operator() (size_t i);

            /**
             * @brief Scalar multiplication
             * @param s [in] scalar to multiply with.
             * @return new polynomial after multiplication.
             */
            const rw::math::PolynomialND< Coef, Scalar > operator* (Scalar s) const;

            /**
             * @brief Scalar division
             * @param s [in] scalar to divide with.
             * @return new polynomial after division.
             */
            const rw::math::PolynomialND< Coef, Scalar > operator/ (Scalar s) const;

            /**
             * @brief Scalar multiplication
             * @param s [in] the scalar to multiply with.
             * @return reference to same polynomial with changed coefficients.
             */
            rw::math::PolynomialND< Coef, Scalar >& operator*= (Scalar s);

            /**
             * @brief Scalar division
             * @param s [in] the scalar to divide with.
             * @return reference to same polynomial with changed coefficients.
             */
            rw::math::PolynomialND< Coef, Scalar >& operator/= (Scalar s);

            /**
             * @brief Scalar multiplication
             * @param s [in] scalar to multiply with.
             * @param p [in] polynomial to multiply with.
             * @return new polynomial after multiplication.
             */
            friend const rw::math::PolynomialND< Coef, Scalar > operator* (Scalar s,
                                                                const rw::math::PolynomialND< Coef, Scalar >& p);

            /**
             * @brief Polynomial subtraction.
             * @param b [in] polynomial of to subtract.
             * @return new polynomial after subtraction.
             */
            const rw::math::PolynomialND< Coef, Scalar > operator- (const rw::math::PolynomialND< Coef, Scalar >& b) const;

            /**
             * @brief Polynomial addition.
             * @param b [in] polynomial to add.
             * @return new polynomial after addition.
             */
            const rw::math::PolynomialND< Coef, Scalar > operator+ (const rw::math::PolynomialND< Coef, Scalar >& b) const;

            /**
             * @brief Polynomial multiplication.
             *
             * A convolution of the coefficients is used.
             * Notice that more efficient algorithms exist for polynomials with scalar coefficients.
             *
             * @param b [in] polynomial to multiply. Post-multiplication is used - the dimensions must
             * match.
             * @return new polynomial after multiplication.
             */
            template< typename OutCoef, typename Coef2 = Coef >
            rw::math::PolynomialND< OutCoef, Scalar > multiply (const rw::math::PolynomialND< Coef2, Scalar >& b) const;

            /**
             * @brief Multiply with a coefficient.
             *
             * Each coefficient is post-multiplied with the given coefficient.
             *
             * @param b [in] coefficient to multiply with. Post-multiplication is used - the dimensions
             * must match.
             * @return new polynomial after multiplication.
             */
            template< typename OutCoef, typename Coef2 = Coef >
            rw::math::PolynomialND< OutCoef, Scalar > multiply (const Coef2& b) const;

            #if !defined(SWIGPYTHON)
                /**
                 * @brief Assignment.
                 * @param b [in] the polynomial to take coefficients from.
                 * @return true if equal, false if not.
                 */
                void operator= (const rw::math::PolynomialND< Coef, Scalar >& b);
            #endif
            /**
             * @brief Negate coefficients.
             * @return new polynomial with coefficients negated.
             */
            const rw::math::PolynomialND< Coef, Scalar > operator- () const;

            /**
             * @brief Check if polynomials are equal.
             * @param b [in] the polynomial to compare with.
             * @return true if equal, false if not.
             */
            bool operator== (const rw::math::PolynomialND< Coef, Scalar >& b) const;

            #define polyND_type rw::math::PolynomialND<Coef,Scalar>
            TOSTRING(polyND_type)
            ARRAYOPERATOR(Coef)

        };

        /*
         * @brief Multiply 3D polynomial matrix with 3D polynomial vector.
         * @param A [in] the matrix expression.
         * @param b [in] the vector expression.
         * @return a 3D polynomial vector.
         */
        rw::math::PolynomialND< Eigen::Matrix<double,3,1>,double> operator* (const rw::math::PolynomialND< Eigen::Matrix<double,3,3>,double>& A,
                                                const rw::math::PolynomialND< Eigen::Matrix<double,3,1>,double >& b);

        /**
         * @brief Multiply 3D polynomial vector with 3D polynomial matrix.
         * @param a [in] the vector expression.
         * @param A [in] the matrix expression.
         * @return a 3D polynomial vector.
         */
        rw::math::PolynomialND< Eigen::Matrix< double, 1, 3 >,double >
        operator* (const rw::math::PolynomialND< Eigen::Matrix< double, 1, 3 >,double >& a,
                const rw::math::PolynomialND< Eigen::Matrix<double,3,3>,double>& A);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,3,3>>&, const
        //! rw::math::PolynomialND<Eigen::Matrix<double,3,1>>&)
        rw::math::PolynomialND< Eigen::Matrix<double,3,1>,double> operator* (const rw::math::PolynomialND< Eigen::Matrix<double,3,3>,double>& A,
                                                const Eigen::Matrix<double,3,1>& b);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,1,3>,double >&, const
        //! rw::math::PolynomialND<Eigen::Matrix<double,3,3>>&)
        rw::math::PolynomialND< Eigen::Matrix< double, 1, 3 >,double >
        operator* (const rw::math::PolynomialND< Eigen::Matrix< double, 1, 3 >,double >& a, const Eigen::Matrix<double,3,3>& A);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,3,3>>&, const
        //! rw::math::PolynomialND<Eigen::Matrix<double,3,1>>&)
        rw::math::PolynomialND< Eigen::Matrix<float,3,1>, float >
        operator* (const rw::math::PolynomialND< Eigen::Matrix<float,3,3>, float >& A,
                const rw::math::PolynomialND< Eigen::Matrix<float,3,1>, float >& b);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,1,3>,double >&, const
        //! rw::math::PolynomialND<Eigen::Matrix<double,3,3>>&)
        rw::math::PolynomialND< Eigen::Matrix< float, 1, 3 >, float >
        operator* (const rw::math::PolynomialND< Eigen::Matrix< float, 1, 3 >, float >& a,
                const rw::math::PolynomialND< Eigen::Matrix<float,3,3>, float >& A);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,3,3>>&, const Eigen::Matrix<double,3,1>&)
        rw::math::PolynomialND< Eigen::Matrix<float,3,1>, float >
        operator* (const rw::math::PolynomialND< Eigen::Matrix<float,3,3>, float >& A, const Eigen::Matrix<float,3,1>& b);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,1,3>,double >&, const Eigen::Matrix<double,3,3>&)
        rw::math::PolynomialND< Eigen::Matrix< float, 1, 3 >, float >
        operator* (const rw::math::PolynomialND< Eigen::Matrix< float, 1, 3 >, float >& a,
                const Eigen::Matrix<float,3,3>& A);

    }}    // namespace rw::math

    %template(PolynomialNDdDouble) rw::math::PolynomialND<double,double>;
    %template(PolynomialNDfFloat) rw::math::PolynomialND<float,float>;
    %template(PolynomialNDEigenRowVector3dDouble) rw::math::PolynomialND<Eigen::Matrix<double,1,3>,double>;
    %template(PolynomialNDEigenVector3dDouble) rw::math::PolynomialND<Eigen::Matrix<double,3,1>,double>;
    %template(PolynomialNDEigenMatrix3dDouble) rw::math::PolynomialND<Eigen::Matrix<double,3,3>,double>;
    %template(PolynomialNDEigenRowVector3fFloat) rw::math::PolynomialND<Eigen::Matrix<float,1,3>,float>;
    %template(PolynomialNDEigenVector3fFloat) rw::math::PolynomialND<Eigen::Matrix<float,3,1>,float>;
    %template(PolynomialNDEigenMatrix3fFloat) rw::math::PolynomialND<Eigen::Matrix<float,3,3>,float>;
    %template(PolynomialNDidComplexDouble) rw::math::PolynomialND<std::complex<double>,std::complex<double>>;
    %template(PolynomialNDEigenRowVector3idComplexDouble) rw::math::PolynomialND<Eigen::Matrix<std::complex<double>,1,3>,std::complex<double>>;
    %template(PolynomialNDEigenVector3idComplexDouble) rw::math::PolynomialND<Eigen::Matrix<std::complex<double>,3,1>,std::complex<double>>;
    %template(PolynomialNDEigenMatrix3ifComplexDouble) rw::math::PolynomialND<Eigen::Matrix<std::complex<float>,3,3>,std::complex<float>>;

// ######################### Polynomial
    namespace rw { namespace math {

        /**
         * @brief Representation of an ordinary polynomial with scalar coefficients (that can be both
         * real and complex).
         *
         * Representation of a polynomial of the following form:
         *
         * @f$
         *  f(x) = c_n x^n + c_(n-1) x^(n-1) + c_2 x^2 + c_1 x + c_0
         * @f$
         *
         * The polynomial is represented as a list of coefficients ordered from lowest-order term to
         * highest-order term, \f${c_0,c_1,...,c_n}\f$.
         */
        template< typename T = double > class Polynomial : public rw::math::PolynomialND< T, T >
        {
        public:
            /**
             * @brief Create polynomial with coefficients initialized to zero.
             * @param order [in] the order of the polynomial.
             */
            explicit Polynomial (std::size_t order);

            /**
             * @brief Create polynomial from vector.
             * @param coefficients [in] the coefficients ordered from lowest-order term to highest-order
             * term.
             */
            Polynomial (const std::vector< T >& coefficients);

            /**
             * @brief Create polynomial from other polynomial.
             * @param p [in] the polynomial to copy.
             */
            Polynomial (const rw::math::Polynomial< T >& p);

            /**
             * @brief Create polynomial from other polynomial.
             * @param p [in] the polynomial to copy.
             */
            Polynomial (const rw::math::PolynomialND< T, T >& p);

            /**
             * @brief Destructor
             */
            virtual ~Polynomial () {}

            //! @copydoc rw::math::PolynomialND<T,T>::evaluate
            T evaluate (const T& x) const;

            /**
             * @brief Evaluate the polynomial using Horner's Method.
             *
             * This function estimates the error of the result.
             * For float or double types, the error type, ErrT, should be the same as the type T.
             * For std::complex<float> or std::complex<double> types, the error type, ErrT, should be
             * either float or double.
             *
             * @param x [in] the input parameter.
             * @param err [out] estimate of the absolute error.
             * @return the value \f$f(x)\f$.
             * @note Error is the absolute size of the interval where \f$f(x)\f$ can be, assuming
             * coefficients are exact.
             */
            template< typename ErrT = T > T evaluate (const T& x, ErrT& err) const;

            //! @copydoc rw::math::PolynomialND<T,T>::evaluateDerivatives
            std::vector< T > evaluateDerivatives (const T& x, std::size_t n = 1) const;

            /**
             * @brief Evaluate the first \b n derivatives of the polynomial using Horner's Method.
             *
             * This function estimates the error of the result.
             * For float or double types, the error type, ErrT, should be the same as the type T.
             * For std::complex<float> or std::complex<double> types, the error type, ErrT, should be
             * either float or double.
             *
             * @param x [in] the input parameter.
             * @param err [out] estimate of the absolute errors.
             * @param n [in] the number of derivatives to find (default is the first derivative only)
             * @return the value \f$\dot{f}(x)\f$.
             * @note Error is the absolute size of the interval where \f$f(x)\f$ can be, assuming
             * coefficients are exact.
             */
            template< typename ErrT = T >
            std::vector< T > evaluateDerivatives (const T& x, std::vector< ErrT >& err,
                                                std::size_t n = 1) const;

            //! @copydoc rw::math::PolynomialND<T,T>::deflate
            rw::math::Polynomial< T > deflate (const T& x) const;

            //! @copydoc rw::math::PolynomialND<T,T>::derivative
            rw::math::Polynomial< T > derivative (std::size_t n = 1) const;

            /**
             * @brief Scalar addition
             * @param s [in] scalar to add.
             * @return new polynomial after addition.
             */
            const rw::math::Polynomial< T > operator+ (T s) const;
            /**
             * @brief Scalar subtraction
             * @param s [in] scalar to subtract.
             * @return new polynomial after subtraction.
             */
            const rw::math::Polynomial< T > operator- (T s) const;

            /**
             * @brief Scalar multiplication
             * @param s [in] scalar to multiply with.
             * @return new polynomial after multiplication.
             */
            const rw::math::Polynomial< T > operator* (T s) const;

            /**
             * @brief Polynomial multiplication
             *
             * This multiplication functions uses a convolution of the coefficients.
             * More efficient implementations are possible.
             *
             * @param polynomial [in] polynomial to multiply with.
             * @return new polynomial after multiplication.
             */
            const rw::math::Polynomial< T > operator* (const rw::math::Polynomial< T >& polynomial) const;

            /**
             * @brief Multiply polynomial with scalar coefficients with a 3D polynomial vector.
             * @param p [in] polynomial with scalar coefficients.
             * @param polynomial [in] polynomial vector.
             * @return a 3D polynomial vector.
             */
            friend rw::math::PolynomialND< Eigen::Matrix< T, 3, 1 >, T >
            operator* (const rw::math::Polynomial< T >& p,
                    const rw::math::PolynomialND< Eigen::Matrix< T, 3, 1 >, T >& polynomial);

            //! @copydoc operator*(const rw::math::Polynomial<T>&, const rw::math::PolynomialND<Eigen::Matrix<T,3,1>,T>&)
            friend rw::math::PolynomialND< Eigen::Matrix< T, 1, 3 >, T >
            operator* (const rw::math::Polynomial< T >& p,
                    const rw::math::PolynomialND< Eigen::Matrix< T, 1, 3 >, T >& polynomial);

            /**
             * @brief Multiply polynomial with scalar coefficients with a 3D polynomial matrix.
             * @param p [in] polynomial with scalar coefficients.
             * @param polynomial [in] polynomial matrix.
             * @return a 3D polynomial matrix.
             */
            friend rw::math::PolynomialND< Eigen::Matrix< T, 3, 3 >, T >
            operator* (const rw::math::Polynomial< T >& p,
                    const rw::math::PolynomialND< Eigen::Matrix< T, 3, 3 >, T >& polynomial);

            /**
             * @brief Multiply polynomial with scalar coefficients with a vector.
             * @param p [in] polynomial with scalar coefficients.
             * @param a [in] vector to multiply with.
             * @return a 3D polynomial vector.
             */
            friend rw::math::PolynomialND< Eigen::Matrix< T, 3, 1 >, T >
            operator* (const rw::math::Polynomial< T >& p, const Eigen::Matrix< T, 3, 1 >& a);

            //! @copydoc operator*(const rw::math::Polynomial<T>&, const Eigen::Matrix<T,3,1>&)
            friend rw::math::PolynomialND< Eigen::Matrix< T, 1, 3 >, T >
            operator* (const rw::math::Polynomial< T >& p, const Eigen::Matrix< T, 1, 3 >& a);

            /**
             * @brief Multiply polynomial with scalar coefficients with a matrix.
             * @param p [in] polynomial with scalar coefficients.
             * @param A [in] matrix to multiply with.
             * @return a 3D polynomial matrix.
             */
            friend rw::math::PolynomialND< Eigen::Matrix< T, 3, 3 >, T >
            operator* (const rw::math::Polynomial< T >& p, const Eigen::Matrix< T, 3, 3 >& A);

            /**
             * @brief Scalar division
             * @param s [in] scalar to divide with.
             * @return new polynomial after division.
             */
            const rw::math::Polynomial< T > operator/ (T s) const;

            /**
             * @brief Scalar addition
             * @param s [in] scalar to add.
             * @return same polynomial with coefficients changed.
             */
            rw::math::Polynomial< T >& operator+= (T s);

            /**
             * @brief Scalar subtraction
             * @param s [in] scalar to subtract.
             * @return same polynomial with coefficients changed.
             */
            rw::math::Polynomial< T >& operator-= (T s);

            /**
             * @brief Scalar multiplication
             * @param s [in] the scalar to multiply with.
             * @return reference to same polynomial with changed coefficients.
             */
            rw::math::Polynomial< T >& operator*= (T s);

            /**
             * @brief Scalar division
             * @param s [in] the scalar to divide with.
             * @return reference to same polynomial with changed coefficients.
             */
            rw::math::Polynomial< T >& operator/= (T s);

            /**
             * @brief Scalar multiplication
             * @param s [in] scalar to multiply with.
             * @param p [in] polynomial to multiply with.
             * @return new polynomial after multiplication.
             */
            friend const rw::math::Polynomial< T > operator* (T s, const rw::math::Polynomial< T >& p);

            /**
             * @brief Polynomial subtraction.
             * @param b [in] polynomial of to subtract.
             * @return new polynomial after subtraction.
             */
            const rw::math::Polynomial< T > operator- (const rw::math::Polynomial< T >& b) const;

            /**
             * @brief Polynomial subtraction.
             * @param b [in] polynomial to subtract.
             * @return same polynomial with different coefficients after subtraction.
             */
            rw::math::Polynomial< T >& operator-= (const rw::math::Polynomial< T >& b);

            /**
             * @brief Polynomial addition.
             * @param b [in] polynomial to add.
             * @return new polynomial after addition.
             */
            const rw::math::Polynomial< T > operator+ (const rw::math::Polynomial< T >& b) const;

            /**
             * @brief Polynomial addition.
             * @param b [in] polynomial to add.
             * @return same polynomial with different coefficients after addition.
             */
            rw::math::Polynomial< T >& operator+= (const rw::math::Polynomial< T >& b);

            #if !defined(SWIGPYTHON)
                /**
                 * @brief Assignment.
                 * @param b [in] the polynomial to take coefficients from.
                 * @return true if equal, false if not.
                 */
                void operator= (const rw::math::Polynomial< T >& b);
            #endif

            /**
             * @brief Negate coefficients.
             * @return new polynomial with coefficients negated.
             */
            const rw::math::Polynomial< T > operator- () const;

            /**
             * @brief Check if polynomials are equal.
             * @param b [in] the polynomial to compare with.
             * @return true if equal, false if not.
             */
            bool operator== (const rw::math::Polynomial< T >& b) const;
            /**
             * @brief Cast to other type.
             * @return a new polynomial after cast to new type.
             */
            template< class Q > const rw::math::Polynomial< Q > cast ();

            TOSTRING(rw::math::Polynomial<T>)
            ARRAYOPERATOR(T)
        };

        /**
         * @brief Multiply 3D polynomial vector with 3D polynomial vector.
         * @param a [in] first polynomial vector (row vector).
         * @param b [in] second polynomial vector (column vector).
         * @return a polynomial with scalar coefficients.
         */
        rw::math::Polynomial<double> operator* (const rw::math::PolynomialND< Eigen::Matrix< double, 1, 3 >,double >& a,
                                const rw::math::PolynomialND< Eigen::Matrix< double, 3, 1 >,double >& b);

        /**
         * @brief Multiply 3D polynomial vector with a polynomial with scalar coefficients.
         * @param polynomial [in] the polynomial vector.
         * @param p [in] polynomial with scalar coefficients.
         * @return a 3D polynomial vector.
         */
        rw::math::PolynomialND< Eigen::Matrix<double,3,1>,double> operator* (const rw::math::PolynomialND< Eigen::Matrix<double,3,1>,double>& polynomial,
                                                const rw::math::Polynomial<double>& p);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,3,1>>&, const rw::math::Polynomial<double>&)
        rw::math::PolynomialND< Eigen::Matrix< double, 1, 3 >,double >
        operator* (const rw::math::PolynomialND< Eigen::Matrix< double, 1, 3 >,double >& polynomial,
                const rw::math::Polynomial<double>& p);

        /**
         * @brief Multiply 3D polynomial matrix with a polynomial with scalar coefficients.
         * @param polynomial [in] the polynomial matrix.
         * @param p [in] polynomial with scalar coefficients.
         * @return a 3D polynomial matrix.
         */
        rw::math::PolynomialND< Eigen::Matrix<double,3,3>,double> operator* (const rw::math::PolynomialND< Eigen::Matrix<double,3,3>,double>& polynomial,
                                                const rw::math::Polynomial<double>& p);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,1,3>,double >&, const
        //! rw::math::PolynomialND<Eigen::Matrix<double,3,1> >&)
        rw::math::Polynomial< float > operator* (const rw::math::PolynomialND< Eigen::Matrix< float, 1, 3 >, float >& a,
                                    const rw::math::PolynomialND< Eigen::Matrix< float, 3, 1 >, float >& b);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,3,1>>&, const rw::math::Polynomial<double>&)
        rw::math::PolynomialND< Eigen::Matrix<float,3,1>, float >
        operator* (const rw::math::PolynomialND< Eigen::Matrix<float,3,1>, float >& polynomial,
                const rw::math::Polynomial< float >& p);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,1,3>,double >&, const rw::math::Polynomial<double>&)
        rw::math::PolynomialND< Eigen::Matrix< float, 1, 3 >, float >
        operator* (const rw::math::PolynomialND< Eigen::Matrix< float, 1, 3 >, float >& polynomial,
                const rw::math::Polynomial< float >& p);

        //! @copydoc operator*(const rw::math::PolynomialND<Eigen::Matrix<double,3,3> >&, const rw::math::Polynomial<double>&)
        rw::math::PolynomialND< Eigen::Matrix<float,3,3>, float >
        operator* (const rw::math::PolynomialND< Eigen::Matrix<float,3,3>, float >& polynomial,
                const rw::math::Polynomial< float >& p);
    }}
    %template(Polynomialf) rw::math::Polynomial<float>;
    %template(Polynomiald) rw::math::Polynomial<double>;
    %template(Polynomialid) rw::math::Polynomial<std::complex<double>>;

// ######################### PolynomialSolver

    /**
     * @brief Find solutions for roots of real and complex polynomial equations.
     *
     * The solutions are found analytically if the polynomial is of maximum order 3.
     * For polynomials of order 4 and higher, Laguerre's Method is used to find
     * roots until the polynomial can be deflated to order 3.
     * The remaining roots will then be found analytically.
     *
     * Some Polynomials are particularly easy to solve. A polynomial of the form
     * \f$ a x^n + b = 0\f$
     * will be solved by taking the n'th roots of \f$-\frac{b}{a}\f$ directly, giving n distinct
     * roots in the complex plane.
     *
     * To illustrate the procedure, consider the equation:
     * \f$ 10^{-15} x^8 - 10^{-15} x^7 + x^7 + 2 x^6 - x^4 - 2x^3 + 10^{-15} x= 0\f$
     *
     * The solver will use the following procedure (here with the precision \f$ \epsilon =
     * 10^{-14}\f$):
     *
     * 1. Remove terms that are small compared to \f$\epsilon\f$: \f$ x^7 + 2 x^6 - x^4 - 2x^3 =
     * 0\f$
     *
     * 2. Find zero roots and reduce the order: There is a triple root in x = 0 and the remaining
     * polynomial becomes: \f$ x^4 + 2 x^3 - x - 2 = 0\f$.
     *
     * 3. Use Laguerre to find a root of \f$ x^4 + 2 x^3 - x - 2 = 0\f$
     *
     * Depending on the initial guess for Laguerre, different roots might be found first.
     * The algorithm will proceed differently depending on the found root:
     *
     * 1. If root x=-2 is found, remaining polynomial after deflation is \f$ x^3 -1 = 0\f$.
     * The roots are found directly as the cubic root of 1, which is three distinct roots in the
     * complex plane (one is on the real axis).
     *
     * 2. If root x=1 is found, remaining polynomial after deflation is \f$ x^3 + 3 x^2 +3 x + 2 =
     * 0\f$. The roots are found analytically, giving one real root x=-2 and two complex conjugate
     * roots \f$x = -0.5 \pm \frac{\sqrt{3}}{2} i\f$.
     *
     * 3. If other roots than x=1 or x=-2 is found (a complex root), remaining polynomial is a third
     * order polynomial with complex coefficients. This polynomial is solved analytically to give
     * remaining two real roots, and one remaining complex root.
     *
     * Notice that cases 2+3 requires analytical solution of the third order polynomial equation.
     * For higher order polynomials Laguerre would need to be used to find the next root.
     * In this case it is particularly lucky to hit case 1, as this gives the solutions right away
     * no matter what order the remaining polynomial is.
     */
    class PolynomialSolver
    {
      public:
        /**
         * @brief Create a solver for a polynomial with real coefficients.
         * @param polynomial [in] the polynomial to find roots for.
         */
        PolynomialSolver (const rw::math::Polynomial< double >& polynomial);

        /**
         * @brief Create a solver for a polynomial with complex coefficients.
         * @param polynomial [in] the polynomial to find roots for.
         */
        PolynomialSolver (const rw::math::Polynomial< std::complex< double > >& polynomial);

        /**
         * @brief Destructor
         */
        virtual ~PolynomialSolver ();

        /**
         * @brief Use a specific initial guess for a root.
         * @param guess [in] a complex initial guess for the algorithm.
         */
        void setInitialGuess (std::complex< double > guess = 0);

        /**
         * @brief Get all real solutions of the equation.
         * @param epsilon [in] the root is considered a real root if \f$ |im(x)| \leq 2 \epsilon
         * |real(x)|\f$ .
         * @return a list of real solutions.
         * @throws rw::core::Exception if the Laguerre method fails, or the maximum number of
         * iterations has been reached.
         * @see PolynomialSolver for more details about the method used.
         */
        std::vector< double > getRealSolutions (double epsilon = 1.0e-14);

        /**
         * @brief Get all solutions of the equation including complex solutions.
         * @param epsilon [in] highest order coefficients will be removed if they have absolute real
         * and imaginary values less than \f$ \epsilon \f$ .
         * @return a list of complex solutions.
         * @throws rw::core::Exception if the Laguerre method fails, or the maximum number of
         * iterations has been reached.
         * @see PolynomialSolver for more details about the method used.
         */
        virtual std::vector< std::complex< double > > getSolutions (double epsilon = 1.0e-14);

        /**
         * @brief Set the number of iterations to take in the Laguerre method.
         * @param iterations [in] the maximum number of iterations (default is 10).
         */
        void setLaguerreIterations (unsigned int iterations = 10);

    };

// ######################### Pose2D
    namespace rw { namespace math {
        /**
         * @brief A Pose3D @f$ \mathbf{x}\in \mathbb{R}^6 @f$ describes a position
         * and orientation in 3-dimensions.
         *
         * @f$ {\mathbf{x}} = \left[
         *  \begin{array}{c}
         *  x \\
         *  y \\
         *  z \\
         *  \theta k_x \\
         *  \theta k_y \\
         *  \theta k_z
         *  \end{array}
         *  \right]
         *  @f$
         *
         * where @f$ (x,y,z)@f$ is the 3d position and @f$ (\theta k_x, \theta k_y,
         * \theta k_z)@f$ describes the orientation in equal angle axis (EAA)
         * format.
         */
        template< class T >

        class Pose2D
        {
        public:
            //! @brief Zero-initialized Pose2D.
            Pose2D ();

            /**
             * @brief Constructor.
             * @param pos [in] the position.
             * @param theta [in] the angle.
             */
            Pose2D (rw::math::Vector2D< T > pos, T theta);

            /**
             * @brief Constructor.
             * @param x [in] the value of the first position dimension.
             * @param y [in] the value of the second position dimension.
             * @param theta [in] the angle.
             */
            Pose2D (T x, T y, T theta);
            /**
             * @brief Constructor.
             * @param transform [in] a 2D transform giving the pose.
             */
            Pose2D (const rw::math::Transform2D< T >& transform);

            /**
             * @brief Get the first dimension of the position vector.
             * @return the position in the first dimension.
             */
            T& x ();

            /**
             * @brief Get the second dimension of the position vector.
             * @return the position in the second dimension.
             */
            T& y ();

            /**
             * @brief Get the angle.
             * @return the angle.
             */
            T& theta ();

            /**
             * @brief Get the position vector.
             * @return the position.
             */
            rw::math::Vector2D< T >& getPos ();

            //! @copydoc getPos()
            const rw::math::Vector2D< T >& getPos () const;

            /**
             * @brief Returns reference to vector element (x,y,theta)
             *
             * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
             *
             * @return const reference to element
             */
            const T& operator() (size_t i) const;

            /**
             * @brief The transform corresponding to the pose.
             * @param pose [in] the pose.
             * @return equivalent 2D transform.
             */
            static rw::math::Transform2D< T > transform (const Pose2D< T >& pose);

            /**
             * @brief return a Eigen vector of (x, y, theta).
             * @return Eigen vector.
             */
            Eigen::Matrix< T, 3, 1 > e () const;

            TOSTRING(rw::math::Pose2D<T>)
            ARRAYOPERATOR(T)
        };
    }} 

    %template(Pose2Dd) rw::math::Pose2D<double>;
    %template(Pose2Df) rw::math::Pose2D<float>;
// ######################### Random
    /**
     * @brief Generation of random numbers.
     */
    class Random
    {
      public:
        Random ()  = delete;
        ~Random () = delete;

        /**
         * @brief A random double in the range [0, 1[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static double ran ();

        /**
         * @brief Seeds the random number generator.
         *
         * @note Uses boost::random
         */
        static void seed (unsigned seed);

        /**
         * @brief Seeds the random number generator with current time of day
         *
         * @note Uses boost::random
         */
        static void seed ();

        /**
         * @brief A random double in the range [from, to[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static double ran (double from, double to);

        /**
         * @brief A random integer in the range [from, to[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static int ranI (int from, int to);

        /**
         * @brief Returns a random sample around \b mean with standard deviation \b sigma  using the
         * normal distribution.
         *
         * @note Uses boost::random
         *
         * @param mean [in] Means value
         * @param sigma [in] Standard deviation
         * @return Random sample
         */
        static double ranNormalDist (double mean, double sigma);
    };
// ######################### Rotation2D
    namespace rw { namespace math {

        /**
         * @brief A 2x2 rotation matrix \f$ \mathbf{R}\in SO(2) \f$
         *
         * @f$
         *  \mathbf{R}=
         *  \left[
         *  \begin{array}{cc}
         *  {}^A\hat{X}_B & {}^A\hat{Y}_B
         *  \end{array}
         *  \right]
         *  =
         *  \left[
         *  \begin{array}{cc}
         *  r_{11} & r_{12} \\
         *  r_{21} & r_{22}
         *  \end{array}
         *  \right]
         * @f$
         */
        template< class T> class Rotation2D
        {
        public:
            /**
             @brief A rotation matrix with uninitialized storage.
            */
            Rotation2D ();

            /**
             * @brief Constructs an initialized 2x2 rotation matrix
             *
             * @param r11 \f$ r_{11} \f$
             * @param r12 \f$ r_{12} \f$
             * @param r21 \f$ r_{21} \f$
             * @param r22 \f$ r_{22} \f$
             *
             * @f$
             *  \mathbf{R} =
             *  \left[
             *  \begin{array}{cc}
             *  r_{11} & r_{12} \\
             *  r_{21} & r_{22}
             *  \end{array}
             *  \right]
             * @f$
             */
            Rotation2D (T r11, T r12, T r21, T r22);

            /**
             * @brief Constructs an initialized 2x2 rotation matrix
             * @f$ \robabx{a}{b}{\mathbf{R}} =
             * \left[
             *  \begin{array}{cc}
             *   \robabx{a}{b}{\mathbf{i}} & \robabx{a}{b}{\mathbf{j}}
             *  \end{array}
             * \right]
             * @f$
             *
             * @param i @f$ \robabx{a}{b}{\mathbf{i}} @f$
             * @param j @f$ \robabx{a}{b}{\mathbf{j}} @f$
             */
            Rotation2D (const Vector2D< T >& i, const Vector2D< T >& j);

            /**
             * @brief Constructs an initialized 2x2 rotation matrix
             * @f$ \robabx{a}{b}{\mathbf{R}} =
             * \left[
             *  \begin{array}{cc}
             *   \robabx{a}{b}{\mathbf{i}} & \robabx{a}{b}{\mathbf{j}}
             *  \end{array}
             * \right]
             * @f$
             *
             * @param theta
             */
            Rotation2D (const T theta);

            /**
             @brief Construct an initialized 2x2 rotation matrix.

            The second of column of the matrix is deduced from the first column.

            @param i [in] The first column of the rotation matrix.
            */
            Rotation2D (const Vector2D< T >& i);

            /**
             * @brief Constructs a 2x2 rotation matrix set to identity
             * @return a 2x2 identity rotation matrix
             *
             * @f$
             * \mathbf{R} =
             * \left[
             * \begin{array}{cc}
             * 1 & 0\\
             * 0 & 1
             * \end{array}
             * \right]
             * @f$
             */
            static const Rotation2D& identity ();

            /**
             * @brief Comparison operator.
             *
             * The comparison operator makes a element wise comparison.
             * Returns true only if all elements are equal.
             *
             * @param rhs [in] Rotation2D to compare with
             * @return True if equal.
             */
            bool operator== (const Rotation2D< T >& rhs) const;

            /**
             * @brief Comparison operator.
             *
             * The comparison operator makes a element wise comparison.
             * Returns true if any of the elements are different.
             *
             * @param rhs [in] Rotation2D to compare with
             * @return True if not equal.
             */
            bool operator!= (const Rotation2D< T >& rhs) const;

            /**
             * @brief Returns a boost 2x2 matrix @f$ \mathbf{M}\in SO(2)
             * @f$ that represents this rotation
             *
             * @return @f$ \mathbf{M}\in SO(2) @f$
             */
            Eigen::Matrix<T,2,2> e ();

            /**
             * @brief Calculates \f$ \robabx{a}{c}{\mathbf{R}} =
             * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{R}} \f$
             *
             * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
             *
             * @param bRc [in] \f$ \robabx{b}{c}{\mathbf{R}} \f$
             *
             * @return \f$ \robabx{a}{c}{\mathbf{R}} \f$
             */
            friend const Rotation2D operator* (const Rotation2D& aRb, const Rotation2D& bRc);

            /**
             * @brief Calculates \f$ \robabx{a}{c}{\mathbf{v}} =
             * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{v}} \f$
             *
             * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
             * @param bVc [in] \f$ \robabx{b}{c}{\mathbf{v}} \f$
             * @return \f$ \robabx{a}{c}{\mathbf{v}} \f$
             */
            friend const Vector2D< T > operator* (const Rotation2D& aRb, const Vector2D< T >& bVc);

            TOSTRING(rw::math::Rotation2D<T>)
            MATRIXOPERATOR(T)
        };

        /**
         * @brief Casts Rotation2D<T> to Rotation2D<Q>
         * @param rot [in] Rotation2D with type T
         * @return Rotation2D with type R
         */
        template< class R, class T > const Rotation2D< R > cast (const Rotation2D< T >& rot);

        /**
         * @brief The inverse @f$ \robabx{b}{a}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}}^{-1} @f$ of a rotation matrix
         *
         * @relates Rotation2D
         *
         * @param aRb [in] the rotation matrix @f$ \robabx{a}{b}{\mathbf{R}} @f$
         *
         * @return the matrix inverse @f$ \robabx{b}{a}{\mathbf{R}} =
         * \robabx{a}{b}{\mathbf{R}}^{-1} @f$
         *
         * @f$ \robabx{b}{a}{\mathbf{R}} = \robabx{a}{b}{\mathbf{R}}^{-1} =
         * \robabx{a}{b}{\mathbf{R}}^T @f$
         */
        template< class T > const Rotation2D< T > inverse (const Rotation2D< T >& aRb);

        /**
         * @brief Find the transpose of \b aRb.
         *
         * The transpose of a rotation matrix is the same as the inverse.
         */
        template< class T > const Rotation2D< T > transpose (const Rotation2D< T >& aRb);

    }} 

    %template(Rotation2Dd) rw::math::Rotation2D<double>;
    %template(Rotation2Df) rw::math::Rotation2D<float>;

// ######################### Statistics
    namespace rw { namespace math {

        /**
         * @brief Class for collecting data and calculating simple statistics.
         */
        template< class T > class Statistics
        {
        public:
            /**
             * @brief Calculates the mean for a list of data
             */
            template< class V > static T mean (const V& data);

            /**
             * @brief Calculates the mean for a list of angles.
             */
            template< class V > static T angularMean (const V& data);

            /**
             * @brief Calcualtes the median for a list of data
             */
            template< class V > static T median (const V& data);

            /**
             * @brief Calculates the variance for a list of data
             * @param data [in] data
             * @param mean [in] The mean value of the data
             * @return variance of data
             */
            template< class V > static T variance (const V& data, const T& mean);

            /**
             * @brief Calculates the variance for a list of angles.
             */
            template< class V > static T angularVariance (const V& data, const T& mean);

            /**
             * Calculates the mean and variance of a list of data
             */
            template< class V > static std::pair< T, T > meanAndVariance (const V& data);

            /**
             * Calculates the mean and variance of a list of angles.
             */
            template< class V > static std::pair< T, T > angularMeanAndVariance (const V& data);

            /**
             * @brief Finds the minimal value in \b data
             */
            template< class V > static T minValue (const V& data);

            /**
             * @brief Finds the maximal value in \b data
             */
            template< class V > static T maxValue (const V& data);

            /**
             * @brief Finds the minimal and maximal value in \b data
             */
            template< class V > static std::pair< T, T > minAndMaxValue (const V& data);

            /**
             * @brief Returns the mean of the values added
             *
             * The mean is computed as @f$ \frac{1}{n} \Sigma_{d\in data}d @f$
             */
            T mean () const;

            /**
             * @brief Returns the angular mean of the values added
             *
             * The angular mean is computed as @f$ \tan^{-1}\frac{\Sigma_{d\in
             * data}\sin(d)}{\Sigma_{d\in data}\cos(d)} @f$
             */
            T angularMean () const;

            /**
             * @brief Returns the median of the values added
             *
             * Given an equal number of element, the mean is calculated as the average of the two center
             * elements.
             */
            T median () const;

            /**
             * @brief Returns the variance of the values added
             * The variance is computed as @f$ \frac{1}{n-1} \Sigma_{d\in data}(m-\mu)^2 @f$
             * where @f$ \mu @f$ is the mean of the data.
             */
            T variance () const;

            /**
             * @brief Returns the angular variance of the values added
             * The variance is computed as @f$ \frac{1}{n-1} \Sigma_{d\in
             * data}\left[\cos^{-1}(\sin(d)\sin(\mu)-\cos(d)\cos(\mu))\right]^2 @f$ where @f$ \mu @f$ is
             * the angular mean of the data.
             */
            T angularVariance () const;

            /**
             * @brief returns the mean and the variance of the data.
             *
             * See documentation of Statistics::mean() and Statistics::variance()
             * for how the mean and variane are computed.
             */
            std::pair< T, T > meanAndVariance () const;

            /**
             * @brief returns the angular mean and the variance of the data.
             *
             * See documentation of Statistics::angularMean() and Statistics::angularVariance()
             * for how the mean and variane are computed.
             */
            std::pair< T, T > angularMeanAndVariance () const;

            /**
             * @brief Returns the minimum value of data added.
             *
             * If no data is added 0 is returned.
             */
            T minValue () const;

            /**
             * @brief Returns the maximum value of data added.
             *
             * If no data is added 0 is returned.
             */
            T maxValue () const;

            /**
             * @brief Returns pair containing the minimum and maximum value of the data added.
             *
             * If no data is added 0 is returned for both values.
             */
            std::pair< T, T > minAndMaxValue () const;

            /**
             * @brief Add data to statistics
             */
            void add (const T& t);

            /**
             * @brief Clear the recorded statistics data
             */
            void clear ();

            /**
             * @brief Provides reference to the internal data container
             */
            const std::list< T >& data () const;

            TOSTRING(rw::math::Statistics<T>)
        };
    }} 

// ######################### Transform2D
    namespace rw { namespace math {
        /**
         * @brief A 4x4 homogeneous transform matrix @f$ \mathbf{T}\in SE(3) @f$
         *
         * @f$
         * \mathbf{T} =
         * \left[
         *  \begin{array}{cc}
         *  \mathbf{R} & \mathbf{d} \\
         *  \begin{array}{ccc}0 & 0 & 0\end{array} & 1
         *  \end{array}
         * \right]
         * @f$
         *
         */
        template< class T = double > class Transform2D
        {
        public:
            /**
             * @brief Default Constructor.
             *
             * Initializes with 0 translation and Identity matrix as rotation
             */
            Transform2D ();

            /**
             * @brief Constructs a homogeneous transform
             * @param d [in] @f$ \mathbf{d} @f$ A 3x1 translation vector
             * @param R [in] @f$ \mathbf{R} @f$ A 3x3 rotation matrix
             */
            Transform2D (const Vector2D< T >& d, const Rotation2D< T >& R);

            /**
             * @brief Constructs the identity transform
             * @return the identity transform
             *
             * @f$
             * \mathbf{T} =
             * \left[
             * \begin{array}{ccc}
             * 1 & 0 & 0 \\
             * 0 & 1 & 0 \\
             * 0 & 0 & 1 \\
             * \end{array}
             * \right]
             * @f$
             */
            static const Transform2D& identity ();

            /**
                 @brief Calculates @f$ \robabx{a}{c}{\mathbf{T}} =
                \robabx{a}{b}{\mathbf{T}} \robabx{b}{c}{\mathbf{T}} @f$

                @param aTb [in] @f$ \robabx{a}{b}{\mathbf{T}} @f$
                @param bTc [in] @f$ \robabx{b}{c}{\mathbf{T}} @f$
                @return @f$ \robabx{a}{c}{\mathbf{T}} @f$

                @f$
                \robabx{a}{c}{\mathbf{T}} =
                \left[
                \begin{array}{cc}
                \robabx{a}{b}{\mathbf{R}}\robabx{b}{c}{\mathbf{R}} &
                \robabx{a}{b}{\mathbf{d}} + \robabx{a}{b}{\mathbf{R}}\robabx{b}{c}{\mathbf{d}} \\
                \begin{array}{ccc} 0 & 0 & 0 \end{array} & 1
                \end{array}
                \right]
                @f$
            */
            friend const Transform2D operator* (const Transform2D& aTb, const Transform2D& bTc);

            /**
             @brief Calculates @f$ \robax{a}{\mathbf{p}} =
            \robabx{a}{b}{\mathbf{T}} \robax{b}{\mathbf{p}} \f$ thus transforming
            point @f$ \mathbf{p} @f$ from frame @f$ b @f$ to frame @f$ a @f$

            @param aTb [in] @f$ \robabx{a}{c}{\mathbf{T}} @f$
            @param bP [in] @f$ \robax{b}{\mathbf{p}} @f$
            @return @f$ \robax{a}{\mathbf{p}} @f$
            */
            friend const Vector2D< T > operator* (const Transform2D& aTb, const Vector2D< T >& bP);

            /**
             * @brief Gets the rotation part @f$ \mathbf{R} @f$ from @f$ \mathbf{T} @f$
             * @return @f$ \mathbf{R} @f$
             */
            Rotation2D< T >& R ();

            /**
             * \brief Gets the position part @f$ \mathbf{d} @f$ from @f$ \mathbf{T} @f$
             * \return @f$ \mathbf{d} @f$
             */
            Vector2D< T >& P ();

            TOSTRING(rw::math::Transform2D<T>)
            MATRIXOPERATOR(T)
        };

        /**
         * @brief Cast Transform2D<T> to Transform2D<Q>
         * @param trans [in] Transform2D with type T
         * @return Transform2D with type Q
         */
        template< class Q, class T > const Transform2D< Q > cast (const Transform2D< T >& trans);

        /**
         * @brief Calculates @f$ \robabx{b}{a}{\mathbf{T}} = \robabx{a}{b}{\mathbf{T}}^{-1} @f$
         * @relates Transform2D
         * @param aTb [in] the transform matrix @f$ \robabx{a}{b}{\mathbf{T}} @f$
         * @return @f$ \robabx{b}{a}{\mathbf{T}} = \robabx{a}{b}{\mathbf{T}}^{-1} @f$
         *
         * @f$
         * \robabx{a}{b}{\mathbf{T}}^{-1} =
         * \left[
         *  \begin{array}{cc}
         *  \robabx{a}{b}{\mathbf{R}}^{T} & - \robabx{a}{b}{\mathbf{R}}^{T} \robabx{a}{b}{\mathbf{d}} \\
         *  \begin{array}{ccc}0 & 0 & 0\end{array} & 1
         *  \end{array}
         * \right]
         *
         * @f$
         */
        template< class T > const Transform2D< T > inverse (const Transform2D< T >& aTb);

    }}  

    %template(Transform2Dd) rw::math::Transform2D<double>;
    %template(Transform2Df) rw::math::Transform2D<float>;

// ######################### Vector
    namespace rw { namespace math {
        /**
         * @brief Configuration vector
         */
        template< class T> class Vector
        {
          public:

            /**
             * @brief A configuration of vector of length \b dim.
             */
            explicit Vector (size_t dim);

            /**
             * @brief Default constructor.
             *
             * The vector will be of dimension zero.
             */
            Vector ();

            /**
             * @brief Creates a Vector of length \b n and initialized with values from \b values
             *
             * The method reads n values from \b values and do not check whether reading out of bounds.
             *
             * @param n [in] Length of q.
             * @param values [in] Values to initialize with
             */
            Vector (size_t n, const T* values);

            /**
             * @brief Creates a Vector of length \b n and initialize all values in Vector to \b value
             *
             * @param n [in] Length of q.
             * @param value [in] Value to initialize
             */
            Vector (size_t n, T value);

            /**
             * @brief Returns Vector of length \b n initialized with 0's
             */
            static Vector zero (int n);

            /**
             * @brief The dimension of the configuration vector.
             */
            size_t size ();

            /**
             @brief True if the configuration is of dimension zero.
            */
            bool empty ();

            /**
             * @brief Construct a configuration vector from a Boost vector
             * expression.
             *
             * @param r [in] An expression for a vector of doubles
             */
            template< class R > explicit Vector (const Eigen::Matrix< R , -1, 1 >& r);

            /**
             * @brief Accessor for the internal Eigen vector state.
             */
            Eigen::Matrix<T,-1,1>& e ();

            /**
             * @brief Extracts a sub part (range) of this Vector.
             * @param start [in] Start index
             * @param cnt [in] the number of elements to include
             * @return
             */
            const Vector getSubPart (size_t start, size_t cnt) const;

            /**
             * @brief Set a part of the vector.
             * @param index [in] first index.
             * @param part [in] the subpart to set.
             */
            void setSubPart (size_t index, const Vector& part);

            //----------------------------------------------------------------------
            // Norm utility methods

            /**
             * @brief Returns the Euclidean norm (2-norm) of the configuration
             * @return the norm
             */
            T norm2 ();

            /**
             * @brief Returns the Manhatten norm (1-norm) of the configuration
             * @return the norm
             */
            T norm1 ();

            /**
             * @brief Returns the infinte norm (\f$\inf\f$-norm) of the configuration
             * @return the norm
             */
            T normInf () const;

            //----------------------------------------------------------------------
            // Various operators

            /**
             @brief Scalar division.
            */
            const Vector operator/ (T s) const;

            /**
             * @brief Scalar multiplication.
             */
            const Vector operator* (T s) const;

            /**
             * @brief Scalar multiplication.
             */
            friend const Vector operator* (T s, const Vector& v);

            /**
             * @brief Vector subtraction.
             */
            const Vector operator- (const Vector& b) const;

            /**
             * @brief Vector addition.
             */
            const Vector operator+ (const Vector& b) const;

            /**
             * @brief Unary minus.
             */
            const Vector operator- () const;

            /**
             * @brief Compares whether this is less than \b q
             *
             * The less operator is defined such that the first index is the most significant. That is
             * if (*this)[0] < q[0] then true is returned. If (*this)[0] > q[0] false is returned and
             * only if (*this)[0] == q[0] is the next index considered.
             */
            bool operator< (const Vector& q) const;

            TOSTRING(rw::math::Vector<T>)
            ARRAYOPERATOR(T)
        };

        /**
         * @brief Compares \b q1 and \b q2 for equality.
         *
         * \b q1 and \b q2 are considered equal if and only if they have equal
         * length and if q1(i) == q2(i) for all i.
         *
         * @relates Vector
         *
         * @param q1 [in]
         * @param q2 [in]
         * @return True if q1 equals q2, false otherwise.
         */
        template< class A > bool operator== (const Vector< A >& q1, const Vector< A >& q2);

        /**
         @brief Inequality operator

        The inverse of operator==().
        */
        template< class A > inline bool operator!= (const Vector< A >& q1, const Vector< A >& q2);

        /**
         * @brief Input streaming operator
         *
         * Parse input stream according to how operator<< streams out
         *
         * @relates Vector
         * @param in [in] Input stream
         * @param q [in] Target of q read in
         * @return reference to \b in
         */
        template< class A > std::istream& operator>> (std::istream& in, Vector< A >& q);

        /**
         @brief The dot product (inner product) of \b a and \b b.

        @relates Vector
        */
        template< class A > A dot (const Vector< A >& a, const Vector< A >& b);

        /**
         * @brief concatenates q1 onto q2 such that the returned q has
         * the configurations of q1 in [0;q1.size()[ and has q2 in
         * [q1.size();q1.size()+q2.size()[
         * @param q1 [in] the first Vector
         * @param q2 [in] the second Vector
         * @return the concatenation of q1 and q2
         */
        template< class A > rw::math::Vector< A > concat (const Vector< A >& q1, const Vector< A >& q2);
    }}    // namespace rw::math

    %template(RWVectord) rw::math::Vector<double>;
    %template(RWVectorf) rw::math::Vector<float>;

// ######################### VectorND
    namespace rw { namespace math {

        /** @addtogroup math */
        /*@{*/

        /**
         * @brief A N-Dimensional Vector
         *
         */
        template< size_t N, class T = double > class VectorND : public rw::common::Serializable
        {
        public:

            /**
             * @brief Creates a N-dimensional VectorND
             */
            VectorND ();

            /**
             * @brief Creates a 3D VectorND from Eigen type.
             *
             * @param v [in] an Eigen vector.
             */
            VectorND (const Eigen::Matrix< T,N,1>& v);

            /**
             @brief Accessor for the internal Eigen VectorND.
            */
            Eigen::Matrix< T,N,1>& e ();

            /**
             * @brief The dimension of the VectorND (i.e. 3).
             * This method is provided to help support generic algorithms using
             size() and operator[].
            */
            size_t size ();

            /**
             @brief Scalar division.
            */
            const VectorND< N, T > operator/ (T s) const;

            /**
             @brief Scalar multiplication.
            */
            const VectorND< N, T > operator* (T s) const;

            /**
             @brief Scalar multiplication.
            */
            friend const VectorND< N, T > operator* (T s, const VectorND< N, T >& v);

            /**
             @brief VectorND subtraction.
            */
            const VectorND< N, T > operator- (const VectorND< N, T >& b) const;

            /**
             @brief VectorND addition.
            */
            const VectorND< N, T > operator+ (const VectorND< N, T >& b) const;

            /**
             @brief Scalar multiplication.
            */
            VectorND< N, T >& operator*= (T s);

            /**
             @brief Scalar division.
            */
            VectorND< N, T >& operator/= (T s);

            /**
             @brief VectorND addition.
            */
            VectorND< N, T >& operator+= (const VectorND< N, T >& v);

            /**
             @brief VectorND subtraction.
            */
            VectorND< N, T >& operator-= (const VectorND< N, T >& v);

            /**
             @brief Unary minus.
            */
            const VectorND< N, T > operator- () const;

            /**
             * @brief Returns the Euclidean norm (2-norm) of the VectorND
             * @return the norm
             */
            T norm2 () const;

            /**
             * @brief Returns the Manhatten norm (1-norm) of the VectorND
             * @return the norm
             */
            T norm1 () const;

            /**
             * @brief Returns the infinte norm (\f$\inf\f$-norm) of the VectorND
             * @return the norm
             */
            T normInf () const;

            /**
             @brief Compare with \b b for equality.
            @param b [in] other vector.
            @return True if a equals b, false otherwise.
            */
            bool operator== (const VectorND< N, T >& b) const;

            //! @copydoc rw::common::Serializable::write
            void write (rw::common::OutputArchive& oarchive, const std::string& id) const;

            //! @copydoc rw::common::Serializable::read
            void read (rw::common::InputArchive& iarchive, const std::string& id);

            /**
             * @brief Get zero-initialized vector.
             * @return vector.
             */
            static VectorND< N, T > zero ();

            #define VectorND_type rw::math::VectorND<N,T>
            TOSTRING(VectorND_type)
            ARRAYOPERATOR(T)
        };

        /**
         * @brief Calculates the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
         * @param v1 [in] @f$ \mathbf{v1} @f$
         * @param v2 [in] @f$ \mathbf{v2} @f$
         *
         * @return the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
         *
         * The 3D VectorND cross product is defined as:
         * @f$
         * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
         *  v1_y * v2_z - v1_z * v2_y \\
         *  v1_z * v2_x - v1_x * v2_z \\
         *  v1_x * v2_y - v1_y * v2_x
         * \end{array}\right]
         * @f$
         *
         * @relates VectorND
         */
        template< size_t ND, class T >
        const VectorND< ND, T > cross (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2);

        /**
         * @brief Calculates the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
         * @param v1 [in] @f$ \mathbf{v1} @f$
         * @param v2 [in] @f$ \mathbf{v2} @f$
         * @param dst [out] the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
         *
         * The 3D VectorND cross product is defined as:
         * @f$
         * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
         *  v1_y * v2_z - v1_z * v2_y \\
         *  v1_z * v2_x - v1_x * v2_z \\
         *  v1_x * v2_y - v1_y * v2_x
         * \end{array}\right]
         * @f$
         *
         * @relates VectorND
         */
        template< size_t ND, class T >
        void cross (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2, VectorND< ND, T >& dst);

        /**
         * @brief Calculates the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
         * @param v1 [in] @f$ \mathbf{v1} @f$
         * @param v2 [in] @f$ \mathbf{v2} @f$
         *
         * @return the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
         *
         * @relates VectorND
         */
        template< size_t ND, class T > T dot (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2);

        /**
         * @brief Returns the normalized VectorND \f$\mathbf{n}=\frac{\mathbf{v}}{\|\mathbf{v}\|} \f$.
         * In case \f$ \|mathbf{v}\| = 0\f$ the zero VectorND is returned.
         * @param v [in] \f$ \mathbf{v} \f$ which should be normalized
         * @return the normalized VectorND \f$ \mathbf{n} \f$
         *
         * @relates VectorND
         */
        template< size_t ND, class T > const VectorND< ND, T > normalize (const VectorND< ND, T >& v);

        /**
         * @brief Calculates the angle from @f$ \mathbf{v1}@f$ to @f$ \mathbf{v2} @f$
         * around the axis defined by @f$ \mathbf{v1} \times \mathbf{v2} @f$ with n
         * determining the sign.
         * @param v1 [in] @f$ \mathbf{v1} @f$
         * @param v2 [in] @f$ \mathbf{v2} @f$
         * @param n [in] @f$ \mathbf{n} @f$
         *
         * @return the angle
         *
         * @relates VectorND
         */
        template< size_t ND, class T >
        double angle (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2,
                    const VectorND< ND, T >& n);

        /**
         * @brief Calculates the angle from @f$ \mathbf{v1}@f$ to @f$ \mathbf{v2} @f$
         * around the axis defined by @f$ \mathbf{v1} \times \mathbf{v2} @f$
         * @param v1 [in] @f$ \mathbf{v1} @f$
         * @param v2 [in] @f$ \mathbf{v2} @f$
         *
         * @return the angle
         *
         * @relates VectorND
         */
        template< size_t ND, class T >
        double angle (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2);

        /**
         * @brief Casts VectorND<N,T> to VectorND<Q>
         * @param v [in] VectorND with type T
         * @return VectorND with type Q
         *
         * @relates VectorND
         */
        template< size_t ND, class Q, class T >
        const VectorND< ND, Q > cast (const VectorND< ND, T >& v);
    }}

// ######################### Math

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

            /**
             * @brief this function converts a EAA object to a Quaternion
             *
             * @param roll [in] rotation around z
             * @param pitch [in] rotation around y
             * @param yaw [in] rotation around x
             *
             * @return a Quaternion object that represents the converted EAA
             */
            template< class A > static rw::math::Rotation3D< A > zyxToRotation3D (A roll, A pitch, A yaw);

            /**
             * \brief Constructs a 3x3 skew-symmetric matrix \f$ S\in so(3)\f$
             * \param s [in] the \f$ s_x \f$, \f$ s_y \f$ and \f$ s_z \f$ of the matrix
             * \return The 3x3 skew-symmetric matrix \f$S\f$
             *
             * \f$
             * S =
             * \left[
             * \begin{array}{ccc}
             *    0 & -s_z &  s_y\\
             *  s_z &    0 & -s_x\\
             * -s_y &  s_x &    0
             * \end{array}
             * \right]
             * \f$
             */
            template< class R > static inline Eigen::Matrix< R, 3, 3 > skew (const Vector3D< R >& s);

             /**
             * @brief clamp val to either min or max
             *
             * @param val [in] the value that is to be clamped
             * @param min [in] the minimum allowed value
             * @param max [in] the maximum allowed value
             * @return the clamped value of val
             */
            static inline double clamp(double val, double min, double max);


            /**
             * @brief Clamps values of \b q with \b min and \b max
             *
             * @param q [in] Values to clamp
             * @param min [min] The minimum value
             * @param max [min] The maximum value
             * @return The clamped values
             */
            static Q clampQ(const Q& q,const Q& min,const Q& max);

            /**
             * @brief Clamps values of \b q with \b bounds.first and \b bounds.second
             *
             * @param q [in] Values to clamp
             * @param bounds [min] The pair containing minimum and maximum values as first and second
             * element
             * @return The clamped values
             */
            static Q clampQ(const Q& q, const std::pair<Q, Q>& bounds);

            /**
             * @brief Clamps values of \b q with \b min and \b max
             *
             * @param q [in] Values to clamp
             * @param min [min] The minimum value
             * @param max [min] The maximum value
             * @return The clamped values
             */
            static rw::math::Vector3D<double> clamp(const rw::math::Vector3D<double>& q,
                                            const rw::math::Vector3D<double>& min,
                                            const rw::math::Vector3D<double>& max);

             /**
             * @brief A random double in the range [0, 1[ using a uniform distribution.
             *
             * @note Uses boost::random
             */
            static double ran();

            /**
             * @brief Seeds the random number generator.
             *
             * @note Uses boost::random
             */
            static void seed(unsigned seed);

            /**
             * @brief Seeds the random number generator with current time of day
             *
             * @note Uses boost::random
             */
            static void seed();

            /**
             * @brief A random double in the range [from, to[ using a uniform distribution.
             *
             * @note Uses boost::random
             */
            static double ran(double from, double to);

            /**
             * @brief A random integer in the range [from, to[ using a uniform distribution.
             *
             * @note Uses boost::random
             */
            static int ranI(int from, int to);

            /**
             * @brief Returns a random sample around \b mean with standard deviation \b sigma using the
             * normal distribution.
             *
             * @note Uses boost::random
             * @warning The number sequence generated can vary in different Boost versions (there is a
             * known change in Boost 1.56.0).
             *
             * @param mean [in] Means value
             * @param sigma [in] Standard deviation
             * @return Random sample
             */
            static double ranNormalDist(double mean, double sigma);

            /**
             * @brief Returns a random Q between with values in the range [from, to[ using a uniform
             * distribution.
             *
             * @note Uses boost::random
             *
             * @param from [in] The lower bound
             * @param to [in] The upper bound
             * @return Random Q
             */
            static Q ranQ(const Q& from, const Q& to);

            /**
             * @brief Returns a random Q between with values in the range [bounds.first, bounds.second[
             * using a uniform distribution.
             *
             * @note Uses boost::random
             *
             * @param bounds [in] The lower and upper bounds
             * @return Random Q
             */
            static Q ranQ(const std::pair<Q,Q>& bounds);

            /**
             * @brief Returns a random direction in \b dim dimensions using the standard normal
             * distribution.
             *
             * The length of the vector is given by \b length;
             *
             * @param dim [in] Number of dimensions
             * @param length [in] Length of return vector. Default is 1;
             * @return Random direction
             *
             * @warning Please see the warning for Math::ranNormalDist
             */
            static Q ranDir(size_t dim, double length = 1);
            
            /**
             * @brief Returns a weighted random direction in \b dim dimensions using the standard normal
             * distribution.
             *
             * The length of the vector is given by \b length;
             *
             * @param dim [in] Number of dimensions
             * @param weights [in] Weights to use
             * @param length [in] Length of return vector when weights are applied as weighted Euclidean
             * metric. Default is 1;
             * @return Random weigthed direction
             *
             * @warning Please see the warning for Math::ranNormalDist
             */
            static Q ranWeightedDir(size_t dim, const Q& weights, double length = 1);
            
            /**
             * @brief Returns a uniformly distributed random orientation.
             *
             * @return Random orientation represented as a Quaternion
             */
            template< class T > static rw::math::Quaternion< T > ranQuaternion ();

            /**
             * @brief Returns a uniformly distributed random orientation.
             *
             * @return Random orientation represented as a Rotation3D
             */
            template< class T > static rw::math::Rotation3D< T > ranRotation3D ();

             /**
             * @brief Returns random Transform3D based on ranDir (using the standard normal
             * distribution) and ranRotation3D (using a uniform distribution).
             *
             * @param translationLength [in]
             * @return Random Transform3D
             */
            template< class T >
            static rw::math::Transform3D< T > ranTransform3D (const double translationLength = 1);


            /**
             * @brief Rounds off to nearest integer
             *
             * With some compilers \b round can be found in math.h. This however does not
             * appear to be ansi C/C++ standard
             *
             * @param d [in] number to round
             * @return d rounded to nearest integer.
             */
            static double round(double d);

            /**
             * @brief The square of \b d
             * 
             * @param d [in] Number to square
             * @return d * d
             */
            template< class T > static inline T sqr (const T& d);

            /**
             * @brief The squares of the elements of \b q.
             */
            static Q sqr(const Q& q);

            /**
             * @brief The square roots of the elements of \b q.
             */
            static Q sqrt(const Q& q);

            /**
             * @brief Returns vector with the absolute values
             *
             * Given a vector \f$v=[v_1,v_2,\ldots,v_n]\f$ then Abs(v) is defined as
             * \f$Abs(v)=[abs(v_1),abs(v_i),\ldots,abs(v_n)] \f$
             *
             * @param v [in] the vector \f$v\f$
             * @return the vector \f$Abs(v)\f$
             */
            static Q abs(const Q& v);
            
            /**
             * @brief Returns the smallest element of v
             *
             * If the vector has zero length, the method returns 0
             *
             * @param v [in] the vector v
             * @return the smallest element
             */
            static double min(const Q& v);

            /**
             * @brief Returns the largest element of v
             *
             * If the vector has zero length, the method returns 0
             *
             * @param v [in] the vector v
             * @return the largest element
             */
            static double max(const Q& v);

            /**
             * @brief Returns vector with the elementwise smallest elements of \b a and \b b
             *
             * @param a [in] the vector \b a
             * @param b [in] the vector \b b
             * @return Q with smallest elements
             */
            template< class T > static T min (const T& a, const T& b);

            /**
             * @brief Returns vector with the elementwise largest elements of \b a and \b b
             *
             * @param a [in] the vector \b a
             * @param b [in] the vector \b b
             * @return Q with largest elements
             */
            template< class T > static T max (const T& a, const T& b);

            /**
             * @brief Returns vector with the absolute values
             *
             * Given a vector \f$v=[v_1,v_2,\ldots,v_n]\f$ then Abs(v) is defined as
             * \f$Abs(v)=[abs(v_1),abs(v_i),\ldots,abs(v_n)] \f$
             *
             * @param v [in] the vector \f$v\f$
             * @return the vector \f$Abs(v)\f$
             */
            template< class T > static Vector3D< T > abs (const Vector3D< T >& v);

            /**
             * @brief Returns the smallest element of v
             *
             * @param v [in] the vector v
             * @return the smallest element
             */
            template< class T > static T min (const Vector3D< T >& v);

            /**
             * @brief Returns the largest element of v
             *
             * @param v [in] the vector v
             * @return the largest element
             */
            template< class T > static T max (const Vector3D< T >& v);

            /**
             * @brief Returns vector with the elementwise smallest elements of \b a and \b b
             *
             * @param a [in] the vector \b a
             * @param b [in] the vector \b b
             * @return Vector with smallest elements
             */
            template< class T >
            static Vector3D< T > min (const Vector3D< T >& a, const Vector3D< T >& b);

            /**
             * @brief Returns vector with the elementwise largest elements of \b a and \b b
             *
             * @param a [in] the vector \b a
             * @param b [in] the vector \b b
             * @return Vector with largest elements
             */
            template< class T >
            static Vector3D< T > max (const Vector3D< T >& a, const Vector3D< T >& b);

            /**
             * @brief Returns the sign of s
             *
             * If s < 0 it return 0. If s >= 0 then 1 is returned.
             *
             * @param s [in] The value for which to return the sign
             * @return The sign
             */
            static double sign(double s);

            /**
             * @brief Returns the sign of each element
             *
             * For each element either -1 or 1 is returned depending on the sign. If \b q(i) equals 0
             * the method returns 1
             *
             * @param q [in] Vector for which to get the signs
             * @return Vector of sign values
             */
            static Q sign(const Q& q);

            /**
             @brief Exact implementation of ceil(log_2(n)) for n > 0.
            */
            static int ceilLog2(int n);
            
            /**
             * @brief Factorial
             * The method does not implement any safe guards for negative numbers of overflow of
             * numbers.
             */
            static long long factorial(long long n);

            /**
             * @brief convert a math vector type to an vector of doubles. The input should
             * have the index operator () in order to use this conversion
             * @param tmp [in] input
             * @param size [in] length of tmp
             * @return vector of doubles
             */
            template< class ARR > static std::vector< double > toStdVector (const ARR& tmp, int size);

            /**
             * @brief convert a math matrix type to an vector of doubles. The input should
             * have the index operator (x,y) in order to use this conversion
             * @param tmp [in] input matrix type
             * @param size1 [in] width of tmp
             * @param size2 [in] height of tmp
             * @return vector of doubles
             */
            template< class MAT >
            static std::vector< double > toStdVector (const MAT& tmp, int size1, int size2);

            /**
             * convert a vector of doubles to a vector math type. The math type should implement
             * the operator () in order to use this function.
             * @param data [in] the input
             * @param tmp [out] the output
             * @return reference to tmp
             */
            template< class T, class ARR >
            static ARR fromStdVector (const std::vector< T >& data, ARR& tmp);

            /**
             * convert a vector of doubles to a matrix math type. The math type should implement
             * the operator (i,j) in order to use this function.
             * @param data [in] the input
             * @param tmp [out] the output
             * @param size1 [in] the size of the first dimension of the matrix.
             * @param size2 [in] the size of the second dimension of the matrix.
             * @return reference to tmp
             */
            template< class T, class MAT >
            static MAT fromStdVectorToMat (const std::vector< T >& data, MAT& tmp, int size1, int size2);
            
            /**
             * @brief Implements an isNaN function
             *
             * Use to make sure code is independent of specific compile specific implementations
             */
            static bool isNaN(double d);

            /**
             * @brief Get a value for NaN.
             *
             * Use to make sure code is independent of specific compile specific implementations
             *
             * @return a double representation of NaN.
             */
            static double NaN ();
        };
    }} // end namespaces

    %template (quaternionToEAA) rw::math::Math::quaternionToEAA<double>;
    %template (quaternionToEAA) rw::math::Math::quaternionToEAA<float>;
    %template (eaaToQuaternion) rw::math::Math::eaaToQuaternion<double>;
    %template (eaaToQuaternion) rw::math::Math::eaaToQuaternion<float>;
    %template (zyxToRotation3D) rw::math::Math::zyxToRotation3D<double>;
    %template (zyxToRotation3D) rw::math::Math::zyxToRotation3D<float>;
    %template (skew) rw::math::Math::skew<double>;
    %template (skew) rw::math::Math::skew<float>;
    %template (ranQuaterniond) rw::math::Math::ranQuaternion<double>;
    %template (ranQuaternionf) rw::math::Math::ranQuaternion<float>;
    %template (ranRotation3Dd) rw::math::Math::ranRotation3D<double>;
    %template (ranRotation3Df) rw::math::Math::ranRotation3D<float>;
    %template (ranTransform3Dd) rw::math::Math::ranTransform3D<double>;
    %template (ranTransform3Df) rw::math::Math::ranTransform3D<float>;
    /*%template (min) rw::math::Math::min<double>;
    %template (min) rw::math::Math::min<float>;
    %template (max) rw::math::Math::max<double>;
    %template (max) rw::math::Math::max<float>;*/
    %template (abs) rw::math::Math::abs<double>;
    %template (abs) rw::math::Math::abs<float>;
    

// END