
/********************************************
 * MATH
 ********************************************/
//! @copydoc rw::math::Matrix
class Matrix {
public:
	
	Matrix(int dimx, int dimy);
	
#if !defined(SWIGJAVA)
    double& operator()(size_t row, size_t column);
    const double& operator()(size_t row, size_t column) const;
#endif

    const Matrix operator+(const Matrix& wrench) const;    
    const Matrix operator-(const Matrix& wrench) const;
    const Matrix operator*(const Matrix& wrench) const;
	
	%extend {
		Matrix pseudoinverse() {
			 return rw::math::LinearAlgebra::pseudoInverse( (*$self) );
		}
		
#if !defined(SWIGJAVA)
		double& elem(int x, int y){
			return (*$self)(x,y);
		}
#endif
		
		/* These accesors are neccesary because Python does not allow
		lvalues consisting of access operators calls (cannot assign to a function call).
		Moreover, it's not possible to dereference a pointer obtained from function returning a reference. */
		double get(int x, int y) {
			return (*$self)(x, y);
		}
		
		void set(int x, int y, double value) {
			(*$self)(x, y) = value;
		}

#if (defined(SWIGPYTHON))
		/* This typemap makes it possible to access Matrix class elements using following syntax:
		
		myMatrix[1, 1] = value
		print myMatrix[1, 1]
		
		-- using __getitem__ and __setitem__ methods. */
		%typemap(in) int[2](int temp[2]) {
			if (PyTuple_Check($input)) {
				if (!PyArg_ParseTuple($input, "ii", temp, temp+1)) {
					PyErr_SetString(PyExc_TypeError, "tuple must have 2 elements");
					return NULL;
				}
				$1 = &temp[0];
			} else {
				PyErr_SetString(PyExc_TypeError, "expected a tuple.");
				return NULL;
			}
		}
		
        double __getitem__(int i[2])const {return (*$self)(i[0], i[1]); }
        void __setitem__(int i[2], double d){ (*$self)(i[0], i[1]) = d; }
#endif
	}
	
};

namespace rw { namespace math {
/**
 * @copydoc rw::math::Q 
 */
class Q
{
public:
    // first we define functions that are native to Q
	Q();
	//%feature("autodoc","1");
	Q(int n, double a0);
    Q(int n, double a0, double a1);
    Q(int n, double a0, double a1, double a2);
    Q(int n, double a0, double a1, double a2, double a3);
    Q(int n, double a0, double a1, double a2, double a3, double a4);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9);

    Q(const std::vector<double>& r);
#if defined(SWIGJAVA)
%apply double[] {double *};
#endif
    Q(int n, const double* values);

    int size() const;

#if !defined(SWIGJAVA)
    %rename(elem) operator[];
    double& operator[](unsigned int i) ;
#endif

    const Q operator-() const;
    Q operator-(const Q& b) const;
    Q operator+(const Q& b) const;
    Q operator*(double s) const;
    Q operator/(double s) const;
    double norm2();
    double norm1();
    double normInf();

    %extend {
		
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::Q>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::Q>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };

};
} }

%template (QVector) std::vector<rw::math::Q>;
%template(QPair) std::pair<rw::math::Q, rw::math::Q>;

namespace rw {
namespace math {

template<class T> class Vector2D
{
public:
    Vector2D();
    %feature("autodoc","1");
    Vector2D(T x, T y);
    size_t size() const;

    T norm2();
    T norm1();
    T normInf();

    //double& operator[](unsigned int i) ;
    %rename(elem) operator[];

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::Vector2D<T> >(*$self); }
        T __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,T d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::Vector2D<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,T d){ (*$self)[i] = d; }
#endif
    };

};

}}
%template (Vector2f) rw::math::Vector2D<float>;
%template (Vector2) rw::math::Vector2D<double>;
%template (Vector2Vector) std::vector<rw::math::Vector2D<double> >;

/**
 * @copydoc rw::math::Vector3D
 */
namespace rw { namespace math {
template<class T> class Vector3D
{
public:
    Vector3D();
    %feature("autodoc","1");
    Vector3D(T x, T y, T z);
    size_t size() const;
    Vector3D operator*(T scale) const;
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    bool operator==(const Vector3D& q);

    T norm2();
    T norm1();
    T normInf(); 

    //double& operator[](unsigned int i) ;
    //%rename(elem) operator[];

    static Vector3D<T> zero();
    static Vector3D<T> x();
    static Vector3D<T> y();
    //! @copydoc rw::math::Q::z
    static Vector3D<T> z();
    
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::Vector3D<T> >(*$self); }
        T __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,T d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::Vector3D<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,T d){ (*$self)[i] = d; }
#endif
#if defined(SWIGPYTHON)
        Wrench6D<T> multiply(const Wrench6D<T>& bV) { return (*$self)*bV; }
#endif
    };
};
}}
%template (Vector3d) rw::math::Vector3D<double>;
%template (Vector3f) rw::math::Vector3D<float>;
%template (Vector3dVector) std::vector< rw::math::Vector3D<double> >;
%template (Vector3fVector) std::vector< rw::math::Vector3D<float> >;

namespace rw { namespace math {
/**
 * @copydoc rw::math::Rotation3D
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
     */
    Rotation3D(T v0,T v1,T v2,
    			T v3,T v4,T v5,
    			T v6,T v7,T v8);

    /**
     * @brief Constructs an initialized 3x3 rotation matrix
     */
    Rotation3D(
        const rw::math::Vector3D<T>& i,
        const rw::math::Vector3D<T>& j,
        const rw::math::Vector3D<T>& k);
    			
    explicit Rotation3D(const Rotation3D<T>& R);

    /**
     * @brief Constructs a 3x3 rotation matrix set to identity
     *
     * @return a 3x3 identity rotation matrix
     */
    static const Rotation3D<T>& identity();
    
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
    bool operator==(const Rotation3D<T> &rhs) const;

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
    bool equal(const Rotation3D<T>& rot, T precision) const;

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
     * @brief Calculates this rotation multiplied by other rotation.
     *
     * @param other [in] rotation to multiply with
     *
     * @return the result of multiplication.
     */
    Rotation3D<T> operator*(const Rotation3D<T>& other) const;

    /**
     * @brief Calculates this rotation multiplied by a vector.
     *
     * @param vec [in] vector to multiply with.
     *
     * @return the result of multiplication.
     */
    rw::math::Vector3D<T> operator*(const rw::math::Vector3D<T>& vec) const;

    /**
     * @brief Creates a skew symmetric matrix from a Vector3D. Also
     * known as the cross product matrix of v.
     *
     * @relates Rotation3D
     *
     * @param v [in] vector to create Skew matrix from
     */
    static Rotation3D<T> skew(const rw::math::Vector3D<T>& v);

    /**
     * @brief Calculate the inverse.
     *
     * @note This function changes the object that it is invoked on, but this is about x5 faster than rot = inverse( rot )
     *
     * @return the inverse rotation.
     */
    Rotation3D<T>& inverse();

    %extend {
    	const rw::math::EAA<T> operator*(const rw::math::EAA<T>& bTKc){
    		return *((rw::math::Rotation3D<T>*)$self) * bTKc;
    	}

#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Rotation3D<T> >(*$self); }
        T __getitem__(int x,int y)const {return (*$self)(x,y); }
        void __setitem__(int x,int y,T d){ (*$self)(x,y) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Rotation3D<T> >(*$self); }
        T get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        void set(std::size_t row, std::size_t column, T d){ (*$self)(row, column) = d; }
#endif
#if defined(SWIGPYTHON)
        Wrench6D<T> multiply(const Wrench6D<T>& bV) { return (*$self)*bV; }
#endif
    };

#if !defined(SWIGLUA)    
    %extend {
    	rw::math::InertiaMatrix<T> multiply(const rw::math::InertiaMatrix<T>& bRc) { return (*$self)*bRc; }
    };
#endif
};
}}

%template (Rotation3d) rw::math::Rotation3D<double>;
%template (Rotation3f) rw::math::Rotation3D<float>;
%template (Rotation3Vector) std::vector< rw::math::Rotation3D<double> >;

namespace rw { namespace math {
/**
 * @brief Calculates the inverse of a rotation matrix
 *
 * @relates Rotation3D
 *
 * @param aRb [in] the rotation matrix.
 *
 * @return the matrix inverse.
 */
template <class T>
const Rotation3D<T> inverse(const Rotation3D<T>& aRb);
}}

%template (inverse) rw::math::inverse<double>;
%template (inverse) rw::math::inverse<float>;

namespace rw { namespace math {
/**
 * @brief An abstract base class for Rotation3D parameterisations
 *
 * Classes that represents a parametrisation of a 3D rotation may inherit
 * from this class
 */
template<class T>
class Rotation3DVector {
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~Rotation3DVector();

    /**
     * @brief Returns the corresponding 3x3 Rotation matrix
     * @return The rotation matrix
     */
    virtual const Rotation3D<T> toRotation3D() const = 0;
};
}}

%template (Rotation3DVectord) rw::math::Rotation3DVector<double>;
%template (Rotation3DVectorf) rw::math::Rotation3DVector<float>;

namespace rw { namespace math {
//! @copydoc rw::math::EAA
template<class T> class EAA: public Rotation3DVector<T>
{
public:
    // Lua methods:
    EAA();
     %feature("autodoc","1");
    EAA(const EAA<T>& eaa);
     %feature("autodoc","1");
    EAA(const Rotation3D<T>& rot);
     %feature("autodoc","1");
    EAA(const rw::math::Vector3D<T>& axis, T angle);
     %feature("autodoc","1");
    EAA(T thetakx, T thetaky, T thetakz);
     %feature("autodoc","1");
    EAA(const rw::math::Vector3D<T>& v1, const rw::math::Vector3D<T>& v2);

    double angle() const;
    rw::math::Vector3D<T> axis() const;

    Rotation3D<T> toRotation3D() const;

	

    //bool operator==(const EAA &rhs) const;
    // std::string __tostring() const;
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<EAA<T> >(*$self); }
        T __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,T d){ (*$self)[i] = d; }
    	T& x() { return (*$self)[0]; }
    	T& y() { return (*$self)[1]; }
    	T& z() { return (*$self)[2]; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<EAA<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,T d){ (*$self)[i] = d; }
    	T x() const { return (*$self)[0]; }
    	T y() const { return (*$self)[1]; }
    	T z() const { return (*$self)[2]; }
#endif
    };
};
}}

%template (EAAd) rw::math::EAA<double>;
%template (EAAf) rw::math::EAA<float>;
%template (EAA3Vector) std::vector< rw::math::EAA<double> >;

namespace rw { namespace math {
//! @copydoc rw::math::RPY
template<class T> class RPY: public Rotation3DVector<T>
{
public:
    // Lua methods:
    RPY();
    RPY(const RPY& eaa);
    RPY(const rw::math::Rotation3D<T>& rot);
    RPY(T roll, T pitch, T yaw);
    rw::math::Rotation3D<T> toRotation3D() const;
    
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<RPY<T> >(*$self); }
        T __getitem__(int i)const {
        	if(i<0 || i>2) throw("Index is outside bounds. Must be in range [0;2]");
        	return (*$self)[i]; 
        }
        void __setitem__(int i,T d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<RPY<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,T d){ (*$self)[i] = d; }
#endif
    };
};
}}

%template (RPYd) rw::math::RPY<double>;
%template (RPYf) rw::math::RPY<float>;
%template (RPYdVector) std::vector< rw::math::RPY<double> >;

//! @copydoc rw::math::Quaternion
namespace rw { namespace math {
template<class T> class Quaternion: public Rotation3DVector<T>
{
public:
    // Lua methods:
    Quaternion();
    Quaternion(T qx, T qy, T qz, T qw);
    Quaternion(const Quaternion& eaa);
    Quaternion(const rw::math::Rotation3D<T>& rot);
    Quaternion operator*(T s);

    void normalize();

    rw::math::Rotation3D<T> toRotation3D() const;
    Quaternion<T> slerp(const Quaternion<T>& v, T t) const;

    T getQx() const;
    T getQy() const;
    T getQz() const;
    T getQw() const;

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::Quaternion<T> >(*$self); }
        T __getitem__(int i)const {return (*$self)(i); }
        void __setitem__(int i,T d){ (*$self)(i) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::Quaternion<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)(i); }
        void set(std::size_t i,T d){ (*$self)(i) = d; }
#endif
    };
};
}}

%template (Quaterniond) rw::math::Quaternion<double>;
%template (Quaternionf) rw::math::Quaternion<float>;
%template (QuaterniondVector) std::vector< rw::math::Quaternion<double> >;

namespace rw { namespace math {
/**
 * @brief A 4x4 homogeneous transform matrix.
 */
template<class T> class Transform3D {
public:
    /**
     * @brief Default Constructor.
     *
     * Initializes with 0 translation and Identity matrix as rotation
     */
	Transform3D();
    Transform3D(const Transform3D<T>& t3d);

    /**
     * @brief Constructs a homogeneous transform
     *
     * @param d [in] A 3x1 translation vector
     * @param R [in] A 3x3 rotation matrix
     */
    Transform3D(const rw::math::Vector3D<T>& position,const rw::math::Rotation3D<T>& rotation);

    /**
     * @brief Constructs a homogeneous transform
     *
     * Calling this constructor is equivalent to the transform
     * Transform3D(d, r.toRotation3D()).
     *
     * @param d [in] A 3x1 translation vector
     * @param r [in] A 3x1 rotation vector
     */
    Transform3D(const Vector3D<T>& d, const Rotation3DVector<T>& r);

    Transform3D operator*(const Transform3D<T>& other) const;
    rw::math::Vector3D<T> operator*(const rw::math::Vector3D<T>& other) const;

    /**
     * @brief Constructs a homogeneous transform using the original
     * Denavit-Hartenberg notation
     *
     * @param alpha [in]
     * @param a [in]
     * @param d [in]
     * @param theta [in]
     *
     * @return transformation matrix.
     */
    static Transform3D<T> DH(T alpha, T a, T d, T theta);

    /**
     * @brief Constructs a homogeneous transform using the Craig (modified)
     * Denavit-Hartenberg notation
     *
     * @param alpha [in]
     * @param a [in]
     * @param d [in]
     * @param theta [in]
     *
     * @return the transformation matrix.
     *
     * @note The Craig (modified) Denavit-Hartenberg notation differs from
     * the original Denavit-Hartenberg notation.
     */
    static Transform3D<T> craigDH(T alpha, T a, T d, T theta);
    
    /**
     * @brief Constructs the identity transform
     *
     * @return the identity transform.
     */
    static const Transform3D& identity();

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
    static Transform3D<T> makeLookAt(const Vector3D<T>& eye, const Vector3D<T>& center, const Vector3D<T>& up);

    /**
     * @brief Gets the position part P from T
     *
     * @return the translation.
     */
    rw::math::Vector3D<T>& P();

    /**
     * @brief Gets the rotation part R from T.
     *
     * @return the rotation.
     */
    rw::math::Rotation3D<T>& R();

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Transform3D<T> >(*$self); }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Transform3D<T> >(*$self); }
#endif
#if defined(SWIGPYTHON)
        Wrench6D<T> multiply(const Wrench6D<T>& bV) { return (*$self)*bV; }
#endif
    };
};
}}

%template (Transform3d) rw::math::Transform3D<double>;
%template (Transform3f) rw::math::Transform3D<float>;
%template (Transform3dVector) std::vector<rw::math::Transform3D<double> >;

namespace rw { namespace math {
/**
 * @brief Calculates the inverse of the transformation matrix.
 *
 * @param aTb [in] the transform matrix.
 *
 * @return the inverse matrix.
 */
template <class T>
const Transform3D<T> inverse(const Transform3D<T>& aTb);
}}

%template (inverse) rw::math::inverse<double>;
%template (inverse) rw::math::inverse<float>;

namespace rw { namespace math {


template<class T> class Pose6D {
public:
	Pose6D(const Pose6D<T>& p6d);
    Pose6D(const Vector3D<T>& position,const EAA<T>& rotation);
    Pose6D(const Transform3D<T>& t3d);

    rw::math::Transform3D<T> toTransform3D();
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Pose6D<T> >(*$self); }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Pose6D<T> >(*$self); }
#endif
    };
};



template<class T> class VelocityScrew6D
{
public:
	VelocityScrew6D();
	VelocityScrew6D(const VelocityScrew6D<T>& p6d);
    VelocityScrew6D(const rw::math::Vector3D<T>& position,const EAA<T>& rotation);
    VelocityScrew6D(const Transform3D<T>& t3d);

    // lua functions
    VelocityScrew6D<T> operator*(T scale) const;
    
    VelocityScrew6D<T> operator+(const VelocityScrew6D<T>& other) const;
    VelocityScrew6D<T> operator-(const VelocityScrew6D<T>& other) const;
    //bool operator==(const VelocityScrew6D<T>& q);

    double norm2();
    double norm1();
    double normInf();

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<VelocityScrew6D<T> >(*$self); }
        T __getitem__(std::size_t i)const {return (*$self)[i]; }
        void __setitem__(std::size_t i,T d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<VelocityScrew6D<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i, T d){ (*$self)[i] = d; }
#endif
    };

    //Transform3D toTransform3D();
    // std::string __tostring() const;
};


template<class T> class Wrench6D
{
public:		
    Wrench6D(T fx, T fy, T fz, T tx, T ty, T tz);

    // TODO: add constructor on vector

    Wrench6D();

    Wrench6D(const rw::math::Vector3D<T>& force, const rw::math::Vector3D<T>& torque);

    const rw::math::Vector3D<T> force() const;
    const rw::math::Vector3D<T> torque() const;

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Wrench6D<T> >(*$self); }
        T __getitem__(std::size_t i)const {return (*$self)[i]; }
        void __setitem__(std::size_t i,T d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Wrench6D<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i, T d){ (*$self)[i] = d; }
#endif
    };

    const Wrench6D<T> operator*(T s) const;

#if !defined(SWIGPYTHON)
    friend const Wrench6D<T> operator*(const Transform3D<T>& aTb,
                                              const Wrench6D<T>& bV);
    
    friend const Wrench6D operator*(const rw::math::Vector3D<T>& aPb, const Wrench6D<T>& bV);


    friend const Wrench6D operator*(const rw::math::Rotation3D<T>& aRb, const Wrench6D<T>& bV);
#endif

    const Wrench6D<T> operator+(const Wrench6D<T>& wrench) const;    
    const Wrench6D<T> operator-(const Wrench6D<T>& wrench) const;

    T norm1() const;
    
    T norm2() const ;
    T normInf() const ;
};

template<class T> class InertiaMatrix{
public:
    InertiaMatrix(
        T r11, T r12, T r13,
        T r21, T r22, T r23,
        T r31, T r32, T r33);

    InertiaMatrix(
        const rw::math::Vector3D<T>& i,
        const rw::math::Vector3D<T>& j,
        const rw::math::Vector3D<T>& k);

    InertiaMatrix(
        T i = 0.0,
        T j = 0.0,
        T k = 0.0);

#if !defined(SWIGJAVA)
    T& operator()(size_t row, size_t column);
    const T& operator()(size_t row, size_t column) const;
#endif

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::InertiaMatrix<T> >(*$self); }
        T __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row,column); }
        void __setitem__(std::size_t row, std::size_t column, T d){ (*$self)(row,column) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::InertiaMatrix<T> >(*$self); }
        T get(std::size_t row, std::size_t column) const { return (*$self)(row,column); }
        void set(std::size_t row, std::size_t column, T val) { (*$self)(row,column) = val; }
#endif
    };

    //const Base& m() const;
    //Base& m();

#if defined(SWIGLUA)
    friend InertiaMatrix<T> operator*(const rw::math::Rotation3D<T>& aRb, const InertiaMatrix<T>& bRc);
    friend InertiaMatrix<T> operator*(const InertiaMatrix<T>& aRb, const Rotation3D<T>& bRc);
    friend InertiaMatrix<T> operator+(const InertiaMatrix<T>& I1, const InertiaMatrix<T>& I2);
    friend Vector3D<T> operator*(const InertiaMatrix<T>& aRb, const Vector3D<T>& bVc);
    //friend InertiaMatrix<T> inverse(const InertiaMatrix<T>& aRb);
    //friend std::ostream& operator<<(std::ostream &os, const InertiaMatrix<T>& r);
#endif

#if !defined(SWIGLUA)    
    %extend {
    	InertiaMatrix<T> multiply(const Rotation3D<T>& bRc) { return (*$self)*bRc; }
    	InertiaMatrix<T> add(const InertiaMatrix<T>& I2) { return (*$self)+I2; }
    	Vector3D<T> multiply(const Vector3D<T>& bVc) { return (*$self)*bVc; }
    };
#endif

    static InertiaMatrix<T> makeSolidSphereInertia(T mass, T radi);
    static InertiaMatrix<T> makeHollowSphereInertia(T mass, T radi);
    static InertiaMatrix<T> makeCuboidInertia(T mass, T x, T y, T z);
};

}}

/**
 * @brief projection matrix
 */
class ProjectionMatrix {
public:
    //! @brief constructor
    ProjectionMatrix();

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
};

%template (Pose6d) rw::math::Pose6D<double>;
%template (Pose6f) rw::math::Pose6D<float>;
%template (Pose6dVector) std::vector<rw::math::Pose6D<double> >;

%template (Wrench6d) rw::math::Wrench6D<double>;
%template (Wrench6f) rw::math::Wrench6D<float>;
%template (Wrench6dVector) std::vector<rw::math::Wrench6D<double> >;

%template (Screw6d) rw::math::VelocityScrew6D<double>;
%template (Screw6f) rw::math::VelocityScrew6D<float>;
%template (Screw6dVector) std::vector<rw::math::VelocityScrew6D<double> >;

%template (InertiaMatrixd) rw::math::InertiaMatrix<double>;
%template (InertiaMatrixf) rw::math::InertiaMatrix<float>;
%template (InertiaMatrixdVector) std::vector<rw::math::InertiaMatrix<double> >;


namespace rw { namespace math {
class Jacobian
{
public:
    Jacobian(int m, int n);

    int size1() const ;
    int size2() const ;

#if (defined(SWIGLUA) || defined(SWIGPYTHON))
    double& elem(int i, int j);

    %extend {
        char *__str__() { return printCString<rw::math::Jacobian>(*$self); }
        double __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row, column); }
        void __setitem__(std::size_t row, std::size_t column,double d){ (*$self)(row, column) = d; }
    };
#elif defined(SWIGJAVA)
    %extend {
        std::string toString() const { return toString<rw::math::Jacobian>(*$self); }
        double elem(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        double get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        void set(std::size_t row, std::size_t column, double d){ (*$self)(row, column) = d; }
    };
#endif
};
} }




%nodefaultctor Metric;
template <class T>
class Metric
{
public:
    double distance(const T& q) const;

    double distance(const T& a, const T& b) const;

    int size() const;

};

%template (MetricQ) Metric<rw::math::Q>;
%template (MetricQPtr) rw::common::Ptr<Metric<rw::math::Q> >;
%template (MetricQCPtr) rw::common::Ptr<const Metric<rw::math::Q> >;
%template (MetricSE3) Metric<rw::math::Transform3D<double> >;
%template (MetricSE3Ptr) rw::common::Ptr<Metric<rw::math::Transform3D<double> > >;
OWNEDPTR(Metric<rw::math::Q> );
OWNEDPTR(Metric<rw::math::Transform3D<double> > );

class MetricFactory
{
  public:
  private:
    MetricFactory();
    MetricFactory(const MetricFactory&);
    MetricFactory& operator=(const MetricFactory&);

};

%extend MetricFactory {
    static rw::common::Ptr<rw::math::Metric<rw::math::Q>> makeEuclideanQ(){
        return rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    }
}
