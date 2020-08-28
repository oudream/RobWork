
%import(module=sdurw_core) <rwlibs/swig/ext_i/std.i>

namespace Eigen{
    template<typename _Scalar, int _Rows, int _Cols>
    class Matrix
    {
      public:
        
        Matrix(int dimx, int dimy);
        
        #if !defined(SWIGJAVA)
            double& operator()(size_t row, size_t column);
            const double& operator()(size_t row, size_t column) const;
        #endif

        const Matrix operator+(const Matrix& wrench) const;    
        const Matrix operator-(const Matrix& wrench) const;
        //const Matrix operator*(const Matrix& wrench) const;
        
        %extend {
            
            #if !defined(SWIGJAVA)
                _Scalar& elem(int x, int y){
                    return (*$self)(x,y);
                }
            #endif
            
            /* These accesors are neccesary because Python does not allow
            lvalues consisting of access operators calls (cannot assign to a function call).
            Moreover, it's not possible to dereference a pointer obtained from function returning a reference. */
            _Scalar get(int x, int y) {
                return (*$self)(x, y);
            }
            
            void set(int x, int y, _Scalar value) {
                (*$self)(x, y) = value;
            }
        }
        MATRIXOPERATOR(_Scalar);
    };

    template<typename T>
    class Quaternion{

    };
}


%template(EigenMatrixXf) Eigen::Matrix<float,-1,-1>;
%template(EigenMatrixXd) Eigen::Matrix<double,-1,-1>;
%template(EigenMatrix2f) Eigen::Matrix<float,2,2>;
%template(EigenMatrix2d) Eigen::Matrix<double,2,2>;
%template(EigenMatrix3f) Eigen::Matrix<float,3,3>;
%template(EigenMatrix3d) Eigen::Matrix<double,3,3>;
%template(EigenMatrix4f) Eigen::Matrix<float,4,4>;
%template(EigenMatrix4d) Eigen::Matrix<double,4,4>;


%template(EigenVectorXf) Eigen::Matrix<float,-1,1>;
%template(EigenVectorXd) Eigen::Matrix<double,-1,1>;
%template(EigenVector2f) Eigen::Matrix<float,2,1>;
%template(EigenVector2d) Eigen::Matrix<double,2,1>;
%template(EigenVector3f) Eigen::Matrix<float,3,1>;
%template(EigenVector3d) Eigen::Matrix<double,3,1>;
%template(EigenVector6f) Eigen::Matrix<float,6,1>;
%template(EigenVector6d) Eigen::Matrix<double,6,1>;
%template(EigenVector7f) Eigen::Matrix<float,7,1>;
%template(EigenVector7d) Eigen::Matrix<double,7,1>;

%template(EigenRowVector3f) Eigen::Matrix<float,1,3>;
%template(EigenRowVector3d) Eigen::Matrix<double,1,3>;
%template(EigenVector3id) Eigen::Matrix<std::complex<double>,3,1>;
%template(EigenRowVector3id) Eigen::Matrix<std::complex<double>,1,3>;
%template(EigenMatrix3id) Eigen::Matrix<std::complex<double>,3,3>;

%template(EigenQuaterniond) Eigen::Quaternion<double>;
%template(EigenQuaternionf) Eigen::Quaternion<float>; 

%template(VectorEigenRowVector3f) std::vector<Eigen::Matrix<float,1,3>>;
%template(VectorEigenRowVector3d) std::vector<Eigen::Matrix<double,1,3>>;
%template(VectorEigenVector3f) std::vector<Eigen::Matrix<float,3,1>>;
%template(VectorEigenVector3d) std::vector<Eigen::Matrix<double,3,1>>;
%template(VectorEigenMatrix3f) std::vector<Eigen::Matrix<float,3,3>>;
%template(VectorEigenMatrix3d) std::vector<Eigen::Matrix<double,3,3>>;

%template(VectorEigenVector3id) std::vector<Eigen::Matrix<std::complex<double>,3,1>>;
%template(VectorEigenMatrix3id) std::vector<Eigen::Matrix<std::complex<double>,3,3>>;
