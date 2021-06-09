
%include <stl.i>
%include <exception.i>

%include <std_vector.i>

%include <std_string.i>

%include <rwlibs/swig/ext_i/os.i>

%typemap(out, fragment="SWIG_From_std_string") std::string&& {
  $result = SWIG_From_std_string(*$1);
}

%typemap(in, fragment="SWIG_AsVal_std_string") std::string&& (std::string temp) {
  int res = SWIG_AsVal_std_string($input, &temp);
  $1 = &temp;
}
#if defined(SWIGPYTHON)
%typemap(out, fragment="SWIG_From_std_string") std::string& {
  $result = SWIG_From_std_string(*$1);
}

%typemap(in, fragment="SWIG_AsVal_std_string") std::string& (std::string temp) {
  int res = SWIG_AsVal_std_string($input, &temp);
  $1 = &temp;
}
#endif

%{
  #include <complex>
%}
namespace std {

    template<typename _Tp>
    struct complex {
        constexpr complex(const _Tp& __r = _Tp(), const _Tp& __i = _Tp());
        constexpr complex(const complex&) = default;
    };

    class exception {
      public:
        exception() noexcept;
        exception(const exception&) noexcept;
#if !defined(SWIGPYTHON)
        exception& operator=(const exception&) noexcept;
#endif
        virtual ~exception();
        virtual const char* what() const noexcept;
  };

}

%template(complexd) std::complex<double>;
%template(complexf) std::complex<float>;

#if !defined(SWIGPYTHON)
    //TODO(kalor) find out why this doesn't work in python
    %template(VectorComplexDouble) std::vector<std::complex<double>>;
#endif

#if (defined(SWIGLUA) || defined(SWIGPYTHON))
	%extend std::vector<std::string> { char *__str__() { return printCString(*$self); } }
#endif

#if ! defined(WIN32)
  typedef long unsigned int size_t;
  typedef unsigned char uint8_t;
  typedef unsigned short uint16_t;
  typedef unsigned int uint32_t;
#endif 

	%template(VectorString) std::vector<std::string>;
  %template(VectorChar) std::vector<char>;
	%template(VectorDouble) std::vector<double>;
  %template(VectorFloat) std::vector<float>;
	%template(VectorInt) std::vector<int>;
  %template(VectorLong) std::vector<long>;
  %template(VectorUInt) std::vector<unsigned int>;
  
#if !defined(WIN32)
  %template(VectorULong) std::vector<unsigned long>;
  %template(VecotrVecotrULong) std::vector<std::vector<unsigned long>>;
#endif 
  %template(Vectorbool) std::vector<bool>;
  %template(pairDoubleDouble) std::pair<double,double>;
  %template(VectorPairDoubleDouble) std::vector<std::pair<double,double>>;
  %template(pairUIntUInt) std::pair<unsigned int, unsigned int>;
  %template(pairBoolDouble) std::pair<bool,double>;
  %template(pairBoolULong) std::pair<bool,unsigned long>;
  %template(pairBoolLong) std::pair<bool,long>;
  %template(pairBoolInt) std::pair<bool,int>;
  %template(pairBoolUInt) std::pair<bool,unsigned int>;
  %template(pairBoolVectorDouble) std::pair<bool,std::vector<double>>;
  %template(pairBoolVectorInt) std::pair<bool,std::vector<int>>;
  %template(pairBoolVectorLong) std::pair<bool,std::vector<long>>;
  %template(pairBoolVectorUInt) std::pair<bool,std::vector<unsigned int>>;
  %template(pairBoolVectorULong) std::pair<bool,std::vector<unsigned long>>;
