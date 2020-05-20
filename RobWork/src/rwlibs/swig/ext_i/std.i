
namespace std {

    template<typename _Tp>
    struct complex {
        constexpr complex(const _Tp& __r = _Tp(), const _Tp& __i = _Tp());
        constexpr complex(const complex&) = default;
    };

}

%template(complexd) std::complex<double>;
%template(complexf) std::complex<float>;

#if !defined(SWIGPYTHON)
    //TODO(kalor) find out why this doesn't work i npython
    %template(VectorComplexDouble) std::vector<std::complex<double>>;
#endif
