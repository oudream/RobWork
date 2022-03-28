
%{
    #include <vector>
    #include <complex>
%}

%include <std_string.i>


#define EVAL(...)  EVAL1(EVAL1(EVAL1(__VA_ARGS__)))
#define EVAL1(...) __VA_ARGS__

#ifndef SWIG_CORE_DEFINE
#define SWIG_CORE_DEFINE(...) __VA_ARGS__
#endif

#ifndef SWIG_TO_STRING
#define SWIG_TO_STRING(...) #__VA_ARGS__
#endif

#if !defined(SWIGPYTHON)
%include <std_vector.i>

%define %std_vector_explicit(name,class,fullclass,fragment)
    %template(name) fullclass;
%enddef

#else 


%fragment("toFromPy","header")
%{
    #include <rw/core/macros.hpp>
    template <class T>
    struct Py{
        static PyObject* toPy(T elem,swig_type_info* type){
            type = SWIG_TypeQuery((std::string(type->str)+"*").c_str());
            if (type == NULL){
                RW_THROW("Cannot convert object to python type when type info is null");
            }
            return SWIG_NewPointerObj(SWIG_as_voidptr(new T (elem)),type,0);
        }

        static void fromPy(PyObject* elem,swig_type_info* type, T& res){
            void *argp;
            type = SWIG_TypeQuery((std::string(type->str)+"*").c_str());
            int test = SWIG_ConvertPtr(elem, &argp, type,SWIG_POINTER_NO_NULL | 0);
            if (SWIG_IsOK(test)) {
                T* tmp_var = reinterpret_cast< T* > (argp);
                res = *tmp_var;
            }else{
                RW_THROW("Could not convert to type: " <<  type->str  << ". Python reports type as: " << elem->ob_type->tp_name );
            }
        }
    };
%}



%fragment("intToFromPy","header",fragment="toFromPy")
%{
    template <>
    struct Py<int>{
        static PyObject* toPy(int elem,swig_type_info* type){
            return PyLong_FromLong(elem);
        }
        static void fromPy(PyObject* elem,swig_type_info* type, int& res){
            res = PyLong_AsLong(elem);
        }
    };

    template<>
    struct Py<long>
    {
        static PyObject* toPy(long elem,swig_type_info* type){
            return PyLong_FromLong(elem);
        }
        static void fromPy(PyObject* elem,swig_type_info* type, long& res){
            res = PyLong_AsLong(elem);
        }
    };
    
        template<>
    struct Py<unsigned long>
    {
        static void fromPy(PyObject* elem,swig_type_info* type, unsigned long& res){
            res = PyLong_AsLong(elem);
        }
        static PyObject* toPy(unsigned long elem,swig_type_info* type){
            return PyLong_FromUnsignedLong(elem);
        }
    };
    template<>
    struct Py<long long>
    {
        static void fromPy(PyObject* elem,swig_type_info* type, long long & res){
            res = PyLong_AsLong(elem);
        }
        static PyObject* toPy(long long elem,swig_type_info* type){
            return PyLong_FromLongLong(elem);
        }
    };

    template <>
    struct Py<unsigned long long >
    {
        static PyObject* toPy(unsigned long long elem,swig_type_info* type){
            return PyLong_FromLongLong(elem);
        }
        static void fromPy(PyObject* elem,swig_type_info* type, unsigned long long& res){
            res = PyLong_AsLong(elem);
        }
    };

    template <>
    struct Py<bool>
    {
        static PyObject* toPy(bool elem,swig_type_info* type){
            return PyBool_FromLong(long(elem));
        }
        static void fromPy(PyObject* elem,swig_type_info* type, bool& res){
            res = PyObject_IsTrue(elem);
        }
    };
    

%}

%fragment("doubleToFromPy","header",fragment="toFromPy")
%{
    template <>
    struct Py<float> {
        static PyObject* toPy(float elem,swig_type_info* type){
            return PyFloat_FromDouble(double(elem));
        }
        static void fromPy(PyObject* elem,swig_type_info* type, float& res){
            res = PyFloat_AsDouble(elem);
        }
    };

    template <>
    struct Py<double> {
        static PyObject* toPy(double elem,swig_type_info* type){
            return PyFloat_FromDouble(elem);
        }

        static void fromPy(PyObject* elem,swig_type_info* type, double& res){
            res = PyFloat_AsDouble(elem);
        }
    };
%}

%fragment("stringToFromPy","header",fragment="toFromPy",fragment="SWIG_From_std_string",fragment="SWIG_AsVal_std_string")
%{
    template <>
    struct Py<std::string> {
        static  PyObject* toPy(std::string elem,swig_type_info* type){
            return SWIG_From_std_string(elem);
        }
        static void fromPy(PyObject* elem,swig_type_info* type, std::string& res){
            SWIG_AsVal_std_string(elem,&res);
        }
    };
%}

%fragment("generalToFromPy","header",fragment="toFromPy")
{
    template <class T>
    struct Py<T*> {
        static PyObject* toPy(T* elem,swig_type_info* type){
            if (type == NULL){
                RW_THROW("Cannot convert object to python type when type info is null");
            }
            return SWIG_NewPointerObj(SWIG_as_voidptr(elem),type, 0 | 0  );
        }

        static void fromPy(PyObject* elem,swig_type_info* type, T*& res){
            void *argp;
            int test = SWIG_ConvertPtr(elem, &argp, type,SWIG_POINTER_NO_NULL | 0);
            if (SWIG_IsOK(test)) {
                res = reinterpret_cast< T* > (argp);
            }else {
                RW_THROW("Could not convert to type: " <<  type->str  << ". Python reports type as: " << elem->ob_type->tp_name );
            }
        }
    };
}


%fragment("vectorToFromPy", "header",fragment="toFromPy")
%{
    #include <vector>

    template<class T>
    struct Py<std::pair<T,T>> {
        static  PyObject* toPy(std::pair<T,T> elem,swig_type_info* type){
            PyObject* pyList = PyTuple_New(2);
            PyTuple_SetItem(pyList,0,Py<T>::toPy(elem.first,type));
            PyTuple_SetItem(pyList,1,Py<T>::toPy(elem.second,type));

            return pyList;
        }

        static void fromPy(PyObject* elem,swig_type_info* type, std::pair<T,T>& res){
            
            Py<T>::fromPy(PySequence_GetItem(elem,0),type,res.first);
            Py<T>::fromPy(PySequence_GetItem(elem,1),type,res.second);
            
        }
    };

    template<class T>
    struct Py<std::vector<T>> {
        static  PyObject* toPy(std::vector<T> elem,swig_type_info* type){
            PyObject* pyList = PyList_New(elem.size());
            for(size_t i = 0; i < elem.size(); i++){
                PyList_SetItem(pyList,i,Py<T>::toPy(elem[i],type));
            }
            return pyList;
        }

        static void fromPy(PyObject* elem,swig_type_info* type, std::vector<T>& res){
            res = std::vector<T>(PySequence_Size(elem));
            for(size_t  i = 0; i < res.size(); i++){
                Py<T>::fromPy(PySequence_GetItem(elem,i),type,res[i]);
            }
        }
    };

    template <class T> 
    PyObject* fromVector (const std::vector<T>& list,swig_type_info* type){
        return Py<std::vector<T>>::toPy(list,type);
    }

    template <class T> 
    PyObject* fromVector (std::vector<T>* list,swig_type_info* type){
        return Py<std::vector<T>>::toPy(*list,type);
    }
    template <class T> 
    PyObject* fromVector (rw::core::Ptr<std::vector<T>> list,swig_type_info* type){
        return Py<std::vector<T>>::toPy(*list,type);
    }
    template <class T> 
    PyObject* fromVector (SwigValueWrapper<rw::core::Ptr<T>>& list,swig_type_info* type){
        return fromVector(*((rw::core::Ptr<T>)list),type);
    }

    template <class T> 
    void toVector(PyObject* list, std::vector< T >& result,swig_type_info* type){
        Py<std::vector< T >>::fromPy(list,type,result);
    }
    template <class T> 
    void toVector(PyObject* list, rw::core::Ptr<std::vector< T >>& result,swig_type_info* type){
        Py<std::vector< T >>::fromPy(list,type,*result);
    }

    template <typename T>
    void toVector(PyObject* list, T& result,swig_type_info* type){
        result = T(PySequence_Size(list));
        for(size_t  i = 0; i < result.size(); i++){
            Py<typename T::value_type>::fromPy(PySequence_GetItem(list,i),type,result[i]);
        }
    }

    template <typename T>
    void toVector(PyObject* list, rw::core::Ptr<T>& result,swig_type_info* type){
        *result = T(PySequence_Size(list));
        for(size_t  i = 0; i < result->size(); i++){
            Py<typename T::value_type>::fromPy(PySequence_GetItem(list,i),type,(*result)[i]);
        }
    }

    template <> 
    void toVector<bool>(PyObject* list, std::vector< bool >& result,swig_type_info* type){
        result = std::vector<bool>(PySequence_Size(list));
        for(size_t  i = 0; i < result.size(); i++){
            result[i] = PyObject_IsTrue(PySequence_GetItem(list,i));
        }
    }
%}

%define %std_vector_explicit(name,the_class,fullclass,frag)
%types(the_class);
%fragment(#name,"header",fragment="vectorToFromPy", fragment=frag)
%{
%}

// ###############################################
// #                TYPECHECK                    #
// ###############################################
%typecheck(SWIG_TYPECHECK_SWIGOBJECT,fragment=#name) fullclass
{
    //TYPECHECK TEST
    $1 = PySequence_Check($input);
}

%typecheck(SWIG_TYPECHECK_SWIGOBJECT,fragment=#name) fullclass&
{
    //TYPECHECK TEST
    $1 = PyList_Check($input);
}

%typecheck(SWIG_TYPECHECK_SWIGOBJECT,fragment=#name) const fullclass&
{
    //TYPECHECK TEST
    $1 = PySequence_Check($input);
}

// ###############################################
// #              function input                 #
// ###############################################

%typemap(in, fragment=#name) fullclass (fullclass temp)
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    try {
        toVector($input,*(&temp),type);
        $1 = temp;
    }catch(rw::core::Exception& e ){
        SWIG_exception(SWIG_RuntimeError,e.what());
    }
}

%typemap(in, fragment=#name) fullclass* (fullclass temp)
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    try {
        toVector($input,temp,type);
        $1 = &temp;
    }catch(rw::core::Exception& e ){
        SWIG_exception(SWIG_RuntimeError,e.what());
    }
}

%typemap(in, fragment=#name) const fullclass& (fullclass temp)
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    try {
        toVector($input,temp,type);
        $1 = &temp;
    }catch(rw::core::Exception& e ){
        SWIG_exception(SWIG_RuntimeError,e.what());
    }
}

%typemap(in, fragment=#name) fullclass& (fullclass temp)
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    try {
        toVector($input,temp,type);
        $1 = &temp;
    }catch(rw::core::Exception& e ){
        SWIG_exception(SWIG_RuntimeError,e.what());
    }
}
//Used to make sure the data is transferred back to python
%typemap(argout, fragment=#name) fullclass&
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    PyObject* obj = fromVector($1, type);
    
    size_t sizeI = PyList_Size(obj);
    if(sizeI > 0){
        size_t sizeO = PyList_Size($input);
        if(sizeO < sizeI){
            for(size_t i = 0; i < (sizeI-sizeO) ;i++){
                PyList_Append($input,Py_None);
            }
        }
        PyList_SetSlice($input,0,sizeI,obj);
    }
}
%typemap(argout, fragment=#name) const fullclass&
{
}

%typemap(in, fragment=#name) rw::core::Ptr<fullclass> (rw::core::Ptr<fullclass> temp)
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    try{
        temp = rw::core::ownedPtr(new fullclass());
        toVector($input,temp,type);
        $1 = temp;
    }catch(rw::core::Exception& e ){
        SWIG_exception(SWIG_RuntimeError,e.what());
    }
}

// ###############################################
// #             Function output                 #
// ###############################################

%typemap(out, fragment=#name) fullclass
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    $result = fromVector(&$1, type);
}

%typemap(out, fragment=#name) rw::core::Ptr<fullclass>
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    $result = fromVector($1, type);
}

%typemap(out, fragment=#name) fullclass *
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    $result = fromVector(*$1, type);
}


%typemap(out, fragment=#name) fullclass &
{
%#if defined(%descriptor(the_class))
    swig_type_info* type = %descriptor(the_class);
%#elif defined(%descriptor(the_class*))
    swig_type_info* type = %descriptor(the_class*);
%#else
    swig_type_info* type = SWIG_TypeQuery(SWIG_TO_STRING(the_class));
%#endif 
    $result = fromVector(*$1, type);
}

%pythoncode {
class name(list):
    """
    This class is deprecated and is basically a wrapper around a list
    """
    def __init__(self,arg1 = None,arg2 = None):
        print("Warning the class",str(name),"is Deprecated, use a list instead")
       
        if isinstance(arg1,int):
            super().__init__([arg2]*arg1) 
        elif isinstance(arg1,(tuple,list)):
            super().__init__(list(arg1)) 
        else:
            super().__init__()
    def size(self):
        return super().__len__()
    def empty(self):
        return self.size() == 0
    def at(self,i):
        return super()[i]
    def front(self):
        return super()[0]
    def back(self):
        return super()[-1]
    def push_back(self,elem):    
        super().append(elem)
    def pop_back(self):
        return super().pop()
    def clear():
        super().clear()
}
%enddef

#endif 

%define %std_vector_f(name,class,vec,frag) 
    %std_vector_explicit(name,class,vec<class>,frag)
%enddef

%define %std_vector(name,class)
    %std_vector_explicit(name,class,std::vector<class>,"generalToFromPy");
%enddef

