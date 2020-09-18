%module sdurw_core

%pragma(java) jniclassclassmodifiers="class"
#if defined (SWIGJAVA)
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
SWIG_JAVABODY_TYPEWRAPPER(public, public, public, SWIGTYPE)
#endif

%include <rwlibs/swig/swig_macros.i>
%include <rwlibs/swig/ext_i/std.i>
%include <rwlibs/swig/ext_i/boost.i>




%rename(getDeref) rw::core::Ptr::operator->;
%rename(deref) rw::core::Ptr::get;

%{
    #include <rw/core/Ptr.hpp>
%}

#if defined(SWIGPYTHON)
    %pythonprepend ownedPtr(T*) %{
    args[0].thisown = 0
    %}
#endif


%include <rw/core/Ptr.hpp>

#if defined(SWIGPYTHON)
    %pythonprepend rw::core::ownedPtr(T*) %{
    args[0].thisown = 0
    %}
#endif

#define NAME_PTR(x) x ## Ptr
#define NAME_CPTR(x) x ## CPtr 
%define OWNEDPTR(ownedPtr_type)
    namespace rw { namespace core {
        #if defined(SWIGJAVA)
            %typemap (in) ownedPtr_type* %{
                jclass objcls = jenv->GetObjectClass(jarg1_);
                const jfieldID memField = jenv->GetFieldID(objcls, "swigCMemOwn", "Z");
                jenv->SetBooleanField(jarg1_, memField, (jboolean)false);
                $1 = *(std::remove_const<ownedPtr_type>::type **)&jarg1;
            %}
        #elif defined(SWIGLUA)
            %typemap (in,checkfn="SWIG_isptrtype") ownedPtr_type* %{
                if (!SWIG_IsOK(SWIG_ConvertPtr(L,$input,(void**)&$1,$descriptor,SWIG_POINTER_DISOWN))){
                    SWIG_fail_ptr("$symname",$input,$descriptor);
            }
            %}
        #endif
        %template (ownedPtr) ownedPtr<ownedPtr_type>;
        #if (defined(SWIGLUA) || defined(SWIGJAVA))
            %clear ownedPtr_type*;
        #endif
    }}
%enddef

%define NAMED_OWNEDPTR(name,ownedPtr_type)
    %template(NAME_PTR(name)) rw::core::Ptr<ownedPtr_type>;
    %template(NAME_CPTR(name)) rw::core::Ptr<ownedPtr_type const>;

    %extend rw::core::Ptr<ownedPtr_type>{
         Ptr< const ownedPtr_type > cptr () {
             return $self->cptr();
         }
    }
    OWNEDPTR(ownedPtr_type);
%enddef

%{
    #include <rw/core/AnyPtr.hpp>
%}
%include <rw/core/AnyPtr.hpp>

%{
    #include <rw/core/DOMElem.hpp>
%}
%include <rw/core/DOMElem.hpp>
NAMED_OWNEDPTR(DOMElem, rw::core::DOMElem);

%{
    #include <rw/core/ExtensionPoint.hpp>
%}
%include <rw/core/ExtensionPoint.hpp>
%template (ExtensionPointDOMParser) rw::core::ExtensionPoint<rw::core::DOMParser>;

%{
    #include <rw/core/DOMParser.hpp>
%}
%include <rw/core/DOMParser.hpp>
NAMED_OWNEDPTR(DOMParser, rw::core::DOMParser);

%{
    #include <rw/core/DOMPropertyMapFormat.hpp>
%}
%include <rw/core/DOMPropertyMapFormat.hpp>

%{
    #include <rw/core/Event.hpp>
%}
%include <rw/core/Event.hpp>

#if defined(SWIGJAVA)
    RENAME(rw::core::Exception, RWException)
#endif

%{
    #include <rw/core/Exception.hpp>
%}
%include <rw/core/Exception.hpp>

%{
    #include <rw/core/Extension.hpp>
%}
%include <rw/core/Extension.hpp>
NAMED_OWNEDPTR(Extension, rw::core::Extension);
%template(VectorExtensionPtr) std::vector<rw::core::Ptr<rw::core::Extension>>;

%{
    #include <rw/core/ExtensionRegistry.hpp>
%}
%include <rw/core/ExtensionRegistry.hpp>

%{
    #include <rw/core/IOUtil.hpp>
%}
%include <rw/core/IOUtil.hpp>

%{
    #include <rw/core/Log.hpp>
%}
%include <rw/core/Log.hpp>
NAMED_OWNEDPTR(Log, rw::core::Log);

%extend rw::core::Log{
    /**
     * @brief Returns the LogWriter that is associated with LogIndex \b id
     *
     * If the \b id is unknown an exception is thrown.
     *
     * @param id [in] loglevel
     * @return Reference to LogWriter object
     */
    rw::core::LogWriter& getLogWriter(LogIndex id) {
        return $self->get(id);
    }
};

%{
    #include <rw/core/LogWriter.hpp>
%}
%include <rw/core/LogWriter.hpp>
NAMED_OWNEDPTR(LogWriter, rw::core::LogWriter);

#if !defined(SWIGLUA) && !defined(SWIGPYTHON)
%{
    #include <rw/core/LogStreamWriter.hpp>
%}
%include <rw/core/LogStreamWriter.hpp>
#endif 

%{
    #include <rw/core/Message.hpp>
%}
%include <rw/core/Message.hpp>

%{
    #include <rw/core/os.hpp>
%}
%include <rw/core/os.hpp>

%{
    #include <rw/core/Plugin.hpp>
%}
%include <rw/core/Plugin.hpp>
NAMED_OWNEDPTR(Plugin, rw::core::Plugin);

%{
    #include <rw/core/Property.hpp>
%}
%include <rw/core/Property.hpp>

%{
    #include <rw/core/PropertyBase.hpp>
%}
%include <rw/core/PropertyBase.hpp>
NAMED_OWNEDPTR(PropertyBase, rw::core::PropertyBase);

%{
    #include <rw/core/PropertyMap.hpp>
%}
%include <rw/core/PropertyMap.hpp>
%template(getBool) rw::core::PropertyMap::get<bool>;
%template(getInt) rw::core::PropertyMap::get<int>;
%template(getFloat) rw::core::PropertyMap::get<float>;
%template(getDouble) rw::core::PropertyMap::get<double>;
#if defined(SWIGPYTHON)
%template(getString) rw::core::PropertyMap::get<std::string>;
#else
%extend rw::core::PropertyMap {
    std::string getString(const std::string& id){ return $self->get<std::string>(id); }
}
#endif
%template(getVectorString) rw::core::PropertyMap::get<std::vector<std::string>>;
%template(getVectorBool) rw::core::PropertyMap::get<std::vector<bool>>;
%template(getVectorInt) rw::core::PropertyMap::get<std::vector<int>>;
%template(getVectorFloat) rw::core::PropertyMap::get<std::vector<float>>;
%template(getVectorDouble) rw::core::PropertyMap::get<std::vector<double>>;
%template(setBool) rw::core::PropertyMap::set<bool>;
%template(setInt) rw::core::PropertyMap::set<int>;
%template(setFloat) rw::core::PropertyMap::set<float>;
%template(setDouble) rw::core::PropertyMap::set<double>;
%template(setString) rw::core::PropertyMap::set<std::string>;
%template(setVectorString) rw::core::PropertyMap::set<std::vector<std::string>>;
%template(setVectorBool) rw::core::PropertyMap::set<std::vector<bool>>;
%template(setVectorInt) rw::core::PropertyMap::set<std::vector<int>>;
%template(setVectorFloat) rw::core::PropertyMap::set<std::vector<float>>;
%template(setVectorDouble) rw::core::PropertyMap::set<std::vector<double>>;



%{
    #include <rw/core/PropertyType.hpp>
%}
%include <rw/core/PropertyType.hpp>

%{
    #include <rw/core/RobWork.hpp>
%}
%include <rw/core/RobWork.hpp>

NAMED_OWNEDPTR(RobWork, rw::core::RobWork);

%{
    #include <rw/core/StringUtil.hpp>
%}
%include <rw/core/StringUtil.hpp>

