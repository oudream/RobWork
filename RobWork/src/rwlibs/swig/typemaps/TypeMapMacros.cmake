set(DEFINES "#ifndef SWIG_POINTER_NO_NULL\n#define SWIG_POINTER_NO_NULL 0\n#endif\n"
    "#ifndef SWIGPtr_pre\n#if defined(SWIGLUA)\n#define SWIGPtr_pre L,\n"
    "#else\n#define SWIGPtr_pre\n#endif\n#endif\n\n\n"
)

macro(GENERATE_INCLUDES )
    set(options APPEND) # Used to marke flags
    set(oneValueArgs CONVERTER FILE) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(INC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(_include)
    foreach(arg ${ARGN})
        string(REPLACE "rw::core::Ptr<" "" arg ${arg})
        string(REPLACE ">" "" arg ${arg})
        string(REPLACE " " "" arg ${arg})
        string(REPLACE "::" "/" arg ${arg})
        string(REPLACE "<" "\\;" arg ${arg})

        string(REPLACE "const" "" arg ${arg})
        foreach(subarg ${arg})
            if(EXISTS ${RW_ROOT}/src/${subarg}.hpp)
                list(APPEND _include "#include <${subarg}.hpp>")
            endif()
        endforeach()
    endforeach()
    list(REMOVE_DUPLICATES _include)
    string(REPLACE ";" "\n" _include "${_include}")

    set(_include "\n\n%fragment(\"${INC_CONVERTER}Include\", \"header\")%{\n${_include}\n%}\n\n")

    if(NOT ${INC_FILE} STREQUAL "")
        if(INC_APPEND)
            file(APPEND ${INC_FILE} "${_include}")
        else()
            file(WRITE ${INC_FILE} ${DEFINES} "${_include}")
        endif()
    endif()
endmacro()

macro(GENERATE_TYPECHECK _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs RESULT FILE) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(TC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(typecheck "#if !defined(SWIGJAVA)\n")
    set(typecheck "${typecheck}%typecheck(SWIG_TYPECHECK_SWIGOBJECT) ${_type} {\n")
    set(typecheck "${typecheck}    void* ptr;\n")
    set(typecheck "${typecheck}    $1 = 0;\n")
    set(typecheck
        "${typecheck}    if (SWIG_IsOK(SWIG_ConvertPtr(SWIGPtr_pre $input, (void **) &ptr, $descriptor( ${_type} *),SWIG_POINTER_NO_NULL | 0))) {\n"
    )
    set(typecheck "${typecheck}        $1 = 1;\n")
    set(typecheck "${typecheck}    }\n")

    foreach(type ${TC_TYPES})
        if(${type} STREQUAL "int")
            set(typecheck "${typecheck}#if defined(SWIGPYTHON)\n")
            set(typecheck "${typecheck}    else if(PyLong_Check($input)){ \n")
            set(typecheck "${typecheck}#elif defined(SWIGLUA)\n")
            set(typecheck "${typecheck}    else if(lua_isnumber(L,$input)){ \n")
            set(typecheck "${typecheck}#else \n")
            set(typecheck "${typecheck}    else if(lua_isnumber(L,$input)){ \n")
            set(typecheck "${typecheck}#endif\n")
            set(typecheck "${typecheck}        $1 = 1;\n")
            set(typecheck "${typecheck}    }\n")
        else()
            set(typecheck
                "${typecheck}    else if(SWIG_IsOK(SWIG_ConvertPtr(SWIGPtr_pre $input, (void **) &ptr, $descriptor( ${type} *),SWIG_POINTER_NO_NULL | 0))){ \n"
            )
            set(typecheck "${typecheck}        $1 = 1;\n")
            set(typecheck "${typecheck}    }\n")
        endif()
    endforeach()

    set(typecheck "${typecheck}    if ( ! $1 ) {\n")
    set(typecheck "${typecheck}    std::cout << \"Failed to verify ${_type} \" << std::endl;\n")
    set(typecheck "${typecheck}    }\n")
    set(typecheck "${typecheck}}\n")
    set(typecheck "${typecheck}#endif\n")

    if(NOT ${TC_RESULT} STREQUAL "")
        set(${TC_RESULT} ${typecheck})
    endif()

    if(NOT ${TC_FILE} STREQUAL "")
        if(TC_APPEND)
            file(APPEND ${TC_FILE} "${typecheck}")
        else()
            file(WRITE ${TC_FILE} ${DEFINES} "${typecheck}")
        endif()
    endif()

endmacro()

macro(GENERATE_TYPEMAP _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs RESULT FILE CONVERTER) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(TM "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(typemap)
    set(typemap "${typemap}#if !defined(SWIGJAVA)\n")
    set(typemap "${typemap}%typemap(in, fragment=\"${TM_CONVERTER}\") ${_type} (void * argp){\n")
    set(typemap
        "${typemap}    int res = SWIG_ConvertPtr(SWIGPtr_pre $input, &argp, $descriptor(${_type} *),SWIG_POINTER_NO_NULL | 0);\n"
    )
    set(typemap "${typemap}    if (SWIG_IsOK(res)) {\n")
    set(typemap "${typemap}        ${_type} * tmp_var = reinterpret_cast< ${_type} * > (argp);\n")
    set(typemap "${typemap}        $1 = *tmp_var;\n")
    set(typemap "${typemap}    }\n")

    foreach(type ${TM_TYPES})
        if(${type} STREQUAL "int")
            set(typemap "${typemap}    if(!SWIG_IsOK(res)) {\n")
            set(typemap "${typemap}#if defined(SWIGPYTHON)\n")
            set(typemap "${typemap}         if(PyLong_Check($input)){ \n")
            set(typemap "${typemap}             $1 = ${TM_CONVERTER} (PyLong_AsLong($input));\n")
            set(typemap "${typemap}#elif defined(SWIGLUA)\n")
            set(typemap "${typemap}         if(lua_isnumber(L,$input)){ \n")
            set(typemap "${typemap}             $1 = ${TM_CONVERTER} (lua_tonumber(L,$input));\n")
            set(typemap "${typemap}#endif\n")
            set(typemap "${typemap}             res = SWIG_OK;\n")
            set(typemap "${typemap}         }\n")
            set(typemap "${typemap}    }\n")
        else()
            set(typemap "${typemap}    if(!SWIG_IsOK(res)) {\n")
            set(typemap
                "${typemap}        res = SWIG_ConvertPtr(SWIGPtr_pre $input, &argp, $descriptor(${type} *),SWIG_POINTER_NO_NULL | 0);\n"
            )
            set(typemap "${typemap}        if (SWIG_IsOK(res)) {\n")
            set(typemap
                "${typemap}            ${type}  * tmp_var = reinterpret_cast< ${type} *> (argp);\n"
            )
            set(typemap "${typemap}            $1 = ${TM_CONVERTER} (tmp_var);\n")
            set(typemap "${typemap}        }\n")
            set(typemap "${typemap}    }\n")
        endif()
    endforeach()

    set(typemap "${typemap}    if(!SWIG_IsOK(res)) {\n")
    set(typemap "${typemap}#if !defined(SWIGLUA)\n")
    set(typemap
        "${typemap}        SWIG_exception_fail(SWIG_ArgError(res), \"could not convert to type ${_type}\");\n"
    )
    set(typemap "${typemap}#else \n")
    set(typemap "${typemap}        lua_pushstring(L, \"could not convert to type ${_type}\");\n")
    set(typemap "${typemap}        SWIG_fail;\n")
    set(typemap "${typemap}#endif \n")
    set(typemap "${typemap}    }\n")
    set(typemap "${typemap}}\n")
    set(typemap "${typemap}#endif\n")

    if(NOT ${TM_RESULT} STREQUAL "")
        set(${TM_RESULT} ${typemap})
    endif()

    if(NOT ${TM_FILE} STREQUAL "")
        if(TM_APPEND)
            file(APPEND ${TM_FILE} "${typemap}")
        else()
            file(WRITE ${TM_FILE} ${DEFINES} "${typemap}")
        endif()
    endif()

endmacro()

macro(GENERATE_STANDARD_PTR_FRAGMENT _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs CONVERTER FILE) # used to marke values with a single value
    set(multiValueArgs)

    cmake_parse_arguments(STD_PTR_F "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(fragment)
    set(fragment "${fragment}%fragment(\"${STD_PTR_F_CONVERTER}\", \"header\",fragment=\"${STD_PTR_F_CONVERTER}Include\"){\n")
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    ${_type} ${STD_PTR_F_CONVERTER}(rw::core::Ptr<T>* in){\n")
    set(fragment "${fragment}        return *in;\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    ${_type} ${STD_PTR_F_CONVERTER}(T* in){\n")
    set(fragment "${fragment}        return in;\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    ${_type} ${STD_PTR_F_CONVERTER}(T in){\n")
    set(fragment "${fragment}        if(in == 0){\n")
    set(fragment "${fragment}            return ${_type}();\n")
    set(fragment "${fragment}        }else{\n")
    set(fragment
        "${fragment}            RW_THROW(\"an non-NULL type number was parsed as a ${_type}\");\n"
    )
    set(fragment "${fragment}        }\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}}\n")

    if(NOT ${STD_PTR_F_FILE} STREQUAL "")
        if(STD_PTR_F_APPEND)
            file(APPEND ${STD_PTR_F_FILE} "${fragment}")
        else()
            file(WRITE ${STD_PTR_F_FILE} ${DEFINES} "${fragment}")
        endif()
    endif()
endmacro()

macro(GENERATE_STANDARD_POINTER_FRAGMENT _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs CONVERTER FILE) # used to marke values with a single value
    set(multiValueArgs)

    cmake_parse_arguments(STD_POINTER_F "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(fragment)
    set(fragment "${fragment}%fragment(\"${STD_POINTER_F_CONVERTER}\", \"header\",fragment=\"${STD_POINTER_F_CONVERTER}Include\"){\n")
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    ${_type} ${STD_POINTER_F_CONVERTER}(rw::core::Ptr<T>* in){\n")
    set(fragment "${fragment}        return in->get();\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    ${_type} ${STD_POINTER_F_CONVERTER}(T* in){\n")
    set(fragment "${fragment}        return in;\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}}\n")

    if(NOT ${STD_POINTER_F_FILE} STREQUAL "")
        if(STD_POINTER_F_APPEND)
            file(APPEND ${STD_POINTER_F_FILE} "${fragment}")
        else()
            file(WRITE ${STD_POINTER_F_FILE} ${DEFINES} "${fragment}")
        endif()
    endif()
endmacro()

macro(GENERATE_TYPEMAP_CHECK _type)
    set(options ADD_RWPTR_FRAGMENT ADD_POINTER_FRAGMENT) # Used to marke flags
    set(oneValueArgs FILE CONVERTER) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(TMC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    generate_typecheck("${_type}" TYPES ${TMC_TYPES} FILE ${TMC_FILE})
    GENERATE_INCLUDES(TYPES ${TMC_TYPES} CONVERTER "${TMC_CONVERTER}" FILE ${TMC_FILE} APPEND)

    if(TMC_ADD_RWPTR_FRAGMENT)
        generate_standard_ptr_fragment(
            "${_type}" CONVERTER "${TMC_CONVERTER}" FILE ${TMC_FILE} APPEND
        )
    elseif(TMC_ADD_POINTER_FRAGMENT)
        generate_standard_pointer_fragment(
            "${_type}" CONVERTER "${TMC_CONVERTER}" FILE ${TMC_FILE} APPEND
        )
    endif()

    generate_typemap(
        "${_type}"
        TYPES
        ${TMC_TYPES}
        FILE
        ${TMC_FILE}
        CONVERTER
        "${TMC_CONVERTER}"
        APPEND
    )
endmacro()
