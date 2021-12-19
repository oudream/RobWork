set(DEFINES "#ifndef SWIG_POINTER_NO_NULL\n#define SWIG_POINTER_NO_NULL 0\n#endif\n"
    "#ifndef SWIGPtr_pre\n#if defined(SWIGLUA)\n#define SWIGPtr_pre L,\n"
    "#else\n#define SWIGPtr_pre\n#endif\n#endif\n\n\n"
)

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
        set(typecheck
            "${typecheck}    else if(SWIG_IsOK(SWIG_ConvertPtr(SWIGPtr_pre $input, (void **) &ptr, $descriptor( ${type} *),SWIG_POINTER_NO_NULL | 0))){ \n"
        )
        set(typecheck "${typecheck}        $1 = 1;\n")
        set(typecheck "${typecheck}    }\n")
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

macro(GENERATE_TYPEMAP_CHECK _type)
    set(options) # Used to marke flags
    set(oneValueArgs FILE CONVERTER) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(TMC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    generate_typecheck("${_type}" TYPES ${TMC_TYPES} FILE ${TMC_FILE})
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

generate_typemap_check(
    "rw::core::Ptr<rw::kinematics::Frame const>"
    TYPES
    "rw::core::Ptr<rw::kinematics::Frame>"
    "rw::kinematics::Frame"
    "const rw::kinematics::Frame"
    "rw::kinematics::FixedFrame"
    "const rw::kinematics::FixedFrame"
    "rw::kinematics::MovableFrame"
    "const rw::kinematics::FixedFrame"
    "rw::core::Ptr<rw::kinematics::FixedFrame>"
    "rw::core::Ptr<rw::kinematics::FixedFrame const>"
    "rw::core::Ptr<rw::kinematics::MovableFrame>"
    "rw::core::Ptr<rw::kinematics::MovableFrame const>"
    FILE
    "./framecptr.i"
    CONVERTER
    "toFrameCPtr"
)

generate_typemap_check(
    "rw::core::Ptr<rw::kinematics::Frame>"
    TYPES
    "rw::core::Ptr<rw::kinematics::Frame>"
    "rw::kinematics::Frame"
    "rw::kinematics::FixedFrame"
    "rw::kinematics::MovableFrame"
    "rw::core::Ptr<rw::kinematics::FixedFrame>"
    "rw::core::Ptr<rw::kinematics::MovableFrame>"
    FILE
    "./frameptr.i"
    CONVERTER
    "toFramePtr"
)
