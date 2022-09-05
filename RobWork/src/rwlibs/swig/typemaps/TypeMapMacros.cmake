set(DEFINES
    "#ifndef SWIG_POINTER_NO_NULL\n
#    define SWIG_POINTER_NO_NULL 0
#endif
#ifndef SWIGPtr_pre
#    if defined(SWIGLUA)
#        define SWIGPtr_pre L,
#    else
#        define SWIGPtr_pre
#    endif
#endif
#ifndef LUASTATE_pre
#    if defined(SWIGLUA)
#        define LUASTATE_pre lua_State *L,
#    else
#        define LUASTATE_pre
#    endif
#endif\n\n\n"
)

macro(TYPEMAP_TOFILE _file _append)

    if(NOT ${_file} STREQUAL "")
        if(${_append})
            file(APPEND ${_file} "${ARGN}")
        else()
            file(WRITE ${_file} ${DEFINES} "${ARGN}")
        endif()
    endif()

endmacro()

macro(TYPEMAP_ISPOINTER _type _res)
    set(${_res} FALSE)
    string(FIND ${_type} "*" PPOS REVERSE)
    string(LENGTH ${_type} TYPE_LENGTH)
    if((CMAKE_VERSION VERSION_GREATER 3.13.5) OR (CMAKE_VERSION VERSION_EQUAL 3.13.5))
        math(EXPR TYPE_LENGTH "${TYPE_LENGTH}-2" OUTPUT_FORMAT DECIMAL)
    else()
        math(EXPR TYPE_LENGTH "${TYPE_LENGTH}-2")
    endif()
    if(${PPOS} GREATER ${TYPE_LENGTH})
        set(${_res} TRUE)
    endif()
endmacro()

macro(TYPEMAP_ISPTR _type _res)
    set(${_res} FALSE)
    string(FIND ${_type} "rw::core::Ptr" PPOS)
    if(${PPOS} GREATER_EQUAL "0")
        set(${_res} TRUE)
    endif()
endmacro()

macro(TYPEMAP_AS_NONE_PTR _type _res)
    string(REPLACE "rw::core::Ptr" "" NONPTR ${_type})
    string(REGEX REPLACE "^\\s*<" "" NONPTR ${NONPTR})
    string(REGEX REPLACE ">\\s*$" "" ${_res} ${NONPTR})

endmacro()

macro(TYPEMAP_AS_NONE_POINTER _type _res)
    string(REPLACE "*" "" ${_res} ${_type})
endmacro()

macro(GENERATE_INCLUDES)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs CONVERTER FILE) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(INC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(_include)
    foreach(arg ${INC_TYPES})
        string(REPLACE "rw::core::Ptr<" "" arg ${arg})
        string(REPLACE ">" "" arg ${arg})
        string(REPLACE " " "" arg ${arg})
        string(REPLACE "::" "/" arg ${arg})
        string(REPLACE "<" "\\;" arg ${arg})

        string(REPLACE "const" "" arg ${arg})
        foreach(subarg ${arg})

            if(EXISTS ${RW_ROOT}/src/${subarg}.hpp)
                list(APPEND _include "#include <${subarg}.hpp>")
            else()
                string(REGEX MATCH "[A-Za-z0-9_]*$" subarg "${subarg}")
                file(
                    GLOB_RECURSE dir
                    RELATIVE "${RW_ROOT}/src"
                    "${RW_ROOT}/src/*/${subarg}.hpp"
                )
                if(EXISTS ${RW_ROOT}/src/${dir} AND NOT "${dir}" STREQUAL "" )
                    list(APPEND _include "#include <${dir}>")
                endif()
            endif()
        endforeach()
    endforeach()
    if("${_include}" STREQUAL "")
        message(STATUS "Could not find includes for: ${INC_TYPES}")
    else()
        list(REMOVE_DUPLICATES _include)
    endif()
    string(REPLACE ";" "\n" _include "${_include}")

    set(_include "\n\n%fragment(\"${INC_CONVERTER}Include\", \"header\")%{\n${_include}\n%}\n\n")

    typemap_tofile(${INC_FILE} ${INC_APPEND} ${_include})
endmacro()

macro(GENERATE_TYPECHECK _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs CONVERTER RESULT FILE) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(TC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(body)
    set(body "${body}    ${_type} res;\n")
    set(body "${body}    $1 = fromSWIG(SWIGPtr_pre $input,res,true);\n")
    set(body "${body}    if ( ! $1 ) {\n")
    set(body "${body}    std::cout << \"Failed to verify ${_type} \" << std::endl;\n")
    set(body "${body}#if defined(SWIGPYTHON)\n")
    set(body
        "${body}     std::cout << \"Python reports type as: \" << $input->ob_type->tp_name << std::endl;\n"
    )
    set(body "${body}#endif\n")
    set(body "${body}    }\n")

    set(typecheck "#if !defined(SWIGJAVA)\n")
    # ##############################################################################################
    # Normal TYPECHECK
    # ##############################################################################################
    set(typecheck
        "${typecheck}%typemap(typecheck, precedence=SWIG_TYPECHECK_SWIGOBJECT, fragment=\"${TC_CONVERTER}FromSwig\") ${_type} {\n"
    )
    set(typecheck "${typecheck}${body}")
    set(typecheck "${typecheck}}\n")
    # ##############################################################################################
    # & TYPECHECK
    # ##############################################################################################
    set(typecheck
        "${typecheck}%typemap(typecheck, precedence=SWIG_TYPECHECK_SWIGOBJECT, fragment=\"${TC_CONVERTER}FromSwig\") ${_type}& {\n"
    )
    set(typecheck "${typecheck}${body}")
    set(typecheck "${typecheck}}\n")
    # ##############################################################################################
    # const & TYPECHECK
    # ##############################################################################################
    set(typecheck
        "${typecheck}%typemap(typecheck, precedence=SWIG_TYPECHECK_SWIGOBJECT, fragment=\"${TC_CONVERTER}FromSwig\") const ${_type}& {\n"
    )
    set(typecheck "${typecheck}${body}")
    set(typecheck "${typecheck}}\n")
    set(typecheck "${typecheck}#endif\n")

    if(NOT ${TC_RESULT} STREQUAL "")
        set(${TC_RESULT} ${typecheck})
    endif()

    typemap_tofile(${TC_FILE} ${TC_APPEND} ${typecheck})
endmacro()

macro(GENERATE_TYPEMAP _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs RESULT FILE CONVERTER) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(TM "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(tm_end)
    set(tm_end "${tm_end}    if(!res) {\n")
    set(tm_end "${tm_end}#if defined(SWIGPYTHON)\n")
    set(tm_end
        "${tm_end}        SWIG_exception_fail(SWIG_ArgError(-1),(std::string(\"Could not convert to type ${_type}\") +\". Python reports type as: \" + std::string($input->ob_type->tp_name)).c_str() );\n"
    )
    set(tm_end "${tm_end}#else \n")
    set(tm_end "${tm_end}        lua_pushstring(L, \"could not convert to type ${_type}\");\n")
    set(tm_end "${tm_end}        SWIG_fail;\n")
    set(tm_end "${tm_end}#endif \n")
    set(tm_end "${tm_end}    }\n")
    set(tm_end "${tm_end}}\n")

    set(typemap)
    set(typemap "${typemap}#if !defined(SWIGJAVA)\n")
    set(typemap "${typemap}%typemap(in, fragment=\"${TM_CONVERTER}FromSwig\") ${_type} {\n")
    set(typemap "${typemap}    bool res = fromSWIG(SWIGPtr_pre $input,$1,false);\n")
    set(typemap "${typemap}${tm_end}")
    set(typemap
        "${typemap}%typemap(in, fragment=\"${TM_CONVERTER}FromSwig\") ${_type}&  (${_type} temp) {\n"
    )
    set(typemap "${typemap}    bool res = fromSWIG(SWIGPtr_pre $input,temp,false);\n")
    set(typemap "${typemap}    $1 = &temp;\n")
    set(typemap "${typemap}${tm_end}")
    set(typemap
        "${typemap}%typemap(in, fragment=\"${TM_CONVERTER}FromSwig\") const ${_type}& (${_type} temp) {\n"
    )
    set(typemap "${typemap}    bool res = fromSWIG(SWIGPtr_pre $input,temp,false);\n")
    set(typemap "${typemap}    $1 = &temp;\n")
    set(typemap "${typemap}${tm_end}")
    set(typemap "${typemap}#endif\n")

    if(NOT ${TM_RESULT} STREQUAL "")
        set(${TM_RESULT} ${typemap})
    endif()

    typemap_tofile(${TM_FILE} ${TM_APPEND} ${typemap})
endmacro()

macro(GENERATE_STANDARD_PTR_FRAGMENT _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs CONVERTER FILE) # used to marke values with a single value
    set(multiValueArgs)

    cmake_parse_arguments(STD_PTR_F "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(fragment)
    set(fragment
        "${fragment}%fragment(\"${STD_PTR_F_CONVERTER}\", \"header\",fragment=\"${STD_PTR_F_CONVERTER}Include\"){\n"
    )
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
    set(fragment "${fragment}}\n\n")

    typemap_tofile(${STD_PTR_F_FILE} ${STD_PTR_F_APPEND} ${fragment})
endmacro()

macro(GENERATE_STANDARD_POINTER_FRAGMENT _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs CONVERTER FILE) # used to marke values with a single value
    set(multiValueArgs)

    cmake_parse_arguments(STD_POINTER_F "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(fragment)
    set(fragment
        "${fragment}%fragment(\"${STD_POINTER_F_CONVERTER}\", \"header\",fragment=\"${STD_POINTER_F_CONVERTER}Include\"){\n"
    )
    typemap_as_none_pointer(${_type} type_nonptr)
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    ${_type} ${STD_POINTER_F_CONVERTER}(rw::core::Ptr<T>* in){\n")
    set(fragment "${fragment}        return in->get();\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    ${_type} ${STD_POINTER_F_CONVERTER}(T* in){\n")
    set(fragment "${fragment}        return in;\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    ${_type} ${STD_POINTER_F_CONVERTER}(T in){\n")
    set(fragment "${fragment}        return NULL;\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}}\n\n")

    typemap_tofile(${STD_POINTER_F_FILE} ${STD_POINTER_F_APPEND} ${fragment})
endmacro()

macro(GENERATE_FROM_SWIG_FRAGMENT _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs FILE CONVERTER) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(FS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    list(REVERSE FS_TYPES)
    typemap_ispointer(${_type} IS_POINTER)
    typemap_isptr(${_type} IS_PTR)

    set(types_done)
    if(${IS_POINTER})
        string(REPLACE "*" "" types_done ${_type})
    endif()

    # ##############################################################################################
    # SETUP FRAGMENT  FROMSWIG           #
    # ##############################################################################################

    set(fragment)
    set(fragment "${fragment}#if defined(SWIGPYTHON) || defined(SWIGLUA)\n")
    set(fragment
        "${fragment}%fragment(\"${FS_CONVERTER}FromSwig\",\"header\",fragment=\"${FS_CONVERTER}\")\n"
    )

    # ##############################################################################################
    # Converter
    # ##############################################################################################
    set(fragment "${fragment}{\n")
    set(fragment "${fragment}    template<class T, class R>\n")
    set(fragment
        "${fragment}    bool convert(LUASTATE_pre swig_type_info* descriptor,R elem, ${_type}& res, bool isCheck){\n"
    )
    set(fragment "${fragment}        void* argp;\n")
    set(fragment
        "${fragment}        int success = SWIG_ConvertPtr(SWIGPtr_pre elem,&argp,descriptor,SWIG_POINTER_NO_NULL|0);\n"
    )
    set(fragment "${fragment}        if(!SWIG_IsOK(success)) return false;\n")
    set(fragment "${fragment}        if(isCheck) return true;\n")
    set(fragment "${fragment}        T tmp_value = reinterpret_cast< T> (argp);\n")
    set(fragment "${fragment}        res = ${FS_CONVERTER} (tmp_value);\n")
    set(fragment "${fragment}        return true;\n")
    set(fragment "${fragment}    }\n\n\n")

    # ##############################################################################################
    # fromSwig Function
    # ##############################################################################################
    set(fragment "${fragment}    template<class T>\n")
    set(fragment "${fragment}    bool fromSWIG(LUASTATE_pre T elem, ${_type}& res,bool isCheck){\n")

    if(${IS_POINTER})
        set(fragment
            "${fragment}        if(convert<${_type}>(SWIGPtr_pre $descriptor(${_type} *),elem,res,isCheck)) return true;\n"
        )
    else()
        set(fragment
            "${fragment}        if(convert<${_type}*>(SWIGPtr_pre $descriptor(${_type} *),elem,res,isCheck)) return true;\n"
        )
    endif()
    foreach(type ${FS_TYPES})
        if(${type} STREQUAL "int")
            set(fragment "${fragment}#if defined(SWIGPYTHON)\n")
            set(fragment "${fragment}        if(PyLong_Check(elem)){ \n")
            set(fragment "${fragment}            long v = PyLong_AsLong(elem);\n")
            set(fragment "${fragment}#else\n")
            set(fragment "${fragment}        if(lua_isnumber(L,elem)){ \n")
            set(fragment "${fragment}            long v = lua_tonumber(L,elem);\n")
            set(fragment "${fragment}#endif\n")
            set(fragment
                "${fragment}            if(isCheck) return v == 0; //Only NULL pointers allowed;\n"
            )
            set(fragment "${fragment}            res = ${FS_CONVERTER} (v);\n")
            set(fragment "${fragment}            return true;\n")
            set(fragment "${fragment}        }\n")
        else()
            set(fragment
                "${fragment}        if(convert<${type}*>(SWIGPtr_pre $descriptor(${type} *),elem,res,isCheck)) return true;\n"
            )
        endif()
    endforeach()
    set(fragment "${fragment}        return false;\n")
    set(fragment "${fragment}    }\n")
    set(fragment "${fragment}}\n")
    set(fragment "${fragment}#endif\n\n\n")

    typemap_tofile(${FS_FILE} ${FS_APPEND} ${fragment})
endmacro()

macro(GENERATE_PYTHON_FRAGMENT _type)
    set(options APPEND) # Used to marke flags
    set(oneValueArgs FILE CONVERTER) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(PYF "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    list(REVERSE PYF_TYPES)
    typemap_ispointer(${_type} IS_POINTER)
    typemap_isptr(${_type} IS_PTR)

    set(types_done)
    if(${IS_POINTER})
        string(REPLACE "*" "" types_done ${_type})
    endif()

    # ##############################################################################################
    # SETUP FRAGMENT  TO PY            #
    # ##############################################################################################

    set(fragment)
    set(fragment "${fragment}#ifdef SWIGPYTHON\n")
    set(fragment
        "${fragment}%fragment(\"${PYF_CONVERTER}Py\",\"header\",fragment=\"toFromPy\",fragment=\"${PYF_CONVERTER}\" ,fragment=\"${PYF_CONVERTER}FromSwig\")\n"
    )
    set(fragment "${fragment}{\n")
    set(fragment "${fragment}    #include <rw/core/macros.hpp>\n")
    set(fragment "${fragment}    template <>\n")
    set(fragment "${fragment}    struct Py<${_type}> {\n")
    set(fragment "${fragment}        static PyObject* toPy(${_type} elem, swig_type_info* type){\n")
    set(fragment "${fragment}            if(type==NULL)\n")
    set(fragment
        "${fragment}                RW_THROW(\"Type info is null, can't make PyObject\");\n"
    )

    # ##############################################################################################
    # Create dynamic converters          #
    # ##############################################################################################
    foreach(type ${PYF_TYPES})
        typemap_isptr(${type} ptr)
        if(${type} STREQUAL "int")

        elseif(${IS_POINTER})
            # Convert type to non ptr type
            if(${ptr})
                typemap_as_none_ptr(${type} type_nonptr)
            else()
                set(type_nonptr ${type})
            endif()

            # check that type has not been added already
            list(FIND types_done ${type_nonptr} IS_ADDED)
            if(${IS_ADDED} LESS 0)

                list(APPEND types_done ${type_nonptr})
                set(fragment "${fragment}            if(dynamic_cast<${type_nonptr}*>(elem)) \n")
                set(fragment
                    "${fragment}                return SWIG_NewPointerObj(SWIG_as_voidptr(elem),$descriptor(${type_nonptr}*),0); \n"
                )
            endif()
        elseif(${IS_PTR} AND ${ptr})
            list(FIND types_done ${type} IS_ADDED)
            if(${IS_ADDED} LESS 0)
                list(APPEND types_done ${type})
                set(fragment "${fragment}            if(elem.cast<${type}::value_type>()) \n")
                set(fragment
                    "${fragment}                return SWIG_NewPointerObj(new ${type}(elem.cast<${type}::value_type>()),$descriptor(${type}*),SWIG_POINTER_OWN); \n"
                )
            endif()
        endif()
    endforeach()
    if(IS_POINTER)
        set(fragment
            "${fragment}        return SWIG_NewPointerObj(SWIG_as_voidptr(elem),type, 0 | 0  );\n"
        )
    else()
        set(fragment
            "${fragment}        return SWIG_NewPointerObj(new ${_type} (elem),type,SWIG_POINTER_OWN);\n"
        )
    endif()
    set(fragment "${fragment}        }\n")
    set(fragment "${fragment}\n")

    # ##############################################################################################
    # Setup Function From Py             #
    # ##############################################################################################

    set(fragment
        "${fragment}        static void fromPy(PyObject* elem,swig_type_info* type, ${_type}& res){\n"
    )
    set(fragment "${fragment}           if(!fromSWIG(elem,res,false)){\n")
    set(fragment
        "${fragment}                RW_THROW(\"Could not convert to type: \" <<  type->str  << \". Python reports type as: \" << elem->ob_type->tp_name );\n"
    )
    set(fragment "${fragment}            }\n")
    set(fragment "${fragment}        }\n")
    set(fragment "${fragment}    };\n")
    set(fragment "${fragment}}\n")
    set(fragment "${fragment}#endif\n")
    set(fragment "${fragment}\n")
    set(fragment "${fragment}\n")

    typemap_tofile(${PYF_FILE} ${PYF_APPEND} ${fragment})

endmacro()

macro(GENERATE_POINTER_CONVERTERS _type)
    set(oneValueArgs CONVERTER) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(GPC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    generate_includes(TYPES ${GPC_TYPES} CONVERTER "${GPC_CONVERTER}" FILE "./${GPC_CONVERTER}.i")

    set(GPC_INCLUDE
        "%fragment(\"${GPC_CONVERTER}PointerInclude\", \"header\",fragment=\"${GPC_CONVERTER}Include\")%{%}\n\n%fragment(\"${GPC_CONVERTER}PtrInclude\", \"header\",fragment=\"${GPC_CONVERTER}Include\")%{%}\n\n%fragment(\"${GPC_CONVERTER}CPtrInclude\", \"header\",fragment=\"${GPC_CONVERTER}Include\")%{%}\n\n"
    )
    typemap_tofile("./${GPC_CONVERTER}.i" TRUE ${GPC_INCLUDE})

    set(GPC_POINTOR_TYPES ${_type} "rw::core::Ptr<${_type}>")
    set(GPC_PTR_TYPES "rw::core::Ptr<${_type}>" ${_type})
    set(GPC_CPTR_TYPES "rw::core::Ptr<${_type} const>" "rw::core::Ptr<${_type}>" ${_type})
    foreach(type ${GPC_TYPES})
        if(${type} STREQUAL "int")
            set(GPC_POINTOR_TYPES ${GPC_POINTOR_TYPES} "${type}")
            set(GPC_PTR_TYPES ${GPC_PTR_TYPES} "${type}")
            set(GPC_CPTR_TYPES ${GPC_CPTR_TYPES} "${type}")
        else()
            set(GPC_POINTOR_TYPES ${GPC_POINTOR_TYPES} "rw::core::Ptr<${type}>")
            set(GPC_PTR_TYPES ${GPC_PTR_TYPES} "${type}" "rw::core::Ptr<${type}>")
            set(GPC_CPTR_TYPES ${GPC_CPTR_TYPES} "rw::core::Ptr<${type} const>"
                "rw::core::Ptr<${type}>" "${type}"
            )
        endif()
    endforeach(type)
    set(GPC_AS_POINTOR "${GPC_CONVERTER}Pointer" FILE "./${GPC_CONVERTER}.i" APPEND)
    set(GPC_AS_PTR "${GPC_CONVERTER}Ptr" FILE "./${GPC_CONVERTER}.i" APPEND)
    set(GPC_AS_CPTR "${GPC_CONVERTER}CPtr" FILE "./${GPC_CONVERTER}.i" APPEND)

    set(GPC_TYPE_POINTOR "${_type}*")
    set(GPC_TYPE_PTR "rw::core::Ptr<${_type}>")
    set(GPC_TYPE_CPTR "rw::core::Ptr<${_type} const>")

    foreach(P_TYPE PTR CPTR POINTOR)

        if(${P_TYPE} STREQUAL POINTOR)
            generate_standard_pointer_fragment(${GPC_TYPE_${P_TYPE}} CONVERTER ${GPC_AS_${P_TYPE}})
        else()
            generate_standard_ptr_fragment(${GPC_TYPE_${P_TYPE}} CONVERTER ${GPC_AS_${P_TYPE}})
        endif()

        generate_from_swig_fragment(
            ${GPC_TYPE_${P_TYPE}} TYPES ${GPC_${P_TYPE}_TYPES} CONVERTER ${GPC_AS_${P_TYPE}}
        )

        generate_python_fragment(
            ${GPC_TYPE_${P_TYPE}} TYPES ${GPC_${P_TYPE}_TYPES} CONVERTER ${GPC_AS_${P_TYPE}}
        )

        generate_typecheck(${GPC_TYPE_${P_TYPE}} TYPES ${GPC_${P_TYPE}_TYPES} CONVERTER
                           ${GPC_AS_${P_TYPE}}
        )

        generate_typemap(${GPC_TYPE_${P_TYPE}} TYPES ${GPC_${P_TYPE}_TYPES} CONVERTER
                         ${GPC_AS_${P_TYPE}}
        )
    endforeach()
endmacro()

macro(GENERATE_TYPEMAP_CHECK _type)
    set(options ADD_RWPTR_FRAGMENT ADD_POINTER_FRAGMENT) # Used to marke flags
    set(oneValueArgs FILE CONVERTER) # used to marke values with a single value
    set(multiValueArgs TYPES)

    cmake_parse_arguments(TMC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    generate_includes(TYPES ${TMC_TYPES} CONVERTER "${TMC_CONVERTER}" FILE ${TMC_FILE})

    if(TMC_ADD_RWPTR_FRAGMENT)
        generate_standard_ptr_fragment(
            "${_type}" CONVERTER "${TMC_CONVERTER}" FILE ${TMC_FILE} APPEND
        )
    elseif(TMC_ADD_POINTER_FRAGMENT)
        generate_standard_pointer_fragment(
            "${_type}" CONVERTER "${TMC_CONVERTER}" FILE ${TMC_FILE} APPEND
        )
    endif()

    generate_from_swig_fragment(
        "${_type}"
        TYPES
        ${TMC_TYPES}
        FILE
        ${TMC_FILE}
        CONVERTER
        "${TMC_CONVERTER}"
        APPEND
    )

    generate_python_fragment(
        "${_type}"
        TYPES
        ${TMC_TYPES}
        FILE
        ${TMC_FILE}
        CONVERTER
        "${TMC_CONVERTER}"
        APPEND
    )

    generate_typecheck(
        "${_type}"
        CONVERTER
        "${TMC_CONVERTER}"
        TYPES
        ${TMC_TYPES}
        FILE
        ${TMC_FILE}
        APPEND
    )

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
