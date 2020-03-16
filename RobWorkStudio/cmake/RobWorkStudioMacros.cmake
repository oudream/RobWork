# ######################################################################################################################
# Add a library target. _name The library name. _component The part of RW that this library belongs to. ARGN The source
# files for the library.
macro(RWS_ADD_PLUGIN _name _lib_type)
    add_library(${_name} ${_lib_type} ${ARGN})

    # Only link if needed
    if(WIN32 AND MSVC)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed,--no-undefined)
    endif()

    if(NOT ("${_lib_type}" STREQUAL "MODULE" AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
        string(REGEX MATCHALL "[0-9]+" VERSIONS ${ROBWORKSTUDIO_VERSION})
        list(GET VERSIONS 0 ROBWORKSTUDIO_VERSION_MAJOR)
        list(GET VERSIONS 1 ROBWORKSTUDIO_VERSION_MINOR)
        list(GET VERSIONS 2 ROBWORKSTUDIO_VERSION_PATCH)

        set_target_properties(
            ${_name}
            PROPERTIES
                VERSION ${ROBWORKSTUDIO_VERSION} SOVERSION ${ROBWORKSTUDIO_VERSION_MAJOR}.${ROBWORKSTUDIO_VERSION_MINOR}
                # DEFINE_SYMBOL "RWAPI_EXPORTS"
        )
    endif(NOT ("${_lib_type}" STREQUAL "MODULE" AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))

    install(
        TARGETS ${_name}
        EXPORT ${PROJECT_PREFIX}Targets
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION "${LIB_INSTALL_DIR}/RobWork/rwsplugins" COMPONENT ${_component}
        ARCHIVE DESTINATION "${LIB_INSTALL_DIR}/RobWork/rwsplugins" COMPONENT ${_component}
    )

endmacro()

# ######################################################################################################################
# Add a library target. _name The library name. _component The part of RW that this library belongs to. ARGN The source
# files for the library.
macro(RWS_ADD_COMPONENT _name)
    add_library(${_name} ${PROJECT_LIB_TYPE} ${ARGN})

    # Only link if needed
    if(WIN32 AND MSVC)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed,--no-undefined)
    endif()
    #
    string(REGEX MATCHALL "[0-9]+" VERSIONS ${ROBWORKSTUDIO_VERSION})
    list(GET VERSIONS 0 PROJECT_VERSION_MAJOR)
    list(GET VERSIONS 1 PROJECT_VERSION_MINOR)
    list(GET VERSIONS 2 PROJECT_VERSION_PATCH)

    if(${PROJECT_USE_SONAME})
        set_target_properties(
            ${_name}
            PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()
    # if(USE_PROJECT_FOLDERS) set_target_properties(${_name} PROPERTIES FOLDER "Libraries") endif(USE_PROJECT_FOLDERS)

    install(
        TARGETS ${_name}
        EXPORT ${PROJECT_PREFIX}Targets
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_name}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_name}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_name}
    )

endmacro()
