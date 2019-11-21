# ######################################################################################################################
# Add a library target. _name The library name. _component The part of RW that this library belongs to. ARGN The source
# files for the library.
macro(RWS_ADD_PLUGIN _name _component _lib_type)
    add_library(${_name} ${_lib_type} ${ARGN})
    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})
    if(NOT RWS_USE_STATIC_LINK_PLUGINS)
        target_link_libraries(${_name} sdurws qtpropertybrowser ${ROBWORK_LIBRARIES} ${QT_LIBRARIES})
    endif()

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

    # The library to build: IF (RWS_USE_STATIC_LINK_PLUGINS) ADD_LIBRARY(${TargetName} STATIC ${SrcFiles} ${MocSrcFiles}
    # ${RccSrcFiles}) INSTALL(TARGETS ${TargetName} DESTINATION ${LIB_INSTALL_DIR}) ELSE ()
    # SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}) ADD_LIBRARY(${TargetName} SHARED ${SrcFiles}
    # ${MocSrcFiles} ${RccSrcFiles}) SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${RWS_LIBRARY_OUT_DIR}) # Link the standard
    # static libraries with with the shared library: TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES})
    # INSTALL(TARGETS ${TargetName} DESTINATION ${BIN_INSTALL_DIR}) ENDIF () Set the VERSION and SOVERSION of the
    # library to the RobWorkStudio major and minor versions On MAC OS we can not do this if we are building a Module
    # (where it does not make much sense anyway)
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
    # if(USE_PROJECT_FOLDERS) set_target_properties(${_name} PROPERTIES FOLDER "Libraries") endif(USE_PROJECT_FOLDERS)

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

    # The library to build: IF (RWS_USE_STATIC_LINK_PLUGINS) ADD_LIBRARY(${TargetName} STATIC ${SrcFiles} ${MocSrcFiles}
    # ${RccSrcFiles}) INSTALL(TARGETS ${TargetName} DESTINATION ${LIB_INSTALL_DIR}) ELSE ()
    # SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}) ADD_LIBRARY(${TargetName} SHARED ${SrcFiles}
    # ${MocSrcFiles} ${RccSrcFiles}) SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${RWS_LIBRARY_OUT_DIR}) # Link the standard
    # static libraries with with the shared library: TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES})
    # INSTALL(TARGETS ${TargetName} DESTINATION ${BIN_INSTALL_DIR}) ENDIF () Set the VERSION and SOVERSION of the
    # library to the RobWorkStudio major and minor versions On MAC OS we can not do this if we are building a Module
    # (where it does not make much sense anyway)
    if(NOT ("${_lib_type}" STREQUAL "MODULE" AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
        string(REGEX MATCHALL "[0-9]+" VERSIONS ${ROBWORKSTUDIO_VERSION})
        list(GET VERSIONS 0 ROBWORKSTUDIO_VERSION_MAJOR)
        list(GET VERSIONS 1 ROBWORKSTUDIO_VERSION_MINOR)
        list(GET VERSIONS 2 ROBWORKSTUDIO_VERSION_PATCH)

        if(${PROJECT_USE_SONAME})
            set_target_properties(
                ${_name}
                PROPERTIES
                    VERSION ${ROBWORKSTUDIO_VERSION} SOVERSION
                    ${ROBWORKSTUDIO_VERSION_MAJOR}.${ROBWORKSTUDIO_VERSION_MINOR}
                    # DEFINE_SYMBOL "RWAPI_EXPORTS"
            )
        endif()
    endif()
    # if(USE_PROJECT_FOLDERS) set_target_properties(${_name} PROPERTIES FOLDER "Libraries") endif(USE_PROJECT_FOLDERS)

    install(
        TARGETS ${_name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION "${LIB_INSTALL_DIR}/RobWork/rwsplugins" COMPONENT ${_component}
        ARCHIVE DESTINATION "${LIB_INSTALL_DIR}/RobWork/rwsplugins" COMPONENT ${_component}
    )

endmacro()

# ######################################################################################################################
# Add a library target. _name The library name. _component The part of RW that this library belongs to. ARGN The source
# files for the library.
macro(RWS_ADD_COMPONENT _name _component)
    add_library(${_name} ${PROJECT_LIB_TYPE} ${ARGN})
    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})
    if(NOT RWS_USE_STATIC_LINK_PLUGINS)
        target_link_libraries(${_name} sdurws qtpropertybrowser ${ROBWORK_LIBRARIES} ${QT_LIBRARIES})
    endif()

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
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
    )

endmacro()
