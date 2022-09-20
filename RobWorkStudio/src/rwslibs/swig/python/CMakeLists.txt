set(SUBSYS_NAME sdurws_python)
set(SUBSYS_DESC "Interface for accessing RobWorkStudio from python.")
set(SUBSYS_DEPS sdurws sdurws_robworkstudioapp RW::sdurw_python)

set(build TRUE)

set(DEFAULT TRUE)
set(REASON)
if(NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()
    if(NOT (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND))
        set(DEFAULT false)
        set(REASON "PYTHONLIBS AND PYTHONINTERP not found!")
    endif()
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    include(UseSWIG)
    include_directories(${PYTHON_INCLUDE_DIRS})

    add_library(${SUBSYS_NAME} INTERFACE)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME} ALIAS ${SUBSYS_NAME})

    include_directories(${RW_ROOT}/src)
    include_directories(${RWS_ROOT}/src)


    configure_file(RobWorkStudio.egg-info.in
                   ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkStudio-${ROBWORKSTUDIO_VERSION}.egg-info
    )

    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkStudio-${ROBWORKSTUDIO_VERSION}.egg-info
            DESTINATION ${PYTHON_INSTALL_DIR}
    )

    if(POLICY CMP0078)
        set(LIB_SUFIX _py)
    else()
        set(LIB_SUFIX)
    endif()

    set(RWS_MODULE sdurws)

    # ############ SWIG COMPILE #################
    set(TARGET_NAME ${RWS_MODULE}${LIB_SUFIX})
    rw_add_swig(
        ${RWS_MODULE} python MODULE
        TARGET_NAME ${TARGET_NAME}
        INSTALL_DIR ${PYTHON_INSTALL_DIR}/${RWS_MODULE}
        LANGUAGE_FILE_DIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWS_MODULE}
                          BINARY_OUTPUT_DIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWS_MODULE}
        SOURCES ../ScriptTypes.cpp
        SWIG_FLAGS ${SWIG_FLAGS}
    )
    if(UNIX)
        target_compile_options(${TARGET_NAME} PRIVATE -Wno-missing-field-initializers)
    endif()
    if(CMAKE_COMPILER_IS_GNUCXX)
        set_source_files_properties(../ScriptTypes.cpp COMPILE_FLAGS -fno-gnu-unique)
    endif()

    if(NOT POLICY CMP0078)
        set(TARGET_NAME ${SWIG_MODULE_sdurws${LIB_SUFIX}_REAL_NAME})
    endif()

    set(RW_USE_NUMPY ${RW_BUILD_WITH_NUMPY})
    target_compile_definitions(${TARGET_NAME} PRIVATE RW_USE_NUMPY)


    # ################# Install files ################
    configure_file(__init__.py.in ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWS_MODULE}/__init__.py)

    if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
        target_link_libraries(${SUBSYS_NAME} INTERFACE ${TARGET_NAME})
    endif()

    install(FILES ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWS_MODULE}/${RWS_MODULE}.py
            DESTINATION "${PYTHON_INSTALL_DIR}/${RWS_MODULE}"
    )
    install(FILES ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWS_MODULE}/__init__.py
            DESTINATION "${PYTHON_INSTALL_DIR}/${RWS_MODULE}"
    )
    # ###### Final linking and dependency checks ######

    target_link_libraries(${python_NAME_sdurws} PUBLIC sdurws sdurws_robworkstudioapp RW::sdurw RW::sdurw_models ${PYTHON_LIBRARIES})
    install(TARGETS ${SUBSYS_NAME} EXPORT ${PROJECT_PREFIX}pythonTargets)

    rw_swig_compile_target(python ${TARGET_NAME})

    export(
        EXPORT ${PROJECT_PREFIX}_${python_NAME_sdurws}Targets
        FILE "${${PROJECT_PREFIX}_ROOT}/cmake/targets/${RWS_MODULE}_py.cmake"
        NAMESPACE ${PROJECT_PREFIX}::
    )
    install(
        EXPORT ${PROJECT_PREFIX}_${python_NAME_sdurws}Targets
        FILE ${RWS_MODULE}_py.cmake
        NAMESPACE ${PROJECT_PREFIX}::
        DESTINATION ${RWS_INSTALL_DIR}/cmake/targets
    )

endif()
