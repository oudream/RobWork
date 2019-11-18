set(SUBSYS_NAME sdurws_py)
set(SUBSYS_DESC "Interface for accessing RobWorkStudio from python.")
set(SUBSYS_DEPS sdurw)

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

rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
rw_add_doc(${SUBSYS_NAME})

if(build)
    include(UseSWIG)
    include_directories(${PYTHON_INCLUDE_DIRS})

    set_source_files_properties(../sdurws.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(../sdurws.i PROPERTIES SWIG_FLAGS "-includeall")
    if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        set_source_files_properties(../sdurw.i PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
    endif()

    execute_process(
        COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(RWS_MODULE sdurws)
    configure_file(RobWorkStudio.egg-info.in
                   ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkStudio-${ROBWORKSTUDIO_VERSION}.egg-info)
    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkStudio-${ROBWORKSTUDIO_VERSION}.egg-info
        DESTINATION ${PYTHON_SITE_PACKAGES}
    )

    if(POLICY CMP0078)
        set(LIB_SUFIX _py)
    else()
        set(LIB_SUFIX)
    endif()

    # ############ SWIG COMPILE #################
    set(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWS_MODULE})

    if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
        swig_add_library(sdurws${LIB_SUFIX} TYPE SHARED LANGUAGE python SOURCES ../sdurws.i ../ScriptTypes.cpp)
    else()
        swig_add_module(sdurws${LIB_SUFIX} python ../sdurws.i ../ScriptTypes.cpp)
    endif()

    if(POLICY CMP0078)
        set(TARGET_NAME sdurws${LIB_SUFIX})
    else()
        set(TARGET_NAME ${SWIG_MODULE_sdurws${LIB_SUFIX}_REAL_NAME})
    endif()

    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()

    # Set build destination
    set_target_properties(
        ${TARGET_NAME}
        PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY
            "${CMAKE_SWIG_OUTDIR}"
            LIBRARY_OUTPUT_DIRECTORY
            "${CMAKE_SWIG_OUTDIR}"
            RUNTIME_OUTPUT_DIRECTORY
            "${CMAKE_SWIG_OUTDIR}"
    )

    # ############# Documentation ##############
    if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        find_program(pydoc pydoc3)
        if(NOT pydoc)
            find_program(pydoc pydoc)
        endif()
        if(pydoc)
            add_custom_command(
                TARGET ${TARGET_NAME} POST_BUILD
                COMMAND
                    ${CMAKE_COMMAND}
                    -E
                    env
                    PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../../../RobWork/libs/${RW_BUILD_TYPE}
                    ${pydoc}
                    -w
                    sdurws
                WORKING_DIRECTORY ${CMAKE_SWIG_OUTDIR}
                COMMENT "Creating pydoc..."
            )
        endif()
    endif()

    # ################# Install files ################
    configure_file(__init__.py.in ${CMAKE_SWIG_OUTDIR}/__init__.py)
    if(PYTHON_SITE_PACKAGES MATCHES "^.?usr")
        install(TARGETS ${TARGET_NAME} DESTINATION "${PYTHON_SITE_PACKAGES}/${RWS_MODULE}")
        install(FILES ${CMAKE_SWIG_OUTDIR}/${RWS_MODULE}.py DESTINATION "${PYTHON_SITE_PACKAGES}/${RWS_MODULE}")
        install(FILES ${CMAKE_SWIG_OUTDIR}/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES}/${RWS_MODULE}")
    endif()
    # ###### Final linking and dependency checks ######

    swig_link_libraries(
        sdurws${LIB_SUFIX}
        ${RWS_COMPONENT_LIBRARIES}
        ${RWS_PLUGIN_LIBRARIES}
        sdurws
        ${ROBWORK_LIBRARIES}
        ${PYTHON_LIBRARIES}
    )

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS SUBSYS_DEPS)
        if(TARGET ${DEP})
            add_dependencies(${TARGET_NAME} ${DEP})
        endif()
    endforeach()

endif()
