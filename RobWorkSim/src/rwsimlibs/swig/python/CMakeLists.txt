set(SUBSYS_NAME sdurwsim_python)
set(SUBSYS_DESC "Interface for accessing RobWorkSim from python.")
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

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    include(UseSWIG)
    include_directories(${PYTHON_INCLUDE_DIRS})

    set_source_files_properties(../sdurwsim.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(../sdurwsim.i PROPERTIES SWIG_FLAGS "-includeall")
    if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        set_source_files_properties(../sdurw.i PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
    endif()

    execute_process(
        COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(RWSIM_MODULE sdurwsim)
    configure_file(RobWorkSim.egg-info.in
                   ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkSim-${ROBWORKSIM_VERSION}.egg-info)

    if(NOT "${PYTHON_SITE_PACKAGES}" STREQUAL "")
        install(
            FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkSim-${ROBWORKSIM_VERSION}.egg-info
            DESTINATION ${PYTHON_SITE_PACKAGES}
        )
    endif()

    if(POLICY CMP0078)
        set(LIB_SUFIX _py)
    else()
        set(LIB_SUFIX)
    endif()

    # ############ SWIG COMPILE #################
    set(CMAKE_SWIG_OUTDIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWSIM_MODULE})

    if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
        swig_add_library(
            sdurwsim${LIB_SUFIX}
            TYPE SHARED
            LANGUAGE python
            SOURCES ../sdurwsim.i ../ScriptTypes.cpp
        )
    else()
        swig_add_module(sdurwsim${LIB_SUFIX} python ../sdurwsim.i ../ScriptTypes.cpp)
    endif()

    if(POLICY CMP0078)
        set(TARGET_NAME sdurwsim${LIB_SUFIX})
    else()
        set(TARGET_NAME ${SWIG_MODULE_sdurwsim${LIB_SUFIX}_REAL_NAME})
    endif()

    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()

    # Set build destination
    set_target_properties(
        ${TARGET_NAME}
        PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SWIG_OUTDIR}"
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SWIG_OUTDIR}" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SWIG_OUTDIR}"
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
                    sdurwsim
                WORKING_DIRECTORY ${CMAKE_SWIG_OUTDIR}
                COMMENT "Creating pydoc..."
            )
        endif()
    endif()

    # ################# Install files ################
    configure_file(__init__.py.in ${CMAKE_SWIG_OUTDIR}/__init__.py)
    if(PYTHON_SITE_PACKAGES MATCHES "^.?usr")
        install(TARGETS ${TARGET_NAME} DESTINATION "${PYTHON_SITE_PACKAGES}/${RWSIM_MODULE}")
        install(
            FILES ${CMAKE_SWIG_OUTDIR}/${RWSIM_MODULE}.py
            DESTINATION "${PYTHON_SITE_PACKAGES}/${RWSIM_MODULE}"
        )
        install(FILES ${CMAKE_SWIG_OUTDIR}/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES}/${RWSIM_MODULE}")
    endif()
    # ###### Final linking and dependency checks ######

    swig_link_libraries(
        sdurwsim${LIB_SUFIX}
        ${RWSIM_ODE_LIBRARY}
        sdurwsim
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
