set(SUBSYS_NAME sdurw_python)
set(SUBSYS_DESC "Interface for accessing RobWork from python.")
set(SUBSYS_DEPS sdurw sdurw_algorithms)

set(build TRUE)

find_package(SWIG 3.0.0 QUIET)

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
    set(SWIG_FLAGS "-includeall")
    if((PYTHON_VERSION_MAJOR VERSION_GREATER 3) OR (PYTHON_VERSION_MAJOR VERSION_EQUAL 3))
        set(SWIG_FLAGS "${SWIG_FLAGS};-py3")
    endif()
    if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        set(SWIG_FLAGS "${SWIG_FLAGS};-doxygen")
    endif()

    execute_process(
        COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    configure_file(setup.py.in ${CMAKE_CURRENT_SOURCE_DIR}/setup.py)
    configure_file(RobWork.egg-info.in ${CMAKE_CURRENT_SOURCE_DIR}/RobWork-${ROBWORK_VERSION}.egg-info)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobWork-${ROBWORK_VERSION}.egg-info DESTINATION ${PYTHON_SITE_PACKAGES})

    if(POLICY CMP0078)
        set(LIB_SUFIX _py)
    else()
        set(LIB_SUFIX)
    endif()

    add_custom_target(${SUBSYS_NAME})
    foreach(
        RW_MODULE
        sdurw
        sdurw_assembly
        sdurw_control
        sdurw_pathoptimization
        sdurw_pathplanners
        sdurw_proximitystrategies
        sdurw_simulation
        sdurw_task
        sdurw_opengl
    )
        set(CMAKE_SWIG_OUTDIR ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE})
        set(RW_MODULE_FILENAME ../${RW_MODULE}.i)

        set_source_files_properties(${RW_MODULE_FILENAME} PROPERTIES CPLUSPLUS ON)
        set_source_files_properties(${RW_MODULE_FILENAME} PROPERTIES SWIG_FLAGS "${SWIG_FLAGS}")

        if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
            swig_add_library(
                ${RW_MODULE}${LIB_SUFIX}
                TYPE SHARED
                LANGUAGE python
                SOURCES ${RW_MODULE_FILENAME} ../ScriptTypes.cpp
            )
        else()
            swig_add_module(${RW_MODULE}${LIB_SUFIX} python ${RW_MODULE_FILENAME} ../ScriptTypes.cpp)
        endif()

        if(POLICY CMP0078)
            set(TARGET_NAME ${RW_MODULE}${LIB_SUFIX})
        else()
            set(TARGET_NAME ${SWIG_MODULE_${RW_MODULE}${LIB_SUFIX}_REAL_NAME})
        endif()

        if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
            set_target_properties(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
        endif()

        #Set build destination
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
        add_dependencies(${SUBSYS_NAME} ${TARGET_NAME})

        if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
            find_program(pydoc pydoc3)
            if(NOT pydoc)
                find_program(pydoc pydoc)
            endif()
            if(pydoc)
                add_custom_command(
                    TARGET ${TARGET_NAME} POST_BUILD
                    COMMAND ${pydoc} -w ${RW_MODULE}
                    WORKING_DIRECTORY ${CMAKE_SWIG_OUTDIR}
                    COMMENT "Creating pydoc..."
                )
            endif()
        endif()

        configure_file(__init__.py.in ${CMAKE_SWIG_OUTDIR}/__init__.py)
        message(STATUS "PYTHON DEST: '${PYTHON_SITE_PACKAGES}'")
        if( PYTHON_SITE_PACKAGES MATCHES "^.?usr" )
            message(STATUS " - MATCH")
        endif()
        if(NOT "${PYTHON_SITE_PACKAGES}" STREQUAL "" OR NOT DEFINED PYTHON_SITE_PACKAGES )
            install(TARGETS ${TARGET_NAME} DESTINATION "${PYTHON_SITE_PACKAGES}/${RW_MODULE}")
            install(FILES ${CMAKE_SWIG_OUTDIR}/${RW_MODULE}.py DESTINATION "${PYTHON_SITE_PACKAGES}/${RW_MODULE}")
            install(FILES ${CMAKE_SWIG_OUTDIR}/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES}/${RW_MODULE}")
        endif()
    endforeach()

    swig_link_libraries(sdurw${LIB_SUFIX} sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_assembly${LIB_SUFIX} sdurw_assembly sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_control${LIB_SUFIX} sdurw_control sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_pathoptimization${LIB_SUFIX} sdurw_pathoptimization sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_pathplanners${LIB_SUFIX} sdurw_pathplanners sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_proximitystrategies${LIB_SUFIX} sdurw_proximitystrategies sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_simulation${LIB_SUFIX} sdurw_simulation sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_task${LIB_SUFIX} sdurw_task sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_opengl${LIB_SUFIX} sdurw_opengl sdurw ${PYTHON_LIBRARIES})

    if(POLICY CMP0078)
        set(LIB_PREFIX "")
    else()
        set(LIB_PREFIX "_")
    endif()

    add_dependencies(${LIB_PREFIX}sdurw_assembly${LIB_SUFIX} ${LIB_PREFIX}sdurw_task${LIB_SUFIX}
                     ${LIB_PREFIX}sdurw${LIB_SUFIX})
    add_dependencies(${LIB_PREFIX}sdurw_control${LIB_SUFIX} ${LIB_PREFIX}sdurw${LIB_SUFIX})
    add_dependencies(${LIB_PREFIX}sdurw_pathoptimization${LIB_SUFIX} ${LIB_PREFIX}sdurw${LIB_SUFIX})
    add_dependencies(${LIB_PREFIX}sdurw_pathplanners${LIB_SUFIX} ${LIB_PREFIX}sdurw${LIB_SUFIX})
    add_dependencies(${LIB_PREFIX}sdurw_proximitystrategies${LIB_SUFIX} ${LIB_PREFIX}sdurw${LIB_SUFIX})
    add_dependencies(${LIB_PREFIX}sdurw_simulation${LIB_SUFIX} ${LIB_PREFIX}sdurw${LIB_SUFIX})
    add_dependencies(${LIB_PREFIX}sdurw_task${LIB_SUFIX} ${LIB_PREFIX}sdurw${LIB_SUFIX})
    add_dependencies(${LIB_PREFIX}sdurw_opengl${LIB_SUFIX} ${LIB_PREFIX}sdurw${LIB_SUFIX})
endif()
