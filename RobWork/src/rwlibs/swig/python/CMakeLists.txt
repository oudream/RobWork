set(SUBSYS_NAME sdurw_python)
set(SUBSYS_DESC "Interface for accessing RobWork from python.")
set(SUBSYS_DEPS sdurw sdurw_algorithms)

set(build TRUE)

find_package(
    SWIG
    3.0.0
    QUIET
)

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
    build
    ${SUBSYS_NAME}
    ${SUBSYS_DESC}
    ${DEFAULT}
    ${REASON}
)
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
    set(CMAKE_SWIG_OUTDIR ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

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
        set(RW_MODULE_FILENAME ../${RW_MODULE}.i)

        set_source_files_properties(
            ${RW_MODULE_FILENAME}
            PROPERTIES
            CPLUSPLUS
            ON
        )
        set_source_files_properties(
            ${RW_MODULE_FILENAME}
            PROPERTIES
            SWIG_FLAGS
            "${SWIG_FLAGS}"
        )

        if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
            swig_add_library( ${RW_MODULE}_py TYPE SHARED LANGUAGE python SOURCES ${RW_MODULE_FILENAME} ../ScriptTypes.cpp)
        else()
            swig_add_module( ${RW_MODULE}_py python ${RW_MODULE_FILENAME} ../ScriptTypes.cpp)
        endif()

        if(POLICY CMP0078)
            set(TARGET_NAME ${RW_MODULE}_py)
        else()
            set(TARGET_NAME ${SWIG_MODULE_${RW_MODULE}_py_REAL_NAME})
        endif()

        if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
            set_target_properties(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
        endif()
        add_dependencies(${SUBSYS_NAME} ${TARGET_NAME})

        if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
            find_program(pydoc pydoc3)
            if(NOT pydoc)
                find_program(pydoc pydoc)
            endif()
            if(pydoc)
                add_custom_command(
                    TARGET
                        ${TARGET_NAME}
                    POST_BUILD
                    COMMAND
                        ${pydoc}
                        -w
                        ${RW_MODULE}
                    WORKING_DIRECTORY ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                    COMMENT "Creating pydoc..."
                )
            endif()
        endif()
    endforeach()

    swig_link_libraries(sdurw_py                        sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_assembly_py               sdurw_assembly sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_control_py                sdurw_control sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_pathoptimization_py       sdurw_pathoptimization sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_pathplanners_py           sdurw_pathplanners sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_proximitystrategies_py    sdurw_proximitystrategies sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_simulation_py             sdurw_simulation sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_task_py                   sdurw_task sdurw ${PYTHON_LIBRARIES})
    swig_link_libraries(sdurw_opengl_py                 sdurw_opengl sdurw ${PYTHON_LIBRARIES})

    if(POLICY CMP0078)
        set(LIB_PREFIX "")
    else()
        set(LIB_PREFIX "_")
    endif()

    add_dependencies(${LIB_PREFIX}sdurw_assembly_py             ${LIB_PREFIX}sdurw_task_py ${LIB_PREFIX}sdurw_py )
    add_dependencies(${LIB_PREFIX}sdurw_control_py              ${LIB_PREFIX}sdurw_py)
    add_dependencies(${LIB_PREFIX}sdurw_pathoptimization_py     ${LIB_PREFIX}sdurw_py)
    add_dependencies(${LIB_PREFIX}sdurw_pathplanners_py         ${LIB_PREFIX}sdurw_py)
    add_dependencies(${LIB_PREFIX}sdurw_proximitystrategies_py  ${LIB_PREFIX}sdurw_py)
    add_dependencies(${LIB_PREFIX}sdurw_simulation_py           ${LIB_PREFIX}sdurw_py)
    add_dependencies(${LIB_PREFIX}sdurw_task_py                 ${LIB_PREFIX}sdurw_py)
    add_dependencies(${LIB_PREFIX}sdurw_opengl_py               ${LIB_PREFIX}sdurw_py)
endif()
