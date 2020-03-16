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

    set_source_files_properties(../sdurws.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(../sdurws.i PROPERTIES SWIG_FLAGS "-includeall")
    if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        set_source_files_properties(../sdurw.i PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
    endif()
    include_directories(${RW_ROOT}/src)

    set(RWS_MODULE sdurws)
    configure_file(RobWorkStudio.egg-info.in
                   ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkStudio-${ROBWORKSTUDIO_VERSION}.egg-info)

    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkStudio-${ROBWORKSTUDIO_VERSION}.egg-info
        DESTINATION ${PYTHON_INSTALL_DIR}
    )

    if(POLICY CMP0078)
        set(LIB_SUFIX _py)
    else()
        set(LIB_SUFIX)
    endif()

    # ############ SWIG COMPILE #################
    set(SWIG_OUT_ORIG ${CMAKE_SWIG_OUTDIR})
    set(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWS_MODULE})

    if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
        swig_add_library(
            sdurws${LIB_SUFIX}
            TYPE SHARED
            LANGUAGE python
            SOURCES ../sdurws.i ../ScriptTypes.cpp
        )
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
            ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SWIG_OUTDIR}"
            LIBRARY_OUTPUT_DIRECTORY
                "${CMAKE_SWIG_OUTDIR}" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SWIG_OUTDIR}"
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
                    ${CMAKE_SWIG_OUTDIR}/${RWS_MODULE}.py
                WORKING_DIRECTORY ${SWIG_OUT_ORIG}
                COMMENT "Creating pydoc..."
            )
        endif()
    endif()

    # ################# Install files ################
    configure_file(__init__.py.in ${CMAKE_SWIG_OUTDIR}/__init__.py)

    target_link_libraries(${SUBSYS_NAME} INTERFACE ${TARGET_NAME})

    install(
            TARGETS ${TARGET_NAME} 
            EXPORT ${PROJECT_PREFIX}Targets 
            DESTINATION "${PYTHON_INSTALL_DIR}/${RWS_MODULE}" COMPONENT swig
        )
        install(
            FILES ${CMAKE_SWIG_OUTDIR}/${RWS_MODULE}.py
            DESTINATION "${PYTHON_INSTALL_DIR}/${RWS_MODULE}"
        )
        install(
            FILES ${CMAKE_SWIG_OUTDIR}/__init__.py
            DESTINATION "${PYTHON_INSTALL_DIR}/${RWS_MODULE}"
        )
    # ###### Final linking and dependency checks ######

    swig_link_libraries(sdurws${LIB_SUFIX} ${SUBSYS_DEPS} RW::sdurw ${PYTHON_LIBRARIES})

    install(
        TARGETS ${SUBSYS_NAME}
        EXPORT ${PROJECT_PREFIX}Targets
    )

endif()
