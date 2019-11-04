SET(SUBSYS_NAME sdurwsim_python )
SET(SUBSYS_DESC "Interface for accessing RobWorkSim from python." )
SET(SUBSYS_DEPS sdurw )

SET(build TRUE)

set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()
    if( NOT (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND) )
        set(DEFAULT false)
        set(REASON "PYTHONLIBS AND PYTHONINTERP not found!")
    endif()
endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
INCLUDE(UseSWIG)
    INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
    if(POLICY CMP0078)
        set(LIB_PREFIX "_")
    else()
        set(LIB_PREFIX "")
    endif()

    SET_SOURCE_FILES_PROPERTIES(../sdurwsim.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../sdurwsim.i PROPERTIES SWIG_FLAGS "-includeall")
    if (NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
    	SET_SOURCE_FILES_PROPERTIES(../sdurw.i PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
    endif()
	
    SET(CMAKE_SWIG_OUTDIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
	IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
		SWIG_ADD_LIBRARY(${LIB_PREFIX}sdurwsim TYPE SHARED LANGUAGE python SOURCES ../sdurwsim.i ../ScriptTypes.cpp)
	ELSE()
		SWIG_ADD_MODULE(${LIB_PREFIX}sdurwsim python ../sdurwsim.i ../ScriptTypes.cpp)
	ENDIF()
    if(POLICY CMP0078)
        set(TARGET_NAME ${LIB_PREFIX}sdurwsim)
    else()
        set(TARGET_NAME ${SWIG_MODULE_sdurwsim_REAL_NAME})
    endif()

    SWIG_LINK_LIBRARIES(${LIB_PREFIX}sdurwsim
        ${RWSIM_ODE_LIBRARY}
        sdurwsim
        ${ROBWORK_LIBRARIES}
        ${PYTHON_LIBRARIES} )
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined )
    ENDIF()

    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS SUBSYS_DEPS)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(${TARGET_NAME} ${DEP})
        ENDIF()
    ENDFOREACH()

    if (NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        find_program(pydoc pydoc3)
        if (NOT pydoc)
            find_program(pydoc pydoc)
        endif()
        if (pydoc)
            add_custom_command(TARGET ${TARGET_NAME}
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../../../RobWork/libs/${RW_BUILD_TYPE} ${pydoc} -w sdurwsim
                WORKING_DIRECTORY ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                COMMENT "Creating pydoc..."
            )
        endif()
    endif()
ENDIF()
