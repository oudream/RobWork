SET(SUBSYS_NAME sdurwsim_lua )
SET(SUBSYS_DESC "Interface for accessing RobWorkSim from lua." )
SET(SUBSYS_DEPS sdurw )

SET(build TRUE)

FIND_PACKAGE(SWIG 3.0.0 QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   SET(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
	#MESSAGE(STATUS "SWIG found adding swig modules!")
	INCLUDE(UseSWIG)
	
	SET(CMAKE_SWIG_FLAGS "")

	SET_SOURCE_FILES_PROPERTIES(../sdurwsim.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../sdurwsim.i PROPERTIES SWIG_FLAGS "-includeall")
	
	# lua interface generation
	IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
		SWIG_ADD_LIBRARY(sdurwsim_lua TYPE MODULE LANGUAGE lua SOURCES ../sdurwsim.i ../ScriptTypes.cpp Lua.cpp)
	ELSE()
		SWIG_ADD_MODULE(sdurwsim_lua lua ../sdurwsim.i ../ScriptTypes.cpp Lua.cpp)
	ENDIF()
	SWIG_LINK_LIBRARIES(sdurwsim_lua ${RWSIM_ODE_LIBRARY} sdurwsim ${ROBWORK_LIBRARIES})
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(sdurwsim_lua PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()

    IF ((CMAKE_VERSION VERSION_GREATER 3.12.0) OR (CMAKE_VERSION VERSION_EQUAL 3.12.0))
        SWIG_ADD_LIBRARY(sdurwsim_lua_s TYPE STATIC LANGUAGE lua SOURCES ../sdurwsim.i ../ScriptTypes.cpp Lua.cpp)
    ELSE()
        ADD_LIBRARY(sdurwsim_lua_s STATIC Lua.cpp ${swig_generated_sources} ${swig_other_sources})
        ADD_DEPENDENCIES(sdurwsim_lua_s sdurwsim_lua) # avoid using the source files before they have been generated
    ENDIF()
	TARGET_LINK_LIBRARIES(sdurwsim_lua_s ${RWSIM_ODE_LIBRARY} sdurwsim ${ROBWORK_LIBRARIES})
	# Make sure dependencies are build before system
    FOREACH(DEP IN LISTS SUBSYS_DEPS)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(sdurwsim_lua_s ${DEP})
        ENDIF()
    ENDFOREACH()

	# this is used to indicate static linking to Visual Studio or mingw
	IF (DEFINED MSVC) 
	    SET_TARGET_PROPERTIES(sdurwsim_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
	ELSE()
	    SET_TARGET_PROPERTIES(sdurwsim_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
	ENDIF()
	
	ADD_LIBRARY(sdurwsimlua_plugin.rwplugin MODULE LuaPlugin.cpp LuaPlugin.hpp)
    TARGET_LINK_LIBRARIES(sdurwsimlua_plugin.rwplugin sdurwsim_lua_s ${ROBWORK_LIBRARIES})
	
	# Make sure dependencies are build before system
    FOREACH(DEP IN LISTS sdurwsim_lua_s ROBWORK_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(sdurwsimlua_plugin.rwplugin ${DEP})
        ENDIF()
	ENDFOREACH()
	
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(sdurwsimlua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()
	
	
ENDIF()