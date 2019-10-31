SET(SUBSYS_NAME sdurws_lua )
SET(SUBSYS_DESC "Interface for accessing RobWorkStudio from lua." )
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

	SET_SOURCE_FILES_PROPERTIES(../sdurws.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../sdurws.i PROPERTIES SWIG_FLAGS "-includeall")
	
	# lua interface generation
	IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
		SWIG_ADD_LIBRARY(sdurws_lua TYPE SHARED LANGUAGE lua SOURCES ../sdurws.i Lua.cpp ../ScriptTypes.cpp)
	ELSE()
		SWIG_ADD_MODULE(sdurws_lua lua ../sdurws.i Lua.cpp ../ScriptTypes.cpp)
	ENDIF()
    SWIG_LINK_LIBRARIES(sdurws_lua
                ${RWS_COMPONENT_LIBRARIES}
                ${RWS_PLUGIN_LIBRARIES}
                sdurws
                ${ROBWORK_LIBRARIES}
    )
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(sdurws_lua PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()

    IF ((CMAKE_VERSION VERSION_GREATER 3.12.0) OR (CMAKE_VERSION VERSION_EQUAL 3.12.0))
        SWIG_ADD_LIBRARY(sdurws_lua_s TYPE STATIC LANGUAGE lua SOURCES ../sdurws.i Lua.cpp ../ScriptTypes.cpp)
    ELSE()
        ADD_LIBRARY(sdurws_lua_s STATIC ${swig_generated_sources} ${swig_other_sources})
    ENDIF()
    TARGET_LINK_LIBRARIES(sdurws_lua_s
                ${RWS_COMPONENT_LIBRARIES}
                ${RWS_PLUGIN_LIBRARIES}
                sdurws
                ${ROBWORK_LIBRARIES}
    )

    set(RWS_COMPONENT_LIBRARIES ${RWS_COMPONENT_LIBRARIES} sdurws_lua_s PARENT_SCOPE)
    #MESSAGE("${ROBWORKSTUDIO_LIBRARIES} \n ROBWORK ${ROBWORK_LIBRARIES}" )
    # this is used to indicate static linking to Visual Studio or mingw 
    	# this is used to indicate static linking to Visual Studio or mingw
	IF (DEFINED MSVC) 
	    SET_TARGET_PROPERTIES(sdurws_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
	ELSE()
	    SET_TARGET_PROPERTIES(sdurws_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
	ENDIF()
    	
    	
	ADD_LIBRARY(sdurwslua_plugin.rwplugin MODULE LuaPlugin.cpp LuaPlugin.hpp)
    TARGET_LINK_LIBRARIES(sdurwslua_plugin.rwplugin sdurws_lua_s ${ROBWORK_LIBRARIES})
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(sdurwslua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()
    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS SUBSYS_DEPS)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(${SUBSYS_NAME} ${DEP})
        ENDIF()
    ENDFOREACH()
	
	
ENDIF()
