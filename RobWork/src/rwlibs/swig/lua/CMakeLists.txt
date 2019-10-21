SET(SUBSYS_NAME sdurw_lua )
SET(SUBSYS_DESC "Interface for accessing RobWork from lua." )
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

    add_custom_target(${SUBSYS_NAME}_all)
	SET(RW_NAME sdurw)
	foreach(RW_MODULE ${RW_NAME} ${RW_NAME}_assembly ${RW_NAME}_control ${RW_NAME}_pathoptimization ${RW_NAME}_pathplanners ${RW_NAME}_proximitystrategies ${RW_NAME}_task ${RW_NAME}_simulation)

		SET(RW_MODULE_FILENAME ../${RW_MODULE}.i)

		SET_SOURCE_FILES_PROPERTIES(${RW_MODULE_FILENAME} PROPERTIES CPLUSPLUS ON)
		SET_SOURCE_FILES_PROPERTIES(${RW_MODULE_FILENAME} PROPERTIES SWIG_FLAGS "-includeall")
		
		configure_file(Lua.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/Lua_${RW_MODULE}.hpp @ONLY)
		configure_file(Lua.cpp.in Lua_${RW_MODULE}.cpp @ONLY)
		configure_file(LuaPlugin.cpp.in LuaPlugin_${RW_MODULE}.cpp @ONLY)
		
	    set(SOURCE)
	    if ("${RW_MODULE}" STREQUAL "sdurw")
	        set(SOURCE ../ScriptTypes.cpp LuaState.cpp)
	    endif()
	    set(SOURCE ${SOURCE} Lua_${RW_MODULE}.cpp)

		# lua interface generation
		IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
			SWIG_ADD_LIBRARY(${RW_MODULE}_lua TYPE SHARED LANGUAGE lua SOURCES ${RW_MODULE_FILENAME} ${SOURCE})
		ELSE()
			SWIG_ADD_MODULE(${RW_MODULE}_lua lua ${RW_MODULE_FILENAME} ${SOURCE})
		ENDIF()
        target_include_directories(${RW_MODULE}_lua PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        add_dependencies(${SUBSYS_NAME}_all ${RW_MODULE}_lua)

	    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
	      SET_TARGET_PROPERTIES(${RW_MODULE}_lua PROPERTIES LINK_FLAGS -Wl,--no-undefined)
	    ENDIF()
	
	    IF ((CMAKE_VERSION VERSION_GREATER 3.12.0) OR (CMAKE_VERSION VERSION_EQUAL 3.12.0))
	        SWIG_ADD_LIBRARY(${RW_MODULE}_lua_s TYPE STATIC LANGUAGE lua SOURCES ${RW_MODULE_FILENAME} ${SOURCE})
	    ELSE()
	        ADD_LIBRARY(${RW_MODULE}_lua_s STATIC ${SOURCE} ${swig_generated_sources} ${swig_other_sources})
	        ADD_DEPENDENCIES(${RW_MODULE}_lua_s sdurw_lua) # avoid using the source files before they have been generated
	    ENDIF()
        target_include_directories(${RW_MODULE}_lua_s PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        add_dependencies(${SUBSYS_NAME}_all ${RW_MODULE}_lua_s)

		# this is used to indicate static linking to Visual Studio or mingw
		IF (DEFINED MSVC) 
		    SET_TARGET_PROPERTIES(${RW_MODULE}_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
		ELSE()
		    SET_TARGET_PROPERTIES(${RW_MODULE}_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
		ENDIF()

        # Add rwplugin
        if (NOT "${RW_MODULE}" STREQUAL "sdurw")
	        add_library(${RW_MODULE}_lua_plugin.rwplugin MODULE LuaPlugin_${RW_MODULE}.cpp)
	        target_link_libraries(${RW_MODULE}_lua_plugin.rwplugin ${RW_MODULE}_lua_s)
	        target_include_directories(${RW_MODULE}_lua_plugin.rwplugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
	        if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
	            set_target_properties(${RW_MODULE}_lua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
	        endif()
	        add_dependencies(${SUBSYS_NAME}_all ${RW_MODULE}_lua_plugin.rwplugin)
        endif()

	    install(TARGETS ${RW_MODULE}_lua ${RW_MODULE}_lua_s 
	        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT swig
	        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig
	        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig)
    endforeach()

    SWIG_LINK_LIBRARIES(${RW_NAME}_lua sdurw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(${RW_NAME}_assembly_lua sdurw_assembly sdurw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(${RW_NAME}_control_lua sdurw_control sdurw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(${RW_NAME}_pathoptimization_lua sdurw_pathoptimization sdurw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(${RW_NAME}_pathplanners_lua sdurw_pathplanners sdurw ${LUA_LIBRARIES})
    SWIG_LINK_LIBRARIES(${RW_NAME}_proximitystrategies_lua sdurw_proximitystrategies sdurw ${LUA_LIBRARIES})
	SWIG_LINK_LIBRARIES(${RW_NAME}_simulation_lua sdurw_simulation sdurw ${LUA_LIBRARIES})
	SWIG_LINK_LIBRARIES(${RW_NAME}_task_lua sdurw_task sdurw ${LUA_LIBRARIES})
	target_include_directories(${RW_NAME}_lua PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    TARGET_LINK_LIBRARIES(${RW_NAME}_lua_s sdurw ${LUA_LIBRARIES})
    TARGET_LINK_LIBRARIES(${RW_NAME}_assembly_lua_s sdurw_assembly ${RW_NAME}_lua_s ${LUA_LIBRARIES})
    TARGET_LINK_LIBRARIES(${RW_NAME}_control_lua_s sdurw_control ${RW_NAME}_lua_s)
    TARGET_LINK_LIBRARIES(${RW_NAME}_pathoptimization_lua_s sdurw_pathoptimization ${RW_NAME}_lua_s ${LUA_LIBRARIES})
    TARGET_LINK_LIBRARIES(${RW_NAME}_pathplanners_lua_s sdurw_pathplanners ${RW_NAME}_lua_s ${LUA_LIBRARIES})
    TARGET_LINK_LIBRARIES(${RW_NAME}_proximitystrategies_lua_s sdurw_proximitystrategies ${RW_NAME}_lua_s ${LUA_LIBRARIES})
	TARGET_LINK_LIBRARIES(${RW_NAME}_simulation_lua_s sdurw_simulation ${RW_NAME}_lua_s ${LUA_LIBRARIES})
	TARGET_LINK_LIBRARIES(${RW_NAME}_task_lua_s sdurw_task ${RW_NAME}_lua_s ${LUA_LIBRARIES})
	target_include_directories(${RW_NAME}_lua_s PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

ENDIF()
