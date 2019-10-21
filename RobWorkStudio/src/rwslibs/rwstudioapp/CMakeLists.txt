SET(SUBSYS_NAME sdurws_robworkstudioapp)
set(SUBSYS_DESC "A rwstudio application that may be started in a thread.")
SET(SUBSYS_DEPS sdurws)

IF( RWS_USE_STATIC_LINK_PLUGINS )
    LIST(APPEND SUBSYS_DEPS sdurws_jog sdurws_log sdurws_planning sdurws_playback sdurws_sensors sdurws_treeview sdurws_propertyview)
	IF ( RWS_HAVE_LUA )
    	LIST(APPEND SUBSYS_DEPS sdurws_luaeditor sdurws_luapl)
    ENDIF() 
ENDIF()

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # Standard cpp files to compile:
    SET(SrcFiles RobWorkStudioApp.cpp)
    SET(SRC_FILES_HPP RobWorkStudioApp.hpp)
    
    RWS_ADD_COMPONENT(${SUBSYS_NAME} plugins ${SrcFiles})
    RW_ADD_INCLUDES(plugins "rwslibs/rwstudioapp" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS SUBSYS_DEPS)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(${SUBSYS_NAME} ${DEP})
        ENDIF()
    ENDFOREACH()

    
    set(RWS_COMPONENT_LIBRARIES ${RWS_COMPONENT_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()

	IF(WIN32 AND MSVC AND PROJECT_LIB_TYPE AND CMAKE_VERSION VERSION_LESS 3.4)
		MESSAGE(FATAL_ERROR "Building shared libraries on Windows requires CMake 3.4 or newer.")
	ENDIF()
ENDIF()


