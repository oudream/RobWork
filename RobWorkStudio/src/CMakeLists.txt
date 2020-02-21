# MESSAGE("ROBWORKSTUDIO LIBS: ${ROBWORKSTUDIO_LIBRARIES}")
add_definitions(-DQT_NO_KEYWORDS)
# First we compile the robworkstudio library
add_subdirectory(rws)

# compile the component libraries (plugins and other static libs)
add_subdirectory(rwslibs)

if(POLICY CMP0077) # Introduce cmake 3.13
    cmake_policy(SET CMP0077 OLD)
endif()

# Then sandbox if choosen
option(RWS_BUILD_SANDBOX "Set when you want to build the sandbox library" ${RWS_BUILD_SANDBOX})
if(RWS_BUILD_SANDBOX)
    add_subdirectory(sandbox)
endif()

# Propagate libraries to parent directory
set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} PARENT_SCOPE)
set(RWS_COMPONENT_LIBRARIES ${RWS_COMPONENT_LIBRARIES} PARENT_SCOPE)

# now create the actual RobWorkStudio executable Now the RobWorkStudio executable need to be build:
set(SUBSYS_NAME RobWorkStudio)
set(SUBSYS_DESC "The RobWorkStudio GUI.")
set(SUBSYS_DEPS sdurws sdurws_robworkstudioapp)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    if(WIN32 AND IS_RELEASE)
        add_executable(${SUBSYS_NAME} WIN32 main.cpp)
    else(WIN32 AND IS_RELEASE)
        add_executable(${SUBSYS_NAME} main.cpp)
    endif(WIN32 AND IS_RELEASE)

    set(LIBS_TMP ${RWS_PLUGIN_LIBRARIES} sdurws ${RWS_COMPONENT_LIBRARIES})
    # MESSAGE("${LIBS_TMP} sdurws_robworkstudioapp ${ROBWORK_LIBRARIES}")

    target_link_libraries(${SUBSYS_NAME} ${LIBS_TMP})
    target_link_libraries(${SUBSYS_NAME} RW::sdurw sdurws_robworkstudioapp)

    if(RWS_USE_STATIC_LINK_PLUGINS)
        if(TARGET sdurws_jog)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_JOG)
        endif()
        if(TARGET sdurws_log)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_LOG)
        endif()
        if(TARGET sdurws_planning)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_PLANNING)
        endif()
        if(TARGET sdurws_playback)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_PLAYBACK)
        endif()
        if(TARGET sdurws_propertyview)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_PROPERTYVIEW)
        endif()
        if(TARGET sdurws_sensors)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_SENSORS)
        endif()
        if(TARGET sdurws_treeview)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_TREEVIEW)
        endif()
        if(TARGET sdurws_workcelleditorplugin)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_WORKCELLEDITOR)
        endif()
        add_dependencies(${SUBSYS_NAME} sdurws ${RWS_COMPONENT_LIBRARIES})
    endif()

    # Some install stuff
    install(TARGETS ${SUBSYS_NAME} DESTINATION ${BIN_INSTALL_DIR} COMPONENT sdurws )
    if(RWS_USE_STATIC_LINK_PLUGINS)
        install(
            FILES "${RWS_ROOT}/bin/RobWorkStudio.ini.template.static"
            DESTINATION ${BIN_INSTALL_DIR}
            COMPONENT sdurws
            RENAME "RobWorkStudio.ini.template"
        )
        configure_file("${RWS_ROOT}/bin/RobWorkStudio.ini.template.static"
                       "${RWS_ROOT}/bin/${RWS_BUILD_TYPE}/RobWorkStudio.ini.template" COPYONLY)
    else()
        install(FILES "${RWS_ROOT}/bin/RobWorkStudio.ini.shared" DESTINATION ${BIN_INSTALL_DIR} RENAME "RobWorkStudio.ini")
        # CONFIGURE_FILE("${RWS_ROOT}/bin/RobWorkStudio.ini.shared" "${RWS_ROOT}/bin/${RWS_BUILD_TYPE}/RobWorkStudio.ini"
        # COPYONLY )
        file(
            GENERATE
            OUTPUT $<TARGET_FILE_DIR:RobWorkStudio>/RobWorkStudio.ini
            INPUT "${RWS_ROOT}/bin/RobWorkStudio.ini.shared"
        )
    endif()
endif()

# Install all headerfiles
install(
    DIRECTORY sdurws
    DESTINATION ${INCLUDE_INSTALL_DIR}
    COMPONENT sdurws
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)
