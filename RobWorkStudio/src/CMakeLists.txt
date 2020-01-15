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
if(WIN32 AND IS_RELEASE)
    add_executable(RobWorkStudio WIN32 main.cpp)
else(WIN32 AND IS_RELEASE)
    add_executable(RobWorkStudio main.cpp)
endif(WIN32 AND IS_RELEASE)

set(LIBS_TMP ${RWS_PLUGIN_LIBRARIES} sdurws ${RWS_COMPONENT_LIBRARIES} ${sdurws_LIBRARIES})
# MESSAGE("${LIBS_TMP} sdurws_robworkstudioapp ${ROBWORK_LIBRARIES}")

target_link_libraries(RobWorkStudio ${LIBS_TMP} sdurws_robworkstudioapp RW::sdurw ${GLUT_glut_LIBRARY})

if(RWS_USE_STATIC_LINK_PLUGINS)
    add_dependencies(RobWorkStudio sdurws ${RWS_COMPONENT_LIBRARIES})
endif()

# Some install stuff
install(TARGETS RobWorkStudio DESTINATION ${BIN_INSTALL_DIR})
if(RWS_USE_STATIC_LINK_PLUGINS)
    install(
        FILES "${RWS_ROOT}/bin/RobWorkStudio.ini.template.static"
        DESTINATION ${BIN_INSTALL_DIR}
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

# Install all headerfiles
install(
    DIRECTORY sdurws
    DESTINATION ${INCLUDE_INSTALL_DIR}
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)
