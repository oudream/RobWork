# ##################################################################################################
# Rcc shared for all plugins
#
set(RccSrcFiles)
if(DEFINED Qt6Core_VERSION)
    qt_add_resources(RccSrcFiles resources.qrc)
else()
    qt5_add_resources(RccSrcFiles resources.qrc)
endif()

# ##################################################################################################
# next compile RWSimulatorPlugin
#
set(SUBSYS_NAME RWSimulatorPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS RWS::sdurws RW::sdurw_simulation RW::sdurw_opengl RW::sdurw sdurwsim)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(SrcFiles RWSimulatorPlugin.cpp)

    # Call the create_plugin macro for creating the plugin
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_cpp(MocSrcFiles RWSimulatorPlugin.hpp TARGET ${SUBSYS_NAME})
    else()
        qt5_wrap_cpp(MocSrcFiles RWSimulatorPlugin.hpp TARGET ${SUBSYS_NAME})
    endif()

    # The shared library to build:
    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    target_link_libraries(
        ${SUBSYS_NAME}
        PRIVATE sdurwsim
        PUBLIC RWS::sdurws
        PRIVATE RW::sdurw_simulation RW::sdurw_opengl
        PUBLIC RW::sdurw
    )

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()

# ##################################################################################################
# next compile SDHPlugin
#
# include_directories(${ROOT}/../../RobWorkHardware/src/)
# link_directories(${ROOT}/../../RobWorkHardware/libs/Release/)
# link_directories(${ROOT}/../../RobWorkHardware/ext/sdh/libs/) link_directories("C:/Program
# Files/ESD/CAN/SDK/lib/vc/i386")

# SET(SrcFiles SDHPlugin.cpp ) SET(MocSrcFiles SDHPlugin.cpp ) SET(MocHeaderFiles SDHPlugin.hpp )
# SET(QrcFiles resources.qrc)

# Call the create_plugin macro for creating the plugin ADD_PLUGIN(SDHPlugin SrcFiles MocHeaderFiles
# QrcFiles) TARGET_LINK_LIBRARIES(SDHPlugin ${ROBWORKSIM_LIBRARIES} sdurwhw_sdh sdh ntcan.lib)

# ##################################################################################################
# next compile SimUtilityPlugin
#
set(SUBSYS_NAME SimUtilityPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(SrcFiles SimUtilityPlugin.cpp)

    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_cpp(MocSrcFiles SimUtilityPlugin.hpp TARGET ${SUBSYS_NAME})
    else()
        qt5_wrap_cpp(MocSrcFiles SimUtilityPlugin.hpp TARGET ${SUBSYS_NAME})
    endif()

    # The shared library to build:
    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    target_link_libraries(
        ${SUBSYS_NAME}
        PRIVATE sdurwsim_gui sdurwsim
        PUBLIC RWS::sdurws RW::sdurw
    )

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()

# ##################################################################################################
# next compile GraspTableGeneratorPlugin
#
set(SUBSYS_NAME GraspTableGeneratorPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(UIS_OUT_H)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H GraspTableGeneratorPlugin.ui)
    else()
        qt5_wrap_ui(UIS_OUT_H GraspTableGeneratorPlugin.ui)
    endif()
    set(SrcFiles GraspTableGeneratorPlugin.cpp)
    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_cpp(MocSrcFiles GraspTableGeneratorPlugin.hpp TARGET ${SUBSYS_NAME})
    else()
        qt5_wrap_cpp(MocSrcFiles GraspTableGeneratorPlugin.hpp TARGET ${SUBSYS_NAME})
    endif()

    # The shared library to build:
    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw)

    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(${SUBSYS_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()

# ##################################################################################################
# The RW Simulation
#
set(SUBSYS_NAME RWSimPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS
    sdurwsim_gui
    sdurwsim
    RWS::sdurws
    RW::sdurw_opengl
    RW::sdurw_simulation
    RW::sdurw_control
    RW::sdurw
)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(UIS_OUT_H)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H RWSimPlugin.ui)
    else()
        qt5_wrap_ui(UIS_OUT_H RWSimPlugin.ui)
    endif()
    set(SrcFiles RWSimPlugin.cpp)
    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles)
    set(RccSrcFiles_RWSimPlugin)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_cpp(MocSrcFiles RWSimPlugin.hpp TARGET ${SUBSYS_NAME})
        qt_add_resources(RccSrcFiles_RWSimPlugin ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)
    else()
        qt5_wrap_cpp(MocSrcFiles RWSimPlugin.hpp TARGET ${SUBSYS_NAME})
        qt5_add_resources(RccSrcFiles_RWSimPlugin ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)
    endif()

    set(RWSIM_LUA_LIB_TMP RW::sdurw_lua_s sdurwsim_lua_s)
    rw_is_targets(test TARGETS ${RWSIM_LUA_LIB_TMP} TARGETS_OUT RWSIM_LUA_LIB)

    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_RWSimPlugin}
                                      ${UIS_OUT_H}
    )
    set_target_properties(${SUBSYS_NAME} PROPERTIES EXCLUDE_FROM_ALL TRUE)

    if(${test})
        target_compile_definitions(${SUBSYS_NAME} PRIVATE RWSIM_PLUGIN_HAVE_LUA)
    endif()

    target_link_libraries(
        ${SUBSYS_NAME}
        PRIVATE sdurwsim_gui
        PUBLIC sdurwsim
        PRIVATE ${RWSIM_LUA_LIB}
        PUBLIC RWS::sdurws
        PRIVATE RW::sdurw_opengl RW::sdurw_simulation RW::sdurw_control
        PUBLIC RW::sdurw
    )

    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(RWSimPlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

    # to avoid windows compile bug
    target_include_directories(RWSimPlugin PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()

# ##################################################################################################
# The SimTaskPlugin
#
set(SUBSYS_NAME SimTaskPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw_task RW::sdurw_opengl RW::sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(UIS_OUT_H)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H SimTaskPlugin.ui)
    else()
        qt5_wrap_ui(UIS_OUT_H SimTaskPlugin.ui)
    endif()
    set(SrcFiles SimTaskPlugin.cpp)
    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles)
    set(RccSrcFiles_SimTaskPlugin)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_cpp(MocSrcFiles SimTaskPlugin.hpp TARGET ${SUBSYS_NAME})
        qt_add_resources(RccSrcFiles_SimTaskPlugin ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)
    else()
        qt5_wrap_cpp(MocSrcFiles SimTaskPlugin.hpp TARGET ${SUBSYS_NAME})
        qt5_add_resources(RccSrcFiles_SimTaskPlugin ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)
    endif()

    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_SimTaskPlugin}
                                      ${UIS_OUT_H}
    )
    # ADD_LIBRARY(SimTaskPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}
    # ${RccSrcFiles_SimTaskPlugin})
    target_link_libraries(
        ${SUBSYS_NAME}
        PRIVATE sdurwsim_gui
        PUBLIC sdurwsim RWS::sdurws
        PRIVATE RW::sdurw_task RW::sdurw_opengl
        PUBLIC RW::sdurw
    )

    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(${SUBSYS_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()

# ##################################################################################################
# The SimTaskVisPlugin -- has been moved to RobWorkStudio now named gtaskplugin
#
set(UIS_OUT_H)
if(DEFINED Qt6Core_VERSION)
    qt_wrap_ui(UIS_OUT_H SimTaskVisPlugin.ui)
else()
    qt5_wrap_ui(UIS_OUT_H SimTaskVisPlugin.ui)
endif()
# SET(SrcFiles SimTaskVisPlugin.cpp) set_source_files_properties(${SrcFiles} PROPERTIES
# OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin SET(MocSrcFiles ) QT4_WRAP_CPP(MocSrcFiles
# SimTaskVisPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS
# -DBOOST_LEXICAL_CAST_INCLUDED) Rcc the files: SET(RccSrcFiles ) QT4_ADD_RESOURCES(RccSrcFiles
# ${RWSIM_ROOT}/src/gfx/SimTaskVisPlugin.qrc)

# ADD_LIBRARY(SimTaskVisPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles} ${UIS_OUT_H})
# ADD_LIBRARY(SimTaskVisPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
# TARGET_LINK_LIBRARIES(SimTaskVisPlugin rwsim_ode ode rwsim_gui rw_task ${ROBWORKSIM_LIBRARIES}
# ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})

# ##################################################################################################
# next compile EngineTestPlugin
#

set(SUBSYS_NAME EngineTestPlugin)
set(SUBSYS_DESC "A plugin for executing standard tests for Physics Engines.")
set(SUBSYS_DEPS sdurwsim_gui RWS::sdurws)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(UIS_OUT_H)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H EngineTestPlugin.ui)
    else()
        qt5_wrap_ui(UIS_OUT_H EngineTestPlugin.ui)
    endif()

    set(SrcFiles EngineTestPlugin.cpp)
    # set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS
    # "${UIS_OUT_H}")

    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_cpp(MocSrcFiles EngineTestPlugin.hpp TARGET ${SUBSYS_NAME})
    else()
        qt5_wrap_cpp(MocSrcFiles EngineTestPlugin.hpp TARGET ${SUBSYS_NAME})
    endif()

    # The shared library to build:
    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC sdurwsim_test
        PRIVATE sdurwsim_gui
        PUBLIC RWS::sdurws
    )

    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(${SUBSYS_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()
