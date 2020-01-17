# ######################################################################################################################
# Rcc shared for all plugins
#
set(RccSrcFiles)
qt5_add_resources(RccSrcFiles resources.qrc)

# ######################################################################################################################
# next compile RWSimulatorPlugin
#
set(SrcFiles RWSimulatorPlugin.cpp)

# Call the create_plugin macro for creating the plugin
qt5_wrap_cpp(MocSrcFiles RWSimulatorPlugin.hpp TARGET RWSimulatorPlugin)

# The shared library to build:
add_library(RWSimulatorPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(RWSimulatorPlugin
    PRIVATE sdurwsim
    PUBLIC RWS::sdurws
    PRIVATE RW::sdurw_simulation RW::sdurw_opengl
    PUBLIC RW::sdurw
)

# to avoid windows compile bug
target_include_directories(RWSimulatorPlugin PRIVATE ${PYTHON_INCLUDE_DIRS})

# ######################################################################################################################
# next compile SDHPlugin
#
# include_directories(${ROOT}/../../RobWorkHardware/src/) link_directories(${ROOT}/../../RobWorkHardware/libs/Release/)
# link_directories(${ROOT}/../../RobWorkHardware/ext/sdh/libs/) link_directories("C:/Program
# Files/ESD/CAN/SDK/lib/vc/i386")

# SET(SrcFiles SDHPlugin.cpp ) SET(MocSrcFiles SDHPlugin.cpp ) SET(MocHeaderFiles SDHPlugin.hpp ) SET(QrcFiles
# resources.qrc)

# Call the create_plugin macro for creating the plugin ADD_PLUGIN(SDHPlugin SrcFiles MocHeaderFiles QrcFiles)
# TARGET_LINK_LIBRARIES(SDHPlugin ${ROBWORKSIM_LIBRARIES} sdurwhw_sdh sdh ntcan.lib)

# ######################################################################################################################
# next compile SimUtilityPlugin
#
set(SrcFiles SimUtilityPlugin.cpp)

# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
qt5_wrap_cpp(MocSrcFiles SimUtilityPlugin.hpp TARGET SimUtilityPlugin)

# The shared library to build:
add_library(SimUtilityPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(SimUtilityPlugin PRIVATE sdurwsim_gui sdurwsim PUBLIC RWS::sdurws RW::sdurw)

# to avoid windows compile bug
target_include_directories(SimUtilityPlugin PRIVATE ${PYTHON_INCLUDE_DIRS})

# ######################################################################################################################
# next compile GraspTableGeneratorPlugin
#
set(UIS_OUT_H)
qt5_wrap_ui(UIS_OUT_H GraspTableGeneratorPlugin.ui)
set(SrcFiles GraspTableGeneratorPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
qt5_wrap_cpp(MocSrcFiles GraspTableGeneratorPlugin.hpp TARGET GraspTableGeneratorPlugin)

# The shared library to build:
add_library(GraspTableGeneratorPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
target_link_libraries(GraspTableGeneratorPlugin PUBLIC sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw)

# Need to add the current binary dir to the include directory because the generated source files are placed here
target_include_directories(GraspTableGeneratorPlugin PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# to avoid windows compile bug
target_include_directories(GraspTableGeneratorPlugin PRIVATE ${PYTHON_INCLUDE_DIRS})

# ######################################################################################################################
# The RW Simulation
#
set(UIS_OUT_H)
qt5_wrap_ui(UIS_OUT_H RWSimPlugin.ui)
set(SrcFiles RWSimPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
qt5_wrap_cpp(MocSrcFiles RWSimPlugin.hpp TARGET RWSimPlugin)

# Rcc the files:
set(RccSrcFiles_RWSimPlugin)
qt5_add_resources(RccSrcFiles_RWSimPlugin ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)
if(RWSIM_HAVE_LUA)
    set(RWSIM_LUA_LIB sdurwsim_lua_s)
endif()

add_library(RWSimPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_RWSimPlugin} ${UIS_OUT_H})
# ADD_LIBRARY(RWSimPluginStatic STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_RWSimPlugin})
target_link_libraries(
    RWSimPlugin
    PRIVATE sdurwsim_gui
    PUBLIC sdurwsim
    PRIVATE ${RWSIM_LUA_LIB}
    PUBLIC RWS::sdurws
    PRIVATE
    RW::sdurw_opengl
    RW::sdurw_simulation
    RW::sdurw_control
    RW::sdurw_lua_s
    PUBLIC RW::sdurw
)

# Need to add the current binary dir to the include directory because the generated source files are placed here
target_include_directories(RWSimPlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# to avoid windows compile bug
target_include_directories(RWSimPlugin PRIVATE ${PYTHON_INCLUDE_DIRS})

# ######################################################################################################################
# The SimTaskPlugin
#
set(UIS_OUT_H)
qt5_wrap_ui(UIS_OUT_H SimTaskPlugin.ui)
set(SrcFiles SimTaskPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
qt5_wrap_cpp(MocSrcFiles SimTaskPlugin.hpp TARGET SimTaskPlugin)
# Rcc the files:
set(RccSrcFiles_SimTaskPlugin)
qt5_add_resources(RccSrcFiles_SimTaskPlugin ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)

add_library(SimTaskPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_SimTaskPlugin} ${UIS_OUT_H})
# ADD_LIBRARY(SimTaskPluginStatic STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_SimTaskPlugin})
target_link_libraries(
    SimTaskPlugin
    PRIVATE sdurwsim_gui
    PUBLIC
    sdurwsim
    RWS::sdurws
    PRIVATE
    RW::sdurw_task
    RW::sdurw_opengl
    PUBLIC RW::sdurw
)

# Need to add the current binary dir to the include directory because the generated source files are placed here
target_include_directories(SimTaskPlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# to avoid windows compile bug
target_include_directories(SimTaskPlugin PRIVATE ${PYTHON_INCLUDE_DIRS})

# ######################################################################################################################
# The SimTaskVisPlugin -- has been moved to RobWorkStudio now named gtaskplugin
#
set(UIS_OUT_H)
qt5_wrap_ui(UIS_OUT_H SimTaskVisPlugin.ui)
# SET(SrcFiles SimTaskVisPlugin.cpp) set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin SET(MocSrcFiles ) QT4_WRAP_CPP(MocSrcFiles SimTaskVisPlugin.hpp
# OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED) Rcc the files: SET(RccSrcFiles )
# QT4_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/SimTaskVisPlugin.qrc)

# ADD_LIBRARY(SimTaskVisPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles} ${UIS_OUT_H})
# ADD_LIBRARY(SimTaskVisPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
# TARGET_LINK_LIBRARIES(SimTaskVisPlugin rwsim_ode ode rwsim_gui rw_task ${ROBWORKSIM_LIBRARIES}
# ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})

# ######################################################################################################################
# next compile EngineTestPlugin
#
set(UIS_OUT_H)
qt5_wrap_ui(UIS_OUT_H EngineTestPlugin.ui)

set(SrcFiles EngineTestPlugin.cpp)
# set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS
# "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
qt5_wrap_cpp(MocSrcFiles EngineTestPlugin.hpp TARGET EngineTestPlugin)

# The shared library to build:
add_library(EngineTestPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
target_link_libraries(EngineTestPlugin PUBLIC sdurwsim_test PRIVATE sdurwsim_gui PUBLIC RWS::sdurws)

# Need to add the current binary dir to the include directory because the generated source files are placed here
target_include_directories(EngineTestPlugin PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# to avoid windows compile bug
target_include_directories(EngineTestPlugin PRIVATE ${PYTHON_INCLUDE_DIRS})
