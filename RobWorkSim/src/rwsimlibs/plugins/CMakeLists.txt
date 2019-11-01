include_directories(${ROBWORKSTUDIO_INCLUDE_DIRS})
link_directories(${ROBWORKSTUDIO_LIBRARY_DIRS})

# to be able to include the generated ui header files
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# to avoid windows compile bug
include_directories(${PYTHON_INCLUDE_DIRS})
# ######################################################################################################################
# Rcc shared for all plugins
#
set(RccSrcFiles)
if(RWS_USE_QT5)
    qt5_add_resources(RccSrcFiles resources.qrc)
else()
    qt4_add_resources(RccSrcFiles resources.qrc)
endif()

# ######################################################################################################################
# next compile RWSimulatorPlugin
#
set(SrcFiles RWSimulatorPlugin.cpp)

# Call the create_plugin macro for creating the plugin
if(RWS_USE_QT5)
    qt5_wrap_cpp(MocSrcFiles RWSimulatorPlugin.hpp)
else()
    qt4_wrap_cpp(
        MocSrcFiles
        RWSimulatorPlugin.hpp
        OPTIONS
        -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
        -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
    )
endif()

# The shared library to build:
add_library(
    RWSimulatorPlugin MODULE
    ${SrcFiles}
    ${MocSrcFiles}
    ${RccSrcFiles}
)
target_link_libraries(
    RWSimulatorPlugin
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
)
# Make sure dependencies are build before system
foreach(
    DEP
    IN
    LISTS
    ROBWORKSIM_LIBRARIES
    ROBWORKSTUDIO_LIBRARIES
)
    if(TARGET ${DEP})
        add_dependencies(RWSimulatorPlugin ${DEP})
    endif()
endforeach()

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
if(RWS_USE_QT5)
    qt5_wrap_cpp(MocSrcFiles SimUtilityPlugin.hpp)
else()
    qt4_wrap_cpp(
        MocSrcFiles
        SimUtilityPlugin.hpp
        OPTIONS
        -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
        -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
    )
endif()

# The shared library to build:
add_library(
    SimUtilityPlugin MODULE
    ${SrcFiles}
    ${MocSrcFiles}
    ${RccSrcFiles}
)
target_link_libraries(
    SimUtilityPlugin
    sdurwsim_gui
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
)

# ######################################################################################################################
# next compile GraspTableGeneratorPlugin
#
set(UIS_OUT_H)
if(RWS_USE_QT5)
    qt5_wrap_ui(UIS_OUT_H GraspTableGeneratorPlugin.ui)
else()
    qt4_wrap_ui(UIS_OUT_H GraspTableGeneratorPlugin.ui)
endif()
set(SrcFiles GraspTableGeneratorPlugin.cpp)
set_source_files_properties(
    ${SrcFiles}
    PROPERTIES
    OBJECT_DEPENDS
    "${UIS_OUT_H}"
)
# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
if(RWS_USE_QT5)
    qt5_wrap_cpp(MocSrcFiles GraspTableGeneratorPlugin.hpp)
else()
    qt4_wrap_cpp(
        MocSrcFiles
        GraspTableGeneratorPlugin.hpp
        OPTIONS
        -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
        -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
    )
endif()

# The shared library to build:
add_library(
    GraspTableGeneratorPlugin MODULE
    ${SrcFiles}
    ${MocSrcFiles}
    ${RccSrcFiles}
    ${UIS_OUT_H}
)
target_link_libraries(
    GraspTableGeneratorPlugin
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
)

# ######################################################################################################################
# The RW Simulation
#
set(UIS_OUT_H)
if(RWS_USE_QT5)
    qt5_wrap_ui(UIS_OUT_H RWSimPlugin.ui)
else()
    qt4_wrap_ui(UIS_OUT_H RWSimPlugin.ui)
endif()
set(SrcFiles RWSimPlugin.cpp)
set_source_files_properties(
    ${SrcFiles}
    PROPERTIES
    OBJECT_DEPENDS
    "${UIS_OUT_H}"
)

# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
if(RWS_USE_QT5)
    qt5_wrap_cpp(MocSrcFiles RWSimPlugin.hpp)
else()
    qt4_wrap_cpp(
        MocSrcFiles
        RWSimPlugin.hpp
        OPTIONS
        -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
        -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
    )
endif()

# Rcc the files:
set(RccSrcFiles_RWSimPlugin)
if(RWS_USE_QT5)
    qt5_add_resources(RccSrcFiles_RWSimPlugin ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)
else()
    qt4_add_resources(RccSrcFiles_RWSimPlugin ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)
endif()
if(RWSIM_HAVE_LUA)
    set(RWSIM_LUA_LIB sdurwsim_lua_s)
    set(
        RWS_LUAPL_LIBRARIES
        sdurws_luapl
        sdurws_sensors
        sdurws_planning
        sdurws_propertyview
        sdurws_playback
        sdurws_treeview
        sdurws_jog
        sdurws_log
        sdurws_robworkstudioapp
    )
endif()

add_library(
    RWSimPlugin MODULE
    ${SrcFiles}
    ${MocSrcFiles}
    ${RccSrcFiles_RWSimPlugin}
    ${UIS_OUT_H}
)
# ADD_LIBRARY(RWSimPluginStatic STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_RWSimPlugin})
target_link_libraries(
    RWSimPlugin
    ${RWS_LUAPL_LIBRARIES}
    sdurwsim_gui
    ${RWSIM_LUA_LIB}
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
    ${ROBWORK_LIBRARIES}
)
add_dependencies(
    RWSimPlugin
    ${RWSIM_LUA_LIB}
    sdurwsim_gui
)
# Message(" LIBS :  ${ROBWORKSIM_LIBRARIES} ")

# ######################################################################################################################
# The SimTaskPlugin
#
set(UIS_OUT_H)
if(RWS_USE_QT5)
    qt5_wrap_ui(UIS_OUT_H SimTaskPlugin.ui)
else()
    qt4_wrap_ui(UIS_OUT_H SimTaskPlugin.ui)
endif()
set(SrcFiles SimTaskPlugin.cpp)
set_source_files_properties(
    ${SrcFiles}
    PROPERTIES
    OBJECT_DEPENDS
    "${UIS_OUT_H}"
)

# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
if(RWS_USE_QT5)
    qt5_wrap_cpp(MocSrcFiles SimTaskPlugin.hpp)
else()
    qt4_wrap_cpp(
        MocSrcFiles
        SimTaskPlugin.hpp
        OPTIONS
        -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
        -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
    )
endif()
# Rcc the files:
set(RccSrcFiles_SimTaskPlugin)
if(RWS_USE_QT5)
    qt5_add_resources(RccSrcFiles_SimTaskPlugin ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)
else()
    qt4_add_resources(RccSrcFiles_SimTaskPlugin ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)
endif()

add_library(
    SimTaskPlugin MODULE
    ${SrcFiles}
    ${MocSrcFiles}
    ${RccSrcFiles_SimTaskPlugin}
    ${UIS_OUT_H}
)
# ADD_LIBRARY(SimTaskPluginStatic STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_SimTaskPlugin})
target_link_libraries(
    SimTaskPlugin
    sdurwsim_gui
    sdurw_task
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
    ${ROBWORK_LIBRARIES}
)

# ######################################################################################################################
# The SimTaskVisPlugin -- has been moved to RobWorkStudio now named gtaskplugin
#
set(UIS_OUT_H)
if(RWS_USE_QT5)
    qt5_wrap_ui(UIS_OUT_H SimTaskVisPlugin.ui)
else()
    qt4_wrap_ui(UIS_OUT_H SimTaskVisPlugin.ui)
endif()
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
if(RWS_USE_QT5)
    qt5_wrap_ui(UIS_OUT_H EngineTestPlugin.ui)
else()
    qt4_wrap_ui(UIS_OUT_H EngineTestPlugin.ui)
endif()

set(SrcFiles EngineTestPlugin.cpp)
# set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS
# "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
set(MocSrcFiles)
if(RWS_USE_QT5)
    qt5_wrap_cpp(MocSrcFiles EngineTestPlugin.hpp)
else()
    qt4_wrap_cpp(
        MocSrcFiles
        EngineTestPlugin.hpp
        OPTIONS
        -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
        -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
    )
endif()

# The shared library to build:
add_library(
    EngineTestPlugin MODULE
    ${SrcFiles}
    ${MocSrcFiles}
    ${RccSrcFiles}
    ${UIS_OUT_H}
)
target_link_libraries(
    EngineTestPlugin
    sdurwsim_test
    sdurwsim_gui
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
)
