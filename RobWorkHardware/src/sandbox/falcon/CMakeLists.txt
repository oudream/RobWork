# ######################################################################################################################
# Template for building RobWork dependent projects - the src should contain code for putting into libraries - the
# plugins contain code for multiple RobWorkStudio plugins - the test for testing - the example dir for illustrativ
# examples of your project
#
# Use config.cmake to change default settings and edit this file for adding additional dependencies to external projects
# ######################################################################################################################

#
# Test CMake version
#
cmake_minimum_required(VERSION 3.5.1)

# The name of the project.
project(NovintFalcon)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(RW_ROOT "/home/nadezda/workspace/rwtrunk/RobWork")

set(RWSTUDIO_ROOT "/home/nadezda/workspace/rwtrunk/RobWorkStudio")
set(RWSIM_ROOT "/home/nadezda/workspace/rwtrunk/RobWorkSim")
set(RobWork_DIR ${RW_ROOT}/cmake)
set(RobWorkSim_DIR ${RWSIM_ROOT}/cmake)
set(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)

# Now set the RW/RWS root (edit this if necessary) SET(RobWorkStudio_DIR
# "/home/nadezda/workspace/rwtrunk/RobWorkStudio") SET(RobWorkSim_DIR "/home/nadezda/workspace/rwtrunk/RobWorkSim")

# Include RobWorkSim settings
find_package(RobWorkStudio REQUIRED)
find_package(RobWorkSim REQUIRED)
include_directories(${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORKSIM_INCLUDE_DIRS})
link_directories(${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS})

# Set the output dir for generated libraries and binaries
if(MSVC)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
endif()

set(CMAKE_CXX_FLAGS ${CMAKE_CSS_FLAGS} "-fPIC")

# if we want to use ui files add them here SET(UIS_FILES src/FalconControl.ui )

# RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

# QT4_WRAP_CPP(MocSrcFiles src/FalconControl.hpp ) Rcc the files:

# QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
set(SrcFiles src/FalconInterface.cpp)

# The shared library to build:
add_library(NovintFalcon ${SrcFiles})
target_link_libraries(NovintFalcon ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES}
                      libnifalcon.so)

add_executable(falcon-test src/falconTest.cpp)
target_link_libraries(
    falcon-test
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
    ${ROBWORK_LIBRARIES}
    NovintFalcon
    libnifalcon.so
)

# also, the plugin:
set(UIS_FILES plugin/FalconPlugin.ui)
rws_qt4_wrap_ui(UIS_OUT_H ${UIS_FILES})
qt4_add_resources(RccSrcFiles plugin/resources.qrc)
qt4_wrap_cpp(MocSrcFiles plugin/FalconPlugin.hpp)
add_library(FalconPlugin MODULE plugin/FalconPlugin.cpp plugin/SimulationTrajectory.cpp ${UIS_OUT_H} ${MocSrcFiles}
                                ${RccSrcFiles})
target_link_libraries(
    FalconPlugin
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
    ${ROBWORK_LIBRARIES}
    NovintFalcon
    libnifalcon.so
)
