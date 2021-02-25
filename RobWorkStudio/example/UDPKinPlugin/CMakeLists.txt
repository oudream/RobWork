# ##################################################################################################
# Template for building RobWork dependent projects - the src should contain code for putting into
# libraries - the plugins contain code for multiple RobWorkStudio plugins - the test for testing -
# the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this file for adding additional dependencies
# to external projects
# ##################################################################################################

#
# Test CMake version
#
cmake_minimum_required(VERSION 3.5)

# The name of the project.
project(PluginUIApp)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# set(CMAKE_VERBOSE_MAKEFILE True)

set(RW_ROOT "${ROOT}/../../../RobWork")
set(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
set(RobWork_DIR "${RW_ROOT}/cmake")
set(RobWorkStudio_DIR "${RWSTUDIO_ROOT}/cmake")

# We use the settings that robwork studio uses
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)

# Set the output dir for generated libraries and binaries
if(MSVC)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
        "${ROOT}/bin"
        CACHE PATH "Runtime directory" FORCE
    )
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
        "${ROOT}/libs"
        CACHE PATH "Library directory" FORCE
    )
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        "${ROOT}/libs"
        CACHE PATH "Archive directory" FORCE
    )
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
        "${ROOT}/bin/${CMAKE_BUILD_TYPE}"
        CACHE PATH "Runtime directory" FORCE
    )
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
        "${ROOT}/libs/${CMAKE_BUILD_TYPE}"
        CACHE PATH "Library directory" FORCE
    )
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        "${ROOT}/libs/${CMAKE_BUILD_TYPE}"
        CACHE PATH "Archive directory" FORCE
    )
endif()

# if we want to use ui files add them here
set(UIS_FILES SamplePlugin.ui)

if(DEFINED Qt6Core_VERSION)
    qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt_wrap_cpp(MocSrcFiles SamplePlugin.hpp TARGET SamplePlugin)
    qt_add_resources(RccSrcFiles resources.qrc)
else()
    qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt5_wrap_cpp(MocSrcFiles SamplePlugin.hpp TARGET SamplePlugin)
    qt5_add_resources(RccSrcFiles resources.qrc)
endif()

set(SrcFiles SamplePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
add_library(SamplePlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
target_include_directories(
    SamplePlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${ROBWORKSTUDIO_INCLUDE_DIRS}
                        ${ROBWORK_INCLUDE_DIRS}
)
