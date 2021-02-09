#####################################################
# Template for building RobWork dependent projects
# - the src should contain code for putting into libraries 
# - the plugins contain code for multiple RobWorkStudio plugins
# - the test for testing 
# - the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

# The name of the project.
PROJECT(PluginUIApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

#set(CMAKE_VERBOSE_MAKEFILE True)

SET(RW_ROOT "${ROOT}/../../../RobWork")
SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
SET(RobWork_DIR "${RW_ROOT}/cmake") 
SET(RobWorkStudio_DIR "${RWSTUDIO_ROOT}/cmake") 

# We use the settings that robwork studio uses
IF( NOT CMAKE_BUILD_TYPE)
SET(CMAKE_BUILD_TYPE Release)
endif()

FIND_PACKAGE(RobWork REQUIRED)
FIND_PACKAGE(RobWorkStudio REQUIRED)

# Set the output dir for generated libraries and binaries
IF(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF()

#FIND_PACKAGE(Boost COMPONENTS asio) 
#IF( Boost_ASIO_FOUND )
# MESSAGE(STATUS "ASIO found - plugin will not be built!")
#ELSE()
# MESSAGE(SEND_ERROR  "ASIO not found - - plugin will not be built!")
#ENDIF()


# if we want to use ui files add them here
SET(UIS_FILES SamplePlugin.ui )

if(DEFINED Qt6Core_VERSION)
    qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt_wrap_cpp(MocSrcFiles SamplePlugin.hpp TARGET SamplePlugin)
    qt_add_resources(RccSrcFiles resources.qrc)
else()
    qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt5_wrap_cpp(MocSrcFiles SamplePlugin.hpp TARGET SamplePlugin)
    qt5_add_resources(RccSrcFiles resources.qrc)
endif()

SET(SrcFiles SamplePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(SamplePlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} )
target_include_directories(SamplePlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIRS})
