cmake_minimum_required(VERSION 3.5.1)
# The name of the project.
project(SamplePluginApp)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
set(RW_ROOT "${ROOT}/../../../RobWork")
set(RWS_ROOT "${ROOT}/../../../RobWorkStudio")

# We use the settings that robwork studio uses
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(RobWorkStudio_DIR "${RWS_ROOT}/cmake")
find_package(RobWorkStudio)

include_directories(${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS})

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

# ########################   From here we add the plugins

if(RWS_USE_QT5)
    qt5_wrap_cpp(MocSrcFiles SamplePlugin.hpp)
    qt5_add_resources(RccSrcFiles resources.qrc)
else()
    qt4_wrap_cpp(
        MocSrcFiles
        SamplePlugin.hpp
        OPTIONS
        -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
        -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
        -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
    )
    qt4_add_resources(RccSrcFiles resources.qrc)
endif()

# The shared library to build:
add_library(SamplePlugin MODULE SamplePlugin.cpp ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
