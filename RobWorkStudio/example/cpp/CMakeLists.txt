cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ExampleSnippets)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "None")
endif()
set(RW_ROOT "$ENV{RW_ROOT}")
set(RWS_ROOT "$ENV{RWS_ROOT}")

#find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
find_package(RobWorkStudio REQUIRED PATHS "${RWS_ROOT}")

macro(rw_build _name)
    # And now we add any targets that we want
    add_executable(${_name} ${_name}.cpp)
    target_link_libraries(${_name} PUBLIC ${ROBWORKSTUDIO_LIBRARIES})
    target_include_directories(${_name} PUBLIC ${ROBWORKSTUDIO_INCLUDE_DIRS})
endmacro()

macro(rw_build_lib _name)
    # And now we add any targets that we want
    add_library(${_name} ${_name}.cpp)
    target_link_libraries(${_name} PUBLIC ${ROBWORKSTUDIO_LIBRARIES})
    target_include_directories(${_name} PUBLIC  ${ROBWORKSTUDIO_INCLUDE_DIRS})
endmacro()

rw_build(SimulatedCameraExample)

foreach(obj ${ROBWORKSTUDIO_INCLUDE_DIRS})
    message(STATUS "INCL_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORKSTUDIO_LIBRARY_DIRS})
    message(STATUS "LIB_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORKSTUDIO_LIBRARIES})
    message(STATUS "LIB: ${obj}")
endforeach()

message(STATUS "${RWS_BUILD_WITH_LIBRARY_SUBDIRS}")
message(STATUS "${RWS_ROOT}")
message(STATUS "${RW_ROOT}")
