cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ExampleSnippets)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "None")
endif()
set(RW_ROOT "$ENV{RW_ROOT}")
#set(RobWork_DIR "../../cmake")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

link_directories(${ROBWORK_LIBRARY_DIRS})

macro(rw_build _name)
    # And now we add any targets that we want
    add_executable(${_name} ${_name}.cpp)
    target_link_libraries(${_name} ${ROBWORK_LIBRARIES})
    target_include_directories(${_name} PRIVATE ${ROBWORK_INCLUDE_DIRS})

endmacro()

rw_build(ex-eaa)
#rw_build(ex-find-from-workcell)
#rw_build(ex-frame-to-frame-transform)
#rw_build(ex-frame-to-frame-transforms)
#rw_build(ex-fwd-kinematics-device)
#rw_build(ex-fwd-kinematics)
rw_build(ex-invkin)
rw_build(ex-load-workcell)
rw_build(ex-motionplanning)
#rw_build(ex-print-devices)
#rw_build(ex-print-kinematic-tree)
rw_build(ex-quaternion)
rw_build(ex-rotation3d)
rw_build(ex-rpy)
#rw_build(ex-world-transforms)