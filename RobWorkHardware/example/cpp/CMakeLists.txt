cmake_minimum_required(VERSION 2.6.0)
PROJECT(sdurwhw-examples)

set(RWHW_ROOT "$ENV{RWHW_ROOT}")


macro(rwhw_build _name _component)
    add_executable(${_name} ${_name}.cpp)
    find_package(RobWorkHardware REQUIRED COMPONENTS ${_component} PATHS "${RWHW_ROOT}")
    target_link_libraries(${_name} ${ROBWORKHARDWARE_LIBRARIES})
    target_include_directories(${_name} PRIVATE ${ROBWORKHARDWARE_INCLUDE_DIRS})
endmacro()

rwhw_build(ex-ur-rtde-connect universalrobots_rtde)
rwhw_build(ex-dockwelder-connect dockwelder)