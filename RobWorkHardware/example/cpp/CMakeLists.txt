cmake_minimum_required(VERSION 3.5.1)
PROJECT(sdurwhw-examples)

set(RWHW_ROOT "$ENV{RWHW_ROOT}")


macro(rwhw_build _name _component)
    add_executable(${_name} ${_name}.cpp)
    find_package(RobWorkHardware REQUIRED COMPONENTS ${_component} PATHS "${RWHW_ROOT}")
    target_link_libraries(${_name} PUBLIC ${ROBWORKHARDWARE_LIBRARIES})
    target_include_directories(${_name} PUBLIC ${ROBWORKHARDWARE_INCLUDE_DIRS})

    foreach(obj ${ROBWORKHARDWARE_INCLUDE_DIRS})
        message(STATUS "INCL_DIR: ${obj}")
    endforeach()
    foreach(obj ${ROBWORKHARDWARE_LIBRARY_DIRS})
        message(STATUS "LIB_DIR: ${obj}")
    endforeach()
    foreach(obj ${ROBWORKHARDWARE_LIBRARIES})
        message(STATUS "LIB: ${obj}")
    endforeach()
endmacro()

rwhw_build(ex-ur-rtde-connect universalrobots_rtde)
rwhw_build(ex-dockwelder-connect dockwelder)


