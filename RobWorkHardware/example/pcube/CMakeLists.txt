# check compiler/operating system

if(NOT DEFINED RWHW_ROOT)
    set(RWHW_ROOT "$ENV{RWHW_ROOT}/cmake")
endif()

find_package(RobWorkHardware REQUIRED COMPONENTS pcube can serialport PATHS "${RWHW_ROOT}")

if(RobWorkHardware_FOUND)
    add_executable(PCubeExample PCubeExample.cpp )
    target_link_libraries(PCubeExample PUBLIC ${ROBWORKHARDWARE_LIBRARIES})
    target_include_directories(PCubeExample PUBLIC ${ROBWORKHARDWARE_INCLUDE_DIRS})
endif()
