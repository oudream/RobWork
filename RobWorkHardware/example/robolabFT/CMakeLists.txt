
if(NOT DEFINED RWHW_ROOT)
    set(RWHW_ROOT "$ENV{RWHW_ROOT}/cmake")
endif()

find_package(RobWorkHardware REQUIRED COMPONENTS robolabFT serialport PATHS "${RWHW_ROOT}")

if(RobWorkHardware_FOUND)
    add_executable(robolabFT robolabFT.cpp)
    target_link_libraries(robolabFT PUBLIC ${ROBWORKHARDWARE_LIBRARIES} -lpthread)
    target_include_directories(robolabFT PUBLIC ${ROBWORKHARDWARE_INCLUDE_DIRS})
endif()
