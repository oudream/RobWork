if(NOT DEFINED RWHW_ROOT)
    set(RWHW_ROOT "$ENV{RWHW_ROOT}/cmake")
endif()

find_package(RobWorkHardware QUIET COMPONENTS sdh PATHS "${RWHW_ROOT}")

if(RobWorkHardware_FOUND)
    add_executable(SDHExample1 SDHExample1.cpp sdhoptions.cpp)
    target_link_libraries(SDHExample1 ${ROBWORKHARDWARE_LIBRARIES})
    target_include_directories(SDHExample1 PUBLIC ${ROBWORKHARDWARE_INCLUDE_DIRS})

    add_executable(demo-simple2 demo-simple2.cpp sdhoptions.cpp)
    target_link_libraries(demo-simple2 ${ROBWORKHARDWARE_LIBRARIES})
    target_include_directories(demo-simple2 PUBLIC ${ROBWORKHARDWARE_INCLUDE_DIRS})
endif()
