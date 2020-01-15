set(SUBSYS_NAME sdurwhw_robotiq)
set(SUBSYS_DESC "driver for robotiq")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
set(DEFAULT TRUE)
set(REASON)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    message(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")

    set(SRC_CPP Robotiq.cpp Robotiq3.cpp Robotiq2.cpp)
    set(SRC_HPP Robotiq.hpp Robotiq3.hpp Robotiq2.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rwhw/robotiq" ${SRC_HPP})
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)

    add_executable(robotiq3_test robotiq3SimpleTest.cpp)
    target_link_libraries(robotiq3_test ${SUBSYS_NAME} RW::sdurw ${CMAKE_THREAD_LIBS_INIT})

    add_executable(robotiq2_test robotiq2SimpleTest.cpp)
    target_link_libraries(robotiq2_test ${SUBSYS_NAME} RW::sdurw ${CMAKE_THREAD_LIBS_INIT})

endif()
