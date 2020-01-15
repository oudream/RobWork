set(SUBSYS_NAME sdurwhw_schunkpg70)
set(SUBSYS_DESC "Driver for schunk gripper pg70")
set(SUBSYS_DEPS sdurwhw_pcube sdurwhw_serialport sdurw)

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
    set(SRC_CPP SchunkPG70.cpp)
    set(SRC_HPP SchunkPG70.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurwhw_pcube sdurwhw_serialport RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rwhw/schunkpg70" ${SRC_HPP})

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
