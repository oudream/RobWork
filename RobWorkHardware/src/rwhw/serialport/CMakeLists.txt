set(SUBSYS_NAME sdurwhw_serialport)
set(SUBSYS_DESC "Serial port abstractions!")
set(SUBSYS_DEPS sdurw)

set(build)
set(DEFAULT TRUE)
set(REASON "")
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    rw_add_library(${SUBSYS_NAME} SerialPort.cpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rwhw/serialport" SerialPort.hpp)

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
