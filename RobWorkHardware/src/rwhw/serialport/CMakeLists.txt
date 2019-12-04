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
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${ROBWORK_LIBRARIES})
    rw_add_includes(${SUBSYS_NAME} "rwhw/serialport" SerialPort.hpp)
    # Make sure dependencies are build before system
    foreach(DEP IN LISTS SUBSYS_DEPS ROBWORK_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(${SUBSYS_NAME} ${DEP})
        endif()
    endforeach()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
