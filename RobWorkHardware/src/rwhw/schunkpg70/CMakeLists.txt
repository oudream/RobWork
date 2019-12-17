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
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurwhw_pcube sdurwhw_serialport ${ROBWORK_LIBRARIES})
    rw_add_includes(${SUBSYS_NAME} "rwhw/schunkpg70" ${SRC_HPP})

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS SUBSYS_DEPS ROBWORK_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(${SUBSYS_NAME} ${DEP})
        endif()
    endforeach()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
