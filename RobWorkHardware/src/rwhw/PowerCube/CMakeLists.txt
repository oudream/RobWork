set(SUBSYS_NAME sdurwhw_pcube)
set(SUBSYS_DESC "Driver for powercube modules")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
set(DEFAULT true)
set(REASON)

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    # MESSAGE(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")

    set(SRC_CPP CubePort.cpp Cube.cpp PCubeProtocol.cpp)
    set(SRC_HPP CubePort.hpp Cube.hpp PCubeProtocol.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${ROBWORK_LIBRARIES})
    rw_add_includes(${SUBSYS_NAME} "rwhw/PowerCube" ${SRC_HPP})

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS ROBWORK_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(${SUBSYS_NAME} ${DEP})
        endif()
    endforeach()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
