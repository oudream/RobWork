set(SUBSYS_NAME sdurwhw_dockwelder)
set(SUBSYS_DESC "Library for controlling the Dockwelder robot")
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
    # MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
    include_directories(${Boost_ASIO_INCLUDE_DIR})
    rw_add_library(${SUBSYS_NAME} DockWelder.cpp DockWelder.hpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${ROBWORK_LIBRARIES} ${RW_BUILD_WITH_LIBRARIES_BOOST})
    rw_add_includes(${SUBSYS_NAME} "rwhw/dockwelder" DockWelder.hpp)

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS ROBWORK_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(${SUBSYS_NAME} ${DEP})
        endif()
    endforeach()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
