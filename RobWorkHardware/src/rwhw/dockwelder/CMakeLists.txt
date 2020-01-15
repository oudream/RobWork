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
    rw_add_library(${SUBSYS_NAME} DockWelder.cpp DockWelder.hpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
    	PRIVATE ${Boost_INCLUDE_DIR}
        INTERFACE
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rwhw/dockwelder" DockWelder.hpp)

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
