set(SUBSYS_NAME sdurwhw_tactile)
set(SUBSYS_DESC "Tactile array sensor library")
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
    set(
        SRC_CPP
            DSACON32.cpp
            TactileMatrix.cpp
            TactileMaskMatrix.cpp
            ConvertUtil.cpp
    )
    set(
        SRC_HPP
            DSACON32.hpp
            TactileMatrix.hpp
            TactileMaskMatrix.hpp
            ConvertUtil.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurwhw_serialport RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rwhw/tactile" ${SRC_HPP})

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_tactile PARENT_SCOPE)
endif()
