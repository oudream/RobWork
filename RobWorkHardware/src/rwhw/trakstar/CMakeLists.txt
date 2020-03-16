set(SUBSYS_NAME sdurwhw_trakstar)
set(SUBSYS_DESC "Calibration of devices")
set(SUBSYS_DEPS RW::sdurw)

set(build TRUE)

find_package(TrakStar QUIET)
set(DEFAULT TRUE)
set(REASON)
if(NOT TRAKSTAR_FOUND)
    set(DEFAULT FALSE)
    set(REASON "Trakstar library not found!")
endif()

if(${BUILD_trakstar})
    find_package(TrakStar REQUIRED)
    if(NOT ${TRAKSTAR_FOUND})
        set(BUILD_trakstar FALSE)
        set(REASON "Could not find Trakstar libraries!")
    endif()
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    find_package(TrakStar)
    set(SRC_CPP)
    set(SRC_HPP)

    list(APPEND SRC_CPP Trakstar.cpp)
    list(APPEND SRC_HPP Trakstar.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw ${TRAKSTAR_LIBRARIES})
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        ${TRAKSTAR_INCLUDE_DIR}
        INTERFACE
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rw_add_includes(${SUBSYS_NAME} "rwhw/trakstar" ${SRC_HPP})
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_trakstar PARENT_SCOPE)
endif()
