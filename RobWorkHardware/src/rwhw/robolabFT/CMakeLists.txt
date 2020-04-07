set(SUBSYS_NAME sdurwhw_robolabFT)
set(SUBSYS_DESC "Driver for robolab force torque device")
set(SUBSYS_DEPS sdurwhw_serialport RW::sdurw)

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
    # INCLUDE_DIRECTORIES( ${robolabFT_INCLUDE_DIR} )

    set(SRC_FILES RobolabFTDriver.cpp)
    set(SRC_FILES_HPP RobolabFTDriver.hpp)


    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurwhw_serialport RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rwhw/robolabFT" ${SRC_FILES_HPP})

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    message(STATUS "robolabFT include dir:" ${robolabFT_INCLUDE_DIR})
endif()
