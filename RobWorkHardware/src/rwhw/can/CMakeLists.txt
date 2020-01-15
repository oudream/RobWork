set(SUBSYS_NAME sdurwhw_can)
set(SUBSYS_DESC "Library of drivers for different CAN devices")
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
    # MESSAGE(STATUS "RobWorkHardware: CAN component ENABLED")

    set(SRC_CPP ${RW_IO_USER_FILES} CANPortFactory.cpp)
    set(SRC_HPP CANPortFactory.hpp CanPort.hpp)

    set(SRC_HPP_ESDCAN)
    find_package(ESDCAN)
    if(ESDCAN_FOUND)
        add_definitions(-DRWHW_HAS_ESDCAN)
        list(APPEND SRC_CPP ./ESDCAN/ESDCANPort.cpp)
        list(APPEND SRC_HPP_ESDCAN ./ESDCAN/ESDCANPort.hpp)
    endif()

    set(SRC_HPP_IEICAN02)
    find_package(IEICAN)
    if(IEICAN_FOUND)
        add_definitions(-DRWHW_HAS_IEICAN)
        list(APPEND SRC_CPP ./IEICAN02/IEICANPort.cpp)
        list(APPEND SRC_HPP_IEICAN02 ./IEICAN02/IEICANPort.hpp)
    endif()

    rw_add_library(
        ${SUBSYS_NAME}
        ${SRC_CPP}
        ${SRC_HPP}
        ${SRC_HPP_IEICAN02}
        ${SRC_HPP_ESDCAN}
    )
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    if(ESDCAN_FOUND)
        target_include_directories(${SUBSYS_NAME} PUBLIC ${ESDCAN_INCLUDE_DIR})
    endif()
    if(IEICAN_FOUND)
        target_include_directories(${SUBSYS_NAME} PRIVATE ${IEICAN_INCLUDE_DIR})
    endif()
    rw_add_includes(${SUBSYS_NAME} "rwhw/can" ${SRC_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwhw/can/ESDCAN" ${SRC_HPP_ESDCAN})
    rw_add_includes(${SUBSYS_NAME} "rwhw/can/IEICAN02" ${SRC_HPP_IEICAN02})

endif()
