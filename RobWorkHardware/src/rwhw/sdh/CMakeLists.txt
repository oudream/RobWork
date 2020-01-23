set(SUBSYS_NAME sdurwhw_sdh)
set(SUBSYS_DESC "Driver for robolab sdh ")
set(SUBSYS_DEPS sdurw)

# make sure that SDH can be found
set(build TRUE)

find_package(SDH QUIET)
set(DEFAULT TRUE)
set(REASON)
if(NOT SDH_FOUND)
    set(DEFAULT false)
    set(REASON "SDH library not found!")
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    find_package(SDH)
    find_package(PEAKCAN)

    set(SRC_FILES)
    set(SRC_FILES_HPP)
    if(TOLUA++_FOUND)
        # ADD_TOLUA_PACKAGE("SDHDriverLua" "SDHDriverLuaStub")
    endif()

    rw_add_library(
        ${SUBSYS_NAME}
        SDHDriver.cpp
        SDHDriver.hpp
        ${SRC_FILES}
        ${SRC_FILES_HPP}
    )
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw ${SDH_LIBRARY} ${PEAKCAN_LIBRARIES})
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        ${SDH_INCLUDE_DIR}
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rwhw/sdh" SDHDriver.hpp SDHDriverLua.hpp)
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)

endif()
