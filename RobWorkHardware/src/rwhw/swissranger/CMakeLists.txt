set(SUBSYS_NAME sdurwhw_swissranger)
set(SUBSYS_DESC "Swiss ranger driver")
set(SUBSYS_DEPS RW::sdurw)

set(build TRUE)
set(DEFAULT FALSE)
set(REASON)

if(${BUILD_swissranger})
    find_package(SwissRanger REQUIRED)
    if(NOT SwissRanger_FOUND)
        set(BUILD_swissranger FALSE)
        set(REASON "Could not find swissranger library")
    endif()
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    rw_add_library(
        ${SUBSYS_NAME}
        SRCalibrationData.cpp
        SwissRanger.cpp
        SRCalibrationData.hpp
        SwissRanger.hpp
        SRConstants.hpp
    )
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(
        ${SUBSYS_NAME}
        "rwhw/swissranger"
        SRCalibrationData.hpp
        SwissRanger.hpp
        SRConstants.hpp
    )
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
