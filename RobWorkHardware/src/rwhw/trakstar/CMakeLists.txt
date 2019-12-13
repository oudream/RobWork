set(SUBSYS_NAME trakstar)
set(SUBSYS_DESC "Calibration of devices")
set(SUBSYS_DEPS sdurw)

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

    include_directories(${TRAKSTAR_INCLUDE_DIR})
    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${ROBWORK_LIBRARIES} ${TRAKSTAR_LIBRARIES})

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS ROBWORK_LIBRARIES TRAKSTAR_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(${SUBSYS_NAME} ${DEP})
        endif()
    endforeach()

    rw_add_includes(${SUBSYS_NAME} "rwhw/trakstar" ${SRC_HPP})
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_trakstar PARENT_SCOPE)
endif()
