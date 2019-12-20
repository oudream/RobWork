set(SUBSYS_NAME sdurwhw_universalrobots_rtde)
set(SUBSYS_DESC "Driver for universal robot using the newer RTDE interface.")
set(SUBSYS_DEPS)

set(DEFAULT TRUE)
set(REASON)

set(RTDE_VERSION_REQUIRED 1.0.0)

find_package(ur_rtde ${RTDE_VERSION_REQUIRED} QUIET CONFIG)

set(UR_RTDE_INTERNAL OFF)
if(NOT ur_rtde_FOUND)
    # Try the internal one
    set(UR_RTDE_INTERNAL ON)
    set(DEFAULT TRUE)
    set(REASON "Building internal ur_rtde library")
else()
    set(DEFAULT TRUE)
    set(REASON "Found ur_rtde library outside RobWorkHardware")
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    if(UR_RTDE_INTERNAL)
        option(PYTHON_BINDINGS "Build python bindings for libraries" OFF)
        set(Boost_NO_BOOST_CMAKE TRUE) # From Boost 1.70, CMake files are provided by Boost - we are not yet
                                       # ready to handle it
        set(tmp ${INCLUDE_INSTALL_DIR})
        set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_DIR}/ext")
        set(PYTHON_BINDINGS OFF)
        add_subdirectory(${RWHW_ROOT}/ext/ur_rtde ur_rtde_ext)
        set(INCLUDE_INSTALL_DIR ${tmp})
    endif()
    # find_package(ur_rtde ${RTDE_VERSION_REQUIRED} REQUIRED CONFIG)

    set(SRC_CPP URRTDE.cpp)
    set(SRC_HPP URRTDE.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PRIVATE ur_rtde::rtde ${ROBWORK_LIBRARIES})
    rw_add_includes(${SUBSYS_NAME} "rwhw/universalrobots_rtde" ${SRC_HPP})

    if(NOT UR_RTDE_INTERNAL)
        # message(STATUS "ur_rtde: ${ur_rtde_FOUND}  - lib: ${ur_rtde_LIBRARIES} - incl:
        # ${ur_rtde_INCLUDE_DIRS}")
        target_include_directories(sdurwhw_universalrobots_rtde PRIVATE ${ur_rtde_INCLUDE_DIRS})
    endif()

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS ROBWORK_LIBRARIES ur_rtde::rtde)
        if(TARGET ${DEP})
            add_dependencies(sdurwhw_universalrobots_rtde ${DEP})
        endif()
    endforeach()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_universalrobots_rtde PARENT_SCOPE)
else()
    message(STATUS "RobWorkHardware: ${component_name} component DISABLED")
endif()
