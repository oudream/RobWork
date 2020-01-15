set(SUBSYS_NAME sdurwhw_netft)
set(SUBSYS_DESC "driver for netft")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
set(default TRUE)

set(Boost_NO_BOOST_CMAKE TRUE) # From Boost 1.70, CMake files are provided by Boost - we are not yet ready to
                               # handle it

find_package(Boost REQUIRED COMPONENTS system regex QUIET)
set(DEFAULT TRUE)
set(REASON)
if(NOT Boost_FOUND)
    set(DEFAULT false)
    set(REASON "Boost system and/or regex not found!")
endif()

set(build TRUE)

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    # MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")
    find_package(Boost REQUIRED COMPONENTS system regex)

    set(SRC_CPP NetFTLogging.cpp FTCompensation.cpp)
    set(SRC_HPP NetFTCommon.hpp NetFTLogging.hpp FTCompensation.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${ROBWORK_LIBRARIES} ${RW_BUILD_WITH_LIBRARIES_BOOST})

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS ROBWORK_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(${SUBSYS_NAME} ${DEP})
        endif()
    endforeach()
    rw_add_includes(${SUBSYS_NAME} "rwhw/netft" ${SRC_HPP})

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
