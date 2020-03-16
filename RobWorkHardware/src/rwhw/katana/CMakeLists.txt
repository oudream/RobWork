set(SUBSYS_NAME sdurwhw_katana)
set(SUBSYS_DESC "Library for controlling the katana robot")
set(SUBSYS_DEPS RW::sdurw)

find_package(KATANA QUIET)
set(DEFAULT TRUE)
set(REASON "katana library found!")
if(NOT KATANA_FOUND)
    set(DEFAULT false)
    set(REASON "katana library not found!")
endif()

set(build TRUE)

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    find_package(KATANA)
    if(KATANA_FOUND)
        message(STATUS "Katana sdk found. Katana driver will be build!")
        include_directories(${KATANA_INCLUDE_DIR})
        add_library(${SUBSYS_NAME} Katana.cpp Katana.hpp)
        install(TARGETS ${SUBSYS_NAME} DESTINATION ${RWHW_LIB_INSTALL_DIR})
        set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
        target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw)
    else()
        message(STATUS "RobWorkHardware: Katana sdk NOT found. Katana driver will NOT be build!")
    endif()
endif()
