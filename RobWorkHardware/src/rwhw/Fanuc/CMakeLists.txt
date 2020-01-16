set(SUBSYS_NAME sdurwhw_fanucdriver)
set(SUBSYS_DESC "Library of drivers for controlling fanuc robot")
set(SUBSYS_DEPS)

set(build TRUE)
set(DEFAULT TRUE)
set(REASON)

if(COMPONENT_${SUBSYS_NAME}_ENABLE AND NOT DEFINED WIN32)
    set(BUILD_${SUBSYS_NAME} FALSE)
    set(REASON "WIN32 only")
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build AND DEFINED WIN32)
    # MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")

    add_library(${SUBSYS_NAME} ./FanucDriver.cpp ./FanucVirtual.cpp ./VelRampProfile.cpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw)
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
