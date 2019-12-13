set(SUBSYS_NAME sdurwhw_motomanIA20)
set(SUBSYS_DESC "Library for controlling the motomanIA20 robot")
set(SUBSYS_DEPS)

set(DEFAULT TRUE)
set(REASON)
if(DEFINED WIN32)
    set(DEFAULT false)
    set(REASON "Does not support windows!")
endif()

set(build TRUE)

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    add_library(${SUBSYS_NAME} MotomanIA20.cpp)
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    message(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")
endif()
