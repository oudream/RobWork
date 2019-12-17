set(SUBSYS_NAME sdurwhw_sick)
set(SUBSYS_DESC "Driver for sick device")
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
    rw_add_library(${SUBSYS_NAME} SickLMS.cpp SickLMS.hpp)
    rw_add_includes(${SUBSYS_NAME} "rwhw/sick" SickLMS.hpp)
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_sick PARENT_SCOPE)
endif()
