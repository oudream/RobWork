set(SUBSYS_NAME sdurwhw_CRSA465)
set(SUBSYS_DESC "Library of drivers for the CRSA465 robot")
set(SUBSYS_DEPS RW::sdurw)

set(build TRUE)
set(DEFAULT FALSE)
set(REASON)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)
if(build)
    add_library(sdurwhw_crsa465 ./CRSA465.cpp ./crs_aci.cpp)
    install(TARGETS sdurwhw_crsa465 DESTINATION ${RWHW_LIB_INSTALL_DIR})
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_crsa465 PARENT_SCOPE)
endif()
