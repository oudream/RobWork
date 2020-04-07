set(SUBSYS_NAME sdurws_sandbox)
set(SUBSYS_DESC "Missing description!")
set(SUBSYS_DEPS RW::sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    set(SRC_FILES Dummy.cpp)

    set(SRC_FILES_HPP Dummy.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    install(FILES ${SRC_FILES_HPP} DESTINATION ${INCLUDE_INSTALL_DIR}/sandbox/sdurws)
endif()
