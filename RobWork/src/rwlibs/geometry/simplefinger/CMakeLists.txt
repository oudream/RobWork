set(SUBSYS_NAME sdurw_simplefinger)
set(SUBSYS_DESC "A module for duing analytic geometry")
set(SUBSYS_DEPS sdurw sdurw_csg)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    rw_add_library(${SUBSYS_NAME} SimpleFinger.cpp SimpleFinger.hpp)
    target_link_libraries(${SUBSYS_NAME} PRIVATE sdurw_csg PUBLIC sdurw)
	
    rw_add_library(${SUBSYS_NAME}.rwplugin COMPONENT ${SUBSYS_NAME} MODULE SimpleFingerPlugin.cpp SimpleFingerPlugin.hpp)
    target_link_libraries(${SUBSYS_NAME}.rwplugin PRIVATE ${SUBSYS_NAME} PUBLIC sdurw)

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
