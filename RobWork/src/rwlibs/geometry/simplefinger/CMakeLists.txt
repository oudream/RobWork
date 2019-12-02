set(SUBSYS_NAME sdurw_simplefinger)
set(SUBSYS_DESC "A module for duing analytic geometry")
set(SUBSYS_DEPS sdurw sdurw_csg)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    rw_add_library(SimpleFinger SimpleFinger.cpp SimpleFinger.hpp)
    target_link_libraries(SimpleFinger PRIVATE sdurw_csg PUBLIC sdurw)
	
    rw_add_library(simplefinger.rwplugin COMPONENT ${SUBSYS_NAME} MODULE SimpleFingerPlugin.cpp SimpleFingerPlugin.hpp)
    target_link_libraries(simplefinger.rwplugin PRIVATE SimpleFinger PUBLIC sdurw)

    if(CMAKE_VERSION VERSION_GREATER 3.3)
        set_target_properties(SimpleFinger PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    endif()

endif()
