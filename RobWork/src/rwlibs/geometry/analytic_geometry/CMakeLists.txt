set(SUBSYS_NAME sdurw_analytic_geometry)
set(SUBSYS_DESC "A module for duing analytic geometry")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    rw_add_library(sdurw_analytic_geometry QuadraticTestObjects.cpp QuadraticTestObjects.hpp)
    target_link_libraries(sdurw_analytic_geometry PUBLIC ${SUBSYS_DEPS})

    rw_add_library(sdurw_analytic_geometry.rwplugin COMPONENT ${SUBSYS_NAME} MODULE AnalyticGeometryPlugin.cpp
                   AnalyticGeometryPlugin.hpp)
    target_link_libraries(sdurw_analytic_geometry.rwplugin PRIVATE sdurw_analytic_geometry PUBLIC sdurw)

    if(CMAKE_VERSION VERSION_GREATER 3.3)
        set_target_properties(sdurw_analytic_geometry PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    endif()
endif()
