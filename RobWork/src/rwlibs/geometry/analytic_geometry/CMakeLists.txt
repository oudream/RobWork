set(SUBSYS_NAME sdurw_analytic_geometry)
set(SUBSYS_DESC "A module for duing analytic geometry")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    rw_add_library(${SUBSYS_NAME} QuadraticTestObjects.cpp QuadraticTestObjects.hpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})

    rw_add_library(${SUBSYS_NAME}.rwplugin COMPONENT ${SUBSYS_NAME} MODULE AnalyticGeometryPlugin.cpp
                   AnalyticGeometryPlugin.hpp)
    target_link_libraries(${SUBSYS_NAME}.rwplugin PRIVATE ${SUBSYS_NAME} PUBLIC sdurw)

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
