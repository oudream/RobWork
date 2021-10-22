set(SUBSYS_NAME sdurw_analytic_geometry)
set(SUBSYS_DESC "A module for duing analytic geometry")
set(SUBSYS_DEPS
    PUBLIC sdurw_core
    PRIVATE sdurw_geometry sdurw_math
)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    rw_add_library(${SUBSYS_NAME} QuadraticTestObjects.cpp QuadraticTestObjects.hpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rw_include_eigen(${SUBSYS_NAME})

    rw_add_plugin(${SUBSYS_NAME}.rwplugin MODULE AnalyticGeometryPlugin.cpp
                   AnalyticGeometryPlugin.hpp)
    target_link_libraries(${SUBSYS_NAME}.rwplugin PRIVATE ${SUBSYS_NAME} PUBLIC sdurw_core)

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
