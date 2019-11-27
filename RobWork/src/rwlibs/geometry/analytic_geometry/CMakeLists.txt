rw_add_library(sdurw_analytic_geometry sdurw_analytic_geometry QuadraticTestObjects.cpp QuadraticTestObjects.hpp)
target_link_libraries(sdurw_analytic_geometry PUBLIC sdurw)

rw_add_plugin(sdurw_analytic_geometry.rwplugin sdurw_analytic_geometry MODULE AnalyticGeometryPlugin.cpp
               AnalyticGeometryPlugin.hpp)
target_link_libraries(sdurw_analytic_geometry.rwplugin PRIVATE sdurw_analytic_geometry PUBLIC sdurw)

if(CMAKE_VERSION VERSION_GREATER 3.3)
    set_target_properties(sdurw_analytic_geometry PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
