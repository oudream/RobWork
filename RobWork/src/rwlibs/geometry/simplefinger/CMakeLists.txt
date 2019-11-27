rw_add_library(SimpleFinger SimpleFinger SimpleFinger.cpp SimpleFinger.hpp)
target_link_libraries(SimpleFinger PRIVATE sdurw_csg PUBLIC sdurw)

rw_add_plugin(simplefinger.rwplugin SimpleFinger MODULE SimpleFingerPlugin.cpp SimpleFingerPlugin.hpp)
target_link_libraries(simplefinger.rwplugin PRIVATE SimpleFinger PUBLIC sdurw)

if(CMAKE_VERSION VERSION_GREATER 3.3)
    set_target_properties(SimpleFinger PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
