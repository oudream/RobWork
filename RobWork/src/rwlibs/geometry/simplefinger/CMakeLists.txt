RW_ADD_LIBRARY(SimpleFinger SimpleFinger SimpleFinger.cpp SimpleFinger.hpp)
TARGET_LINK_LIBRARIES(SimpleFinger PRIVATE sdurw_csg PUBLIC sdurw)

RW_ADD_LIBRARY(simplefinger.rwplugin SimpleFinger MODULE SimpleFingerPlugin.cpp SimpleFingerPlugin.hpp)
TARGET_LINK_LIBRARIES(simplefinger.rwplugin PRIVATE SimpleFinger PUBLIC sdurw)

IF(CMAKE_VERSION VERSION_GREATER 3.3)
	SET_TARGET_PROPERTIES(SimpleFinger PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
ENDIF()