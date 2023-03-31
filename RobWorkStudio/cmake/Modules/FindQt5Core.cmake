
get_filename_component(MOD_ROOT ${CMAKE_CURRENT_LIST_FILE} PATH)
set(FIND_VERSION_MAJOR 5)
include("${MOD_ROOT}/Find_QT_DIR.cmake")
find_qt_package(Qt5Core)