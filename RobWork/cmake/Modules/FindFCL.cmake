# Find FCL using pkg-config files
include(FindPackageHandleStandardArgs)
find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(FCL fcl)
endif()
IF(FCL_FOUND)
    SET(FCL_INCLUDE_DIRS ${FCL_INCLUDEDIR})
ELSE(FCL_FOUND)
  SET( FCL_INCLUDE_DIRS )
  SET( FCL_LIBRARY_DIRS )
  SET( FCL_LIBRARIES )
  SET( FCL_VERSION )
ENDIF(FCL_FOUND)
find_package_handle_standard_args(FCL
    FOUND_VAR FCL_FOUND
    REQUIRED_VARS FCL_LIBRARIES FCL_INCLUDE_DIRS FCL_LIBRARY_DIRS
    VERSION_VAR FCL_VERSION
)
