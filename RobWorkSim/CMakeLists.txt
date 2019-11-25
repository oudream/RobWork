# ######################################################################################################################
# Test CMake version and CMAKE_BUILD_TYPE
cmake_minimum_required(VERSION 2.8.12)

if(POLICY CMP0072) # Introduce cmake 3.11
    cmake_policy(SET CMP0072 NEW)
endif()
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0078)
    cmake_policy(SET CMP0078 NEW)
endif()
if(POLICY CMP0086) # Introduce cmake 3.14
    cmake_policy(SET CMP0086 NEW)
endif()

# ######################################################################################################################
# The name of the project.
project(RobWorkSim)
if(DEFINED VERSION)
    set(ROBWORKSIM_VERSION ${VERSION})
    set(RWSIM_GOT_VERSION True)
else()
    set(ROBWORKSIM_VERSION 6.6.6)
    set(RWSIM_GOT_VERSION False)
endif()
set(RWSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "RobWorkSim version ${ROBWORKSIM_VERSION}")

# ######################################################################################################################
# lets start by finding RobWork since this is the primary dependency and because we want to use the macros available in
# RobWork if RW_ROOT is defined then we use that, else we try finding it the traditional way
if(RW_ROOT)
    set(RobWork_DIR ${RW_ROOT}/cmake)
else()
    # try finding it relative to this directory
    find_file(ROBWORKDEVEL_FOUND RobWorkSetup.cmake ${RWSIM_ROOT}/../RobWork/cmake NO_DEFAULT_PATH)
    if(ROBWORKDEVEL_FOUND)
        set(RobWork_DIR ${RWSIM_ROOT}/../RobWork/cmake)
    endif()
endif()
find_package(RobWork ${ROBWORKSIM_VERSION} REQUIRED)

# ######################################################################################################################
# include the macros from robwork
include(${RW_ROOT}/cmake/RobWorkMacros.cmake)

# for backward compatibility
set(ROOT RWSIM_ROOT)
rw_init_project(${RWSIM_ROOT} RobWorkSim RWSIM ${ROBWORKSIM_VERSION})
rw_get_os_info()
rw_set_install_dirs(RobWorkSim RWSIM)
rw_options(RWSIM)

# ######################################################################################################################
# Add an "uninstall" target configure_file("${RWSIM_ROOT}/cmake/uninstall_target.cmake.in"
# "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY) add_custom_target(uninstall "${CMAKE_COMMAND}" -P
# "${CMAKE_BINARY_DIR}/uninstall_target.cmake")

# ######################################################################################################################
# setup stuff for RobWorkStudio

# add the RWSIM modules to the module path
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${RWSIM_ROOT}/cmake/Modules/")

# This sets up ROBWORKSIM_INCLUDE_DIR and ROBWORKSIM_LIBRARIES RobWork dependencies are included in these vars
include(${RWSIM_ROOT}/cmake/RobWorkSimSetup.cmake)
rw_set_install_dirs(RobWorkSim RWSIM)

# now lets find RobWorkStudio

find_package(RobWorkStudio ${ROBWORKSIM_VERSION} REQUIRED)

# ######################################################################################################################
# CONFIGURATION configure build/RobWorkStudioConfig.cmake.in
#

# first configure the header file
configure_file(${RWSIM_ROOT}/src/RobWorkSimConfig.hpp.in "${RWSIM_ROOT}/src/RobWorkSimConfig.hpp" @ONLY)

# next build information script
configure_file(${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig.cmake.in
               "${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake" @ONLY)

# Configure build/RobWorkSimConfig.cmake.in such that other projects might use robworkstudio
configure_file(${RWSIM_ROOT}/cmake/RobWorkSimConfig.cmake.in "${RWSIM_ROOT}/cmake/RobWorkSimConfig.cmake" @ONLY)

# and the version info
configure_file(${RWSIM_ROOT}/cmake/RobWorkSimConfigVersion.cmake.in "${RWSIM_ROOT}/cmake/RobWorkSimConfigVersion.cmake"
               @ONLY)

include_directories(${ROBWORKSIM_INCLUDE_DIR})
link_directories(${ROBWORKSIM_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# Store the test results in a RWSimConfig.hpp file.
configure_file(${RWSIM_ROOT}/src/RWSimConfig.hpp.in ${RWSIM_ROOT}/src/RobWorkSimConfig.hpp)

configure_file(${RWSIM_ROOT}/src/rwsimlibs/gui/RWSimGuiConfig.hpp.in ${RWSIM_ROOT}/src/rwsimlibs/gui/RWSimGuiConfig.hpp)

# process sub directories add_subdirectory(ext)
add_subdirectory(src)

# ######################################################################################################################
# Installation stuff
#

# install configuration
install(
    FILES ${RWSIM_ROOT}/cmake/RobWorkSimConfigVersion.cmake ${RWSIM_ROOT}/cmake/RobWorkSimConfig.cmake
    DESTINATION ${RWSIM_INSTALL_DIR}/cmake
)

install(FILES "cmake/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake" DESTINATION ${RWSIM_INSTALL_DIR}/cmake)

# misc
install(FILES LICENSE.txt NOTICE.txt ChangeLog.txt DESTINATION "${RWSIM_INSTALL_DIR}/")

# ######################################################################################################################
# setup testing
#
configure_file("${RWSIM_ROOT}/cmake/CTestCustom.cmake.in" "CTestCustom.cmake")
# configure testing configuration Dashboard
configure_file("${RWSIM_ROOT}/CTestConfig.cmake.in" "${RWSIM_ROOT}/CTestConfig.cmake")

include(CMakeDependentOption)
cmake_dependent_option(RWSIM_IS_TESTS_ENABLED "Set when you want to build the tests" ON "${RWSIM_BUILD_TESTS}" OFF)
if(RWSIM_IS_TESTS_ENABLED)
    message(STATUS "RobWork tests ENABLED!")
    # TODO Specify SET(SITE "myname") to configure the site name to use when uploading
    set(BUILDNAME "${INFO}")
    enable_testing()
    include(CTest)
    add_subdirectory(test)
    if(RW_BUILD_WITH_GTEST)
        add_subdirectory(gtest)
    endif()
else()
    message(STATUS "RobWork tests DISABLED!")
endif()

configure_file(${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig.cmake.in
               "${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake")

# Packaging
include(cmake/packing.cmake)
