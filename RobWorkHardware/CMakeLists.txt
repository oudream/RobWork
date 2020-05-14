# ##################################################################################################
# Test CMake version and CMAKE_BUILD_TYPE
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
set(CMAKE_CXX_STANDARD 11)
# ##################################################################################################
# The name of the project.
project(RobWorkHardware)

set(RWHW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(RobWork REQUIRED PATHS ${RWHW_ROOT}/../RobWork/cmake )
set(Boost_USE_STATIC_LIBS ${RW_BUILD_WITH_BOOST_USE_STATIC_LIB})

# ##################################################################################################
# include the macros from robwork
include(${RW_ROOT}/cmake/RobWorkMacros.cmake)

if(DEFINED VERSION)
    set(ROBWORKHARDWARE_VERSION ${VERSION})
    set(RWHW_GOT_VERSION True)
else()
    rw_get_git_version(ROBWORKHARDWARE_VERSION ROBWORKHARDWARE_BRANCH)
    set(RWHW_GOT_VERSION False)
endif()

rw_init_project(${RWHW_ROOT} RobWorkHardware RWHW ${ROBWORKHARDWARE_VERSION})
rw_get_os_info()
rw_set_install_dirs(RobWorkHardware RWHW)
rw_options(RWHW)

# ##################################################################################################
# Add an "uninstall" target
configure_file("${RWHW_ROOT}/cmake/uninstall_target.cmake.in"
               "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY
)
add_custom_target(uninstall_sdurwhw "${CMAKE_COMMAND}" -P
                                    "${CMAKE_BINARY_DIR}/uninstall_target.cmake"
)

# ##################################################################################################
# setup stuff for RobWorkHardware

# make sure that libraries can reach the cmake modules
set(CMAKE_MODULE_PATH ${RWHW_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})

# Subdirectories will add libraries to this variable
set(ROBWORKHARDWARE_LIBRARIES)

if(DEFINED USE_WERROR)
    if(${USE_WERROR})
        set(WERROR_FLAG "-Werror")
    endif()
endif()

# also make sure flags from robwork is also used in robworkhardware
set(RWHW_CXX_FLAGS
    "${WERROR_FLAG} ${RW_BUILD_WITH_CXX_FLAGS}"
    CACHE STRING "Change this to force using your own flags and not those of RobWorkHardware"
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWHW_CXX_FLAGS}")

# ##################################################################################################
# now add all source directories to process.
add_subdirectory(ext)
add_subdirectory(src)

# ##################################################################################################
# testing
rw_sys_info(INFO)
message(STATUS "RobWorkStudio: platform ${INFO} ")

configure_file("${RWHW_ROOT}/cmake/CTestCustom.cmake.in" "CTestCustom.cmake")
include(CMakeDependentOption)
cmake_dependent_option(
    RWHW_IS_TESTS_ENABLED "Set when you want to build the tests" ON "${RWHW_BUILD_TESTS}" OFF
)
if(RWHW_IS_TESTS_ENABLED)
    message(STATUS "RobWorkHardware: tests ENABLED!")
    include(CTest)
    add_subdirectory(test)
else()
    message(STATUS "RobWorkHardware: tests DISABLED!")
endif()

# ##################################################################################################
# Try to find the current revision
rw_get_revision(${RWHW_ROOT} ROBWORKHARDWARE)

# ##################################################################################################
# CONFIGURATION configure build/RobWorkHardwareConfig.cmake.in

# first configure the header file
configure_file(${RWHW_ROOT}/src/RobWorkHardwareConfig.hpp.in
               ${RWHW_ROOT}/src/RobWorkHardwareConfig.hpp
)

# next build information script make a list of all enabled components
set(RWHW_ENABLED_COMPONENTS "")
foreach(_name ${RW_SUBSYSTEMS})
    if(${BUILD_${_name}})
        list(APPEND RWHW_ENABLED_COMPONENTS ${_name})
    endif()
endforeach()
# TODO these depend on packages...
set(ROBWORKHARDWARE_LIBRARY_DIRS ${RWHW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(ROBWORKHARDWARE_INCLUDE_DIR "${RWHW_ROOT}/src ${RWHW_ROOT}/ext")
configure_file(${RWHW_ROOT}/cmake/RobWorkHardwareBuildConfig.cmake.in
               "${RWHW_ROOT}/cmake/RobWorkHardwareBuildConfig_${RW_BUILD_TYPE}.cmake" @ONLY
)

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use robworkstudio
configure_file(${RWHW_ROOT}/cmake/RobWorkHardwareConfig.cmake.in
               "${RWHW_ROOT}/cmake/RobWorkHardwareConfig.cmake" @ONLY
)

# and the version info
configure_file(${RWHW_ROOT}/cmake/RobWorkHardwareConfigVersion.cmake.in
               "${RWHW_ROOT}/cmake/RobWorkHardwareConfigVersion.cmake"
)

export(
    EXPORT ${PROJECT_PREFIX}Targets
    # FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake"
    FILE "${RWHW_ROOT}/cmake/${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${PROJECT_PREFIX}::
)

# ##################################################################################################
# Installation stuff
#

# configuration
install(FILES ${RWHW_ROOT}/cmake/RobWorkHardwareConfigVersion.cmake
              ${RWHW_ROOT}/cmake/RobWorkHardwareConfig.cmake DESTINATION ${RWHW_INSTALL_DIR}/cmake
)

install(FILES "cmake/RobWorkHardwareBuildConfig_${RW_BUILD_TYPE}.cmake"
        DESTINATION ${RWHW_INSTALL_DIR}/cmake
)

install(FILES "${RWHW_ROOT}/src/RobWorkHardwareConfig.hpp" DESTINATION "${INCLUDE_INSTALL_DIR}")

install(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_PREFIX}::
    DESTINATION ${RWHW_INSTALL_DIR}/cmake
)

message(STATUS "Install dir: ${RWHW_INSTALL_DIR} and ${INCLUDE_INSTALL_DIR}")

rw_create_installer()
