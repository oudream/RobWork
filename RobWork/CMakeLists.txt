# Test CMake version
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

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

# The name of the project.
project(RobWork)

# Setup RobWork Version
if(DEFINED VERSION)
    set(ROBWORK_VERSION ${VERSION})
    set(RW_GOT_VERSION True)
else()
    set(ROBWORK_VERSION 6.6.6)
    set(RW_GOT_VERSION False)
endif()

# set robwork root
set(RW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# guard against in-source builds
set(CMAKE_MODULE_PATH ${RW_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})
include(${RW_ROOT}/cmake/RobWorkMacros.cmake)

rw_init_project(${RW_ROOT} RobWork RW ${ROBWORK_VERSION})
rw_get_os_info()

rw_options(RW)

# Add an "uninstall" target
configure_file("${RW_ROOT}/cmake/uninstall_target.cmake.in"
               "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall_sdurw "${CMAKE_COMMAND}" -P
                  "${CMAKE_BINARY_DIR}/uninstall_target.cmake")

# This sets up ROBWORK_INCLUDE_DIR and ROBWORK_LIBRARIES
include(${RW_ROOT}/cmake/RobWorkSetup.cmake)

# Setup install Directories
rw_set_install_dirs(RobWork RW) # Must be called after setup
set(RW_EXT_INSTALL_DIR ${INCLUDE_INSTALL_DIR}/ext/)

# Subdirectories to process:
add_subdirectory(ext)
add_subdirectory(src)

rw_sys_info(INFO)
message(STATUS "RobWork: system id=${INFO} ")

# Compile the RobWork tools folder
add_subdirectory(tools)

# ##################################################################################################
# setup testing
#
configure_file("${RW_ROOT}/cmake/CTestCustom.cmake.in" "CTestCustom.cmake")
# configure testing configuration
configure_file("${RW_ROOT}/CTestConfig.cmake.in" "${RW_ROOT}/CTestConfig.cmake")

include(CMakeDependentOption)
cmake_dependent_option(RW_IS_TESTS_ENABLED "Set when you want to build the tests" ON
                       "${RW_BUILD_TESTS}" OFF)
if(RW_IS_TESTS_ENABLED)
    message(STATUS "RobWork tests ENABLED!")
    # TODO Specify SET(SITE "myname") to configure the site name to use when uploading
    set(BUILDNAME "${INFO}")
    enable_testing() # Should be unnecessary due to the CMake module CTest also calls this command
    include(CTest)
    add_subdirectory(test)
    if(RW_HAVE_GTEST)
        add_subdirectory(gtest)
    endif()
else()
    message(STATUS "RobWork tests DISABLED!")
endif()

# ##################################################################################################
# Test if we should compile the EXAMPLES
#
cmake_dependent_option(RW_IS_EXAMPLES_ENABLED "Set when you want to build the examples" ON
                       "${RW_BUILD_EXAMPLES}" OFF)
if(RW_IS_EXAMPLES_ENABLED)
    message(STATUS "RobWork examples ENABLED!")
    add_subdirectory(example)
else()
    message(STATUS "RobWork example DISABLED!")
endif()

# Try to find the current revision
rw_get_revision(${RW_ROOT} ROBWORK)

# Store the test results in a RobWorkConfig.hpp file.
configure_file(${RW_ROOT}/src/RobWorkConfig.hpp.in ${RW_ROOT}/src/RobWorkConfig.hpp)

# Configure cmake/RobWorkBuildConfig.cmake.in such that other projects might use RobWork

configure_file(${RW_ROOT}/cmake/RobWorkBuildConfig.cmake.in
               "${RW_ROOT}/cmake/RobWorkBuildConfig_${RW_BUILD_TYPE}.cmake" @ONLY)

# configure version information
configure_file(${RW_ROOT}/cmake/RobWorkConfigVersion.cmake.in
               "${RW_ROOT}/cmake/RobWorkConfigVersion.cmake" @ONLY)

configure_file(${RW_ROOT}/cmake/RobWorkConfig.cmake.in "${RW_ROOT}/cmake/RobWorkConfig.cmake" @ONLY)

export(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE "${RW_ROOT}/cmake/${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${PROJECT_PREFIX}::
)
if(RW_HAVE_GTEST)
    if(TARGET ${GTEST_LIBRARIES} AND TARGET ${GTEST_MAIN_LIBRARIES})
        export(
            TARGETS ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
            NAMESPACE ${PROJECT_PREFIX}::
            APPEND
            FILE "${RW_ROOT}/cmake/gtestTargets.cmake"
        )
    endif()
endif()

# ##################################################################################################
# Installation stuff
#

# configuration
install(
    FILES ${RW_ROOT}/cmake/RobWorkConfigVersion.cmake ${RW_ROOT}/cmake/RobWorkConfig.cmake
    DESTINATION ${RW_INSTALL_DIR}/cmake
    COMPONENT cmake
)

# Install stuff
install(FILES LICENSE.txt NOTICE.txt ChangeLog.txt DESTINATION "${RW_INSTALL_DIR}/" COMPONENT cmake)

install(
    FILES "cmake/RobWorkBuildConfig_${RW_BUILD_TYPE}.cmake" "cmake/RobWorkMacros.cmake"
    DESTINATION ${RW_INSTALL_DIR}/cmake
    COMPONENT cmake
)

install(
    FILES "${RW_ROOT}/src/RobWorkConfig.hpp"
    DESTINATION "${INCLUDE_INSTALL_DIR}"
    COMPONENT sdurw
)

install(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_PREFIX}::
    DESTINATION ${RW_INSTALL_DIR}/cmake
)

# Install dependencies on Windows
if(WIN32)
    # Trim the Boost library lists and take only the libraries corresponding to the build type
    set(BOOST_LIBRARIES_INSTALL "")
    foreach(s ${Boost_LIBRARIES})
        if("${s}" STREQUAL "optimized")

        elseif("${s}" STREQUAL "debug")

        else()
            get_filename_component(_dir "${s}" DIRECTORY )
            get_filename_component(_file "${s}" NAME_WLE )
            string(REGEX MATCH "[a-z]*_[a-z]*" _file ${_file})
            file(GLOB _files "${_dir}/${_file}*lib")
            foreach(file ${_files})
                list(APPEND BOOST_LIBRARIES_INSTALL "${file}")
            endforeach()
        endif()
    endforeach()

    # Install external libraries
    install(FILES ${BOOST_LIBRARIES_INSTALL} DESTINATION ${LIB_INSTALL_DIR} COMPONENT boost)
    install(
        DIRECTORY "${Boost_INCLUDE_DIR}/boost/"
        DESTINATION "${RW_INSTALL_DIR}/ext/boost/"
        COMPONENT boost
    )

    if(RW_HAVE_XERCES)
        install(FILES ${XERCESC_LIBRARIES} DESTINATION ${LIB_INSTALL_DIR} COMPONENT external)
        install(
            DIRECTORY "${XERCESC_INCLUDE_DIR}/"
            DESTINATION "${RW_INSTALL_DIR}/ext/xerces/"
            COMPONENT xerces
        )
    endif()

    install(
        DIRECTORY "${RW_ROOT}/ext/eigen3/Eigen/"
        DESTINATION "${RW_INSTALL_DIR}/ext/Eigen/"
        COMPONENT eigen
    )
endif()

# And if possible we would also like to install all the dll's and stuff that we use this will
# configure the dependency file that will be used if install is invoked
# GET_TARGET_PROPERTY(rw_executable_name loader-test LOCATION) # not allowed anymore - must use
# generator expressions instead
configure_file("${RW_ROOT}/cmake/dependencies.cmake.in" "${RW_ROOT}/cmake/dependencies.cmake.gen"
               @ONLY)

# Disabled generation of the dependencies file (problem with multi-configuration generators like
# Visual Studio) FILE(GENERATE OUTPUT "${RW_ROOT}/cmake/dependencies.cmake" INPUT
# "${RW_ROOT}/cmake/dependencies.cmake.gen" ) INSTALL(SCRIPT "${RW_ROOT}/cmake/dependencies.cmake")

# Install the example directories
install(
    DIRECTORY "${RW_ROOT}/example/"
    DESTINATION "${RW_INSTALL_DIR}/example/"
    COMPONENT example
    PATTERN ".svn" EXCLUDE
    PATTERN "example/CMakeLists.txt" EXCLUDE
    PATTERN "Build" EXCLUDE
)

# Install the API doc if doxygen has been run IF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")
# INSTALL(DIRECTORY "${RW_ROOT}/apidocs/html" DESTINATION "${RW_INSTALL_DIR}/apidocs" PATTERN ".svn"
# EXCLUDc ) ENDIF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")

# Documentation build target
find_program(doc_executable doxygen)
if(doc_executable)
    add_custom_target(doc doxygen WORKING_DIRECTORY ${RW_ROOT})
endif()

# Packaging
rw_create_installer()

string(REPLACE ";" "\n" TOFILE_DST "${ROBWORK_INCLUDE_DIR}")
file(WRITE "include_dirs" "${TOFILE_DST}")
