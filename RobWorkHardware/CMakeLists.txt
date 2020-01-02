###########################################
# Test CMake version and CMAKE_BUILD_TYPE
cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()


###########################################
# The name of the project.
project(RobWorkHardware)
if(DEFINED VERSION)
  set(ROBWORKHARDWARE_VERSION ${VERSION})
  set(RWHW_GOT_VERSION True)
else()
  set(ROBWORKHARDWARE_VERSION 6.6.6)
  set(RWHW_GOT_VERSION False)
endif()

SET(RWHW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

###########################################
# lets start by finding RobWork since this is the primary dependency and because we
# want to use the macros available in RobWork
# if RW_ROOT is defined then we use that, else we try finding it the traditional way
IF(RW_ROOT)
    SET(RobWork_DIR ${RW_ROOT}/cmake)
ELSE()
    # try finding it relative to this directory
    FIND_FILE(ROBWORKDEVEL_FOUND RobWorkSetup.cmake 
        ${RWHW_ROOT}/../RobWork/cmake NO_DEFAULT_PATH)
    IF(ROBWORKDEVEL_FOUND)
        SET(RW_ROOT ${RWHW_ROOT}/../RobWork/)
        SET(RobWork_DIR ${RW_ROOT}/cmake)
    ENDIF()
ENDIF()
FIND_PACKAGE(RobWork ${ROBWORKHARDWARE_VERSION} REQUIRED)

###########################################
# include the macros from robwork
INCLUDE(${RW_ROOT}/cmake/RobWorkMacros.cmake)
#INCLUDE(${RWS_ROOT}/cmake/RobWorkStudioMacros.cmake)

RW_INIT_PROJECT(${RWHW_ROOT} RobWorkHardware RWHW ${ROBWORKHARDWARE_VERSION})
RW_GET_OS_INFO()
RW_SET_INSTALL_DIRS(RobWorkHardware RWHW)
if( NOT MSVC )
    set(RWHW_SHARED_LIBS ON)
endif()
RW_OPTIONS(RWHW)

###########################################
# Add an "uninstall" target
CONFIGURE_FILE("${RWHW_ROOT}/cmake/uninstall_target.cmake.in"
               "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall_sdurwhw "${CMAKE_COMMAND}" -P
                  "${CMAKE_BINARY_DIR}/uninstall_target.cmake")

###########################################
# setup stuff for RobWorkHardware

# make sure that libraries can reach the cmake modules
SET(CMAKE_MODULE_PATH ${RWHW_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})

# Subdirectories will add libraries to this variable
set(ROBWORKHARDWARE_LIBRARIES)

if( DEFINED USE_WERROR)
  set(WERROR_FLAG "-Werror")
endif()

# also make sure flags from robwork is also used in robworkhardware
SET(RWHW_CXX_FLAGS "${WERROR_FLAG} ${RW_BUILD_WITH_CXX_FLAGS}" 
	CACHE STRING "Change this to force using your own flags and not those of RobWorkHardware"
)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWHW_CXX_FLAGS}")

# setup general include and link stuff
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
# if we have any exe files or plugins that depend on the libraries
INCLUDE_DIRECTORIES( src )
LINK_DIRECTORIES(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# Include directories.
INCLUDE_DIRECTORIES(
  ${ROOT}/ext
  ${ROOT}/src
)

###########################################
# now add all source directories to process.
add_subdirectory(ext)
add_subdirectory(src)
#add_subdirectory(test CMakeTmp/test)
add_subdirectory(example)

###########################################
# testing 
RW_SYS_INFO(INFO)
MESSAGE(STATUS "RobWorkStudio: platform ${INFO} ")

CONFIGURE_FILE("${RWHW_ROOT}/cmake/CTestCustom.cmake.in" "CTestCustom.cmake")
INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(RWHW_IS_TESTS_ENABLED "Set when you want to build the tests" 
                   ON "${RWHW_BUILD_TESTS}" OFF)
IF( RWS_IS_TESTS_ENABLED )
    MESSAGE(STATUS "RobWorkHardware: tests ENABLED!")
    INCLUDE(CTest)
    ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "RobWorkHardware: tests DISABLED!")
ENDIF()

###############################################################
# Try to find the current revision
RW_GET_REVISION(${RWHW_ROOT} ROBWORKHARDWARE)

###############################################################
# CONFIGURATION
# configure build/RobWorkHardwareConfig.cmake.in 


# first configure the header file
CONFIGURE_FILE(
  ${RWHW_ROOT}/src/RobWorkHardwareConfig.hpp.in
  ${RWHW_ROOT}/src/RobWorkHardwareConfig.hpp)

# next build information script
# make a list of all enabled components 
SET( RWHW_ENABLED_COMPONENTS "")
foreach(_name ${RW_SUBSYSTEMS})
IF( ${BUILD_${_name}} )
    LIST(APPEND RWHW_ENABLED_COMPONENTS ${_name})
ENDIF()
endforeach() 
# TODO these depend on packages...
SET(ROBWORKHARDWARE_LIBRARY_DIRS ${RWHW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
SET(ROBWORKHARDWARE_INCLUDE_DIR "${RWHW_ROOT}/src ${RWHW_ROOT}/ext")
CONFIGURE_FILE(
  ${RWHW_ROOT}/cmake/RobWorkHardwareBuildConfig.cmake.in
  "${RWHW_ROOT}/cmake/RobWorkHardwareBuildConfig_${RW_BUILD_TYPE}.cmake"
  @ONLY
)

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use robworkstudio
CONFIGURE_FILE(
  ${RWHW_ROOT}/cmake/RobWorkHardwareConfig.cmake.in
  "${RWHW_ROOT}/cmake/RobWorkHardwareConfig.cmake" @ONLY)

# and the version info
CONFIGURE_FILE(
  ${RWHW_ROOT}/cmake/RobWorkHardwareConfigVersion.cmake.in
  "${RWHW_ROOT}/cmake/RobWorkHardwareConfigVersion.cmake"
)



##################################################################
# Installation stuff
#

# configuration
INSTALL(FILES 
      ${RWHW_ROOT}/cmake/RobWorkHardwareConfigVersion.cmake
      ${RWHW_ROOT}/cmake/RobWorkHardwareConfig.cmake
      DESTINATION ${RWHW_INSTALL_DIR}/cmake ) 

INSTALL(FILES
      "cmake/RobWorkHardwareBuildConfig_${RW_BUILD_TYPE}.cmake"
      DESTINATION ${RWHW_INSTALL_DIR}/cmake)
    
INSTALL(FILES "${RWHW_ROOT}/src/RobWorkHardwareConfig.hpp"
      DESTINATION "${INCLUDE_INSTALL_DIR}")

message(STATUS "Install dir: ${RWHW_INSTALL_DIR} and ${INCLUDE_INSTALL_DIR}")

rw_create_installer()