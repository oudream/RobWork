# Test CMake version
cmake_minimum_required(VERSION 3.5.1)

# The name of the project.
project(RobWorkProject)

set(RWP_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(RW_ROOT ${ROOT}/RobWork/)
set(RWS_ROOT ${ROOT}/RobWorkStudio/)
set(RWSIM_ROOT ${ROOT}/RobWorkSim/)
set(RWHW_ROOT ${ROOT}/RobWorkHardware/)

option(WITH_RWHW "Set true to enable compilation of RobWorkHardware!" TRUE)
option(WITH_RWS "Set true to enable compilation of RobWorkStudio!" TRUE)
option(WITH_RWSIM "Set true to enable compilation of RobWorkSim!" TRUE)

add_subdirectory(RobWork)
if(WITH_RWHW)
    add_subdirectory(RobWorkHardware)
endif()
if(WITH_RWS)
    add_subdirectory(RobWorkStudio)
endif()
if(WITH_RWSIM)
    add_subdirectory(RobWorkSim)
endif()

# #######################################################33 now configure documentation
configure_file(${ROOT}/doc/Doxyfile.in ${ROOT}/doc/Doxyfile)

find_program(doc_executable doxygen HINTS ${DOXYGEN_PATH} $ENV{DOXYGEN_PATH})
if(doc_executable)
    add_custom_target(sdurwdoc ${doc_executable} WORKING_DIRECTORY ${ROOT}/doc)
endif()

# Packaging INCLUDE(build/packing.cmake)

# ##################################################################################################
# Windows INSTALLER Packaging
#

set(SLASH "/")
if(DEFINED MSVC)
    set(SLASH "\\\\")
endif()

set(CPACK_PACKAGE_NAME "RobWork")
set(CPACK_PACKAGE_VENDOR "University of Southern Denmark")
set(
    CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "RobWork is a collection of C++ libraries for simulation and control of robot systems. RobWork is used for research and education as well as for practical robot applications."
)
set(CPACK_PACKAGE_HOMEPAGE_URL "robwork.dk")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "RobWork")
set(CPACK_PACKAGE_ICON "${RW_ROOT}${SLASH}cmake${SLASH}images${SLASH}rw_logo_48x48.ico")
set(CPACK_RESOURCE_FILE_LICENSE "${RWP_ROOT}${SLASH}LICENSE")
set(CPACK_PACKAGE_CONTACT "Kasper Høj Lorenzen (kalor@mmmi.sdu.dk)")
set(CPACK_NSIS_MODIFY_PATH ON)

# set version
if(DEFINED VERSION)
    set(CPACK_PACKAGE_VERSION ${VERSION})
else()
    set(CPACK_PACKAGE_VERSION 6.6.6)
endif()

string(REGEX MATCHALL "[0-9]+" VERSIONS_TMP ${CPACK_PACKAGE_VERSION})
list(GET VERSIONS_TMP 0 CPACK_PACKAGE_VERSION_MAJOR)
list(GET VERSIONS_TMP 1 CPACK_PACKAGE_VERSION_MINOR)
list(GET VERSIONS_TMP 2 CPACK_PACKAGE_VERSION_PATCH)

set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME})

# Define components and their display names
set(CPACK_COMPONENTS_ALL "") # install components

set(RW_SUBSYS_BUILD_unspecified TRUE)
foreach(_comp unspecified ${RW_SUBSYSTEMS} ${RWS_SUBSYSTEMS} ${RWHW_SUBSYSTEMS} ${RWSIM_SUBSYSTEMS})
    if(${RW_SUBSYS_BUILD_${_comp}})

        string(TOUPPER "${_comp}" _COMP)

        if(NOT "${_comp}" STREQUAL "unspecified")
            set(CPACK_COMPONENT_${_COMP}_DISPLAY_NAME "${_comp}")
            set(CPACK_COMPONENT_${_COMP}_DESCRIPTION ${RW_SUBSYS_DESC_${_comp}})
            string(TOLOWER "${RW_SUBSYS_PREFIX_${_comp}}" tmp)
            set(CPACK_COMPONENT_${_COMP}_GROUP sdu${tmp})

        else()
            set(CPACK_COMPONENT_${_COMP}_DISPLAY_NAME "miscellaneous")
            set(CPACK_COMPONENT_${_COMP}_DESCRIPTION "Other Components")
            set(CPACK_COMPONENT_${_COMP}_GROUP "Other")
        endif()
        set(CPACK_COMPONENT_${_COMP}_INSTALL_TYPES Full Developer)
        set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${_comp})
        message(STATUS "component: ${CPACK_COMPONENT_${_COMP}_DISPLAY_NAME} - group: ${CPACK_COMPONENT_${_COMP}_GROUP}")
    else()
        message(STATUS "Not build component ${_comp}")
    endif()
endforeach()

# Define dependencies between components. example headers dpend on lib
# set(CPACK_COMPONENT_HEADERS_DEPENDS libraries)

set(CPACK_COMPONENT_GROUP_SDURW_DESCRIPTION "Tools for controlling robots")
set(CPACK_COMPONENT_GROUP_SDURWS_DESCRIPTION "Tools for Visulizing robot control")
set(CPACK_COMPONENT_GROUP_SDURWHW_DESCRIPTION "Drivers for robot platforms")
set(CPACK_COMPONENT_GROUP_SDURWSIM_DESCRIPTION "Tools for simulating robots")
set(CPACK_COMPONENT_GROUP_UNSPECIFIED_DESCRIPTION "extra tools")

# Define NSIS installation types
set(CPACK_ALL_INSTALL_TYPES Full Developer)
set(CPACK_COMPONENT_UNSPECIFIED_INSTALL_TYPES Full Developer)

# Must be after the last CPACK macros
include(CPack)
