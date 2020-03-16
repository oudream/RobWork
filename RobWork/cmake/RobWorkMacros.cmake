#
# This is a collection of macros used throughout the robwork project
#

# ##################################################################################################
# Converts a standard cmake list to a python string list
#
macro(RW_TO_PYTHON_STR_LIST ITEMS OUTPUT)

    set(RESULT_STR "'")
    foreach(item ${ITEMS})
        set(RESULT_STR "${RESULT_STR}${item}','")
    endforeach()
    set(${OUTPUT} "${RESULT_STR}'")

endmacro()

# ##################################################################################################
# Converts a standard VERSION 0.1.2 to three version numbers
#
macro(RW_SPLIT_VERSION VERSION MAJOR MINOR PATCH)
    string(REGEX MATCHALL "[0-9]+" VERSIONS ${VERSION})
    list(GET VERSIONS 0 ${MAJOR})
    list(GET VERSIONS 1 ${MINOR})
    list(GET VERSIONS 2 ${PATCH})
endmacro()

# ##################################################################################################
# Get a string describing the current system, e.g. windows-mingw-x64, mac-x64 or ubuntu-11.04-x64
#
macro(RW_SYS_INFO INFO)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(ARCH "x86")
    else()
        set(ARCH "amd64")
    endif()

    # rehat: /etc/redhat-release Slackware: /etc/slackware-version Slamd64:   /etc/slamd64-version
    # Fedora: /etc/fedora-

    if(UNIX)
        if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            set(SUFFIX "mac-${ARCH}_${RW_BUILD_TYPE}")
        else()
            if(EXISTS "/etc/lsb-release")
                execute_process(COMMAND cat /etc/lsb-release OUTPUT_VARIABLE SUFFIX)
                string(REGEX MATCHALL "\".+\"" SUFFIX ${SUFFIX})
                # this will add kernel version eg Linux_3.0....
                set(SUFFIX "${SUFFIX}_${ARCH}_${RW_BUILD_TYPE}")
                string(REPLACE "\"" "" SUFFIX ${SUFFIX})
                string(REPLACE " " "_" SUFFIX ${SUFFIX})
            elseif(EXISTS "/etc/os-release")
                execute_process(COMMAND cat /etc/os-release OUTPUT_VARIABLE SUFFIX)
                string(REGEX MATCHALL "[^_]ID=\"?[^(\n)]+" SUFFIX1 ${SUFFIX})
                string(REPLACE "ID=" "" SUFFIX1 ${SUFFIX1})
                string(REPLACE "\"" "" SUFFIX1 ${SUFFIX1})
                string(REGEX MATCHALL "VERSION_ID=\"[^\"]+\"" SUFFIX2 ${SUFFIX})
                string(LENGTH ${SUFFIX2} SUFFIX2_LEN)
                math(EXPR SUFFIX2_LEN "${SUFFIX2_LEN}-13")
                string(SUBSTRING ${SUFFIX2} 12 ${SUFFIX2_LEN} SUFFIX2)
                # this will add kernel version eg Linux_3.0....
                set(SUFFIX "${SUFFIX1}-${SUFFIX2}-${ARCH}_${RW_BUILD_TYPE}")
                string(REPLACE "\"" "" SUFFIX ${SUFFIX})
                string(REPLACE " " "_" SUFFIX ${SUFFIX})
            elseif(EXISTS "/etc/redhat-release")
                set(SUFFIX "redhat-${ARCH}_${RW_BUILD_TYPE}")
            elseif(EXISTS "/etc/slackware-version")
                set(SUFFIX "slackware-${ARCH}_${RW_BUILD_TYPE}")
            elseif(EXISTS "/etc/fedora-release")
                set(SUFFIX "fedora-${ARCH}_${RW_BUILD_TYPE}")
            else()
                # this will make it lowercase
                set(SUFFIX "linux-${ARCH}")
            endif()
        endif()
    elseif(MINGW)
        set(SUFFIX "windows-mingw-${ARCH}")
    elseif(MSVC)
        if(MSVC80)
            set(SUFFIX "windows-msvc2005-${ARCH}")
        elseif(MSVC90)
            set(SUFFIX "windows-msvc2008-${ARCH}")
        elseif(MSVC10)
            set(SUFFIX "windows-msvc2010-${ARCH}")
        endif()
    else()
        # Trouble

    endif()
    set(${INFO} ${SUFFIX})
endmacro()

# ##################################################################################################
# Try to find the revision, first from Git, then from SVN
#
macro(RW_GET_REVISION DIR PREFIX)
    find_package(Git QUIET)
    if(Git_FOUND)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} describe --dirty --always
            WORKING_DIRECTORY ${DIR}
            OUTPUT_VARIABLE ${PREFIX}_WC_INFO
            RESULT_VARIABLE Git_info_result
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(NOT ${Git_info_result} EQUAL 0)
            message(
                STATUS
                    "Does not appear to be from Git repository. Command \"${GIT_EXECUTABLE} -C ${DIR} describe --dirty --always\" failed."
            )

            # Try to find Subversion revision
            find_package(Subversion QUIET)
            if(Subversion_FOUND)
                # Subversion_WC_INFO(${DIR} RobWork)

                set(_Subversion_SAVED_LC_ALL "$ENV{LC_ALL}")
                set(ENV{LC_ALL} C)

                execute_process(
                    COMMAND ${Subversion_SVN_EXECUTABLE} info ${DIR}
                    OUTPUT_VARIABLE ${PREFIX}_WC_INFO
                    ERROR_VARIABLE Subversion_svn_info_error
                    RESULT_VARIABLE Subversion_svn_info_result
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                )

                if(NOT ${Subversion_svn_info_result} EQUAL 0)
                    message(
                        STATUS
                            "Does not appear to be from SVN repository. Command \"${Subversion_SVN_EXECUTABLE} info ${DIR}\" failed."
                    )
                    # with output:\n${Subversion_svn_info_error}
                else()
                    string(
                        REGEX
                        REPLACE
                            "^(.*\n)?Revision: ([^\n]+).*" "\\2" ${PREFIX}_WC_REVISION
                            "${${PREFIX}_WC_INFO}"
                    )
                endif()

                # restore the previous LC_ALL
                set(ENV{LC_ALL} ${_Subversion_SAVED_LC_ALL})

                message(STATUS "Current revision is ${${PREFIX}_WC_REVISION}")
                set(${PREFIX}_REVISION ${${PREFIX}_WC_REVISION})
            endif(Subversion_FOUND)
        else()
            set(${PREFIX}_WC_REVISION ${${PREFIX}_WC_INFO})
            set(${PREFIX}_REVISION ${${PREFIX}_WC_REVISION})
            message(STATUS "Current Git revision is ${${PREFIX}_REVISION}")
        endif()
    endif()
endmacro()

# ##################################################################################################
# This is a default project setup. It enables multiple build trees for multiple configuration eg.
# CMAKE_BUILD_TYPE
#
# input: ROOT : root of the project folder. if not defined then it will be defined as PROJECT_NAME:
# name of project, something like RobWork or RobWorkStudio, MyProject, PREFIX: for RobWork its RW,
# for RobWorkStudio its RWS. It will be used as suffix to project specific paths
#
# defines : ${PREFIX}_CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PREFIX}_CMAKE_ARCHIVE_OUTPUT_DIRECTORY
# ${PREFIX}_CMAKE_LIBRARY_OUTPUT_DIRECTORY and sets up cmake variables
macro(RW_INIT_PROJECT ROOT PROJECT_NAME PREFIX VERSION)
    # MESSAGE("ROOOT, ${ROOT} ${PROJECT_NAME} ${PREFIX}") Allow the syntax else (), endif (), etc.
    set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

    # Enable new linker path policy.
    if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
    endif()

    # OPTION(RW_VERBOSE "Set to true if cmake build information should be printet!" False)

    # Specify wether to default compile in Release, Debug, MinSizeRel, RelWithDebInfo mode
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE None CACHE STRING "Choose the type of build,
      options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
      RelWithDebInfo MinSizeRel.")
        set(
            ${PREFIX}_BUILD_TYPE
            "None"
            CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE
        )
    else()
        # we need to force the right configuration
        string(TOLOWER ${CMAKE_BUILD_TYPE} TMP_BUILD_TYPE)
        if(${TMP_BUILD_TYPE} STREQUAL "release")
            set(
                ${PREFIX}_BUILD_TYPE
                "Release"
                CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE
            )
        elseif(${TMP_BUILD_TYPE} STREQUAL "debug")
            set(
                ${PREFIX}_BUILD_TYPE
                "Debug"
                CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE
            )
        elseif(${TMP_BUILD_TYPE} STREQUAL "relwithdebinfo")
            set(
                ${PREFIX}_BUILD_TYPE
                "RelWithDebInfo"
                CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE
            )
        elseif(${TMP_BUILD_TYPE} STREQUAL "minsizerel")
            set(
                ${PREFIX}_BUILD_TYPE
                "MinSizeRel"
                CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE
            )
        elseif(${TMP_BUILD_TYPE} STREQUAL "none")
            set(
                ${PREFIX}_BUILD_TYPE
                "None"
                CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE
            )
        else()
            message(
                FATAL_ERROR
                    "Build type: ${CMAKE_BUILD_TYPE} not supported! please select one of: Release, Debug, RelWithDebInfo, MinSizeRel"
            )
        endif()

    endif()

    string(TOLOWER ${${PREFIX}_BUILD_TYPE} ${PREFIX}_BUILD_TYPE)
    message(STATUS "${PROJECT_NAME}: Build configuration: ${${PREFIX}_BUILD_TYPE}")

    # Load the optional Default.cmake file.
    include(${ROOT}/config.cmake OPTIONAL)
    if(NOT EXISTS ${ROOT}/config.cmake)
        if(EXISTS ${ROOT}/config.cmake.template)
            # Setup the default settings in case no RobWork.cmake exist.
            include(${ROOT}/config.cmake.template)
            # MESSAGE(STATUS "Using default settings from config.cmake.template")
        endif()
    endif()

    set(
        ${PREFIX}_CMAKE_RUNTIME_OUTPUT_DIRECTORY
        "${ROOT}/bin/${${PREFIX}_BUILD_TYPE}"
        CACHE PATH "Runtime directory" FORCE
    )
    set(
        ${PREFIX}_CMAKE_LIBRARY_OUTPUT_DIRECTORY
        "${ROOT}/libs/${${PREFIX}_BUILD_TYPE}"
        CACHE PATH "Library directory" FORCE
    )
    set(
        ${PREFIX}_CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        "${ROOT}/libs/${${PREFIX}_BUILD_TYPE}"
        CACHE PATH "Archive directory" FORCE
    )

    # Output goes to bin/<CONFIG> and libs/<CONFIG> unless specified otherwise by the user.
    if(DEFINED MSVC)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
    else()
        set(
            CMAKE_RUNTIME_OUTPUT_DIRECTORY
            "${ROOT}/bin/${${PREFIX}_BUILD_TYPE}"
            CACHE PATH "Runtime directory" FORCE
        )
        set(
            CMAKE_LIBRARY_OUTPUT_DIRECTORY
            "${ROOT}/libs/${${PREFIX}_BUILD_TYPE}"
            CACHE PATH "Library directory" FORCE
        )
        set(
            CMAKE_ARCHIVE_OUTPUT_DIRECTORY
            "${ROOT}/libs/${${PREFIX}_BUILD_TYPE}"
            CACHE PATH "Archive directory" FORCE
        )
    endif()

    string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UP)
    # MESSAGE("uppercase ${PROJECT_NAME_UP}_VERSION")

    if(${${PREFIX}_GOT_VERSION})
        set(PROJECT_USE_SONAME True)
    else()
        set(PROJECT_USE_SONAME False)
    endif()

    set(${PROJECT_NAME_UP}_VERSION ${VERSION} CACHE STRING "Project Version Nr" FORCE)
    string(REGEX MATCHALL "[0-9]+" ${PROJECT_NAME_UP}_VERSIONS ${VERSION})
    list(GET ${PROJECT_NAME_UP}_VERSIONS 0 ${PROJECT_NAME_UP}_VERSION_MAJOR)
    list(GET ${PROJECT_NAME_UP}_VERSIONS 1 ${PROJECT_NAME_UP}_VERSION_MINOR)
    list(GET ${PROJECT_NAME_UP}_VERSIONS 2 ${PROJECT_NAME_UP}_VERSION_PATCH)
    set(PROJECT_VERSION ${${PROJECT_NAME_UP}_VERSION})
    set(PROJECT_VERSION_MAJOR ${${PROJECT_NAME_UP}_VERSION_MAJOR})
    set(PROJECT_VERSION_MINOR ${${PROJECT_NAME_UP}_VERSION_MINOR})
    set(PROJECT_VERSION_PATCH ${${PROJECT_NAME_UP}_VERSION_PATCH})
    message(STATUS "${PROJECT_NAME}: Version ${${PROJECT_NAME_UP}_VERSION}")

    set(PROJECT_PREFIX ${PREFIX} CACHE INTERNAL "Current project PREFIX" FORCE)
    set(${PROJECT_PREFIX}_SUBSYSTEMS "" CACHE INTERNAL "Internal list of subsystems" FORCE)
    # setup install directories
endmacro()

macro(RW_GET_OS_INFO)
    # Get the compiler architecture
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(AMD64 TRUE)
    else()
        set(AMD64 FALSE)
    endif()
endmacro()

# ##################################################################################################
# Set the destination directories for installing stuff. input: PREFIX: project prefix id Sets
# LIB_INSTALL_DIR. Install libraries here. Sets BIN_INSTALL_DIR. Install binaries here. Sets
# INCLUDE_INSTALL_DIR. Install include files here, preferably in a
macro(RW_SET_INSTALL_DIRS PROJECT_NAME PREFIX)
    string(TOLOWER ${PREFIX} PREFIX_LOWER)
    string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
    string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
    if(NOT DEFINED LIB_INSTALL_DIR)
        set(LIB_INSTALL_DIR "lib")
    endif()
    set(
        INCLUDE_INSTALL_ROOT
        "include/${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}"
    )
    set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_ROOT}")
    set(EXT_INSTALL_DIR ${INCLUDE_INSTALL_DIR}/ext/)
    if(NOT DEFINED BIN_INSTALL_DIR)
        set(BIN_INSTALL_DIR "bin")
    endif()
    set(PKGCFG_INSTALL_DIR "${LIB_INSTALL_DIR}/pkgconfig")

    if(NOT DEFINED LUA_INSTALL_DIR AND LUA_FOUND)
        set(LUA_INSTALL_DIR "${LIB_INSTALL_DIR}/lua/${LUA_VERSION_MAJOR}.${LUA_VERSION_MINOR}")
    elseif(NOT DEFINED LUA_INSTALL_DIR AND RW_BUILD_WITH_LUA)
        set(LUA_INSTALL_DIR "${LIB_INSTALL_DIR}/lua/${RW_BUILD_WITH_LUA_VERSION}")
    else()
        set(LUA_INSTALL_DIR "${LIB_INSTALL_DIR}/lua/RobWork")
    endif()

    set(RW_PLUGIN_INSTALL_DIR "${LIB_INSTALL_DIR}/RobWork/rwplugins")
    set(RWS_PLUGIN_INSTALL_DIR "${LIB_INSTALL_DIR}/RobWork/rwsplugins")
    set(STATIC_LIB_INSTALL_DIR "${LIB_INSTALL_DIR}/RobWork")
    set(JAVA_INSTALL_DIR "${LIB_INSTALL_DIR}/RobWork/Java")

    execute_process(
        COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
        OUTPUT_VARIABLE PYTHON_INSTALL_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if("${PYTHON_INSTALL_DIR}" STREQUAL "")
        set(PYTHON_INSTALL_DIR "${LIB_INSTALL_DIR}/RobWork/Python")
    endif()

    if(WIN32)
        set(
            ${PREFIX}_INSTALL_DIR
            "${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}"
        )
        set(
            CONFIG_INSTALL_DIR
            "${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}/cmake"
        )
    else()
        set(
            ${PREFIX}_INSTALL_DIR
            "share/${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}"
        )
        set(
            CONFIG_INSTALL_DIR
            "share/${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}"
        )
    endif()
endmacro()

macro(RW_IS_RELEASE IS_RELEASE)
    if(
        ${RW_BUILD_TYPE} STREQUAL "release"
        OR ${RW_BUILD_TYPE} STREQUAL "relwithdebinfo"
        OR ${RW_BUILD_TYPE} STREQUAL "minsizerel"
    )
        set(${IS_RELEASE} TRUE)
    else()
        set(${IS_RELEASE} FALSE)
    endif()
endmacro()

macro(RW_OPTIONS PREFIX)
    # Build shared libraries by default.
    if(NOT DEFINED ${PREFIX}_SHARED_LIBS)
        set(${PREFIX}_SHARED_LIBS OFF)
    endif()

    if(POLICY CMP0077) # Introduce cmake 3.13
        cmake_policy(SET CMP0077 OLD)
    endif()

    option(${PREFIX}_SHARED_LIBS "Build shared libraries." ${${PREFIX}_SHARED_LIBS})
    if(${PREFIX}_SHARED_LIBS)
        set(PROJECT_LIB_PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX})
        set(PROJECT_LIB_SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX})
        set(PROJECT_LIB_TYPE "SHARED")
    else()
        set(PROJECT_LIB_PREFIX ${CMAKE_STATIC_LIBRARY_PREFIX})
        set(PROJECT_LIB_SUFFIX ${CMAKE_STATIC_LIBRARY_SUFFIX})
        set(PROJECT_LIB_TYPE "STATIC")
    endif()
    mark_as_advanced(${PREFIX}_SHARED_LIBS)
endmacro()

# ##################################################################################################
# Add a set of include files to install. _component The part of RW that the install files belong to.
# _subdir The sub- directory for these include files. ARGN The include files.
macro(RW_ADD_INCLUDES _component _subdir)
    install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir} COMPONENT ${_component})
endmacro()

# ##################################################################################################
# Add a set of include files to install. _component The part of RW that the install files belong to.
# _subdir The sub- directory for these include files. ARGN The include files.
macro(RW_ADD_INCLUDE_DIRS _component _subdir)
    install(
        DIRECTORY ${ARGN}
        DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir}
        COMPONENT ${_component}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
    )
endmacro()

# ##################################################################################################
# Add a library target. _name The library name. _component The part of RW that this library belongs
# to. ARGN The source files for the library.
macro(RW_ADD_LIBRARY _name)
    set(options STATIC SHARED MODULE) # Used to marke flags
    set(oneValueArgs COMPONENT) # used to marke values with a single value
    set(multiValueArgs)

    cmake_parse_arguments(SUBSYS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(_component ${_name})
    if(NOT "${SUBSYS_COMPONENT}" STREQUAL "")
        set(_component ${SUBSYS_COMPONENT})
    endif()

    set(LIB_TYPE ${PROJECT_LIB_TYPE})
    if(SUBSYS_STATIC)
        set(LIB_TYPE STATIC)
    elseif(SUBSYS_SHARED)
        set(LIB_TYPE SHARED)
    elseif(SUBSYS_MODULE)
        set(LIB_TYPE MODULE)
    endif()

    add_library(${_name} ${LIB_TYPE} ${SUBSYS_UNPARSED_ARGUMENTS})
    add_library(${PROJECT_PREFIX}::${_name} ALIAS ${_name})

    # Only link if needed
    if(WIN32 AND MSVC)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed,--no-undefined)
    endif()
    #
    if(${PROJECT_USE_SONAME})
        set_target_properties(
            ${_name}
            PROPERTIES
                VERSION ${PROJECT_VERSION} SOVERSION
                ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()
    
    install(
        TARGETS ${_name}
        EXPORT ${PROJECT_PREFIX}Targets
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
    )

endmacro()


# ######################################################################################################################
# Add a library target. _name The library name. _component The part of RW that this plugin belongs to. ARGN The source
# files for the library.
macro(RW_ADD_PLUGIN _name _lib_type )
    add_library(${_name} ${_lib_type} ${ARGN})

    # Only link if needed
    if(WIN32 AND MSVC)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed,--no-undefined)
    endif()
    #
    if(${PROJECT_USE_SONAME})
        set_target_properties(
            ${_name}
            PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()

    install(TARGETS ${_name} DESTINATION ${RW_PLUGIN_INSTALL_DIR} COMPONENT rwplugin)

endmacro()

# ######################################################################################################################
# Set a value in a global, cached map. _map The map name. _key The key name. _value The value.
macro(SET_IN_GLOBAL_MAP _map _key _value)
    set("${_map}_${_key}" "${_value}" CACHE INTERNAL "Map value" FORCE)
endmacro()

# ##################################################################################################
# Get a value from a map. _dest The name of the variable to store the value in. _map The map name.
# _key The key name.
macro(GET_IN_MAP _dest _map _key)
    set(${_dest} ${${_map}_${_key}})
endmacro()

# ##################################################################################################
# Make one subsystem depend on one or more other subsystems, and disable it if they are not being
# built. * _var The cumulative build variable. This will be set to FALSE if the dependencies are not
# met. * _name The name of the subsystem. * ARGN The subsystems and external libraries to depend on.
macro(RW_SUBSYS_DEPEND _name)
    if(${ARGC} GREATER 1)
        set_in_global_map(RW_SUBSYS_DEPEND ${_name} ${ARGN})
    else()
        set_in_global_map(RW_SUBSYS_DEPEND ${_name} "")
    endif()
endmacro()

# ##################################################################################################
# Set the include directory name of a subsystem. _name Subsystem name. _includedir Name of
# subdirectory for includes ARGN[0] Reason for not building.
macro(RW_SET_SUBSYS_INCLUDE_DIR _name _includedir)
    set_in_global_map(RW_SUBSYS_INCLUDE ${_name} ${_includedir})
endmacro()

# ##################################################################################################
# Get the include directory name of a subsystem - return _name if not set _var Destination variable.
# _name Name of the subsystem.
macro(RW_GET_SUBSYS_INCLUDE_DIR _var _name)
    get_in_map(${_var} RW_SUBSYS_INCLUDE ${_name})
    if(NOT ${_var})
        set(${_var} ${_name})
    endif(NOT ${_var})
endmacro()

# ##################################################################################################
# Register a subsystem. _name Subsystem name. _desc Description of the subsystem
macro(RW_ADD_SUBSYSTEM _name _desc)
    set(_temp ${${PROJECT_PREFIX}_SUBSYSTEMS})
    list(APPEND _temp ${_name})
    set(${PROJECT_PREFIX}_SUBSYSTEMS ${_temp} CACHE INTERNAL "Internal list of subsystems" FORCE)
    set_in_global_map(RW_SUBSYS_DESC ${_name} ${_desc})
    set_in_global_map(RW_SUBSYS_PREFIX ${_name} ${PROJECT_PREFIX})
endmacro()

# ##################################################################################################
# Add an option to build a subsystem or not. _var The name of the variable to store the option in.
# _name The name of the option's target subsystem. _desc The description of the subsystem. _default
# The default value (TRUE or FALSE) ARGV5 The reason for disabling if the default is FALSE.
macro(RW_SUBSYS_OPTION _var _name _desc _default)

    set(options ADD_DOC) # Used to marke flags
    set(oneValueArgs REASON) # used to marke values with a single value
    set(multiValueArgs DEPENDS DEPENDS_EXT)
    cmake_parse_arguments(SUBSYS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(_opt_name "BUILD_${_name}")
    rw_get_subsys_hyperstatus(subsys_status ${_name})
    if(NOT ("${subsys_status}" STREQUAL "AUTO_OFF"))

        RW_IS_TARGETS(_status_depend TARGETS ${SUBSYS_DEPENDS} NOT_TARGETS_OUT _un_med_depend)

        option(${_opt_name} ${_desc} ${_default})
        if(NOT ${_default} AND NOT ${_opt_name})
            set(${_var} FALSE)

            if("${SUBSYS_REASON}" STREQUAL "")
                set(_reason "Disabled by default.")
            else()
                set(_reason ${SUBSYS_REASON})
            endif()

            rw_set_subsys_status(${_name} FALSE ${_reason})
            message(STATUS "${_opt_name}  ${BUILD_${_name}} : ${_reason}")
            rw_disable_dependies(${_name})
        elseif(NOT ${_opt_name})
            set(${_var} FALSE)
            rw_set_subsys_status(${_name} FALSE "Disabled manually.")
            message(STATUS "${_opt_name}  ${BUILD_${_name}} : Disabled manually.")
            rw_disable_dependies(${_name})
        elseif(NOT ${_status_depend})
            set(${_var} FALSE)
            rw_set_subsys_status(${_name} FALSE "Unmet Dependencies: ${_un_med_depend}")
            set(BUILD_${_name} OFF)
            message(STATUS "${_opt_name}  ${BUILD_${_name}} : Unmet Dependencies: ${_un_med_depend}")
            rw_disable_dependies(${_name})
        else()
            set(${_var} TRUE)
            if("${SUBSYS_REASON}" STREQUAL "")
                set(_reason "Enabled by default.")
            else()
                set(_reason ${SUBSYS_REASON})
            endif()

            rw_set_subsys_status(${_name} TRUE ${_reason})
            message(STATUS "${_opt_name}  ${BUILD_${_name}} : ${_reason}")
            rw_enable_dependies(${_name})
        endif()
    endif(NOT ("${subsys_status}" STREQUAL "AUTO_OFF"))

    

    if(${SUBSYS_ADD_DOC})
        rw_add_doc(${_name})
    endif()
    set_in_global_map(RW_SUBSYS_BUILD ${_name} ${${_var}})
    rw_subsys_depend(${_name} ${SUBSYS_DEPENDS})
    rw_add_subsystem(${_name} ${_desc})
endmacro()

# ##################################################################################################
# Check if a list of targets are acually targets

macro(RW_IS_TARGETS _status)
    set(options) # Used to marke flags
    set(oneValueArgs TARGETS_OUT NOT_TARGETS_OUT) # used to marke values with a single value
    set(multiValueArgs TARGETS )
    cmake_parse_arguments(IS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(${_status} TRUE)

    foreach(rw_target ${IS_TARGETS})
        if(TARGET ${rw_target})
            list(APPEND ${IS_TARGETS_OUT} ${rw_target})
        else()
            list(APPEND ${IS_NOT_TARGETS_OUT} ${rw_target})
            set(${_status} FALSE)
        endif()
        
    endforeach()
    
endmacro()


# ##################################################################################################
# Macro to disable subsystem dependies _subsys IN subsystem name
macro(RW_DISABLE_DEPENDIES _subsys)
    string(TOUPPER "sdurw_${_subsys}_dependies" RW_SUBSYS_DEPENDIES)
    if(NOT ("${${RW_SUBSYS_DEPENDIES}}" STREQUAL ""))
        foreach(dep ${${RW_SUBSYS_DEPENDIES}})
            rw_set_subsys_hyperstatus(${_subsys} ${dep} AUTO_OFF "Automatically disabled.")
            set(BUILD_${dep} OFF CACHE BOOL "Automatically disabled ${dep}" FORCE)
        endforeach(dep)
    endif(NOT ("${${RW_SUBSYS_DEPENDIES}}" STREQUAL ""))
endmacro()

# ##################################################################################################
# Macro to enable subsystem dependies _subsys IN subsystem name
macro(RW_ENABLE_DEPENDIES _subsys)
    string(TOUPPER "sdurw_${_subsys}_dependies" RW_SUBSYS_DEPENDIES)
    if(NOT ("${${RW_SUBSYS_DEPENDIES}}" STREQUAL ""))
        foreach(dep ${${RW_SUBSYS_DEPENDIES}})
            rw_get_subsys_hyperstatus(dependee_status ${_subsys} ${dep})
            if("${dependee_status}" STREQUAL "AUTO_OFF")
                rw_set_subsys_hyperstatus(${_subsys} ${dep} AUTO_ON)
                get_in_map(desc RW_SUBSYS_DESC ${dep})
                set(BUILD_${dep} ON CACHE BOOL "${desc}" FORCE)
            endif("${dependee_status}" STREQUAL "AUTO_OFF")
        endforeach(dep)
    endif(NOT ("${${RW_SUBSYS_DEPENDIES}}" STREQUAL ""))
endmacro()

# ##################################################################################################
# Get the status of a subsystem _var Destination variable. _name Name of the subsystem.
macro(RW_GET_SUBSYS_STATUS _var _name)
    get_in_map(${_var} RW_SUBSYS_STATUS ${_name})
endmacro()

# ##################################################################################################
# Set the status of a subsystem. _name Subsystem name. _status TRUE if being built, FALSE otherwise.
# ARGN[0] Reason for not building.
macro(RW_SET_SUBSYS_STATUS _name _status)
    if(${ARGC} EQUAL 3)
        set(_reason ${ARGV2})
    else(${ARGC} EQUAL 3)
        set(_reason "No reason")
    endif(${ARGC} EQUAL 3)
    set_in_global_map(RW_SUBSYS_STATUS ${_name} ${_status})
    set_in_global_map(RW_SUBSYS_REASONS ${_name} ${_reason})
endmacro()

# ##################################################################################################
# Set the hyperstatus of a subsystem and its dependee _name Subsystem name. _dependee Dependant
# subsystem. _status AUTO_OFF to disable AUTO_ON to enable ARGN[0] Reason for not building.
macro(RW_SET_SUBSYS_HYPERSTATUS _name _dependee _status)
    set_in_global_map(RW_SUBSYS_HYPERSTATUS ${_name}_${_dependee} ${_status})
    if(${ARGC} EQUAL 4)
        set_in_global_map(RW_SUBSYS_REASONS ${_dependee} ${ARGV3})
    endif()
endmacro()

# ##################################################################################################
# Get the hyperstatus of a subsystem and its dependee _name IN subsystem name. _dependee IN
# dependant subsystem. _var OUT hyperstatus ARGN[0] Reason for not building.
macro(RW_GET_SUBSYS_HYPERSTATUS _var _name)
    set(${_var} "AUTO_ON")
    if(${ARGC} EQUAL 3)
        get_in_map(${_var} RW_SUBSYS_HYPERSTATUS ${_name}_${ARGV2})
    else()
        foreach(subsys ${RW_SUBSYS_DEPS_${_name}})
            if("${RW_SUBSYS_HYPERSTATUS_${subsys}_${_name}}" STREQUAL "AUTO_OFF")
                set(${_var} "AUTO_OFF")
                break()
            endif("${RW_SUBSYS_HYPERSTATUS_${subsys}_${_name}}" STREQUAL "AUTO_OFF")
        endforeach(subsys)
    endif()
endmacro()

# ##################################################################################################
# Macro to build subsystem centric documentation _subsys IN the name of the subsystem to generate
# documentation for
macro(RW_ADD_DOC _subsys)
    string(TOUPPER "${_subsys}" SUBSYS)
    set(doc_subsys "doc_${_subsys}")
    get_in_map(dependencies RW_SUBSYS_DEPS ${_subsys})
    if(DOXYGEN_FOUND)
        if(HTML_HELP_COMPILER)
            set(DOCUMENTATION_HTML_HELP YES)
        else(HTML_HELP_COMPILER)
            set(DOCUMENTATION_HTML_HELP NO)
        endif(HTML_HELP_COMPILER)
        if(DOXYGEN_DOT_EXECUTABLE)
            set(HAVE_DOT YES)
        else(DOXYGEN_DOT_EXECUTABLE)
            set(HAVE_DOT NO)
        endif(DOXYGEN_DOT_EXECUTABLE)
        if(NOT "${dependencies}" STREQUAL "")
            set(STRIPPED_HEADERS "${RW_SOURCE_DIR}/${dependencies}/include")
            string(
                REPLACE
                    ";" "/include \\\n\t\t\t\t\t\t\t\t\t\t\t\t ${RW_SOURCE_DIR}/" STRIPPED_HEADERS
                    "${STRIPPED_HEADERS}"
            )
        endif(NOT "${dependencies}" STREQUAL "")
        set(DOC_SOURCE_DIR "\"${CMAKE_CURRENT_SOURCE_DIR}\"\\")
        foreach(dep ${dependencies})
            set(
                DOC_SOURCE_DIR
                "${DOC_SOURCE_DIR}\n\t\t\t\t\t\t\t\t\t\t\t\t \"${RW_SOURCE_DIR}/${dep}\"\\"
            )
        endforeach(dep)
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
        set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
        configure_file("${RW_SOURCE_DIR}/doc/doxygen/doxyfile.in" ${doxyfile})
        add_custom_target(${doc_subsys} ${DOXYGEN_EXECUTABLE} ${doxyfile})
        if(USE_PROJECT_FOLDERS)
            set_target_properties(${doc_subsys} PROPERTIES FOLDER "Documentation")
        endif(USE_PROJECT_FOLDERS)
    endif()
endmacro()

# ##################################################################################################
# Use this macro to generate a windows installer
macro(RW_CREATE_INSTALLER)
    if(DEFINED MSVC)
        set(SLASH "\\\\")

        #### Description and names ########
        set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
        set(CPACK_PACKAGE_VENDOR "University of Southern Denmark")
        set(
            CPACK_PACKAGE_DESCRIPTION_SUMMARY
            "RobWork is a collection of C++ libraries for simulation and control of robot systems. RobWork is used for research and education as well as for practical robot applications."
        )
        set(CPACK_PACKAGE_HOMEPAGE_URL "robwork.dk")
        set(CPACK_PACKAGE_INSTALL_DIRECTORY "RobWork${SLASH}${PROJECT_PREFIX}")
        set(CPACK_RESOURCE_FILE_LICENSE "${${PROJECT_PREFIX}_ROOT}${SLASH}..${SLASH}LICENSE")
        set(CPACK_PACKAGE_CONTACT "Kasper Høj Lorenzen (kalor@mmmi.sdu.dk)")

        #### NSIS setup ####

        set(CPACK_NSIS_DISPLAY_NAME "${PROJECT_NAME}")
        set(CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME}")
        set(CPACK_NSIS_HELP_LINK ${CPACK_PACKAGE_HOMEPAGE_URL})
        set(CPACK_NSIS_URL_INFO_ABOUT ${CPACK_PACKAGE_HOMEPAGE_URL})
        set(CPACK_NSIS_MUI_ICON "${${PROJECT_PREFIX}_ROOT}${SLASH}..${SLASH}RobWork${SLASH}cmake${SLASH}images${SLASH}rw_logo_48x48.ico")
        set(CPACK_NSIS_MUI_UNWELCOMEFINISHPAGE_BITMAP "${${PROJECT_PREFIX}_ROOT}${SLASH}..${SLASH}RobWork${SLASH}cmake${SLASH}images${SLASH}rw_logo_128x64.bmp")
        set(CPACK_NSIS_MUI_WELCOMEFINISHPAGE_BITMAP ${CPACK_NSIS_MUI_UNWELCOMEFINISHPAGE_BITMAP})
        set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON) #When installing on a previus instalation ask to uninstall first
        set(CPACK_NSIS_UNINSTALL_NAME "${PROJECT_PREFIX}_uninstall")
        set(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})

        #### add to path option #####
        set(CPACK_NSIS_MODIFY_PATH ON) #Add the binary folder to PATH
        set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS #let cmake find robwork
            "WriteRegStr HKCU \\\"Software${SLASH}Kitware${SLASH}CMake${SLASH}Packages${SLASH}${PROJECT_NAME}\\\" \\\"Location\\\" \\\"$INSTDIR\\\""
            "WriteRegStr HKLM \\\"Software${SLASH}Kitware${SLASH}CMake${SLASH}Packages${SLASH}${PROJECT_NAME}\\\" \\\"Location\\\" \\\"$INSTDIR\\\""
        )
        set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS #clean up when uninstalling robwork
            "DeleteRegKey HKCU \\\"Software${SLASH}Kitware${SLASH}CMake${SLASH}Packages${SLASH}${PROJECT_NAME}\\\""
            "DeleteRegKey HKLM \\\"Software${SLASH}Kitware${SLASH}CMake${SLASH}Packages${SLASH}${PROJECT_NAME}\\\""
        )
        string (REPLACE ";" "\n" CPACK_NSIS_EXTRA_INSTALL_COMMANDS "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}")
        string (REPLACE ";" "\n" CPACK_NSIS_EXTRA_UNSTALL_COMMANDS "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}")

        #### set version #####
        if(DEFINED VERSION)
            set(CPACK_PACKAGE_VERSION ${VERSION})
        else()
            set(CPACK_PACKAGE_VERSION 6.6.6)
        endif()

        string(REGEX MATCHALL "[0-9]+" VERSIONS_TMP ${CPACK_PACKAGE_VERSION})
        list(GET VERSIONS_TMP 0 CPACK_PACKAGE_VERSION_MAJOR)
        list(GET VERSIONS_TMP 1 CPACK_PACKAGE_VERSION_MINOR)
        list(GET VERSIONS_TMP 2 CPACK_PACKAGE_VERSION_PATCH)

        set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION})

        get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS) #Get all components
        list(REMOVE_ITEM CPACK_COMPONENTS_ALL "pkgconfig" "rwtest") #Remove unnessesary components
        include(CPack)
        #### Setup Install Groupes ####
        cpack_add_component_group(RW
            DISPLAY_NAME "RobWork"
            DESCRIPTION "Tools for controlling robots"
            EXPANDED 
            BOLD_TITLE
        )
        cpack_add_component_group(RWS
            DISPLAY_NAME "RobWorkStudio"
            DESCRIPTION "Tools for Visulizing robot control"
            EXPANDED 
            BOLD_TITLE
        )                        
        cpack_add_component_group(RWHW
            DISPLAY_NAME "RobWorkHardware"
            DESCRIPTION "Tools for Visulizing robot control"
            EXPANDED 
            BOLD_TITLE
        )
        cpack_add_component_group(RWSIM
            DISPLAY_NAME "RobWorkSim"
            DESCRIPTION "Tools for simulating robots"
            EXPANDED 
            BOLD_TITLE
        )
        cpack_add_component_group(MISC
            DISPLAY_NAME "Miscellaneous"
            DESCRIPTION "Theses are Other Packages"
        )
        cpack_add_component_group(DEP
            DISPLAY_NAME "Dependencies"
            DESCRIPTION "Theses are external dependencies for the RobWork packages"
        )
        cpack_add_component_group(DEVEL
            DISPLAY_NAME "Development Files"
            DESCRIPTION "Theses are packages needed when you want to use robwork for development"
        )

        #### Setup Install Types ####
        cpack_add_install_type(Dev DISPLAY_NAME "Devel")
        cpack_add_install_type(Full DISPLAY_NAME "Full")
        cpack_add_install_type(RW_i DISPLAY_NAME "RobWork")
        cpack_add_install_type(RWS_i DISPLAY_NAME "RobWorkStudio")
        cpack_add_install_type(RWHW_i DISPLAY_NAME "RobWorkHardware")
        cpack_add_install_type(RWSIM_i DISPLAY_NAME "RobWorkSim")    
       
        
        #### Setup Install Components
        set(SPECIAL_COMPONENTS rwplugin cmake example swig rwtest)
        set(EXTERNAL_COMPONENTS yaobi pqp eigen zlib xerces boost lua fcl assimp sdurw_csgjs)
        set(BLOCKED_COMPONENTS pkgconfig rwtest)

        foreach(_comp ${CPACK_COMPONENTS_ALL})
            string(TOUPPER "${_comp}" _COMP)

            if(${RW_SUBSYS_BUILD_${_comp}})
                cpack_add_component(${_comp}
                    DISPLAY_NAME "${_comp}"
                    DESCRIPTION "${RW_SUBSYS_DESC_${_comp}}"
                    GROUP "${RW_SUBSYS_PREFIX_${_comp}}"
                    DEPENDS "${RW_SUBSYS_DEPEND_${_comp}}"
                    INSTALL_TYPES ${RW_SUBSYS_PREFIX_${_comp}}_i Full Dev 
                    #DOWNLOADED
                    #ARCHIVE_FILE #Name_of_file_to_generate_for_download
                )
                #message(STATUS "component: ${CPACK_COMPONENT_${_COMP}_DISPLAY_NAME} - group: ${CPACK_COMPONENT_${_COMP}_GROUP}")
            elseif(NOT ${RW_SUBSYS_BUILD_${_comp}})
                #message(STATUS "Component: ${_comp} not installed")
            elseif(${_comp} IN_LIST EXTERNAL_COMPONENTS)
                cpack_add_component(${_comp}
                    DISPLAY_NAME "${_comp}"
                    DESCRIPTION "RobWorkDependencie"
                    GROUP DEP
                    INSTALL_TYPES Full Dev 
                    #DOWNLOADED
                    #ARCHIVE_FILE #Name_of_file_to_generate_for_download
                )
            else()
                cpack_add_component(${_comp}
                    DISPLAY_NAME "${_comp}"
                    GROUP MISC
                    INSTALL_TYPES Full Dev
                    #DOWNLOADED
                    #ARCHIVE_FILE #Name_of_file_to_generate_for_download
                )
                #message(STATUS "from: ${_COMP} - component: ${CPACK_COMPONENT_${_COMP}_DISPLAY_NAME} - group: ${CPACK_COMPONENT_${_COMP}_GROUP}")
            endif()
        endforeach()
    endif()
endmacro()
