# Find and sets up RobWorkStudio.
#
# ROBWORKSTUDIO_INCLUDE_DIR - Where to find robwork include sub-directory. ROBWORKSTUDIO_LIBRARIES   - List of libraries
# when using RobWork (includes all libraries that RobWork depends on). ROBWORKSTUDIO_LIBRARY_DIRS - List of directories
# where libraries of RobWork are located. ROBWORKSTUDIO_FOUND       - True if RobWork was found. (not impl yet)
#
# RWS_ROOT             - If set this defines the root of ROBWORKSTUDIO if not set then it if possible be autodetected.
#

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

if(RWSTUDIO_ROOT)
    set(RWS_ROOT ${RWSTUDIO_ROOT})
endif()

# Check if RWstudio_ROOT path are setup correctly
find_file(ROBWORKSTUDIO_FOUND RobWorkStudioSetup.cmake ${RWSTUDIO_ROOT}/cmake ${RWS_ROOT}/cmake NO_DEFAULT_PATH)
if(NOT ROBWORKSTUDIO_FOUND)
    message(
        SEND_ERROR
            "RobWorkStudio: Path to RobWorkStudio root (RWSTUDIO_ROOT) is incorrectly setup! \nRWSTUDIO_ROOT == ${RWSTUDIO_ROOT}"
    )
endif()
message(STATUS "RobWorkStudio: ROOT dir: ${RWS_ROOT}")

#
# Setup the default include and library dirs for RobWorkStudio
#
# INCLUDE("${RWSTUDIO_ROOT}/cmake/RobWorkStudioBuildConfig${CMAKE_BUILD_TYPE}.cmake")

# ######################################################################################################################
# DEPENDENCIES - REQUIRED Check for all dependencies, this adds LIBRARY_DIRS and include dirs that the configuration
# depends on
#

find_package(PythonInterp 3 QUIET)
find_package(PythonLibs 3 QUIET)
if(PYTHONLIBS_FOUND)
    set(RWS_USE_PYTHON3 true)
    set(RWS_USE_PYTHON true)
else()
    find_package(PythonLibs QUIET)
    if(PYTHONLIBS_FOUND)
        set(RWS_USE_PYTHON2 true)
        set(RWS_USE_PYTHON true)
    endif()
endif()

if(NOT PYTHONINTERP_FOUND)
    find_package(PythonInterp QUIET)
endif()

if(PYTHONINTERP_FOUND AND PYTHONLIBS_FOUND)
    if(NOT (PYTHONLIBS_VERSION_STRING STREQUAL PYTHON_VERSION_STRING))
        string(ASCII 27 Esc)
        message(
            WARNING
                "${Esc}[33mMatching Versions of python intepretor and python library NOT FOUND. \r"
                "Found versions are python libs ${PYTHONLIBS_VERSION_STRING} and python intepretor ${PYTHON_VERSION_STRING}. \n"
                "This can be because you haven't installed python${PYTHON_VERSION_MAJOR}-dev package\n${Esc}[m"
        )
    endif()
endif()

if(PYTHONINTERP_FOUND)
    message(STATUS "Found Python interpreter ${PYTHON_VERSION_STRING}")
endif()
if(PYTHONLIBS_FOUND)
    message(STATUS "Found Python libraries ${PYTHONLIBS_VERSION_STRING}")
endif()

if(NOT PYTHON_LIBRARIES)
	set(PYTHON_LIBRARIES "")
endif()

# Find and setup OpenGL.
if(POLICY CMP0072) # Introduce cmake 3.11
    cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)

set(Boost_NO_BOOST_CMAKE TRUE) # From Boost 1.70, CMake files are provided by Boost - we are not yet ready to handle it
                               # And some extra packages for boost
unset(Boost_USE_STATIC_LIBS)
unset(Boost_FIND_QUIETLY)
if(DEFINED UNIX)
    find_package(Boost REQUIRED program_options)
elseif(DEFINED WIN32)
    set(Boost_USE_STATIC_LIBS ON)
    find_package(Boost COMPONENTS program_options)
    # If static libraries for Windows were not found, try searching again for the shared ones
    if(NOT Boost_PROGRAM_OPTIONS_FOUND)
        set(Boost_USE_STATIC_LIBS OFF)
        find_package(Boost REQUIRED program_options)
    endif()
endif()

# Find and setup Qt.
find_package(Qt5Core QUIET)
find_package(Qt5Gui QUIET)
find_package(Qt5Widgets QUIET)
find_package(Qt5OpenGL QUIET)
if(Qt5Core_FOUND AND Qt5Gui_FOUND AND Qt5Widgets_FOUND AND Qt5OpenGL_FOUND)
    set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
    message(STATUS "RobWorkStudio: Using Qt5.")
else()
    message(STATUS "RobWorkStudio: One or more Qt5 modules not found:")
    if(Qt5Core_FOUND)
        message(STATUS "RobWorkStudio: - Qt5Core found.")
    else()
        message(STATUS "RobWorkStudio: - Qt5Core NOT found. Please set Qt5Core_DIR to find.")
    endif()
    if(Qt5Gui_FOUND)
        message(STATUS "RobWorkStudio: - Qt5Gui found.")
    else()
        message(STATUS "RobWorkStudio: - Qt5Gui NOT found. Please set Qt5Gui_DIR to find.")
    endif()
    if(Qt5Widgets_FOUND)
        message(STATUS "RobWorkStudio: - Qt5Widgets found.")
    else()
        message(STATUS "RobWorkStudio: - Qt5Widgets NOT found. Please set Qt5Widgets_DIR to find.")
    endif()
    if(Qt5OpenGL_FOUND)
        message(STATUS "RobWorkStudio: - Qt5OpenGL found.")
    else()
        message(STATUS "RobWorkStudio: - Qt5OpenGL NOT found. Please set Qt5OpenGL_DIR to find.")
    endif()
    message(
        FATAL_ERROR
            "RobWorkStudio: Could NOT find Qt5. Please set the Qt5 directories."
    )
endif()

# ######################################################################################################################
# DEPENDENCIES - OPTIONAL these dependencies are optional, which is the user can switch off modules

set(RWS_HAVE_GLUT False)

find_package(GLUT QUIET)
if(NOT GLUT_FOUND) # Check if free glut exsist
    find_package(FreeGLUT QUIET)
    if(FreeGLUT_FOUND)
        set(GLUT_glut_LIBRARY FreeGLUT::freeglut)
        set(GLUT_FOUND ${FreeGLUT_FOUND})
    endif()
endif()
if(OPENGL_FOUND AND GLUT_FOUND)
    set(RWS_HAVE_GLUT True)
    message(STATUS "RobWork: OpenGL and GLUT ENABLED! FOUND!")
else()
    set(GLUT_glut_LIBRARY "")
    message(STATUS "RobWork: OpenGL and GLUT NOT FOUND! code disabled!")
endif()

# optional compilation of sandbox
if(RWS_BUILD_SANDBOX)
    message(STATUS "RobWorkStudio: Sandbox ENABLED!")
    set(SANDBOX_LIB "rsdurws_sandbox")
    set(RWS_HAVE_SANDBOX true)
else()
    message(STATUS "RobWorkStudio: Sandbox DISABLED!")
endif()

# Check if SWIG is available
if(RW_BUILD_WITH_SWIG AND NOT DEFINED SWIG_EXECUTABLE)
    set(SWIG_EXECUTABLE ${RW_BUILD_WITH_SWIG_CMD})
    set(SWIG_VERSION ${RW_BUILD_WITH_SWIG_VERSION})
endif()

find_package(SWIG 3.0.0 QUIET) # At least SWIG 3 to support C++11
if(SWIG_FOUND)
    message(STATUS "RobWorkStudio: SWIG ${SWIG_VERSION} found!")
else()
    message(STATUS "RobWorkStudio: SWIG 3+ not found!")
endif()

# optional compilation of LUA interface
include(CMakeDependentOption)
set(RWS_HAVE_LUA False)
cmake_dependent_option(RWS_DISABLE_LUA "Set when you want to disable lua!" OFF "RW_BUILD_WITH_LUA AND SWIG_FOUND" ON)
if(NOT RWS_DISABLE_LUA)
    if(NOT SWIG_FOUND)
        message(STATUS "RobWorkStudio: Lua DISABLED! - SWIG 3+ was not found!")
        set(RWS_HAVE_LUA False)
    elseif(RW_BUILD_WITH_LUA)
        message(STATUS "RobWorkStudio: Lua ENABLED!")
        set(RWS_LUA "sdurws_lua_s;sdurws_luaeditor")
        set(RWS_HAVE_LUA True)
    else()
        message(STATUS "RobWorkStudio: Lua DISABLED! - RobWork is NOT compiled with Lua support!")
        set(RWS_HAVE_LUA False)
    endif()
else()
    message(STATUS "RobWorkStudio: Lua DISABLED!")
endif()

# ######################################################################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Set extra compiler flags. The user should be able to change this. The compiler flags from RobWork are automatically
# set
#
rw_is_release(IS_RELEASE)

if(NOT DEFINED RWS_CXX_FLAGS)
    set(
        RWS_CXX_FLAGS
        "${RW_BUILD_WITH_CXX_FLAGS} ${RWS_CXX_FLAGS_TMP}"
        CACHE STRING "Change this to force using your own flags and not those of RobWorkSutdio"
    )
endif()

if(NOT DEFINED RWS_DEFINITIONS)
    if(${IS_RELEASE})
        set(RWS_DEFINITIONS_TMP "-DQT_NO_DEBUG")
    else()
        set(RWS_DEFINITIONS_TMP "-DQT_DEBUG")
    endif()

    set(
        RWS_DEFINITIONS
        "${RW_BUILD_WITH_DEFINITIONS};${RWS_DEFINITIONS_TMP}"
        CACHE STRING "Change this to force using your own definitions and not those of RobWorkSutdio"
    )
endif()

add_definitions(${RWS_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWS_CXX_FLAGS}")
message(STATUS "RobWorkStudio: Adding RWS CXX flags: ${RWS_CXX_FLAGS}")
message(STATUS "RobWorkStudio: Addubg RWS definitions: ${RWS_DEFINITIONS}")

#
# Set extra linker flags. The user should be able to change this. The linker flags from RobWork are automatically set.
#
if(DEFINED RWS_LINKER_FLAGS)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${RWS_LINKER_FLAGS}" CACHE STRING "" FORCE)
    if(WIN32)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${RWS_LINKER_FLAGS}" CACHE STRING "" FORCE)
    endif()

    message(STATUS "RobWorkStudio: Adding RWS linker flags: ${RWS_LINKER_FLAGS}")
endif()
# MESSAGE(STATUS "${RW_BUILD_WITH_CXX_FLAGS}") If we are using static linking
if(RWS_USE_STATIC_LINK_PLUGINS)
    message(STATUS "RobWorkStudio: Using static linking of default plugins!")
else()
    message(STATUS "RobWorkStudio: Using dynamic linking of default plugins!")
endif()

# ######################################################################################################################
# SETTING UP VARS here we setup the output variables
#

# Setup RobWorkStudio include and link directories
set(ROBWORKSTUDIO_INCLUDE_DIR ${RWS_ROOT}/src/)
set(ROBWORKSTUDIO_LIBRARY_DIRS ${RWS_LIBS_DIR})
#
# The include dirs
#
set(ROBWORKSTUDIO_INCLUDE_DIR ${RWS_ROOT}/src ${Boost_INCLUDE_DIR} ${ROBWORK_INCLUDE_DIRS}
                              ${RWS_ROOT}/ext/qtpropertybrowser/src/)

#
# The library dirs
#
set(ROBWORKSTUDIO_LIBRARY_DIRS ${Boost_LIBRARY_DIRS} ${ROBWORK_LIBRARY_DIRS} ${RWS_ROOT}/libs/${RWS_BUILD_TYPE})

#
# Setup the Library List here. We need to make sure the correct order is maintained which is crucial for some compilers.
#
set(
    ROBWORKSTUDIO_LIBRARIES
    sdurws_robworkstudioapp
    sdurws_workcelleditor
    ${RWS_SANDBOX}
    ${RWS_LUA}
    sdurws
    qtpropertybrowser
    ${ROBWORK_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENGL_LIBRARIES}
)
