# Find and sets up RobWorkSim.
#
# ROBWORKSIM_INCLUDE_DIR - Where to find robwork include sub-directory. ROBWORKSIM_LIBRARIES   -
# List of libraries when using RobWork (includes all libraries that RobWork depends on).
# ROBWORKSIM_LIBRARY_DIRS - List of directories where libraries of RobWork are located.
# ROBWORKSIM_FOUND       - True if RobWork was found. (not impl yet)
#
# RWSIM_ROOT             - If set this defines the root of ROBWORKSIM if not set then it if possible
# be autodetected.
#

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Check if RWstudio_ROOT path are setup correctly
find_file(ROBWORKSIM_FOUND RobWorkSimSetup.cmake ${RWSIM_ROOT}/cmake NO_DEFAULT_PATH)
if(NOT ROBWORKSIM_FOUND)
    message(
        SEND_ERROR
            "RobWorkSim: Path to RobWorkSim root (RWSIM_ROOT) is incorrectly setup! \nRWSIM_ROOT == ${RWSIM_ROOT}"
    )
endif()
message(STATUS "RobWorkSim: ROOT dir: ${RWSIM_ROOT}")

#
# Setup the default include and library dirs for RobWorkSim
#
# INCLUDE("${RWSIM_ROOT}/build/RobWorkSimBuildConfig${CMAKE_BUILD_TYPE}.cmake")

# ##################################################################################################
# DEPENDENCIES - REQUIRED Check for all dependencies, this adds LIBRARY_DIRS and include dirs that
# the configuration depends on
#
# Find Python Prefer Python 3
find_package(PythonInterp 3 QUIET)
find_package(PythonLibs 3 QUIET)

if(NOT PYTHONINTERP_FOUND)
    find_package(PythonInterp QUIET)
endif()
if(NOT PythonLibs)
    find_package(PythonLibs QUIET)
endif()

if(PYTHONINTERP_FOUND)
    message(STATUS "RobWorkSim: Python interpreter ${PYTHON_VERSION_STRING} Found")
endif()
if(PYTHONLIBS_FOUND)
    message(STATUS "RobWorkSim: Python libraries ${PYTHONLIBS_VERSION_STRING} Found")
endif()

if(NOT PYTHON_LIBRARIES)
    set(PYTHON_LIBRARIES "")
endif()

# Find and setup OpenGL.
if(POLICY CMP0072) # Introduce cmake 3.11
    cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)

# ##################################################################################################
# DEPENDENCIES - OPTIONAL these dependencies are optional, which is the user can switch off modules

# optional compilation of sandbox
if(RWSIM_BUILD_SANDBOX)
    message(STATUS "RobWorkSim: Sandbox ENABLED!")
    set(SANDBOX_LIB "sdurwsim_sandbox")
    set(RWSIM_HAVE_SANDBOX true)
else()
    message(STATUS "RobWorkSim: Sandbox DISABLED!")
    set(RWSIM_HAVE_SANDBOX false)
endif()

# Check if SWIG is available
if(RW_BUILD_WITH_SWIG AND NOT DEFINED SWIG_EXECUTABLE)
    set(SWIG_EXECUTABLE ${RW_BUILD_WITH_SWIG_CMD})
endif()
find_package(SWIG 3.0.0 QUIET) # At least SWIG 3 to support C++11
if(SWIG_FOUND)
    message(STATUS "RobWorkSim: SWIG ${SWIG_VERSION} found!")
else()
    message(STATUS "RobWorkSim: SWIG 3+ not found!")
endif()

include(CMakeDependentOption)
# optional compilation of sandbox
set(RWSIM_HAVE_LUA False)
cmake_dependent_option(RWSIM_DISABLE_LUA "Set when you want to disable lua!" OFF "RW_BUILD_WITH_LUA"
                       ON)
if(NOT RWSIM_DISABLE_LUA)
    if(NOT SWIG_FOUND)
        message(STATUS "RobWorkSim: Lua DISABLED! - SWIG 3+ was not found!")
        set(RWSIM_HAVE_LUA False)
    elseif(RW_BUILD_WITH_LUA)
        message(STATUS "RobWorkSim: Lua ENABLED!")
        set(RWSIM_LUA "rwsim_lua")
        set(RWSIM_HAVE_LUA True)
    else()
        message(
            STATUS
                "RobWorkSim: Lua DISABLED! - RobWork is NOT compiled with Lua support! Set RWSIM_DISABLE_LUA=ON"
        )
        set(RWSIM_HAVE_LUA False)
    endif()
else()
    message(STATUS "RobWorkSim: Lua DISABLED!")
endif()
# MESSAGE(" ${RWSIM_DISABLE_LUA} and ${RW_BUILD_WITH_LUA} ")

# test if Bullet exists
set(RWSIM_HAVE_BULLET False)
# OPTION(RWSIM_USE_BULLET "Set to ON if Bullet should be use. you may need to set BULLET_ROOT")
find_package(Bullet QUIET)
if(NOT DEFINED RWSIM_DISABLE_BULLET)
    set(RWSIM_DISABLE_BULLET OFF)
endif()
cmake_dependent_option(RWSIM_DISABLE_BULLET "Set when you want to disable Bullet!"
                       ${RWSIM_DISABLE_BULLET} "BULLET_FOUND" ON)
if(NOT RWSIM_DISABLE_BULLET)
    find_package(Bullet)
    if(BULLET_FOUND)
        set(RWSIM_HAVE_BULLET TRUE)
        # INCLUDE_DIRECTORIES( ${BULLET_INCLUDE_DIR} ${BULLET_ROOT}/Demos/)
        set(RWSIM_BULLET_LIBRARY sdurwsim_bullet ${BULLET_LIBRARIES})

        # BULLET_LIBRARIES
        message(STATUS "RobWorkSim: Bullet enabled and found.")
        set(RW_BULLET_INCLUDE_DIR ${BULLET_INCLUDE_DIR})
    else()
        set(RWSIM_HAVE_BULLET FALSE)
        message(
            SEND_ERROR
                "RobWorkSim: Bullet enabled but not found. Please setup BULLET_ROOT."
                ${RWSIM_USE_BULLET}
        )
    endif()
else()
    message(STATUS "RobWorkSim: Bullet disabled.")
endif()

# test if ODE exists
set(RWSIM_HAVE_ODE False)
find_package(ODE QUIET)
# MESSAGE("ODE_FOUND: ${ODE_FOUND} ")
cmake_dependent_option(RWSIM_DISABLE_ODE "Set when you want to disable ODE!" OFF
                       "${ODE_FOUND};${RW_BUILD_WITH_PQP}" ON)
# OPTION(RWSIM_USE_ODE "Set to ON if ODE should be use. you may need to set ODE_ROOT"
# ${RWSIM_USE_ODE})
if(NOT RWSIM_DISABLE_ODE)
    find_package(ODE)
    if(ODE_FOUND)
        if(RW_BUILD_WITH_PQP)
            set(RWSIM_HAVE_ODE TRUE)
            # INCLUDE_DIRECTORIES( ${ODE_INCLUDE_DIR} )
            set(RWSIM_ODE_LIBRARY sdurwsim_ode ${ODE_LIBRARIES})
            # ODE_LIBRARIES
            message(STATUS "RobWorkSim: ODE enabled and found. Using ${ODE_BUILD_WITH}")
            set(RW_ODE_INCLUDE_DIR ${ODE_INCLUDE_DIR})
        else()
            set(RWSIM_HAVE_ODE FALSE)
            message(
                SEND_ERROR
                    "RobWorkSim: ODE enabled but RobWork was not build with PQP. Please compile RobWork with PQP support."
            )
        endif()
    else()
        set(RWSIM_HAVE_ODE FALSE)
        message(SEND_ERROR "RobWorkSim: ODE enabled but not found. Please setup ODE_ROOT.")
    endif()
else()
    message(STATUS "RobWorkSim: ODE disabled.")
endif()

set(RWSIM_HAVE_RWPE False)
option(RWSIM_DISABLE_RWPE "Set when you want to disable RobWorkPhysicsEngine!" OFF)
if(NOT RWSIM_DISABLE_RWPE)
    set(RWSIM_HAVE_RWPE TRUE)
    set(RWSIM_RWPE_LIBRARY sdurwsim_rwpe)
    message(STATUS "RobWorkSim: RobWorkPhysicsEngine enabled.")
endif()

set(RWSIM_HAVE_RWPHYS False)
option(RWSIM_DISABLE_RWPHYS "Set when you want to disable RWPhysics engine!" OFF)
if(NOT RWSIM_DISABLE_RWPHYS)
    set(RWSIM_HAVE_RWPHYS TRUE)
    message(STATUS "RobWorkSim: RWPhysics enabled.")
endif()

# Add additional packages that are required by your project here
if(USE_OPENCV AND DEFINED OpenCV_ROOT_DIR)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ROOT}/cmake/Modules)

    set(OpenCV_FIND_REQUIRED_COMPONENTS CV CXCORE HIGHGUI)
    find_package(OpenCV REQUIRED "CV CXCORE HIGHGUI")
    include_directories(${OPENCV_INCLUDE_DIR})
    message(STATUS "RobWorkSim: USING OPENCV")
endif()

# ##################################################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Set extra compiler flags. The user should be able to change this. The compiler flags from RobWork
# are automatically set
#

if(DEFINED USE_WERROR)
    if(${USE_WERROR})
        set(WERROR_FLAG "-Werror")
    endif()
endif()

if(NOT DEFINED RWSIM_CXX_FLAGS)
    set(
        RWSIM_CXX_FLAGS
        "${RW_BUILD_WITH_CXX_FLAGS} ${RWSIM_CXX_FLAGS_TMP}"
        CACHE STRING "Change this to force using your own flags and not those of RobWorkSim"
    )
endif()
set(RWSIM_CXX_FLAGS "${RWSIM_CXX_FLAGS}${WERROR_FLAG}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWSIM_CXX_FLAGS}")
message(STATUS "RobWorkSim: Adding RWSIM CXX flags: ${RWSIM_CXX_FLAGS}")

#
# Set extra linker flags. The user should be able to change this. The linker flags from RobWork are
# automatically set.
#
if(DEFINED RWSIM_LINKER_FLAGS)
    set(
        CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} ${RWSIM_LINKER_FLAGS}"
        CACHE STRING "" FORCE
    )
    if(WIN32)
        set(
            CMAKE_EXE_LINKER_FLAGS
            "${CMAKE_EXE_LINKER_FLAGS} ${RWSIM_LINKER_FLAGS}"
            CACHE STRING "" FORCE
        )
    endif()

    message(STATUS "RobWorkSim: Adding RWSIM linker flags: ${RWSIM_LINKER_FLAGS}")
endif()

# ##################################################################################################
# SETTING UP VARS here we setup the output variables
#

# Setup RobWorkSim include and link directories
#
# The include dirs
#
set(ROBWORKSIM_INCLUDE_DIRS ${RWSIM_ROOT}/src ${RW_ODE_INCLUDE_DIR}
                            ${RW_BULLET_INCLUDE_DIR})
#
# The library dirs
#
set(ROBWORKSIM_LIBRARY_DIRS ${Boost_LIBRARY_DIRS} ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                            ${RWSIM_LIBRARY_OUT_DIR} ${RWSIM_ARCHIVE_OUT_DIR}
                            ${ROBWORK_LIBRARY_DIRS})

#
# Setup the Library List here. We need to make sure the correct order is maintained which is crucial
# for some compilers.
#
set(ROBWORKSIM_LIBRARIES ${RWSIM_SANDBOX} sdurwsim_bullet sdurwsim_ode sdurwsim_gui sdurwsim)

set(ROBWORKSIM_DEPEND ${BULLET_LIBRARIES} ${ODE_LIBRARIES})
