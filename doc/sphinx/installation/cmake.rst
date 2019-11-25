CMake Options & Environment
=================================================================

It is possible to customize the RobWork installation through various CMake options.
RobWork has many dependencies, of which many are optional.
It is important to ensure that CMake finds the correct optional dependencies to enable certain parts of RobWork.
Here we will give an introduction to the CMake system and the most relevant options.
We will also provide some information about how to control where CMake searches for the dependencies.

.. warning::
    This page is still being written. The following sections will therefore include a non compleate list of CMake options

Common Options
--------------
- LIB_INSTALL_DIR=<Install_DIR>
    - Default: "lib"
    - Use this argument to specify where the libraries should be installed.

- CMAKE_BUILD_TYPE=[Release|None|Debug|RelWithDebInfo]
    - Default: "None"
    - This value specify what kind of build should be compiled.
      None means the compiler decides

- CMAKE_DISABLE_FIND_PACKAGE_<PackageName>=[True|False]
    - Default: False
    - CMake provides the option to disable using optional packages.
      For most of the packages disabling them this way will either result in removing packages/functionality
      or making sure that internal package of the same name will be compiled and used.
    - RobWork Optional packages
        - GLUT, FreeGLUT, XercesC, Yaobi, FCL, EIGEN3, SWIG, LUA51, LUA, ASSIMP, ZLIB, MINIZIP, PythonInterp, PythonLibs,
          GTest, Mathematica, OpenMP

- SWIG_EXECUTABLE=<path\to\swig.exe>
    - Default : <let CMake find the path>
    - Use this argument to specify which swig executable should be used.

Robwork Options
---------------

- BUILD_SHARED_LIBS=[ON|OFF]
    - Default: OFF
    - Used to build assimp and GTESTS as a shared library.
      It also builds zlib and lua as DLL on windows.

- RW_ENABLE_ASSERT=[ON|OFF]
    - Default: ON
    - Use this to disable/enable RW_ASSERT from generating output messages with regards to errors.
      Only works for non-debug builds.

- RW_USE_ASSIMP=[ON|OFF]
    - Default: NOT RW_DISABLE_ASSIMP
    - Use this or the disable argument (see default) to prevent RobWork from using Assimp.

- RW_USE_FCL=[ON|OFF]
    - Default: NOT RW_DISABLE_FCL
    - Use this or the disable argument (see default) to prevent RobWork from using FCL.

- RW_USE_YAOBI=[ON|OFF]
    - Default: NOT RW_DISABLE_YAOBI
    - Use this or the disable argument (see default) to prevent RobWork from using Yaobi.

- RW_USE_PQP=[ON|OFF]
    - Default: NOT RW_DISABLE_PQP
    - Use this or the disable argument (see default) to prevent RobWork from using PQP.

- RW_USE_LUA=[ON|OFF]
    - Default: NOT RW_DISABLE_LUA
    - Use this or the disable argument (see default) to prevent RobWork from using lua.

- RW_USE_GTEST=[ON|OFF]
    - Default: NOT RW_DISABLE_GTEST
    - Use this or the disable argument (see default) to prevent RobWork from using GTest.

- RW_USE_MATHEMATICA=[ON|OFF]
    - Default: RW_ENABLE_MATHEMATICA
    - Use this or the enable argument (see default) to prevent RobWork from using assimp.

- RW_IS_EXAMPLES_ENABLED=[ON|OFF]
    - Default: RW_BUILD_EXAMPLES
    - Use this or the enable argument (see default) to prevent RobWork from building the provided examples.

- RW_IS_TESTS_ENABLED=[ON|OFF]
    - Default: RW_BUILD_TESTS
    - Use this or the enable argument (see default) to prevent RobWork from building tests.

RobWorkStudio Options
---------------------

- RWS_SHARED_LIBS=[ON|OFF]
    - Default: OFF
    - Use this option to compile RobWorkStudio Libraries as shared libraries.

- RWS_USE_STATIC_LINK_COMPONENTS
    - Default: ON
    - This Options links the RWS components statically to RobWorkStudio.

- RWS_USE_STATIC_LINK_PLUGINS=OFF
    - Default: ON
    - This option creates the RobWorkStudio plugins as static plugins, loaded at compile time.

- USE_WERROR=[ON|OFF]
    - Default=OFF
    - Make all warnings during compilation appear as errors.


RobWorkSim Options
------------------
- RWSIM_SHARED_LIBS=ON
    - Default: OFF
    - Use this option to compile RobWorkStudio Libraries as shared libraries.

- USE_WERROR=[ON|OFF]
    - Default=OFF
    - Make all warnings during compilation appear as errors.

RobworkHardWare Options
-----------------------

- RWHW_SHARED_LIBS=[ON|OFF]
    - Default: OFF
    - Use this option to compile RobworkHardWare Libraries as shared libraries.
      When using ur_rtde this option should be use.

- USE_WERROR=[ON|OFF]
    - Default=OFF
    - Make all warnings during compilation appear as errors.

Package build control
---------------------
With the modular setup of RobWork it is possible to disable the build of the individual RobWork packages from CMake.
Use the following options to prevent the build of a package.
They all default to ON unless a required dependency isn't present.
Running CMake will show which packages are not being build and why.


**RobWork**

- BUILD_sdurw=[ON|OFF]
- BUILD_sdurw_algorithms=[ON|OFF]
- BUILD_sdurw_assembly=[ON|OFF]
- BUILD_sdurw_calibration=[ON|OFF]
- BUILD_sdurw_control=[ON|OFF]
- BUILD_sdurw_opengl=[ON|OFF]
- BUILD_sdurw_mathematica=[ON|OFF]
- BUILD_sdurw_proximitystrategies=[ON|OFF]
- BUILD_sdurw_pathoptimization=[ON|OFF]
- BUILD_sdurw_pathplanners=[ON|OFF]
- BUILD_sdurw_task=[ON|OFF]
- BUILD_sdurw_simulation=[ON|OFF]
- BUILD_sdurw_lua=[ON|OFF]
- BUILD_sdurw_python=[ON|OFF]
- BUILD_sdurw_java=[ON|OFF]
- BUILD_sdurw_softbody=[ON|OFF]
- BUILD_sdurw_csg=[ON|OFF]

**RobWorkStudio**

- BUILD_sdurws_atask=[ON|OFF]
- BUILD_sdurws_gtask=[ON|OFF]
- BUILD_sdurws_jog=[ON|OFF]
- BUILD_sdurws_log=[ON|OFF]
- BUILD_sdurws_playback=[ON|OFF]
- BUILD_sdurws_propertyview=[ON|OFF]
- BUILD_sdurws_treeview=[ON|OFF]
- BUILD_sdurws_planning=[ON|OFF]
- BUILD_sdurws_sensors=[ON|OFF]
- BUILD_sdurws_luaeditor=[ON|OFF]
- BUILD_sdurws_luapl=[ON|OFF]
- BUILD_sdurws_robworkstudioapp=[ON|OFF]
- BUILD_sdurws_lua=[ON|OFF]
- BUILD_sdurws_java=[ON|OFF]
- BUILD_sdurws_python=[ON|OFF]

**RobWorkSim**

- BUILD_sdurwsim_bullet=[ON|OFF]
- BUILD_sdurwsim_luai=[ON|OFF]
- BUILD_sdurwsim_java=[ON|OFF]
- BUILD_sdurwsim_python=[ON|OFF]

**RobworkHardWare**

- BUILD_camera=[ON|OFF]
- BUILD_CAN=[ON|OFF]
- BUILD_CRSA465=[ON|OFF]
- BUILD_dockwelder=[ON|OFF]
- BUILD_katana=[ON|OFF]
- BUILD_pa10=[ON|OFF]
- BUILD_pcube=[ON|OFF]
- BUILD_serialport=[ON|OFF]
- BUILD_swissranger=[ON|OFF]
- BUILD_tactile=[ON|OFF]
- BUILD_motomanIA20=[ON|OFF]
- BUILD_sdh=[ON|OFF]
- BUILD_universalrobots=[ON|OFF]
- BUILD_universalrobots_rtde=[ON|OFF]
- BUILD_fanucdriver=[ON|OFF]
- BUILD_schunkpg70=[ON|OFF]
- BUILD_netft=[ON|OFF]
- BUILD_trakstar=[ON|OFF]
- BUILD_robolabFT=[ON|OFF]
- BUILD_robotiq=[ON|OFF]
