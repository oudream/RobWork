CMake Options & Environment
=================================================================

It is possible to customize the RobWork installation through various CMake options.
RobWork has many dependencies, of which many are optional. It is important to ensure that CMake finds the correct optional dependencies to enable certain parts of RobWork. 
Here we will give an introduction to the CMake system and the most relevant options.
We will also provide some information about how to control where CMake searches for the dependencies.

.. warning::
    This page is still being written. The following sections will therefore include a non compleate list of cmake options

Common Options
--------------
- LIB_INSTALL_DIR=/usr/lib/$(DEB_TARGET_MULTIARCH)
- CMAKE_BUILD_TYPE=[Release|None|Debug|RelWithDebInfo]

Robwork Options
---------------

- BUILD_SHARED_LIBS=ON #Used to build zlib, assimp as shared lib
- CMAKE_DISABLE_FIND_PACKAGE_SWIG=True
- CMAKE_DISABLE_FIND_PACKAGE_XercesC=True
- RW_ENABLE_ASSERT=ON
- RW_USE_ASSIMP=OFF
    - RW_DISABLE_ASSIMP=ON
- RW_USE_FCL=OFF
    - RW_DISABLE_FCL=ON
- RW_USE_YAOBI=OFF
    - RW_DISABLE_YAOBI=ON
- RW_USE_PQP=OFF
    - RW_DISABLE_PQP=ON
- RW_USE_LUA=OFF
    - RW_DISBLE_LUA=ON
- RW_USE_GTEST=OFF
    - RW_DISABLE_GTEST=ON
- RW_USE_MATHEMATICA=OFF
    - RW_ENABLE_MATHEMATICA=OFF
- RW_IS_EXAMPLES_ENABLED=OFF
    - RW_BUILD_EXAMPLES=OFF
- RW_IS_TESTS_ENABLED=OFF
    - RW_BUILD_TESTS=OFF #Seams no to work
- SWIG_EXECUTABLE=%SWIG_ROOT%\swig.exe

- BUILD_sdurw=ON
- BUILD_sdurw_algorithms=ON
- BUILD_sdurw_assembly=ON
- BUILD_sdurw_calibration=ON
- BUILD_sdurw_control=ON
- BUILD_sdurw_opengl=ON
- BUILD_sdurw_mathematica  OFF
- BUILD_sdurw_proximitystrategies=ON
- BUILD_sdurw_pathoptimization=ON
- BUILD_sdurw_pathplanners=ON
- BUILD_sdurw_task=ON
- BUILD_sdurw_simulation=ON
- BUILD_sdurw_lua=ON
- BUILD_sdurw_python=ON
- BUILD_sdurw_java=ON
- BUILD_sdurw_softbody=OFF
- BUILD_sdurw_csg=ON

RobWorkStudio Options
---------------------

- BUILD_sdurws_atask=ON
- BUILD_sdurws_gtask=ON
- BUILD_sdurws_jog=ON
- BUILD_sdurws_log=ON
- BUILD_sdurws_playback=ON
- BUILD_sdurws_propertyview=ON
- BUILD_sdurws_treeview=ON
- BUILD_sdurws_planning=ON
- BUILD_sdurws_sensors=ON
- BUILD_sdurws_luaeditor=ON
- BUILD_sdurws_luapl=ON
- BUILD_sdurws_robworkstudioapp=ON
- BUILD_sdurws_lua=ON
- BUILD_sdurws_java=ON
- BUILD_sdurws_python=ON
- RWS_SHARED_LIBS=ON
- RWS_USE_STATIC_LINK_COMPONENTS=OFF
- RWS_USE_STATIC_LINK_PLUGINS=OFF
- USE_WERROR=true

- SWIG_EXECUTABLE=%SWIG_ROOT%\swig.exe

RobWorkSim Options
------------------


- BUILD_sdurwsim_bullet=ON
- Bullet will be compiled with single precision.
- BUILD_sdurwsim_luai=ON
- BUILD_sdurwsim_java=ON
- BUILD_sdurwsim_python=ON
- RWSIM_SHARED_LIBS=ON
- USE_WERROR=true
- SWIG_EXECUTABLE=%SWIG_ROOT%\swig.exe

RobworkHardWare Options
-----------------------

- RWHW_SHARED_LIBS=ON
- USE_WERROR=true

Unknown Options
---------------
These are options where it is unknown which project supports them

- BUILD_SHARED_LIBS=ON \
- CMAKE_DISABLE_FIND_PACKAGE_SWIG=True
