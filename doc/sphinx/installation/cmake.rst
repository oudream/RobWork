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

- BUILD_rw=ON
- BUILD_rw_algorithms=ON
- BUILD_rw_assembly=ON
- BUILD_rw_calibration=ON
- BUILD_rw_control=ON
- BUILD_rw_opengl=ON
- BUILD_rw_mathematica  OFF
- BUILD_rw_proximitystrategies=ON
- BUILD_rw_pathoptimization=ON
- BUILD_rw_pathplanners=ON
- BUILD_rw_task=ON
- BUILD_rw_simulation=ON
- BUILD_rw_lua=ON
- BUILD_rw_python=ON
- BUILD_rw_java=ON
- BUILD_rw_softbody=OFF
- BUILD_rw_csg=ON

RobWorkStudio Options
---------------------

- BUILD_rws_atask=ON
- BUILD_rws_gtask=ON
- BUILD_rws_jog=ON
- BUILD_rws_log=ON
- BUILD_rws_playback=ON
- BUILD_rws_propertyview=ON
- BUILD_rws_treeview=ON
- BUILD_rws_planning=ON
- BUILD_rws_sensors=ON
- BUILD_rws_luaeditor=ON
- BUILD_rws_luapl=ON
- BUILD_rws_rwstudioapp=ON
- BUILD_rws_lua=ON
- BUILD_rws_java=ON
- BUILD_rws_python=ON
- RWS_SHARED_LIBS=ON
- RWS_USE_STATIC_LINK_COMPONENTS=OFF
- RWS_USE_STATIC_LINK_PLUGINS=OFF
- USE_WERROR=true

- SWIG_EXECUTABLE=%SWIG_ROOT%\swig.exe

RobWorkSim Options
------------------


- BUILD_rwsim_bullet=ON
- Bullet will be compiled with single precision.
- BUILD_rwsim_luai=ON
- BUILD_rwsim_java=ON
- BUILD_rwsim_python=ON
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
