Ubuntu installation by PPA
*****************************

Precompiled Debian packages exist for Ubuntu 16.04 and 18.04.
They can be installed by adding sdurobotics/robwork to 'apt' ppa repositories.

.. code-block:: bash

    sudo add-apt-repository ppa:sdurobotics/robwork
    sudo apt-get update

The Simplest install to get all our packages can then be done with:

.. code-block:: bash

    sudo apt-get install libsdurw-all-dev \
                         libsdurws-all-dev \
                         libsdurwhw-all-dev \
                         libsdurwsim-all-dev

.. note::

    When using the precompiled packages the following interfaces will NOT be available : Java, Lua, Python. Matlab.

PPA packages
------------
To allow for customization of your RobWork installation.
Here is a more detailed overview of the different packages available from our PPA.


Individual packages
###################

The RobWork ppa contains many different packages to allow, custom installations of only the needed components.
The first array of packages are the individual RobWork packages. Which can be downloaded in versioned mode with:

.. code-block:: bash

    sudo apt-get install lib<package><version>

For the development packages including the newest version of robwork and the include files the command is:

.. code-block:: bash

    sudo apt-get install lib<package>-dev

+---------------------------------+-----------------------------------+------------------------------------+-----------------------+
| RobWork                         | RobWorkStudio                     | RobWorkHardWare                    | RobWorkSim            |
+---------------------------------+-----------------------------------+------------------------------------+-----------------------+
|  - libsdurw                     |  - libsdurws                      |  - libsdurwhw-camera               |  - libsdurwsim        |
|  - libsdurw-algorithms          |  - libsdurws-atask                |  - libsdurwhw-can                  |  - libsdurwsim-bullet |
|  - libsdurw-analytic-geometry   |  - libsdurws-gtask                |  - libsdurwhw-dockwelder           |                       |
|  - libsdurw-assembly            |  - libsdurws-jog                  |  - libsdurwhw-netft                |                       |
|  - libsdurw-calibration         |  - libsdurws-log                  |  - libsdurwhw-pcube                |                       |
|  - libsdurw-control             |  - libsdurws-planning             |  - libsdurwhw-robolabft            |                       |
|  - libsdurw-csg                 |  - libsdurws-playback             |  - libsdurwhw-robotiq              |                       |
|  - libsdurw-opengl              |  - libsdurws-propertyview         |  - libsdurwhw-schunkpg70           |                       |
|  - libsdurw-pathoptimization    |  - libsdurws-robworkstudioapp     |  - libsdurwhw-serialport           |                       |
|  - libsdurw-pathplanners        |  - libsdurws-sensors              |  - libsdurwhw-tactile              |                       |
|  - libsdurw-proximitystrategies |  - libsdurws-treeview             |  - libsdurwhw-universalrobots      |                       |
|  - libsdurw-simulation          |  - libsdurws-workcelleditor       |  - libsdurwhw-universalrobots-rtde |                       |
|  - libsdurw-task                |  - libsdurws-workcelleditorplugin |                                    |                       |
+---------------------------------+-----------------------------------+------------------------------------+-----------------------+

Python and Lua packages
########################

To use RobWork with Lua scripting or Python3, the following packages are available as wrapped c++ code.
They are downloaded in the same way as c++ libraries

.. code-block:: bash

    sudo apt-get install python3-<package>
    # or
    sudo apt-get install lua-<package>

+---------------------------------------+-----------------------------------+
| RobWork                               | Lua Packages                      |
+---------------------------------------+-----------------------------------+
|  - python3-sdurw                      |  - lua-sdurw                      |
|  - python3-sdurw-assembly             |  - lua-sdurw-assembly             |
|  - python3-sdurw-control              |  - lua-sdurw-control              |
|  - python3-sdurw-opengl               |  - lua-sdurw-opengl               |
|  - python3-sdurw-pathoptimization     |  - lua-sdurw-pathoptimization     |
|  - python3-sdurw-pathplanners         |  - lua-sdurw-pathplanners         |
|  - python3-sdurw-proximitystrategies  |  - lua-sdurw-proximitystrategies  |
|  - python3-sdurw-simulation           |  - lua-sdurw-simulation           |
|  - python3-sdurw-task                 |  - lua-sdurw-task                 |
|  - python3-sdurws                     |  - lua-sdurws                     |
|  - python3-sdurwsim                   |  - lua-sdurwsim                   |
+---------------------------------------+-----------------------------------+

Bundled packages
################

Other then installing all the packages individually,
some packages containing all the individual packages exists.
These packages is made for quick installation.
In the package name "lib<Library>-all" means all packages belonging to <Library>,
and "-all-dev" is a development version of the "-all" package.
The ".robwork-all" includes all packages for all four main robwork code libraries.

- libsdurw-all-dev
- libsdurw-all
- libsdurwhw-all-dev
- libsdurwhw-all
- libsdurws-all-dev
- libsdurws-all
- libsdurwsim-all-dev
- libsdurwsim-all
- python3-robwork-all
- lua-robwork-all


cmake packages
##############

To allow CMake to find RobWork and it's different files CMake packages has been made for the main libraries.
It should not be necessary to get these packages specifically.
As they are automatically downloaded, when needed.
Do notice that the cmake packages are versioned to fit with the RobWork Versions.

- sdurw-cmake<version>
    - This package is only installed it libsdurw-all-dev as it currently can't handle the individual components.
      It will therefore always try to load all sdurw libraries.
- sdurwhw-cmake<version>
    - Is supplied with the individual "-dev" packages
- sdurws-cmake<version>
    - It is fetched together with libsdurws-dev.
- sdurwsim-cmake<version>
    - This package is only installed it libsdurwsim-all-dev as it currently can't handle the individual components.
      It will therefore always try to load all sdurwsim libraries.


special mentions
################

These are the remaning special packages not mentioned yet.

- libsdurwhw-dev
    - This package is automatically fetched when needed and contains the shared include files for RobWorkHardWare
- robworkstudio
    - This packages contains the binary for running robworkstudio.
    - The program will automatically detect and load sdurws plugins as you download them.
- sdurw-doc
    - This packages don't contain anything, but it is planned to contain a complete version of the documentation.

