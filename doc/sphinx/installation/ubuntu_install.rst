Ubuntu installation by PPA
*****************************

Precompiled Debian packages exist for:
 * Ubuntu 18.04
 * Ubuntu 20.04
 
They can be installed by adding sdurobotics/robwork to 'apt' ppa repositories.

.. code-block:: bash

    sudo add-apt-repository ppa:sdurobotics/robwork
    sudo apt-get update

The Simplest install to get all our packages can then be done with:

.. code-block:: bash

    # c++ interface 
    sudo apt-get install libsdurw-all-dev \
                         libsdurws-all-dev \
                         libsdurwsim-all-dev

    # python interface 
    sudo apt install python3-robwork 

    # lua interface
    sudo apt install lua-robwork


.. note::

    When using the precompiled packages the following interfaces will NOT be available : Java, Matlab.

PPA packages
------------
To allow for customization of your RobWork installation.
Here is a more detailed overview of the different packages available from our PPA.


Individual packages
###################

The RobWork ppa contains many different packages to allow, custom installations of only the needed components.
The first array of packages are the individual RobWork packages. Which can be downloaded in versioned mode with:

.. code-block:: bash

    sudo apt-get install lib<package>

For the development packages including the newest version of robwork and the include files the command is:

.. code-block:: bash

    sudo apt-get install lib<package>-dev

+---------------------------------+-----------------------------------+-----------------------+
| RobWork                         | RobWorkStudio                     | RobWorkSim            |
+---------------------------------+-----------------------------------+-----------------------+
|  - libsdurw                     |  - libsdurws                      |  - libsdurwsim        |
|  - libsdurw-algorithms          |  - libsdurws-atask                |  - libsdurwsim-bullet |
|  - libsdurw-geometry            |  - libsdurws-gtask                |  - libsdurws-gui      |
|  - libsdurw-assembly            |  - libsdurws-jog                  |  - libsdurwsim-ode    |
|  - libsdurw-calibration         |  - libsdurws-log                  |                       |
|  - libsdurw-control             |  - libsdurws-planning             |                       |
|  - libsdurw-csg                 |  - libsdurws-playback             |                       |
|  - libsdurw-opengl              |  - libsdurws-propertyview         |                       |
|  - libsdurw-pathoptimization    |  - libsdurws-robworkstudioapp     |                       |
|  - libsdurw-pathplanners        |  - libsdurws-sensors              |                       |
|  - libsdurw-proximitystrategies |  - libsdurws-treeview             |                       |
|  - libsdurw-simulation          |  - libsdurws-workcelleditor       |                       |
|  - libsdurw-task                |  - libsdurws-workcelleditorplugin |                       |
|  - libsdurw-core                |  - libsdurws-charts               |                       |
|  - libsdurw-common              |  - libsdurws-charts-qt            |                       |
|  - libsdurw-math                |                                   |                       |
|  - libsdurw-geometry-expanded   |                                   |                       |
|  - libsdurw-plots               |                                   |                       |
|  - libsdurw-plots-mathgl        |                                   |                       |
+---------------------------------+-----------------------------------+-----------------------+

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
|  - python3-sdurw-core                 |  - lua-sdurw-core                 |
|  - python3-sdurw-common               |  - lua-sdurw-common               |
|  - python3-sdurw-math                 |  - lua-sdurw-math                 |
|  - python3-sdurw-kinematics           |  - lua-sdurw-kinematics           |
|  - python3-sdurw-geometry             |  - lua-sdurw-geometry             |
|  - python3-sdurw-sensor               |  - lua-sdurw-sensor               |
|  - python3-sdurw-models               |  - lua-sdurw-models               |
|  - python3-sdurw-graspplanning        |  - lua-sdurw-graspplanning        |
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
The "-robwork-all" includes all packages for all four main robwork code libraries.

- libsdurw-all-dev
- libsdurw-all
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

- sdurw-cmake
    - This package is only installed it libsdurw-all-dev as it currently can't handle the individual components.
      It will therefore always try to load all sdurw libraries.
- sdurws-cmake
    - It is fetched together with libsdurws-dev.
- sdurwsim-cmake
    - This package is only installed it libsdurwsim-all-dev as it currently can't handle the individual components.
      It will therefore always try to load all sdurwsim libraries.


special mentions
################

These are the remaining special packages not mentioned yet.

- robworkstudio
    - This packages contains the binary for running robworkstudio.
    - The program will automatically detect and load sdurws plugins as you download them.
- sdurw-doc
    - This packages don't contain anything, but it is planned to contain a complete version of the documentation.

Ubuntu uninstallation by PPA
*****************************

There are a few ways of uninstalling the packages once installed with ppa.

Uninstall by knowledge
-----------------------

If you know the specific packages installed like:

.. code-block:: bash

    sudo apt-get install libsdurw-all-dev \
                         libsdurws-all-dev \
                         libsdurwsim-all-dev

Then the uninstall is mostly the reverse:
.. code-block:: bash

    sudo apt-get remove libsdurw-all-dev  \
                        libsdurws-all-dev \
                        libsdurwsim-all-dev

    sudo apt-get autoremove

The autoremove is very important as, the first command only removes the specified packages,
but doesn't remove all the dependencies of the installed packages.
In the given example since <package>-all-dev is a metapackage that doesn't contain anything
the entirety of robwork will still be installed after "apt-get remove ..." is called.
It is only after autoremove is called that it is uninstalled.

Uninstall by search
-------------------

If you don't remember which packages you installed then it is possible to make apt-get look for them.
To make sure that the correct packages are selected, before removing them, it is good practice to make a simulated run.

.. code-block:: bash

    sudo apt-get -s remove *sdurw*

This command will find all packages installed and not installed that includes sdurw, which all our packages except for robworkstudio does.
If you look through the output and find everything satisfying then all of RobWork can be uninstall with:

.. code-block:: bash

    sudo apt-get remove *sdurw*
    sudo apt-get remove robworkstudio
    sudo apt-get autoremove

For good measure autoremove is still used, to make sure that robwork's external dependencies that haven't been installed intentionally are removed.

