Ubuntu installation by PPA
*****************************

Precompiled Debian packages exist for Ubuntu 16.04 and 18.04


Install the 4 packages like this::

    sudo add-apt-repository ppa:sdurobotics/robwork
    sudo apt-get update

    sudo apt-get install libsdurw-all-dev libsdurws-all-dev libsdurwhw-all-dev libsdurwsim-all-dev

.. note::

   When using the precompiled packages the following interfaces will NOT be available : Java, Lua, Python. Matlab.