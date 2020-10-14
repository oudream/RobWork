Eigen3
******
::
    - git clone https://gitlab.com/libeigen/eigen.git
    - cd eigen
    - git checkout 3.3.7
    - mkdir Build
    - cd Build
    - cmake .. -G "Visual Studio 16 2019" -A x64  -DCMAKE_INSTALL_PREFIX=%Install_DIR%\eigen
    - mkdir %Install_DIR%\eigen
    - msbuild INSTALL.vcxproj /property:Configuration=Release
