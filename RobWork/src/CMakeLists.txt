# Subdirectories to process.

add_subdirectory(rw/core)
add_subdirectory(rw/plugin)
add_subdirectory(rw/common)
add_subdirectory(rw/math)
add_subdirectory(rw/kinematics)
add_subdirectory(rw/geometry)
add_subdirectory(rw/sensor)
add_subdirectory(rw/models)
add_subdirectory(rw/proximity)
add_subdirectory(rw)
add_subdirectory(rwlibs)

# Build sandbox if chosen
option(RW_BUILD_SANDBOX "Set when you want to build the sandbox library" OFF)
if(RW_BUILD_SANDBOX)
    add_subdirectory(sandbox)
    set(RW_SANDBOX_LIB sandbox)
endif()
