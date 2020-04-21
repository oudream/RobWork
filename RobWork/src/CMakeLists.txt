
# Subdirectories to process.

add_subdirectory(rw/core)
add_subdirectory(rw)
add_subdirectory(rwlibs)

# Build sandbox if chosen
OPTION(RW_BUILD_SANDBOX "Set when you want to build the sandbox library" OFF)
IF ( RW_BUILD_SANDBOX )
  ADD_SUBDIRECTORY(sandbox)
  SET(RW_SANDBOX_LIB sandbox)
ENDIF ()

