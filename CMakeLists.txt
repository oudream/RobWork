# Test CMake version
cmake_minimum_required(VERSION 3.5.1)

# The name of the project.
project(RobWorkProject)

set(RWP_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(RW_ROOT ${ROOT}/RobWork/)
set(RWS_ROOT ${ROOT}/RobWorkStudio/)
set(RWSIM_ROOT ${ROOT}/RobWorkSim/)
set(RWHW_ROOT ${ROOT}/RobWorkHardware/)

option(WITH_RWHW "Set true to enable compilation of RobWorkHardware!" TRUE)
option(WITH_RWS "Set true to enable compilation of RobWorkStudio!" TRUE)
option(WITH_RWSIM "Set true to enable compilation of RobWorkSim!" TRUE)

unset(SLIB_COMPILE_BUFFER CACHE) 

add_subdirectory(RobWork)
if(WITH_RWHW)
    add_subdirectory(RobWorkHardware)
endif()
if(WITH_RWS)
    add_subdirectory(RobWorkStudio)
endif()
if(WITH_RWSIM)
    add_subdirectory(RobWorkSim)
endif()

# #######################################################33 now configure documentation
configure_file(${ROOT}/doc/Doxyfile.in ${ROOT}/doc/Doxyfile)

find_program(doc_executable doxygen HINTS ${DOXYGEN_PATH} $ENV{DOXYGEN_PATH})
if(doc_executable)
    add_custom_target(sdurwdoc ${doc_executable} WORKING_DIRECTORY ${ROOT}/doc)
endif()

# Packaging INCLUDE(build/packing.cmake)

# ##################################################################################################
# Windows INSTALLER Packaging
#

