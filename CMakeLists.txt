# Test CMake version
cmake_minimum_required(VERSION 2.8.12)

cmake_policy(SET CMP0011 NEW)
cmake_policy(SET CMP0012 NEW)

# The name of the project.
project(RobWorkProject)

set(RWP_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(RW_ROOT ${ROOT}/RobWork/)
set(RWS_ROOT ${ROOT}/RobWorkStudio/)
set(RWSIM_ROOT ${ROOT}/RobWorkSim/)
set(RWHW_ROOT ${ROOT}/RobWorkHardware/)

add_subdirectory(RobWork)
add_subdirectory(RobWorkHardware)
add_subdirectory(RobWorkStudio)
add_subdirectory(RobWorkSim)

# #######################################################33 now configure documentation
configure_file(${ROOT}/doc/Doxyfile.in ${ROOT}/doc/Doxyfile)

find_program(doc_executable doxygen HINTS ${DOXYGEN_PATH} $ENV{DOXYGEN_PATH})
if(doc_executable)
    add_custom_target(sdurwdoc ${doc_executable} WORKING_DIRECTORY ${ROOT}/doc)
endif()

# Packaging INCLUDE(build/packing.cmake)
