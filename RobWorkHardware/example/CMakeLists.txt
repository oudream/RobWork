cmake_minimum_required(VERSION 3.5.1)
project(RWHWExamples)

set(RWHW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

# Subdirectories to process.
add_subdirectory(camera)
add_subdirectory(pcube)
add_subdirectory(sdh)
add_subdirectory(trakstar)
add_subdirectory(robolabFT)
add_subdirectory(ur_rtde)
add_subdirectory(ur)
