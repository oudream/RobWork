# Test CMake version
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# The name of the project.
project(TutorialCollection)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the RobWork.
set(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
find_package(RobWork REQUIRED)

include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS})

# Set the output dir for generated libraries and binaries
if(MSVC)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH
        "Runtime directory" FORCE
    )
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH
        "Library directory" FORCE
    )
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH
        "Archive directory" FORCE
    )
endif()

# And now we add any targets that we want
add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries(HelloWorld ${ROBWORK_LIBRARIES})

add_executable(InverseKinematics InverseKinematics.cpp)
target_link_libraries(InverseKinematics ${ROBWORK_LIBRARIES})

add_executable(MathJogling MathJogling.cpp)
target_link_libraries(MathJogling ${ROBWORK_LIBRARIES})

add_executable(WorkCellAndDevices WorkCellAndDevices.cpp)
target_link_libraries(WorkCellAndDevices ${ROBWORK_LIBRARIES})

if(CMAKE_COMPILER_IS_GNUCC)
    # Do not warn about unused variables. They are there for demonstration.
    target_compile_options(
        InverseKinematics PRIVATE "-Wno-unused-variable;-Wno-unused-but-set-variable"
    )
    target_compile_options(
        WorkCellAndDevices PRIVATE "-Wno-unused-variable;-Wno-unused-but-set-variable"
    )
endif()
