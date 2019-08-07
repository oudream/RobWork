cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ExampleSnippets)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "None")
endif()
find_package(RobWork REQUIRED)

link_directories(${ROBWORK_LIBRARY_DIRS})

# And now we add any targets that we want
add_executable(invkin invkin.cpp)
target_link_libraries(invkin ${ROBWORK_LIBRARIES})
target_include_directories(invkin PRIVATE ${ROBWORK_INCLUDE_DIRS})
