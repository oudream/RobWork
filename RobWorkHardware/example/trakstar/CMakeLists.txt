# check compiler/operating system

if(COMPONENT_trakstar_ENABLE)
    #
    # Test CMake version
    #
    cmake_minimum_required(VERSION 3.5.1)

    # The name of the project. (EDIT THIS)
    project(TrakstarExamples)

    # Used to resolve absolute path names
    set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

    if(DEFINED MSVC)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
    else()
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
    endif()

    set(RW_ROOT "${ROOT}/../../../RobWork")
    set(RWHW_ROOT "${ROOT}/../../../RobWorkHardware")

    set(CMAKE_MODULE_PATH ${RW_ROOT}/build ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})

    find_package(RobWork)
    find_package(RobWorkHardware)
    find_package(TrakStar)

    include_directories(${ROBWORK_INCLUDE_DIR} ${ROBWORKHARDWARE_INCLUDE_DIR} ${TRAKSTAR_INCLUDE_DIR})
    link_directories(${ROBWORK_LIBRARY_DIRS} ${ROBWORKHARDWARE_LIBRARY_DIRS} ${TRAKSTAR_LIBRARY_DIR})
    message(${TRAKSTAR_LIBRARY_DIR})
    add_executable(TrakstarLogger TrakstarLogger.cpp)

    target_link_libraries(TrakstarLogger sdurwhw_trakstar ${TRAKSTAR_LIBRARIES} ${ROBWORK_LIBRARIES})

    message(${TRAKSTAR_LIBRARIES})
endif(COMPONENT_trakstar_ENABLE)
