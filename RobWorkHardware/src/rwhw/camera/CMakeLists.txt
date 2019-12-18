set(SUBSYS_NAME sdurwhw_camera)
set(SUBSYS_DESC "Library of drivers for different cameras")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
set(DEFAULT TRUE)
set(REASON)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    message(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")

    set(SRC_CPP ${RW_IO_USER_FILES} CameraFactory.cpp)
    set(SRC_HPP CameraFactory.hpp)

    find_package(CMU1394)
    if(CMU1394_FOUND AND DEFINED MSVC) # currently CMU1394 is only supported by MSVC compiler
        message(STATUS "RobWorkHardware: ${SUBSYS_NAME} - CMU1394 library found. ")
        list(APPEND SRC_CPP CMU1394Camera.cpp)
        list(APPEND SRC_HPP CMU1394Camera.hpp)
        include_directories(${CMU1394_INCLUDE_DIRS})
    else()
        message(STATUS "RobWorkHardware: ${SUBSYS_NAME} - CMU1394 library NOT found. ")
    endif()

    find_package(CMU1394C)
    if(CMU1394C_FOUND)
        message(STATUS "RobWorkHardware: ${SUBSYS_NAME} - CMU1394C library found. ")
        list(APPEND SRC_CPP CMU1394Camera_c.cpp)
        list(APPEND SRC_HPP CMU1394Camera_c.hpp)
        include_directories(${CMU1394C_INCLUDE_DIR})
    else()
        message(STATUS "RobWorkHardware: ${SUBSYS_NAME} - CMU1394C library NOT found. ")
    endif()

    find_package(DC1394)
    if(DC1394_FOUND)
        message(STATUS "RobWorkHardware: ${SUBSYS_NAME} - DC1394 library found. ")
        list(APPEND SRC_CPP DC1394Camera.cpp DC1394CameraFactory.cpp)
        list(APPEND SRC_HPP DC1394Camera.hpp DC1394CameraFactory.hpp)
        include_directories(${DC1394_INCLUDE_DIR})
    else()
        message(STATUS "RobWorkHardware: ${SUBSYS_NAME} - DC1394 library NOT found. ")
    endif()

    find_package(OpenCV COMPONENTS CV CXCORE HIGHGUI CVCAM QUIET)
    if(OpenCV_FOUND AND OpenCV_CVCAM_INCLUDE_DIR)
        message(STATUS "RobWorkHardware: ${SUBSYS_NAME} - OpenCV library found. ")
        list(APPEND SRC_CPP OpenCVCamera.cpp)
        list(APPEND SRC_HPP OpenCVCamera.hpp)
        include_directories(${OpenCV_INCLUDE_DIRS})
    else()
        message(STATUS "RobWorkHardware: ${SUBSYS_NAME} - OpenCV library NOT found. ")
    endif()

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${ROBWORK_LIBRARIES})
    if(DC1394_FOUND)
        target_link_libraries(${SUBSYS_NAME} PUBLIC ${DC1394_LIBRARY})
    endif()
    rw_add_includes(${SUBSYS_NAME} "rwhw/camera" ${SRC_HPP})
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS ROBWORK_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(${SUBSYS_NAME} ${DEP})
        endif()
    endforeach()

endif()
