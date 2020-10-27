set(SUBSYS_NAME sdurw)
set(SUBSYS_DESC "The core RobWork library!")
set(SUBSYS_DEPS
    sdurw_core
    sdurw_common
    sdurw_math
    sdurw_kinematics
    sdurw_plugin
    sdurw_geometry
    sdurw_sensor
    sdurw_models
    sdurw_proximity
)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS})

if(build)

    set(RW_LIB_FILES_CPP ${RW_USER_FILES})
    set(RW_LIB_FILES_HPP ${RW_USER_FILES_HPP})

    add_subdirectory(invkin)
    add_subdirectory(loaders)
    add_subdirectory(pathplanning)
    add_subdirectory(graspplanning)
    add_subdirectory(trajectory)
    add_subdirectory(graphics)

    # Add the RobWork files
    set(FILES_CPP RobWork.cpp)

    set(FILES_HPP
        graphics.hpp
        graspplanning.hpp
        invkin.hpp
        loaders.hpp
        pathplanning.hpp
        proximity.hpp
        RobWork.hpp
        rw.hpp
        trajectory.hpp
        math_fwd.hpp
    )

    file(GLOB SRC_FILES_CPP ${FILES_CPP})
    file(GLOB SRC_FILES_HPP ${FILES_HPP})

    list(APPEND RW_LIB_FILES_CPP ${SRC_FILES_CPP})
    list(APPEND RW_LIB_FILES_HPP ${SRC_FILES_HPP})

    rw_add_library(${SUBSYS_NAME} ${RW_LIB_FILES_CPP} ${RW_LIB_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS})

    if(RW_HAVE_OMP)
        target_link_libraries(${SUBSYS_NAME} PUBLIC ${OpenMP_CXX_LIBRARIES})
    endif()
    if(RW_HAVE_XERCES)
        target_link_libraries(${SUBSYS_NAME} PUBLIC ${XERCESC_LIBRARIES})
    endif()
    if(RW_HAVE_ASSIMP)
        target_link_libraries(${SUBSYS_NAME} PRIVATE ${ASSIMP_LIBRARIES})
    endif()

    target_link_libraries(${SUBSYS_NAME} PRIVATE ${QHULL_LIBRARIES} ${CMAKE_DL_LIBS})

    target_include_directories(
        ${SUBSYS_NAME}
        PUBLIC ${Boost_INCLUDE_DIR} ${XERCESC_INCLUDE_DIR} $<BUILD_INTERFACE:${PQP_INCLUDE_DIR}>
               $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    if(RW_ENABLE_INTERNAL_PQP_TARGET)
        target_include_directories(
            ${SUBSYS_NAME} PUBLIC $<INSTALL_INTERFACE:${RW_EXT_INSTALL_DIR}/rwpqp>
        )
    else()
        target_include_directories(${SUBSYS_NAME} PUBLIC $<INSTALL_INTERFACE:${PQP_INCLUDE_DIR}>)
    endif()
    rw_include_eigen(${SUBSYS_NAME})

    rw_add_includes(${SUBSYS_NAME} "rw" ${FILES_HPP})
    rw_add_include_dirs(
        ${SUBSYS_NAME}
        "rw"
        graspplanning
        invkin
        loaders
        pathplanning
        trajectory
        graphics
        constraints
    )

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
