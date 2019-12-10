set(SUBSYS_NAME sdurw_proximitystrategies)
set(SUBSYS_DESC "Collection of proximity strategies!")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT})
rw_add_doc(${SUBSYS_NAME})

if(build)
    set(OptionalFiles)
    if(RW_HAVE_PQP)
        list(APPEND OptionalFiles "./ProximityStrategyPQP.cpp")
        list(APPEND OptionalFiles_hpp "./ProximityStrategyPQP.hpp")
    else()
        message(STATUS "sdurw_proximitystrategies: Not building PQP interface since PQP is disabled!")
    endif()

    if(RW_HAVE_YAOBI)
        list(APPEND OptionalFiles "./ProximityStrategyYaobi.cpp")
        list(APPEND OptionalFiles_hpp "./ProximityStrategyYaobi.hpp")
    else()
        message(STATUS "sdurw_proximitystrategies: Not building Yaobi interface since Yaobi is disabled!")
    endif()

    if(RW_HAVE_BULLET)
        list(APPEND OptionalFiles "./ProximityStrategyBullet.cpp")
        list(APPEND OptionalFiles_hpp "./ProximityStrategyBullet.hpp")
    else()
        message(STATUS "sdurw_proximitystrategies: Not building Bullet interface since Bullet is disabled!")
    endif()

    if(RW_HAVE_FCL)
        list(APPEND OptionalFiles "./ProximityStrategyFCL.cpp")
        list(APPEND OptionalFiles_hpp "./ProximityStrategyFCL.hpp")
    else()
        message(STATUS "sdurw_proximitystrategies: Not building FCL interface since FCL is disabled!")
    endif()

    set(SRC_FILES ${RW_PROXIMITYSTRATEGIES_USER_FILES} ${OptionalFiles} ProximityStrategyFactory.cpp)

    set(SRC_FILES_HPP ${OptionalFiles_hpp} ProximityStrategyFactory.hpp)

    rw_add_library(${SUBSYS_NAME} proximitystrategies ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(proximitystrategies "rwlibs/proximitystrategies" ${SRC_FILES_HPP})
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC ${SUBSYS_DEPS} ${PQP_LIBRARIES} ${BULLET_LIBRARIES}
        PRIVATE ${YAOBI_LIBRARIES} ${FCL_LIBRARIES}
    )
    target_include_directories(${SUBSYS_NAME} PRIVATE ${FCL_INCLUDE_DIRS})
    add_dependencies(${SUBSYS_NAME} ${SUBSYS_DEPS})

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

    add_library(${SUBSYS_NAME}.rwplugin MODULE ProximityStrategyPlugin.cpp)
    target_link_libraries(${SUBSYS_NAME}.rwplugin ${SUBSYS_NAME})
    install(TARGETS ${SUBSYS_NAME}.rwplugin LIBRARY DESTINATION ${RW_PLUGIN_INSTALL_DIR})
endif()
