set(SUBSYS_NAME sdurws_pathvisualizer)
set(SUBSYS_DESC "a plugin to cisulize paths")
set(SUBSYS_DEPS PUBLIC sdurws RW::sdurw_kinematics RW::sdurw_math)

set(build true)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)
if(build)
    # Standard cpp files to compile:
    set(SrcFiles PathVisualizer.cpp)
    set(SRC_FILES_HPP PathVisualizer.hpp)

    # Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(RccSrcFiles resources_pathvisu.qrc ../../rws/rwstudio_resources.qrc)
    else()
        qt5_add_resources(RccSrcFiles resources_pathvisu.qrc ../../rws/rwstudio_resources.qrc)
    endif()

    set(CMAKE_AUTOUIC ON)

    # Set extra linker flags for apple macos computers
    rws_add_plugin(${SUBSYS_NAME} ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${RccSrcFiles})
    rw_add_includes(${SUBSYS_NAME} "rwslibs/pathvisualizer" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} ${SUBSYS_DEPS} ${QT_LIBRARIES})
    target_include_directories(
        ${SUBSYS_NAME}
        INTERFACE $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rws_plugin_load_details(${SUBSYS_NAME} 2 PathVisualizer false)

    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    endif()
endif()
