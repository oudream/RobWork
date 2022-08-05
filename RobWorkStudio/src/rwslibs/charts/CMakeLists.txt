set(SUBSYS_NAME sdurws_charts_qt)
set(SUBSYS_DESC "QChartView based PlotView.")
set(SUBSYS_DEPS
    PUBLIC sdurws
)

find_package(Qt6 COMPONENTS Charts QUIET)

if(Qt6Charts_FOUND)
    set(DEFAULT TRUE)
else()
    find_package(Qt5Charts 5.5.1 QUIET)
    if(Qt5Charts_FOUND)
        set(DEFAULT TRUE)
    else()
        set(DEFAULT FALSE)
    endif()
endif()
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(SRC_FILES
        ChartView.cpp
        ChartViewGenerator.cpp
    )

    set(SRC_FILES_HPP
        ChartView.hpp
        ChartViewGenerator.hpp
    )

    rws_add_component(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwslibs/charts" ${SRC_FILES_HPP})
    if(DEFINED Qt6Charts_FOUND)
        target_link_libraries(${SUBSYS_NAME} ${SUBSYS_DEPS}
            PUBLIC Qt6::Charts
        )
    else()
        target_link_libraries(${SUBSYS_NAME} ${SUBSYS_DEPS}
            PUBLIC Qt5::Charts
        )
    endif()

    target_include_directories(
        ${SUBSYS_NAME}
        INTERFACE $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    if(CMAKE_COMPILER_IS_GNUCXX)
        set_source_files_properties(ChartView.cpp PROPERTIES COMPILE_FLAGS -fno-gnu-unique)
    endif()

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

    set(SUBSYS_NAME_PLUGIN ${SUBSYS_NAME}.rwplugin)
    set(SUBSYS_DESC_PLUGIN "Plugin providing QChartView based PlotView extension.")
    set(SUBSYS_DEPS_PLUGIN ${SUBSYS_NAME})

    set(build_plugin TRUE)
    rw_subsys_option(
        build_plugin ${SUBSYS_NAME_PLUGIN} ${SUBSYS_DESC_PLUGIN} ${DEFAULT}
        DEPENDS ${SUBSYS_DEPS_PLUGIN}
        ADD_DOC
    )

    if(build_plugin)
        rw_add_plugin(${SUBSYS_NAME}.rwplugin MODULE ChartViewPlugin.cpp)
        target_link_libraries(${SUBSYS_NAME}.rwplugin PRIVATE ${SUBSYS_NAME} PUBLIC RW::sdurw_core)

        if(CMAKE_COMPILER_IS_GNUCXX)
            set_source_files_properties(ChartViewPlugin.cpp PROPERTIES COMPILE_FLAGS -fno-gnu-unique)
        endif()
    endif()
endif()
