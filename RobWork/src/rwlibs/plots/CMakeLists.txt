set(SUBSYS_NAME sdurw_plots_mathgl)
set(SUBSYS_DESC "MathGL based Plot generator.")
set(SUBSYS_DEPS
    PUBLIC sdurw_core sdurw_graphics
)

find_package(MathGL QUIET)

if(DEFINED MATHGL_FOUND)
    message(STATUS "Found MathGL ${MATHGL_VERSION_STRING}")
    set(DEFAULT TRUE)
else()
    set(DEFAULT FALSE)
endif()
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(SRC_FILES
        MathGLPlot.cpp
        MathGLPlotGenerator.cpp
    )

    set(SRC_FILES_HPP
        MathGLPlot.hpp
        MathGLPlotGenerator.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/plots" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} ${SUBSYS_DEPS}
        PRIVATE ${MATHGL_LIBRARIES}
    )

    target_include_directories(${SUBSYS_NAME} PRIVATE ${MATHGL_INCLUDE_DIRS})
    target_include_directories(
        ${SUBSYS_NAME}
        INTERFACE $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

    set(SUBSYS_NAME_PLUGIN ${SUBSYS_NAME}.rwplugin)
    set(SUBSYS_DESC_PLUGIN "Plugin providing MathGL based plot extension.")
    set(SUBSYS_DEPS_PLUGIN ${SUBSYS_NAME})

    set(build_plugin TRUE)
    rw_subsys_option(
        build_plugin ${SUBSYS_NAME_PLUGIN} ${SUBSYS_DESC_PLUGIN} ${DEFAULT}
        DEPENDS ${SUBSYS_DEPS_PLUGIN}
        ADD_DOC
    )

    if(build_plugin)
        rw_add_plugin(${SUBSYS_NAME}.rwplugin MODULE MathGLPlugin.cpp)
        target_link_libraries(${SUBSYS_NAME}.rwplugin PRIVATE ${SUBSYS_NAME} PUBLIC sdurw_core)

        if(CMAKE_COMPILER_IS_GNUCXX)
            set_source_files_properties(MathGLPlugin.cpp PROPERTIES COMPILE_FLAGS -fno-gnu-unique)
        endif()
    endif()
endif()
