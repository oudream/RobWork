set(SUBSYS_NAME SimulatorLogViewer)
set(SUBSYS_DESC "Stand-alone application for visualization of internal data from a Physics Engine.")
set(SUBSYS_DEPS
    PRIVATE sdurwsim_gui
            sdurwsim
            RW::sdurw_common
    PUBLIC  RW::sdurw_core
)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS}
)
rw_add_doc(${SUBSYS_NAME})

if(build)
    # Standard cpp files to compile:
    set(SrcFiles SimulatorLogViewer.cpp)
    set(SRC_FILES_HPP SimulatorLogViewer.hpp)

    set_source_files_properties(
        ${SrcFiles}
        PROPERTIES
        OBJECT_DEPENDS
        "${UIS_OUT_H}"
    )

    set(MocSrcFiles ${SRC_FILES_HPP})
    set(RccSrcFiles resources.qrc)

    add_executable(SimulatorLogViewer ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_FILES})
    target_link_libraries(
        SimulatorLogViewer
        ${SUBSYS_DEPS}
        PUBLIC ${QT_LIBRARIES}
    )
    # Need to add the current binary dir to the include directory because the generated source files are placed here
    target_include_directories(SimulatorLogViewer PUBLIC $<BUILD_INTERFACE:${RWSIM_ROOT}/src> PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

endif()
