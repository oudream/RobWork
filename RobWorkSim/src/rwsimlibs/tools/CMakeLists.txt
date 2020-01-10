set(SUBSYS_NAME sdurwsim_gtask)
set(SUBSYS_DESC "Stand-alone application for visualization of internal data from a Physics Engine.")
set(SUBSYS_DEPS sdurws sdurw)

set(build TRUE)
rw_subsys_option(
    build
    ${SUBSYS_NAME}
    ${SUBSYS_DESC}
    ON
)
rw_add_doc(${SUBSYS_NAME})

if(build)
    LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})

    # if we want to use ui files add them here
    set(UIS_FILES SimulatorLogViewer.ui)
    qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    # Need to add the current binary dir to the include directory because the generated source files are placed here
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    # Standard cpp files to compile:
    set(SrcFiles SimulatorLogViewer.cpp)
    set(SRC_FILES_HPP SimulatorLogViewer.hpp)

    set_source_files_properties(
        ${SrcFiles}
        PROPERTIES
        OBJECT_DEPENDS
        "${UIS_OUT_H}"
    )

    qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP})
    # Rcc the files:
    qt5_add_resources(RccSrcFiles resources.qrc)

    add_executable(SimulatorLogViewer ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_FILES})
    target_link_libraries(
        SimulatorLogViewer
        sdurwsim_gui
        ${ROBWORK_LIBRARIES}
        ${ROBWORKSIM_LIBRARIES}
        ${ROBWORKSTUDIO_LIBRARIES}
    )

    foreach(
        DEP IN
        LISTS
            sdurwsim_gui
            ROBWORK_LIBRARIES
            ROBWORKSIM_LIBRARIES
            ROBWORKSTUDIO_LIBRARIES
    )
        if(TARGET ${DEP})
            add_dependencies(sdurwsim_gui ${DEP})
        endif()
    endforeach()

endif()
