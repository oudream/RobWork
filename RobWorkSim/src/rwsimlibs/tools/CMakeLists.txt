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
    #INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
    LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})

    # if we want to use ui files add them here
    set(UIS_FILES SimulatorLogViewer.ui)
    if(RWS_USE_QT5)
        qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    else()
        qt4_wrap_ui(UIS_OUT_H ${UIS_FILES})
    endif()
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

    if(RWS_USE_QT5)
        qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP})
        # Rcc the files:
        qt5_add_resources(RccSrcFiles resources.qrc)
    else()
        # Header files to moc and compile:
        QT4_WRAP_CPP(MocSrcFiles ${SrcFiles_HPP} OPTIONS -DBOOST_TT_HAS_PLUS_HPP_INCLUDED -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED)
        # Rcc the files:
        qt4_add_resources(RccSrcFiles resources.qrc)
    endif()

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
