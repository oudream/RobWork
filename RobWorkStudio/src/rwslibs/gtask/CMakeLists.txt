set(SUBSYS_NAME sdurws_gtask)
set(SUBSYS_DESC "A sensor view plugin")
set(SUBSYS_DEPS sdurws)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # if we want to use ui files add them here
    set(UIS_FILES GTaskVisPlugin.ui)
    if(RWS_USE_QT5)
        qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    else()
        qt4_wrap_ui(UIS_OUT_H ${UIS_FILES})
    endif()
    # Need to add the current binary dir to the include directory because the generated source files are
    # placed here
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    # Standard cpp files to compile:
    set(SrcFiles GTaskVisPlugin.cpp)
    set(SRC_FILES_HPP GTaskVisPlugin.hpp)

    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    if(RWS_USE_QT5)
        qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP})
        # Rcc the files:
        qt5_add_resources(RccSrcFiles gtask_resources.qrc)
    else()
        # Header files to moc and compile:
        qt4_wrap_cpp(
            MocSrcFiles
            ${SRC_FILES_HPP}
            OPTIONS
            -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
            -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
            -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
            -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
        )
        # Rcc the files:
        qt4_add_resources(RccSrcFiles gtask_resources.qrc)
    endif()

    rws_add_plugin(
        ${SUBSYS_NAME}
        MODULE
        ${SrcFiles}
        ${MocSrcFiles}
        ${RccSrcFiles}
        ${UIS_OUT_H}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/gtask" ${SRC_FILES_HPP})
    target_link_libraries(
        ${SUBSYS_NAME}
        ${ROBWORKSTUDIO_LIBRARIES}
        sdurw_task
        sdurw_proximitystrategies
        ${ROBWORK_LIBRARIES}
    )
    add_dependencies(${SUBSYS_NAME} ${SUBSYS_DEPS})
endif()
