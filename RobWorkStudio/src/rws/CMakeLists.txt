set(SUBSYS_NAME sdurws)
set(SUBSYS_DESC "Visulization tool")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    # if we want to use ui files add them here
    set(UIS_FILES AboutBox.ui propertyview/PropertyViewEditor.ui propertyview/PropertyViewDialog.ui)

    if(RWS_USE_QT5)
        qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    else()
        qt4_wrap_ui(UIS_OUT_H ${UIS_FILES})
    endif()

    # Standard files to compile:
    set(
        SRC_FILES_CPP
        ArcBallController.cpp
        # FixedAxisController.cpp
        RWSImageLoaderPlugin.cpp
        ImageUtil.cpp
        RobWorkStudio.cpp
        RWStudioView3D.cpp
        SceneOpenGLViewer.cpp
        RobWorkStudioPlugin.cpp
        AboutBox.cpp
        HelpAssistant.cpp
        ImageView.cpp
        propertyview/PropertyViewEditor.cpp
        propertyview/PropertyViewDialog.cpp
    )

    set(
        SRC_FILES_HPP
        ArcBallController.hpp RWSImageLoaderPlugin.hpp ImageUtil.hpp
        # FixedAxisController.hpp
        RobWorkStudio.hpp
        SceneOpenGLViewer.hpp
        RobWorkStudioPlugin.hpp
        RWStudioView3D.hpp
        AboutBox.hpp
        HelpAssistant.hpp
        ImageView.hpp
        propertyview/PropertyViewEditor.hpp
        propertyview/PropertyViewDialog.hpp
    )

    # Files to moc and compile:
    set(
        MocHeaderFiles
        RobWorkStudioPlugin.hpp
        RobWorkStudio.hpp
        SceneOpenGLViewer.hpp
        AboutBox.hpp
        RWStudioView3D.hpp
        ImageView.hpp
        propertyview/PropertyViewEditor.hpp
        propertyview/PropertyViewDialog.hpp
    )

    if(RWS_USE_PYTHON)
        set(SRC_FILES_CPP ${SRC_FILES_CPP} pythonpluginloader/PyPlugin.cpp)
        set(SRC_FILES_HPP ${SRC_FILES_HPP} pythonpluginloader/PyPlugin.hpp)
        set(MocHeaderFiles ${MocHeaderFiles} pythonpluginloader/PyPlugin.hpp)
    endif()

    set_source_files_properties(${SRC_FILES_CPP} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    # Add the cmake binary dir because this is where the ui files are placed MESSAGE(STATUS
    # "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    set(MocSrcFiles)
    if(RWS_USE_QT5)
        qt5_wrap_cpp(MocSrcFiles ${MocHeaderFiles})
    else()
        qt4_wrap_cpp(
            MocSrcFiles
            ${MocHeaderFiles}
            OPTIONS
            -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
            -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
            -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
            -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
        )
    endif()
    set_source_files_properties(${MocSrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    # Resource files to rcc and compile:
    if(RWS_BUILD_SANDBOX)
        # SET(SandboxResourceFiles rwstudio_sandbox_resources.qrc)
    endif()
    # Rcc the files:
    if(RWS_USE_QT5)
        qt5_add_resources(ResourceFiles rwstudio_resources.qrc)
    else()
        qt4_add_resources(ResourceFiles rwstudio_resources.qrc)
    endif()
    # We create a static library named rwstudio that all plugins should link to. Our RobWorkStudio executable
    # links to this library also.
    rw_add_library(
        ${SUBSYS_NAME}
        ${UIS_OUT_H}
        ${SRC_FILES_CPP}
        ${MocSrcFiles}
        ${ResourceFiles}
    )

    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC
            qtpropertybrowser
            ${PYTHON_LIBRARIES}
            ${QT_LIBRARIES}
            ${ROBWORK_LIBRARIES}
            ${GLUT_glut_LIBRARY}
    )

    include_directories(${PYTHON_INCLUDE_DIRS})
    set_target_properties(sdurws PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

    # Install stuff
    install(TARGETS sdurws DESTINATION ${LIB_INSTALL_DIR})
    install(FILES ${SRC_FILES_HPP} DESTINATION ${INCLUDE_INSTALL_DIR}/sdurws)

    add_library(sdurws_plugin.rwplugin MODULE RWSImageLoaderPlugin.cpp RWSImageLoaderPlugin.hpp)
    target_link_libraries(sdurws_plugin.rwplugin sdurws ${QT_LIBRARIES} ${ROBWORK_LIBRARIES})
    install(TARGETS sdurws_plugin.rwplugin DESTINATION ${RW_PLUGIN_INSTALL_DIR})

endif()
