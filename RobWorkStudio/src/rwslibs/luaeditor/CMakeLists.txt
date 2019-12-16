set(SUBSYS_NAME sdurws_luaeditor)
set(SUBSYS_DESC "A lua editor plugin")
set(SUBSYS_DEPS sdurws)

set(build ${RWS_HAVE_LUA})
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RWS_HAVE_LUA} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # if we want to use ui files add them here
    set(UIS_FILES LuaEditorWindow.ui)
    if(RWS_USE_QT5)
        qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    else()
        qt4_wrap_ui(UIS_OUT_H ${UIS_FILES})
    endif()
    # Need to add the current binary dir to the include directory because the generated source files are
    # placed here
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    set(
        SrcFiles
        LuaEditorWindow.cpp
        LuaHighlighter.cpp
        CodeEditor.cpp
        TreeModelCompleter.cpp
        LuaExecutionThread.cpp
        LuaConsoleWidget.cpp
    )
    set(
        SRC_FILES_HPP
        LuaEditorWindow.hpp
        LuaHighlighter.hpp
        CodeEditor.hpp
        TreeModelCompleter.hpp
        LuaExecutionThread.hpp
        LuaConsoleWidget.hpp
    )

    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    if(RWS_USE_QT5)
        qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP})
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
    endif()

    rws_add_component(
        ${SUBSYS_NAME}
        ${SrcFiles}
        ${MocSrcFiles}
        ${UIS_OUT_H}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/luaeditor" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} ${SUBSYS_DEPS})

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS SUBSYS_DEPS)
        if(TARGET ${DEP})
            add_dependencies(${SUBSYS_NAME} ${DEP})
        endif()
    endforeach()

    set(RWS_COMPONENT_LIBRARIES ${RWS_COMPONENT_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
