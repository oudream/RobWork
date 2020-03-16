set(SUBSYS_NAME sdurws_luaeditor)
set(SUBSYS_DESC "A lua editor plugin")
set(SUBSYS_DEPS sdurws RW::sdurw RW::sdurw_lua_s)

set(build ${RWS_HAVE_LUA})
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RWS_HAVE_LUA} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # if we want to use ui files add them here
    set(UIS_FILES LuaEditorWindow.ui)
    qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})

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

    qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP})

    rws_add_component(
        ${SUBSYS_NAME}
        ${SrcFiles}
        ${MocSrcFiles}
        ${UIS_OUT_H}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/luaeditor" ${SRC_FILES_HPP})
    # Need to add the current binary dir to the include directory because the generated source files are placed here
    target_include_directories(${SUBSYS_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} ${QT_LIBRARIES})
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
