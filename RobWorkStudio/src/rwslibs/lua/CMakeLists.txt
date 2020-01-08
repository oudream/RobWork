set(SUBSYS_NAME sdurws_luapl)
set(SUBSYS_DESC "A lua plugin")
set(SUBSYS_DEPS sdurws sdurws_luaeditor sdurws_lua_s)

set(build ${RWS_HAVE_LUA})
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RWS_HAVE_LUA} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # Standard cpp files to compile:
    set(SrcFiles Lua.cpp)
    set(SRC_FILES_HPP Lua.hpp)

    qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP} TARGET ${SUBSYS_NAME})
    # Rcc the files:
    qt5_add_resources(RccSrcFiles resources.qrc)

    # Set extra linker flags for apple macos computers
    rws_add_plugin(
        ${SUBSYS_NAME}
        ${RWS_DEFAULT_LIB_TYPE}
        ${SrcFiles}
        ${MocSrcFiles}
        ${RccSrcFiles}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/lua" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurws ${QT_LIBRARIES} ${ROBWORK_LIBRARIES} PRIVATE sdurws_luaeditor sdurws_lua_s)

    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    endif()
endif()
