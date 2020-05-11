set(SUBSYS_NAME sdurws_luapl)
set(SUBSYS_DESC "A lua plugin")
set(SUBSYS_DEPS sdurws sdurws_luaeditor RW::sdurw_lua RW::sdurw)

set(DEFAULT ${RWS_HAVE_LUA})
set(REASON)

# Verify that sdurws_lua_s will be build
if(
    ${DEFAULT}
    AND (DEFINED BUILD_sdurws_robworkstudioapp AND NOT ${BUILD_sdurws_robworkstudioapp})
    AND (DEFINED BUILD_sdurws_lua AND NOT ${BUILD_sdurws_lua})
)
    set(DEFAULT FALSE)
    set(REASON "Unmet dependencies sdurws_lua or sdurws_robworkstudioapp")
endif()

set(build ${RWS_HAVE_LUA})
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} REASON ${REASON} ADD_DOC)

if(build)
    # Standard cpp files to compile:
    set(SrcFiles Lua.cpp)
    set(SRC_FILES_HPP Lua.hpp)

    #qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP} TARGET ${SUBSYS_NAME})
    # Rcc the files:
    qt5_add_resources(RccSrcFiles resources.qrc)

    # Set extra linker flags for apple macos computers
    rws_add_plugin(${SUBSYS_NAME} ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    rw_add_includes(${SUBSYS_NAME} "rwslibs/lua" ${SRC_FILES_HPP})
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC sdurws ${QT_LIBRARIES} RW::sdurw
        PRIVATE sdurws_luaeditor sdurws_lua_s
    )
    target_include_directories(
        ${SUBSYS_NAME}
        INTERFACE $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rws_plugin_load_details(${SUBSYS_NAME} 2 Lua false)
    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    else()
        set_target_properties(${SUBSYS_NAME} PROPERTIES INSTALL_RPATH "${LUA_INSTALL_DIR}")
    endif()
endif()
