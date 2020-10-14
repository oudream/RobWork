set(SUBSYS_NAME sdurws_lua)
set(SUBSYS_DESC "Interface for accessing RobWorkStudio from lua.")
set(SUBSYS_DEPS sdurws_robworkstudioapp RW::sdurw_lua RW::sdurw_core_lua)

set(build TRUE)

set(DEFAULT TRUE)
set(REASON)
if(NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(RWSIM_HAVE_LUA
        TRUE
        CACHE INTERNAL ""
    )

    include(UseSWIG)

    include_directories(${RW_ROOT}/src)

    set(RWS_MODULE sdurws)
    set(TARGET_NAME ${RWS_MODULE}_lua)
    # ############ lua interface generation ##############

    rw_add_swig(
        ${RWS_MODULE} lua SHARED
        TARGET_NAME ${TARGET_NAME}
        SOURCES ../sdurws.i Lua.cpp ../ScriptTypes.cpp
        INSTALL_DIR ${LUA_INSTALL_DIR}
    )
    unset(SLIB_COMPILE_BUFFER CACHE)
    if(NOT DEFINED MSVC) # This code is to make it easier to make debian packages
        set_target_properties(
            ${TARGET_NAME}
            PROPERTIES PREFIX ""
                       OUTPUT_NAME ${RWS_MODULE}
                       ARCHIVE_OUTPUT_DIRECTORY "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                       LIBRARY_OUTPUT_DIRECTORY "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                       LIBRARY_OUTPUT_DIRECTORY "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
        )
    endif()

    swig_link_libraries(
        ${TARGET_NAME}
        ${SUBSYS_DEPS}
        ${RWS_PLUGIN_LIBRARIES}
        ${RWS_MODULE}
        RW::sdurw_lua
        RW::sdurw
        ${RW_BUILD_WITH_LIBRARIES_LUA}
    )

    target_include_directories(${TARGET_NAME} PUBLIC ${RW_BUILD_WITH_LUA_INCLUDE_DIR})

    # ############ lua static interface generation ##############

    rw_add_swig(
        ${RWS_MODULE} lua STATIC
        TARGET_NAME ${TARGET_NAME}_s
        SOURCES ../sdurws.i Lua.cpp ../ScriptTypes.cpp
        INSTALL_DIR ${STATIC_LIB_INSTALL_DIR}
    )

    target_link_libraries(
        ${TARGET_NAME}_s
        ${SUBSYS_DEPS}
        ${RWS_PLUGIN_LIBRARIES}
        ${RWS_MODULE}
        RW::sdurw
        RW::sdurw_lua_s
        RW::sdurw_core_lua_s
        ${RW_BUILD_WITH_LIBRARIES_LUA}
    )
    target_include_directories(${TARGET_NAME}_s PUBLIC ${RW_BUILD_WITH_LUA_INCLUDE_DIR})

    # Visual Studio or mingw this is used to indicate static linking to Visual Studio or mingw
    if(DEFINED MSVC)
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
    else()
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
    endif()

    add_library(sdurwslua_plugin.rwplugin MODULE LuaPlugin.cpp LuaPlugin.hpp)
    target_link_libraries(sdurwslua_plugin.rwplugin ${TARGET_NAME}_s RW::sdurw RW::sdurw_lua_s)
    install(TARGETS ${RWS_MODULE}lua_plugin.rwplugin DESTINATION ${RW_PLUGIN_INSTALL_DIR})

    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(sdurwslua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()
endif()
