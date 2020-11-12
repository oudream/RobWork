set(SUBSYS_NAME sdurwsim_lua)
set(SUBSYS_DESC "Interface for accessing RobWorkSim from lua.")
set(SUBSYS_DEPS RW::sdurw_lua RWS::sdurws_lua sdurwsim)

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
rw_add_doc(${SUBSYS_NAME})

if(build)
    set(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
    # MESSAGE(STATUS "SWIG found adding swig modules!")
    include(UseSWIG)

    include_directories(${RWSIM_ROOT}/src ${RW_ROOT}/src)

    set(RWSIM_MODULE sdurwsim)
    set(TARGET_NAME ${RWSIM_MODULE}_lua)

    # ############ lua interface generation ##############
    rw_add_swig(
        ${RWSIM_MODULE} lua SHARED 
        TARGET_NAME ${TARGET_NAME}
        SOURCES ../sdurwsim.i Lua.cpp ../ScriptTypes.cpp
        INSTALL_DIR ${LUA_INSTALL_DIR}
    )


    if(NOT DEFINED MSVC) # This code is to make it easier to make debian packages
        set_target_properties(
            ${TARGET_NAME}
            PROPERTIES
                PREFIX ""
                OUTPUT_NAME ${RWSIM_MODULE}
                ARCHIVE_OUTPUT_DIRECTORY "${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                LIBRARY_OUTPUT_DIRECTORY "${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                LIBRARY_OUTPUT_DIRECTORY "${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
        )
    endif()

    swig_link_libraries(${TARGET_NAME} ${RWSIM_ODE_LIBRARY} ${RWSIM_MODULE} ${RW_BUILD_WITH_LIBRARIES_LUA})
    target_include_directories(${TARGET_NAME} PUBLIC ${RW_BUILD_WITH_LUA_INCLUDE_DIR})



    # ############ lua static interface generation ##############
    rw_add_swig(
        ${RWSIM_MODULE} lua STATIC 
        TARGET_NAME ${TARGET_NAME}_s
        SOURCES ../sdurwsim.i Lua.cpp ../ScriptTypes.cpp
        INSTALL_DIR ${LUA_INSTALL_DIR}
    )

    target_link_libraries(${TARGET_NAME}_s ${RWSIM_ODE_LIBRARY} ${RWSIM_MODULE} ${RW_BUILD_WITH_LIBRARIES_LUA})
    target_include_directories(${TARGET_NAME}_s PUBLIC ${RW_BUILD_WITH_LUA_INCLUDE_DIR})

    # this is used to indicate static linking to Visual Studio or mingw
    if(DEFINED MSVC)
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
    else()
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
    endif()

    add_library(sdurwsimlua_plugin.rwplugin MODULE LuaPlugin.cpp LuaPlugin.hpp)
    set_target_properties(sdurwsimlua_plugin.rwplugin PROPERTIES EXCLUDE_FROM_ALL TRUE)

    target_link_libraries(sdurwsimlua_plugin.rwplugin ${TARGET_NAME}_s)
    install(TARGETS ${RWSIM_MODULE}lua_plugin.rwplugin DESTINATION ${RW_PLUGIN_INSTALL_DIR})

    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(sdurwsimlua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()

    # ############ INSTALL LUA LIBRARY #############################
    install(TARGETS ${TARGET_NAME} DESTINATION ${LUA_INSTALL_DIR})

    add_custom_target(
        lua
        COMMAND ${CMAKE_COMMAND} -E echo "Done Building Lua"
        DEPENDS ${TARGET_NAME} ${TARGET_NAME}_s sdurwsimlua_plugin.rwplugin
    )

endif()
