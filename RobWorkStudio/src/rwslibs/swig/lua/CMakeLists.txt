set(SUBSYS_NAME sdurws_lua)
set(SUBSYS_DESC "Interface for accessing RobWorkStudio from lua.")
set(SUBSYS_DEPS sdurw)

set(build TRUE)

set(DEFAULT TRUE)
set(REASON)
if(NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

endif()

rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
rw_add_doc(${SUBSYS_NAME})

if(build)
    set(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
    # MESSAGE(STATUS "SWIG found adding swig modules!")
    include(UseSWIG)

    set(CMAKE_SWIG_FLAGS "")

    set_source_files_properties(../sdurws.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(../sdurws.i PROPERTIES SWIG_FLAGS "-includeall")

    set(RWS_MODULE sdurws)
    set(TARGET_NAME ${RWS_MODULE}_lua)

    # ############ lua interface generation ##############
    if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
        swig_add_library(${TARGET_NAME} TYPE SHARED LANGUAGE lua SOURCES ../sdurws.i Lua.cpp ../ScriptTypes.cpp)
    else()
        swig_add_module(${TARGET_NAME} lua ../sdurws.i Lua.cpp ../ScriptTypes.cpp)
    endif()
    
    set_target_properties(
        ${TARGET_NAME}
        PROPERTIES
            PREFIX ""
            OUTPUT_NAME ${RWS_MODULE}
            ARCHIVE_OUTPUT_DIRECTORY "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
            LIBRARY_OUTPUT_DIRECTORY "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
            LIBRARY_OUTPUT_DIRECTORY "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
    )
    swig_link_libraries(${TARGET_NAME} ${RWS_COMPONENT_LIBRARIES} ${RWS_PLUGIN_LIBRARIES} ${RWS_MODULE}
                        ${ROBWORK_LIBRARIES})
    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()

    # ############ lua static interface generation ##############
    if((CMAKE_VERSION VERSION_GREATER 3.12.0) OR (CMAKE_VERSION VERSION_EQUAL 3.12.0))
        swig_add_library(${TARGET_NAME}_s TYPE STATIC LANGUAGE lua SOURCES ../sdurws.i Lua.cpp ../ScriptTypes.cpp)
    else()
        add_library(${TARGET_NAME}_s STATIC ${swig_generated_sources} ${swig_other_sources})
    endif()
    target_link_libraries(${TARGET_NAME}_s ${RWS_COMPONENT_LIBRARIES} ${RWS_PLUGIN_LIBRARIES} ${RWS_MODULE}
                          ${ROBWORK_LIBRARIES})

    set(RWS_COMPONENT_LIBRARIES ${RWS_COMPONENT_LIBRARIES} ${TARGET_NAME}_s PARENT_SCOPE)

    # Visual Studio or mingw this is used to indicate static linking to Visual Studio or mingw
    if(DEFINED MSVC)
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
    else()
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
    endif()

    add_library(sdurwslua_plugin.rwplugin MODULE LuaPlugin.cpp LuaPlugin.hpp)
    target_link_libraries(sdurwslua_plugin.rwplugin ${TARGET_NAME}_s ${ROBWORK_LIBRARIES})

    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(sdurwslua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()

    # ############ INSTALL LUA LIBRARY #############################
    install(
        TARGETS ${TARGET_NAME}
        DESTINATION ${LUA_INSTALL_DIR}
    )

    # ################# Make sure dependencies are build before system #############
    foreach(DEP IN LISTS SUBSYS_DEPS)
        if(TARGET ${DEP})
            add_dependencies(${TARGET_NAME} ${DEP})
            add_dependencies(${TARGET_NAME}_s ${DEP})
        endif()
    endforeach()
endif()
