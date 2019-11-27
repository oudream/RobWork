set(SUBSYS_NAME sdurw_lua)
set(SUBSYS_DESC "Interface for accessing RobWork from lua.")
set(SUBSYS_DEPS sdurw)

set(build TRUE)

find_package(SWIG 3.0.0 QUIET)
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

    add_custom_target(${SUBSYS_NAME}_all)
    set(RW_NAME sdurw)
    foreach(
        RW_MODULE
        ${RW_NAME}
        ${RW_NAME}_assembly
        ${RW_NAME}_control
        ${RW_NAME}_pathoptimization
        ${RW_NAME}_pathplanners
        ${RW_NAME}_proximitystrategies
        ${RW_NAME}_task
        ${RW_NAME}_simulation
        ${RW_NAME}_opengl
    )

        set(RW_MODULE_FILENAME ../${RW_MODULE}.i)
        set(TARGET_NAME ${RW_MODULE}_lua)

        set_source_files_properties(${RW_MODULE_FILENAME} PROPERTIES CPLUSPLUS ON)
        set_source_files_properties(${RW_MODULE_FILENAME} PROPERTIES SWIG_FLAGS "-includeall")

        configure_file(Lua.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/Lua_${RW_MODULE}.hpp @ONLY)
        configure_file(Lua.cpp.in Lua_${RW_MODULE}.cpp @ONLY)
        configure_file(LuaPlugin.cpp.in LuaPlugin_${RW_MODULE}.cpp @ONLY)

        set(SOURCE)
        if("${RW_MODULE}" STREQUAL "sdurw")
            set(SOURCE ../ScriptTypes.cpp LuaState.cpp)
        endif()
        set(SOURCE ${SOURCE} Lua_${RW_MODULE}.cpp)

        # ############ lua interface generation ##############
        if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
            swig_add_library(${TARGET_NAME} TYPE SHARED LANGUAGE lua SOURCES ${RW_MODULE_FILENAME} ${SOURCE})
        else()
            swig_add_module(${TARGET_NAME} lua ${RW_MODULE_FILENAME} ${SOURCE})
        endif()

        if(NOT DEFINED MSVC)
            set_target_properties(
                ${TARGET_NAME}
                PROPERTIES
                    PREFIX ""
                    OUTPUT_NAME ${RW_MODULE}
                    ARCHIVE_OUTPUT_DIRECTORY "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                    LIBRARY_OUTPUT_DIRECTORY "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                    LIBRARY_OUTPUT_DIRECTORY "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
            )
        endif()

        target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        add_dependencies(${SUBSYS_NAME}_all ${TARGET_NAME})

        # ############ lua static interface generation ##############
        if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
            set_target_properties(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
        endif()

        if((CMAKE_VERSION VERSION_GREATER 3.12.0) OR (CMAKE_VERSION VERSION_EQUAL 3.12.0))
            swig_add_library(${TARGET_NAME}_s TYPE STATIC LANGUAGE lua SOURCES ${RW_MODULE_FILENAME} ${SOURCE})
        else()
            add_library(${TARGET_NAME}_s STATIC ${SOURCE} ${swig_generated_sources} ${swig_other_sources})
            add_dependencies(${TARGET_NAME}_s sdurw_lua) # avoid using the source files before they have been generated
        endif()
        target_include_directories(${TARGET_NAME}_s PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        add_dependencies(${SUBSYS_NAME}_all ${TARGET_NAME}_s)

        # this is used to indicate static linking to Visual Studio or mingw
        if(DEFINED MSVC)
            set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
        else()
            set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
        endif()

        # Add rwplugin
        if(NOT "${RW_MODULE}" STREQUAL "sdurw")
            add_library(${RW_MODULE}_lua_plugin.rwplugin MODULE LuaPlugin_${RW_MODULE}.cpp)
            target_link_libraries(${RW_MODULE}_lua_plugin.rwplugin ${RW_MODULE}_lua_s)
            target_include_directories(${RW_MODULE}_lua_plugin.rwplugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
            if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
                set_target_properties(${RW_MODULE}_lua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
            endif()
            add_dependencies(${SUBSYS_NAME}_all ${RW_MODULE}_lua_plugin.rwplugin)
        endif()

        install(
            TARGETS ${TARGET_NAME} ${TARGET_NAME}_s
            RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT swig
            LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig
            ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig
        )

        install(TARGETS ${TARGET_NAME} DESTINATION ${LUA_INSTALL_DIR})

    endforeach()

    swig_link_libraries(${RW_NAME}_lua sdurw ${LUA_LIBRARIES})
    swig_link_libraries(${RW_NAME}_assembly_lua sdurw_assembly sdurw ${LUA_LIBRARIES})
    swig_link_libraries(${RW_NAME}_control_lua sdurw_control sdurw ${LUA_LIBRARIES})
    swig_link_libraries(${RW_NAME}_pathoptimization_lua sdurw_pathoptimization sdurw ${LUA_LIBRARIES})
    swig_link_libraries(${RW_NAME}_pathplanners_lua sdurw_pathplanners sdurw ${LUA_LIBRARIES})
    swig_link_libraries(${RW_NAME}_proximitystrategies_lua sdurw_proximitystrategies sdurw ${LUA_LIBRARIES})
    swig_link_libraries(${RW_NAME}_simulation_lua sdurw_simulation sdurw ${LUA_LIBRARIES})
    swig_link_libraries(${RW_NAME}_task_lua sdurw_task sdurw ${LUA_LIBRARIES})
    swig_link_libraries(${RW_NAME}_opengl_lua sdurw_opengl sdurw ${LUA_LIBRARIES})

    target_include_directories(${RW_NAME}_lua PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    target_link_libraries(${RW_NAME}_lua_s sdurw ${LUA_LIBRARIES})
    target_link_libraries(${RW_NAME}_assembly_lua_s sdurw_assembly ${RW_NAME}_lua_s ${LUA_LIBRARIES})
    target_link_libraries(${RW_NAME}_control_lua_s sdurw_control ${RW_NAME}_lua_s)
    target_link_libraries(${RW_NAME}_pathoptimization_lua_s sdurw_pathoptimization ${RW_NAME}_lua_s ${LUA_LIBRARIES})
    target_link_libraries(${RW_NAME}_pathplanners_lua_s sdurw_pathplanners ${RW_NAME}_lua_s ${LUA_LIBRARIES})
    target_link_libraries(${RW_NAME}_proximitystrategies_lua_s sdurw_proximitystrategies ${RW_NAME}_lua_s
                          ${LUA_LIBRARIES})
    target_link_libraries(${RW_NAME}_simulation_lua_s sdurw_simulation ${RW_NAME}_lua_s ${LUA_LIBRARIES})
    target_link_libraries(${RW_NAME}_task_lua_s sdurw_task ${RW_NAME}_lua_s ${LUA_LIBRARIES})
    target_link_libraries(${RW_NAME}_opengl_lua_s sdurw_opengl ${RW_NAME}_lua_s ${LUA_LIBRARIES})

    target_include_directories(${RW_NAME}_lua_s PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

endif()
