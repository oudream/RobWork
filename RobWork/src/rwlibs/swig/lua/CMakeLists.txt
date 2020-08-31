set(SUBSYS_NAME sdurw_lua)
set(SUBSYS_DESC "Interface for accessing RobWork from lua.")
set(SUBSYS_DEPS)

set(build TRUE)

find_package(SWIG 3.0.0 QUIET)
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
    set(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
    # MESSAGE(STATUS "SWIG found adding swig modules!")
    include(UseSWIG)

    set(CMAKE_SWIG_FLAGS "")

    add_custom_target(${SUBSYS_NAME}_all)
    include_directories(${RW_ROOT}/src)

    foreach(
        RW_MODULE
        sdurw_core
        sdurw
        sdurw_task
        sdurw_assembly
        sdurw_control
        sdurw_pathplanners
        sdurw_proximitystrategies
        sdurw_pathoptimization
        sdurw_simulation
        sdurw_opengl
    )
        if(TARGET ${RW_MODULE})
            configure_file(Lua.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/Lua_${RW_MODULE}.hpp @ONLY)
            configure_file(Lua.cpp.in Lua_${RW_MODULE}.cpp @ONLY)
            configure_file(LuaPlugin.cpp.in LuaPlugin_${RW_MODULE}.cpp @ONLY)

            set(SOURCE)
            if("${RW_MODULE}" STREQUAL "sdurw")
                set(SOURCE ../ScriptTypes.cpp)
            endif()
            if("${RW_MODULE}" STREQUAL "sdurw_core")
                set(SOURCE ${SOURCE} LuaState.cpp)
            endif()
            set(SOURCE ${SOURCE} Lua_${RW_MODULE}.cpp)

            # ############ lua interface generation ##############

            set(TARGET_NAME ${RW_MODULE}_lua)
            rw_add_swig(
                ${RW_MODULE} lua SHARED
                TARGET_NAME ${TARGET_NAME}
                SOURCES ${SOURCE}
                INSTALL_DIR ${LUA_INSTALL_DIR}
            )

            if(NOT DEFINED MSVC) # This code is to make it easier to make debian packages
                set_target_properties(
                    ${TARGET_NAME}
                    PROPERTIES
                        PREFIX ""
                        OUTPUT_NAME ${RW_MODULE}
                        ARCHIVE_OUTPUT_DIRECTORY "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                        RUNTIME_OUTPUT_DIRECTORY "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                        LIBRARY_OUTPUT_DIRECTORY "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
                )
            endif()

            if(NOT RW_ENABLE_INTERNAL_LUA_TARGET)
                target_include_directories(${TARGET_NAME} PUBLIC ${LUA_INCLUDE_DIR})
            endif()

            add_dependencies(${SUBSYS_NAME}_all ${TARGET_NAME})

            # ############ lua static interface generation ##############
            rw_add_swig(
                ${RW_MODULE} lua STATIC
                TARGET_NAME ${TARGET_NAME}_s
                SOURCES ${SOURCE}
                INSTALL_DIR ${STATIC_LIB_INSTALL_DIR}
            )

            if(NOT RW_ENABLE_INTERNAL_LUA_TARGET)
                target_include_directories(${TARGET_NAME}_s PUBLIC ${LUA_INCLUDE_DIR})
            endif()

            add_dependencies(${SUBSYS_NAME}_all ${TARGET_NAME}_s)

            # this is used to indicate static linking to Visual Studio or mingw
            if(DEFINED MSVC)
                set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
            else()
                set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
            endif()

            # ############### Add rwplugin #####################
            if(NOT "${RW_MODULE}" STREQUAL "sdurw" AND NOT "${RW_MODULE}" STREQUAL "sdurw_core")
                add_library(${RW_MODULE}_lua_plugin.rwplugin MODULE LuaPlugin_${RW_MODULE}.cpp)
                target_link_libraries(${RW_MODULE}_lua_plugin.rwplugin ${RW_MODULE}_lua_s)
                target_include_directories(
                    ${RW_MODULE}_lua_plugin.rwplugin
                    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                )
                if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
                    set_target_properties(
                        ${RW_MODULE}_lua_plugin.rwplugin
                        PROPERTIES LINK_FLAGS -Wl,--no-undefined
                    )
                endif()
                add_dependencies(${SUBSYS_NAME}_all ${RW_MODULE}_lua_plugin.rwplugin)
                install(
                    TARGETS ${RW_MODULE}_lua_plugin.rwplugin
                    LIBRARY DESTINATION ${RW_PLUGIN_INSTALL_DIR}
                )
            endif()

            get_in_map(depends RW_SUBSYS_DEPEND ${RW_MODULE})
            swig_link_libraries(${RW_MODULE}_lua ${RW_MODULE} ${depends} ${LUA_LIBRARIES})
            if(CMAKE_VERSION VERSION_GREATER 3.12.0)
                list(TRANSFORM depends APPEND "_lua_s")
            else()
                set(cpd)
                foreach(dep ${depends})
                    set(cpd ${cpd} "${dep}_lua_s")
                endforeach()
                set(depends "${cpd}")
            endif()
            target_link_libraries(${RW_MODULE}_lua_s ${RW_MODULE} ${depends} ${LUA_LIBRARIES})
        endif()

    endforeach()

    if(TARGET sdurw_lua)
        target_include_directories(sdurw_lua PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
        target_include_directories(sdurw_lua_s PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    endif()
    if(TARGET sdurw_core_lua)
        target_include_directories(sdurw_core_lua PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
        target_include_directories(sdurw_core_lua_s PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    endif()

endif()
