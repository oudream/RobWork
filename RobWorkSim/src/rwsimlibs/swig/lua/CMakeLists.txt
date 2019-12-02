set(SUBSYS_NAME sdurwsim_lua)
set(SUBSYS_DESC "Interface for accessing RobWorkSim from lua.")
set(SUBSYS_DEPS sdurw)

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

    set(CMAKE_SWIG_FLAGS "")

    set_source_files_properties(../sdurwsim.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(../sdurwsim.i PROPERTIES SWIG_FLAGS "-includeall")

    set(RWSIM_MODULE sdurwsim)
    set(TARGET_NAME ${RWSIM_MODULE}_lua)

    # ############ lua interface generation ##############
    if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
        swig_add_library(
            ${TARGET_NAME}
            TYPE SHARED
            LANGUAGE lua
            SOURCES ../sdurwsim.i ../ScriptTypes.cpp Lua.cpp
        )
    else()
        swig_add_module(
            ${TARGET_NAME}
            lua
            ../sdurwsim.i
            ../ScriptTypes.cpp
            Lua.cpp
        )
    endif()

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

    swig_link_libraries(${TARGET_NAME} ${RWSIM_ODE_LIBRARY} ${RWSIM_MODULE} ${ROBWORK_LIBRARIES})
    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(${TARGET_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()

    # ############ lua static interface generation ##############
    if((CMAKE_VERSION VERSION_GREATER 3.12.0) OR (CMAKE_VERSION VERSION_EQUAL 3.12.0))
        swig_add_library(
            ${TARGET_NAME}_s
            TYPE STATIC
            LANGUAGE lua
            SOURCES ../sdurwsim.i ../ScriptTypes.cpp Lua.cpp
        )
    else()
        add_library(${TARGET_NAME}_s STATIC Lua.cpp ${swig_generated_sources} ${swig_other_sources})
        add_dependencies(${TARGET_NAME}_s ${TARGET_NAME}) # avoid using the source files before they have been
                                                          # generated
    endif()
    target_link_libraries(${TARGET_NAME}_s ${RWSIM_ODE_LIBRARY} ${RWSIM_MODULE} ${ROBWORK_LIBRARIES})

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS SUBSYS_DEPS)
        if(TARGET ${DEP})
            add_dependencies(${TARGET_NAME}_s ${DEP})
        endif()
    endforeach()

    # this is used to indicate static linking to Visual Studio or mingw
    if(DEFINED MSVC)
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
    else()
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
    endif()

    add_library(sdurwsimlua_plugin.rwplugin MODULE LuaPlugin.cpp LuaPlugin.hpp)
    target_link_libraries(sdurwsimlua_plugin.rwplugin ${TARGET_NAME}_s ${ROBWORK_LIBRARIES})

    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(sdurwsimlua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()

    # ############ INSTALL LUA LIBRARY #############################
    install(TARGETS ${TARGET_NAME} DESTINATION ${LUA_INSTALL_DIR})

endif()
