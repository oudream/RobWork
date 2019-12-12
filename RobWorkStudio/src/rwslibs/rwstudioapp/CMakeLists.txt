set(SUBSYS_NAME sdurws_robworkstudioapp)
set(SUBSYS_DESC "A rwstudio application that may be started in a thread.")
set(SUBSYS_DEPS sdurws)

if(RWS_USE_STATIC_LINK_PLUGINS)
    list(
        APPEND
            SUBSYS_DEPS
            sdurws_jog
            sdurws_log
            sdurws_planning
            sdurws_playback
            sdurws_sensors
            sdurws_treeview
            sdurws_propertyview
    )
    if(RWS_HAVE_LUA)
        list(APPEND SUBSYS_DEPS sdurws_luaeditor sdurws_luapl)
    endif()
endif()

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # Standard cpp files to compile:
    set(SrcFiles RobWorkStudioApp.cpp)
    set(SRC_FILES_HPP RobWorkStudioApp.hpp)

    rws_add_component(${SUBSYS_NAME} ${SrcFiles})
    rw_add_includes(${SUBSYS_NAME} "rwslibs/rwstudioapp" ${SRC_FILES_HPP})
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
