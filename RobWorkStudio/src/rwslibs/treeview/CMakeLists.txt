set(SUBSYS_NAME sdurws_treeview)
set(SUBSYS_DESC "A treeview plugin")
set(SUBSYS_DEPS sdurws)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # Standard cpp files to compile:
    set(SrcFiles TreeView.cpp)
    set(SRC_FILES_HPP TreeView.hpp)

    if(RWS_USE_QT5)
        qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP})
        # Rcc the files:
        qt5_add_resources(RccSrcFiles resources.qrc)
    else()
        # Header files to moc and compile:
        qt4_wrap_cpp(
            MocSrcFiles
            ${SRC_FILES_HPP}
            OPTIONS
            -DBOOST_TT_HAS_PLUS_HPP_INCLUDED
            -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED
            -DBOOST_TT_HAS_MINUS_HPP_INCLUDED
            -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED
        )
        # Rcc the files:
        qt4_add_resources(RccSrcFiles resources.qrc)
    endif()

    rws_add_plugin(
        ${SUBSYS_NAME}
        ${RWS_DEFAULT_LIB_TYPE}
        ${SrcFiles}
        ${MocSrcFiles}
        ${RccSrcFiles}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/jog" ${SRC_FILES_HPP})
    add_dependencies(${SUBSYS_NAME} ${SUBSYS_DEPS})

    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    endif()
endif()
