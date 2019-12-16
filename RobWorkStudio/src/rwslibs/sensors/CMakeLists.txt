set(SUBSYS_NAME sdurws_sensors)
set(SUBSYS_DESC "A sensor view plugin")
set(SUBSYS_DEPS sdurws)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # if we want to use ui files add them here
    set(UIS_FILES SensorsPlugin.ui)
    if(RWS_USE_QT5)
        qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    else()
        qt4_wrap_ui(UIS_OUT_H ${UIS_FILES})
    endif()
    # Need to add the current binary dir to the include directory because the generated source files are
    # placed here
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    # Standard cpp files to compile:
    set(SrcFiles Sensors.cpp SensorView.cpp)
    set(SRC_FILES_HPP Sensors.hpp SensorView.hpp)

    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

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
        ${UIS_OUT_H}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/sensors" ${SRC_FILES_HPP})
    add_dependencies(${SUBSYS_NAME} ${SUBSYS_DEPS})

    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    endif()
endif()
