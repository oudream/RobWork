set(SUBSYS_NAME sdurwsim_gui)
set(SUBSYS_DESC "RobWorkSim gui")
set(SUBSYS_DEPS )

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # ##############################################################################################
    # first compile the general stuff into a static gui lib
    set(
        FILES_UI
        RestingPoseDialog.ui
        GraspSelectionDialog.ui
        GraspRestingPoseDialog.ui
        SupportPoseAnalyserDialog.ui
        CreateEngineDialog.ui
        SimCfgDialog.ui
        SDHPlugin.ui
        TactileSensorDialog.ui
        # GraspTableGeneratorPlugin.ui
        log/SimulatorLogWidget.ui
        log/BodyMotionWidget.ui
        log/CollisionResultWidget.ui
        log/DistanceResultWidget.ui
        log/ConstraintWidget.ui
        log/ForceTorqueWidget.ui
        log/ContactSetWidget.ui
        log/ContactVelocitiesWidget.ui
        log/EquationSystemWidget.ui
        log/LogValuesWidget.ui
        log/SimulatorStatisticsWidget.ui
    )

    set(
        SrcFiles_HPP
        ContactTableWidget.hpp
        JointControlDialog.hpp
        JogGroup.hpp
        RestingPoseDialog.hpp
        GraspSelectionDialog.hpp
        GraspRestingPoseDialog.hpp
        SupportPoseAnalyserDialog.hpp
        GLViewRW.hpp
        CreateEngineDialog.hpp
        SimCfgDialog.hpp
        BodyControllerWidget.hpp
        TactileSensorDialog.hpp
        log/SimulatorLogWidget.hpp
        log/SimulatorLogModel.hpp
        log/SimulatorLogEntryWidget.hpp
        log/BodyMotionWidget.hpp
        log/CollisionResultWidget.hpp
        log/DistanceMultiResultWidget.hpp
        log/DistanceResultWidget.hpp
        log/ConstraintWidget.hpp
        log/ForceTorqueWidget.hpp
        log/ContactSetWidget.hpp
        log/ContactVelocitiesWidget.hpp
        log/EquationSystemWidget.hpp
        log/LogValuesWidget.hpp
        log/LogMessageWidget.hpp
        log/SimulatorLogWidget.hpp
        log/SimulatorStatisticsWidget.hpp
    )

    set(
        SrcFiles
        ContactTableWidget.cpp
        JointControlDialog.cpp
        JogGroup.cpp
        BodyControllerWidget.cpp
        RestingPoseDialog.cpp
        GraspSelectionDialog.cpp
        GraspRestingPoseDialog.cpp
        SupportPoseAnalyserDialog.cpp
        GLViewRW.cpp
        CreateEngineDialog.cpp
        SimCfgDialog.cpp
        # TactilePadItem.cpp
        TactileSensorDialog.cpp
        log/SimulatorLogModel.cpp
        log/SimulatorLogEntryWidget.cpp
        log/SimulatorLogWidget.cpp
        log/BodyMotionWidget.cpp
        log/CollisionResultWidget.cpp
        log/DistanceMultiResultWidget.cpp
        log/DistanceResultWidget.cpp
        log/ConstraintWidget.cpp
        log/ForceTorqueWidget.cpp
        log/ContactSetWidget.cpp
        log/ContactVelocitiesWidget.cpp
        log/EquationSystemWidget.cpp
        log/LogValuesWidget.cpp
        log/LogMessageWidget.cpp
        log/SimulatorLogWidget.cpp
        log/SimulatorStatisticsWidget.cpp
        log/MathematicaPlotWidget.cpp
    )
    set(ODE_LIB)
    if(RWSIM_HAVE_ODE)
        set(FILES_UI ${FILES_UI} ODESimCfgForm.ui)
        set(SrcFiles_HPP ${SrcFiles_HPP} ODESimCfgDialog.hpp)
        set(SrcFiles ${SrcFiles} ODESimCfgDialog.cpp)
        set(ODE_LIB sdurwsim_ode)
    endif()

    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H ${FILES_UI})
        qt_wrap_cpp(MocSrcFiles ${SrcFiles_HPP})
        qt_add_resources(RccSrcFiles resources.qrc)
    else()
        qt5_wrap_ui(UIS_OUT_H ${FILES_UI})
        qt5_wrap_cpp(MocSrcFiles ${SrcFiles_HPP})
        qt5_add_resources(RccSrcFiles resources.qrc)
    endif()

    rw_add_library(${SUBSYS_NAME} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
    rw_add_includes(${SUBSYS_NAME} "rwsimlibs/gui" ${SrcFiles_HPP})

    if(DEFINED Qt6Core_VERSION)
        target_link_libraries(
            sdurwsim_gui
            PUBLIC
                sdurwsim
                RWS::sdurws
                Qt6::OpenGLWidgets
                Qt6::OpenGL
                Qt6::Widgets
                Qt6::Gui
                Qt6::Core
            PRIVATE ${ODE_LIB}
        )
    else()
        target_link_libraries(
            sdurwsim_gui
            PUBLIC
                sdurwsim
                RWS::sdurws
                Qt5::OpenGL
                Qt5::Widgets
                Qt5::Gui
                Qt5::Core
            PRIVATE ${ODE_LIB}
        )
    endif()

    if(TARGET RW::sdurw_mathematica)
        target_link_libraries(sdurwsim_gui PRIVATE RW::sdurw_mathematica)
    endif()
    # to be able to include the generated ui header files
    target_include_directories(
        sdurwsim_gui
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
        INTERFACE $<BUILD_INTERFACE:${RWSIM_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
endif()
