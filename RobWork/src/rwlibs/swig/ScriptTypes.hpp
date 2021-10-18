/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWLIBS_SWIG_SCRIPTTYPES_HPP_
#define RWLIBS_SWIG_SCRIPTTYPES_HPP_

#include <RobWorkConfig.hpp>
#include <rw/pathplanning.hpp>
//#include <rw/plugin.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <rw/proximity.hpp>
#include <rw/trajectory.hpp>
#ifdef RW_HAVE_XERCES
#include <rw/loaders/xml/XMLTrajectoryLoader.hpp>
#include <rw/loaders/xml/XMLTrajectorySaver.hpp>
#endif

#include <rwlibs/assembly/AssemblyControlResponse.hpp>
#include <rwlibs/assembly/AssemblyControlStrategy.hpp>
#include <rwlibs/assembly/AssemblyParameterization.hpp>
#include <rwlibs/assembly/AssemblyRegistry.hpp>
#include <rwlibs/assembly/AssemblyResult.hpp>
#include <rwlibs/assembly/AssemblyState.hpp>
#include <rwlibs/assembly/AssemblyTask.hpp>
#include <rwlibs/control/Controller.hpp>
#include <rwlibs/control/JointController.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/pathplanners/arw/ARWExpand.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>
#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLExpand.hpp>
#include <rwlibs/pathplanners/sbl/SBLOptions.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLSetup.hpp>
#include <rwlibs/pathplanners/z3/Z3Planner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/task/GraspResult.hpp>
#include <rwlibs/task/GraspSubTask.hpp>
#include <rwlibs/task/GraspTarget.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwlibs/task/Task.hpp>

#include <iostream>
#include <sstream>

namespace rwlibs {

/**
 * @brief Define helper functions and all the classes that are being wrapped by SWIG.
 * The wrapped classes are defined as typedefs of other classes in RobWork.
 */
namespace swig {

    /**
     * @name loaders
     * Wrapped classes in loaders.
     */
    ///@{
    //! @copydoc rw::loaders::ImageLoader
    typedef rw::loaders::ImageLoader ImageLoader;
    //! @copydoc rw::loaders::ImageLoader::Factory
    typedef rw::loaders::ImageLoader::Factory ImageLoaderFactory;
    //! @copydoc rw::loaders::WorkCellLoader
    typedef rw::loaders::WorkCellLoader WorkCellLoader;
    //! @copydoc rw::loaders::WorkCellLoader::Factory
    typedef rw::loaders::WorkCellLoader::Factory WorkCellLoaderFactory;
#ifdef RW_HAVE_XERCES
    //! @copydoc rw::loaders::XMLTrajectoryLoader
    typedef rw::loaders::XMLTrajectoryLoader XMLTrajectoryLoader;
    //! @copydoc rw::loaders::XMLTrajectorySaver
    typedef rw::loaders::XMLTrajectorySaver XMLTrajectorySaver;
#endif
    //! @copydoc rw::loaders::STLFile
    typedef rw::loaders::STLFile STLFile;
    ///@}

    /**
     * @name pathplanning
     * Wrapped classes in pathplanning.
     */
    ///@{
    //! @copydoc rw::pathplanning::QConstraint
    typedef rw::pathplanning::QConstraint QConstraint;
    //! @copydoc rw::pathplanning::QEdgeConstraint
    typedef rw::pathplanning::QEdgeConstraint QEdgeConstraint;
    //! @copydoc rw::pathplanning::QEdgeConstraintIncremental
    typedef rw::pathplanning::QEdgeConstraintIncremental QEdgeConstraintIncremental;
    //! @copydoc rw::pathplanning::QIKSampler
    typedef rw::pathplanning::QIKSampler QIKSampler;
    //! @copydoc rw::pathplanning::QNormalizer
    typedef rw::pathplanning::QNormalizer QNormalizer;
    //! @copydoc rw::pathplanning::QSampler
    typedef rw::pathplanning::QSampler QSampler;
    //! @copydoc rw::pathplanning::QToQPlanner
    typedef rw::pathplanning::QToQPlanner QToQPlanner;
    //! @copydoc rw::pathplanning::QToQSamplerPlanner
    typedef rw::pathplanning::QToQSamplerPlanner QToQSamplerPlanner;
    //! @copydoc rw::pathplanning::QToTPlanner
    typedef rw::pathplanning::QToTPlanner QToTPlanner;

    //! @copydoc rw::pathplanning::StopCriteria
    typedef rw::pathplanning::StopCriteria StopCriteria;
    //! @copydoc rw::pathplanning::PlannerConstraint
    typedef rw::pathplanning::PlannerConstraint PlannerConstraint;
    //! @copydoc rw::pathplanning::StateConstraint
    typedef rw::pathplanning::StateConstraint StateConstraint;
    ///@}

    // plugin

    /**
     * @name assembly
     * Wrapped classes in assembly.
     */
    ///@{
    //! @copydoc rwlibs::assembly::AssemblyControlResponse
    typedef rwlibs::assembly::AssemblyControlResponse AssemblyControlResponse;
    //! @copydoc rwlibs::assembly::AssemblyControlStrategy
    typedef rwlibs::assembly::AssemblyControlStrategy AssemblyControlStrategy;
    //! @copydoc rwlibs::assembly::AssemblyParameterization
    typedef rwlibs::assembly::AssemblyParameterization AssemblyParameterization;
    //! @copydoc rwlibs::assembly::AssemblyRegistry
    typedef rwlibs::assembly::AssemblyRegistry AssemblyRegistry;
    //! @copydoc rwlibs::assembly::AssemblyResult
    typedef rwlibs::assembly::AssemblyResult AssemblyResult;
    //! @copydoc rwlibs::assembly::AssemblyState
    typedef rwlibs::assembly::AssemblyState AssemblyState;
    //! @copydoc rwlibs::assembly::AssemblyTask
    typedef rwlibs::assembly::AssemblyTask AssemblyTask;
    ///@}

    /**
     * @name trajectory
     * Wrapped classes in trajectory.
     */
    ///@{
    //! @copydoc rw::trajectory::TimedState
    typedef rw::trajectory::TimedState TimedState;
    //! @copydoc rw::trajectory::TimedStatePath
    typedef rw::trajectory::TimedStatePath PathTimedState;
    //! @copydoc rw::trajectory::Timed
    typedef rw::trajectory::Timed< AssemblyState > TimedAssemblyState;
    ///@}

    // rwlibs algorithms

    // rwlibs calibration

    /**
     * @name control
     * Wrapped classes in control.
     */
    ///@{
    //! @copydoc rwlibs::control::Controller
    typedef rwlibs::control::Controller Controller;
    //! @copydoc rwlibs::control::JointController
    typedef rwlibs::control::JointController JointController;
    ///@}

    /**
     * @name opengl
     * Wrapped classes in opengl.
     */

    //! @copydoc rwlibs::opengl::RenderImage
    typedef rwlibs::opengl::RenderImage RenderImage;

    // rwlibs os

    /**
     * @name pathoptimization
     * Wrapped classes in pathoptimization.
     */
    ///@{
    //! @copydoc rwlibs::pathoptimization::PathLengthOptimizer
    typedef rwlibs::pathoptimization::PathLengthOptimizer PathLengthOptimizer;
    //! @copydoc rwlibs::pathoptimization::ClearanceOptimizer
    typedef rwlibs::pathoptimization::ClearanceOptimizer ClearanceOptimizer;
    ///@}

    /**
     * @name pathplanners
     * Wrapped classes in pathplanners.
     */
    ///@{
    //! @copydoc rwlibs::pathplanners::ARWExpand
    typedef rwlibs::pathplanners::ARWExpand ARWExpand;
    //! @copydoc rwlibs::pathplanners::ARWPlanner
    typedef rwlibs::pathplanners::ARWPlanner ARWPlanner;
    //! @copydoc rwlibs::pathplanners::PRMPlanner
    typedef rwlibs::pathplanners::PRMPlanner PRMPlanner;
    //! @copydoc rwlibs::pathplanners::RRTPlanner
    typedef rwlibs::pathplanners::RRTPlanner RRTPlanner;
    //! @copydoc rwlibs::pathplanners::SBLExpand
    typedef rwlibs::pathplanners::SBLExpand SBLExpand;
    //! @copydoc rwlibs::pathplanners::SBLPlannerConstraint
    typedef rwlibs::pathplanners::SBLPlannerConstraint SBLPlannerConstraint;
    //! @copydoc rwlibs::pathplanners::SBLOptions
    typedef rwlibs::pathplanners::SBLOptions SBLOptions;
    //! @copydoc rwlibs::pathplanners::SBLPlanner
    typedef rwlibs::pathplanners::SBLPlanner SBLPlanner;
    //! @copydoc rwlibs::pathplanners::SBLSetup
    typedef rwlibs::pathplanners::SBLSetup SBLSetup;
    //! @copydoc rwlibs::pathplanners::Z3Planner
    typedef rwlibs::pathplanners::Z3Planner Z3Planner;
    ///@}

    /**
     * @name proximitystrategies
     * Wrapped classes in proximitystrategies.
     */
    ///@{
    //! @copydoc rwlibs::proximitystrategies::ProximityStrategyFactory
    typedef rwlibs::proximitystrategies::ProximityStrategyFactory ProximityStrategyFactory;
    ///@}

    // rwlibs softbody

    // rwlibs swig

    /**
     * @name task
     * Wrapped classes in task.
     */
    ///@{
    //! @copydoc rwlibs::task::Task
    typedef rwlibs::task::Task< rw::math::Transform3D< double > > TaskSE3;
    //! @copydoc rwlibs::task::GraspTask
    typedef rwlibs::task::GraspTask GraspTask;
    //! @copydoc rwlibs::task::GraspSubTask
    typedef rwlibs::task::GraspSubTask GraspSubTask;
    //! @copydoc rwlibs::task::GraspTarget
    typedef rwlibs::task::GraspTarget GraspTarget;
    //! @copydoc rwlibs::task::GraspResult
    typedef rwlibs::task::GraspResult GraspResult;
    ///@}

    // rwlibs tools

    // helper functions
    /**
     * @brief Write message to log.
     * @param msg [in] message to write.
     */
    void writelog (const std::string& msg);

    /**
     * @brief Set the writer to write log to.
     * @param writer [in] the writer.
     */
    void setlog (::rw::core::LogWriter::Ptr writer);

    /**
     * @brief Math helper function to obtain random rotation.
     * @return a random rotation.
     */
    //Rotation3Dd getRandomRotation3D ();

    /**
     * @brief Math helper function to obtain random transform.
     * @param translationLength [in] (optional) the length to translate - default is one meter.
     * @return a random transform.
     */
    //Transform3Dd getRandomTransform3D (const double translationLength = 1);
    /*@}*/
}    // namespace swig
}    // namespace rwlibs

#endif /* RWLIBS_SWIG_SCRIPTTYPES_HPP_ */
