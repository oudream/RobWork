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
#include <rw/geometry.hpp>
#include <rw/graphics.hpp>
//#include <rw/graspplanning.hpp>
#include <rw/invkin.hpp>
#include <rw/kinematics.hpp>
#include <rw/models.hpp>
#include <rw/pathplanning.hpp>
//#include <rw/plugin.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <rw/proximity.hpp>
#include <rw/sensor.hpp>
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
#include <rwlibs/simulation/FrameGrabber.hpp>
#include <rwlibs/simulation/FrameGrabber25D.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>
#include <rwlibs/simulation/SimulatedScanner2D.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
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

    /** @addtogroup swig */
    /*@{*/

    ///@}
    /**
     * @name geometry
     * Wrapped classes in geometry.
     */
    ///@{
    //! @copydoc rw::geometry::GeometryData
    typedef rw::geometry::GeometryData GeometryData;
    //! @copydoc rw::geometry::GeometryData::GeometryType
    typedef rw::geometry::GeometryData::GeometryType GeometryType;
    //! @copydoc rw::geometry::Primitive
    typedef rw::geometry::Primitive Primitive;
    //! @copydoc rw::geometry::Box
    typedef rw::geometry::Box Box;
    //! @copydoc rw::geometry::Cone
    typedef rw::geometry::Cone Cone;
    //! @copydoc rw::geometry::Sphere
    typedef rw::geometry::Sphere Sphere;
    //! @copydoc rw::geometry::Plane
    typedef rw::geometry::Plane Plane;
    //! @copydoc rw::geometry::Cylinder
    typedef rw::geometry::Cylinder Cylinder;
    //! @copydoc rw::geometry::Triangle
    typedef rw::geometry::Triangle< double > Triangle;
    //! @copydoc rw::geometry::Triangle
    typedef rw::geometry::Triangle< float > Trianglef;
    //! @copydoc rw::geometry::TriangleN1
    typedef rw::geometry::TriangleN1< double > TriangleN1;
    //! @copydoc rw::geometry::TriangleN1
    typedef rw::geometry::TriangleN1< float > TriangleN1f;
    //! @copydoc rw::geometry::TriMesh
    typedef rw::geometry::TriMesh TriMesh;
    //! @copydoc rw::geometry::PlainTriMesh
    typedef rw::geometry::PlainTriMesh< Triangle > PlainTriMesh;
    //! @copydoc rw::geometry::PlainTriMesh
    typedef rw::geometry::PlainTriMesh< Trianglef > PlainTriMeshf;
    //! @copydoc rw::geometry::PlainTriMesh
    typedef rw::geometry::PlainTriMesh< TriangleN1 > PlainTriMeshN1;
    //! @copydoc rw::geometry::PlainTriMesh
    typedef rw::geometry::PlainTriMesh< TriangleN1f > PlainTriMeshN1f;
    //! @copydoc rw::geometry::ConvexHull3D
    typedef rw::geometry::ConvexHull3D ConvexHull3D;
    //! @copydoc rw::geometry::Geometry
    typedef rw::geometry::Geometry Geometry;
    //! @copydoc rw::geometry::PointCloud
    typedef rw::geometry::PointCloud PointCloud;
    ///@}

    /**
     * @name graphics
     * Wrapped classes in graphics.
     */
    ///@{
    //! @copydoc rw::graphics::Model3D
    typedef rw::graphics::Model3D Model3D;
    //! @copydoc rw::graphics::Model3D::Material
    typedef rw::graphics::Model3D::Material Model3DMaterial;
    //! @copydoc rw::graphics::Render
    typedef rw::graphics::Render Render;
    //! @copydoc rw::graphics::WorkCellScene
    typedef rw::graphics::WorkCellScene WorkCellScene;
    //! @copydoc rw::graphics::SceneViewer
    typedef rw::graphics::SceneViewer SceneViewer;
    //! @copydoc rw::graphics::SceneNode
    typedef rw::graphics::SceneNode SceneNode;
    //! @copydoc rw::graphics::DrawableNode
    typedef rw::graphics::DrawableNode DrawableNode;
    //! @copydoc rw::graphics::DrawableNode::RenderInfo
    typedef rw::graphics::DrawableNode::RenderInfo RenderInfo;
    //! @copydoc rw::graphics::SceneDescriptor
    typedef rw::graphics::SceneDescriptor SceneDescriptor;
    //! @copydoc rw::graphics::SceneCamera
    typedef rw::graphics::SceneCamera SceneCamera;
    ///@}

    // graspplanning

    /**
     * @name invkin
     * Wrapped classes in invkin.
     */
    ///@{
    //! @copydoc rw::invkin::InvKinSolver
    typedef rw::invkin::InvKinSolver InvKinSolver;
    //! @copydoc rw::invkin::IterativeIK
    typedef rw::invkin::IterativeIK IterativeIK;
    //! @copydoc rw::invkin::JacobianIKSolver
    typedef rw::invkin::JacobianIKSolver JacobianIKSolver;
    //! @copydoc rw::invkin::IterativeMultiIK
    typedef rw::invkin::IterativeMultiIK IterativeMultiIK;
    //! @copydoc rw::invkin::JacobianIKSolverM
    typedef rw::invkin::JacobianIKSolverM JacobianIKSolverM;
    //! @copydoc rw::invkin::IKMetaSolver
    typedef rw::invkin::IKMetaSolver IKMetaSolver;
    //! @copydoc rw::invkin::ClosedFormIK
    typedef rw::invkin::ClosedFormIK ClosedFormIK;
    //! @copydoc rw::invkin::ClosedFormIKSolverKukaIIWA
    typedef rw::invkin::ClosedFormIKSolverKukaIIWA ClosedFormIKSolverKukaIIWA;
    //! @copydoc rw::invkin::ClosedFormIKSolverUR
    typedef rw::invkin::ClosedFormIKSolverUR ClosedFormIKSolverUR;
    //! @copydoc rw::invkin::PieperSolver
    typedef rw::invkin::PieperSolver PieperSolver;
    ///@}

    /**
     * @name kinematics
     * Wrapped classes in kinematics.
     */
    ///@{
    //! @copydoc rw::kinematics::StateData
    typedef rw::kinematics::StateData StateData;
    //! @copydoc rw::kinematics::FKRange
    typedef rw::kinematics::FKRange FKRange;
    //! @copydoc rw::kinematics::FKTable
    typedef rw::kinematics::FKTable FKTable;
    //! @copydoc rw::kinematics::Frame
    typedef rw::kinematics::Frame Frame;
    //! @copydoc rw::kinematics::FrameType
    typedef rw::kinematics::FrameType FrameType;
    //! @copydoc rw::kinematics::MovableFrame
    typedef rw::kinematics::MovableFrame MovableFrame;
    //! @copydoc rw::kinematics::FixedFrame
    typedef rw::kinematics::FixedFrame FixedFrame;
    //! @copydoc rw::kinematics::State
    typedef rw::kinematics::State State;
    //! @copydoc rw::kinematics::StateSetup
    typedef rw::kinematics::StateSetup StateSetup;
    //! @copydoc rw::kinematics::StateCache
    typedef rw::kinematics::StateCache StateCache;
    //! @copydoc rw::kinematics::Stateless
    typedef rw::kinematics::Stateless Stateless;
    //! @copydoc rw::kinematics::StateStructure
    typedef rw::kinematics::StateStructure StateStructure;
    //! @copydoc rw::kinematics::Kinematics
    typedef rw::kinematics::Kinematics Kinematics;
    //! @copydoc rw::kinematics::QState
    typedef rw::kinematics::QState QState;
    //! @copydoc rw::kinematics::TreeState
    typedef rw::kinematics::TreeState TreeState;
    ///@}

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
     * @name models
     * Wrapped classes in models.
     */
    ///@{
    //! @copydoc rw::models::JacobianCalculator
    typedef rw::models::JacobianCalculator JacobianCalculator;
    //! @copydoc rw::models::JointDeviceJacobianCalculator
    typedef rw::models::JointDeviceJacobianCalculator JointDeviceJacobianCalculator;
    //! @copydoc rw::models::DeviceJacobianCalculator
    typedef rw::models::DeviceJacobianCalculator DeviceJacobianCalculator;
    //! @copydoc rw::models::JacobianUtil
    typedef rw::models::JacobianUtil JacobianUtil;

    //! @copydoc rw::models::WorkCell
    typedef rw::models::WorkCell WorkCell;
    typedef rw::models::WorkCell::WorkCellChangedListener WorkCellChangedListener;

    //! @copydoc rw::models::Joint
    typedef rw::models::Joint Joint;
    //! @copydoc rw::models::RevoluteJoint
    typedef rw::models::RevoluteJoint RevoluteJoint;
    //! @copydoc rw::models::PrismaticJoint
    typedef rw::models::PrismaticJoint PrismaticJoint;
    //! @copydoc rw::models::PrismaticSphericalJoint
    typedef rw::models::PrismaticSphericalJoint PrismaticSphericalJoint;
    //! @copydoc rw::models::PrismaticUniversalJoint
    typedef rw::models::PrismaticUniversalJoint PrismaticUniversalJoint;
    //! @copydoc rw::models::SphericalJoint
    typedef rw::models::SphericalJoint SphericalJoint;
    //! @copydoc rw::models::DependentJoint
    typedef rw::models::DependentJoint DependentJoint;
    //! @copydoc rw::models::DependentPrismaticJoint
    typedef rw::models::DependentPrismaticJoint DependentPrismaticJoint;
    //! @copydoc rw::models::DependentRevoluteJoint
    typedef rw::models::DependentRevoluteJoint DependentRevoluteJoint;
    //! @copydoc rw::models::VirtualJoint
    typedef rw::models::VirtualJoint VirtualJoint;
    //! @copydoc rw::models::UniversalJoint
    typedef rw::models::UniversalJoint UniversalJoint;

    //! @copydoc rw::models::Object
    typedef rw::models::Object Object;
    //! @copydoc rw::models::DeformableObject
    typedef rw::models::DeformableObject DeformableObject;
    //! @copydoc rw::models::RigidObject
    typedef rw::models::RigidObject RigidObject;

    //! @copydoc rw::models::Device
    typedef rw::models::Device Device;
    //! @copydoc rw::models::JointDevice
    typedef rw::models::JointDevice JointDevice;
    //! @copydoc rw::models::MobileDevice
    typedef rw::models::MobileDevice MobileDevice;
    //! @copydoc rw::models::SerialDevice
    typedef rw::models::SerialDevice SerialDevice;
    //! @copydoc rw::models::TreeDevice
    typedef rw::models::TreeDevice TreeDevice;
    //! @copydoc rw::models::CompositeDevice
    typedef rw::models::CompositeDevice CompositeDevice;
    //! @copydoc rw::models::CompositeJointDevice
    typedef rw::models::CompositeJointDevice CompositeJointDevice;
    //! @copydoc rw::models::ParallelDevice
    typedef rw::models::ParallelDevice ParallelDevice;
    //! @copydoc rw::models::SE3Device
    typedef rw::models::SE3Device SE3Device;

    //! @copydoc rw::models::ParallelLeg
    typedef rw::models::ParallelLeg ParallelLeg;
    //! @copydoc rw::models::DHParameterSet
    typedef rw::models::DHParameterSet DHParameterSet;
    //! @copydoc rw::models::RigidBodyInfo
    typedef rw::models::RigidBodyInfo RigidBodyInfo;

    //! @copydoc rw::models::ControllerModel
    typedef rw::models::ControllerModel ControllerModel;
    //! @copydoc rw::models::Models
    typedef rw::models::Models Models;
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
     * @name proximity
     * Wrapped classes in proximity.
     */
    ///@{
    //! @copydoc rw::proximity::CollisionDetector
    typedef rw::proximity::CollisionDetector CollisionDetector;
    typedef rw::proximity::CollisionDetector::QueryResult CollisionDetectorQueryResult;
    //! @copydoc rw::proximity::CollisionStrategy
    typedef rw::proximity::CollisionSetup CollisionSetup;
    //! @copydoc rw::proximity::CollisionStrategy
    typedef rw::proximity::CollisionStrategy CollisionStrategy;
    typedef rw::proximity::CollisionStrategy::Result CollisionStrategyResult;
    typedef rw::proximity::CollisionStrategy::Result::CollisionPair CollisionStrategyCollisionPair;
    //! @copydoc rw::proximity::CollisionStrategy
    typedef rw::proximity::CollisionToleranceStrategy CollisionToleranceStrategy;
    //! @copydoc rw::proximity::DistanceCalculator
    typedef rw::proximity::DistanceCalculator DistanceCalculator;
    //! @copydoc rw::proximity::DistanceStrategy
    typedef rw::proximity::DistanceStrategy DistanceStrategy;
    typedef rw::proximity::DistanceStrategy::Result DistanceStrategyResult;
    //! @copydoc rw::proximity::DistanceMultiStrategy
    typedef rw::proximity::DistanceMultiStrategy DistanceMultiStrategy;
    typedef rw::proximity::DistanceMultiStrategy::Result DistanceMultiStrategyResult;
    //! @copydoc rw::proximity::ProximityCache
    typedef rw::proximity::ProximityCache ProximityCache;
    //! @copydoc rw::proximity::ProximityModel
    typedef rw::proximity::ProximityModel ProximityModel;
    //! @copydoc rw::proximity::ProximityStrategy
    typedef rw::proximity::ProximityData ProximityData;
    //! @copydoc rw::proximity::ProximityStrategy
    typedef rw::proximity::ProximityStrategy ProximityStrategy;
    //! @copydoc rw::proximity::ProximityStrategyData
    typedef rw::proximity::ProximityStrategyData ProximityStrategyData;
    //! @copydoc rw::proximity::ProximityFilterStrategy
    typedef rw::proximity::ProximityFilterStrategy ProximityFilterStrategy;
    //! @copydoc rw::proximity::ProximityFilter
    typedef rw::proximity::ProximityFilter ProximityFilter;
    //! @copydoc rw::proximity::ProximitySetupRule
    typedef rw::proximity::ProximitySetupRule ProximitySetupRule;
    //! @copydoc rw::proximity::ProximitySetup
    typedef rw::proximity::ProximitySetup ProximitySetup;

    ///@}

    /**
     * @name sensor
     * Wrapped classes in sensor.
     */
    ///@{
    //! @copydoc rw::sensor::Camera
    typedef rw::sensor::Camera Camera;
    //! @copydoc rw::sensor::CameraModel
    typedef rw::sensor::CameraModel CameraModel;
    //! @copydoc rw::sensor::CameraListener
    typedef rw::sensor::CameraListener CameraListener;
    //! @copydoc rw::sensor::CameraFirewire
    typedef rw::sensor::CameraFirewire CameraFirewire;
    //! @copydoc rw::sensor::StereoCameraModel
    typedef rw::sensor::StereoCameraModel StereoCameraModel;
    //! @copydoc rw::sensor::Contact2D
    typedef rw::sensor::Contact2D Contact2D;
    //! @copydoc rw::sensor::Contact3D
    typedef rw::sensor::Contact3D Contact3D;
    //! @copydoc rw::sensor::FTSensor
    typedef rw::sensor::FTSensor FTSensor;
    //! @copydoc rw::sensor::FTSensorModel
    typedef rw::sensor::FTSensorModel FTSensorModel;
    //! @copydoc rw::sensor::Image
    typedef rw::sensor::Image Image;
    //! @copydoc rw::sensor::ImageUtil
    typedef rw::sensor::ImageUtil ImageUtil;
    //! @copydoc rw::sensor::Pixel4f
    typedef rw::sensor::Pixel4f Pixel4f;
    //! @copydoc rw::sensor::Image::Pixel4i
    typedef rw::sensor::Image::Pixel4i Pixel4i;
    //! @copydoc rw::sensor::Sensor
    typedef rw::sensor::Sensor Sensor;
    //! @copydoc rw::sensor::SensorModel
    typedef rw::sensor::SensorModel SensorModel;
    //! @copydoc rw::sensor::SensorData
    typedef rw::sensor::SensorData SensorData;
    //! @copydoc rw::sensor::Scanner
    typedef rw::sensor::Scanner Scanner;
    //! @copydoc rw::sensor::Scanner1D
    typedef rw::sensor::Scanner1D Scanner1D;
    //! @copydoc rw::sensor::Scanner2D
    typedef rw::sensor::Scanner2D Scanner2D;
    //! @copydoc rw::sensor::Scanner2DModel
    typedef rw::sensor::Scanner2DModel Scanner2DModel;
    //! @copydoc rw::sensor::Scanner25D
    typedef rw::sensor::Scanner25D Scanner25D;
    //! @copydoc rw::sensor::Scanner25DModel
    typedef rw::sensor::Scanner25DModel Scanner25DModel;
    //! @copydoc rw::sensor::RGBDCameraModel
    typedef rw::sensor::RGBDCameraModel RGBDCameraModel;

    //! @copydoc rw::sensor::TactileArray
    typedef rw::sensor::TactileArray TactileArray;
    //! @copydoc rw::sensor::TactileArrayModel
    typedef rw::sensor::TactileArrayModel TactileArrayModel;
    //! @copydoc rw::sensor::TactileArrayUtil
    typedef rw::sensor::TactileArrayUtil TactileArrayUtil;

    ///@}

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
   /* //! @copydoc rw::trajectory::Trajectory
    typedef rw::trajectory::Trajectory< State > TrajectoryState;
    //! @copydoc rw::trajectory::Trajectory
    typedef rw::trajectory::Trajectory< Q > TrajectoryQ;
    //! @copydoc rw::trajectory::Trajectory
    typedef rw::trajectory::Trajectory< double > TrajectoryR1;
    //! @copydoc rw::trajectory::Trajectory
    typedef rw::trajectory::Trajectory< Vector2d > TrajectoryR2;
    //! @copydoc rw::trajectory::Trajectory
    typedef rw::trajectory::Trajectory< Vector3d > TrajectoryR3;
    //! @copydoc rw::trajectory::Trajectory
    typedef rw::trajectory::Trajectory< Rotation3Dd > TrajectorySO3;
    //! @copydoc rw::trajectory::Trajectory
    typedef rw::trajectory::Trajectory< Transform3Dd > TrajectorySE3;*/

   /* //! @copydoc rw::trajectory::LinearInterpolator
    typedef rw::trajectory::LinearInterpolator< double > LinearInterpolator;
    //! @copydoc rw::trajectory::LinearInterpolator
    typedef rw::trajectory::LinearInterpolator< rw::math::Q > LinearInterpolatorQ;
    //! @copydoc rw::trajectory::LinearInterpolator
    typedef rw::trajectory::LinearInterpolator< Vector2d > LinearInterpolatorR2;
    //! @copydoc rw::trajectory::LinearInterpolator
    typedef rw::trajectory::LinearInterpolator< rw::math::Rotation3D< double > >
        LinearInterpolatorR3;
    //! @copydoc rw::trajectory::LinearInterpolator
    typedef rw::trajectory::LinearInterpolator< rw::math::Rotation3D< double > >
        LinearInterpolatorSO3;
    //! @copydoc rw::trajectory::LinearInterpolator
    typedef rw::trajectory::LinearInterpolator< rw::math::Transform3D< double > >
        LinearInterpolatorSE3;*/

  /*  //! @copydoc rw::trajectory::RampInterpolator
    typedef rw::trajectory::RampInterpolator< double > RampInterpolator;
    //! @copydoc rw::trajectory::RampInterpolator
    typedef rw::trajectory::RampInterpolator< rw::math::Q > RampInterpolatorQ;
    //! @copydoc rw::trajectory::RampInterpolator
    typedef rw::trajectory::RampInterpolator< Vector2d > RampInterpolatorR2;
    //! @copydoc rw::trajectory::RampInterpolator
    typedef rw::trajectory::RampInterpolator< Vector3d > RampInterpolatorR3;
    //! @copydoc rw::trajectory::RampInterpolator
    typedef rw::trajectory::RampInterpolator< rw::math::Rotation3D< double > > RampInterpolatorSO3;
    //! @copydoc rw::trajectory::RampInterpolator
    typedef rw::trajectory::RampInterpolator< rw::math::Transform3D< double > > RampInterpolatorSE3;*/

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

    /**
     * @name simulation
     * Wrapped classes in simulation.
     */
    ///@{
    //! @copydoc rwlibs::simulation::FrameGrabber
    typedef rwlibs::simulation::FrameGrabber FrameGrabber;
    //! @copydoc rwlibs::simulation::FrameGrabber25D
    typedef rwlibs::simulation::FrameGrabber25D FrameGrabber25D;
    //! @copydoc rwlibs::simulation::GLFrameGrabber
    typedef rwlibs::simulation::GLFrameGrabber GLFrameGrabber;
    //! @copydoc rwlibs::simulation::GLFrameGrabber25D
    typedef rwlibs::simulation::GLFrameGrabber25D GLFrameGrabber25D;
    //! @copydoc rwlibs::simulation::SimulatedCamera
    typedef rwlibs::simulation::SimulatedCamera SimulatedCamera;
    //! @copydoc rwlibs::simulation::SimulatedController
    typedef rwlibs::simulation::SimulatedController SimulatedController;
    //! @copydoc rwlibs::simulation::SimulatedSensor
    typedef rwlibs::simulation::SimulatedSensor SimulatedSensor;
    //! @copydoc rwlibs::simulation::SimulatedScanner2D
    typedef rwlibs::simulation::SimulatedScanner2D SimulatedScanner2D;
    //! @copydoc rwlibs::simulation::SimulatedScanner25D
    typedef rwlibs::simulation::SimulatedScanner25D SimulatedScanner25D;
    //! @copydoc rwlibs::simulation::Simulator
    typedef rwlibs::simulation::Simulator Simulator;
    //! @copydoc rwlibs::simulation::Simulator::UpdateInfo
    typedef rwlibs::simulation::Simulator::UpdateInfo UpdateInfo;
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
