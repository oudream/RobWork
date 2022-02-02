/********************************************************************************
 * Copyright 2011 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_RWSIM_HPP_
#define RWSIM_RWSIM_HPP_

#include <rwsim/contacts/BallBallStrategy.hpp>
#include <rwsim/contacts/Contact.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactDetectorData.hpp>
#include <rwsim/contacts/ContactModel.hpp>
#include <rwsim/contacts/ContactStrategy.hpp>
#include <rwsim/contacts/ContactStrategyData.hpp>
#include <rwsim/control/PDController.hpp>
#include <rwsim/control/SerialDeviceController.hpp>
#include <rwsim/control/SuctionCupController.hpp>
#include <rwsim/control/SyncPDController.hpp>
#include <rwsim/control/VelRampController.hpp>
#include <rwsim/drawable/RenderCircles.hpp>
#include <rwsim/drawable/RenderContacts.hpp>
#include <rwsim/drawable/RenderGhost.hpp>
#include <rwsim/drawable/RenderPlanes.hpp>
#include <rwsim/drawable/RenderPoints.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
#include <rwsim/dynamics/ContactDataMap.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/MaterialDataMap.hpp>
#include <rwsim/dynamics/OBRManifold.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/dynamics/SuctionCup.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/loaders/ScapePoseFormat.hpp>
#include <rwsim/rwphysics/BodyController.hpp>
#include <rwsim/rwphysics/BodyIntegrator.hpp>
#include <rwsim/rwphysics/CNodePairMap.hpp>
#include <rwsim/rwphysics/CNodePool.hpp>
#include <rwsim/rwphysics/ConstantForceManipulator.hpp>
#include <rwsim/rwphysics/ConstraintEdge.hpp>
#include <rwsim/rwphysics/ConstraintNode.hpp>
#include <rwsim/rwphysics/ConstraintSolver.hpp>
#include <rwsim/rwphysics/ContactGraph.hpp>
#include <rwsim/rwphysics/ContactModel.hpp>
#include <rwsim/rwphysics/ContactModelFactory.hpp>
#include <rwsim/rwphysics/EulerIntegrator.hpp>
#include <rwsim/rwphysics/RWBody.hpp>
#include <rwsim/rwphysics/RWBodyPool.hpp>
#include <rwsim/rwphysics/RWDebugRender.hpp>
#include <rwsim/rwphysics/RWSimulator.hpp>
#include <rwsim/rwphysics/SequintialImpulseSolver.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>
#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/PhysicsEngineFactory.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsim/util/CircleModel.hpp>
#include <rwsim/util/CollisionFreeSampler.hpp>
#include <rwsim/util/DistModel.hpp>
#include <rwsim/util/FiniteStateSampler.hpp>
#include <rwsim/util/GraspPolicy.hpp>
#include <rwsim/util/GraspPolicyFactory.hpp>
#include <rwsim/util/GraspStrategy.hpp>
#include <rwsim/util/GraspStrategyFactory.hpp>
#include <rwsim/util/HughLineExtractor.hpp>
#include <rwsim/util/MovingAverage.hpp>
#include <rwsim/util/PlanarSupportPoseGenerator.hpp>
#include <rwsim/util/PlaneModel.hpp>
#include <rwsim/util/PointRANSACFitting.hpp>
#include <rwsim/util/PreshapeSampler.hpp>
#include <rwsim/util/RestingPoseGenerator.hpp>
#include <rwsim/util/SpherePoseSampler.hpp>
#include <rwsim/util/StateSampler.hpp>
#include <rwsim/util/SupportPose.hpp>
#include <rwsim/util/TargetConfigGraspPolicy.hpp>
//#include <rwsim/rwphysics/GuendelContactModel.hpp>
//#include <rwsim/#rwphysics/ConstraintSolver.hpp>
//#include <rwsim/#util/LineFit.hpp>
//#include <rwsim/#util/LinePolar.hpp>

//#include <rwsim/#dynamics/Contact.hpp>

#define USE_ROBWORKSIM_NAMESPACE              \
    namespace rwsim { namespace contacts {    \
    }}                                        \
    namespace rwsim { namespace control {     \
    }}                                        \
    namespace rwsim { namespace drawable {    \
    }}                                        \
    namespace rwsim { namespace dynamics {    \
    }}                                        \
    namespace rwsim { namespace loaders {     \
    }}                                        \
    namespace rwsim { namespace rwphysics {   \
    }}                                        \
    namespace rwsim { namespace sensor {      \
    }}                                        \
    namespace rwsim { namespace simulator {   \
    }}                                        \
    namespace rwsim { namespace util {        \
    }}                                        \
    namespace rwsimlibs { namespace gui {     \
    }}                                        \
    namespace rwsimlibs { namespace ode {     \
    }}                                        \
    namespace rwsimlibs { namespace bullet {  \
    }}                                        \
    namespace rwsimlibs { namespace lua {     \
    }}                                        \
    namespace rwsimlibs { namespace plugins { \
    }}                                        \
    namespace robworksim {                    \
    using namespace rwsim;                    \
    using namespace rwsim::contacts;          \
    using namespace rwsim::control;           \
    using namespace rwsim::drawable;          \
    using namespace rwsim::dynamics;          \
    using namespace rwsim::loaders;           \
    using namespace rwsim::rwphysics;         \
    using namespace rwsim::sensor;            \
    using namespace rwsim::simulator;         \
    using namespace rwsim::util;              \
    using namespace rwsimlibs::gui;           \
    using namespace rwsimlibs::ode;           \
    using namespace rwsimlibs::bullet;        \
    using namespace rwsimlibs::lua;           \
    using namespace rwsimlibs::plugins;       \
    }

#define RWSIM_USE_RWP_NAMESPACE \
    USE_ROBWORKSIM_NAMESPACE    \
    namespace rwp {             \
    using namespace robworksim; \
    }

#endif /* RWSIM_HPP_ */
