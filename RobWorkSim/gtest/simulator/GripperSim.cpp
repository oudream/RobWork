/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "../TestEnvironment.hpp"

#include <rw/geometry/Box.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/models/WorkCell.hpp>
#include <rwsim/control/PDController.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

#include <gtest/gtest.h>

using namespace rw::core;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::models;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::loaders;
using namespace rwsim::control;

TEST(LoadAndTest, GripperSim) {
    std::string dwc_file     = TestEnvironment::testfilesDir() + "/devices/PG70/test_scene.dwc.xml";
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(dwc_file);
    State initState          = dwc->getWorkcell()->getDefaultState();
    WorkCell::Ptr wc         = dwc->getWorkcell();

    RigidDevice::Ptr device = dwc->findDevice<RigidDevice>("PG70");
    EXPECT_FALSE(device.isNull());

    PDController::Ptr ctrl = dwc->findController<PDController>("GraspController");
    EXPECT_FALSE(ctrl.isNull());

    PhysicsEngine::Ptr engine       = PhysicsEngine::Factory::makePhysicsEngine("ODE", dwc);
    DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc, engine));

    simulator->init(initState);
    EXPECT_NEAR(device->getJointDevice()->getQ(simulator->getState()).norm2(),0.0,1e-5);

    double t = 0;
    while(t < 1.0) {
        ctrl->setTargetPos(rw::math::Q(0.01));
        simulator->step(0.01);
        t = simulator->getTime();
    }
    EXPECT_NEAR(device->getJointDevice()->getQ(simulator->getState()).norm2(),0.01,1e-5);
}