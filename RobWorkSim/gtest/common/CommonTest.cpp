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

#include "../TestEnvironment.hpp"

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedKinect.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>
#include <rwlibs/simulation/SimulatedScanner2D.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwsim/control/BeamJointController.hpp>
#include <rwsim/control/BodyController.hpp>
#include <rwsim/control/PDController.hpp>
#include <rwsim/control/PoseController.hpp>
#include <rwsim/control/SerialDeviceController.hpp>
#include <rwsim/control/SpringJointController.hpp>
#include <rwsim/control/SyncPDController.hpp>
#include <rwsim/control/VelRampController.hpp>
#include <rwsim/dynamics/BeamBody.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/dynamics/SuctionCup.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/loaders/DynamicWorkCellSaver.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

using namespace rwsim::dynamics;
using namespace rwsim::control;
using namespace rwsim::loaders;
using namespace rwsim::sensor;
using namespace rwlibs::simulation;

class DynamicWorkCellSaverTest : public ::testing::TestWithParam< const char* >
{
  protected:
    DynamicWorkCellSaverTest () {}

    void SetUp ()
    {
        DynamicWorkCell::Ptr dwc1 = DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                                 std::string (GetParam ()));
    }

    DynamicWorkCell::Ptr getDwc () { return dwc; }

    DynamicWorkCell::Ptr dwc;

    
};

TEST (CommonTest, DynamicWorkCellLoaderTest)
{
    // check failed loading of non-existing scene
    EXPECT_THROW (DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                               "/devices/does_not_exist.dwc.xml"),
                  std::exception);

    // check successfull loading of different dynamic workcells
    DynamicWorkCell::Ptr dwc_pg70, dwc_sdh, dwc_ur;
    EXPECT_NO_THROW (dwc_pg70 = DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                             "/devices/PG70/test_scene.dwc.xml"));
    EXPECT_NO_THROW (dwc_sdh = DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                            "/devices/SDH2/test_scene.dwc.xml"));
    EXPECT_NO_THROW (dwc_ur = DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                           "/devices/UR6855A/test_scene.dwc.xml"));

    // check that the scenes contain correct information
    EXPECT_FALSE (dwc_pg70->findController ("GraspController").isNull ());
    EXPECT_FALSE (dwc_pg70->findDevice ("PG70").isNull ());
    EXPECT_FALSE (dwc_pg70->findDevice< RigidDevice > ("PG70").isNull ());
    EXPECT_FALSE (dwc_pg70->findBody ("PG70.Base").isNull ());
    EXPECT_FALSE (dwc_pg70->findBody ("PG70.RightFinger").isNull ());

    // check different constraints
    DynamicWorkCell::Ptr fixed, prismatic, revolute, universal, spherical, piston, prismaticRotoid,
        prismaticUniversal, free, free_spring, revolute_limits;
    EXPECT_NO_THROW (fixed = DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                          "/scene/constraints/fixed.dwc.xml"));
    EXPECT_NO_THROW (prismatic =
                         DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                      "/scene/constraints/prismatic.dwc.xml"));
    EXPECT_NO_THROW (revolute = DynamicWorkCellLoader::load (
                         TestEnvironment::testfilesDir () + "/scene/constraints/revolute.dwc.xml"));
    EXPECT_NO_THROW (universal =
                         DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                      "/scene/constraints/universal.dwc.xml"));
    EXPECT_NO_THROW (spherical =
                         DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                      "/scene/constraints/spherical.dwc.xml"));
    EXPECT_NO_THROW (piston = DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                           "/scene/constraints/piston.dwc.xml"));
    EXPECT_NO_THROW (
        prismaticRotoid = DynamicWorkCellLoader::load (
            TestEnvironment::testfilesDir () + "/scene/constraints/prismaticRotoid.dwc.xml"));
    EXPECT_NO_THROW (
        prismaticUniversal = DynamicWorkCellLoader::load (
            TestEnvironment::testfilesDir () + "/scene/constraints/prismaticUniversal.dwc.xml"));
    EXPECT_NO_THROW (free = DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                         "/scene/constraints/free.dwc.xml"));
    EXPECT_NO_THROW (free_spring =
                         DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                      "/scene/constraints/free_spring.dwc.xml"));
    EXPECT_NO_THROW (
        revolute_limits = DynamicWorkCellLoader::load (
            TestEnvironment::testfilesDir () + "/scene/constraints/revolute_limits.dwc.xml"));

    // check information
    EXPECT_FALSE (fixed->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (prismatic->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (revolute->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (universal->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (spherical->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (piston->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (prismaticRotoid->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (prismaticUniversal->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (free->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (free_spring->findConstraint ("Constraint").isNull ());
    EXPECT_FALSE (revolute_limits->findConstraint ("Constraint").isNull ());

    EXPECT_FALSE (prismatic->findConstraint ("Constraint")->getSpringParams ().enabled);
    EXPECT_TRUE (free_spring->findConstraint ("Constraint")->getSpringParams ().enabled);
    EXPECT_FALSE (revolute->findConstraint ("Constraint")->getLimit (0).lowOn);
    EXPECT_FALSE (revolute->findConstraint ("Constraint")->getLimit (0).highOn);
    EXPECT_TRUE (revolute_limits->findConstraint ("Constraint")->getLimit (0).lowOn);
    EXPECT_TRUE (revolute_limits->findConstraint ("Constraint")->getLimit (0).highOn);
}

INSTANTIATE_TEST_CASE_P (DynamicWorkCell, DynamicWorkCellSaverTest,
                         ::testing::ValuesIn ({"/devices/PG70/test_scene.dwc.xml",
                                               "/devices/SDH2/test_scene.dwc.xml",
                                               "/devices/UR6855A/test_scene.dwc.xml",
                                               "/scene/constraints/fixed.dwc.xml",
                                               "/scene/constraints/prismatic.dwc.xml",
                                               "/scene/constraints/revolute.dwc.xml",
                                               "/scene/constraints/universal.dwc.xml",
                                               "/scene/constraints/spherical.dwc.xml",
                                               "/scene/constraints/piston.dwc.xml",
                                               "/scene/constraints/prismaticRotoid.dwc.xml",
                                               "/scene/constraints/prismaticUniversal.dwc.xml",
                                               "/scene/constraints/free.dwc.xml",
                                               "/scene/constraints/free_spring.dwc.xml",
                                               "/scene/constraints/revolute_limits.dwc.xml",
                                               "/scene/stacking/test_scene.dwc.xml",
                                               "/scene/simple/cup_on_tray_stability.dwc.xml",
                                               "/scene/sensors/single_object_tactile_array.dwc.xml",
                                               "/scene/sensors/single_object.dwc.xml",
                                               "/scene/peginhole/test_scene.dwc.xml"}));

TEST_P (DynamicWorkCellSaverTest, DynamicWorkCellSaverTest)
{
    std::string savedir = TestEnvironment::testfilesDir () + "SavedFiles";
    boost::filesystem::create_directories (savedir + "/dwc1");

    DynamicWorkCell::Ptr dwc1 = DynamicWorkCellLoader::load (TestEnvironment::testfilesDir () +
                                                             "/devices/PG70/test_scene.dwc.xml");
    DynamicWorkCellSaver::save (
        dwc1, dwc1->getWorkcell ()->getDefaultState (), savedir + "/dwc1/dwc1.dwc.xml");

    DynamicWorkCell::Ptr dwc2;
    EXPECT_NO_THROW (dwc2 = DynamicWorkCellLoader::load (savedir + "/dwc1/dwc1.dwc.xml"));

    /*************************************
     * CHECK FOR BODIES
     *************************************/
    ASSERT_EQ (dwc2->getBodies ().size (), dwc1->getBodies ().size ());
    for (size_t i = 0; i < dwc2->getBodies ().size (); i++) {
        Body::Ptr b1 = dwc1->getBodies ()[i];
        Body::Ptr b2 = dwc2->getBodies ()[i];
        EXPECT_EQ (b1->getName (), b2->getName ());
        EXPECT_EQ (b1->getBodyFrame ()->getName (), b2->getBodyFrame ()->getName ());
        EXPECT_EQ (b1->getInfo ().material, b2->getInfo ().material);
        EXPECT_EQ (b1->getInfo ().objectType, b2->getInfo ().objectType);
        EXPECT_EQ (b1->getInfo ().mass, b2->getInfo ().mass);
        EXPECT_NEAR (b1->getInfo ().masscenter.norm2 (), b2->getInfo ().masscenter.norm2 (), 1e-15);
        for (size_t j = 0; j < 3; j++) {
            for (size_t k = 0; k < 3; k++) {
                EXPECT_NEAR (b1->getInfo ().inertia (j, k), b2->getInfo ().inertia (j, k), 1e-15);
            }
        }
        EXPECT_EQ (b1->getInfo ().integratorType, b2->getInfo ().integratorType);
        ASSERT_EQ (b1->getInfo ().objects.size (), b2->getInfo ().objects.size ());
        for (size_t j = 0; j < b1->getInfo ().objects.size (); j++) {
            EXPECT_EQ (b1->getInfo ().objects[j]->getName (),
                       b2->getInfo ().objects[j]->getName ());
        }
        if (b1.cast< RigidBody > ()) {
            EXPECT_TRUE (b2.cast< RigidBody > ());
        }
        else if (b1.cast< KinematicBody > ()) {
            EXPECT_TRUE (b2.cast< KinematicBody > ());
        }
        else if (b1.cast< FixedBody > ()) {
            EXPECT_TRUE (b2.cast< FixedBody > ());
        }
        else if (b1.cast< BeamBody > ()) {
            EXPECT_TRUE (b2.cast< BeamBody > ());
        }
        else {
            EXPECT_FALSE (b2.cast< FixedBody > ());
            EXPECT_FALSE (b2.cast< KinematicBody > ());
            EXPECT_FALSE (b2.cast< RigidBody > ());
            EXPECT_FALSE (b2.cast< BeamBody > ());
        }
    }

    /*************************************
     * CHECK FOR CONSTRAINTS
     *************************************/
    ASSERT_EQ (dwc2->getConstraints ().size (), dwc1->getConstraints ().size ());
    for (size_t i = 0; i < dwc2->getConstraints ().size (); i++) {
        Constraint::Ptr c1 = dwc1->getConstraints ()[i];
        Constraint::Ptr c2 = dwc2->getConstraints ()[i];

        EXPECT_EQ (c1->getBody1 ()->getName (), c2->getBody1 ()->getName ());
        EXPECT_EQ (c1->getBody2 ()->getName (), c2->getBody2 ()->getName ());
        ASSERT_EQ (c1->getDOF (), c2->getDOF ());
        EXPECT_EQ (c1->getDOFLinear (), c2->getDOFLinear ());
        EXPECT_EQ (c1->getDOFAngular (), c2->getDOFAngular ());
        EXPECT_EQ (c1->getTransform (), c2->getTransform ());
        if (c1->getSpringParams ().enabled) {
            EXPECT_TRUE (c2->getSpringParams ().enabled);
            EXPECT_EQ (c1->getSpringParams ().compliance, c2->getSpringParams ().compliance);
            EXPECT_EQ (c1->getSpringParams ().damping, c2->getSpringParams ().damping);
        }
        else {
            EXPECT_FALSE (c2->getSpringParams ().enabled);
        }
        for (size_t j = 0; j < c1->getDOF (); j++) {
            Constraint::Limit l1 = c1->getLimit (j);
            Constraint::Limit l2 = c2->getLimit (j);

            if (l1.lowOn) {
                EXPECT_TRUE (l2.lowOn);
                EXPECT_EQ (l1.low, l2.low);
            }
            else {
                EXPECT_FALSE (l2.lowOn);
            }
            if (l1.highOn) {
                EXPECT_TRUE (l2.highOn);
                EXPECT_EQ (l1.high, l2.high);
            }
            else {
                EXPECT_FALSE (l2.highOn);
            }
        }
    }

    /*************************************
     * CHECK FOR DYNAMICDEVICES
     *************************************/
    ASSERT_EQ (dwc2->getDynamicDevices ().size (), dwc1->getDynamicDevices ().size ());
    for (size_t i = 0; i < dwc2->getDynamicDevices ().size (); i++) {
        DynamicDevice::Ptr d1 = dwc1->getDynamicDevices ()[i];
        DynamicDevice::Ptr d2 = dwc2->getDynamicDevices ()[i];

        EXPECT_EQ (d1->getName (), d2->getName ());
        EXPECT_EQ (d1->getModel ().getName (), d2->getModel ().getName ());
        EXPECT_EQ (d1->getBase ()->getName (), d2->getBase ()->getName ());

        ASSERT_EQ (d1->getLinks ().size (), d2->getLinks ().size ());
        for (size_t j = 0; j < d1->getLinks ().size (); j++) {
            EXPECT_EQ (d1->getLinks ()[j]->getName (), d2->getLinks ()[j]->getName ());
        }

        if (d1.cast< KinematicDevice > ()) {
            EXPECT_TRUE (d2.cast< KinematicDevice > ());
            KinematicDevice::Ptr k1 = d1.cast< KinematicDevice > ();
            KinematicDevice::Ptr k2 = d2.cast< KinematicDevice > ();
            EXPECT_EQ (k1->getMaxAcc (), k2->getMaxAcc ());
            EXPECT_EQ (k1->getMaxVel (), k2->getMaxVel ());
            EXPECT_EQ (k1->getJointDevice ()->getName (), k2->getJointDevice ()->getName ());
        }
        else if (d1.cast< RigidDevice > ()) {
            EXPECT_TRUE (d2.cast< RigidDevice > ());
            RigidDevice::Ptr k1 = d1.cast< RigidDevice > ();
            RigidDevice::Ptr k2 = d2.cast< RigidDevice > ();
            EXPECT_EQ (k1->getMotorForceLimits (), k2->getMotorForceLimits ());
        }
        else if (d1.cast< SuctionCup > ()) {
            EXPECT_TRUE (d2.cast< SuctionCup > ());
            RW_THROW ("TEST NOT IMPLEMENTED");
        }
        else {
            EXPECT_FALSE (d2.cast< KinematicDevice > ());
            EXPECT_FALSE (d2.cast< RigidDevice > ());
            EXPECT_FALSE (d2.cast< SuctionCup > ());
        }
    }

    /*************************************
     * CHECK FOR CONTROLLERS
     *************************************/
    ASSERT_EQ (dwc2->getControllers ().size (), dwc1->getControllers ().size ());
    for (size_t i = 0; i < dwc2->getControllers ().size (); i++) {
        SimulatedController::Ptr s1 = dwc1->getControllers ()[i];
        SimulatedController::Ptr s2 = dwc2->getControllers ()[i];

        EXPECT_EQ (s1->getControllerName (), s2->getControllerName ());
        EXPECT_EQ (s1->isEnabled (), s2->isEnabled ());
        EXPECT_EQ (s1->getControllerModel ()->getFrame ()->getName (),
                   s2->getControllerModel ()->getFrame ()->getName ());
        EXPECT_EQ (s1->getControllerModel ()->getName (), s2->getControllerModel ()->getName ());

        if (s1.cast< PDController > ()) {
            EXPECT_TRUE (s2.cast< PDController > ());
            PDController::Ptr k1 = s1.cast< PDController > ();
            PDController::Ptr k2 = s2.cast< PDController > ();
            EXPECT_EQ (k1->getName (), k2->getName ());
            EXPECT_EQ (k1->getModel ().getName (), k2->getModel ().getName ());
            EXPECT_EQ (k1->getSampleTime (), k2->getSampleTime ());
            EXPECT_EQ (k1->getControlModes (), k2->getControlModes ());
            ASSERT_EQ (k1->getParameters ().size (), k2->getParameters ().size ());
            for (size_t j = 0; j < k1->getParameters ().size (); j++) {
                EXPECT_FLOAT_EQ (k1->getParameters ()[j].P, k2->getParameters ()[j].P);
                EXPECT_FLOAT_EQ (k1->getParameters ()[j].D, k2->getParameters ()[j].D);
            }
        }
        else if (s1.cast< PoseController > ()) {
            EXPECT_TRUE (s2.cast< PoseController > ());
            PoseController::Ptr k1 = s1.cast< PoseController > ();
            PoseController::Ptr k2 = s2.cast< PoseController > ();
            EXPECT_EQ (k1->getControlledDevice ()->getName (),
                       k2->getControlledDevice ()->getName ());
            EXPECT_EQ (k1->getSampleTime (), k2->getSampleTime ());
        }
        else if (s1.cast< SerialDeviceController > ()) {
            EXPECT_TRUE (s2.cast< PoseController > ());
            SerialDeviceController::Ptr k1 = s1.cast< SerialDeviceController > ();
            SerialDeviceController::Ptr k2 = s2.cast< SerialDeviceController > ();
            EXPECT_EQ (k1->getDynamicDevice ()->getName (), k2->getDynamicDevice ()->getName ());
        }
        else if (s1.cast< SpringJointController > ()) {
            EXPECT_TRUE (s2.cast< SpringJointController > ());
            SpringJointController::Ptr k1 = s1.cast< SpringJointController > ();
            SpringJointController::Ptr k2 = s2.cast< SpringJointController > ();
            EXPECT_EQ (k1->getName (), k2->getName ());
            EXPECT_EQ (k1->getModel ().getName (), k2->getModel ().getName ());
            EXPECT_EQ (k1->getSampleTime (), k2->getSampleTime ());
            ASSERT_EQ (k1->getParameters ().size (), k2->getParameters ().size ());
            for (size_t j = 0; j < k1->getParameters ().size (); j++) {
                EXPECT_FLOAT_EQ (k1->getParameters ()[j].dampening,
                                 k2->getParameters ()[j].dampening);
                EXPECT_FLOAT_EQ (k1->getParameters ()[j].elasticity,
                                 k2->getParameters ()[j].elasticity);
                EXPECT_FLOAT_EQ (k1->getParameters ()[j].offset, k2->getParameters ()[j].offset);
            }
        }
        else {
            EXPECT_FALSE (s2.cast< BeamJointController > ());
            EXPECT_FALSE (s2.cast< BodyController > ());
            EXPECT_FALSE (s2.cast< PDController > ());
            EXPECT_FALSE (s2.cast< PoseController > ());
            EXPECT_FALSE (s2.cast< SerialDeviceController > ());
            EXPECT_FALSE (s2.cast< SpringJointController > ());
            EXPECT_FALSE (s2.cast< SyncPDController > ());
            EXPECT_FALSE (s2.cast< VelRampController > ());
        }
    }

    /*************************************
     * CHECK FOR SENSORS
     *************************************/
    ASSERT_EQ (dwc2->getSensors ().size (), dwc1->getSensors ().size ());
    for (size_t i = 0; i < dwc2->getSensors ().size (); i++) {
        SimulatedSensor::Ptr s1 = dwc1->getSensors ()[i];
        SimulatedSensor::Ptr s2 = dwc2->getSensors ()[i];

        EXPECT_EQ (s1->getName (), s2->getName ());
        EXPECT_EQ (s1->getSensorModel ()->getName (), s2->getSensorModel ()->getName ());
        EXPECT_EQ (s1->getFrame ()->getName (), s2->getFrame ()->getName ());

        if (s1.cast< SimulatedCamera > ()) {
            EXPECT_TRUE (s2.cast< SimulatedCamera > ());
            SimulatedCamera::Ptr k1 = s1.cast< SimulatedCamera > ();
            SimulatedCamera::Ptr k2 = s2.cast< SimulatedCamera > ();
            EXPECT_EQ (k1->getWidth (), k2->getWidth ());
            EXPECT_EQ (k1->getHeight (), k2->getHeight ());

            RW_THROW ("NOT IMPLEMENTED");
        }
        else if (s1.cast< SimulatedKinect > ()) {
            EXPECT_TRUE (s2.cast< SimulatedKinect > ());
            SimulatedKinect::Ptr k1 = s1.cast< SimulatedKinect > ();
            SimulatedKinect::Ptr k2 = s2.cast< SimulatedKinect > ();
            EXPECT_EQ (k1->getFrameRate (), k2->getFrameRate ());

            RW_THROW ("NOT IMPLEMENTED");
        }
        else if (s1.cast< SimulatedScanner25D > ()) {
            EXPECT_TRUE (s2.cast< SimulatedScanner25D > ());
            SimulatedScanner25D::Ptr k1 = s1.cast< SimulatedScanner25D > ();
            SimulatedScanner25D::Ptr k2 = s2.cast< SimulatedScanner25D > ();

            RW_THROW ("NOT IMPLEMENTED");
        }
        else if (s1.cast< SimulatedScanner2D > ()) {
            EXPECT_TRUE (s2.cast< SimulatedScanner2D > ());
            SimulatedScanner2D::Ptr k1 = s1.cast< SimulatedScanner2D > ();
            SimulatedScanner2D::Ptr k2 = s2.cast< SimulatedScanner2D > ();

            RW_THROW ("NOT IMPLEMENTED");
        }
        else if (s1.cast< SimulatedTactileSensor > ()) {
            EXPECT_TRUE (s2.cast< SimulatedTactileSensor > ());
            SimulatedTactileSensor::Ptr k1 = s1.cast< SimulatedTactileSensor > ();
            SimulatedTactileSensor::Ptr k2 = s2.cast< SimulatedTactileSensor > ();

            RW_THROW ("NOT IMPLEMENTED");
        }
        else if (s1.cast< TactileArraySensor > ()) {
            EXPECT_TRUE (s2.cast< TactileArraySensor > ());
            TactileArraySensor::Ptr k1 = s1.cast< TactileArraySensor > ();
            TactileArraySensor::Ptr k2 = s2.cast< TactileArraySensor > ();

            RW_THROW ("NOT IMPLEMENTED");
        }
        else {
            EXPECT_FALSE (s2.cast< SimulatedCamera > ());
            EXPECT_FALSE (s2.cast< SimulatedKinect > ());
            EXPECT_FALSE (s2.cast< SimulatedScanner25D > ());
            EXPECT_FALSE (s2.cast< SimulatedScanner2D > ());
            EXPECT_FALSE (s2.cast< SimulatedTactileSensor > ());
            EXPECT_FALSE (s2.cast< TactileArraySensor > ());
        }
    }

    /*************************************
     * CHECK FOR ContactData
     *************************************/
    ASSERT_EQ (dwc2->getContactData ().getObjectTypes ().size (),
               dwc1->getContactData ().getObjectTypes ().size ());

    for (size_t i = 0; i < dwc2->getContactData ().getObjectTypes ().size (); i++) {
        std::string s1 = dwc1->getContactData ().getObjectTypes ()[i];
        std::string s2 = dwc2->getContactData ().getObjectTypes ()[i];

        EXPECT_EQ (s1, s2);

        for (size_t j = 0; j < dwc2->getContactData ().getObjectTypes ().size (); j++) {
            std::string s1x = dwc1->getContactData ().getObjectTypes ()[j];
            std::string s2x = dwc2->getContactData ().getObjectTypes ()[j];

            try {
                ContactDataMap::NewtonData d1 = dwc1->getContactData ().getNewtonData (s1, s1x);
                ContactDataMap::NewtonData d2;
                EXPECT_NO_THROW (d2 = dwc2->getContactData ().getNewtonData (s2, s2x));
                EXPECT_DOUBLE_EQ (d1.cr, d2.cr);
            }
            catch (...) {
                EXPECT_THROW (dwc2->getContactData ().getNewtonData (s2, s2x), rw::core::Exception);
            }
            try {
                ContactDataMap::ChatterjeeData d1 =
                    dwc1->getContactData ().getChatterjeeData (s1, s1x);
                ContactDataMap::ChatterjeeData d2;
                EXPECT_NO_THROW (d2 = dwc2->getContactData ().getChatterjeeData (s2, s2x));
                EXPECT_DOUBLE_EQ (d1.crN, d2.crN);
                EXPECT_DOUBLE_EQ (d1.crT, d2.crT);
            }
            catch (...) {
                EXPECT_THROW (dwc2->getContactData ().getChatterjeeData (s2, s2x),
                              rw::core::Exception);
            }
        }
    }
    /*************************************
     * CHECK FOR MaterialData
     *************************************/

    ASSERT_EQ (dwc2->getMaterialData ().getMaterials ().size (),
               dwc1->getMaterialData ().getMaterials ().size ());

    for (size_t i = 0; i < dwc2->getMaterialData ().getMaterials ().size (); i++) {
        std::string s1 = dwc1->getMaterialData ().getMaterials ()[i];
        std::string s2 = dwc2->getMaterialData ().getMaterials ()[i];

        EXPECT_EQ (s1, s2);

        for (size_t j = 0; j < dwc2->getMaterialData ().getMaterials ().size (); j++) {
            std::string s1x = dwc1->getMaterialData ().getMaterials ()[j];
            std::string s2x = dwc2->getMaterialData ().getMaterials ()[j];
            EXPECT_EQ (s1x, s2x);

            if (dwc1->getMaterialData ().hasFrictionData (s1x, s1)) {
                ASSERT_TRUE (dwc2->getMaterialData ().hasFrictionData (s2, s2x));
                FrictionData d1 = dwc1->getMaterialData ().getFrictionData (s1, s1x);
                FrictionData d2 = dwc2->getMaterialData ().getFrictionData (s2, s2x);
                EXPECT_EQ (d1.type, d2.type);
                EXPECT_EQ (d1.typeName, d2.typeName);
                ASSERT_EQ (d1.parameters.size (), d2.parameters.size ());
                for (size_t k = 0; k < d1.parameters.size (); k++) {
                    EXPECT_EQ (d1.parameters[k].first, d2.parameters[k].first);
                    EXPECT_EQ (d1.parameters[k].second, d2.parameters[k].second);
                }
            }
            else {
                EXPECT_FALSE (dwc2->getMaterialData ().hasFrictionData (s2, s2x));
            }
        }
    }

    /*************************************
     * CHECK FOR EngineSettings
     *************************************/
    EXPECT_EQ (dwc1->getEngineSettings ().getName (), dwc2->getEngineSettings ().getName ());
    for (auto& p1 : dwc1->getEngineSettings ().getProperties ()) {
        ASSERT_TRUE (dwc2->getEngineSettings ().has (p1->getIdentifier ()));
        std::string id = p1->getIdentifier ();
        try {
            std::string v1 = dwc1->getEngineSettings ().get< std::string > (id);
            std::string v2;
            EXPECT_NO_THROW (v2 = dwc2->getEngineSettings ().get< std::string > (id));
            EXPECT_EQ (v1, v2);
        }
        catch (...) {
            EXPECT_THROW (dwc2->getEngineSettings ().get< std::string > (id), rw::core::Exception);
        }
        try {
            double v1 = dwc1->getEngineSettings ().get< double > (id);
            double v2;
            EXPECT_NO_THROW (v2 = dwc2->getEngineSettings ().get< double > (id));
            EXPECT_EQ (v1, v2);
        }
        catch (...) {
            EXPECT_THROW (dwc2->getEngineSettings ().get< double > (id), rw::core::Exception);
        }
    }
    for (auto& p2 : dwc2->getEngineSettings ().getProperties ()) {
        ASSERT_TRUE (dwc1->getEngineSettings ().has (p2->getIdentifier ()));
    }
    /*************************************
     * CHECK FOR Misc
     *************************************/

    EXPECT_FLOAT_EQ (dwc1->getCollisionMargin (), dwc2->getCollisionMargin ());
    EXPECT_EQ (dwc1->getGravity (), dwc2->getGravity ());
    EXPECT_EQ (dwc1->getWorkcell ()->getFilePath (), dwc2->getWorkcell ()->getFilePath ());
}