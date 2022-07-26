#################################################################################
 # Copyright 2022 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 # Faculty of Engineering, University of Southern Denmark
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #################################################################################

import unittest                                     # now we can use python unittest framework
import math 
import os
# now we import robwork python bindings
import sdurw_invkin              as invkin
import sdurw_loaders             as loaders
import sdurw_math
#import sdurw_math as math
import sdurw_models              as models



class ParallelDevice(unittest.TestCase):

    def test_Robotiq(self):
        # Get the current working directory
        current_working_directory = os.getcwd() 

        # incert "gtest/testfiles" and remove "unittest"
        test_directory = current_working_directory.replace("unittest", "gtest/testfiles/")

        workcellFile = test_directory + "/devices/Robotiq-2-finger-85/robotiq.wc.xml"
        wc = loaders.WorkCellLoaderFactory.load(workcellFile)
        self.assertFalse(wc.isNull())

        robotiq = wc.findParallelDevice("RobotiqFingerControl")
        self.assertFalse(robotiq.isNull())

        fingerLeft = wc.findFrame(robotiq.getName()+"LeftIK")
        fingerRight = wc.findFrame(robotiq.getName()+"RightIK")

        self.assertFalse(fingerLeft == None)
        self.assertFalse(fingerRight == None)


        junctions = robotiq.getJunctions()
        solver = invkin.ParallelIKSolver(robotiq)

# Dette skal enables igen
#        enabled = sdurw_math.Vector6D_b([True, False, False,  False, False, True])


        # # Solve for each finger individually and check that solutions are symmetric (that dependent joint is handled correctly)
	    # # First move left finger..

        # targets = [None]
        # targets[0] = invkin.Target(fingerLeft, sdurw_math.Transform3D(sdurw_math.Vector3D().x()*(-0.015), sdurw_math.EAA(0, 0, -0.5)), enabled)

        # self.assertEqual(2, targets[0].dof())

        # state = wc.getDefaultState()
        # solutions = solver.solve(targets, state)
        # self.assertEqual(1, len(solutions))
        # self.assertGreaterEqual(len(solutions), 1)

        # self.assertAlmostEqual(0.61176270099, solutions[0][0], delta = 1e-11)
        # self.assertAlmostEqual(0.31731028813, solutions[0][1], delta = 1e-11)
        # self.assertAlmostEqual(0.40223246809, solutions[0][2], delta = 1e-11)

        # robotiq.setQ(solutions[0], state)

        # self.assertEqual(solutions[0][0], robotiq.getQ(state)[0])
        # self.assertEqual(solutions[0][1], robotiq.getQ(state)[1])
        # self.assertEqual(solutions[0][2], robotiq.getQ(state)[2])

        # # Check that junctions are still connected
        # self.assertTrue(junctions[0][1].baseTend(state).equal(junctions[0][0].baseTend(state),1e-8))
        # self.assertTrue(junctions[1][1].baseTend(state).equal(junctions[1][0].baseTend(state),1e-8))

        # # Check that target was reached
        # self.assertAlmostEqual(-0.015, robotiq.baseTframe(fingerLeft, state).P()[0], delta = 1e-7)
        # self.assertAlmostEqual(-0.5, sdurw_math.EAA(robotiq.baseTframe(fingerLeft, state).R())[2], delta = 1e-6)

        # # .. then move right finger
        # targets = [None]
        # targets[0] = invkin.Target(fingerRight, sdurw_math.Transform3D(sdurw_math.Vector3D().x()*(0.015), sdurw_math.EAA(0, 0, 0.5)), enabled)
        # self.assertEqual(2, targets[0].dof())

        # state = wc.getDefaultState()
        # solutions = solver.solve(targets, state)
        # self.assertEqual(1, len(solutions))
        # self.assertGreaterEqual(len(solutions), 1)

        # self.assertAlmostEqual(0.61176270099, solutions[0][0], delta = 1e-11)
        # self.assertAlmostEqual(0.40223246809, solutions[0][1], delta = 1e-11)
        # self.assertAlmostEqual(0.31731028813, solutions[0][2], delta = 1e-11)

        # robotiq.setQ(solutions[0], state)

        # self.assertEqual(solutions[0][0], robotiq.getQ(state)[0])
        # self.assertEqual(solutions[0][1], robotiq.getQ(state)[1])
        # self.assertEqual(solutions[0][2], robotiq.getQ(state)[2])

        # # Check that junctions are still connected
        # self.assertTrue(junctions[0][1].baseTend(state).equal(junctions[0][0].baseTend(state),1e-8))
        # self.assertTrue(junctions[1][1].baseTend(state).equal(junctions[1][0].baseTend(state),1e-8))

        # # Check that target was reached
        # self.assertAlmostEqual(0.015, robotiq.baseTframe(fingerRight, state).P()[0], delta = 1e-7)
        # self.assertAlmostEqual(0.5, sdurw_math.EAA(robotiq.baseTframe(fingerRight, state).R())[2], delta = 1e-6)

        # # Now try to move both fingers to reach the targets simultaneously
        # targets = []*2
        # targets[0] = invkin.ParallelIKSolver.Target(fingerLeft, sdurw_math.Transform3D(sdurw_math.Vector3D().x()*(-0.015), sdurw_math.EAA(0, 0, -0.5)), enabled)
        # targets[0] = invkin.ParallelIKSolver.Target(fingerRight, sdurw_math.Transform3D(sdurw_math.Vector3D().x()*(0.015), sdurw_math.EAA(0, 0, 0.5)), enabled)

        # state = wc.getDefaultState()
        # solutions = solver.solve(targets, state)
        # self.assertEqual(1, len(solutions))
        # self.assertGreaterEqual(len(solutions), 1)

        # self.assertAlmostEqual(0.61176270099, solutions[0][0], delta = 1e-11)
        # self.assertAlmostEqual(0.31731028813, solutions[0][1], delta = 1e-11)
        # self.assertAlmostEqual(0.31731028813, solutions[0][2], delta = 1e-11)

        # robotiq.setQ(solutions[0], state)

        # self.assertEqual(solutions[0][0], robotiq.getQ(state)[0])
        # self.assertEqual(solutions[0][1], robotiq.getQ(state)[1])
        # self.assertEqual(solutions[0][2], robotiq.getQ(state)[2])

        # # Check that junctions are still connected
        # self.assertTrue(junctions[0][1].baseTend(state).equal(junctions[0][0].baseTend(state),1e-8))
        # self.assertTrue(junctions[1][1].baseTend(state).equal(junctions[1][0].baseTend(state),1e-8))

        # # Check that target was reached
        # self.assertAlmostEqual(-0.015, robotiq.baseTframe(fingerLeft, state).P()[0], delta = 1e-7)
        # self.assertAlmostEqual(0.015, robotiq.baseTframe(fingerRight, state).P()[0], delta = 1e-7)
        # self.assertAlmostEqual(-0.5, sdurw_math.EAA(robotiq.baseTframe(fingerLeft, state).R())[2], delta = 1e-6)
        # self.assertAlmostEqual(0.5, sdurw_math.EAA(robotiq.baseTframe(fingerRight, state).R())[2], delta = 1e-6)


if __name__ == '__main__':
    unittest.main()