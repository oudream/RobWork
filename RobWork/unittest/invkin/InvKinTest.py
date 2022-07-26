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
import random
# now we import robwork python bindings
import sdurw_core                as core
import sdurw_invkin              as invkin
import sdurw_kinematics          as kinematics
import sdurw_loaders             as loaders
import sdurw_math
import sdurw_models              as models
from sdurw_invkin.sdurw_invkin import IterativeIK



# Python version of C++ "typedef MakeIKSolver"
class MakeIKSolver():
    def __init__(self, SerialDevice, State):
        self.SerialDevice             = SerialDevice
        self.State                    = State


# Python version of C++ "typedef MakeMultiIKSolver"
class MakeMultiIKSolver():
    def __init__(self, TreeDevice, State):
        self.TreeDevice               = TreeDevice
        self.State                    = State



def testIKSolver(tester, solverName, maker, relativeDisplacement):
    # Load a serial device that has revolute joints only.

    # Get the current working directory
        current_working_directory = os.getcwd() 

        # incert "gtest/testfiles" and remove "unittest"
        test_directory = current_working_directory.replace("unittest", "gtest/testfiles/")

        workcellFile = test_directory + "PA10/pa10.xml"
        workcell = loaders.WorkCellLoaderFactory.load(workcellFile)

        any_device = workcell.getDevices()[0]
        device = any_device                                         # Does something like "device = dynamic_cast<SerialDevice*>(any_device)" exist in python ?

        tester.assertTrue(device)

        # Take a configuration somewhere in the valid range.
        pair = device.getBounds()

        q_zero =  (pair[0] + pair[1]) * 0.45

        maxCnt = 10

        # The maximum amounts with which each joint is displaced.
        displacements = (pair[0]- pair[1]) * relativeDisplacement

        # Create a number of small modifications of this configurations.
        q_targets = []

        for cnt in range(0, maxCnt, 1):
            q = q_zero
            dof = len(q_zero)
            for i in range(0, dof, 1):
                d = abs(displacements[i])
                q[i] =  q[i] + sdurw_math.Math.ran(-d, d)
                q_targets.append(q)

        # Use these to create targets.
        targets = []
        for i in range(0, maxCnt, 1):
            state = workcell.getDefaultState()
            device.setQ(q_targets[i], state)
            targets.append(device.baseTend(state))

        initial_state = workcell.getDefaultState()
        solver = maker(device, initial_state)

        # Check if IK can be solved for all of the targets for a starting
        # configuration of q_zero.

        device.setQ(q_zero, initial_state)
        errcnt = 0
        for i in range(0, maxCnt, 1):
            solver_result = solver.solve(targets[i], initial_state)
            if len(solver_result) == 0:
                print("\n Could not solve IK for solver ", solverName)
                errcnt = errcnt+1

        return errcnt


def testMultiIKSolver(tester, solverName, maker, relativeDisplacement):
    # Load a tree device that has revolute joints only.

    # Get the current working directory
        current_working_directory = os.getcwd() 

        # incert "gtest/testfiles" and remove "unittest"
        test_directory = current_working_directory.replace("unittest", "gtest/testfiles/")

        workcellFile = test_directory + "SchunkHand/SchunkHand.xml"
        workcell = loaders.WorkCellLoaderFactory.load(workcellFile)
        any_device = workcell.getDevices()[0].get()
        device = any_device                                         # Does something like "device = dynamic_cast<SerialDevice*>(any_device)" exist in python ?

        tester.assertTrue(device)

        # Take a configuration somewhere in the valid range.
        pair = device.getBounds()
        q_zero = 0.45 * (pair[0] + pair[1])

        maxCnt = 10

        # The maximum amounts with which each joint is displaced.
        displacements = relativeDisplacement * (pair[0]- pair[1])

        # Create a number of small modifications of this configurations.
        q_targets = []

        for cnt in range(0, maxCnt, 1):
            q = q_zero
            dof = len(q_zero)
            for i in range(0, dof, 1):
                d = displacements[i]
                q[i] =  q[i] + math.ran(-d, d)

                q_targets.append(q)

        # Use these to create targets.
        targets = []
        for i in range(0, maxCnt, 1):
            state = workcell.getDefaultState()
            device.setQ(q_targets[i], state)
            target = []
            for j in range(0, len(device.getEnds()), 1):
                target.append(device.baseTframe(device.getEnds()[j],state))
            targets.append(target)

        initial_state = workcell.getDefaultState()
        solver = maker(device, initial_state)

        # Check if IK can be solved for all of the targets for a starting
        # configuration of q_zero.

        device.setQ(q_zero, initial_state)
        errcnt = 0
        for i in range(0, maxCnt, 1):
            solver_result = solver.solve(targets[i], initial_state).empty()
            if solver_result == False:
                print("\n Could not solve IK for solver ", solverName)
                errcnt = errcnt+1

        return errcnt


def makeCCD(device, state):
    return invkin.ownedPtr(invkin.CCDSolver(device, state))


def makeJacobianIKSolverSVD(device, state):
    sol = invkin.ownedPtr(invkin.JacobianIKSolver(device,state))
    sol.setSolverType(invkin.JacobianIKSolver.SVD)
    return sol


def makeJacobianIKSolverTranspose(device, state):
    sol = invkin.ownedPtr(invkin.JacobianIKSolver(device,state))
    sol.setSolverType(invkin.JacobianIKSolver.Transpose)
    sol.setMaxIterations(16)
    return sol


def makeJacobianIKSolverDLS(device, state):
    sol = invkin.ownedPtr(invkin.JacobianIKSolver(device,state))
    sol.setSolverType(invkin.JacobianIKSolver.DLS)
    sol.setMaxIterations(50)
    return sol


def makeJacobianIKSolverMSVD(device, state):
    sol = invkin.ownedPtr(invkin.JacobianIKSolverM(device,state))
    # Should use SVD as default though
    sol.setSolverType(invkin.JacobianIKSolverM.SVD)
    # sol.setMaxLocalStep(0.4,5.0)
    return sol


def makeJacobianIKSolverMTranspose(device, state):
    sol = invkin.ownedPtr(invkin.JacobianIKSolverM(device,state))
    # Should use SVD as default though
    sol.setSolverType(invkin.JacobianIKSolverM.Transpose)
    sol.setMaxIterations(350)
    return sol


def makeJacobianIKSolverMDLS(device, state):
    sol = invkin.ownedPtr(invkin.JacobianIKSolverM(device,state))
    # Should use SVD as default though
    sol.setSolverType(invkin.JacobianIKSolverM.DLS)
    sol.setMaxIterations(155)
    return sol


def testClosedFormWithQ(tester, q, dhparams):
    # Transform from the three intersection axis to tool
    T06 = sdurw_math.Transform3D().identity()

    for i in range(0, len(dhparams), 1):
#        print(dhparams[i])
        T06 = T06*sdurw_math.Transform3D().craigDH(
                                                    dhparams[i].alpha(),
                                                    dhparams[i].a(),
                                                    dhparams[i].d(),
                                                    q[i] )

    T6tool = sdurw_math.Transform3D(sdurw_math.Vector3D(0.1, 0.2, 0.3), sdurw_math.RPY(1, 2, 3))
    baseTend = T06*T6tool

    solver = invkin.PieperSolver(dhparams, T6tool)
    state = kinematics.State()
    solutions = solver.solve(baseTend, state)

    for qres in solutions:
        T06 = sdurw_math.Transform3D().identity()
        for i in range(0, len(dhparams), 1):
            T06 = T06*sdurw_math.Transform3D().craigDH(
                                                        dhparams[i].alpha(),
                                                        dhparams[i].a(),
                                                        dhparams[i].d(),
                                                        qres[i] )

        T6tool = sdurw_math.Transform3D(sdurw_math.Vector3D(0.1, 0.2, 0.3), sdurw_math.RPY(1, 2, 3))
        baseTend2 = T06*T6tool

        diff = sdurw_math.inverse(baseTend)*baseTend2

        for i in range(0, 3, 1):
            for j in range(0, 4, 1):
                if (i == j):
                    tester.assertLess(math.fabs(diff[i,j]-1) , 1e-12)
                else:
                    tester.assertLess(math.fabs(diff[i,j])   , 1e-12)

    return len(solutions)


class InvKinTest(unittest.TestCase):

    def test_IterativeInverseKinematics(self):
        errcnt = 0
        # We seed the random number generator so that we get reproducible results.
        random.seed(0)

        # The IK solvers really aren't impressive right now. In particular the CCD
        # solver isn't very reliable. Some tuning of these is necessary, I think.
        # Also perhaps the testIKSolver() should just verify that a _reasonably_
        # large percentage of the IK calculations succeed.

        # The relative displacement has been set to a small enough number to ensure that only a few iterations are needed to find a solution for the Transpose and DLS solvers
        # Increase the number of allowed iterations accordingly to the chosen relative displacement

        # Too slow to be considered correct.

        errcnt = testIKSolver(self, "CCD", makeCCD, 0.002)
        self.assertEqual(errcnt, 0)
        # testIKSolver("IKQPSolver", makeIKQPSolver, 0.2);
        errcnt = testIKSolver(self, "JacobianIKSolver using SVD", makeJacobianIKSolverSVD, 0.2)
        self.assertEqual(errcnt, 0)
        # errcnt = testIKSolver("JacobianIKSolver using Transpose", makeJacobianIKSolverTranspose, 0.00002);
        self.assertLessEqual(errcnt, 2) # maxIteration is set low enough to not find a solution for up to 2 of the tests
        # errcnt = testIKSolver("JacobianIKSolver using DLS", makeJacobianIKSolverDLS, 0.00002);
        self.assertEqual(errcnt, 0)


    def test_ClosedFormInverseKinematics(self):
        q = sdurw_math.Q().zero(6)

        dhparams = []
        dhparams.append(models.DHParameterSet(0, 0, 0, 0))
        dhparams.append(models.DHParameterSet(-90*sdurw_math.Deg2Rad, 0.26, 0, 0))
        dhparams.append(models.DHParameterSet(0, 0.68, 0, 0))
        dhparams.append(models.DHParameterSet(-90*sdurw_math.Deg2Rad, -0.035, 0.67, 0))
        dhparams.append(models.DHParameterSet(-90*sdurw_math.Deg2Rad, 0, 0, 0))
        dhparams.append(models.DHParameterSet(90*sdurw_math.Deg2Rad, 0, 0, 0))

        cnt = testClosedFormWithQ(self, q, dhparams)
        self.assertEqual(cnt, 8)

        q[0] = 0.5
        q[1] = 1
        q[2] = 1.5
        q[3] = 2
        q[4] = 2.5
        q[5] = 3

        cnt = testClosedFormWithQ(self, q, dhparams)
        self.assertEqual(cnt, 8)

        # Test special case with a1=0
        dhparams2 = []
        dhparams2.append(models.DHParameterSet(0, 0, 0, 0))
        dhparams2.append(models.DHParameterSet(-90*sdurw_math.Deg2Rad, 0, 0, 0))
        dhparams2.append(models.DHParameterSet(0, 0.68, 0, 0))
        dhparams2.append(models.DHParameterSet(-90*sdurw_math.Deg2Rad, -0.035, 0.67, 0))
        dhparams2.append(models.DHParameterSet(-90*sdurw_math.Deg2Rad, 0, 0, 0))
        dhparams2.append(models.DHParameterSet(90*sdurw_math.Deg2Rad, 0, 0, 0))

        cnt = testClosedFormWithQ(self, q, dhparams2)
        self.assertEqual(cnt, 8)

        # Test special case with alpha1 = 0
        dhparams3 = []
        dhparams3.append(models.DHParameterSet(0, 0, 0, 0))
        dhparams3.append(models.DHParameterSet(0, 0.26, 0, 0))
        dhparams3.append(models.DHParameterSet(90*sdurw_math.Deg2Rad, 0.68, 0, 0))
        dhparams3.append(models.DHParameterSet(-90*sdurw_math.Deg2Rad, -0.035, 0.67, 0))
        dhparams3.append(models.DHParameterSet(-90*sdurw_math.Deg2Rad, 0, 0, 0))
        dhparams3.append(models.DHParameterSet(90*sdurw_math.Deg2Rad, 0, 0, 0))

        q[0] = 1
        q[1] = 1
        q[2] = 1
        q[3] = 1.4
        q[4] = 1.5
        q[5] = 1.6

        cnt = testClosedFormWithQ(self, q, dhparams3)
# Dette skal enables igen
#        self.assertEqual(cnt, 8)


if __name__ == '__main__':
    unittest.main()