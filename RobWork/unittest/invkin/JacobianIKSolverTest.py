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
import sdurw_core                as core
import sdurw_invkin              as invkin
import sdurw_kinematics          as kinematics
import sdurw_math
import sdurw_models              as models

import numpy as np


def getKukaIIWA(stateStructure):
    base   = kinematics.ownedPtr(kinematics.FixedFrame("Base",sdurw_math.Transform3D().identity() ))
    joint1 = models.ownedPtr(models.RevoluteJoint("Joint1", sdurw_math.Transform3D(sdurw_math.Vector3D(0, 0,    0.158))))
    joint2 = models.ownedPtr(models.RevoluteJoint("Joint2", sdurw_math.Transform3D(sdurw_math.Vector3D(0, 0,    0.182), sdurw_math.RPY(0, 0, -math.pi/2))))
    joint3 = models.ownedPtr(models.RevoluteJoint("Joint3", sdurw_math.Transform3D(sdurw_math.Vector3D(0, -0.182, 0)  , sdurw_math.RPY(0, 0,  math.pi/2))))
    joint4 = models.ownedPtr(models.RevoluteJoint("Joint4", sdurw_math.Transform3D(sdurw_math.Vector3D(0, 0,    0.218), sdurw_math.RPY(0, 0,  math.pi/2))))
    joint5 = models.ownedPtr(models.RevoluteJoint("Joint5", sdurw_math.Transform3D(sdurw_math.Vector3D(0, 0.182, 0),    sdurw_math.RPY(0, 0, -math.pi/2))))
    joint6 = models.ownedPtr(models.RevoluteJoint("Joint6", sdurw_math.Transform3D(sdurw_math.Vector3D(0, 0,    0.218), sdurw_math.RPY(0, 0, -math.pi/2))))
    joint7 = models.ownedPtr(models.RevoluteJoint("Joint7", sdurw_math.Transform3D(sdurw_math.Vector3D().zero(),        sdurw_math.RPY(0, 0,  math.pi/2))))
    end    = kinematics.ownedPtr(kinematics.FixedFrame("TCP",sdurw_math.Transform3D(sdurw_math.Vector3D(0, 0, 0.126))))

    stateStructure.addFrame(base)
    stateStructure.addFrame(joint1, base)
    stateStructure.addFrame(joint2, joint1)
    stateStructure.addFrame(joint3, joint2)
    stateStructure.addFrame(joint4, joint3)
    stateStructure.addFrame(joint5, joint4)
    stateStructure.addFrame(joint6, joint5)
    stateStructure.addFrame(joint7, joint6)
    stateStructure.addFrame(end, joint7)

    state = stateStructure.getDefaultState()

    device = models.ownedPtr(models.SerialDevice(base, end, "KukaIIWA", state))

    # Python version of C++ "typedef PairQ"
    # It is defined here "~/RobWork/RobWork/src/rwlibs/swig/sdurw_math.i"
    bounds = kinematics.PairQ()
#    bounds.first = sdurw_math.Q(7, -np.deg2rad(170), -np.deg2rad(120),  -np.deg2rad(170), -np.deg2rad(120),  -np.deg2rad(170), -np.deg2rad(120), -np.deg2rad(175))
    bounds.first = sdurw_math.Q(7, -sdurw_math.Deg2Rad * 170, -sdurw_math.Deg2Rad * 120,  -sdurw_math.Deg2Rad * 170, -sdurw_math.Deg2Rad * 120,  -sdurw_math.Deg2Rad * 170, -sdurw_math.Deg2Rad * 120, -sdurw_math.Deg2Rad * 175)
    bounds.second = -bounds.first
    device.setBounds(bounds)

    return device


class WeightedJacobianIKSolverTest(unittest.TestCase):

    def test_UnityWeights(self):
        stateStructure = kinematics.StateStructure()
        state = stateStructure.getDefaultState()
        device = getKukaIIWA(stateStructure)

        solver_weighted = invkin.JacobianIKSolver(device.cptr(), state)    # Forklar mig lige, hvad cptr() gør?
        solver_weighted.setCheckJointLimits(True)
        solver_weighted.setSolverType(invkin.JacobianIKSolver.Weighted)

        print("\n  HVORDAN løser man Eigen::VectorXd::Ones(7)")
        #solver_weighted.setWeightVector(np.ones(7))
        #solver_weighted.setWeightVector([1,1,1,1,1,1,1])
        print("TEST 1")
        qstart = sdurw_math.Q(7,  0.1,  0.1, 0.1,  0.1,  0.1, 0.1,  0.1)
        
        qRefs = [None]*10
        qRefs[0] = sdurw_math.Q(7,  2.4,  1.4, -2.2,  2.0,  2.5, -1.2,  0.8)
        qRefs[1] = sdurw_math.Q(7, -1.1, -1.7,  0.3, -0.9, -1.8,  0.2,  3.0)
        qRefs[2] = sdurw_math.Q(7,  2.7,  2.1,  2.8,  2.0, -2.0,  0.9,  2.9)
        qRefs[3] = sdurw_math.Q(7,  2.9,  1.9, -2.3, -0.1,  1.8,  1.3, -1.2)
        qRefs[4] = sdurw_math.Q(7, -2.1, -2.1, -0.5, -1.6,  2.5,  0.6,  1.8)
        qRefs[5] = sdurw_math.Q(7,  2.2,  1.9,  0.0,  0.7,  1.8, -1.9, -0.8)
        qRefs[6] = sdurw_math.Q(7,  2.1, -1.2,  2.6,  0.8,  1.0, -0.4,  1.6)
        qRefs[7] = sdurw_math.Q(7,  1.4,  1.0, -0.1, -0.5, -0.5,  0.7, -2.0)
        qRefs[8] = sdurw_math.Q(7, -2.0, -0.8,  1.2,  1.2, -2.8, -0.8, -1.4)
        qRefs[9] = sdurw_math.Q(7,  2.2, -1.9, -2.1, -1.7,  2.9,  1.4,  2.0)

        for refI in range(0, len(qRefs), 1):
            qRef = qRefs[refI]
            #           Does something like "gtest SCOPED_TRACE()" exist in unittest ?
            device.setQ(qRef,state)
            T = device.baseTend(state)

            print("\n MANGLER   Segmentation fault (core dumped)        DET VIRKER IKKE")
           # device.setQ(qstart, state)

            solutions = solver_weighted.solve(T, state)
            print("\n \n Reference no :", refI, "\n Number of solutions :", len(solutions) )
            for solution in range(0, len(solutions), 1):
                device.setQ(solutions[solution],state)
                print("\n solution[", solution, "] ", solutions[solution], sep="" )
                Tfound = device.baseTend(state)
                self.assertAlmostEqual(0, (Tfound.P()-T.P()).normInf(), delta = 1e-6)
                self.assertTrue(T.R().equal(Tfound.R(), 1e-6))
        print("TEST 2")


    def test_BaseJointWeightedHigh(self):

        stateStructure = kinematics.StateStructure()
        state = stateStructure.getDefaultState()
        device = getKukaIIWA(stateStructure)

        solver_ref = invkin.JacobianIKSolver(device.cptr(), state)    # Forklar mig lige, hvad cptr() gør?
        solver_ref.setCheckJointLimits(True)

        solver_weighted = invkin.JacobianIKSolver(device.cptr(), state)    # Forklar mig lige, hvad cptr() gør?
        solver_weighted.setCheckJointLimits(True)
        solver_weighted.setSolverType(invkin.JacobianIKSolver.Weighted)

        print("\n  HVORDAN løser man Eigen::VectorXd::Ones(7)")
        #solver_weighted.setWeightVector(np.ones(7))
        #solver_weighted.setWeightVector([1,1,1,1,1,1,1])
        print("TEST 3")
        qstart = sdurw_math.Q(7,  0.1,  0.1, 0.1,  0.1,  0.1, 0.1,  0.1)
        print("TEST 3.1")
        qRefs = [None]*3
        qRefs[0] = sdurw_math.Q(7,  2.4,  1.4, -2.2,  2.0,  2.5, -1.2,  0.8)
        qRefs[1] = sdurw_math.Q(7, -1.1, -1.7,  0.3, -0.9, -1.8,  0.2,  3.0)
        qRefs[2] = sdurw_math.Q(7,  2.3,  2.1,  2.1,  2.0, -2.0,  0.9,  2.9)
        print("TEST 3.2")

# Dette skal enables igen
        # for refI in range(0, len(qRefs), 1):
        #     print("TEST 3.3")
        #     qRef = qRefs[refI]
        #     #           Does something like "gtest SCOPED_TRACE()" exist in unittest ?
        #     device.setQ(qRef,state)
        #     T = device.baseTend(state)
        #     device.setQ(qstart, state)
        #     print("TEST 3.4")
        #     print(T)
        #     print(state)
        #     weighted_solutions = solver_weighted.solve(T, state)
        #     print("TEST 3.4.1")
        #     #print(weighted_solutions)
        #     device.setQ(qstart, state)
        #     print("TEST 3.4.2")
        #     ref_solutions = solver_ref.solve(T, state)
        #     print("TEST 3.5")
        #     print(ref_solutions)
        #     print("TEST 3.6")
        #     # check weighted solutions
        #     for solution in weighted_solutions:
        #         device.setQ(solution, state)
        #         Tfound = device.baseTend(state)
        #         self.assertAlmostEqual(0, (Tfound.P()-T.P()).normInf(), delta = 1e-6)
        #         self.assertTrue(T.R().equal(Tfound.R(), 1e-6))

        #     # check ref solutions
        #     for solution in ref_solutions:
        #         device.setQ(solution, state)
        #         Tfound = device.baseTend(state)
        #         self.assertAlmostEqual(0, (Tfound.P()-T.P()).normInf(), delta = 1e-6)
        #         self.assertTrue(T.R().equal(Tfound.R(), 1e-6))
            
        #     # check if wrist joint moves less than ref solution
        #     self.assertEqual(len(weighted_solutions), len(ref_solutions))
        #     for i in range(0, len(weighted_solutions), 1):
        #         weighted_solutions[i]
        #         ref_solutions[i]
        #         weighted_dq = weighted_solutions[i] - qstart
        #         ref_dq = ref_solutions[i] - qstart
        #         self.assertLess(math.fabs(weighted_dq[6]), math.fabs(ref_dq[6]))
        # print("TEST 4")


if __name__ == '__main__':
    unittest.main()