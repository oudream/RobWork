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
#import random
#import sys
# now we import robwork python bindings
#import sdurw_core                as core
#import sdurw_common              as common
#import sdurw_geometry            as geometry
import sdurw_invkin              as invkin
import sdurw_kinematics          as kinematics
import sdurw_loaders             as loaders
import sdurw_math
#import sdurw_math as math
import sdurw_models              as models
#import sdurw_pathplanners        as pathplanners
#import sdurw_proximity           as proximity
#import sdurw_proximitystrategies as proximitystrategies
#import sdurw_trajectory          as trajectory


class ClosedFormIKSolverURTest(unittest.TestCase):

    def test_ClosedFormIKSolverURTest(self):
        EPS = 1e-14

        # Get the current working directory
        current_working_directory = os.getcwd() 

        # incert "gtest/testfiles" and remove "unittest"
        test_directory = current_working_directory.replace("unittest", "gtest/testfiles/")

        workcellFile = test_directory + "devices/UR6855A/UR6855A.wc.xml"
        wc = loaders.WorkCellLoaderFactory.load(workcellFile)
        self.assertFalse(wc.isNull())
        device = wc.findDevice("UR-6-85-5-A")
        self.assertFalse(device.isNull())

        solver = invkin.ClosedFormIKSolverUR(device, wc.getDefaultState())

        state = wc.getDefaultState()
        qRef = sdurw_math.Q(6,0.352,-2.408,-0.785,-1.78,2.199,0.785)
        device.setQ(qRef, state)
        T = device.baseTend(state)
        solutions = solver.solve(T, state)
        self.assertGreater(len(solutions), 0)
        found = False
        for sol in solutions:
            if ((sol-qRef).normInf() <= EPS):
                found = True
            device.setQ(sol, state)
            Tfound = device.baseTend(state)
            self.assertLess((Tfound.P()-T.P()).normInf(), EPS)
            self.assertTrue(T.R().equal(Tfound.R(), EPS))
        self.assertTrue(found)


        workcellFile = test_directory + "devices/UR5e/UR5e.xml"
        wc = loaders.WorkCellLoaderFactory.load(workcellFile)
        self.assertFalse(wc.isNull())
        device = wc.findDevice("UR5e_2018")
        self.assertFalse(device.isNull())

        solver = invkin.ClosedFormIKSolverUR(device, wc.getDefaultState())

        state     = kinematics.State()
        T         = sdurw_math.Transform3D()
        solutions = []

        state = wc.getDefaultState()
        qRef = sdurw_math.Q(6,0.352,-2.408,-0.785,-1.78,2.199,0.785)
        device.setQ(qRef, state)
        T = device.baseTend(state)
        solutions = solver.solve(T, state)
        self.assertGreater(len(solutions), 0)
        found = False
        for sol in solutions:
            if ((sol-qRef).normInf() <= EPS):
                found = True
            device.setQ(sol, state)
            Tfound = device.baseTend(state)
            self.assertLess((Tfound.P()-T.P()).normInf(), EPS)
            self.assertTrue(T.R().equal(Tfound.R(), EPS))
        self.assertTrue(found)

        state = wc.getDefaultState()
        T = sdurw_math.Transform3D(sdurw_math.Vector3D(0.0853719, -0.0565455, 0.725042), sdurw_math.RPY(-0.785398, 3.16998e-17, 4.71377e-17))
        solutions = solver.solve(T, state)
        self.assertEqual(len(solutions), 0)


if __name__ == '__main__':
    unittest.main()