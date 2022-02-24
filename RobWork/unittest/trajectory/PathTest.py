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
import os
import sdurw_loaders
import sdurw_math
import sdurw_trajectory as trajectory

class PathTest(unittest.TestCase):

    def test_PrintInformation(self):
        # This is used to make debugging in gitlab CI easier
        print("\n Testing /RobWork/RobWork/unittest/trajectory/PathTest.py")


    def test_simpleQPath(self):
        path = trajectory.QPath()
        q = sdurw_math.Q(7)
        q[0] = 1
        path.append(q)
        q[0] = 2
        path.append(q)
        q[0] = 3
        path.append(q)

        index = 1
        for q in path:
            self.assertEqual(q[0], index)
            index = index+1


    def test_simpleTransform3DPath(self):
        path  = trajectory.PathTransform3D()
        t = sdurw_math.Transform3D()
        t[0, 0] = 1
        path.append(t)
        t[0, 0] = 2
        path.append(t)
        t[0, 0] = 3
        path.append(t)

        index = 1
        for t in path:
            self.assertEqual(t[0,0], index)
            index = index+1


    def test_simpleStatePath(self):
#        statepath = trajectory.PathState()
        
        # Get the current working directory
        current_working_directory = os.getcwd()

        # incert "unittest" and remove "gtest/testfiles"
        test_directory = current_working_directory.replace("unittest", "gtest/testfiles")

        wcPath = test_directory + "/MultiRobotDemo/Scene.wc.xml"
        workcell = sdurw_loaders.WorkCellLoaderFactory.load(wcPath)

        self.assertFalse(workcell.isNull())
        dev = workcell.getDevices()[0]
        defstate = workcell.getDefaultState()

        # Test StatePath and the possibility of creating
        statepath = trajectory.PathState()
        state = defstate

#        statepath.append(state)
        statepath.push_back(state)
        q = dev.getQ(state)
        for i in range(0, q.size(), 1):
            q[i] = 0.5
        dev.setQ(q, state)
        statepath.push_back(state)
        for i in range(0, q.size(), 1):
            q[i] = -1
        dev.setQ(q, state)
        statepath.push_back(state)

        statetrajectory = trajectory.TrajectoryFactory.makeLinearTrajectoryUnitStep(statepath)
        s0 = statetrajectory.x(0)
        q  = dev.getQ(s0)
        self.assertEqual(q[0], 0)

        s05 = statetrajectory.x(0.5)
        q  = dev.getQ(s05)
        self.assertEqual(q[0], 0.25)

        s1 = statetrajectory.x(1)
        q  = dev.getQ(s1)
        self.assertEqual(q[0], 0.5)

        s15 = statetrajectory.x(1.5)
        q  = dev.getQ(s15)
        self.assertEqual(q[0], -0.25)

        s2 = statetrajectory.x(2)
        q  = dev.getQ(s2)
        self.assertEqual(q[0], -1)


        # Test StatePath and the possibility of creating
        timedStatePath = trajectory.PathTimedState()
        workcell = sdurw_loaders.WorkCellLoaderFactory.load(wcPath)

        self.assertFalse(workcell.isNull())

        dev = workcell.getDevices()[0]
        state = workcell.getDefaultState()
        timedStatePath.push_back(trajectory.TimedState(0, state))

        q = dev.getQ(state)
        for i in range(0, q.size(), 1):
            q[i] = 0.5
        dev.setQ(q, state)
        timedStatePath.push_back(trajectory.TimedState(12, state))

        q = dev.getQ(state)
        for i in range(0, q.size(), 1):
            q[i] = -1
        dev.setQ(q, state)
        timedStatePath.push_back(trajectory.TimedState(24, state))

        statetrajectory = trajectory.TrajectoryFactory_makeLinearTrajectory(timedStatePath)
        s0 = statetrajectory.x(0)
        q  = dev.getQ(s0)
        self.assertEqual(q[0], 0)

        s3 = statetrajectory.x(3)
        q  = dev.getQ(s3)
        self.assertEqual(q[0], 0.125)

        s1 = statetrajectory.x(12)
        q  = dev.getQ(s1)
        self.assertEqual(q[0], 0.5)

        s15 = statetrajectory.x(18)
        q  = dev.getQ(s15)
        self.assertEqual(q[0], -0.25)

        s2 = statetrajectory.x(24)
        q  = dev.getQ(s2)
        self.assertEqual(q[0], -1)


if __name__ == '__main__':
    unittest.main()