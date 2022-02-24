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
import sdurw_control
import sdurw_kinematics
import sdurw_loaders
import sdurw_math


class SyncVelocityRampTest(unittest.TestCase):

    def test_PrintInformation(self):
        # This is used to make debugging in gitlab CI easier
        print("\n Testing /RobWork/RobWork/unittest/trajectory/SyncVelocityRampTest.py")

    def test_SyncVelocityRampTest(self):
        n = 6

        # Test when reaching the velocity limit
        dqlimit = sdurw_math.Q(n)
        ddqlimit = sdurw_math.Q(n)
        for i in range(0, n, 1):
            dqlimit[i] = 2
            ddqlimit[i] = 4

        ramp = sdurw_control.SyncVelocityRamp(dqlimit, ddqlimit)
        q1 = sdurw_math.Q(n)
        q2 = sdurw_math.Q(n)
        for i in range(0, n, 1):
            q1[i] = 1
            q2[i] = 7

        ramp.setTarget(q1, q2)

        t = ramp.duration()
        self.assertLess(math.fabs(t - 3.5) , 1e-12)
        self.assertEqual(ramp.x(0) , q1)
        self.assertEqual(ramp.x(t) , q2)
        
        # We need to test element for element "self.assertEqual(ramp.x(t/2.0) , (q1+q2)/2.0)" is not enough and will fail
        qtest1=sdurw_math.Q((q1+q2)/2.0)
        qtest2=sdurw_math.Q(ramp.x(t/2.0))
        for i in range(qtest1.size()):
            self.assertAlmostEqual(qtest1[i], qtest2[i], delta=1e-15)

        self.assertEqual(ramp.dx(0) , sdurw_math.Q(n, 0))
        self.assertEqual(ramp.dx(t) , sdurw_math.Q(n, 0))
        self.assertEqual(ramp.dx(t/2) , dqlimit)

        self.assertEqual(ramp.ddx(0) , ddqlimit)
        self.assertEqual(ramp.ddx(t-0.001) , -ddqlimit)
        self.assertEqual(ramp.ddx(t/2.0) , sdurw_math.Q(n, 0))


        # Test when not reaching the velocity limit
        dqlimit = sdurw_math.Q(n)
        ddqlimit = sdurw_math.Q(n)
        for i in range(0, n, 1):
            dqlimit[i] = 4
            ddqlimit[i] = 4

        ramp = sdurw_control.SyncVelocityRamp(dqlimit, ddqlimit)
        q1 = sdurw_math.Q(n)
        q2 = sdurw_math.Q(n)
        for i in range(0, n, 1):
            q1[i] = 0.5
            q2[i] = -0.5

        ramp.setTarget(q1, q2)

        t = ramp.duration()
        self.assertEqual(t , 1.0)
        self.assertEqual(ramp.x(0) , q1)
        self.assertEqual(ramp.x(t) , q2)
        self.assertEqual(ramp.x(t/2.0) , (q1+q2)/2.0)

        self.assertEqual(ramp.dx(0) , sdurw_math.Q(n, 0))
        self.assertEqual(ramp.dx(t) , sdurw_math.Q(n, 0))
        self.assertEqual(ramp.dx(t/2) , ddqlimit*(-t)/2.0)

        self.assertEqual(ramp.ddx(0) , -ddqlimit)
        self.assertEqual(ramp.ddx(t-0.001) , ddqlimit)


if __name__ == '__main__':
    unittest.main()