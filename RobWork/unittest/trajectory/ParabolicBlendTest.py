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
import sdurw_kinematics
import sdurw_math
import sdurw_trajectory


# The range function does not work with floats. Only integer values can be specified as the start, stop, and step arguments.
# This function should rander the problem.
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step


class ParabolicBlendTest(unittest.TestCase):

    def test_PrintInformation(self):
        # This is used to make debugging in gitlab CI easier
        print("\n Testing /RobWork/RobWork/unittest/trajectory/ParabolicBlendTest.py")

    def test_Constructor(self):
        p1 = sdurw_trajectory.LinearInterpolatorVector2D(sdurw_math.Vector2D(0, 0), sdurw_math.Vector2D(1, 5), 2)
        p2 = sdurw_trajectory.LinearInterpolatorVector2D(sdurw_math.Vector2D(1, 5), sdurw_math.Vector2D(1, 2), 0.1)
        p3 = sdurw_trajectory.LinearInterpolatorVector2D(sdurw_math.Vector2D(1, 5), sdurw_math.Vector2D(2, 0), 2)
        p4 = sdurw_trajectory.LinearInterpolatorVector2D(sdurw_math.Vector2D(1, 5), sdurw_math.Vector2D(2, 0), 0.1)
        p5 = sdurw_trajectory.LinearInterpolatorVector2D(sdurw_math.Vector2D(1, 4), sdurw_math.Vector2D(2, 0), 2)
        p6 = sdurw_trajectory.LinearInterpolatorVector2D(sdurw_math.Vector2D(1, 5), sdurw_math.Vector2D(1, 0), 2.5)

        with self.assertRaises(RuntimeError):
            b1 = sdurw_trajectory.ParabolicBlendVector2D(p1, p2, 0.5)

        b2 = sdurw_trajectory.ParabolicBlendVector2D(p1, p3, 0.5)

        with self.assertRaises(RuntimeError):
            self.assertRaises(RuntimeError, b3 = sdurw_trajectory.ParabolicBlendVector2D(p1, p4, 0.5))

        with self.assertRaises(RuntimeError):
            self.assertRaises(RuntimeError, b4 = sdurw_trajectory.ParabolicBlendVector2D(p1, p5, 0.5))
        
        b5 = sdurw_trajectory.ParabolicBlendVector2D(p1, p6, 0.6)

        self.assertEqual(b2.tau1(), b2.tau2())
        self.assertEqual(b2.tau1(), 0.5)
        self.assertEqual(b2.duration(), 4.0)

        self.assertEqual(b5.tau1(), b5.tau2())
        self.assertEqual(b5.tau1(), 0.6)
        self.assertEqual(b5.duration(), 4.5)


    def test_PathVerification(self):
        traj = sdurw_trajectory.InterpolatorTrajectoryVector2D()
        intersect = sdurw_math.Vector2D(1, 5)
        end = sdurw_math.Vector2D(2, 0)
        p1 = sdurw_trajectory.ownedPtr(sdurw_trajectory.LinearInterpolatorVector2D(sdurw_math.Vector2D(0, 0), intersect, 2))
        p2 = sdurw_trajectory.ownedPtr(sdurw_trajectory.LinearInterpolatorVector2D(intersect, end, 1))
        b1 = sdurw_trajectory.ParabolicBlendVector2D(p1.deref(), p2.deref(), 0.25)

        traj.add(p1)
        traj.add(b1, p2)
#
        min_dist_to_intersect = (traj.x(0) - intersect).norm2()
        max_dist_between_points = 0

        acc_vel = traj.dx(0)
        acc_pos = traj.x(0)
        vel_pos = traj.x(0)

        dt = 0.001
        for t in range_with_floats(dt, traj.duration(), dt):
            t = t+dt
            d_to_i = (traj.x(t) - intersect).norm2()
            if (d_to_i < min_dist_to_intersect):
                min_dist_to_intersect = d_to_i
            t1 = t - dt
            dist = (traj.x (t) - traj.x (t1)).norm2()
            if (dist > max_dist_between_points):
                max_dist_between_points = dist
            print("\n MANGLER   TypeError: unsupported operand type(s) for *: 'float' and 'Vector2D'       DET VIRKER IKKE")
#            acc_pos = acc_pos + acc_vel * dt + 1 / 2.0 * traj.ddx (t) * dt * dt
#            acc_vel = acc_vel + traj.ddx (t1) * dt
#            vel_pos = vel_pos + traj.dx (t1) * dt

        self.assertLess(min_dist_to_intersect, 0.42)
        self.assertLess(max_dist_between_points, 0.0051)
#        self.assertLess((end - vel_pos).norm2 (), 0.004)
#        self.assertLess((end - acc_pos).norm2 (), 0.03)


if __name__ == '__main__':
    unittest.main()