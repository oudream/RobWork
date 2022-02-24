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
import sdurw_math       as math
import sdurw_trajectory as trajectory
import numpy            as np


# The range function does not work with floats. Only integer values can be specified as the start, stop, and step arguments.
# This function should rander the problem.
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step


def close(lhs, rhs, error = 1e-14):
    for x in range(0, rhs.asNumpy().shape[0], 1):
        for y in range(0, rhs.asNumpy().shape[1], 1):
            if (abs (lhs.asNumpy () [x, y] - rhs.asNumpy () [x, y]) > error):
                print("\n error: ", lhs.asNumpy () [x, y] - rhs.asNumpy () [x, y] )
                print("\n point: ", x , ",", y, " is ", lhs.asNumpy () [x, y], " = ", rhs.asNumpy () [x, y])
                return False
    return True


def add(position, displacement, dt = 1.0):
    if isinstance(position, math.Vector3D) and isinstance(displacement, math.Vector3D):
        return position + displacement * dt
    if isinstance(position, math.Rotation3D) and isinstance(displacement, math.Rotation3D):
        return position * math.EAA(displacement).scaleAngle(dt).toRotation3D()
    if isinstance(position, math.Transform3D) and isinstance(displacement, math.Transform3D):
        res = math.Transform3D(math.Vector3D  ( position.P () + displacement.P () * dt ), 
              math.Rotation3D( position.R () * math.EAA(displacement.R()).scaleAngle(dt)) )
        return res


class LinearInterpolatorTypedTest :

    def __init__(self, type):
        
        self.type  = type
        self.start = type
        self.duration = 7.66

        self.typeNum = 0
        if type == math.Vector3D:
            self.typeNum = 1
        elif type == math.Rotation3D:
            self.typeNum = 2        
        elif type == math.Transform3D:
            self.typeNum = 3
        else:
            raise Exception("An error occurred in defining types")


    ############### end ########################
    def end(self):
        if self.typeNum == 1:                                               # type Vector3D
            return math.Math_ranTransform3D(3).P ()
        elif self.typeNum == 2:                                             # type Rotation3D
            return math.Math_ranTransform3D(3).R ()
        elif self.typeNum == 3:                                             # type Transform3D
            return math.Math_ranTransform3D(3)


class LinearInterpolatorTest(unittest.TestCase):


    def test_PrintInformation(self):
        # This is used to make debugging in gitlab CI easier
        print("\n Testing /RobWork/RobWork/unittest/trajectory/LinearInterpolatorTest.py")


    def test_Constructor(self):
        for type in (math.Vector3D, math.Rotation3D, math.Transform3D):
            this = LinearInterpolatorTypedTest(type)

            s  = this.start ()
            e  = this.end ()
            d  = this.duration

            if type ==  math.Vector3D:
                li = trajectory.LinearInterpolatorVector3D(s, e, d)
            elif type == math.Rotation3D:
                li = trajectory.LinearInterpolatorRotation3D(s, e, d)
            elif type == math.Transform3D:
                li = trajectory.LinearInterpolatorTransform3D(s, e, d)
            else:
                raise Exception("An error occurred in constructing types")

        self.assertEqual(li.getStart (), s)
        self.assertTrue(close (li.getEnd (), e))
        self.assertEqual(li.duration (), d)

        # Test Exception
        if type ==  math.Vector3D:
            with self.assertRaises(Exception):
                error = trajectory.LinearInterpolatorVector3D(s, e, 0.0)
                error = trajectory.LinearInterpolatorVector3D(s, e, -1)
        elif type == math.Rotation3D:
            with self.assertRaises(Exception):
                error = trajectory.LinearInterpolatorRotation3D(s, e, 0.0)
                error = trajectory.LinearInterpolatorRotation3D(s, e, -1)
        elif type == math.Transform3D:
            with self.assertRaises(Exception):
                error = trajectory.LinearInterpolatorTransform3D(s, e, 0.0)
                error = trajectory.LinearInterpolatorTransform3D(s, e, -1)


    def test_InterpolationSetup(self):
        for type in (math.Vector3D, math.Rotation3D, math.Transform3D):
            this = LinearInterpolatorTypedTest(type)

            s  = this.start ()
            e  = this.end ()
            d  = this.duration

            if type ==  math.Vector3D:
                li = trajectory.LinearInterpolatorVector3D(s, e, d)
            elif type == math.Rotation3D:
                li = trajectory.LinearInterpolatorRotation3D(s, e, d)
            elif type == math.Transform3D:
                li = trajectory.LinearInterpolatorTransform3D(s, e, d)
            else:
                raise Exception("An error occurred in constructing types")
            
            self.assertTrue(close (li.x (0), s))
            self.assertTrue(close (li.x (d), e))

            sv = li.dx (0)

            tt = li.duration () / 10
            for t in range_with_floats(0, li.duration (), tt):
                self.assertEqual(li.dx (t), sv)

            sa = li.ddx (0)
            self.assertTrue(close (sa, type()))

            tt = li.duration () / 10
            for t in range_with_floats(0, li.duration (), tt):
                self.assertEqual(li.ddx (t), sa)


    def test_isLinear(self):
        for type in (math.Vector3D, math.Rotation3D, math.Transform3D):
            this = LinearInterpolatorTypedTest(type)

            s  = this.start ()
            e  = this.end ()
            d  = 10

            if type ==  math.Vector3D:
                li = trajectory.LinearInterpolatorVector3D(s, e, d)
            elif type == math.Rotation3D:
                li = trajectory.LinearInterpolatorRotation3D(s, e, d)
            elif type == math.Transform3D:
                li = trajectory.LinearInterpolatorTransform3D(s, e, d)
            else:
                raise Exception("An error occurred in constructing types")
            
            pos = li.x (0)
            
            dt = 0.5
            for t in range_with_floats(0, d, dt):
                pos = add (pos, li.dx (t), dt)

            self.assertTrue(close (pos, li.x (d), 2))
        

if __name__ == '__main__':
    unittest.main()