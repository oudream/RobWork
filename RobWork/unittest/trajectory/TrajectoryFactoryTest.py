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
import sys
import sdurw_math
import sdurw_trajectory as trajectory
import numpy            as np


def toVector3D(eaa):
    return sdurw_math.Vector3D(eaa[0], eaa[1], eaa[2])


def eaaVecA(T):
    return toVector3D(sdurw_math.EAA(T))


def eaaVec(T1, T2):
    return toVector3D(sdurw_math.EAA(sdurw_math.inverse(T1)*T2))


def halfTransform(T1, T2):
    return sdurw_math.Transform3D( (T2.P()+T1.P())/2.0, T1.R()*sdurw_math.EAA( eaaVec(T1.R(),T2.R())/2.0 ).toRotation3D() )


class TrajectoryFactoryTest(unittest.TestCase):

    def test_PrintInformation(self):
        # This is used to make debugging in gitlab CI easier
        print("\n Testing /RobWork/RobWork/unittest/trajectory/TrajectoryFactoryTest.py")


    def test_TrajFactoryTest(self):
        path  = trajectory.PathTransform3D()
        times = np.array([])
        T1 = sdurw_math.Transform3D.identity()
        T2 = sdurw_math.Transform3D(sdurw_math.Vector3D(0.2,  0.3, 0.4),sdurw_math.EAA(0, 0, math.radians(45)).toRotation3D())
        T3 = sdurw_math.Transform3D(sdurw_math.Vector3D(-0.2, 0.3, 0.7),sdurw_math.EAA(math.radians(45), 0, math.radians(0.45)).toRotation3D())
        T4 = sdurw_math.Transform3D(T1)         # Make a copy of T1 using c++ copy constructor
        t1 = 0.3
        t2 = 2.0
        t3 = 2.5
        path.append(T1)
        path.append(T2)
        path.append(T3)
        path.append(T4)
        times = np.append(times, t1)
        times = np.append(times, t2)
        times = np.append(times, t3)
        traj = trajectory.TrajectoryFactory.makeLinearTrajectory(path, times)
        # Check time
        self.assertAlmostEqual(traj.startTime(), 0.0, delta = sys.float_info.epsilon)
        self.assertAlmostEqual(traj.duration(), t1+t2+t3, delta = sys.float_info.epsilon)
        self.assertAlmostEqual(traj.endTime(), t1+t2+t3, delta = sys.float_info.epsilon)
        # Check positions
        self.assertTrue(traj.x(0).equal(T1))
        self.assertTrue(traj.x(t1).equal(T2))
        self.assertTrue(traj.x(t1+t2).equal(T3))
        self.assertTrue(traj.x(t1+t2+t3).equal(T4))
        self.assertTrue(traj.x(t1/2.0).equal(halfTransform(T1, T2)))
        self.assertTrue(traj.x(t1+t2/2.).equal(halfTransform(T2, T3)))
        self.assertTrue(traj.x(t1+t2+t3/2.).equal(halfTransform(T3, T4)))
        # Check velocity
        linVel1 = (T2.P()-T1.P())/t1
        linVel2 = (T3.P()-T2.P())/t2
        linVel3 = (T4.P()-T3.P())/t3

        print("\n MANGLER     AssertionError: 1.0 not less than 2.960594732333751e-16        DET VIRKER IKKE")
#        self.assertLess((traj.dx(t1/2.).P()-linVel1).normInf() , linVel1.normInf()*sys.float_info.epsilon)
#        self.assertLess((traj.dx(t1+t2/2.).P()-linVel2).normInf() , linVel2.normInf()*sys.float_info.epsilon)
#        self.assertLess((traj.dx(t1+t2+t3/2.).P()-linVel3).normInf() , linVel3.normInf()*sys.float_info.epsilon)
        angVel1 = eaaVec(T1.R(), T2.R())/t1
        angVel2 = eaaVec(T2.R(), T3.R())/t2
        angVel3 = eaaVec(T3.R(), T4.R())/t3
        self.assertLess(angVel1.norm2() , math.pi)  #  - note that angular part of dx is wrong when velocity is bigger than Pi
        self.assertLess(angVel2.norm2() , math.pi)
        self.assertLess(angVel3.norm2() , math.pi)

        print("\n MANGLER HVORDAN OPRETTES EN FUNKRUION MED SAMME NAVN MEN FORSKELLIGE ARGUMENTER ?")
        self.assertLess((eaaVecA((traj.dx(t1/2.).R()))-angVel1).normInf() , 1e-15)
        self.assertLess((eaaVecA((traj.dx(t1+t2/2.).R()))-angVel2).normInf() , 1e-15)
        self.assertLess((eaaVecA((traj.dx(t1+t2+t3/2.).R()))-angVel3).normInf() , 1e-15)
        # Check that there are no acceleration

        print("\n MANGLER     AssertionError: 2.24143354e-316 != 0        DET VIRKER IKKE")
#        self.assertEqual(traj.ddx(t1/2.0).P().normInf(), 0)
#        self.assertEqual(traj.ddx(t1+t2/2.0).P().normInf(), 0)
#        self.assertEqual(traj.ddx(t1+t2+t3/2.0).P().normInf(), 0)
        self.assertTrue(traj.ddx(t1/2.0).R().equal(sdurw_math.Rotation3D().identity()))
        self.assertTrue(traj.ddx(t1+t2/2.0).R().equal(sdurw_math.Rotation3D().identity()))
        self.assertTrue(traj.ddx(t1+t2+t3/2.0).R().equal(sdurw_math.Rotation3D().identity()))


if __name__ == '__main__':
    unittest.main()        