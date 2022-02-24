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
import sdurw_math
import sdurw_trajectory as trajectory


# The range function does not work with floats. Only integer values can be specified as the start, stop, and step arguments.
# This function should rander the problem.
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step


def isContinues(tester, traj, stepSize):
    t=traj.startTime()
    last_x   = traj.x(t);
    last_dx  = traj.dx(t);
    last_ddx = traj.ddx(t);

    for t in range_with_floats(0 , traj.endTime(), traj.duration()*stepSize):
        tester.assertLess( sdurw_math.MetricUtil.dist2(last_x,traj.x(t)), 0.15 )
        tester.assertLess( sdurw_math.MetricUtil.dist2(last_dx,traj.dx(t)), 0.15)

        last_x = traj.x(t)
        last_dx = traj.dx(t)
        last_ddx = traj.ddx(t)

    return True


class Trajectory(unittest.TestCase):


    def test_PrintInformation(self):
        # This is used to make debugging in gitlab CI easier
        print("\n Testing /RobWork/RobWork/unittest/trajectory/Trajectory.py")


    def isContinues(self, traj, stepSize):
        t=traj.startTime()
        last_x   = traj.x(t)
        last_dx  = traj.dx(t)
        last_ddx = traj.ddx(t)
        for t in range(t, traj.endTime(), traj.duration()*stepSize):
            self.assertLess( sdurw_math.MetricUtil.dist2(last_x,traj.x(t)), 0.15)
            self.assertLess( sdurw_math.MetricUtil.dist2(last_dx,traj.dx(t)), 0.15)

            last_x = traj.x(t)
            last_dx = traj.dx(t)
            last_ddx = traj.ddx(t)

        return True


    def test_RampInterpolatorTest(self):
        n = 5
        dqlimit = sdurw_math.Q(n)
        ddqlimit= sdurw_math.Q(n)
        for i in range(0, n, 1):
            dqlimit[i] = 2
            ddqlimit[i] = 4

        q1 = sdurw_math.Q(n)
        q2 = sdurw_math.Q(n)

        ramp = trajectory.RampInterpolatorQ(q1, q2, dqlimit, ddqlimit)

        stepSize = ramp.duration()/100.0
        for i in range(0, 100, 1):
            t = stepSize*i
            q = ramp.x(t)
        
        start = sdurw_math.RPY(0.2, 0, 0)
        end = sdurw_math.RPY(0.2, 0.6, 0.9)

        print("\n MANGLER   TypeError: Wrong number or type of arguments for overloaded function 'new_RampInterpolatorRotation3D'        DET VIRKER IKKE")
        # TO DO Problems with swig
#       rampR = trajectory.RampInterpolatorRotation3D(start.toRotation3D(), end.toRotation3D(), 0.8, 0.1)
#
#        stepSize = rampR.duration()/100.0
#        for i in range(0, 100, 1):
#            t = stepSize*i
#            rot = sdurw_math.Rotation3D(rampR.x(t))
#            eaa = sdurw_math.EAA(rot)
#            axis = eaa.axis()
#            angle = eaa.angle()
#            print("\n ", t, ";", angle, ";", axis(0), ";", axis(1), ";", axis(2))

        startP = sdurw_math.Vector3D(0 ,0 , 0)
        endP   = sdurw_math.Vector3D(0, 0, 0.04)
        print("\n MANGLER   TypeError: Wrong number or type of arguments for overloaded function 'new_RampInterpolatorVector3D'        DET VIRKER IKKE")
        # TO DO Problems with swig
#        rampP = trajectory.RampInterpolatorVector3D(startP, endP, 0.05, 0.005)

#        stepSize = rampP.duration()/100.0
#        for i in range(0, 100, 1):
#            t = stepSize*i
#            p = rampP.x(t)
#            print("\n ", t, ";", p(0), ";", p(1), ";", p(2))
#        print("\n ","duration: ", rampP.duration() )
#        print("\n ", rampP.x(0))
#        print("\n ", rampP.x(rampP.duration()))


    def test_IteratorTest(self):
        traj = trajectory.InterpolatorTrajectoryQ() 
        q1 = sdurw_math.Q(1)
        q2 = sdurw_math.Q(1)
        q3 = sdurw_math.Q(1)
        q4 = sdurw_math.Q(1)
        q1[0]  = 1
        q2[0] = -1
        q3[0] = 2
        q4[0] = -2
        int1 = trajectory.LinearInterpolatorQ(q1, q2, 1)
        int2 = trajectory.LinearInterpolatorQ(q2, q3, 1)
        int3 = trajectory.LinearInterpolatorQ(q3, q4, 1)

        print("\n MANGLER   TypeError: Wrong number or type of arguments for overloaded function 'InterpolatorTrajectoryQ_add'        DET VIRKER IKKE")
#        traj.add(int1)
#        traj.add(int2)
#        traj.add(int3)

        pIterator = traj.getIterator(0.01)
        iterator = pIterator

        print("\n HVORDAN skal dette implementeres ? ")
#        for t in range_with_floats(0, 1, 0.01):
#            self.assertEqual(iterator.x() , int1.x(t))
#            iterator = iterator+1
        
#        iterator = iterator+1

        # This loop will never run, nor will the c++ version
        for t in range_with_floats(0, 0, -0.01):
            print("\n t", t )
#            self.assertEqual(iterator.x() , int2.x(t))
#            iterator = iterator-1

#        iterator = iterator+1
#        for t in range_with_floats(iterator.getTime()-2, 1, 0.1):
#            self.assertEqual(iterator.x() , int3.x(t))
#            iterator = iterator+0.1


    def test_TrajectorySequenceTest(self):
        trajectory1 = trajectory.ownedPtr(trajectory.InterpolatorTrajectoryQ())
        trajectory2 = trajectory.ownedPtr(trajectory.InterpolatorTrajectoryQ())
        trajectory3 = trajectory.ownedPtr(trajectory.InterpolatorTrajectoryQ())
        q1 = sdurw_math.Q(2, 0, 0)
        q2 = sdurw_math.Q(2, 1, 1)
        q3 = sdurw_math.Q(2, -1, 2)
        q4 = sdurw_math.Q(2, 1, 3)

        int1 = trajectory.LinearInterpolatorQ(q1, q2, 1)
        int2 = trajectory.LinearInterpolatorQ(q2, q3, 1)
        int3 = trajectory.LinearInterpolatorQ(q3, q4, 1)

        trajectories = trajectory.VectorTrajectoryQPtr()
        print("\n MANGLER   TypeError: in method 'VectorTrajectoryQPtr_push_back', argument 2 of type 'std::vector< rw::core::Ptr< rw::trajectory::Trajectory< rw::math::Q > > >::value_type const &'        DET VIRKER IKKE")
#        trajectories.push_back(trajectory1)
#        trajectories.push_back(trajectory2)
#        trajectories.push_back(trajectory3)
        print("\n MANGLER   AttributeError: No constructor defined - class is abstract        DET VIRKER IKKE")
#        seq = trajectory.TrajectorySequenceQ(trajectories)
#         self.assertEqual(seq.x(0) , trajectory1.x(0))
#         self.assertEqual(seq.x(0.5) , trajectory1.x(0.5))
#         self.assertEqual(seq.x(1) , trajectory1.x(1))
#         self.assertEqual(seq.x(1) , trajectory2.x(0))
#         self.assertEqual(seq.x(1.5) , trajectory2.x(0.5))
#         self.assertEqual(seq.x(2.5) , trajectory2.x(1.5))
#         self.assertEqual(seq.x(3) , trajectory3.x(0))
#         self.assertEqual(seq.x(4) , trajectory3.x(1))
#         self.assertEqual(seq.x(6) , trajectory3.x(3))

#         dt = 0.1
#         pIterator = seq.getIterator(dt)
#         iterator = pIterator
#         self.assertEqual(iterator.x() , seq.x(0))
#         iterator = iterator + 1
#         self.assertEqual(iterator.x() , seq.x(dt))
#         iterator = iterator + 1
#         self.assertEqual(iterator.x() , seq.x(1+dt))
#         iterator = iterator - 1
#         self.assertEqual(iterator.x() , seq.x(1))


    def test_CubicSplineInterpolation(self):
        # check throw on empty QPath
        path = trajectory.PathQ()
        with self.assertRaises(Exception):
            error = trajectory.CubicSplineFactory.makeNaturalSpline(path)
        path.append(sdurw_math.Q(3))
        with self.assertRaises(Exception):
            error = trajectory.CubicSplineFactory.makeNaturalSpline(path)
        
        # here we test the cubic path interpolation
        path = trajectory.PathQ()
        q = sdurw_math.Q.zero(7)
        q[0] = 1
        path.append(q)
        q[0] = 8
        path.append(q)
        q[0] = -1
        path.append(q)
        q[0] = 4
        path.append(q)

        traj = trajectory.CubicSplineFactory.makeNaturalSpline(path)
        self.assertFalse(traj.isNull())
        self.assertAlmostEqual(traj.duration(), path.size()-1, delta = 0.00001)

        self.assertTrue(isContinues(self, traj, traj.duration()*0.0001))

        start = sdurw_math.Q.zero(7)
        end = sdurw_math.Q.zero(7)
        start[0] = 1
        end[0] = -1

        traj = trajectory.CubicSplineFactory.makeClampedSpline(path, start, end)

        self.assertFalse(traj.isNull())
        self.assertAlmostEqual(traj.duration(), path.size() - 1, delta=0.0001)

        self.assertTrue(isContinues(self, traj, traj.duration()*0.0001))

        self.assertAlmostEqual(traj.dx(0)[0], 1, delta=0.001)
        self.assertAlmostEqual(traj.dx( traj.duration() )[0], -1, delta=0.001)

        # here we test the cubic path interpolation

        path = trajectory.ownedPtr(trajectory.PathTimedQ())
        q = sdurw_math.Q.zero(7)
        q[0] = 1
        path.append(trajectory.TimedQ(0, q))
        q[0] = 8
        path.append(trajectory.TimedQ(1, q))
        q[0] = -1
        path.append(trajectory.TimedQ(4, q))
        q[0] = 4
        path.append(trajectory.TimedQ(6, q))

        traj = trajectory.CubicSplineFactory.makeNaturalSpline(path)

        self.assertFalse(traj.isNull())
        self.assertAlmostEqual(traj.duration(), path.back().getTime() - path.front().getTime(), delta=0.0001)
        self.assertTrue(isContinues(self, traj, traj.duration()*0.0001))

        start = sdurw_math.Q.zero(7)
        end = sdurw_math.Q.zero(7)
        start[0] = 1
        end[0] = -1
        traj = trajectory.CubicSplineFactory.makeClampedSpline(path, start, end)

        self.assertFalse(traj.isNull())
        self.assertAlmostEqual(traj.duration(), path.back().getTime() - path.front().getTime(), delta=0.0001)
        self.assertTrue(isContinues(self, traj, traj.duration()*0.0001))


if __name__ == '__main__':
    unittest.main()