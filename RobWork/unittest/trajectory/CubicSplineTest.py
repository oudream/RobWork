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
import sdurw_math
import sdurw_trajectory as trajectory


# The range function does not work with floats. Only integer values can be specified as the start, stop, and step arguments.
# This function should rander the problem.
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step


def close (q1, q2, diff):
    res = ( math.fabs ((q1).getQx () - (q2).getQx ()) < diff[0] and 
            math.fabs ((q1).getQy () - (q2).getQy ()) < diff[1] and 
            math.fabs ((q1).getQz () - (q2).getQz ()) < diff[2] and 
            math.fabs ((q1).getQw () - (q2).getQw ()) < diff[3] )
    if res == False:
        print("\n ")
        print("\n diff1", math.fabs ((q1).getQx () - (q2).getQx ()) )
        print("\n diff1", math.fabs ((q1).getQy () - (q2).getQy ()) )
        print("\n diff1", math.fabs ((q1).getQz () - (q2).getQz ()) )
        print("\n diff1", math.fabs ((q1).getQw () - (q2).getQw ()) )
    return res


class CubicSplineTypedTest:

    def __init__(self, type):
        
        self.type = type

        self.angles = [sdurw_math.EAA(22, 0, 0), sdurw_math.EAA(0, -1, 0), sdurw_math.EAA(0, 0, 1), sdurw_math.EAA(0, 2, 0)] 
        self.pos    = [sdurw_math.Vector3D(0, 0.5, 0), sdurw_math.Vector3D(0, 0, 0.5), sdurw_math.Vector3D(0.5, 0, 0), sdurw_math.Vector3D(0, 0, 0)]

        self.typeNum = 0
        if type == sdurw_math.Q:
            self.typeNum = 1
        elif type == sdurw_math.Transform3D:
            self.typeNum = 2        
        elif type == sdurw_math.Transform3DVector:
            self.typeNum = 3
        elif type == sdurw_math.Vector3D:
            self.typeNum = 4
        elif type == sdurw_math.Quaternion:
            self.typeNum = 5
        else:
            raise Exception("An error occurred in defining types")
    
    ############### getPoints ########################
    def getPoint(self, i):
        if self.typeNum == 1:                                               # type Q
            q = sdurw_math.Q.zero(7)
            if i == 0:
                q[0] = 1
                return q
            elif i == 1:
                q[0] = 8
                return q
            elif i == 2:
                q[0] = -1
                return q
            elif i == 3:
                q[0] = 4
                return q
            else:
                return q

        elif self.typeNum == 2:                                             # type Transform3D
            return sdurw_math.Transform3D(self.pos[i], self.angles[i])

        elif self.typeNum == 3:                                             # type Transform3DVector
            return sdurw_math.Transform3DVector(self.pos[i], self.angles[i])

        elif self.typeNum == 4:                                             # type Vector3D
            return sdurw_math.Vector3D(self.pos[i])

        elif self.typeNum == 5:                                             # type Quaternion
            return sdurw_math.Quaternion(self.angles[i])

    ############### start ########################
    def start(self):
        if self.typeNum == 1:                                               # type Q
            return (self.getPoint (0) - self.getPoint (1) / 2)

        elif self.typeNum == 2:                                             # type Transform3D
            return sdurw_math.Transform3D((self.getPoint (0).P () - self.getPoint (1).P () / 2),
                                          (self.getPoint (0).R () * self.getPoint (1).R ().inverse (True)))

        elif self.typeNum == 3:                                             # type Transform3DVector
            return (self.getPoint (0) - self.getPoint (1) / 2)

        elif self.typeNum == 4:                                             # type Vector3D
            return (self.getPoint (0) - self.getPoint (1) / 2)

        elif self.typeNum == 5:                                             # type Quaternion
            return (self.getPoint (0) - self.getPoint (1)).elemDivide (2)

    ############### end ########################
    def end(self):
        if self.typeNum == 1:                                               # type Q
            return (self.getPoint (2) - self.getPoint (3) / 2)

        elif self.typeNum == 2:                                             # type Transform3D
            return sdurw_math.Transform3D((self.getPoint (2).P () - self.getPoint (3).P () / 2),
                                          (self.getPoint (2).R () * self.getPoint (3).R ().inverse (True)))

        elif self.typeNum == 3:                                             # type Transform3DVector
            return (self.getPoint (2) - self.getPoint (3) / 2)

        elif self.typeNum == 4:                                             # type Vector3D
            return (self.getPoint (2) - self.getPoint (3) / 2)

        elif self.typeNum == 5:                                             # type Quaternion
            return (self.getPoint (2) - self.getPoint (3)).elemDivide (2)

    ############## type #########################
    def type(self):
        if self.typeNum == 1:                                               # type Q
            return 0

        elif self.typeNum == 2:                                             # type Transform3D
            return 1

        elif self.typeNum == 3:                                             # type Transform3DVector
            return 2

        elif self.typeNum == 4:                                             # type Vector3D
            return 3

        elif self.typeNum == 5:                                             # type Quaternion
            return 4

    ############### isContinues ####################
    def isContinues(self, tester, traj):
         value = traj.x(traj.startTime())
         if isinstance(value, sdurw_math.Q):                                                        # type Q
            start      = traj.startTime ()
            acc_vel    = traj.dx (start)
            acc_pos    = traj.x (start)
            vel_pos    = traj.x (start)

            last_x     = traj.x (start)
            last_dx    = traj.dx (start)
            last_ddx   = traj.ddx (start)

            dt         = 0.0005
            for t in range_with_floats(start + dt, traj.endTime (), dt):
                t1 = t - dt
                acc_pos += acc_vel * dt + traj.ddx (t) * (1.0 / 2.0) * dt * dt
                acc_vel += traj.ddx (t1) * dt
                vel_pos += traj.dx (t1) * dt
                
                tester.assertLess(sdurw_math.MetricUtil.dist2 (acc_pos, traj.x (t)), 0.041)
                tester.assertLess(sdurw_math.MetricUtil.dist2 (vel_pos, traj.x (t)), 0.004)

                tester.assertLess(sdurw_math.MetricUtil.dist2 (last_x, traj.x (t)), 0.007)
                tester.assertLess(sdurw_math.MetricUtil.dist2 (last_dx, traj.dx (t)), 0.03)
                tester.assertLess(sdurw_math.MetricUtil.dist2 (last_ddx, traj.ddx (t)), 0.06)

                last_x   = traj.x (t)
                last_dx  = traj.dx (t)
                last_ddx = traj.ddx (t)

            return True

         elif isinstance(value, sdurw_math.Transform3D):                                            # type Transform3D
            start   = traj.startTime ()
            last_px = traj.x (start).P ()

            dt = 0.001
            for t in range_with_floats(start + dt, traj.endTime (), dt):
                tester.assertLess(sdurw_math.MetricUtil.dist2 (last_px , traj.x (t).P ()), 0.003)
                last_px = traj.x (t).P ()

            last_rx = traj.x (start).R ()

            dt = 0.0005
            for t in range_with_floats(start + dt, traj.endTime (), dt):
                tester.assertTrue( close (last_rx, sdurw_math.Quaternion (traj.x (t).R ()),[0.002, 0.0017, 0.001, 0.003]) )
                last_rx = traj.x (t).R ()

            return True

         elif isinstance(value,sdurw_math.Transform3DVector):                                      # type Transform3DVector
            start       = traj.startTime ()
            last_px = traj.x (start).toVector3D ()

            dt = 0.001
            for t in range_with_floats(start + dt, traj.endTime (), dt):
                tester.assertLess(sdurw_math.MetricUtil.dist2 (last_px , traj.x (t).toVector3D ()), 0.003)
                last_px = traj.x (t).toVector3D ()

            last_rx = traj.x (start).toQuaternion ()

            dt = 0.0005
            for t in range_with_floats(start + dt, traj.endTime (), dt):
                tester.assertTrue( close (last_rx, traj.x (t).toQuaternion (),[0.002, 0.0017, 0.001, 0.003]) )
                last_rx = traj.x (t).toQuaternion ()
            
            return True

         elif isinstance(value, sdurw_math.Vector3D):                                               # type Vector3D
            start    = traj.startTime ()
            acc_vel  = traj.dx (start)
            acc_pos  = traj.x (start)
            vel_pos  = traj.x (start)

            last_x   = traj.x (start)
            last_dx  = traj.dx (start)
            last_ddx = traj.ddx (start)

            dt = 0.001
            for t in range_with_floats(start + dt, traj.endTime (), dt):
                t1 = t - dt

                acc_pos += acc_vel * dt + traj.ddx (t) * (1.0 / 2.0) * dt * dt
                acc_vel += traj.ddx (t1) * dt
                vel_pos += traj.dx (t1) * dt

                tester.assertLess(sdurw_math.MetricUtil.dist2 (acc_pos, traj.x (t)), 0.01)
                tester.assertLess(sdurw_math.MetricUtil.dist2 (vel_pos, traj.x (t)), 0.005)

                tester.assertLess(sdurw_math.MetricUtil.dist2 (last_x, traj.x (t)), 0.003)
                tester.assertLess(sdurw_math.MetricUtil.dist2 (last_dx, traj.dx (t)), 0.006)
                tester.assertLess(sdurw_math.MetricUtil.dist2 (last_ddx, traj.ddx (t)), 0.01)

                last_x   = traj.x (t)
                last_dx  = traj.dx (t)
                last_ddx = traj.ddx (t)

            return True

         elif isinstance(value,sdurw_math.Quaternion):                                             # type Quaternion
            start  = traj.startTime ()
            last_x = traj.x (start)

            dt = 0.0005
            for t in range_with_floats(start + dt, traj.endTime (), dt):
                tester.assertTrue( close (last_x, traj.x (t),[0.002, 0.0017, 0.001, 0.003]) )
                last_x = traj.x (t)

            return True


def createPath(type):
    print("Temporary Fix for creating paths")
    if type == sdurw_math.Q:
        return trajectory.PathQ()
    elif type == sdurw_math.Transform3D:
        return trajectory.PathTransform3D()  
    elif type == sdurw_math.Quaternion:
        return trajectory.PathQuaternion()
    elif type == sdurw_math.Vector3D:
        return trajectory.PathVector3D()
    elif type == sdurw_math.Transform3DVector:
        return trajectory.PathTransform3DVector()
    else:
        raise Exception("An error occurred in defining types")


class CubicSplineTest(unittest.TestCase):


    def test_PrintInformation(self):
        # This is used to make debugging in gitlab CI easier
        print("\n Testing /RobWork/RobWork/unittest/trajectory/CubicSplineTest.py")


    def test_PathToShort(self):
        # check throw on empty QPath
        path = trajectory.PathQ()
        with self.assertRaises(Exception):
            error = trajectory.CubicSplineFactory.makeNaturalSpline(path)

        path.append(sdurw_math.Q(3))
        with self.assertRaises(Exception):
            error = trajectory.CubicSplineFactory.makeNaturalSpline(path)


    def test_PathToShort_QPtr(self):
        # check throw on empty QPath
        path = trajectory.PathQ()
        with self.assertRaises(Exception):
            error = trajectory.CubicSplineFactory.makeNaturalSpline(path)

        path.append(sdurw_math.Q(3))
        with self.assertRaises(Exception):
            error = trajectory.CubicSplineFactory.makeNaturalSpline(path)


    def test_PathTest(self):
        for type in (sdurw_math.Q, sdurw_math.Transform3D, sdurw_math.Quaternion, sdurw_math.Vector3D, sdurw_math.Transform3DVector):
            this = CubicSplineTypedTest(type)

            path = createPath(type)
            path.push_back(this.getPoint(0))
            path.push_back(this.getPoint(1))
            path.push_back(this.getPoint(2))
            path.push_back(this.getPoint(3))

            traj = trajectory.CubicSplineFactory.makeNaturalSpline(path)

            self.assertTrue(traj != None)
            self.assertAlmostEqual(traj.duration(), path.size() - 1, delta=0.0001)

            self.assertTrue(this.isContinues (self, traj))


    def test_PathTest_QPtr(self):
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
        self.assertTrue(traj != None)
        self.assertAlmostEqual(traj.duration(), path.size() - 1, delta=0.0001)

        this = CubicSplineTypedTest(sdurw_math.Q)
        self.assertTrue(this.isContinues (self, traj))

        start = sdurw_math.Q.zero(7)
        end   = sdurw_math.Q.zero(7)
        start[0] = 1
        end[0] = -1
        traj = trajectory.CubicSplineFactory.makeClampedSpline(path, start, end)

        self.assertTrue(traj != None)
        self.assertAlmostEqual(traj.duration(), path.size() - 1, delta=0.0001)

        this = CubicSplineTypedTest(sdurw_math.Q)
        self.assertTrue(this.isContinues (self, traj))


if __name__ == '__main__':
    unittest.main()