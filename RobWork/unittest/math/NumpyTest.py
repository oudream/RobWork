#################################################################################
 # Copyright 2021 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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


#from sdurw import *
import unittest                                     # now we can use python unittest framework
import math, sys
import sdurw, sdurw_math
import numpy as np
from sdurw_math.sdurw_math import Vector3D


class UtilTest(unittest.TestCase):

    def test_Util(self):
        q = sdurw_math.Quaternion(0.1, 0.2, 0.3, 0.4)
        q.normalize()
        eaa = sdurw_math.EAA(1,2,3)

        eaa_q = sdurw_math.Math.quaternionToEAA(q)
        q_eaa = sdurw_math.Math.eaaToQuaternion(eaa_q)

        self.assertAlmostEqual(q[0],q_eaa[0], delta = 1e-12)
        self.assertAlmostEqual(q[1],q_eaa[1], delta = 1e-12)
        self.assertAlmostEqual(q[2],q_eaa[2], delta = 1e-12)
        self.assertAlmostEqual(q[3],q_eaa[3], delta = 1e-12)


    def test_Vector3D_cross(self):
        v1 = sdurw_math.Vector3D(1.0, 2.0, 2.0)
        v2 = sdurw_math.Vector3D(v1)                      # Make a copy of v1 using c++ copy constructor

        self.assertEqual(sdurw_math.MetricUtil.normInf(sdurw_math.cross(v1, v2)), 0)

    def test_Transform3DAngleMetric(self):
        v1 = sdurw_math.Vector3D(1.0, 2.0, 2.0)
        t1 = sdurw_math.Transform3D(sdurw_math.Vector3D(1, 0, 0), sdurw_math.Rotation3D.identity())
        t2 = sdurw_math.Transform3D(sdurw_math.Vector3D(1, 0, 0), sdurw_math.RPY(1.4, 0, 0).toRotation3D())

        x1 = sdurw_math.Transform3DAngleMetric(1.0, 0.0).distance(t1, t2)
        x2 = sdurw_math.Transform3DAngleMetric(0.0, 1.0).distance(t1, t2)
        self.assertAlmostEqual(x1, 0, delta = 1e-15)
        self.assertAlmostEqual(x2, 1.4, delta = 1e-15)

    def test_Wrench6D(self):
        wrench = sdurw_math.Wrench6D(1,2,3,4,5,6)
        self.assertEqual(wrench.force()[0] , 1)
        self.assertEqual(wrench.force()[1] , 2)
        self.assertEqual(wrench.force()[2] , 3)
        self.assertEqual(wrench.torque()[0] , 4)
        self.assertEqual(wrench.torque()[1] , 5)
        self.assertEqual(wrench.torque()[2] , 6)


    def test_Vector3D_Conversion(self):
        #Test conversion, if it contains "Swig Object" then it is a failure
        obj1 = sdurw_math.Vector3D()
        s = str(obj1)
        self.assertNotIn("Swig Object", s)

        obj1f = sdurw_math.Vector3Df()
        s = str(obj1f)
        self.assertNotIn("Swig Object", s)


        # some extra information
        print("NumPy Version: ", end = '')
        print(np.__version__)
       
        arr = np.array([1, 2, 3, 4], ndmin=5)
        print(arr)
        print('number of dimensions :', arr.ndim)   



if __name__ == '__main__':
    unittest.main()    
        