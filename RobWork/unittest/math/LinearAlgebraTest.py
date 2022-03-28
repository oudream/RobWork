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


import unittest                                     # now we can use python unittest framework
import math, sys
import sdurw_math
import numpy as np


class LinAlgeTest(unittest.TestCase):

    def test_LinearAlgebraTest(self):
        eaa = sdurw_math.EAA(sdurw_math.Vector3D(1.0, 0.0, 0.0), math.pi/4.0)
        r = eaa.toRotation3D ()
        minv =  np.linalg.inv(r.asNumpy())
        self.assertLess(  np.linalg.norm(( np.linalg.inv(r.asNumpy()) - minv) ), 1e-10)

        minv =  np.linalg.inv(r.asNumpy())
        self.assertLess(  np.linalg.norm(( np.linalg.inv(r.asNumpy()) - minv) ), 1e-10)

        A = np.zeros((4,4))


        # MatrixXd A = MatrixXd::Zero(4,4);
        A[0][0] = 1
        A[1][1] = 2
        A[2][2] = 3
        A[3][3] = 4
        A[3][0] = 1
        A[0][3] = 1

        val1 = sdurw_math.LinearAlgebra.eigenDecompositionSymmetric(A)


    # MANGLER
    print("\n Resten MANGLER, MANGLER: ")


if __name__ == '__main__':
    unittest.main()        
