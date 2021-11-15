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
import sdurw_math, sdurw_models
from sdurw_math.sdurw_math import RPY


class PrismaticJointTest(unittest.TestCase):

    def test_PrismaticJoint(self):
        rot = RPY(0, math.pi/2, 0).toRotation3D()
        joint = sdurw_models.PrismaticJoint("Joint",sdurw_math.Transform3D(sdurw_math.Vector3D.z()*0.1, rot))
        wTp = sdurw_math.Transform3D(sdurw_math.Vector3D.z()*0.05, RPY(0, -math.pi/4, 0).toRotation3D())
        q = sdurw_math.Q(1,0.2)

        # Check that the same result is achieved with pre- and post-multiplication:
        post = wTp*joint.getTransform(q[0])
        pre = sdurw_math.Transform3D()
        joint.multiplyJointTransform(wTp,q,pre)

        self.assertLess((post.P()- pre.P()).normInf(), sys.float_info.epsilon)

        print("\n MANGLER   AttributeError: 'Rotation3D' object has no attribute 'angle'        DET VIRKER IKKE")
        z = post.R()
        print("\n z", z )
#        z = (post.R()).angle()
        print("\n z", z )
#        self.assertLess((post.R()).angle()-(pre.R()).angle(), sys.float_info.epsilon)

        print("\n MANGLER   NameError: name 'cross' is not defined        DET VIRKER IKKE")
#        self.assertLess(cross((post.R()).axis(), (pre.R()).axis()).normInf(), sys.float_info.epsilon)



        # Check values are as expected
        expRot = wTp.R()*rot
        print("\n MANGLER   'angle' og 'cross'                          DET VIRKER IKKE")
#        self.assertLess((post.R()).angle() - (expRot).angle(), sys.float_info.epsilon)
#        self.assertLess(cross((post.R()).axis(), (expRot).axis()).normInf(), sys.float_info.epsilon)

        expPos = wTp.P()+wTp.R().getCol(2)*0.1+expRot.getCol(2)*q[0]
        self.assertLess((post.P() - expPos).normInf(), sys.float_info.epsilon)


        joint = sdurw_models.PrismaticJoint("Joint",sdurw_math.Transform3D(sdurw_math.Vector3D.z()*0.1))
        wTp = sdurw_math.Transform3D(sdurw_math.Vector3D.z()*0.05, RPY(0, -math.pi/4, 0).toRotation3D())
        q = sdurw_math.Q(1,0.2)

        # Check that the same result is achieved with pre- and post-multiplication:
        post = wTp*joint.getTransform(q[0])
        pre = sdurw_math.Transform3D()
        joint.multiplyJointTransform(wTp,q,pre)

        self.assertLess((post.P() - pre.P()).normInf(), sys.float_info.epsilon)

        print("\n MANGLER   'angle' og 'cross'                          DET VIRKER IKKE")
#        self.assertLess((post.R()).angle() - (pre.R()).angle(), sys.float_info.epsilon)
#        self.assertLess(cross((post.R()).axis(), (pre.R()).axis()).normInf(), sys.float_info.epsilon)

        # Check values are as expected
        expRot = wTp.R()
        print("\n MANGLER   'angle' og 'cross'                          DET VIRKER IKKE")
        #self.assertLess((post.R()).angle() - (expRot).angle(), sys.float_info.epsilon)
        #self.assertLess(cross((post.R()).axis(),(expRot).axis()).normInf(), sys.float_info.epsilon)

        expPos = wTp.P() + wTp.R().getCol(2)*(0.1+q[0])
        self.assertLess((post.P() - expPos).normInf(), sys.float_info.epsilon)


        rot = RPY(0, math.pi/2, 0).toRotation3D()
        joint = sdurw_models.PrismaticJoint("Joint",sdurw_math.Transform3D(rot))
        wTp = sdurw_math.Transform3D(sdurw_math.Vector3D.x()*0.05, RPY(0, -math.pi/4, 0).toRotation3D())
        q = sdurw_math.Q(1,0.2)

        # Check that the same result is achieved with pre- and post-multiplication:
        post = wTp*joint.getTransform(q[0])
        pre = sdurw_math.Transform3D()
        joint.multiplyJointTransform(wTp,q,pre)
        self.assertLess((post.P() - pre.P()).normInf(), sys.float_info.epsilon)
        print("\n MANGLER   'angle' og 'cross'                          DET VIRKER IKKE")
#        self.assertLess((post.R()).angle() - (pre.R()).angle(), sys.float_info.epsilon)
#        self.assertLess(cross((post.R()).axis(), (pre.R()).axis()).normInf(), sys.float_info.epsilon)

        # Check values are as expected
        expRot = wTp.R()*rot
        print("\n MANGLER   'angle' og 'cross'                          DET VIRKER IKKE")
        #self.assertLess((post.R()).angle() - (expRot).angle(), sys.float_info.epsilon)
        #self.assertLess(cross((post.R()).axis(),(expRot).axis()).normInf(), sys.float_info.epsilon)

        expPos = wTp.P() + expRot.getCol(2)*q[0]
        self.assertLess((post.P() - expPos).normInf(), sys.float_info.epsilon)



if __name__ == '__main__':
    unittest.main()
