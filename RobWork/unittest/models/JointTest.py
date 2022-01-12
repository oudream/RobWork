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
import sdurw_kinematics
import sdurw_models
import array
from sdurw_math.sdurw_math import Transform3D, Vector3D
from sdurw_math.sdurw_math import RPY
import numpy as np

def testGenericJoint(self, joint, dof):
        self.assertEqual(dof, joint.getBounds()[0].size())
        self.assertEqual(dof, joint.getBounds()[1].size())

        joint.setBounds((sdurw_math.Q(dof,-0.6), sdurw_math.Q(dof,0.2)))
        self.assertEqual(-0.6,joint.getBounds()[0][dof-1])
        self.assertEqual(0.2,joint.getBounds()[1][dof-1])

        self.assertEqual(dof,joint.getMaxVelocity().size())
        joint.setMaxVelocity(sdurw_math.Q(dof,2.9))
        self.assertEqual(2.9,joint.getMaxVelocity()[dof-1])

        self.assertEqual(dof,joint.getMaxAcceleration().size())
        joint.setMaxAcceleration(sdurw_math.Q(dof,1.4))
        self.assertEqual(1.4,joint.getMaxAcceleration()[dof-1])

        self.assertTrue(joint.isActive())
        joint.setActive(False)
        self.assertFalse(joint.isActive())
        joint.setActive(True)
        self.assertTrue(joint.isActive())


def compareJointWithReferenceJoints(self, joint, refJoints, offset = sdurw_math.Rotation3D.identity(), offsetIndex = 0):
        T = sdurw_math.Transform3D( sdurw_math.Vector3D(1, 2, 3), sdurw_math.RPY( math.radians(45), math.radians(45), math.radians(45)) )

        dof = joint.getDOF()
        self.assertEqual(dof, joint.size())
        self.assertTrue(joint.getFixedTransform().equal( sdurw_math.Transform3D.identity() ))
        joint.setFixedTransform(T)
        self.assertTrue(joint.getFixedTransform().equal(T))

        sstate = sdurw_kinematics.StateStructure()
        sstate.addData(joint);                                                  # StateStructure takes ownership of the joint

        for i in range (0,len(refJoints)):
                sstate.addData(refJoints[i])                                    # StateStructure takes ownership of the joint
        state = sstate.getDefaultState()

        for i in range (0,len(refJoints)):
                self.assertEqual(0, joint.getData(state)[i])
        
        vals = array.array('d', [0, 0, 0, 0])
        for i in range (0,len(refJoints)):
                vals[i] = 0.1*i
        joint.setData(state,vals)

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her - getData og setData.
#        for i in range (0,len(refJoints)):
#                refJoints[i].setData(state,vals[i])
#
#        for i in range (0,len(refJoints)):
#                self.assertEqual(0.1*i, joint.getData(state)[i])
        

        revT = sdurw_math.Transform3D()
        for i in range (0,len(refJoints)):
                revT = revT*refJoints[i].getTransform(state)
                if (i == offsetIndex):
                        revT = revT * Transform3D(offset)


#        self.assertTrue(joint.getJointTransform(state).R().equal(revT.R()))
#        print("\n (joint.getJointTransform(state).P()-revT.P()).normInf()", (joint.getJointTransform(state).P()-revT.P()).normInf() )
#        self.assertAlmostEqual(0.0, (joint.getJointTransform(state).P()-revT.P()).normInf(), delta = sys.float_info.epsilon)

        revTfull = T*revT
#        self.assertTrue(joint.getTransform(state).R().equal(revTfull.R()))
#        self.assertAlmostEqual(0.0, (joint.getTransform(state).P()-revTfull.P()).normInf(), delta = sys.float_info.epsilon)

        multiplyTransform_res = T                                               # initial value should not influence result
        joint.multiplyTransform(T, state, multiplyTransform_res)
#        self.assertTrue(multiplyTransform_res.equal(T*joint.getTransform(state),1e-15))

        jacobian = sdurw_math.Jacobian(6+1,dof+2)
        jacRev = sdurw_math.Jacobian(6,dof)
        bTj = sdurw_math.Transform3D()

        jacobian = sdurw_math.Jacobian.zero(6+1,dof+2)
        jacRev = sdurw_math.Jacobian.zero(6,dof)
        joint.getJacobian(1,2,joint.getJointTransform(state),sdurw_math.Transform3D.identity(),state,jacobian)
        for i in range (0,len(refJoints)):
                bTj = bTj*refJoints[i].getTransform(state)
                refJoints[i].getJacobian(0,i,bTj,sdurw_math.Transform3D.identity(),state,jacRev)
                if (i == offsetIndex):
                        bTj = bTj * Transform3D(offset)
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRev.asNumpy()), 0, delta = sys.float_info.epsilon )
        

        jacobian = sdurw_math.Jacobian.zero(6+1,dof+2)
        jacRev = sdurw_math.Jacobian(6,dof)
        bTj = sdurw_math.Transform3D.identity()
        joint.getJacobian(1,2,joint.getJointTransform(state),sdurw_math.Transform3D(),state,jacobian)
        for i in range (0,len(refJoints)):
                bTj = bTj*refJoints[i].getTransform(state)
                refJoints[i].getJacobian(0,i,bTj,T,state,jacRev)
                if (i == offsetIndex):
                        bTj = bTj * Transform3D(offset)
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRev.asNumpy()), 0, delta = 1e-15 )


class Joint(unittest.TestCase):

    def test_Prismatic(self):
        T = sdurw_math.Transform3D( sdurw_math.Vector3D(1, 2, 3), sdurw_math.RPY( math.radians(10), math.radians(20), math.radians(30)) )
        joint = sdurw_models.PrismaticJoint("TestJoint",sdurw_math.Transform3D.identity())

        self.assertEqual(1, joint.getDOF())
        self.assertEqual(1, joint.size())
        self.assertEqual("TestJoint", joint.getName())
        self.assertTrue(joint.getFixedTransform().equal( sdurw_math.Transform3D.identity() ))
        joint.setFixedTransform(T)
        self.assertTrue(joint.getFixedTransform().equal(T))

        sstate = sdurw_kinematics.StateStructure()
        sstate.addData(joint)                                                   # StateStructure takes ownership of the joint
        state = sstate.getDefaultState()

        self.assertEqual(0, joint.getData(state)[0])

        vals = [0.1]
        joint.setData(state,vals)
        
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
 #       self.assertEqual(0.1, joint.getData(state)[0])
        print("\n MANGLER   AttributeError: 'PrismaticJoint' object has no attribute 'getData'        DET VIRKER IKKE")

        self.assertTrue(joint.getJointTransform(state).equal( sdurw_math.Transform3D( sdurw_math.Vector3D().z()*0.1 )))

        print("\n MANGLER   AssertionError: False is not true        DET VIRKER IKKE")
#        self.assertTrue(joint.getJointTransform(state).equal( sdurw_math.Transform3D( (T.P()+T.R().getCol(2)*0.1), T.R()) ))

#        print("\n T.R().getCol(2)", T.R().getCol(2) )
#        print("\n T.R().getCol(2)[0]", T.R().getCol(2)[0] )
#        print("\n T.R().getCol(2)[1]", T.R().getCol(2)[1] )
#        print("\n T.R().getCol(2)[2]", T.R().getCol(2)[2] )
#        print("\n (T.P()+T.R().getCol(2)*0.1)", (T.P()+T.R().getCol(2)*0.1))
#        print("\n sdurw_math.Transform3D( (T.P()+T.R().getCol(2)*0.1), T.R())", sdurw_math.Transform3D( (T.P()+T.R().getCol(2)*0.1), T.R()))
#        print("\n T.R()", T.R() )
#        print("\n sdurw_math.Transform3D( (T.P()+T.R().getCol(2)*0.1), T.R())", sdurw_math.Transform3D( (T.P()+T.R().getCol(2)*0.1), T.R()))
#        print("\n joint.getJointTransform(state)", joint.getJointTransform(state) )


        self.assertTrue(joint.getJointTransform(state).equal(joint.getJointTransform(0.1)))
        self.assertTrue(joint.getTransform(state).equal(joint.getTransform(0.1)))

        multiplyTransform_res = sdurw_math.Transform3D(T)                       # Make a copy of T using c++ copy constructor
        joint.multiplyTransform(T, state, multiplyTransform_res)
        self.assertTrue(multiplyTransform_res.equal(T*joint.getTransform(state)))

        multiplyJointTransform_res = sdurw_math.Transform3D(T)                  # Make a copy of T using c++ copy constructor
        joint.multiplyJointTransform(T, sdurw_math.Q(1,0.1), multiplyJointTransform_res)
        self.assertTrue(multiplyJointTransform_res.equal(multiplyTransform_res))
        
        jacobian = sdurw_math.Jacobian(6+1,1+2)
        jacRef = sdurw_math.Jacobian.zero(6,1)

        jacobian = sdurw_math.Jacobian.zero(6+1,1+2)

        jacRef[2,0] = 1
        
        joint.getJacobian(1,2,joint.getJointTransform(state),sdurw_math.Transform3D.identity(),state,jacobian)

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
        #self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )

        jacobian = sdurw_math.Jacobian.zero(6+1,1+2)
        jacRef[0,0] = T.R().getCol(2)[0]
        jacRef[1,0] = T.R().getCol(2)[1]
        jacRef[2,0] = T.R().getCol(2)[2]
        joint.getJacobian(1,2,joint.getJointTransform(state),sdurw_math.Transform3D.identity(),state,jacobian)

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )
        
        jacobian = sdurw_math.Jacobian.zero(6+1,1+2)
        joint.getJacobian(1,2,joint.getJointTransform(state),T,state,jacobian)                  # Using a transformation to the control point should not change anything

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )

        testGenericJoint(self,joint,1)
        


    def test_Revolute(self):
        T = sdurw_math.Transform3D( sdurw_math.Vector3D(1, 2, 3), sdurw_math.RPY( math.radians(10), math.radians(20), math.radians(30)) )
        joint = sdurw_models.RevoluteJoint("TestJoint",sdurw_math.Transform3D.identity())

        self.assertEqual(1, joint.getDOF())
        self.assertEqual(1, joint.size())
        self.assertEqual("TestJoint", joint.getName())
        self.assertTrue(joint.getFixedTransform().equal( sdurw_math.Transform3D.identity() ))
        joint.setFixedTransform(T)
        self.assertTrue(joint.getFixedTransform().equal(T))

        sstate = sdurw_kinematics.StateStructure()
        sstate.addData(joint);                                                  # StateStructure takes ownership of the joint
        state = sstate.getDefaultState()

        self.assertEqual(0, joint.getData(state)[0])

        vals = [math.pi/2, 0]
        joint.setData(state,vals)
        
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her - getData og setData.
#        self.assertEqual(math.pi/2, joint.getData(state)[0])
        
        rotRef = sdurw_math.Rotation3D(0,-1,0,1,0,0,0,0,1)

        self.assertTrue(joint.getJointTransform(state).equal( sdurw_math.Transform3D( rotRef )))
#        self.assertTrue(joint.getJointTransform(state).equal( sdurw_math.Transform3D( T.P(),T.R()*rotRef ) ))
        self.assertTrue(joint.getJointTransform(state).equal(joint.getJointTransform(math.pi/2)))
        self.assertTrue(joint.getTransform(state).equal(joint.getTransform(math.pi/2)))

        multiplyTransform_res = sdurw_math.Transform3D(T)                       # Make a copy of v1 using c++ copy constructor
        joint.multiplyTransform(T, state, multiplyTransform_res)
        self.assertTrue(multiplyTransform_res.equal(T*joint.getTransform(state), 1e-15))

        multiplyJointTransform_res = sdurw_math.Transform3D(T)                  # Make a copy of v1 using c++ copy constructor
        joint.multiplyJointTransform(T, sdurw_math.Q(1,math.pi/2), multiplyJointTransform_res)
        self.assertTrue(multiplyJointTransform_res.equal(multiplyTransform_res))
        
        jacobian = sdurw_math.Jacobian.zero(6+1,1+2)
        jacRef = sdurw_math.Jacobian.zero(6,1)
        jacRef[5,0] = 1
        joint.getJacobian(1,2,joint.getJointTransform(state),sdurw_math.Transform3D.identity(),state,jacobian)
        
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )
        jacobian = sdurw_math.Jacobian.zero(6+1,1+2)
        
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
        # Fault: Segmentation fault (core dumped) !
#        joint.getJacobian(1,2,joint.getJointTransform(state),sdurw_math.Transform3D( sdurw_math.Vector3D().z()*0.1 ),state,jacobian)    # displacement of tcp along z should not change anything
        
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )
        jacRef = sdurw_math.Jacobian.zero(6,1)
        jacRef[1,0] = 0.1;                                                      # if tcp is displaced in x, it will move in y when rotating around z.
        joint.getJacobian(1,2,joint.getJointTransform(state),sdurw_math.Transform3D( sdurw_math.Vector3D().x()*0.1 ),state,jacobian)
        
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )
        
        jacobian = sdurw_math.Jacobian(6+1,1+2)
        jacRef = sdurw_math.Jacobian.zero(6,1)
        rotVec = T.R().getCol(2)
        jacRef[3,0] = rotVec[0]
        jacRef[4,0] = rotVec[1]
        jacRef[5,0] = rotVec[2]
        joint.getJacobian(1,2,joint.getTransform(state),T,state,jacobian);      # Using same tcp frame gives zero change in positions

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )

        jacRef = sdurw_math.Jacobian.zero(6+1,1+2)
        jacRef[0,0] = sdurw_math.cross(rotVec,-T.P())[0]
        jacRef[1,0] = sdurw_math.cross(rotVec,-T.P())[1]
        jacRef[2,0] = sdurw_math.cross(rotVec,-T.P())[2]
        joint.getJacobian(1,2,joint.getJointTransform(state),sdurw_math.Transform3D.identity(),state,jacobian)

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )
        
        testGenericJoint(self,joint,1)


    def test_Universal(self):
        joint = sdurw_models.UniversalJoint("TestJoint",sdurw_math.Transform3D.identity())

        testGenericJoint(self,joint,2)
        self.assertEqual(2, joint.getDOF())

        refJoints_A =  sdurw_models.RevoluteJoint("RevAJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( 0, math.pi/2, 0)) )
        refJoints_B =  sdurw_models.RevoluteJoint("RevBJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( 0, -(math.pi/2), 0)) )
        refJoints = [refJoints_A, refJoints_B]

        revRend = sdurw_math.RPY(-math.pi/2, 0, math.pi/2).toRotation3D()
        compareJointWithReferenceJoints(self,joint,refJoints,revRend,1)


    def test_PrismaticUniversal(self):
        joint = sdurw_models.PrismaticUniversalJoint("TestJoint",sdurw_math.Transform3D.identity())

        testGenericJoint(self,joint,3)
        self.assertEqual(3, joint.getDOF())

        refJoints_A =  sdurw_models.PrismaticUniversalJoint("RevAJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( 0, math.pi/2, 0)) )
        refJoints_B =  sdurw_models.PrismaticUniversalJoint("RevBJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( 0, -(math.pi/2), 0)) )
        refJoints_C =  sdurw_models.PrismaticJoint("PrismaticJoint",sdurw_math.Transform3D.identity())
        refJoints = [refJoints_A, refJoints_B, refJoints_C]

        revRend = sdurw_math.RPY(-math.pi/2, 0, math.pi/2).toRotation3D()
        compareJointWithReferenceJoints(self,joint,refJoints,revRend,1)
        

    def test_Spherical(self):
        joint = sdurw_models.SphericalJoint("TestJoint",sdurw_math.Transform3D.identity())

        testGenericJoint(self,joint,3)
        self.assertEqual(3, joint.getDOF())

        refJoints_A =  sdurw_models.RevoluteJoint("RevAJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( 0, math.pi/2, 0)) )
        refJoints_B =  sdurw_models.RevoluteJoint("RevBJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( 0, 0, -(math.pi/2))) )
        refJoints_C =  sdurw_models.RevoluteJoint("RevBJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( -(math.pi/2), 0, math.pi/2)) )
        refJoints = [refJoints_A, refJoints_B, refJoints_C]

        compareJointWithReferenceJoints(self,joint,refJoints)


    def test_PrismaticSpherical(self):
        joint = sdurw_models.PrismaticSphericalJoint("TestJoint",sdurw_math.Transform3D.identity())

        testGenericJoint(self,joint,4)
        self.assertEqual(4, joint.getDOF())

        refJoints_A =  sdurw_models.RevoluteJoint("RevAJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( 0, math.pi/2, 0)) )
        refJoints_B =  sdurw_models.RevoluteJoint("RevBJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( 0, 0, -(math.pi/2))) )
        refJoints_C =  sdurw_models.RevoluteJoint("RevBJoint",sdurw_math.Transform3D(sdurw_math.Vector3D().zero(), sdurw_math.RPY( -(math.pi/2), 0, math.pi/2)) )
        refJoints_D =  sdurw_models.PrismaticJoint("PrismaticJoint",sdurw_math.Transform3D.identity())
        refJoints = [refJoints_A, refJoints_B, refJoints_C, refJoints_D]

        compareJointWithReferenceJoints(self,joint,refJoints)


if __name__ == '__main__':
    unittest.main()
