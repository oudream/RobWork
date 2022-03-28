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

# now we can use python unittest framework
import unittest
import sdurw_math as math
import sdurw_kinematics as kinematics
import sdurw_models as models
from sdurw_math.sdurw_math import Transform3D, Vector3D
from sdurw_math.sdurw_math import RPY
import numpy as np


def testGenericJoint(self, joint, dof):
    self.assertEqual(dof, joint.getBounds()[0].size())
    self.assertEqual(dof, joint.getBounds()[1].size())

    joint.setBounds((math.Q(dof, -0.6), math.Q(dof, 0.2)))
    self.assertEqual(-0.6, joint.getBounds()[0][dof-1])
    self.assertEqual(0.2, joint.getBounds()[1][dof-1])

    self.assertEqual(dof, joint.getMaxVelocity().size())
    joint.setMaxVelocity(math.Q(dof, 2.9))
    self.assertEqual(2.9, joint.getMaxVelocity()[dof-1])

    self.assertEqual(dof, joint.getMaxAcceleration().size())
    joint.setMaxAcceleration(math.Q(dof, 1.4))
    self.assertEqual(1.4, joint.getMaxAcceleration()[dof-1])

    self.assertTrue(joint.isActive())
    joint.setActive(False)
    self.assertFalse(joint.isActive())
    joint.setActive(True)
    self.assertTrue(joint.isActive())


def compareJointWithReferenceJoints(self, joint, refJoints, offset=math.Rotation3D.identity(), offsetIndex=0):
    T = math.Transform3D(math.Vector3D(1, 2, 3), math.RPY(
        math.Deg2Rad * 45, math.Deg2Rad * 45, math.Deg2Rad * 45))

    dof = joint.getDOF()
    self.assertEqual(dof, joint.size())
    self.assertTrue(joint.getFixedTransform().equal(
        math.Transform3D.identity()))
    joint.setFixedTransform(T)
    self.assertTrue(joint.getFixedTransform().equal(T))

    sstate = kinematics.StateStructure()
    # StateStructure takes ownership of the joint
    sstate.addData(joint)

    for i in range(0, len(refJoints)):
        # StateStructure takes ownership of the joint
        sstate.addData(refJoints[i])
    state = sstate.getDefaultState()

    for i in range(0, len(refJoints)):
        self.assertEqual(0, joint.getData(state)[i])

    vals = [0, 0, 0, 0]
    for i in range(0, len(refJoints)):
        vals[i] = 0.1*i
    joint.setData(state, vals)

    for i in range(0, len(refJoints)):
        refJoints[i].setData(state, vals[i])

    for i in range(0, len(refJoints)):
        self.assertEqual(0.1*i, joint.getData(state)[i])

    revT = math.Transform3D()
    for i in range(0, len(refJoints)):
        revT = revT*refJoints[i].getTransform(state)
        if (i == offsetIndex):
            revT = revT * Transform3D(offset)

    print(joint.getJointTransform(state).R())
    print(revT.R())
    print(joint.getJointTransform(state).R().equal(revT.R()))
    print("\n")
#    self.assertTrue(joint.getJointTransform(state).R().equal(revT.R()))
#    print("\n (joint.getJointTransform(state).P()-revT.P()).normInf()", (joint.getJointTransform(state).P()-revT.P()).normInf() )
#    self.assertAlmostEqual(0.0, (joint.getJointTransform(state).P()-revT.P()).normInf(), delta = sys.float_info.epsilon)

    revTfull = T*revT
#    self.assertTrue(joint.getTransform(state).R().equal(revTfull.R()))
#    self.assertAlmostEqual(0.0, (joint.getTransform(state).P()-revTfull.P()).normInf(), delta = sys.float_info.epsilon)

    # initial value should not influence result
    multiplyTransform_res = T
    joint.multiplyTransform(T, state, multiplyTransform_res)
#    self.assertTrue(multiplyTransform_res.equal(T*joint.getTransform(state),1e-15))

    jacobian = math.Jacobian(6+1, dof+2)
    jacRev = math.Jacobian(6, dof)
    bTj = math.Transform3D()

    jacobian = math.Jacobian.zero(6+1, dof+2)
    jacRev = math.Jacobian.zero(6, dof)
    joint.getJacobian(1, 2, joint.getJointTransform(state),
                      math.Transform3D.identity(), state, jacobian)
    for i in range(0, len(refJoints)):
        bTj = bTj*refJoints[i].getTransform(state)
        refJoints[i].getJacobian(
            0, i, bTj, math.Transform3D.identity(), state, jacRev)
        if (i == offsetIndex):
            bTj = bTj * Transform3D(offset)
    # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
        #self.assertTrue( np.array_equal((jacobian.asNumpy()[1:7,2] - jacRev.asNumpy().transpose()[0]),np.zeros(6)))

    jacobian = math.Jacobian.zero(6+1, dof+2)
    jacRev = math.Jacobian(6, dof)
    bTj = math.Transform3D.identity()
    joint.getJacobian(1, 2, joint.getJointTransform(state),
                      math.Transform3D(), state, jacobian)
    for i in range(0, len(refJoints)):
        bTj = bTj*refJoints[i].getTransform(state)
        refJoints[i].getJacobian(0, i, bTj, T, state, jacRev)
        if (i == offsetIndex):
            bTj = bTj * Transform3D(offset)
    # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#       self.assertTrue( np.array_equal((jacobian.asNumpy()[1:7,2] - jacRev.asNumpy().transpose()[0]),np.zeros(6)))


class Joint(unittest.TestCase):

    def test_Prismatic(self):
        T = math.Transform3D(math.Vector3D(1, 2, 3), math.RPY(
            math.Deg2Rad * 10, math.Deg2Rad * 10, math.Deg2Rad * 30))
        joint = models.PrismaticJoint(
            "TestJoint", math.Transform3D.identity())

        self.assertEqual(1, joint.getDOF())
        self.assertEqual(1, joint.size())
        self.assertEqual("TestJoint", joint.getName())
        self.assertTrue(joint.getFixedTransform().equal(
            math.Transform3D.identity()))
        joint.setFixedTransform(T)
        self.assertTrue(joint.getFixedTransform().equal(T))

        sstate = kinematics.StateStructure()
        # StateStructure takes ownership of the joint
        sstate.addData(joint)
        state = sstate.getDefaultState()

        self.assertEqual(0, joint.getData(state)[0])

        joint.setData(state, 0.1)
        self.assertEqual(0.1, joint.getData(state)[0])

        self.assertTrue(joint.getJointTransform(state).equal(
            math.Transform3D(math.Vector3D().z()*0.1)))
        self.assertTrue(joint.getTransform(state).equal(
            math.Transform3D((T.P()+T.R().getCol(2)*0.1), T.R())))
        self.assertTrue(joint.getJointTransform(
            state).equal(joint.getJointTransform(0.1)))
        self.assertTrue(joint.getTransform(
            state).equal(joint.getTransform(0.1)))

        # Make a copy of T using c++ copy constructor
        multiplyTransform_res = math.Transform3D(T)
        joint.multiplyTransform(T, state, multiplyTransform_res)
        self.assertTrue(multiplyTransform_res.equal(
            T*joint.getTransform(state)))

        # Make a copy of T using c++ copy constructor
        multiplyJointTransform_res = math.Transform3D(T)
        joint.multiplyJointTransform(T, math.Q(
            1, 0.1), multiplyJointTransform_res)
        self.assertTrue(multiplyJointTransform_res.equal(
            multiplyTransform_res))

        jacRef = math.Jacobian.zero(6, 1)
        jacobian = math.Jacobian.zero(6+1, 1+2)

        jacRef[2, 0] = 1
        joint.getJacobian(1, 2, joint.getJointTransform(
            state), math.Transform3D.identity(), state, jacobian)
        self.assertTrue(np.array_equal(
            (jacobian.asNumpy()[1:7, 2] - jacRef.asNumpy().transpose()[0]), np.zeros(6)))

        jacobian = math.Jacobian.zero(6+1, 1+2)
        jacRef[0, 0] = T.R()[0, 2]
        jacRef[1, 0] = T.R()[1, 2]
        jacRef[2, 0] = T.R()[2, 2]
        joint.getJacobian(1, 2, joint.getJointTransform(
            state), math.Transform3D.identity(), state, jacobian)

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
        #self.assertTrue( np.array_equal((jacobian.asNumpy()[1:7,2] - jacRef.asNumpy().transpose()[0]),np.zeros(6)))

        jacobian = math.Jacobian.zero(6+1, 1+2)
        # Using a transformation to the control point should not change anything
        joint.getJacobian(1, 2, joint.getJointTransform(
            state), T, state, jacobian)

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )

        testGenericJoint(self, joint, 1)

    def test_Revolute(self):
        T = math.Transform3D(math.Vector3D(1, 2, 3), math.RPY(
            math.Deg2Rad * 10, math.Deg2Rad * 20, math.Deg2Rad * 30))
        joint = models.RevoluteJoint(
            "TestJoint", math.Transform3D.identity())

        self.assertEqual(1, joint.getDOF())
        self.assertEqual(1, joint.size())
        self.assertEqual("TestJoint", joint.getName())
        self.assertTrue(joint.getFixedTransform().equal(
            math.Transform3D.identity()))
        joint.setFixedTransform(T)
        self.assertTrue(joint.getFixedTransform().equal(T))

        sstate = kinematics.StateStructure()
        # StateStructure takes ownership of the joint
        sstate.addData(joint)
        state = sstate.getDefaultState()

        self.assertEqual(0, joint.getData(state)[0])

        vals = [math.Pi/2, 0]
        joint.setData(state, vals)

        self.assertEqual(math.Pi/2, joint.getData(state)[0])

        rotRef = math.Rotation3D(0, -1, 0, 1, 0, 0, 0, 0, 1)

        self.assertTrue(joint.getJointTransform(
            state).equal(math.Transform3D(rotRef)))
        self.assertTrue(joint.getTransform(state).equal(
            math.Transform3D(T.P(), T.R()*rotRef)))
        self.assertTrue(joint.getJointTransform(
            state).equal(joint.getJointTransform(math.Pi/2)))
        self.assertTrue(joint.getTransform(
            state).equal(joint.getTransform(math.Pi/2)))

        # Make a copy of v1 using c++ copy constructor
        multiplyTransform_res = math.Transform3D(T)
        joint.multiplyTransform(T, state, multiplyTransform_res)
        self.assertTrue(multiplyTransform_res.equal(
            T*joint.getTransform(state), 1e-15))

        # Make a copy of v1 using c++ copy constructor
        multiplyJointTransform_res = math.Transform3D(T)
        joint.multiplyJointTransform(T, math.Q(
            1, math.Pi/2), multiplyJointTransform_res)
        self.assertTrue(multiplyJointTransform_res.equal(
            multiplyTransform_res))

        jacobian = math.Jacobian.zero(6+1, 1+2)
        jacRef = math.Jacobian.zero(6, 1)
        jacRef[5, 0] = 1
        joint.getJacobian(1, 2, joint.getJointTransform(
            state), math.Transform3D.identity(), state, jacobian)

        self.assertTrue(np.array_equal(
            (jacobian.asNumpy()[1:7, 2] - jacRef.asNumpy().transpose()[0]), np.zeros(6)))
        jacobian = math.Jacobian.zero(6+1, 1+2)
        joint.getJacobian(1, 2, joint.getJointTransform(state), math.Transform3D(math.Vector3D(
        ).z()*0.1), state, jacobian)    # displacement of tcp along z should not change anything
        self.assertTrue(np.array_equal(
            (jacobian.asNumpy()[1:7, 2] - jacRef.asNumpy().transpose()[0]), np.zeros(6)))

        jacRef = math.Jacobian.zero(6, 1)
        # if tcp is displaced in x, it will move in y when rotating around z.
        jacRef[1, 0] = 0.1
        joint.getJacobian(1, 2, joint.getJointTransform(state), math.Transform3D(
            math.Vector3D().x()*0.1), state, jacobian)

        # print(jacobian.asNumpy()[1:7,2])
        # print(jacRef.asNumpy().transpose()[0])
        #print((jacobian.asNumpy()[1:7,2] - jacRef.asNumpy().transpose()[0]))
        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
        #self.assertTrue( np.array_equal((jacobian.asNumpy()[1:7,2] - jacRef.asNumpy().transpose()[0]),np.zeros(6)))

        jacobian = math.Jacobian(6+1, 1+2)
        jacRef = math.Jacobian.zero(6, 1)
        rotVec = T.R().getCol(2)
        jacRef[3, 0] = rotVec[0]
        jacRef[4, 0] = rotVec[1]
        jacRef[5, 0] = rotVec[2]
        # Using same tcp frame gives zero change in positions
        joint.getJacobian(1, 2, joint.getTransform(state), T, state, jacobian)

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )

        jacRef = math.Jacobian.zero(6+1, 1+2)
        jacRef[0, 0] = math.cross(rotVec, -T.P())[0]
        jacRef[1, 0] = math.cross(rotVec, -T.P())[1]
        jacRef[2, 0] = math.cross(rotVec, -T.P())[2]
        joint.getJacobian(1, 2, joint.getJointTransform(
            state), math.Transform3D.identity(), state, jacobian)

        # TO DO Kasper skal kigge lidt nærmere på hvad der sker her.
#        self.assertAlmostEqual( (jacobian.asNumpy()[0:2, 1:7] - jacRef.asNumpy()), 0, delta = sys.float_info.epsilon )

        testGenericJoint(self, joint, 1)

    def test_Universal(self):
        joint = models.UniversalJoint(
            "TestJoint", math.Transform3D.identity())

        testGenericJoint(self, joint, 2)
        self.assertEqual(2, joint.getDOF())

        refJoints_A = models.RevoluteJoint("RevAJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(0, math.Pi/2, 0)))
        refJoints_B = models.RevoluteJoint("RevBJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(0, -(math.Pi/2), 0)))
        refJoints = [refJoints_A, refJoints_B]

        revRend = math.RPY(-math.Pi/2, 0, math.Pi/2).toRotation3D()
        print("Universal")
        compareJointWithReferenceJoints(self, joint, refJoints, revRend, 1)

    def test_PrismaticUniversal(self):
        joint = models.PrismaticUniversalJoint(
            "TestJoint", math.Transform3D.identity())

        testGenericJoint(self, joint, 3)

        refJoints_A = models.RevoluteJoint("RevAJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(0, math.Pi/2, 0)))
        refJoints_B = models.RevoluteJoint("RevBJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(0, -(math.Pi/2), 0)))
        refJoints_C = models.PrismaticJoint(
            "PrismaticJoint", math.Transform3D.identity())
        refJoints = [refJoints_A, refJoints_B, refJoints_C]

        revRend = math.RPY(-math.Pi/2, 0, math.Pi/2).toRotation3D()
        self.assertEqual(3, joint.getDOF())
        print("PrismaticUniversal")
        compareJointWithReferenceJoints(self, joint, refJoints, revRend, 1)

    def test_Spherical(self):
        joint = models.SphericalJoint(
            "TestJoint", math.Transform3D.identity())

        testGenericJoint(self, joint, 3)
        self.assertEqual(3, joint.getDOF())

        refJoints_A = models.RevoluteJoint("RevAJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(0, math.Pi/2, 0)))
        refJoints_B = models.RevoluteJoint("RevBJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(0, 0, -(math.Pi/2))))
        refJoints_C = models.RevoluteJoint("RevBJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(-(math.Pi/2), 0, math.Pi/2)))
        refJoints = [refJoints_A, refJoints_B, refJoints_C]

        print("Spherical")
        compareJointWithReferenceJoints(self, joint, refJoints)

    def test_PrismaticSpherical(self):
        joint = models.PrismaticSphericalJoint(
            "TestJoint", math.Transform3D.identity())

        testGenericJoint(self, joint, 4)
        self.assertEqual(4, joint.getDOF())

        refJoints_A = models.RevoluteJoint("RevAJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(0, math.Pi/2, 0)))
        refJoints_B = models.RevoluteJoint("RevBJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(0, 0, -(math.Pi/2))))
        refJoints_C = models.RevoluteJoint("RevBJoint", math.Transform3D(
            math.Vector3D().zero(), math.RPY(-(math.Pi/2), 0, math.Pi/2)))
        refJoints_D = models.PrismaticJoint(
            "PrismaticJoint", math.Transform3D.identity())
        refJoints = [refJoints_A, refJoints_B, refJoints_C, refJoints_D]

        print("PrismaticSpherical")
        compareJointWithReferenceJoints(self, joint, refJoints)


if __name__ == '__main__':
    unittest.main()
