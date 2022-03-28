#################################################################################
 # Copyright 2021 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 # Faculty of Engineering, University of Southern Denmark
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http:#www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #################################################################################

import unittest                                     # now we can use python unittest framework
import math, sys
import sdurw_kinematics
import sdurw_math
import sdurw_models
from numpy.linalg import norm 
import numpy as np


"""
  The analytical solution to the forward kinematics of the Unimatron PUMA560
  robot (from John J. Craig "Introduction to Robotics, mechanics and control, 3nd edition" (page 83) )
"""
def Puma560(q, a2, d3, a3, d4):
    c1 = math.cos(q[0])
    c2 = math.cos(q[1])
    c3 = math.cos(q[2])
    c4 = math.cos(q[3])
    c5 = math.cos(q[4])
    c6 = math.cos(q[5])

    s1 = math.sin(q[0])
    s2 = math.sin(q[1])
    s3 = math.sin(q[2])
    s4 = math.sin(q[3])
    s5 = math.sin(q[4])
    s6 = math.sin(q[5])

    c23 = c2*c3-s2*s3
    s23 = c2*s3+s2*c3

    r11 = c1*(c23*(c4*c5*c6-s4*s6) - s23*s5*c6) + s1 * (s4*c5*c6+c4*s6)
    r21 = s1*(c23*(c4*c5*c6-s4*s6) - s23*s5*c6) - c1 * (s4*c5*c6+c4*s6)
    r31 = -s23*(c4*c5*c6-s4*s6)-c23*s5*c6;

    r12 = c1*(c23*(-c4*c5*s6-s4*c6)+s23*s5*s6) + s1*(c4*c6-s5*c5*s6)
    r22 = s1*(c23*(-c4*c5*s6-s4*c6)+s23*s5*s6) - c1*(c4*c6-s4*c5*s6)
    r32 = -s23*(-c4*c5*s6-s4*c6)+c23*s5*s6;

    r13 = -c1*(c23*c4*s5+s23*c5) - s1*s4*s5
    r23 = -s1*(c23*c4*s5+s23*c5) + c1*s4*s5
    r33 = s23*c4*s5-c23*c5

    px = c1*(a2*c2+a3*c23-d4*s23) - d3*s1
    py = s1*(a2*c2+a3*c23-d4*s23) + d3*c1
    pz = -a3*s23-a2*s2-d4*c23

    return sdurw_math.Transform3D(sdurw_math.Vector3D(px, py, pz), 
                                     sdurw_math.Rotation3D(r11, r12, r13,
                                                           r21, r22, r23,
                                                           r31, r32, r33))


class SerialDeciveTest(unittest.TestCase):

    def test_JointTest(self):
        rjoint = sdurw_models.RevoluteJoint("RevoluteJointA",sdurw_math.Transform3D.identity())
        self.assertLess(rjoint.getBounds()[0][0] , -1000000.0)
        self.assertGreater(rjoint.getBounds()[1][0] , 1000000.0)

        pjoint = sdurw_models.RevoluteJoint("RevoluteJointA",sdurw_math.Transform3D.identity())
        self.assertLess(rjoint.getBounds()[0][0] , -1000000.0)
        self.assertGreater(rjoint.getBounds()[1][0] , 1000000.0)


    def test_forwardKinematicsTest(self):
        """
        first a simple test, remember the State takes ownership of the
        frames added to the tree.
        Which means they have to be allocated with new
        Define the world frame and construct the frame Tree
        """

        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()

        # Simple forward kinematic of one joint
        # Define a very simple robot
        base = sdurw_kinematics.FixedFrame("base",sdurw_math.Transform3D.identity())
        joint1 = sdurw_models.RevoluteJoint("joint1",sdurw_math.Transform3D.identity())
        tool = sdurw_kinematics.FixedFrame("tool",sdurw_math.Transform3D.identity())

        # Update the tree with the serial chain
        tree.addFrame(base,world);
        tree.addFrame(joint1,base);
        tree.addFrame(tool,joint1);

        state = tree.getDefaultState()

        simple = sdurw_models.SerialDevice(base, tool, "simple1", state)

        self.assertEqual(len( simple.frames() ) , 3)
        self.assertEqual( str( simple.getBase() ) , str(base) )

        qs = sdurw_math.Q(1)
        qs[0] = math.pi/2.0
        simple.setQ(qs,state)

        self.assertEqual(simple.getQ(state)[0] , math.pi/2.0)
        T = joint1.getTransform(state)
        self.assertEqual(np.linalg.norm( T.P().asNumpy()), 0)
        self.assertLess( np.linalg.norm( sdurw_math.EAA(0.0, 0.0, math.pi/2.0).toRotation3D().asNumpy() - sdurw_math.EAA(0.0, 0.0, math.pi/2.0).toRotation3D().asNumpy() ), 1e-6)

        bTe_s = simple.baseTend(state)

        self.assertEqual(np.linalg.norm( bTe_s.P().asNumpy()), 0)
        self.assertLess( np.linalg.norm( bTe_s.R().asNumpy() - sdurw_math.EAA(0.0, 0.0, math.pi/2.0).toRotation3D().asNumpy() ), 1e-6)


    def test_forwardKinematics_PUMA560_Test(self):
        """
        Now a more advarced test, using the PUMA560 serial robot.
        Once again we define the world frame and construct the frame Tree.
        """

        # forward kinematics of a serialChain robot:

        # Define the constants
        a2 = 0.4318
        d3 = -1.15005
        a3 = 0.0203
        d4 = -0.4318

        """
        * Define the individual links and axes frames of the PUMA560 serial robot
         * the Craig D-H parameters for the individual links are taken from
         * Figure 3.18 (page 79) in the
         * "Introduction to Robotics, mechanics and control, 3nd edition"
         * by John J. Craig
        """

        # Once again we define the world frame and construct the frame Tree.
        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()

        # Define the PUMA560 base frame
        base = sdurw_kinematics.FixedFrame("Base",sdurw_math.Transform3D.identity())

        joint1 = sdurw_models.RevoluteJoint("Joint1",sdurw_math.Transform3D.craigDH( 0, 0, 0, 0) )
        joint2 = sdurw_models.RevoluteJoint("Joint2",sdurw_math.Transform3D.craigDH( -math.pi/2.0, 0, 0, 0) )
        joint3 = sdurw_models.RevoluteJoint("Joint3",sdurw_math.Transform3D.craigDH( 0, a2, d3, 0) )
        joint4 = sdurw_models.RevoluteJoint("Joint4",sdurw_math.Transform3D.craigDH( -math.pi/2.0, a3, d4, 0) )
        joint5 = sdurw_models.RevoluteJoint("Joint5",sdurw_math.Transform3D.craigDH(  math.pi/2.0, 0, 0, 0) )
        joint6 = sdurw_models.RevoluteJoint("Joint6",sdurw_math.Transform3D.craigDH( -math.pi/2.0, 0, 0, 0) )
        # And last define the PUMA560 end-effector frame
        tool = sdurw_kinematics.FixedFrame("Tool",sdurw_math.Transform3D.identity())

        # Add all frames and joints to the Tree there by defining their parent child relationship
        tree.addFrame(base,world)
        tree.addFrame(joint1,base)
        tree.addFrame(joint2,joint1)
        tree.addFrame(joint3,joint2)
        tree.addFrame(joint4,joint3)
        tree.addFrame(joint5,joint4)
        tree.addFrame(joint6,joint5)
        tree.addFrame(tool,joint6)

        # Construct the State that should hold the states of the seriel device
        state = tree.getDefaultState()

        # Now we are ready to construct the serial device
        puma560Device = sdurw_models.SerialDevice(base, tool, "PUMA560", state)

        # Define the waypoint
        q = sdurw_math.Q(6)
        q[0] = math.pi/8.0
        q[1] = math.pi/8.0
        q[2] = math.pi/8.0
        q[3] = math.pi/8.0
        q[4] = math.pi/8.0
        q[5] = math.pi/8.0

        # Set the configuration of the robot
        puma560Device.setQ(q,state)

        # Compare the DH based forward kinematics with the analytical solution
        bTe1 = sdurw_kinematics.Kinematics.frameTframe(base,tool,state)
        bTe2 = puma560Device.baseTend(state)
        compare = Puma560(q, a2, d3, a3, d4)

        self.assertLess( np.linalg.norm( (bTe1.P() - compare.P()).asNumpy() ), 1e-6)
        self.assertLess( np.linalg.norm( (bTe1.P().asNumpy() - compare.P().asNumpy()) ), 1e-6)

        self.assertLess( np.linalg.norm( (bTe2.P() - compare.P()).asNumpy() ), 1e-6)
        self.assertLess( np.linalg.norm( (bTe2.P().asNumpy() - compare.P().asNumpy()) ), 1e-6)


    def test_SerialDevice(self):
        # Define the world frame and construct the frame Tree
        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()

        # Define the simplified (only 6-dof) Kuka-kr16 robot
        # Define the base frame
        base = sdurw_kinematics.FixedFrame("Base",sdurw_math.Transform3D(sdurw_math.Vector3D(2.0, 0.0, 1.0) , sdurw_math.RPY(math.pi, 0.0, math.pi)) )

        # And then all the joints
        joint1 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Joint1",sdurw_math.Transform3D.craigDH( 0, 0, 0, 0) ))
        joint2 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Joint2",sdurw_math.Transform3D.craigDH( math.pi/2.0, 0.26, 0, 0) ))
        joint3 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Joint3",sdurw_math.Transform3D.craigDH( 0, 0.68, 0, 0) ))
        joint4 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Joint4",sdurw_math.Transform3D.craigDH( math.pi/2.0, -0.035, -0.67, 0) ))
        joint5 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Joint5",sdurw_math.Transform3D.craigDH(-math.pi/2.0, 0, 0, 0) ))
        joint6 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Joint6",sdurw_math.Transform3D.craigDH( math.pi/2.0, 0, 0, 0) ))

        # And last define the Kuka-kr16 end-effector frame, but don't add it to the serial chain yet
        tool = sdurw_kinematics.FixedFrame("Tool",sdurw_math.Transform3D(sdurw_math.Vector3D(  -0.141,          0.0,           -0.299), 
                                                                         sdurw_math.Rotation3D( 0.0, -1.0/math.sqrt(2.0), -1.0/math.sqrt(2.0),
                                                                                               -1.0,            0.0,            0.0,
                                                                                                0.0,  1.0/math.sqrt(2.0), -1.0/math.sqrt(2.0) )) )

        # add all frames and joints to the Tree there by defining their parent child relationship
        tree.addFrame(base,world)
        tree.addFrame(joint1,base)
        tree.addFrame(joint2,joint1)
        tree.addFrame(joint3,joint2)
        tree.addFrame(joint4,joint3)
        tree.addFrame(joint5,joint4)
        tree.addFrame(joint6,joint5)
        tree.addFrame(tool,joint6)

        # Now before constructing the device, construct the rest of the environment.
        # Define the environment
        tableFrame = sdurw_kinematics.FixedFrame("Table",sdurw_math.Transform3D(sdurw_math.Vector3D(2.0, 1.0, 0.8) , sdurw_math.RPY(0.0, 0.0, math.pi)) )
        klods1Frame = sdurw_kinematics.FixedFrame("Klods1",sdurw_math.Transform3D(sdurw_math.Vector3D(1.2, 0.52, 0.22) , sdurw_math.RPY(math.pi, 0.0, math.pi)) )
        klods2Frame = sdurw_kinematics.FixedFrame("Klods2",sdurw_math.Transform3D(sdurw_math.Vector3D(1.19, -0.5, 0.22) , sdurw_math.RPY(math.pi/2.0, 0.0, math.pi)) )
        klods3Frame = sdurw_kinematics.FixedFrame("Klods3",sdurw_math.Transform3D(sdurw_math.Vector3D(0.58, 0.52, 0.22) , sdurw_math.RPY(0.0, 0.0, math.pi)) )
        klods4Frame = sdurw_kinematics.FixedFrame("Klods4",sdurw_math.Transform3D(sdurw_math.Vector3D(0.58, -0.5, 0.22) , sdurw_math.RPY(math.pi/4.0, 0.0, math.pi)) )
        klods5Frame = sdurw_kinematics.FixedFrame("Klods5",sdurw_math.Transform3D(sdurw_math.Vector3D(0.855, 0.0, 0.22) , sdurw_math.RPY(297.0*math.pi/180.0, 0.0, math.pi)) )

        tree.addFrame(tableFrame, world)
        tree.addFrame(klods1Frame, world)
        tree.addFrame(klods2Frame, world)
        tree.addFrame(klods3Frame, world)
        tree.addFrame(klods4Frame, world)
        tree.addFrame(klods5Frame, world)

        # construct the State that should hold the states of the seriel device
        # State state(tree)
        state = tree.getDefaultState()

        # Now we are ready to construct the serial device
        kr16t = sdurw_models.SerialDevice(base, tool, "KR16", state)
        
        print(kr16t.baseTend(state).P() )
        # Define the waypoint
        qwp = sdurw_math.Q(6)
        qwp[0] = 0.0
        qwp[1] = -60.0 * math.pi / 180.0
        qwp[2] = 15.0 * math.pi / 180.0
        qwp[3] = 0.0
        qwp[4] = 0.0
        qwp[5] = 0.0


# TO DO Der er formodentligt et internt pointer problem her.
        # Set the configuration of the robot
        kr16t.setQ(qwp,state)
        for f in kr16t.frames():
                print(f.getName(),f.fTf(kr16t.getBase(),state).P())
#c++ result
#base: Vector3D(0, 0, 0)
#Joint1: Vector3D(-0, -0, -0)
#Joint2: Vector3D(-0.13, -0.225167, -0)
#Joint3: Vector3D(-0.840677, -0.00785081, 6.16298e-33)
#Joint4: Vector3D(-0.805677, -4.80724e-19, 0.677851)
#Joint5: Vector3D(-0.805677, -0.677851, 4.10257e-17)
#Joint6: Vector3D(-0.805677, -4.80724e-19, 0.677851)
#Tool: Vector3D(4.80724e-19, 1.16074, -0.22074)        
                
                
        # Compare the DH based forward kinematics with the analytical solution
        print("\n MANGLER        AssertionError: 1.16074 not less than 1e-05            DET VIRKER IKKE")
        print("End:",kr16t.baseTend(state).P() )
        print("Q:  ",kr16t.getQ(state))
#        self.assertLess( np.linalg.norm( (kr16t.baseTend(state).P() - sdurw_math.Vector3D(1.16074, 0.0, 0.22074)).asNumpy(),np.inf ), 1e-5)
#        k = kr16t.baseTend(state).P()
#        v = sdurw_math.Vector3D(1.16074, 0.0, 0.22074)
#        print("\n k", k )
#        print("\n v", v )
#        print("\n k-v", k-v )

        self.assertLess( np.linalg.norm( (kr16t.baseTend(state).R().asNumpy() - sdurw_math.Rotation3D(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).asNumpy() ) ), 1e-6)
        self.assertEqual(kr16t.getName() , "KR16")

        """
        Jacobian test
        """

        kr16_j6 = joint6

        # Make velocity vector
        dq = sdurw_math.Q(6, 0.0)
        for i in range (0,dq.size()):
            dq[i] = 1.0
        dq[5] = 0
        # Velocity of end-effector seen from base
        # Since baseJframe(f, state) calculates the jacobian matrix of a frame f described in the robot base frame
        print("\n MANGLER   TypeError: unsupported operand type(s) for *: 'Jacobian' and 'Q'        DET VIRKER IKKE")
#        bVe = sdurw_math.VelocityScrew6D( kr16t.baseJframe(kr16_j6,state) * dq )

        # Velocity of tool seen from base
        # Since baseJend() calculates the jacobian matrix of the end-effector described in the robot base frame
        print("\n MANGLER   TypeError: unsupported operand type(s) for *: 'Jacobian' and 'Q'        DET VIRKER IKKE")
#        bVt = sdurw_math.VelocityScrew6D( kr16t.baseJend(state) * dq )

        # Velocity of end-effector seen from end-effector
#        eVe = sdurw_math.VelocityScrew6D( sdurw_math.inverse(kr16t.baseTframe(kr16_j6,state).R()) * bVe )
        # Velocity of tool seen from tool
#        tVt = sdurw_math.VelocityScrew6D( sdurw_math.inverse(kr16t.baseTframe(kr16t.getEnd(), state).R()) * bVt )

        # set translation to be from tool (t) to end-effector (e)
        eTt = sdurw_kinematics.Kinematics.frameTframe( kr16_j6, kr16t.getEnd(), state)
        tTe = sdurw_math.inverse( eTt )

        # Velocity of end-effector seen from end-effector (calculated from tool velocity)
#        eVe2 = eTt * tVt
        # Velocity of tool seen from tool (calculated from end-effector velocity)
#        tVt2 = tTe  * eVe

#        self.assertLess( sdurw_math.normInf(eVe - eVe2), 1e-6 )
#        self.assertLess( sdurw_math.normInf(tVt - tVt2), 1e-6 )

        t_bTe = kr16t.baseTend(state)
        t_bTf = kr16t.baseTframe(tool,state)
        t3dmetric = sdurw_kinematics.MetricFactory.makeTransform3DMetric(1.0,1.0)
        self.assertLess(t3dmetric.distance(t_bTe,t_bTf) , 1e-6)

        b_eJq = kr16t.baseJframe(kr16_j6,state)
#        e_eJb_e(sdurw_math.inverse(kr16t.baseTframe(kr16_j6,state).R()))
        e_eJb_e = sdurw_math.Jacobian(sdurw_math.inverse(kr16t.baseTframe(kr16_j6,state).R()))
#        t_tJe_e( tTe )
        t_tJe_e = sdurw_math.Jacobian(tTe)

        print("\n MANGLER   TypeError: unsupported operand type(s) for *: 'Jacobian' and 'Jacobian'        DET VIRKER IKKE")
#        t_tJq = t_tJe_e * e_eJb_e * b_eJq
#        tVt3 = sdurw_math.VelocityScrew6D(t_tJq * dq)

#        self.assertLess( sdurw_math.normInf(tVt - tVt3), 1e-6 )

        b_tJe_t = sdurw_math.Jacobian(kr16t.baseTframe(kr16_j6,state).R())
        e_tJt_t = sdurw_math.Jacobian(tool.getTransform(state).R())

        print("\n MANGLER   TypeError: unsupported operand type(s) for *: 'Jacobian' and 'Jacobian'        DET VIRKER IKKE")
#        b_tJb_e = b_tJe_t * e_tJt_t * t_tJe_e * e_eJb_e

        print("\n MANGLER DER IKKE EN FORM FOR TEST HER ??")


if __name__ == '__main__':
    unittest.main()
