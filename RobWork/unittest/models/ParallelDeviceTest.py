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

from pickle import NONE
from typing import Callable
# now we can use python unittest framework
import unittest
import math
import sys
import os
import sdurw_kinematics
import sdurw_loaders
import sdurw_math
import sdurw_models
import numpy as np

# Joint structure to test:
#
#  Fixed
#  |
#  A
#  |
#  |-Ga------------Gb---H
#  |                    |
#  |             __|    |
#  |          _Fb       |
#  |        _/          |
#  |      Fa            |
#  |      |
#  |-B---------------D
#            |       |
#            Ca      |
#            |       |
#            |       |
#            Cb   ---E
#
# Where all are revolute joints, except Fb which is prismatic
# Goal is that Fb/Gb remains connected, as well as D/H and Cb/E.
#
# Kinematic structure:
#                                      ________
#           /-------------------------|-Ca--Cb |
#          /                  ___     |        |
#     /-- B -----------------|-D-|----|-E      |
#    /    \     ________     |   |    |________|
#   /      \---|-Fa--Fb |    |   |
#  /           |        |    |   |
# A -----------|-Ga--Gb-|----|-H-|
#              |________|    |___|
#


def makeLeg(A,  B,  C=None,  D=None,  E=None,  F=None):
    size = 2 if C is None else 3 if D is None else 4 if E is None else 5 if F is None else 6
    chain = [None] *size
    chain[0] = A
    chain[1] = B
    if C is not None:
        chain[2] = C
    if D is not None:
        chain[3] = D
    if E is not None:
        chain[4] = E
    if F is not None:
        chain[5] = F

    return sdurw_models.ParallelLeg(chain)


def ParallelLegHEXAPOD(base, scope, legBase, tool, yaw, pitch):
    joint1 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
        scope+".joint1", sdurw_math.Transform3D.craigDH(0, 0, 0, yaw)))
    joint2 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
        scope+".joint2", sdurw_math.Transform3D.craigDH(math.pi/2.0, 0, 0, math.pi/2.0+pitch)))
    joint3 = sdurw_models.ownedPtr(sdurw_models.PrismaticJoint(
        scope+".joint3", sdurw_math.Transform3D.craigDH(math.pi/2.0, 0, 0, 0)))
    joint4 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
        scope+".joint4", sdurw_math.Transform3D.craigDH(-math.pi/2.0, 0, 0, math.pi/2.0)))
    joint5 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
        scope+".joint5", sdurw_math.Transform3D.craigDH(math.pi/2.0, 0, 0, -math.pi/2.0)))
    joint6 = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
        scope+".joint6", sdurw_math.Transform3D.craigDH(math.pi/2.0, 0, 0, math.pi/2.0)))

    joint1.setActive(False)
    joint2.setActive(False)
    joint3.setActive(True)
    joint4.setActive(False)
    joint5.setActive(False)
    joint6.setActive(False)

    serialChain = (base, sdurw_kinematics.FixedFrame(
        scope+".legOffset", legBase), joint1, joint2, joint3,
        joint4, joint5, joint6, sdurw_kinematics.FixedFrame(scope+".Tool", tool), 
        sdurw_kinematics.FixedFrame(scope+".End", sdurw_math.Transform3D.identity()))

    return serialChain


class ParallelDevice(unittest.TestCase):

    def test_Junctions(self):
        lAB = 0.055
        lGaGb = 0.055
        lBFa = 0.015
        lFaFb = 0.020
        lFaD = lGaGb-lBFa
        angleFb = math.atan2(lAB, lFaD)
        lFbFend = math.sqrt(lAB*lAB+lFaD*lFaD)-lFaFb

        base = sdurw_kinematics.FixedFrame(
            "Fixed", sdurw_math.Transform3D.identity())
        A = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("A",   sdurw_math.Transform3D(
            sdurw_math.Vector3D.zero(),         sdurw_math.RPY(0, 0, math.pi/2))))
        Aend = sdurw_kinematics.FixedFrame(
            "Aend", sdurw_math.Transform3D.identity())
        B = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("B",   sdurw_math.Transform3D(
            sdurw_math.Vector3D(0, 0, lAB),     sdurw_math.RPY(0, 0, -math.pi/2))))
        Bend = sdurw_kinematics.FixedFrame(
            "Bend", sdurw_math.Transform3D.identity())
        Ca = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
            "Ca",  sdurw_math.Transform3D(sdurw_math.Vector3D(0.030, 0, 0))))
        Cb = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
            "Cb",  sdurw_math.Transform3D(sdurw_math.Vector3D(0, -0.045, 0))))
        D = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
            "D",   sdurw_math.Transform3D(sdurw_math.Vector3D(lGaGb, 0, 0))))
        E = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint(
            "E",   sdurw_math.Transform3D(sdurw_math.Vector3D(0, -0.045, 0))))
        Eend = sdurw_kinematics.FixedFrame(
            "Eend", sdurw_math.Transform3D(sdurw_math.Vector3D(-0.025, 0, 0)))
        Fa = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Fa",  sdurw_math.Transform3D(
            sdurw_math.Vector3D(lBFa, 0, 0),    sdurw_math.RPY(angleFb, 0, 0))))
        Fb = sdurw_models.ownedPtr(sdurw_models.PrismaticJoint("Fb",  sdurw_math.Transform3D(
            sdurw_math.Vector3D(lFaFb, 0, 0),   sdurw_math.RPY(0, math.pi/2, 0))))
        Fbend = sdurw_kinematics.FixedFrame("Fbend", sdurw_math.Transform3D(
            sdurw_math.Vector3D(0, 0, lFbFend), sdurw_math.RPY(0, -math.pi/2, 0)))
        Ga = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Ga",  sdurw_math.Transform3D(
            sdurw_math.Vector3D.zero(),         sdurw_math.RPY(0, 0, -math.pi/2))))
        Gb = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("Gb",  sdurw_math.Transform3D(
            sdurw_math.Vector3D(lGaGb, 0, 0),   sdurw_math.RPY(angleFb, 0, 0))))
        H = sdurw_models.ownedPtr(sdurw_models.RevoluteJoint("H",   sdurw_math.Transform3D(
            sdurw_math.Vector3D.zero(),         sdurw_math.RPY(-angleFb, 0, 0))))
        Hend = sdurw_kinematics.FixedFrame(
            "Hend", sdurw_math.Transform3D(sdurw_math.Vector3D(0, -lAB, 0)))

        A.setActive(True)
        B.setActive(False)
        Ca.setActive(False)
        Cb.setActive(False)
        D.setActive(False)
        E.setActive(False)
        Fa.setActive(False)
        Fb.setActive(True)
        Ga.setActive(False)
        Gb.setActive(False)
        H.setActive(False)

        sstruct = sdurw_kinematics.StateStructure()
        sstruct.addData(base)
        sstruct.addFrame(A, base)
        sstruct.addFrame(Aend, A)
        sstruct.addFrame(Ga, Aend)
        sstruct.addFrame(Gb, Ga)
        sstruct.addFrame(H, Gb)
        sstruct.addFrame(Hend, H)
        sstruct.addFrame(B, Aend)
        sstruct.addFrame(Bend, B)
        sstruct.addFrame(D, Bend)
        sstruct.addFrame(E, D)
        sstruct.addFrame(Eend, E)
        sstruct.addFrame(Ca, Bend)
        sstruct.addFrame(Cb, Ca)
        sstruct.addFrame(Fa, Bend)
        sstruct.addFrame(Fb, Fa)
        sstruct.addFrame(Fbend, Fb)
        state = sstruct.getDefaultState()

        junctions = [[], [], []]
        junctions[0].append(makeLeg(Aend, Ga, Gb))
        junctions[0].append(makeLeg(Aend, B, Bend, Fa, Fb, Fbend))
        junctions[1].append(makeLeg(Aend, Ga, Gb, H, Hend))
        junctions[1].append(makeLeg(Aend, B, Bend, D))
        junctions[2].append(makeLeg(Bend, Ca, Cb))
        junctions[2].append(makeLeg(Bend, D, E, Eend))

        self.assertTrue(junctions[0][1].baseTend(
            state).equal(junctions[0][0].baseTend(state)))
        self.assertTrue(junctions[1][1].baseTend(
            state).equal(junctions[1][0].baseTend(state)))
        self.assertTrue(junctions[2][1].baseTend(
            state).equal(junctions[2][0].baseTend(state)))

        joints = [None] *11
        joints[0] = A
        joints[1] = B
        joints[2] = Ca
        joints[3] = Cb
        joints[4] = D
        joints[5] = E
        joints[6] = Fa
        joints[7] = Fb
        joints[8] = Ga
        joints[9] = Gb
        joints[10] = H

        device = sdurw_models.ParallelDevice(
            "TestDevice", base, E, joints, state, junctions)

        # Generic Device functions
        self.assertEqual("TestDevice", device.getName())
        self.assertTrue(device.baseTend(state).equal(
            sdurw_math.Transform3D(sdurw_math.Vector3D(lGaGb, -lAB-0.045, 0))))
        self.assertTrue(device.baseTframe(H, state).equal(
            sdurw_math.Transform3D(sdurw_math.Vector3D(lGaGb, 0, 0))))
        self.assertTrue(device.worldTbase(state).equal(
            sdurw_math.Transform3D.identity()))

        # Generic JointDevice functions
        self.assertEqual(base, device.getBase())
        self.assertEqual(E.deref(), device.getEnd())
        self.assertEqual(2, len(device.getJoints()))
        self.assertGreaterEqual(2, len(device.getJoints()))
        self.assertEqual(A.deref(), device.getJoints()[0])
        self.assertEqual(Fb.deref(), device.getJoints()[1])
        self.assertEqual(2, device.getDOF())
        self.assertEqual(2, len(device.getBounds()[0]))
        self.assertEqual(2, len(device.getBounds()[1]))
        self.assertEqual(2, len(device.getVelocityLimits()))
        self.assertEqual(2, len(device.getAccelerationLimits()))

        # # ParallelDevice functions
        # There should be no legs when using the junction concept.
        self.assertEqual(0, len(device.getLegs()))
        self.assertEqual(2, len(device.getActiveJoints()))
        self.assertGreaterEqual(2, len(device.getActiveJoints()))
        self.assertEqual(A.deref(), device.getActiveJoints()[0])
        self.assertEqual(Fb.deref(), device.getActiveJoints()[1])
        self.assertEqual(11, len(device.getAllJoints()))
        self.assertEqual(11, device.getFullDOF())
        self.assertEqual(11, len(device.getAllBounds()[0]))
        self.assertEqual(11, len(device.getAllBounds()[1]))

        # Set full Q
        self.assertEqual(11, device.getFullQ(state).size())
        vals = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        device.setFullQ(sdurw_math.Q(vals), state)
        for i in range(0, 11):
            self.assertEqual(i, device.getFullQ(state)[i])
        device.setFullQ(sdurw_math.Q(11, 0.0), state)

        # Set Q
        self.assertEqual(2, device.getQ(state).size())
        device.setQ(sdurw_math.Q(2, 0.0, 0.015), state)
        # save for later comparison
        qTest = device.getFullQ(state)
        lFaFend = lFaFb+lFbFend+0.015
        angleFaRef = math.acos(
            (lFaD*lFaD+lFaFend*lFaFend-lAB*lAB)/(lFaD*lFaFend*2))
        angleDRef = math.acos((lFaD*lFaD+lAB*lAB-lFaFend*lFaFend)/(lFaD*lAB*2))
        # Check full Q vector {A,B,Ca,Cb,D,E,Fa,Fb,Ga,Gb,H}
        eps = 1e-4
        # A (controlled)
        self.assertAlmostEqual(0, device.getFullQ(state)[0], delta=eps)
        # B
        self.assertAlmostEqual(angleDRef-math.pi/2,
                               device.getFullQ(state)[1], delta=eps)
        # Ca
        self.assertAlmostEqual(-(angleDRef-math.pi/2),
                               device.getFullQ(state)[2], delta=eps)
        # CB
        self.assertAlmostEqual(angleDRef-math.pi/2,
                               device.getFullQ(state)[3], delta=eps)
        # D
        self.assertAlmostEqual(-(angleDRef-math.pi/2),
                               device.getFullQ(state)[4], delta=eps)
        # E
        self.assertAlmostEqual(angleDRef-math.pi/2,
                               device.getFullQ(state)[5], delta=eps)
        # Fa
        self.assertAlmostEqual(
            angleFaRef-angleFb, device.getFullQ(state)[6], delta=eps)
        # Fb (controlled)
        self.assertAlmostEqual(0.015, device.getFullQ(state)[7], delta=eps)
        # Ga
        self.assertAlmostEqual(angleDRef-math.pi/2,
                               device.getFullQ(state)[8], delta=eps)
        # Gb
        self.assertAlmostEqual(
            angleFaRef-angleFb, device.getFullQ(state)[9], delta=eps)
        self.assertAlmostEqual(-(angleDRef-math.pi/2+angleFaRef-angleFb),
                               device.getFullQ(state)[10], delta=eps)                # H

        self.assertTrue(junctions[0][1].baseTend(state).equal(
            junctions[0][0].baseTend(state), 1e-6))
        self.assertTrue(junctions[1][1].baseTend(state).equal(
            junctions[1][0].baseTend(state), 1e-6))
        self.assertTrue(junctions[2][1].baseTend(state).equal(
            junctions[2][0].baseTend(state), 1e-6))
        device.setQ(sdurw_math.Q(2, 0.0, -0.015), state)

        lFaFend = lFaFb+lFbFend-0.015
        angleFaRef = math.acos(
            (lFaD*lFaD+lFaFend*lFaFend-lAB*lAB)/(lFaD*lFaFend*2))
        angleDRef = math.acos((lFaD*lFaD+lAB*lAB-lFaFend*lFaFend)/(lFaD*lAB*2))
        # Check full Q vector {A,B,Ca,Cb,D,E,Fa,Fb,Ga,Gb,H}
        eps = 1e-4
        # A (controlled)
        self.assertAlmostEqual(0, device.getFullQ(state)[0], delta=eps)
        # B
        self.assertAlmostEqual(angleDRef-math.pi/2,
                               device.getFullQ(state)[1], delta=eps)
        # Ca
        self.assertAlmostEqual(-(angleDRef-math.pi/2),
                               device.getFullQ(state)[2], delta=eps)
        # CB
        self.assertAlmostEqual(angleDRef-math.pi/2,
                               device.getFullQ(state)[3], delta=eps)
        # D
        self.assertAlmostEqual(-(angleDRef-math.pi/2),
                               device.getFullQ(state)[4], delta=eps)
        # E
        self.assertAlmostEqual(angleDRef-math.pi/2,
                               device.getFullQ(state)[5], delta=eps)
        # Fa
        self.assertAlmostEqual(
            angleFaRef-angleFb, device.getFullQ(state)[6], delta=eps)
        # Fb (controlled)
        self.assertAlmostEqual(-0.015, device.getFullQ(state)[7], delta=eps)
        # Ga
        self.assertAlmostEqual(angleDRef-math.pi/2,
                               device.getFullQ(state)[8], delta=eps)
        # Gb
        self.assertAlmostEqual(
            angleFaRef-angleFb, device.getFullQ(state)[9], delta=eps)
        self.assertAlmostEqual(-(angleDRef-math.pi/2+angleFaRef-angleFb),
                               device.getFullQ(state)[10], delta=eps)                # H

        self.assertTrue(junctions[0][1].baseTend(state).equal(
            junctions[0][0].baseTend(state), 1e-6))
        self.assertTrue(junctions[1][1].baseTend(state).equal(
            junctions[1][0].baseTend(state), 1e-6))
        self.assertTrue(junctions[2][1].baseTend(state).equal(
            junctions[2][0].baseTend(state), 1e-6))
#        del device

        # Test that we get same result with extra active joint, that is then disabled when setting setQ
        B.setActive(True)
        state = sstruct.getDefaultState()
        devExtraJoint = sdurw_models.ParallelDevice(
            "TestDevice", base, E, joints, state, junctions)
        enabled = [True, True, True]
        enabled[1] = False
        devExtraJoint.setQ(sdurw_math.Q(3, 0.0, 0.2, 0.015), enabled, state)
        self.assertTrue(np.allclose(devExtraJoint.getFullQ(state).asNumpy(
        ) - qTest.asNumpy(), np.zeros([1, 11]).transpose(), 1e-4, 1e-4))

        # Then test that we can control the other active joint instead
        enabled[1] = True
        enabled[2] = False
        devExtraJoint.setQ(sdurw_math.Q(3, 0.0, 0.2, 0.015), enabled, state)

        lFaFendRef = math.sqrt(lFaD*lFaD+lAB*lAB+lFaD*lAB*2*math.sin(0.2))
        angleFaRef = math.acos((lFaD+lAB*math.sin(0.2))/lFaFendRef)
        # Check full Q vector {A,B,Ca,Cb,D,E,Fa,Fb,Ga,Gb,H}
        eps = 1e-4
        # A (controlled)
        self.assertAlmostEqual(0, devExtraJoint.getFullQ(state)[0], delta=eps)
        # B (controlled)
        self.assertAlmostEqual(0, devExtraJoint.getFullQ(state)[0], delta=eps)
        self.assertAlmostEqual(0.2, devExtraJoint.getFullQ(
            state)[1], delta=eps)                                                 # Ca
        # CB
        self.assertAlmostEqual(-0.2,
                               devExtraJoint.getFullQ(state)[2], delta=eps)
        self.assertAlmostEqual(0.2, devExtraJoint.getFullQ(
            state)[3], delta=eps)                                                 # D
        # E
        self.assertAlmostEqual(-0.2,
                               devExtraJoint.getFullQ(state)[4], delta=eps)
        self.assertAlmostEqual(0.2, devExtraJoint.getFullQ(
            state)[5], delta=eps)                                                 # Fa
        # Fa
        self.assertAlmostEqual(
            angleFaRef-angleFb, devExtraJoint.getFullQ(state)[6], delta=eps)
        self.assertAlmostEqual(lFaFendRef-lFaFb-lFbFend, devExtraJoint.getFullQ(state)[
                               7], delta=eps)                            # Fb
        self.assertAlmostEqual(0.2, devExtraJoint.getFullQ(
            state)[8], delta=eps)                                                 # Ga
        # Gb
        self.assertAlmostEqual(
            angleFaRef-angleFb, devExtraJoint.getFullQ(state)[9], delta=eps)
        self.assertAlmostEqual(-(0.2+angleFaRef-angleFb), devExtraJoint.getFullQ(state)[
                               10], delta=eps)                          # H

        self.assertTrue(junctions[0][1].baseTend(state).equal(
            junctions[0][0].baseTend(state), 1e-6))
        self.assertTrue(junctions[1][1].baseTend(state).equal(
            junctions[1][0].baseTend(state), 1e-6))
        self.assertTrue(junctions[2][1].baseTend(state).equal(
            junctions[2][0].baseTend(state), 1e-6))

    def test_SerialChains(self):
        world = sdurw_kinematics.FixedFrame(
            "World", sdurw_math.Transform3D.identity())
        R = sdurw_math.EAA(sdurw_math.Vector3D(1, 0, 1).normalize(), math.pi)

        legChains = [None]*6

        # Leg A
        baseTA = sdurw_math.Transform3D(
            sdurw_math.Vector3D(-60.206416539249155, -11.580306882912305, 0), R)
        toolTA = sdurw_math.Transform3D(sdurw_math.Vector3D(
            14.533675867691082, -8.586935664729255, 5.486283046170201), sdurw_math.RPY(0, -0.5058297976189102, 0.2554802362083838))
        legChains[0] = ParallelLegHEXAPOD(
            world, "LegA", baseTA, toolTA, -0.2554802362083838, 0.5058297976189102)

        # Leg B
        baseTB = sdurw_math.Transform3D(
            sdurw_math.Vector3D(-40.132048213846446, -46.350132752361176, 0), R)
        toolTB = sdurw_math.Transform3D(sdurw_math.Vector3D(-2.6407792002537804, 16.673016630261774,
                                        5.486283046154413), sdurw_math.RPY(0, -0.4486102166464917, 0.3501394675976276))
        legChains[1] = ParallelLegHEXAPOD(
            world, "LegB", baseTB, toolTB, -0.3501394675976276, 0.4486102166464917)

        # Leg C
        baseTC = sdurw_math.Transform3D(sdurw_math.Vector3D(
            40.13204821384643, -46.35013275236119, 0), R)
        toolTC = sdurw_math.Transform3D(sdurw_math.Vector3D(
            2.6407792002537804, 16.673016630261774, 5.486283046154413), sdurw_math.RPY(0, 0.4486102166464917, 0.3501394675976276))
        legChains[2] = ParallelLegHEXAPOD(
            world, "LegC", baseTC, toolTC, -0.3501394675976276, -0.4486102166464917)

        # Leg D
        baseTD = sdurw_math.Transform3D(sdurw_math.Vector3D(
            60.20641653924915, -11.580306882912327, 0), R)
        toolTD = sdurw_math.Transform3D(sdurw_math.Vector3D(-14.533675867691082, -8.586935664729255,
                                        5.486283046170201), sdurw_math.RPY(0, 0.5058297976189102, 0.2554802362083838))
        legChains[3] = ParallelLegHEXAPOD(
            world, "LegD", baseTD, toolTD, -0.2554802362083838, -0.5058297976189102)

        # Leg E
        baseTE = sdurw_math.Transform3D(sdurw_math.Vector3D(
            20.074368325402705, 57.930439635273515, 0), R)
        toolTE = sdurw_math.Transform3D(sdurw_math.Vector3D(-15.112667091527895, -7.521335766866318,
                                        5.486283046171359), sdurw_math.RPY(0, 0.05084170375926413, -0.5595869360340129))
        legChains[4] = ParallelLegHEXAPOD(
            world, "LegE", baseTE, toolTE, 0.5595869360340129, -0.05084170375926413)

        # Leg F
        baseTF = sdurw_math.Transform3D(
            sdurw_math.Vector3D(-20.0743683254027, 57.93043963527352, 0), R)
        toolTF = sdurw_math.Transform3D(sdurw_math.Vector3D(
            15.112667091527895, -7.521335766866318, 5.486283046171359), sdurw_math.RPY(0, -0.05084170375926413, -0.5595869360340129))
        legChains[5] = ParallelLegHEXAPOD(
            world, "LegF", baseTF, toolTF, 0.5595869360340129, 0.05084170375926413)

        sstruct = sdurw_kinematics.StateStructure()
        sstruct.addData(world)
        for i in range(0, len(legChains), 1):
            for j in range(1, len(legChains[i]), 1):
                sstruct.addFrame(legChains[i][j], legChains[i][j-1])

        state = sstruct.getDefaultState()
        qh = sdurw_math.Q(6, 0.0, 0.0, 92.6865, 0.0, 0.0, 0.0)

        legs = [None] * len(legChains)

        for i in range(0, len(legChains), 1):
            legs[i] = sdurw_models.ParallelLeg(legChains[i])
            legs[i].setQ(qh, state)

        hexapod = sdurw_models.ParallelDevice(legs,"Hexapod",state)
        q = hexapod.getQ(state)

        hexapod.setQ(q,state)


        bTe =hexapod.baseTend(state)
        self.assertAlmostEqual(0,bTe.P()[0],delta = 5e-12)
        self.assertAlmostEqual(0,bTe.P()[1],delta = 5e-12)
        self.assertAlmostEqual(78.5,bTe.P()[2],delta = 1e-4)
        self.assertTrue(bTe.R().equal(sdurw_math.Rotation3D.identity(),1e-13))

        for i in range (0,len(legs)):
            self.assertTrue(legs[i].baseTend(state).R().equal(hexapod.baseTend(state).R(),1e-12))
            self.assertAlmostEqual(0,(legs[i].baseTend(state).P()-hexapod.baseTend(state).P()).normInf(),1e-10)

        q[0] += 1
        hexapod.setQ(q,state)
        for i in range (0,len(legs)):
            self.assertTrue(legs[i].baseTend(state).R().equal(hexapod.baseTend(state).R(),1e-12))
            self.assertAlmostEqual(0,(legs[i].baseTend(state).P()-hexapod.baseTend(state).P()).normInf(),1e-10)

        q[1] += 2
        q[2] += 1
        q[3] += 3

        hexapod.setQ(q,state)
        for i in range (0,len(legs)):
            self.assertTrue(legs[i].baseTend(state).R().equal(hexapod.baseTend(state).R(),5e-10))
            self.assertAlmostEqual(0,(legs[i].baseTend(state).P()-hexapod.baseTend(state).P()).normInf(),5e-9)

        q[1] += 2
        q[2] += 1
        q[3] += 3
        q[4] += 20
        q[5] += 3

        hexapod.setQ(q,state)
        for i in range (0,len(legs)):
            self.assertTrue(legs[i].baseTend(state).R().equal(hexapod.baseTend(state).R(),1e-11))
            self.assertAlmostEqual(0,(legs[i].baseTend(state).P()-hexapod.baseTend(state).P()).normInf(),1e-9)

        self.assertEqual(6,q.size())
        self.assertGreaterEqual(6,q.size())
        self.assertEqual(93.6865, q[0])
        self.assertEqual(96.6865, q[1])
        self.assertEqual(94.6865, q[2])
        self.assertEqual(98.6865, q[3])
        self.assertEqual(112.6865, q[4])
        self.assertEqual(95.6865, q[5])

        qFull = hexapod.getFullQ(state)
        self.assertEqual(36, qFull.size())
        self.assertGreaterEqual(36, qFull.size())
        self.assertEqual(93.6865, qFull[2])
        self.assertEqual(96.6865, qFull[8])
        self.assertEqual(94.6865, qFull[14])
        self.assertEqual(98.6865, qFull[20])
        self.assertEqual(112.6865, qFull[26])
        self.assertEqual(95.6865, qFull[32])

    def test_Robotiq(self):

        path = os.getcwd()+"/../gtest/testfiles/devices/Robotiq-2-finger-85/robotiq.wc.xml"
        if not os.path.exists(path):
            path = os.getcwd()+"/../../gtest/testfiles/devices/Robotiq-2-finger-85/robotiq.wc.xml"
        wc = sdurw_loaders.WorkCellLoaderFactory.load(path)
        self.assertFalse(wc.isNull())

        robotiqDist = wc.findParallelDevice("RobotiqDistanceControl")
        robotiqFinger = wc.findParallelDevice("RobotiqFingerControl")
        self.assertFalse(robotiqDist.isNull())
        self.assertFalse(robotiqFinger.isNull())

        state = wc.getDefaultState()

        # Generic Device functions
        self.assertEqual("RobotiqDistanceControl", robotiqDist.getName())
        self.assertEqual("RobotiqFingerControl", robotiqFinger.getName())
        self.assertTrue(robotiqDist.baseTend(state).equal(sdurw_math.Transform3D(
            sdurw_math.Vector3D(-0.0425, 0.119008, 0), sdurw_math.RPY(0, math.pi/2, 0)), 1e-6))
        self.assertTrue(robotiqFinger.baseTend(state).equal(sdurw_math.Transform3D(
            sdurw_math.Vector3D(-0.0425, 0.119008, 0), sdurw_math.RPY(0, math.pi/2, 0)), 1e-4))

        # Generic JointDevice functions
        self.assertEqual("RobotiqDistanceControl.Base",
                         robotiqDist.getBase().getName())
        self.assertEqual("RobotiqDistanceControl.LeftMotorEnd",
                         robotiqDist.getEnd().getName())
        self.assertEqual("RobotiqFingerControl.Base",
                         robotiqFinger.getBase().getName())
        self.assertEqual("RobotiqFingerControl.LeftMotorEnd",
                         robotiqFinger.getEnd().getName())
        self.assertEqual(3, len(robotiqDist.getJoints()))

        self.assertEqual(3, robotiqDist.getDOF())
        self.assertEqual(3, robotiqDist.getBounds()[0].size())
        self.assertEqual(3, robotiqDist.getBounds()[1].size())
        self.assertEqual(3, robotiqDist.getVelocityLimits().size())
        self.assertEqual(3, robotiqDist.getAccelerationLimits().size())
        # two motor joints maps to one DOF (dependent joints)
        self.assertEqual(4, len(robotiqFinger.getJoints()))

        self.assertEqual(3, robotiqFinger.getDOF())
        self.assertEqual(3, robotiqFinger.getBounds()[0].size())
        self.assertEqual(3, robotiqFinger.getBounds()[1].size())
        self.assertEqual(3, robotiqFinger.getVelocityLimits().size())
        self.assertEqual(3, robotiqFinger.getAccelerationLimits().size())

        # ParallelDevice functions
        # There should be no legs when using the junction concept.
        self.assertEqual(0, len(robotiqDist.getLegs()))
        self.assertEqual(3, len(robotiqDist.getJunctions()))
        self.assertGreaterEqual(len(robotiqDist.getJunctions()), 3)
        self.assertEqual(2, len(robotiqDist.getJunctions()[0]))
        self.assertEqual(2, len(robotiqDist.getJunctions()[1]))
        self.assertEqual(2, len(robotiqDist.getJunctions()[2]))
        self.assertEqual(3, len(robotiqDist.getActiveJoints()))
        self.assertEqual(13, len(robotiqDist.getAllJoints()))
        self.assertEqual(12, robotiqDist.getFullDOF())
        self.assertEqual(12, robotiqDist.getAllBounds()[0].size())
        self.assertEqual(12, robotiqDist.getAllBounds()[1].size())
        # There should be no legs when using the junction concept.
        self.assertEqual(0, len(robotiqFinger.getLegs()))
        self.assertEqual(2, len(robotiqFinger.getJunctions()))
        self.assertGreaterEqual(len(robotiqFinger.getJunctions()), 2)
        self.assertEqual(2, len(robotiqFinger.getJunctions()[0]))
        self.assertEqual(2, len(robotiqFinger.getJunctions()[1]))
        self.assertEqual(4, len(robotiqFinger.getActiveJoints()))
        self.assertEqual(10, len(robotiqFinger.getAllJoints()))
        self.assertEqual(9, robotiqFinger.getFullDOF())
        self.assertEqual(9, robotiqFinger.getAllBounds()[0].size())
        self.assertEqual(9, robotiqFinger.getAllBounds()[1].size())

        self.assertEqual(3, robotiqDist.getQ(state).size())
        self.assertEqual(12, robotiqDist.getFullQ(state).size())
        self.assertEqual(3, robotiqFinger.getQ(state).size())
        self.assertEqual(9, robotiqFinger.getFullQ(state).size())

        #  Set Q
        robotiqDist.setQ(sdurw_math.Q(3, 0., 0.04, 0.), state)

        eps = 1e-4
        self.assertAlmostEqual(
            0.4506, robotiqDist.getFullQ(state)[0], delta=eps)
        self.assertAlmostEqual(
            0.0008, robotiqDist.getFullQ(state)[1], delta=eps)
        self.assertAlmostEqual(-0.4514,
                               robotiqDist.getFullQ(state)[2], delta=eps)
        self.assertAlmostEqual(
            0.4511, robotiqDist.getFullQ(state)[3], delta=eps)
        self.assertAlmostEqual(-0.4511,
                               robotiqDist.getFullQ(state)[4], delta=eps)
        self.assertAlmostEqual(
            0.0008, robotiqDist.getFullQ(state)[5], delta=eps)
        self.assertAlmostEqual(-0.4514,
                               robotiqDist.getFullQ(state)[6], delta=eps)
        self.assertAlmostEqual(
            0.4511, robotiqDist.getFullQ(state)[7], delta=eps)
        self.assertAlmostEqual(-0.4511,
                               robotiqDist.getFullQ(state)[8], delta=eps)
        self.assertEqual(0, robotiqDist.getFullQ(state)[9])
        self.assertEqual(0.04, robotiqDist.getFullQ(state)[10])
        self.assertEqual(0, robotiqDist.getFullQ(state)[11])

        self.assertTrue(robotiqDist.getJunctions()[0][0].baseTend(state).equal(
            robotiqDist.getJunctions()[0][1].baseTend(state), 1e-6))
        self.assertTrue(robotiqDist.getJunctions()[1][0].baseTend(state).equal(
            robotiqDist.getJunctions()[1][1].baseTend(state), 1e-6))
        self.assertTrue(robotiqDist.getJunctions()[2][0].baseTend(state).equal(
            robotiqDist.getJunctions()[2][1].baseTend(state), 1e-6))

        robotiqDist.setQ(sdurw_math.Q(3, -0.35, 0.04, 0.35), state)

        self.assertAlmostEqual(
            0.4858, robotiqDist.getFullQ(state)[0], delta=eps)
        self.assertAlmostEqual(-0.2596,
                               robotiqDist.getFullQ(state)[1], delta=eps)
        self.assertAlmostEqual(
            0.1237, robotiqDist.getFullQ(state)[2], delta=eps)
        self.assertAlmostEqual(
            0.2841, robotiqDist.getFullQ(state)[3], delta=eps)
        self.assertAlmostEqual(
            0.0659, robotiqDist.getFullQ(state)[4], delta=eps)
        self.assertAlmostEqual(-0.2596,
                               robotiqDist.getFullQ(state)[5], delta=eps)
        self.assertAlmostEqual(
            0.1237, robotiqDist.getFullQ(state)[6], delta=eps)
        self.assertAlmostEqual(
            0.2841, robotiqDist.getFullQ(state)[7], delta=eps)
        self.assertAlmostEqual(
            0.0658, robotiqDist.getFullQ(state)[8], delta=eps)
        self.assertEqual(-0.35, robotiqDist.getFullQ(state)[9])
        self.assertEqual(0.04, robotiqDist.getFullQ(state)[10])
        self.assertEqual(0.35, robotiqDist.getFullQ(state)[11])

        self.assertTrue(robotiqDist.getJunctions()[0][0].baseTend(state).equal(
            robotiqDist.getJunctions()[0][1].baseTend(state), 1e-6))
        self.assertTrue(robotiqDist.getJunctions()[1][0].baseTend(state).equal(
            robotiqDist.getJunctions()[1][1].baseTend(state), 1e-6))
        self.assertTrue(robotiqDist.getJunctions()[2][0].baseTend(state).equal(
            robotiqDist.getJunctions()[2][1].baseTend(state), 1e-6))

        robotiqFinger.setQ(sdurw_math.Q(3, 0.35, 0.25, 0.25), state)

        self.assertEqual(0.35, robotiqFinger.getFullQ(state)[0])
        self.assertAlmostEqual(-0.1294,
                               robotiqFinger.getFullQ(state)[1], delta=eps)
        self.assertAlmostEqual(-0.0422,
                               robotiqFinger.getFullQ(state)[2], delta=eps)
        self.assertEqual(0.25, robotiqFinger.getFullQ(state)[3])
        self.assertAlmostEqual(-0.0716,
                               robotiqFinger.getFullQ(state)[4], delta=eps)
        self.assertAlmostEqual(-0.1294,
                               robotiqFinger.getFullQ(state)[5], delta=eps)
        self.assertAlmostEqual(-0.0422,
                               robotiqFinger.getFullQ(state)[6], delta=eps)
        self.assertEqual(0.25, robotiqFinger.getFullQ(state)[7])
        self.assertAlmostEqual(-0.0716,
                               robotiqFinger.getFullQ(state)[8], delta=eps)

        self.assertTrue(robotiqFinger.getJunctions()[0][0].baseTend(state).equal(
            robotiqFinger.getJunctions()[0][1].baseTend(state), 1e-6))
        self.assertTrue(robotiqFinger.getJunctions()[1][0].baseTend(state).equal(
            robotiqFinger.getJunctions()[1][1].baseTend(state), 1e-6))

        robotiqFinger.setQ(sdurw_math.Q(3, 0.5, 0.52, 0.2), state)

        self.assertEqual(0.5, robotiqFinger.getFullQ(state)[0])
        self.assertAlmostEqual(
            0.0327, robotiqFinger.getFullQ(state)[1], delta=eps)
        self.assertAlmostEqual(-0.5776,
                               robotiqFinger.getFullQ(state)[2], delta=eps)
        self.assertEqual(0.52, robotiqFinger.getFullQ(state)[3])
        self.assertAlmostEqual(-0.5650,
                               robotiqFinger.getFullQ(state)[4], delta=eps)
        self.assertAlmostEqual(-0.3623,
                               robotiqFinger.getFullQ(state)[5], delta=eps)
        self.assertAlmostEqual(
            0.3618, robotiqFinger.getFullQ(state)[6], delta=eps)
        self.assertEqual(0.2, robotiqFinger.getFullQ(state)[7])
        self.assertAlmostEqual(
            0.2995, robotiqFinger.getFullQ(state)[8], delta=eps)

        self.assertTrue(robotiqFinger.getJunctions()[0][0].baseTend(state).equal(
            robotiqFinger.getJunctions()[0][1].baseTend(state), 1e-6))
        self.assertTrue(robotiqFinger.getJunctions()[1][0].baseTend(state).equal(
            robotiqFinger.getJunctions()[1][1].baseTend(state), 1e-6))


if __name__ == '__main__':
    unittest.main()
