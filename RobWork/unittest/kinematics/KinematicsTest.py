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
from sdurw_kinematics.sdurw_kinematics import Stateless
import sdurw_math
import sdurw_kinematics
from sdurw_math.sdurw_math import Vector3D
#from sdurw_math.sdurw_math import RPY
#import numpy as np


class KinematicsTest(unittest.TestCase):

    def test_StatelessObjectTest(self):

        print("\n We have not implemented StatelessData in python yet")
#       We have not implemented StatelessData in python yet thus left out
#
#        class MyObj(sdurw_kinematics.Stateless):
#            def __init__(self):
#                self._ival = sdurw_kinematics.StatelessData_i()
#               self._v3d = sdurw_kinematics.StatelessDataVector3D()
#
#                super().add(self._ival)
#                super().add(self._v3d)


    def test_StateStructureTest(self):
        l1 = sdurw_kinematics.FixedFrame("l1",sdurw_math.Transform3D(Vector3D(1,2,3)))
        m1 = sdurw_kinematics.MovableFrame("m1")
        daf = sdurw_kinematics.FixedFrame("daf",sdurw_math.Transform3D(Vector3D(1,2,3)))

        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()
        tree.addFrame(l1,world)
        tree.addFrame(m1,world)
        tree.addDAF(daf,world)

        state = tree.getDefaultState()

        m1_t3d = sdurw_math.Transform3D(sdurw_math.Vector3D(1,2,3))
        m1.setTransform(m1_t3d, state)
        m1_t3d_b = sdurw_math.Transform3D()
        m1_t3d_b = m1.getTransform(state)

        self.assertAlmostEqual( m1_t3d_b.P()[0], m1_t3d.P()[0], delta = 1e-6 )
        self.assertAlmostEqual( m1_t3d_b.P()[1], m1_t3d.P()[1], delta = 1e-6 )
        self.assertAlmostEqual( m1_t3d_b.P()[2], m1_t3d.P()[2], delta = 1e-6 )

        # TO DO 
        # "We need to beable to compare the frames directly"
        # So this would work:  self.assertEqual( world, daf.getParent(state) )
        #
        self.assertEqual( world.getName() , daf.getParent(state).getName() )

        daf.attachTo(m1, state)
#        self.assertEqual( m1 , daf.getParent(state) )
        daf.attachTo(l1, state)
#        self.assertEqual( l1 , daf.getParent(state) )

        tree.setDefaultState(state)
        tree.remove(l1)

        # the daf should have changed its parent to world in the default state
        dafParent = daf.getParent(tree.getDefaultState())
        self.assertEqual( world.getName(), dafParent.getName() )

        # now add a new l1 frame. Remember the tree took ownership of the old
        # l1 frame so we are not allowed to use that name again
        l1 = sdurw_kinematics.FixedFrame("l1b",sdurw_math.Transform3D(Vector3D(1,2,3)))
        tree.addFrame(l1,world)
        state = tree.upgradeState(state)
        daf.attachTo(l1, state)
        tree.setDefaultState(state)

        m2 = sdurw_kinematics.MovableFrame("m2")
        tree.addFrame(m2,world)
        nstate = tree.getDefaultState()
        nstate.copy(state)

#        self.assertEqual( l1 , daf.getParent(nstate) )
        m1_t3d_b = m1.getTransform(nstate)
        self.assertAlmostEqual( m1_t3d_b.P()[0], m1_t3d.P()[0], delta = 1e-6 )
        self.assertAlmostEqual( m1_t3d_b.P()[1], m1_t3d.P()[1], delta = 1e-6 )
        self.assertAlmostEqual( m1_t3d_b.P()[2], m1_t3d.P()[2], delta = 1e-6 )

        # also check if old state values are not deleted
#        self.assertEqual( l1 , daf.getParent(state) )
        m1_t3d_b = m1.getTransform(state)
        self.assertAlmostEqual( m1_t3d_b.P()[0], m1_t3d.P()[0], delta = 1e-6 )
        self.assertAlmostEqual( m1_t3d_b.P()[1], m1_t3d.P()[1], delta = 1e-6 )
        self.assertAlmostEqual( m1_t3d_b.P()[2], m1_t3d.P()[2], delta = 1e-6 )

        # this should test the influence on StateCache when state is copied/deep-copied

        # TO DO
        # AttributeError: module 'sdurw_kinematics' has no attribute 'StateDataWithCache'
        #
#        ol = sdurw_kinematics.StateDataWithCache()
#        tree.addData( o1 )
        nstate = tree.getDefaultState()
#        o1.setA(1,nstate)

        # standard shallow copy
        cstate = nstate
        # first test that the values are the same in the two states
#        self.assertEqual( o1.getA(nstate),o1.getA(cstate) )
        # now change it in one state and verify that the values are still the same
#        o1.setA(20,nstate)
#        self.assertEqual( o1.getA(nstate),o1.getA(cstate) )

        # deep copy
        dcstate = nstate.clone()
        # first test that the values are the same in the two states
#        self.assertEqual( o1.getA(nstate),o1.getA(dcstate) )
        # now change it in one state and verify that the values are NOT the same anymore
#        o1.setA(50,nstate)
#        self.assertNotEqual( o1.getA(nstate),o1.getA(dcstate) )


    def test_removeFramesTest(self):
        l1 = sdurw_kinematics.FixedFrame("l1",sdurw_math.Transform3D())
        l2 = sdurw_kinematics.FixedFrame("l2",sdurw_math.Transform3D())
        l3 = sdurw_kinematics.FixedFrame("l3",sdurw_math.Transform3D())
        l4 = sdurw_kinematics.FixedFrame("l4",sdurw_math.Transform3D())
        l5 = sdurw_kinematics.FixedFrame("l5",sdurw_math.Transform3D())
        l6 = sdurw_kinematics.FixedFrame("l6",sdurw_math.Transform3D())
        l7 = sdurw_kinematics.FixedFrame("l7",sdurw_math.Transform3D())

        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()
        tree.addFrame(l1,world)
        tree.addFrame(l2,l1)
        tree.addFrame(l3,l2)
        tree.addFrame(l4,l3)
        tree.addFrame(l5,l4)
        tree.addFrame(l6,l5)
        tree.addFrame(l7,l6)
        
        state = tree.getDefaultState()

        frame = l7
        while(frame != world) :
            parent = frame.getParent(tree.getDefaultState())
            tree.remove(frame)
            frame = parent
            
    def test_stateStructureFrame(self):
        l1 = sdurw_kinematics.FixedFrame("l1",sdurw_math.Transform3D())
        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()     
        self.assertEqual(world,tree.getRoot())
            

    def test_removeMovableFramesTest(self):
        l1 = sdurw_kinematics.MovableFrame("l1")
        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()
        tree.addFrame(l1,world)
        state = tree.getDefaultState()
        l1.setTransform(sdurw_math.Transform3D.identity(), state)
        tree.remove(l1)
        self.assertEqual( None , tree.findFrame("l1") )
        

    def test_singleChainTest(self):
        l1 = sdurw_kinematics.FixedFrame("l1",sdurw_math.Transform3D(sdurw_math.Vector3D(1, 2, 3)))
        l2 = sdurw_kinematics.FixedFrame("l2",sdurw_math.Transform3D(sdurw_math.Vector3D(2, 3, 4)))
        l3 = sdurw_kinematics.FixedFrame("l3",sdurw_math.Transform3D(sdurw_math.Vector3D(3, 4, 5)))
        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()
        tree.addFrame(l1,world)
        tree.addFrame(l2,l1)
        tree.addFrame(l3,l2)
        state = tree.getDefaultState()

        transform = sdurw_kinematics.Kinematics.frameTframe(world, l3, state)
        self.assertEqual( transform.P()[0] , 6.0 )
        self.assertEqual( transform.P()[1] , 9.0 )
        self.assertEqual( transform.P()[2] , 12.0 )


    def test_multipleChainTest(self):
        l1 = sdurw_kinematics.FixedFrame("l1",sdurw_math.Transform3D(sdurw_math.Vector3D(1, 2, 3)))
        l2 = sdurw_kinematics.FixedFrame("l2",sdurw_math.Transform3D(sdurw_math.Vector3D(2, 3, 4)))
        tree = sdurw_kinematics.StateStructure()
        world = tree.getRoot()
        tree.addFrame(l1,world)
        tree.addFrame(l2,world)

        state = tree.getDefaultState()
        
        transform = sdurw_kinematics.Kinematics.frameTframe(world, l1, state)
        self.assertEqual( transform.P()[0] , 1.0 )
        self.assertEqual( transform.P()[1] , 2.0 )
        self.assertEqual( transform.P()[2] , 3.0 )

        transform = sdurw_kinematics.Kinematics.frameTframe(world, l2, state)
        self.assertEqual( transform.P()[0] , 2.0 )
        self.assertEqual( transform.P()[1] , 3.0 )
        self.assertEqual( transform.P()[2] , 4.0 )


if __name__ == '__main__':
    unittest.main()
