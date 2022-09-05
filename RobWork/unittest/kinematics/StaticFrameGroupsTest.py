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
from sdurw_common.sdurw_common import ownedPtr
import sdurw_math
import sdurw_kinematics
import sdurw_models
from sdurw_math.sdurw_math import Vector3D



def addTestFrames(world):
    frame1 = sdurw_kinematics.MovableFrame("Frame1")
    frame2 = sdurw_kinematics.MovableFrame("Frame2")
    frame3 = sdurw_kinematics.MovableFrame("Frame3")
    frame4 = sdurw_kinematics.FixedFrame("Frame4",sdurw_math.Transform3D(Vector3D()))
    frame5 = sdurw_kinematics.FixedFrame("Frame5",sdurw_math.Transform3D(Vector3D()))
    frame6 = sdurw_kinematics.MovableFrame("Frame6")
    frame7 = sdurw_kinematics.MovableFrame("Frame7")
    frame8 = sdurw_kinematics.MovableFrame("Frame8")
    frame9 = sdurw_kinematics.MovableFrame("Frame9")
    frame10 = sdurw_kinematics.MovableFrame("Frame10")
    frame11 = sdurw_kinematics.MovableFrame("Frame11")
    frame12 = sdurw_kinematics.FixedFrame("Frame12",sdurw_math.Transform3D(Vector3D()))

    world.addFrame(frame1)
    world.addFrame(frame3, frame1)
    world.addFrame(frame4, frame1)
    world.addDAF(frame8, frame1)
    world.addDAF(frame7, frame4)

    world.addFrame(frame2)
    world.addFrame(frame5, frame2)
    world.addFrame(frame6, frame5)
    world.addFrame(frame9, frame6)
    world.addDAF(frame10, frame9)
    world.addDAF(frame11, frame9)

    world.addFrame(frame12)


class Kinematics(unittest.TestCase):

    def test_getStaticFrameGroups(self):
        world = sdurw_models.WorkCell("The World")
        world.getName()

        addTestFrames(world)

        
        frameInGroup = []
        for i in range(13):
            frameInGroup.append(False)

        root = world.getWorldFrame()
        
        staticGroups = sdurw_kinematics.Kinematics.getStaticFrameGroups (root, world.getDefaultState ())
        self.assertEqual(10, len(staticGroups))
         
        for group in staticGroups:
            for frame in group:
                if (frame.getName() == "WORLD"):
                    self.assertFalse(frameInGroup[0])
                    frameInGroup[0] = True
                else:
                    id = int(frame.getName()[5:])                   # Remove 5 first characters by slicing
                    self.assertFalse(frameInGroup[id])
                    frameInGroup[id] = True

        for groupe in frameInGroup:
            self.assertTrue(groupe)


    def test_getStaticFrameGroupsConst(self):
        # Does it make sence to test this if we cannot use std::vector< ConstFrameList >    ?
        pass


if __name__ == '__main__':
    unittest.main()
