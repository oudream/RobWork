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
import sdurw_kinematics
import sdurw_math
import sdurw_models


# NOTE TIL Ken: frame 3 og farme 6 er fjernet da det ikke giver mening, så det skal koden også afspejle.

class WorkCell(unittest.TestCase):

    def test_AddRemoveFrame(self):
        frame1 = sdurw_kinematics.ownedPtr(sdurw_kinematics.MovableFrame("Frame1"))
        frame2 = sdurw_kinematics.MovableFrame("Frame2")

        frame4 = sdurw_kinematics.ownedPtr(sdurw_kinematics.MovableFrame("Frame4"))
        frame5 = sdurw_kinematics.MovableFrame("Frame5")

        # FIRST
        world = sdurw_models.WorkCell("The World")
        with self.assertRaises(Exception):
            world.addFrame(frame1.deref())
        world.addFrame(frame2)
        world.addFrame(frame1,frame2.deref()) 
        del world

        #Second
        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
        world.addFrame(frame2)
        del world       
        
        #Third
        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
        del world
        del frame2
        
        #Fourth
        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
        with self.assertRaises(Exception):
                world.addDAF(frame1.deref())
        world.addDAF(frame2)
        with self.assertRaises(Exception):
            world.addDAF(frame1.deref(),frame2)
        del world       
        
        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
        world.addFrame(frame2)
        world.addFrame(frame5)
        world.remove(frame2);             # deletes frame2
        self.assertEqual(-1, frame2.getID())
        world.remove(frame5)               # deletes frame5
        self.assertEqual(-1, frame5.getID())
        world.addFrame(frame2)
        world.addFrame(frame5)
        del world

if __name__ == '__main__':
    unittest.main()
