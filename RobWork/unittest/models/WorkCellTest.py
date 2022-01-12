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

        world = sdurw_models.WorkCell("The World")

        with self.assertRaises(Exception):
            world.addFrame(frame1)

# TO DO Kig nærmere på dette problem omkring world.addFrame():
        print("\n MANGLER   TypeError: Wrong number or type of arguments for overloaded function 'WorkCell_addFrame'.        DET VIRKER IKKE")
#        world.addFrame(frame2)

        # Note to translation of "EXPECT_NO_THROW" into python: Simply call your functionality. If an unhandled exception gets raised, the test automatically fails! There is really no reason to do anything else. 
        #

        print("\n MANGLER   Her får man alligevel en RuntimeError        DET VIRKER IKKE")
#        self.assertRaises(RuntimeError, world.addFrame(frame1,frame2) )
        del world

        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
        print("\n MANGLER   TypeError: Wrong number or type of arguments for overloaded function 'WorkCell_addFrame'.        DET VIRKER IKKE")
#        world.addFrame(frame2)
        del world

        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
#        world.addFrame(frame3)
#        self.assertRaises(Exception, world.addFrame(frame2,frame3) )
        del world
        del frame2
        
        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
#        self.assertRaises(Exception, world.addDAF(frame1) )
#        world.addDAF(frame2)
#        world.addDAF(frame3)
#        self.assertRaises(Exception, world.addDAF(frame1,frame2) )
        del world        
        
        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
#        world.addFrame(frame2)
#        world.addFrame(frame3,frame2)
#        world.addFrame(frame6,frame2)
        del world 

        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
#        world.addFrame(frame3)
#        self.assertRaises(Exception, world.addDAF(frame2,frame3) )
#        world.addDAF(frame6,frame3)
        del world
        del frame2

        world = sdurw_models.WorkCell("The World")
        frame2 = sdurw_kinematics.MovableFrame("Frame2")
#        world.addFrame(frame2)
#        world.addFrame(frame3)
#        world.addFrame(frame5)
#        world.addFrame(frame6)
#        world.remove(frame2);              # deletes frame2
#        world.remove(frame3)
#        self.assertEqual(-1, frame3.getID())
#        world.remove(frame5)               # deletes frame5
#        world.remove(frame6)
#        self.assertEqual(-1, frame6.getID())
#        world.addFrame(frame3)
#        world.addFrame(frame6)
        del world

#        del frame1.get()
#        del frame4.get()


if __name__ == '__main__':
    unittest.main()
