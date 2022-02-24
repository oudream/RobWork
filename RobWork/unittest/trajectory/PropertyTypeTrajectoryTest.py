#################################################################################
 # Copyright 2022 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
import sdurw_core
import sdurw_trajectory as trajectory


class PropertyTypeTrajectory(unittest.TestCase):

    def test_PrintInformation(self):
        # This is used to make debugging in gitlab CI easier
        print("\n Testing /RobWork/RobWork/unittest/trajectory/PropertyTypeTrajectoryTest.py")

    def test_getType(self):
        value = trajectory.QPath()
        self.assertEqual(sdurw_core.PropertyType.QPath, sdurw_core.PropertyType.getType(value).getId())

        value = trajectory.PathQPtr()
        self.assertEqual(sdurw_core.PropertyType.QPathPtr, sdurw_core.PropertyType.getType(value).getId())

        value = trajectory.PathTransform3D()
        self.assertEqual(sdurw_core.PropertyType.Transform3DPath, sdurw_core.PropertyType.getType(value).getId())

        value = trajectory.PathTransform3DPtr()
        self.assertEqual(sdurw_core.PropertyType.Transform3DPathPtr, sdurw_core.PropertyType.getType(value).getId())



if __name__ == '__main__':
    unittest.main()