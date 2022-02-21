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
import sdurw_core


class PropertyTypeCore(unittest.TestCase):

    def test_PropertyType(self):
        proptype = sdurw_core.PropertyType()
        self.assertEqual(sdurw_core.PropertyType.Unknown, proptype.getId())             # Unknown = -1

        proptype = sdurw_core.PropertyType(int(6))
        self.assertEqual(sdurw_core.PropertyType.Int, proptype.getId())


    def test_getType(self):
        map = sdurw_core.PropertyMap()
        x = sdurw_core.PropertyType.getType(map).getId()
        self.assertEqual(sdurw_core.PropertyType.PropertyMap, sdurw_core.PropertyType.getType(map).getId())

        #map = sdurw_core.PropertyMap.ptr()

        #std::vector<rw::core::Ptr<rw::core::PropertyValueBase> > list;

        print("\n Mangler Resten")


if __name__ == '__main__':
    unittest.main()
