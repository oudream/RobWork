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

# MANGLER AT IMPLEMENTERE DETTE:
#namespace {
#    void listen(int* cnt, PropertyBase* const pbase) {
#        (*cnt)++;
#    }
#    void listenValue(int* cnt, PropertyValueBase* const pbase) {
#        (*cnt)++;
#    }
#}


class Property(unittest.TestCase):

    def test_double(self):
        val = 0.1
        # Test ordinary constructor, clone() and standard interface functions
        propertyA = sdurw_core.Property("TestId", "Desc", val)

        self.assertEqual("TestId", propertyA.getIdentifier())
        self.assertEqual("Desc", propertyA.getDescription())
        self.assertEqual(0.1, propertyA.getValue())
        self.assertEqual(sdurw_core.PropertyType.Double, propertyA.getType().getId())
        self.assertEqual(0.1, propertyA.getPropertyValue().getValue())
        self.assertEqual(sdurw_core.PropertyType.Double, propertyA.getPropertyValue().getType().getId())        
        clone = propertyA.clone()


if __name__ == '__main__':
    unittest.main()
