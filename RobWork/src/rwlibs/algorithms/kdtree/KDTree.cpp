/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "KDTree.hpp"

#include <rw/math/Vector3D.hpp>
//#include <rw/math/Pose6D.hpp>

namespace rwlibs { namespace algorithms {
    template class KDTree<rw::math::Vector3D<>, 3>;
}}    // namespace rwlibs::algorithms

// template class KDTree<rw::math::Pose6D<>, 6>;
