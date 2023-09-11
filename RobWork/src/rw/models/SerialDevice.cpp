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

#include "SerialDevice.hpp"

#include "DependentJoint.hpp"
#include "Joint.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <vector>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

namespace {

std::vector<Joint*> getJointsFromFrames(const std::vector<Frame*>& frames) {
    std::vector<Joint*> active;

    typedef std::vector<Frame*>::const_iterator I;
    for(I p = frames.begin(); p != frames.end(); ++p) {
        rw::core::Ptr<Frame> frame = *p;
        Joint::Ptr joint           = frame.cast<Joint>();
        if((joint != NULL && joint->isActive()) || frame.cast<DependentJoint>() != NULL)
            active.push_back(joint.get());
    }
    return active;
}

// From the root 'first' to the child 'last' inclusive.
std::vector<Frame*> getChain(rw::core::Ptr<Frame> first, rw::core::Ptr<Frame> last,
                             const State& state) {
    std::vector<Frame*> init = Kinematics::parentToChildChain(first, last, state);

    init.push_back(last.get());
    return init;
}
}    // namespace

SerialDevice::SerialDevice(rw::core::Ptr<Frame> first, rw::core::Ptr<Frame> last,
                           const std::string& name, const State& state) :
    JointDevice(name, first, last, getJointsFromFrames(getChain(first, last, state)), state),
    _kinematicChain(getChain(first, last, state)) {}

const std::vector<Frame*>& SerialDevice::frames() const {
    return _kinematicChain;
}

SerialDevice::SerialDevice(const std::vector<Frame*>& serialChain, const std::string& name,
                           const State& state) :
    JointDevice(name, serialChain.front(), serialChain.back(), getJointsFromFrames(serialChain),
                state),
    _kinematicChain(serialChain) {}
