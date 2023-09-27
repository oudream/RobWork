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

#include "ScapePoseFormat.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <fstream>

using namespace rwsim::dynamics;
using namespace rwsim;
using namespace std;
using namespace rw::kinematics;
using namespace rw::math;

// saving poses to SCAPE file
void ScapePoseFormat::savePoses(const std::string& FileNameAndPath,
                                const std::vector<RigidBody::Ptr>& bodies,
                                const rw::kinematics::State& state, const std::string& ObjectName,
                                const std::string& SimulationDescription) {
    ofstream fout(FileNameAndPath.c_str());
    fout << "CObjectPoseContainer  Version: 0.1" << std::endl;
    fout << "  Description: " << SimulationDescription << std::endl;
    fout << "  IsObjectsInitialized: 1" << std::endl;
    fout << "    CPart  Version: 0.20" << std::endl;
    fout << "      IsInitialized: 1" << std::endl;
    fout << "        Name: " << ObjectName << std::endl;
    fout << "        Description: EMPTY" << std::endl;
    fout << "        CADFileName: EMPTY" << std::endl;
    fout << "        Symmetry: UNDEFINED " << std::endl;
    fout << "  IsPosesInitialized: 1" << std::endl;
    for(size_t i = 0; i < bodies.size(); i++) {
        rw::core::Ptr<rw::kinematics::Frame> frame = bodies[i]->getBodyFrame();
        if((frame != NULL) && (bodies[i].cast<RigidBody>())) {
            Vector3D<double>& P   = frame->getTransform(state).P();
            Rotation3D<double>& R = frame->getTransform(state).R();
            fout.precision(10);
            fout.setf(std::ios::fixed, std::ios::floatfield);
            fout << "    CObjectPose  Pose: (" << P(0) << "," << P(1) << "," << P(2) << ")"
                 << "  XDir: (" << R(0, 0) << "," << R(1, 0) << "," << R(2, 0) << ")"
                 << "  YDir: (" << R(0, 1) << "," << R(1, 1) << "," << R(2, 1) << ")"
                 << " Active: 1 Name: " << ObjectName << std::endl;
        }
    }
    fout.flush();
    fout.close();
}

void ScapePoseFormat::savePoses(const std::string& FileNameAndPath,
                                const std::vector<dynamics::RigidBody::Ptr>& bodies,
                                const std::vector<rw::kinematics::State> states,
                                const std::string& ObjectName,
                                const std::string& SimulationDescription) {
    ofstream fout(FileNameAndPath.c_str());
    fout << "CObjectPoseContainer  Version: 0.1" << std::endl;
    fout << "  Description: " << SimulationDescription << std::endl;
    fout << "  IsObjectsInitialized: 1" << std::endl;
    fout << "    CPart  Version: 0.20" << std::endl;
    fout << "      IsInitialized: 1" << std::endl;
    fout << "        Name: " << ObjectName << std::endl;
    fout << "        Description: EMPTY" << std::endl;
    fout << "        CADFileName: EMPTY" << std::endl;
    fout << "        Symmetry: UNDEFINED " << std::endl;
    fout << "  IsPosesInitialized: 1" << std::endl;
    for(size_t j = 0; j < states.size(); j++) {
        const State& state = states[j];
        for(size_t i = 0; i < bodies.size(); i++) {
            rw::core::Ptr<rw::kinematics::Frame> frame = bodies[i]->getBodyFrame();
            if((frame != NULL) && (bodies[i].cast<RigidBody>())) {
                Vector3D<double>& P   = frame->getTransform(state).P();
                Rotation3D<double>& R = frame->getTransform(state).R();
                fout.precision(10);
                fout.setf(std::ios::fixed, std::ios::floatfield);
                fout << "    CObjectPose  Pose: (" << P(0) << "," << P(1) << "," << P(2) << ")"
                     << "  XDir: (" << R(0, 0) << "," << R(1, 0) << "," << R(2, 0) << ")"
                     << "  YDir: (" << R(0, 1) << "," << R(1, 1) << "," << R(2, 1) << ")"
                     << " Active: 1 Name: " << ObjectName << "_m" << j << std::endl;
            }
        }
    }
    fout.flush();
    fout.close();
}
