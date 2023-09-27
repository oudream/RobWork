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

#include "ODEVelocityDevice.hpp"

#include "ODEJoint.hpp"
#include "ODESimulator.hpp"

#include <rw/math/MetricUtil.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>
#include <rwsim/dynamics/BodyUtil.hpp>

#include <ode/ode.h>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;

namespace {

bool equalSign(double v1, double v2) {
    return (v1 < 0 && v2 < 0) || (v1 > 0 && v2 > 0);
}

}    // namespace
/*
ODEVelocityDevice::ODEVelocityDevice(
            RigidDevice *rdev,
            std::vector<ODEJoint*> odeJoints,
            Q maxForce):
        _rdev(rdev),
        _odeJoints(odeJoints),
        _maxForce(maxForce)
{

}
*/
ODEVelocityDevice::~ODEVelocityDevice() {
    for(ODEJoint* joint : _odeJoints) { delete joint; }

    for(ODEBody* body : _ode_bodies) { delete body; }
}

void ODEVelocityDevice::reset(rw::kinematics::State& state) {
    rw::math::Q q    = _rdev->getModel().getQ(state);
    rw::math::Q flim = _rdev->getMotorForceLimits();
    int qi           = 0;
    for(size_t i = 0; i < _odeJoints.size(); i++) {
        _odeJoints[i]->setVelocity(0);

        bool depends = _odeJoints[i]->isDepend();
        if(depends) { continue; }

        _odeJoints[i]->setAngle(q(qi));
        _odeJoints[i]->setMaxForce(flim(qi));

        qi++;
    }
    for(size_t i = 0; i < _odeJoints.size(); i++) {
        bool depends = _odeJoints[i]->isDepend();

        if(depends) {
            double angle = _odeJoints[i]->getOwner()->getAngle();
            _odeJoints[i]->setAngle(angle);
        }
        _odeJoints[i]->reset(state);
    }
}
namespace {
int sign(double val) {
    if(val < 0) return -1;
    return 1;
}
}    // namespace

void ODEVelocityDevice::update(const rwlibs::simulation::Simulator::UpdateInfo& info,
                               rw::kinematics::State& state) {
    double dt = info.dt;
    _lastDt   = info.dt_prev;

    bool fmaxChanged       = false;
    bool motormodesChanged = false;
    if(!info.rollback) {
        // check if any control modes have been changed
        std::vector<RigidDevice::MotorControlMode> mstate = _rdev->getMotorModes(state);
        for(size_t i = 0; i < mstate.size(); i++) {
            motormodesChanged |= mstate[i] != _modes[i];
            _modes[i] = mstate[i];
        }

        // check if force limits have changed
        rw::math::Q flim = _rdev->getMotorForceLimits();
        if(MetricUtil::dist1(flim, _maxForce) > 0.0001) {
            fmaxChanged = true;
            _maxForce   = flim;
        }
    }

    _lastQ = _rdev->getModel().getQ(state);

    rw::math::Q targets = _rdev->getMotorTargets(state);
    // rw::math::Q velQ = _rdev->getVelocity(state);
    // rw::math::Q accLim = _rdev->getModel().getAccelerationLimits();
    // std::cout << velQ << "\n";

    int qi = 0;
    for(size_t i = 0; i < _odeJoints.size(); i++) {
        // dependend joints need to be handled separately
        bool depends = _odeJoints[i]->isDepend();

        if(depends) { continue; }
        // TODO: make sure to stay within the actual acceleration limits
        // double vel = velQ(qi);
        // double avel = _odeJoints[i]->getActualVelocity();
        // double acc = (vel-avel)/dt;
        // std::cout << avel << ",";
        // if( fabs(acc)>accLim(qi) )
        //	acc = sign(acc)*accLim(qi);
        // vel = acc*dt+avel;
        // std::cout << accLim(qi) << ",";

        if(fmaxChanged) _odeJoints[i]->setMaxForce(_maxForce(qi));

        if(_modes[qi] == RigidDevice::Velocity) {
            if(motormodesChanged) _odeJoints[i]->setMotorEnabled(true);
            _odeJoints[i]->setVelocity(targets[qi]);
        }
        else {
            // mode is force mode
            if(motormodesChanged) _odeJoints[i]->setMotorEnabled(false);
            _odeJoints[i]->setForce(targets[qi]);
            // std::cout << targets[qi] << " ; ";
        }
        qi++;
    }
    // std::cout  << " \n ";

    // we now handle the dependent joints
    for(size_t i = 0; i < _odeJoints.size(); i++) {
        bool depends = _odeJoints[i]->isDepend();

        // dependend joints need to be handled separately
        if(depends) {
            double oa  = _odeJoints[i]->getOwner()->getAngle();
            double ov  = _odeJoints[i]->getOwner()->getVelocity();
            double s   = _odeJoints[i]->getScale();
            double off = _odeJoints[i]->getOffset();

            // double v = _odeJoints[i]->getVelocity();
            double a = _odeJoints[i]->getAngle();

            // the dependent joint need to be controlled such that the position/angle
            // in the next timestep will match that of the other joint

            // so first we look at the current position error. This should be
            // cancelled by adding a velocity

            double aerr  = (oa * s + off) - a;
            double averr = aerr / dt;    // velocity that will cancel the error

            RW_ASSERT(_odeJoints[i]);
            rw::models::DependentPrismaticJoint* depJoint = NULL;
            if(_odeJoints[i]->getJoint() != NULL)
                depJoint =
                    dynamic_cast<rw::models::DependentPrismaticJoint*>(_odeJoints[i]->getJoint());

            if(depJoint != NULL) {
                // specific PG70 solution

                // double aerr_pg70 = (oa*s+off)*2-a;
                _odeJoints[i]->setVelocity(2 * ov * s /*+ aerr_pg70/dt*/);

                double oaerr_n = ((a / 2) - off) / s - oa;
                _odeJoints[i]->getOwner()->setVelocity(0.5 * oaerr_n / dt);

            }
            else {
                //_odeJoints[i]->setAngle(oa*s+off);
                // double averr = ov*s;
                // now we add the velocity that we expect the joint to have
                // averr += ov*s;

                // general solution
                //_odeJoints[i]->getOwner()->setVelocity(ov-0.1*averr/s);
                _odeJoints[i]->setVelocity(ov * s + averr);
            }
        }
    }
}

void ODEVelocityDevice::postUpdate(rw::kinematics::State& state) {
    rw::math::Q velQ      = _rdev->getJointVelocities(state);
    rw::math::Q q         = _rdev->getModel().getQ(state);
    rw::math::Q actualVel = velQ;
    int qi                = 0;
    for(size_t i = 0; i < _odeJoints.size(); i++) {
        if(_odeJoints[i]->isDepend()) { continue; }
        actualVel(qi) = _odeJoints[i]->getActualVelocity();
        q(qi) = _odeJoints[i]->getAngle();
        qi++;
    }

    _rdev->getModel().setQ(q, state);
    _rdev->setJointVelocities(actualVel, state);

}

ODEVelocityDevice::ODEVelocityDevice(ODEBody* base, RigidDevice* rdev,
                                     const rw::kinematics::State& state, ODESimulator* sim) :
    _rdev(rdev),
    _modes(rdev->getMotorForceLimits().size(), RigidDevice::Velocity), _sim(sim) {
    // we use hashspace here because devices typically have
    // relatively few bodies
    dSpaceID space = dHashSpaceCreate(sim->getODESpace());

    _maxForce = rdev->getMotorForceLimits();

    init(rdev, state, space, base);
}
namespace {
void addToMap(ODEBody* b, std::map<Frame*, ODEBody*>& frameToODEBody) {
    std::vector<Frame*> frames = b->getRwBody()->getFrames();
    for(Frame* f : frames) { frameToODEBody[f] = b; }
}

Joint* getParentJoint(Frame* f, const State& state) {
    Joint* j      = dynamic_cast<Joint*>(f);
    Frame* parent = f;
    while(j == NULL && parent != NULL) {
        j      = dynamic_cast<Joint*>(parent);
        parent = f->getParent(state);
    }
    return j;
}

}    // namespace
void ODEVelocityDevice::init(RigidDevice* rdev, const rw::kinematics::State& state,
                             dSpaceID spaceId, ODEBody* baseODEBody) {
    // dBodyID baseBodyID = baseODEBody->getBodyID();
    std::map<Frame*, ODEBody*> frameToODEBody;

    addToMap(baseODEBody, frameToODEBody);

    std::map<Joint*, Body::Ptr> jointChildMap;
    std::map<Joint*, Body::Ptr> jointParentMap;

    std::map<Body::Ptr, Body::Ptr> childToParentMap;
    std::vector<std::pair<Body::Ptr, Body::Ptr>> childToParentList;
    baseODEBody->setTransform(state);
    // first we create rigid bodies from all of the links of the RigidDevice
    for(Body::Ptr body : rdev->getLinks()) {
        // std::cout << "LINK: " << body->getName() << std::endl;
        ODEBody* odebody = ODEBody::makeRigidBody(body, spaceId, _sim);
        odebody->setTransform(state);
        //_sim->addODEBody(odebody);
        _ode_bodies.push_back(odebody);
        addToMap(odebody, frameToODEBody);
        if(body->getBodyFrame() == rdev->getKinematicModel()->getBase()) {
            childToParentMap[body] = NULL;    // base
            continue;
        }
        // locate the parent body
        Body::Ptr jparent = BodyUtil::getParentBody(body, _sim->getDynamicWorkCell(), state);
        if(jparent == NULL)
            RW_THROW("The body \"" << body->getName()
                                   << "\" does not seem to have any parent body!");
        childToParentMap[body] = jparent;    // base
        childToParentList.push_back(std::make_pair(body, jparent));
    }

    std::map<Joint*, ODEJoint*> jointMap;
    // now locate all joints connecting the child-parent body pairs
    typedef std::pair<Body::Ptr, Body::Ptr> BodyPair;
    for(BodyPair bodyPair : childToParentList) {
        std::vector<Frame*> chain = Kinematics::parentToChildChain(
            bodyPair.second->getBodyFrame(), bodyPair.first->getBodyFrame(), state);
        chain.push_back(bodyPair.first->getBodyFrame());
        std::vector<Joint*> joints;
        for(int i = 1; i < (int) chain.size(); i++) {
            Joint* joint = dynamic_cast<Joint*>(chain[i]);
            if(joint != NULL) {
                // connect this joint to the parent body and if
                joints.push_back(joint);
            }
        }

        ODEBody* parent = frameToODEBody[bodyPair.second->getBodyFrame()];
        for(int i = 0; i < (int) joints.size(); i++) {
            // the child of the joint is either bodyPair.first or another joint in which case we
            // need to make an ODEBody
            ODEBody* child = frameToODEBody[bodyPair.first->getBodyFrame()];

            if(child == NULL) {
                // in case not bodies are placed between two joints then add a virtual body...
                // TODO: this is somewhat of a hack please remove/change/fix
                // the user should specify links in the DWC
                std::cout << "creating virtual body" << std::endl;
                dBodyID bTmp = dBodyCreate(_sim->getODEWorldId());
                ODEUtil::setODEBodyMass(bTmp,
                                        0.01,
                                        Vector3D<>(0, 0, 0),
                                        InertiaMatrix<>::makeSolidSphereInertia(0.01, 1));
                child = new ODEBody(bTmp, joints[i]);
                child->setTransform(state);
                _sim->addODEBody(child);
            }

            ODEJoint* odeJoint = new ODEJoint(joints[i], parent, child, _sim, state);
            _sim->addODEJoint(odeJoint);
            jointMap[joints[i]] = odeJoint;
        }
    }

    // in the end we fix the force limits
    std::vector<ODEJoint*> odeJoints;
    Q maxForce                        = rdev->getMotorForceLimits();
    size_t i                          = 0;
    rw::models::JointDevice::Ptr jdev = rdev->getJointDevice();
    for(Joint* joint : jdev->getJoints()) {
        ODEJoint* odeJoint = jointMap[joint];
        RW_ASSERT(odeJoint != NULL);
        _odeJoints.push_back(odeJoint);
        odeJoint->setMaxForce(maxForce(i));
        i++;
    }
}
