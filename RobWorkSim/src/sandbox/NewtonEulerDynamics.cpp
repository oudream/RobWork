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

#include "NewtonEulerDynamics.hpp"

using namespace rw::core::kinematics;
using namespace rw::core::math;
using namespace rw::core::models;
using namespace std;

NewtonEulerDynamics::NewtonEulerDynamics(const SerialDevice& rob, bool print) :
    Z(0, 0, 1), R(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) {
    bodies.resize(0);
    if(print) { std::cout << "[NewtonEulerDynamics] Adding RigidBodies:" << std::endl; }
    robot            = &rob;
    base             = robot->getBase();
    const Frame* end = robot->getEnd();

    bool endisnotbase = true;

    while(endisnotbase) {
        if(end == base) { endisnotbase = false; }

        Frame::const_iterator_pair iter_pair = end->getChildren();
        while(iter_pair.first != iter_pair.second) {
            const Frame* tmp = &(*iter_pair.first);
            // check if frame is a RigidBody
            const RigidBody* body = dynamic_cast<const RigidBody*>(tmp);
            if(body != NULL) {
                if(print) { std::cout << "adding \"" << body->getName() << "\"" << std::endl; }
                bodies.push_back((RigidBody*) body);
            }
            iter_pair.first++;
        }
        end = end->getParent();
    }
    if(print) {
        std::cout << "[NewtonEulerDynamics] List of Rigidbodies: " << std::endl;

        for(unsigned int h = 0; h < bodies.size(); h++) {
            // cur_body =
            std::cout << h << " : " << bodies[h]->getName() << std::endl;
        }
        std::cout << "[NewtonEulerDynamics] Finished adding RigidBodies" << std::endl;
    }

    links = bodies.size();
    w.resize(links + 1);
    wd.resize(links + 1);
    vd.resize(links + 1);
    vdC.resize(links + 1);
    F.resize(links + 1);
    Nout.resize(links + 1);

    f.resize(links + 1);
    n.resize(links + 1);
    tau.resize(links);
}

void NewtonEulerDynamics::execute(State& state, const Q& q, const Q& qd, const Q& qdd,
                                  const Vector3D<double>& w0, const Vector3D<double>& wd0,
                                  const Vector3D<double>& vd0, const Vector3D<double>& f_end,
                                  const Vector3D<double>& n_end, bool print, bool printres) {
    int i, j;

    w[0]   = w0;
    wd[0]  = wd0;
    vd[0]  = vd0;
    vdC[0] = Z * 0.0;

    robot->setQ(q, state);
    base = robot->getBase();

    Frame::const_iterator_pair iter_pair = base->getChildren();

    if(print) {
        std::cout << "[NewtonEulerDynamics] Executing N-E Algorithm " << std::endl;
        std::cout << "[NewtonEulerDynamics] Outward iterations" << std::endl;
    }
    bool first = true;
    /*outward iterations*/
    for(i = 0; i < links; i++) {
        if(print) {
            std::cout << "--------------------------------------------" << std::endl;
            std::cout << "i : " << i << std::endl;
        }
        j = links - 1 - i;

        iter_pair = base->getChildren();
        base      = &(*iter_pair.first);

        Ti = base->getTransform(state);
        // cout<<"[NewtonEulerDynamics] OI - Ti :"<<std::endl;
        if(print) printT(Ti, 1.0e-8);

        cur_body = bodies[j];

        pci = (cur_body->getTransform(state)).P();

        m = cur_body->getMass();

        // cI			= cur_body->getInertia();
        cI = prod(cur_body->getTransform(state).R().m(), cur_body->getInertia());
        // cI			= cur_body->getTransform(state).R() * cI;

        if(print) { std::cout << inverse(Ti).R() << " * " << w[i] << " + " << qd[i] * Z << std::endl; }
        first = false;

        w[i + 1] = inverse(Ti).R() * w[i] + qd[i] * Z;
        if(print) std::cout << w[i + 1] << std::endl << std::endl;

        if(print) {
            std::cout << inverse(Ti).R() << " * " << wd[i] << " + " << inverse(Ti).R() << " * " << w[i]
                 << " x " << qd[i] * Z << " + " << qdd[i] * Z << std::endl;
            std::cout << inverse(Ti).R() * wd[i] << " + " << inverse(Ti).R() * w[i] << " x " << qd[i] * Z
                 << " + " << qdd[i] * Z << std::endl;
            std::cout << inverse(Ti).R() * wd[i] << " + " << cross(inverse(Ti).R() * w[i], qd[i] * Z)
                 << " + " << qdd[i] * Z << std::endl;
        }

        wd[i + 1] = inverse(Ti).R() * wd[i] + cross(inverse(Ti).R() * w[i], qd[i] * Z) + qdd[i] * Z;
        if(print) std::cout << wd[i + 1] << std::endl << std::endl;

        temp = (cross(wd[i], Ti.P()) + cross(w[i], cross(w[i], Ti.P())) + vd[i]);

        vd[i + 1] = inverse(Ti).R() * temp;

        vdC[i + 1] = cross(wd[i + 1], pci) + cross(w[i + 1], cross(w[i + 1], pci)) + vd[i + 1];

        F[i + 1] = m * vdC[i + 1];

        temp = prod(cI, w[i + 1]);

        Nout[i + 1] = prod(cI, wd[i + 1]) + cross(w[i + 1], prod(cI, w[i + 1]));
    }

    if(print) { std::cout << "[NewtonEulerDynamics] Finished outward iterations " << std::endl; }
    if(print || printres) { printout(); }
    if(print) { std::cout << "[NewtonEulerDynamics] Inward iterations " << std::endl; }

    f[links]  = f_end;
    n[links]  = n_end;
    iter_pair = base->getChildren();
    base      = &(*iter_pair.first);
    /*inward iterations*/
    for(i = links; i > 0; i--) {
        if(print) { std::cout << "-------------------------" << std::endl << "i : " << i << std::endl; }
        j        = links - i;
        cur_body = bodies[j];
        pci      = (cur_body->getTransform(state)).P();
        Ti       = base->getTransform(state);
        base     = base->getParent();

        if(print) {
            std::cout << "f" << std::endl;
            std::cout << Ti.R() << " * " << f[i] << " + " << F[i] << std::endl;
            std::cout << Ti.R() * f[i] << " + " << F[i] << std::endl;
        }

        f[i - 1] = Ti.R() * f[i] + F[i];
        if(print) { std::cout << f[i - 1] << std::endl; }

        n[i - 1] = Nout[i] + (Ti.R() * n[i]) + cross(pci, F[i]) + cross(Ti.P(), Ti.R() * f[i]);

        if(print) {
            std::cout << "n" << std::endl;
            std::cout << Nout[i] << " + " << Ti.R() << " * " << n[i] << " + " << pci << " x " << F[i]
                 << " + " << Ti.P() << " x (" << Ti.R() << " * " << f[i] << ")" << std::endl;
            std::cout << Nout[i] << " + " << (Ti.R() * n[i]) << " + " << cross(pci, F[i]) << " + "
                 << cross(Ti.P(), Ti.R() * f[i]) << std::endl;
            std::cout << n[i - 1] << std::endl;
        }
        // tau[i]	= ((inverse(Ti).R())*n[i-1])(2);
        tau[i - 1] = dot(n[i - 1], Z);
        if(print) std::cout << "tau[i]: " << tau[i] << std::endl;
    }
    if(print || printres) { printin(); }
    if(print) {
        std::cout << "[NewtonEulerDynamics] Finished inward iterations " << std::endl;
        std::cout << "[NewtonEulerDynamics] Finished executing N-E algorithm" << std::endl;
    }
}

const std::vector<double>* NewtonEulerDynamics::readTau() {
    // cout<<"[NewtonEulerDynamics] Reading results"<<std::endl;
    return &tau;
}

void NewtonEulerDynamics::printout() {
    unsigned int i;

    // w
    std::cout << "  \t";
    for(i = 0; i < w.size(); i++) { printf("%+2.4f  \t", w[i](0)); }
    std::cout << std::endl;

    std::cout << "w:\t";
    for(i = 0; i < w.size(); i++) { printf("%+2.4f  \t", w[i](1)); }
    std::cout << std::endl;

    std::cout << "  \t";
    for(i = 0; i < w.size(); i++) { printf("%+2.4f  \t", w[i](2)); }
    std::cout << std::endl;
    std::cout << std::endl;

    // wd
    std::cout << "  \t";
    for(i = 0; i < wd.size(); i++) { printf("%+2.4f  \t", wd[i](0)); }
    std::cout << std::endl;

    std::cout << "wd:\t";
    for(i = 0; i < wd.size(); i++) { printf("%+2.4f  \t", wd[i](1)); }
    std::cout << std::endl;

    std::cout << "  \t";
    for(i = 0; i < wd.size(); i++) { printf("%+2.4f  \t", wd[i](2)); }
    std::cout << std::endl;
    std::cout << std::endl;

    // vd
    std::cout << "  \t";
    for(i = 0; i < vd.size(); i++) { printf("%+2.4f  \t", vd[i](0)); }
    std::cout << std::endl;

    std::cout << "vd:\t";
    for(i = 0; i < vd.size(); i++) { printf("%+2.4f  \t", vd[i](1)); }
    std::cout << std::endl;

    std::cout << "  \t";
    for(i = 0; i < vd.size(); i++) { printf("%+2.4f  \t", vd[i](2)); }
    std::cout << std::endl;
    std::cout << std::endl;

    // vdC
    std::cout << "  \t";
    for(i = 0; i < vdC.size(); i++) { printf("%+2.4f  \t", vdC[i](0)); }
    std::cout << std::endl;

    std::cout << "vdC:\t";
    for(i = 0; i < vdC.size(); i++) { printf("%+2.4f  \t", vdC[i](1)); }
    std::cout << std::endl;

    std::cout << "  \t";
    for(i = 0; i < vdC.size(); i++) { printf("%+2.4f  \t", vdC[i](2)); }
    std::cout << std::endl;
    std::cout << std::endl;

    // F

    std::cout << "  \t";
    for(i = 0; i < F.size(); i++) { printf("%+2.4f  \t", F[i](0)); }
    std::cout << std::endl;

    std::cout << "F:\t";
    for(i = 0; i < F.size(); i++) { printf("%+2.4f  \t", F[i](1)); }
    std::cout << std::endl;

    std::cout << "  \t";
    for(i = 0; i < F.size(); i++) { printf("%+2.4f  \t", F[i](2)); }
    std::cout << std::endl;
    std::cout << std::endl;

    // N
    std::cout << "  \t";
    for(i = 0; i < Nout.size(); i++) { printf("%+2.4f  \t", Nout[i](0)); }
    std::cout << std::endl;

    std::cout << "N:\t";
    for(i = 0; i < Nout.size(); i++) { printf("%+2.4f  \t", Nout[i](1)); }
    std::cout << std::endl;

    std::cout << "  \t";
    for(i = 0; i < Nout.size(); i++) { printf("%+2.4f  \t", Nout[i](2)); }
    std::cout << std::endl;
    std::cout << std::endl;
}

void NewtonEulerDynamics::printin() {
    unsigned int i;

    // f
    std::cout << "  \t";
    for(i = 0; i < f.size(); i++) { printf("%+2.4f  \t", f[i](0)); }
    std::cout << std::endl;

    std::cout << "f:\t";
    for(i = 0; i < f.size(); i++) { printf("%+2.4f  \t", f[i](1)); }
    std::cout << std::endl;

    std::cout << "  \t";
    for(i = 0; i < f.size(); i++) { printf("%+2.4f  \t", f[i](2)); }
    std::cout << std::endl;
    std::cout << std::endl;

    // n
    std::cout << "  \t";
    for(i = 0; i < n.size(); i++) { printf("%+2.4f  \t", n[i](0)); }
    std::cout << std::endl;

    std::cout << "n:\t";
    for(i = 0; i < n.size(); i++) { printf("%+2.4f  \t", n[i](1)); }
    std::cout << std::endl;

    std::cout << "  \t";
    for(i = 0; i < n.size(); i++) { printf("%+2.4f  \t", n[i](2)); }
    std::cout << std::endl;
    std::cout << std::endl;

    // tau
    std::cout << "tau:\t";
    for(i = 0; i < tau.size(); i++) { printf("%+2.4f  \t", tau[i]); }
    std::cout << std::endl;
    std::cout << std::endl;
}

void NewtonEulerDynamics::printT(const Transform3D<double>& t, double b) {
    int i, j;
    double a;

    for(i = 0; i < 3; i++) {
        for(j = 0; j < 4; j++) {
            if(j != 3) {
                a = (t.R())(i, j);
                if(fabs(a) < b) { a = 0; }
                std::cout << a << "\t";
            }
            else { std::cout << (t.P())(i); }
        }
        std::cout << std::endl;
    }
    std::cout << "0\t0\t0\t1" << std::endl;
    std::cout << std::endl;
}
