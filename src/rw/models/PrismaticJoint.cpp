/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "PrismaticJoint.hpp"

#include <rw/kinematics/State.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

PrismaticJoint::PrismaticJoint(
    const std::string& name,
    const Transform3D<>& transform)
    :
    Joint(name),
    _transform(transform)
{}

Transform3D<> PrismaticJoint::getPrismaticTransform(
    const Transform3D<>& displacement, double q)
{
    Transform3D<> move = Transform3D<>::identity();
    move(2, 3) = q;
    return displacement * move;
}

void PrismaticJoint::getJointValueTransform(
    const Transform3D<>& parent,
    double q,
    Transform3D<>& result) const
{
    Rotation3D<>::rotationMultiply(parent.R(), _transform.R(), result.R());

    const double bx = _transform.P()(0);
    const double by = _transform.P()(1);
    const double bz = _transform.P()(2);

    const double b02 = _transform.R()(0, 2);
    const double b12 = _transform.R()(1, 2);
    const double b22 = _transform.R()(2, 2);
    const Vector3D<> p(bx + b02 * q, by + b12 * q, bz + b22 * q);

    Rotation3D<>::rotationVectorMultiply(parent.R(), p, result.P());
    result.P() += parent.P();
}

void PrismaticJoint::doGetTransform(
    const Transform3D<>& parent,
    const State& state,
    Transform3D<>& result) const
{
    getJointValueTransform(parent, *getQ(state), result);
}

Transform3D<> PrismaticJoint::getTransform(const State& state) const
{
    return getPrismaticTransform(_transform, *getQ(state));
}
