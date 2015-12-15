/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "RWPEFrictionModelCoulomb.hpp"
#include "RWPEContact.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::rwpe;

RWPEFrictionModelCoulomb::RWPEFrictionModelCoulomb():
	_mu(0)
{
}

RWPEFrictionModelCoulomb::RWPEFrictionModelCoulomb(const PropertyMap &map) {
	_mu = map.get<double>("mu",-1);
	if (_mu < 0)
		_mu = map.get<double>("Mu",-1);
}

RWPEFrictionModelCoulomb::RWPEFrictionModelCoulomb(double mu):
	_mu(mu)
{
}

RWPEFrictionModelCoulomb::~RWPEFrictionModelCoulomb() {
}

const RWPEFrictionModel* RWPEFrictionModelCoulomb::withProperties(const PropertyMap &map) const {
	if (!map.has("mu"))
		RW_THROW("RWPEFrictionModelCoulomb (withProperties): could not create model as property \"mu\" was not found in map.");
	return new RWPEFrictionModelCoulomb(map);
}

RWPEFrictionModel::DryFriction RWPEFrictionModelCoulomb::getDryFriction(
	const RWPEContact& contact,
	const RWPEIslandState& islandState,
	const State& rwstate,
	const RWPEFrictionModelData* data) const
{
	// Find the current relative velocity:
	const Vector3D<> n = contact.getNormalW(islandState);
	const Vector3D<> velP = contact.getVelocityParentW(islandState,rwstate).linear();
	const Vector3D<> velC = contact.getVelocityChildW(islandState,rwstate).linear();
	const Vector3D<> relVel = velP-velC;
	const Vector3D<> relVelDir = normalize(relVel-dot(relVel,n)*n);

	DryFriction res;
	if (_mu > 0) {
		res.enableTangent = true;
		res.tangent = _mu;
		res.tangentDirection = relVelDir;
	}
	return res;
}

Wrench6D<> RWPEFrictionModelCoulomb::getViscuousFriction(
	const RWPEContact& contact,
	const RWPEIslandState& islandState,
	const State& rwstate,
	const RWPEFrictionModelData* data) const
{
	return Wrench6D<>(Vector3D<>::zero(),Vector3D<>::zero());
}
