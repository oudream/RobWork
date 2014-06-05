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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCONSTRAINTCORRECTION_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCONSTRAINTCORRECTION_HPP_

/**
 * @file TNTConstraintCorrection.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTConstraintCorrection
 */

#include <list>

// Forward declarations
namespace rw { namespace kinematics { class State; } }

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTConstraint;
class TNTIslandState;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Positional correction of bodies to satisfy the constraints.
 */
class TNTConstraintCorrection {
public:
	//! @brief Constructor.
	TNTConstraintCorrection();

	//! @brief Destructor.
	virtual ~TNTConstraintCorrection();

	/**
	 * @brief Do the correction.
	 * @param constraints [in] vector of constraints to correct.
	 * @param tntstate [in/out] the state that will be updated.
	 * @param rwstate [in] the state of the system.
	 */
	virtual void correct(const std::list<TNTConstraint*>& constraints, TNTIslandState& tntstate, const rw::kinematics::State& rwstate) const;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCONSTRAINTCORRECTION_HPP_ */