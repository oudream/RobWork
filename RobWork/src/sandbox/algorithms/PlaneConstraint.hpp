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
 
 

#ifndef RW_ALGORITHMS_PlaneConstraint_HPP
#define RW_ALGORITHMS_PlaneConstraint_HPP



/**
 * @file PlaneConstraint.hpp
 */

#include <rw/math/Vector3D.hpp>

#include "ConstraintModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A plane constraint model.
 * 
 * Describes a plane constraint, i.e. a surface.
 */
class PlaneConstraint : public ConstraintModel {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<PlaneConstraint> Ptr;
		
		//! @copydoc ConstraintModel::MinSamples
		static const int MinSamples = 3; // ??
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		PlaneConstraint() {};
		
		//! @brief Destructor.
		virtual ~PlaneConstraint() {};

	public: // methods
		//! @copydoc sandbox::algorithms::RANSACModel::fitError
		virtual double fitError(ConstraintSample sample) const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::invalid
		virtual bool invalid() const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::refit
		virtual void refit() const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::getMinReqData
		virtual int getMinReqData() const { return MinSamples; }
		
		//! @copydoc ConstraintModel::update
		virtual void update(ConstraintSample sample);
	
	protected: // body
		// how to represent a surface?
};



}} // /namespaces

#endif // include guard