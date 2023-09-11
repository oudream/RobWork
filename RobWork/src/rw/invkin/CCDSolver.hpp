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

#ifndef RW_INVKIN_CCDSOLVER_HPP
#define RW_INVKIN_CCDSOLVER_HPP

/**
 * @file CCDSolver.hpp
 */
#if !defined(SWIG)
#include <rw/core/PropertyMap.hpp>
#include <rw/invkin/IterativeIK.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/math/Transform3D.hpp>
#endif
namespace rw { namespace models {
    class JacobianCalculator;
    class SerialDevice;
}}    // namespace rw::models

namespace rw { namespace invkin {

    /** @addtogroup invkin */
    /*@{*/

    /**
     * \brief This inverse kinematics method is a heuristic search technique called
     * the Cyclic-Coordinate Descent method. The method attempts to minimize position
     * and orientation errors by varying individual joints at a time.
     *
     * Notice that the CCDSolver only work on devices with 1-dof joints.
     */
    class CCDSolver : public rw::invkin::IterativeIK
    {
      public:
        /**
         * @brief Constructor
         */
        CCDSolver(const rw::models::SerialDevice* device, const rw::kinematics::State& state);

#if !defined(SWIGJAVA)

        /**
         * @brief Construct new CCSSolver
         * @note The dimensions will be automatically extracted from the device, using an arbitrary
         * state.
         * @param device [in] the device.
         * @param state [in] the state to use to extract dimensions.
         * @exception rw::core::Exception if device is not castable to SerialDevice
         */
#endif
        CCDSolver(const rw::core::Ptr<const rw::models::SerialDevice> device,
                  const rw::kinematics::State& state);

        /**
         * @brief Sets the maximal size of a local step
         * @param quatlength [in] Maximal length for quartenion
         */
        void setMaxLocalStep(double quatlength);

        /**
         * \copydoc rw::invkin::IterativeIK::solve
         *
         * Example:\n
         * CCDAlgorithm r;\n
         * r.inverseKinematics(device, Ttarget);
         */
        std::vector<rw::math::Q> solve(const rw::math::Transform3D<double>& baseTend,
                                       const rw::kinematics::State& state) const;

        /**
         * @brief performs a local search toward the the target bTed. No via points
         * are generated to support the convergence and robustness.
         * @param bTed [in] the target end pose
         * @param maxError [in] the maximal allowed error
         * @param state [in/out] the starting position for the search. The end position will
         * also be saved in this state.
         * @param maxIter [in] max number of iterations
         * @return true if error is below max error
         * @note the result will be saved in state
         */
        bool solveLocal(const rw::math::Transform3D<double>& bTed, double maxError,
                        rw::kinematics::State& state, int maxIter) const;

        virtual void setCheckJointLimits(bool check){};

        /**
         * @copydoc InvKinSolver::getTCP
         */
        virtual rw::core::Ptr<const rw::kinematics::Frame> getTCP() const;

      private:
        double _maxQuatStep;

        const rw::core::Ptr<const models::SerialDevice> _device;

        core::PropertyMap _properties;

        kinematics::FKRange _fkrange;
        rw::core::Ptr<rw::models::JacobianCalculator> _devJac;
    };

    /*@}*/
}}    // namespace rw::invkin

#endif    // end include guard
