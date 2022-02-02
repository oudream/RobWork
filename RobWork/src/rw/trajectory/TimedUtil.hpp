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

#ifndef RW_TRAJECTORY_TIMEDUTIL_HPP
#define RW_TRAJECTORY_TIMEDUTIL_HPP
#if !defined(SWIG)
#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/Timed.hpp>

#include <rw/math/Q.hpp>
#endif
/**
   @file TimedUtil.hpp
   @brief Class rw::trajectory::TimedUtil
*/

namespace rw { namespace kinematics {
    class State;
}}    // namespace rw::kinematics
namespace rw { namespace models {
    class Device;
}}    // namespace rw::models
namespace rw { namespace models {
    class WorkCell;
}}    // namespace rw::models

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
       @brief Construction of paths of Timed values.
    */
    class TimedUtil
    {
      public:
        /**
           @brief A path of time stamped configurations.

           The time stamp of the first configuration is \b offset, and the time
           for the remaining configurations are computed using the joint speed
           velocities \b speed.
        */
        static rw::trajectory::Path< rw::trajectory::Timed< rw::math::Q > >
        makeTimedQPath (const rw::math::Q& speed, const rw::trajectory::Path< rw::math::Q >& path,
                        double offset = 0);

        /**
           @brief A path of time stamped configurations.

           The time stamp of the first configuration is \b offset , and the time for
           the remaining configurations are computed using the joint speed
           velocities of \b device.
         */
        static rw::trajectory::Path< rw::trajectory::Timed< rw::math::Q > >
        makeTimedQPath (const rw::models::Device& device,
                        const rw::trajectory::Path< rw::math::Q >& path, double offset = 0);

        /**
           @brief A path of time stamped states.

           The time stamp of the first state is \b offset, and the time for the
           remaining states are computed using the maximum joint speed
           velocities of \b workcell.
         */
        static rw::trajectory::Path< rw::trajectory::Timed< rw::kinematics::State > >
        makeTimedStatePath (const rw::models::WorkCell& workcell, const  rw::trajectory::Path<rw::kinematics::State>& path,
                            double offset = 0);

        /**
           @brief A path of time stamped states.

           The time stamp of the first state is \b offset, and the time for the
           remaining states are computed using the maximum joint speed
           velocities of \b device.
        */
        static rw::trajectory::Path< rw::trajectory::Timed< rw::kinematics::State > >
        makeTimedStatePath (const rw::models::Device& device,
                            const rw::trajectory::Path< rw::math::Q >& path,
                            const rw::kinematics::State& state, double offset = 0);
    };

    /*@}*/
}}    // namespace rw::trajectory

#endif    // end include guard
