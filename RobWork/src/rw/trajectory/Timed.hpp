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

#ifndef RW_TRAJECTORY_TIMED_HPP
#define RW_TRAJECTORY_TIMED_HPP

/**
   @file Timed.hpp
   @brief Class rw::interpolator::Timed
*/
#if !defined(SWIG)
#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#endif
namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
       @brief A tuple of (time, value).
     */
    template< class T > class Timed
    {
      public:
        /**
           Constructor
         */
        Timed (double time, const T& value) : _time (time), _value (value) {}

        /**
           Default constructor
        */
        Timed () : _time (0.0), _value () {}

#ifdef SWIG
        SWIG_IGNORE (getTime () const);
#endif

        /**
           @brief The time
        */
        double& getTime () { return _time; }
        
        double getTime () const { return _time; }

#if !defined(SWIGJAVA)
        /**
           @brief The value
        */
        const T& getValue () const { return _value; }
#endif

        T& getValue () { return _value; }

#ifdef SWIG
         SWIG_SET_TIME();
#endif

      private:
        double _time;
        T _value;
    };

    /**
       @brief A tuple of (time, value).
    */
    template< class T > rw::trajectory::Timed< T > makeTimed (double time, const T& value)
    {
        return rw::trajectory::Timed< T > (time, value);
    }

    //! A tuple of (time, Q).
    typedef rw::trajectory::Timed< rw::math::Q > TimedQ;

    //! A tuple of (time, State). See rw::trajectory::Timed<t> template for more info.
    typedef rw::trajectory::Timed< rw::kinematics::State > TimedState;

    /*@}*/
}}    // namespace rw::trajectory

#endif    // end include guard
