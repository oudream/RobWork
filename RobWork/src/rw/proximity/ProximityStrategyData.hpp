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

#ifndef RW_PROXIMITY_PROXIMITYSTRATEGYDATA_HPP_
#define RW_PROXIMITY_PROXIMITYSTRATEGYDATA_HPP_

/**
 * @file rw/proximity/ProximityStrategyData.hpp
 */

#include "CollisionStrategy.hpp"
#include "DistanceMultiStrategy.hpp"
#include "DistanceStrategy.hpp"
#include "ProximityCache.hpp"
#include "ProximityModel.hpp"

namespace rw { namespace proximity {
    //! @addtogroup proximity
    // @{

    // for backward compatability
    typedef CollisionStrategy::Result CollisionResult;
    typedef DistanceStrategy::Result DistanceResult;
    typedef DistanceMultiStrategy::Result MultiDistanceResult;

    /***
     * @brief A generic object for containing data that is essential in
     * proximity queries between two ProximityModels.
     *
     * The ProximityData object is used for Collision queries, tolerance and distance queries
     * between two ProximityModels. example: collision result, cached variables for faster collision
     * detection,
     *
     */
    class ProximityStrategyData
    {
      public:
        typedef rw::common::Ptr< ProximityStrategyData > Ptr;
        /**
         * @brief Create Empty ProximityData
         */
        ProximityStrategyData () :
            rel_err (0), abs_err (0), _colQueryType (CollisionStrategy::FirstContact),
            _collides (false)
        {
            _collisionData.clear ();
            _distanceData.clear ();
            _multiDistanceData.clear ();
        }

        /**
         * @brief Copy Constructor
         */
        ProximityStrategyData (const ProximityStrategyData& data) :
            rel_err (data.rel_err), abs_err (data.abs_err), _colQueryType (data._colQueryType),
            _collides (data._collides), _collisionData (data._collisionData),
            _distanceData (data._distanceData), _multiDistanceData (data._multiDistanceData),
            _cache (data._cache)
        {}

        /**
         * @brief Get the underlying cache
         * @return pointer to cache
         */
        ProximityCache::Ptr& getCache () { return _cache; }

        // CollisionData interface
        /**
         * @brief get the result from the collision check
         * @return Result of Collision strategy if available
         */
        CollisionStrategy::Result& getCollisionData () { return _collisionData; }

        /**
         * @brief get the result from the collision check
         * @return Result of Collision strategy if available
         */
        const CollisionStrategy::Result& getCollisionData () const { return _collisionData; }

        /**
         * @brief was collision check in collision
         * @return true if in collision
         */
        bool inCollision () { return _collides; }

        /**
         * @brief set the Collision Query type
         * @param qtype [in] the used Query type
         */
        void setCollisionQueryType (CollisionStrategy::QueryType qtype) { _colQueryType = qtype; }

        /**
         * @brief Get the used Collision Query type
         * @return Querytype
         */
        CollisionStrategy::QueryType getCollisionQueryType () const { return _colQueryType; }

        // Distance query interfaces
        /**
         * @brief get The result of a distance query
         * @return result of a distance query
         */
        DistanceStrategy::Result& getDistanceData () { return _distanceData; }

        // Distance query interfaces
        /**
         * @brief get The result of a distance query
         * @return result of a distance query
         */
        const DistanceStrategy::Result& getDistanceData () const { return _distanceData; }

        // For Multi distance interface
        /**
         * @brief get The result of a multi distance query
         * @return result of a distance query
         */
        DistanceMultiStrategy::Result& getMultiDistanceData () { return _multiDistanceData; }

        /**
         * @brief get The result of a multi distance query
         * @return result of a distance query
         */
        const DistanceMultiStrategy::Result& getMultiDistanceData () const
        {
            return _multiDistanceData;
        }

        //! @brief relative acceptable error
        double rel_err;
        //! @brief absolute acceptable error
        double abs_err;

      private:
        CollisionStrategy::QueryType _colQueryType;

        // Following belongs to CollisionData interface
        //! true if the models are colliding
        bool _collides;
        //! @brief the features that where colliding
        CollisionStrategy::Result _collisionData;
        DistanceStrategy::Result _distanceData;
        DistanceMultiStrategy::Result _multiDistanceData;

        //! @brief proximity cache
        ProximityCache::Ptr _cache;
    };
    // @}
}}    // namespace rw::proximity

#endif /* RW_PROXIMITY_PROXIMITYSTRATEGYDATA_HPP_ */
