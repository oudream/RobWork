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

#ifndef RW_PROXIMITY_COLLISIONRESULT_HPP_
#define RW_PROXIMITY_COLLISIONRESULT_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/proximity/ProximityModel.hpp>

namespace rw { namespace proximity {
    /**
     * @brief result of a single collision pair
     *
     * A collision result is one or all colliding triangles between two objects which may have
     * several geometries attached.
     * The collision result does not have access to the actual triangle meshes of the geometries
     * so to extract the actual contact location the user has to supply the triangles meshes of
     * the geometries himself.
     */
    struct CollisionResult
    {
        //! @brief reference to the first model
        ProximityModel::Ptr a;

        //! @brief reference to the second model
        ProximityModel::Ptr b;

        //! @brief a collision pair of
        struct CollisionPair
        {
            //! @brief Default constructor
            CollisionPair () : geoIdxA (0), geoIdxB (0), startIdx (0), size (0) {}
            
            //! @brief  Copy Constructor
            CollisionPair (const CollisionPair& copy) :
                geoIdxA (copy.geoIdxA), geoIdxB (copy.geoIdxB), startIdx (copy.startIdx),
                size (copy.size)
            {}

            //! @brief geometry index
            int geoIdxA, geoIdxB;
            /**
             *  @brief indices into the geomPrimIds array, which means that inidicies
             * [_geomPrimIds[startIdx];_geomPrimIds[startIdx+size]] are the colliding primitives
             * between geometries geoIdxA and geoIdxB
             */
            int startIdx, size;
        };

        //! @brief transformation from a to b
        rw::math::Transform3D< double > _aTb;

        //! @brief the collision pairs
        std::vector< CollisionPair > _collisionPairs;

        /**
         * @brief indices of triangles/primitives in geometry a and b that are colliding
         * all colliding triangle indices are in this array also those that are from different
         * geometries
         */
        std::vector< std::pair< int, int > > _geomPrimIds;

        int _nrBVTests, _nrPrimTests;

        int getNrPrimTests () { return _nrPrimTests; }
        int getNrBVTests () { return _nrBVTests; }

        /**
         * @brief clear all result values
         */
        void clear ()
        {
            a    = NULL;
            b    = NULL;
            _aTb = rw::math::Transform3D< double >::identity ();
            _collisionPairs.clear ();
            _geomPrimIds.clear ();
            _nrBVTests   = 0;
            _nrPrimTests = 0;
        }
    };
}}    // namespace rw::proximity

#endif