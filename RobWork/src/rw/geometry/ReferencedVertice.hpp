/********************************************************************************
 * Copyright 2022 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_REFERENCEDVERTICE_HPP_
#define RW_GEOMETRY_REFERENCEDVERTICE_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/geometry/CSGEngine.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw { namespace geometry {

    /**
     * @brief The ReferencedVertice contains a reference to the mesh it has been created from.
     * Therefore any changes to the vertice wil be reflected in the original mesh
     */
    class ReferencedVertice
    {
      public:
        /**
         * @brief Construct a new Referenced Vertice.
         * @param ref pointer to the referenced mesh
         * @param vertice  index of the referenced vertice
         */
        ReferencedVertice (TriMeshData::Ptr ref, uint32_t vertice);

        /**
         * @brief access to the index this mesh is refering to
         * @return uint32_t&
         */
        uint32_t& idx () { return _verIndex; }


        /**
         * @brief access to the index this mesh is refering to
         * @return uint32_t&
         */
        uint32_t idx () const { return _verIndex; }

        /**
         * @brief acces the vertice data
         * @param i index for the vertix corrdinate x=0 y=1 z=2
         * @return double&
         */
        double& operator[] (size_t i) const;

        /**
         * @brief Copy the data of a vector into the vertice
         * @param rhs the vector to copy
         * @return this Vertice
         */
        ReferencedVertice& operator= (rw::math::Vector3D< double > rhs);

        /**
         * @brief Copy the data of a vector into the vertice
         * @param rhs the vector to copy
         * @return this Vertice
         */
        ReferencedVertice& operator= (rw::math::Vector3D< float > rhs);

        /**
         * @brief apply a transform to this vertice.
         * @param rhs the transform to apply
         * @return this Vertice
         */
        ReferencedVertice& operator*= (rw::math::Transform3D< double > rhs);

        /**
         * @brief Serilization operator for converting to string
         * @param os [in/out] the output stream
         * @param v [in] the vertice
         * @return std::ostream&
         */
        friend std::ostream& operator<< (std::ostream& os, const ReferencedVertice& v);

        //! @brief implicit convertion to Vector3D
        operator rw::math::Vector3D< double > () const;

        //! @brief implicit convertion to Vector3D
        operator rw::math::Vector3D< float > () const;

      private:
        friend class ReferencedTriangle;
        friend class ReferencedEdge;

        ReferencedVertice ();
        TriMeshData::Ptr _mesh;
        uint32_t _verIndex;
    };
}}    // namespace rw::geometry
#endif