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

#ifndef RW_GEOMETRY_REFERENCEDEDGE_HPP_
#define RW_GEOMETRY_REFERENCEDEDGE_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/geometry/CSGEngine.hpp>
#include <rw/geometry/ReferencedVertice.hpp>

namespace rw { namespace geometry {

    class ReferencedEdge
    {
      public:
        ReferencedEdge (TriMeshData::Ptr ref, uint32_t vertice1, uint32_t vertice2);

        /**
         * @brief get the referencedvertexes belonging to the edge.
         * @param i [in] index of the wanted vertex
         * @return ReferencedVertice
         */
        ReferencedVertice operator[] (size_t i) const;

        /**
         * @brief get the mesh index of the vertexes belonging to this edge
         * @param i [in] index of the wanted mesh index
         * @return uint32_t&
         */
        uint32_t idx (size_t i) const;

        /**
         * @brief get the mesh index of the vertexes belonging to this edge
         * @param i [in] index of the wanted mesh index
         * @return uint32_t&
         */
        uint32_t& idx (size_t i);

        /**
         * @brief check if the vertice is part of the edge
         * @param v
         * @return true if vertice is part of the edge
         * @return false if vertice is not part of the edge or the mesh
         */
        bool has (const ReferencedVertice& v) const;

        /**
         * @brief check if the vertices is part of the edge
         * @param v1
         * @param v2
         * @return true if both vertices is part of the edge
         * @return false if both vertices is not part of the edge or the mesh
         */
        bool has (const ReferencedVertice& v1, const ReferencedVertice& v2) const;

      private:
        friend class ReferencedTriangle;
        ReferencedEdge ();
        TriMeshData::Ptr _mesh;
        uint32_t _edgeIdx1, _edgeIdx2;
    };

}}    // namespace rw::geometry
#endif