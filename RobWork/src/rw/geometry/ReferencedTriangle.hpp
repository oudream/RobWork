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

#ifndef RW_GEOMETRY_REFERENCEDTRIANGLE_HPP_
#define RW_GEOMETRY_REFERENCEDTRIANGLE_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/geometry/CSGEngine.hpp>
#include <rw/geometry/ReferencedEdge.hpp>
#include <rw/geometry/ReferencedVertice.hpp>

namespace rw { namespace geometry {
    /**
     * @brief The ReferencedTriangle contains a reference to the mesh it has been created from.
     * Therefore any changes to the triangle wil be reflected in the original mesh
     */
    class ReferencedTriangle
    {
      public:
        /**
         * @brief Construct a new Referenced Triangle a mesh and the index of the triangle in the
         * mesh
         * @param ref [in] the TriMesh the triangle is a part of
         * @param triangle [in] the index of the triangle in the mesh
         */
        ReferencedTriangle (TriMeshData::Ptr ref, uint32_t triangle);

        /**
         * @brief Acces the vertices of the triangle
         * @param i [in] which of the 3 vertices to access
         * @return ReferencedVertice. Any changes to the vertice will modify the mesh
         */
        ReferencedVertice operator[] (size_t i) const;

        /**
         * @brief get the indexes of the vertices that constetutes this triangle
         * @param i [in] which of the 3 vertices to access
         * @return reference to the trangle index. Changing this value will modefiy the mesh
         */
        uint32_t& idx (size_t i);

        /**
         * @brief get the mesh index of the vertexes belonging to this triangle
         * @param i [in] index of the wanted mesh index
         * @return uint32_t&
         */
        uint32_t idx (size_t i) const;

        /**
         * @brief get one of the 3 edges that makes up this triangle
         * @param i [in] index of the edge
         * @return ReferencedEdge
         */
        ReferencedEdge edge (size_t i) const;

        /**
         * @brief check if triangle got edge
         * @param edge [in] the edge
         * @return true if got edge
         * @return false if dosen't have edge or don't share mesh
         */
        bool has (const ReferencedEdge& edge) const;

        /**
         * @brief check if triangle got vertice
         * @param vertice [in] the vertice
         * @return true if got vertice
         * @return false if dosen't have vertice or don't share mesh
         */
        bool has (const ReferencedVertice& vertice) const;

#ifndef SWIGJAVA
        /**
         * @brief get the vertice oposit the \b edge
         * @param edge [in] the edge
         * @return ReferencedVertice
         * @exception rw::core::Exception if edge not part of triangle
         */
#endif
        ReferencedVertice opposit (const ReferencedEdge& edge) const;

#ifndef SWIGJAVA
        /**
         * @brief get the edge oposit the \b vertice
         * @param vertice [in] the vertice
         * @return ReferencedEdge
         * @exception rw::core::Exception if vertice not part of triangle
         */
#endif
        ReferencedEdge opposit (const ReferencedVertice& vertice) const;

        //###########################################
        //#             Functions                   #
        //###########################################

        /**
         * @brief Check if this triangle intersects \b t
         * @param t [in] other triangle to check for intersections
         * @return true if intersects
         * @return false if don't intersect or if they are not part of the same mesh
         */
        bool intersects (const ReferencedTriangle& t) const;

        /**
         * @brief Check if the triangles are part of the same mesh.
         * @param t [in] the other triangle
         * @return true if part of the same mesh
         * @return false if not part of the same mesh
         */
        bool sameMesh (const ReferencedTriangle& t) const;

        //###########################################
        //#             Operators                   #
        //###########################################

        /**
         * @brief assignment operator. Over writes the values of the vertices of this triangle with
         * the values of \b rhs. These changes will directly propergate the the mesh of this
         * triangle
         * @param rhs [in] the new values of this triangle
         * @return ReferencedTriangle&
         */
        ReferencedTriangle& operator= (rw::geometry::Triangle< double > rhs);

        /**
         * @brief assignment operator. Over writes the values of the vertices of this triangle with
         * the values of \b rhs. These changes will directly propergate the the mesh of this
         * triangle
         * @param rhs [in] the new values of this triangle
         * @return ReferencedTriangle&
         */
        ReferencedTriangle& operator= (rw::geometry::Triangle< float > rhs);

        /**
         * @brief assignment operator. Over writes the values of which verticies this triangle
         * points at. These changes will directly propergate the the mesh of this triangle
         * @param rhs [in] the new values of this triangle
         * @return ReferencedTriangle&
         */
        ReferencedTriangle& operator= (rw::math::Vector3D< int > rhs);

        /**
         * @brief Serilization operator for converting to string
         * @param os [in/out] the output stream
         * @param t [in] the triangle
         * @return std::ostream&
         */
        friend std::ostream& operator<< (std::ostream& os, const ReferencedTriangle& t);

        /**
         * @brief conversion operator to non-referenced triangle. This creates a copy of the values
         * in this triangle, for making the new triangle
         * @return rw::geometry::Triangle< double >
         */
        operator rw::geometry::Triangle< double > () const;

        /**
         * @brief conversion operator to non-referenced triangle. This creates a copy of the values
         * in this triangle, for making the new triangle
         * @return rw::geometry::Triangle< double >
         */
        operator rw::geometry::Triangle< float > () const;

        /**
         * @brief convert the refrenced triangle to the index of the triangle in the mesh
         * @return int
         */
        operator int () const;

      private:
        ReferencedTriangle ();
        TriMeshData::Ptr _mesh;
        uint32_t _triIndex;
    };

}}    // namespace rw::geometry
#endif