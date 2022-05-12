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

#ifndef RW_GEOMETRY_SIMPLETRIMESH_HPP_
#define RW_GEOMETRY_SIMPLETRIMESH_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/geometry/TriMesh.hpp>

#include <Eigen/Core>

namespace rw { namespace geometry {

    /**
     * @brief a simple data structure for keeping the information on vertices and triangles
     */
    struct SimpleTriMeshData
    {
        using Ptr = rw::core::Ptr< SimpleTriMeshData >;

        Eigen::Matrix< uint32_t, Eigen::Infinity, 3 > _triangles;
        Eigen::Matrix< double, Eigen::Infinity, 3 > _vertecies;
    };

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
        ReferencedVertice (SimpleTriMeshData::Ptr ref, uint32_t vertice);

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
         * @brief Serilization operator for converting to string
         * @param os [in/out] the output stream
         * @param v [in] the vertice
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& os , const ReferencedVertice& v);

        //! @brief implicit convertion to Vector3D
        operator rw::math::Vector3D< double > () const;

        //! @brief implicit convertion to Vector3D
        operator rw::math::Vector3D< float > () const;

      private:
        ReferencedVertice ();
        SimpleTriMeshData::Ptr _mesh;
        uint32_t _verIndex;
    };

    /**
     * @brief The ReferencedTriangle contains a reference to the mesh it has been created from.
     * Therefore any changes to the triangle wil be reflected in the original mesh
     */
    class ReferencedTriangle
    {
      public:
        ReferencedTriangle (SimpleTriMeshData::Ptr ref, uint32_t triangle);

        ReferencedVertice operator[] (size_t i) const;
        uint32_t& idx (size_t i) const;

        ReferencedTriangle& operator= (rw::geometry::Triangle< double > rhs);
        ReferencedTriangle& operator= (rw::geometry::Triangle< float > rhs);

        /**
         * @brief Serilization operator for converting to string
         * @param os [in/out] the output stream
         * @param t [in] the triangle
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& os , const ReferencedTriangle& t);

        operator rw::geometry::Triangle< double > () const;
        operator rw::geometry::Triangle< float > () const;

      private:
        ReferencedTriangle ();
        SimpleTriMeshData::Ptr _mesh;
        uint32_t _triIndex;
    };

    class SimpleTriMesh : public TriMesh
    {
      public:
        /**
         * @brief Construct an empty TriMesh if \b data is null else take ownership of \b data
         *
         */
        SimpleTriMesh (SimpleTriMeshData::Ptr data = NULL);

        virtual ~SimpleTriMesh ();

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh (const SimpleTriMesh& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         * @param prox [in] the distance between to vertexes to be counted as the same
         */
        SimpleTriMesh (const rw::geometry::TriMesh& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         * @param prox [in] the distance between to vertexes to be counted as the same
         */
        SimpleTriMesh (rw::geometry::GeometryData& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         * @param prox [in] the distance between to vertexes to be counted as the same
         */
        SimpleTriMesh (const rw::core::Ptr< rw::geometry::GeometryData >& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         * @param prox [in] the distance between to vertexes to be counted as the same
         */
        SimpleTriMesh (const rw::core::Ptr< rw::geometry::TriMesh >& copy);

        //###########################################
        //#        Overloaded function              #
        //###########################################

        /**
         * @brief gets the triangle at index idx.
         * @param idx [in] the index of the triangle.
         * @return the triangle at index idx
         */
        virtual rw::geometry::Triangle< double > getTriangle (size_t idx) const;

        /**
         * @brief gets the triangle at index idx.
         * @param idx [in] the index of the triangle.
         * @param dst [out] where to store the triangle at index idx
         */
        virtual void getTriangle (size_t idx, rw::geometry::Triangle< double >& dst) const;

        /**
         * @brief gets the triangle at index idx. Using Floating point presicion
         * @param idx [in] the index of the triangle.
         * @param dst [out] where to store the triangle at index idx
         */
        virtual void getTriangle (size_t idx, rw::geometry::Triangle< float >& dst) const;

        /**
         * @brief gets the number of triangles in the triangle array.
         */
        virtual size_t size () const;

        /**
         * @brief make a clone of this triangle mesh
         * @return clone of this trimesh
         */
        virtual rw::core::Ptr< TriMesh > clone () const;

        /**
         * @brief Scale all vertices in the mesh.
         */
        virtual void scale (double scale);

        /**
         * @brief the type of this primitive
         */
        virtual GeometryType getType () const { return GeometryData::GeometryType::UserType; };

        /**
         * @brief gets a trimesh representation of this geometry data.
         *
         * The trimesh that is returned is by default a copy, which means
         * ownership is transfered to the caller. Specifying \b forceCopy to false
         * will enable copy by reference and ownership is not necesarilly transfered.
         * This is more efficient, though pointer is only alive as long as this
         * GeometryData is alive.
         *
         * @return TriMesh representation of this GeometryData
         */
        virtual rw::core::Ptr< TriMesh > getTriMesh (bool forceCopy = true);

        //###########################################
        //#             Data Access                 #
        //###########################################

        /**
         * @brief get triangle at index \b idx. Changes to the triangle will be reflected in this
         * mesh
         * @param idx index of the triangle
         * @return ReferencedTriangle
         */
        ReferencedTriangle triangle (size_t idx) const;

        /**
         * @brief get number of triangles
         * @return size_t
         */
        size_t triangles () const;

        /**
         * @brief get vertice at index \b idx. Changes to the vertice will be reflected in this mesh
         * @param idx the index of the wanted vertice
         * @return ReferencedVertice
         */
        ReferencedVertice vertice (size_t idx) const;

        /**
         * @brief number of vertices in this mesh
         * @return size_t
         */
        size_t vertices () const;

        /**
         * @brief Change the number of vertices and triangles in this mesh
         * @param triangles the amount of triangles in the mesh
         * @param vertices the amount of vertices in the mesh
         */
        void resize (size_t triangles, size_t vertices);

      private:
        void fromTriMesh (const rw::geometry::TriMesh& copy);

        SimpleTriMesh(std::vector<ReferencedTriangle> triangles);

        rw::core::Ptr< SimpleTriMeshData > _data;
    };

}}    // namespace rw::geometry

#endif