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

#include <RobWorkConfig.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/geometry/CSGEngine.hpp>
#include <rw/geometry/TriMesh.hpp>

#include <Eigen/Core>

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
        ReferencedVertice ();
        TriMeshData::Ptr _mesh;
        uint32_t _verIndex;
    };

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
        uint32_t& idx (size_t i) const;

        /**
         * @brief assign new values to the vertices of this triangle, by copying them.
         * OBS this will modify the mesh.
         * @param rhs
         * @return ReferencedTriangle&
         */
        ReferencedTriangle& operator= (rw::geometry::Triangle< double > rhs);

        /**
         * @brief assign new values to the vertices of this triangle, by copying them.
         * OBS this will modify the mesh.
         * @param rhs
         * @return ReferencedTriangle&
         */
        ReferencedTriangle& operator= (rw::geometry::Triangle< float > rhs);

        /**
         * @brief Serilization operator for converting to string
         * @param os [in/out] the output stream
         * @param t [in] the triangle
         * @return std::ostream&
         */
        friend std::ostream& operator<< (std::ostream& os, const ReferencedTriangle& t);

        //! @brief Implicit converter to other triangle type
        operator rw::geometry::Triangle< double > () const;

        //! @brief Implicit converter to other triangle type
        operator rw::geometry::Triangle< float > () const;

        //! @brief implicit acces the triangle index
        operator int () const;

      private:
        ReferencedTriangle ();
        TriMeshData::Ptr _mesh;
        uint32_t _triIndex;
    };

    /**
     * @brief A TriMesh with better interactions with the underlying data, and more capabilities
     */
    class SimpleTriMesh : public TriMesh
    {
      public:
        using Ptr = rw::core::Ptr< SimpleTriMesh >;
        /**
         * @brief Construct an empty TriMesh if \b data is null else take ownership of \b data
         * @param data [in] the data to take ownership of. If now shared pointer, then the
         * destructor cleans it up
         */
        SimpleTriMesh (TriMeshData::Ptr data = NULL);

        /**
         * @brief if the internal data object is not a shared pointer the data is deleted, when the
         * destructor is called
         */
        virtual ~SimpleTriMesh ();

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh (const SimpleTriMesh& copy);

        /**
         * @brief Transfere the data from a temporary TriMesh. This will simply copy the shared
         * pointer to the data object
         * @param tmp the object to take the data from
         */
        SimpleTriMesh (const SimpleTriMesh&& tmp);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh (const rw::core::Ptr< SimpleTriMesh >& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh (const rw::geometry::TriMesh& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh (rw::geometry::GeometryData& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh (rw::geometry::GeometryData&& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh (const rw::core::Ptr< rw::geometry::GeometryData >& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
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
         * @brief Scale all vertices in the mesh.
         * @param scale [in] how each axis should be scaled.
         */
        virtual void scale (const rw::math::Vector3D< double >& scale);

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

        /**
         * @brief get direct access to the data container
         */

        TriMeshData::Ptr getData () { return _data; }

        /**
         * @brief get the current CSGEngine
         * @return Null if none is found else a pointer to the engine
         */
        CSGEngine::Ptr getCSGEngine () const { return _engine; }

        /**
         * @brief set CSG engine
         * @param engine [in] pointer to the new engine
         */
        void setCSGEngine (CSGEngine::Ptr engine) { _engine = engine; }

        //###########################################
        //#      Mesh Operations & analysis         #
        //###########################################

        /**
         * @brief Check if there are non-connected meshes inside this mesh. If ther are then
         * seperate and return them. non-connected means that you have meshes that does not share a triangluar edge
         * @return if no seperate meshes return this mesh. Else seperate the Meshes and return them.
         */
        std::vector< SimpleTriMesh > separateMeshes () const;

        /**
         * @brief combine two meshes. OBS! this is not a union operation has all vertices and
         * triangles are just directly copied over
         *
         * @param mesh [in] the mesh, that his mesh should be combined with
         * @return the resulting trimesh
         */
        SimpleTriMesh combine (const SimpleTriMesh& mesh) const;

        //###########################################
        //#               Operators                 #
        //###########################################

        /**
         * @brief Move this TriMesh.
         * @param trans [in] apply this transform to all vertices.
         * @return a reference to this TriMesh.
         */
        SimpleTriMesh& operator*= (const rw::math::Transform3D< double >& trans);

        /**
         * @brief Create a copy of the TriMesh and move it.
         * @param trans [in] apply this transform to all vertices.
         * @return a new TriMesh
         */
        SimpleTriMesh operator* (const rw::math::Transform3D< double >& trans) const;

        /**
         * @brief Compute the Union of two TriMeshes, into a new TriMesh.
         * @param rhs [in] the TriMesh to create a union with
         * @throw rw::core::Exception if no CSGEngine can be found
         * @return a new TriMesh of the union
         */
        SimpleTriMesh operator+ (const SimpleTriMesh& rhs) const;

        /**
         * @brief Compute the Union of two TriMeshes, into this TriMesh.
         * @param rhs [in] the TriMesh to create a union with
         * @throw rw::core::Exception if no CSGEngine can be found
         * @return a reference to the result
         */
        SimpleTriMesh& operator+= (const SimpleTriMesh& rhs);

        /**
         * @brief Compute the Difference of two TriMeshes, into a new TriMesh.
         * @param rhs [in] the TriMesh to create a difference with
         * @throw rw::core::Exception if no CSGEngine can be found
         * @return a new TriMesh of the difference
         */
        SimpleTriMesh operator- (const SimpleTriMesh& rhs) const;

        /**
         * @brief Compute the Difference of two TriMeshes, into this TriMesh.
         * @param rhs [in] the TriMesh to create a Difference with
         * @throw rw::core::Exception if no CSGEngine can be found
         * @return a reference to the result
         */
        SimpleTriMesh& operator-= (const SimpleTriMesh& rhs);

        /**
         * @brief Compute the Intersection of two TriMeshes, into a new TriMesh.
         * @param rhs [in] the TriMesh to create a Intersection with
         * @throw rw::core::Exception if no CSGEngine can be found
         * @return a new TriMesh of the Intersection
         */
        SimpleTriMesh operator& (const SimpleTriMesh& rhs) const;

        /**
         * @brief Compute the Intersection of two TriMeshes, into this TriMesh.
         * @param rhs [in] the TriMesh to create a Intersection with
         * @throw rw::core::Exception if no CSGEngine can be found
         * @return a reference to the result
         */
        SimpleTriMesh& operator&= (const SimpleTriMesh& rhs);
        /**
         * @brief Compute the Symetric Difference of two TriMeshes, into a new TriMesh.
         * @param rhs [in] the TriMesh to create a Symetric Difference with
         * @throw rw::core::Exception if no CSGEngine can be found
         * @return a new TriMesh of the Symetric Difference
         */
        SimpleTriMesh operator^ (const SimpleTriMesh& rhs) const;

        /**
         * @brief Compute the Symetric Difference of two TriMeshes, into this TriMesh.
         * @param rhs [in] the TriMesh to create a Symetric Difference with
         * @throw rw::core::Exception if no CSGEngine can be found
         * @return a reference to the result
         */
        SimpleTriMesh& operator^= (const SimpleTriMesh& rhs);

        //###########################################
        //#         Assignment Operators            #
        //###########################################

        /**
         * @brief set the current data pointer equal to the new one
         * @param data [in] the data to take ownership of. If not shared pointer, then the
         * destructor cleans it up
         */
        SimpleTriMesh& operator= (TriMeshData::Ptr data);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh& operator= (const SimpleTriMesh& copy);

        /**
         * @brief Transfere the data from a temporary TriMesh. This will simply copy the shared
         * pointer to the data object
         * @param tmp the object to take the data from
         */
        SimpleTriMesh& operator= (const SimpleTriMesh&& tmp);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh& operator= (const rw::core::Ptr< SimpleTriMesh >& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh& operator= (const rw::geometry::TriMesh& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh& operator= (rw::geometry::GeometryData& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh& operator= (rw::geometry::GeometryData&& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh& operator= (const rw::core::Ptr< rw::geometry::GeometryData >& copy);

        /**
         * @brief Copy the data from an existing trimesh
         * @param copy the object to be copied
         */
        SimpleTriMesh& operator= (const rw::core::Ptr< rw::geometry::TriMesh >& copy);

      private:
        void fromTriMesh (const rw::geometry::TriMesh& copy);

        SimpleTriMesh (std::vector< ReferencedTriangle > triangles, rw::core::Ptr< TriMeshData > data);
        rw::core::Ptr< TriMeshData > _data;
        CSGEngine::Ptr _engine;
    };

}}    // namespace rw::geometry

#endif