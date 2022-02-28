#ifndef RW_GEOMETRY_OBJECT3D_HPP_
#define RW_GEOMETRY_OBJECT3D_HPP_

#if !defined(SWIG)
#include <rw/geometry/IndexedPolygon.hpp>
#include <rw/geometry/IndexedTriangle.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#endif 

namespace rw { namespace geometry {

    /**
     * @brief An abstract 3d object consisting of geometry information, material and texture.
     *
     * To reduce memory, the geometry is implemented slightly differently for different mesh
     * sizes. One of the concrete Object3D implementations should be used in practice.
     */
    class Object3DGeneric : public TriMesh
    {
      public:
        //! @brief Smart pointer type for Object3DGeneric.
        typedef rw::core::Ptr< Object3DGeneric > Ptr;

        //! @brief Mapping from triangles to materials.
        struct MaterialMapData
        {
            /**
             * @brief Constructor.
             * @param m [in] material id.
             * @param sidx [in] start index of triangles.
             * @param s [in] number of triangles that use the material.
             */
            MaterialMapData (std::size_t m, std::size_t sidx, std::size_t s) :
                matId (m), startIdx (sidx), size (s)
            {}
            //! @brief material that is used for these triangles
            std::size_t matId;
            //! @brief the start index of the triangles
            std::size_t startIdx;
            //! @brief number of triangles from startIdx that use this material
            std::size_t size;
        };

        /**
         * @brief ordering polygons by material consumes more memmory but reduce switches between
         * textures. All indices \b _subFaces share material \b _matIndex.
         */
        struct MaterialPolys
        {
            //! @brief Smart pointer type for MaterialPolys.
            typedef rw::core::Ptr< MaterialPolys > Ptr;

            /**
             *  @brief  Index into the vertice array of the Object3D.
             *  The _subFaces is a subset of _indices from Object3D
             */
            std::vector< rw::geometry::IndexedPolygonN< uint16_t > > _subPolys;

            //! @brief the material index shared by all polygons \b _subPolys
            int _matIndex;
        };

        //! @brief name/id of object
        std::string _name;
        //! @brief index of parent object
        int _parentObj;
        //! true if any of the materials used has texture
        bool _hasTexture;

        //! @brief Vertice array
        std::vector< rw::math::Vector3D< float > > _vertices;
        //! @brief Normal array, there must be exactly one normal per vertex
        std::vector< rw::math::Vector3D< float > > _normals;

        /**
         * @brief Texture coordinate array, the texture coordinates can be mapped to
         * either vertices or faces. The reason for this is that often vertices
         * share two or more texcoordinates and if mapping directly to vertices then additional
         * vertices is necessary.
         */
        std::vector< rw::math::Vector2D< float > > _texCoords;

        /**
         * @brief if true then the tex coodinates are mapped to faces and not vertices. if false
         * then the texCoords are mapped to each vertice
         */
        bool _mappedToFaces;

        //! @brief Transform of the object.
        rw::math::Transform3D< float > _transform;
        //! @brief Child objects.
        std::vector< Object3DGeneric::Ptr > _kids;
        //! @brief Offset of texture.
        rw::math::Vector2D< float > _texOffset;
        //! @brief Repeat texture.
        rw::math::Vector2D< float > _texRepeat;

        /**
         * @brief maps material into a range of triangles.
         */
        std::vector< MaterialMapData > _materialMap;

        //! @brief Polygons ordered according to material.
        std::vector< MaterialPolys::Ptr > _matPolys;

        /**
         * @brief destructor
         */
        virtual ~Object3DGeneric () {}

        //! @brief test if the object is textured
        bool hasTexture () const { return _hasTexture; }

        /**
         * @brief set the material used by addTriangles
         * @param material
         */
        void setMaterial (std::size_t material);

        /**
         * @brief Get the number of faces.
         * @return the number of faces.
         */
        virtual std::size_t countFaces () const = 0;

        /**
         * @brief Returns vertices corresponding to the \b idx face
         * @param idx [in] Index of the face
         * @return List with vertices
         */
        virtual std::vector< rw::math::Vector3D< float > > getFaceVertices (size_t idx) const = 0;

        /**
         * @brief gets the triangle at index idx.
         * @param idx [in] the index of the triangle.
         * @return the triangle at index idx
         */
        virtual rw::geometry::Triangle< double > getTriangle (size_t idx) const = 0;

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
         * @brief the type of this primitive
         */
        virtual GeometryType getType () const { return GeometryType::UserType; }

        virtual Object3DGeneric::Ptr copy() const = 0;

      protected:
        /**
         * @brief constructor
         * @param name [in] name of object
         */
        Object3DGeneric (const std::string& name) :
            _name (name), _parentObj (-1), _hasTexture (false), _mappedToFaces (false),
            _texOffset (0, 0), _texRepeat (0, 0)
        {}
    };

    /**
     * @brief A concrete 3d object consisting of geometry information, material and texture.
     *
     * The template parameter should be chosen based on the number of vertices in the mesh, in
     * order to reduce memory consumption.
     *
     * For a mesh that has 255 vertices or less, use Object3D<uint8_t>.
     *
     * For a mesh that has 65535 vertices or less, use Object3D<uint16_t>.
     *
     * For a mesh that has more than 65535 vertices, use Object3D<uint32_t>.
     */
    template< class T = uint16_t > class Object3D : public Object3DGeneric
    {
      public:
        //! @brief Smart pointer type for Object3D.
        typedef rw::core::Ptr< Object3D > Ptr;

        /**
         * @brief constructor
         * @param name [in] name of object
         */
        Object3D (const std::string& name) : Object3DGeneric (name) {}

        /**
         * @brief destructor
         */
        virtual ~Object3D () {}

        //! @copydoc Object3DGeneric::countFaces
        virtual std::size_t countFaces () const { return _faces.size (); }

        //! @copydoc Object3DGeneric::getFaceVertices
        virtual std::vector< rw::math::Vector3D< float > > getFaceVertices (size_t idx) const
        {
            RW_ASSERT (idx < _faces.size ());
            std::vector< rw::math::Vector3D< float > > result;
            result.push_back (_vertices[_faces[idx].getVertexIdx (0)]);
            result.push_back (_vertices[_faces[idx].getVertexIdx (1)]);
            result.push_back (_vertices[_faces[idx].getVertexIdx (2)]);
            return result;
        }

        /**
         * @brief gets the triangle at index idx.
         * @param idx [in] the index of the triangle.
         * @return the triangle at index idx
         */
        virtual rw::geometry::Triangle< double > getTriangle (size_t idx) const
        {
            rw::geometry::Triangle< double > res (
                rw::math::Vector3D< double > (_vertices[_faces[idx].getVertexIdx (0)].e ()),
                rw::math::Vector3D< double > (_vertices[_faces[idx].getVertexIdx (1)].e ()),
                rw::math::Vector3D< double > (_vertices[_faces[idx].getVertexIdx (2)].e ()));
            return res;
        }

        //! add triangle using currently selected material
        void addTriangle (const rw::geometry::IndexedTriangle< T >& tri)
        {
            _faces.push_back (tri);
            _materialMap.back ().size += 1;
        }

        //! add triangles using currently selected material
        void addTriangles (const std::vector< rw::geometry::IndexedTriangle< T > >& tris)
        {
            T startIdx          = (T) _faces.size ();
            std::size_t newSize = _faces.size () + tris.size ();
            if (newSize > 65535)
                RW_THROW ("Model3D has two many faces! - max is 65535.");
            _faces.resize (newSize);
            for (size_t i = 0; i < tris.size (); i++) {
                _faces[startIdx + i] = tris[i];
            }
            _materialMap.back ().size += tris.size ();
        }

        /**
         * @brief add triangles to this object using a specific material in the Model3D
         * @param material [in] index of the material to be used
         * @param tris [in] triangles to add
         */
        void addTriangles (T material,
                           const std::vector< rw::geometry::IndexedTriangle< T > >& tris)
        {
            setMaterial (material);
            T startIdx          = (T) _faces.size ();
            std::size_t newSize = _faces.size () + tris.size ();
            if (newSize > 65535)
                RW_THROW ("Model3D has two many faces! - max is 65535.");
            _faces.resize (newSize);
            for (size_t i = 0; i < tris.size (); i++) {
                _faces[startIdx + i] = tris[i];
            }
            _materialMap.back ().size += tris.size ();
        }

        /**
         * @brief Scales the model by \b scale.
         *
         * The transformation of the model is not scaled.
         *
         * @param scale [in] scaling factor
         */
        void scale (double scale)
        {
            for (rw::math::Vector3D< float >& v : _vertices) {
                v *= scale;
            }

            for (rw::math::Vector2D< float >& v : _texCoords) {
                v *= scale;
            }
            _transform.P () *= scale;
            _texOffset *= scale;
            _texRepeat *= scale;

            for (Object3DGeneric::Ptr kid : _kids) {
                kid->scale (scale);
            }
        }

        /**
         * @brief gets the number of triangles in the triangle array.
         */
        virtual size_t getSize () const { return _faces.size (); };

        /**
         * @brief gets the number of triangles in the triangle array.
         */
        virtual size_t size () const { return _faces.size (); };

        /**
         * @brief make a clone of this triangle mesh
         * @return clone of this trimesh
         */
        virtual rw::core::Ptr< TriMesh > clone () const
        {
            return rw::core::ownedPtr (new Object3D< T > (*this));
        }

        /**
         * @brief make a clone of this triangle mesh
         * @return clone of this Object3D
         */
        virtual Object3DGeneric::Ptr copy() const {

            return rw::core::ownedPtr (new Object3D< T > (*this));
        }

        /**
         * @brief list containing indexed polygons. The polygons index into the
         * \b _vertices array and the \b _normals array
         * The normal is implicitly indexed and defined as same index as the
         * vertex.
         */
        std::vector< rw::geometry::IndexedTriangle< T > > _faces;

        /**
         * @brief list containing indexed polygons. The polygons index into the
         * \b _vertices array and the \b _normals array
         * The normal is implicitly indexed and defined as same index as the
         * vertex.
         */
        std::vector< rw::geometry::IndexedPolygonN< T > > _polys;
    };

}}    // namespace rw::geometry
#endif