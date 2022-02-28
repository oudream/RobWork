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

#ifndef RW_GEOMETRY_Model3D_HPP_
#define RW_GEOMETRY_Model3D_HPP_

//! @file Model3D.hpp
#if !defined(SWIG)
#include <rw/core/AnyPtr.hpp>
#include <rw/geometry/GeometryData.hpp>
#include <rw/geometry/IndexedPolygon.hpp>
#include <rw/geometry/IndexedTriangle.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/geometry/Object3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>

#include <vector>
#endif
namespace rw { namespace geometry {
    class Geometry;
}}    // namespace rw::geometry
namespace rw { namespace geometry {
    class TriMesh;
}}    // namespace rw::geometry

namespace rw { namespace geometry {

    //! @addtogroup graphics
    // @{

    /**
     * @brief a 3d model that has geometry but also material and color.
     * he model can be composed of multiple objects that are connected in
     * a hierarchical manner. The model is designed for efficient drawing and as such special
     * structures are used to order the indexes such that efficient drawing is possible.
     */
    class Model3D
    {
      public:
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr< Model3D > Ptr;

        /**
         * Constructor.
         * @param name [in] name of the model.
         */
        Model3D (const std::string& name);

        /**
         * @brief Copy constructor, make a copy of the 3D object
         * 
         * @param model 
         */
        Model3D(const Model3D& model);

        //! @brief destructor
        virtual ~Model3D ();

        /**
         * @brief describes material properties. A material can be either simple or "advanced"
         * and in both cases it can be textured.
         * A simple material is described by a 4-tuple of RGBA values. The advanced material
         * defines multiple properties: diffuse, ambient, emissive, specular, shininess and
         * transparency
         */
        struct Material
        {
            //! @brief default constructor
            Material ();

            //! @brief constructor for simple material
            Material (const std::string& nam, float r, float g, float b, float a = 1.0);

            /**
             * @brief Check if material has texture.
             * @return true if material has texture.
             */
            bool hasTexture () const { return texId >= 0; }

            /**
             * @brief Get id of the texture for this material.
             * @return the texture id.
             */
            int getTextureID () const { return texId; }

            //! @brief material name, not necesarily unique
            std::string name;
            //! @brief true if this material is a simple material
            bool simplergb;
            //! @brief index to a texture which is stored in Model3D, -1 if not used
            short int texId;
            //! @brief Red, Green, Blue and alpha color (simple) or diffues color(advanced)
            float rgb[4];
            //! @brief Ambient color as RGBA
            float ambient[4];
            //! @brief Emissive color as RGBA
            float emissive[4];
            //! @brief Specular color as RGB
            float specular[4];

            //! @brief The shininess \f$\in [0,128] \f$
            float shininess;
            //! @brief Transparency \f$ in [0, 1]\f$
            float transparency;
        };

        using MaterialPolys = rw::geometry::Object3DGeneric::MaterialPolys;
        using Object3DGeneric = rw::geometry::Object3DGeneric;
        template< class T = uint16_t> 
        using Object3D = rw::geometry::Object3D< T >;

        //! @brief Method to do smoothing.
        typedef enum {
            //! vertex normal is determine as an avarage of all adjacent face normals
            AVERAGED_NORMALS,
            /**
             * @brief vertex normal is determined as AVARAGED_NORMALS, but with the
             * face normals scaled by the face area
             */
            WEIGHTED_NORMALS
        } SmoothMethod;

        /**
         * @brief optimize vertices and vertice normals
         *
         * removes redundant vertices and recalculates all vertice normals based on the face normals
         * and the angle between face normals \b smooth_angle.
         * @param smooth_angle
         * @param method
         */
        void optimize (double smooth_angle, SmoothMethod method = WEIGHTED_NORMALS);

        /**
         * @brief add an Object to this Model3D
         * @param obj [in] the geometric object to add.
         * @return index of object in model3d
         */
        int addObject (Object3DGeneric::Ptr obj);

        /**
         * @brief add geometry to this model3d
         * @param mat [in] the material properties to use for the geometry.
         * @param geom [in] the geometry to add.
         */
        void addGeometry (const Material& mat, rw::core::Ptr< class rw::geometry::Geometry > geom);

        /**
         * @brief add a triangle mesh to this model3d
         * @param mat [in] the material properties to use for the mesh.
         * @param mesh [in] the mesh geometry.
         */
        void addTriMesh (const Material& mat, const rw::geometry::TriMesh& mesh);

        /**
         * @brief add a triangle mesh to this model3d
         * @param mat [in] the material properties to use for the mesh.
         * @param mesh [in] the mesh geometry.
         */
        void addTriMesh (const Material& mat, rw::core::Ptr< const rw::geometry::TriMesh > geom);

        /**
         * @brief all objects in a model use the materials defined on the model
         * @param mat [in] material to add.
         * @return id of the newly added material.
         */
        int addMaterial (const Material& mat);

        /**
         * @brief get material with string id matid
         * @param matid [in] string id
         * @return pointer to Matrial data
         */
        Material* getMaterial (const std::string& matid);

        /**
         * @brief check if model has material with id matid
         * @param matid [in] string id of material
         * @return true if exists in model
         */
        bool hasMaterial (const std::string& matid);

        /**
         * @brief remove object with string id name
         * @param name [in] name of object to remove
         */
        void removeObject (const std::string& name);

        //! @copydoc Object3DGeneric::scale
        void scale (float scale);

        //! @brief get all materials that are available in this model
        std::vector< Material >& getMaterials () { return _materials; }

        //! @brief get all objects that make out this model
        std::vector< Object3DGeneric::Ptr >& getObjects () { return _objects; }

        //! get pose of this Model3D
        const rw::math::Transform3D<>& getTransform () { return _transform; }
        //! set the pose of this Model3D
        void setTransform (const rw::math::Transform3D<>& t3d) { _transform = t3d; }

        //! get string identifier of this model3d
        const std::string& getName () { return _name; }
        //! get filePath of this model3d
        const std::string& getFilePath () { return _filePath; }
        //! set string identifier of this model3d
        void setName (const std::string& name) { _name = name; }
        //! set filePath this model3d
        void setFilePath (const std::string& name) { _filePath = name; }

        //! get mask of this model3d
        int getMask () { return _mask; }
        //! set mask of this model3d
        void setMask (int mask) { _mask = mask; }

        /**
         * @brief convert this model3d to a geometry. Notice that geometry does not hold any
         * color information.
         * @return a geometry of this model3d
         */
        rw::geometry::GeometryData::Ptr toGeometryData ();

        //! true if data in the model are expected to change
        bool isDynamic () const { return _isDynamic; }
        //! set to true if data in the model are expected to change
        void setDynamic (bool dynamic) { _isDynamic = dynamic; }

        template< class S > std::vector< S >& getTextures ()
        {
            std::vector< S >* ptr = NULL;
            if (_textures.isNull ()) {
                _textures = rw::core::AnyPtr (rw::core::ownedPtr (new std::vector< S > ()));
            }
            ptr = _textures.get< std::vector< S > > ();
            if (ptr == NULL) {
                RW_THROW ("Incompatible texture types");
            }
            return *ptr;
        }

        //! @brief The array of materials.
        std::vector< Material > _materials;
        //! @brief The array of objects in the model
        std::vector< Object3DGeneric::Ptr > _objects;

      protected:
        //! @brief The transform of the model.
        rw::math::Transform3D<> _transform;
        //! @brief Name of the model.
        std::string _name;
        //! @brief FilePath of the model, if model was constructed from file.
        std::string _filePath;
        //! @brief The DrawableNode::DrawableTypeMask
        int _mask;
        //! @brief If the data can be expected to change.
        bool _isDynamic;

      private:
        rw::core::AnyPtr _textures;
    };
    //! @}
}}    // namespace rw::geometry

#endif