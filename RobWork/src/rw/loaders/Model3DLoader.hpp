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

#ifndef RW_GRAPHICS_MODEL3DLOADER_HPP_
#define RW_GRAPHICS_MODEL3DLOADER_HPP_

//! @file Model3DLoader.hpp
#if !defined(SWIG)
#include <rw/common/FileCache.hpp>
#include <rw/core/ExtensionPoint.hpp>
#include <rw/graphics/Model3D.hpp>
#endif

namespace rw { namespace loaders {
    /** @addtogroup loaders */
    //! @{

    /**
     * @brief interface for classes that are able to load 3d models
     */
    class Model3DLoader
    {
      public:
        //! smart pointer type
        typedef rw::core::Ptr< Model3DLoader > Ptr;

        //! destructor
        virtual ~Model3DLoader (){};
        
        /**
         * @brief load a Model3D from file \b filename
         * @param filename [in] name of file to load
         * @return a model3d if loaded successfully else NULL (or exception)
         */
        virtual rw::geometry::Model3D::Ptr load (const std::string& filename) = 0;

        // virtual void save(Model3DPtr model, const std::string& filename) = 0;

        /**
         * @brief get the list of supported 3D model formats (as extensions)
         * @return
         */
        virtual std::vector< std::string > getModelFormats () = 0;

        /**
         * @brief Check if the loader support a specific format
         * @param format [in] the extension to check if is supported
         * @return true if format is supported 
         */
        bool isSupported(std::string format);

        /**
         * @brief set a name that the loader can use, if it can't find anyother
         * @param name 
         */
        void setDefaultName(std::string name);

        /**
         * @brief set which material to use if the File dosen't include a material/ texturing it self
         * @param mat 
         */
        void setDefaultMaterial(rw::geometry::Model3D::Material mat);

        /**
         * @addtogroup extensionpoints
         * @extensionpoint{rw::loaders::Model3DLoader::Factory,rw::loaders::Model3DLoader,rw.loaders.Model3DLoader}
         */

        /**
         * @brief a factory for Model3DLoaders. This factory defines an
         * extension point for Model3DLoaders.
         */
        class Factory : public rw::core::ExtensionPoint< Model3DLoader >
        {
          public:
            //! constructor
            Factory () :
                rw::core::ExtensionPoint< Model3DLoader > ("rw.loaders.Model3DLoader",
                                                           "Example extension point"){};

            /**
             * get loader for a specific file format (extension)
             * @param format [in] extension of file
             * @param skip [in] skip the first few valid loaders
             * @return
             */
            static rw::core::Ptr< Model3DLoader > getModel3DLoader (const std::string& format, size_t skip = 0);

            /**
             * test if a loader exist for a specific file format (extension)
             * @param format [in] extension of file
             * @return
             */
            static bool hasModel3DLoader (const std::string& format);

            /**
             * @brief get a list of supported formats
             * @return
             */
            static std::vector< std::string > getSupportedFormats ();

        };
   
   
          protected:
          Model3DLoader();
          std::string _defaultName;
          rw::geometry::Model3D::Material _defaultMat;
    };

    // typedef Model3DLoader::Factory Model3DFactory;

    //! @}

}}     // namespace rw::loaders
#endif /* RW_GRAPHICS_MODEL3DLOADER_HPP_ */
