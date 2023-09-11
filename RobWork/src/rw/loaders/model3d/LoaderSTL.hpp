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

#ifndef RW_LOADER_LOADERSTL_HPP_
#define RW_LOADER_LOADERSTL_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/loaders/Model3DLoader.hpp>
#include <rw/loaders/model3d/STLFile.hpp>

namespace rw { namespace loaders {
    //! @addtogroup geometry
    // @{

    /**
     * @brief static methods for reading and writing geometry to and from
     * STL files.
     */
    class LoaderSTL : public Model3DLoader
    {
      public:
      public:
        /**
         * @brief constructor
         */
        LoaderSTL(){};

        /**
         * @brief destructor
         */
        virtual ~LoaderSTL(){};

        //! @copydoc Model3DLoader::load
        rw::graphics::Model3D::Ptr load(const std::string& filename);

        //! @copydoc Model3DLoader::getModelFormats
        std::vector<std::string> getModelFormats() { return {".STL", ".STLA", ".STLB"}; }
    };
    // @}
}}    // namespace rw::loaders

#endif /*STLFILE_HPP_*/
