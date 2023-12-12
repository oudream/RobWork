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

#ifndef RW_LOADERS_LOADERSTEP_HPP_
#define RW_LOADERS_LOADERSTEP_HPP_

//! @file LoaderSTEP.hpp

#include "../Model3DLoader.hpp"

#include <RobWorkConfig.hpp>

#include <string>

class TopoDS_Shape;

namespace rw { namespace loaders {

    //! @addtogroup graphics
    // @{

    /**
     * @brief Class for loading in IVG files.
     * TODO: add documentation on IVG format
     */
    class LoaderSTEP : public Model3DLoader
    {
      public:
        /**
         * @brief constructor
         */
        LoaderSTEP(){};

        /**
         * @brief destructor
         */
        virtual ~LoaderSTEP(){};

        //! @copydoc Model3DLoader::load
        rw::graphics::Model3D::Ptr load(const std::string& filename);

        /**
         * @brief convert an OpenCasCade Shape to a RobWorkModel. this will try to triangulate the
         * shape
         * @param shape [in] the OpenCasCade Shape to convert. OBS not all shapes can be
         * triangulated
         * @param name [in] name of the model
         */
        static rw::graphics::Model3D::Ptr toModel(const TopoDS_Shape& shape, std::string name);

        /**
         * @brief sets how much the Surface should deflect before a new triangle is made.
         * check BRepMesh_IncrementalMesh at
         * https://dev.opencascade.org/doc/refman/html/class_b_rep_mesh___incremental_mesh.html for
         * more information
         * @param linDef [in] linear Defelection
         * @param angDef [in] angular deflection
         */
        static void setDeflection(double linDef = 0.01, double angDef = 0.5);

#if RW_HAVE_OCC
        //! @copydoc Model3DLoader::getModelFormats
        std::vector<std::string> getModelFormats() {
            return {".STP", ".STEP", ".IGS", ".BREP"};
        }
#else
        //! @copydoc Model3DLoader::getModelFormats
        std::vector<std::string> getModelFormats() {
            return {};
        }
#endif

      private:
        inline static double _linDef = 0.01;
        inline static double _angDef = 0.5;
    };

    //! @}

}}    // namespace rw::loaders

#endif    // end include guard
