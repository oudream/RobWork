/*****************************************************************************
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
 *****************************************************************************/

#ifndef RW_GRAPHICS_PLOTGENERATOR_HPP_
#define RW_GRAPHICS_PLOTGENERATOR_HPP_

/**
 * @file PlotGenerator
 *
 * \copydoc rw::graphics::PlotGenerator
 */

#include <rw/core/ExtensionPoint.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/graphics/Plot.hpp>

namespace rw {
namespace graphics {
//! @addtogroup graphics

//! @{
//! @brief Interface for for plot generators.
class PlotGenerator
{
    protected:
        //! @brief Constructor.
        PlotGenerator() {}

    public:
        //! @brief Smart pointer type.
        typedef rw::core::Ptr< PlotGenerator > Ptr;

        //! @brief Destructor.
        virtual ~PlotGenerator() {}

        /**
         * @brief Create a new Plot.
         *
         * @return a new plot.
         */
        virtual Plot::Ptr makePlot() = 0;

#if !defined(SWIG)
        /**
         * @addtogroup extensionpoints
         * @extensionpoint{ rw::graphics::PlotGenerator::Factory,rw::graphics::PlotGenerator,rw.graphics.PlotGenerator }
         * \class PlotGenerator
         */
#endif
        /**
         * @brief A factory for PlotGenerators. This factory defines an
         * extension point for PlotGenerators (rw.graphics.PlotGenerator).
         * Typically this is for generation of plots in the form of images.
         * The extensions registered at this extension point must have an
         * extension descriptor with a property called "generator" giving
         * a unique identifier for that particular generator.
         */
        class Factory : public rw::core::ExtensionPoint< PlotGenerator >
        {
          private:
                Factory ();

          public:
            /**
             * @brief Get a specific type of PlotGenerator.
             * @param implementation [in] name of the implementation to use.
             * @return a PlotGenerator if found, nullptr otherwise.
             */
            static rw::core::Ptr< PlotGenerator > getPlotGenerator (const std::string& implementation);

            /**
             * @brief Check if the factory has a specific PlotGenerator.
             * @param implementation [in] name of the implementation.
             * @return true if implementation was found, false otherwise.
             */
            static bool hasPlotGenerator (const std::string& implementation);

            /**
             * @brief Get a list of PlotGenerator.
             * @return a list of names for generator implementaitons.
             */
            static std::vector< std::string > getPlotGenerators ();
        };
};
//! @}

} /* namespace graphics */
} /* namespace rw */

#endif /* RW_GRAPHICS_PLOTGENERATOR_HPP_ */
