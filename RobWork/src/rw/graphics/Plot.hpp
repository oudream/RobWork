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

#ifndef RW_GRAPHICS_PLOT_HPP_
#define RW_GRAPHICS_PLOT_HPP_

/**
 * @file Plot.hpp
 *
 * \copydoc rw::graphics::Plot
 */

#include <rw/core/ExtensionPoint.hpp>
#include <rw/core/Ptr.hpp>

namespace rw { namespace sensor {
    class Image;
}}

namespace rw {
namespace graphics {
//! @addtogroup graphics

//! @{
//! @brief Interface for generation of plot images.
class Plot
{
    protected:
        //! @brief Constructor.
        Plot() {}

    public:
        //! @brief Smart pointer type.
        typedef rw::core::Ptr< Plot > Ptr;

        //! @brief Destructor.
        virtual ~Plot() {}

        /**
         * @brief Do a ListPlot with the given x- and y-values.
         *
         * @param x [in] the x-values.
         * @param y [in] the y-values.
         * @param title [in] (optional) the plot title.
         * @param xlabel [in] (optional) x axis label.
         * @param ylabel [in] (optional) y axis label.
         */
        virtual void listPlot (const std::vector< double >& x, const std::vector< double >& y,
                       const std::string& title = "", const std::string& xlabel = "",
                       const std::string& ylabel = "") = 0;

        /**
         * @brief Render the plot as an image.
         *
         * @param width [in] desired width.
         * @param height [in] desired height.
         * @return image with the rendered plot.
         */
        virtual rw::core::Ptr<rw::sensor::Image> render(unsigned int width, unsigned int height) = 0;
};
//! @}

} /* namespace graphics */
} /* namespace rw */

#endif /* RW_GRAPHICS_PLOT_HPP_ */
