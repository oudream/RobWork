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

#ifndef RWLIBS_PLOTS_MATHGLPLOT_HPP_
#define RWLIBS_PLOTS_MATHGLPLOT_HPP_

/**
 * @file MathGLPlotGenerator
 *
 * \copydoc rwlibs::plots::MathGLPlotGenerator
 */

#include <rw/graphics/Plot.hpp>

namespace rwlibs { namespace plots {
    //! @addtogroup plots

    //! @{
    //! @brief MathGL based implementation of a plot.
    class MathGLPlot : public rw::graphics::Plot
    {
      public:
        //! @brief Constructor.
        MathGLPlot();

        //! @brief Destructor.
        virtual ~MathGLPlot();

        //! @coypdoc Plot::listPlot
        virtual void listPlot(const std::vector<double>& x, const std::vector<double>& y,
                              const std::string& title = "", const std::string& xlabel = "",
                              const std::string& ylabel = "");

        //! @coypdoc Plot::render
        virtual rw::core::Ptr<rw::sensor::Image> render(unsigned int width, unsigned int height);

      private:
        struct PlotData;
        PlotData* const _data;
    };
    //! @}

}}    // namespace rwlibs::plots

#endif /* RWLIBS_PLOTS_MATHGLPLOT_HPP_ */
