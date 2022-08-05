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

#ifndef RWSLIBS_CHARTS_CHARTVIEWGENERATOR_H_
#define RWSLIBS_CHARTS_CHARTVIEWGENERATOR_H_

#include <rw/graphics/PlotGenerator.hpp>

namespace rws {

class ChartViewGenerator: public rw::graphics::PlotGenerator
{
    public:
        //! @brief Smart pointer type.
        typedef rw::core::Ptr< PlotGenerator > Ptr;

        //! @brief Constructor.
        ChartViewGenerator();

        //! @brief Destructor.
        virtual ~ChartViewGenerator();

        /**
         * @brief Create a new Plot.
         *
         * @return a new plot.
         */
        virtual rw::graphics::Plot::Ptr makePlot();
};

} /* namespace rws */

#endif /* RWSLIBS_CHARTS_CHARTVIEWGENERATOR_H_ */
