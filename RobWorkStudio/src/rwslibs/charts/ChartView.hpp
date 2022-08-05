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

#ifndef RWSLIBS_CHARTS_CHARTVIEW_HPP_
#define RWSLIBS_CHARTS_CHARTVIEW_HPP_

#include <rws/PlotView.hpp>

namespace rws {

class ChartView: public PlotView
{
    public:
        ChartView();
        virtual ~ChartView();

        //! @copydoc PlotView::listPlot
        virtual void listPlot (const std::vector< double >& x, const std::vector< double >& y,
                       const std::string& title = "", const std::string& xlabel = "",
                       const std::string& ylabel = "");

        //! @copydoc PlotView::getWidget
        virtual QGraphicsView* getWidget(QWidget* parent);

        //! @brief Dispatcher for plots.
        class Dispatcher: public PlotView::Dispatcher
        {
            public:
                //! @brief Smart pointer type.
                typedef rw::core::Ptr< const Dispatcher > Ptr;

                //! @brief Constructor.
                Dispatcher();

                //! @brief Destructor.
                virtual ~Dispatcher();

                /**
                 * @brief Create a new PlotView.
                 *
                 * @return a new PlotView.
                 */
                virtual PlotView::Ptr makePlotView() const;
        };

    private:
        class PrivateImpl;
        PrivateImpl* const _impl;
};

} /* namespace rws */

#endif /* RWSLIBS_CHARTS_CHARTVIEW_HPP_ */
