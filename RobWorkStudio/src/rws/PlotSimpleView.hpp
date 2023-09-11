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

#ifndef RWS_PLOTSIMPLEVIEW_HPP_
#define RWS_PLOTSIMPLEVIEW_HPP_

#include "PlotView.hpp"

namespace rws {

class PlotSimpleView : public PlotView
{
  public:
    //! @brief Constructor.
    PlotSimpleView();

    //! @brief Destructor.
    virtual ~PlotSimpleView();

    //! @copydoc PlotView::listPlot
    virtual void listPlot(const std::vector<double>& x, const std::vector<double>& y,
                          const std::string& title = "", const std::string& xlabel = "",
                          const std::string& ylabel = "");

    //! @copydoc PlotView::getWidget
    virtual QGraphicsView* getWidget(QWidget* parent);

  private:
    class PrivateImpl;
    PrivateImpl* const _impl;
};

} /* namespace rws */

#endif /* RWS_PLOTSIMPLEVIEW_HPP_ */
