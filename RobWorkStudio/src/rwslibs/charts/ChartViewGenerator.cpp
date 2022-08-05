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

#include "ChartViewGenerator.hpp"
#include "ChartView.hpp"

#include <rws/ImageUtil.hpp>

using rw::graphics::Plot;
using rw::sensor::Image;
using namespace rws;

namespace {
class ChartViewPlot: public Plot
{
    public:
        ChartViewPlot():
            Plot(),
            _view(new ChartView())
        {
            std::cout << "ChartViewPlot Create" << std::endl;
        }

        virtual ~ChartViewPlot()
        {
            std::cout << "ChartViewPlot Delete" << std::endl;
            delete _view;
        }

        void listPlot (const std::vector< double >& x, const std::vector< double >& y,
                       const std::string& title, const std::string& xlabel,
                       const std::string& ylabel)
        {
            _view->listPlot(x, y, title, xlabel, ylabel);
        }

        Image::Ptr render(unsigned int width, unsigned int height)
        {
            std::cout << "ChartViewGenerator rendering.." << width << " " << height << std::endl;
            QGraphicsView* const widget = _view->getWidget(0);
            widget->resize(width, height);
            std::cout << "ChartViewGenerator rendering.. A" << std::endl;
            QPixmap p = widget->grab();
            Image::Ptr image = ImageUtil::toRwImage(p.toImage());
            std::cout << "ChartViewGenerator rendering.. C" << std::endl;
            image->saveAsPPM("test.PPM");
            delete widget;
            std::cout << "ChartViewGenerator rendering.. D" << std::endl;
            return image;
        }

    private:
        ChartView* const _view;
};
}

ChartViewGenerator::ChartViewGenerator():
    PlotGenerator()
{
}

ChartViewGenerator::~ChartViewGenerator()
{
}

Plot::Ptr ChartViewGenerator::makePlot()
{
    return new ChartViewPlot();
}
