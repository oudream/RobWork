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

#include "PlotSimpleView.hpp"
#include "ImageUtil.hpp"

#include <rw/graphics/Plot.hpp>
#include <rw/graphics/PlotGenerator.hpp>
#include <rw/sensor/Image.hpp>

#include <QGraphicsPixmapItem>
#include <QGraphicsView>
#include <QResizeEvent>

using rw::core::ownedPtr;
using namespace rw::graphics;
using rw::sensor::Image;
using namespace rws;

namespace {
class OwnGraphicsView: public QGraphicsView {
    public:
        OwnGraphicsView(QWidget* parent):
            QGraphicsView(parent),
            _scene(new QGraphicsScene()),
            _pixmap(_scene->addPixmap(QPixmap(0, 0)))
        {
            setBackgroundBrush(Qt::SolidPattern);
            setScene(_scene);
            setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
        }

        ~OwnGraphicsView()
        {
        }

        void setPlot(const Plot::Ptr& plot)
        {
            _curPlot = plot;
        }

    protected:
        void resizeEvent (QResizeEvent* event)
        {
            const int width  = event->size ().width ();
            const int height = event->size ().height ();
            render(width, height);
            QGraphicsView::resizeEvent(event);
            _scene->setSceneRect(_scene->itemsBoundingRect());
        }

    public:
        void render(unsigned int w = 0, unsigned int h = 0)
        {
            if (w == 0 && h == 0) {
                w = width();
                h = height();
            }
            if (!_curPlot.isNull()) {
                const Image::Ptr rendering = _curPlot->render(w, h);
                const QImage* const qimage = ImageUtil::toQtImage(*rendering);
                _pixmap->setPixmap(QPixmap::fromImage (*qimage));
                delete qimage;
            }
        }

        QGraphicsScene* const _scene;
        QGraphicsPixmapItem* const _pixmap;
        Plot::Ptr _curPlot;
};
}

class PlotSimpleView::PrivateImpl {
    public:
        PrivateImpl():
            _generators(PlotGenerator::Factory::getPlotGenerators())
        {
        }
        ~PrivateImpl() {}

        const std::vector<std::string> _generators;
        PlotGenerator::Ptr _curGen;
        Plot::Ptr _curPlot;
};

PlotSimpleView::PlotSimpleView():
    PlotView(),
    _impl(new PrivateImpl())
{
}

PlotSimpleView::~PlotSimpleView()
{
    delete _impl;
}

void PlotSimpleView::listPlot (const std::vector< double >& x,
        const std::vector< double >& y,
        const std::string& title,
        const std::string& xlabel,
        const std::string& ylabel)
{
    if (_impl->_curGen.isNull()) {
        if (_impl->_generators.size() > 0)
            _impl->_curGen = PlotGenerator::Factory::getPlotGenerator(_impl->_generators[1]);
    }
    if (!_impl->_curGen.isNull()) {
        _impl->_curPlot = _impl->_curGen->makePlot();
        _impl->_curPlot->listPlot(x, y, title, xlabel, ylabel);
    }
}

QGraphicsView* PlotSimpleView::getWidget(QWidget* parent)
{
    OwnGraphicsView* view = new OwnGraphicsView(parent);
    view->setPlot(_impl->_curPlot);
    view->render();
    return view;
}
