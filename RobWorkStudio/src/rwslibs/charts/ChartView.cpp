/*****************************************************************************
 * Copyright 2021 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ChartView.hpp"
#include <RobWorkStudioConfig.hpp>

#include <QAction>
#include <QChartGlobal>
#include <QChartView>
#include <QLineSeries>
#include <QMenu>
#include <QScatterSeries>

using rw::core::ownedPtr;
using namespace rws;

#ifdef RWS_USE_QT5
#include <QtCharts>
#include <QtCharts/QLineSeries>
//#include <QtCharts/QChartView>
using namespace QtCharts;
#endif 

namespace {
class OwnChartView: public QChartView {
    public:
        OwnChartView(QWidget* parent):
            QChartView(parent)
        {
            setRenderHint(QPainter::Antialiasing);
            chart()->setDropShadowEnabled(false);

            _zoomAction = new QAction("&Zoom", this);
            _resetZoomAction = new QAction("&Reset", this);

            connect(_zoomAction, &QAction::triggered, this, &OwnChartView::zoom);
            connect(_resetZoomAction, &QAction::triggered, this, &OwnChartView::resetZoom);
        }

        ~OwnChartView()
        {
        }

        void plot (const rw::core::Ptr<QLineSeries>& series)
        {
            _series = series;
            chart()->addSeries(_series.get());
        }

    protected:
        void contextMenuEvent(QContextMenuEvent* event)
        {
            QMenu menu(this);
            menu.addAction(_zoomAction);
            menu.addAction(_resetZoomAction);
            menu.exec(event->globalPos());
        }

    private:
        void zoom()
        {
            //setRubberBand(QChartView::HorizontalRubberBand);
            setRubberBand(QChartView::RectangleRubberBand);
        }

        void resetZoom()
        {
            chart()->zoomReset();
        }

    private:
        rw::core::Ptr<QLineSeries> _series;
        QAction* _zoomAction;
        QAction* _resetZoomAction;
};
}

struct ChartView::PrivateImpl {
    PrivateImpl()
    {
        series = ownedPtr(new QLineSeries());
    }

    rw::core::Ptr<QLineSeries> series;
    QString title;
    QString xlabel;
    QString ylabel;
};

ChartView::ChartView():
    PlotView(),
    _impl(new PrivateImpl())
{
}

ChartView::~ChartView()
{
    delete _impl;
}

void ChartView::listPlot (const std::vector< double >& x,
        const std::vector< double >& y,
        const std::string& title,
        const std::string& xlabel,
        const std::string& ylabel)
{
    QLineSeries& series = *_impl->series;
    series.clear();
    series.setName(QString::fromStdString(title));
    //series.setMarkerShape(QLineSeries::MarkerShapeCircle);
    //series.setMarkerSize(5.0);

    for (std::size_t i = 0; i < x.size(); i++)
        series.append(x[i], y[i]);

    _impl->title = QString::fromStdString(title);
    _impl->xlabel = QString::fromStdString(xlabel);
    _impl->ylabel = QString::fromStdString(ylabel);
}

QGraphicsView* ChartView::getWidget(QWidget* parent)
{
    OwnChartView* view = new OwnChartView(parent);
    view->plot(_impl->series);
    view->chart()->setTitle(_impl->title);
    view->chart()->createDefaultAxes();
    view->chart()->axes(Qt::Horizontal)[0]->setTitleText(_impl->xlabel);
    view->chart()->axes(Qt::Vertical)[0]->setTitleText(_impl->ylabel);
    return view;
}

ChartView::Dispatcher::Dispatcher():
    PlotView::Dispatcher()
{
}

ChartView::Dispatcher::~Dispatcher()
{
}

PlotView::Ptr ChartView::Dispatcher::makePlotView() const
{
    return ownedPtr(new ChartView());
}
