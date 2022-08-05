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

#include "ChartViewPlugin.hpp"
#include "ChartView.hpp"
#include "ChartViewGenerator.hpp"

using rw::graphics::PlotGenerator;
using namespace rw::core;
using namespace rws;

RW_ADD_PLUGIN(ChartViewPlugin)

ChartViewPlugin::ChartViewPlugin():
    Plugin("rws.PlotView", "QChartView based PlotViewer.", "1.0")
{
}

ChartViewPlugin::~ChartViewPlugin()
{
}

std::vector< Extension::Descriptor > ChartViewPlugin::getExtensionDescriptors ()
{
    std::vector< Extension::Descriptor > exts;
    exts.push_back(Extension::Descriptor("rwslibs.charts.qchartview","rws.PlotView"));
    exts.back().name = "QChartView";
    exts.back().getProperties().set<std::string>("identifier", "rwslibs.charts.qchartview");
    exts.push_back(Extension::Descriptor("rwslibs.charts.qchartgenerator","rw.graphics.PlotGenerator"));
    exts.back().name = "QChartView Generator";
    exts.back().getProperties().set<std::string>("generator", "rwslibs.charts.qchartgenerator");
    return exts;
}

Extension::Ptr ChartViewPlugin::makeExtension (const std::string& id)
{
    if(id == "rwslibs.charts.qchartview") {
        const Extension::Ptr extension = ownedPtr(new Extension("rwslibs.charts.qchartview","rws.PlotView", this, ownedPtr(new ChartView::Dispatcher).cast< const PlotView::Dispatcher > () ) );
        extension->getProperties().set<std::string>("identifier", "rwslibs.charts.qchartview");
        return extension;
    }
    else if(id == "rwslibs.charts.qchartgenerator") {
        const Extension::Ptr extension = ownedPtr(new Extension("rwslibs.charts.qchartgenerator","rw.graphics.PlotGenerator", this, ownedPtr(new ChartViewGenerator).cast< PlotGenerator > () ) );
        extension->getProperties().set<std::string>("generator", "rwslibs.charts.qchartgenerator");
        return extension;
    }
    return nullptr;
}
