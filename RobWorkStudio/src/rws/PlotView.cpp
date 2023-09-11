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

#include "PlotView.hpp"

#include "PlotSimpleView.hpp"

using namespace rw::core;
using rws::PlotView;

PlotView::PlotView() {}

PlotView::~PlotView() {}

PlotView::Dispatcher::Dispatcher() {}

PlotView::Dispatcher::~Dispatcher() {}

PlotView::Factory::Factory() :
    ExtensionPoint<Dispatcher>("rws.PlotView",
                               "Plugin that provides GUI elements for visualization of plots.") {}

PlotView::Ptr PlotView::Factory::makePlotView(const std::string& identifier) {
    PlotView::Factory ep;
    const std::vector<Extension::Ptr> exts = ep.getExtensions();
    for(const Extension::Ptr& ext : exts) {
        if(ext == nullptr) continue;
        if(ext->getProperties().get("identifier", ext->getName()) == identifier) {
            const PlotView::Dispatcher::Ptr dispatcher =
                ext->getObject().cast<const PlotView::Dispatcher>();
            return dispatcher->makePlotView();
        }
    }
    if(identifier == "rws.PlotSimpleView") return ownedPtr(new PlotSimpleView());
    return nullptr;
}

bool PlotView::Factory::hasPlotViewDispatcher(const std::string& identifier) {
    PlotView::Factory ep;
    const std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    for(const Extension::Descriptor& ext : exts) {
        if(ext.getProperties().get("identifier", ext.name) == identifier) return true;
    }
    if(identifier == "rws.PlotSimpleView") return true;
    return false;
}

std::vector<std::string> PlotView::Factory::getIdentifiers() {
    PlotView::Factory ep;
    std::vector<std::string> ids;
    const std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    for(const Extension::Descriptor& ext : exts) {
        ids.push_back(ext.getProperties().get("identifier", ext.name));
    }
    ids.push_back("rws.PlotSimpleView");
    return ids;
}
