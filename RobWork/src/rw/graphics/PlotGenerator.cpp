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

#include "PlotGenerator.hpp"

using namespace rw::core;
using rw::graphics::PlotGenerator;

PlotGenerator::Factory::Factory():
    ExtensionPoint< PlotGenerator > ("rw.graphics.PlotGenerator", "Generator that creates plots.")
{
}

PlotGenerator::Ptr
PlotGenerator::Factory::getPlotGenerator (const std::string& implementation)
{
    PlotGenerator::Factory ep;
    const std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (const Extension::Ptr& ext : exts) {
        if (ext == nullptr)
            continue;
        if (ext->getProperties ().get ("generator", ext->getName ()) == implementation) {
            return ext->getObject ().cast< PlotGenerator > ();
        }
    }
    return nullptr;
}

bool PlotGenerator::Factory::hasPlotGenerator (const std::string& implementation)
{
    PlotGenerator::Factory ep;
    const std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (const Extension::Descriptor& ext : exts) {
        if (ext.getProperties ().get ("generator", ext.name) == implementation)
            return true;
    }
    return false;
}

std::vector< std::string > PlotGenerator::Factory::getPlotGenerators ()
{
    PlotGenerator::Factory ep;
    std::vector< std::string > ids;
    const std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (const Extension::Descriptor& ext : exts) {
        ids.push_back (ext.getProperties ().get ("generator", ext.name));
    }
    return ids;
}
