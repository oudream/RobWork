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

#include "MathGLPlugin.hpp"
#include "MathGLPlotGenerator.hpp"

#include <rw/core/Extension.hpp>

using namespace rw::core;
using rw::graphics::PlotGenerator;
using rwlibs::plots::MathGLPlugin;

RW_ADD_PLUGIN(MathGLPlugin)

MathGLPlugin::MathGLPlugin():
    Plugin("rwlibs.plots.mathgl", "MathGL", "1.0")
{
}

MathGLPlugin::~MathGLPlugin()
{
}

std::vector< Extension::Descriptor > MathGLPlugin::getExtensionDescriptors ()
{
    std::vector< Extension::Descriptor > exts;
    exts.push_back(Extension::Descriptor("rwlibs.plots.mathgl","rw.graphics.PlotGenerator"));
    exts.back().name = "MathGL";
    exts.back().getProperties().set<std::string>("generator", "rwlibs.plots.mathgl");
    return exts;
}

Extension::Ptr MathGLPlugin::makeExtension (const std::string& id)
{
    if(id == "rwlibs.plots.mathgl") {
        const Extension::Ptr extension = ownedPtr(new Extension("rwlibs.plots.mathgl","rw.graphics.PlotGenerator", this, ownedPtr(new MathGLPlotGenerator).cast< PlotGenerator > () ) );
        extension->getProperties().set<std::string>("generator", "rwlibs.plots.mathgl");
        return extension;
    }
    return nullptr;
}
