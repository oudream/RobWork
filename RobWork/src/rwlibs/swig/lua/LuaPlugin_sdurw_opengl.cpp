/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#include <LuaPlugin.hpp>
#include "LuaState.hpp"
#include "Lua_sdurw_opengl.hpp"

using namespace rw::common;
using namespace rwlibs::swig;

RW_ADD_PLUGIN(LuaPlugin)

namespace {
struct RWLuaLibrary: LuaState::LuaLibrary {
    virtual const std::string getId()
    {
        return "RWLua_sdurw_opengl";
    }

    virtual bool initLibrary(LuaState& state)
    {
        openLuaLibRW_sdurw_opengl( state.get() );
        return true;
    };
};
}

LuaPlugin::LuaPlugin():
    Plugin("RWLuaPlugin_sdurw_opengl", "RWLuaPlugin_sdurw_opengl", "0.1")
{
}

LuaPlugin::~LuaPlugin()
{
}

std::vector<Extension::Descriptor> LuaPlugin::getExtensionDescriptors()
{
    std::vector<Extension::Descriptor> exts;
    exts.push_back(Extension::Descriptor("RWLua_sdurw_opengl","rwlibs.swig.LuaState.LuaLibrary"));
    exts.back().getProperties().set<std::string>("ID", "sdurw_opengl");
    return exts;
}

Extension::Ptr LuaPlugin::makeExtension(const std::string& str)
{
    if(str=="RWLua_sdurw_opengl") {
        Extension::Ptr extension = rw::common::ownedPtr( new Extension("RWLua_sdurw_opengl","rwlibs.swig.LuaState.LuaLibrary",
                this, ownedPtr(new RWLuaLibrary()) ) );
        extension->getProperties().set<std::string>("ID", "sdurw_opengl");
        return extension;
    }
    return NULL;
}
