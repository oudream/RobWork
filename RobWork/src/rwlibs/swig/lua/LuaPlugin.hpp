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

#ifndef RWLIBS_SWIG_LUA_LUAPLUGIN_HPP_
#define RWLIBS_SWIG_LUA_LUAPLUGIN_HPP_

/**
 * @file LuaPlugin.hpp
 *
 * \copydoc rwlibs::swig::LuaPlugin
 */

#include <rw/common/Plugin.hpp>

namespace rwlibs {
    namespace swig {

        //! @addtogroup swig

        //! @{
        /**
         * @brief A Lua plugin that define extensions for rwlibs.swig.LuaState.LuaLibrary.
         *
         * One plugin is generated for each of the Lua modules in RobWork (except the rw module itself).
         */
        class LuaPlugin: public rw::common::Plugin
        {
            public:
                //! @brief Constructor.
                LuaPlugin();

                //! @brief Destructor.
                virtual ~LuaPlugin();

                //! @copydoc rw::common::Plugin::getExtensionDescriptors
                std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();

                //! @copydoc rw::common::Plugin::makeExtension
                rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& str);
        };
    //! @}

    } /* namespace swig */
} /* namespace rwlibs */

#endif /* RWLIBS_SWIG_LUA_LUAPLUGIN_HPP_ */