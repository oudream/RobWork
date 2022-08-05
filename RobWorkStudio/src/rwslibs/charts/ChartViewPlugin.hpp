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

#ifndef RWSLIBS_CHARTS_CHARTVIEWPLUGIN_HPP_
#define RWSLIBS_CHARTS_CHARTVIEWPLUGIN_HPP_

#include <rw/core/Plugin.hpp>

namespace rws {

//! @brief Plugin providing plots based on QChartView.
class ChartViewPlugin: public rw::core::Plugin
{
    public:
        //! @brief Constructor.
        ChartViewPlugin();

        //! @brief Destructor.
        virtual ~ChartViewPlugin();

        //! @copydoc Plugin::getExtensionDescriptors
        virtual std::vector< rw::core::Extension::Descriptor > getExtensionDescriptors ();

        //! @copydoc Plugin::makeExtension
        virtual rw::core::Ptr< rw::core::Extension > makeExtension (const std::string& id);
};

} /* namespace rws */

#endif /* RWSLIBS_CHARTS_CHARTVIEWPLUGIN_HPP_ */
