/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ********************************************************************************/

#ifndef RW_COMMON_EXTENSIONPOINT_HPP
#define RW_COMMON_EXTENSIONPOINT_HPP

#include <rw/core/ExtensionPoint.hpp>
#include <rw/core/os.hpp>

/**
 * @brief Deprecated namespace since 16/4-2020 for this class
 * @deprecated use rw::core not rw::common
 */
namespace rw { namespace common {
    template< class ExtensionInterface >
    using ExtensionPoint = rw::core::ExtensionPoint< ExtensionInterface >;
}}    // namespace rw::common

#ifdef RW_WIN32
#pragma message( \
    "#include <rw/common/ExtensionPoint.hpp> is deprecated use #include <rw/core/ExtensionPoint.hpp> instead")
#else
#warning \
    "#include <rw/common/ExtensionPoint.hpp> is deprecated use #include <rw/core/ExtensionPoint.hpp> instead"
#endif

#endif