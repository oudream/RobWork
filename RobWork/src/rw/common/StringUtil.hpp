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

#ifndef RW_COMMON_STRINGUTIL_HPP
#define RW_COMMON_STRINGUTIL_HPP

#include <rw/core/StringUtil.hpp>

/**
 * @brief Deprecated namespace since 16/4-2020 for this class
 * @deprecated use rw::core not rw::common
 */
namespace rw { namespace common {
    using StringUtil     = rw::core::StringUtil;
    using StringPair     = rw::core::StringPair;
    using StringPairList = rw::core::StringPairList;
}}    // namespace rw::common

#warning \
    "#include <rw/core/StringUtil.hpp> is deprecated use #include <rw/core/StringUtil.hpp> instead"

#endif