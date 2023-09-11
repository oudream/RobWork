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

#ifndef RW_COMMON_PTR_HPP
#define RW_COMMON_PTR_HPP

#include <rw/core/Ptr.hpp>
#include <rw/core/os.hpp>

/**
 * @brief Deprecated namespace since 16/4-2020 for this class
 * @deprecated use rw::core not rw::common
 */
namespace rw { namespace common {
    template<class T> using Ptr = rw::core::Ptr<T>;

    /**
     * @brief A Ptr that takes ownership over a raw pointer \b ptr.
     * @relates Ptr
     */
    template<class T> Ptr<T> ownedPtr(T* ptr) {
        return Ptr<T>(typename Ptr<T>::shared_ptr(ptr));
    }
}}    // namespace rw::common

#ifdef RW_WIN32
#pragma message("#include <rw/common/Ptr.hpp> is deprecated use #include <rw/core/Ptr.hpp> instead")
#else
#warning "#include <rw/common/Ptr.hpp> is deprecated use #include <rw/core/Ptr.hpp> instead"
#endif
#endif