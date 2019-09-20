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

#ifndef RWLIBS_SWIG_LUA_rw_pathoptimization_HPP_
#define RWLIBS_SWIG_LUA_rw_pathoptimization_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>


#ifdef __cplusplus
}
#endif

namespace rwlibs {
namespace swig {

    /**
     * @brief Initialize a Lua state.
     * @param L [in/out] the Lua state to add module contents to.
     * @return returns 1 according to Lua documentation.
     */
    int openLuaLibRW_rw_pathoptimization(lua_State* L);

}
}
#endif /* RWLIBS_SWIG_LUA_rw_pathoptimization_HPP_ */
