/*
 * Lua.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: jimali
 */

#ifndef RWSLIBS_SWIG_LUA_HPP_
#define RWSLIBS_SWIG_LUA_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

#ifdef __cplusplus
}
#endif

namespace rwslibs { namespace swig {

    /**
     * @brief initialize a lua state
     * @param L
     * @return
     */
    int openLuaLibRWS(lua_State* L);

}}     // namespace rwslibs::swig
#endif /* LUA_HPP_ */
