#include <iostream>
#include <rwlibs/swig/lua/Lua_rw.hpp>
#include <rwlibs/swig/lua/Lua_rw_assembly.hpp>
#include <rwlibs/swig/lua/Lua_rw_control.hpp>
#include <rwlibs/swig/lua/Lua_rw_pathoptimization.hpp>
#include <rwlibs/swig/lua/Lua_rw_pathplanners.hpp>
#include <rwlibs/swig/lua/Lua_rw_proximitystrategies.hpp>
#include <rwlibs/swig/lua/Lua_rw_simulation.hpp>
#include <rwlibs/swig/lua/Lua_rw_task.hpp>

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <lua-script>\n";
        return 1;
    }

    lua_State *L = luaL_newstate();
    luaL_openlibs(L);

    rwlibs::swig::openLuaLibRW_rw(L);
    rwlibs::swig::openLuaLibRW_rw_assembly(L);
    rwlibs::swig::openLuaLibRW_rw_control(L);
    rwlibs::swig::openLuaLibRW_rw_pathoptimization(L);
    rwlibs::swig::openLuaLibRW_rw_pathplanners(L);
    rwlibs::swig::openLuaLibRW_rw_proximitystrategies(L);
    rwlibs::swig::openLuaLibRW_rw_simulation(L);
    rwlibs::swig::openLuaLibRW_rw_task(L);

    const int error = luaL_dofile(L, argv[1]);
    if (error) std::cerr << lua_tostring(L, -1) << "\n";
    lua_close(L);

    return error;
}
