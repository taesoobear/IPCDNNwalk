
#ifndef FL_LAYOUT_LUA_H_
#define FL_LAYOUT_LUA_H_
#pragma once


namespace FlLayout_lua
{
	FlLayout* create(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* filename);
	lua_State* getLuaState(FlLayout* luawin);
	void callConstructor(FlLayout* luawin);
}



#endif
