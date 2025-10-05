#include "stdafx.h"
extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
	//#include "luadebug.h"
}

#include "../OgreFltk/MotionPanel.h"
#include "../BaseLib/motion/MotionRetarget.h"
#include "../BaseLib/motion/MotionUtil.h"
#include "../BaseLib/math/Operator.h"



#include "LUAwrapper.h"

LUAwrapper::LUAwrapper()
	:lunaStack(NULL) // normal stack
{
	errorFunc=0;
	L = lua_open();
	_ownState=true;
	luaopen_base(L);
	luaL_openlibs(L);
	setCheckFromTop();
}
LUAwrapper::LUAwrapper(const char* fn)
{
	errorFunc=0;
	L = lua_open();
	_ownState=true;
	luaopen_base(L);
	luaL_openlibs(L);
	setCheckFromTop();
	dofile(fn);
}

LUAwrapper::LUAwrapper(lua_State* _L)
{
	errorFunc=0;
	L=_L;
	_ownState=false;
	setCheckFromTop();
}
LUAwrapper::~LUAwrapper()
{
	if(_ownState)
		lua_close(L);
	else
	{
		if(errorFunc && errorFunc==gettop())
			lua_pop(L,1);
	}
}

static void handleLUAerror(lua_State* L)
{
	printf("handleLUAerror:\n");
	luna_printStack(L);
	luaL_dostring(L, "dbg.traceBack()");
	luaL_dostring(L, "dbg.console()");
}
void LUAwrapper::registerErrorFunc()
{
	if (errorFunc==0){
		getglobal("dbg", "console");
		errorFunc=gettop();
		if (errorFunc==0) printf("Warning! cannot find the error function!\n");
	}
}
void LUAwrapper::dofile(const char* pFilename )
{
	printf("pFileName=%s\n", pFilename);

	/*	- simple way without error_checking
	lua_dofile(L, pFilename );*/

	if(!IsFileExist(pFilename))
		printf("error1:LUAwrapper\n");
	try {
		if (0 != luaL_loadfile(L, pFilename))
		{
			if(errorFunc) return handleLUAerror(L);
			printf("here\n");
			TString errorMsg;
			errorMsg.format("Lua Error - Script Load\nScript Name:%s\nError Message:%s\n", pFilename, luaL_checkstring(L, -1));
			printf("%s\n", errorMsg.ptr());
			ASSERT(0);
			throw std::runtime_error(errorMsg.ptr());
		}

		if (0 != lua_pcall(L, 0, LUA_MULTRET, errorFunc))
		{
			printf("here2\n");
			TString errorMsg;
			errorMsg.format("Lua Error - Script run\nScript Name:%s\nError Message:%s\n", pFilename, luaL_checkstring(L, -1));
			printf("%s\n", errorMsg.ptr());
			ASSERT(0);
			throw std::runtime_error(errorMsg.ptr());
		}
	}	
	catch(std::exception& e)
	{
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch (char* error)
	{
		Msg::msgBox("%s", error);
	}
	catch(...)
	{
		Msg::msgBox("some error");
		ASSERT(0);
	}

}

void LUAwrapper::dostring(const char* script)
{
	/* -simple way without error_checking
	lua_dostring(L, script);*/

	if (0 != luaL_loadbuffer(L, script, strlen(script), NULL))
	{
		TString errorMsg;
		errorMsg.format("Lua Error - String Load\nString:%s\nError Message:%s\n", script, luaL_checkstring(L, -1));
		throw std::runtime_error(errorMsg.ptr());
	}

	if (0 != lua_pcall(L, 0, LUA_MULTRET, errorFunc))
	{
		TString errorMsg;
		errorMsg.format("Lua Error - String run\nString:%s\nError Message:%s\n", script, luaL_checkstring(L, -1));
		throw std::runtime_error(errorMsg.ptr());
	}

}
void LUAwrapper::pcall(int narg)
{
	pcall(narg, LUA_MULTRET);
}
void LUAwrapper::pcall(int narg, int nout)
{
	if (0 != lua_pcall(L, narg, nout, errorFunc))
	{
		TString errorMsg;
		errorMsg.format("Lua Error - pcall Error Message:%s\n", luaL_checkstring(L, -1));
		throw std::runtime_error(errorMsg.ptr());
	}
	setCheckFromTop();
}

vector3 LUAwrapper::getVec3(const char* szVarname)
{
	assert(false);
	return vector3(0,0,0);
}

double LUAwrapper::getDouble(const char* key)
{
	double val;
	get_double(key, val);
	return val;
}
int LUAwrapper::getInt(const char* key)
{
	int val;
	get_int(key, val);
	return val;
}
void LUAwrapper::get_double(const char* key, double& val)
{
	lua_pushstring(L, key);
	lua_gettable(L, LUA_GLOBALSINDEX);
	val=tonumber(-1);	
	lua_pop(L,1);
}
void LUAwrapper::get_int(const char* key, int& val)
{
	lua_pushstring(L, key);
	lua_gettable(L, LUA_GLOBALSINDEX);
	val=(int)tonumber(-1);	
	lua_pop(L,1);
}
int LUAwrapper::getInt(const char* szName, const char* szParam)
{
	getglobal(szName);

	int numCtr;
	(*this)<< std::string(szParam);
	call(1,1);
	(*this)>> numCtr;
	return numCtr;
}


double LUAwrapper::getDouble(const char* szName, const char* szParam)
{
	getglobal(szName);

	double numCtr;
	(*this)<< std::string(szParam);

	call(1,1);
	(*this)>> numCtr;
	return numCtr;
}
std::string LUAwrapper::getString(const char* szName, const char* szParam)
{
	getglobal(szName);

	std::string numCtr;
	(*this)<< std::string(szParam);
	call(1,1);
	(*this)>> numCtr;
	return numCtr;
}

std::string LUAwrapper::getString(const char* key)
{
	std::string out;
	lua_pushstring(L, key);
	lua_gettable(L, LUA_GLOBALSINDEX);
	out=tostring(-1);
	lua_pop(L,1);
	return out;
}
bool LUAwrapper::getBool(const char* key)
{
	bool out;
	lua_pushstring(L, key);
	lua_gettable(L, LUA_GLOBALSINDEX);
	out=toboolean(-1);
	lua_pop(L,1);
	return out;
}
void LUAwrapper::setString(const char* key, const char* val)
{
	lua_pushstring(L, key);
	lua_pushstring(L, val);
	lua_settable(L, LUA_GLOBALSINDEX);
}
