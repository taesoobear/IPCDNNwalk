#ifndef _LUAWRAPPER_H_
#define _LUAWRAPPER_H_
#pragma once

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
	//#include "luadebug.h"
}
#include "luna.h"

#define BEGIN_LUAWRAPPER_GETVAL try{
#define END_LUAWRAPPER_GETVAL }catch(...){Msg::error("in LUAwrapper::getVal(%s)", szVarname);}
class LUAwrapper : public lunaStack
{
	int errorFunc;
	bool _ownState;
public:
	LUAwrapper();
	LUAwrapper(lua_State* L);
	LUAwrapper(const char* fn);
	~LUAwrapper();
	class Worker
	{
	public:
		Worker(){}
		virtual ~Worker(){}
		// reimplement this
		virtual int work(TString const& workname, lunaStack& L)				{return 0;}
	};

	// register dbg.console() 
	void registerErrorFunc();

	lua_State* getState() { return this->L;}
	void call(int narg, int nout) { lunaStack::call(narg, nout); }// output goes to this 
	void pcall(int narg); // output goes to this.
	void pcall(int narg, int nout); // output goes to this.

	void call(int narg, int nout, lunaStack& out) // output goes to out
	{
		out.L=this->L;
		out.call(narg, nout);
	}

	void dofile(const char* script);
	inline void dofile(std::string script) { dofile(script.c_str());}
	void dostring(const char* string);

	template <class T>
	void setVal(const char* szVarName, T const& val)
	{
		set<T>(szVarName, new T(val), LUA_GLOBALSINDEX,true); 
	}

	// getVal<vector3>("pos", pos)
	template <class T>
	void getVal(const char* szVarname, T& val)
	{
		val=*get<T>(szVarname); 
	}

	template <class T>
	T getValue(const char* szVarname)
	{
		T val=*get<T>(szVarname); 
		return val;
	}

	double getDouble(const char* key);
	int getInt(const char* key);
	std::string getString(const char* key);
	bool getBool(const char* key);

	double getDouble(const char* key, const char* param);
	int getInt(const char* key, const char* param);
	std::string getString(const char* key, const char* param);

	void setString(const char* key, const char* val);

	void get_double(const char* key, double& val);
	void get_int(const char* key, int& val);

	// lua: a={1,2,3}
	vector3 getVec3(const char* szVarName);

	template <class T>
	void setRef(const char* szVarName, T & val)
	{
		lunaStack l(L);
		l.set<T>(szVarName, &val, LUA_GLOBALSINDEX,false); 
	}

#ifdef _MSC_VER
	// std::string& name=getRef<std::string>("name") -> name을 고치면 lua에 반영된다.
	template <class T>
	T& getRef(const char* szVarname)
	{
		lunaStack l(L);
		return *l.get<T>(szVarname);
	}
#endif

};



#endif
