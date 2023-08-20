
#include "stdafx.h"
#include "luna.h"
void luna_printStack(lua_State* L, bool compact)
{
	if(compact)
		Msg::print("stack top:%d - ", lua_gettop(L)); 
	else
		Msg::print("stack trace: top %d\n", lua_gettop(L)); 

	for(int ist=1; ist<=lua_gettop(L); ist++) {
		if(compact)
			Msg::print("%d:%c",ist,luaL_typename(L,ist)[0]);
		else
			Msg::print("%d:%s",ist,luaL_typename(L,ist));
		if(lua_isnumber(L,ist) ==1) {
			Msg::print("=%f ",(float)lua_tonumber(L,ist));
		} else if(lua_isstring(L,ist) ==1){
			Msg::print("=%s ",lua_tostring(L,ist));
		} else {
			Msg::print(" ");
		}
		if( !compact)Msg::print("\n");
	}
	Msg::print("\n");
}
void luna_dostring(lua_State* L, const char* luacode)
{
	// luaL_dostring followed by pcall error checking 
	if (luaL_dostring(L, luacode)==1)
	{
		Msg::print("Lua error: stack :\n");
		luna_printStack(L,false);
	}
}
luna_derived_object::~luna_derived_object()
{
	if(_l.L)
	{
		_l.getglobal("__luna", "luna_derived_objects");
		_l<<(double)_uniqueID;
		lua_pushnil(_l.L);
		lua_settable(_l.L, -3);
		//printf("dtor called %d\n", _uniqueID);
		//luaL_dostring(_l.L, "dbg.console()");
		_l.pop();
	}
}
void luna_derived_object::call_ctor( int numArg)
{
	int numOut=_l.beginCall(numArg);
	assert(numOut==1);
	_l.endCall(0); // leave the stack as is. The new vector3 object is stored at the top (-1).
	_uniqueID=_storeNewObject(); // store the new object in the __luna.luna_derived_objects table, and remove the object from the stack.
}
int luna_derived_object::_storeNewObject() 
{
	lua_State* L=_l.L;
	int uid;
	_l.getglobal("__luna", "luna_derived_objects");
	if (lua_isnil(L, -1))
	{
		lua_pop(L,1);
		_l.getglobal("__luna");
		if (lua_isnil(L, -1)) printf("error: no __luna table. create __luna table before calling this. \n");
		_l.replaceTop_newTable("luna_derived_objects");
	}

	// __luna.luna_derived_objects[n]=vector3(x,y,z);
	int n=_l.arraySize(-1);
	uid=n+1;
	lua_insert(L, -2);  // swap luna_derived_objects and the new vector3 object.
	_l<<(double)uid;
	lua_insert(L, -2);  // swap the new vector3 object and uniqueID
	lua_settable(L, -3);
	lua_pop(L, 1); // pop luna_derived_objects table
	return uid;
}
void luna_derived_object::pushMemberOnly(const char* name)
{
	_l.getglobal("__luna", "luna_derived_objects");
	_l.replaceTop(_uniqueID);
	lua_getmetatable(_l.L, -1);
	_l.replaceTop(name);
	_l.popSecondLast(); // pop out the self object.
}
void luna_derived_object::pushMemberAndSelf(const char* name)
{
	_l.getglobal("__luna", "luna_derived_objects");
	_l.replaceTop(_uniqueID);
	lua_getmetatable(_l.L, -1);
	_l.replaceTop(name);
	lua_insert(_l.L,-2); // swap the member function and the self object.
}

void luna_derived_object::push(luna_derived_object const& o)
{
	_l.getglobal("__luna", "luna_derived_objects");
	_l.replaceTop(o._uniqueID);
}
lunaStack::~lunaStack()
{
}
std::string lunaStack::lunaType(int i)
{
	if(luaType(i)==LUA_TUSERDATA)
	{
		lua_getmetatable(L,i);
		if(!lua_isnil(L,-1))
		{
			lua_pushstring(L,"luna_class");
			lua_gettable(L,-2);
			if(!lua_isnil(L,-1))
			{
				std::string t=lua_tostring(L,-1);
				lua_pop(L,2);
				return t;
			}
			else lua_pop(L,2);
		}
		return std::string("unknown type");
	}
	return std::string(luaL_typename(L,i));
}
int lunaStack::arraySize(int tblindex)
{
	if (tblindex==-1) tblindex=gettop();
	for (int i=1; 1; i++){
		gettable(tblindex,i);
		if(lua_isnil(L,-1))
		{
			lua_pop(L,1);
			return i-1;
		}
		lua_pop(L,1);
		//luna_printStack(L, true);
	}
}
int lunaStack::treeSize(int tblindex)
{
	if(tblindex==-1) tblindex=gettop();
	if (lua_type(L,tblindex)!=LUA_TTABLE)
		return 1;
	int count=0;
	int arrSize=arraySize(tblindex);
	for (int i=1; i<=arrSize; i++)
	{
		gettable(tblindex,i);
		count+=treeSize(-1);
		//printf("count%d\n", count);
		pop();
	}
	return count+1;
}

int lunaStack::beginCall(int numIn){
	//printStack();
	int func=gettop()-numIn;
	lua_call(L, numIn, LUA_MULTRET);
	// prepare to read-out results in proper order
	setCheckFromBottom();
	//printf("%d %d\n", gettop(), func);
	currArg=func;
	return gettop()-func+1; // returns numOut
}
int lunaStack::beginPcall(int numIn, int errorFunc){
	//printStack();
	int func=gettop()-numIn;
	lua_pcall(L, numIn, LUA_MULTRET, errorFunc);
	// prepare to read-out results in proper order
	setCheckFromBottom();
	//printf("%d %d\n", gettop(), func);
	currArg=func;
	return gettop()-func+1; // returns numOut
}
void lunaStack::endCall(int numOut){
	lua_pop(L,numOut);
	setCheckFromTop(); // return to the normal stack mode.
}

// delete following lines if QPerformanceTimer is not used.
#include "../../BaseLib/utility/QPerformanceTimer.h"
QPerformanceTimerCount2 FractionTimer :: gTimerInside;
QPerformanceTimerCount2 FractionTimer :: gTimerOutside;
int FractionTimer :: gCount;
double FractionTimer :: gOverhead;
