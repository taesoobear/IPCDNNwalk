
#include "stdafx.h"

#include "../../MainLib/WrapperLua/luna.h"
class Motion;
#include "../../MainLib/WrapperLua/LUAwrapper.h"
#include "../../BaseLib/image/Image.h"
#include "../../BaseLib/motion/Motion.h"
#include "../../BaseLib/motion/MotionLoader.h"
#include "../../BaseLib/math/bitVectorN.h"
#include "../../MainLib/OgreFltk/framemoveobject.h"
#include "../../MainLib/OgreFltk/timesensor.h"
#include "../../MainLib/OgreFltk/AnimationObject.h"
#include "../../MainLib/OgreFltk/FltkRenderer.h"
#include "../../MainLib/OgreFltk/FltkAddon.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"
#ifndef NO_GUI
#include <Fl/Fl_Group.H>
#include <Fl/Fl_Double_Window.H>
#include <OgrePrerequisites.h>
#include <OgreColourValue.h>
#include <OgreMovableObject.h>
#endif
#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../MainLib/WrapperLua/luna_mainlib.h"
#include "../../MainLib/OgreFltk/VRMLloader.h"
#include "../../MainLib/OgreFltk/VRMLloaderView.h"

void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);

static void releaseScript(lua_State* L, FlLayout& l);

#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"
class FltkRenderer;
class VRMLloader;
#ifndef NO_GUI
class FlLayoutLua: public FlLayout, public FrameMoveObject, public FltkMotionWindow::EventReceiver
#else
class FlLayoutLua: public FlLayout, public FrameMoveObject
#endif
{
public:	
	lua_State* L;

public:

	FlLayoutLua(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~FlLayoutLua(void);

	void loadScript(const char* fileName);
	void callConstructor();

	FltkRenderer& mRenderer;
	MotionPanel& m_motionPanel;

#ifndef NO_GUI
	// FltkMotionWindow::EventReceiver
	virtual void OnFrameChanged(FltkMotionWindow*, int currFrame);
#endif

	virtual int work(TString const& workname, lunaStack& L);

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);

	virtual void show();
	virtual void hide();

};
#include "FlLayoutLua.h"

#define USE_LUNA_PCALL
static void handleLUAerror(lua_State* L)
{
	printf("handleLUAerror:\n");
	luna_printStack(L);
	luaL_dostring(L, "dbg.traceBack()");
	luaL_dostring(L, "dbg.console()");
}
class GlobalUI;
GlobalUI* getGlobalUI();

#ifdef USE_MPI
#include <mpi.h>
#endif

static void fastPrint(const char* out)
{
	static TString prevOut(" ");

	int start;
	for(start=0; out[start]!=0 && prevOut[start]!=0 &&
			out[start]==prevOut[start]; start++)
	{
		// do nothing
	}

	if (start> prevOut.length()-5)
	{
		start=MAX(0, prevOut.length()-5);
	}
	printf("-%s", &out[start]);
	fflush(stdout);
	prevOut=out;
}

static void printFlush(const char* out)
{
	printf("%s", out);
	fflush(stdout);
}

#define CATCH_LUABIND_ERROR(x,y)

#ifdef _MSC_VER
TString getCurrentDirectory();
#endif
void FlLayoutLua::show()
{
	//new OpenHRP::DynamicsSimulator_UT_penalty();
	//	mRenderer.ogreRenderer().fixedTimeStep(true);
	//	mRenderer.ogreRenderer().setCaptureFPS(30);
	FlLayout::show();
}
void FlLayoutLua::hide()
{
	//	mRenderer.ogreRenderer().fixedTimeStep(false);
	//	mRenderer.ogreRenderer().setCaptureFPS(30);
	FlLayout::hide();
}



// calculate all center of masses and inertia tensors based on geometry.



	FlLayoutLua::FlLayoutLua(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: FlLayout(x,y,w,h),
	m_motionPanel(mp),
	mRenderer(renderer)
{
	L=NULL;
	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);
}


static int errorFunc=0;
inline static void checkErrorFunc(lunaStack&l)
{
	if (errorFunc==0){
		l.getglobal("dbg", "console");
		errorFunc=l.gettop();
		if (errorFunc==0) printf("Warning! cannot find the error function!\n");
	}
}
inline static void luna_call(lunaStack& l,int numIn, int numOut)
{
	if(lua_pcall(l.L,numIn,numOut,errorFunc))
		handleLUAerror(l.L);
	l.setCheckFromTop();
	//l.call(numIn, numOut);
}

#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../BaseLib/utility/QPerformanceTimer.h"
static void releaseScript(lua_State* L, FlLayout& l)
{
	if(L)
	{
#ifdef LUNA_GEN_PROFILER
		FractionTimer::printSummary("test", "C++", "lua");
#endif


		lunaStack l(L);
		l.getglobal("dtor");
		luna_call(l,0,0);

		lua_close(L);
		L=NULL;
	}

}


FlLayoutLua::~FlLayoutLua(void)
{
	releaseScript(L, *this);
	L=NULL;
}


void FlLayoutLua::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	lunaStack l(L);
	l.getglobal("onCallback");
	l.push<FlLayout::Widget>(&w);
	l<<(double)userData;
	luna_call(l,2,0);
}

#ifndef NO_GUI
void FlLayoutLua::OnFrameChanged(FltkMotionWindow*, int currFrame)
{
	if(L)
	{
		lunaStack l(L);

		l.getglobal("onFrameChanged");
		l<<(double)currFrame;
		luna_call(l,1,0);
	}

}
#endif


// FrameMoveObject
int FlLayoutLua::FrameMove(float fElapsedTime)
{
#ifdef _MSC_VER
	if(getCurrentDirectory().find("script")!=-1)
		printf("error\n");
#endif
	if (L){
		lunaStack l(L);
		l.getglobal("frameMove");
		l<<(double)fElapsedTime;
		luna_call(l,1,0);
	}
	return 1;
}




FlLayout* FlLayout_lua::create(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* filename)
{
	FlLayoutLua* p=new FlLayoutLua(x,y,w,h,mp, renderer);
	p->loadScript(filename);
	return (FlLayout*)p;
}
lua_State* FlLayout_lua::getLuaState(FlLayout* luawin)
{
	return ((FlLayoutLua*)luawin)->L;
}

void FlLayout_lua::callConstructor(FlLayout* luawin)
{
	return ((FlLayoutLua*)luawin)->callConstructor();
}
// #ifdef _MSC_VER
// void addKFPointTracker (lua_State* L);
// #endif

static int add_file_and_line(lua_State* L)
{
	luaL_dostring(L, "dbg.traceBack()");
	luaL_dostring(L, "dbg.console()");
	return 1;
}

static lua_State* _initLuaEnvironment(FlLayout* win)
{
	lua_State* L=lua_open();
	luaopen_base(L);
#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
	// lua 5.1 or newer
	luaL_openlibs(L);
#else
	// lua 5.0.2 or older
	lua_baselibopen(L);
#endif
	Register_baselib(L);
	Register_mainlib(L);
	lunaStack ls(L);
	ls.set<FlLayout>("this", win);
	ls.set<GlobalUI>("GUI", getGlobalUI());
	luaL_dostring(L," if test then local test_names={'TRect', 'CImage', 'intvectorn', 'intmatrixn', 'bitvectorn', 'TStrings', 'binaryFile', 'boolN', 'vector3','quater','vector3N','vector3NView','quaterN', 'quaterNView', 'vectorn','vectornView', 'matrixn', 'matrixnView','matrix3','matrix4','transf'} for i,v in ipairs(test_names) do _G[v]=test[v] end for k, v in pairs(test.math) do math[k]=v end USE_LUNA_GEN=true end");
	return L;
}
static void _loadScript(lua_State* L, FlLayout* win, const char* script)
{
	if(luaL_dofile(L, script)==1)
		handleLUAerror(L);
}







int FlLayoutLua::work(TString const& workname, lunaStack& L)
{
	printf("%s:work\n", workname.ptr());
	if(workname=="exit")
	{
		releaseScript(this->L, *this);
		RE::FltkRenderer().onCallback(NULL, Hash("softKill"));
	}
	else if(workname=="exit!")
	{
		printf("exit!!!!\n");
		releaseScript(this->L, *this);
		RE::FltkRenderer().onCallback(NULL, Hash("softKill"));
		exit(0);
	}
	else if(workname=="callCallbackFunction")
	{
		TString str;
		L>>str;
		callCallbackFunction(findWidget(str));		
	}
	else if(workname=="_initLuaEnvironment")
	{
		releaseScript(this->L, *this);
		this->L=_initLuaEnvironment(this);
	}
	else if(workname=="dostring" && this->L)
	{
		TString str;
		L>>str;
		printf("%s\n", str.ptr());
#ifdef USE_LUNA_PCALL
		lunaStack l(this->L);
		checkErrorFunc(l);
		int func=luaL_loadstring(this->L, str);
		if(func || lua_pcall(this->L, 0, LUA_MULTRET, errorFunc))
#else
		if (luaL_dostring(this->L, str)==1)
#endif

		{
			printf("dostring:\n");
			handleLUAerror(this->L);
		}
	}
	return 0;
}



void FlLayoutLua::loadScript(const char* fileName)
{
	releaseScript(L, *this);
	L=_initLuaEnvironment(this);
	_loadScript(L, this, fileName);
}
void FlLayoutLua::callConstructor()
{
	lunaStack l(L);
#ifdef USE_LUNA_PCALL
	checkErrorFunc(l);
#endif
	l.getglobal("ctor");
	luna_call(l,0,0);
}
