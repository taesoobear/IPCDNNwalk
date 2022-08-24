#include "stdafx.h"
#include "../BaseLib/utility/scoped_ptr.h"
#include "../BaseLib/utility/configtable.h"
#include "../BaseLib/utility/operatorString.h"

#include "../MainLib/WrapperLua/luna.h"
#include "../MainLib/WrapperLua/LUAwrapper.h"
#ifndef NO_GUI
#include <Fl/Fl_Group.H>
#include <OgrePrerequisites.h>
#include <OgreColourValue.h>
#include <OgreMovableObject.h>
#endif
#include "../MainLib/OgreFltk/FlLayout.h"
#include "../MainLib/WrapperLua/luna_mainlib.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../MainLib/OgreFltk/VRMLloaderView.h"
#include "../BaseLib/motion/VRMLloader_internal.h"
#define USE_LUNA_PCALL
void Register_lunatest2(lua_State*L);
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);

//#include "RMatrixLUA.h"

//#include "dependency_support/octave_wrap.h"


#include "../MainLib/OgreFltk/framemoveobject.h"
#include "ScriptWin.h"
static void handleLUAerror(lua_State* L)
{
	Msg::print("handleLUAerror:\n");
	luna_printStack(L);
	luaL_dostring(L, "dbg.traceBack()");
	luaL_dostring(L, "dbg.console()");
}
class GlobalUI;
GlobalUI* getGlobalUI();

inline void getglobal(lunaStack& l, const char* func)
{
	l.getglobal(func);
	if(lua_isnil(l.L,-1)){
		Msg::error("%s  is nil!!!\n", func);
	}
}
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
	Msg::print("-%s", &out[start]);
	fflush(stdout);
	prevOut=out;
}

static void printFlush(const char* out)
{
	Msg::print("%s", out);
	fflush(stdout);
}

#define CATCH_LUABIND_ERROR(x,y)

#ifdef _MSC_VER
TString getCurrentDirectory();
#endif
void ScriptWin::show()
{
#ifndef NO_GUI
	if(m_renderer) m_renderer->setHandler(this);
#endif
	FlLayout::show();
}
void ScriptWin::hide()
{
	FlLayout::hide();
}




static VRMLTransform* VRMLloader_upcast(Bone& bone)
{
	return dynamic_cast<VRMLTransform*>(&bone);
}



static TString defaultScriptFolder="../Resource/scripts/ui/ScriptBaseWin/";

void ScriptWin::setLabel(Fl_Button* b, TString const& name)
{
//	ASSERT(name.left(defaultScriptFolder.length()).toUpper()==defaultScriptFolder.toUpper());
//	b->copy_label(name.right(-1*defaultScriptFolder.length()).ptr());
	TString fn=name;
	sz0::filename()(fn);
	b->copy_label(fn.ptr());
	_scriptFn=name;
}

TString ScriptWin::getLabel(Fl_Button* b)
{
	//return defaultScriptFolder+b->label();
	return _scriptFn;
}

ScriptWin::ScriptWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* defaultScript, const char* _defaultScriptFolder)
: FlLayout(x,y,w,h),
m_motionPanel(&mp),
m_renderer(&renderer)
{
	errorFunc=0;
	default_script =defaultScript;
	defaultScriptFolder=_defaultScriptFolder;
	if (_defaultScriptFolder=="")
	{
		int i=default_script.findCharRight('/')+1;
		defaultScriptFolder=default_script.left(i);
		default_script=default_script.right(-1*i);
	}

	setUniformGuidelines(10);
	create("Button", "scriptfn", "scriptfn", 0,7);
	setLabel(button(0), defaultScriptFolder+default_script);
#ifndef NO_GUI
	button(0)->shortcut('o');
	button(0)->tooltip("o");
#endif

	create("Button", "load", "load", 7,9);
#ifndef NO_GUI
#ifdef __APPLE__
	button(0)->shortcut('a');
	button(0)->tooltip("a");
#else
	button(0)->shortcut(FL_ALT+'l');
	button(0)->tooltip("ALT+l");
#endif
#endif
	create("Button", "X", "X", 9);
	resetToDefault();

	L=NULL;

	updateLayout();

#ifdef NO_GUI
	renderer.ogreRenderer().addFrameMoveObject(this);
#else
	renderer.setHandler(this);
	renderer.ogreRenderer().addFrameMoveObject(this);
#endif
}

ScriptWin::ScriptWin(int w, int h, const char* title, const char* defaultScript, const char* _defaultScriptFolder)
: FlLayout(w,h,title),
m_motionPanel(NULL),
m_renderer(NULL)
{
	errorFunc=0;
	default_script =defaultScript;
	defaultScriptFolder=_defaultScriptFolder;

	L=NULL;
}

void ScriptWin::checkErrorFunc(lunaStack&l)
{
#ifdef USE_LUNA_PCALL
	if (errorFunc==0){
		l.getglobal("dbg", "console");
		errorFunc=l.gettop();
		if (errorFunc==0) Msg::print("Warning! cannot find the error function!\n");
	}
#endif
}
void ScriptWin::luna_call(lunaStack& l,int numIn, int numOut)
{
#ifdef USE_LUNA_PCALL
	checkErrorFunc(l);
	if(lua_pcall(l.L,numIn,numOut,errorFunc))
	{
		Msg::print("Error in ScriptWin::luna_call.\n");
		handleLUAerror(l.L);
	}
#else
	lua_call(l.L,numIn,numOut);
#endif
	l.setCheckFromTop();
}

#include "../MainLib/WrapperLua/luna_baselib.h"
#include "../BaseLib/utility/QPerformanceTimer.h"

static lua_State* loadScript(FlLayout* win, const char* script);
void ScriptWin::releaseScript()
{
	FlLayout& l=*this;
	if(L)
	{
#ifdef LUNA_GEN_PROFILER
		FractionTimer::printSummary("test", "C++", "lua");
#endif


		lunaStack l(L);
		getglobal(l,"dtor");
		luna_call(l,0,0);

		lua_close(L);
		L=NULL;
	}

	//int wi=l.widgetIndex("load");
	int wi=l.widgetIndex("X");

	if(wi!=2)
		l.removeWidgets(wi+1);

	L=NULL;
}

ScriptWin::~ScriptWin(void)
{
	releaseScript();
}

	
TString FlChooseFile(const char* message, const char* path, const char* Mask, bool bCreate);
void ScriptWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="scriptfn")
	{

		TString new_script=FlChooseFile("choose a script", defaultScriptFolder.ptr(), "*.lua");
		if(new_script.length())
		{
#ifdef _MSC_VER
			{
				Msg::print("scriptfn: %s\n", new_script.ptr());
				LUAwrapper L;
				if(IsFileExist("../MainLib/WrapperLua/mylib.lua"))
					L.dofile("../MainLib/WrapperLua/mylib.lua");
				else
					L.dofile("../../taesooLib/MainLib/WrapperLua/mylib.lua");
				L.getglobal("os", "absoluteToRelativePath");
				L<<new_script;
				L.call(1,1);
				L>>new_script;
				Msg::print("scriptfn: %s\n", new_script.ptr());
			}
#endif
			setLabel(findButton("scriptfn"), new_script);
			redraw();

			callCallbackFunction(findWidget("load"));
		}


	}
	else if(w.mId=="load")
	{
		releaseScript();
#ifdef _MSC_VER
		printf("load %d\n", w.button()->value());
#endif
		loadScript( getLabel(findButton("scriptfn")));
	}
	else if(w.mId=="X")
	{
		releaseScript();
	}
	else
	{
		lunaStack l(L);
		getglobal(l,"onCallback");
		l.push<FlLayout::Widget>(&w);
		l<<(double)userData;
		luna_call(l,2,0);
	}
}

#ifndef NO_GUI
void ScriptWin::OnFrameChanged(FltkMotionWindow*, int currFrame)
{
	if(L)
	{
		lunaStack l(L);

		getglobal(l,"onFrameChanged");
		l<<(double)currFrame;
		luna_call(l,1,0);
	}

}
#endif


// FrameMoveObject
int ScriptWin::FrameMove(float fElapsedTime)
{
#ifdef _MSC_VER
	if(getCurrentDirectory().find("script")!=-1)
		printf("error\n");
#endif
	if (L){
		lunaStack l(L);
		getglobal(l,"frameMove");
		l<<(double)fElapsedTime;
		luna_call(l,1,0);
	}
	return 1;
}



FlLayout* createScriptWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* a, const char* b)
{
	return new ScriptWin(x,y,w,h,mp, renderer,
			a,b);
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
	//Register_physicsbind(L);
	lunaStack ls(L);
	ls.set<FlLayout>("this", win);
	ls.set<GlobalUI>("GUI", getGlobalUI());
	luaL_dostring(L," if test then local test_names={'TRect', 'CImage', 'intvectorn', 'intmatrixn', 'bitvectorn', 'TStrings', 'binaryFile', 'boolN', 'vector3','quater','vector3N','vector3NView','quaterN', 'quaterNView', 'vectorn','vectornView', 'matrixn', 'matrixnView','matrix3','matrix4','transf'} for i,v in ipairs(test_names) do _G[v]=test[v] end for k, v in pairs(test.math) do math[k]=v end USE_LUNA_GEN=true end");
	return L;
}


static void _loadScript(lua_State* L, FlLayout* win, const char* script)
{
	if(script && luaL_dofile(L, script)==1)
		handleLUAerror(L);
}

void ScriptWin::initLuaEnvironment()
{
	L=_initLuaEnvironment(this);
	_loadScript(L, this, "config.lua");
#ifdef USE_LUNA_PCALL
	lunaStack l(this->L);
	checkErrorFunc(l);
#endif
}
void ScriptWin::loadScript(const char* script, const char* scriptstring)
{
	initLuaEnvironment();

	char luastring[2000];
	sprintf(luastring, "g_luaScript='%s'", script);
	luaL_dostring(L, luastring);
	::_loadScript(L, this, script);

	if(scriptstring)
		luaL_dostring(L, scriptstring);
	if(script){
		lunaStack l(L);
		getglobal(l,"ctor");
		if(lua_isnil(l.L, -1))
			Msg::error("error! ctor is nil");
		else
			luna_call(l,0,0);
	}
}



void VRMLTransform_scaleMass(VRMLTransform& bone, m_real scalef)
{
	if(bone.mSegment)
	{
		bone.mSegment->mass*=scalef;
		bone.mSegment->momentsOfInertia*=scalef;
	}
}



int ScriptWin::work(TString const& workname, lunaStack& L)
{
	//printf("%s:work\n", workname.ptr());
	if(workname=="exit")
	{
		releaseScript();
		RE::FltkRenderer().onCallback(NULL, Hash("softKill"));
	}
	else if(workname=="exit!")
	{
		releaseScript();
		RE::FltkRenderer().onCallback(NULL, Hash("softKill"));
		exit(0);
	}
	else if(workname=="getState")
	{
		L.push<lunaState>(new lunaState(L.L), true);
		return 1;
	}
	else if(workname=="callCallbackFunction")
	{
		TString str;
		L>>str;
		callCallbackFunction(findWidget(str));		
	}
	else if(workname=="load")
	{
		TString str;
		L>>str;
		printf("%s:work\n", str.ptr());
		if(str.length())
		{
			setLabel(findButton("scriptfn"), str);
			redraw();

			callCallbackFunction(findWidget("load"));
		}
	}
	else if(workname=="_initLuaEnvironment")
	{
		releaseScript();
		initLuaEnvironment();
	}
	else if(workname=="_load")
	{
		TString str;
		L>>str;
		printf("%s\n",str.ptr());
		if(str.length())
		{
			setLabel(findButton("scriptfn"), str);
			redraw();
			::_loadScript(this->L, this, getLabel(findButton("scriptfn")));
		}
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
	else 
		return FlLayout::work(workname, L);
}




bool OIS_event_ctrl();
bool OIS_event_shift();
bool OIS_event_alt();
#ifndef NO_GUI
#include <FL/Fl.H>

int ScriptWin::handleRendererMouseEvent(int ev, int x, int y, int button)
{
	if(!visible() || !OIS_event_shift() ) return 0;
	TString evs="";
	switch(ev)
	{
	case FL_PUSH:
		evs="PUSH";
		break;
	case FL_MOVE:
		evs="MOVE";
		break;
	case FL_DRAG:
		evs="DRAG";
		break;
	case FL_RELEASE:
		evs="RELEASE";
		break;
	}

	lunaStack l(L);
	l.getglobal("handleRendererEvent");
	if(lua_isnil(l.L, -1))
	{
		return 0;
	}
	else
	{
		l<<evs<<(double)button<<(double)x<<(double)y;
		luna_call(l,4,1);
		int out;
		l>>out;
		return int(out);
	}
}

int	ScriptWin::handleRendererEvent(int ev) 
{	
//	if(visible()) RE::output("shifted", "%d", Fl::event_state()&FL_SHIFT);

	if(!visible() || !OIS_event_shift() ) return 0;

	//printf("event:%s\n",ev);
	TString evs="";
	//printf("key:%d\n",key);

	switch(ev)
	{
	case FL_PUSH:
	case FL_MOVE:
	case FL_DRAG:
	case FL_RELEASE:
		{
		int button=Fl::event_button();
		int x=Fl::event_x();
		int y=Fl::event_y();
		return handleRendererMouseEvent(ev, x, y, button);
		}
	case FL_KEYUP:
	case FL_KEYDOWN:
		{
			switch(ev)
			{
				case FL_KEYUP: evs="KEYUP"; break;
				case FL_KEYDOWN: evs="KEYDOWN"; break;
			}
			int key;
			lunaStack l(L);
			key=Fl::event_key();
			l.getglobal("handleRendererEvent");
			if(lua_isnil(l.L, -1))
			{
				return 0;
			}
			else
			{
				TString keys;
				if(key>='a' && key<='z')
					keys.format("%c", key);
				else
					keys.format("%d", key);
				l<<evs<<keys;
				luna_call(l,2,1);
				double out;
				l>>out;
				return int(out);
			}
		}
		break;
	}

	return 0;
}
#endif
