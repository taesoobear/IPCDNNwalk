#define NULL 0
#ifdef NO_GUI
#include <MainLib/console/dummies.h>
#endif
#include "stdafx.h"
#include <MainLib/OgreFltk/FlLayout.h>
#include "PythonExtendWin.h"
#include  <boost/python.hpp>
#include <Python.h>
#include "MainlibPython.h"
#include <MainLib/OgreFltk/MotionPanel.h>
#include <MainLib/OgreFltk/FltkRenderer.h>
#include <MainLib/OgreFltk/MovableText.h>
#include <MainLib/OgreFltk/Loader.h>
//#include <MainLib/OgreFltk/Joystick.h"
#include <MainLib/OgreFltk/MotionPanel.h>
//#include <MainLib/OgreFltk/OgreMotionLoader.h"
#include "BaseLib/utility/checkPoints.h"
#include "BaseLib/motion/MotionRetarget.h"
#include "BaseLib/math/hyperMatrixN.h"


	

//////////////////////////////////////////////////////////////////////////////
// extend

#include <boost/python.hpp>
#include <Python.h>
using namespace boost::python;
void PythonExtendWin::__loadScript(const char* script) {
	releaseScript();
	loadScript(script, NULL);
}
PythonExtendWin::PythonExtendWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
//:FlLayout(x,y,w,h),m_motionPanel(mp),mRenderer(renderer)
:ScriptBaseWin(x,y,w,h,mp, renderer,  "WRLviewer.lua",  "../Samples/scripts/RigidBodyWin/")
{
	m_main_module=import("__main__");
	m_main_namespace=m_main_module.attr("__dict__");

	//removeWidgets(-2);
	//setUniformGuidelines(10);
	//create("Button", "X", "X", 9);
	updateLayout();

#ifndef NO_OGRE
	renderer.setHandler(this);
#endif
	renderer.ogreRenderer().addFrameMoveObject(this);
}

PythonExtendWin::~PythonExtendWin(void)
{
}

#include <fstream>
void readFile(TString & a, const char* fn)
{
	std::ifstream inFile;
	inFile.open(fn);
	if(inFile)
	{
		char buf[1000];
		while(!inFile.eof())
		{
			inFile.getline(buf,1000,'\n');
#ifdef _DEBUG
			// import mainlibÀ» import mainlib_debug as mainlibÀ¸?? Ä¡È¯???Ö¾????Ñ´?.

			TString temp=buf;
			if(temp.find("import mainlib")!=-1)
				sprintf(buf, "import mainlib_debug as mainlib");

#endif
			a.add("%s\n", buf);
		}
		inFile.close();
	}
}


void PythonExtendWin::show()
{
	FlLayout::show();
#ifndef NO_GUI
	m_renderer->setHandler(this);
#endif
}

void PythonExtendWin::hide()
{
	FlLayout::hide();
}
void PythonExtendWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="scriptfn"|| w.mId=="load"|| w.mId=="X")
		ScriptBaseWin::onCallback(w, pWidget, userData);
	else
		m_main_namespace["onCallback"](w, userData); 
}

void PythonExtendWin::OnFrameChanged(FltkMotionWindow* p, int currFrame)
{
	m_main_namespace["onFrameChanged"](currFrame); 
}

// PLDPrimSkin::DrawCallback
void PythonExtendWin::draw(const Motion& mot, int iframe)
{
}

// FrameMoveObject
int PythonExtendWin::FrameMove(float fElapsedTime)
{
	m_main_namespace["frameMove"](fElapsedTime); 
	return 1;
}

// FrameSensor::Trigger 
void PythonExtendWin::Triggered(FrameSensor* pTimer)
{
}

bool OIS_event_ctrl();
bool OIS_event_shift();
bool OIS_event_alt();
int PythonExtendWin::handleRendererKeyboardEvent(int ev, int key)
{
	if(!visible() || !OIS_event_shift() ) return 0;
	TString evs="";
	switch(ev)
	{
		case FL_KEYUP: evs="KEYUP"; break;
		case FL_KEYDOWN: evs="KEYDOWN"; break;
	}
	lunaStack l(L);

	{
		TString keys;
		if(key>='a' && key<='z')
			keys.format("%c", key);
		else
			keys.format("%d", key);
		PyObject* callback = object(m_main_namespace["handleRendererEvent"]).ptr();
		double out= call<double> (callback,evs.ptr(), keys.ptr(),0,0);
		return int(out);
	}
}
int PythonExtendWin::handleRendererMouseEvent(int ev, int x, int y, int button)
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

	PyObject* callback = object(m_main_namespace["handleRendererEvent"]).ptr();
	double out=call<double>(callback,evs.ptr(), button, x, y);

	return int(out);
}

// FltkRenderer::Handler
int	PythonExtendWin::handleRendererEvent(int ev) 
{
	// should be similar to ScriptWin::handleRendererEvent
#ifndef NO_GUI
	if(visible()) RE::output("shifted", "%d", Fl::event_state()&FL_SHIFT);

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
			int key;
			key=Fl::event_key();
			return handleRendererKeyboardEvent(ev, key);
		}
		break;
	}

#endif
	return 0;
}

void Register_classificationLib_bind(lua_State*L);
void Register_QP(lua_State*L);
void PythonExtendWin::initLuaEnvironment()
{
	ScriptBaseWin::initLuaEnvironment();
	Register_classificationLib_bind(L);
	Register_QP(L);
}

void PythonExtendWin::dostring(const char* script)
{
	LUAwrapper l(L);
	l.registerErrorFunc();
	l.dostring(script);
}
void PythonExtendWin::dofile(const char* script)
{
	LUAwrapper l(L);
	l.registerErrorFunc();
	l.dofile(script);
}
#include <MainLib/WrapperLua/luna_baselib.h>
#include <MainLib/WrapperLua/luna_mainlib.h>
int PythonExtendWin::work(TString const& workname, lunaStack& L)
{
	if (workname=="pycall")
	{
		int numArg=L.gettop()-2;
		boost::python::list arg;
		while(L.currArg<=L.numArg())  
		{
			//printf("%d %d\n", L.currArg, L.numArg());
			std::string tn=L.lunaType();
			if (tn=="number")
			{
				double res;
				L>>res;
				arg.append(res);
			}
			else if(tn=="string")
			{
				std::string res;
				L>>res;
				arg.append(res);
			}
			else if(tn=="boolean")
			{
				bool res;
				L>>res;
				arg.append(res);
			}
			else if(tn=="vectorn")
			{
				vectorn* v=L.check<vectorn>();
				arg.append(boost::ref(v)); // call by reference
			}
			else if(tn=="vectornView")
			{
				vectorn* v=L.check<vectornView>();
				arg.append(boost::ref(v)); // call by reference
			}
			else if(tn=="vector3")
			{
				vector3* v=L.check<vector3>();
				arg.append(boost::ref(v));// call by reference
			}
			else if(tn=="quater")
			{
				quater* v=L.check<quater>();
				arg.append(boost::ref(v));// call by reference
			}
			else if(tn=="matrixn")
			{
				matrixn* v=L.check<matrixn>();
				arg.append(boost::ref(v));// call by reference
			}
			else if(tn=="matrixnView")
			{
				matrixnView* v=L.check<matrixnView>();
				arg.append(boost::ref(v));// call by reference
			}
			else if(tn=="hypermatrixn")
			{
				hypermatrixn* v=L.check<hypermatrixn>();
				arg.append(boost::ref(v));// call by reference
			}
			else
			{
				Msg::error("PythonExtendWin.cpp: %s not implemented yet", tn.c_str());
				arg.append(-10000);
			}
		}
		//printf("%d\n", L.gettop());
		 //L.printStack();
		// assumes "import luamodule as lua " was already done in python. 
		import("luamodule").attr("pycallFromLua")(arg);
		return 0;
	}
	else if (workname=="run")
	{
		std::string res;
		L>>res;
		PyRun_SimpleString(res.c_str());
		return 0;
	}
	else if (workname=="hasPython")
	{
		L<<true;
		return 1;
	}

	else return ScriptBaseWin::work(workname, L);
}
