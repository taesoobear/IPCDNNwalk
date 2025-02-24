//
#include "stdafx.h"
#ifdef NO_GUI
#include <MainLib/console/dummies.h>
#endif
#include <MainLib/MainLib.h>
#include <MainLib/OgreFltk/FltkAddon.h>
#include <MainLib/OgreFltk/FlLayout.h>
#include "PythonExtendWin.h"
#include <Python.h>
#include "MainlibPython.h"
#include <MainLib/OgreFltk/MotionPanel.h>
#include <MainLib/OgreFltk/FltkRenderer.h>
#include <MainLib/OgreFltk/MovableText.h>
#include <MainLib/OgreFltk/Loader.h>
//#include <MainLib/OgreFltk/Joystick.h"
#include <MainLib/OgreFltk/MotionPanel.h>
#include <BaseLib/utility/checkPoints.h>
#include <BaseLib/motion/MotionRetarget.h>
//enum { WIDTH=1275-(1000-640), HEIGHT=700, RENDERER_WIDTH=640, RENDERER_HEIGHT=260, RENDERER=0};// dump video2 (half view)
//enum { WIDTH=1275-(1000-640), HEIGHT=700, RENDERER_WIDTH=640, RENDERER_HEIGHT=300, RENDERER=0};// classification 1024*757
//enum { WIDTH=1024, HEIGHT=768, RENDERER_WIDTH=800, RENDERER_HEIGHT=600, RENDERER=0};	// demo
//enum { WIDTH=632, HEIGHT=453, RENDERER_WIDTH=500, RENDERER_HEIGHT=275, RENDERER=1};// classification demo
//enum { WIDTH=1275-(1000-640), HEIGHT=1024, RENDERER_WIDTH=640, RENDERER_HEIGHT=500, RENDERER=0};// classification

namespace RE
{
	extern Globals* g_pGlobals;
}
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
class MainWin : public Fl_Window
{
public:
	MainWin(int w, int h, int rw, int rh, OgreRenderer * pOgreRenderer, const char* title=NULL) 
	: Fl_Window(w, h, title)		
	{

#ifndef NO_GUI
		m_tile=new Fl_Tile (0,0,w,h);
#endif
		int RENDERER_WIDTH=rw;
		int RENDERER_HEIGHT=rh;
		int WIDTH=w;
		int HEIGHT=h;
		
		m_motionPanel=new MotionPanel(0,RENDERER_HEIGHT,WIDTH, HEIGHT-RENDERER_HEIGHT);

		int ww, hh;

		ww=WIDTH-RENDERER_WIDTH;
		hh=RENDERER_HEIGHT;

#if 1
		m_Renderer=new FltkRenderer(ww, 0, RENDERER_WIDTH, hh,pOgreRenderer);
		m_Renderer->end();
		m_pRightWin=new PythonExtendWin(0,0,ww,RENDERER_HEIGHT, *m_motionPanel, *m_Renderer);
#else
		m_Renderer=new FltkRenderer(0,0,RENDERER_WIDTH,RENDERER_HEIGHT,pOgreRenderer);
		m_Renderer->end();
		m_pRightWin=new PythonExtendWin(RENDERER_WIDTH, 0, ww, hh, *m_motionPanel, *m_Renderer);
#endif

#ifndef NO_GUI
		m_tile->end();
#endif
		end();		

#ifndef NO_GUI
		resizable(this);
#endif
	}
	virtual ~MainWin()
	{		
	}

#ifndef NO_GUI
	Fl_Tile* m_tile;
#endif
	FltkRenderer* m_Renderer;
	PythonExtendWin* m_pRightWin;
	MotionPanel* m_motionPanel;
};


static OgreRenderer* g_pRenderer=NULL;
static MainWin* g_pMainWin=NULL;
void _createMainWin(int w, int h, int rw, int rh, float UIscaleFactor, OgreRenderer* _renderer);
void createMainWin(int w, int h, int rw, int rh, float UIscaleFactor)
{
    #ifndef NO_GUI 
        if(g_pRenderer) { printf("mainwin already created\n"); return;} 
    #endif
	FlLayout::setUIscaleFactor(UIscaleFactor);
	srand((unsigned)time( NULL ));

	auto* renderer=RE::_createRenderer(w, rw); // w, rw can be changed by the renderer.
	_createMainWin(w, h, rw, rh, UIscaleFactor, renderer);
}

void createMainWin(int w, int h, int rw, int rh, float UIscaleFactor, const char* configFileName, const char* plugins_file, const char* ogre_config)
{
    #ifndef NO_GUI 
	    if(g_pRenderer) { printf("mainwin already created\n"); return;}
    #endif
	FlLayout::setUIscaleFactor(UIscaleFactor);
	srand((unsigned)time( NULL ));
	_createMainWin(w,h, rw, rh, UIscaleFactor, new OgreRenderer(configFileName, configFileName, plugins_file, ogre_config));
}

void releaseMainWin()
{
	delete g_pMainWin;
	g_pMainWin=NULL;
	delete g_pRenderer;
	g_pRenderer=NULL;
}
void _createInvisibleMainWin()
{
    #ifndef NO_GUI 
        if(g_pRenderer) { printf("mainwin already created\n"); return;} 
    #endif
	FlLayout::setUIscaleFactor(1.5);
	srand((unsigned)time( NULL ));

	RE::g_pGlobals=new RE::Globals();
	_createMainWin(10, 10, 10, 10, 1.5, NULL);
}
void _createMainWin(int w, int h, int rw, int rh, float UIscaleFactor, OgreRenderer* _renderer)
{

	try {

		//netlab::gptest();
		//netlab::gptest2();

#ifndef NO_GUI
		if (!Fl::visual(FL_DOUBLE|FL_INDEX))
			printf("Xdbe not supported, faking double buffer with pixmaps.\n"); 
		Fl::scheme("plastic");
#endif
		g_pRenderer=_renderer;

		OgreRenderer& renderer=*g_pRenderer;
		{
			g_pMainWin=new MainWin(w,h,rw,rh,&renderer, "KickBoxer");
			MainWin& win=*g_pMainWin;

		}
	}
#ifndef NO_OGRE
	catch( Ogre::Exception& e ) 
	{
		std::cout<<e.getFullDescription().c_str()<<std::endl;

		Msg::msgBox(e.getFullDescription().c_str());

	}	
#endif
	catch(std::exception& e)
	{
		std::cout << e.what() <<std::endl;
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(char * error)
	{
		Msg::msgBox("%s", error);
	}
	catch(const char* error)
	{
		Msg::msgBox("%s", error);
	}
	catch(...)
	{
		PyErr_Print();
		Msg::msgBox("some error");
		ASSERT(0);
	}
}

void showMainWin()
{
	MainWin& win=*g_pMainWin;
	win.show();

	win.m_Renderer->firstInit(&win);

	{
        #ifndef NO_GUI
		LUAwrapper L;
		Register_baselib(L.L);
		Register_mainlib(L.L);

		L.dofile(RE::taesooLibPath()+"Resource/scripts/loadBG_default.lua");
        #endif
	}
}

void startMainLoop()
{
	try
	{
		MainWin& win=*g_pMainWin;
		win.m_Renderer->loop(win);	
	}
#ifndef NO_OGRE
	catch( Ogre::Exception& e ) 
	{
		Msg::msgBox(e.getFullDescription().c_str());

	}	
#endif
	catch(char * error)
	{
		Msg::msgBox("%s", error);
	}
	catch(const char* error)
	{
		Msg::msgBox("%s", error);
	}
	catch(std::runtime_error& e)
	{
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(...)
	{
		Msg::msgBox("some error (unknown type)");
		ASSERT(0);
	}
}

bool hasPythonWin()
{
	if(!g_pMainWin) return false;
	if(!g_pMainWin->m_pRightWin) return false;
	return true;
}
PythonExtendWin* getPythonWin()
{
	if(!g_pMainWin) throw std::runtime_error("mainwin ==NULL");
	if(!g_pMainWin->m_pRightWin) throw std::runtime_error("mainwin.rightwin ==NULL");
	return g_pMainWin->m_pRightWin;
}
