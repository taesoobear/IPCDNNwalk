//
  
#include "stdafx.h"
#include <stdio.h>
#ifdef USE_MPI
#include <mpi.h>
#endif
#include "OgreFltk.h"

#undef RENDERER_WIDTH
#undef RENDERER_HEIGHT
#undef HEIGHT
#undef WIDTH
#ifdef USE_LUABIND
#include <luabind/error.hpp>
#endif
#include "../../BaseLib/motion/Motion.h"
//#ifndef NO_GSL
//#include "../../MainLib/gsl_addon/gsl_wrap.h"
//#endif
#include <math.h>

#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "../../BaseLib/utility/operatorString.h"
#include "../../MainLib/OgreFltk/fastCapture.h"
#include "../../MainLib/OgreFltk/GlobalUI.h"

/*
#include "../../MainLib/WrapperLua/BaselibLUA.h"
#include "../../MainLib/WrapperLua/MainlibLUA.h"
*/

#ifndef NO_GUI
#include "../../MainLib/OgreFltk/Joystick.h"

#include <Fl/Fl_Tile.h>
#include <Fl/x.h>
#include <Fl/Fl_ask.h>

#endif
#ifndef NO_OGRE
#include <OgreRoot.h>
#endif

//#include "../dependencies/vld-19/include/vld.h"
//#include "dependency_support/octave_wrap.h"

extern ConfigTable config;
//FlLayout* createSegmentationWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
FlLayout* createRigidBodyWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	
MotionPanel* g_motionPanel=NULL;

MainWin::~MainWin()
{
	// parentWin should be removed before the motionpanel and fltkrender are removed.
#ifndef NO_GUI
	m_pTile->remove(m_pParentWin);
#endif
	delete m_pParentWin;
	m_pParentWin=NULL;
}

MainWin::MainWin(int w, int h, int rw, int rh, OgreRenderer * pOgreRenderer, const char* title)
	: Fl_Window(w, h, title)
{
#ifndef NO_GUI
	m_pTile=new Fl_Tile (0,0,WIDTH,HEIGHT);
#endif
	int RENDERER_WIDTH=rw;
	int RENDERER_HEIGHT=rh;
	int WIDTH=w;
	int HEIGHT=h;
	m_motionPanel=new MotionPanel(0,RENDERER_HEIGHT,WIDTH, HEIGHT-RENDERER_HEIGHT);

	if(RENDERER==1)
	{
		m_Renderer=new FltkRenderer(0,0,RENDERER_WIDTH,RENDERER_HEIGHT,pOgreRenderer);
		m_Renderer->end();
	}
	else
		m_Renderer=new FltkToolkitRenderer(0,0,RENDERER_WIDTH,RENDERER_HEIGHT,pOgreRenderer);

	int ww, hh;

	ww=WIDTH-RENDERER_WIDTH;
	hh=RENDERER_HEIGHT;
	m_Renderer->ogreRenderer().setResourceFile("../Resource/resources_small.cfg");
	const int numWin=1;

	m_pParentWin=new FlChoiceWins(RENDERER_WIDTH, 0, ww, hh, numWin);
	int curWin=0;

	m_pParentWin->window(curWin++, "Rigid body", createRigidBodyWin(0,0,ww,hh, *m_motionPanel, *m_Renderer));
	//m_pParentWin->window(curWin++, "TestPmQmWin",createTestPmQmWin(0,0, ww, hh, *m_motionPanel, *m_Renderer));
	Msg::verify(curWin==numWin, "NumWindow Error");
#ifndef NO_GUI

	m_pTile->end();

	end();

	resizable(this);
#endif
	g_motionPanel=m_motionPanel;// deprecated

}

static FastCapture* mFastCapture=NULL;

bool MainWin::frameStarted(const Ogre::FrameEvent& evt)
{
	return true;
}

// for window capture
bool MainWin::frameEnded(const Ogre::FrameEvent& evt)
{
#ifndef NO_GUI
  if(mFastCapture)
	{
		int width=w();
		int height=h();
		make_current();

		//BEGIN_TIMER(screenshot);
		//for(int i=0; i<10; i++)
		mFastCapture->setCursorPos(RE::getGlobalMouseX(), RE::getGlobalMouseY());
		mFastCapture->Screenshot(fl_gc, width, height);
		//f->Screenshot(width, height);
		//END_TIMER(screenshot);

//		cout << "screenshot" <<endl;
	}
#endif
	return true;
}

int MainWin::handle(int ev)
{
#ifndef NO_GUI
	if(ev==FL_MOVE || ev==FL_DRAG)
	{
	  RE::setGlobalMousePos(Fl::event_x() ,Fl::event_y());
	}
	else if(ev==FL_KEYBOARD)
	{
		if(Fl::event_ctrl() && (Fl::event_state()&FL_SHIFT) && Fl::event_key()=='c' )
		{
			if(!mFastCapture)
			{
				mFastCapture=new FastCapture("../dump/mainwin.dat");
				printf("Mainwin capture started\n");
				RE::renderer().mbScreenshot=true;
			}
			else
			{
				delete mFastCapture;
				mFastCapture=NULL;
				RE::renderer().mbScreenshot=false;
			}
			return 1;
		}
	}

	return Fl_Window::handle(ev);
#else 
  return 1;
 
#endif


}

#ifndef NO_GUI
void MainWin::resize(int x, int y, int w, int h)
{
	Fl_Window::resize(x,y,w,h);
}
#else
void MainWin::resize(int x, int y, int w, int h)
{
	Fl_Window::resize(x,y,w,h);
}
#endif

#ifndef NO_GUI
int functionTest();
void ogreTest();
#endif
#ifndef WIN32
#include <unistd.h>
#endif

//#ifdef _MSC_VER
//int _tmain(int argc, _TCHAR* argv[])
//#else
int main(int argc, char ** argv)
//#endif
{
	if(argc>1){
		TString arg1=argv[1];
		if(arg1.left(5)=="-cwd:"){
#ifndef WIN32
			chdir(arg1.right(-5));
			char buf[1000];
			getcwd(buf,1000);
			printf("cwd:%s\n", buf);
#endif
			argc--;
			argv=&argv[1];
		}
	}
#ifdef USE_MPI
  int rank, size, tag, rc, i;
  MPI_Status status;
  char message[20];

  rc = MPI_Init(&argc, &argv);
  rc = MPI_Comm_size(MPI_COMM_WORLD, &size);
  rc = MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  tag = 100;
#else
  int rank=0;
#endif
	  
  if(rank == 0) {
#ifdef USE_MPI
    strcpy(message, "Hello, world");
    for (i=1; i<size; i++) 
      rc = MPI_Send(message, 13, MPI_CHAR, i, tag, MPI_COMM_WORLD);
#endif
  } 
  else 
#ifdef USE_MPI
    rc = MPI_Recv(message, 13, MPI_CHAR, 0, tag, MPI_COMM_WORLD, &status);

#endif

#ifndef NO_GSL
	gsl::initGlobal();
#endif

	//createOctaveWrap();
#ifndef NO_GUI

	if (functionTest())
		return 1;

	srand((unsigned)time( NULL ));
#endif

	try {
#ifndef NO_GUI

		if (!Fl::visual(FL_DOUBLE|FL_INDEX))
			printf("Xdbe not supported, faking double buffer with pixmaps.\n");
		Fl::scheme("plastic");
#endif

		OgreRenderer renderer;	// renderer has to be destroyed later than all fltk windows.
		{
			int RENDERER_WIDTH=config.GetInt("renderer_width");
			int RENDERER_HEIGHT=config.GetInt("renderer_height");
			int WIDTH=RENDERER_WIDTH+180;
			int HEIGHT=RENDERER_WIDTH+220;
			MainWin win(WIDTH, HEIGHT, RENDERER_WIDTH, RENDERER_HEIGHT, &renderer, "KickBoxer");

			win.show();

			win.m_Renderer->firstInit(&win);
#ifndef NO_GUI

			initJoystick(HWND (win.m_Renderer->m_hWnd));
			renderer.mRoot->addFrameListener(&win);
#endif
			try
			{
#ifndef NO_OGRE
				ogreTest();

				{
					LUAwrapper L;
					addMainlibModule(L.L);

					L.dofile("../Resource/scripts/loadBG_default.lua");
				}
#endif
				GlobalUI ui(win.m_pParentWin, argc, argv);
				win.m_Renderer->loop(win);
			}
#ifdef USE_LUABIND
			catch(luabind::error& e)
			{
				printf("lua error %s\n", e.what());
				int n=lua_gettop(e.state());
				Msg::msgBox("lua error %s", lua_tostring(e.state(), n));
				ASSERT(0);
			}
#endif
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

			
#ifndef NO_GUI
			deinitJoystick();
#endif
		}
	}
#ifndef NO_OGRE
	catch( Ogre::Exception& e )
	{
		Msg::msgBox(e.getFullDescription().c_str());

	}
#endif
	catch(std::exception& e)
	{
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
		Msg::msgBox("some error");
		ASSERT(0);
	}

#ifndef NO_GSL
	gsl::deinitGlobal();
#endif
	TString a("alkjsdlkj");
	a.format("asdfasdf%s", "asdf");
//	_CrtDumpMemoryLeaks();
#ifdef USE_MPI
	  printf( "node %d : %.13s\n", rank,message);
  rc = MPI_Finalize();
#endif
  return 0;

}


