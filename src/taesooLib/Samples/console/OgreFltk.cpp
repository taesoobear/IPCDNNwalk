//
	 
#include "stdafx.h"
#include "../../MainLib/MainLib.h"

#include "../../MainLib/OgreFltk/FltkAddon.h"
#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/OgreFltk/FlChoice.h"


#include <math.h>
#include "../../BaseLib/motion/Motion.h"
#include "../../BaseLib/utility/operatorString.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "../../MainLib/OgreFltk/fastCapture.h"
#include "../../MainLib/OgreFltk/MotionManager.h"
#include "../../MainLib/OgreFltk/FltkRenderer.h"
#include "../../MainLib/OgreFltk/Line3D.h"
//#include "RigidBodyWin.h"

#ifndef NO_GUI
#include <Ogre.h>
#include <Fl/Fl_Tile.H>
#endif

#include "OgreFltk.h"

#ifdef USE_MPI
#include <mpi.h>
#endif
MotionPanel* g_motionPanel=NULL;
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);


FlLayout* createRigidBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[]);
FlLayout* createRigidBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[], const char* a, const char* b);
void RigidBodyWin_firstInit(FlLayout* l);

FlLayout* g_testWin;
MainWin::MainWin(OgreRenderer * pOgreRenderer, int argc, char* argv[], const char* title) 
	: Fl_Window(WIDTH, HEIGHT, title)		
{
#ifndef NO_GUI
	m_tile=new Fl_Tile (0,0,WIDTH,HEIGHT);
#endif
	
	m_motionPanel=new MotionPanel(0,RENDERER_HEIGHT,WIDTH, HEIGHT-RENDERER_HEIGHT);
	m_Renderer=new FltkRenderer(0,0,RENDERER_WIDTH,RENDERER_HEIGHT,pOgreRenderer);
	m_Renderer->end();
	
	int ww, hh;

	ww=WIDTH-RENDERER_WIDTH;
	hh=RENDERER_HEIGHT;

	int numWin=1;
	m_pParentWin=new FlChoiceWins(RENDERER_WIDTH, 0, ww, hh, numWin);
	{
	//	g_testWin=createRigidBodyWin(RENDERER_WIDTH, 0, ww, hh, *m_motionPanel, *m_Renderer );
		int curWin=0;
		g_testWin=createRigidBodyWin(0, 0, ww, hh, *m_motionPanel, *m_Renderer , argc, argv, argv[1], argv[2]);
		m_pParentWin->window(curWin++, "lua scripts", g_testWin);
		Msg::verify(curWin==numWin, "NumWindow Error");
	}

#ifndef NO_GUI
	m_tile->end();
#endif
	end();		

#ifndef NO_GUI
	resizable(this);
#endif
	g_motionPanel=m_motionPanel;
}

MainWin::~MainWin()
{
	// manually removed g_testWin so that its destructor is called before those of other sub-windows.
	delete g_testWin;
	g_testWin=NULL;
}

int main(int argc, char* argv[])
{
#ifndef _MSC_VER
	//srand((unsigned)time( NULL ));
#endif
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
	{
#ifdef USE_MPI
		rc = MPI_Recv(message, 13, MPI_CHAR, 0, tag, MPI_COMM_WORLD, &status);
#endif
	}
	
	try {


		OgreRenderer renderer;

		{
			MainWin win(&renderer, argc, argv, "KickBoxer");

			win.show();

			win.m_Renderer->firstInit(&win);

			try
			{
				RigidBodyWin_firstInit(g_testWin);

				win.m_Renderer->loop(win);	
			}
			catch (char* error)
			{
				Msg::msgBox("%s", error);
				assert(false);
			}
			catch(std::exception& e)
			{
				Msg::msgBox("c++ error : %s", e.what());
				ASSERT(0);
			}
			catch(...)
			{
				Msg::msgBox("some error");
				ASSERT(0);
			}

		}
	} 
	catch(char * error)
	{
		Msg::msgBox("%s", error);
	}
	catch(const char* error)
	{
		Msg::msgBox("%s", error);
	}
	catch(std::exception& e)
	{
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(...)
	{
		Msg::msgBox("some error");
		ASSERT(0);
	}

#ifdef USE_MPI
	printf( "node %d : %.13s\n", rank,message);
	rc = MPI_Finalize();
#endif
	return 0;
}

