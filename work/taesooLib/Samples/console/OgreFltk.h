// OgreFltk.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//
	 

#include "stdafx.h"


enum { WIDTH=644+180, HEIGHT=700, RENDERER_WIDTH=644, RENDERER_HEIGHT=484};// dump video (dump render window only -640*480 )
//enum { WIDTH=640, HEIGHT=480, RENDERER_WIDTH=480, RENDERER_HEIGHT=300};		// dump video (dump main window -640*480)
//enum { WIDTH=1280, HEIGHT=768, RENDERER_WIDTH=1024, RENDERER_HEIGHT=600};	// demo
//enum { RENDERER_WIDTH=2200, RENDERER_HEIGHT=1400, WIDTH=RENDERER_WIDTH+180, HEIGHT=RENDERER_HEIGHT+180};// figure (high-resolution)

class MotionPanel;
class Fl_Tile;
class FlChoiceWins;
#ifndef NO_GUI
#include <OgreFrameListener.h>
#endif
class MainWin : public Fl_Window, public Ogre::FrameListener
{
public:

	MainWin(OgreRenderer * pOgreRenderer, int argc, char* argv[], const char* title=NULL ) ;
	~MainWin();

	FlChoiceWins* createTools(int ww,int hh);
	
#ifndef NO_GUI
	Fl_Tile* m_tile;
#endif
	FltkRenderer* m_Renderer;
	MotionPanel* m_motionPanel;
	FlChoiceWins* m_pParentWin;
};

