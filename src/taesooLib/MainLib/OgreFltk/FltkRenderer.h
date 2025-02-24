#ifndef FLTKRENDERER_H_
#define FLTKRENDERER_H_

#pragma once

#include "FltkAddon.h"
#include <vector>
#include "renderer.h"
#include "framemoveobject.h"
#ifndef NO_GUI
#include <OgreFrameListener.h>
#endif
#include "../../BaseLib/motion/viewpoint.h"
#include "../../BaseLib/motion/intersectionTest.h"
#if !defined NO_GUI 
#include <Fl/Fl_Button.H>
#include <Fl/Fl_Light_Button.H>
#include <Fl/Fl_Check_Button.H>
#include <Fl/Fl_Value_Slider.H>
#if defined _MSC_VER
#include <FL/Fl_Window.H>
#else
#include <FL/Fl_Gl_Window.H>
#endif
#endif

#if defined(NO_GUI) || defined(_MSC_VER)
#define NO_OIS
#endif

/**
 * FltkRenderer는 위쪽에 Renderer화면이 있고 아래쪽에 필요한 버튼이 있는 window이다. Callback 메시지 처리를 위해 FltkCallee도 상속했다.
 *
 * \author TAESOO
 *
 * \todo 
 * 현재 resize, activate, deactivate, hide, show 에 대한 고려가 안되있다. 
 * resize의 경우 Renderer::WndProc(..,WM_SIZE, ..) and Renderer::WndProc(..,WM_EXITSIZEMOVE, ..) 만 call해주면 된다.
 * \bug 
 *
 */

class FastCapture;
namespace Ogre
{
	class PlaneBoundedVolumeListSceneQuery;
	class RaySceneQuery;
}

#ifdef NO_GUI
class FltkRenderer : public Fl_Window, public FlCallee, public FrameMoveObject, public Ogre::FrameListener
{
#else

#ifndef NO_OIS
 #include <OISMouse.h>
 #include <OISKeyboard.h>
 #include <OISJoyStick.h>
 #include <OISInputManager.h>
class FltkRenderer : public OIS::KeyListener, public OIS::MouseListener, public Fl_Window, public FlCallee, public FrameMoveObject, public Ogre::FrameListener
#else
class FltkRenderer : public Fl_Window, public FlCallee, public FrameMoveObject, public Ogre::FrameListener
#endif
{
#endif
public:

	FltkRenderer(int x, int y, int w, int h, OgreRenderer* pOgreRenderer, int renderViewOffsetX=0, int renderViewOffsetY=0);
	virtual ~FltkRenderer();
	virtual int handle(int ev);
	int handle_mouse(int ev, int x, int y, int button);
	virtual void firstInit(Fl_Window* topmostWin);
	virtual void loop(Fl_Window& mainWin);
		
	struct Handler
	{
		virtual int handleRendererEvent(int ev)=0;
		virtual int handleRendererMouseEvent(int ev, int x, int y, int button ){return 0;}
		virtual int handleRendererKeyboardEvent(int ev, int key){return 0;}
	};
	void setHandler(Handler* pHandler=NULL)	{ m_pHandler=pHandler;}

	virtual int FrameMove(float fElapsedTime);

	Viewpoint& view()	{ return *mOgreRenderer->viewport().m_pViewpoint;}
	OgreRenderer& ogreRenderer()	{ return *mOgreRenderer;}

	OgreRenderer* ogreRenderer_test() { return mOgreRenderer;}

	vector3 screenToWorldXZPlane(float x, float y, float height=0.0);
	void screenToWorldLine(float x, float y, vector3& lineStart, vector3& lineEnd);
	void screenToWorldRay(float x, float y, Ray& ray) const;
	void worldToScreen(vector3 const& w, float& x, float& y) const;
	inline vector2 worldToScreen(vector3 const& w) const { float x,y; worldToScreen(w, x, y); return vector2((double)x, (double)y);}


#ifndef NO_GUI
	void volumeQuery(TStrings& nodeNames, float left, float top, float right, float bottom);
	void rayQuery(TStrings& nodeNames, float x, float y);
	Ogre::PlaneBoundedVolumeListSceneQuery* createVolumeQuery(float left, float top, float right, float bottom);
	Ogre::RaySceneQuery* createRayQuery(float x, float y);
	int renderWindowWidth() const ;
	int renderWindowHeight() const;
      virtual	  void resize(int x,int y,int w,int h);
	
#endif
	

	virtual void viewLock()			{ mbViewLock=true;}
	virtual void viewUnlock()		{ mbViewLock=false;}

	virtual void onCallback(Fl_Widget * pWidget, int userData);

	bool frameStarted(const Ogre::FrameEvent& evt){ return true;}
	
	// for window capture
	bool frameEnded(const Ogre::FrameEvent& evt);

	void saveView(int slot);
	void changeView(int slot);
	void changeViewNoAnim(int slot);

	void* m_hWnd;              // The main app window

#ifndef NO_OIS
	bool keyPressed( const OIS::KeyEvent &e );
	bool keyReleased( const OIS::KeyEvent &e );
	bool mouseMoved( const OIS::MouseEvent &e );
	bool mousePressed( const OIS::MouseEvent &e, OIS::MouseButtonID id );
	bool mouseReleased( const OIS::MouseEvent &e, OIS::MouseButtonID id );
#endif

	Handler* m_pHandler;
protected:
	// Fl_Window
	
	friend class OgreRenderer;

	void initCapture(const char* prefix, int motionblur);
	void capture();
	void endCapture();


	FastCapture *mFC;

	int cursorPosX, cursorPosY;

	bool mbSkybox;
	bool mbViewLock;
	int m_eShadowTechnique;

	Viewpoint mSavedView[5];
	m_real mfViewAnim;
	Viewpoint mViewAnim[2];

	// m_RenderView (Fl_Window) 는 group object이므로 맨 마지막에 생성되어야 한다. 안그러면 버튼이 이 그룹안으로 들어간다.
	Fl_Window * m_RenderView;

	OgreRenderer* mOgreRenderer;
};

class FltkToolkitRenderer: public FltkRenderer
{
public:
	FltkToolkitRenderer(int x, int y, int w, int h, OgreRenderer* pOgreRenderer);
	/// 아래 widget들의 순서가 중요하다. (생성되는 순서)
	Fl_Light_Button m_bDumpScreen;
	Fl_Check_Button m_bLockViewpoint;
	Fl_Check_Button m_bPause;
	Fl_Button m_bOneStep;	
	Fl_Value_Slider m_sliderSpeed;
	FlMenu m_menuOp;

	virtual void viewLock()			{ mbViewLock=true; m_bLockViewpoint.value(1); redraw();}
	virtual void viewUnlock()		{ mbViewLock=false; m_bLockViewpoint.value(0);redraw();}

	void onCallback(Fl_Widget * pWidget, int userData);
};


#endif
