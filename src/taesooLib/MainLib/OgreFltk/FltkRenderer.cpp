#include "stdafx.h"

#include "OgreFltk/timesensor.h"
#include "FltkRenderer.h"
#include "OgreFltk/AlzzaPostureIP.h"
#include "../BaseLib/math/Operator.h"
#include "TraceManager.h"
#include "FlLayout.h"
#include "FlChoice.h"
#ifndef WIN32 
#include <unistd.h>
#endif
#ifdef NO_GUI
#include "../MainLib/console/traceManager.h"
#include "../../BaseLib/motion/viewpoint.h"

#else
#include <FL/Fl_Color_Chooser.H>
#include <FL/fl_ask.H>
#include <FL/x.H>

void FltkRenderer_toggleCursor();
namespace Msg
{
	/// 사용법: Msg::print(...);
	class FltkMsg : public Base
	{
	public:
		FltkMsg (){}
		virtual ~FltkMsg (){}
		// inherit and implement followings
		virtual void print(const char* msg)	{ printf("%s",msg);}
		virtual void flush()				{ fflush(stdout);}
		virtual void error(const char* msg) { fl_message("Error! %s",msg);ASSERT(0); throw(std::runtime_error(msg));}
		virtual void msgBox(const char* msg){ fl_message("%s",msg);}
		// not implemented yet.
		virtual bool confirm(const char* msg) { return fl_ask("%s", msg);}
		virtual void output(const char* key, const char* msg) { RE::output(key, "%s",msg);}
	};
}
Msg::FltkMsg g_cFltkMsgUtil;

#endif

#ifndef NO_OGRE
#include <Ogre.h>
#if OGRE_VERSION_MINOR>=9|| OGRE_VERSION_MAJOR>=13

//http://www.ogre3d.org/forums/viewtopic.php?f=2&t=79694
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayContainer.h>
#if OGRE_VERSION_MINOR>=12|| OGRE_VERSION_MAJOR>=13

#include <Overlay/OgreOverlay.h>
#endif
#endif // OGRE_VERSION_MINOR==9
#endif // NO_OGRE
#include "../BaseLib/utility/operatorString.h"
#include "fastCapture.h"
#include "../BaseLib/utility/QPerformanceTimer.h"
#include <stdexcept>


static bool softKill=false;



namespace RE
{
	extern Globals* g_pGlobals;
}

//#ifdef __APPLE__
#ifndef NO_GUI
static bool shift=false;
static bool __control=false;
static bool __alt=false;
bool useSeperateOgreWindow();
bool OIS_event_ctrl()
{
#ifdef NO_OIS
	return Fl::event_ctrl() || __control;
#else
	return Fl::event_ctrl() || __control || (RE::renderer().mKeyboard&&RE::renderer().mKeyboard->isModifierDown(OIS::Keyboard::Ctrl));
#endif
}
bool OIS_event_shift()
{
#ifdef NO_OIS
	return (Fl::event_state() & FL_SHIFT || shift);
#else
	return (Fl::event_state()&FL_SHIFT || shift ||(RE::renderer().mKeyboard&&RE::renderer().mKeyboard->isModifierDown(OIS::Keyboard::Shift)));
#endif
}
bool OIS_event_alt()
{
#ifdef __APPLE__
	if(useSeperateOgreWindow())
		return Fl::event_alt() || __alt ||(RE::renderer().mKeyboard&&RE::renderer().mKeyboard->isModifierDown(OIS::Keyboard::Alt));
	return Fl::event_command() || __alt ||(RE::renderer().mKeyboard&&RE::renderer().mKeyboard->isModifierDown(OIS::Keyboard::Alt));
#else
#ifdef NO_OIS
	return Fl::event_alt() || __alt;
#else
	return Fl::event_alt() || __alt ||(RE::renderer().mKeyboard&&RE::renderer().mKeyboard->isModifierDown(OIS::Keyboard::Alt));
#endif
#endif
}

int queryMouseX();
int queryMouseY();
int queryFrameWidth();
int queryFrameHeight();


#if !defined(__APPLE__) && !defined(_MSC_VER)
#include <X11/Xutil.h>
void mouseXY(int& win_x, int& win_y)
{
	Window window_returned;
	int root_x, root_y;
	int screen_count;
	unsigned int mask_return;
	int res;
	screen_count=XScreenCount(fl_display);


				size_t hWnd = 0;
	RE::renderer().  mWnd->getCustomAttribute("WINDOW", &hWnd);
	for(int i=0; i<screen_count; i++)
	{
		//res=XQueryPointer(fl_display, XRootWindow(fl_display, i), &window_returned, &window_returned, &root_x, &root_y, &win_x, &win_y,&mask_return);
		res=XQueryPointer(fl_display, hWnd, &window_returned, &window_returned, &root_x, &root_y, &win_x, &win_y,&mask_return);
		win_y+=55; // hardcoded topBar height.. 
		//printf("%d %d %d %d %d %d\n", i, mMouse->getMouseState().X.abs, mMouse->getMouseState().Y.abs, win_x, win_y, res);
		if (res) break;
	}
}
#endif

#ifndef NO_OIS
static int mouseX(const OIS::MouseEvent& e)
{
#ifdef __APPLE__
	int retina_sf=RE::FltkRenderer().renderWindowWidth()/queryFrameWidth();
	return queryMouseX()*retina_sf; 
	//return e.state.X.abs;// OIS doesn't work well
#else

#ifndef _MSC_VER
	// OIS doesn't work well in the windowed mode
	//printf("x: %d\n", e.state.X.abs);
	int win_x, win_y;
	mouseXY(win_x,win_y);
	return win_x;
#else
	return e.state.X.abs;
#endif
#endif
}

#include "../../BaseLib/math/Operator.h"
static int mouseY(const OIS::MouseEvent& e)
{
#ifdef __APPLE__
	int retina_sf=RE::FltkRenderer().renderWindowWidth()/queryFrameWidth();
	int qfh=queryFrameHeight();
	int titleHeight=qfh*retina_sf-RE::FltkRenderer().renderWindowHeight();

	double t=sop::map(queryMouseY(), 0, qfh-titleHeight/retina_sf, RE::FltkRenderer().renderWindowHeight(),
				0);
	return ROUND(t);
	//return (e.state.Y.abs) // OIS doesn't work well
#else
#ifndef _MSC_VER
	int win_x, win_y;
	mouseXY(win_x,win_y);
	return win_y;
	//printf("y: %d\n", e.state.Y.abs);
#else
	return e.state.Y.abs;
#endif
#endif
}
#endif
#else
bool OIS_event_ctrl() { return false;}
bool OIS_event_shift() { return false;}
bool OIS_event_alt() { return false;}
#endif

#ifdef USE_FL_GL_WINDOW
#define FltkRenderer_Super Fl_Gl_Window
#else
#define FltkRenderer_Super Fl_Window
#endif

FltkRenderer::FltkRenderer(int x, int y, int w, int h, OgreRenderer* pOgreRenderer, int renderViewOffsetX, int renderViewOffsetY)
#ifndef NO_GUI
: FltkRenderer_Super(x,y,w,h)
#endif
{
	mFC=NULL;
#ifndef NO_GUI
#ifdef _MSC_VER
	Fl_Box* b=new Fl_Box(0,0, w+renderViewOffsetX, h+renderViewOffsetY);
	b->box(FL_DOWN_FRAME);

	m_RenderView=new Fl_Window(2,2,w+renderViewOffsetX-4,h+renderViewOffsetY-4);

	mbViewLock=false;
	m_RenderView->end();
	resizable(m_RenderView);
#else
	m_RenderView=NULL;
#if defined(USE_FL_GL_WINDOW)
	mode(FL_RGB| FL_DEPTH| FL_STENCIL);//|FL_SINGLE);
#endif
	resizable(this);
#endif
	Msg::g_pMsgUtil=&g_cFltkMsgUtil;

#endif
	mfViewAnim=FLT_MAX;
	//end();

	m_pHandler =NULL;
	RE::g_pGlobals->pFltkRenderer=this;
	mOgreRenderer =NULL;
	
	mOgreRenderer = pOgreRenderer;
	mbViewLock=false;
}
FltkRenderer::~FltkRenderer()
{
}

vector3 FltkRenderer::screenToWorldXZPlane(float x, float y, float height)
{
	/*
	vector3 lineStart, lineEnd;
	screenToWorldLine(x,y, lineStart, lineEnd);

	float x1 = lineStart.x;
	float x2 = lineEnd.x;
	float y1 = lineStart.y;
	float y2 = lineEnd.y;
	float z1 = lineStart.z;
	float z2 = lineEnd.z;

	float xp,yp,zp;

	if((y1-y2) == 0.0f)
	{
		xp = 0; yp = 0; zp = 0;
	}

	xp = x1 +(x1-x2)/(y1-y2)*(height-y1);
	yp = height;
	zp = z1 +(z1-z2)/(y1-y2)*(height-y1);

	return vector3(xp,yp,zp);
	*/
	Ray ray;
	screenToWorldRay(x, y, ray);
	Plane p(vector3(0,1,0), height);
	std::pair<bool, m_real> out=ray.intersects(p);
	return ray.getPoint(out.second);
}

#ifndef NO_GUI
int FltkRenderer::renderWindowWidth() const 
{
	if(useSeperateOgreWindow())
	{
		unsigned int width, height, depth;
		int top, left;
#if OGRE_VERSION_MAJOR<13
		RE::renderer().mWnd->getMetrics(width, height, depth, left, top);
#else
		RE::renderer().mWnd->getMetrics(width, height, left, top);
#endif
		return width;
	}

	return (m_RenderView)?m_RenderView->w():w();
}
int FltkRenderer::renderWindowHeight() const
{
	if(useSeperateOgreWindow())
	{
		unsigned int width, height, depth;
		int top, left;
#if OGRE_VERSION_MAJOR<13
		RE::renderer().mWnd->getMetrics(width, height, depth, left, top);
#else
		RE::renderer().mWnd->getMetrics(width, height, left, top);
#endif
		return height;
	}
	return (m_RenderView)?m_RenderView->h():h();
}
#endif
void FltkRenderer::screenToWorldRay(float x, float y, Ray& ray) const
{
#ifndef NO_GUI
	float tx = (float)(1.0f / renderWindowWidth()) * x;
	float ty = (float)(1.0f / renderWindowHeight()) * y;

//	RE::output("TxTY", "%f %f %d tx %f ty %f", x, y, renderWindowWidth(), tx, ty);
	Ogre::Ray oray=((FltkRenderer*)this)->ogreRenderer().viewport().mCam->getCameraToViewportRay(tx , ty);
	ray.origin()=ToBase(oray.getOrigin());
	ray.direction()=ToBase(oray.getDirection());
#endif
}

#ifndef NO_GUI

Ogre::RaySceneQuery* FltkRenderer::createRayQuery(float x, float y)
{
	float tx = (float)(1.0f / renderWindowWidth()) * x;
	float ty = (float)(1.0f / renderWindowHeight()) * y;

	Ogre::Ray oray=((FltkRenderer*)this)->ogreRenderer().viewport().mCam->getCameraToViewportRay(tx , ty);

	return RE::ogreSceneManager()->createRayQuery(oray);
}

void FltkRenderer::volumeQuery(TStrings& nodeNames, float left, float top, float right, float bottom)
{
	Ogre::PlaneBoundedVolumeListSceneQuery* q=createVolumeQuery(left, top, right, bottom);
    Ogre::SceneQueryResult result = q->execute();
	Ogre::SceneQueryResultMovableList::iterator itr;
	nodeNames.resize(0);
    for (itr = result.movables.begin(); itr != result.movables.end(); ++itr)
	{
		TString nodeId=(*itr)->getName().c_str();
		nodeNames.pushBack(nodeId);
	}
	delete q;
}
void FltkRenderer::rayQuery(TStrings& nodeNames, float x, float y)
{
	Ogre::RaySceneQuery* q=createRayQuery(x, y);
    Ogre::RaySceneQueryResult result = q->execute();
	Ogre::RaySceneQueryResult::iterator itr;
	nodeNames.resize(0);
    for (itr = result.begin(); itr != result.end(); ++itr)
	{
		if(itr->movable)
		{
			TString nodeId=itr->movable->getName().c_str();
			nodeNames.pushBack(nodeId);
		}
	}
	delete q;
}

Ogre::PlaneBoundedVolumeListSceneQuery* FltkRenderer::createVolumeQuery(float left, float top, float right, float bottom)
{
	if (left > right)
		std::swap(left, right);

	if (top > bottom)
		std::swap(top, bottom);

	float tx = (float)(1.0f / renderWindowWidth()) * left;
	float ty = (float)(1.0f / renderWindowHeight()) * top;
	float tx2 = (float)(1.0f / renderWindowWidth()) * right;
	float ty2 = (float)(1.0f / renderWindowHeight()) * bottom;
	// Convert screen positions into rays
	Ogre::Ray topLeft = RE::renderer().viewport().mCam->getCameraToViewportRay(tx, ty);
	Ogre::Ray topRight = RE::renderer().viewport().mCam->getCameraToViewportRay(tx2, ty);
	Ogre::Ray bottomLeft = RE::renderer().viewport().mCam->getCameraToViewportRay(tx, ty2);
	Ogre::Ray bottomRight = RE::renderer().viewport().mCam->getCameraToViewportRay(tx2, ty2);

	// The plane faces the counter clockwise position.
	Ogre::PlaneBoundedVolume vol;
	vol.planes.push_back(Ogre::Plane(topLeft.getPoint(3), topRight.getPoint(3), bottomRight.getPoint(3)));         // front plane
	vol.planes.push_back(Ogre::Plane(topLeft.getOrigin(), topLeft.getPoint(100), topRight.getPoint(100)));         // top plane
	vol.planes.push_back(Ogre::Plane(topLeft.getOrigin(), bottomLeft.getPoint(100), topLeft.getPoint(100)));       // left plane
	vol.planes.push_back(Ogre::Plane(topLeft.getOrigin(), bottomRight.getPoint(100), bottomLeft.getPoint(100)));   // bottom plane
	vol.planes.push_back(Ogre::Plane(topLeft.getOrigin(), topRight.getPoint(100), bottomRight.getPoint(100)));     // right plane

	Ogre::PlaneBoundedVolumeList volList;
	volList.push_back(vol);

	return RE::renderer().viewport().mScene->createPlaneBoundedVolumeQuery(volList);
}
#endif
void FltkRenderer::worldToScreen(vector3 const& w, float& x, float& y) const
{
#ifndef NO_GUI
	OgreRenderer& R=(const_cast<FltkRenderer*>(this))->ogreRenderer();
	Ogre::Vector4 res=R.viewport().mCam->getProjectionMatrix()*R.viewport().mCam->getViewMatrix()*Ogre::Vector4(w.x, w.y, w.z,1.0);

	res.x/=res.w;
	res.y/=res.w;
	res.z/=res.w;

	x=(res.x+1.0)*0.5*renderWindowWidth();
	y=(res.y-1.0)*-0.5*renderWindowHeight();
#endif
}

void FltkRenderer::screenToWorldLine(float x, float y, vector3& lineStart, vector3& lineEnd)
{
#ifndef NO_GUI
	Ogre::Vector3 vecAfterProjection;
	Ogre::Vector3 vecBeforeViewing1;
	Ogre::Vector3 vecBeforeViewing2;

	Ogre::Matrix4 mtrxTemp;
	Ogre::Vector3 vecTemp;

	vecAfterProjection.x = (-1)*((float)(x - 0)/((float)w())*2.f-1.f);
	vecAfterProjection.y = ((float)(y - 0)/((float)h()-20.0)*2.f-1.f);
	vecAfterProjection.z = 0.0;
	vecAfterProjection*=-1;

	mtrxTemp = ogreRenderer().viewport().mCam->getProjectionMatrix() * ogreRenderer().viewport().mCam->getViewMatrix();
	mtrxTemp = mtrxTemp.inverse();
	vecBeforeViewing1 = mtrxTemp * vecAfterProjection;

	vecAfterProjection.x = (-1)*((float)(x - 0)/((float)w())*2.f-1.f);
	vecAfterProjection.y = ((float)(y - 0)/((float)h()-20.0)*2.f-1.f);
	vecAfterProjection.z = -10.0;
	vecAfterProjection*=-1;

	mtrxTemp = ogreRenderer().viewport().mCam->getProjectionMatrix() * ogreRenderer().viewport().mCam->getViewMatrix();
	mtrxTemp = mtrxTemp.inverse();
	vecBeforeViewing2 = mtrxTemp * vecAfterProjection;

	lineStart=ToBase(vecBeforeViewing1);
	lineEnd=ToBase(vecBeforeViewing2);
#endif
}

#ifndef NO_GUI
static bool resizeNecessary=false; // apple only
static void resizeOgreWin(OgreRenderer* mOgreRenderer, int ww, int hh);
#endif
void FltkRenderer::firstInit(Fl_Window* topmostwin)
{
#ifndef NO_GUI
	//m_RenderView->show();
#ifdef _MSC_VER
	FltkRenderer_Super::show();
	// Create renderer (DirectX object)
	HWND handle;
	handle = (HWND)fl_xid(m_RenderView);

	m_hWnd=(void*)handle;
	mOgreRenderer->firstInit(handle, m_RenderView->w(), m_RenderView->h());
/*#elif defined(__APPLE__)
	make_current();
	mOgreRenderer->firstInit((void*)(Fl_Window*)this, w(), h());
#elif defined(USE_FL_GL_WINDOW)
	make_current();
	mOgreRenderer->firstInit((void*)(Fl_Window*)this, w(), h());
	*/
#else
	while(!topmostwin->visible())
			Fl::wait();
	assert(topmostwin->visible());
	assert(visible());
	Fl::check();
	make_current();
	mOgreRenderer->firstInit((void*)(Fl_Window*)this, w(), h());
#endif 

#else // NO_GUI
  mOgreRenderer->firstInit(NULL,10,10);
#endif
	mbSkybox=true;

	mOgreRenderer->addFrameMoveObject(this);
#ifndef NO_GUI

	// for screen capture
	mOgreRenderer->mRoot->addFrameListener(this);
	// Create a skybox
	//mScene->setSkyBox(true, "Examples/SpaceSkyBox", 50 );

#endif
#ifndef NO_OGRE
	// Skybox.
	//mOgreRenderer->mScene->setSkyBox(mbSkybox, "My/SkyBox2"); ->moved to lua
  	m_eShadowTechnique=(int)mOgreRenderer->viewport().mScene->getShadowTechnique();
#endif
	mSavedView[0]=mSavedView[1]=mSavedView[2]=mSavedView[3]=mSavedView[4]=*mOgreRenderer->viewport().m_pViewpoint;
//#ifdef __APPLE__
#ifndef NO_GUI

//#if defined (__APPLE__) || defined(OCULUS)	
	if(useSeperateOgreWindow())
	{
		// ASSERT(useSeperateOgreWindow()== true);
		Ogre::LogManager::getSingleton().logMessage("GTH_log_1");
#ifndef NO_OIS
		mOgreRenderer->mKeyboard->setEventCallback( this );
		mOgreRenderer->mMouse->setEventCallback( this );
#else
		Ogre::LogManager::getSingleton().logMessage("OIS disabled!!!");
		printf("OIS disabled!!!");
#endif
		Ogre::LogManager::getSingleton().logMessage("GTH_log_2");
	//onCallback(NULL, Hash("TgCs"));
	}
	else
	{
#ifdef __APPLE__
		resizeNecessary=true; // I don't know why this is necessary.
#endif
	}
//#endif
#endif
}


bool FltkRenderer::frameEnded(const Ogre::FrameEvent& evt)
{
	if(mFC)
		capture();
	return true;
}

#if defined _MSC_VER || (!defined NO_GUI && !defined __APPLE__)
const int captureMethod=1;
#else
const int captureMethod=0;
#endif
// 아래 두개 변수는 capture method 가 0또는 2일때만 사용됨.
// nframe: 0-> 생성, 1->진행, -1 ->캡쳐 종료.
static int nframe=-1;
static TString g_prefix;

void FltkRenderer::initCapture(const char* prefix, int motionblur)
{
#ifndef NO_GUI
  switch(captureMethod)
	{
		case 0:
		case 2:
			nframe=0;
			g_prefix=prefix;
			break;
		case 1:
			{
				TString fn(prefix);
				fn+=".dat";

				bool bAskBeforeConverting=(fn=="../dump/dump.dat");

//				if(fn=="../dump/capture.dat")
					mFC=new FastCaptureDirect(fn); // directly saving to JPG sequence is faster now. 
					mFC->setAccumulation(motionblur);
					//mFC=malloc(sizeof(FastCaptureDirect))
					//mFC->mWidth=0;
					//mFC->mHEight=0;
					//...

//				else
//					mFC=new FastCapture(fn, bAskBeforeConverting);

			}
			break;
	}
#endif
}

void FltkRenderer::endCapture()
{
#ifndef NO_GUI
	switch(captureMethod)
	{
	case 0:
	case 2:
		nframe=-1;
		break;
	case 1:
		delete mFC;
		mFC=NULL;		
		break;
	}

#endif // NO_GUI
}


void FltkRenderer::capture()
{
	//printf("FltkRenderer::capture()\n");
#ifndef NO_GUI

	// FLTK provides a single function for reading from the current window or off?screen buffer into a RGB(A)
	// image buffer.
	switch(captureMethod)
	{
	case 0:
		{
			if(nframe==-1) break;
			TString filename(sz0::format0("%s%5d.bmp", g_prefix.ptr(), nframe));
			nframe++;
			int width=m_RenderView->w();
			int height=m_RenderView->h();
			m_RenderView->make_current();

#ifdef _MSC_VER
			// 항상 동작. 나름 빠름. BMP시퀀스로 저장.
			ScreenshotNew(filename, fl_gc, width, height);
#else
			printf("mWnd\n");
			mOgreRenderer->mWnd->writeContentsToFile(filename.ptr());
#endif
			

		}
		break;
	case 1:
		{
			// 32비트 화면 모드에서만 동작.
			mFC->setCursorPos(cursorPosX, cursorPosY);
			//BEGIN_TIMER(screenshot);
			//for(int i=0; i<10; i++)
#ifdef _MSC_VER
			Fl_Window* win=m_RenderView;
#else
			Fl_Window* win=this;
#endif
			int width=win->w();
			int height=win->h();
			mFC->Screenshot(win, width, height);
			//f->Screenshot(width, height);
			//END_TIMER(screenshot);
		}
		break;
	case 2:
		{

#ifdef COMMENT_OUT
			if(nframe==-1) break;
			TString filename(sz0::format0("%s%5d.bmp", g_prefix.ptr(), nframe));
			nframe++;
			static uchar *s_buffer=NULL;
			static int bufferSize=0;
			for(int iii=0; iii<10; iii++)
			{


				m_RenderView->make_current();
				// following function doesn't have definition in DLL.-.-

				int width=m_RenderView->w();
				int height=m_RenderView->h();

				if(bufferSize<width*height*3)
				{
					delete s_buffer;
					s_buffer=new uchar[width*height*3];
					bufferSize=width*height*3;
				}

				fl_read_image(s_buffer, 0, 0, width, height);

				CImage image;
				image.Create(m_RenderView->w(), m_RenderView->h(), 24);

				CColorPixelPtr ip(image);

				uchar * buffer=s_buffer;
				for(int j=0; j<height; j++)
				{
					for(int i=0; i< width; i++)
					{
						ip[j][i].R=*buffer;
						*buffer++;
						ip[j][i].G=*buffer;
						*buffer++;
						ip[j][i].B=*buffer;
						*buffer++;
					}
				}

				image.Save(filename);
			}
#endif// COMMENT_OUT

		}
		break;
	}
#endif
}

int FltkRenderer::FrameMove(float fElapsedTime)
{
	// 시점 애니메이션(view animation)

	const m_real duration=1.0;
	if(mfViewAnim<duration)
	{
		mfViewAnim+=fElapsedTime;
		m_real t=sop::smoothTransition(sop::clampMap(mfViewAnim, 0, duration));
		mOgreRenderer->viewport().m_pViewpoint->interpolate(t, mViewAnim[0], mViewAnim[1]);
	}

	return 1;
}

void FltkRenderer::loop(Fl_Window& win)
{
	for (;;)
	{
	    if (softKill) break;
#ifndef NO_GUI
		if (win.visible())
		{
			if (!Fl::check())
				break;	// returns immediately
		}
		else
		{
			if (!Fl::wait()) break;	// waits until something happens
		}
		if(resizeNecessary)
		{
			resizeOgreWin(mOgreRenderer, w(), h());
			resizeNecessary=false;
		}
#endif

  		// idle time
		mOgreRenderer->renderOneFrame();

#ifdef _MSC_VER
//		::Sleep(10);
#elif defined(__APPLE__) && !defined(NO_GUI)
		usleep(10);
#endif

	}
}

namespace RE_ {
	// returns false only when something went obviously wrong.
	bool renderOneFrame(bool check)
	{
		if(!RE::rendererValid() ) return false;

#ifndef NO_GUI
		Fl_Window& win=RE::FltkRenderer();
		if(check)
		{
			if (win.visible())
			{
				if (!Fl::check())
					return false;	// returns immediately
			}
			else
			{
				if (!Fl::wait()) return false;	// waits until something happens
			}
			if(resizeNecessary)
			{
				resizeOgreWin(&RE::renderer(), win.w(), win.h());
				resizeNecessary=false;
			}
			if (softKill) return false;
		}
#endif


		RE::renderer().renderOneFrame();
#ifdef _MSC_VER
		//		::Sleep(10);
#elif defined(__APPLE__) && !defined(NO_GUI)
		usleep(10);
#endif
		return true;
	}
}



void FltkRenderer::onCallback(Fl_Widget * pWidget, int userData)
{
    if(userData==Hash("softKill"))
    {
      softKill=true;
    }
	else if(userData==Hash("Save view (slot 1)"))
		saveView(0);
	else if(userData==Hash("Save view (slot 2)"))
		saveView(1);
	else if(userData==Hash("Save view (slot 3)"))
		saveView(2);
	else if(userData==Hash("Save view (slot 4)"))
		saveView(3);
	else if(userData==Hash("Save view (slot 5)"))
		saveView(4);
	else if(userData==Hash("Change view (slot 1)"))
		changeView(0);
	else if(userData==Hash("Change view (slot 2)"))
		changeView(1);
	else if(userData==Hash("Change view (slot 3)"))
		changeView(2);
	else if(userData==Hash("Change view (slot 4)"))
		changeView(3);
	else if(userData==Hash("Change view (slot 5)"))
		changeView(4);
	else
#ifdef COMMENT_OUT
	if(pWidget==&m_bDumpScreen)
	{
		if(m_bDumpScreen.value())
			Renderer::StartDump();
		else
			Renderer::EndDump();
	}
	else if(pWidget==&m_bLockViewpoint)
	{
	}
	else if(pWidget==&m_bPause)
	{
		if(!m_bPause.value())
			m_bFrameMoving=TRUE;
		Renderer::Pause((bool)m_bPause.value());
	}
	else if(pWidget==&m_bOneStep)
	{
		Renderer::Pause(FALSE);
		m_bPause.value(true);
		m_bFrameMoving=FALSE;
		m_bSingleStep=TRUE;
	}
	else if(pWidget==&m_sliderSpeed)
	{
		m_fTimeScaling=m_sliderSpeed.value();
	}
#endif //COMMENT_OUT
	if(userData==Hash("OgreTraceManager"))
	{
#ifdef NO_OGRE
		static TraceManager* otm=NULL;
		if (!otm)
		{
			otm=new TraceManager(); // tracemanager1
			otm=new TraceManager(); // tracemanager2
			printf("TraceManager created\n");
		}
#else
		static OgreTraceManager* otm=NULL;
		if(!otm)
		{
			otm=new OgreTraceManager(0, 0, 640, 240);
#ifdef __APPLE__

			int _w=w()/2;
#ifndef NO_OGRE
			_w=mOgreRenderer ->viewport().m_pViewpoint->m_iWidth/2;
#endif
			otm=new OgreTraceManager(_w, 240*2, _w, 480);
#else
			otm=new OgreTraceManager(w()/2, 240, w()/2, 480);
#endif

			printf("TraceManager created\n");
		}
#endif
		//else 
		//{
		//	delete otm;
		//	otm=NULL;
		//}
	}
#ifndef NO_GUI
	else if(userData==Hash("Set speed"))
	{
		FlLayout o(400, 30, "Set speed for playback, and press x");
		o.create("Value_Slider", "Set speed", "Set speed",1);
		o.slider(0)->range(0.1, 30.0);
		o.slider(0)->step(0.1);
		o.slider(0)->value(1.0);
		o.updateLayout();
		o.show();
		while(o.visible())
			Fl::wait();

		mOgreRenderer->speedControl(o.slider(0)->value());
	}
	else if(userData==Hash("SetCaptureFPS"))
	{
		FlLayout o(400, 10+25+25+5, "Set FPS for screen capture, and press x");

		o.create("Choice", "frame rate","frame rate",1);

		int fps[]={7,15,30,60,120};
		o.menu(0)->size(5);
		o.menu(0)->item(0, "7FPS");
		o.menu(0)->item(1, "15FPS");
		o.menu(0)->item(2, "30FPS");
		o.menu(0)->item(3, "60FPS");
		o.menu(0)->item(4, "120FPS");
		o.updateLayout();

		o.show();
		while(o.visible())
			Fl::wait();

		mOgreRenderer->setCaptureFPS(fps[o.menu(0)->value()]);
	}
	else if(userData==Hash("Toggle background"))
	{
		try{
			Ogre::SceneNode* pBg=mOgreRenderer->viewport().mScene->getSceneNode("BackgroundNode");
			pBg->flipVisibility();
		}
		catch(Ogre::Exception& e ) {
				Msg::msgBox("%s", e.getFullDescription().c_str());
		}
	}
	else if(userData==Hash("Turn-off fog"))
	{
		mOgreRenderer->viewport().mScene->setFog(Ogre::FOG_NONE,Ogre::ColourValue(0,0,0), 0, 0, 0);
	}
	else if(userData==Hash("TgLg"))
	{
		try{
			Ogre::Overlay* overlay=Ogre::OverlayManager::getSingleton().getByName("LogoOverlay");

			if(overlay)
			{
				if(overlay->isVisible())
					overlay->hide();
				else
					overlay->show();
			}
		}
		catch(Ogre::Exception& e ) {
				Msg::msgBox("%s", e.getFullDescription().c_str());
		}
	}
	else if(userData==Hash("TgCs"))
	{
		FltkRenderer_toggleCursor();
	}
	else if(userData==Hash("ChBg"))
	{
		double r=1, g=1, b=1;

		// skybox가 켜있으면 끈다.
		mbSkybox=true;
		onCallback(pWidget, Hash("TgSb"));

		if (fl_color_chooser(0,r,g,b))
		{
			auto c=Ogre::ColourValue(r,g,b,1.f);
			mOgreRenderer->viewport().mView->setBackgroundColour(c);

			bool bOrthographic=mOgreRenderer->viewport().m_pViewpoint->getOrthographicMode();	
			if (bOrthographic)
				RE::ogreSceneManager()->setFog(Ogre::FOG_LINEAR, c,0.0, 2800, 3800 );
			else
				RE::ogreSceneManager()->setFog(Ogre::FOG_LINEAR, c,0.0, 1600, 3500 );
		}
	}
	else if(userData==Hash("TgSb"))
	{
		try{

			mbSkybox=!mbSkybox;
			mOgreRenderer->viewport().mScene->setSkyBox(mbSkybox, "My/SkyBox2");
			// followings doesn't work. So I hardcorded skybox material-.-.
			//Ogre::SceneNode* pBg=mOgreRenderer->mScene->getSceneNode("SkyBoxNode");
			//pBg->flipVisibility();
		}
		catch(Ogre::Exception& e ) {
				Msg::msgBox("%s", e.getFullDescription().c_str());
		}
	}
	else if(userData==Hash("ChSh"))
	{
		m_eShadowTechnique=(m_eShadowTechnique+1)%4;
		mOgreRenderer->viewport().mScene->setShadowTechnique( (Ogre::ShadowTechnique)m_eShadowTechnique);

		switch((Ogre::ShadowTechnique)m_eShadowTechnique)
		{
		case Ogre::SHADOWTYPE_NONE:
			Msg::msgBox("NONE");break;
		case Ogre::SHADOWTYPE_STENCIL_MODULATIVE:
			Msg::msgBox("SHADOWTYPE_STENCIL_MODULATIVE");break;
		case Ogre::SHADOWTYPE_STENCIL_ADDITIVE:
			Msg::msgBox("SHADOWTYPE_STENCIL_ADDITIVE");break;
		case Ogre::SHADOWTYPE_TEXTURE_MODULATIVE:
			Msg::msgBox("SHADOWTYPE_TEXTURE_MODULATIVE");break;
		}
	}
	else if(userData==Hash("Capt"))
	{
		#ifdef _MSC_VER
		Fl_Window* target=m_RenderView;
		//Fl_Window* target=this;

		int width=target->w();
		int height=target->h();
		target->make_current();
		ScreenshotNew("capture.bmp", fl_gc, width, height);
#else
		static FastCaptureDirect* FC=NULL;

		TString fn="../dump/capture.dat";
		if (FC==NULL)
			FC=new FastCaptureDirect(fn.ptr()); // directly saving to JPG sequence is faster now. 

		int cursorPosX=Fl::event_x();
		int cursorPosY=Fl::event_y();
		FC->setCursorPos(cursorPosX, cursorPosY);
		FC->Screenshot(this, w(), h());

		printf("captured one frame\n");
		/*
		 아래 코드 필요없음. Screenshot에 해당기능 내장.
		static int count=1;
		TString filename(sz0::format("%s/%05d.jpg",fn.left(-4).ptr(), 0));
		TString filename2(sz0::format("%s/%05d.jpg",fn.left(-4).ptr(), count++));
		TString cmd;
		cmd.format("mv %s %s", filename.ptr(), filename2.ptr());
		system(sz0::format.ptr());
		*/

		#endif
	}
#endif
}

void drawCursor(int x, int y)
{
	
#ifndef NO_GUI
	Ogre::Overlay* overlay=Ogre::OverlayManager::getSingleton().getByName("CursorOverlay");

	if(overlay)
	{
#if OGRE_VERSION_MINOR<9
		Ogre::OverlayElement* pElt=overlay->getChild("TCursor");
#else
		Ogre::OverlayContainer* pElt=overlay->getChild("TCursor");
#endif
		pElt->setPosition(x-3,y);
	}
#endif
}

void FltkRenderer_toggleCursor()
{
#ifndef NO_GUI
	try{
		Ogre::Overlay* overlay=Ogre::OverlayManager::getSingleton().getByName("CursorOverlay");

		if(overlay)
		{
			if(overlay->isVisible())
				overlay->hide();
			else
			{
				overlay->show();

#if OGRE_VERSION_MINOR<9

				Ogre::OverlayElement* pElt=overlay->getChild("TCursor");
				pElt->setPosition(500,300);
				pElt->show();
#else
				Ogre::OverlayContainer* pElt=overlay->getChild("TCursor");
				pElt->setPosition(100,100);
				pElt->show();
#endif

			}
		}
	}
	catch(Ogre::Exception& e ) {
		Msg::msgBox("%s", e.getFullDescription().c_str());
	}
#endif
}
int FltkRenderer::handle(int ev)
{
#ifndef NO_GUI
	if(ev==FL_MOVE ||ev==FL_DRAG)
	{
		cursorPosX=Fl::event_x();
		cursorPosY=Fl::event_y();

		RE::setGlobalMousePos(x()+cursorPosX, y()+cursorPosY);
		drawCursor(Fl::event_x(), Fl::event_y());
	}
	if(m_pHandler && m_pHandler->handleRendererEvent(ev) ) return 1;

	if(OIS_event_alt())//  && !mbViewLock)
	{
		if(ev== FL_KEYBOARD)
		{
			static int i=0;

			switch (Fl::event_key())
			{
			case FL_Up:
				{
				  //RE::output("key", "zoom %d", i++);
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::MBUTTONDOWN, 10, 10);
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::MBUTTONUP, 10, 20);
				}
				return 1;
			case FL_Down:
				{
				// RE::output("key", "zoom %d", i--);
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::MBUTTONDOWN, 10, 20);
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::MBUTTONUP, 10, 10);
				}
				return 1;
			case FL_KP_Enter:
				{
					mOgreRenderer->toggleScreenShotMode();
				}
				return 1;
			default:
				return 0;
			}
		}
	}
	if(ev==FL_MOUSEWHEEL)
	{
		if(OIS_event_alt())//  && !mbViewLock)
		{
			if(OIS_event_shift())
			{
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::RBUTTONDOWN, 10, 10);
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::RBUTTONUP, 10, 10-Fl::event_dy()*30);
			}
			else
			{
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::RBUTTONDOWN, 10, 10);
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::RBUTTONUP, 10-Fl::event_dy()*30, 10);
			}
		}
		else if(OIS_event_ctrl())
		{
			if(OIS_event_shift() )
			{
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::LBUTTONDOWN, 10, 10);
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::LBUTTONUP, 10, 10-Fl::event_dy()*30);
			}
			else
			{
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::LBUTTONDOWN, 10, 10);
				mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::LBUTTONUP, 10-Fl::event_dy()*30, 10);
			}
		}
		else if(OIS_event_shift())
		{
			mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::MBUTTONDOWN, 10, 10);
			mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  Viewpoint::MBUTTONUP, 10, 10-Fl::event_dy()*15);
		}
		return 1;
	}

	switch(ev)
	{

	case FL_ACTIVATE:
	case FL_SHOW:
	case FL_DEACTIVATE:
	case FL_HIDE:
		break;

	case FL_ENTER:
		take_focus();
		//RE::output("mouse","enter");
		break;

	case FL_LEAVE:
		//RE::output("mouse","leave");
		break;

	case FL_FOCUS:
	case FL_UNFOCUS:
		return 1;
	case FL_KEYUP:
	case FL_KEYDOWN:
		{
			int x=Fl::event_x();
			int y=Fl::event_y();

			if(y<h()-20 && x<w())
			{
				static int count=0;
				//Msg::output("view event","%d %d %d %d", count++, ev, Fl::event_ctrl(),Fl::event_key());
			}
		}
		break;
	case FL_PUSH:
	case FL_DRAG:
	case FL_RELEASE:
		{

			int y=Fl::event_y();
			// Ignore events that are from outside of RenderView.
			if(ev==FL_PUSH && y>h()-20 )
				break;

			int button=Fl::event_button();
			int x=Fl::event_x();
			if(handle_mouse(ev, x,y,button))
				return 1;
		}
		break;
	case FL_MOVE:
		{
			//				printf("move %d %d %d\n",  Fl::event_ctrl(),Fl::event_x(), Fl::event_y());
			take_focus();
		}
		break;
	}


	int ret=FltkRenderer_Super::handle(ev);

	switch(ev)
	{
	case FL_ACTIVATE:
	case FL_SHOW:
#ifdef _MSC_VER
		if(mOgreRenderer->isActive()) { mOgreRenderer->mWnd->windowMovedOrResized();
		}
#endif
		break;
	}

	return ret;
#else
  return true;
#endif
}

void FltkRenderer::saveView(int slot)
{
	mSavedView[slot]=*mOgreRenderer->viewport().m_pViewpoint;
}
void FltkRenderer::changeView(int curView)
{
	mViewAnim[0]=*mOgreRenderer->viewport().m_pViewpoint;
	mViewAnim[1]=mSavedView[curView];
	mfViewAnim=0.0;
}
void FltkRenderer::changeViewNoAnim(int curView)
{
	*mOgreRenderer->viewport().m_pViewpoint=mSavedView[curView];
}

#ifndef NO_GUI

static void resizeOgreWin(OgreRenderer* mOgreRenderer, int ww, int hh)
{
	mOgreRenderer->mWnd->resize(ww,hh);
	mOgreRenderer->mWnd->windowMovedOrResized();


	double aspRatio=(double)ww/(double)hh;
	Ogre::Camera* cam=mOgreRenderer->viewport().mCam;
	cam->setAspectRatio(Ogre::Real(aspRatio));
	if(aspRatio>1.8) // wider than wide display
					 // use vertical FOV
		cam->setFOVy(Ogre::Radian(Ogre::Degree(45)/aspRatio*1.8));
	else
		cam->setFOVy(Ogre::Radian(Ogre::Degree(45)));
}

void FltkRenderer::resize(int x,int y,int w,int h)
{
	FltkRenderer_Super::resize(x,y,w,h);
	if(useSeperateOgreWindow()) return;
	if(mOgreRenderer->isActive() )
	{
#ifdef _MSC_VER
		int ww=m_RenderView->w();
		int hh=m_RenderView->h();
#else
		int ww=w;
		int hh=h;
#endif
		resizeOgreWin(mOgreRenderer, ww, hh);
	}
	else
		resizeNecessary=true;
	//mOgreRenderer->mView->getActualWidth()) / Ogre::Real(mOgreRenderer->mView->getActualHeight()));
}
#endif


FltkToolkitRenderer::FltkToolkitRenderer(int x, int y, int w, int h, OgreRenderer* pOgreRenderer)
: FltkRenderer(x,y,w,h,pOgreRenderer, 0,-20),
m_bDumpScreen(				80*0,h-20,80,20,"Capture"),
m_bOneStep(					80*1,h-20,80,20,"SingleStep"),
m_bLockViewpoint(			80*2,h-20,80,20,"View-lock"),
m_bPause(					80*3,h-20,80,20,"Pause"),
m_sliderSpeed(				80*4,h-20,200,20,"")
{
#ifndef NO_GUI

  m_bDumpScreen.labelsize(11);
	m_bOneStep.labelsize(11);
	m_bLockViewpoint.labelsize(11);
	m_bLockViewpoint.value(0);
	m_bPause.labelsize(11);

	int crx=80*4+200;//+80;
	int cry=h-20;
	m_menuOp.initChoice(crx, cry, 100, 20, "");
	m_menuOp.size(11);
	int c=0;
	m_menuOp.item(c++, "Operations");
	m_menuOp.item(c++, "Toggle background",0, Hash("Toggle background"), FL_CTRL+'g');
	m_menuOp.item(c++, "Toggle skybox", 0, Hash("TgSb"));
	m_menuOp.item(c++, "Turn-off fog", 0, Hash("Turn-off fog"));
	m_menuOp.item(c++, "Change background color", 0, Hash("ChBg"));
	m_menuOp.item(c++, "Change shadow technique", 0, Hash("ChSh"));
	m_menuOp.item(c++, "Toggle logo", 0, Hash("TgLg"));
	m_menuOp.item(c++, "Toggle cursor", 0, Hash("TgCs"));
	m_menuOp.item(c++, "Capture (capture.jpg)", 0, Hash("Capt"));
	m_menuOp.item(c++, "OgreTraceManager", 0, Hash("OgreTraceManager"));
	m_menuOp.item(c++, "SetCaptureFPS", 0, Hash("SetCaptureFPS"));

	m_menuOp.value(0);
	connect(m_menuOp);
	// this (Fl_Window) 그룹이 끝났음

	connect(&m_bDumpScreen);
	connect(&m_bLockViewpoint);
	connect(&m_bPause);
	connect(&m_bOneStep);
	connect(&m_sliderSpeed);

	m_sliderSpeed.type(FL_HOR_SLIDER);
	m_sliderSpeed.range(0.1, 3.0);
	m_sliderSpeed.step(0.1);
	m_sliderSpeed.value(1.0);
	m_sliderSpeed.align(FL_ALIGN_LEFT);

	end();
#endif
}

void FltkToolkitRenderer::onCallback(Fl_Widget * pWidget, int userData)
{
	if(pWidget == &m_bDumpScreen)
	{
		if(m_bDumpScreen.value())
			mOgreRenderer->screenshot(true);
		else mOgreRenderer->screenshot(false);
	}
	else if(pWidget == &m_bPause)
	{
		if(m_bPause.value())
			mOgreRenderer->pause(true);
		else mOgreRenderer->pause(false);
	}
	else if(pWidget == &m_sliderSpeed)
	{
		float val = (float)m_sliderSpeed.value();
		mOgreRenderer->speedControl(val);
	}
	else if(pWidget==&m_bLockViewpoint)
	{
		mbViewLock=m_bLockViewpoint.value();
	}
	else	FltkRenderer::onCallback(pWidget, userData);

}





#ifndef NO_GUI
//#ifdef __APPLE__
#ifndef NO_OIS
static int OISkeycodetoFLTK(int ekey)
{
	int key;
	switch(ekey)
	{
		case OIS::KC_Q: key='q'; break;
		case OIS::KC_W: key='w'; break;
		case OIS::KC_E: key='e'; break;
		case OIS::KC_R: key='r'; break;
		case OIS::KC_T: key='t'; break;
		case OIS::KC_Y: key='y'; break;
		case OIS::KC_U: key='u'; break;
		case OIS::KC_I: key='i'; break;
		case OIS::KC_O: key='o'; break;
		case OIS::KC_P: key='p'; break;
		case OIS::KC_A: key='a'; break;
		case OIS::KC_S: key='s'; break;
		case OIS::KC_D: key='d'; break;
		case OIS::KC_F: key='f'; break;
		case OIS::KC_G: key='g'; break;
		case OIS::KC_H: key='h'; break;
		case OIS::KC_J: key='j'; break;
		case OIS::KC_K: key='k'; break;
		case OIS::KC_L: key='l'; break;
		case OIS::KC_Z: key='z'; break;
		case OIS::KC_X: key='x'; break;
		case OIS::KC_C: key='c'; break;
		case OIS::KC_V: key='v'; break;
		case OIS::KC_B: key='b'; break;
		case OIS::KC_N: key='n'; break;
		case OIS::KC_M: key='m'; break;
	};
	return key;
}
bool FltkRenderer::keyPressed( const OIS::KeyEvent &e ) {
	if(m_pHandler && m_pHandler->handleRendererKeyboardEvent(FL_KEYDOWN, OISkeycodetoFLTK(e.key)))
		return true;
	if( e.key == OIS::KC_W)
	{
		printf("w pressed\n");
	}
	if( e.key == OIS::KC_A)
	{
		printf("a pressed\n");
		__control=false;
		__alt=false;
	}
	if( e.key == OIS::KC_Z)
	{
		printf("shift pressed\n");
		shift=true;
	}
	if( e.key == OIS::KC_X)
	{
		printf("control pressed\n");
		__control=true;
	}
	else if( e.key == OIS::KC_C)
	{
		printf("alt pressed\n");
		__alt=true;
	}
    return true;
}
 
bool FltkRenderer::keyReleased( const OIS::KeyEvent &e ) {
	if(m_pHandler && m_pHandler->handleRendererKeyboardEvent(FL_KEYUP, OISkeycodetoFLTK(e.key)))
		return true;
	if( e.key == OIS::KC_W)
	{
		printf("w released\n");
	}
	if( e.key == OIS::KC_A)
	{
		printf("a released\n");
	}
	if( e.key == OIS::KC_Q)
	{
		if (OIS_event_ctrl())
			softKill=true;
	}

	if( e.key == OIS::KC_Z)
		shift=false;
	if( e.key == OIS::KC_X)
		__control=false;
	else if( e.key == OIS::KC_C)
		__alt=false;
    return true;
}
 
static bool isDragging=false;
static int lastButton=1;
bool FltkRenderer::mouseMoved( const OIS::MouseEvent &e ) {

	int ev;
	if(isDragging)
		ev=FL_DRAG;
	else
		ev=FL_MOVE;

	drawCursor(mouseX(e), mouseY(e));
	if(m_pHandler && 
			m_pHandler->handleRendererMouseEvent(ev, mouseX(e), mouseY(e), lastButton)
	  )
		return true;
	handle_mouse(ev, mouseX(e), mouseY(e), lastButton);

    return true;
}
 
static int OIS_MouseButtonID_to_int(OIS::MouseButtonID id)
{
	switch(id)
	{
		case OIS::MB_Left:
			lastButton=1;
			break;
		case OIS::MB_Middle:
			lastButton=2;
			break;
		case OIS::MB_Right:
			lastButton=3;
			break;
	}
	if(OIS_event_alt())
		return 3;
	return lastButton;
}

bool FltkRenderer::mousePressed( const OIS::MouseEvent &e, OIS::MouseButtonID id ) {
	isDragging=true;
	if(m_pHandler && 
			m_pHandler->handleRendererMouseEvent(FL_PUSH, mouseX(e), mouseY(e), OIS_MouseButtonID_to_int(id))
	  )
		return true;
	handle_mouse(FL_PUSH, mouseX(e), mouseY(e), OIS_MouseButtonID_to_int(id));
    return true;
}
 
bool FltkRenderer::mouseReleased( const OIS::MouseEvent &e, OIS::MouseButtonID id ) {
	isDragging=false;
	if(m_pHandler && 
			m_pHandler->handleRendererMouseEvent(FL_RELEASE, mouseX(e), mouseY(e), OIS_MouseButtonID_to_int(id))
	  )
		return true;
	handle_mouse(FL_RELEASE, mouseX(e), mouseY(e), OIS_MouseButtonID_to_int(id));
    return true;
}

#endif
#endif
#ifndef NO_GUI
int FltkRenderer::handle_mouse(int ev, int x, int y, int button)
{
	switch(ev)
	{
	case FL_PUSH:
	case FL_DRAG:
	case FL_RELEASE:
		{
			//RE::output("viewev", "view event %d %d %d %d %d %d\n", ev, Fl::event_ctrl(), OIS_event_ctrl(), OIS_event_alt(), Fl::event_x(), Fl::event_y());

			if(!mOgreRenderer->isActive()) break;

			if(mbViewLock) printf("viewlock\n");
			if(mbViewLock && !Fl::event_ctrl())
				break;

			Viewpoint::msgT msg;

			{
				if(ev==FL_DRAG)
				{
					msg=Viewpoint::MOUSEMOVE;

					//RE::output("mouse","drag %d %d", x, y);
					//vector3 cursorPos=screenToWorldXZPlane(x, y);
					//RE::createEntity("cursorPos", "sphere1010.mesh")->translate(ToOgre(cursorPos));
					

				}
				else
				{
					//RE::output("mouse","push or release");
					switch(button)
					{
					case 1: msg=(ev==FL_PUSH)?Viewpoint::LBUTTONDOWN:Viewpoint::LBUTTONUP; break;
					case 2: msg=(ev==FL_PUSH)?Viewpoint::RBUTTONDOWN:Viewpoint::RBUTTONUP; break;
					case 3: msg=(ev==FL_PUSH)?Viewpoint::MBUTTONDOWN:Viewpoint::MBUTTONUP; break;
					}
				}

				// M-button emulation mode.
				//if(OIS_event_ctrl() && ev!=FL_DRAG)
				if(!OIS_event_shift() &&((OIS_event_alt()||OIS_event_ctrl()) && ev!=FL_DRAG))
				{
					msg=(ev==FL_PUSH)?Viewpoint::RBUTTONDOWN:Viewpoint::RBUTTONUP;
					if(OIS_event_alt() ) // seperate ogre window 사용시 alt키 event 잘못 들어옴. -> use c key instead.
					{
						//printf("alt pressed %d %d\n", Fl::event_alt() , (RE::renderer().mKeyboard&&RE::renderer().mKeyboard->isModifierDown(OIS::Keyboard::Alt)));
						msg=(ev==FL_PUSH)?Viewpoint::MBUTTONDOWN:Viewpoint::MBUTTONUP;
					}
				}
				//else if(OIS_event_shift() && ev!=FL_DRAG)
				//{
				//	msg=(ev==FL_PUSH)?Viewpoint::MBUTTONDOWN:Viewpoint::MBUTTONUP;
				//}
				//printf("%d %d %d\n",msg, x, y);
				//mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages(  msg, wParam, lParam);
				if(!OIS_event_shift()) // shift는 UI에 써야함.
					mOgreRenderer->viewport().m_pViewpoint->HandleMouseMessages2(  msg, x, y);
			}

			return 1;
		}
		break;
	}
	return 0;

}
#endif
