#ifdef __APPLE__
#include <Carbon/Carbon.h>
#ifdef verify 
#undef verify
#endif
#endif
#include "stdafx.h"
#include <stdio.h>
#include <sstream>
#include "../BaseLib/utility/util.h"
#include "MotionManager.h"
#include "../BaseLib/motion/viewpoint.h"
//#include "netlab.h"
//#include "gaussianprocess/gaussianprocess.h"
#include "framemoveobject.h"
#include "FltkRenderer.h"
#include "renderer.h"
#include "../../BaseLib/utility/TypeString.h"
#include "../../BaseLib/motion/VRMLloader.h"
#include "RE.h"
#ifndef NO_OGRE
#include <Ogre.h>
#include <OgreOverlay.h>
#include <OgreWindow.h>
#include "OgreOverlaySystem.h"
#include "OgreOverlayManager.h"
#include "MovableText.h"

#include "OgreCamera.h"
#include "OgreItem.h"

#include "OgreHlmsUnlit.h"
#include "OgreHlmsPbs.h"
#include "OgreHlmsManager.h"

#include "OgreHlmsPbsDatablock.h"
#include "OgreHlmsSamplerblock.h"

#include "OgreRoot.h"
#include "OgreHlmsManager.h"
#include "OgreTextureGpuManager.h"
#include "OgreTextureFilters.h"
#include "OgreHlmsPbs.h"
#include "OgreArchiveManager.h"

#include "Compositor/OgreCompositorManager2.h"

#include "OgreOverlaySystem.h"
#include "OgreOverlayManager.h"

#include "OgreTextureGpuManager.h"
#include "OgreStagingTexture.h"

#include "OgreWindowEventUtilities.h"
#include "OgreWindow.h"

#include "OgreFileSystemLayer.h"

#include "OgreHlmsDiskCache.h"
#include "OgreGpuProgramManager.h"

#include "OgreLogManager.h"

#include "OgrePlatformInformation.h"
#ifdef SEP_USE_SDL2
    #include <SDL_syswm.h>
#include <FL/Fl_Window.H>
#endif

#include "Compositor/OgreCompositorManager2.h"
#include "Compositor/OgreCompositorNodeDef.h"
#include "Compositor/OgreCompositorShadowNode.h"
#include "Compositor/Pass/PassScene/OgreCompositorPassSceneDef.h"

#include <fstream>


#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE || OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
#include "OSX/macUtils.h"
#include <Carbon/Carbon.h>
#include <ApplicationServices/ApplicationServices.h>
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
#include "System/iOS/iOSUtils.h"
#else
#include "OSXUtils.h"
#endif
#endif
#ifndef NO_OIS
 #include <OISMouse.h>
 #include <OISKeyboard.h>
 #include <OISJoyStick.h>
 #include <OISInputManager.h>
#endif
//#include "Bites/OgreBitesConfigDialog.h"
#include "ShadowMapFromCodeGameState.h"
#endif
using namespace std;

ConfigTable config;
#if defined( __APPLE__) && !defined(NO_GUI)
Ogre::Rect getWindowBounds(void* handle);
#endif

	class OgreMaterialCreator : public VRMLloader::MaterialCreator
	{
		public:
		OgreMaterialCreator(){}
		virtual void createMaterial(const char* id, const vector3 & diffuse, const vector3& specular, const vector3&  emissive, double shininess){
			printf("creating material: %s %s\n", id, diffuse.output().c_str());
			RE::renderer().createMaterial(id, diffuse, specular, shininess);
		}
	};
#ifndef _MSC_VER
 #if FL_MAJOR_VERSION == 2
#ifndef ___APPLE_CC__
 #include <fltk/x.h>
#endif
 #elif FL_MAJOR_VERSION == 1
#ifndef __APPLE_CC__
 #include <FL/x.H>
#endif
 #endif
#endif

static int cameraIndex=0;
//*********************************************************************
//************ FrameListener
//*********************************************************************
//
bool useSeperateOgreWindow()
{
#ifdef NO_GUI
	return true;
#else
	static bool _useSeperateOgreWindowCached=false;
	static bool _useSeperateOgreWindow;
	if(!_useSeperateOgreWindowCached)
	{
		const char* str=config.Find("useSeperateOgreWindow");
		printf("test:%s\n", str);
		_useSeperateOgreWindow=(str && atoi(str)==1);
		_useSeperateOgreWindowCached=true;
	}
	return _useSeperateOgreWindow;
#endif
}


#ifndef NO_OGRE

extern bool softKill;

#if !defined(__APPLE__) && !defined(_MSC_VER)
#include <X11/Xlib.h>
#endif

#endif // NO_OGRE
namespace RE	
{
	extern Globals* g_pGlobals;
}
#ifndef NO_OGRE

Ogre::Matrix4 BuildScaledOrthoMatrix(double left, double right, double bottom, double top, double fnear, double ffar)
{
  double invw = 1 / (right - left);
  double invh = 1 / (top - bottom);
  double invd = 1 / (ffar - fnear);

  Ogre::Matrix4 proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2 * invw;
  proj[0][3] = -(right + left) * invw;
  proj[1][1] = 2 * invh;
  proj[1][3] = -(top + bottom) * invh;
  proj[2][2] = -2 * invd;
  proj[2][3] = -(ffar + fnear) * invd;
  proj[3][3] = 1;

  return proj;
}
inline Ogre::Matrix4 BuildScaledOrthoMatrix(double zoom, double aspectRatio)
{
	double scale=500;
	double dfar;
	double dnear;
	dfar= config.GetFloat("farClipDistance") * 10;
	dnear=config.GetFloat("nearClipDistance");
	return BuildScaledOrthoMatrix(-scale/zoom, scale/zoom, -scale/aspectRatio/zoom, scale/aspectRatio/zoom, dnear, dfar);
}
#endif


#ifndef NO_OGRE
/* todo2
static Ogre::TexturePtr rtt_texture;
static Ogre::RenderTexture *renderTexture=NULL;
*/
static bool mbUseRTTcapture=false;
static bool mbUseOGREcapture=false;
#endif

void OgreRenderer::setBackgroundColour(float r, float g, float b)
{
#ifndef NO_OGRE
	printf("OgreRenderer::setBackgroundColour currently doesn't work.\n If necessary, adjust the 1st line of OgreRenderer::_construct(...) in renderer.cpp directly!!!\n");
	/* todo2
	viewport().mView->setBackgroundColour(Ogre::ColourValue(r,g,b,1.f));
	if(mbUseRTTcapture){
		renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue(r,g, b, 1.f));
	}
	*/
#endif
}
void OgreRenderer::Viewport::setupRTT(OgreRenderer& renderer, int width, int height)
{
#ifndef NO_OGRE

	// Viewports.

	//todo2 mView->setBackgroundColour(Ogre::ColourValue(0.f, 0.6f, 0.8f, 0.9f));

	bool bOrthographic=config.GetInt("useOrthographicProjection")==1;

	setOrthographicMode(bOrthographic);
	//mScene->setShadowUseInfiniteFarPlane(false);
	if(bOrthographic)
		mScene->setShadowFarDistance(config.GetFloat("shadowFarDistance")*10); 
	else
		mScene->setShadowFarDistance(config.GetFloat("shadowFarDistance")); 


	if(config.Find("shadowDirLightTextureOffset"))
	{
		float f=config.GetFloat("shadowDirLightTextureOffset");
		printf("shadowDirLightTextureOffset %f\n", f);
		mScene->setShadowDirLightTextureOffset(f);
	}

	mbUseRTTcapture=config.GetInt("useRTTcapture");
	if(config.Find("useOGREcapture"))
		mbUseOGREcapture=config.GetInt("useOGREcapture");

	if (mbUseRTTcapture)
	{
		/* todo2
		rtt_texture = Ogre::TextureManager::getSingleton().createManual("RttTex", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

		renderTexture = rtt_texture->getBuffer()->getRenderTarget();

		renderTexture->addViewport(mCam);
		renderTexture->getViewport(0)->setClearEveryFrame(true);
		//renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
		renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue(0.f, 0.6f, 0.8f, 0.9f));
		renderTexture->getViewport(0)->setOverlaysEnabled(true);
		renderTexture->setAutoUpdated(true);
		*/
	}

#endif
}
void OgreRenderer::Viewport::init(OgreRenderer& renderer, vectorn const& param)
{
	int width=param(0);
	int height=param(1);
#ifndef NO_OGRE
	const size_t numThreads = 1;
	//const size_t numThreads = std::max<size_t>( 1, Ogre::PlatformInformation::getNumLogicalCores() );
	//mScene = renderer.mRoot->createSceneManager("OctreeSceneManager", "OgreFltk");
	mScene = renderer.mRoot->createSceneManager( Ogre::ST_GENERIC,
			numThreads,
			"ExampleSMInstance" );

	mScene->addRenderQueueListener(renderer.mOverlaySystem);
	mScene->getRenderQueue()->setSortRenderQueue(
			Ogre::v1::OverlayManager::getSingleton().mDefaultRenderQueueId,
			Ogre::RenderQueue::StableSort );

	//Set sane defaults for proper shadow mapping
	mScene->setShadowDirectionalLightExtrusionDistance( 500.0f );
	mScene->setShadowFarDistance( 5000.0f );
#endif
	// init camera
	createCamera(renderer, width, height);
	if(param.size()==3)
	{
		// manually setup mView and mScene options
	}
	else // default automatic setup
	{
#ifndef NO_OGRE
		/*todo2
		mView = renderer.mWnd->addViewport(mCam,cameraIndex-1 );
		setupRTT(renderer, width, height);
		m_pViewpoint->m_iWidth=mView->getActualWidth();
		m_pViewpoint->m_iHeight=mView->getActualHeight();
		*/
#endif
	}
/*
	AssimpLoader l;

	Ogre::MeshPtr meshPtr;
	Ogre::SkeletonPtr mSkeleton;
	Ogre::MaterialPtr materialPtr;
	std::string my_mesh_name;
	AssimpLoader::AssOptions opt;
	opt.source="../media/models/uploads_files_838702_Civilian3.FBX";
	l.convert( opt, meshPtr, mSkeleton, materialPtr,  my_mesh_name, 1);
*/
}


void OgreRenderer::Viewport::createCamera(OgreRenderer& renderer,int width, int height)
{
	// Taesoo Camera.
	m_pViewpoint=new Viewpoint();

#ifdef NO_OGRE
	m_pViewpoint->setDefaultView();
#else
	FILE* fp;
	VERIFY(fp=fopen((RE::taesooLibPath()+"Resource/viewpoint.txt").c_str(),"r"));
	m_pViewpoint->ReadViewPoint(fp);	
//	m_pViewpoint->ReadViewPoint((mTaesooLib_path+"Resource/viewpoint.txt");	
	fclose(fp);
	FileCloseForGetToken();
#endif


	/*m_pViewpoint->GetArcBall().SetWindow( width, height,0.85f );
	  m_pViewpoint->GetArcBall().SetRadius(300.0f);
	  m_pViewpoint->GetArcBall().SetRightHanded( TRUE );*/

	m_pViewpoint->m_iWidth=width;
	m_pViewpoint->m_iHeight=height;

#ifndef NO_OGRE
	// Ogre::Camera.
	TString cameraname;
	cameraname.format("Camera %d", cameraIndex++);
	mCam = mScene->createCamera(cameraname.ptr());

	mCam->setNearClipDistance(config.GetInt("nearClipDistance"));
	mCam->setFarClipDistance(config.GetInt("farClipDistance"));
	mCam->setAutoAspectRatio(true);
//	mCam->setPosition(m_pViewpoint->m_vecVPos.x, m_pViewpoint->m_vecVPos.y, m_pViewpoint->m_vecVPos.z);
	mCameraNode=mScene->getRootSceneNode()->createChildSceneNode();
	printf("c");fflush(stdout);
	((Ogre::SceneNode*)mCam->getParentNode())->detachObject(mCam);
	//
	mCameraNode->attachObject(mCam);
	mCameraNode->setFixedYawAxis(true); // fix lookAt calls
	mCameraNode->lookAt(Ogre::Vector3(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z), Ogre::Node::TS_PARENT);
	mCam->setFOVy(Ogre::Radian(Ogre::Degree(45)));


#endif
}
void OgreRenderer::Viewport::setOrthographicMode(bool isOrtho)
{
#ifndef NO_OGRE
	m_real aspectRatio=(m_real)getActualWidth()/(m_real)getActualHeight();
	printf("aspect Ratio %f\n", aspectRatio);
	if(isOrtho)
	{
		m_pViewpoint-> m_fDepth*=4;// 카메라를 멀리 놓는다. (fog 세팅 바꿀 것.)
		m_pViewpoint->UpdateVPosFromVHD();
		changeView(*m_pViewpoint);
		
		mCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
		//	mCam->setPosition(Ogre::Vector3(0, 10000, 0));
		m_real zoom=1.5;
		m_pViewpoint->setZoom(zoom);
		//	m_real zoom=m_pViewpoint->getZoom();
		mCam->setCustomProjectionMatrix( true, BuildScaledOrthoMatrix(zoom, aspectRatio));
		m_pViewpoint->setOrthographicMode(true);
	}
	else
	{
		mCam->setProjectionType(Ogre::PT_PERSPECTIVE);
		mCam->setAspectRatio(Ogre::Real(aspectRatio));
		mCam->setCustomProjectionMatrix( false);
		m_pViewpoint->setOrthographicMode(false);
	}
#endif
}
int OgreRenderer::Viewport::getActualWidth() const { return m_pViewpoint->m_iWidth;}
int OgreRenderer::Viewport::getActualHeight() const { return m_pViewpoint->m_iHeight;}
void OgreRenderer::Viewport::setCustomProjectionMatrix(matrix4 const& mat_proj)
{
#ifndef NO_GUI
  Ogre::Matrix4 proj ;
  proj[0][0]=mat_proj._11;
  proj[0][1]=mat_proj._12;
  proj[0][2]=mat_proj._13;
  proj[0][3]=mat_proj._14;
  proj[1][0]=mat_proj._21;
  proj[1][1]=mat_proj._22;
  proj[1][2]=mat_proj._23;
  proj[1][3]=mat_proj._24;
  proj[2][0]=mat_proj._31;
  proj[2][1]=mat_proj._32;
  proj[2][2]=mat_proj._33;
  proj[2][3]=mat_proj._34;
  proj[3][0]=mat_proj._41;
  proj[3][1]=mat_proj._42;
  proj[3][2]=mat_proj._43;
  proj[3][3]=mat_proj._44;
	mCam->setCustomProjectionMatrix	(true,proj);
#endif
}

void OgreRenderer::Viewport::init(OgreRenderer& renderer, OgreRenderer::Viewport & other)
{
#ifndef NO_OGRE
	mScene=other.mScene;
#endif
	createCamera(renderer, other.m_pViewpoint->m_iWidth, other.m_pViewpoint->m_iHeight);
#ifndef NO_OGRE
	// todo2 mView = renderer.mWnd->addViewport(mCam,cameraIndex-1 );
	// todo2 mView->setBackgroundColour(other.mView->getBackgroundColour());
#endif
}
OgreRenderer::OgreRenderer()
#if !defined( NO_OGRE) 
	: BaseSystem(new ShadowMapFromCodeGameState( "")),
		mRoot(NULL),
		mWnd(NULL),
#if !defined(NO_OIS)
		mMouse(NULL),
		mKeyboard(NULL),
		mInputSystem(NULL),
#endif
#else
	:
#endif
		mStatsOn(true),
		mbPause(false), 
		mbScreenshot(false),
		mbFixedTimeStep(false),
		m_fTimeScaling(1.f),
		mScreenshotPrefix("../dump/dump")
{
	_locateTaesooLib();
	std::string mTaesooLib_path=RE::taesooLibPath();

#ifndef NO_OGRE
	((ShadowMapFromCodeGameState*)mCurrentGameState)->_notifyGraphicsSystem(this);
#endif

#ifdef _MSC_VER // WINDOWS
#if defined(_DEBUG)
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig.txt").c_str(),(mPluginPath+"plugins2_d.cfg").c_str(), (mPluginPath+"ogre2.cfg").c_str());
#else
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig.txt").c_str(),(mPluginPath+"plugins2.cfg").c_str(), (mPluginPath+"ogre2.cfg").c_str());
#endif
#elif defined(__APPLE__) 
#if defined(_DEBUG)
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig_mac.txt").c_str(),(mPluginPath+"plugins2_mac_d.cfg").c_str(), (mPluginPath+"ogre2_mac.cfg").c_str());
#else
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig_mac.txt").c_str(),(mPluginPath+"plugins2_mac.cfg").c_str(), (mPluginPath+"ogre2_mac.cfg").c_str());
#endif
#else // LINUX

	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig_linux.txt").c_str(), (mPluginPath+"plugins2_linux.cfg").c_str(), (mPluginPath+"ogre2_linux.cfg").c_str());
#endif
}


OgreRenderer::OgreRenderer(const char* fallback_configFileName, const char* configFileName, const char* plugins_file, const char* ogre_config)
#if !defined( NO_OGRE)
	: BaseSystem(new ShadowMapFromCodeGameState( "This sample is almost exactly the same as ShadowMapFromCode.\n")),
	mRoot(NULL),
	mWnd(NULL),
#ifdef SEP_USE_SDL2
	mSdlWindow( 0 ),
#endif
#if !defined(NO_OIS)
		mMouse(NULL),
		mKeyboard(NULL),
		mInputSystem(NULL),
#endif
#else
	:
#endif
		mStatsOn(true),
		mbPause(false), 
		mbScreenshot(false),
		mbFixedTimeStep(false),
		m_fTimeScaling(1.f),
		mScreenshotPrefix("../dump/dump")
{
	_locateTaesooLib();
	_constructor(fallback_configFileName, configFileName, plugins_file, ogre_config);
}

void OgreRenderer::_locateTaesooLib()
{
	if(RE::taesooLibPath()=="work/taesooLib/")
	{
		mScreenshotPrefix="work/taesooLib/dump/dump";
		mPluginPath="work/";
	}
}
void OgreRenderer::_constructor(const char* fallback_configFileName, const char* configFileName, const char* plugins_file, const char* ogre_config)
{
#ifndef NO_OGRE
	mBackgroundColour=Ogre::ColourValue( 0.2f, 0.4f, 0.6f );
#endif
	printf("loading %s\n", configFileName);
#if !defined(NO_GUI)
	config.load(fallback_configFileName, configFileName);
#endif
	ASSERT(RE::g_pGlobals==NULL);
	RE::g_pGlobals=new RE::Globals();
	RE::g_pGlobals->pRenderer = this;    
	m_pMotionManager=new MotionManager((RE::taesooLibPath()+"Resource/motion.lua").c_str());
	m_fElapsedTime=0.f;
	m_fCaptureFPS=30.f;
	mbTimeStop=false;

	auto mTaesooLib_path=RE::taesooLibPath();
	if(mTaesooLib_path=="../")
	{
		mResourceFile=(mTaesooLib_path+"Resource/resources2.cfg").c_str();
	}
	else
	{
		mResourceFile=(mTaesooLib_path+"Resource/resources2_relative.cfg").c_str();
	}
	mWriteAccessFolder=mTaesooLib_path+"Resource/";

#ifndef NO_OGRE
	mOverlaySystem=NULL;
	// Make the root

	printf("starting ogre"); fflush(stdout);
	Ogre::Log* log=NULL;

	if(config.GetInt("enableLog")==0)	 {
		Ogre::LogManager* logMgr=new Ogre::LogManager();
		log=Ogre::LogManager::getSingleton().createLog("", true, false, false);
	}
	printf("."); fflush(stdout);


	mRoot = new Ogre::Root(plugins_file, ogre_config, (log)?"":"Ogre.log");
	printf("."); fflush(stdout);

	static Ogre::MovableTextFactory _mMovableTextFactory;
	mMovableTextFactory=&_mMovableTextFactory;
	mRoot->addMovableObjectFactory(mMovableTextFactory);
		
	mStaticPluginLoader.install( mRoot );
	Ogre::RenderSystemList::const_iterator itor = mRoot->getAvailableRenderers().begin();
	Ogre::RenderSystemList::const_iterator endt = mRoot->getAvailableRenderers().end();

	// enable sRGB Gamma Conversion mode by default for all renderers,
	// but still allow to override it via config dialog
	while( itor != endt )
	{
		Ogre::RenderSystem *rs = *itor;
		rs->setConfigOption( "sRGB Gamma Conversion", "Yes" );
		++itor;
	}

	printf("."); fflush(stdout);


#endif	
	// now OgreRenderer can do non-ogre stuff.
}

void OgreRenderer::setResourceFile(const char* fn)
{
	mResourceFile=fn;
}
/*bool OgreRenderer_manualInitialize(OgreRenderer& renderer, const Ogre::String &desiredRenderer)
  {
  Ogre::RenderSystem *renderSystem;
  bool ok = false;
  Ogre::RenderSystemList *renderers =
  Ogre::Root::getSingleton().getAvailableRenderers();

// See if the list is empty (no renderers available)
if(renderers->empty())
return false;

for(Ogre::RenderSystemList::iterator it = renderers->begin();
it != renderers->end(); it++)
{
renderSystem = (*it);
if(strstr(renderSystem->getName().c_str(), desiredRenderer.c_str()))
{
ok = true;
break;
}
}

if(!ok) {
// We still don't have a renderer; pick
// up the first one from the list
renderSystem = (*renderers->begin());
}

Ogre::Root::getSingleton().setRenderSystem(renderSystem);

// Manually set some configuration options (optional)
//renderSystem->setConfigOption("Video Mode", "640 x 480");

return true;

}*/
#if !defined(NO_OGRE) && defined(SEP_USE_SDL2)
void translateSDLEventToFLTK(const SDL_Event& sdlEvent) {

	static int fltkButton=0;
	static bool isDragging=false;
	int mouse_ev=FL_NO_EVENT;
    switch (sdlEvent.type) {
        case SDL_KEYDOWN:
		case SDL_KEYUP: 
			{
				int fltkKey = -1;
				switch (sdlEvent.key.keysym.sym) 
				{
					case SDLK_ESCAPE: 
						fltkKey = FL_Escape; softKill=true; break;
					case SDLK_RETURN: 
						fltkKey = FL_Enter; break;
					case SDLK_SPACE: 
						fltkKey = ' '; break;
					case SDLK_LSHIFT:
						{
							if(sdlEvent.type==SDL_KEYUP)
								Fl::e_state&=~FL_SHIFT;
							else
								Fl::e_state|=FL_SHIFT;
							printf("shift %d\n", Fl::event_state()&FL_SHIFT);
						}
						break;
					case SDLK_LCTRL:
					case 1073741881: //   
						{
							if(sdlEvent.type==SDL_KEYUP)
								Fl::e_state&=~FL_CTRL;
							else
								Fl::e_state|=FL_CTRL;
							printf("ctrl %d\n", Fl::event_ctrl());
						}
						break;

					case SDLK_LALT:
						{
							if(sdlEvent.type==SDL_KEYUP)
								Fl::e_state&=~FL_ALT;
							else
								Fl::e_state|=FL_ALT;
							printf("alt %d\n", Fl::event_state()&FL_ALT);
						}
						break;

						// 다른 키들에 대한 매핑 추가
					default: 
						{
							auto sym=sdlEvent.key.keysym.sym;
							if (sym>=SDL_SCANCODE_A && sym<=SDL_SCANCODE_Z)
								fltkKey='a'+sym-SDL_SCANCODE_A;
							else if (sym>=SDL_SCANCODE_1 && sym<=SDL_SCANCODE_9)
								fltkKey='1'+sym-SDL_SCANCODE_1;
							else if (sym==SDL_SCANCODE_0)
								fltkKey='0';
							else
								fltkKey=sym;
							break;
						}
				}
				if(fltkKey!=-1)
				{
					printf("key%d\n", fltkKey);
					//Fl::e_keysym = fltkKey;
					Fl::e_keysym = sdlEvent.key.keysym.sym; 

					if(sdlEvent.type==SDL_KEYUP)
						Fl::handle_(FL_KEYUP, &RE::FltkRenderer());
					else
						Fl::handle_(FL_KEYDOWN,& RE::FltkRenderer());
				}
            break;
        }
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP: {
            switch (sdlEvent.button.button) {
                case SDL_BUTTON_LEFT: fltkButton = 1; break;
                case SDL_BUTTON_RIGHT: fltkButton = 2; break;
                case SDL_BUTTON_MIDDLE: fltkButton = 3; break;
            }
			if(sdlEvent.type==SDL_MOUSEBUTTONDOWN)
			{
				isDragging=true;
				mouse_ev=FL_PUSH;
			}
			else
			{
				isDragging=false;
				mouse_ev=FL_RELEASE;
			}

			printf("button: %d\n", fltkButton);
            break;
        }
        case SDL_MOUSEMOTION: {
			if(isDragging)
				mouse_ev=FL_DRAG;
			else
				mouse_ev=FL_MOVE;
            break;
        }
        // 다른 SDL 이벤트에 대한 처리 추가
        default:
            break;
    }

	if(mouse_ev!=FL_NO_EVENT)
	{
		auto* m_pHandler=RE::FltkRenderer().m_pHandler;

		if(m_pHandler && m_pHandler->handleRendererMouseEvent(mouse_ev, sdlEvent.motion.x, sdlEvent.motion.y, fltkButton))
			return;
		RE::FltkRenderer().handle_mouse(mouse_ev, sdlEvent.motion.x, sdlEvent.motion.y, fltkButton);
	}
}

#endif
void OgreRenderer::initialize(void* handle, int width, int height)
{

	_hWnd=handle;
#ifndef NO_OGRE
    #ifdef SEP_USE_SDL2
        //if( SDL_Init( SDL_INIT_EVERYTHING ) != 0 )
        if( SDL_Init( SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK |
                      SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS ) != 0 )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR, "Cannot initialize SDL2!",
                         "GraphicsSystem::initialize" );
        }
    #endif

		mOverlaySystem=new Ogre::v1::OverlaySystem();
	try
	{

		// Set parameters of render system (window size, etc.)

		// Show the configuration dialog and initialise the system
		// You can skip this and use root.restoreConfig() to load configuration
		// settings if you were sure there are valid ones saved in ogre.cfg
#ifndef NO_OGRE
		if(config.GetInt("showConfigDlg"))
		{
			if(!mRoot->showConfigDialog())
			{
				Msg::error("Configuration canceled");
			}
		}
		else
		{

			if(!mRoot->restoreConfig())
			{
				printf("Displaying configuration dialog...\n");
				if(!mRoot->showConfigDialog())
				{
					Msg::error("Configuration canceled");
				}
			}
		}

#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
	if(!mRoot->getRenderSystem())
	{
		Ogre::RenderSystem *renderSystem =
			mRoot->getRenderSystemByName( "Metal Rendering Subsystem" );
		mRoot->setRenderSystem( renderSystem );
	}
#endif
#if OGRE_PLATFORM == OGRE_PLATFORM_ANDROID
	if( !mRoot->getRenderSystem() )
	{
		Ogre::RenderSystem *renderSystem =
			mRoot->getRenderSystemByName( "Vulkan Rendering Subsystem" );
		mRoot->setRenderSystem( renderSystem );
	}
#endif
		/*if(!config.GetInt("showConfigDlg"))
		  {
		  mRoot->getRenderSystem()->validateConfigOptions();
		//OgreRenderer_manualInitialize(*this, "OpenGL");
		}*/

		// If returned true, user clicked OK so initialise
		// Here we choose to let the system create a default rendering window by passing 'true'
		try
		{
			// Root and Scene.
			if(useSeperateOgreWindow())
			{
#ifdef SEP_USE_SDL2
				if(config.Find("useOGREcapture")) mbUseOGREcapture=config.GetInt("useOGREcapture");
				mWnd = mRoot->initialise(false);

				Ogre::ConfigOptionMap& cfgOpts = mRoot->getRenderSystem()->getConfigOptions();

				width   = 1280;
				height  = 720;

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
				{
					Ogre::Vector2 screenRes = iOSUtils::getScreenResolutionInPoints();
					width = static_cast<int>( screenRes.x );
					height = static_cast<int>( screenRes.y );
				}
#endif

				Ogre::ConfigOptionMap::iterator opt = cfgOpts.find( "Video Mode" );
				if( opt != cfgOpts.end() && !opt->second.currentValue.empty() )
				{
					//Ignore leading space
					const Ogre::String::size_type start = opt->second.currentValue.find_first_of("012356789");
					//Get the width and height
					Ogre::String::size_type widthEnd = opt->second.currentValue.find(' ', start);
					// we know that the height starts 3 characters after the width and goes until the next space
					Ogre::String::size_type heightEnd = opt->second.currentValue.find(' ', widthEnd+3);
					// Now we can parse out the values
					width   = Ogre::StringConverter::parseInt( opt->second.currentValue.substr( 0, widthEnd ) );
					height  = Ogre::StringConverter::parseInt( opt->second.currentValue.substr(
								widthEnd+3, heightEnd ) );
				}

				Ogre::NameValuePairList params;
				bool fullscreen = Ogre::StringConverter::parseBool( cfgOpts["Full Screen"].currentValue );
				fullscreen=false;
				int screen = 0;
				int posX = SDL_WINDOWPOS_CENTERED_DISPLAY(screen);
				int posY = SDL_WINDOWPOS_CENTERED_DISPLAY(screen);

				SDL_DisplayMode DM;
				SDL_GetCurrentDisplayMode(0, &DM);
				Fl_Window* parent= (Fl_Window*)handle;
				while(parent->parent())
					parent=(Fl_Window*)parent->parent();
				posX= (DM.w-width)/2+parent->w()/2;
				/*

				Fl_Window* parent= (Fl_Window*)handle;

				printf("pos %d\n", Fl::event_x_root()- Fl::event_x());
				posX=parent->x();
				while(parent->parent())
					parent=(Fl_Window*)parent->parent();

				printf("posX %d %d\n", parent->x(), posX);
				posX=parent->x();
				posY=parent->y();
				*/

				if(fullscreen)
				{
					posX = SDL_WINDOWPOS_UNDEFINED_DISPLAY(screen);
					posY = SDL_WINDOWPOS_UNDEFINED_DISPLAY(screen);
				}

				mSdlWindow = SDL_CreateWindow(
						"TaesooLib",    // window title
						posX,               // initial x position
						posY,               // initial y position
						width,              // width, in pixels
						height,             // height, in pixels
						SDL_WINDOW_SHOWN
						| (fullscreen ? SDL_WINDOW_FULLSCREEN : 0) | SDL_WINDOW_RESIZABLE );

				//Get the native whnd
				SDL_SysWMinfo wmInfo;
				SDL_VERSION( &wmInfo.version );

				if( SDL_GetWindowWMInfo( mSdlWindow, &wmInfo ) == SDL_FALSE )
				{
					OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR,
							"Couldn't get WM Info! (SDL2)",
							"GraphicsSystem::initialize" );
				}

				switch( wmInfo.subsystem )
				{
#if defined(SDL_VIDEO_DRIVER_WINDOWS)
					case SDL_SYSWM_WINDOWS:
						// Windows code
						handle=(void*)wmInfo.info.win.window ;
						break;
#endif
#if defined(SDL_VIDEO_DRIVER_WINRT)
					case SDL_SYSWM_WINRT:
						// Windows code
						handle = (void*)wmInfo.info.winrt.window ;
						break;
#endif
#if defined(SDL_VIDEO_DRIVER_COCOA)
					case SDL_SYSWM_COCOA:
						handle  = (void*)WindowContentViewHandle(wmInfo);
						break;
#endif
#if defined(SDL_VIDEO_DRIVER_X11)
					case SDL_SYSWM_X11:
						handle = (void*)wmInfo.info.x11.window ;
						params.insert( std::make_pair(
									"SDL2x11", Ogre::StringConverter::toString( (uintptr_t)&wmInfo.info.x11 ) ) );
						break;
#endif
					default:
						OGRE_EXCEPT( Ogre::Exception::ERR_NOT_IMPLEMENTED,
								"Unexpected WM! (SDL2)",
								"GraphicsSystem::initialize" );
						break;
				}

				Ogre::String winHandle;
#ifdef _MSC_VER
				winHandle = Ogre::StringConverter::toString((uintptr_t)handle); 
#elif defined(__APPLE__)
				//misc["currentGLContext"]=Ogre::String("True");  
				winHandle  = Ogre::StringConverter::toString(WindowContentViewHandleFltk(handle));
#else
				winHandle=Ogre::StringConverter::toString((uintptr_t)handle);
				//XSync(fl_display, False);
#endif
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WINRT
				params.insert( std::make_pair("externalWindowHandle",  winHandle) );
#else
				params.insert( std::make_pair("parentWindowHandle",  winHandle) );
#endif

				params.insert( std::make_pair("title", "win title") );
				params.insert( std::make_pair("gamma", cfgOpts["sRGB Gamma Conversion"].currentValue) );
				if( cfgOpts.find( "VSync Method" ) != cfgOpts.end() )
					params.insert( std::make_pair( "vsync_method", cfgOpts["VSync Method"].currentValue ) );
				params.insert( std::make_pair("FSAA", cfgOpts["FSAA"].currentValue) );
				params.insert( std::make_pair("vsync", cfgOpts["VSync"].currentValue) );
				params.insert( std::make_pair("reverse_depth", "Yes" ) );

				initMiscParamsListener( params );

				mWnd = mRoot->createRenderWindow("My sub render window", width, height, false, &params); 

				size_t hWnd = 0;
				//mWnd->getCustomAttribute("WINDOW", &hWnd);
				_hWnd=handle;
				hWnd=(size_t)_hWnd;

				createInputSystems(hWnd);
#else // ifndef SEP_USE_SDL2
				mWnd = mRoot->initialise(true); // create renderwindow
				size_t hWnd = 0;

				//mWnd->getCustomAttribute("WINDOW", &hWnd);
				hWnd=(size_t)_hWnd;

				createInputSystems(hWnd);
#endif
			}
			else
				mWnd = mRoot->initialise(false);
		}
		catch(Ogre::Exception &e)
		{
			printf("%s\n", e.what());
			Msg::msgBox("init failed - showing configuration dialog");
			if(!mRoot->showConfigDialog( ))
			{
				Msg::error("Configuration canceled");
			}
			if(useSeperateOgreWindow())
				mWnd = mRoot->initialise(true); // create renderwindow
			else
				mWnd = mRoot->initialise(false);
		}
		
		// Render system.
		mRSys = mRoot->getRenderSystem();

		if(!useSeperateOgreWindow())
		{
			createRenderWindow(handle,width, height);
		}

		setupResources();
		vectorn param(2);
		param(0)=width;
		param(1)=height;

		mViewports.resize(mViewports.size()+1);
		mViewports.back()=new Viewport();
		mCurrViewports=mViewports.size()-1;
#else // #ifndef NO_OGRE
		vectorn param(2);
		param(0)=10;
		param(1)=10;
		mViewports.resize(mViewports.size()+1);
		mViewports.back()=new Viewport();
		mCurrViewports=mViewports.size()-1;
#endif
		viewport().init(*this, param);

#ifndef NO_OGRE		

		mWorkspace = setupCompositor();
		BaseSystem::initialize();


		//mScene->setShowDebugShadows(true);

		// Lights.
		// light는 프로그램에서 생성하시오.

		//frame listener 장착
		mRoot->addFrameListener(this);


		// All set up, activate.

		// Do not create scene here! :TAESOO
		createScene01();

		// todo2: leak : OgreMaterialCreator
		VRMLloader::registerMaterialCreator(new OgreMaterialCreator());
		setupShadowNode(true);

	}
	catch(Ogre::Exception &e)
	{
		Ogre::String s = "OgreApp::Init() - Exception:\n" + e.getFullDescription() + "\n";
		Ogre::LogManager::getSingleton().logMessage(s, Ogre::LML_CRITICAL);
		//Msg::error("Exception%s\n",e.getFullDescription().c_str());
		printf("Exception%s\n",e.getFullDescription().c_str());

		// Clean up.
		if(mRoot)
		{
			delete mOverlaySystem;
			delete mRoot;
		}
		mRoot=NULL;
		throw std::runtime_error(e.getFullDescription());
	}
#endif
}

void OgreRenderer::deinitialize()
{
#ifndef NO_OGRE
	BaseSystem::deinitialize();

	saveTextureCache();
	saveHlmsDiskCache();

	if( getSceneManager() )
		getSceneManager()->removeRenderQueueListener( mOverlaySystem );

	delete mOverlaySystem;
	mOverlaySystem = 0;


	delete mRoot;
	mRoot = 0;
#ifdef SEP_USE_SDL2
	if( mSdlWindow )
	{
		// Restore desktop resolution on exit
		SDL_SetWindowFullscreen( mSdlWindow, 0 );
		SDL_DestroyWindow( mSdlWindow );
		mSdlWindow = 0;
	}

	SDL_Quit();
#endif
#endif

}

OgreRenderer::Viewport& OgreRenderer::viewport()
{
	return *mViewports[mCurrViewports];
}
OgreRenderer::Viewport const& OgreRenderer::viewport()	const 
{
	return *mViewports[mCurrViewports];
}
OgreRenderer::Viewport& OgreRenderer::viewport(int viewport)	{ mCurrViewports=viewport; return *mViewports[mCurrViewports];}
OgreRenderer::Viewport::Viewport()
{
	m_pViewpoint=NULL;
#ifndef NO_OGRE
	mScene=NULL;
	mCam=NULL;
	//mView=NULL;
#endif
}
OgreRenderer::Viewport::~Viewport()
{
	delete m_pViewpoint;
}
OgreRenderer::~OgreRenderer()
{
	delete RE::g_pGlobals;
	RE::g_pGlobals=NULL;
	//RE::g_pGlobals->pRenderer=NULL;
#ifndef NO_OGRE
	//todo2 rtt_texture.setNull();
	delete mRoot;
	delete mCurrentGameState;
#endif
	delete m_pMotionManager;	
}


void OgreRenderer::Viewport::changeView(matrix4 const& matview)
{
#ifndef NO_OGRE
	m_pViewpoint->SetViewMatrix(matview);
	mCameraNode->setPosition(m_pViewpoint->m_vecVPos.x, m_pViewpoint->m_vecVPos.y, m_pViewpoint->m_vecVPos.z);
	mCameraNode->lookAt(Ogre::Vector3(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z), Ogre::Node::TS_PARENT);
//	mCam->lookAt(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z);
#endif
}

void OgreRenderer::Viewport::changeView(Viewpoint const& view)
{
	*m_pViewpoint=view;
	m_real m_zoom;
#ifndef NO_OGRE
	//float interval = isLeft ? -config.GetFloat("interval") : config.GetInt("interval");
	mCameraNode->setPosition(m_pViewpoint->m_vecVPos.x, m_pViewpoint->m_vecVPos.y, m_pViewpoint->m_vecVPos.z);
	mCameraNode->lookAt(Ogre::Vector3(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z), Ogre::Node::TS_PARENT);
	mCameraNode->setFixedYawAxis( true, ToOgre(m_pViewpoint->m_vecVUp));

	//	std::cout << mCam->getName() << mCam->getPosition() << std::endl;
	if(mCam->isCustomProjectionMatrixEnabled())
	{
		// orthographic mode.

		bool bOrthographic=m_pViewpoint->getOrthographicMode();	
		if(bOrthographic)
		{
			//OrthoMode
			m_zoom=m_pViewpoint->getZoom();
			m_real aspectRatio=(m_real)getActualWidth()/(m_real)getActualHeight();
			m_real zoom=m_zoom;
			mCam->setCustomProjectionMatrix( true, BuildScaledOrthoMatrix(zoom, aspectRatio));
		}
		//	cout<<"m_zoom="<<m_zoom<<endl;
	}
#endif
}



TString getCurrentDirectory();
bool OgreRenderer::frameStarted(const Ogre::FrameEvent& evt)
{
#ifndef NO_OGRE

	if(mWnd->isClosed()) return false;
	if(mbPause) return true;

	float fElapsedTime =evt.timeSinceLastFrame*m_fTimeScaling ;
#else
	float fElapsedTime=0.03333; 
#endif

	if(fElapsedTime>0.1*m_fTimeScaling)
		fElapsedTime=0.1*m_fTimeScaling;	// 시간이 너무 많이 진행하지 않도록 막았다.

	// 100frame/s 이상은 나오지 않도록 막았다. 이렇게함으로써 입력이 약간 부드러워진다.-다시 없앴다; 입력이 부드러워질려면 실제 렌더링을 건너뛰어야한다.
	/*if(m_fElapsedTime+fElapsedTime<0.01)
	  {
	  m_fElapsedTime+=fElapsedTime;
	  return true;
	  }*/

	m_fElapsedTime+=fElapsedTime;

	if(mbTimeStop)
	{
		mbTimeStop=false;
		m_fElapsedTime=0.0001;
	}

	if(mbScreenshot || mbFixedTimeStep)
	{
		m_fElapsedTime=1.f/m_fCaptureFPS *m_fTimeScaling;
	}


	{
		std::list<FrameMoveObject*>::iterator i;

		for (i = mFrameMoveObjects.begin(); i != mFrameMoveObjects.end(); ++i)
		{
			(*i)->FrameMove(m_fElapsedTime);
		}
	}
	{
		std::list<FrameMoveObject*>::iterator i;

		for (i = mAfterFrameMoveObjects.begin(); i != mAfterFrameMoveObjects.end(); ++i)
		{
			(*i)->FrameMove(m_fElapsedTime);
		}
	}

#ifndef NO_OGRE
	BaseSystem::update( static_cast<float>( 1.0/60.0 ) );
#endif
	for (int i=0,n=mViewports.size();i<n; i++)
		mViewports[i]-> changeView(*mViewports[i]->m_pViewpoint);
	
	//viewport().changeView(*viewport().m_pViewpoint);
	m_fElapsedTime=0;
	return true;
}


bool OgreRenderer::frameEnded(const Ogre::FrameEvent& evt)
{
	return true;
}

void OgreRenderer::speedControl(float f)
{
	m_fTimeScaling= f;
}

void OgreRenderer::setScreenshotPrefix(const char* prefix)
{
	mScreenshotPrefix=prefix;
}


void OgreRenderer::pause(bool b)
{
	mbPause = b;
}

void OgreRenderer::toggleScreenShotMode()
{
	if(mbScreenshot)
		screenshot(false);
	else
	{
		cout << "Screen capture started" << endl;
		screenshot(true);
	}
}

void OgreRenderer::fixedTimeStep(bool b)
{
	mbFixedTimeStep=b;
}

static int mMotionBlurAmount=1;
void OgreRenderer::setScreenshotMotionBlur(int n)
{
	mMotionBlurAmount=n;
}
void OgreRenderer::screenshot(bool b)
{
	mbScreenshot = b;
#ifndef NO_OGRE
	if(!mbUseRTTcapture && !mbUseOGREcapture)
#endif
	{
		if(b)
			RE::FltkRenderer().initCapture(mScreenshotPrefix, mMotionBlurAmount);
		else
			RE::FltkRenderer().endCapture();
	}
}

void OgreRenderer::addFrameMoveObject(FrameMoveObject* pFMO)
{
	mFrameMoveObjects.remove(pFMO);
	mFrameMoveObjects.push_back(pFMO);
}

void OgreRenderer::removeFrameMoveObject(FrameMoveObject* pFMO)
{
	mFrameMoveObjects.remove(pFMO);
}

void OgreRenderer::addAfterFrameMoveObject(FrameMoveObject* pFMO)
{
	mAfterFrameMoveObjects.push_back(pFMO);
}

void OgreRenderer::removeAfterFrameMoveObject(FrameMoveObject* pFMO)
{
	mAfterFrameMoveObjects.remove(pFMO);
}
/*
#include "../BaseLib/image/Image.h"
#include "../BaseLib/image/ImagePixel.h"
void Renderer::DumpCurrentScreen(const char* filename)
{
#ifdef DUMPSCREEN_ENABLED
// X8,R8,G8,B8 format
LPDIRECT3DSURFACE8 backBuffer;
backBuffer=GetBackBuffer();
D3DSURFACE_DESC sdBB;
backBuffer->GetDesc(&sdBB);

CImage image;
image.Create(sdBB.Width, sdBB.Height, 24);
CImagePixel cIP(&image);

ASSERT(sdBB.Width<1000);
D3DLOCKED_RECT lrBB;
while(backBuffer->LockRect(&lrBB,NULL,D3DLOCK_READONLY)!=S_OK) Sleep(5);
//D3DVERIFY(backBuffer->LockRect(&lrBB,NULL,D3DLOCK_READONLY));
//D3DVERIFY(backBuffer->LockRect(&lrBB,NULL,D3DLOCK_NOSYSLOCK));

tagCOLOR* pIP;
DWORD pixel;
for( DWORD y=0; y<sdBB.Height; y++ )
{
DWORD* p32 = (DWORD*)((BYTE*)lrBB.pBits+ y*lrBB.Pitch);

for( DWORD x=0; x<sdBB.Width; x++ )
{
pixel=*p32;
pIP=&cIP.Pixel(x,y);
pIP->R=(pixel>>16)&0xff;
pIP->G=(pixel>>8)&0xff;
pIP->B=(pixel)&0xff;
p32++;
}//end for x=width
} //end for y=height

backBuffer->UnlockRect();
image.Save(filename);
#endif
}

void Renderer::StartDump()
{
m_nCurrDumpSequence=0;
}

void Renderer::EndDump()
{
m_nCurrDumpSequence=-1;
}

*/
#ifndef NO_OGRE
void OgreRenderer::addResourceLocation( const Ogre::String &archName, const Ogre::String &typeName,
		const Ogre::String &secName )
{
//#if (OGRE_PLATFORM == OGRE_PLATFORM_APPLE) || (OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS)
#if (OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS)
	// OS X does not set the working directory relative to the app,
	// In order to make things portable on OS X we need to provide
	// the loading with it's own bundle path location
	Ogre::ResourceGroupManager::getSingleton().addResourceLocation( Ogre::String( Ogre::macBundlePath() + "/" + archName ), typeName, secName );
#else
	Ogre::ResourceGroupManager::getSingleton().addResourceLocation( archName, typeName, secName);
#endif
}

void OgreRenderer::setupResources(void)
{
	// Load resource paths from config file
	Ogre::ConfigFile cf;
	//cf.load( AndroidSystems::openFile( mResourcePath + "resources2.cfg" ) );
	cf.load( mResourceFile.ptr() ) ;

	// Go through all sections & settings in the file
	Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

	Ogre::String secName, typeName, archName;
	while( seci.hasMoreElements() )
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();

		if( secName != "Hlms" )
		{
			Ogre::ConfigFile::SettingsMultiMap::iterator i;
			for (i = settings->begin(); i != settings->end(); ++i)
			{
				typeName = i->first;
				archName = i->second;
				addResourceLocation( archName, typeName, secName );
			}
		}
	}
	// loadResources
	registerHlms();

	loadTextureCache();
	loadHlmsDiskCache();

	// Initialise, parse scripts etc
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups( true );

	// Initialize resources for LTC area lights and accurate specular reflections (IBL)
	Ogre::Hlms *hlms = mRoot->getHlmsManager()->getHlms( Ogre::HLMS_PBS );
	OGRE_ASSERT_HIGH( dynamic_cast<Ogre::HlmsPbs*>( hlms ) );
	Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlms );
	try
	{
		hlmsPbs->loadLtcMatrix();
	}
	catch( Ogre::FileNotFoundException &e )
	{
		Ogre::LogManager::getSingleton().logMessage( e.getFullDescription(), Ogre::LML_CRITICAL );
		Ogre::LogManager::getSingleton().logMessage(
				"WARNING: LTC matrix textures could not be loaded. Accurate specular IBL reflections "
				"and LTC area lights won't be available or may not function properly!",
				Ogre::LML_CRITICAL );
	}

}
#endif

bool OgreRenderer::isActive()		
{ 
#ifndef NO_OGRE
	return mWnd && mWnd->isVisible();
#else
	return true;
#endif
}

#ifndef NO_OGRE

Ogre::Quaternion ToOgre(const quater& q)
{
	return Ogre::Quaternion(q.w, q.x, q.y, q.z);
}

Ogre::Vector3 ToOgre(const vector3& v)
{
	return Ogre::Vector3(v.x, v.y, v.z);
}

vector3 ToBase(const Ogre::Vector3& v)
{
	return vector3(v.x, v.y, v.z);
}

quater ToBase(const Ogre::Quaternion& v)
{
	return quater(v.w, v.x, v.y, v.z);
}
#endif
void OgreRenderer::renderOneFrame()
{
#ifndef NO_OGRE
	if(useSeperateOgreWindow() || isActive())
	{
		if(useSeperateOgreWindow()){
#ifdef __APPLE__
		CGDisplayShowCursor(kCGDirectMainDisplay);
		CGAssociateMouseAndMouseCursorPosition(TRUE);
#endif
#ifndef NO_OIS
		if( mMouse ) {
			mMouse->capture();
		}

		if( mKeyboard ) {
			mKeyboard->capture();
		}
#endif
		}
		BaseSystem::finishFrame();

		// update shadowmap
		Ogre::WindowEventUtilities::messagePump();
    #ifdef SEP_USE_SDL2
        SDL_Event evt;
        while( SDL_PollEvent( &evt ) )
        {
            switch( evt.type )
            {
            case SDL_WINDOWEVENT:
                handleWindowEvent( evt );
                break;
            case SDL_QUIT:
                softKill = true;
                break;
            default:
                break;
            }

			translateSDLEventToFLTK(evt);
            //mInputHandler->_handleSdlEvents( evt );
        }
    #endif

		if( !mWnd->isVisible() )
		{
			//Don't burn CPU cycles unnecessary when we're minimized.
			Ogre::Threads::Sleep( 500 );
		}

		mRoot->renderOneFrame();

		/*todo2
		if(mbScreenshot && renderTexture) {
			TString fn;
			static int _c=0;
			fn.format("%s/%05d.jpg", mScreenshotPrefix.ptr(), _c++);
			renderTexture->writeContentsToFile(fn.ptr());
		}
		*/
		//if(mbScreenshot && mbUseOGREcapture){
		if(mbScreenshot ){
			TString fn;
			static int _c=0;
			fn.format("%s/%05d.jpg", mScreenshotPrefix.ptr(), _c++);
			printf("%d\n", mWnd->getTexture()->getNumMipmaps());
			//
			//
#ifdef __APPLE__
#ifdef __MAC_15_0

			static bool bFirst=true;
			if(bFirst)
			{
				Msg::msgBox("To capture a screenshot on MacOS v15, use the ogre-next3 branch of taesooLib-next.git and ogre-next3.git" );
				bFirst=false;
			}

			/*
			 * everything doesn't work.  
			CGImageRef screenShot = takeScreenShot(_hWnd);
			
			

			CFStringRef file ;
			file = CFStringCreateWithCString(kCFAllocatorDefault, fn.ptr(), kCFStringEncodingMacRoman);
			CFStringRef type = CFSTR("public.jpeg");
			CFURLRef urlRef = CFURLCreateWithFileSystemPath( kCFAllocatorDefault, file, kCFURLPOSIXPathStyle, false );
			CGImageDestinationRef image_destination = CGImageDestinationCreateWithURL( urlRef, type, 1, NULL );
			CGImageDestinationAddImage( image_destination, screenShot, NULL );
			CGImageDestinationFinalize( image_destination );
			CFRelease(file);
			*/
#else
			//mWnd->getCustomAttribute("WINDOW", &hWnd);
			Ogre::Rect _r = getWindowBounds(_hWnd);
			CGRect r=createCGrect(_hWnd);

			CGImageRef screenShot = CGWindowListCreateImage( r, kCGWindowListOptionOnScreenOnly, kCGNullWindowID, kCGWindowImageDefault);

			CFStringRef file ;
			file = CFStringCreateWithCString(kCFAllocatorDefault, fn.ptr(), kCFStringEncodingMacRoman);
			CFStringRef type = CFSTR("public.jpeg");
			CFURLRef urlRef = CFURLCreateWithFileSystemPath( kCFAllocatorDefault, file, kCFURLPOSIXPathStyle, false );
			CGImageDestinationRef image_destination = CGImageDestinationCreateWithURL( urlRef, type, 1, NULL );
			CGImageDestinationAddImage( image_destination, screenShot, NULL );
			CGImageDestinationFinalize( image_destination );
			CFRelease(file);
#endif
#else
			if(mbUseOGREcapture){
				fn.format("%s/%05d.png", mScreenshotPrefix.ptr(), _c-1);
				printf("%s\n", fn.ptr());
				mWnd->getTexture()->writeContentsToFile(fn.ptr(),0,0);
			}
#endif
		}
	}
#else
	frameStarted(Ogre::FrameEvent());
	frameEnded(Ogre::FrameEvent());
#endif
}
    #ifdef SEP_USE_SDL2
    void OgreRenderer::handleWindowEvent( const SDL_Event& evt )
    {
		auto* mRenderWindow=mWnd;
        switch( evt.window.event )
        {
            /*case SDL_WINDOWEVENT_MAXIMIZED:
                SDL_SetWindowBordered( mSdlWindow, SDL_FALSE );
                break;
            case SDL_WINDOWEVENT_MINIMIZED:
            case SDL_WINDOWEVENT_RESTORED:
                SDL_SetWindowBordered( mSdlWindow, SDL_TRUE );
                break;*/
            case SDL_WINDOWEVENT_SIZE_CHANGED:
                int w,h;
                SDL_GetWindowSize( mSdlWindow, &w, &h );
#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
                mRenderWindow->requestResolution( w, h );
#endif
                mRenderWindow->windowMovedOrResized();
                break;
            case SDL_WINDOWEVENT_RESIZED:
#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
                mRenderWindow->requestResolution( evt.window.data1, evt.window.data2 );
#endif
                mRenderWindow->windowMovedOrResized();
                break;
            case SDL_WINDOWEVENT_CLOSE:
                break;
        case SDL_WINDOWEVENT_SHOWN:
            mRenderWindow->_setVisible( true );
            break;
        case SDL_WINDOWEVENT_HIDDEN:
            mRenderWindow->_setVisible( false );
            break;
        case SDL_WINDOWEVENT_FOCUS_GAINED:
            mRenderWindow->setFocused( true );
            break;
        case SDL_WINDOWEVENT_FOCUS_LOST:
            mRenderWindow->setFocused( false );
            break;
        }
    }
    #endif
void OgreRenderer::addNewViewport()
{

	mViewports.resize(mViewports.size()+1);
	mViewports.back()=new Viewport();
	mViewports[mViewports.size()-1]->init(*this,* mViewports[0]);


}
#define TEXTURE_MAX_RESOLUTION 2048
int OgreRenderer::_getOgreTextureWidth(const char* texturename)
{
	
#ifndef NO_OGRE
	/*todo2
	Ogre::TexturePtr texture =Ogre::TextureManager::getSingleton().getByName(texturename);
	if(texture.isNull())
		return -1;
	else
		return texture->getWidth();
		*/
	return -1;
#else
	return -1;
#endif
}
void OgreRenderer::_updateDynamicTexture(const char* texturename, CImage const& image, bool reuse)	
{
#ifndef NO_OGRE
	int _width=image.GetWidth();
	int width2=TEXTURE_MAX_RESOLUTION;
	while(width2>_width) width2/=2;

	if(width2<2)
	{
		Msg::msgBox( "??? resolution error %d.\n Retry after deleting *.texturecache in the fbx folder.", image.GetWidth());
	}

	Ogre::TextureGpuManager *textureManager = getRoot()->getRenderSystem()->getTextureGpuManager();
	bool hasTexture=textureManager-> hasTextureResource( Ogre::String (texturename), 
			Ogre::ResourceGroupManager:: AUTODETECT_RESOURCE_GROUP_NAME);
	//printf("has texture %d %d", hasTexture, width2);
	if (hasTexture && !reuse)
		Msg::error("I dont' know how to remove texture:%s", texturename);
	Ogre::TextureGpu* texture=NULL;

	if(!reuse || !hasTexture)
	{
		//printf("creating %s\n", texturename);

        Ogre::Image2 origImage;
        //Load floor diffuse
        //origImage.load( "BlueCircle.png", Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME ); width2=origImage.getWidth();
        origImage.createEmptyImage( width2, width2, 1, Ogre::TextureTypes::Type2D, Ogre:: PFG_RGBA8_UNORM);

		{
			Ogre::uint8* pDest = (Ogre::uint8*)origImage.getRawBuffer();
			Ogre::uint8* originalDest=pDest;
			size_t bytesPerRow=width2*4;


			if (image.getOpacityMap())
			{
				int skip=image.GetWidth()/width2;
				for (size_t j = 0; j < width2; j++)
				{
					int h=j*image.GetHeight()/width2;
					//printf("%d %d\n", j,j);
					if(h>image.GetHeight()-1) h=image.GetHeight()-1;
					//CPixelRGB8* c_line=image.GetPixel(0, h);
					CPixelRGB8* c_line=image.GetPixel(0, image.GetHeight()-h-1); // flip_y
					const unsigned char* c_linea=image.getOpacity(0,image.GetHeight()-h-1); // flip_y

					//std::cout << y <<"/"<<image.GetHeight()<<std::endl;
					for(size_t i = 0; i < width2; i++)
					{
						*pDest++ = c_line->R; // R
						*pDest++ = c_line->G ; // G
						*pDest++ = c_line->B; // B
						*pDest++ = *c_linea; // A
						c_line+=skip;
						c_linea+=skip;
					}
				}

			}
			else
			{
				int skip=image.GetWidth()/width2;
				for (size_t j = 0; j < width2; j++)
				{
					int h=j*image.GetHeight()/width2;
					//printf("%d %d\n", j,j);
					if(h>image.GetHeight()-1) h=image.GetHeight()-1;
					//CPixelRGB8* c_line=image.GetPixel(0, h);
					CPixelRGB8* c_line=image.GetPixel(0, image.GetHeight()-h-1); // flip_y

					//std::cout << y <<"/"<<image.GetHeight()<<std::endl;
					for(size_t i = 0; i < width2; i++)
					{
						*pDest++ = c_line->R; // R
						*pDest++ = c_line->G ; // G
						*pDest++ = c_line->B; // B
						*pDest++ = 255; // A
						c_line+=skip;
					}

				}
			}
		}
        origImage.generateMipmaps( true );
		texture = textureManager->createTexture(
				Ogre::String(texturename),
				Ogre::GpuPageOutStrategy::SaveToSystemRam,
				Ogre::TextureFlags::ManualTexture ,
				Ogre::TextureTypes::Type2DArray );
		//texture->setPixelFormat( Ogre::PFG_RGBA8_UINT );
        texture->setPixelFormat( Ogre::PixelFormatGpuUtils::
                                getEquivalentSRGB( 
                                 origImage.getPixelFormat() ) );
		texture->setNumMipmaps(origImage.getNumMipmaps());
		texture->setResolution( (Ogre::uint32) width2, (Ogre::uint32) width2 );

        texture->scheduleTransitionTo( Ogre::GpuResidency::Resident );

		//printf("%d \n", texture->getDepthOrSlices());

        origImage.uploadTo( texture, 0, origImage.getNumMipmaps() - 1u, 0u );
	}
	else
	{
		printf("warning! not updating texture %s\n", texturename);
		//texture=textureMgr->
		//width2=texture->getWidth();
		return;
	}
#endif
}
// Create the texture
void OgreRenderer::createDynamicTexture(const char* name, CImage const& image)
{
#ifndef NO_OGRE
	std::ostringstream nameStream;
	nameStream << name << "Texture";
	std::string nameString=nameStream.str();
	const char* texturename=nameString.c_str();
	 
	_updateDynamicTexture( texturename,  image)	;

	/*
	// Create a material using the texture
	// change bygth
	//	const char materialName[]=		"DynamicTextureMaterial"; // name
	const char* materialName=name; // name
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(materialName);
	
	if(material.isNull())
	{
		// I hardcoded material "DynamicTextureMaterial" in the CheckBoard.material so the below code won't execute unless you delete the material.
		// change bygth
		material=Ogre::MaterialManager::getSingleton().create(
				materialName,
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(texturename);
		// todo2 material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	}
	*/


#endif
}
void OgreRenderer::createDynamicTexture(const char* name, CImage const& image, vector3 const& diffuseColor, vector3 const& specularColor, double shininess)
{
#ifndef NO_OGRE
	std::ostringstream nameStream;
	nameStream << name << "Texture"<<image.GetWidth();
	std::string nameString=nameStream.str();
	const char* texturename=nameString.c_str();
	_updateDynamicTexture(texturename, image);
	 
	Ogre::HlmsManager *hlmsManager = getRoot()->getHlmsManager();


	auto* datablock0=hlmsManager->getDatablockNoDefault(name);
	if(datablock0)
	{
		//printf("hihi %s\n", name);
		return;
	}
	else
	{
		//printf("creating %s\n", name);
	}

	assert( dynamic_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms( Ogre::HLMS_PBS ) ) );
	Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms(Ogre::HLMS_PBS) );

	auto macroblock=Ogre::HlmsMacroblock();

	if (image.getOpacityMap())
		macroblock.mDepthWrite=false;


	Ogre::HlmsPbsDatablock *datablock = static_cast<Ogre::HlmsPbsDatablock*>(
			hlmsPbs->createDatablock( name,
				name,
				macroblock,
				Ogre::HlmsBlendblock(),
				Ogre::HlmsParamVec() ) );


        
	//datablock->setTexture( Ogre::PBSM_DIFFUSE, "BlueCircle.png" );
	datablock->setTexture( Ogre::PBSM_DIFFUSE, texturename );
	datablock->setTexture( Ogre::PBSM_SPECULAR, texturename );
	if (image.getOpacityMap())
		datablock->setTransparency(1);
	datablock->setDiffuse( ToOgre(diffuseColor));
	datablock->setSpecular( ToOgre(specularColor));
	//shiness= 100/(100 * roughness + 0.01)
	double roughness=(100.0/shininess-0.01)/20;
	
	datablock->setRoughness( roughness);
#endif
}
void OgreRenderer::_linkMaterialAndTexture(const char* materialName, const char* textureName)
{
#ifndef NO_OGRE
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(materialName);
	
	if(material.isNull())
	{
		printf("%s not found\n", materialName);
	}
	else
	{
		material->getTechnique(0)->getPass(0)->removeTextureUnitState(0);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(textureName);
	}
#endif
}
void OgreRenderer::createMaterial(const char* name, vector3 const& diffuseColor, vector3 const& specularColor, double shininess)
{
#ifndef NO_OGRE
	/*
	 * legacy material
	// Create a material that doesn't use any texture
	const char* materialName=name; // name
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(materialName);
	
	if(material.isNull())
	{
		material=Ogre::MaterialManager::getSingleton().create(
				materialName,
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->setDiffuse(diffuseColor.x, diffuseColor.y, diffuseColor.z, 1.0);
		material->getTechnique(0)->setSpecular(specularColor.x, specularColor.y, specularColor.z, 1.0);
		material->getTechnique(0)->setShininess(shininess); // overall better than the default value 1.0
	}
	*/
	Ogre::HlmsManager *hlmsManager = getRoot()->getHlmsManager();

	assert( dynamic_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms( Ogre::HLMS_PBS ) ) );

	Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms(Ogre::HLMS_PBS) );

	auto* datablock0=hlmsManager->getDatablockNoDefault(name);
	if(datablock0)
	{
		return;
	}
	Ogre::HlmsPbsDatablock *datablock = static_cast<Ogre::HlmsPbsDatablock*>(
			hlmsPbs->createDatablock( name,
				name,
				Ogre::HlmsMacroblock(),
				Ogre::HlmsBlendblock(),
				Ogre::HlmsParamVec() ) );

	/*
	Ogre::TextureGpu *texture = textureMgr->createOrRetrieveTexture(
			"SaintPetersBasilica.dds",
			Ogre::GpuPageOutStrategy::Discard,
			Ogre::TextureFlags::PrefersLoadingFromFileAsSRGB,
			Ogre::TextureTypes::TypeCube,
			Ogre::ResourceGroupManager::
			AUTODETECT_RESOURCE_GROUP_NAME,
			Ogre::TextureFilter::TypeGenerateDefaultMipmaps );

	datablock->setTexture( Ogre::PBSM_REFLECTION, texture );
	*/
	datablock->setDiffuse( ToOgre(diffuseColor));
	datablock->setSpecular( ToOgre(specularColor));
	//shiness= 100/(100 * roughness + 0.01)
	double roughness=(100.0/shininess-0.01)/30.0;
	
	datablock->setRoughness( roughness);
#endif
}
/*
Ogre::TexturePtr renderTexture = Ogre::TextureManager::getSingleton().createManual(MY_TEXTURE, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, gameManager.getWindow()->getWidth(), gameManager.getWindow()->getHeight(), 0, Ogre::PF_A8R8G8B8, Ogre::TU_DYNAMIC_WRITE_ONLY);

Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(MY_MATERIAL, GENERAL_RESOURCE_GROUP);
material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
material->getTechnique(0)->getPass(0)->createTextureUnitState(MY_TEXTURE);
material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
*/
void OgreRenderer:: createRenderTexture(const char* type, int width, int height, bool useCurrentViewport, const char* name)
{
#ifndef NO_OGRE
	/** todo2
	Ogre::Camera* pCamera=NULL;

	if (useCurrentViewport)
		pCamera=viewport().mCam;
	Ogre::TexturePtr depthTexture ;
	// Create the depth render texture
	if (TString(type)=="R8G8B8") 
		depthTexture = Ogre::TextureManager::getSingleton().createManual(
				name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
				Ogre::TEX_TYPE_2D, width, height,
				0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);
	else
		depthTexture = Ogre::TextureManager::getSingleton().createManual(
				name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
				Ogre::TEX_TYPE_2D, width, height,
				0, Ogre::PF_FLOAT16_R, Ogre::TU_RENDERTARGET);

	// Get its render target and add a viewport to it
	Ogre::RenderTarget* depthTarget = depthTexture->getBuffer()->getRenderTarget();
	Ogre::Viewport* depthViewport;
	if(useCurrentViewport)
		depthViewport = depthTarget->addViewport(pCamera, 40);
	else
	{
		mViewports.resize(mViewports.size()+1);
		mViewports.back()=new Viewport();
		vectorn param(3);
		param(0)=width;
		param(1)=height;
		param(2)=2; // scene manager name
		mViewports[mViewports.size()-1]->init(*this,param);
		pCamera=mViewports[mViewports.size()-1]->mCam;
		depthViewport = depthTarget->addViewport(pCamera, 40);
		mViewports[mViewports.size()-1]->mView=depthViewport;
	}

	depthViewport->setBackgroundColour(Ogre::ColourValue::Black);

	// Register 'this' as a render target listener
	//depthTarget->addListener(this);

	//// I will do the followings manually in lua scripts.
	//// Get the technique to use when rendering the depth render texture
	//MaterialPtr mDepthMaterial = MaterialManager::getSingleton().getByName("DepthMap");
	//mDepthMaterial->load(); // needs to be loaded manually
	////mDepthTechnique = mDepthMaterial->getBestTechnique();
	//// Create a custom render queue invocation sequence for the depth render texture
	//RenderQueueInvocationSequence* invocationSequence =
	//	Root::getSingleton().createRenderQueueInvocationSequence("DepthMap");
	//// Add a render queue invocation to the sequence, and disable shadows for it
	//RenderQueueInvocation* invocation = invocationSequence->add(RENDER_QUEUE_MAIN, "main");
	//invocation->setSuppressShadows(true);
	//// Set the render queue invocation sequence for the depth render texture viewport
	//depthViewport->setRenderQueueInvocationSequenceName("DepthMap");
	*/
#endif
}
void OgreRenderer::updateRenderTexture(const char* param)
{
#ifndef NO_OGRE
	/* todo2
	Ogre::TexturePtr texPtr = Ogre::TextureManager::getSingleton().getByName(param);
	Msg::verify(!texPtr.isNull(), "ninjadepthmap==null");
	Ogre::RenderTexture* depth_map = texPtr->getBuffer()->getRenderTarget();
	mViewports[1]->changeView(*mViewports[1]->m_pViewpoint);
	depth_map->update();
	*/
#endif
}
void OgreRenderer::setMaterialParam(const char* mat, const char* paramName, double param_value)
{
#ifndef NO_OGRE
	Ogre::MaterialPtr mDepthMaterial = Ogre::MaterialManager::getSingleton().getByName(mat);
	//mDepthMaterial->getTechnique(0)->getPass(0)->getVertexProgramParameters()->setNamedConstant(paramName, param_value);
	mDepthMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant(paramName, Ogre::Real(param_value));
#endif
}
void OgreRenderer::cloneMaterial(const char* mat, const char* newMat)
{
#ifndef NO_OGRE
	Ogre::MaterialPtr baseMaterial = Ogre::MaterialManager::getSingleton().getByName(mat);
	Msg::verify(!baseMaterial.isNull(), "%s not found",  mat);
    baseMaterial->clone(newMat);
#endif
}

void OgreRenderer::createInputSystems(size_t hWnd)
{
#ifndef NO_OIS
				//mInputSystem = OIS::InputManager::createInputSystem(hWnd);

				std::ostringstream windowHndStr;
				OIS::ParamList pl;

				windowHndStr << hWnd;
				pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

#if defined OIS_WIN32_PLATFORM
				pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND" )));
				pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")));
				pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_FOREGROUND")));
				pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_NONEXCLUSIVE")));
#elif defined OIS_LINUX_PLATFORM
				pl.insert(std::make_pair(std::string("x11_mouse_grab"), std::string("false")));
				pl.insert(std::make_pair(std::string("x11_mouse_hide"), std::string("false")));
				pl.insert(std::make_pair(std::string("x11_keyboard_grab"), std::string("false")));
				//pl.insert(std::make_pair(std::string("XAutoRepeatOn"), std::string("true")));
    
#endif

				mInputSystem = OIS::InputManager::createInputSystem( pl );


				if (mInputSystem->getNumberOfDevices(OIS::OISKeyboard) > 0) {
					mKeyboard = static_cast<OIS::Keyboard*>( mInputSystem->createInputObject( OIS::OISKeyboard, true ) );
					printf("keyboard created\n");
				}

				if (mInputSystem->getNumberOfDevices(OIS::OISMouse) > 0) {
					mMouse = static_cast<OIS::Mouse*>( mInputSystem->createInputObject( OIS::OISMouse, true ) );
					printf("mouse created\n");
					// Get window size
					unsigned int _width, _height, depth;
					int left, top;
					mWnd->getMetrics( _width, _height, left, top );
					int width=_width;
					int height=_height;

					// Set mouse region
					//this->setWindowExtents( width, height );
					const OIS::MouseState &mouseState = mMouse->getMouseState();
					mouseState.width  = width;
					mouseState.height = height;
				}
#ifdef __APPLE__
				CGDisplayShowCursor(kCGDirectMainDisplay);
				CGAssociateMouseAndMouseCursorPosition(TRUE);
				Ogre::Rect r = getWindowBounds((void*)hWnd);
				CGDisplayMoveCursorToPoint(kCGDirectMainDisplay, CGPointMake(r.left + r.right / 2, r.top - r.bottom / 2));
#endif
#endif
}




#if defined(__APPLE__) && !defined(NO_GUI)
void setMacRenderConfig( void* handle, Ogre::NameValuePairList &misc);
#endif
void OgreRenderer::createRenderWindow(void* handle, int width, int height)
{

#ifndef NO_OGRE

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
	{
		Ogre::Vector2 screenRes = iOSUtils::getScreenResolutionInPoints();
		width = static_cast<int>( screenRes.x );
		height = static_cast<int>( screenRes.y );
	}
#endif

	Ogre::ConfigOptionMap& cfgOpts = mRoot->getRenderSystem()->getConfigOptions();
	Ogre::ConfigOptionMap::iterator opt = cfgOpts.find( "Video Mode" );
	if( opt != cfgOpts.end() && !opt->second.currentValue.empty() )
	{
		//Ignore leading space
		const Ogre::String::size_type start = opt->second.currentValue.find_first_of("012356789");
		//Get the width and height
		Ogre::String::size_type widthEnd = opt->second.currentValue.find(' ', start);
		// we know that the height starts 3 characters after the width and goes until the next space
		Ogre::String::size_type heightEnd = opt->second.currentValue.find(' ', widthEnd+3);
		// Now we can parse out the values
		width   = Ogre::StringConverter::parseInt( opt->second.currentValue.substr( 0, widthEnd ) );
		height  = Ogre::StringConverter::parseInt( opt->second.currentValue.substr(
					widthEnd+3, heightEnd ) );
	}

	Ogre::NameValuePairList params;
	Ogre::String winHandle;
#ifdef _MSC_VER
	params["externalWindowHandle"] = Ogre::StringConverter::toString((size_t)handle); 
	mWnd = mRoot->createRenderWindow("My sub render window", width, height, false, &misc); 
#else
#if defined(__APPLE__)
	//misc["currentGLContext"]=Ogre::String("True");  
		winHandle  = Ogre::StringConverter::toString(WindowContentViewHandleFltk(handle));
	
#else
	{
		/*
		Ogre::StringVector paramVector;
		paramVector.push_back(Ogre::StringConverter::toString(fl_display));
		paramVector.push_back(Ogre::StringConverter::toString(fl_screen));
		paramVector.push_back(Ogre::StringConverter::toString(fl_xid((Fl_Window*)handle)));
		paramVector.push_back(Ogre::StringConverter::toString(fl_visual));
		// params["externalWindowHandle"] =  StringConverter::toString(reinterpret_cast<unsigned long>(xDisplay)) + ":" +
		//                      StringConverter::toString(static_cast<unsigned int>(xScreenNumber)) + ":" +
		//                      StringConverter::toString(static_cast<unsigned long>(xWindow)) + ":" +
		//                     StringConverter::toString(reinterpret_cast<unsigned long>(&info));
		//misc["parentWindowHandle"]=Ogre::StringConverter::toString (paramVector);
		misc["externalWindowHandle"]=Ogre::StringConverter::toString (paramVector);
		*/

		winHandle=Ogre::StringConverter::toString(fl_xid((Fl_Window*)handle));

		XSync(fl_display, False);
	}
#endif
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WINRT
	params.insert( std::make_pair("externalWindowHandle",  winHandle) );
#else
	params.insert( std::make_pair("parentWindowHandle",  winHandle) );
#endif

	params.insert( std::make_pair("title", "win title") );
	params.insert( std::make_pair("gamma", cfgOpts["sRGB Gamma Conversion"].currentValue) );
	if( cfgOpts.find( "VSync Method" ) != cfgOpts.end() )
		params.insert( std::make_pair( "vsync_method", cfgOpts["VSync Method"].currentValue ) );
	params.insert( std::make_pair("FSAA", cfgOpts["FSAA"].currentValue) );
	params.insert( std::make_pair("vsync", cfgOpts["VSync"].currentValue) );
	params.insert( std::make_pair("reverse_depth", "Yes" ) );

	initMiscParamsListener( params );

	mWnd = mRoot->createRenderWindow("My sub render window", width, height, false, &params); 
#endif // not _MSC_VER
#endif
}
void OgreRenderer::ChageTextureImage(const char* name, const char* textureName)
{
#ifndef NO_OGRE
	const char* materialName=name; // name
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(materialName);
	material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(textureName);
#endif
}
#ifndef NO_OGRE
void OgreRenderer::createPcfShadowNode(void)
{
	Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
	Ogre::RenderSystem *renderSystem = mRoot->getRenderSystem();

	Ogre::ShadowNodeHelper::ShadowParamVec shadowParams;

	Ogre::ShadowNodeHelper::ShadowParam shadowParam;
	memset( &shadowParam, 0, sizeof(shadowParam) );

	//First light, directional
	shadowParam.technique = Ogre::SHADOWMAP_PSSM;
	shadowParam.numPssmSplits = 3u;
	shadowParam.resolution[0].x = 2048u;
	shadowParam.resolution[0].y = 2048u;
	for( size_t i=1u; i<4u; ++i )
	{
		shadowParam.resolution[i].x = 1024u;
		shadowParam.resolution[i].y = 1024u;
	}
	shadowParam.atlasStart[0].x = 0u;
	shadowParam.atlasStart[0].y = 0u;
	shadowParam.atlasStart[1].x = 0u;
	shadowParam.atlasStart[1].y = 2048u;
	shadowParam.atlasStart[2].x = 1024u;
	shadowParam.atlasStart[2].y = 2048u;

	shadowParam.supportedLightTypes = 0u;
	shadowParam.addLightType( Ogre::Light::LT_DIRECTIONAL );
	shadowParams.push_back( shadowParam );

	//Second light, directional, spot or point
	shadowParam.technique = Ogre::SHADOWMAP_FOCUSED;
	shadowParam.resolution[0].x = 2048u;
	shadowParam.resolution[0].y = 2048u;
	shadowParam.atlasStart[0].x = 0u;
	shadowParam.atlasStart[0].y = 2048u + 1024u;

	shadowParam.supportedLightTypes = 0u;
	shadowParam.addLightType( Ogre::Light::LT_DIRECTIONAL );
	shadowParam.addLightType( Ogre::Light::LT_POINT );
	shadowParam.addLightType( Ogre::Light::LT_SPOTLIGHT );
	shadowParams.push_back( shadowParam );

	//Third light, directional, spot or point
	shadowParam.atlasStart[0].y = 2048u + 1024u + 2048u;
	shadowParams.push_back( shadowParam );

	Ogre::ShadowNodeHelper::createShadowNodeWithSettings( compositorManager,
			renderSystem->getCapabilities(),
			"ShadowMapFromCodeShadowNode",
			shadowParams, false );
}

void OgreRenderer::createEsmShadowNodes(void)
{
	Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
	Ogre::RenderSystem *renderSystem = mRoot->getRenderSystem();

	Ogre::ShadowNodeHelper::ShadowParamVec shadowParams;

	Ogre::ShadowNodeHelper::ShadowParam shadowParam;
	memset( &shadowParam, 0, sizeof(shadowParam) );

	//First light, directional
	shadowParam.technique = Ogre::SHADOWMAP_PSSM;
	shadowParam.numPssmSplits = 3u;
	shadowParam.resolution[0].x = 1024u;
	shadowParam.resolution[0].y = 1024u;
	shadowParam.resolution[1].x = 2048u;
	shadowParam.resolution[1].y = 2048u;
	shadowParam.resolution[2].x = 1024u;
	shadowParam.resolution[2].y = 1024u;
	shadowParam.atlasStart[0].x = 0u;
	shadowParam.atlasStart[0].y = 0u;
	shadowParam.atlasStart[1].x = 0u;
	shadowParam.atlasStart[1].y = 1024u;
	shadowParam.atlasStart[2].x = 1024u;
	shadowParam.atlasStart[2].y = 0u;

	shadowParam.supportedLightTypes = 0u;
	shadowParam.addLightType( Ogre::Light::LT_DIRECTIONAL );
	shadowParams.push_back( shadowParam );

	//Second light, directional, spot or point
	shadowParam.technique = Ogre::SHADOWMAP_FOCUSED;
	shadowParam.resolution[0].x = 1024u;
	shadowParam.resolution[0].y = 1024u;
	shadowParam.atlasStart[0].x = 0u;
	shadowParam.atlasStart[0].y = 2048u + 1024u;

	shadowParam.supportedLightTypes = 0u;
	shadowParam.addLightType( Ogre::Light::LT_DIRECTIONAL );
	shadowParam.addLightType( Ogre::Light::LT_POINT );
	shadowParam.addLightType( Ogre::Light::LT_SPOTLIGHT );
	shadowParams.push_back( shadowParam );

	//Third light, directional, spot or point
	shadowParam.atlasStart[0].x = 1024u;
	shadowParams.push_back( shadowParam );

	const Ogre::RenderSystemCapabilities *capabilities = renderSystem->getCapabilities();
	Ogre::RenderSystemCapabilities capsCopy = *capabilities;

	//Force the utility to create ESM shadow node with compute filters.
	//Otherwise it'd create using what's supported by the current GPU.
	capsCopy.setCapability( Ogre::RSC_COMPUTE_PROGRAM );
	Ogre::ShadowNodeHelper::createShadowNodeWithSettings(
			compositorManager, &capsCopy,
			"ShadowMapFromCodeEsmShadowNodeCompute",
			shadowParams, true );

	//Force the utility to create ESM shadow node with graphics filters.
	//Otherwise it'd create using what's supported by the current GPU.
	capsCopy.unsetCapability( Ogre::RSC_COMPUTE_PROGRAM );
	Ogre::ShadowNodeHelper::createShadowNodeWithSettings(
			compositorManager, &capsCopy,
			"ShadowMapFromCodeEsmShadowNodePixelShader",
			shadowParams, true );
}
#endif

void OgreRenderer::setrotate(m_real degree, const char* name)
{
#ifndef NO_OGRE
	const char* materialName=name; // name
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(materialName);
	material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureRotate(Ogre::Radian(Ogre::Degree(degree))); 
#endif

}




#ifndef NO_OGRE
Ogre::CompositorWorkspace* OgreRenderer::setupCompositor(void)
{
	Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
		const Ogre::String workspaceName( "ShadowMapFromCodeWorkspace" );

		if( !compositorManager->hasWorkspaceDefinition( workspaceName ) )
		{
			compositorManager->createBasicWorkspaceDef( workspaceName, mBackgroundColour,
					Ogre::IdString() );

			const Ogre::String nodeDefName = "AutoGen " +
				Ogre::IdString(workspaceName +
						"/Node").getReleaseText();
			Ogre::CompositorNodeDef *nodeDef =
				compositorManager->getNodeDefinitionNonConst( nodeDefName );

			Ogre::CompositorTargetDef *targetDef = nodeDef->getTargetPass( 0 );
			const Ogre::CompositorPassDefVec &passes = targetDef->getCompositorPasses();

			assert( dynamic_cast<Ogre::CompositorPassSceneDef*>( passes[0] ) );
			Ogre::CompositorPassSceneDef *passSceneDef =
				static_cast<Ogre::CompositorPassSceneDef*>( passes[0] );
			passSceneDef->mShadowNode = "ShadowMapFromCodeShadowNode";

			createPcfShadowNode();
			createEsmShadowNodes();
		}

		mWorkspace = compositorManager->addWorkspace( getSceneManager(), mWnd->getTexture(),
				getCamera(), "ShadowMapFromCodeWorkspace", true );
		return mWorkspace;
}


void OgreRenderer::loadTextureCache(void)
{
#if !OGRE_NO_JSON
	Ogre::ArchiveManager &archiveManager = Ogre::ArchiveManager::getSingleton();
	Ogre::Archive *rwAccessFolderArchive = archiveManager.load( mWriteAccessFolder,
			"FileSystem", true );
	try
	{
		const Ogre::String filename = "textureMetadataCache.json";
		if( rwAccessFolderArchive->exists( filename ) )
		{
			Ogre::DataStreamPtr stream = rwAccessFolderArchive->open( filename );
			std::vector<char> fileData;
			fileData.resize( stream->size() + 1 );
			if( !fileData.empty() )
			{
				stream->read( &fileData[0], stream->size() );
				//Add null terminator just in case (to prevent bad input)
				fileData.back() = '\0';
				Ogre::TextureGpuManager *textureManager =
					mRoot->getRenderSystem()->getTextureGpuManager();
				textureManager->importTextureMetadataCache( stream->getName(), &fileData[0], false );
			}
		}
		else
		{
			Ogre::LogManager::getSingleton().logMessage(
					"[INFO] Texture cache not found at " + mWriteAccessFolder +
					"/textureMetadataCache.json" );
		}
	}
	catch( Ogre::Exception &e )
	{
		Ogre::LogManager::getSingleton().logMessage( e.getFullDescription() );
	}

	archiveManager.unload( rwAccessFolderArchive );
#endif
}
//-----------------------------------------------------------------------------------
void OgreRenderer::saveTextureCache(void)
{
	if( mRoot->getRenderSystem() )
	{
		Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();
		if( textureManager )
		{
			Ogre::String jsonString;
			textureManager->exportTextureMetadataCache( jsonString );
			const Ogre::String path = mWriteAccessFolder + "/textureMetadataCache.json";
			std::ofstream file( path.c_str(), std::ios::binary | std::ios::out );
			if( file.is_open() )
				file.write( jsonString.c_str(), static_cast<std::streamsize>( jsonString.size() ) );
			file.close();
		}
	}
}
//-----------------------------------------------------------------------------------
void OgreRenderer::loadHlmsDiskCache(void)
{
	if( !mUseMicrocodeCache && !mUseHlmsDiskCache )
		return;

	Ogre::HlmsManager *hlmsManager = mRoot->getHlmsManager();
	Ogre::HlmsDiskCache diskCache( hlmsManager );

	Ogre::ArchiveManager &archiveManager = Ogre::ArchiveManager::getSingleton();

	Ogre::Archive *rwAccessFolderArchive = archiveManager.load( mWriteAccessFolder,
			"FileSystem", true );

	if( mUseMicrocodeCache )
	{
		//Make sure the microcode cache is enabled.
		Ogre::GpuProgramManager::getSingleton().setSaveMicrocodesToCache( true );
		const Ogre::String filename = "microcodeCodeCache.cache";
		if( rwAccessFolderArchive->exists( filename ) )
		{
			Ogre::DataStreamPtr shaderCacheFile = rwAccessFolderArchive->open( filename );
			Ogre::GpuProgramManager::getSingleton().loadMicrocodeCache( shaderCacheFile );
		}
	}

	if( mUseHlmsDiskCache )
	{
		for( size_t i=Ogre::HLMS_LOW_LEVEL + 1u; i<Ogre::HLMS_MAX; ++i )
		{
			Ogre::Hlms *hlms = hlmsManager->getHlms( static_cast<Ogre::HlmsTypes>( i ) );
			if( hlms )
			{
				Ogre::String filename = "hlmsDiskCache" +
					Ogre::StringConverter::toString( i ) + ".bin";

				try
				{
					if( rwAccessFolderArchive->exists( filename ) )
					{
						Ogre::DataStreamPtr diskCacheFile = rwAccessFolderArchive->open( filename );
						diskCache.loadFrom( diskCacheFile );
						diskCache.applyTo( hlms );
					}
				}
				catch( Ogre::Exception& )
				{
					Ogre::LogManager::getSingleton().logMessage(
							"Error loading cache from " + mWriteAccessFolder + "/" +
							filename + "! If you have issues, try deleting the file "
							"and restarting the app" );
				}
			}
		}
	}

	archiveManager.unload( mWriteAccessFolder );
}
//-----------------------------------------------------------------------------------
void OgreRenderer::saveHlmsDiskCache(void)
{
	if( mRoot->getRenderSystem() && Ogre::GpuProgramManager::getSingletonPtr() &&
			(mUseMicrocodeCache || mUseHlmsDiskCache) )
	{
		Ogre::HlmsManager *hlmsManager = mRoot->getHlmsManager();
		Ogre::HlmsDiskCache diskCache( hlmsManager );

		Ogre::ArchiveManager &archiveManager = Ogre::ArchiveManager::getSingleton();

		Ogre::Archive *rwAccessFolderArchive = archiveManager.load( mWriteAccessFolder,
				"FileSystem", false );

		if( mUseHlmsDiskCache )
		{
			for( size_t i=Ogre::HLMS_LOW_LEVEL + 1u; i<Ogre::HLMS_MAX; ++i )
			{
				Ogre::Hlms *hlms = hlmsManager->getHlms( static_cast<Ogre::HlmsTypes>( i ) );
				if( hlms )
				{
					diskCache.copyFrom( hlms );

					Ogre::DataStreamPtr diskCacheFile =
						rwAccessFolderArchive->create( "hlmsDiskCache" +
								Ogre::StringConverter::toString( i ) +
								".bin" );
					diskCache.saveTo( diskCacheFile );
				}
			}
		}

		if( Ogre::GpuProgramManager::getSingleton().isCacheDirty() && mUseMicrocodeCache )
		{
			const Ogre::String filename = "microcodeCodeCache.cache";
			Ogre::DataStreamPtr shaderCacheFile = rwAccessFolderArchive->create( filename );
			Ogre::GpuProgramManager::getSingleton().saveMicrocodeCache( shaderCacheFile );
		}

		archiveManager.unload( mWriteAccessFolder );
	}
}
void OgreRenderer::stopCompositor(void)
{
	if( mWorkspace )
	{
		Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
		compositorManager->removeWorkspace( mWorkspace );
		mWorkspace = 0;
	}
}
void OgreRenderer::restartCompositor(void)
{
	stopCompositor();
	mWorkspace = setupCompositor();
}
void OgreRenderer::registerHlms(void)
{
	Ogre::ConfigFile cf;
	cf.load(  mResourceFile.ptr() ) ;

//#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE || OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
	Ogre::String rootHlmsFolder = Ogre::macBundlePath() + '/' +
		cf.getSetting( "DoNotUseAsResource", "Hlms", "" );
#else
	std::string mTaesooLib_path=RE::taesooLibPath();
	std::string mResourcePath=mTaesooLib_path+"Resource/";
	Ogre::String rootHlmsFolder = mResourcePath + cf.getSetting( "DoNotUseAsResource", "Hlms", "" );
#endif

	if( rootHlmsFolder.empty() )
		rootHlmsFolder =  "./";
	else if( *(rootHlmsFolder.end() - 1) != '/' )
		rootHlmsFolder += "/";

	//At this point rootHlmsFolder should be a valid path to the Hlms data folder

	Ogre::HlmsUnlit *hlmsUnlit = 0;
	Ogre::HlmsPbs *hlmsPbs = 0;

	//For retrieval of the paths to the different folders needed
	Ogre::String mainFolderPath;
	Ogre::StringVector libraryFoldersPaths;
	Ogre::StringVector::const_iterator libraryFolderPathIt;
	Ogre::StringVector::const_iterator libraryFolderPathEn;

	Ogre::ArchiveManager &archiveManager = Ogre::ArchiveManager::getSingleton();

	//const Ogre::String &archiveType = getMediaReadArchiveType();
	const Ogre::String archiveType = "FileSystem";

	{
		//Create & Register HlmsUnlit
		//Get the path to all the subdirectories used by HlmsUnlit
		Ogre::HlmsUnlit::getDefaultPaths( mainFolderPath, libraryFoldersPaths );
		Ogre::Archive *archiveUnlit = archiveManager.load( rootHlmsFolder + mainFolderPath,
				archiveType, true );
		Ogre::ArchiveVec archiveUnlitLibraryFolders;
		libraryFolderPathIt = libraryFoldersPaths.begin();
		libraryFolderPathEn = libraryFoldersPaths.end();
		while( libraryFolderPathIt != libraryFolderPathEn )
		{
			Ogre::Archive *archiveLibrary =
				archiveManager.load( rootHlmsFolder + *libraryFolderPathIt, archiveType, true );
			archiveUnlitLibraryFolders.push_back( archiveLibrary );
			++libraryFolderPathIt;
		}

		//Create and register the unlit Hlms
		hlmsUnlit = OGRE_NEW Ogre::HlmsUnlit( archiveUnlit, &archiveUnlitLibraryFolders );
		Ogre::Root::getSingleton().getHlmsManager()->registerHlms( hlmsUnlit );
	}

	{
		//Create & Register HlmsPbs
		//Do the same for HlmsPbs:
		Ogre::HlmsPbs::getDefaultPaths( mainFolderPath, libraryFoldersPaths );
		Ogre::Archive *archivePbs = archiveManager.load( rootHlmsFolder + mainFolderPath,
				archiveType, true );

		//Get the library archive(s)
		Ogre::ArchiveVec archivePbsLibraryFolders;
		libraryFolderPathIt = libraryFoldersPaths.begin();
		libraryFolderPathEn = libraryFoldersPaths.end();
		while( libraryFolderPathIt != libraryFolderPathEn )
		{
			Ogre::Archive *archiveLibrary =
				archiveManager.load( rootHlmsFolder + *libraryFolderPathIt, archiveType, true );
			archivePbsLibraryFolders.push_back( archiveLibrary );
			++libraryFolderPathIt;
		}

		//Create and register
		hlmsPbs = OGRE_NEW Ogre::HlmsPbs( archivePbs, &archivePbsLibraryFolders );
		Ogre::Root::getSingleton().getHlmsManager()->registerHlms( hlmsPbs );
	}


	Ogre::RenderSystem *renderSystem = mRoot->getRenderSystem();
	if( renderSystem->getName() == "Direct3D11 Rendering Subsystem" )
	{
		//Set lower limits 512kb instead of the default 4MB per Hlms in D3D 11.0
		//and below to avoid saturating AMD's discard limit (8MB) or
		//saturate the PCIE bus in some low end machines.
		bool supportsNoOverwriteOnTextureBuffers;
		renderSystem->getCustomAttribute( "MapNoOverwriteOnDynamicBufferSRV",
				&supportsNoOverwriteOnTextureBuffers );

		if( !supportsNoOverwriteOnTextureBuffers )
		{
			hlmsPbs->setTextureBufferDefaultSize( 512 * 1024 );
			hlmsUnlit->setTextureBufferDefaultSize( 512 * 1024 );
		}
	}
}
void OgreRenderer::initMiscParamsListener( Ogre::NameValuePairList &params )
{
}
void OgreRenderer::setupShadowNode(bool useESM)
{
	// use ESM. (high quality)
	Ogre::Hlms *hlms = mRoot->getHlmsManager()->getHlms( Ogre::HLMS_PBS );
	OGRE_ASSERT_HIGH( dynamic_cast<Ogre::HlmsPbs*>( hlms ) );
	Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlms );
	printf("current filter %d\n", hlmsPbs->getShadowFilter());
	if(useESM)
	{
		Ogre::HlmsPbs::ShadowFilter newfilter=  Ogre::HlmsPbs::ExponentialShadowMaps ;
		hlmsPbs->setShadowSettings( newfilter );
		((ShadowMapFromCodeGameState*)mCurrentGameState)->setupShadowNode( true );
	}
	else
	{
		//Ogre::HlmsPbs::ShadowFilter newfilter=  Ogre::HlmsPbs:: PCF_3x3;
		Ogre::HlmsPbs::ShadowFilter newfilter=  Ogre::HlmsPbs:: PCF_6x6;
		hlmsPbs->setShadowSettings( newfilter );
		((ShadowMapFromCodeGameState*)mCurrentGameState)->setupShadowNode( false );
	}

}
#endif

void OgreRenderer::_toggleHelpMode()
{
#ifndef NO_OGRE
	// called from MotionPanel.
	ShadowMapFromCodeGameState* out=(ShadowMapFromCodeGameState*)mCurrentGameState;
    out->mDisplayHelpMode = (out->mDisplayHelpMode + 1) % out->mNumDisplayHelpModes;
	printf("%d\n", out->mDisplayHelpMode );
	_updateDebugCaption();
#endif
}

#ifndef NO_OGRE
#include "OgreTextAreaOverlayElement.h"
#endif
void OgreRenderer::_updateDebugCaption()
{
#ifndef NO_OGRE
	ShadowMapFromCodeGameState* out=(ShadowMapFromCodeGameState*)mCurrentGameState;
	Ogre::String finalText;
	out->generateDebugText( 0, finalText );
	out->mDebugText->setCaption( finalText );
	out->mDebugTextShadow->setCaption( finalText );
#endif
}
