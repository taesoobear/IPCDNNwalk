#ifdef __APPLE__
#include <Carbon/Carbon.h>
#ifdef verify 
#undef verify
#endif
#endif
#include "stdafx.h"
#include <stdio.h>
#include "../BaseLib/utility/util.h"
#include "MotionManager.h"
#include "../BaseLib/motion/viewpoint.h"
//#include "netlab.h"
//#include "gaussianprocess/gaussianprocess.h"
#include "framemoveobject.h"
#include "FltkRenderer.h"
#include "renderer.h"
#include "../../BaseLib/utility/TypeString.h"
#include "RE.h"
#ifndef NO_OGRE
#include <Ogre.h>
#ifndef NO_OIS
 #include <OISMouse.h>
 #include <OISKeyboard.h>
 #include <OISJoyStick.h>
 #include <OISInputManager.h>
#endif
#if OGRE_VERSION_MINOR>=12 || OGRE_VERSION_MAJOR>=13
#include "Bites/OgreBitesConfigDialog.h"
#endif
#endif
using namespace std;

ConfigTable config;
#if defined( __APPLE__) && !defined(NO_GUI)
Ogre::Rect getWindowBounds(Ogre::RenderWindow *renderWindow);
#endif

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

#ifdef INCLUDE_OGRESKINENTITY
#include "../Ogre/OgreSkinEntity.h"
#endif

#ifndef NO_OGRE
#if !defined(__APPLE__) && !defined(_MSC_VER)
#include <X11/Xlib.h>
#endif
#endif
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
static Ogre::TexturePtr rtt_texture;
static Ogre::RenderTexture *renderTexture=NULL;
static bool mbUseRTTcapture=false;
static bool mbUseOGREcapture=false;
#endif

void OgreRenderer::setBackgroundColour(float r, float g, float b)
{
#ifndef NO_OGRE
	viewport().mView->setBackgroundColour(Ogre::ColourValue(r,g,b,1.f));
	if(mbUseRTTcapture){
		renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue(r,g, b, 1.f));
	}
#endif
}
void OgreRenderer::Viewport::setupRTT(OgreRenderer& renderer, int width, int height)
{
#ifndef NO_OGRE

	// Viewports.

	mView->setBackgroundColour(Ogre::ColourValue(0.f, 0.6f, 0.8f, 0.9f));

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
		rtt_texture = Ogre::TextureManager::getSingleton().createManual("RttTex", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

		renderTexture = rtt_texture->getBuffer()->getRenderTarget();

		renderTexture->addViewport(mCam);
		renderTexture->getViewport(0)->setClearEveryFrame(true);
		//renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
		renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue(0.f, 0.6f, 0.8f, 0.9f));
		renderTexture->getViewport(0)->setOverlaysEnabled(true);
		renderTexture->setAutoUpdated(true);
	}

	// Shadow
	//mScene->setShadowTechnique( Ogre::SHADOWTYPE_TEXTURE_MODULATIVE);
	Ogre::ShadowTechnique shadowTechnique=(Ogre::ShadowTechnique )config.GetInt("shadowTechnique");
	int depthShadow=config.GetInt("depthShadow")==1;

#if OGRE_VERSION_MINOR>=12 || OGRE_VERSION_MAJOR>=13
	if (depthShadow)
	{
		mScene->setShadowTexturePixelFormat(Ogre::PF_FLOAT32_R);
		shadowTechnique=Ogre::SHADOWTYPE_TEXTURE_ADDITIVE_INTEGRATED;
	}
#endif

	mScene->setShadowTextureSize(512);
	mScene->setShadowTechnique( shadowTechnique);		

	if (renderer.mRoot->getRenderSystem()->getCapabilities()->hasCapability(Ogre::RSC_HWRENDER_TO_TEXTURE))
	{
		// In D3D, use a 1024x1024 shadow texture
		//mScene->setShadowTextureSettings(1024, 2);
		//
		int count=2;
		if (config.Find("shadowmapCount"))
			count=config.GetInt("shadowmapCount");
		mScene->setShadowTextureSettings(config.GetInt("shadowResolution"),count);
		//mScene->setShadowTextureSize(config.GetInt("shadowResolution"));
	}
	else
	{
		// Use 512x512 texture in GL since we can't go higher than the window res
		mScene->setShadowTextureSettings(256, 2);
	}

	double shadowColor=0.5;
	mScene->setShadowColour(Ogre::ColourValue(shadowColor, shadowColor, shadowColor));


	if (depthShadow){
#if OGRE_VERSION_MINOR>=12 || OGRE_VERSION_MAJOR>=13
		auto mSceneMgr=mScene;
		std::string CUSTOM_ROCKWALL_MATERIAL("Ogre/DepthShadowmap/Receiver/RockWall");
		std::string CUSTOM_CASTER_MATERIAL("PSSM/shadow_caster");
		std::string CUSTOM_RECEIVER_MATERIAL("Ogre/DepthShadowmap/Receiver/Float");
		std::string CUSTOM_ATHENE_MATERIAL("Ogre/DepthShadowmap/Receiver/Athene");
		auto themat = Ogre::MaterialManager::getSingleton().getByName(CUSTOM_CASTER_MATERIAL);
		mSceneMgr->setShadowTextureCasterMaterial(themat);
		mSceneMgr->setShadowTextureSelfShadow(true);    
		// 모든 shadow receiver들도 특수 material사용해야함. see loadBG_default.lua (search "if depthShadow")
#endif

	}
#endif
}
void OgreRenderer::Viewport::init(OgreRenderer& renderer, vectorn const& param)
{
	int width=param(0);
	int height=param(1);
#ifndef NO_OGRE
	//mScene = renderer.mRoot->createSceneManager("OctreeSceneManager", "OgreFltk");
	mScene = renderer.mRoot->createSceneManager();
#if OGRE_VERSION_MINOR>=9 || OGRE_VERSION_MAJOR>=13
	mScene->addRenderQueueListener(renderer.mOverlaySystem);
#endif
#endif
	// init camera
	_init(renderer, width, height);
	if(param.size()==3)
	{
		// manually setup mView and mScene options
	}
	else // default automatic setup
	{
#ifndef NO_OGRE
		mView = renderer.mWnd->addViewport(mCam,cameraIndex-1 );
		setupRTT(renderer, width, height);
		m_pViewpoint->m_iWidth=mView->getActualWidth();
		m_pViewpoint->m_iHeight=mView->getActualHeight();
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


void OgreRenderer::Viewport::_init(OgreRenderer& renderer,int width, int height)
{
	// Taesoo Camera.
	m_pViewpoint=new Viewpoint();

#ifdef NO_OGRE
	m_pViewpoint->setDefaultView();
#else
	FILE* fp;
	VERIFY(fp=fopen((renderer.mTaesooLib_path+"Resource/viewpoint.txt").c_str(),"r"));
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

	mCam->setFarClipDistance(config.GetInt("farClipDistance"));
//	mCam->setPosition(m_pViewpoint->m_vecVPos.x, m_pViewpoint->m_vecVPos.y, m_pViewpoint->m_vecVPos.z);
#if OGRE_VERSION_MAJOR<13
	mCam->lookAt(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z);
#else
	mCameraNode=mScene->getRootSceneNode()->createChildSceneNode();
	mCameraNode->attachObject(mCam);
	mCameraNode->setFixedYawAxis(true); // fix lookAt calls
	mCameraNode->lookAt(Ogre::Vector3(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z), Ogre::Node::TS_PARENT);
#endif
	mCam->setNearClipDistance(config.GetInt("nearClipDistance"));
	mCam->setFOVy(Ogre::Radian(Ogre::Degree(45)));


#endif
}
void OgreRenderer::Viewport::setOrthographicMode(bool isOrtho)
{
#ifndef NO_OGRE
	m_real aspectRatio=(m_real)mView->getActualWidth()/(m_real)mView->getActualHeight();
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
	_init(renderer, other.m_pViewpoint->m_iWidth, other.m_pViewpoint->m_iHeight);
#ifndef NO_OGRE
	mView = renderer.mWnd->addViewport(mCam,cameraIndex-1 );
	mView->setBackgroundColour(other.mView->getBackgroundColour());
#endif
}
OgreRenderer::OgreRenderer()
	:

#if !defined( NO_OGRE) 
		mRoot(NULL),
		mWnd(NULL),
#if !defined(NO_OIS)
		mMouse(NULL),
		mKeyboard(NULL),
		mInputSystem(NULL),
#endif
#endif
		mStatsOn(true),
		mbPause(false), 
		mbScreenshot(false),
		mbFixedTimeStep(false),
		m_fTimeScaling(1.f),
		mScreenshotPrefix("../dump/dump"),
		mTaesooLib_path	("../")
{
	_locateTaesooLib();
#ifdef _MSC_VER // WINDOWS
#if defined(_DEBUG)
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig.txt").c_str(),(mPluginPath+"plugins_d.cfg").c_str(), (mPluginPath+"ogre.cfg").c_str());
#else
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig.txt").c_str(),(mPluginPath+"plugins.cfg").c_str(), (mPluginPath+"ogre.cfg").c_str());
#endif
#elif defined(__APPLE__) 
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig_mac.txt").c_str(),(mPluginPath+"plugins_mac.cfg").c_str(), (mPluginPath+"ogre_mac.cfg").c_str());
#else // LINUX

#if OGRE_VERSION_MINOR >= 12 || OGRE_VERSION_MAJOR>=13
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig_linux12.txt").c_str(), (mPluginPath+"plugins_linux12.cfg").c_str(), (mPluginPath+"ogre_linux12.cfg").c_str());
#else
	_constructor((mTaesooLib_path+"Resource/ogreconfig_personal.txt").c_str(), (mTaesooLib_path+"Resource/ogreconfig_linux.txt").c_str(), (mPluginPath+"plugins_linux.cfg").c_str(), (mPluginPath+"ogre_linux.cfg").c_str());
#endif
#endif
}


OgreRenderer::OgreRenderer(const char* fallback_configFileName, const char* configFileName, const char* plugins_file, const char* ogre_config)
	:

#if !defined( NO_OGRE)
	mRoot(NULL),
	mWnd(NULL),
#if !defined(NO_OIS)
		mMouse(NULL),
		mKeyboard(NULL),
		mInputSystem(NULL),
#endif
#endif
		mStatsOn(true),
		mbPause(false), 
		mbScreenshot(false),
		mbFixedTimeStep(false),
		m_fTimeScaling(1.f),
		mScreenshotPrefix("../dump/dump"),
		mTaesooLib_path	("../")
{
	_locateTaesooLib();
	_constructor(fallback_configFileName, configFileName, plugins_file, ogre_config);
}

void OgreRenderer::_locateTaesooLib()
{
	mTaesooLib_path=RE::taesooLibPath();
	if( mTaesooLib_path=="work/taesooLib/")
	{
		mScreenshotPrefix="work/taesooLib/dump/dump";
		mPluginPath="work/";
	}
}
void OgreRenderer::_constructor(const char* fallback_configFileName, const char* configFileName, const char* plugins_file, const char* ogre_config)
{
	printf("loading %s\n", configFileName);
#if !defined(NO_GUI)
	config.load(fallback_configFileName, configFileName);
#endif
	ASSERT(RE::g_pGlobals==NULL);
	RE::g_pGlobals=new RE::Globals();
	RE::g_pGlobals->pRenderer = this;    
	m_pMotionManager=new MotionManager((mTaesooLib_path+"Resource/motion.lua").c_str());
	m_fElapsedTime=0.f;
	m_fCaptureFPS=30.f;
	mbTimeStop=false;

	if(mTaesooLib_path=="../")
	{
#if	OGRE_VERSION_MAJOR>=13
		mResourceFile=(mTaesooLib_path+"Resource/resources13.cfg").c_str();
#elif OGRE_VERSION_MINOR>=12 
		mResourceFile=(mTaesooLib_path+"Resource/resources12.cfg").c_str();
#else
		mResourceFile=(mTaesooLib_path+"Resource/resources.cfg").c_str();
#endif
	}
	else
	{
#if	OGRE_VERSION_MAJOR>=13
		mResourceFile=(mTaesooLib_path+"Resource/resources13_relative.cfg").c_str();
#elif OGRE_VERSION_MINOR>=12 
		mResourceFile=(mTaesooLib_path+"Resource/resources12_relative.cfg").c_str();
#else
		mResourceFile=(mTaesooLib_path+"Resource/resources_relative.cfg").c_str();
#endif

	}

#ifndef NO_OGRE
#if OGRE_VERSION_MINOR>=9 || OGRE_VERSION_MAJOR>=13
	mOverlaySystem=NULL;
#endif
	// Make the root

	printf("starting ogre"); fflush(stdout);
	Ogre::Log* log=NULL;

	if(config.GetInt("enagleLog")==0)	 {
		Ogre::LogManager* logMgr=new Ogre::LogManager();
		log=Ogre::LogManager::getSingleton().createLog("", true, false, false);
	}
	printf("."); fflush(stdout);


	mRoot = new Ogre::Root(plugins_file, ogre_config, (log)?"":"Ogre.log");
	printf("."); fflush(stdout);

#if OGRE_VERSION_MINOR>=9 || OGRE_VERSION_MAJOR>=13
	mOverlaySystem=new Ogre::OverlaySystem();
#endif
	printf("."); fflush(stdout);

#ifdef INCLUDE_OGRESKINENTITY
	mRoot->addMovableObjectFactory(new Ogre::SkinEntityFactory ());
#endif
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
void OgreRenderer::firstInit(void* handle, int width, int height)
{

#ifndef NO_OGRE

	try
	{

		// Set parameters of render system (window size, etc.)

		// Show the configuration dialog and initialise the system
		// You can skip this and use root.restoreConfig() to load configuration
		// settings if you were sure there are valid ones saved in ogre.cfg
		setupResources();

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
				mWnd = mRoot->initialise(true); // create renderwindow
				size_t hWnd = 0;

				mWnd->getCustomAttribute("WINDOW", &hWnd);

				createInputSystems(hWnd);
			}
			else
				mWnd = mRoot->initialise(false);
		}
		catch(Ogre::Exception &e)
		{
			Msg::msgBox("init failed - showing configuration dialog");
#if OGRE_VERSION_MINOR>=12 || OGRE_VERSION_MAJOR>=13
			if(!mRoot->showConfigDialog( OgreBites::getNativeConfigDialog()))
#else
			if(!mRoot->showConfigDialog())
#endif
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
		vectorn param(2);
		param(0)=width;
		param(1)=height;

		mViewports.resize(mViewports.size()+1);
		mViewports.back()=new Viewport();
		mCurrViewports=mViewports.size()-1;

		// Set default mipmap level (NB some APIs ignore this)
		Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

		// Load resource
		Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();


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

		//mScene->setShowDebugShadows(true);

		// Lights.
		// light는 프로그램에서 생성하시오.

		//frame listener 장착
		mRoot->addFrameListener(this);


		// All set up, activate.
		mWnd->setActive(true);		
		mWnd->setVisible(true);

		// Do not create scene here! :TAESOO

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
#if OGRE_VERSION_MINOR>=9 || OGRE_VERSION_MAJOR>=13
			delete mOverlaySystem;
#endif
			delete mRoot;
		}
		mRoot=NULL;
		throw std::runtime_error(e.getFullDescription());
	}
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
	mView=NULL;
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
	rtt_texture.setNull();
	delete mRoot;
#endif
	delete m_pMotionManager;	
}


void OgreRenderer::Viewport::changeView(matrix4 const& matview)
{
#ifndef NO_OGRE
	m_pViewpoint->SetViewMatrix(matview);
#if OGRE_VERSION_MAJOR<13
	mCam->setPosition(m_pViewpoint->m_vecVPos.x, m_pViewpoint->m_vecVPos.y, m_pViewpoint->m_vecVPos.z);
	mCam->lookAt(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z);
#else
	mCameraNode->setPosition(m_pViewpoint->m_vecVPos.x, m_pViewpoint->m_vecVPos.y, m_pViewpoint->m_vecVPos.z);
	mCameraNode->lookAt(Ogre::Vector3(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z), Ogre::Node::TS_PARENT);
#endif
//	mCam->lookAt(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z);
#endif
}

void OgreRenderer::Viewport::changeView(Viewpoint const& view)
{
	*m_pViewpoint=view;
	m_real m_zoom;
#ifndef NO_OGRE
	//float interval = isLeft ? -config.GetFloat("interval") : config.GetInt("interval");
#if OGRE_VERSION_MAJOR<13
	mCam->setPosition(m_pViewpoint->m_vecVPos.x, m_pViewpoint->m_vecVPos.y, m_pViewpoint->m_vecVPos.z);
	mCam->lookAt(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z);
	mCam->setFixedYawAxis( true, ToOgre(m_pViewpoint->m_vecVUp));
#else
	mCameraNode->setPosition(m_pViewpoint->m_vecVPos.x, m_pViewpoint->m_vecVPos.y, m_pViewpoint->m_vecVPos.z);
	mCameraNode->lookAt(Ogre::Vector3(m_pViewpoint->m_vecVAt.x, m_pViewpoint->m_vecVAt.y, m_pViewpoint->m_vecVAt.z), Ogre::Node::TS_PARENT);
	mCameraNode->setFixedYawAxis( true, ToOgre(m_pViewpoint->m_vecVUp));
#endif

	//	std::cout << mCam->getName() << mCam->getPosition() << std::endl;
	if(mCam->isCustomProjectionMatrixEnabled())
	{
		// orthographic mode.

		bool bOrthographic=m_pViewpoint->getOrthographicMode();	
		if(bOrthographic)
		{
			//OrthoMode
			m_zoom=m_pViewpoint->getZoom();
			m_real aspectRatio=(m_real)mView->getActualWidth()/(m_real)mView->getActualHeight();
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



#ifndef NO_OGRE

	{
		std::list<Ogre::AnimationState*>::iterator i;

		for(i=mAnimationStates.begin(); i != mAnimationStates.end(); ++i)
		{
			(*i)->addTime(m_fElapsedTime);
		}
	}
#endif
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

void OgreRenderer::addNewDynamicObj(Ogre::AnimationState* const as)
{
	mAnimationStates.push_back(as);
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

void OgreRenderer::setupResources(void)
{
#ifndef NO_OGRE
	// Load resource paths from config file
	Ogre::ConfigFile cf;
	cf.load(mResourceFile.ptr());

	// Go through all sections & settings in the file
	Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

	Ogre::String secName, typeName, archName;
	while (seci.hasMoreElements())
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			typeName = i->first;
			archName = i->second;
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
					archName, typeName, secName);
		}
	}
	if(config.GetInt("showConfigDlg"))
	{
#if OGRE_VERSION_MINOR>=12 || OGRE_VERSION_MAJOR>=13
		if(!mRoot->showConfigDialog( OgreBites::getNativeConfigDialog()))
#else
		if(!mRoot->showConfigDialog())
#endif
		{
			Msg::error("Configuration canceled");
		}
	}
	else
	{

		if(!mRoot->restoreConfig())
		{
			printf("Displaying configuration dialog...\n");
#if OGRE_VERSION_MINOR>=12 || OGRE_VERSION_MAJOR>=13
			if(!mRoot->showConfigDialog( OgreBites::getNativeConfigDialog()))
#else
			if(!mRoot->showConfigDialog())
#endif
			{
				Msg::error("Configuration canceled");
			}
		}
	}

#endif
}

bool OgreRenderer::isActive()		
{ 
#ifndef NO_OGRE
	return mWnd && mWnd->isActive();
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
		mRoot->renderOneFrame();
		if(mbScreenshot && renderTexture) {
			TString fn;
			static int _c=0;
			fn.format("%s/%05d.jpg", mScreenshotPrefix.ptr(), _c++);
			renderTexture->writeContentsToFile(fn.ptr());
		}
		if(mbScreenshot && mbUseOGREcapture){
			TString fn;
			static int _c=0;
			fn.format("%s/%05d.jpg", mScreenshotPrefix.ptr(), _c++);
			mWnd->writeContentsToFile(fn.ptr());
		}
	}
#else
	frameStarted(Ogre::FrameEvent());
	frameEnded(Ogre::FrameEvent());
#endif
}
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
	Ogre::TexturePtr texture =Ogre::TextureManager::getSingleton().getByName(texturename);
	if(texture.isNull())
		return -1;
	else
		return texture->getWidth();
#else
	return -1;
#endif
}
void OgreRenderer::_updateDynamicTexture(const char* texturename, CImage const& image, bool reuse)	
{
#ifndef NO_OGRE
	int width=image.GetWidth();
	int width2=TEXTURE_MAX_RESOLUTION;
	while(width2>width) width2/=2;

	if(width2<2)
	{
		Msg::msgBox( "??? resolution error %d.\n Retry after deleting *.texturecache in the fbx folder.", image.GetWidth());
	}
	Ogre::TexturePtr texture =Ogre::TextureManager::getSingleton().getByName(texturename);
	if (!texture.isNull() && !reuse)
		Ogre::TextureManager::getSingleton().remove(texturename);
	if(!reuse || texture.isNull())
		texture = Ogre::TextureManager::getSingleton().createManual(
					texturename, // name
					Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
					Ogre::TEX_TYPE_2D,      // type
					width2, width2,         // width & height
					Ogre::MIP_UNLIMITED,                // number of mipmaps
					Ogre::PF_BYTE_BGRA,     // pixel format
					Ogre::TU_DEFAULT);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
				//Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
						  // textures updated very often (e.g. each frame)
	else
		width2=texture->getWidth();
	
	// Get the pixel buffer
	Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();
	 
	// Lock the pixel buffer and get a pixel box
	pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
	//pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // for best performance use HBL_DISCARD!
	const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
	 
	Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
	 

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
				*pDest++ = c_line->B; // B
				*pDest++ = c_line->G ; // G
				*pDest++ = c_line->R; // R
				*pDest++ = *c_linea; // A
				c_line+=skip;
				c_linea+=skip;
			}
		}
	 
		pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
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
				*pDest++ = c_line->B; // B
				*pDest++ = c_line->G ; // G
				*pDest++ = c_line->R; // R
				*pDest++ = 255; // A
				c_line+=skip;
			}

			pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
		}
	}
	 
	// Unlock the pixel buffer
	pixelBuffer->unlock();
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
		material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	}
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
		if(image.getOpacityMap())
		{
			material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
			material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		}
		material->getTechnique(0)->setDiffuse(diffuseColor.x, diffuseColor.y, diffuseColor.z, 1.0);
		material->getTechnique(0)->setSpecular(specularColor.x, specularColor.y, specularColor.z, 1.0);
		material->getTechnique(0)->setShininess(shininess); 
	}
	else
	{
		material->getTechnique(0)->getPass(0)->removeTextureUnitState(0);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(texturename);
		if(image.getOpacityMap())
		{
			material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
			material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		}
		material->getTechnique(0)->setDiffuse(diffuseColor.x, diffuseColor.y, diffuseColor.z, 1.0);
		material->getTechnique(0)->setSpecular(specularColor.x, specularColor.y, specularColor.z, 1.0);
		material->getTechnique(0)->setShininess(shininess); 
	}
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

	/* I will do the followings manually in lua scripts.
	// Get the technique to use when rendering the depth render texture
	MaterialPtr mDepthMaterial = MaterialManager::getSingleton().getByName("DepthMap");
	mDepthMaterial->load(); // needs to be loaded manually
	//mDepthTechnique = mDepthMaterial->getBestTechnique();

	// Create a custom render queue invocation sequence for the depth render texture
	RenderQueueInvocationSequence* invocationSequence =
		Root::getSingleton().createRenderQueueInvocationSequence("DepthMap");

	// Add a render queue invocation to the sequence, and disable shadows for it
	RenderQueueInvocation* invocation = invocationSequence->add(RENDER_QUEUE_MAIN, "main");
	invocation->setSuppressShadows(true);

	// Set the render queue invocation sequence for the depth render texture viewport
	depthViewport->setRenderQueueInvocationSequenceName("DepthMap");
	*/
#endif
}
void OgreRenderer::updateRenderTexture(const char* param)
{
#ifndef NO_OGRE
	Ogre::TexturePtr texPtr = Ogre::TextureManager::getSingleton().getByName(param);
	Msg::verify(!texPtr.isNull(), "ninjadepthmap==null");
	Ogre::RenderTexture* depth_map = texPtr->getBuffer()->getRenderTarget();
	mViewports[1]->changeView(*mViewports[1]->m_pViewpoint);
	depth_map->update();
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
#if OGRE_VERSION_MAJOR<13
					mWnd->getMetrics( _width, _height, depth, left, top );
#else
					mWnd->getMetrics( _width, _height, left, top );
#endif
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
				Ogre::Rect r = getWindowBounds(mWnd);
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
	Ogre::NameValuePairList misc; 
#ifdef _MSC_VER
	misc["externalWindowHandle"] = Ogre::StringConverter::toString((size_t)handle); 
	mWnd = mRoot->createRenderWindow("My sub render window", width, height, false, &misc); 
#else
#if !defined(GLX_EXTERNAL) && defined(__APPLE__)
	//misc["currentGLContext"]=Ogre::String("True");  
#ifndef NO_GUI
	setMacRenderConfig(handle, misc);
#endif
	
#else
	{
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
		XSync(fl_display, False);
	}
#endif
	mWnd = mRoot->createRenderWindow("My sub render window", width, height, false, &misc); 
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

void OgreRenderer::setrotate(m_real degree, const char* name)
{
#ifndef NO_OGRE
	const char* materialName=name; // name
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(materialName);
	material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureRotate(Ogre::Radian(Ogre::Degree(degree))); 
#endif
}


