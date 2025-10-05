#pragma once
#include <list>

class FrameMoveObject;
class MotionManager;
class Viewpoint;
class CImage;

#ifndef NO_OGRE
#include <OgreFrameListener.h>
#else
#include "../console/dummies.h"
#endif

#if defined(NO_GUI) || defined(_MSC_VER)
#define NO_OIS
#endif

#ifndef _MSC_VER
 #if FL_MAJOR_VERSION == 2
 
 #include <fltk/visual.h>
 #include <fltk/Window.h>
 
 typedef fltk::Window FLTKWindow;
 
 #elif FL_MAJOR_VERSION == 1
 
 #include <FL/Fl_Window.H>
 
 typedef Fl_Window FLTKWindow;
 
 #endif
#endif
#ifndef NO_OIS
 namespace OIS { class Mouse; class Keyboard; class InputManager;}
#endif

#ifndef NO_OGRE
#ifdef None
#undef None
#endif
#include <OgreOverlaySystem.h>
#include "StaticPluginLoader.h"
#if !defined( __APPLE__) && !defined(_MSC_VER) 
#define SEP_USE_SDL2
#endif

#ifdef SEP_USE_SDL2
    #include <SDL.h>
#endif

namespace Ogre
{
	class MovableTextFactory;
}
#endif

// Ogre의 GraphicsSystem.cpp를 많이 참고함. 
#ifndef NO_OGRE
#include "BaseSystem.h"
class OgreRenderer : public Ogre::FrameListener, public BaseSystem
#else
class OgreRenderer : public Ogre::FrameListener
#endif
{
protected:
	void* _hWnd;
	void _constructor(const char* fallback_configFileName, const char* configFileName, const char* plugins_file, const char* ogre_config);
	std::string mPluginPath;
    #ifdef SEP_USE_SDL2
        SDL_Window          *mSdlWindow;
        void handleWindowEvent( const SDL_Event& evt );
    #endif
public:
#ifndef NO_OGRE
	Ogre::MovableTextFactory * mMovableTextFactory;
#endif
	void _locateTaesooLib();
	OgreRenderer(const char* fallback_configFileName, const char* configFileName, const char* plugins_file, const char* ogre_config);
	OgreRenderer();
	virtual ~OgreRenderer();
	int getConfig(const char* id);
	double getConfigFloat(const char* id);
	void createInputSystems(size_t hWnd);
	virtual void initialize(void* handle, int width, int height);
	void deinitialize(void);
	void speedControl(float f);	//normal speed에 fact만큼 배한다.
	void pause(bool b);	//pause enable disable
	void setScreenshotMotionBlur(int n);
	void screenshot(bool b);
	void setScreenshotPrefix(const char* prefix/* e.g. "../dump/dump"*/);
	void fixedTimeStep(bool b);
	void toggleScreenShotMode();
	void setResourceFile(const char* filename);
	bool isActive()	;

	// type: "R8G8B8" or "FLOAT16_R"
	void createRenderTexture(const char* type, int width, int height, bool useCurrentViewport, const char* name);
	void updateRenderTexture(const char* param);
	void setMaterialParam(const char* mat, const char* paramName, double value);
	void cloneMaterial(const char* mat, const char* newMat);
	void _updateDynamicTexture(const char* textureName, CImage const& image, bool reuseIfExists=true);	
	void _linkMaterialAndTexture(const char* materialName, const char* textureName);
	void createDynamicTexture(const char* name, CImage const& image);	
	void createDynamicTexture(const char* name, CImage const& image, vector3 const& diffuseColor, vector3 const& specular_color, double shininess=10.0);	
	void createMaterial(const char* name, vector3 const& diffuseColor, vector3 const& specular_color, double shininess=10.0);	
	void addFrameMoveObject(FrameMoveObject* pFMO);
	void removeFrameMoveObject(FrameMoveObject* pFMO);
	void addAfterFrameMoveObject(FrameMoveObject* pFMO);
	void removeAfterFrameMoveObject(FrameMoveObject* pFMO);

	//void setrotate(m_real degree, const char* materialName);
	void ChageTextureImage(const char* name, const char* textureName);
	void setrotate(m_real degree, const char* name);

	MotionManager *m_pMotionManager;

	class Viewport
	{
		public:
		Viewport();
		virtual ~Viewport();
		virtual void init(OgreRenderer& renderer, vectorn const& param);
		virtual void init(OgreRenderer& renderer, Viewport & ref);
		virtual void changeView(Viewpoint const& view);	
		virtual void changeView(matrix4 const& matview);	
		void setOrthographicMode(bool isOrtho);
		void setCustomProjectionMatrix(matrix4 const& mat_proj);
		
		virtual void createCamera(OgreRenderer& renderer, int width, int height);
		void setupRTT(OgreRenderer& renderer, int width, int heigth);

		int getActualWidth() const;
		int getActualHeight() const;

#ifndef NO_OGRE
		Ogre::SceneManager *mScene;
		Ogre::Camera *mCam;
		//Ogre::Viewport *mView;
        Ogre::SceneNode* mCameraNode;       // camera node
#endif
		Viewpoint* m_pViewpoint;
	};


	int numViewport() 	{ return mViewports.size();}
	Viewport& viewport();
	Viewport const& viewport()	const;
	Viewport& viewport(int viewport);
	void addNewViewport();

#ifndef NO_OGRE
	Ogre::SceneManager* getSceneManager() { return viewport().mScene;}
	Ogre::Camera* getCamera(void) const                     { return viewport().mCam; }
	Ogre::Root* getRoot() { return mRoot;}
	Ogre::Window* getRenderWindow(void) const               { return mWnd; }
	Ogre::CompositorWorkspace* getCompositorWorkspace(void) const { return mWorkspace; }
	Ogre::v1::OverlaySystem* getOverlaySystem(void) const   { return mOverlaySystem; }
	virtual void stopCompositor(void);
	virtual void restartCompositor(void);
#endif

	int _getOgreTextureWidth(const char* texturename);
#ifndef NO_OGRE
	Ogre::Root *mRoot;
	Ogre::Window *mWnd;
	Ogre::RenderSystem *mRSys;
	Ogre::CompositorWorkspace   *mWorkspace;
#ifndef NO_OIS
    OIS::Mouse        *mMouse;
    OIS::Keyboard     *mKeyboard;
    OIS::InputManager *mInputSystem;
#endif
	Ogre::v1::OverlaySystem*  mOverlaySystem;
	StaticPluginLoader          mStaticPluginLoader;
	bool                mUseHlmsDiskCache;
	bool                mUseMicrocodeCache;

	Ogre::ColourValue   mBackgroundColour;
	void loadTextureCache(void);
	void saveTextureCache(void);
	void loadHlmsDiskCache(void);
	void saveHlmsDiskCache(void);
	virtual void registerHlms(void);
	/// Optional override method where you can perform resource group loading
	/// Must at least do ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
	//virtual void chooseSceneManager(void); -> Viewport::init
	//virtual void createCamera(void); ->Viewport::_init
	/// Virtual so that advanced samples such as Sample_Compositor can override this
	/// method to change the default behavior if setupCompositor() is overridden, be
	/// aware @mBackgroundColour will be ignored
	virtual Ogre::CompositorWorkspace* setupCompositor(void);

	/// Called right before initializing Ogre's first window, so the params can be customized
	virtual void initMiscParamsListener( Ogre::NameValuePairList &params );

	/// Optional override method where you can create resource listeners (e.g. for loading screens)
	virtual void createResourceListener(void) {}
	void createPcfShadowNode(void);
	void createEsmShadowNodes(void);
#endif
	void ignoreTime()	{mbTimeStop=true;}
	void setCaptureFPS(float fps)	{m_fCaptureFPS=fps;}
	void createRenderWindow(void* handle, int width, int height);
	bool mbScreenshot;	//screenshot enable disable. almost private.
	void renderOneFrame();
	void setBackgroundColour(float r, float g, float b);
#ifndef NO_OGRE
	void addResourceLocation( const Ogre::String &archName, const Ogre::String &typeName, const Ogre::String &secName );
	void setupShadowNode(bool useESM);
#endif
protected:
	std::vector<Viewport*> mViewports;
	int mCurrViewports;
	TString mResourceFile;
	std::string mWriteAccessFolder;
	TString mScreenshotPrefix;
	bool mbFixedTimeStep;
	bool mbTimeStop;
	bool mStatsOn;
	bool mbPause;	//animation pause
	
	float m_fCaptureFPS;	// default: 30
	float m_fTimeScaling;	//frame speed fact
	float m_fElapsedTime;

	std::list<FrameMoveObject*> mFrameMoveObjects;		
	std::list<FrameMoveObject*> mAfterFrameMoveObjects;		

#ifndef NO_OGRE
	virtual void setupResources(void);
#endif

	bool frameStarted(const Ogre::FrameEvent& evt);
	bool frameEnded(const Ogre::FrameEvent& evt);

public:
	void _toggleHelpMode();
	void _updateDebugCaption();
};

#ifndef NO_OGRE
Ogre::Quaternion ToOgre(const quater& q);
Ogre::Vector3 ToOgre(const vector3& v);
vector3 ToBase(const Ogre::Vector3& v);
quater ToBase(const Ogre::Quaternion& v);

#endif
