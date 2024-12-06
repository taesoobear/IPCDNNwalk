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
#if OGRE_VERSION_MINOR>=9 || OGRE_VERSION_MAJOR>=13
#include <Overlay/OgreOverlaySystem.h>
#endif
#endif
// frame listener 선언 
class OgreRenderer : public Ogre::FrameListener
{
protected:
	void _constructor(const char* fallback_configFileName, const char* configFileName, const char* plugins_file, const char* ogre_config);
	void _locateTaesooLib();
	std::string mTaesooLib_path;
	std::string mPluginPath;
public:
	std::string taesooLibPath() const { return mTaesooLib_path;}
	OgreRenderer(const char* fallback_configFileName, const char* configFileName, const char* plugins_file, const char* ogre_config);
	OgreRenderer();
	virtual ~OgreRenderer();
	void createInputSystems(size_t hWnd);
	virtual void firstInit(void* handle, int width, int height);
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
	void addNewDynamicObj(Ogre::AnimationState* const as);
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
		
		virtual void _init(OgreRenderer& renderer, int width, int height);
		void setupRTT(OgreRenderer& renderer, int width, int heigth);
#ifndef NO_OGRE
		Ogre::SceneManager *mScene;
		Ogre::Camera *mCam;
		Ogre::Viewport *mView;
#if OGRE_VERSION_MAJOR>=13
        Ogre::SceneNode* mCameraNode;       // camera node
#endif
#endif
		Viewpoint* m_pViewpoint;
	};


	int numViewport() 	{ return mViewports.size();}
	Viewport& viewport();
	Viewport const& viewport()	const;
	Viewport& viewport(int viewport);
	void addNewViewport();

	int _getOgreTextureWidth(const char* texturename);
#ifndef NO_OGRE
	Ogre::Root *mRoot;
	Ogre::RenderWindow *mWnd;
	Ogre::RenderSystem *mRSys;
#ifndef NO_OIS
    OIS::Mouse        *mMouse;
    OIS::Keyboard     *mKeyboard;
    OIS::InputManager *mInputSystem;
#endif
#if OGRE_VERSION_MINOR>=9 || OGRE_VERSION_MAJOR>=13
	Ogre::OverlaySystem*  mOverlaySystem;
#endif
#endif
	void ignoreTime()	{mbTimeStop=true;}
	void setCaptureFPS(float fps)	{m_fCaptureFPS=fps;}
	void createRenderWindow(void* handle, int width, int height);
	bool mbScreenshot;	//screenshot enable disable. almost private.
	void renderOneFrame();
	void setBackgroundColour(float r, float g, float b);
protected:
	std::vector<Viewport*> mViewports;
	int mCurrViewports;
	TString mResourceFile;
	TString mScreenshotPrefix;
	bool mbFixedTimeStep;
	bool mbTimeStop;
	bool mStatsOn;
	bool mbPause;	//animation pause
	
	float m_fCaptureFPS;	// default: 30
	float m_fTimeScaling;	//frame speed fact
	float m_fElapsedTime;

	std::list<Ogre::AnimationState*> mAnimationStates;
	// for backward compatibility
	std::list<FrameMoveObject*> mFrameMoveObjects;		
	std::list<FrameMoveObject*> mAfterFrameMoveObjects;		


	virtual void setupResources(void);

	bool frameStarted(const Ogre::FrameEvent& evt);
	bool frameEnded(const Ogre::FrameEvent& evt);
};

#ifndef NO_OGRE
Ogre::Quaternion ToOgre(const quater& q);
Ogre::Vector3 ToOgre(const vector3& v);
vector3 ToBase(const Ogre::Vector3& v);
quater ToBase(const Ogre::Quaternion& v);

#endif
