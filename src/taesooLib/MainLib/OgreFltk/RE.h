#ifndef _RE_H_
#define _RE_H_
#pragma once

class MotionManager;
class MotionLoader;
class VRMLloader;
class Motion;
class MotionDOF;
class MotionWrap;
class FrameMoveObject;
#ifndef NO_OGRE
class LineSegment;
class Circle;
class LineStrip;
#endif
class OgreTraceManager;
class FrameSensor;
class FltkRenderer;
class OgreRenderer;
class MotionPanel;
namespace RE
{
	enum Color { WHITE, BLUE, GREEN, RED, BLACK};
}
class PLDPrimSkin;
class PLDPrimVRML;
class PLDPrimOgreSkin;

namespace Ogre
{
	class SceneManager;
	class SceneNode;
	class Entity;
	class SimpleRenderable;
}
class AbstractTraceManager
{
public:
	AbstractTraceManager(){}
	virtual~AbstractTraceManager(){}
	virtual void message(const char* id, const char* content)=0;
	virtual void dumpMessage(TStrings& out)=0;
};


/// RE::g_pGlobals->pRenderer, g_pFltkRenderer를 직접 access하는 것을 최대한 줄이려고 만든 global namespace.
/**
global variable이 access되는 것을 한곳에 localise함으로써 나중에 렌더러가 여러개 된다던지, 멀티 쓰레딩, DLL 사용시 문제요소를 최소화한다.
*/
namespace RE	
{
	struct Globals
	{
		Globals()	{ pRenderer=NULL; pFltkRenderer=NULL; pMotionPanel=NULL;}
		OgreRenderer* pRenderer;
		FltkRenderer* pFltkRenderer;
		MotionPanel* pMotionPanel;
		std::vector<void*> defaultSkins;
		int global_mouse_x, global_mouse_y;
	};
	

	void setGlobalMousePos(int x, int y);
	int getGlobalMouseX();
	int getGlobalMouseY();
	bool useSeperateOgreWindow();

	// do not delete the returned MotionLoader instance.
	MotionLoader* motionLoader(const char* name);
	MotionLoader* createMotionLoader(const char* filename, const char * key);
	
	Ogre::SceneManager* ogreSceneManager();
	Ogre::SceneNode* ogreRootSceneNode();
	
	// use only in main.cpp or main_standalone.cpp. 
	OgreRenderer* _createRenderer(int& w, int &rw);
	OgreTraceManager* createTraceManager();

	OgreRenderer& renderer();
	FltkRenderer& FltkRenderer();
	MotionManager& motionManager();
#ifndef NO_GUI
	MotionPanel& motionPanel();
#endif
	bool rendererValid();
	bool motionPanelValid();
	
	enum PLDPrimSkinType {PLDPRIM_BLOB, PLDPRIM_LINE, PLDPRIM_SKIN, PLDPRIM_POINT, PLDPRIM_CYLINDER, PLDPRIM_CYLINDER_LOWPOLY, PLDPRIM_BOX};	// 각각 PLDPrimBone, PLDPrimLine, PLDPrimOgreSKin, PLDPrimPoint, PLDPrimCylinder를 뜻한다.

	/// create skin using default skin: blob model at first, skin model after changeDefaultSkin.
	PLDPrimSkin* createSkin(const MotionLoader& skel);
	PLDPrimSkin* createSkin(const MotionLoader& skel, PLDPrimSkinType t);
	PLDPrimSkin* createSkin(const PLDPrimOgreSkin& other);	
	PLDPrimSkin* createSkin(const Motion& mot); //!< utility function. 
	PLDPrimSkin* createSkin(const Motion& mot, PLDPrimSkinType t); //!< utility function. 
	PLDPrimSkin* createSkin(const MotionDOF& mot); //!< utility function. 
	PLDPrimSkin* createSkin(const MotionWrap& mot);
	PLDPrimSkin* createSkin(const MotionWrap& mot, PLDPrimSkinType t);
	PLDPrimVRML* createVRMLskin(VRMLloader*pTgtSkel, bool bDrawSkeleton=false);
	PLDPrimSkin* createOgreSkin(const Motion& mot);
	Ogre::SceneNode* getSceneNode(PLDPrimSkin* skin);
	Ogre::SceneNode* getSceneNode(const char* node_name);
	// 만약 id이름을 가지는 scenenode가 존재하면, 지우고 다시 생성한다.
	Ogre::SceneNode* createSceneNode(const char* node_name);
	Ogre::SceneNode* createChildSceneNode(Ogre::SceneNode* parent, const char* child_name);
	
	Ogre::Entity* createPlane(const char* id, m_real width, m_real height, int xsegment, int ysegment, int texSegx, int texSegy);
	Ogre::Entity* createTerrain(const char* id, const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ);

	// create a scenenode and an entity attached to the scenenode. using these functions simplify library dependencies as you don't need to include some <Ogre~.h>
	Ogre::SceneNode* createEntity(Ogre::SceneNode* parentNode, const char* node_name, const char* filename); //!< returns a new child scene node.
	Ogre::SceneNode* createEntity(const char* node_name, const char* filename);
	Ogre::SceneNode* createEntity(const char* node_name, const char* filename, const char* materialName);
	Ogre::SceneNode* createEntity(const char* node_name, const char* filename, vector3 const& pos);
	Ogre::SceneNode* createEntity(const char* node_name, const char* filename, quater const& rot, vector3 const& pos);
	Ogre::SceneNode* createMovableText(const char* node_name, const char* contents, int charHeight = 16, RE::Color c=RE::WHITE, const char* fontName = "TrebuchetMSBold" );
	FrameSensor* createFrameSensor();
	
	// move sceneNode containing entities. using these functions simplify library dependencies as you don't need to include some <Ogre~.h>
	void moveEntity(Ogre::SceneNode* node, vector3 const& p);
	void moveEntity(Ogre::SceneNode* node, quater const& q, vector3 const& pos);
	void moveEntity(Ogre::SceneNode* node, vector3 const& scale, vector3 const& pos);
	void moveEntity(Ogre::SceneNode* node, double scale, vector3 const& pos);
	void moveEntity(Ogre::SceneNode* node, vector3 const& scale, quater const& q, vector3 const& pos);
	void moveEntity(Ogre::SceneNode* node, double scale, quater const& q, vector3 const& pos);

	/// change default skin to an ogre skin.
	void changeDefaultSkin(const MotionLoader& skel, Ogre::Entity* entity, const char* mappingTable=NULL, bool bCurrPoseAsBindPose=true);
	void resetDefaultSkin(const MotionLoader& skel);
	
	/// create ogre skin
	PLDPrimSkin* createOgreSkin(const MotionLoader& skel, Ogre::Entity* entity, const char* mappingTable=NULL, bool bCurrPoseAsBindPose=true, double scale=1.0);

#ifndef NO_OGRE
	LineSegment* createLine();
	LineStrip* createThinLine();
	Circle* createCircle();
#endif
	//	SurfelClouds* createSurfel(ResourceHandle hTexture);

	// remove skin
	void remove(FrameMoveObject* pRP);
	// remove a sceneNode containing entities and child scene nodes
	// all attached object will be destroyed.
	void removeEntity(Ogre::SceneNode* node);
	void removeEntity(const char* node_name);

	// get the first entity attached to node.
	Ogre::Entity* getEntity(Ogre::SceneNode* node);

	// tip: automatic deletion of Ogre::SceneNode! 
	// use boost::shared_ptr<Ogre::SceneNode>, or boost::scoped_tpr<Ogre::SceneNode>
	//  ex:
	// boost::shared_ptr<Ogre::SceneNode> mNode;
	// mNode.reset(RE::createSceneNode("graphNode"), (void(*)(Ogre::SceneNode* node))RE::removeEntity);
	// -> You don't need to worry about deleting mNode.

	TString generateUniqueName();

	void setColor(Ogre::SimpleRenderable *pRendererble, Color c);
	void setEntityColor(Ogre::SceneNode* pNode, Color c);
	void setMaterialName(Ogre::SceneNode* pNode, const char* mat);
	TString getMaterialName(Color c, bool bSolidColor=false);

	void outputState(bool bOutput);
	void output(const char* key, const char* formatString, ...);
	int numOutputManager();
	void usleep(int usec);

	namespace motion
	{
		void loadAnimation(MotionLoader& skel, Motion& mot, const char* fn);
		void concatFromFile(Motion& motion, const char* fn);
	}

	// private
	extern std::vector<AbstractTraceManager*> g_traceManagers;
}


#endif
