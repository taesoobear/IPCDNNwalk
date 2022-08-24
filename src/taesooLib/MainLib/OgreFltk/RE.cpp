#include "stdafx.h"
#include "RE.h"
#include "renderer.h"
#include "../BaseLib/motion/Motion.h"
#include "../BaseLib/motion/MotionWrap.h"
#include "MotionManager.h"
#include "Circle.h"
#include "timesensor.h"
#include "LineSegment.h"
#include "renderer.h"
#include "Mesh.h"
#include "MovableText.h"
#include "VRMLloader.h"
#ifndef NO_OGRE
#include "Ogre.h"
#endif
#include "pldprimskin_impl.h"
namespace RE
{
	Globals* g_pGlobals=NULL;
}

MotionLoader* RE::motionLoader(const char* name)
{
	return RE::renderer().m_pMotionManager->GetMotionLoaderPtr((char*)name);
}
MotionLoader* RE::createMotionLoader(const char* name, const char* key)
{
	MotionLoader* l=MotionManager::createMotionLoader(TString(name));
	RE::renderer().m_pMotionManager->createMotionLoaderExt(key, l);
	return l;
}

OgreRenderer& RE::renderer()
{
	return *RE::g_pGlobals->pRenderer;
}

FltkRenderer& RE::FltkRenderer()
{
	return *RE::g_pGlobals->pFltkRenderer;
}
Ogre::SceneManager* RE::ogreSceneManager()
{
  #ifndef NO_OGRE
	return RE::renderer().viewport().mScene;
#else
  return NULL;
#endif
}

Ogre::SceneNode* RE::ogreRootSceneNode()
{
  #ifndef NO_OGRE

	return RE::renderer().viewport().mScene->getRootSceneNode();
#else
return NULL;
#endif
}

MotionManager& RE::motionManager()
{
	return *RE::renderer().m_pMotionManager;
}

#ifndef NO_GUI
MotionPanel& RE::motionPanel()
{
	return *RE::g_pGlobals->pMotionPanel;
}
#endif
bool RE::motionPanelValid()
{
#ifndef NO_GUI
	return RE::g_pGlobals &&RE::g_pGlobals->pMotionPanel!=NULL;
#else
  return false;
#endif
}

bool RE::rendererValid()
{
	// this works for both console and GUI apps. 
	// To distinguish console app from GUI apps, use motionPanelValid()
	return RE::g_pGlobals &&RE::g_pGlobals->pRenderer!=NULL;
}

PLDPrimSkin* RE::createSkin(const MotionLoader& skel,PLDPrimSkinType  t)
{
	PLDPrimSkin* pSkin;
	switch(t)
	{
	case PLDPRIM_BLOB:
		pSkin=new PLDPrimBone((MotionLoader*)(&skel), RE::renderer());
		RE::renderer().addFrameMoveObject(pSkin);
		return pSkin;
		break;
	case PLDPRIM_LINE:
		pSkin=new PLDPrimLine((MotionLoader*)(&skel), RE::renderer());
		RE::renderer().addFrameMoveObject(pSkin);
		return pSkin;
		break;
	case PLDPRIM_SKIN:
		// return default skin. (default skin= ogreskin after changeDefaultSkin is applied.)
		break;
	case PLDPRIM_POINT:
		pSkin=new PLDPrimPoint((MotionLoader*)(&skel), RE::renderer());
		RE::renderer().addFrameMoveObject(pSkin);
		return pSkin;
		break;
	case PLDPRIM_CYLINDER:
		pSkin=new PLDPrimCyl((MotionLoader*)(&skel), RE::renderer());
		RE::renderer().addFrameMoveObject(pSkin);
		return pSkin;
		break;
	case PLDPRIM_CYLINDER_LOWPOLY:
		pSkin=new PLDPrimCyl((MotionLoader*)(&skel), RE::renderer(), true);
		RE::renderer().addFrameMoveObject(pSkin);
		return pSkin;
	case PLDPRIM_BOX:
		pSkin=new PLDPrimBox((MotionLoader*)(&skel), RE::renderer());
		RE::renderer().addFrameMoveObject(pSkin);
		return pSkin;
		break;
	}

	return createSkin(skel);
}
PLDPrimSkin* RE::createSkin(const MotionDOF& mot) //!< utility function. 
{
	VRMLloader* skel=dynamic_cast<VRMLloader*>(&mot.mInfo.skeleton());
	assert(skel);
	PLDPrimVRML* pSkin=RE::createVRMLskin(skel);
	pSkin->scale(100,100,100);
	pSkin->applyAnim(mot);
	return pSkin;
}

PLDPrimSkin* RE::createSkin(const MotionWrap& mot)
{
	if(mot.hasMotionDOF())
		return RE::createSkin(mot.motdof());
	return RE::createSkin(mot.mot());
}

PLDPrimSkin* RE::createSkin(const MotionWrap& mot, PLDPrimSkinType t)
{
	if(mot.hasMotionDOF())
	{
		if(t==RE::PLDPRIM_BLOB)
		{
			return RE::createSkin(mot.motdof());
		}
		else
		{
			PLDPrimSkin* pSkin=RE::createSkin(mot.motdof().mInfo.skeleton(),t);
			pSkin->scale(100,100,100);
			pSkin->applyAnim(mot.motdof());
			return pSkin;
		}
	}
	return RE::createSkin(mot.mot(), t);
}

PLDPrimSkin* RE::createSkin(const MotionLoader& skel)
{
	PLDPrimSkin* pSkin;
	if(RE::g_pGlobals->defaultSkins.size() && RE::g_pGlobals->defaultSkins[skel.getHandle()])
		pSkin=createSkin(*reinterpret_cast<PLDPrimOgreSkin*>(RE::g_pGlobals->defaultSkins[skel.getHandle()]));
	else
	{
		pSkin=new PLDPrimCyl((MotionLoader*)(&skel), RE::renderer());
		RE::renderer().addFrameMoveObject(pSkin);
	}
	return pSkin;
}

PLDPrimSkin* RE::createSkin(const Motion& mot)
{
	PLDPrimSkin* pSkin=RE::createSkin(mot.skeleton());
	pSkin->ApplyAnim(mot);
	pSkin->m_pTimer->StartAnim();
	return pSkin;
}

PLDPrimSkin* RE::createSkin(const Motion& mot, PLDPrimSkinType t)
{
	PLDPrimSkin* pSkin=RE::createSkin(mot.skeleton(),t);
	pSkin->ApplyAnim(mot);
	pSkin->m_pTimer->StartAnim();
	return pSkin;
}


#ifdef USE_LUABIND
#include "../WrapperLua/BaselibLUA.h"
#include <luabind/luabind.hpp>
#include <luabind/operator.hpp>

#include "../BaseLib/motion/version.h"
void PREPAIR_SKIN(const Motion & curMot, TString& meshFile, TString& mappingFile)
{
	LUAwrapper config;

	config.setVal<const char*>("motionId", curMot.GetIdentifier());
	config.dofile("../Resource/skinning.lua");



	TString bindPose;
	mappingFile=config.getValue<const char*>("mappingFile");
	bindPose=config.getValue<const char*>("bindPoseFile");
	meshFile=config.getValue<const char*>("meshFile");
	if(bindPose.length())
	{
		loadPose(curMot.pose(0), bindPose);
	}
	curMot.setSkeleton(0);
}

#else
#include "../BaseLib/motion/version.h"
#include "../../MainLib/WrapperLua/LUAwrapper.h"
void Register_baselib(lua_State*L);
void PREPAIR_SKIN(const Motion & curMot, TString& meshFile, TString& mappingFile)
{
	LUAwrapper config;
	Register_baselib(config.L);

	config.setString("motionId", curMot.GetIdentifier());
	config.dofile("../Resource/skinning.lua");



	TString bindPose;
	mappingFile=config.getString("mappingFile").c_str();
	bindPose=config.getString("bindPoseFile").c_str();
	meshFile=config.getString("meshFile").c_str();
	if(bindPose.length())
	{
		loadPose(curMot.pose(0), bindPose);
	}
	curMot.setSkeleton(0);
}
#endif

//#include "../Ogre/PLDPrimCustumSkin.h"
#ifdef INCLUDE_OGRESKINENTITY
#include "../Ogre/OgreSkinEntity.h"
#endif
/*
PLDPrimSkin* RE::createCustumSkin(const Motion& curMot)
{
	TString meshFile, mappingFile;
	PREPAIR_SKIN(curMot, meshFile, mappingFile);

	PLDPrimSkin* pSkin=new PLDPrimCustumSkin(&curMot.skeleton(), Ogre::createSkinEntity(*(RE::renderer().mScene), RE::generateUniqueName().ptr(), meshFile.ptr()), mappingFile, RE::renderer(), true);
	RE::renderer().addFrameMoveObject(pSkin);

	pSkin->ApplyAnim(curMot);

	return pSkin;

}*/

PLDPrimSkin* RE::createOgreSkin(const Motion& curMot)
{
#ifdef NO_OGRE
  return RE::createSkin(curMot);
#else

  TString meshFile, mappingFile;
	PREPAIR_SKIN(curMot, meshFile, mappingFile);

	PLDPrimSkin* pSkin=RE::createOgreSkin(curMot.skeleton(),

		RE::renderer().viewport().mScene->createEntity(RE::generateUniqueName().ptr(), meshFile.ptr()),
		mappingFile);

	pSkin->ApplyAnim(curMot);

	return pSkin;
#endif
}

PLDPrimSkin* RE::createSkin(const PLDPrimOgreSkin& other)
{
	static int g_nDefaultSkin=0;
	TString nameid;
	nameid.format("_ds%d", g_nDefaultSkin);
	g_nDefaultSkin++;

	PLDPrimSkin* pSkin=new PLDPrimOgreSkin(other, RE::renderer(), nameid);
	RE::renderer().addFrameMoveObject(pSkin);
	return pSkin;
}

void RE::moveEntity(Ogre::SceneNode* node, vector3 const& p)
{
 #ifndef NO_OGRE
	node->resetToInitialState();
	node->translate(ToOgre(p));
#endif
}

void RE::moveEntity(Ogre::SceneNode* node, quater const& q, vector3 const& p)
{
   #ifndef NO_OGRE

	node->resetToInitialState();
	node->rotate(ToOgre(q));
	node->translate(ToOgre(p));
#endif
}

void RE::moveEntity(Ogre::SceneNode* node, vector3 const& scale, vector3 const& p)
{
 #ifndef NO_OGRE
	node->resetToInitialState();
	node->scale(ToOgre(scale));
	node->translate(ToOgre(p));
#endif
}
void RE::moveEntity(Ogre::SceneNode* node, double scale, vector3 const& p)
{
 #ifndef NO_OGRE
	node->resetToInitialState();
	node->scale(scale, scale, scale);
	node->translate(ToOgre(p));
#endif
}

void RE::moveEntity(Ogre::SceneNode* node, vector3 const& scale, quater const& q, vector3 const& p)
{
 #ifndef NO_OGRE
	node->resetToInitialState();
	node->scale(ToOgre(scale));
	node->rotate(ToOgre(q));
	node->translate(ToOgre(p));
#endif
}
void RE::moveEntity(Ogre::SceneNode* node, double scale, quater const& q, vector3 const& p)
{
 #ifndef NO_OGRE
	node->resetToInitialState();
	node->scale(scale, scale, scale);
	node->rotate(ToOgre(q));
	node->translate(ToOgre(p));
#endif
}

Ogre::SceneNode* RE::createChildSceneNode(Ogre::SceneNode* parent, const char* node_name)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	try {
		// 있으면 지운다.
		pNode=((Ogre::SceneNode*)parent->getChild(node_name));
		RE::removeEntity(pNode);
	}
	catch( Ogre::Exception& e )
	{
		// 없으면 okay.
	}

	pNode=parent->createChildSceneNode(node_name);
	pNode->setVisible(true);

	return pNode;
#else
  return NULL;
#endif
}

Ogre::SceneNode* RE::getSceneNode(const char* node_name)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	try {
		// 있으면 지운다.
		pNode=RE::ogreSceneManager()->getSceneNode(node_name);
		return pNode;
	}
	catch( Ogre::Exception& e )
	{
		// 없으면 okay.
		return NULL;
	}
#else
	return NULL;
#endif
}

Ogre::SceneNode* RE::createSceneNode(const char* node_name)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	try {
		// 있으면 지운다.
		pNode=RE::ogreSceneManager()->getSceneNode(node_name);
		RE::removeEntity(pNode);
	}
	catch( Ogre::Exception& e )
	{
		// 없으면 okay.
	}

	pNode=RE::ogreRootSceneNode()->createChildSceneNode(node_name);
	pNode->setVisible(true);

	return pNode;
#else 
    return NULL;
#endif
}

Ogre::SceneNode* RE::createEntity(const char* id, const char* filename)
{
	return RE::createEntity(RE::ogreRootSceneNode(), id, filename);
}
Ogre::SceneNode* RE::createEntity(const char* id, const char* filename, const char* materialName)
{
	Ogre::SceneNode* parentNode=RE::ogreRootSceneNode();
 #ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	try {
		// 같은 이름의 SceneNod가 있으면 그냥 재활용한다.
		pNode=RE::ogreSceneManager()->getSceneNode(id);
		pNode->resetToInitialState();
	}
	catch( Ogre::Exception& e )
	{
		TString entity_name;
		entity_name.format("_entity_%s", id);
		pNode=parentNode->createChildSceneNode(id);
		Ogre::Entity *entity = RE::ogreSceneManager()->createEntity(entity_name.ptr(), filename);
		entity->setMaterialName(materialName);
		pNode->attachObject(entity);
		pNode->setVisible(true);
	}

	return pNode;
#else 
  return NULL;
#endif
}

Ogre::SceneNode* RE::createEntity(Ogre::SceneNode* parentNode, const char* id, const char* filename)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	try {
		// 같은 이름의 SceneNod가 있으면 그냥 재활용한다.
		pNode=RE::ogreSceneManager()->getSceneNode(id);
		pNode->resetToInitialState();
	}
	catch( Ogre::Exception& e )
	{
		TString entity_name;
		entity_name.format("_entity_%s", id);
		pNode=parentNode->createChildSceneNode(id);
		Ogre::Entity *entity = RE::ogreSceneManager()->createEntity(entity_name.ptr(), filename);
		pNode->attachObject(entity);
		pNode->setVisible(true);
	}

	return pNode;
#else 
  return NULL;
#endif
}

const Ogre::ColourValue& RE_colorToOgreColour(RE::Color c)
{
#ifdef NO_GUI
	return Ogre::ColourValue::White;
#else
	switch(c)
	{
		case RE::WHITE:
			return Ogre::ColourValue::White;
		case RE::BLUE:
			return Ogre::ColourValue::Blue;
		case RE::GREEN:
			return Ogre::ColourValue::Green;
		case RE::RED:
			return Ogre::ColourValue::Red;
		case RE::BLACK:
			return Ogre::ColourValue::Black;
	}
	return Ogre::ColourValue::Black;
#endif
}

Ogre::SceneNode* RE::createMovableText(const char* node_name, const char* contents, int charHeight , RE::Color c, const char* fontName )
{
	const Ogre::ColourValue &color =
		RE_colorToOgreColour(c);
 #ifndef NO_OGRE
	// MovableText에 bug가 있어서 매번 지우고 다시 setCaption한다.
	TString entity_name;
	entity_name.format("_entity_mt_%s", node_name);
	Ogre::SceneNode* pNode=NULL;
	try {
		// 있으면 그냥 재활용한다.
		pNode=RE::ogreSceneManager()->getSceneNode(node_name);
		pNode->resetToInitialState();
		delete pNode->detachObject((unsigned short)0);
	}
	catch( Ogre::Exception& e )
	{
		pNode=RE::ogreRootSceneNode()->createChildSceneNode(Ogre::String(node_name));
	}


	Ogre::MovableText* entity=new Ogre::MovableText(entity_name.ptr(), contents, fontName, charHeight, color);
//	entity->setColor(Ogre::ColourValue(1.0,1.0,1.0));
	pNode->attachObject(entity);
	pNode->setVisible(true);

	return pNode;
#else
  return NULL;
#endif
}
void RE::setGlobalMousePos(int x, int y)
{
	RE::g_pGlobals->global_mouse_x=x;
	RE::g_pGlobals->global_mouse_y=y;

}
int RE::getGlobalMouseX()
{
	return RE::g_pGlobals->global_mouse_x;
}
int RE::getGlobalMouseY()
{
	return RE::g_pGlobals->global_mouse_y;
}


Ogre::SceneNode* RE::createEntity(const char* id, const char* filename, vector3 const& pos)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* sn;
	sn=createEntity(id, filename);
	sn->translate(ToOgre(pos));
	return sn;
#else
  return NULL;
#endif
}

Ogre::SceneNode* RE::createEntity(const char* id, const char* filename, quater const& ori, vector3 const& pos)
{
#ifndef NO_OGRE
	Ogre::SceneNode* sn;
	sn=createEntity(id, filename);
	sn->rotate(ToOgre(ori));
	sn->translate(ToOgre(pos));
	return sn;
#else
  return NULL;
#endif
}


void RE::changeDefaultSkin(const MotionLoader& skel, Ogre::Entity* entity, const char* mappingTable, bool bCurrPoseAsBindPose)
{
#ifndef NO_OGRE
	if(RE::g_pGlobals->defaultSkins.size()==0)
	{
		RE::g_pGlobals->defaultSkins.resize(RE::renderer().m_pMotionManager->GetNumResource());
		for (int i=0; i<RE::g_pGlobals->defaultSkins.size(); i++)
			RE::g_pGlobals->defaultSkins[i]=NULL;
	}

	if(RE::g_pGlobals->defaultSkins[skel.getHandle()])
		RE::remove(reinterpret_cast<PLDPrimOgreSkin*>(RE::g_pGlobals->defaultSkins[skel.getHandle()]));

	RE::g_pGlobals->defaultSkins[skel.getHandle()]=reinterpret_cast<void*>(static_cast<PLDPrimOgreSkin*>(createOgreSkin(skel, entity, mappingTable, bCurrPoseAsBindPose)));

	reinterpret_cast<PLDPrimOgreSkin*>(RE::g_pGlobals->defaultSkins[skel.getHandle()])->SetVisible(false);
#endif
}

void RE::resetDefaultSkin(const MotionLoader& skel)
{
  #ifndef NO_OGRE

	if(RE::g_pGlobals->defaultSkins.size()!=0)
	{
		if(RE::g_pGlobals->defaultSkins[skel.getHandle()])
		{
			RE::remove(reinterpret_cast<PLDPrimOgreSkin*>(RE::g_pGlobals->defaultSkins[skel.getHandle()]));
			RE::g_pGlobals->defaultSkins[skel.getHandle()]=NULL;
		}
	}
#endif
}

Ogre::Entity* RE::createTerrain(const char* id, const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ)
{
  #ifndef NO_OGRE

	OBJloader::Mesh mesh;

	OBJloader::createTerrain(mesh, filename, imageSizeX, imageSizeY, sizeX, sizeZ, heightMax, ntexSegX, ntexSegZ);

	return OBJloader::createMeshEntity(mesh, id);
#else
  return NULL;
#endif
}

Ogre::Entity* RE::createPlane(const char* id, m_real width, m_real height, int xsegment, int ysegment, int texSegx, int texSegy)
{
#ifndef NO_OGRE
	try {


	// Floor plane
	Ogre::Plane plane;
	plane.normal = Ogre::Vector3::UNIT_Y;
	plane.d = -0.5;

	TString meshId;
	static int meshIdd=0;
	meshId.format("mesh_%s_%d", id, meshIdd++);

	Ogre::MeshManager::getSingleton().createPlane(meshId.ptr(),
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
		width,height,xsegment,ysegment,true,1,texSegx,texSegy,Ogre::Vector3::UNIT_Z);

	Ogre::Entity* pPlaneEnt = RE::ogreSceneManager()->createEntity( id, meshId.ptr() );

	pPlaneEnt->setCastShadows(false);

	return pPlaneEnt ;
	}
	catch ( Ogre::Exception& e ) {Msg::msgBox(e.getFullDescription().c_str());}
#endif
	return NULL;

  
}

PLDPrimSkin* RE::createOgreSkin(const MotionLoader& skel, Ogre::Entity* entity, const char* mappingTable, bool bCurrPoseAsBindPose, double scale)
{
	PLDPrimSkin* pSkin;
	if (!mappingTable || strlen(mappingTable)==0 || strcmp(mappingTable,"NONE")==0)
		//pSkin=new PLDPrimPreciseOgreSkin((MotionLoader*)&skel, entity, RE::renderer(), scale);
		pSkin=new PLDPrimOgreSkin((MotionLoader*)&skel, entity, NULL, RE::renderer(), bCurrPoseAsBindPose, scale);
	else
		pSkin=new PLDPrimOgreSkin((MotionLoader*)&skel, entity, mappingTable, RE::renderer(), bCurrPoseAsBindPose, scale);
	RE::renderer().addFrameMoveObject(pSkin);
	return pSkin;
}
Ogre::SceneNode* RE::getSceneNode(PLDPrimSkin* skin)
{
	return skin->m_pSceneNode;
}

#ifndef NO_OGRE

LineSegment* RE::createLine()
{
  return new LineSegment(RE::renderer());
}

LineStrip* RE::createThinLine()
{
	return new LineStrip(RE::renderer());
}

Circle* RE::createCircle()
{
	return new Circle(RE::renderer());
}
#endif

FrameSensor* RE::createFrameSensor()
{
	FrameSensor* ps=new FrameSensor();
	RE::renderer().addFrameMoveObject(ps);
	return ps;
}

void RE::removeEntity(const char* id)
{
#ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	try {
		// 있으면 지운다.
		pNode=RE::ogreSceneManager()->getSceneNode(id);
		RE::removeEntity(pNode);
	}
	catch( Ogre::Exception& e )
	{
		// 없으면 okay.
	}
#endif
}

Ogre::Entity* RE::getEntity(Ogre::SceneNode* node)
{
#ifndef NO_OGRE
	Ogre::SceneNode::ObjectIterator obji=node->getAttachedObjectIterator();
	while (obji.hasMoreElements())
	{
		Ogre::MovableObject* mobj = obji.getNext();
		if (mobj->getMovableType() == "Entity")
		{
			return (Ogre::Entity* )mobj;
		}
	}

	Ogre::SceneNode::ChildNodeIterator childi=node->getChildIterator();
	while(childi.hasMoreElements())
	{
		Ogre::Entity* out=RE::getEntity((Ogre::SceneNode*)childi.getNext());
		if(out)
			return out;
	}
#endif
	return NULL;
}

void RE::removeEntity(Ogre::SceneNode* node)
{
#ifndef NO_OGRE
	try
	{
		// assumes that all of node's children are sceneNodes and that all of attached objects are Entity.
		if(node)
		{
			while(node->numChildren())
			{
				RE::removeEntity(dynamic_cast<Ogre::SceneNode* >(node->getChild(0)));
			}

			while(node->numAttachedObjects())
			{
				Ogre::MovableObject* mo = node->detachObject((unsigned short)(0));

				// factory가 존재하는 type은 destroyMovableObject를 사용할수 있다.
				if(mo->getMovableType()=="Entity" ||
					mo->getMovableType()=="SkinEntity" ||
					mo->getMovableType()=="Light" ||
					mo->getMovableType()=="ManualObject")
					RE::ogreSceneManager()->destroyMovableObject(mo);
				else
					delete mo;
				/*
				if(mo->getMovableType()=="Entity")
					RE::ogreSceneManager()->destroyEntity((Ogre::Entity*)mo);
				else if(mo->getMovableType()=="Light")
					RE::ogreSceneManager()->destroyLight((Ogre::Light*)mo);
				else if(mo->getMovableType()=="ManualObject")
					RE::ogreSceneManager()->destroyManualObject((Ogre::ManualObject*)mo);
				else
					delete mo;*/
			}

			//std::cout<<"name :::"<<node->getName()<<std::endl;

#if OGRE_VERSION_MINOR >= 12  || OGRE_VERSION_MAJOR>=13
			((Ogre::SceneNode*)(node->getParent()))->removeAndDestroyChild(node);
#else
			((Ogre::SceneNode*)(node->getParent()))->removeAndDestroyChild(node->getName());
#endif
			node=NULL;
		}
	}
	catch( Ogre::Exception& e )
	{
		Msg::msgBox(e.getFullDescription().c_str());
	}
#endif
}
void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name);

void RE::setColor(Ogre::SimpleRenderable *pRendererble, Color c)
{
  #ifndef NO_OGRE

	switch(c)
	{
	case WHITE:
		_setMaterial(pRendererble, "solidwhite");
		break;
	case BLUE:
		_setMaterial(pRendererble, "solidblue");
		break;
	case GREEN:
		_setMaterial(pRendererble, "solidgreen");
		break;
	case RED:
		_setMaterial(pRendererble, "solidred");
		break;
	case BLACK:
		_setMaterial(pRendererble, "solidblack");
		break;
	}
#endif
}

TString RE::getMaterialName(Color c, bool bSolidColor)
{

	TString matName;
	switch(c)
	{
	case WHITE:
		matName="white";
		break;
	case BLUE:
		matName="blue";
		break;
	case GREEN:
		matName="green";
		break;
	case RED:
		matName="red";
		break;
	case BLACK:
		matName="black";
		break;
	}

	if(bSolidColor) return "solid"+matName;
	return matName;
}

void RE::setEntityColor(Ogre::SceneNode* pNode, Color c)
{
  #ifndef NO_OGRE
	TString mat=getMaterialName(c);
	setMaterialName(pNode, mat);
#endif
}

void RE::setMaterialName(Ogre::SceneNode* pNode, const char* mat)
{
  #ifndef NO_OGRE

	((Ogre::Entity*)pNode->getAttachedObject(0))->setMaterialName(mat);
#endif
}
/*
SurfelClouds* RE::createSurfel(ResourceHandle thandle)
{
	SurfelClouds* surf=(SurfelClouds*)RE::renderer().m_pRenderPrimFactory->CreateRenderPrimByClass(RenderPrimFactory::SURFEL_CLOUDS);
	surf->SetTexture(thandle);
	return surf;
}*/

void RE::remove(FrameMoveObject* pRP)
{
	ASSERT(RE::g_pGlobals->pRenderer);
		
	delete pRP;
}

namespace RE
{
class SkinFactory : public TFactory<PLDPrimSkin>
{
	const MotionLoader& m_skeleton;
	PLDPrimSkinType  m_type;
	public:
		SkinFactory(MotionLoader* skeleton, PLDPrimSkinType  t):m_skeleton(*skeleton), m_type(t){}
		virtual ~SkinFactory() {}
		virtual PLDPrimSkin* create(int index=-1)const 	{ return createSkin(m_skeleton, m_type); }
		virtual void release(PLDPrimSkin* pElt)const 		{ if(pElt) remove(pElt); }	// release
};
}

RE::SkinArray::SkinArray(MotionLoader* skeleton, PLDPrimSkinType  t,bool bHideFoot)
:m_bHideFootPrint(bHideFoot),
m_type(t)
{
	changeFactory(new SkinFactory(skeleton, t));
}

RE::SkinArray::~SkinArray()
{
}


void RE::SkinArray::show()
{
	for(int i=0; i<size(); i++)
	{
		data(i).SetVisible(true);
		//if(m_bHideFootPrint) data(i).m_pFootPrint->SetVisible(false);
	}
}

void RE::SkinArray::hide()
{
	for(int i=0; i<size(); i++)
	{
		if(data(i).m_pTimer)
				data(i).m_pTimer->StopAnim();
		data(i).SetVisible(false);
	}
}


void RE::SkinArray::setSkeleton(MotionLoader* skeleton, PLDPrimSkinType  t, bool bHideFoot)
{
	release();
	m_type=t;
	changeFactory(new SkinFactory(skeleton, m_type));
	m_bHideFootPrint=bHideFoot;
}


void RE::SkinArray::changeColor(Color c)
{
	if(size())
	{
		if(dynamic_cast<PLDPrimBone*>(&data(0)))
		{
			TString matName;
			if(c==GREEN)
				matName="green";

			if(matName.length())
			{
				for(int i=0; i<size(); i++)
				{
					((PLDPrimBone*)(&data(i)))->setMaterial(matName);
				}
			}
		}
		else if(dynamic_cast<PLDPrimLine*>(&data(0)))
		{
			for(int i=0; i<size(); i++)
			{
				((PLDPrimLine*)(&data(i)))->setColor(c);
			}
		}
	}
}

static TString fixMotionFileName(const char* fn)
{
	TString filename;
	filename=fn;
	TString const& defaultPath=RE::renderer().m_pMotionManager->getDefaultPath();

	if(filename.left(1)!="." && filename.left(1)!="/" && filename[1]!=':')
		filename=defaultPath+filename;

	return filename;
}

void RE::motion::concatFromFile(Motion& motion, const char* fn)
{
	if(motion.numFrames())
	{
		int old=motion.numFrames();
		Motion temp;
		motion.skeleton().loadAnimation(temp, fixMotionFileName(fn));
		motion.Concat(&temp);

		motion.setDiscontinuity(old, true);
	}
	else
	{
		motion.skeleton().loadAnimation(motion, fixMotionFileName(fn));
	}
}

void RE::motion::loadAnimation(MotionLoader& skel, Motion& mot, const char* fn)
{
	skel.loadAnimation(mot, fixMotionFileName(fn));
}
int RE::numOutputManager()
{
	return g_traceManagers.size();
}

#ifdef _MSC_VER
#include <windows.h>
#else
#include <unistd.h>
#endif
void RE::usleep(int usec)
{
#ifdef _MSC_VER
	HANDLE timer; 
    LARGE_INTEGER ft; 

    ft.QuadPart = -(10*((__int64)usec)); // Convert to 100 nanosecond interval, negative value indicates relative time

    timer = CreateWaitableTimer(NULL, TRUE, NULL); 
    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0); 
    WaitForSingleObject(timer, INFINITE); 
    CloseHandle(timer); 
#else
	::usleep(usec);
#endif
}
