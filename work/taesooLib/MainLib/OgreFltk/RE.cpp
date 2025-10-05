#include "stdafx.h"
#include "RE.h"
#include "renderer.h"
#include "../BaseLib/motion/Motion.h"
#include "../BaseLib/motion/MotionWrap.h"
#include "../BaseLib/motion/viewpoint.h"
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
#include "OgreItem.h"
#include "OgreMeshManager2.h"
#include "objectList.h"
#include "../../BaseLib/math/conversion.h"
#include <OgreMesh.h>
#endif
#include "pldprimskin_impl.h"
namespace RE
{
	Globals* g_pGlobals=NULL;
}

bool useSeperateOgreWindow();
bool RE::useSeperateOgreWindow()
{
	return ::useSeperateOgreWindow();
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
return RE::renderer().viewport().mScene->getRootSceneNode(Ogre::SCENE_DYNAMIC);
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
		//pSkin=new PLDPrimLine((MotionLoader*)(&skel), RE::renderer());
		pSkin=new PLDPrimThickLine((MotionLoader*)(&skel), RE::renderer());
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
	//if(RE::g_pGlobals->defaultSkins.size() && RE::g_pGlobals->defaultSkins[skel.getHandle()])
	//	pSkin=createSkin(*reinterpret_cast<PLDPrimOgreSkin*>(RE::g_pGlobals->defaultSkins[skel.getHandle()]));
	//else
	{
		//pSkin=new PLDPrimCyl((MotionLoader*)(&skel), RE::renderer());
		//pSkin=new PLDPrimLine((MotionLoader*)(&skel), RE::renderer());
		pSkin=new PLDPrimThickLine((MotionLoader*)(&skel), RE::renderer());
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


/*
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
*/

void RE::moveEntity(Ogre::SceneNode* node, vector3 const& p)
{
 #ifndef NO_OGRE
	RE::resetToInitialState(node);
	node->translate(ToOgre(p));
#endif
}

void RE::moveEntity(Ogre::SceneNode* node, quater const& q, vector3 const& p)
{
   #ifndef NO_OGRE

	RE::resetToInitialState(node);
	node->rotate(ToOgre(q));
	node->translate(ToOgre(p));
#endif
}

void RE::moveEntity(Ogre::SceneNode* node, vector3 const& scale, vector3 const& p)
{
 #ifndef NO_OGRE
	RE::resetToInitialState(node);
	node->scale(ToOgre(scale));
	node->translate(ToOgre(p));
#endif
}
void RE::moveEntity(Ogre::SceneNode* node, double scale, vector3 const& p)
{
 #ifndef NO_OGRE
	RE::resetToInitialState(node);
	node->scale(scale, scale, scale);
	node->translate(ToOgre(p));
#endif
}

void RE::moveEntity(Ogre::SceneNode* node, vector3 const& scale, quater const& q, vector3 const& p)
{
 #ifndef NO_OGRE
	RE::resetToInitialState(node);
	node->scale(ToOgre(scale));
	node->rotate(ToOgre(q));
	node->translate(ToOgre(p));
#endif
}
void RE::moveEntity(Ogre::SceneNode* node, double scale, quater const& q, vector3 const& p)
{
 #ifndef NO_OGRE
	node->setScale(scale, scale, scale);
	node->setOrientation(ToOgre(q));
	node->setPosition(ToOgre(p));
#endif
}

Ogre::SceneNode* RE::createChildSceneNode(Ogre::SceneNode* parent, const char* node_name)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	if (pNode=RE::getSceneNode(node_name))
		RE::removeEntity(pNode);

	pNode=parent->createChildSceneNode();
	pNode->setName(node_name);
	RE::g_pGlobals->mNamedSceneNodes.insert({std::string(node_name), pNode});
	pNode->setVisible(true);

	return pNode;
#else
  return NULL;
#endif
}
#ifndef NO_OGRE
unsigned int RE::nameToUID(const char* node_name)
{
	Ogre::IdString temp(node_name);
	return temp.mHash;
}
#endif

Ogre::SceneNode* RE::getSceneNode(const char* node_name)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	auto& named_nodes=RE::g_pGlobals->mNamedSceneNodes;
	auto i=named_nodes.find(node_name);
	if (i!=named_nodes.end())
		return i->second;
	return NULL;
#else
	return NULL;
#endif
}

Ogre::SceneNode* RE::createSceneNode(const char* node_name)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* pNode=NULL;
	pNode=RE::getSceneNode(node_name);
	if(pNode)
		RE::removeEntity(pNode);

	pNode=RE::ogreRootSceneNode()->createChildSceneNode();
	pNode->setName(node_name);
	RE::g_pGlobals->mNamedSceneNodes.insert({std::string(node_name), pNode});
	pNode->setVisible(true);

	return pNode;
#else 
    return NULL;
#endif
}

Ogre::Item* RE::_createEntity( const char* filename)
{
#ifndef NO_OGRE
	Ogre::MeshPtr pMesh;
	// Get mesh (load if required)
	try{
		pMesh = Ogre::MeshManager::getSingleton().load( filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	}
	catch ( Ogre::Exception& e )
	{
		if(TString(e.what()).findStr(0, "implementation for mesh version")!=-1)
		{
			Ogre::String v2meshId(filename);
			v2meshId+="@v2";
			if(Ogre::MeshManager::getSingleton().resourceExists(v2meshId))
				pMesh = Ogre::MeshManager::getSingleton().load( v2meshId, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
			/*
			else if(filename[0]=='h' && strcmp(&filename[3], ".mesh")==0)
			{
				printf("house mesh detected\n");
				vector3N lines;
				Ogre::v1::MeshPtr pMesh1= Ogre::v1::MeshManager::getSingleton().load( filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
				Msg::verify(!pMesh1->sharedVertexData[Ogre::VpNormal],"has shared vertex");
				for(int imesh=0; imesh<pMesh1->getNumSubMeshes(); imesh++)
				{
					auto* vertexData=pMesh1->getSubMesh(imesh)->vertexData[Ogre::VpNormal];
					auto& vertexDecl=vertexData->vertexDeclaration->getElements();
					auto iter=vertexDecl.begin();
					iter++;
					Msg::verify(iter->getSemantic()==Ogre::VES_NORMAL, "no normal?");

					Msg::verify(vertexData->vertexBufferBinding->getBufferCount()==1, "seperate buffers?");
					const Ogre::v1::HardwareVertexBufferSharedPtr &vBuffer = vertexData->vertexBufferBinding-> getBuffer( 0 );
					Ogre::v1::HardwareBufferLockGuard srcLock;
					srcLock.lock( vBuffer, Ogre::v1::HardwareBuffer::HBL_NORMAL );
					float* vertex=static_cast<float*>(srcLock.pData);

					for(int v=0; v<vertexData->vertexCount; v++)
					{
						vector3 pos(vertex[0], vertex[1], vertex[2]);
						vector3 normal(vertex[3], vertex[4], vertex[5]);
						lines.pushBack(pos);
						lines.pushBack(pos+normal*10);
						// flip normal
						//vertex[3]*=-1.f;
						//vertex[4]*=-1.f;
						//vertex[5]*=-1.f;
						vertex+=vBuffer->getVertexSize()/sizeof(float);
					}
					srcLock.unlock();
				}
				static ObjectList normalVis;
				std::cout<<lines.size()<<"::::"<< lines(0) <<std::endl<<lines(1)<<std::endl;
				normalVis.registerObject("normals2", "LineList", "solidblue", matView(lines));

				pMesh = Ogre::MeshManager::getSingleton().createByImportingV1( v2meshId, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pMesh1.get(), true, true, true );
			}
			*/
			else
			{
				Ogre::v1::MeshPtr pMesh1= Ogre::v1::MeshManager::getSingleton().load( filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
				pMesh = Ogre::MeshManager::getSingleton().createByImportingV1( v2meshId, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pMesh1.get(), true, true, true );
			}
		}
		else
			throw e;
	}
	Ogre::Item *item = RE::ogreSceneManager()->createItem( pMesh, Ogre::SCENE_DYNAMIC );
	return item;
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
	Ogre::SceneNode* pNode;		
	pNode=RE::getSceneNode(id);
	// 같은 이름의 SceneNod가 있으면 그냥 재활용한다.
	if(pNode)
		RE::resetToInitialState(pNode);
	else
	{
		pNode=RE::createChildSceneNode(parentNode, id);
		//Ogre::Item *entity = RE::ogreSceneManager()->createItem(filename);
		Ogre::Item *entity = RE::_createEntity(filename);
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
	Ogre::SceneNode* pNode;
		// 같은 이름의 SceneNod가 있으면 그냥 재활용한다.
	pNode=RE::getSceneNode(id);
	if(pNode)
		RE::resetToInitialState(pNode);
	else
	{
		pNode=RE::createChildSceneNode(parentNode, id);
		//Ogre::Item *entity = RE::ogreSceneManager()->createItem(filename);
		Ogre::Item *entity = RE::_createEntity(filename);
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
	Ogre::SceneNode* pNode=NULL;
	// 있으면 그냥 재활용한다.
	pNode=RE::getSceneNode(node_name);
	if(pNode)
	{
		RE::resetToInitialState(pNode);
		auto* obj=pNode->getAttachedObject(0);
		pNode->detachObject(obj);
		RE::ogreSceneManager()->destroyMovableObject(obj);
	}
	else
	{
		pNode=RE::createSceneNode(node_name);
	}

		Ogre::NameValuePairList params;
		params["fontName"]=fontName;
		params["caption"]=contents;
		params["fontSize"]=charHeight;

	Ogre::MovableText* entity=new Ogre::MovableText(RE::generateUniqueID(), RE::_objectMemoryManager(), RE::ogreSceneManager(), &params);
	entity->setColor(color);
	pNode->attachObject(entity);
	pNode->setVisible(true);

	return pNode;
#else
  return NULL;
#endif
}

unsigned int RE::generateUniqueID()
{
#ifndef NO_OGRE
	return Ogre::Id::generateNewId<Ogre::MovableObject>();
#else
	static int uid=0;
	return uid++;
#endif
}
void RE::resetToInitialState(Ogre::SceneNode* pNode)
{
#ifndef NO_OGRE
	pNode->setScale(1.0,1.0,1.0);
	pNode->resetOrientation();
	pNode->setPosition(0.0,0.0,0.0);
#endif
}
Ogre::ObjectMemoryManager* RE::_objectMemoryManager()
{
#ifndef NO_OGRE
	return &RE::ogreSceneManager()->_getEntityMemoryManager(Ogre::SCENE_DYNAMIC );
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


Ogre::Item* RE::createTerrain(const char* id, const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ)
{
  #ifndef NO_OGRE

	OBJloader::Mesh mesh;

	OBJloader::createTerrain(mesh, filename, imageSizeX, imageSizeY, sizeX, sizeZ, heightMax, ntexSegX, ntexSegZ);

	return OBJloader::createMeshEntity(mesh, id);
#else
  return NULL;
#endif
}

Ogre::Item* RE::createPlane(const char* id, m_real width, m_real height, int xsegment, int ysegment, int texSegx, int texSegy)
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

	Ogre::v1::MeshPtr planeMeshV1=Ogre::v1::MeshManager::getSingleton().createPlane((meshId+"___v1").ptr(),
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
		width,height,xsegment,ysegment,true,1,texSegx,texSegy,Ogre::Vector3::UNIT_Z,
                                            Ogre::v1::HardwareBuffer::HBU_STATIC,
                                            Ogre::v1::HardwareBuffer::HBU_STATIC );

	Ogre::MeshPtr planeMesh = Ogre::MeshManager::getSingleton().createByImportingV1(
			meshId.ptr(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			planeMeshV1.get(), true, true, true );


	Ogre::Item *item = RE::ogreSceneManager()->createItem( planeMesh, Ogre::SCENE_DYNAMIC );
	return item;
	}
	catch ( Ogre::Exception& e ) {Msg::msgBox(e.getFullDescription().c_str());}
#endif
	return NULL;

  
}

Ogre::SceneNode* RE::getSceneNode(PLDPrimSkin* skin)
{
	return skin->m_pSceneNode;
}

#ifndef NO_OGRE



Circle* RE::createCircle()
{
	return NULL;
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
	// 있으면 지운다.
	pNode=RE::getSceneNode(id);
	if(pNode) RE::removeEntity(pNode);
#endif
}

Ogre::v1::Entity* RE::getEntity(Ogre::SceneNode* node)
{
#ifndef NO_OGRE
	for (int i=0, ni=node->numAttachedObjects(); i<ni; i++)
	{
		Ogre::MovableObject* mobj = node->getAttachedObject(i);
		if (mobj->getMovableType() == "Entity")
		{
			return (Ogre::v1::Entity* )mobj;
		}
	}

	auto childi=node->getChildIterator();
	while(childi.hasMoreElements())
	{
		Ogre::v1::Entity* out=RE::getEntity((Ogre::SceneNode*)childi.getNext());
		if(out)
			return out;
	}
#endif
	return NULL;
}
Ogre::Item* RE::getItem(Ogre::SceneNode* node)
{
#ifndef NO_OGRE
	for (int i=0, ni=node->numAttachedObjects(); i<ni; i++)
	{
		Ogre::MovableObject* mobj = node->getAttachedObject(i);
		if (mobj->getMovableType() == "Item")
		{
			return (Ogre::Item* )mobj;
		}
	}

	auto childi=node->getChildIterator();
	while(childi.hasMoreElements())
	{
		Ogre::Item* out=RE::getItem((Ogre::SceneNode*)childi.getNext());
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

			int errc=0;
			while(node->numAttachedObjects()-errc>0)
			{

				Ogre::MovableObject* mo = node->getAttachedObject(0);
				if(!mo) {
					printf("null???\n");
					errc++;
				}
				else {
					node->detachObject(mo);

					// factory가 존재하는 type은 destroyMovableObject를 사용할수 있다.
					Ogre::String mt=mo->getMovableType();
					if(mt=="Entity" ||
							mt=="SkinEntity" ||
							mt=="Light" ||
							mt=="ManualObject"||
							mt=="Item")
						RE::ogreSceneManager()->destroyMovableObject(mo);
					else if(mt=="MovableText")
					{
						// memory bug. destroy doesns't work. so currently a memory leak.
					}
					else
					{
						//printf("deleting movable object %s\n", mt.c_str());
						delete mo;
					}
				}
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

			if (node->getName().length()>0)
				RE::g_pGlobals->mNamedSceneNodes.erase(node->getName());
			((Ogre::SceneNode*)(node->getParent()))->removeAndDestroyChild(node);
			node=NULL;
		}
	}
	catch( Ogre::Exception& e )
	{
		Msg::msgBox(e.getFullDescription().c_str());
	}
#endif
}
void _setMaterial(Ogre::v1::SimpleRenderable* ptr, const char* name);

void RE::setColor(Ogre::v1::SimpleRenderable *pRendererble, Color c)
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

	((Ogre::v1::Entity*)pNode->getAttachedObject(0))->setMaterialName(mat);
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
		/*todo2
		else if(dynamic_cast<PLDPrimLine*>(&data(0)))
		{
			for(int i=0; i<size(); i++)
			{
				((PLDPrimLine*)(&data(i)))->setColor(c);
			}
		}
		*/
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
#ifndef NO_OGRE
#include "TraceManager.h"
#endif
OgreTraceManager* RE::createTraceManager()
{
#ifndef NO_OGRE
	OgreTraceManager* otm=NULL;
	int _w=RE::renderer().viewport().m_pViewpoint->m_iWidth;
	int _h=RE::renderer().viewport().m_pViewpoint->m_iHeight;

	otm=new OgreTraceManager(0, 0, _w, _h);
	return otm;
#else
	return NULL;
#endif
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
extern ConfigTable config;
OgreRenderer* RE::_createRenderer(int& w, int &rw)
{
	OgreRenderer* renderer=NULL;
	try {
		renderer=new OgreRenderer();
	}
#ifndef NO_OGRE
	catch( Ogre::Exception& e ) 
	{
		std::cout<<e.getFullDescription().c_str()<<std::endl;

		Msg::msgBox(e.getFullDescription().c_str());

	}	
#endif
	catch(std::exception& e)
	{
		std::cout<<e.what()<<std::endl;
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(char * error)
	{
		Msg::msgBox("%s", error);
	}
	if (!config.Find("renderer_width"))
		return renderer;
	int DEFAULT_RENDERER_WIDTH=config.GetInt("renderer_width");
	int DEFAULT_WIDTH=DEFAULT_RENDERER_WIDTH+config.GetInt("right_panel_width");
	if (DEFAULT_RENDERER_WIDTH<20)
	{
		rw=DEFAULT_RENDERER_WIDTH; // seperate win
		w=DEFAULT_WIDTH;
	}
	return renderer;
}

void RE::buildEdgeList(const char* meshName)
{
#ifndef NO_GUI
	/*
	Ogre::String groupName = Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME;
	// Get mesh (load if required)
	Ogre::MeshPtr pmesh = Ogre::v1::MeshManager::getSingleton().load(meshName, groupName);
	pmesh->freeEdgeList();
	//pmesh->prepareForShadowVolume();
	pmesh->buildEdgeList();
	//pmesh-> setAutoBuildEdgeLists(true);
	//pmesh->load();
	//*/
#endif
}
int RE::getOgreVersionMinor()
{
#ifdef NO_GUI
	return 7;
#else
#if OGRE_VERSION_MAJOR>=13 ||  OGRE_VERSION_MAJOR==2 || OGRE_VERSION_MAJOR==3
	return OGRE_VERSION_MAJOR;
#endif
	return OGRE_VERSION_MINOR;
#endif
}
