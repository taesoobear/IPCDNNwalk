#include "stdafx.h"
#include "objectList.h"
#include "../BaseLib/math/conversion.h"
#include "../BaseLib/motion/viewpoint.h"
#include "../MainLib/OgreFltk/Line3D.h"
#include "../MainLib/OgreFltk/MovableText.h"
#include "../MainLib/OgreFltk/renderer.h"

//#include <boost/smart_ptr.hpp>
#include <memory>
#include "../../BaseLib/utility/namedmapsupport.h"
#ifndef NO_OGRE
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreItem.h>
#include "RE.h"
void _setMaterial(Ogre::Renderable* ptr, const char* name);

static vector3 materialToColor(const char* materialName)
{
	TString m(materialName);
	m.makeUpper();
	if(m.findStr(0, "RED")!=-1)
		return vector3(1.0,0.0,0.0);
	else if(m.findStr(0, "BLUE")!=-1)
		return vector3(0.0,0.2,0.8);
	else if(m.findStr(0, "GREEN")!=-1)
		return vector3(0.1,0.9,0.1);
	else if(m.findStr(0, "WHITE")!=-1)
		return vector3(1.0,1.0,1.0);
	else if(m.findStr(0, "GREY")!=-1)
		return vector3(0.5,0.5,0.5);
	else 
		return vector3(0.0,0.0,1.0);
}
Ogre::MovableObject	* createObject(const char* _node_name, const char* typeName, const char* materialName, matrixn const& _data, m_real thickness)
{
	unsigned int objectID=RE::nameToUID(_node_name);
	TString tn=typeName;
	if (_data.rows()==0) 
		return NULL;
	if(tn=="LineList")
	{
		vector3NView data=vec3ViewCol(_data);
		//LineList* line=new LineList();
		ColorBillboardLineList * line=new ColorBillboardLineList (objectID, data.size()/2, 0.6);
		vector3 color=materialToColor(materialName);
		for(int i=0, ni=data.size()/2; i<ni; i++)
		{
			//line->line(i, data[i*2], data[i*2+1]);
			line->line(i, data[i*2], data[i*2+1], color);
		}
		return line;
	}
	else if(tn=="MovableText")
	{
		Ogre::ColourValue c(1,1,1,1);
		if (_data.cols()==4)
		{
			c.r=_data(0,0);
			c.g=_data(0,1);
			c.b=_data(0,2);
			c.a=_data(0,3);
		}
		Ogre::NameValuePairList params;
		//params["name"]=_node_name;
		params["name"]=RE::generateUniqueName();
		params["fontName"]="BlueHighway";
		params["caption"]=materialName;
		TString fh("", (int) thickness);
		params["fontSize"]=fh.ptr();
		/*
		params["colorR"]=c.r;
		params["colorG"]=c.g;
		params["colorB"]=c.b;
		params["colorA"]=c.a;
		*/

		auto* item=RE::ogreSceneManager()->createMovableObject(Ogre::MovableTextFactory::FACTORY_TYPE_NAME, RE::_objectMemoryManager(), & params);
		
		((Ogre::MovableText*)item)->setColor(c);
		((Ogre::MovableText*)item)->setRenderQueueGroup(254u); // last V1_Fast queue.
		return item;
	}
	else if(tn=="Curve")
	{
		vector3NView data=vec3ViewCol(_data);
		ColorBillboardLineList * line=new ColorBillboardLineList (objectID, data.size()-1, 0.6);
		vector3 color=materialToColor(materialName);
		for(int i=0, ni=data.size()-1; i<ni; i++)
		{
			line->line(i, data[i], data[i+1], color);
		}
		return line;
	}
	else if(tn=="LineList2D")
	{
		ColorBillboardLineList * line=new ColorBillboardLineList (objectID, _data.rows()/2, 0.6);
		vector3 color=materialToColor(materialName);
		for(int i=0, ni=_data.rows()/2; i<ni; i++)
		{
			vector3 s(_data[i*2][0], 3, _data[i*2][1]);
			vector3 e(_data[i*2+1][0], 3, _data[i*2+1][1]);
			line->line(i, s, e, color);
		}
		return line;
	}
	else if(tn=="LineList2D_ZX")
	{
		ColorBillboardLineList * line=new ColorBillboardLineList (objectID, _data.rows()/2, 0.6);
		vector3 color=materialToColor(materialName);
		for(int i=0, ni=_data.rows()/2; i<ni; i++)
		{
			vector3 s(_data[i*2][1], 3, _data[i*2][0]);
			vector3 e(_data[i*2+1][1], 3, _data[i*2+1][0]);
			line->line(i, s, e, color);
		}
		return line;
	}

	else if(tn=="BillboardLineList")
	{
		vector3NView data=vec3ViewCol(_data);

		if (thickness==0) thickness=0.7;
		ColorBillboardLineList * line=new ColorBillboardLineList (objectID, data.size()/2, thickness);
		vector3 color=materialToColor(materialName);
		for(int i=0, ni=data.size()/2; i<ni; i++)
		{
			line->line(i, data[i*2], data[i*2+1], color);
		}
		return line;
	}
	else if(tn=="ColorBillboardLineList")
	{
		vector3NView data=vec3ViewCol(_data);

		if (thickness==0) thickness=0.7;
		ColorBillboardLineList * line=new ColorBillboardLineList (objectID, data.size()/3, thickness);
		for(int i=0, ni=data.size()/3; i<ni; i++)
		{
			line->line(i, data[i*3], data[i*3+1], data[i*3+2]);
		}
		//line->setMaterialName(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		return line;
	}
	else if(tn=="ColorWidthBillboardLineList")
	{

		if (thickness==0) thickness=0.7;
		ColorWidthBillboardLineList * line=new ColorWidthBillboardLineList (objectID, _data.rows(), thickness);
		for(int i=0, ni=_data.rows(); i<ni; i++)
		{
			auto v=_data.row(i);
			line->line(i, v.toVector3(0), v.toVector3(3), v.toVector3(6), v(9), v(10), v(11), v(12));
		}
		//line->setMaterialName(materialName);
		return line;
	}
	else if(tn=="BillboardChain")
	{
		if (thickness==0) thickness=0.7;
		Ogre::v1::BillboardChain* line=new Ogre::v1::BillboardChain (
				RE::generateUniqueID(), RE::_objectMemoryManager(), RE::ogreSceneManager(), 
				_data.rows());
		for(int i=0, ni=_data.rows(); i<ni; i++)
		{
			vectornView r=_data.row(i);
			line->addChainElement(0, Ogre::v1::BillboardChain::Element(ToOgre(r.toVector3(0)), r(6), r(7), Ogre::ColourValue(r(3),r(4), r(5), 1), Ogre::Quaternion(1,0,0,0)));
		}
		//line->setMaterialName(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		return line;
	}
	else if(tn.left(8)=="QuadList" )
	{

		if (thickness==0) thickness=10;

		vector3 normal=vector3(0,1,0);

		if(tn.right(1)=="Z")
			normal=vector3(0,0,1);
		else if(tn.right(1)=="X") // incorrect but for backward compatibility
			normal=vector3(0,1,0);
		else if(tn.right(1)=="x")  // use lowcase
			normal=vector3(1,0,0);
		else if(tn.right(1)=="Y")
			normal=vector3(0,1,0);
		else if(tn.right(1)=="V")
		{
			RE::renderer().viewport().m_pViewpoint->GetViewDir(normal);
			normal.normalize();
			normal*=-1;
		}

		QuadList* quads=new QuadList(normal, thickness);
		quads->begin(_data.rows());
		for(int i=0, ni=_data.rows(); i<ni; i++)
		{
			quads->quad(i, _data.row(i).toVector3(0));
		}
		quads->end();
		_setMaterial(quads,materialName);
		return quads;
	}
	else if(tn.left(9)=="PointList" )
	{
		ColorPointList* quads=NULL;
		if(_data.cols()==3){
			vector3NView data=vec3ViewCol(_data);

			quads=new ColorPointList();
			quads->begin(data.size());
			for(int i=0, ni=data.size(); i<ni; i++)
			{
				quads->point(i, vector3(1,1,1), data[i]);
			}
			quads->end();
		}else if(_data.cols()==6){
			vector3NView color=vec3ViewCol(_data,0);
			vector3NView data=vec3ViewCol(_data,3);

			quads=new ColorPointList();
			quads->begin(data.size());
			for(int i=0, ni=data.size(); i<ni; i++)
			{
				quads->point(i, color[i], data[i]);
			}
			quads->end();
		}
		if(materialName&& strlen(materialName)>0)
			_setMaterial(quads, materialName);
		return quads;
	}
	else 
		Msg::error("unknown typename: %s", tn.ptr());
	return NULL;
}

#endif
typedef std::list<Ogre::SceneNode*>::iterator objectIterator;
typedef std::map<TString, objectIterator, cmpTString>::iterator nameIterator;
struct ScheduledSceneNodePtr
{
	Ogre::SceneNode* ptr;
	m_real timeLeft;
};
class objectList_data
{
	public:
		objectList_data(){mRootSceneNode=NULL;}

	//std::list<SceneNodePtr> mObjects;

	//std::map<TString, objectIterator, cmpTString> mObjectNames;

	std::list<ScheduledSceneNodePtr> mScheduledObjects;
	bool isVisible;
	Ogre::SceneNode* mRootSceneNode;
	std::unordered_map<std::string, Ogre::SceneNode*> mUnscheduledObjects;
};
ObjectList::ObjectList()
{ 
	_members=new objectList_data();
	_members->isVisible=true;

}
Ogre::SceneNode* ObjectList::registerNamedEntity(const char* node_name, const char* filename, 
			const char* titleText, double textHeight, vector3 textColor)
{
#ifndef NO_OGRE
	Ogre::SceneNode* node=registerEntity(node_name, filename);
	Ogre::SceneNode* node2=node->createChildSceneNode();

	Ogre::ColourValue c(textColor.x,textColor.y,textColor.z,1);
		Ogre::NameValuePairList params;
		params["name"]="DebugFont";
		params["caption"]=titleText;
	node2->attachObject( new Ogre::MovableText(RE::generateUniqueID(), RE::_objectMemoryManager(), RE::ogreSceneManager(),& params));
	node2->translate(Ogre::Vector3(0, textHeight, 0));
	return node;
#else
	return NULL;
#endif
}

Ogre::MovableObject* ObjectList::_find(const char* node_name)
{
#ifndef NO_OGRE
	Ogre::SceneNode* pNode=findNode(node_name);
	if(pNode)
		return pNode->getAttachedObject((short int)0);
#endif
	return NULL;
}
ObjectList::~ObjectList()
{
	_members->mScheduledObjects.clear();
	_members->mUnscheduledObjects.clear();
	if(RE::rendererValid()&&_members->mRootSceneNode)
	{
		RE::removeEntity(_members->mRootSceneNode);
		RE::renderer().removeFrameMoveObject(this);
	}
	delete _members;
}

Ogre::SceneNode* ObjectList::getCurrRootSceneNode() 
{
#ifndef NO_OGRE
	if (!_members->mRootSceneNode)
	{
		RE::renderer().addFrameMoveObject(this);
		_members->mRootSceneNode=RE::ogreRootSceneNode()->createChildSceneNode(Ogre::SCENE_DYNAMIC);
	}
	return _members->mRootSceneNode;
#else
	return NULL;
#endif
}
Ogre::SceneNode* ObjectList::registerEntity(const char* node_name, const char* filename)
{
	Ogre::SceneNode* pNode=	createSceneNode(node_name);
#ifndef NO_OGRE
	pNode->attachObject(RE::_createEntity(filename));
	pNode->setVisible(_members->isVisible);
#endif
	return pNode;
}

void ObjectList::clear()
{
	//_members->mObjectNames.clear();
	//_members->mObjects.clear();
	if(_members->mRootSceneNode)
		RE::removeEntity(_members->mRootSceneNode);
	_members->mScheduledObjects.clear();
	_members->mUnscheduledObjects.clear();
#ifndef NO_OGRE
	_members->mRootSceneNode=NULL;
	_members->mRootSceneNode=getCurrRootSceneNode();
#endif
}
void ObjectList::setVisible(bool bVisible)
{
#ifndef NO_OGRE
	/*
	std::list<SceneNodePtr>::iterator i;

	for(i=_members->mObjects.begin(); i!=_members->mObjects.end(); ++i)
		(*i)->setVisible(bVisible);
		*/
	if(_members->mRootSceneNode)
		_members->mRootSceneNode->setVisible(bVisible);
	_members->isVisible=bVisible;
#endif
}

Ogre::SceneNode* ObjectList::registerEntity(const char* node_name, const char* filename, const char* materialName)
{
	Ogre::SceneNode* pNode=	createSceneNode(node_name);
#ifndef NO_OGRE
	auto* pEntity=RE::_createEntity(filename);
	if(materialName) 
		pEntity->setDatablockOrMaterialName(materialName);
	pNode->attachObject(pEntity);
	pNode->setVisible(_members->isVisible);
#endif
	return pNode;
}
Ogre::SceneNode* ObjectList::registerEntity(const char* node_name, Ogre::Item* pObject)
{
	return registerObject(node_name, pObject);
}
// delete로 지울수 있는 무슨 오브젝트든 가능. (pObject는 반드시 new해서 집어넣는다.)
Ogre::SceneNode* ObjectList::registerObject(const char* node_name, Ogre::MovableObject* pObject)
{
	Ogre::SceneNode* pNode=createSceneNode(node_name);
#ifndef NO_OGRE
	if(!pObject) return pNode;
	pNode->attachObject(pObject);
	pNode->setVisible(_members->isVisible);
#endif
	return pNode;
}

Ogre::SceneNode* ObjectList::registerObject(const char* node_name, const char* typeName, const char* materialName, vector3N const& _data, m_real thickness)
{
	return registerObject(node_name, typeName, materialName, matView(_data), thickness);
}
Ogre::SceneNode* ObjectList::registerObject(const char* node_name, const char* typeName, const char* materialName, matrixn const& _data, m_real thickness)
{
#ifdef NO_OGRE
    return registerObject(node_name, NULL);
#else
	return registerObject(node_name, createObject(node_name, typeName, materialName, _data, thickness));
#endif
	return NULL;
}
Ogre::SceneNode* ObjectList::registerObjectScheduled(m_real time, const char* typeName, const char* materialName, matrixn const& _data, m_real thickness)
{
#ifdef NO_OGRE
    return registerObjectScheduled(NULL, time);
#else
	return registerObjectScheduled( 
			createObject(RE::generateUniqueName(), typeName, materialName, _data, thickness),
			time
			);
#endif
	return NULL;
}

Ogre::SceneNode* ObjectList::findNode(const char* node_name)
{
	Ogre::SceneNode* pNode=NULL;
	if(!_members->mRootSceneNode) return pNode;

#ifndef NO_OGRE
	try {
		auto i=_members->mUnscheduledObjects.find(node_name);
		if(i!=_members->mUnscheduledObjects.end())
		{
			return i->second;
		}
	}
	catch( Ogre::Exception& e )
	{
		printf("%s\n", e.what());
		// 없으면 okay.
	}
#endif

	return pNode;

	/*
	nameIterator i;
	if((i=_members->mObjectNames.find(node_name))==_members->mObjectNames.end()) return NULL;
	return i->second->get();
	*/
}

void ObjectList::erase(const char* node_name)
{
	Ogre::SceneNode* pNode=findNode(node_name);
	if(pNode)
	{
		_members->mUnscheduledObjects.erase(std::string(node_name));
		RE::removeEntity(pNode);
	}
	/*
	nameIterator i=_members->mObjectNames.find(node_name);
	if(i==_members->mObjectNames.end())
	{
//		Msg::print2("no object %s", node_name);
	}
	else
	{
		_members->mObjects.erase(i->second);
		_members->mObjectNames.erase(i);
	}
	*/
}

Ogre::SceneNode* ObjectList::createSceneNode(const char* node_name)
{
	Ogre::SceneNode* pNode=findNode(node_name);
#ifndef NO_OGRE
	if(pNode) {
		RE::removeEntity(pNode);
		_members->mUnscheduledObjects.erase(node_name);
	}
	//RE::removeEntity(node_name); // I  do not know why this is necessary.. but sometimes it is necessary...
	pNode=getCurrRootSceneNode()->createChildSceneNode(Ogre::SCENE_DYNAMIC);
	_members->mUnscheduledObjects.insert({std::string(node_name), pNode});
#endif

	return pNode;
}


Ogre::SceneNode* ObjectList::registerObjectScheduled(Ogre::MovableObject* pObject, m_real destroyTime)
{
	std::list<ScheduledSceneNodePtr>::iterator i=_members->mScheduledObjects.end();
	i=_members->mScheduledObjects.insert(i, ScheduledSceneNodePtr());
#ifndef NO_OGRE
	Ogre::SceneNode* pNode=getCurrRootSceneNode()->createChildSceneNode(Ogre::SCENE_DYNAMIC);
	pNode->attachObject(pObject);
	pNode->setVisible(_members->isVisible);
#else
	Ogre::SceneNode* pNode=createSceneNode(RE::generateUniqueName());
#endif
	i->ptr=pNode;
	i->timeLeft=destroyTime;
	return pNode;
}



Ogre::SceneNode* ObjectList::registerEntityScheduled(const char* filename, m_real destroyTime)
{
 #ifdef NO_OGRE
  return registerObjectScheduled(NULL, destroyTime);
#else
	return registerObjectScheduled(RE::_createEntity(filename), destroyTime);
#endif
}


void ObjectList::eraseAllScheduled()
{
	_members->mScheduledObjects.clear();
}
int ObjectList::FrameMove(float fElapsedTime)
{
	for(std::list<ScheduledSceneNodePtr>::iterator i=_members->mScheduledObjects.begin();
		i!=_members->mScheduledObjects.end(); )
	{
		i->timeLeft-=fElapsedTime;

		if(i->timeLeft<0.0)
		{
			RE::removeEntity(i->ptr);
			i=_members->mScheduledObjects.erase(i);
		}
		else
			i++;
	}
	return 1;
}
void ObjectList::drawSphere(vector3 const& pos, const char* nameid, const char* _materialName, double _scale)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* comEntity=registerEntity(nameid, "sphere1010.mesh", _materialName);
	if (comEntity )
	{
		comEntity->setScale(_scale, _scale, _scale);
		comEntity->setPosition(pos.x, pos.y, pos.z);
	}
#endif
}
void ObjectList::drawLine(vector3 const& startpos, vector3 const& endpos, const char* _nameid, const char* _materialName)
{
#ifndef NO_OGRE
   vector3N lines;
   lines.setSize(2);
   lines(0)=startpos;
   lines(1)=endpos;
   TString nameid;
   assert(startpos.x==startpos.x);
   
   if (_nameid)
	   nameid=_nameid;
   else
      nameid=RE::generateUniqueName();

   TString materialname;
   if (_materialName)
	   materialname=_materialName;
   else
	   materialname="solidred";
   registerObject(nameid, "LineList", materialname.ptr(), matView(lines),0);
#endif
}
void ObjectList::drawAxes(transf const& tf, const char* nameid, double _scale, double posScale)
{
 #ifndef NO_OGRE
	Ogre::SceneNode* comEntity=registerEntity(nameid, "axes.mesh");
	if (comEntity )
	{
		comEntity->setScale(_scale, _scale, _scale);
		vector3 pos=tf.translation*posScale;
		comEntity->setPosition(pos.x, pos.y, pos.z);
	}
	quater r=tf.rotation;
	comEntity->setOrientation(r.w, r.x,r.y, r.z);
#endif
}
