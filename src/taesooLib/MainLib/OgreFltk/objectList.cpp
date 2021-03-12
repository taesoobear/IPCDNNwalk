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
void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name);

Ogre::MovableObject	* createObject(const char* node_name, const char* typeName, const char* materialName, matrixn const& _data, m_real thickness)
{
	TString tn=typeName;
	if(tn=="LineList")
	{
		vector3NView data=vec3ViewCol(_data);
		LineList* line=new LineList();
		line->begin(data.size()/2);
		for(int i=0, ni=data.size()/2; i<ni; i++)
		{
			line->line(i, data[i*2], data[i*2+1]);
		}
		line->end();
		_setMaterial(line, materialName);
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
		return new Ogre::MovableText(RE::generateUniqueName().ptr(), materialName, "BlueHighway", thickness,c);
	}
	else if(tn=="Curve")
	{
		vector3NView data=vec3ViewCol(_data);
		LineList* line=new LineList();
		line->begin(data.size()-1);
		for(int i=0, ni=data.size()-1; i<ni; i++)
		{
			line->line(i, data[i], data[i+1]);
		}
		line->end();
		_setMaterial(line, materialName);
		return line;
	}
	else if(tn=="LineList2D")
	{
		LineList* line=new LineList();
		line->begin(_data.rows()/2);
		for(int i=0, ni=_data.rows()/2; i<ni; i++)
		{
			vector3 s(_data[i*2][0], 3, _data[i*2][1]);
			vector3 e(_data[i*2+1][0], 3, _data[i*2+1][1]);
			line->line(i, s, e);
		}
		line->end();
		_setMaterial(line, materialName);
		return line;
	}
	else if(tn=="LineList2D_ZX")
	{
		LineList* line=new LineList();
		line->begin(_data.rows()/2);
		for(int i=0, ni=_data.rows()/2; i<ni; i++)
		{
			vector3 s(_data[i*2][1], 3, _data[i*2][0]);
			vector3 e(_data[i*2+1][1], 3, _data[i*2+1][0]);
			line->line(i, s, e);
		}
		line->end();
		_setMaterial(line, materialName);
		return line;
	}

	else if(tn=="BillboardLineList")
	{
		vector3NView data=vec3ViewCol(_data);

		if (thickness==0) thickness=0.7;
		BillboardLineList * line=new BillboardLineList (TString(node_name)+"__bbll", data.size()/2, thickness);
		for(int i=0, ni=data.size()/2; i<ni; i++)
		{
			line->line(i, data[i*2], data[i*2+1]);
		}
		line->setMaterialName(materialName);
		return line;
	}
	else if(tn=="ColorBillboardLineList")
	{
		vector3NView data=vec3ViewCol(_data);

		if (thickness==0) thickness=0.7;
		ColorBillboardLineList * line=new ColorBillboardLineList (TString(node_name)+"__cbbll", data.size()/3, thickness);
		for(int i=0, ni=data.size()/3; i<ni; i++)
		{
			line->line(i, data[i*3], data[i*3+1], data[i*3+2]);
		}
		line->setMaterialName(materialName);
		return line;
	}
	else if(tn=="ColorWidthBillboardLineList")
	{
		vector3NView data=vec3ViewCol(_data);

		if (thickness==0) thickness=0.7;
		ColorWidthBillboardLineList * line=new ColorWidthBillboardLineList (TString(node_name)+"__cbbll", data.size()/3, thickness);
		for(int i=0, ni=data.size()/4; i<ni; i++)
		{
			line->line(i, data[i*4], data[i*4+1], data[i*4+2], data[i*4+3][0]);
		}
		//line->setMaterialName(materialName);
		return line;
	}
	else if(tn=="BillboardChain")
	{
		if (thickness==0) thickness=0.7;
		Ogre::BillboardChain* line=new Ogre::BillboardChain ((TString(node_name)+"__cbbll").ptr(), _data.rows());
		for(int i=0, ni=_data.rows(); i<ni; i++)
		{
			vectornView r=_data.row(i);
#if OGRE_VERSION_MINOR >= 8 
			line->addChainElement(0, Ogre::BillboardChain::Element(ToOgre(r.toVector3(0)), r(6), r(7), Ogre::ColourValue(r(3),r(4), r(5), 1), Ogre::Quaternion(1,0,0,0)));
#else
			line->addChainElement(0, Ogre::BillboardChain::Element(ToOgre(r.toVector3(0)), r(6), r(7), Ogre::ColourValue(r(3),r(4), r(5), 1)));
#endif
		}
		line->setMaterialName(materialName);
		return line;
	}
	else if(tn.left(8)=="QuadList" )
	{
		vector3NView data=vec3ViewCol(_data);

		if (thickness==0) thickness=10;

		vector3 normal=vector3(0,1,0);

		if(tn.right(1)=="Z")
			normal=vector3(0,0,1);
		else if(tn.right(1)=="X")
			normal=vector3(0,1,0);
		else if(tn.right(1)=="V")
		{
			RE::renderer().viewport().m_pViewpoint->GetViewDir(normal);
			normal.normalize();
			normal*=-1;
		}

		QuadList* quads=new QuadList(normal, thickness);
		quads->begin(data.size());
		for(int i=0, ni=data.size(); i<ni; i++)
		{
			quads->quad(i, data[i]);
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
		return quads;
	}
	return NULL;
}

#endif
typedef std::shared_ptr<Ogre::SceneNode> SceneNodePtr;
typedef std::list<SceneNodePtr>::iterator objectIterator;
typedef std::map<TString, objectIterator, cmpTString>::iterator nameIterator;
struct ScheduledSceneNodePtr
{
	SceneNodePtr ptr;
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
};
ObjectList::ObjectList()
{ 
	_members=new objectList_data();
	static int _objlist_unique_id=1;
	_uniqueId.format("objl_%d", _objlist_unique_id++);
	_members->isVisible=true;

}
Ogre::SceneNode* ObjectList::registerNamedEntity(const char* node_name, const char* filename, 
			const char* titleText, double textHeight, vector3 textColor)
{
#ifndef NO_OGRE
	Ogre::SceneNode* node=registerEntity(node_name, filename);
	Ogre::SceneNode* node2=node->createChildSceneNode();

	TString entity_name;
	entity_name.format("_mt_object%s_%s", _uniqueId.ptr(),  node_name);
	Ogre::ColourValue c(textColor.x,textColor.y,textColor.z,1);
	node2->attachObject(
			new Ogre::MovableText(entity_name.ptr(), titleText, "BlueHighway", textHeight,c));
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
	if(RE::rendererValid()&&_members->mRootSceneNode)
	{
		RE::removeEntity(_members->mRootSceneNode);
		RE::renderer().removeFrameMoveObject(this);
	}
	_members->mScheduledObjects.clear();
	delete _members;
}

Ogre::SceneNode* ObjectList::getCurrRootSceneNode() 
{
#ifndef NO_OGRE
	if (!_members->mRootSceneNode)
	{
		RE::renderer().addFrameMoveObject(this);
		_members->mRootSceneNode=RE::ogreRootSceneNode()->createChildSceneNode(_uniqueId.ptr());
	}
	return _members->mRootSceneNode;
#else
	return NULL;
#endif
}
Ogre::SceneNode* ObjectList::registerEntity(const char* node_name, const char* filename)
{
	TString entity_name;
	entity_name.format("_entity_%s_%s", _uniqueId.ptr(),  node_name);
	Ogre::SceneNode* pNode=	createSceneNode(node_name);
#ifndef NO_OGRE
	pNode->attachObject(RE::ogreSceneManager()->createEntity(entity_name.ptr(), filename));
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
	TString entity_name;
	entity_name.format("_entity_%s_%s", _uniqueId.ptr(),  node_name);

	Ogre::SceneNode* pNode=	createSceneNode(node_name);
#ifndef NO_OGRE
	Ogre::Entity* pEntity=RE::ogreSceneManager()->createEntity(entity_name.ptr(), filename);
	if(materialName)
		pEntity->setMaterialName(materialName);
	pNode->attachObject(pEntity);
	pNode->setVisible(_members->isVisible);
#endif
	return pNode;
}
Ogre::SceneNode* ObjectList::registerEntity(const char* node_name, Ogre::Entity* pObject)
{
	return registerObject(node_name, pObject);
}
// delete로 지울수 있는 무슨 오브젝트든 가능. (pObject는 반드시 new해서 집어넣는다.)
Ogre::SceneNode* ObjectList::registerObject(const char* node_name, Ogre::MovableObject* pObject)
{
	Ogre::SceneNode* pNode=createSceneNode(node_name);
#ifndef NO_OGRE
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
			createObject(RE::generateUniqueName().ptr(), typeName, materialName, _data, thickness),
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
		pNode=(Ogre::SceneNode*)_members->mRootSceneNode->getChild(node_name);
	}
	catch( Ogre::Exception& e )
	{
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
		RE::removeEntity(pNode);
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
/*
	// erase first.
	{
		nameIterator i=_members->mObjectNames.find(node_name);
		if(i!=_members->mObjectNames.end())
		{
			_members->mObjects.erase(i->second);
			_members->mObjectNames.erase(i);
		}
	}

	objectIterator i=_members->mObjects.end();
	i=_members->mObjects.insert(i, SceneNodePtr());
	Ogre::SceneNode* pNode=createSceneNode(_uniqueId+node_name);
	// automatic removal
	i->reset(pNode, (void(*)(Ogre::SceneNode*))RE::removeEntity);
	_members->mObjectNames[node_name]=i;
	*/
	Ogre::SceneNode* pNode=findNode(node_name);
#ifndef NO_OGRE
	if(pNode) {
		RE::removeEntity(pNode);
	}
	RE::removeEntity(node_name); // I  do not know why this is necessary.. but sometimes it is necessary...
	pNode=getCurrRootSceneNode()->createChildSceneNode(node_name);
#endif

	return pNode;
}


Ogre::SceneNode* ObjectList::registerObjectScheduled(Ogre::MovableObject* pObject, m_real destroyTime)
{
	std::list<ScheduledSceneNodePtr>::iterator i=_members->mScheduledObjects.end();
	i=_members->mScheduledObjects.insert(i, ScheduledSceneNodePtr());
	Ogre::SceneNode* pNode=createSceneNode(RE::generateUniqueName());
#ifndef NO_OGRE
	pNode->attachObject(pObject);
	pNode->setVisible(_members->isVisible);
#endif
	// automatic removal
	i->ptr.reset(pNode, (void(*)(Ogre::SceneNode*))RE::removeEntity);
	i->timeLeft=destroyTime;
	return pNode;
}



Ogre::SceneNode* ObjectList::registerEntityScheduled(const char* filename, m_real destroyTime)
{
 #ifdef NO_OGRE
  return registerObjectScheduled(NULL, destroyTime);
#else
	return registerObjectScheduled(RE::ogreSceneManager()->createEntity(RE::generateUniqueName().ptr(), filename), destroyTime);
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
			i=_members->mScheduledObjects.erase(i);
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
   registerObject(nameid.ptr(), "LineList", materialname.ptr(), matView(lines),0);
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
