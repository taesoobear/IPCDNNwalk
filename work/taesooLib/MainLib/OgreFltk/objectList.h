#ifndef _OBJECTLIST_H_
#define _OBJECTLIST_H_

#if _MSC_VER>1000
#pragma once
#endif
class objectList_data;
#include "framemoveobject.h"
namespace Ogre
{
	namespace v1{
	class Entity;
	}
	class MovableObject;
	class SceneNode;
}
// ObjectList는 rootSceneNode의 child SceneNode 한개에 해당한다. 
// ObjectList에서 생성한 개체는 ObjectList::erase 또는 RE::removeEntity로 지울 수 있다.
class ObjectList: public FrameMoveObject
{
	objectList_data* _members;
public:
	ObjectList();
	~ObjectList();

	// Sometimes, manual cleanup may be needed
	void clear();
	void setVisible(bool bVisible);
	void drawSphere(vector3 const& pos, const char* nameid, const char* materialName=NULL, double scale=5.0);
	void drawAxes(transf const& tf, const char* nameid, double scale=2.0, double positionscale=1.0);
	void drawLine(vector3 const& startPos, vector3 const& endpos, const char* nameid, const char* materialName=NULL);

	Ogre::SceneNode* registerEntity(const char* node_name, const char* filename);
	Ogre::SceneNode* registerEntity(const char* node_name, const char* filename, const char* materialName);
	Ogre::SceneNode* registerEntity(const char* node_name, Ogre::Item* pObject);
	// shows title above the entity
	Ogre::SceneNode* registerNamedEntity(const char* node_name, const char* filename, 
			const char* titleText, double textHeight, vector3 textColor);

	// delete로 지울수 있는 무슨 오브젝트든 가능. (pObject는 반드시 new해서 집어넣는다.)
	Ogre::SceneNode* registerObject(const char* node_name, Ogre::MovableObject* pObject);

	// Factory function. Supported typeName="LineList", "BillboardLineList", "QuadList",...
	Ogre::SceneNode* registerObject(const char* node_name, const char* typeName, const char* materialName, matrixn const& data, m_real thickness=0);
	Ogre::SceneNode* registerObject(const char* node_name, const char* typeName, const char* materialName, vector3N const& data, m_real thickness=0);

	// destroyTime이 지나면 자동 폭파된다.
	Ogre::SceneNode* registerEntityScheduled(const char* filename, m_real destroyTime);
	Ogre::SceneNode* registerObjectScheduled(Ogre::MovableObject* pObject, m_real destroyTime);
	Ogre::SceneNode* registerObjectScheduled(m_real destroyTime, const char* typeName, const char* materialName, matrixn const& data, m_real thickness=0);

	Ogre::SceneNode* findNode(const char* node_name);
	void erase(const char* node_name);
	void eraseAllScheduled();

	Ogre::MovableObject* _find(const char* node_name);
	template <class T>
	T* find(const char* node_name)
	{
		return (T*)_find(node_name);
	}

	virtual int FrameMove(float fElapsedTime);

	Ogre::SceneNode* createSceneNode(const char* node_name);
	Ogre::SceneNode* getCurrRootSceneNode() ; // do not cache this value. This can change over time.
private:

};
#endif
