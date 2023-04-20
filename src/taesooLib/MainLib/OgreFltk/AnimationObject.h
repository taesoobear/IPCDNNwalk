#pragma once

#include "framemoveobject.h"

namespace Ogre
{
	class SceneNode;
}
class TimeSensor;
class AnimationObject : public FrameMoveObject
{
public:
	AnimationObject ();
	virtual ~AnimationObject ();

	virtual void SetVisible(bool bVisible);
	bool GetVisible() const { return mbVisible;}
	virtual int FrameMove(float fElapsedTime);

	Ogre::SceneNode* m_pSceneNode;
	TimeSensor* m_pTimer;
	vector3 m_vTrans;

	virtual void setTranslation(float x, float y, float z);
	vector3 const& getTranslation() const { return m_vTrans;}

	const TString& getType() const	{return mType;}

protected:

	TString mType;
	bool mbVisible;
};


// 실제로 보여줄 AnimationObject는 없지만 scroll바가 동작하도록 만들고 싶을때 사용.

namespace RE
{
	AnimationObject* createEmptyAnimationObject(int numFrames, float frameTime=1.0/30.0);
}
