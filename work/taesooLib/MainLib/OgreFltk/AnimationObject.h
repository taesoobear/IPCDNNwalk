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

#ifndef NO_OGRE
	Ogre::SceneNode* m_pSceneNode;
#endif
	TimeSensor* m_pTimer;
	vector3 m_vTrans;

	virtual void setTranslation(float x, float y, float z);
	void setTranslation(vector3 const& x) {  setTranslation(x.x, x.y, x.z);}
	vector3 const& getTranslation() const { return m_vTrans;}

	const TString& getType() const	{return mType;}

	void attachTimer(float frameTime, int numFrames);
protected:

	TString mType;
	bool mbVisible;
};


// 실제로 보여줄 AnimationObject는 없지만 scroll바가 동작하도록 만들고 싶을때 사용.

namespace RE
{
	AnimationObject* createEmptyAnimationObject(int numFrames, float frameTime=1.0/30.0);
}
