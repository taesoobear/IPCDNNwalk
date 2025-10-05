#include "stdafx.h"
#include "renderer.h"
#include "timesensor.h"
#include "AnimationObject.h"
#include "MotionPanel.h"
#include "FltkScrollPanel.h"
#ifndef NO_OGRE
#include <OgreSceneNode.h>
#endif
//#include <OgreSceneManager.h>
//#include <OgreEntity.h>
AnimationObject::AnimationObject()
{
	m_pTimer=NULL;
	m_pSceneNode=NULL;
	mbVisible=true;

	mType="AnimationObject";
	m_vTrans.setValue(0,0,0);

}

AnimationObject::~AnimationObject()
{

	if(m_pTimer) {
#ifndef NO_OGRE
		if(RE::motionPanelValid())
			RE::motionPanel().motionWin()->detachSkin(this);
#endif

		delete m_pTimer;
	}
	if(m_pSceneNode)
	{
		RE::removeEntity(m_pSceneNode);
		//((Ogre::SceneNode*)m_pSceneNode->getParent())->removeAndDestroyChild(m_pSceneNode->getName());
		m_pSceneNode=NULL;
	}
}

void AnimationObject::setTranslation(float x, float y, float z)
{
	m_vTrans.setValue(x,y,z);
#ifndef NO_OGRE

	if(m_pSceneNode)
		m_pSceneNode->setPosition(x, y, z);
#endif
}

int AnimationObject::FrameMove(float fElapsedTime)
{
	int ret=FALSE;
	if(m_pTimer) 
	{
		ret|=m_pTimer->FrameMove(fElapsedTime);
	}
	return ret;
}

void AnimationObject::SetVisible(bool bVisible)
{
#ifndef NO_OGRE
	m_pSceneNode->setVisible(bVisible);
#endif
	mbVisible=bVisible;
}

class EmptyAnimationObject : public AnimationObject
{
public:
	EmptyAnimationObject (int numFrames, float frameTime=1.0/30.0);
	~EmptyAnimationObject (){}
};


EmptyAnimationObject ::EmptyAnimationObject (int numFrames, float frameTime)
		:AnimationObject()
	{
		InterpolatorLinear * pInterpolator=new InterpolatorLinear();
		pInterpolator->init(frameTime, numFrames);
		m_pTimer=new TimeSensor();
		m_pTimer->AttachInterpolator(pInterpolator);
	}


AnimationObject* RE::createEmptyAnimationObject(int numFrames, float frameTime)
{
	EmptyAnimationObject 	* animo=new EmptyAnimationObject(numFrames, frameTime);
	RE::renderer().addFrameMoveObject(animo);
	return animo;
}
