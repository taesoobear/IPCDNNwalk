#ifndef POINT_CLOUDS_H_
#define POINT_CLOUDS_H_
#ifndef NO_GUI
#pragma once

#include "framemoveobject.h"
class OgreRenderer;
class Posture;
class MotionLoader;

class PointClouds: public FrameMoveObject
{
public:
	PointClouds(const OgreRenderer& renderer);
public:
	virtual ~PointClouds(void);

	virtual void SetVisible(bool bVisible);
	bool GetVisible() const { return mbVisible;}
	virtual int FrameMove(float fElapsedTime);

	virtual void destructScene();


	Ogre::SceneNode* m_pSceneNode;

	void SetTranslation(float x, float y, float z);
	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton){}
	
	void setMaterial(const char* mat);
	void setRadius(m_real r);

	virtual void update(matrixn const& points);
private:	
	bool mbVisible;
	vector3 m_vTrans;
	m_real mRadius;
	const OgreRenderer& mRenderer;
	std::vector<Ogre::SceneNode*> mSceneNodes;
};

namespace RE	
{
	::PointClouds* createPointClouds();
}
#include "OgreManualObject.h"
#include "../Ogre/intersectionTest.h"
class TRect;
class FltkRenderer;
class SelectionRectangle : public Ogre::v1::ManualObject
{
	
public:
	m_real left, top, right, bottom;
	m_real nl, nt, nr, nb;	// in [0,1]
    SelectionRectangle(const char* name);

    /**
    * Sets the corners of the SelectionRectangle.  Every parameter should be in the
    * range [0, 1] representing a percentage of the screen the SelectionRectangle
    * should take up.
    */
    void setCorners(float left, float top, float right, float bottom);

	// parameter should be in screen coordinate (e.g. 640*480)
	void setCorners(TRect const& rect);

	void constructSelectionVolumn(FltkRenderer const& renderer,std::vector<Plane>& vol);



};
#endif
#endif
