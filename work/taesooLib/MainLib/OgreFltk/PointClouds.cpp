#include "stdafx.h"
#ifndef NO_GUI
#include "PointClouds.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "renderer.h"
#include "FltkRenderer.h"
extern OgreRenderer* g_pRenderer;

PointClouds* RE::createPointClouds()
{
	
	::PointClouds* pSkin=new PointClouds(RE::renderer());
	RE::renderer().addFrameMoveObject(pSkin);
	return pSkin;
}

PointClouds::PointClouds(const OgreRenderer& renderer)
:mRenderer(renderer)
{
	m_pSceneNode=NULL;
	mbVisible=true;
	m_vTrans.setValue(0,0,0);

	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();

	setRadius(4);
	mSceneNodes.resize(0);
}

PointClouds::~PointClouds(void)
{
	destructScene();
}



void PointClouds::destructScene()
{
	if(m_pSceneNode)
	{
		m_pSceneNode->getParentSceneNode()->removeAndDestroyChild(m_pSceneNode);
		m_pSceneNode=NULL;
	}
}

void PointClouds::SetTranslation(float x, float y, float z)
{
	m_vTrans.setValue(x,y,z);
	m_pSceneNode->setPosition(x, y, z);
}


void PointClouds::setRadius(m_real r)
{
	mRadius=r;
}

void PointClouds::SetVisible(bool bVisible)
{
	m_pSceneNode->setVisible(bVisible);
	mbVisible=bVisible;
}

void PointClouds::setMaterial(const char* mat)
{
	for(int i=2; i<mSceneNodes.size(); i++)
		((Ogre::v1::Entity*)mSceneNodes[i]->getAttachedObject(0))->setMaterialName(mat);
}

void PointClouds::update(matrixn const & points)
{
	if(mSceneNodes.size()<points.rows())
	{
		int prevSize=mSceneNodes.size();
		mSceneNodes.resize(points.rows());

		for(int i=prevSize; i<points.rows(); i++)
		{
			mSceneNodes[i]=m_pSceneNode->createChildSceneNode();
			//Ogre::Entity *limbEntity = mSphere;
			static int ientity=0;
			TString ename;
			ename.format("spp%d", ientity);
			ientity++;
			Ogre::v1::Entity *limbEntity =mRenderer.viewport().mScene->createEntity(ename.ptr(), "sphere1010.mesh");
//			limbEntity ->setNormaliseNormals(true);
			mSceneNodes[i]->attachObject(limbEntity);
		}
	}
	
	for(int i=0; i<points.rows(); i++)
	{
		Ogre::SceneNode* pNode=mSceneNodes[i];
		Ogre::Vector3 center;
		center.x=points(i,0);
		center.y=points(i,1);
		center.z=points(i,2);
		pNode->resetOrientation();
		pNode->setScale(mRadius/2,mRadius/2,mRadius/2);
		pNode->setPosition(center);
		pNode->setVisible(true);
	}

	for(int i=points.rows(); i<mSceneNodes.size(); i++)
	{
		mSceneNodes[i]->setVisible(false);		
	}

}

int PointClouds::FrameMove(float fElapsedTime)
{
	return 1;
}



SelectionRectangle::SelectionRectangle(const char* name)
: Ogre::v1::ManualObject(RE::nameToUID(name), RE::_objectMemoryManager(), RE::ogreSceneManager())
{
	setUseIdentityProjection(true);
    setUseIdentityView(true);
	//setRenderQueueGroup(Ogre::v1::RENDER_QUEUE_OVERLAY);
    setQueryFlags(0);
	setCastShadows(false);
}

void SelectionRectangle::setCorners(TRect const& rect)
{
	float width=(float)RE::FltkRenderer().renderWindowWidth();
	float height=(float)RE::FltkRenderer().renderWindowHeight();

	left=rect.left;
	top=rect.top;
	right=rect.right;
	bottom=rect.bottom;
//	cout<< rect.left <<rect.top<<rect.right<< rect.top<<endl;
	setCorners(rect.left/width, rect.top/height, rect.right/width, rect.bottom/height);
}

void SelectionRectangle::setCorners(float l, float t, float r, float b)
{
	nl=l;
	nt=t;
	nr=r;
	nb=b;

	l = l * 2 - 1;
    r = r * 2 - 1;
    t = 1 - t * 2;
    b = 1 - b * 2;
	
    clear();
	begin("solidblue", Ogre::OT_LINE_STRIP);
        position(l, t, -1);
        position(r, t, -1);
        position(r, b, -1);
        position(l, b, -1);
        position(l, t, -1);
    end();

	//todo2 Ogre::AxisAlignedBox box;
    //box.setInfinite();
    //setBoundingBox(box);
}

void SelectionRectangle::constructSelectionVolumn(FltkRenderer const& mRenderer,std::vector<Plane>& vol)
{
	if (left > right)
		std::swap(left, right);

	if (top > bottom)
		std::swap(top, bottom);

	if ((right - left) * (bottom - top) < 0.0001)
	{
		vol.resize(0);
		return;
	}	

	// Convert screen positions into rays
	Ray topLeft, topRight, bottomLeft, bottomRight;

	mRenderer.screenToWorldRay(left,top,topLeft);
	mRenderer.screenToWorldRay(right,top,topRight);
	mRenderer.screenToWorldRay(left,bottom, bottomLeft);
	mRenderer.screenToWorldRay(right,bottom, bottomRight);

	// The plane faces the counter clockwise position (normals face inward)
	vol.resize(5);
	vol[0].setPlane(topLeft.getPoint(3), topRight.getPoint(3), bottomRight.getPoint(3));// front plane
	vol[1].setPlane(topLeft.origin(), topLeft.getPoint(100), topRight.getPoint(100));// top plane
	vol[2].setPlane(topLeft.origin(), bottomLeft.getPoint(100), topLeft.getPoint(100));// left plane
	vol[3].setPlane(bottomLeft.origin(), bottomRight.getPoint(100), bottomLeft.getPoint(100));// bottom plane
	vol[4].setPlane(topRight.origin(), topRight.getPoint(100), bottomRight.getPoint(100));// right plane
}
#endif
