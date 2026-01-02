#include "stdafx.h"
#ifndef NO_OGRE

#include "Line3D.h" 
#include "../BaseLib/math/conversion.h"
#include "../BaseLib/math/Operator.h"
#include <OgreMeshManager.h>
#include <OgreSubMesh.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "renderer.h"
#include "../BaseLib/motion/viewpoint.h"


#include <OgreMaterialManager.h>
#include <OgreMesh.h>
void _setMaterial(Ogre::Renderable* ptr, const char* name)
{
	ptr->setDatablockOrMaterialName(name, Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME );
}
void _setMaterial(Ogre::v1::SimpleRenderable* ptr, const char* name)
{
	ptr->setDatablockOrMaterialName(name, Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME );
}
void QuadList::setMaterial(const char* name) { _setMaterial(this, name); }
Ogre::Real SimplerRenderable::getSquaredViewDepth(const Ogre::Camera *cam) const 
{ 
   Ogre::Vector3 vMin, vMax, vMid, vDist; 
   vMin = Ogre::v1::SimpleRenderable::mBox.getMinimum(); 
   vMax = Ogre::v1::SimpleRenderable::mBox.getMaximum(); 
   vMid = ((vMin - vMax) * 0.5) + vMin; 
   vDist = cam->getDerivedPosition() - vMid; 

   return vDist.squaredLength(); 
} 

Ogre::Real SimplerRenderable::getBoundingRadius(void) const 
{ 
   return Ogre::Math::Sqrt(MAX(mBox.getMaximum().squaredLength(), mBox.getMinimum().squaredLength())); 
   //return mRadius; 
} 


QuadList::QuadList(vector3 const& normal, m_real width)
:mNormal(normal), mWidth(width)
{
	initialize(Ogre::OT_TRIANGLE_LIST, false);

}

QuadList::~QuadList(void) 
{ 
} 

void QuadList::begin(int n)
{
	mPoints.setSize(n);
}

void QuadList::quad(int i, vector3 const& pos)
{
	mPoints[i]=pos;
}

void QuadList::createVertexDeclaration()
{
	size_t offset=0;

	auto& vertexElements=mRenderOp.vertexElements;
	vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_POSITION ) );
	vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES ) );
	offset+=sizeof(float)*5;
	mRenderOp.offset=offset;
}

void QuadList::end()
{
	fillHardwareBuffers();
}

void QuadList::fillHardwareBuffers()
{
	if(!mPoints.size())
	{
		return ;
	}

   // Initialization stuff 
	prepareHardwareBuffers(mPoints.size()*6, 0);


   // Drawing stuff 
   int size = mPoints.size(); 


   m_real halfWidth=mWidth/2;
   vector3 pos[4], texCoord[4] ;      

   vector3 axis1, axis2;
	quater qy(RE::renderer().viewport().m_pViewpoint->m_fHAngle, vector3(0,1,0));
	axis1.cross(mNormal, qy*vector3(0,0,1));
	if(axis1.length()<0.01)
		axis1.cross(mNormal, qy*vector3(1,0,0));

	axis1.normalize();
	axis2.cross(axis1, mNormal);

   pos[0]=axis1*halfWidth+axis2*halfWidth;
   pos[1]=axis1*halfWidth+axis2*-halfWidth;
   pos[2]=axis1*-halfWidth+axis2*-halfWidth;
   pos[3]=axis1*-halfWidth+axis2*halfWidth;

   texCoord[0]=vector3(1, 0, 1);
   texCoord[1]=vector3(1, 0, 0);
   texCoord[2]=vector3(0, 0, 0);
   texCoord[3]=vector3(0, 0, 1);

   vector3 pp;

   float* prPos=(float*) vertices;
   for(int i = 0; i < size; i++) 
   {   
	   for(int j=0; j<3; j++)
	   {
		   pp=pos[j]+mPoints[i];
			// lower triangle
			*prPos++ = (float)pp.x; 
			*prPos++ = (float)pp.y; 
			*prPos++ = (float)pp.z; 
			*prPos++ = (float)texCoord[j].x;
			*prPos++ = (float)texCoord[j].z;
	   }

	   for(int index=0; index<3; index++)
	   {
		   // upper triangle
		   int j=(index+2)%4;
		   pp=pos[j]+mPoints[i];
			// upper triangle
			*prPos++ = (float)pp.x; 
			*prPos++ = (float)pp.y; 
			*prPos++ = (float)pp.z; 
			*prPos++ = (float)texCoord[j].x;
			*prPos++ = (float)texCoord[j].z;
	   }
   }
   assert(((unsigned char*)prPos)==vertices+(mRenderOp.offset*size*6));

   	finalizeHardwareBuffers();


}

//using namespace Ogre; 





/*
void DynamicLines::getWorldTransforms(Matrix4 *xform) const
{
   // return identity matrix to prevent parent transforms
   *xform = Matrix4::IDENTITY;
}
*/
/*
const Quaternion &DynamicLines::getWorldOrientation(void) const
{
   return Quaternion::IDENTITY;
}

const Vector3 &DynamicLines::getWorldPosition(void) const
{
   return Vector3::ZERO;
}
*/

void BillboardLineList::line(int i, vector3 const& start, vector3 const & end, m_real tu1, m_real tu2)
{
	addChainElement(i, Ogre::v1::BillboardChain::Element(
		ToOgre(start), thickness, tu1, Ogre::ColourValue(1,1,1,1), Ogre::Quaternion(1,0,0,0)));
	addChainElement(i, Ogre::v1::BillboardChain::Element(
		ToOgre(end), thickness, tu2, Ogre::ColourValue(1,1,1,1), Ogre::Quaternion(1,0,0,0)));
}

void ColorBillboardLineList::line(int i, vector3 const& start, vector3 const & end, vector3 const & rgbcolor, m_real tu1, m_real tu2)
{

	addChainElement(i, Ogre::v1::BillboardChain::Element(
		ToOgre(start), thickness, tu1, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1), Ogre::Quaternion(1,0,0,0)));
	addChainElement(i, Ogre::v1::BillboardChain::Element(
		ToOgre(end), thickness, tu2, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1), Ogre::Quaternion(1,0,0,0)));
}

void ColorWidthBillboardLineList::line(int i, vector3 const& start, vector3 const & end, vector3 const & rgbcolor, m_real width, m_real width2, m_real tu1, m_real tu2)
{

	addChainElement(i, Ogre::v1::BillboardChain::Element(
		ToOgre(start), width, tu1, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1), Ogre::Quaternion(1,0,0,0)));
	addChainElement(i, Ogre::v1::BillboardChain::Element(
		ToOgre(end), width2, tu2, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1), Ogre::Quaternion(1,0,0,0)));
}
ColorPointList::ColorPointList()
{
//	initialize(Ogre::OT_POINT_LIST, false); // doesn't work
	initialize(Ogre::OT_TRIANGLE_LIST, false);

}

ColorPointList::~ColorPointList(void) 
{ 
} 

void ColorPointList::begin(int n)
{
	mPoints.setSize(n);
	mColors.setSize(n);

}

void ColorPointList::point(int i, vector3 const& color, vector3 const& pos)
{
	mColors[i]=color;
	mPoints[i]=pos;
}

void ColorPointList::createVertexDeclaration()
{
	size_t offset=0;
	auto& vertexElements=mRenderOp.vertexElements;
	vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_POSITION ) );
	// vertex color doesn't work without pbs modification.
	//vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_DIFFUSE ) );
	vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES ) );
	offset+=sizeof(float)*5;
	mRenderOp.offset=offset;
}

void ColorPointList::end()
{
	fillHardwareBuffers();
}

void ColorPointList::fillHardwareBuffers()
{
	if(!mPoints.size())
	{
		return; 
	}

#if 0
   // Initialization stuff 
	prepareHardwareBuffers(mPoints.size(), 0);


   // Drawing stuff 
   int size = mPoints.size(); 


   float* prPos=(float*) vertices;


   for(int i = 0; i < size; i++) 
   {   
	   vector3& pp=mPoints[i];
	   *prPos++ = pp.x; 
	   *prPos++ = pp.y; 
	   *prPos++ = pp.z; 
	   *prPos++ = mColors[i].x;
	   *prPos++ = mColors[i].y;
	   //*prPos++ = mColors[i].z;
   }
#else
	prepareHardwareBuffers(mPoints.size()*6, 0);


   // Drawing stuff 
   int size = mPoints.size(); 

   vector3 mNormal;
   RE::renderer().viewport().m_pViewpoint->GetViewDir(mNormal);
   mNormal.normalize();
   mNormal*=-1;

   m_real halfWidth=0.5;
   vector3 pos[4], texCoord[4] ;      

   vector3 axis1, axis2;

	axis1.cross(mNormal, vector3(0,1,0));
	if(axis1.length()<0.01)
		axis1.cross(mNormal, vector3(1,0,0));

	axis1.normalize();
	axis2.cross(axis1, mNormal);

   pos[0]=axis1*halfWidth+axis2*halfWidth;
   pos[1]=axis1*halfWidth+axis2*-halfWidth;
   pos[2]=axis1*-halfWidth+axis2*-halfWidth;
   pos[3]=axis1*-halfWidth+axis2*halfWidth;

   texCoord[0]=vector3(1, 0, 1);
   texCoord[1]=vector3(1, 0, 0);
   texCoord[2]=vector3(0, 0, 0);
   texCoord[3]=vector3(0, 0, 1);

   for(int i=0; i<4; i++)
	   texCoord[i]*=0.01; // near-constant tex coord is fine.

   vector3 pp;

   float* prPos=(float*) vertices;
   for(int i = 0; i < size; i++) 
   {   
	   double width=mColors[i].z;
	   for(int j=0; j<3; j++)
	   {
		   pp=pos[j]*width+mPoints[i];
			// lower triangle
			*prPos++ = (float)pp.x; 
			*prPos++ = (float)pp.y; 
			*prPos++ = (float)pp.z; 
			*prPos++ = (float)mColors[i].x+texCoord[j].x;
			*prPos++ = (float)1.0-mColors[i].y+texCoord[j].z;
	   }

	   for(int index=0; index<3; index++)
	   {
		   // upper triangle
		   int j=(index+2)%4; // 2 3 0
		   pp=pos[j]*width+mPoints[i];
			// upper triangle
			*prPos++ = (float)pp.x; 
			*prPos++ = (float)pp.y; 
			*prPos++ = (float)pp.z; 
			*prPos++ = (float)mColors[i].x+texCoord[j].x;
			*prPos++ = (float)1.0-mColors[i].y+texCoord[j].z;
	   }
   }
   assert(((unsigned char*)prPos)==vertices+(mRenderOp.offset*size*6));

#endif

   	finalizeHardwareBuffers();
}

#endif
