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

#if OGRE_VERSION_MINOR>=12
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name)
{
	ptr->setMaterial(Ogre::MaterialManager::getSingleton().getByName(name));
}
void LineList::setMaterial(const char* name) { _setMaterial(this, name); }
void QuadList::setMaterial(const char* name) { _setMaterial(this, name); }
#else
void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name)
{
	ptr->setMaterial(name);
}
#endif
Ogre::Real SimplerRenderable::getSquaredViewDepth(const Ogre::Camera *cam) const 
{ 
   Ogre::Vector3 vMin, vMax, vMid, vDist; 
   vMin = Ogre::SimpleRenderable::mBox.getMinimum(); 
   vMax = Ogre::SimpleRenderable::mBox.getMaximum(); 
   vMid = ((vMin - vMax) * 0.5) + vMin; 
   vDist = cam->getDerivedPosition() - vMid; 

   return vDist.squaredLength(); 
} 

Ogre::Real SimplerRenderable::getBoundingRadius(void) const 
{ 
   return Ogre::Math::Sqrt(MAX(mBox.getMaximum().squaredLength(), mBox.getMinimum().squaredLength())); 
   //return mRadius; 
} 

DynamicLines ::DynamicLines (OperationType opType)
{
  initialize(opType,false);
  _setMaterial(this, "BaseWhiteNoLighting");
  mDirty = true;
}

DynamicLines::~DynamicLines()
{
}

void DynamicLines::setOperationType(OperationType opType)
{
  mRenderOp.operationType = opType;
}

Ogre::RenderOperation::OperationType DynamicLines::getOperationType() const
{
  return mRenderOp.operationType;
}

void DynamicLines::addPoint(const Ogre::Vector3 &p)
{
   mPoints.push_back(p);
   mDirty = true;
}
void DynamicLines::addPoint(Ogre::Real x, Ogre::Real y, Ogre::Real z)
{
   mPoints.push_back(Ogre::Vector3(x,y,z));
   mDirty = true;
}
const Ogre::Vector3& DynamicLines::getPoint(unsigned short index) const
{
   assert(index < mPoints.size() && "Point index is out of bounds!!");
   return mPoints[index];
}
unsigned short DynamicLines::getNumPoints(void) const
{
  return (unsigned short)mPoints.size();
}
void DynamicLines::setPoint(unsigned short index, const Ogre::Vector3 &value)
{
  assert(index < mPoints.size() && "Point index is out of bounds!!");

  mPoints[index] = value;
  mDirty = true;
}
void DynamicLines::clear()
{
  mPoints.clear();
  mDirty = true;
}

void DynamicLines::update()
{
  if (mDirty) fillHardwareBuffers();
}

void DynamicLines::createVertexDeclaration()
{
  Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
  decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
}

void DynamicLines::fillHardwareBuffers()
{
  int size = mPoints.size();

  prepareHardwareBuffers(size,0);

  if (!size) { 
    mBox.setExtents(Vector3::ZERO,Vector3::ZERO);
    mDirty=false;
    return;
  }
  
  Vector3 vaabMin = mPoints[0];
  Vector3 vaabMax = mPoints[0];

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);

  Ogre::Real *prPos = static_cast<Real*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  {
   for(int i = 0; i < size; i++)
   {
      *prPos++ = mPoints[i].x;
      *prPos++ = mPoints[i].y;
      *prPos++ = mPoints[i].z;

      if(mPoints[i].x < vaabMin.x)
         vaabMin.x = mPoints[i].x;
      if(mPoints[i].y < vaabMin.y)
         vaabMin.y = mPoints[i].y;
      if(mPoints[i].z < vaabMin.z)
         vaabMin.z = mPoints[i].z;

      if(mPoints[i].x > vaabMax.x)
         vaabMax.x = mPoints[i].x;
      if(mPoints[i].y > vaabMax.y)
         vaabMax.y = mPoints[i].y;
      if(mPoints[i].z > vaabMax.z)
         vaabMax.z = mPoints[i].z;
   }
  }
  vbuf->unlock();

 // mBox.setExtents(vaabMin, vaabMax);
   // -> buggy
   

	Ogre::AxisAlignedBox box;
    box.setInfinite();
    setBoundingBox(box);

  mDirty = false;
}

void LineList::begin()
{
	clear();
}

void LineList::begin(int n)	
{ clear(); mPoints.reserve(n);}
void LineList::line(int i, vector3 const& start, vector3 const& end)	
{ addPoint(ToOgre(start)); addPoint(ToOgre(end));}

void LineList::addLine(vector3 const& start, vector3 const& end)
{
	addPoint(ToOgre(start)); addPoint(ToOgre(end));
}

void LineList::addCircle(vector3 const& center, vector3 const& axis, m_real halfR)
{
	int numSect=20;

	
	vector3 random(0,0,1);
	if(ABS(random%axis)<0.01)
		random=vector3(1,0,0);

	vector3 axis2;
	axis2.cross(axis, random);
	axis2.normalize();

	vector3 prev=center+axis2;
	vector3 cur;

	mPoints.reserve(mPoints.size()+numSect*2);

	for(int i=1; i<=numSect; i++)
	{
		cur.rotate(
			quater(sop::clampMap(i, 0, numSect, 0.0, TO_RADIAN(360)), axis)
			, axis2);
		cur+=center;
		addLine(prev, cur);
		prev=cur;
	}
}

void LineList::addAxisAlignedBox(vector3 const& minCorner, vector3 const& maxCorner)
{
	double minx=minCorner.x;
	double maxx=maxCorner.x;
	double miny=minCorner.y;
	double maxy=maxCorner.y;
	double minz=minCorner.z;
	double maxz=maxCorner.z;

	mPoints.reserve(mPoints.size()+12*2);
	// line 0
	addLine(vector3(minx,
	miny,
	minz),
	vector3(maxx,
	miny,
	minz));
	
	addLine(vector3(minx,
	miny,
	minz),
	vector3(minx,
	miny,
	maxz));
	
	addLine(vector3(minx,
	miny,
	minz),
	vector3(minx,
	maxy,
	minz));
	
	addLine(vector3(minx,
	maxy,
	minz),
	vector3(minx,
	maxy,
	maxz));
	
	addLine(vector3(minx,
	maxy,
	minz),
	vector3(maxx,
	maxy,
	minz));
	
	addLine(vector3(maxx,
	miny,
	minz),
	vector3(maxx,
	miny,
	maxz));
	
	addLine(vector3(maxx,
	miny,
	minz),
	vector3(maxx,
	maxy,
	minz));
	
	addLine(vector3(minx,
	maxy,
	maxz),
	vector3(maxx,
	maxy,
	maxz));
	
	addLine(vector3(minx,
	maxy,
	maxz),
	vector3(minx,
	miny,
	maxz));
	
	addLine(vector3(maxx,
	maxy,
	minz),
	vector3(maxx,
	maxy,
	maxz));
	
	addLine(vector3(maxx,
	miny,
	maxz),
	vector3(maxx,
	maxy,
	maxz));
	
	addLine(vector3(minx,
	miny,
	maxz),
	vector3(maxx,
	miny,
	maxz));
}
#ifdef USE_SIMPLER_RENDERABLE
LineList::LineList()
{
   mRenderOp.vertexData = NULL;
   this->setMaterial("solidwhite");    
}

LineList::~LineList(void) 
{ 
   delete mRenderOp.vertexData; 
} 

void LineList::begin(int n)
{
	mPoints.setSize(n*2);
}

void LineList::line(int i, vector3 const& start, vector3 const& end)
{
	startPoint(i)=start;
	endPoint(i)=end;
}

void LineList::end()
{
   // Initialization stuff 

	delete mRenderOp.vertexData;
	mRenderOp.vertexData= new Ogre::VertexData(); 


   mRenderOp.indexData = 0; 
   mRenderOp.vertexData->vertexCount = mPoints.size()*2; 
   mRenderOp.vertexData->vertexStart = 0; 
   mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;//, OT_LINE_STRIP 
   mRenderOp.useIndexes = false; 

   if(mPoints.size() ==0)
	   return;

   Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration; 
   Ogre::VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding; 

   decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION); 

   Ogre::HardwareVertexBufferSharedPtr vbuf = 
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer( 
         decl->getVertexSize(POSITION_BINDING), 
         mRenderOp.vertexData->vertexCount, 
		 Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

   bind->setBinding(POSITION_BINDING, vbuf); 

   // Drawing stuff 
   int size = mPoints.size(); 

   vectorn min, max;
   min.minimum(matView(mPoints));
   max.maximum(matView(mPoints));

   Ogre::Real *prPos = (Ogre::Real*)(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

   for(int i = 0; i < size; i++) 
   { 
      *prPos++ = mPoints[i].x; 
      *prPos++ = mPoints[i].y; 
      *prPos++ = mPoints[i].z; 
   } 

   vbuf->unlock(); 

   mBox.setExtents(ToOgre(min.toVector3()), ToOgre(max.toVector3())); 
}
#endif
//

QuadList::QuadList(vector3 const& normal, m_real width)
:mNormal(normal), mWidth(width)
{
#ifdef USE_SIMPLER_RENDERABLE
	mRenderOp.vertexData = NULL;
	mRenderOp.indexData= NULL;
	mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
	mRenderOp.useIndexes = false; 
#else
	initialize(Ogre::RenderOperation::OT_TRIANGLE_LIST, false);
#endif

   _setMaterial(this, "solidwhite");
}

QuadList::~QuadList(void) 
{ 
#ifdef USE_SIMPLER_RENDERABLE
	delete mRenderOp.indexData;
	delete mRenderOp.vertexData; 
#endif
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
  Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
	size_t offset=0;
	// create/bind positions and texture coords
	if(!decl->findElementBySemantic(Ogre::VES_POSITION))
	   decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION); 
	offset+=Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

	if(!decl->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES))
	   decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
}

void QuadList::end()
{
	fillHardwareBuffers();
}

void QuadList::fillHardwareBuffers()
{
	if(!mPoints.size())
	{
		mBox.setExtents(Ogre::Vector3(0,0,0), Ogre::Vector3(0,0,0));
		return ;
	}

   // Initialization stuff 
	prepareHardwareBuffers(mPoints.size()*6, 0);


   // Drawing stuff 
   int size = mPoints.size(); 

   vectorn min, max;
   min.minimum(matView(mPoints));
   max.maximum(matView(mPoints));

   Ogre::HardwareVertexBufferSharedPtr vbuf =mRenderOp.vertexData->vertexBufferBinding->getBuffer(POS_TEX_BINDING);
   
   Ogre::Real *prPos = (Ogre::Real*)(vbuf ->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

   m_real halfWidth=mWidth/2;
   vector3 pos[4], texCoord[4] ;      

   vector3 axis1, axis2;

	axis1.cross(mNormal, vector3(0,0,1));
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

   vector3 pp;

   for(int i = 0; i < size; i++) 
   {   
	   for(int j=0; j<3; j++)
	   {
		   pp=pos[j]+mPoints[i];
			// lower triangle
			*prPos++ = pp.x; 
			*prPos++ = pp.y; 
			*prPos++ = pp.z; 
			*prPos++ = texCoord[j].x;
			*prPos++ = texCoord[j].z;
	   }

	   for(int index=0; index<3; index++)
	   {
		   // upper triangle
		   int j=(index+2)%4;
		   pp=pos[j]+mPoints[i];
			// upper triangle
			*prPos++ = pp.x; 
			*prPos++ = pp.y; 
			*prPos++ = pp.z; 
			*prPos++ = texCoord[j].x;
			*prPos++ = texCoord[j].z;
	   }
   }

   vbuf->unlock(); 

   //mBox.setExtents(ToOgre(min.toVector3()-vector3(0, halfWidth, 0)), ToOgre(max.toVector3()+vector3(0, halfWidth, 0)));
   // -> buggy
   

	Ogre::AxisAlignedBox box;
    box.setInfinite();
    setBoundingBox(box);
}

//using namespace Ogre; 

Line3D::Line3D(void) 
{ 
   mRenderOp.vertexData = new Ogre::VertexData(); 
   m_cLineColor = 0;

   _setMaterial(this, "BaseWhiteNoLighting");    
} 

Line3D::~Line3D(void) 
{ 
   delete Ogre::SimpleRenderable::mRenderOp.vertexData; 
} 

void Line3D::addPoint(const Ogre::Vector3 &p) 
{ 
   mPoints.push_back(p); 
} 

void Line3D::removeAllpoints()
{
	mPoints.clear();
}

const Ogre::Vector3 &Line3D::getPoint(unsigned short index) const 
{ 
   assert(index < mPoints.size() && "Point index is out of bounds!!"); 

   return mPoints[index]; 
} 

unsigned short Line3D::getNumPoints(void) const 
{ 
   return (unsigned short)mPoints.size(); 
} 

void Line3D::updatePoint(unsigned short index, const Ogre::Vector3 &value) 
{ 
   assert(index < mPoints.size() && "Point index is out of bounds!!"); 

   mPoints[index] = value; 
} 


void Line3D::drawLine(Ogre::Vector3 &start, Ogre::Vector3 &end) 
{ 
   if(mPoints.size()) 
      mPoints.clear(); 

   mPoints.push_back(start); 
   mPoints.push_back(end); 

   drawLines(); 
} 

void Line3D::SetColor(int i) 
{
	m_cLineColor=i;
	switch(m_cLineColor)
	{
	case 0:
		_setMaterial(this, "solidwhite");
		break;
	case 1:
		_setMaterial(this, "solidblue");
		break;
	case 2:
		_setMaterial(this, "solidgreen");
		break;
	case 3:
		_setMaterial(this, "solidred");
		break;
	case 4:
		_setMaterial(this, "solidblack");
		break;

	default:
		printf("error color %d unknown\n", m_cLineColor);
		break;
	}

}

void Line3D::drawLines(void) 
{ 
   // Initialization stuff 
   Ogre::SimpleRenderable::mRenderOp.indexData = 0; 
   Ogre::SimpleRenderable::mRenderOp.vertexData->vertexCount = mPoints.size(); 
   Ogre::SimpleRenderable::mRenderOp.vertexData->vertexStart = 0; 
   Ogre::SimpleRenderable::mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_STRIP; // OT_LINE_LIST, OT_LINE_STRIP 
   Ogre::SimpleRenderable::mRenderOp.useIndexes = false; 

   if(mPoints.size() < 2)
	   return;

 
   Ogre::VertexDeclaration *decl = Ogre::SimpleRenderable::mRenderOp.vertexData->vertexDeclaration; 
   Ogre::VertexBufferBinding *bind = Ogre::SimpleRenderable::mRenderOp.vertexData->vertexBufferBinding; 

   decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION); 

   Ogre::HardwareVertexBufferSharedPtr vbuf = 
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer( 
         decl->getVertexSize(POSITION_BINDING), 
         Ogre::SimpleRenderable::mRenderOp.vertexData->vertexCount, 
         Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

   bind->setBinding(POSITION_BINDING, vbuf); 

   // Drawing stuff 
   int size = mPoints.size(); 
   Ogre::Vector3 vaabMin = mPoints[0]; 
   Ogre::Vector3 vaabMax = mPoints[0]; 

   Ogre::Real *prPos = static_cast<Ogre::Real*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

   for(int i = 0; i < size; i++) 
   { 
      *prPos++ = mPoints[i].x; 
      *prPos++ = mPoints[i].y; 
      *prPos++ = mPoints[i].z; 

      if(mPoints[i].x < vaabMin.x) 
         vaabMin.x = mPoints[i].x; 
      if(mPoints[i].y < vaabMin.y) 
         vaabMin.y = mPoints[i].y; 
      if(mPoints[i].z < vaabMin.z) 
         vaabMin.z = mPoints[i].z; 

      if(mPoints[i].x > vaabMax.x) 
         vaabMax.x = mPoints[i].x; 
      if(mPoints[i].y > vaabMax.y) 
         vaabMax.y = mPoints[i].y; 
      if(mPoints[i].z > vaabMax.z) 
         vaabMax.z = mPoints[i].z; 
   } 

   vbuf->unlock(); 

   mBox.setExtents(vaabMin, vaabMax); 
} 


const Ogre::Quaternion &Line3D::getWorldOrientation(void) const 
{ 
   return Ogre::Quaternion::IDENTITY; 
} 

const Ogre::Vector3 &Line3D::getWorldPosition(void) const 
{ 
   return Ogre::Vector3::ZERO; 
} 


using namespace Ogre;
/// From the WIKI -- tesselate a sphere
void createSphere(const std::string& strName, const float r, const int nRings , const int nSegments )
{
	MeshPtr pSphere = MeshManager::getSingleton().createManual(strName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	SubMesh *pSphereVertex = pSphere->createSubMesh();

	pSphere->sharedVertexData = new VertexData();
	VertexData* vertexData = pSphere->sharedVertexData;

	// define the vertex format
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	// normals
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	// two dimensional texture coordinates
	vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
	currOffset += VertexElement::getTypeSize(VET_FLOAT2);

	// allocate the vertex buffer
	vertexData->vertexCount = (nRings + 1) * (nSegments+1);
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);
	float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));

	// allocate index buffer
	pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
	pSphereVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_DISCARD));

	float fDeltaRingAngle = (Math::PI / nRings);
	float fDeltaSegAngle = (2 * Math::PI / nSegments);
	unsigned short wVerticeIndex = 0 ;

	// Generate the group of rings for the sphere
	for( int ring = 0; ring <= nRings; ring++ ) {
		float r0 = r * sinf (ring * fDeltaRingAngle);
		float y0 = r * cosf (ring * fDeltaRingAngle);

		// Generate the group of segments for the current ring
		for(int seg = 0; seg <= nSegments; seg++) {
			float x0 = r0 * sinf(seg * fDeltaSegAngle);
			float z0 = r0 * cosf(seg * fDeltaSegAngle);

			// Add one vertex to the strip which makes up the sphere
			*pVertex++ = x0;
			*pVertex++ = y0;
			*pVertex++ = z0;

			Vector3 vNormal = Vector3(x0, y0, z0).normalisedCopy();
			*pVertex++ = vNormal.x;
			*pVertex++ = vNormal.y;
			*pVertex++ = vNormal.z;

			*pVertex++ = (float) seg / (float) nSegments;
			*pVertex++ = (float) ring / (float) nRings;

			if (ring != nRings) {
				// each vertex (except the last) has six indicies pointing to it
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex;               
				*pIndices++ = wVerticeIndex + nSegments;
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex + 1;
				*pIndices++ = wVerticeIndex;
				wVerticeIndex ++;
			}
		}; // end for seg
	} // end for ring

	// Unlock
	vBuf->unlock();
	iBuf->unlock();
	// Generate face list
	pSphereVertex->useSharedVertices = true;

	// the original code was missing this line:
	pSphere->_setBounds( AxisAlignedBox( Vector3(-r, -r, -r), Vector3(r, r, r) ), false );
	pSphere->_setBoundingSphereRadius(r);
	// this line makes clear the mesh is loaded (avoids memory leakes)
	pSphere->load();
}





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
#if OGRE_VERSION_MINOR >= 8 
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(start), thickness, tu1, Ogre::ColourValue(1,1,1,1), Ogre::Quaternion(1,0,0,0)));
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(end), thickness, tu2, Ogre::ColourValue(1,1,1,1), Ogre::Quaternion(1,0,0,0)));
#else
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(start), thickness, tu1, Ogre::ColourValue(1,1,1,1)));
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(end), thickness, tu2, Ogre::ColourValue(1,1,1,1)));
#endif
}

void ColorBillboardLineList::line(int i, vector3 const& start, vector3 const & end, vector3 const & rgbcolor, m_real tu1, m_real tu2)
{
#if OGRE_VERSION_MINOR >= 8 
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(start), thickness, tu1, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1), Ogre::Quaternion(1,0,0,0)));
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(end), thickness, tu2, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1), Ogre::Quaternion(1,0,0,0)));
#else
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(start), thickness, tu1, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1)));
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(end), thickness, tu2, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1)));
#endif
}

void ColorWidthBillboardLineList::line(int i, vector3 const& start, vector3 const & end, vector3 const & rgbcolor, m_real width, m_real tu1, m_real tu2)
{
#if OGRE_VERSION_MINOR >= 8 
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(start), width, tu1, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1), Ogre::Quaternion(1,0,0,0)));
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(end), width, tu2, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1), Ogre::Quaternion(1,0,0,0)));
#else
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(start), width, tu1, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1)));
	addChainElement(i, Ogre::BillboardChain::Element(
		ToOgre(end), width, tu2, Ogre::ColourValue(rgbcolor[0],rgbcolor[1],rgbcolor[2],1)));
#endif
}
ColorPointList::ColorPointList()
{
#ifdef USE_SIMPLER_RENDERABLE
	mRenderOp.vertexData = NULL;
	mRenderOp.indexData= NULL;
	mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
	mRenderOp.useIndexes = false; 
#else
	initialize(Ogre::RenderOperation::OT_POINT_LIST, false);
#endif

   _setMaterial(this, "solidwhitepoint");
}

ColorPointList::~ColorPointList(void) 
{ 
#ifdef USE_SIMPLER_RENDERABLE
	delete mRenderOp.indexData;
	delete mRenderOp.vertexData; 
#endif
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
  Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
	size_t offset=0;
	// create/bind positions and texture coords
	if(!decl->findElementBySemantic(Ogre::VES_POSITION))
	   decl->addElement(POS_COLOUR_BINDING, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION); 
	offset+=Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

	if(!decl->findElementBySemantic(Ogre::VES_DIFFUSE))
	   decl->addElement(POS_COLOUR_BINDING, offset, Ogre::VET_FLOAT3, Ogre::VES_DIFFUSE, 0);
}

void ColorPointList::end()
{
	fillHardwareBuffers();
}

void ColorPointList::fillHardwareBuffers()
{
	if(!mPoints.size())
	{
		mBox.setExtents(Ogre::Vector3(0,0,0), Ogre::Vector3(0,0,0));
		return ;
	}

   // Initialization stuff 
	prepareHardwareBuffers(mPoints.size()*6, 0);


   // Drawing stuff 
   int size = mPoints.size(); 

   vectorn min, max;
   min.minimum(matView(mPoints));
   max.maximum(matView(mPoints));

   Ogre::HardwareVertexBufferSharedPtr vbuf =mRenderOp.vertexData->vertexBufferBinding->getBuffer(POS_COLOUR_BINDING);
   
   Ogre::Real *prPos = (Ogre::Real*)(vbuf ->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 


   for(int i = 0; i < size; i++) 
   {   
	   vector3& pp=mPoints[i];
	   *prPos++ = pp.x; 
	   *prPos++ = pp.y; 
	   *prPos++ = pp.z; 
	   *prPos++ = mColors[i].x;
	   *prPos++ = mColors[i].y;
	   *prPos++ = mColors[i].z;
   }

   vbuf->unlock(); 

   //mBox.setExtents(ToOgre(min.toVector3()-vector3(0, halfWidth, 0)), ToOgre(max.toVector3()+vector3(0, halfWidth, 0)));
   // -> buggy
   

	Ogre::AxisAlignedBox box;
    box.setInfinite();
    setBoundingBox(box);
}

#endif
