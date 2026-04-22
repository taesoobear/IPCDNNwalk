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
	axis1.cross(mNormal, qy*vector3(0,1,0));
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
	quater qy(RE::renderer().viewport().m_pViewpoint->m_fHAngle, vector3(0,1,0));
	axis1.cross(mNormal, qy*vector3(0,1,0));
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
#include <OgreMeshManager2.h>
#include <OgreMesh2.h>
#include <OgreSubMesh2.h>
#include <Vao/OgreVaoManager.h>
#include <Vao/OgreVertexArrayObject.h>
#include <Vao/OgreVertexBufferPacked.h>
#include <OgreRoot.h>
#include <OgreItem.h>
#include <OgreAxisAlignedBox.h>

#if 0
// in ogre-next, OT_POINT_LIST doesn't work
Ogre::Item* createPointCloudEntity(
    const std::string& meshName,
    const float* xyz_half,   // half3 packed
    const uint8_t* color,       // ubyte4
    const float* covd_half,  // half3
    const float* covu_half,  // half3
    size_t n)
{
    // Mesh 생성
	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
        meshName,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

	Ogre::SubMesh* subMesh = mesh->createSubMesh();

    // Ogre-Next 핵심: VAO 사용
	Ogre::VaoManager* vaoManager = Ogre::Root::getSingleton().getRenderSystem()->getVaoManager();

    std::vector<Ogre::VertexElement2> vertexElements;

    // layout 정의 (Python decl.addElement 대응)
    vertexElements.push_back(Ogre::VertexElement2(
        Ogre::VET_FLOAT3, Ogre::VES_POSITION));

    vertexElements.push_back(Ogre::VertexElement2(
        Ogre::VET_UBYTE4_NORM, Ogre::VES_DIFFUSE));

    vertexElements.push_back(Ogre::VertexElement2(
        Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES));

    vertexElements.push_back(Ogre::VertexElement2(
        Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES));

    size_t vertexSize = 12 +   // position
                        4 + //Ogre::VertexElement2::getTypeSize(VET_UBYTE4_NORM) +
                        12 + //Ogre::VertexElement2::getTypeSize(VET_FLOAT3) +
                        12; //Ogre::VertexElement2::getTypeSize(VET_FLOAT3);

    // interleaved buffer 구성
    std::vector<uint8_t> interleaved(vertexSize * n);

	Ogre::Vector3 minV( std::numeric_limits<float>::max() );
	Ogre::Vector3 maxV(-std::numeric_limits<float>::max() );
    for (size_t i = 0; i < n; ++i)
    {
        uint8_t* dst = interleaved.data() + i * vertexSize;

        size_t offset = 0;

        memcpy(dst + offset, &xyz_half[i * 3], sizeof(float) * 3);
        offset += sizeof(float) * 3;
		minV.x=MIN(minV.x, xyz_half[i*3]);
		minV.y=MIN(minV.y, xyz_half[i*3+1]);
		minV.z=MIN(minV.z, xyz_half[i*3+2]);
		maxV.x=MAX(maxV.x, xyz_half[i*3]);
		maxV.y=MAX(maxV.y, xyz_half[i*3+1]);
		maxV.z=MAX(maxV.z, xyz_half[i*3+2]);

        memcpy(dst + offset, &color[i * 4], sizeof(uint8_t) * 4);
        offset += sizeof(uint8_t) * 4;

        memcpy(dst + offset, &covd_half[i * 3], sizeof(float) * 3);
        offset += sizeof(float) * 3;

        memcpy(dst + offset, &covu_half[i * 3], sizeof(float) * 3);
    }

    // VertexBuffer 생성
	Ogre::VertexBufferPacked* vertexBuffer =
        vaoManager->createVertexBuffer(
            vertexElements,
            n,
            Ogre::BT_IMMUTABLE,
            interleaved.data(),
            false);

    // VAO 생성
	Ogre::VertexBufferPackedVec vertexBuffers;
    vertexBuffers.push_back(vertexBuffer);

	Ogre::IndexBufferPacked* indexBuffer;
	{
		// createIndices
		Ogre::SubMesh* sub = mesh->getSubMesh(0);

		const size_t numVertices = vertexBuffer->getNumElements();

		Ogre::VaoManager* vaoManager = Ogre::Root::getSingleton().getRenderSystem()->getVaoManager();

		// 인덱스 데이터 생성 (0,1,2,...)
		std::vector<uint32_t> indices(numVertices);
		for (uint32_t i = 0; i < numVertices; ++i)
			indices[i] = i;

		// IndexBuffer 생성
		indexBuffer = vaoManager->createIndexBuffer(
			Ogre::IndexBufferPacked::IT_32BIT,
			numVertices,
			Ogre::BT_DYNAMIC_DEFAULT,   // Python의 HBU_CPU_TO_GPU 대응
			indices.data(),       // 초기 데이터 바로 전달 (lock 필요 없음)
			false                 // shadow buffer
		);

	}

	Ogre::VertexArrayObject* vao =
        vaoManager->createVertexArrayObject(
            vertexBuffers,
            indexBuffer, 
            Ogre::OT_POINT_LIST);
	subMesh->mVao[Ogre::VpNormal].resize(1);
	subMesh->mVao[Ogre::VpShadow].resize(1);
    subMesh->mVao[Ogre::VpNormal][0]=vao;
    subMesh->mVao[Ogre::VpShadow][0]=vao;
	subMesh->setMaterialName("pointcloud2");
    mesh->_setBounds(Ogre::Aabb(minV, maxV), false);
    mesh->_setBoundingSphereRadius((maxV - minV).length() * 0.5f);
	auto* entity=RE::_createEntity(meshName.c_str());
	entity->setCastShadows(false);

	return  entity;
}
#endif
Ogre::Item* createPointCloudEntity_quad(
    const std::string& meshName,
    const float* xyz_half,   // half3 packed
    const uint8_t* color,       // ubyte4
    const float* covd_half,  // half3
    const float* covu_half,  // half3
    size_t n)
{
	struct QuadVertex
	{
		float x, y; // inPos (-1 ~ 1)
	};

	QuadVertex quad[4] = {
		{-1.f, -1.f},
		{ 1.f, -1.f},
		{ 1.f,  1.f},
		{-1.f,  1.f}
	};

	// Index buffer (Quad: 2 triangles)
	static const uint16_t quadIndices[] = { 0, 1, 2, 0, 2, 3 };

	struct GaussianSplatInstance
	{
		float pos[3];       // xyz position
		uint8_t color[4];     // rgba
		float covD[3];      // covariance diagonal 
		float covU[3];      // covariance upper triangle 
		float quad[2];      // 16-byte alignment ok.
	};

    // Mesh 생성
	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
        meshName,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

	Ogre::SubMesh* subMesh = mesh->createSubMesh();

    // Ogre-Next 핵심: VAO 사용
	Ogre::VaoManager* vaoManager = Ogre::Root::getSingleton().getRenderSystem()->getVaoManager();

	size_t numSplats=n;
	std::vector<GaussianSplatInstance> splatData(numSplats*4);


	Ogre::VertexElement2Vec vertexElements = {
		Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_POSITION), // pos
		Ogre::VertexElement2(Ogre::VET_UBYTE4_NORM, Ogre::VES_DIFFUSE), // color
		Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES), // covD
		Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES), // covU
		Ogre::VertexElement2(Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES), // quad
	};

    size_t vertexSize = 12 +   // position
                        4 + //Ogre::VertexElement2::getTypeSize(VET_UBYTE4_NORM) +
                        12 + //Ogre::VertexElement2::getTypeSize(VET_FLOAT3) +
                        12 +
						8; //Ogre::VertexElement2::getTypeSize(VET_FLOAT3);

    // interleaved buffer 구성
	Ogre::Vector3 minV( std::numeric_limits<float>::max() );
	Ogre::Vector3 maxV(-std::numeric_limits<float>::max() );
    for (size_t i = 0; i < n; ++i)
    {
		for (size_t j=0; j<4; j++)
		{
			GaussianSplatInstance& dst = splatData[i*4+j];


			memcpy(&dst.pos , &xyz_half[i * 3], sizeof(float) * 3);
			minV.x=MIN(minV.x, xyz_half[i*3]);
			minV.y=MIN(minV.y, xyz_half[i*3+1]);
			minV.z=MIN(minV.z, xyz_half[i*3+2]);
			maxV.x=MAX(maxV.x, xyz_half[i*3]);
			maxV.y=MAX(maxV.y, xyz_half[i*3+1]);
			maxV.z=MAX(maxV.z, xyz_half[i*3+2]);

			memcpy(&dst .color, &color[i * 4], sizeof(uint8_t) * 4);

			memcpy(dst .covD, &covd_half[i * 3], sizeof(float) * 3);

			memcpy(dst .covU, &covu_half[i * 3], sizeof(float) * 3);

			memcpy(dst.quad, &quad[j], sizeof(float)*2);
		}
    }

    // VertexBuffer 생성
	Ogre::VertexBufferPacked* vertexBuffer =
        vaoManager->createVertexBuffer(
            vertexElements,
            n*4,
            Ogre::BT_IMMUTABLE,
            //Ogre::BT_DYNAMIC_DEFAULT,
            splatData.data(),
            false);
				  

    // VAO 생성
	Ogre::VertexBufferPackedVec vertexBuffers;
    vertexBuffers.push_back(vertexBuffer);


	Ogre::IndexBufferPacked* indexBuffer;
	{
		// createIndices
		Ogre::SubMesh* sub = mesh->getSubMesh(0);

		const size_t numVertices = numSplats*6;

		Ogre::VaoManager* vaoManager = Ogre::Root::getSingleton().getRenderSystem()->getVaoManager();

		// 인덱스 데이터 생성 (0,1,2,...)
		std::vector<uint32_t> indices(numVertices);
		for (uint32_t i = 0; i < numSplats; ++i)
			for (uint32_t j=0; j<6; ++j)
				indices[i*6+j] = i*4+quadIndices[j];

		// IndexBuffer 생성
		indexBuffer = vaoManager->createIndexBuffer(
			Ogre::IndexBufferPacked::IT_32BIT,
			numVertices,
			//Ogre::BT_DYNAMIC_DEFAULT,   // Python의 HBU_CPU_TO_GPU 대응
			Ogre::BT_DYNAMIC_PERSISTENT,   // Python의 HBU_CPU_TO_GPU 대응
			indices.data(),       // 초기 데이터 바로 전달 (lock 필요 없음)
			false                 // shadow buffer
		);

	}

	Ogre::VertexArrayObject* vao =
        vaoManager->createVertexArrayObject(
            vertexBuffers,
            indexBuffer, 
            Ogre::OT_TRIANGLE_LIST);


	subMesh->mVao[Ogre::VpNormal].resize(1);
	subMesh->mVao[Ogre::VpShadow].resize(1);
    subMesh->mVao[Ogre::VpNormal][0]=vao;
    subMesh->mVao[Ogre::VpShadow][0]=vao;
	subMesh->setMaterialName("pointcloud_quad");
    mesh->_setBounds(Ogre::Aabb(minV, maxV), false);
    mesh->_setBoundingSphereRadius((maxV - minV).length() * 0.5f);
	auto* entity=RE::_createEntity(meshName.c_str());
	entity->setCastShadows(false);

	/*
	const Ogre::VertexArrayObjectArray& vaos = subMesh->mVao[Ogre::VpNormal];
    for(auto* vao : vaos) {
        for(auto& vertexBuffer : vao->getVertexBuffers()) {
            auto& elements = vertexBuffer->getVertexElements();
            for(auto& elem : elements) {
                Ogre::LogManager::getSingleton().logMessage(
                    "Semantic: " + Ogre::StringConverter::toString(elem.mSemantic) +
                    " Type: " + Ogre::StringConverter::toString(elem.mType)
                );
            }
        }
    }
	*/
	return  entity;
}

void _updatePointCloudEntity(Ogre::Item* item, const int* pidx, int idx_size)  
{
	Ogre::MeshPtr mesh = item->getMesh();
    Ogre::SubMesh* subMesh = mesh->getSubMesh( 0 );

	size_t numVertices = idx_size; // assuming std::vector or similar

	const Ogre::VertexArrayObjectArray& vaos = subMesh->mVao[Ogre::VpNormal];
	Ogre::VertexArrayObject* vao = vaos[0];
	Ogre::IndexBufferPacked* indexBuffer = vao->getIndexBuffer();

	static const uint16_t quadIndices[] = { 0, 1, 2, 0, 2, 3 };
	int numSplats=idx_size;
	// Lock buffer
	int* indices = (int*)indexBuffer->map(
			0,
			numVertices*6 );

	for (uint32_t i = 0; i < numSplats; ++i)
		for (uint32_t j=0; j<6; ++j)
			indices[i*6+j] = pidx[i]*4+quadIndices[j];

	// Unlock buffer
	indexBuffer->unmap(Ogre::UO_KEEP_PERSISTENT);
}


void updatePointCloudEntity(Ogre::Item* item, intvectorn const & idx)  
{
	_updatePointCloudEntity(item, &idx[0], idx.size()) ; 
}

Ogre::Item* createPointCloudEntity(
    const std::string& meshName,
    const vectorn& xyz,   // half3 packed
    const intvectorn& color,       // ubyte4
    const vectorn& covd,  // half3
    const vectorn& covu,  // half3
    int n)
{
	float *arr_xyz=new float[n*3];
	float *arr_covd=new float[n*3];
	float *arr_covu=new float[n*3];
	uint8_t *arr_color=new uint8_t[n*4];

	for(int i=0; i<n*3; i++)
	{
		arr_xyz[i]=xyz[i];
		arr_covd[i]=covd[i];
		arr_covu[i]=covu[i];

	}
	for(int i=0; i<n*4; i++)
		arr_color[i]=(uint8_t)color[i];

	Ogre::Item* item=createPointCloudEntity_quad(meshName, arr_xyz, arr_color, arr_covd, arr_covu, n);
	delete[] arr_xyz;
	delete[] arr_covd;
	delete[] arr_covu;
	delete[] arr_color;
	return item;
}


#endif
