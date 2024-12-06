#include "stdafx.h"
#include "Mesh.h"
#include "../BaseLib/math/Operator.h"
#include <iostream>
#include <fstream>

#ifndef NO_OGRE
#include "OgreMotionLoader.h"

#include <OgreMeshManager.h>
#include <OgreSubMesh.h>
#if OGRE_VERSION_MINOR >= 12|| OGRE_VERSION_MAJOR>=13

#include <OgreMesh.h>
#endif
#include <OgreRoot.h>
#include <OgreRenderSystem.h>
//#include "../Ogre/PLDPrimCustumSkin.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#endif
using namespace OBJloader;

#ifndef NO_OGRE
Ogre::Vector3 ToOgre(const vector3& v);
void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name);
#endif
static void minVec3(vector3& a, vector3 const& b)
{
	a.x=MIN(a.x, b.x);
	a.y=MIN(a.y, b.y);
	a.z=MIN(a.z, b.z);
}

static void maxVec3(vector3& a, vector3 const& b)
{
	a.x=MAX(a.x, b.x);
	a.y=MAX(a.y, b.y);
	a.z=MAX(a.z, b.z);
}
void scale_Mesh(OBJloader::Mesh& mesh, m_real scaleFactor)
{
	for(int i=0; i<mesh.numVertex(); i++)
		mesh.getVertex(i)*=scaleFactor;
}





#ifndef NO_OGRE
using namespace Ogre;

#include "OgreAxisAlignedBox.h"
#endif

// Ogre::Mesh와 OBJloader::Mesh간의 자료교환을 담당하는 클래스.
namespace OBJloader
{
	class MeshToEntity_DATA
	{
	public:
		std::vector<_tvector<int,Buffer::NUM_BUFFER> > ogreVertices;
		std::vector<_tvector<int,3> > ogreIndexes;
		std::vector<std::vector<int> > ogreVertexHash;
		MeshToEntity_DATA(int numVertex, int numFace)
		{
			ogreVertexHash.resize(numVertex);
			ogreIndexes.resize(numFace);
		}

		~MeshToEntity_DATA()
		{
		}
		int find(_tvector<int, Buffer::NUM_BUFFER> const& i)
		{
			std::vector<int>& hash=ogreVertexHash[i(0)];
			for(int ii=0; ii<hash.size(); ii++)
			{
				if(ogreVertices[hash[ii]]==i)
					return hash[ii];
			}
			return -1;
		}

		int insert(_tvector<int, Buffer::NUM_BUFFER> const& i)
		{
			ASSERT(find(i)==-1);
			ogreVertexHash[i(0)].push_back(ogreVertices.size());
			ogreVertices.push_back(i);
			return ogreVertices.size()-1;
		}

		int numOgreVertex()	{return ogreVertices.size();}
	};
}

MeshToEntity::~MeshToEntity()
{
	delete mData;
}
MeshToEntity::Option::Option( bool _useColor, bool _useNormal, bool _useTexCoord, bool _buildEdgeList, bool _dynamicUpdate)
{
	useColor=_useColor, useNormal=_useNormal, useTexCoord=_useTexCoord; buildEdgeList=_buildEdgeList; dynamicUpdate=_dynamicUpdate;
	meshId=RE::generateUniqueName();
}
MeshToEntity::Option::Option()
{ 
	useColor=false, useNormal=true, useTexCoord=true; buildEdgeList=false; dynamicUpdate=false;
	meshId=RE::generateUniqueName();
}
MeshToEntity::MeshToEntity(const OBJloader::Mesh& mesh, const char* meshId, bool buildEdgeList, bool dynamicUpdate, bool useNormal, bool useTexCoord)
:MeshToEntity(mesh, meshId, Option(false, useNormal, useTexCoord, buildEdgeList, dynamicUpdate))
{
	//std::cout<<"b "<<buildEdgeList <<" d "<<dynamicUpdate<<std::endl;
	//std::cout<<"b "<<mSavedOption.buildEdgeList <<" d "<<mSavedOption.dynamicUpdate<<std::endl;
}
MeshToEntity::MeshToEntity(const OBJloader::Mesh& mesh, const char* meshId, bool buildEdgeList, bool dynamicUpdate, bool useNormal, bool useTexCoord, bool useColor)
:MeshToEntity(mesh, meshId, Option(useColor, useNormal, useTexCoord, buildEdgeList, dynamicUpdate))
{
	//std::cout<<"b "<<buildEdgeList <<" d "<<dynamicUpdate<<std::endl;
	//std::cout<<"b "<<mSavedOption.buildEdgeList <<" d "<<mSavedOption.dynamicUpdate<<std::endl;
}
MeshToEntity::MeshToEntity(const OBJloader::Mesh& mesh, const std::string& meshId, bool buildEdgeList, bool dynamicUpdate, bool useNormal, bool useTexCoord, bool useColor)
:MeshToEntity(mesh, meshId.c_str(), Option(useColor, useNormal, useTexCoord, buildEdgeList, dynamicUpdate))
{
	//std::cout<<"b "<<buildEdgeList <<" d "<<dynamicUpdate<<std::endl;
	//std::cout<<"b "<<mSavedOption.buildEdgeList <<" d "<<mSavedOption.dynamicUpdate<<std::endl;
}
		
		
MeshToEntity::MeshToEntity(const OBJloader::Mesh& mesh, const char* meshId, Option option)
:mInputMesh(mesh)
{
	mEntity=NULL;
#ifdef NO_OGRE
  mData=NULL;
#else
  mData=new MeshToEntity_DATA(mesh.numVertex(), mesh.numFace());

  option.meshId=meshId;
	if(mesh.numTexCoord()==0 && option.useTexCoord)
		option.useTexCoord=false;

	if(mesh.numNormal()==0 && option.useNormal)
		option.useNormal=false;

	if(mesh.numColor()==0 && option.useColor)
		option.useColor=false;

	mSavedOption=option;


	for(int i=0; i<mesh.numFace(); i++)
	{
		for(int j=0; j<3; j++)
		{
			_tvector<int, Buffer::NUM_BUFFER> const& idx=mesh.getFace(i).indexes[j];
			int ogreVertex=mData->find(idx);
			if(ogreVertex==-1)
				ogreVertex=mData->insert(idx);
			mData->ogreIndexes[i][j]=ogreVertex;
		}
	}

	if(MeshManager::getSingleton().resourceExists(meshId))
	{
		Ogre::ResourcePtr ptr=MeshManager::getSingleton().getByName(meshId);
		MeshManager::getSingleton().remove(ptr);
	}
	mMesh= (Ogre::Mesh*)MeshManager::getSingleton().createManual(meshId, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME).get();

	SubMesh *pMeshVertex = mMesh->createSubMesh();

	mMesh->sharedVertexData = new VertexData();
	VertexData* vertexData = mMesh->sharedVertexData;


	// define the vertex format
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);

	if(option.useNormal)
	{
		// normals
		vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
		currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	}

	if(option.useTexCoord)
	{
		// two dimensional texture coordinates
		vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
		currOffset += VertexElement::getTypeSize(VET_FLOAT2);
	}

	if(option.useColor)
	{
		// two dimensional texture coordinates
		vertexDecl->addElement(0, currOffset, VET_COLOUR, VES_DIFFUSE);
		currOffset += VertexElement::getTypeSize(VET_COLOUR);
	}

	HardwareVertexBuffer::Usage usage=HardwareBuffer::HBU_STATIC_WRITE_ONLY;
	HardwareVertexBuffer::LockOptions lockOption=HardwareBuffer::HBL_DISCARD;
	if(option.dynamicUpdate)
	{
		usage=HardwareBuffer::HBU_DYNAMIC ;
		lockOption=HardwareBuffer::HBL_NORMAL ;
	}

	// allocate the vertex buffer
	vertexData->vertexCount = mData->numOgreVertex();
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, usage, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);

	unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(lockOption));
	//float* prPos = static_cast<float*>(vBuf->lock(lockOption));

	// allocate index buffer
	pMeshVertex->indexData->indexCount = mesh.numFace()*3;
	pMeshVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pMeshVertex->indexData->indexCount, usage, false);
	HardwareIndexBufferSharedPtr iBuf = pMeshVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(lockOption));


	// Drawing stuff
	Ogre::AxisAlignedBox box;

	const VertexDeclaration::VertexElementList& elemList = vertexDecl->getElements();

	RenderSystem* rs = Root::getSingleton().getRenderSystem();
	ASSERT(rs);

	for(int i=0; i<mData->numOgreVertex(); i++)
	{

		for (VertexDeclaration::VertexElementList::const_iterator elem = elemList.begin();
			elem != elemList.end(); ++elem)
		{
			float* pFloat=0;
			RGBA* pRGBA=0;
			switch(elem->getType())
			{
			case VET_FLOAT1:
			case VET_FLOAT2:
			case VET_FLOAT3:
				elem->baseVertexPointerToElement(vertex, &pFloat);
				break;
			case VET_COLOUR:
#if OGRE_VERSION_MAJOR<13
			case VET_COLOUR_ABGR:
			case VET_COLOUR_ARGB:
#endif
				elem->baseVertexPointerToElement(vertex, &pRGBA);
				break;
			default:
				// nop ?
				break;
			};

			switch(elem->getSemantic())
			{
			case VES_POSITION:
				{
					vector3 const& pp=mesh.getVertex(mData->ogreVertices[i][Buffer::VERTEX]);
					*pFloat++ = pp.x;
					*pFloat++ = pp.y;
					*pFloat++ = pp.z;
					box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));
				}
				break;
			case VES_NORMAL:
				{
					vector3 const& pp=mesh.getNormal(mData->ogreVertices[i][Buffer::NORMAL]);

					ASSERT(option.useNormal);
					*pFloat++ = pp.x;
					*pFloat++ = pp.y;
					*pFloat++ = pp.z;
				}
				break;
			case VES_TEXTURE_COORDINATES:
				{
					ASSERT(option.useTexCoord);
					ASSERT(VertexElement::getTypeCount(elem->getType())==2);

					vector2 const& pp=mesh.getTexCoord(mData->ogreVertices[i][Buffer::TEXCOORD]);
					*pFloat++ = pp(0);
					*pFloat++ = pp(1);
				}
				break;
			case VES_DIFFUSE:
				{
					ASSERT(option.useColor);
					vector4 const& c=mesh.getColor(mData->ogreVertices[i][Buffer::COLOR]);
					Ogre::ColourValue colour(
						float(c(0)),
						float(c(1)),
						float(c(2)),
						float(c(3)));

					rs->convertColourValue(colour, pRGBA++);
				}
				break;
			default:
				// nop ?
				break;
			};
		}

		vertex += vBuf->getVertexSize();
	}

	for(int i=0; i<mesh.numFace(); i++)
	{
		*pIndices++=(unsigned short)mData->ogreIndexes[i](0);
		*pIndices++=(unsigned short)mData->ogreIndexes[i](1);
		*pIndices++=(unsigned short)mData->ogreIndexes[i](2);
	}
	vBuf->unlock();
	iBuf->unlock();

	// Generate face list
	pMeshVertex ->useSharedVertices=true;

	//mMesh->_setBounds(Ogre::AxisAlignedBox(ToOgre(min), ToOgre(max)), false);
	//mMesh->_setBoundingSphereRadius(min.distance(max)*2);

	if(option.dynamicUpdate)
	{
		mMesh->_setBounds(Ogre::AxisAlignedBox(Ogre::AxisAlignedBox::EXTENT_INFINITE));
		mMesh->_setBoundingSphereRadius(1e5);
	}
	else
	{
		mMesh->_setBounds(box);
		mMesh->_setBoundingSphereRadius(box.getMinimum().distance(box.getMaximum()));
	}

	if(option.buildEdgeList)
	{
		mMesh->freeEdgeList();
		mMesh->buildEdgeList();
	}

	mMesh->load();

#endif
}


void OBJloader::MeshToEntity::updatePositions()
{
  #ifndef NO_OGRE
	Option& option=mSavedOption;

	const VertexElement* posElem = mMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
	HardwareVertexBufferSharedPtr vBuf = mMesh->sharedVertexData->vertexBufferBinding->getBuffer(posElem->getSource());

	unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(HardwareBuffer::HBL_NORMAL ));

	const OBJloader::Mesh& mesh=mInputMesh;

	float* prPos;
	// Drawing stuff
	vector3 min, max;

	min.x=min.y=min.z=FLT_MAX;
	max.x=max.y=max.z=-FLT_MAX;

	Ogre::AxisAlignedBox box;

	for(int i=0; i<mData->numOgreVertex(); i++)
	{
		vector3 const& pp=mesh.getVertex(mData->ogreVertices[i][Buffer::VERTEX]);
		posElem->baseVertexPointerToElement(vertex, &prPos);

		*prPos++ = pp.x;
		*prPos++ = pp.y;
		*prPos++ = pp.z;

		box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));

		vertex += vBuf->getVertexSize();
	}
	vBuf->unlock();

	mMesh->_setBounds(box);
	mMesh->_setBoundingSphereRadius(box.getMinimum().distance(box.getMaximum()));

	if(option.buildEdgeList)
	{
		mMesh->freeEdgeList();
		mMesh->buildEdgeList();
	}
#endif
}
void OBJloader::MeshToEntity::updatePositions(const vector3N & vertices)
{
  #ifndef NO_OGRE
	Option& option=mSavedOption;

	const VertexElement* posElem = mMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
	HardwareVertexBufferSharedPtr vBuf = mMesh->sharedVertexData->vertexBufferBinding->getBuffer(posElem->getSource());

	unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(HardwareBuffer::HBL_NORMAL ));


	float* prPos;
	// Drawing stuff
	vector3 min, max;

	min.x=min.y=min.z=FLT_MAX;
	max.x=max.y=max.z=-FLT_MAX;

	Ogre::AxisAlignedBox box;

	for(int i=0; i<mData->numOgreVertex(); i++)
	{
		vector3 const& pp=vertices(mData->ogreVertices[i][Buffer::VERTEX]);
		posElem->baseVertexPointerToElement(vertex, &prPos);

		*prPos++ = pp.x;
		*prPos++ = pp.y;
		*prPos++ = pp.z;

		box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));

		vertex += vBuf->getVertexSize();
	}
	vBuf->unlock();

	mMesh->_setBounds(box);
	mMesh->_setBoundingSphereRadius(box.getMinimum().distance(box.getMaximum()));

	if(option.buildEdgeList)
	{
		mMesh->freeEdgeList();
		mMesh->buildEdgeList();
	}
#endif
}


void OBJloader::MeshToEntity::updatePositionsAndNormals()
{
  #ifndef NO_OGRE
	Option& option=mSavedOption;

	bool buildEdgeList=option.buildEdgeList;
	option.buildEdgeList=false;
	updatePositions();
	option.buildEdgeList=buildEdgeList;

	if(option.useNormal)
	{

		const VertexElement* normalElem = mMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
		HardwareVertexBufferSharedPtr vBuf = mMesh->sharedVertexData->vertexBufferBinding->getBuffer(normalElem->getSource());

		unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(HardwareBuffer::HBL_NORMAL ));

		const OBJloader::Mesh& mesh=mInputMesh;

		float* prnormal;
		for(int i=0; i<mData->numOgreVertex(); i++)
		{
			normalElem->baseVertexPointerToElement(vertex, &prnormal);

			vector3 const& pp=mesh.getNormal(mData->ogreVertices[i][Buffer::NORMAL]);

			*prnormal++= pp.x;
			*prnormal++= pp.y;
			*prnormal++= pp.z;

			vertex += vBuf->getVertexSize();
		}
		vBuf->unlock();
	}

	if(option.buildEdgeList)
	{
		mMesh->freeEdgeList();
		mMesh->buildEdgeList();
	}
#endif
}

Ogre::Entity* OBJloader::MeshToEntity::createEntity(const char* entityName, const char* materialName)
{
  #ifndef NO_OGRE
	if(RE::ogreSceneManager()->hasEntity(entityName))
		RE::ogreSceneManager()->destroyEntity(entityName);

	Ogre::Entity* entity=RE::ogreSceneManager()->createEntity(entityName, mMesh->getName());
	mEntity=entity;
	entity->setMaterialName(materialName);

	return entity;
#else 
  return NULL;
#endif
}

Ogre::Entity* OBJloader::createMeshEntity(Mesh const& mesh, const char* entityName, const char* materialName)
{
	MeshToEntity::Option opt;
	return OBJloader::createMeshEntity( mesh,  entityName,  materialName,opt);
}

Ogre::Entity* OBJloader::createMeshEntity(Mesh const& mesh, const char* entityName, const char* materialName,MeshToEntity::Option &opt)
{
  #ifndef NO_OGRE
	TString meshId=opt.meshId;

	MeshToEntity mc(mesh, meshId, opt);
	Ogre::Entity* pPlaneEnt = mc.createEntity(entityName, materialName);

	//Ogre::Entity* pPlaneEnt = RE::ogreSceneManager()->createEntity( entityName, meshId.ptr() );
	//pPlaneEnt->setCastShadows(false);
	return pPlaneEnt;
#else
    return NULL;
#endif
}






#ifndef NO_OGRE
TriangleMesh ::TriangleMesh ()
:MeshEntity()
{
	mRenderOp.vertexData=NULL;
	mRenderOp.indexData= NULL;
	mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
	mRenderOp.useIndexes = true; 

	_setMaterial(this, "green");
}

TriangleMesh ::~TriangleMesh ()
{
	delete mRenderOp.indexData;
	delete mRenderOp.vertexData; 

}

void TriangleMesh ::firstInit()
{
	const bool useNormal=true;
	// Initialization stuff 
	delete mRenderOp.vertexData;
	mRenderOp.vertexData=NULL;
	
	delete mRenderOp.indexData;
	mRenderOp.indexData=NULL;

	setVisible(true);

	if(!mRenderOp.vertexData)
	{
		mRenderOp.vertexData= new Ogre::VertexData(); 
		mRenderOp.vertexData->vertexStart = 0; 
		mRenderOp.vertexData->vertexCount = numVertex();

		mRenderOp.indexData=new Ogre::IndexData();
		mRenderOp.indexData->indexStart=0;
		mRenderOp.indexData->indexCount=numFace()*3;

		// define the vertex format
		Ogre::VertexDeclaration* vertexDecl = mRenderOp.vertexData->vertexDeclaration;
		Ogre::VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding; 

		size_t currOffset = 0;
		// positions
		vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
		currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

		if(useNormal)
		{
			// normals
			vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
			currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
		}
		// two dimensional texture coordinates
		vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
		currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

		Ogre::HardwareVertexBufferSharedPtr vbuf =
			Ogre::HardwareBufferManager::getSingleton().createVertexBuffer( 
			vertexDecl->getVertexSize(0), 
			mRenderOp.vertexData->vertexCount, 
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

		bind->setBinding(0, vbuf); 

		// allocate index buffer
		mRenderOp.indexData->indexBuffer = 
			Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
			Ogre::HardwareIndexBuffer::IT_16BIT, 
			mRenderOp.indexData->indexCount, 
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);


	}



	// Drawing stuff 

	vector3 min, max;

	Ogre::HardwareVertexBufferSharedPtr vbuf =mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
	Ogre::Real *prPos = (Ogre::Real*)(vbuf ->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

	Ogre::HardwareIndexBufferSharedPtr iBuf = mRenderOp.indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));


	min.x=min.y=min.z=FLT_MAX;
	max.x=max.y=max.z=-FLT_MAX;
	for(int i=0; i<numVertex(); i++)
	{
		vector3& pp=getVertex(i);
		*prPos++ = pp.x; 
		*prPos++ = pp.y; 
		*prPos++ = pp.z; 

		min.x=std::min(pp.x, min.x);
		min.y=std::min(pp.x, min.y);
		min.z=std::min(pp.x, min.z);

		max.x=std::max(pp.x, max.x);
		max.y=std::max(pp.x, max.y);
		max.z=std::max(pp.x, max.z);

		if(useNormal)
		{
			*prPos++= getNormal(i).x;
			*prPos++= getNormal(i).y;
			*prPos++= getNormal(i).z;
		}

		*prPos++ = getTexCoord(i)(0);
		*prPos++ = getTexCoord(i)(1);
	}

	for(int i=0; i<numFace(); i++)
	{
		*pIndices++=(unsigned short)getFace(i).vertexIndex(0);
		*pIndices++=(unsigned short)getFace(i).vertexIndex(1);
		*pIndices++=(unsigned short)getFace(i).vertexIndex(2);
	}
	vbuf->unlock(); 
	iBuf->unlock();

	//mBox.setExtents(ToOgre(min), ToOgre(max)); // 2D폴리곤에서 이상동작함.. 여유공간을 두어야할듯.
	mBox.setInfinite();
}

void TriangleMesh::update()
{
	firstInit();
}
MeshLineStrip::MeshLineStrip()
:MeshEntity()
{
	edges=NULL;
   mRenderOp.vertexData = NULL;
   _setMaterial(this, "solidblue");    
}

MeshLineStrip::~MeshLineStrip()
{
	delete edges;
}

void MeshLineStrip::_createGraph()
{	
	// construct mesh graph
	delete edges;
	edges=new EdgeConnectivity(*this);
}

void MeshLineStrip::firstInit()
{
	_createGraph();

	update();
}
void MeshLineStrip::update()
{
	// when vertex position is changed you can call update();
	delete mRenderOp.vertexData;
	mRenderOp.vertexData= new Ogre::VertexData(); 

   mRenderOp.indexData = 0; 
   mRenderOp.vertexData->vertexCount = edges->mMeshGraph.numEdges()*2; 
   mRenderOp.vertexData->vertexStart = 0; 
   mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;//, OT_LINE_STRIP 
   mRenderOp.useIndexes = false; 

   if(edges->mMeshGraph.numEdges()==0)
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
   int size = edges->mMeshGraph.numEdges();

   vector3 minv(FLT_MAX, FLT_MAX, FLT_MAX);
   vector3 maxv(-FLT_MAX, -FLT_MAX, -FLT_MAX);

   Ogre::Real *prPos = (Ogre::Real*)(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

   EdgeConnectivity::edgeT e;
   TUGL_for_all_edges(e, edges->mMeshGraph)
   {
	   for(int i=0; i<2; i++)
	   {
		   vector3 const& pp=(i==0)?getVertex(e.v1().index()):getVertex(e.v2().index());
		  *prPos++ = pp.x; 
		  *prPos++ = pp.y; 
		  *prPos++ = pp.z;

		  minVec3(minv, pp);
		  maxVec3(maxv, pp);
	   } 
   }

   vbuf->unlock(); 

   mBox.setExtents(ToOgre(minv), ToOgre(maxv));
}
void MeshLineStripReduced::update()
{
	m_real thickness=1.0;

	// when vertex position is changed you can call update();
	delete mRenderOp.vertexData;
	mRenderOp.vertexData= new Ogre::VertexData(); 

	mRenderOp.indexData = 0; 
	mRenderOp.vertexData->vertexCount = numFace()*6;
	mRenderOp.vertexData->vertexStart = 0; 
	mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;//, OT_LINE_STRIP 
	mRenderOp.useIndexes = false; 

	if(numFace()==0)
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

	vector3 minv(FLT_MAX, FLT_MAX, FLT_MAX);
	vector3 maxv(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	Ogre::Real *prPos = (Ogre::Real*)(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

	vector3 pos[3], offsetPos[3];

	for(int face=0; face<numFace(); face++)
	{
		for(int i=0; i<3; i++)
		{
		   pos[i]=getVertex(getFace(face).vertexIndex(i));
		   minVec3(minv, pos[i]);
		   maxVec3(maxv, pos[i]);
		}

		// 내심 구하기.
		m_real d01=pos[0].distance(pos[1]);
		m_real d12=pos[1].distance(pos[2]);
		m_real d20=pos[2].distance(pos[0]);


		vector3 center=(pos[0]*d12+pos[1]*d20+pos[2]*d01)/(d01+d12+d20);

		if(d01<thickness || d12<thickness || d20 <thickness)
		{
			center=(pos[0]+pos[1]+pos[2])*0.333333;
			for(int i=0; i<3; i++)
				// thickness/offset=sin theta;
				offsetPos[i]=center + (pos[i]-center)*0.8;
		}
		else
		{
			//// scaling
			//pos[0].interpolate(0.1, pos[0], center);
			//pos[1].interpolate(0.1, pos[1], center);
			//pos[2].interpolate(0.1, pos[2], center);
			//for(int i=0; i<3; i++)
			//{
			//  *prPos++ = pos[i].x; 
			//  *prPos++ = pos[i].y; 
			//  *prPos++ = pos[i].z;
			//  *prPos++ = pos[(i+1)%3].x; 
			//  *prPos++ = pos[(i+1)%3].y; 
			//  *prPos++ = pos[(i+1)%3].z;
			//} 

			// offsetting (CCW-order를 가정하였다.)

			//                                  c thick  v1
			//       v2     v1          //       \     | 
			//		  +----+			//		  +----+
			//        |   /				//         \   |
			//        |  /				//  offset  \  |
			//        | /				//           \ |
			//        |/				//        	  \|
			//        v0				//            v0   
			vector3 normal;
			normal.cross(pos[1]-pos[0], pos[2]-pos[0]);
			normal.normalize();

			for(int i=0; i<3; i++)
			{
				vector3 v01, c0;
				v01.difference(pos[i], pos[(i+1)%3]);
				c0.difference(pos[i], center);
				c0.normalize();

				quater q;
				q.axisToAxis(v01, c0);
				m_real theta=q.rotationAngleAboutAxis(normal);

				// thickness/offset=sin theta;
				offsetPos[i]=pos[i]+c0*(thickness/sin(theta));
			}
		}

		for(int i=0; i<3; i++)
		{
		  *prPos++ = offsetPos[i].x; 
		  *prPos++ = offsetPos[i].y; 
		  *prPos++ = offsetPos[i].z;
		  *prPos++ = offsetPos[(i+1)%3].x; 
		  *prPos++ = offsetPos[(i+1)%3].y; 
		  *prPos++ = offsetPos[(i+1)%3].z;
		}
	}

	vbuf->unlock(); 

	mBox.setExtents(ToOgre(minv), ToOgre(maxv));
}
#endif
void testOBJloader()
{
	OBJloader::Mesh temp;
	//temp.loadObj("../resource/mesh/iguana_pskinned.obj");
	//temp.loadObj("../resource/mesh/iguana_bound.obj");
	//temp.loadObj("../resource/mesh/iguana_high2.obj");
	//temp.loadObj("../resource/mesh/iguana_high.obj");
	//temp.loadObj("../resource/mesh/untitled.obj");
	//OBJloader::createBox(temp, 100,100,100);
	//OBJloader::createCylinder(temp, 100,100,10);
	temp.loadObj("../resource/mesh/footmodel.obj");

	//temp.vertFlipUV();
	//temp.calculateVertexNormal();
	//temp.mergeDuplicateVertices();
	//temp.saveObj("../resource/mesh/iguana_high2.obj", true,true);

	OBJloader::MeshToEntity::Option o;
	o.buildEdgeList=true;
	o.useTexCoord=false;
	o.useNormal=true;

	//TString mat="lambert6";
	TString mat="white";

#ifndef NO_OGRE
	RE::ogreRootSceneNode()->attachObject(OBJloader::createMeshEntity(temp, RE::generateUniqueName(), mat,o));
#endif
}

#if OGRE_VERSION_MINOR >= 12|| OGRE_VERSION_MAJOR>=13

void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name);
void OBJloader::MeshEntity::setMaterial(const char* name)
{
	_setMaterial(this, name);
}
#endif

void OBJloader::MeshToEntity::removeAllResources() // entity and mesh will be removed. This is not automatically called in dtor.
{
  #ifndef NO_OGRE
	TString meshId=mSavedOption.meshId;
	if(MeshManager::getSingleton().resourceExists(meshId.ptr()))
	{
		if(mEntity)
			RE::ogreSceneManager()->destroyMovableObject(mEntity);
		Ogre::ResourcePtr ptr=MeshManager::getSingleton().getByName(meshId.ptr());
		MeshManager::getSingleton().remove(ptr);
	}
#endif
}
