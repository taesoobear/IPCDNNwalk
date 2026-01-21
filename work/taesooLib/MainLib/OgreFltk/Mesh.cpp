#include "stdafx.h"
#include "Mesh.h"
#include "../BaseLib/math/Operator.h"
#include <iostream>
#include <fstream>

#ifndef NO_OGRE

#include <OgreMeshManager.h>

// v1
#include <OgreSubMesh.h>
#include <OgreMesh.h>

// v2
#include <OgreSubMesh2.h>
#include <OgreMesh2.h>


#include <OgreRoot.h>
#include <OgreRenderSystem.h>
//#include "../Ogre/PLDPrimCustumSkin.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMeshManager2.h>
#include <OgreItem.h>
#include <Vao/OgreVaoManager.h>
#include "renderer.h"
#endif
using namespace OBJloader;

#ifndef NO_OGRE
Ogre::Vector3 ToOgre(const vector3& v);
void _setMaterial(Ogre::v1::SimpleRenderable* ptr, const char* name);
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
		
		
MeshToEntity::~MeshToEntity()
{
	for(int i=0; i<mData.size(); i++)
		delete mData[i];
}
#ifdef NO_OGRE
int MeshToEntity::getRawData(Mesh const& mesh, int ielt, matrixn& vertices, intvectorn& indices) 
{
	MeshToEntity_DATA& data=*mData[ielt];
	auto& option=mSavedOption;

	int offset=0;
	offset+=3;

	if(option.useNormal)
		offset+=3;
	if(option.useTexCoord)
		offset+=2;
	if(option.useColor)
		offset+=4;

	vertices.setSize(data.numOgreVertex(), offset);
	for(int i=0; i<data.numOgreVertex(); i++)
	{
		double* pFloat=&vertices(i,0);
		vector3 const& pp=mesh.getVertex(data.ogreVertices[i][Buffer::VERTEX]);
		*pFloat++ = pp.x;
		*pFloat++ = pp.y;
		*pFloat++ = pp.z;

		if(option.useNormal)
		{
			vector3 const& pp=mesh.getNormal(data.ogreVertices[i][Buffer::NORMAL]);
			*pFloat++ = pp.x;
			*pFloat++ = pp.y;
			*pFloat++ = pp.z;
		}

		if(option.useTexCoord)
		{
			vector2 const& pp=mesh.getTexCoord(data.ogreVertices[i][Buffer::TEXCOORD]);
			*pFloat++ = pp(0);
			*pFloat++ = pp(1);
		}
		if(option.useColor)
		{
			vector4 const& c=mesh.getColor(data.ogreVertices[i][Buffer::COLOR]);
			*pFloat++ = c(0);
			*pFloat++ = c(1);
			*pFloat++ = c(2);
			*pFloat++ = c(3);
		}
	}

	const Geometry* geom=dynamic_cast<const Geometry*>(&mesh);
	int startFace=0;
	int endFace=mesh.numFace();
	if(geom)
	{
		startFace=mesh.faceGroups.start(ielt);
		endFace=mesh.faceGroups.end(ielt);
	}

	int numFace=endFace-startFace;
	unsigned int indexCount = numFace*3;
	indices.setSize(indexCount);
	{
		int* pIndices=&indices(0);
		for(int i=0; i<numFace; i++)
		{
			*pIndices++=data.ogreIndexes[i](0);
			*pIndices++=data.ogreIndexes[i](1);
			*pIndices++=data.ogreIndexes[i](2);
		}
	}
	return mData.size();
}
void MeshToEntity::getRawData_pos(Mesh const& mesh, int ielt, matrixn& vertices) 
{
	MeshToEntity_DATA& data=*mData[ielt];
	auto& option=mSavedOption;

	int offset=3;

	vertices.setSize(data.numOgreVertex(), offset);
	for(int i=0; i<data.numOgreVertex(); i++)
	{
		double* pFloat=&vertices(i,0);
		vector3 const& pp=mesh.getVertex(data.ogreVertices[i][Buffer::VERTEX]);
		*pFloat++ = pp.x;
		*pFloat++ = pp.y;
		*pFloat++ = pp.z;
	}
}
void MeshToEntity::getRawData_posAndNormal(Mesh const& mesh, int ielt, matrixn& vertices) 
{
	MeshToEntity_DATA& data=*mData[ielt];
	auto& option=mSavedOption;

	int offset=0;
	offset+=3;

	if(option.useNormal)
		offset+=3;

	vertices.setSize(data.numOgreVertex(), offset);
	for(int i=0; i<data.numOgreVertex(); i++)
	{
		double* pFloat=&vertices(i,0);
		vector3 const& pp=mesh.getVertex(data.ogreVertices[i][Buffer::VERTEX]);
		*pFloat++ = pp.x;
		*pFloat++ = pp.y;
		*pFloat++ = pp.z;

		if(option.useNormal)
		{
			vector3 const& pp=mesh.getNormal(data.ogreVertices[i][Buffer::NORMAL]);
			*pFloat++ = pp.x;
			*pFloat++ = pp.y;
			*pFloat++ = pp.z;
		}
	}
}

void MeshToEntity::_constructSubMesh(int ielt, int startFace, int endFace, const Mesh& mesh, const char* meshId, const MeshToEntity::Option & option)
#else
void MeshToEntity::_constructSubMesh(int ielt, int startFace, int endFace, const Mesh& mesh, const char* meshId, const MeshToEntity::Option & option, Ogre::Aabb& box)
#endif
{
	MeshToEntity_DATA& data=*mData[ielt];

	int numFace=endFace-startFace;
	for(int i=0; i<numFace; i++)
	{
		for(int j=0; j<3; j++)
		{
			_tvector<int, Buffer::NUM_BUFFER> const& idx=mesh.getFace(i+startFace).indexes[j];
			int ogreVertex=data.find(idx);
			if(ogreVertex==-1)
				ogreVertex=data.insert(idx);
			data.ogreIndexes[i][j]=ogreVertex;
		}
	}

#ifndef NO_OGRE

	Ogre::Root *root = RE::renderer().getRoot();
	Ogre::RenderSystem *renderSystem = root->getRenderSystem();
	Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

	//Create one submesh
	Ogre::SubMesh *subMesh = mMesh->createSubMesh();

	//Vertex declaration
	size_t color_offset=0;
	size_t offset=0;
	Ogre::VertexElement2Vec vertexElements;
	vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_POSITION ) );
	offset+=sizeof(float)*3;

	if(option.useNormal)
	{
		vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT3, Ogre::VES_NORMAL ) );
		offset+=sizeof(float)*3;
	}
	if(option.useTexCoord)
	{
		vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES ) );
		offset+=sizeof(float)*2;
	}
	if(option.useColor)
	{
		vertexElements.push_back( Ogre::VertexElement2( Ogre::VET_COLOUR, Ogre::VES_DIFFUSE ) );
		color_offset=offset;
		offset += sizeof(RGBA);
	}

	Ogre::VertexBufferPacked *vertexBuffer = 0;
	Ogre::BufferType bt=Ogre::BT_IMMUTABLE;
	if(option.dynamicUpdate)
		//bt=Ogre::BT_DYNAMIC_PERSISTENT;
		bt=Ogre::BT_DEFAULT;

	unsigned char* vertex=(unsigned char*)OGRE_MALLOC_SIMD(offset*data.numOgreVertex(), Ogre::MEMCATEGORY_GEOMETRY );
	_updateVertices(&data, box, vertex, option);

	try
	{
		vertexBuffer = vaoManager->createVertexBuffer( vertexElements, data.numOgreVertex(),
				bt,
				vertex, true );
	}
	catch( Ogre::Exception &e )
	{
		// Important: Please note that we passed keepAsShadow = true to createVertexBuffer,
		// thus Ogre will free the pointer. However had we passed keepAsShadow = false,
		// it would be YOUR responsability to free the pointer, not Ogre's
		OGRE_FREE_SIMD( vertexBuffer, Ogre::MEMCATEGORY_GEOMETRY );
		vertexBuffer = 0;
		throw e;
	}

	if(option.dynamicUpdate)
		data.mVertexBuffer=vertex;

	Ogre::IndexBufferPacked *indexBuffer = 0;

	unsigned int indexCount = numFace*3;

	if(data.numOgreVertex()<65535)
	{
		Ogre::uint16* indices=(Ogre::uint16*) OGRE_MALLOC_SIMD( sizeof(Ogre::uint16) *indexCount, Ogre::MEMCATEGORY_GEOMETRY);

		Ogre::uint16* pIndices = indices;
		for(int i=0; i<numFace; i++)
		{
			*pIndices++=(Ogre::uint16)data.ogreIndexes[i](0);
			*pIndices++=(Ogre::uint16)data.ogreIndexes[i](1);
			*pIndices++=(Ogre::uint16)data.ogreIndexes[i](2);
		}

		try
		{
			indexBuffer = vaoManager->createIndexBuffer( Ogre::IndexBufferPacked::IT_16BIT,
					indexCount,
					Ogre::BT_IMMUTABLE,
					indices, true );
		}
		catch( Ogre::Exception &e )
		{
			OGRE_FREE_SIMD( indexBuffer, Ogre::MEMCATEGORY_GEOMETRY );
			indexBuffer = 0;
			throw e;
		}
		//OGRE_FREE_SIMD( indices, Ogre::MEMCATEGORY_GEOMETRY ); ogre does this.
	}
	else
	{
		Ogre::uint32* indices=(Ogre::uint32*) OGRE_MALLOC_SIMD( sizeof(Ogre::uint32) *indexCount, Ogre::MEMCATEGORY_GEOMETRY);
		Ogre::uint32* pIndices = indices;
		for(int i=0; i<numFace; i++)
		{
			*pIndices++=(Ogre::uint32)data.ogreIndexes[i](0);
			*pIndices++=(Ogre::uint32)data.ogreIndexes[i](1);
			*pIndices++=(Ogre::uint32)data.ogreIndexes[i](2);
		}
		try
		{
			indexBuffer = vaoManager->createIndexBuffer( Ogre::IndexBufferPacked::IT_32BIT,
					indexCount,
					Ogre::BT_IMMUTABLE,
					indices, true );
		}
		catch( Ogre::Exception &e )
		{
			OGRE_FREE_SIMD( indexBuffer, Ogre::MEMCATEGORY_GEOMETRY );
			indexBuffer = 0;
			throw e;
		}
		//OGRE_FREE_SIMD( indices, Ogre::MEMCATEGORY_GEOMETRY ); ogre does this
	}

	//Now the Vao. We'll just use one vertex buffer source (multi-source not working yet)
	Ogre::VertexBufferPackedVec vertexBuffers;
	vertexBuffers.push_back( vertexBuffer );
	Ogre::VertexArrayObject *vao = vaoManager->createVertexArrayObject(
			vertexBuffers, indexBuffer, Ogre::OT_TRIANGLE_LIST );

	//Each Vao pushed to the vector refers to an LOD level.
	//Must be in sync with mesh->mLodValues & mesh->mNumLods if you use more than one level
	subMesh->mVao[Ogre::VpNormal].push_back( vao );
	//Use the same geometry for shadow casting.
	subMesh->mVao[Ogre::VpShadow].push_back( vao );



	const Geometry* geom=dynamic_cast<const Geometry*>(&mesh);
	if(geom)
	{
		auto& mat=geom->element(ielt).material;
		if(mat.length()>6 && mat.substr(0, 6)=="@color")
		{
			printf("creating material %s\n", mat.c_str());
			vectorn color;
			color.parseString(4, mat.substr(6));
			assert(color.size()>=3);

			RE::renderer().createMaterial(mat.c_str(), color.toVector3(0), color.toVector3(0),10);

			subMesh->setMaterialName(mat.c_str());
		}
		else if(mat.length()>0)
		{
			printf("set material '%s'\n", mat.c_str());
			subMesh->setMaterialName(mat.c_str());
		}
	}

#endif
}
// Ogre::Mesh와 OBJloader::Mesh간의 자료교환을 담당하는 클래스.
MeshToEntity::MeshToEntity(const OBJloader::Mesh& mesh, const char* meshId, bool buildEdgeList, bool dynamicUpdate, bool useNormal, bool useTexCoord)
	:MeshToEntity(mesh, meshId, OBJloader::MeshToEntity::Option(false, useNormal, useTexCoord, buildEdgeList, dynamicUpdate))
{
	//std::cout<<"b "<<buildEdgeList <<" d "<<dynamicUpdate<<std::endl;
	//std::cout<<"b "<<mSavedOption.buildEdgeList <<" d "<<mSavedOption.dynamicUpdate<<std::endl;
}
MeshToEntity::MeshToEntity(const OBJloader::Mesh& mesh, const char* meshId, bool buildEdgeList, bool dynamicUpdate, bool useNormal, bool useTexCoord, bool useColor)
	:MeshToEntity(mesh, meshId, OBJloader::MeshToEntity::Option(useColor, useNormal, useTexCoord, buildEdgeList, dynamicUpdate))
{
	//std::cout<<"b "<<buildEdgeList <<" d "<<dynamicUpdate<<std::endl;
	//std::cout<<"b "<<mSavedOption.buildEdgeList <<" d "<<mSavedOption.dynamicUpdate<<std::endl;
}
MeshToEntity::MeshToEntity(const OBJloader::Mesh& mesh, const std::string& meshId, bool buildEdgeList, bool dynamicUpdate, bool useNormal, bool useTexCoord, bool useColor)
	:MeshToEntity(mesh, meshId.c_str(), OBJloader::MeshToEntity::Option(useColor, useNormal, useTexCoord, buildEdgeList, dynamicUpdate))
{
	//std::cout<<"b "<<buildEdgeList <<" d "<<dynamicUpdate<<std::endl;
	//std::cout<<"b "<<mSavedOption.buildEdgeList <<" d "<<mSavedOption.dynamicUpdate<<std::endl;
}


MeshToEntity::MeshToEntity(const OBJloader::Mesh& mesh, const char* meshId, OBJloader::MeshToEntity::Option option)
	:mInputMesh(mesh)
{
	mEntity=NULL;

	option.meshId=meshId;
	if(mesh.numTexCoord()==0 && option.useTexCoord)
		option.useTexCoord=false;

	if(mesh.numNormal()==0 && option.useNormal)
		option.useNormal=false;

	if(mesh.numColor()==0 && option.useColor)
		option.useColor=false;

	mSavedOption=option;


#ifndef NO_OGRE
	Ogre::Root *root = RE::renderer().getRoot();
	Ogre::RenderSystem *renderSystem = root->getRenderSystem();
	Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();
	
	if(Ogre::MeshManager::getSingleton().resourceExists(meshId))
	{
		Ogre::ResourcePtr ptr=Ogre::MeshManager::getSingleton().getByName(meshId);
		Ogre::MeshManager::getSingleton().remove(ptr);
	}
	//Create the mesh
	mMesh= Ogre::MeshManager::getSingleton().createManual(meshId, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME).get();

	Ogre::Aabb box;
#endif
	const Geometry* geom=dynamic_cast<const Geometry*>(&mesh);
	if(geom)
	{
		mData.resize(geom->numElements());
		for(int ielt=0; ielt<geom->numElements(); ielt++)
		{
			mData[ielt]=new OBJloader::MeshToEntity_DATA(mesh.numVertex(), mesh.numFace());
			int startFace=mesh.faceGroups.start(ielt);
			int endFace=mesh.faceGroups.end(ielt);
#ifdef NO_OGRE
			_constructSubMesh(ielt, startFace, endFace, mesh, meshId, option);
#else
			_constructSubMesh(ielt, startFace, endFace, mesh, meshId, option, box);
#endif
		}
	}
	else
	{
		mData.resize(1);
		mData[0]=new OBJloader::MeshToEntity_DATA(mesh.numVertex(), mesh.numFace());
#ifdef NO_OGRE
		_constructSubMesh(0, 0, mesh.numFace(), mesh, meshId, option);
#else
		_constructSubMesh(0, 0, mesh.numFace(), mesh, meshId, option, box);
#endif
	}
#ifndef NO_OGRE

	//Set the bounds to get frustum culling and LOD to work correctly.
	mMesh->_setBounds( box, false );
	mMesh->_setBoundingSphereRadius( box.getRadius());

	//OGRE_FREE_SIMD( vertex, Ogre::MEMCATEGORY_GEOMETRY ); ogre does this.

#endif
}


void OBJloader::MeshToEntity::updatePositions()
{
	updatePositionsAndNormals();
}

void OBJloader::MeshToEntity::updatePositions(const vector3N & vertices)
{
#ifndef NO_OGRE
	Msg::error("OBJloader::MeshToEntity::updatePositions(const vector3N) not ported to ogre-next yet");
#endif
}


void OBJloader::MeshToEntity::setBuildEdgeList(bool value)
{
	MeshToEntity::Option& option=mSavedOption;
	option.buildEdgeList=value;
}
void OBJloader::MeshToEntity::updatePositionsAndNormals()
{
#ifndef NO_OGRE
	MeshToEntity::Option& option=mSavedOption;

	if(!mSavedOption.dynamicUpdate)
	{
		Msg::error("dynamicUpdate option not set when creating this.\n");
	}


	int isubMesh=0;
	auto& partialVertexBuffer=mMesh->getSubMesh(isubMesh)->mVao[Ogre::VpNormal][0]->getVertexBuffers()[0];

	Ogre::Aabb box;
	auto* data=mData[isubMesh];
	_updateVertices(data, box, data->mVertexBuffer, mSavedOption);
	partialVertexBuffer->upload( data->mVertexBuffer, 0, data->numOgreVertex() );

	mMesh->_setBounds( box, false );
	mMesh->_setBoundingSphereRadius( box.getRadius());
#endif
}

Ogre::Item* OBJloader::MeshToEntity::createEntity(const char* entityName, const char* materialName)
{
#ifndef NO_OGRE
	//if(mEntity)
	//	RE::ogreSceneManager()->destroyItem(mEntity);

	Ogre::Item* entity=RE::ogreSceneManager()->createItem(mMesh->getName());
	mEntity=entity;
	if (TString("use_vertexcolor")!=materialName)
		entity->setDatablockOrMaterialName(materialName);

	return entity;
#else 
	return NULL;
#endif
}

Ogre::Item* OBJloader::createMeshEntity(Mesh const& mesh, const char* entityName, const char* materialName)
{
	MeshToEntity::Option opt;
	return OBJloader::createMeshEntity( mesh,  entityName,  materialName,opt);
}

Ogre::Item* OBJloader::createMeshEntity(Mesh const& mesh, const char* entityName, const char* materialName,MeshToEntity::Option &opt)
{
#ifndef NO_OGRE
	TString meshId=opt.meshId;

	MeshToEntity mc(mesh, meshId, opt);
	Ogre::Item* pPlaneEnt = mc.createEntity(entityName, materialName);

	//Ogre::Entity* pPlaneEnt = RE::ogreSceneManager()->createEntity( entityName, meshId.ptr() );
	//pPlaneEnt->setCastShadows(false);
	return NULL;
#else
	return NULL;
#endif
}



#ifndef NO_OGRE
void OBJloader::MeshToEntity::_updateVertices(MeshToEntity_DATA* data,  Ogre::Aabb & box, unsigned char* vertex, const OBJloader::MeshToEntity::Option& option) const
{
	const OBJloader::Mesh& mesh=mInputMesh;
	size_t color_offset=0;
	size_t offset=sizeof(float)*3;

	unsigned char* cvertex=vertex;
	if(option.useNormal)
		offset+=sizeof(float)*3;
	if(option.useTexCoord)
		offset+=sizeof(float)*2;
	if(option.useColor)
	{
		color_offset=offset;
		offset += sizeof(RGBA);
	}

	// Drawing stuff
	for(int i=0; i<data->numOgreVertex(); i++)
	{
		float* pFloat=(float*)cvertex;
		vector3 const& pp=mesh.getVertex(data->ogreVertices[i][Buffer::VERTEX]);
		*pFloat++ = pp.x;
		*pFloat++ = pp.y;
		*pFloat++ = pp.z;
		box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));

		if(option.useNormal)
		{
			vector3 const& pp=mesh.getNormal(data->ogreVertices[i][Buffer::NORMAL]);
			*pFloat++ = pp.x;
			*pFloat++ = pp.y;
			*pFloat++ = pp.z;
		}

		if(option.useTexCoord)
		{
			vector2 const& pp=mesh.getTexCoord(data->ogreVertices[i][Buffer::TEXCOORD]);
			*pFloat++ = pp(0);
			*pFloat++ = pp(1);
		}
		if(option.useColor)
		{
			RGBA* pRGBA=(RGBA*)(cvertex+color_offset);

			vector4 const& c=mesh.getColor(data->ogreVertices[i][Buffer::COLOR]);
			Ogre::ColourValue colour(
					float(c(0)),
					float(c(1)),
					float(c(2)),
					float(c(3)));

#if OGRE_PLATFORM != OGRE_PLATFORM_WIN32 && OGRE_PLATFORM != OGRE_PLATFORM_WINRT
			*pRGBA++=colour.getAsARGB();
#else
			*pRGBA++=colour.getAsABGR();
#endif
		}

		cvertex += offset;
	}
	assert(cvertex==vertex+(offset*data->numOgreVertex()));
}
#endif


void OBJloader::MeshToEntity::removeAllResources() // entity and mesh will be removed. This is not automatically called in dtor.
{
#ifndef NO_OGRE
	TString meshId=mSavedOption.meshId;
	if(Ogre::MeshManager::getSingleton().resourceExists(meshId.tostring()))
	{
		if(mEntity)
			RE::ogreSceneManager()->destroyMovableObject(mEntity);
		Ogre::ResourcePtr ptr=Ogre::MeshManager::getSingleton().getByName(meshId.ptr());
		Ogre::MeshManager::getSingleton().remove(ptr);
	}
#endif
}












#ifndef NO_OGRE
TriangleMesh ::TriangleMesh ()
:MeshEntity()
{
	mRenderOp.vertexData=NULL;
	mRenderOp.indexData= NULL;
	mRenderOp.operationType = Ogre::OT_TRIANGLE_LIST;
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
		mRenderOp.vertexData= new Ogre::v1::VertexData(&Ogre::v1::HardwareBufferManager::getSingleton());
		mRenderOp.vertexData->vertexStart = 0; 
		mRenderOp.vertexData->vertexCount = numVertex();

		mRenderOp.indexData=new Ogre::v1::IndexData();
		mRenderOp.indexData->indexStart=0;
		mRenderOp.indexData->indexCount=numFace()*3;

		// define the vertex format
		Ogre::v1::VertexDeclaration* vertexDecl = mRenderOp.vertexData->vertexDeclaration;
		Ogre::v1::VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding; 

		size_t currOffset = 0;
		// positions
		vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
		currOffset += Ogre::v1::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

		if(useNormal)
		{
			// normals
			vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
			currOffset += Ogre::v1::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
		}
		// two dimensional texture coordinates
		vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
		currOffset += Ogre::v1::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

		Ogre::v1::HardwareVertexBufferSharedPtr vbuf =
			Ogre::v1::HardwareBufferManager::getSingleton().createVertexBuffer( 
			vertexDecl->getVertexSize(0), 
			mRenderOp.vertexData->vertexCount, 
			Ogre::v1::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

		bind->setBinding(0, vbuf); 

		// allocate index buffer
		mRenderOp.indexData->indexBuffer = 
			Ogre::v1::HardwareBufferManager::getSingleton().createIndexBuffer(
			Ogre::v1::HardwareIndexBuffer::IT_16BIT, 
			mRenderOp.indexData->indexCount, 
			Ogre::v1::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);


	}



	// Drawing stuff 

	vector3 min, max;

	Ogre::v1::HardwareVertexBufferSharedPtr vbuf =mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
	Ogre::Real *prPos = (Ogre::Real*)(vbuf ->lock(Ogre::v1::HardwareBuffer::HBL_DISCARD)); 

	Ogre::v1::HardwareIndexBufferSharedPtr iBuf = mRenderOp.indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(Ogre::v1::HardwareBuffer::HBL_DISCARD));


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
	mRenderOp.vertexData= new Ogre::v1::VertexData(&Ogre::v1::HardwareBufferManager::getSingleton());

   mRenderOp.indexData = 0; 
   mRenderOp.vertexData->vertexCount = edges->mMeshGraph.numEdges()*2; 
   mRenderOp.vertexData->vertexStart = 0; 
   mRenderOp.operationType = Ogre::OT_LINE_LIST;//, OT_LINE_STRIP 
   mRenderOp.useIndexes = false; 

   if(edges->mMeshGraph.numEdges()==0)
	   return;

   Ogre::v1::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration; 
   Ogre::v1::VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding; 

   decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION); 

   Ogre::v1::HardwareVertexBufferSharedPtr vbuf = 
      Ogre::v1::HardwareBufferManager::getSingleton().createVertexBuffer( 
         decl->getVertexSize(POSITION_BINDING), 
         mRenderOp.vertexData->vertexCount, 
         Ogre::v1::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

   bind->setBinding(POSITION_BINDING, vbuf); 

   // Drawing stuff 
   int size = edges->mMeshGraph.numEdges();

   vector3 minv(FLT_MAX, FLT_MAX, FLT_MAX);
   vector3 maxv(-FLT_MAX, -FLT_MAX, -FLT_MAX);

   Ogre::Real *prPos = (Ogre::Real*)(vbuf->lock(Ogre::v1::HardwareBuffer::HBL_DISCARD)); 

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
	mRenderOp.vertexData= new Ogre::v1::VertexData(&Ogre::v1::HardwareBufferManager::getSingleton());

	mRenderOp.indexData = 0; 
	mRenderOp.vertexData->vertexCount = numFace()*6;
	mRenderOp.vertexData->vertexStart = 0; 
	mRenderOp.operationType = Ogre::OT_LINE_LIST;//, OT_LINE_STRIP 
	mRenderOp.useIndexes = false; 

	if(numFace()==0)
	   return;

	Ogre::v1::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration; 
	Ogre::v1::VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding; 

	decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION); 

	Ogre::v1::HardwareVertexBufferSharedPtr vbuf = 
	  Ogre::v1::HardwareBufferManager::getSingleton().createVertexBuffer( 
		 decl->getVertexSize(POSITION_BINDING), 
		 mRenderOp.vertexData->vertexCount, 
		 Ogre::v1::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

	bind->setBinding(POSITION_BINDING, vbuf); 

	// Drawing stuff 

	vector3 minv(FLT_MAX, FLT_MAX, FLT_MAX);
	vector3 maxv(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	Ogre::Real *prPos = (Ogre::Real*)(vbuf->lock(Ogre::v1::HardwareBuffer::HBL_DISCARD)); 

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


#ifndef NO_OGRE

void _setMaterial(Ogre::v1::SimpleRenderable* ptr, const char* name);
void OBJloader::MeshEntity::setMaterial(const char* name)
{
	_setMaterial(this, name);
}

#endif
