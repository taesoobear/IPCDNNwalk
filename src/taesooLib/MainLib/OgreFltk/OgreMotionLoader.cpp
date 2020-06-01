#include "stdafx.h"
#ifndef NO_OGRE

#include "renderer.h"
#include "FltkMotionWindow.h"
#include "OgreMotionLoader.h"

//#define USE_SKIN_ENTITY
#ifdef USE_SKIN_ENTITY
#include "../Ogre/OgreSkinEntity.h"
#include "../Ogre/OgreSkinSubEntity.h"

#endif
#include <OgreSubMesh.h>
#include "../BaseLib/math/dualQuaternion.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreBone.h>
#include <OgreMesh.h>
#include <OgreSkeleton.h>
#include <OgreSkeletonInstance.h>
#include <iostream>
using namespace std;

//////////////////////////////////////////////////////////////////////////
// Mesh Bone Implementation
//////////////////////////////////////////////////////////////////////////
MeshBone::MeshBone()
:Bone()
{
	mOgreBoneHandle=-1;
}

MeshBone::~MeshBone(){}

void MeshBone::Unpack(Ogre::Bone* pBone)
{
	SetNameId(pBone->getName().c_str());
	mOgreBoneHandle=pBone->getHandle();
	Ogre::Vector3 position = pBone->getPosition();
	//부모를 기준으로한 position 즉 offset 입력
	vector3 offset;
	offset.setValue(position.x, position.y,position.z);
	//matrix의 초기화

	//Ogre::Quaternion rot = pBone->getWorldOrientation();

	Ogre::Quaternion rot = pBone->getOrientation();
	quater tRotation(rot.w, rot.x, rot.y, rot.z);
	//m_matRot.setTransform(tRotation, offset);

	m_transfOrig.rotation=quater(1,0,0,0);
	m_transfOrig.translation=offset;

	//채널에 대한 정보는 Skinned Mesh가 가지고 있지 않다.
	if(numChannels()==0)
		setChannels("", "ZXY");	// rotation만 된다고 가정.


	int numChildren = pBone->numChildren();
	for(int i=0;i<numChildren;i++)
	{
		Ogre::Bone* pChildBone = (Ogre::Bone*)pBone->getChild(i);
		if(pChildBone)
		{
			MeshBone* pChildMeshBone = new MeshBone();
			AddChild(pChildMeshBone);
			pChildMeshBone->Unpack(pChildBone);
		}
	}

	mDerivedScale=ToBase(pBone->_getDerivedScale());
	ASSERT(isSimilar(mDerivedScale.distance(vector3(1,1,1)), 0));

	mBindDerivedInverseScale=ToBase(pBone->_getBindingPoseInverseScale());
	mBindDerivedInversePosition=ToBase(pBone->_getBindingPoseInversePosition());
	mBindDerivedInverseOrientation=ToBase(pBone->_getBindingPoseInverseOrientation());
}

void MeshBone::getOffsetTransform(matrix4& m) const
{
    // Combine scale with binding pose inverse scale,
    // NB just combine as equivalent axes, no shearing
	vector3 scale;
	scale= mDerivedScale.mult(mBindDerivedInverseScale);

    // Combine orientation with binding pose inverse orientation
    quater rotate = getRotation() * mBindDerivedInverseOrientation;

    // Combine position with binding pose inverse position,
    // Note that translation is relative to scale & rotation,
    // so first reverse transform original derived position to
    // binding pose bone space, and then transform to current
    // derived bone space.

	// _getDerivedPosition과 달리, getTranslation은 scale이 1,1,1일때만 동작한다. 아닌경우 이상하게 그려질듯.
    vector3 translate = getTranslation() + rotate * (scale.mult(mBindDerivedInversePosition));

    m.setTransform(translate, scale, rotate);
}



void skinnedMeshLoader_retrieveData(SkinnedMeshLoader::VertexBlendMesh& mesh, intvectorn const& ogreBoneToBone, int current_offset, Ogre::Mesh::IndexMap const& indexMap, Ogre::VertexData* sourceVertexData, bool);

#ifdef USE_SKIN_ENTITY
void MeshShape_countIndicesAndVertices(Ogre::SkinEntity * entity, size_t & index_count, size_t & vertex_count);
#else
void MeshShape_countIndicesAndVertices(Ogre::Entity * entity, size_t & index_count, size_t & vertex_count);
#endif

SkinnedMeshLoader::SkinnedMeshLoader(const char* ogreMeshFile, bool unpackTexCoord)
:MotionLoader()
{
#ifdef USE_SKIN_ENTITY
	Ogre::SkinEntity* pEntity=Ogre::createSkinEntity(*(RE::renderer().mScene), RE::generateUniqueName().ptr(), ogreMeshFile);
#else
	Ogre::Entity* pEntity=RE::ogreSceneManager()->createEntity(RE::generateUniqueName().ptr(), ogreMeshFile);
#endif

	if(!pEntity)
	{
		Msg::error("Entity 생성 실패");
	}
	if(!pEntity->hasSkeleton())
	{
		Msg::error("Skeleton Entity가 아님");
	}

	Ogre::SkeletonInstance* si = pEntity->getSkeleton();

	//Dump(pEntity);
	cout << endl;
	cout << "Skinned Skeleton Character Structure" << endl;
	cout << "==========================================================="<<endl;
	int num = si->getNumBones();
	Ogre::String name = si->getName();
	Ogre::Skeleton::BoneIterator iter = si->getBoneIterator();
	cout << "Number of Bones : " << num << endl;
	cout << "Name of the Skeletal Entity : " << name << endl;
	cout << "-----------------------------------------------------------"<<endl;
	cout << "Now See the Tree Structure" << endl;
	cout << "-----------------------------------------------------------"<<endl;
#if OGRE_VERSION_MINOR>=12
	Ogre::Bone* pRootBone = si->getRootBones()[0];
#else
	Ogre::Bone* pRootBone = si->getRootBone();
#endif
	DumpChildParser(pRootBone);
	cout << "-----------------------------------------------------------"<<endl;

	//생성자에서는 m_pTreeRoot에 pEntity의 구조를 저장해 주어야 한다.
	int numBone = si->getNumBones();

	//dummy root를 달아 놓는다. 나중에 모델을 자유롭게 이동회전시킨다.
	m_pTreeRoot = new MeshBone();
	m_pTreeRoot->SetNameId("DUMMY");

	// root
	m_pTreeRoot->AddChild(new MeshBone());
	((Bone*)m_pTreeRoot)->child()->setChannels("XYZ", "ZXY");

#if OGRE_VERSION_MINOR>=12
	Ogre::Bone* pBone = si->getRootBones()[0];
#else
	Ogre::Bone* pBone = si->getRootBone();
#endif
	((MeshBone*)(m_pTreeRoot->m_pChildHead))->Unpack(pBone);

	int num_channel;
	MakeBoneArrayFromTree(num_channel);
	//TargetIndex 멤버 셋팅

	ogreBoneToBone.setSize(si->getNumBones());
	ogreBoneToBone.setAllValue(-1);

	for(int i=0; i<GetNumTreeNode(); i++)
	{
		MeshBone &bone=(MeshBone&)getBoneByTreeIndex(i);
		if(bone.mOgreBoneHandle!=65535)
			ogreBoneToBone[bone.mOgreBoneHandle]=i;
	}


	// mesh정보 가져오기.
	int numSubMesh=pEntity->getMesh()->getNumSubMeshes();

	size_t index_count;
	size_t vertex_count;
	MeshShape_countIndicesAndVertices(pEntity, index_count, vertex_count);

	mMesh.mesh.resize(vertex_count, index_count/3);
	mMesh.blendIndex.setSize(vertex_count, 4);
	mMesh.blendWeight.setSize(vertex_count, 4);

    bool added_shared = false;
	size_t current_offset = 0;
	size_t shared_offset = 0;
	size_t next_offset = 0;
	size_t index_offset = 0;

	Ogre::Mesh* mesh=pEntity->getMesh().getPointer();

	// Run through the submeshes again, adding the data into the arrays
	for ( size_t i = 0; i < mesh->getNumSubMeshes(); ++i)
	{
		Ogre::SubMesh* submesh = mesh->getSubMesh(i);
		bool useSharedVertices = submesh->useSharedVertices;

		//----------------------------------------------------------------
		// GET VERTEXDATA
		//----------------------------------------------------------------
		Ogre::VertexData * vertex_data;
		vertex_data = useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

		if((!useSharedVertices)||(useSharedVertices && !added_shared))
		{
			if(useSharedVertices)
			{
				ASSERT(mesh->getSharedVertexDataAnimationType() == Ogre::VAT_NONE);
				added_shared = true;
				shared_offset = current_offset;
			}

			skinnedMeshLoader_retrieveData(mMesh, ogreBoneToBone, current_offset,
				useSharedVertices? mesh->sharedBlendIndexToBoneIndexMap:submesh->blendIndexToBoneIndexMap, vertex_data, unpackTexCoord);
            next_offset += vertex_data->vertexCount;
		}

        //----------------------------------------------------------------
        // GET INDEXDATA
        //----------------------------------------------------------------

		Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
		Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

		bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

		Ogre::uint32 *pLong = static_cast<Ogre::uint32*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        Ogre::uint16* pShort = reinterpret_cast<Ogre::uint16*>(pLong);


        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

        if ( use32bitindexes )
        {
			for ( size_t k = 0; k < numTris; ++k)
			{
			   mMesh.mesh.getFace(index_offset++).setIndex(
				   pLong[k*3] + static_cast<int>(offset),
				   pLong[k*3+1] + static_cast<int>(offset),
				   pLong[k*3+2] + static_cast<int>(offset));
			}
        }
        else
        {
			for ( size_t k = 0; k < numTris; ++k)
			{
			   mMesh.mesh.getFace(index_offset++).setIndex(
				   pShort[k*3] + static_cast<int>(offset),
				   pShort[k*3+1] + static_cast<int>(offset),
				   pShort[k*3+2] + static_cast<int>(offset));
			}
        }

        ibuf->unlock();
		current_offset = next_offset;
	}

	if(mMesh.option.useNormal)
		mMesh.mesh.copyIndex(OBJloader::Buffer::VERTEX, OBJloader::Buffer::NORMAL);
	if(mMesh.option.useTexCoord)
		mMesh.mesh.copyIndex(OBJloader::Buffer::VERTEX, OBJloader::Buffer::TEXCOORD);
	if(mMesh.option.useColor)
		mMesh.mesh.copyIndex(OBJloader::Buffer::VERTEX, OBJloader::Buffer::COLOR);

	RE::renderer().viewport().mScene->destroyMovableObject(pEntity);
}

void SkinnedMeshLoader::setCurPoseAsBindPose()
{
	OBJloader::Mesh curMesh;
	retrieveAnimatedMesh(curMesh);
	for(int i=1; i<numBone(); i++)
	{
		MeshBone& bone=	(MeshBone&) getBoneByTreeIndex(i);
		ASSERT(bone.mDerivedScale.distance(vector3(1,1,1))<0.001);
		ASSERT(bone.mBindDerivedInverseScale.distance(vector3(1,1,1))<0.001);

		// set bindDerivedInverseOrienation and position s.t. getOffsetTransform generate the identity transform.
		// getOffsetTransform 참고.
		bone.mBindDerivedInverseOrientation=bone.getRotation().inverse();
		bone.mBindDerivedInversePosition=bone.getTranslation()*-1;
	}
	// curMesh has only normal and vertex buffers.
	for(int i=0; i<curMesh.numVertex(); i++)
		mMesh.mesh.getVertex(i)=curMesh.getVertex(i);
	for(int i=0; i<curMesh.numNormal(); i++)
		mMesh.mesh.getNormal(i)=curMesh.getNormal(i);
}

void SkinnedMeshLoader::gotoBindPose()
{
	for(int i=1; i<numBone(); i++)
	{
		MeshBone& bone=(MeshBone&)getBoneByTreeIndex(i);
		// getOffsetTransform should return identity matrix.

		bone._getFrame().rotation=bone.mBindDerivedInverseOrientation.inverse();
		bone._getFrame().translation=(bone.mBindDerivedInversePosition*-1);
	}
}

#include "MotionPanel.h"

void testSkinnedMeshLoader()
{
	Motion* mot=new Motion(RE::motionLoader("iguana.skl"));
	RE::motion::concatFromFile(*mot, "trc/iguana_motion_set.mot");
	RE::motionPanel().motionWin()->addSkin(RE::createSkin(*mot));

	SkinnedMeshLoader* skel2=new SkinnedMeshLoader ("iguana_pskinned.mesh", true);
	skel2->mMesh.mesh.calculateVertexNormal();
	skel2->sortBones(mot->skeleton());
	skel2->gotoBindPose();
	Posture bindpose;
	skel2->getPose(bindpose);
	mot->pose(0)=bindpose;

	PLDPrimMesh* pSkin;
	pSkin=RE::createMesh(skel2, "lambert6");
	pSkin->ApplyAnim(*mot);
	RE::motionPanel().motionWin()->addSkin(pSkin);

	SkinnedMeshLoader* skel=new SkinnedMeshLoader("iguana_physics.mesh");
	skel->mMesh.mergeDuplicateVertices();
	skel->mMesh.mesh.calculateVertexNormal();
	skel->sortBones(mot->skeleton());
	skel->setPose(bindpose);
	skel->setCurPoseAsBindPose();

	pSkin=RE::createMesh(skel);
	pSkin->ApplyAnim(*mot);
	RE::motionPanel().motionWin()->addSkin(pSkin);

	pSkin->mMesh.saveObj("iguana.obj",true, false);

	/*
	// initial bone끼리 똑같은 모양일꺼다.
	mot->skeleton().UpdateInitialBone();
	skel->UpdateInitialBone();

	PoseTransfer pt(&mot->skeleton(), skel);
	pt.setTargetSkeleton(mot->pose(0));

	OBJloader::Mesh mesh=skel->mMesh.mesh;
	skel->retrieveAnimatedMesh(mesh);

	RE::ogreRootSceneNode()->attachObject(
		createMeshEntity(mesh, RE::generateUniqueName(),"white", skel->mMesh.option));

	RE::createSkin(*skel);*/
};



void SkinnedMeshLoader::DumpChildParser(Ogre::Node* pNode, int tab)
{
	Ogre::Vector3 pos = pNode->getPosition();
	Ogre::Matrix3 localAxis = pNode->getLocalAxes();
	Ogre::Quaternion rotation = pNode->getOrientation();
	Ogre::Radian ang;
	Ogre::Vector3 axis;
	//rotation.
	rotation.ToAngleAxis(ang, axis);

	for(int i=0;i<tab-1;i++) cout<<"    ";
	if(tab-1>=0) cout <<"+-->";
	cout << pNode->getName() << " P("<<pos.x<<","<<pos.y<<","<<pos.z<<") "
		<< "A(" <<axis.x<<","<<axis.y<<","<<axis.z<<") "<<
		ang.valueDegrees() << " degree" << endl;
	for(int i=0; i<pNode->numChildren(); i++)
	{
		DumpChildParser(pNode->getChild(i), tab+1);
	}
}
SkinnedMeshLoader::~SkinnedMeshLoader()
{
}

void SkinnedMeshLoader::sortBones(MotionLoader const& referenceSkeleton)
{
	std::vector<Bone*> bones;

	bones.resize(numBone());
	for(int i=0; i<numBone(); i++)
		bones[i]=&getBoneByTreeIndex(i);

	MotionLoader::sortBones(referenceSkeleton);

	// Update blend indexes.
	for(int i=0; i<mMesh.blendIndex.rows(); i++)
		for(int j=0; j<mMesh.blendIndex.cols(); j++)
			mMesh.blendIndex[i][j]=bones[mMesh.blendIndex[i][j]]->treeIndex();
}


void SkinnedMeshLoader::retrieveAnimatedMesh(OBJloader::Mesh& mesh)
{
	_cachedBoneMatrices.resize(GetNumTreeNode());
	for(int i=1; i<GetNumTreeNode(); i++)
	{
		((MeshBone&)getBoneByTreeIndex(i)).getOffsetTransform(_cachedBoneMatrices[i]);
	}

#define USE_DUAL_QUATERNION
#ifdef USE_DUAL_QUATERNION
	std::vector<dualQuaternion> _cachedDualQuaternions;
	_cachedDualQuaternions.resize(GetNumTreeNode());

	for(int i=1; i<GetNumTreeNode(); i++)
	{
		_cachedDualQuaternions[i]=_cachedBoneMatrices[i];
	}
	dualQuaternion dqTemp;
	dualQuaternion dqArray[4];
#endif
	// 아래 코드는 Ogre::SkinEntity::softwareVertexBlend를 고쳐서 작성하였음.
	size_t numVertices=mMesh.mesh.numVertex();

	ASSERT(mMesh.mesh.numVertex()==mMesh.mesh.numNormal());

	mesh.resizeBuffer(OBJloader::Buffer::VERTEX, numVertices);
	mesh.resizeBuffer(OBJloader::Buffer::NORMAL, numVertices);

    // Accumulation vectors
    vector3 accumVecPos, accumVecNorm;
	quater qtemp;

	unsigned short numWeightsPerVertex=mMesh.blendIndex.cols();
	ASSERT(numWeightsPerVertex<=4);

    // Loop per vertex
    for (size_t vertIdx = 0; vertIdx < numVertices; ++vertIdx)
    {
        // Load accumulators
        accumVecPos.setValue(0,0,0);
        accumVecNorm.setValue(0,0,0);

		m_real * pBlendWeight=mMesh.blendWeight[vertIdx];
		int * pBlendIndex=mMesh.blendIndex[vertIdx];

		vector3& sourceVec=mMesh.mesh.getVertex(vertIdx);

		vector3* sourceNor=(mMesh.option.useNormal)?&mMesh.mesh.getNormal(vertIdx):NULL;

#ifdef USE_DUAL_QUATERNION

		for (unsigned short blendIdx = 0; blendIdx < numWeightsPerVertex; ++blendIdx)
		{
			dqArray[blendIdx]=_cachedDualQuaternions[pBlendIndex[blendIdx]];

		}

		dqTemp=dualQuaternion::sDLB(numWeightsPerVertex, pBlendWeight, dqArray);

		accumVecPos=dqTemp.transform(sourceVec);
		accumVecNorm.rotate(dqTemp.mReal, *sourceNor);
#else
		for (unsigned short blendIdx = 0; blendIdx < numWeightsPerVertex; ++blendIdx)
        {
			// Blend position, use 3x4 matrix
			const matrix4& mat = _cachedBoneMatrices[pBlendIndex[blendIdx]];
			qtemp.setRotation(mat);

			// Blend by multiplying source by blend matrix and scaling by weight
            // Add to accumulator
            // NB weights must be normalised!!
            m_real weight = pBlendWeight[blendIdx];
            if (weight)
            {
				accumVecPos+= mat*sourceVec*weight;

                if (sourceNor)
					accumVecNorm+= qtemp*(*sourceNor)*weight;
            }
		}
#endif
		mesh.getVertex(vertIdx)=accumVecPos;

		// Stored blended vertex in temp buffer
        if (sourceNor)
        {
			accumVecNorm.normalize();
			mesh.getNormal(vertIdx)=accumVecNorm;
        }
    }
}

bool isSimilar(quater const& a, quater const& b)
{
	return isSimilar(a.x, b.x) &&
		isSimilar(a.y, b.y) &&
		isSimilar(a.z, b.z) &&
		isSimilar(a.w, b.w);
}



class UnmergeInfo
{
public:
	UnmergeInfo(){}
	OBJloader::Mesh backupMesh;
	intvectorn newVertexIndex;
};
void SkinnedMeshLoader::VertexBlendMesh::unmergeDuplicateVertices(UnmergeInfo* uifo)
{
	matrixn newblendWeight;
	intmatrixn newblendIndex;

	int vertex_count=uifo->backupMesh.numVertex();
	newblendIndex.resize(vertex_count, blendIndex.cols());
	newblendWeight.resize(vertex_count, blendWeight.cols());

	for(int i=0; i<vertex_count; i++)
	{
		uifo->backupMesh.getVertex(i)=mesh.getVertex(uifo->newVertexIndex[i]);
		uifo->backupMesh.getNormal(i)=mesh.getNormal(uifo->newVertexIndex[i]);

		newblendWeight.row(i)=blendWeight.row(uifo->newVertexIndex[i]);
		newblendIndex.row(i)=blendIndex.row(uifo->newVertexIndex[i]);
	}


	blendWeight=newblendWeight;
	blendIndex=newblendIndex;
	mesh=uifo->backupMesh;
}

void SkinnedMeshLoader::VertexBlendMesh::reorderVertices()
{
	intvectorn newIndex;
	newIndex.setSize(mesh.numVertex());

	bitvectorn usedVertex;
	usedVertex.resize(mesh.numVertex());
	usedVertex.clearAll();

	int count=0;
	for(int i=0; i<mesh.numFace(); i++)
	{
		for(int j=0; j<3; j++)
		{
			int v=mesh.getFace(i).vi(j);
			if(!usedVertex(v))
			{
				newIndex[v]=count++;
				usedVertex.setAt(v);
			}
		}
	}

	matrixn backupblendWeight=blendWeight;
	intmatrixn backupBlendIndex=blendIndex;
	OBJloader::Mesh backupmesh=mesh;

	for(int i=0; i<mesh.numVertex(); i++)
	{
		mesh.getVertex(newIndex[i])=backupmesh.getVertex(i);
		blendWeight.row(newIndex[i])=backupblendWeight.row(i);
		blendIndex.row(newIndex[i])=backupBlendIndex.row(i);
	}

	for(int i=0; i<mesh.numFace(); i++)
	{
		mesh.getFace(i).setIndex(
			newIndex[backupmesh.getFace(i).vi(0)],
			newIndex[backupmesh.getFace(i).vi(1)],
			newIndex[backupmesh.getFace(i).vi(2)]);
	}
}

UnmergeInfo* SkinnedMeshLoader::VertexBlendMesh::mergeDuplicateVertices(bool bReturnMergeInfo, double distThr)
{
	UnmergeInfo* uifo=new UnmergeInfo();
	uifo->backupMesh=mesh;
	matrixn backupBlendWeight=blendWeight;
	intmatrixn backupBlendIndex=blendIndex;

	OBJloader::EdgeConnectivity ec(mesh);
	bitvectorn isBoundaryVertex;
	isBoundaryVertex.resize(mesh.numVertex());
	isBoundaryVertex.clearAll();

	for(int i=0; i<ec.numEdges(); i++)
	{
		if(ec.getEdge(i).numFaceEdge<2)
		{
			isBoundaryVertex.setAt(ec.source(i));
			isBoundaryVertex.setAt(ec.target(i));
		}
	}

	intvectorn boundaryVertexIndex;
	boundaryVertexIndex.findIndex(isBoundaryVertex,true);

	intvectorn vertexMerge;
	uifo->newVertexIndex;
	size_t vertex_count=uifo->backupMesh.numVertex();

	vertexMerge.colon(0, vertex_count);

	double MaxDist=0;
	double dist;

	// for each pair of boundary vertices, test proximity.
	for(int ii=0; ii<boundaryVertexIndex.size(); ii++)
	{
		int i=boundaryVertexIndex[ii];

		for(int jj=ii+1; jj<boundaryVertexIndex.size(); jj++)
		{
			int j=boundaryVertexIndex[jj];
			dist=uifo->backupMesh.getVertex(i).distance(uifo->backupMesh.getVertex(j));
			if(dist>MaxDist) MaxDist=dist;
			if(dist<=distThr)
			{
				vertexMerge[j]=vertexMerge[i];
				printf("merging vertices v%d and v%d\n", i,j);
			}
		}
	}

	printf("max Dist=%f\n", MaxDist);
	uifo->newVertexIndex.setSize(vertex_count);
	int c=0;
	for(int i=0; i<vertex_count; i++)
	{
		if(vertexMerge[i]==i)
		{
			uifo->newVertexIndex[i]=c;
			c++;
		}
		else
		{
			uifo->newVertexIndex[i]=uifo->newVertexIndex[vertexMerge[i]];
		}
	}

	ASSERT(mesh.numVertex()==mesh.numNormal());

	mesh.resizeBuffer(OBJloader::Buffer::VERTEX, c);
	if(option.useNormal)
		mesh.resizeBuffer(OBJloader::Buffer::NORMAL, c);

	blendWeight.setSize(c, backupBlendWeight.cols());
	blendIndex.setSize(c, backupBlendIndex.cols());

	intvectorn count(c);
	count.setAllValue(0);
	for(int i=0; i<vertex_count; i++)
	{
		mesh.getVertex(uifo->newVertexIndex[i])=uifo->backupMesh.getVertex(i);

		// 당연히 texcoord는 merge하지 않는다.

		if(option.useNormal)
		{
			if(count[uifo->newVertexIndex[i]]==0)
				mesh.getNormal(uifo->newVertexIndex[i]).setValue(0,0,0);

			mesh.getNormal(uifo->newVertexIndex[i])+=uifo->backupMesh.getNormal(i);
		}
		count[uifo->newVertexIndex[i]]+=1;

		blendWeight.row(uifo->newVertexIndex[i])=backupBlendWeight.row(i);
		blendIndex.row(uifo->newVertexIndex[i])=backupBlendIndex.row(i);
	}

	for(int i=0; i<c; i++)
	{
		mesh.getNormal(i)/=(double)count[i];
		mesh.getNormal(i).normalize();
	}


	int numTris=uifo->backupMesh.numFace();
	mesh.resizeIndexBuffer(numTris);
	for(int i=0; i<numTris; i++)
	{
		mesh.getFace(i).vertexIndex(0)=uifo->newVertexIndex[uifo->backupMesh.getFace(i).vertexIndex(0)];
		mesh.getFace(i).vertexIndex(1)=uifo->newVertexIndex[uifo->backupMesh.getFace(i).vertexIndex(1)];
		mesh.getFace(i).vertexIndex(2)=uifo->newVertexIndex[uifo->backupMesh.getFace(i).vertexIndex(2)];
	}

	mesh.copyIndex(OBJloader::Buffer::VERTEX, OBJloader::Buffer::NORMAL);

	if(bReturnMergeInfo)
		return uifo;
	delete uifo;
	return NULL;
}




///////////////////////
using namespace Ogre;
//------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
/// Counts how many indices (faces) and vertices an entity contains.
/// @param[in]  entity Entity to count its data.
/// @param[out] index_count  Number of indices.
/// @param[out] vertex_count Number of vertices.
/// @author Yavin from the Ogre4J team
///////////////////////////////////////////////////////////////////////////////
#ifdef USE_SKIN_ENTITY
void MeshShape_countIndicesAndVertices(SkinEntity * entity, size_t & index_count, size_t & vertex_count)
#else
void MeshShape_countIndicesAndVertices(Entity * entity, size_t & index_count, size_t & vertex_count)
#endif
{
   Ogre::Mesh * mesh = entity->getMesh().getPointer();

  bool added_shared = false;
  index_count  = 0;
  vertex_count = 0;

  // Calculate how many vertices and indices we're going to need
  for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
     SubMesh* submesh = mesh->getSubMesh( i );

     // We only need to add the shared vertices once
     if(submesh->useSharedVertices)
     {
        if( !added_shared )
        {
           vertex_count += mesh->sharedVertexData->vertexCount;
           added_shared = true;
        }
     }
     else
     {
        vertex_count += submesh->vertexData->vertexCount;
     }

     // Add the indices
     index_count += submesh->indexData->indexCount;
  }
}

#include <OgreOptimisedUtil.h>

void skinnedMeshLoader_retrieveData(SkinnedMeshLoader::VertexBlendMesh& mesh, intvectorn const& ogreBoneToBone, int current_offset, Ogre::Mesh::IndexMap const& indexMap, VertexData* sourceVertexData, bool unpackTexCoord)
{
	// 아래 코드는 Ogre::SkinEntity::softwareVertexBlend를 고쳐서 작성하였음.

	assert(indexMap.size() <= 256);
	bool blendNormals=true;

    // Get elements for source
    const VertexElement* srcElemPos = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_POSITION);
    const VertexElement* srcElemNorm = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_NORMAL);
	const VertexElement* srcElemTex = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_TEXTURE_COORDINATES);
    const VertexElement* srcElemBlendIndices = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_BLEND_INDICES);
    const VertexElement* srcElemBlendWeights = sourceVertexData->vertexDeclaration->findElementBySemantic(VES_BLEND_WEIGHTS);
    assert (srcElemPos && srcElemBlendIndices && srcElemBlendWeights && "You must supply at least positions, blend indices and blend weights");


    // Do we have normals and want to blend them?
    bool includeNormals = blendNormals && (srcElemNorm != NULL);
	bool includeTexCoords = unpackTexCoord && (srcElemTex!=NULL);

	if(includeNormals)
		mesh.mesh.resizeBuffer(OBJloader::Buffer::NORMAL, mesh.mesh.numVertex());

	if(includeTexCoords)
		mesh.mesh.resizeBuffer(OBJloader::Buffer::TEXCOORD, mesh.mesh.numVertex());

	mesh.option.useColor=false;
	mesh.option.useTexCoord=includeTexCoords ;

	mesh.option.useNormal=includeNormals;

    // Get buffers for source
    HardwareVertexBufferSharedPtr srcPosBuf, srcNormBuf, srcTexCoordBuf, srcIdxBuf, srcWeightBuf;
    size_t srcPosStride = 0;
    size_t srcNormStride = 0;
	size_t srcTexCoordStride = 0;
    size_t blendWeightStride = 0;
    size_t blendIdxStride = 0;

	srcPosBuf = sourceVertexData->vertexBufferBinding->getBuffer(srcElemPos->getSource());
    srcPosStride = srcPosBuf->getVertexSize();


    srcIdxBuf = sourceVertexData->vertexBufferBinding->getBuffer(srcElemBlendIndices->getSource());
    blendIdxStride = srcIdxBuf->getVertexSize();
    srcWeightBuf = sourceVertexData->vertexBufferBinding->getBuffer(srcElemBlendWeights->getSource());
    blendWeightStride = srcWeightBuf->getVertexSize();
    if (includeNormals)
    {
        srcNormBuf = sourceVertexData->vertexBufferBinding->getBuffer(srcElemNorm->getSource());
        srcNormStride = srcNormBuf->getVertexSize();
    }
    if(includeTexCoords)
	{
		srcTexCoordBuf= sourceVertexData->vertexBufferBinding->getBuffer(srcElemTex->getSource());
		srcTexCoordStride=srcTexCoordBuf->getVertexSize();
	}
    void* pBuffer;

	// Lock source buffers for reading
	float *pSrcPos = 0;
	float *pSrcNorm = 0;
	float* pSrcTexCoord=0;
	float *pBlendWeight = 0;
    unsigned char* pBlendIdx = 0;

    pBuffer = srcPosBuf->lock(HardwareBuffer::HBL_READ_ONLY);
    srcElemPos->baseVertexPointerToElement(pBuffer, &pSrcPos);

	if(includeTexCoords)
	{
		if (srcTexCoordBuf !=srcPosBuf) pBuffer = srcTexCoordBuf ->lock(HardwareBuffer::HBL_READ_ONLY);
		srcElemTex->baseVertexPointerToElement(pBuffer, &pSrcTexCoord);
	}
    if (includeNormals)
    {
        if (srcNormBuf != srcPosBuf && srcNormBuf !=srcTexCoordBuf) pBuffer = srcNormBuf->lock(HardwareBuffer::HBL_READ_ONLY);
        srcElemNorm->baseVertexPointerToElement(pBuffer, &pSrcNorm);
    }

    // Indices must be 4 bytes
    assert(srcElemBlendIndices->getType() == VET_UBYTE4 &&
           "Blend indices must be VET_UBYTE4");
    pBuffer = srcIdxBuf->lock(HardwareBuffer::HBL_READ_ONLY);
    srcElemBlendIndices->baseVertexPointerToElement(pBuffer, &pBlendIdx);
    if (srcWeightBuf != srcIdxBuf) pBuffer = srcWeightBuf->lock(HardwareBuffer::HBL_READ_ONLY);
    srcElemBlendWeights->baseVertexPointerToElement(pBuffer, &pBlendWeight);
    unsigned short numWeightsPerVertex =
        VertexElement::getTypeCount(srcElemBlendWeights->getType());
	ASSERT(numWeightsPerVertex ==4);

	int numVertices=sourceVertexData->vertexCount;

	for(size_t vertIdx=0; vertIdx<numVertices; vertIdx++)
	{
		mesh.mesh.getVertex(vertIdx+current_offset).x=pSrcPos[0];
		mesh.mesh.getVertex(vertIdx+current_offset).y=pSrcPos[1];
		mesh.mesh.getVertex(vertIdx+current_offset).z=pSrcPos[2];

		for(unsigned short blendIdx=0; blendIdx<numWeightsPerVertex; ++blendIdx)
		{
			mesh.blendWeight(vertIdx+current_offset,blendIdx)=pBlendWeight[blendIdx];
			mesh.blendIndex(vertIdx+current_offset,blendIdx)=ogreBoneToBone[indexMap[pBlendIdx[blendIdx]]];
		}

		if(pSrcTexCoord)
		{
			mesh.mesh.getTexCoord(vertIdx+current_offset)(0)=pSrcTexCoord[0];
			mesh.mesh.getTexCoord(vertIdx+current_offset)(1)=pSrcTexCoord[1];
			advanceRawPointer(pSrcTexCoord, srcTexCoordStride);
		}
		if(pSrcNorm)
		{
			mesh.mesh.getNormal(vertIdx+current_offset).x=pSrcNorm[0];
			mesh.mesh.getNormal(vertIdx+current_offset).y=pSrcNorm[1];
			mesh.mesh.getNormal(vertIdx+current_offset).z=pSrcNorm[2];
			advanceRawPointer(pSrcNorm, srcNormStride);
		}

		advanceRawPointer(pSrcPos, srcPosStride);
		advanceRawPointer(pBlendWeight, blendWeightStride);
		advanceRawPointer(pBlendIdx, blendIdxStride);
	}

    // Unlock source buffers
    srcPosBuf->unlock();
    srcIdxBuf->unlock();
    if (srcWeightBuf != srcIdxBuf) srcWeightBuf->unlock();
	if(includeTexCoords
		&& srcTexCoordBuf!=srcPosBuf) srcTexCoordBuf->unlock();
    if (includeNormals
		&& srcNormBuf != srcPosBuf
		&& srcNormBuf !=srcTexCoordBuf) srcNormBuf->unlock();
}





PLDPrimMesh ::PLDPrimMesh (SkinnedMeshLoader *pTgtSkel, const char* materialName)
:PLDPrimSkin(),
mSkeleton(*pTgtSkel)
{
	mMesh=pTgtSkel->mMesh.mesh;
	//pTgtSkel->mMesh.option.buildEdgeList=true;
	mMeshToEntity=new OBJloader::MeshToEntity(mMesh, RE::generateUniqueName(), pTgtSkel->mMesh.option);

	m_pSceneNode=RE::ogreRootSceneNode()->createChildSceneNode();
	m_pSceneNode->attachObject(mMeshToEntity->createEntity(RE::generateUniqueName(), materialName));
}

PLDPrimMesh ::~PLDPrimMesh ()
{
	delete mMeshToEntity;
}
void PLDPrimMesh::SetPose(const Posture & posture, const MotionLoader& skeleton)
{
	mSkeleton.setPose(posture);
	_updateMesh();
}
void PLDPrimMesh::_updateMesh()
{
	mSkeleton.retrieveAnimatedMesh(mMesh);
	mMeshToEntity->updatePositionsAndNormals();
	m_pSceneNode->_updateBounds();
}

class PLDPrimTransferredSkin: public PLDPrimMesh
{
public:
	PLDPrimTransferredSkin(MotionLoader* pSrcSkel, SkinnedMeshLoader *pTgtSkel, const char* convfilename, bool bCurrPoseAsBindPose)
		:PLDPrimMesh(pTgtSkel, "white"),
		mPoseTransfer(pSrcSkel, pTgtSkel, convfilename, bCurrPoseAsBindPose)
	{

	}

	virtual ~PLDPrimTransferredSkin(){}
	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton)
	{
		mPoseTransfer.setTargetSkeleton(posture);
		_updateMesh();
	}
	PoseTransfer mPoseTransfer;
};



PLDPrimMesh* RE::createMesh(SkinnedMeshLoader *pTgtSkel, const char*materialName)
{
	PLDPrimMesh* pSkin;
	pSkin=new PLDPrimMesh(pTgtSkel, materialName);
	RE::renderer().addFrameMoveObject(pSkin);
	return pSkin;
}

PLDPrimMesh* RE::createMesh(const Motion& mot, SkinnedMeshLoader *pTgtSkel, const char* convfilename, bool bCurrPoseAsBindPose)
{
	PLDPrimMesh* pSkin;
	pSkin=new PLDPrimTransferredSkin(&mot.skeleton(), pTgtSkel, convfilename, bCurrPoseAsBindPose);
	RE::renderer().addFrameMoveObject(pSkin);
	pSkin->ApplyAnim(mot);
	pSkin->m_pTimer->StartAnim();
	return pSkin;
}

#endif
