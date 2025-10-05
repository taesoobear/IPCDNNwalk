// OgreMotionLoader.h: interface for the Ogre Skinned Mesh Character Skeleton
//
//////////////////////////////////////////////////////////////////////
#pragma once

//#include "nodestack.h"
#include "../../BaseLib/motion/MotionLoader.h"
#include "Mesh.h"
#include "pldprimskin.h"

#ifndef NO_OGRE
/// OgreMesh파일의 skeleton구조의 한 본에 해당한다. 
class MeshBone : public Bone
{
public:
	MeshBone();
	virtual ~MeshBone();
	unsigned short mOgreBoneHandle;	
	void Unpack(Ogre::Bone* pBone);
	void getOffsetTransform(matrix4& m) const;
	void printBindingInfo() const;
	virtual void printHierarchy(int depth=0);

	vector3 mDerivedScale;
	vector3 mBindDerivedInverseScale;
	vector3 mBindDerivedInversePosition;
	quater mBindDerivedInverseOrientation;
};

class UnmergeInfo;

// Skeleton 정보를 읽고 tree hierarchy를 만든다. 
// 또한 skinning에 필요한 모든 정보를 읽어온다.
class SkinnedMeshLoader : public MotionLoader
{
	bool _useDQinterpolation;
public:
	SkinnedMeshLoader(const char* ogreMeshFile, bool unpackTexCoord=false, bool useDQinterpolation=true);
	//SkinnedMeshLoader(const char *filename);
	virtual ~SkinnedMeshLoader();
	
	void getInfo(SkinnedMeshFromVertexInfo& vi) const;
	class VertexBlendMesh
	{
	public:
		// src mesh. 원본 데이타이므로 건드리지 말 것.
		OBJloader::Mesh mesh;
		OBJloader::MeshToEntity::Option option;

		// size: numVertices by numWeightsPerVertex
		matrixn blendWeight;
		intmatrixn blendIndex;

		// 여러 vertex가 위치가 같은경우 한군데로 merge한다. (normal이나 texCoord가 다르면 위치가 같은 vertex가 duplicate되어 있는 경우가 있고, 에디팅이 어려워진다.)
		UnmergeInfo* mergeDuplicateVertices(bool bReturnMergeInfo=false,double distThr=0.000001);
		void unmergeDuplicateVertices(UnmergeInfo*);
		void reorderVertices();
	};
	inline void reorderVertices()
	{
		mMesh.mergeDuplicateVertices();
		mMesh.mesh.calculateVertexNormal();
		mMesh.reorderVertices();	// to match iguana_bound.obj
	}
	inline void loadMesh(const char* fn) // 인덱싱만 바꾼 메시 읽기.
	{
		mMesh.mesh.loadMesh(fn,false);	// 내가 인덱싱만 바꾼 메쉬를 읽는다. -  volume mesh 와 호환되도록 바꾸었다.
		mMesh.mesh.resizeBuffer(OBJloader::Buffer::NORMAL, mMesh.mesh.numVertex());
		mMesh.mesh.copyIndex(OBJloader::Buffer::VERTEX, OBJloader::Buffer::NORMAL);
	}
	OBJloader::Mesh& getCurrMesh() {return mMesh.mesh;}

	void gotoBindPose();
	void setCurPoseAsBindPose();

	virtual void sortBones(MotionLoader const& referenceSkeleton);

	VertexBlendMesh mMesh;

	// setSkeleton시 업데이트된 position과 normal을 가져온다. index는 가져오지 않는다!
	void retrieveAnimatedMesh(OBJloader::Mesh& mesh);
	void getVertexInfo(int vertexIndex, intvectorn& treeIndices, vector3N& localpos, vectorn &weights);
	double getDerivedScale(int treeIndex);
	double getBindingPoseInverseScale(int treeIndex);

	intvectorn ogreBoneToBone;

private:
	std::vector<matrix4> _cachedBoneMatrices;
	void DumpChildParser(Ogre::Node* , int tab=0);
};



class PLDPrimMesh : public PLDPrimSkin
{
public:
	PLDPrimMesh (SkinnedMeshLoader *pTgtSkel, const char* materialName);
	virtual ~PLDPrimMesh ();
	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton);
	
	// automatically called inside SetPose.
	void _updateMesh();	
	SkinnedMeshLoader& mSkeleton;
	OBJloader::Mesh mMesh;
	OBJloader::MeshToEntity* mMeshToEntity;
};




namespace RE
{
	PLDPrimMesh* createMesh(SkinnedMeshLoader *pTgtSkel, const char* materialName="white");
	PLDPrimMesh* createMesh(const Motion& mot, SkinnedMeshLoader *pTgtSkel, const char* convfilename=NULL, bool bCurrPoseAsBindPose=false);
}

#endif
