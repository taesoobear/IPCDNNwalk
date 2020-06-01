#ifndef NO_OGRE
#pragma once
namespace Ogre
{
	class SkinEntity;
}
class PLDPrimCustumSkin : public PLDPrimSkin
{
public:
	/*!
	joint의 targetindex를 conversion table을 사용해 세팅한다.
	\param convfilename BVHTransform노드의 이름을 CombinedTransform의 이름과 매치시키는 conversion table파일이름 
	ApplyAnimToSkin하는 순간에 entity의 skeleton 이 update되어 있고, 자세가 bvh의 initial 또는 current 자세와 유사하다고 가정한다. (see bCurrPoseAsBindPos)
	(일반적으로는 모든 팔다리가 수직으로 내려와 있는 자세)
	참고: skel->_updateTransforms()
	*/

	PLDPrimCustumSkin(MotionLoader* pMotionLoader, Ogre::SkinEntity* pEntity, const char* convfilename, const OgreRenderer& renderer, bool bCurrPoseAsBindPose=false);

	PLDPrimCustumSkin(const PLDPrimCustumSkin& other, const OgreRenderer& renderer, const char* nameid);
	virtual ~PLDPrimCustumSkin();
	virtual void ApplyAnim(const Motion& mot);

	virtual void SetTranslation(float x, float y, float z);
	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton);

	Ogre::SkinEntity* getSkinEntity() const		{return m_pEntity;}
protected:
	
	void SetPose(const Posture& posture, int* aJointToTreeIndex, MotionLoader* pSkeleton);
	// Ogre사용해서 그리는 경우 (Skinning)
	Ogre::SkinEntity* m_pEntity;
	TArray<quater> m_aLocalRotOrig;
	TArray<quater> m_aRotOrigComb;
	TArray<quater> m_aInvRotOrigComb;
	TArray<quater> m_aBindPose;
	intvectorn parentIdx;
	intvectorn m_aTargetIndex;
};
namespace MeshLoader
{
	class Mesh;
}

namespace RE
{
PLDPrimSkin* createCustumSkin(const Motion& mot);
}
/*
// retrieve the results of vertex weighted skinning . mesh.firstInit() is not called inside the function!
class GetMeshAnimation_OLD
{
	PLDPrimCustumSkin* mSkin;
	void* m_data;
	m_real mScaleFactor;
	bool mSkinOwner;
public:
	GetMeshAnimation_OLD(m_real scaleFactor=1.0):mSkin(NULL),m_data(NULL), mScaleFactor(scaleFactor), mSkinOwner(true){}
	~GetMeshAnimation_OLD();
	void setScaleFactor(m_real scaleF)	{ mScaleFactor=scaleF;}
	m_real getScaleFactor() const		{ return mScaleFactor;}
	void create(Motion const& mot, MeshLoader::Mesh& mesh, int iframe);
	void create(PLDPrimCustumSkin* pSkin, MeshLoader::Mesh& mesh, int iframe);
	void update(MeshLoader::Mesh& mesh, int iframe);
	void update(MeshLoader::Mesh& mesh, Posture const& pose);
	void update(vector3N& vertexPos, int iframe);
	static void* _create(PLDPrimCustumSkin* pSkin, MeshLoader::Mesh& mesh, int iframe, bool cleanup=true);
};*/

#endif
