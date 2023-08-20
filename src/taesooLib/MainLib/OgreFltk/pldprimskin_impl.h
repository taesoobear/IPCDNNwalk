#ifndef PLDPRIMSKIN_IMPL_H_
#define PLDPRIMSKIN_IMPL_H_
#if defined (_MSC_VER) && (_MSC_VER >= 1000)
#pragma once
#endif

#include "../../BaseLib/math/mathclass.h"
class MotionLoader;
class Interpolator;
class TimeSensor;
class Posture;
class OgreRenderer;
class MotionManager;
class Motion;
#ifndef NO_OGRE
class SurfelClouds;
class LineStrip;
class Circle;
#endif
class PLDPrimSkin_impl : public PLDPrimSkin
{
	public:
	PLDPrimSkin_impl();
	virtual ~PLDPrimSkin_impl();
	virtual void setDrawConstraint(int con, float radius, RE::Color c);
	virtual void setDrawOrientation(int ijoint);
	virtual void setScale(double x, double y, double z);
	protected:
	void _drawConstraints(const Posture& posture, const MotionLoader& skeleton);
	void __drawConstraints(const MotionLoader& skeleton);
#ifndef NO_OGRE
	struct PLDPrimSkinCircle
	{
		Circle* pCircle;
		float radius;
		RE::Color c;
	};

	struct PLDPrimSkinAxes
	{
		Ogre::SceneNode* sceneNode;
		int ijoint;
	};

	double mScale;
	std::vector<PLDPrimSkinCircle*> mCircles;
	std::vector<PLDPrimSkinAxes> mAxes;
#endif
};

class PLDPrimSkel : public PLDPrimSkin_impl
{
public:
	PLDPrimSkel (MotionLoader* pBVHL);
	virtual ~PLDPrimSkel();

	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton);
	virtual void getPose(Posture & posture);

	virtual void updateBoneLength(MotionLoader const& loader){ mChain->updateBoneLength(loader); }

protected:

	int ToInitialPose();	//!< Model에 저장되어 있는 초기 상태로 돌린다. 에니메이션의 첫프레임과 다르다. m_pTimer를 delete해야 첫프레임으로 돌아가지 않는다.
	virtual int UpdateBone();

	MotionLoader* mSkel;
};
class PLDPrimBone : public PLDPrimSkel
{
public:
	PLDPrimBone(MotionLoader* pBVHL, const OgreRenderer& renderer);
	virtual ~PLDPrimBone();


	static void createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode) ;
	static void setLimb(Ogre::SceneNode *pNode, vector3 const& p0, vector3 const& p1, float thick=7) ;
	void setMaterial(const char* mat) override;
protected:
	virtual int UpdateBone();
	std::vector<Ogre::SceneNode*> mSceneNodes;
};

class PLDPrimPoint : public PLDPrimSkel
{
public:
	PLDPrimPoint(MotionLoader* pBVHL, const OgreRenderer& renderer);
	virtual ~PLDPrimPoint();


	static void createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode) ;
	static void setLimb(Ogre::SceneNode *pNode, vector3 const& p0, vector3 const& p1, float thick=7) ;
	void setMaterial(const char* mat) override;

	virtual void setScale(double x, double y, double z) override;
protected:
	virtual int UpdateBone() override;
	std::vector<Ogre::SceneNode*> mSceneNodes;
	std::vector <double> _dist;
};

class PLDPrimBox : public PLDPrimSkel
{
public:
	PLDPrimBox(MotionLoader* pBVHL, const OgreRenderer& renderer);
	virtual ~PLDPrimBox();


	void createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode, const char* nameId) ;
	void setMaterial(const char* mat) override;
	std::vector<Ogre::SceneNode*> mSceneNodes;
protected:
	virtual int UpdateBone() override;

};


class PLDPrimCyl : public PLDPrimSkel
{
public:
	PLDPrimCyl(MotionLoader* pBVHL, const OgreRenderer& renderer, bool lowPoly=false);
	virtual ~PLDPrimCyl();


	static void createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode) ;
	static void setLimb(Ogre::SceneNode *pNode, vector3 const& p0, vector3 const& p1, float thick=7) ;
	void setMaterial(const char* mat) override;
	virtual void setThickness(float thick);
protected:
	m_real thick;
	virtual int UpdateBone() override;
	std::vector<Ogre::SceneNode*> mSceneNodes;
	std::vector<Ogre::SceneNode*> mSceneNodesPoint;
};

class PLDPrimLine : public PLDPrimSkel
{
public:
	PLDPrimLine(MotionLoader* pBVHL, const OgreRenderer& renderer);
	virtual ~PLDPrimLine();

	virtual void SetVisible(bool bVisible);
	void setColor(RE::Color c);
protected:
	virtual int UpdateBone() override;
	#ifndef NO_OGRE
	std::vector<LineStrip*> mLines;
#endif
};

class PLDPrimThickLine : public PLDPrimSkel
{
public:
	PLDPrimThickLine(MotionLoader* pBVHL, const OgreRenderer& renderer);
	virtual ~PLDPrimThickLine();

	virtual void SetVisible(bool bVisible);
	virtual void setMaterial(const char* mat) override;
	virtual void setScale(double x, double y, double z);
	virtual int FrameMove(float fElapsedTime);
protected:
	virtual int UpdateBone() override;
	std::string _materialName;
#ifndef NO_OGRE
	double _thickness;
	BillboardLineList * _lines;
	Ogre::SceneNode* mSceneNode;
#endif
};

class PLDPrimOgreSkin : public PLDPrimSkin_impl
{
public:
	/*!
	joint의 targetindex를 conversion table을 사용해 세팅한다.
	\param convfilename BVHTransform노드의 이름을 CombinedTransform의 이름과 매치시키는 conversion table파일이름
	ApplyAnimToSkin하는 순간에 entity의 skeleton 이 update되어 있고, 자세가 bvh의 initial 또는 current 자세와 유사하다고 가정한다. (see bCurrPoseAsBindPos)
	(일반적으로는 모든 팔다리가 수직으로 내려와 있는 자세)
	참고: skel->_updateTransforms()
	*/

	PLDPrimOgreSkin(MotionLoader* pMotionLoader, Ogre::Entity* pEntity, const char* convfilename, const OgreRenderer& renderer, bool bCurrPoseAsBindPose=false, double motion_scale=1);

	PLDPrimOgreSkin(const PLDPrimOgreSkin& other, const OgreRenderer& renderer, const char* nameid);
	virtual ~PLDPrimOgreSkin();
	virtual void ApplyAnim(const Motion& mot);

	virtual void SetTranslation(float x, float y, float z);
	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton);
	int getOgreBoneIndex(int ibone) const;
	int parentRotJointIndex(int iRotJoint) const;

	quaterN m_aLocalRotOrig;
	quaterN m_aRotOrigComb;
	quaterN m_aInvRotOrigComb;
	quaterN m_aBindPose;
protected:

	void SetPose(const Posture& posture, int* aJointToTreeIndex, MotionLoader* pSkeleton);
	// Ogre사용해서 그리는 경우 (Skinning)
	Ogre::Entity* m_pEntity;
	intvectorn parentIdx;
	intvectorn m_aTargetIndex; // ByRotJointIndex
	intvectorn m_aTargetIndexByTransJointIndex;
	double mMotionScale;
	bool m_bMapPositionsToo; // works only when m_aBindPose, m_aLocalRotOrig are all identity.
};

// matchs both bone orientations and translations
// this class does not support conversion tables.
class PLDPrimPreciseOgreSkin : public PLDPrimSkin_impl
{
public:

	PLDPrimPreciseOgreSkin(MotionLoader* pMotionLoader, Ogre::Entity* pEntity, const OgreRenderer& renderer, double motion_scale=1);
	virtual ~PLDPrimPreciseOgreSkin();
	virtual void ApplyAnim(const Motion& mot);

	virtual void SetTranslation(float x, float y, float z);
	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton);
	int getOgreBoneIndex(int ibone) const;

	quaterN m_aLocalRotOrig;
	vector3N m_aLocalTransOrig;
	quaterN m_aRotOrigComb;
	vector3N m_aTransOrigComb;
	quaterN m_aInvRotOrigComb;
	vector3N m_aInvTransOrigComb;
protected:

	void SetPose(const Posture& posture, int* aJointToTreeIndex, MotionLoader* pSkeleton);
	// Ogre사용해서 그리는 경우 (Skinning)
	Ogre::Entity* m_pEntity;
	intvectorn m_aTargetIndexByTreeIndex; 
	double mMotionScale;
};

#endif /* _INC_PLDPRIMSKIN_38BF502900AA_INCLUDED */
