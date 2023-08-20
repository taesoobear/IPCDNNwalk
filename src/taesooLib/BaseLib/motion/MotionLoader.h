#ifndef BASELIB_MOTIONLOADER_H_
#define BASELIB_MOTIONLOADER_H_
#pragma once
// MotionLoader.h: interface for the MotionLoader class.
//      written by Taesoo Kwon.

#include "BoneKinematics.h"

class Bone : public Node
{
	int m_rotJointIndex;
	int m_transJointIndex;
	int m_voca;
	MotionLoader* m_skeleton;
	TString m_transChannels;
	TString m_rotChannels;
	
	// previously m_AnyrotAxis, m_AnyrotAngle
	//임의의 축을 설정할 때(jointAxis = "A" , jointAxis2= vector4 값)angle, axis를 저장하는 변수들
	vector3* m_rotAxes;//임의의 축을 설정할 때(jointAxis = "A" , jointAxis2= vector4 값)angle, axis를 저장하는 변수
protected:
	transf m_transfOrig; //!< translation후 rotation하는 matrix, initial posture를 저장한다. (BVH 파일에서는 translation만 갖고 있다.)

	friend class MotionLoader;
	friend class ASFLoader;
	friend class TRCLoader;
public:
	Bone ();
	virtual ~Bone();

	inline Bone* child() const			{ return (Bone*)m_pChildHead;}
	inline Bone* sibling() const		{ return (Bone*)m_pSibling;}
	inline Bone* parent() const			{ return (Bone*)m_pParent;}
	bool isRootBone() const				{ return parent()->parent()==NULL;}
	inline int treeIndex() const		{ return GetIndex();}
	inline int rotJointIndex() const	{ return m_rotJointIndex;}
	inline int transJointIndex() const	{ return m_transJointIndex;}
	inline int voca() const				{ return m_voca;}	// nickname.

	bool isDescendent(const Bone * parent) const;
	TString name() const;
	/// e.g. setChannels("XYZ", "ZXY")
	MotionLoader const& getSkeleton() const		{return *m_skeleton;}

	vector3 axis(int ichannel=0);

	// returns # of channels
	int getLocalTrans(vector3& trans, const double* dof);
	int getLocalOri(quater& out, const double* dof);

	void setChannels(const char* translation_axis, const char* rotation_axis);
	// channel with an arbitrary axis is marked as 'A'
	void setArbitraryAxes(vector3* axisValue);
	/// e.g. "ZXY"=getRotationalChannels()
	///  : matrot=RZ*RX*RY
	///  : the same as a chain of three Bones: RZ->RX->RY
	///      where a->b denotes that a is the parent bone of b.
	TString const& getRotationalChannels() const;
	TString const& getTranslationalChannels() const;

	vector3 getArbitraryAxis(int i);
	int numChannels() const;

	m_real length() const;
	void getOffset(vector3& offset) const;
	transf const& getOffsetTransform() const;

	// forward kinematics.
	// retrieve global transformations
	quater const& getRotation() const;
	vector3 const& getTranslation() const;
	void getRotation(quater& rot) const;
	void getTranslation(vector3& trans) const;
	transf const& getFrame() const;

	// retrieve local transformations
	transf const& getLocalFrame() const;

	virtual void pack(BinaryFile& bf, int version) const;
	virtual void unpack(BinaryFile& bf, int version);

	void packBVH(FILE* file, int level, MotionLoader* pLoader);

	virtual void printHierarchy(int depth=0);

	// almost private functions. (for direct manipulation of the skeleton)
	transf & _getOffsetTransform() ;
	transf & _getFrame() const;
	transf & _getLocalFrame() const;

	void updateBoneLengthFromGlobal();
};




/**
 * \ingroup group_motion
 *
 * 동작의 skeleton을 로딩하고 tree 자료구조를 갖는다. 로딩한 동작 데이타는 m_cPostureIP에 저장한다.
 *
 * Bone은 하나의 transformation매트릭스에 해당하고, 하나의 본은 rotationalJoint와 translationJoint에 동시에 해당할수도 있고 둘중 하나에만 해당할 수 있다. 물론 joint가 전혀 연결되지 않을수도 있다. 이경우 이 트랜스폼 매트릭스는 constant가 된다.

 주의 : TreeIndex와 JointIndex는 다르다. GetTreeIndex를 하면 tree bone의 index가 나오고, getRotJointIndexByTreeIndex를 하면 rotational joint의 index가 나온다.
joint index는 tree bone중 모션 캡쳐 데이타와 연결된 본들만, 순서를 따로 매긴것으로 PostureIP의 m_aRotations에 접근할때 사용하는 인덱스이다.
 */

class MotionLoader : public ModelLoader
{
	TFactory<Posture>* m_pFactory;
	mutable BoneForwardKinematics m_defaultFK;
	friend class Bone;
public:
	MotionLoader();

	MotionDOFinfo dofInfo;	

	virtual TString getName() const;
	// filename: "~.skl" or "~.mot"  (.mot인경우 m_cPostureIP도 만들어진다.)
	// option=="loadSkeletonOnly" or NULL
	MotionLoader(const char* filename, const char* option=NULL);
	virtual ~MotionLoader();

	inline void printHierarchy() { GetNode(0)->printHierarchy();}
	// exportSkeleton("~.skl");
	void exportSkeleton(const char* filename) const;

	// loadAnimation("~.anim" or "~.mot"), 상속한 클래스에서는 "~.bvh", "~.amc", "~.trc"등을 읽을 때도 사용된다.
	virtual void loadAnimation(Motion& mot, const char* filename) const;

	void updateBoneLengthFromGlobal();
	virtual void _initDOFinfo();

	// Read vocaburary information
	void readJointIndex(const char* filename);

	int numRotJoint() const;	//!< KeyFrame되어 있는 조인트의 개수를 구한다.
	int numTransJoint() const;
	int numBone() const; //!< including the DUMMY bone (= bone 0).
	inline int numEndBone() const { return m_nNumEndNode;}

	// related to BoneForwardKinematics m_defaultFK
	BoneForwardKinematics & fkSolver() const	{return m_defaultFK;}
	
	void setPose(PoseWrap pose) const;
	void setPose(const Posture& pose) const;
	void getPose(Posture& pose) const;
	void setPoseDOF(const vectorn& poseDOF) const;
	void setSphericalQ(const vectorn& q) const {fkSolver().setSphericalQ(q);}
	void getPoseDOF(vectorn& poseDOF) const;
	inline vectorn getPoseDOF() const { vectorn v; getPoseDOF(v); return v;}

	// 모든 트리를 update하지 않고, 한개의 chain만 update한다.
	void setChain(const Posture& pose, int ijoint) const;
	void setChain(const Posture& pose, Bone& bone) const;
	

	/// .bvh 또는 .mot 에서 읽은 모션데이타.
	/// .asf, .skl파일을 로딩한 경우 m_cPostureIP.numFrames()==0이다.
	Motion m_cPostureIP;

	// Posture대신 상속한 클래스를 사용하고 싶은경우 호출.
	void changeFactory(TFactory<Posture>* pFactory)	;
	const TFactory<Posture>* factory() const;

	///////////////////////////////////////////////////////
	// 뼈대 편집
	///////////////////////////////////////////////////////
	/// child의 transform은 identity가 되고, bMoveChildren==true일경우 parent의 children은 모두 새로운 차일드에 붙는다.
	virtual void insertChildBone(Bone& parent, const char* nameId, bool bMoveChildren=true);
	virtual void createDummyRootBone();
	void insertSiteBones();
	/// type="T" (translational-아직 구현안됨) or "R" (rotational) or "RT" (both)
	/// dependentMotion: all Motions that are connected to this skeleton should be updated accordingly.
	void insertJoint(Bone& target, const char* type, std::vector<Motion*> dependentMotions);
	void insertJoint(Bone& target, const char* type);	//!< assumes that only m_cPostureIP is dependent.

	// convert to a subtree.
    // m_cPostureIP and dependentMotions will be modified accordingly, so concatenate all dependent motions into m_cPostureIP before calling setNewRoot.
	void setNewRoot(Bone& newRoot);

	virtual void removeBone(Bone& target);//!< assumes that only m_cPostureIP uses this skeleton.
	virtual void removeAllRedundantBones();

	// reference skeleton과 같은 본 순서를 갖도록, bone 을 sorting한다.
	// assumption: this and reference skeleton have the same number of bones with the same hierarchy and the same naming convention, and the same reference pose.
	// also, assumes that m_cPostureIP is empty. otherwise, m_cPostureIP first needs to be manually modified using the PoseTransfer class.
	virtual void sortBones(MotionLoader const& referenceSkeleton);

	///////////////////////////////////////////////////////
	// retrieve Bone or joint indexes.
	// ex)
	// rootBone= getBoneByTreeIndex(1);
	//  or  getBoneByRotJointIndex(0);
	//  or  getBoneByTransJointIndex(0);
	//  or  getBoneByVoca(MotionLoader::HIPS); <- .ee needed
	//  or  getBoneByName("Pelvis");<- depends on motion file
	///////////////////////////////////////////////////////

	Bone& bone(int index) const;	//!< a shortcut to getBoneByTreeIndex.
	Bone& getBoneByTreeIndex(int index)	const;
	Bone& getBoneByRotJointIndex(int iRotJoint)	const;
	Bone& getBoneByTransJointIndex(int iTransJoint)	const;
	Bone& getBoneByVoca(int jointVoca)	const;
	Bone& getBoneByName(const char*) const;

	int getTreeIndexByName(const char* name) const;
	int getTreeIndexByRotJointIndex(int rotjointIndex) const;
	int getTreeIndexByTransJointIndex(int transjointIndex) const;
	int getTreeIndexByVoca(int jointVoca) const;

	/// search by name is slow because of linear traversal.
	int getRotJointIndexByName(const char* nameID) const;
	int getRotJointIndexByTreeIndex(int treeIndex) const;
	int getRotJointIndexByVoca(int jointVoca) const;

	int getTransJointIndexByName(const char* nameID);
	int getTransJointIndexByTreeIndex(int treeIndex) const;

	// linear search required.
	int getVocaByTreeIndex(int treeIndex) const;
	int getVocaByRotJointIndex(int rotjointIndex) const;

	/// to retrieve the initial pose (identity pose)
	void UpdateInitialBone();
	// almost private.
	void UpdateBone();	//!< global joint포지션 구하기 위해서 사용된다.
	void _updateTreeIndex();

	virtual void setCurPoseAsInitialPose();

	virtual void Scale(float fScale);
	virtual void scale(float fScale, Motion& mot);

	// almost private
	void _changeVoca(int jointVoca, Bone & bone);

	// translation table (joint vocaburary)
	enum { HIPS, LEFTHIP , LEFTKNEE, LEFTANKLE, LEFTTOES,RIGHTHIP, RIGHTKNEE, RIGHTANKLE, RIGHTTOES, CHEST, CHEST2, LEFTCOLLAR, LEFTSHOULDER, LEFTELBOW, LEFTWRIST, RIGHTCOLLAR, RIGHTSHOULDER, RIGHTELBOW, RIGHTWRIST, NECK, HEAD, 
		LThWrist, LThMetac, LThIntra1, LF1Metac, LF1Intra1, LF1Intra2, LF2Metac, LF2Intra1, LF2Intra2, LF3Metac, LF3Intra1, LF3Intra2, LF4Metac, LF4Intra1, LF4Intra2, // left fingers (see retarget_common.lua)
		RThWrist, RThMetac, RThIntra1, RF1Metac, RF1Intra1, RF1Intra2, RF2Metac, RF2Intra1, RF2Intra2, RF3Metac, RF3Intra1, RF3Intra2, RF4Metac, RF4Intra1, RF4Intra2, // right fingers (see retarget_common.lua)
		NUM_JOINT_VOCA};

	void pack(BinaryFile & bf, int nVersion) const;

	int getHandle() const							{ return m_handle;}

	// translation table (joint vocaburary)
	utility::NameTable m_translationTable;



	int unpack(BinaryFile & bf) ;
protected:

	void _Scale(float fScale);

	// Motion Manager specific
	friend class MotionManager;
	int m_handle;


	// tree는 있지만, node array가 없는 경우 또는 node array도 있기는 하지만 순서가 DFS순임이 보장되지 않는 경우 call해준다.
	// output numChannel.
	void MakeBoneArrayFromTree(int& numChannel);



	// 아래 내용은 모두 MakeBoneArrayFromTree수행시 계산됨
	// Posture의 joint index, voca관련
	intvectorn m_aRotJoint2TreeIndex;	//!< m_aRotJoint2TreeIndex[joint index]=TreeNode index
	intvectorn m_aTransJoint2TreeIndex;
	intvectorn m_aVoca2TreeIndex;

	void _updateVoca2TreeIndex();

	// aTree2RotJointIndex[tree node index]=joint index
	void _getTree2RotJointIndexArray(intvectorn& aTree2RotJointIndex) const;
	void _getTree2TransJointIndexArray(intvectorn& aTree2TransJointIndex) const;
	void _setTree2RotJointIndexArray(intvectorn& aTree2RotJointIndex) ;
	void _setTree2TransJointIndexArray(intvectorn& aTree2TransJointIndex) ;

	// 종류별 노드 개수.
	int m_nNumRotNode;
	int m_nNumTransNode;
	int m_nNumEndNode;	//!< EndNode(site)도 모션과 연결되어있지 않음.
};


// SourceSkeleton의 Pose를 TargetSkeleton으로 옮긴다.
class PoseTransfer
{
	void _ctor(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, const char* convfilename, bool bCurrPoseAsBindPose);
	void _ctor_part2(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, TStrings& convTable, bool bCurrPoseAsBindPose);
public:
	/*!
	joint의 targetindex를 conversion table을 사용해 세팅한다.
	\param convfilename source Bone의 이름을 target Bone의 이름과 매치시키는 conversion table파일이름.
	 - NULL인 경우, 양쪽 skeleton의 bone이 이름이 매칭된다고 가정한다.

	bCurrPoseAsBindPose가 true인 경우,
	생성하는 순간 두 skeleton이 모두 UpdateBone()되어 있고,
	두 skeleton의 자세가 거의 동일하다고 가정한다. (see bCurrPoseAsBindPos)
	(initialPose를 사용하는 것이 simple하다: 일반적으로는 모든 팔다리가 수직으로 내려와 있는 자세 skel.updateInitialBone() - identity pose.)
	*/
	PoseTransfer(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, const char* convfilename=NULL, bool bCurrPoseAsBindPose=false);
	PoseTransfer(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, bool bCurrPoseAsBindPose);
	PoseTransfer(MotionLoader* pSrcSkel, MotionLoader* pTgtSkel, TStrings const& convInfoA, TStrings const& convInfoB, bool bCurrPoseAsBindPose);

	// srcposture: srcSkeleton과 호환되는 posture.
	// results can be retrieved using
	// this->target()->getPose(..)
	void setTargetSkeleton(const Posture & srcposture);	

	// The above function modifies only the orientations for reasons.
	// if translations should also be matched, than call this function instead of the above one.
	void setTargetSkeletonBothRotAndTrans(const Posture& srcposture);

	MotionLoader* source()  { return mpSrcSkel;}
	MotionLoader* target()	{ return mpTgtSkel;}
private:
	MotionLoader* mpSrcSkel;
	MotionLoader* mpTgtSkel;
	quaterN m_aLocalRotOrig;
	quaterN m_aRotOrigComb;
	quaterN m_aInvRotOrigComb;
	quaterN m_aBindPose;
	intvectorn parentIdx;
	intvectorn m_aTargetIndex;	// from src rot joint to target tree index
	intvectorn m_aTargetIndexByTransJoint; // from src trans joint to target tree index
};
//
//param
// loaderA : source pose
// loaderB : target pose
// convInfoA (optional) : bone names of A (can be a subset)
// convInfoB (optional) : the corresponding bone names of B
// posScaleFactor : skinScaleB/skinScaleA
//
// usage:
//        1. setPose loaderA and loaderB so that both loaders are at the same pose. 
//        (The same pose can be expressed using different quaternions though)
//
// 	      PT=PoseTransfer2(loaderA, loaderB)
//        PT:setTargetSkeleton(poseA)
//        local poseB=Pose()
//        loaderB:getPose(poseB) -- now poseB (quaternions and positions) obtained
//
//        or 
//
//        local posedofB=vectorn()
//        loaderB:getPoseDOF(posedofB) -- now poseB (DOFs) obtained
//
class PoseTransfer2
{
	public:
	void _ctor(MotionLoader* loaderA, MotionLoader* loaderB, TStrings const& convInfoA, TStrings const& convInfoB, double posScaleFactor);
	public:
	PoseTransfer2(MotionLoader* loaderA, MotionLoader* loaderB, TStrings const& convInfoA, TStrings const& convInfoB, double posScaleFactor);
	PoseTransfer2(MotionLoader* loaderA, MotionLoader* loaderB);
	PoseTransfer2(MotionLoader* loaderA, MotionLoader* loaderB, const char* convfilename, double posScaleFactor);

	void _setTargetSkeleton();
	inline void setTargetSkeleton(Posture const& poseA) { loaderA->setPose(poseA); _setTargetSkeleton(); }
	inline void setTargetSkeleton(vectorn const& poseA) { loaderA->setPoseDOF(poseA); _setTargetSkeleton(); }
	inline void setTargetSkeleton(BoneForwardKinematics const& poseA) { loaderA->fkSolver()=poseA; _setTargetSkeleton(); }
	inline MotionLoader* source()  { return loaderA;}
	inline MotionLoader* target()  { return loaderB;}
	MotionLoader *loaderA, *loaderB;
	intvectorn rAtoB_additionalAindices, rAtoB_additionalBindices, targetIndexAtoB, BtoA, parentIdx;
	quaterN rAtoB, rAtoB_missing, rAtoB_additional;
	Posture bindPoseA, bindPoseB;
	double posScaleFactor;
	vector3 rootAtoB;
};


// deprecated.
int dep_GetParentJoint(MotionLoader const&, int rotjointIndex) ;
Bone& dep_GetBoneFromCon(MotionLoader const& ml, int constraint) ;	// ToeBone
Bone& dep_GetSiteBone(MotionLoader const& ml, int ijoint)	;
Bone& dep_GetSiteBoneVoca(MotionLoader const& ml, int jointVoca) ;
#endif
