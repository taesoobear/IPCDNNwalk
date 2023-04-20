
#include "stdafx.h"
#include "BoneKinematics.h"
#include "MotionLoader.h"

	transf const& BoneForwardKinematics::global(const Bone& bone) const	{ return m_global[bone.treeIndex()];}
	transf const& BoneForwardKinematics::local(const Bone& bone) const	{ return m_local[bone.treeIndex()];}
	transf & BoneForwardKinematics::_global(const Bone& bone) 	{ return m_global[bone.treeIndex()];}
	transf & BoneForwardKinematics::_local(const Bone& bone) 	{ return m_local[bone.treeIndex()];}


BoneForwardKinematics::BoneForwardKinematics(MotionLoader* pskel)
:m_skeleton(pskel)
{	
}

void BoneForwardKinematics::forwardKinematics()
{
	// this is thread unsafe so.. 
	// NodeStack  & stack=m_skeleton->m_TreeStack;
	NodeStack  stack;
	stack.Initiate();

	Node *src=m_skeleton->m_pTreeRoot->m_pChildHead;	// dummy노드는 사용안함.
	int index=-1;

	while(TRUE) 
	{
		while(src)
		{
			index++;
			ASSERT(src->NodeType==BONE);
			Bone* pBone=(Bone*)src;
			int treeindex=pBone->treeIndex();
			if(stack.GetTop())
				_global(treeindex).mult(
				global(*((Bone*)stack.GetTop())), local(treeindex));
			else
				_global(treeindex)=local(treeindex);

			stack.Push(src);

			src=src->m_pChildHead;
		}
		stack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
}



void BoneForwardKinematics::init()
{
	m_local.resize(m_skeleton->numBone());
	m_global.resize(m_skeleton->numBone());
	m_local[0].identity();
	m_global[0].identity();

	for(int i=1, ni=m_skeleton->numBone(); i<ni; i++)
		m_local[i]=m_skeleton->bone(i).getOffsetTransform();

	forwardKinematics();
}

void BoneForwardKinematics::operator=(BoneForwardKinematics const& other)
{
	ASSERT(getSkeleton().numBone()==other.getSkeleton().numBone());

	for(int i=0,ni=getSkeleton().numBone(); i<ni; i++)
	{
		_local(i)=other.local(i);
		_global(i)=other.global(i);	
	}
}
void BoneForwardKinematics::setPose(const Posture& pose)
{
	if(m_local.size()==0)
		init();
	// update root position and rotations
	for(int ijoint=0, nj=m_skeleton->numRotJoint(); ijoint<nj; ijoint++)
	{
		// update rotations
		int target=m_skeleton->getTreeIndexByRotJointIndex(ijoint);
		m_local[target].rotation=pose.m_aRotations[ijoint];
	}

	for(int ijoint=0, nj=MIN(m_skeleton->numTransJoint(), pose.m_aTranslations.size()); ijoint<nj; ijoint++)
	{
		// update translations
		int target=m_skeleton->getTreeIndexByTransJointIndex(ijoint);
		m_local[target].translation=pose.m_aTranslations[ijoint];
	}

	forwardKinematics();
}

void BoneForwardKinematics::setPoseDOF(const vectorn& dof)
{
	MotionDOFinfo const& dofInfo=m_skeleton->dofInfo;
	setPoseDOFusingCompatibleDOFinfo(dofInfo,  dof);

}
void BoneForwardKinematics::setPoseDOFusingCompatibleDOFinfo(MotionDOFinfo const& dofInfo, const vectorn& dof)
{
	// thread unsafe equivalent: setPose(m_skeleton->dofInfo.setDOF(poseDOF));	
	MotionLoader* skeleton=&dofInfo.skeleton();
	
	int start=0;
	for(int i=1; i<skeleton->numBone(); i++)
	{
		Bone& bone=skeleton->bone(i);
		if(bone.transJointIndex()!=-1)
		{
			vector3& trans=m_local[bone.treeIndex()].translation;
			int nc=bone.getLocalTrans(trans, &dof[start]);
			start+=nc;
		}

		int ri=bone.rotJointIndex();
		if(ri==-1) continue;

		quater& rotation=m_local[bone.treeIndex()].rotation;
		if(dofInfo.hasQuaternion(i))
		{
			rotation=dof.toQuater(start);
			start+=4;
		}

		if(dofInfo.hasAngles(i))
		{
			int nc=bone.getLocalOri(rotation, &dof[start]);
			start+=nc;
		}
	}
	assert(start==dof.size());
	forwardKinematics();
}

void BoneForwardKinematics::setSphericalQ(const vectorn& q)
{
	MotionDOFinfo& dofInfo=m_skeleton->dofInfo;
	int nquat=dofInfo.numSphericalJoint();
	int ndof=dofInfo.numDOF();
	int qindex=0;
	int qsindex=ndof-nquat*4;

	ASSERT(q.size()==ndof);

	for(int ibone=1; ibone<dofInfo.numBone(); ibone++)
	{
		Bone& bone=m_skeleton->bone(ibone);
		int ndof=dofInfo.numDOF(ibone);
		transf& _local=m_local[ibone];

		bool hasTrans=(bone.transJointIndex()!=-1);
		bool hasQuat=dofInfo.hasQuaternion(ibone);
		if(ibone==1 && hasTrans && hasQuat)
		{
			// free joint
			// translation
			_local.translation=q.toVector3(qindex);
			qindex+=3;

			// rotational
			_local.rotation=q.toQuater(qsindex);
			qsindex+=4;
		}
		else if(hasQuat)
		{
			_local.rotation= q.toQuater(qsindex);
			qsindex+=4;
		}
		else if(hasTrans)
		{
			ASSERT(ndof>0); // 
			vector3& trans=_local.translation;
			int nc=bone.getLocalTrans(trans, &q(qindex));
			qindex+=nc;
		}
		else if(dofInfo.hasAngles(ibone))
		{
			quater& rotation=_local.rotation;
			int nc=bone.getLocalOri(rotation, &q(qindex));
			qindex+=nc;
		}
	}
	ASSERT(qindex==dofInfo.numDOF()-nquat*4);
	ASSERT(qsindex==q.size());
	forwardKinematics();
}

void BoneForwardKinematics::getPoseFromLocal(Posture& pose) const
{
	pose.Init(m_skeleton->numRotJoint(), m_skeleton->numTransJoint());

	// update root position and rotations
	for(int ijoint=0, nj=m_skeleton->numRotJoint(); ijoint<nj; ijoint++)
	{
		// update rotations
		int target=m_skeleton->getTreeIndexByRotJointIndex(ijoint);
		pose.m_aRotations[ijoint]=m_local[target].rotation;
	}

	for(int ijoint=0, nj=m_skeleton->numTransJoint(); ijoint<nj; ijoint++)
	{
		// update translations
		int target=m_skeleton->getTreeIndexByTransJointIndex(ijoint);
		pose.m_aTranslations[ijoint]=m_local[target].translation;
	}
}

void BoneForwardKinematics::getPoseFromGlobal(Posture& pose) const
{
	pose.Init(m_skeleton->numRotJoint(), m_skeleton->numTransJoint());

	// update root position and rotations
	for(int ijoint=0, nj=m_skeleton->numRotJoint(); ijoint<nj; ijoint++)
	{
		// update rotations
        Bone& target=m_skeleton->getBoneByRotJointIndex(ijoint);
		pose.m_aRotations[ijoint]=global(*target.parent()).toLocalRot(global(target).rotation);
	}
	for(int ijoint=0, nj=m_skeleton->numTransJoint(); ijoint<nj; ijoint++)
	{
		// update translations
        Bone& target=m_skeleton->getBoneByTransJointIndex(ijoint);
		pose.m_aTranslations[ijoint]=global(*target.parent()).toLocalPos(global(target).translation);
	}
}
void BoneForwardKinematics::updateBoneLength(MotionLoader const& loader)
{
	for(int i=2; i<loader.numBone()-1 ; i++)
		m_local[i].translation=loader.bone(i).getOffsetTransform().translation;

	forwardKinematics();
}

void BoneForwardKinematics::getPoseDOFfromGlobal(vectorn& poseDOF) const
{
	Posture p;
	getPoseFromGlobal(p);
	m_skeleton->dofInfo.getDOF(p, poseDOF);
}
void BoneForwardKinematics::getPoseDOFfromLocal(vectorn& poseDOF) const
{
	Posture p;
	getPoseFromLocal(p);
	m_skeleton->dofInfo.getDOF(p, poseDOF);
}
void BoneForwardKinematics::inverseKinematics()
{
	Posture pose;
	getPoseFromGlobal(pose);
	setPose(pose);
}
void BoneForwardKinematics::inverseKinematicsExact()
{
	auto& loader=*m_skeleton;
	for(int i=2; i<loader.numBone(); i++)
	{
		int parent=loader.bone(i).parent()->treeIndex();
		// T_p*T_local=T
		_local(i).mult(global(parent).inverse(),global (i));
	}
	_local(1)=global(1);
}

void BoneForwardKinematics::setChain(const Posture& pose, const Bone & bone)
{
	// update rotate chains
	vector3 offset;
	const Bone* pBone=&bone;
	while(pBone->child())
		pBone=pBone->child();
	do
	{
		if(pBone->rotJointIndex()!=-1)
			_local(*pBone).rotation=pose.m_aRotations[pBone->rotJointIndex()];

		if(pBone->transJointIndex()!=-1)
			_local(*pBone).translation=pose.m_aTranslations[pBone->transJointIndex()];

		pBone=pBone->parent();
	}
	while(pBone);

	setChain(bone);
}

void BoneForwardKinematics::setChain(const Bone & bone)
{

#if _WIN32
	int chain[100];
	assert(m_local.size()<100); // stack
#else
	int chain[m_local.size()]; // stack
#endif
	int chain_size=0;

	const Bone* ibone=&bone;
	while(ibone->child())
		ibone=ibone->child();

	while(1)
	{
		chain[chain_size++]=ibone->treeIndex();
		Msg::verify(chain_size<20,"bone_FK stack overflow");

		Bone* parent=ibone->parent();
		if(!parent) break;
		ibone=parent;
	}


	ASSERT(&m_skeleton->bone(chain[chain_size-1])==(Bone*)m_skeleton->m_pTreeRoot);	// dummy node
	ASSERT(&m_skeleton->bone(chain[chain_size-2])==((Bone*)m_skeleton->m_pTreeRoot)->child());

	_global(chain[chain_size-2])=local(chain[chain_size-2]);
	for(int i=chain_size-3; i>=0; i--)
		_global(chain[i]).mult(global(chain[i+1]), local(chain[i]));
}

BoneVelocityForwardKinematics::BoneVelocityForwardKinematics(const MotionLoader* loader):_loader(loader){
	int nb=loader->numBone();
	array_loc_lin_vel.setSize(nb);
	array_loc_ang_vel.setSize(nb);
	array_rel_lin_vel.setSize(nb);
	array_rel_ang_vel.setSize(nb);
}

void BoneVelocityForwardKinematics::setBodyVel(int ibone, vector3 const& lin_vel, vector3 const& ang_vel)
{
	array_loc_lin_vel(ibone)=lin_vel;
	array_loc_ang_vel(ibone)=ang_vel;
}
void BoneVelocityForwardKinematics::getJointVel(int ibone, vector3 & lin_vel, vector3 & ang_vel)
{
	lin_vel=array_rel_lin_vel(ibone);
	ang_vel=array_rel_ang_vel(ibone);
}

void BoneVelocityForwardKinematics::computeDS(BoneForwardKinematics const& fk)
{
	int nb=_loader->numBone();
	for(int i=1; i<nb; i++){
		const auto& _local=fk.local(i);
		auto* parent=_loader->bone(i).parent();
		auto& rel_lin_vel=array_rel_lin_vel(i);
		auto& rel_ang_vel=array_rel_ang_vel(i);
		auto& loc_lin_vel=array_loc_lin_vel(i);
		auto& loc_ang_vel=array_loc_ang_vel(i);
		if(i!=1)
		{
			auto& parent_loc_lin_vel=array_loc_lin_vel(parent->treeIndex());
			auto& parent_loc_ang_vel=array_loc_ang_vel(parent->treeIndex());
			quater t_rel_att;
			t_rel_att.inverse(_local.rotation);
			vector3 v1, v2;
			// compute loc_lin_vel
			v1.cross(parent_loc_ang_vel, _local.translation);
			v2.rotate(t_rel_att, parent_loc_lin_vel);
			loc_lin_vel.rotate(t_rel_att, v1);
			loc_lin_vel += v2;
			loc_lin_vel += rel_lin_vel;
			// compute loc_ang_vel
			loc_ang_vel.rotate(t_rel_att, parent_loc_ang_vel);
			loc_ang_vel += rel_ang_vel;
		}
		else
		{
			loc_ang_vel=rel_ang_vel;
			loc_lin_vel=rel_lin_vel;
		}
	}
}


void BoneVelocityForwardKinematics::computeDQfromDS(BoneForwardKinematics const& fk)
{
	int nb=_loader->numBone();
	for(int i=1; i<nb; i++){
		const auto& _local=fk.local(i);
		auto* parent=_loader->bone(i).parent();
		auto& rel_lin_vel=array_rel_lin_vel(i);
		auto& rel_ang_vel=array_rel_ang_vel(i);
		auto& loc_lin_vel=array_loc_lin_vel(i);
		auto& loc_ang_vel=array_loc_ang_vel(i);
		if(i!=1)
		{
			const auto& parent_loc_lin_vel=array_loc_lin_vel(parent->treeIndex());
			const auto& parent_loc_ang_vel=array_loc_ang_vel(parent->treeIndex());

			quater t_rel_att;
			t_rel_att.inverse(_local.rotation);
			vector3 v1, v2;
			// compute loc_lin_vel
			v1.cross(parent_loc_ang_vel, _local.translation);
			v2.rotate(t_rel_att, parent_loc_lin_vel);

			//loc_lin_vel= A+v2+rel_lin_vel
			//-> rel_lin_vel=-A-v2+loc_linvel
			rel_lin_vel.rotate(t_rel_att, v1);
			rel_lin_vel *=-1; // -A
			rel_lin_vel -= v2;
			rel_lin_vel += loc_lin_vel;

			// loc_ang_vel= B+rel_ang_vel
			// -> rel_ang_vel= -B+loc_ang_vel
			rel_ang_vel.rotate(t_rel_att, parent_loc_ang_vel);
			rel_ang_vel *=-1; // -B
			rel_ang_vel += loc_ang_vel ;
			//printf("%s %s\n", loc_ang_vel.output().ptr(), rel_ang_vel.output().ptr());
		}
		else
		{
			rel_ang_vel=loc_ang_vel;
			rel_lin_vel=loc_lin_vel;
		}
	}
}
