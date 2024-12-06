#pragma once
#include "node.h"
#include "ModelLoader.h"
#include "ModelLoader.h"
#include "Motion.h"
#include "MotionDOF.h"
#include "../utility/NameTable.h"

struct PoseWrap;
class MotionLoader;
class Bone;
class BoneForwardKinematics
{
	MotionLoader* m_skeleton;
	std::vector<transf> m_local;
	std::vector<transf> m_global;

public:
	BoneForwardKinematics(MotionLoader* );

	// reset to initial pose of motion data. (The initial pose is the identity pose, most of the case.)
	void init();
	// calc global from local.
	void forwardKinematics();
	// calc local from global. Bone lengths are fixed.
	void inverseKinematics();
	int numBone() const { return m_local.size();}

	// calc local from global. Bone lengths can change.
	void inverseKinematicsExact();

	void updateBoneLength(MotionLoader const& loader); 

	void operator=(BoneForwardKinematics const& other);
	void setPose(const Posture& pose);
	void setPoseDOF(const vectorn& poseDOF);
	void setPoseDOFusingCompatibleDOFinfo(MotionDOFinfo const& dofInfo, const vectorn& poseDOF);
	void setPoseDOFignoringTranslationalJoints(const vectorn& posedof_for_vrmlloader);

	// for skeletons with spherical joints, get/setSphericalQ is often more convenient than getPoseDOF/setPoseDOF.
	/* 
	 * our spherical state packing is different!!!
	 *
	 *  in our case, linear parts appear first, and then 3 DOF ball joints (YUP).
	 *         root                                     |     3           3         ...   
	 q_linear= [x, y, z, hinge1, hinge2, hinge3, hinge4]
	 q_quat  =                                          [qw,qx,qy,qz, qw2,qx2,qy2,qz2, qw3,qx3,qy3,qz3, ....]
	 q= [q_linear, q_quat]
	 */
	void setSphericalQ(const vectorn& q);

	void setChain(const Posture& pose, const Bone& bone);
	void setChain(const Bone& bone);
	void getPoseFromGlobal(Posture& pose) const;
	void getPoseDOFfromGlobal(vectorn& poseDOF) const;

	void getPoseFromLocal(Posture& pose) const;
	void getPoseDOFfromLocal(vectorn& poseDOF) const;
	void getPoseDOFignoringTranslationalJoints(vectorn& poseDOF) const;
	inline Posture getPose() { Posture pose; getPoseFromLocal(pose); return pose;}
	inline vectorn getPoseDOF() { vectorn pose; getPoseDOFfromLocal(pose); return pose;}
	inline vectorn getPoseDOFignoringTranslationalJoints() { vectorn pose; getPoseDOFignoringTranslationalJoints(pose); return pose;}

	MotionLoader const& getSkeleton() const		{return *m_skeleton;}

	// read operations
	inline transf const& local(int i) const			{ return m_local[i];}
	transf const& local(const Bone& bone) const;

	inline transf const& global(int i) const		{ return m_global[i];}
	transf const& global(const Bone& bone) const;

	// write operations
	inline transf& _local(int i)					{ return m_local[i];}
	transf& _local(const Bone& bone);

	inline transf& _global(int i)					{ return m_global[i];}
	transf& _global(const Bone& bone);
};

class BoneVelocityForwardKinematics
{
	const MotionLoader* _loader;
public:
	vector3N array_loc_lin_vel;
	vector3N array_loc_ang_vel;
	vector3N array_rel_lin_vel;
	vector3N array_rel_ang_vel;

	BoneVelocityForwardKinematics(const MotionLoader* loader);
	
	// compute local velocity from relative velocity (forwardKinematics)
	void computeDS(BoneForwardKinematics const& fk);

	// inverseKinematics:
	// 1. set local velocity
	void setBodyVel(int ibone, vector3 const& lin_vel, vector3 const& ang_vel);

	// 2. compute relative velocity from local velocity
	void computeDQfromDS(BoneForwardKinematics const& fk);

	// 3. get relative velocity
	void getJointVel(int ibone, vector3 & lin_vel, vector3 & ang_vel);
};
