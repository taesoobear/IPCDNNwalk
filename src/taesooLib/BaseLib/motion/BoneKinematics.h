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

	void updateBoneLength(MotionLoader const& loader); 

	void operator=(BoneForwardKinematics const& other);
	void setPose(const Posture& pose);
	void setPoseDOF(const vectorn& poseDOF);

	void setChain(const Posture& pose, const Bone& bone);
	void setChain(const Bone& bone);
	void getPoseFromGlobal(Posture& pose) const;
	void getPoseDOFfromGlobal(vectorn& poseDOF) const;

	void getPoseFromLocal(Posture& pose) const;
	void getPoseDOFfromLocal(vectorn& poseDOF) const;

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

	void setBodyVel(int ibone, vector3 const& lin_vel, vector3 const& ang_vel);
	void getJointVel(int ibone, vector3 & lin_vel, vector3 & ang_vel);
	void computeDS(BoneForwardKinematics const& fk);
	void computeDQfromDS(BoneForwardKinematics const& fk);
};
