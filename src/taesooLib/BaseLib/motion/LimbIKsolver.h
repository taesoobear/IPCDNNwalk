#ifndef LIMBIKSOLVER_H_
#define LIMBIKSOLVER_H_
#include "../BaseLib/motion/FullbodyIK_MotionDOF.h"


namespace MotionUtil
{
class LimbIKsolver : public FullbodyIK_MotionDOF3
{
public:
	std::vector<const Bone*> mHipBones;
	std::vector<const Bone*> mKneeBones;
	std::vector<const Bone*> mAnkleBones;
	vector3N mAxis;
	Posture tempp;
	bool mbAdjustLen;

	LimbIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& hip_bone_indexes, intvectorn const& knee_bone_indexes, vectorn const& axis_sign);
	LimbIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign);
	LimbIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, const vector3N & axes);
	LimbIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, Bone const& left_knee, Bone const& right_knee);
	void setSkeleton(vectorn & temp);
	void setOption(const char* type, double val);

	vectorn temp;

	void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);
	void IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con);
	void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
};

}
#endif
