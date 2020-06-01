#ifndef COM_IKSOLVER_H_
#define COM_IKSOLVER_H_
#include "../math/optimize.h"
class VRMLloader;
#include "FullbodyIK_MotionDOF.h"

namespace MotionUtil{
	class COM_IKsolver: public ::Optimize
{
public:
	std::vector<Effector> mEffectors; 
	MotionDOFinfo const& mDofInfo;
	VRMLloader const& mSkeleton;

	std::vector<const Bone*> mHipBones;
	std::vector<const Bone*> mKneeBones;
	std::vector<const Bone*> mAnkleBones;
	vector3N mAxis;
	Posture tempp;
	bool mbAdjustLen;
	Optimize::ConjugateGradient cgmethod;


	COM_IKsolver(VRMLloader const& skel, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign);
	void setSkeleton(vectorn & temp);

	vectorn temp;
	vector3 goal, sh, elb, v1, v2, v3, v4, wrist,hand;
	quater q1, q2;
	quater q0;
	vectorn r;
	Posture currPose;
	quater  ankleGlobal[4];
	vector3N con;
	vector3 com, ocom, desiredCom;
	double totalMass;
  vectorn importance;

  void _prepare(int c);
  void _ikSolve(int c);
  void _iksolve(vector3 const& newPelvisPos);
  void calcPelvisPos(vectorn& origRootTF_, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& _con, vectorn const& _importance,vector3 const& _desiredCOM, vector3& pelvisPos);
  void IKsolve(vectorn& origRootTF_, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& _con, vectorn const& _importance, vector3 const& _desiredCOM);
  virtual m_real objectiveFunction(vectorn const& x);
  void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
};
}
#endif
