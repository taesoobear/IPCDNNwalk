#ifndef HANDIKSOLVER_H_
#define HANDIKSOLVER_H_
#include "FullbodyIK_MotionDOF.h"
#include "../math/optimize.h"
#include "LimbIKsolver.h"
#include "LimbIKsolver2.h"
#include <math.h>
#include "../../BaseLib/motion/Motion.h"
#include "../../BaseLib/utility/operatorString.h"
#include "../../BaseLib/motion/VRMLexporter.h"
#include "../../BaseLib/motion/VRMLloader.h"
namespace MotionUtil{
	class HandIKsolver: public ::Optimize
{
	void _handIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance, vectorn & r);
	void _forwardKinematics(int c, quater const& theta);
public:
	std::vector<Effector> mEffectors; 
	MotionDOFinfo const& mDofInfo;
	MotionLoader const& mSkeleton;

	int dofNum,finNum;
	std::vector<const Bone*> mHandBones;
	Optimize::ConjugateGradient cgmethod;
	std::vector<Effector> limbEffectors;
//LimbIKsolver limbIk;
	//LimbIKsolver *limbIK;
	//LimbIKsolver2 *limbIK;
	FullbodyIK_MotionDOF3 *limbIK;
	void* _options;
	transf transfSave;//pointcloudmetric 을 하는데 사용하는 변수
	transf transfSave2;//KovarMetric 을 하는데 사용하는 변수
	int wrist_num;
	vector3N wrist_conpos;
	quaterN wrist_conori;
	vectorn wrist_impo;
	intvectorn _wrist_bone_indexes;
//	std::vector<LimbIKsolver> limbMik;

	//HandIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& hand_bone_indexes,vectorn const& axis_sign,LimbIKsolver wristIK);
	HandIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& hand_bone_indexes,intvectorn const& wrist_bone_indexes,vectorn const& axis_sign);
	virtual ~HandIKsolver();
	void setOption(const char* id, double val);

	vector3N con; 
	vectorn impor;
	vectorn dofOrigin;
	vectorn poseTemp;
	transf TFtemp;
	//transf origRootTF;

	void IKsolve(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);
	void IKsolve(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance, vectorn const& importance_wrist);

	virtual m_real objectiveFunction(vectorn const& x);
};

}
#endif
