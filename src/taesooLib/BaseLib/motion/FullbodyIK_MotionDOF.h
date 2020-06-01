#pragma once

#include "FullbodyIK.h"
class MotionDOFinfo;

namespace MotionUtil
{
	// Abstract class (type3)
	class FullbodyIK_MotionDOF3
	{
	public:
		std::vector<Effector> mEffectors; 
		MotionDOFinfo const& mDofInfo;
		MotionLoader const& mSkeleton;
		FullbodyIK_MotionDOF3(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors);

		virtual~ FullbodyIK_MotionDOF3(){}
		virtual void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance)=0;
		virtual void setOption(const char* type, double val){}
	};

	// Abstract class. All IK solvers should reimplement IKsolve(...)
	class FullbodyIK_MotionDOF
	{
	public:
		FullbodyIK_MotionDOF(){}
		FullbodyIK_MotionDOF(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors){}
		virtual~ FullbodyIK_MotionDOF(){}

		// general purpose functions to provide some solver-specific parameters 
		virtual void setParam(const char* type, double value){}
		virtual void setParam(const char* type, double value, double value2){}

		// returns true if bonelength change after creation of IKsolver is supported.
		// call this whenever bone-length changes.
		virtual bool _updateBoneLength(MotionLoader const& loader){ return false;}

		// interface type 1. All derived classes should reimplement this interface.
		virtual void IKsolve(vectorn const& input, vectorn& output, vector3N const& constraintPositions)=0;

		// interface type 2. Utility function. Simple to use. 
		void IKsolve(vectorn& poseInOut, vector3N const& constraintPositions);

		// optionally implement the following functions.
		// return true when successful.
		// currently only the lbfgs solver has all the implementations.
		virtual bool _changeNumEffectors(int n) { return false; }
		virtual bool _changeNumConstraints(int n) { return false; }
		virtual bool _setEffector(int i, Bone* bone, vector3 const& lpos) { return false;}
		virtual bool _setOrientationConstraint(int i, Bone* bone, quater const& desired_ori) { return false; }
		virtual bool _setOrientationConstraint(int i, Bone* bone, quater const& desired_ori, double weight) { return false; }
		virtual bool _setPositionConstraint(int i, Bone* bone, vector3 const&lpos, vector3 const& desired_pos, double wx, double wy, double wz) { return false; }
		virtual bool _setRelativeConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2) { return false;}
		virtual bool _setPlaneDistanceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth) { return false;}
		virtual bool _setHalfSpaceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth) { return false;}
		virtual bool _setCOMConstraint(int i, vector3 const& com) { return false;}
		virtual bool _setCOMConstraint(int i, vector3 const& com, double wx, double wy, double wz) { return false;}
		virtual bool _setMomentumConstraint(int i, vector3 const& ang, vector3 const& lin, double weight=0.1){return false;}
		virtual bool _setEffectorWeight(int i, double w=1.0) { return false;}
		virtual bool _setPoseConstraint(int i, vectorn const& pose, double weight, int startBoneIndex=1, int endBoneIndex=INT_MAX){return false;}
		// Allows deviation only along the vertical upward direction. 
		// Results are visible only when _setEffectorWeight(i,0.0) for all effectors so that this constraint is used instead.
		virtual bool _setEffectorYConstraint(int i, double weight, const vectorn& effectorWeights){ return false;}

		// after updating all effectors and constraints call the following function.
		virtual bool _effectorUpdated(){ return false;}
	};

	// 모든경우에 동작하며, 속도도 괜찮음. 
	FullbodyIK_MotionDOF* createFullbodyIk_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& info, std::vector<Effector>& effectors, std::vector<RelativeConstraint>& constraints);
	// COM constraint가 사용되는 경우 아래 함수 사용할 것. 이 경우 모든 조인트가 사용되기때문에 느려짐.
	FullbodyIK_MotionDOF* createFullbodyIk_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& info); 

	// 모든 경우에 동작하지만 느릴 수 있음. 비추.
	FullbodyIK_MotionDOF* createFullbodyIk_MotionDOF_MultiTarget(MotionDOFinfo const& info, std::vector<Effector>& effectors);
	FullbodyIK_MotionDOF* createFullbodyIk_MotionDOF_MultiTarget(MotionDOFinfo const& info, std::vector<Effector>& effectors, std::vector<RelativeConstraint>& constraints);
	FullbodyIK_MotionDOF* createFullbodyIk_MotionDOF_MultiTarget_Selected(MotionDOFinfo const& info, std::vector<Effector>& effectors, intvectorn const& selectedTreeIndex);

	// effector는 heels, hips combination만 가능하다. Knee본의 DOF가 1일때만 정상 동작한다. Knee본의 DOF가 3인 경우 FullbodyIK_LimbIK사용할 것.
	FullbodyIK_MotionDOF* createFullbodyIkDOF_limbIK(MotionDOFinfo const& info, std::vector<Effector>& effectors, Bone const& left_knee, Bone const& right_knee);
	FullbodyIK_MotionDOF* createFullbodyIkDOF_limbIK(MotionDOFinfo const& info, std::vector<Effector>& effectors, Bone const& left_knee, Bone const& right_knee, bool reversedAxis);
	// or you can use LimbIKsolver.h, LimbIKsolver2.h, and LimbIKsolverLua.h for more features.
	// see testLimbIK.lua for more details

	FullbodyIK_MotionDOF* createFullbodyIkDOF_limbIK_straight(MotionDOFinfo const& info, std::vector<Effector>& effectors, Bone const& left_knee, Bone const& right_knee);

	void setLimbIKParam_straight(FullbodyIK_MotionDOF* ik, bool bStraight);
	
	// IKsolve_markers works only for createFullbodyIk_MotionDOF_MultiTarget_Selected.
	void IKsolve_markers(FullbodyIK_MotionDOF* solver, vectorn const& pose, vectorn& output, vector3N const& effectors, intvectorn const& effectorTreeIndices, vector3N const& target, int max_iter);
}
