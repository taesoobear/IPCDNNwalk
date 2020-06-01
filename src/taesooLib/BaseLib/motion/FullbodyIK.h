#pragma once

class Bone;
class Motion;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;
class VRMLloader;
class Posture;

namespace MotionUtil
{
	struct Effector
	{
		Effector()	{localpos=vector3(0,0,0);}
		Bone* bone;
		vector3 localpos;
		quater  localori;
	};


	// actually all types of constraints
	struct RelativeConstraint
	{
		class ConstraintInfo
		{
			public:
				ConstraintInfo(){}
				// theta: euler pose.
				virtual void initializeConstraint(vectorn const& poseDOF, double *theta){}
				virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double weight){return 0;}
		};
		// order is important!
		enum { RELATIVE_POSITION, PLANE_DISTANCE, HALF_SPACE, MOMENTUM, POSE, COM, ROT, OTHERS};
		int eType;
		Bone* bone1;
		vector3 localpos1;
		double weight;
		RelativeConstraint() { eType=RELATIVE_POSITION; bone1=NULL; bone2=NULL; localpos1=vector3(0,0,0); weight=1.0;}
		union {
			struct {
				// Used when eType==RELATIVE_POSITION
				// Bone1's localpos1 is attached to Bone2's (0,0,0)
				// Relative
				Bone* bone2;
			};
			struct {
				// Used when eType==PLANE_DISTANCE or HALF_SPACE
				// desired Plane (n*x+idepth=0)
				double idepth;
				double normal[3]; // global normal.x,y,z
			};
			struct {
				// Used when eType>=MOMENTUM
				ConstraintInfo* pInfo;
			};
		};
	};

	// Abstract class. All IK solvers should reimplement IKsolve(...)
	// This class is for loaders having 3-DOF knee joints
	// For 1-DOF knees, use FullbodyIK_MotionDOF.
	class FullbodyIK
	{
	public:
		FullbodyIK(){}
		FullbodyIK(MotionLoader& skeleton, std::vector<Effector>& effectors){}
		virtual~ FullbodyIK(){}


		// interface type 1. All derived classes should reimplement this interface.
		virtual void IKsolve(Posture const& input_pose, vector3N const& constraintPositions, intvectorn & rot_joint_index, quaterN& delta_rot, intvectorn & trans_joint_index, vector3N& delta_trans)=0;
		
		// interface type 2. Utility function. Simple to use. 
		void IKsolve(Posture& poseInOut, vector3N const& constraintPositions);

		void IKsolve(Posture const& input, Posture& output, vector3N const& constraintPositions);

		// interface type 3. Utility function. - iksolver에서 translation은 건드리지 않는경우 이 함수를 쓰는 것이 더 인터페이스가 간단할지도.
		void IKsolve(Posture const& input_pose, vector3N const& constraintPositions, intvectorn & rot_joint_index, quaterN& delta_rot);

		virtual void getAffectedDOF(intvectorn & rot_joint_index, intvectorn & trans_joint_index)=0;		
	};

	// localpos=(0,0,0), effector가 각각 독립된 길이 3짜리 체인으로 가정하고 동작. (TOE나 FINGERTIP본의 경우 체인 길이 4)
	FullbodyIK* createFullbodyIk_LimbIK(MotionLoader& skeleton, std::vector<Effector>& effectors, bool bUseKneeDamping=true);

	// 모든 경우에 동작하지만 느림.
	FullbodyIK* createFullbodyIk_MultiTarget(MotionLoader& skeleton, std::vector<Effector>& effectors);
}
