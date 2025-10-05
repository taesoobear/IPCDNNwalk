#ifndef LIMBIKSOLVER2_H_
#define LIMBIKSOLVER2_H_
#include "FullbodyIK_MotionDOF.h"
#include "../math/optimize.h"

namespace MotionUtil{
	class LimbIKsolver2: public ::Optimize, public FullbodyIK_MotionDOF3

{
	void _limbIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance);
	void _forwardKinematics(int c, quater const& theta);
public:
	std::vector<const Bone*> mPelvis; // 실제로는 왼쪽힙과 오른쪽힙으로 부터 가장 가까운 공통 부모 조인트.
	Posture tempp;
	Optimize::ConjugateGradient cgmethod;

	LimbIKsolver2(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign);
	void setSkeleton(vectorn & temp);

	struct LimbIKinfo {
		//sh : hip position
		//q1 : hip angle
		//elb : knee position
		//q2 : knee angle
		//v1 : hip-knee  nat
		//v3 : hip-knee  cur
		//wrist : ankle position
		//v2 : knee-ankle nat
		//v4 : knee-ankle cur
		//goal : ankle
		quater qo1, qo2,qt;
		quater q0, q1, q2;
		vector3 goal, sh, elb, v1, v2, v3, v4,v5, wrist,hand, leg_delta;
		vector3 hipoffset;
		const Bone *hip, *knee, *ankle;
		vector3 axis;
		void prepare(quater const& conori, const vector3 & localpos);
		void limbIK(double importance, quater const& conori);
	};
	LimbIKinfo _limbikInfo[4];
	inline Bone* getHipBone(int i) const { return (Bone*)_limbikInfo[i].hip;}
	inline Bone* getKneeBone(int i) const { return (Bone*)_limbikInfo[i].knee;}
	inline Bone* getAnkleBone(int i) const { return (Bone*)_limbikInfo[i].ankle;}

	vectorn temp;
	double valL,valM,valN;
	int iterNum;

	vector3N con; 
	quaterN conori;
	vectorn impor;
 	quater theta;
	vector3N mRootPos;
	vector3N OrRoot;
	quaterN mRootOri;

	int flagR, flagT, flagC;	
	//error 계산식에 사용될 가중치
	double l,m,n; 

	int optiR,optiT,iterik;
	void _updateRootInfo();

	void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);
	void IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con);
	void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
	void setOption(int option);
	void setOption(const char* type, double val);
	void setValue(double ValL,double ValM,double ValN,int IterNum);
	virtual m_real objectiveFunction(vectorn const& x);
	virtual m_real _prepare(int c);
	virtual m_real _calcRootRotation();
	virtual m_real _calcRootTranslation();
};

}
#endif
