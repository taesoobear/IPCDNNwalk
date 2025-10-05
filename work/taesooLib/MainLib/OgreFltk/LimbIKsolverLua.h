#ifndef LimbIKsolverLua_H_
#define LimbIKsolverLua_H_
#include "../../BaseLib/motion/FullbodyIK_MotionDOF.h"
#include "../../BaseLib/math/optimize.h"
#include "../../MainLib/WrapperLua/luna.h"


namespace MotionUtil{
	class LimbIKsolverLua: public ::Optimize, public FullbodyIK_MotionDOF3

{
public:
	void _limbIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance);
	void _forwardKinematics(int c, quater const& theta);

	std::vector<const Bone*> mPelvis; // 실제로는 왼쪽힙과 오른쪽힙으로 부터 가장 가까운 공통 부모 조인트.
	Posture tempp;
	Optimize::ConjugateGradient cgmethod;

	inline Bone* getCenterBone(int i) const { return (Bone*)mPelvis[i];}
	LimbIKsolverLua(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& hip_bone_indexes, intvectorn const& knee_bone_indexes, vectorn const& axis_sign, lua_State* L);
	LimbIKsolverLua(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign, lua_State* L) ;
	void setSkeleton(vectorn & temp);
	lua_State* L;
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
		vector3 goal, sh, elb, v1, v2, v3, v4,v5, wrist,hand;
		vector3 hipoffset;
		const Bone *hip, *knee, *ankle;
		vector3 axis;
		void prepare(quater const& conori);
		void limbIK(double importance, quater const& conori);
	};
	LimbIKinfo _limbikInfo[4];
	inline Bone* getHipBone(int i) const { return (Bone*)_limbikInfo[i].hip;}
	inline Bone* getKneeBone(int i) const { return (Bone*)_limbikInfo[i].knee;}
	inline Bone* getAnkle(int i) const { return (Bone*)_limbikInfo[i].ankle;}
	vector3N con; 
	quaterN conori;
	vectorn impor;
 	quater theta;
	vector3N mRootPos;
	quaterN mRootOri;


	void IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con);
	void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
	void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);

	void init_cg(int ndim, double grad_step, int max_iter, double tol, double thr);

	virtual m_real objectiveFunction(vectorn const& x);
};

	class LimbIKsolverHybrid: public ::Optimize, public FullbodyIK_MotionDOF3

{
public:
	void _limbIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance);
	void _forwardKinematics(int c, quater const& theta);

	std::vector<const Bone*> mPelvis; // 실제로는 왼쪽힙과 오른쪽힙으로 부터 가장 가까운 공통 부모 조인트.
	Posture tempp;
	Optimize::ConjugateGradient cgmethod;

	std::vector<Effector> mOtherEffectors;
	vector3N mOtherConPos;
	vectorn mOtherImportance;

	inline Bone* getCenterBone(int i) const { return (Bone*)mPelvis[i];}
	LimbIKsolverHybrid(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& hip_bone_indexes, intvectorn const& knee_bone_indexes, vectorn const& axis_sign);
	LimbIKsolverHybrid(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign) ;
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
		vector3 goal, sh, elb, v1, v2, v3, v4,v5, wrist,hand;
		vector3 hipoffset;
		const Bone *hip, *knee, *ankle;
		vector3 axis;
		void prepare(quater const& conori);
		void limbIK(double importance, quater const& conori);
	};
	LimbIKinfo _limbikInfo[4];
	inline Bone* getHipBone(int i) const { return (Bone*)_limbikInfo[i].hip;}
	inline Bone* getKneeBone(int i) const { return (Bone*)_limbikInfo[i].knee;}
	inline Bone* getAnkle(int i) const { return (Bone*)_limbikInfo[i].ankle;}
	vector3N con; 
	quaterN conori;
	vectorn impor;
 	quater theta;
	vector3N mRootPos;
	quaterN mRootOri;

	void IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con);
	void IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con);
	void IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance);

	void init_cg(int ndim, double grad_step, int max_iter, double tol, double thr);

	virtual m_real objectiveFunction(vectorn const& x);
};

}
#endif
