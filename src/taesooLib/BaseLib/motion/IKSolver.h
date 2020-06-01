#pragma once

class Motion;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;
class Bone;

namespace MotionUtil
{
	// bone을 goal로 옮겨라.(setPose된 skeleton이 입력) for 3DOF knee
	Bone& conToBone(int con, MotionLoader const& skeleton);
	void IKSolveAnalytic(const MotionLoader& input_pose, Bone& bone, vector3 goal, intvectorn& index, quaterN& delta_rot, bool bKneeDamping=true, bool bToeCorrection=false);

	int limbIK( const vector3& goal, const vector3& sh, const vector3& v1, const vector3& v2, const vector3& v3, const vector3& v4,
			quater& qq1, quater& qq2, m_real ii, bool kneeDamping=true);

	void setKneeDampingCoef_RO(double ro);
	int limbIK_1DOFknee( const vector3& goal, const vector3& sh, const vector3& v1, const vector3& v2, const vector3& v3, const vector3& v4,
			quater& qq1, quater& qq2, vector3 const& axis, bool kneeDamping, m_real* lengthAdjust=NULL, double kneeDampingConstant=1.0);

	void setLimb(int con, int& index_up, int& index_mid, int& index_low, vector3& axis, const MotionLoader& skeleton);
	// 현재 다리가 펴져있는 정도를 계산한다. 1이 완전히 펴진상태를 뜻한다.
	m_real calcIKlength(const MotionLoader& skeleton, int con);
	bool isIKpossible(const MotionLoader& skeleton, int con, const vector3& input_goal, m_real lengthGoal=0.95, m_real distGoal=0.3);
}
