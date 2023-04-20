#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "./IKSolver.h"
#include "motion/Motion.h"
#include "motion/MotionUtil.h"
#include "motion/MotionLoader.h"

#define NO_DEBUG_GUI
//#define DEBUG_OUTPUT
static double _RO=TO_RADIAN(130);
// l1과 l2 사이의 각도.  in [0, M_PI]
std::pair<m_real,bool> angle(m_real l1, m_real l2, m_real l3)
{
	//printf("l: %f %f %f\n", l1, l2, l3);
	m_real cos_theta=(l1*l1+l2*l2-l3*l3)/(2*l1*l2);
	if( cos_theta>1 || cos_theta<-1 )
	{
		if(cos_theta>0)
			return std::pair<m_real, bool>(	0, false);
		return std::pair<m_real, bool>(	M_PI, false);
	}

	return std::pair<m_real, bool>(acos(cos_theta), true);
}

static m_real increaseLengthAmt(m_real l1, m_real l2, m_real l3, m_real cos_theta)
{
	// (l1+r), (l2+r) 사이각이 cos_theta이면 맞은편 길이가 l3가 되는 r구하기.

	// cos_theta=((l1+r)^2+(l2+r)^2-l3^2) / 2*(l1+r)*(l2+r)

	// ->

	// (2-2cos_theta) r^2 + (2-2cos_theta)*(l1+l2)*r +l1^2+l2^2-l3^2-2*l1*l2*cos_theta=0

	// case 1: 2-2cos_theta=0인경우.
	if(isSimilar(cos_theta, 1))
		return 0;

	m_real a=2-2.0*cos_theta;
	m_real b=a*(l1+l2);
	m_real c=l1*l1+l2*l2-l3*l3-2*l1*l2*cos_theta;

	m_real t=sqrt(b*b-4*a*c);
	m_real x1=(-b+t)/(2*a);
	m_real x2=(-b-t)/(2*a);

	return x1;	// 항상 큰 값이 원하는 답.
}

m_real Alpha(m_real t)
{
	// Kovar paper.
	// \integral alpha
	return t*t*t*(0.5*t-1.0)+t;
}

m_real F(m_real x, m_real ro)
{
	m_real t;
	t=(x-ro)/(M_PI-ro);
	//ASSERT(t>=0.0-FERR && t<=1.0+FERR);
	return Alpha(t)*(M_PI-ro);
}
m_real integral_f(m_real a, m_real b, m_real ro)
{
	if(a>b)
		return -1*integral_f(b,a,ro);

	ASSERT(a<=b);
	ASSERT(b>=ro);

	if(a<ro)
	{
		// integral_a^ro + integral_ro^b
		return ro-a+F(b, ro); // -F(ro, ro) is omit because F(ro, ro)=0
	}
	else
		return F(b, ro)-F(a, ro);
}

Bone&  MotionUtil::conToBone(int con, MotionLoader const& skeleton)
{

	int index_up;
	int index_low;
	int index_mid;
	vector3 axis(0,0,1);
	setLimb(con, index_up,  index_mid, index_low, axis, skeleton);
	return skeleton.bone(index_up);
}


// limb ik solver
// goal is the goal position of wrist (global)
// sh is the shoulder position (global)
// v1 is the vector from the shoulder to the elbow (in neutral pose : offset)
// v2 is the vector from the elbow to the wrist : offset
// v3 is the vector from the shoulder to the elbow (in current pose != offset)
// v4 is the vector from the elbow to the wrist (in current pose)
// qq1 is the input and output shoulder joint angle (global)
// qq2 is the input and output elbow joint angle (global)
// ii is the importance value
// axis is the elbow rotation axis
//
double MotionUtil::limbIK( const vector3& goal, const vector3& sh, const vector3& v1, const vector3& v2, const vector3& v3, const vector3& v4,
			quater& qq1, quater& qq2, m_real ii, bool kneeDamping)
{
	// 1은 shoulder, 2는 elbow, 3은 wrist
	const vector3& d12=v3;
	const vector3& d23=v4;
	vector3 d13=d12+d23;
	vector3 d1G=goal-sh;
	m_real d12_len=d12.length();
	m_real d23_len=d23.length();
	m_real d13_len=d13.length();
	m_real d1G_len=d1G.length();

	m_real desiredAngle=angle(d12_len, d23_len, d1G_len).first;
	m_real currAngle;
	quater q2;
	q2.axisToAxis(d23, -d12);

	vector3 axis_original;
	q2.toAxisAngle(axis_original, currAngle);


	// knee damping suggested in Foot skate cleanup by kovar
	if(kneeDamping)
	{
		m_real theta_o=currAngle;
		m_real& theta=desiredAngle;

		m_real dtheta=theta-theta_o;

		// f(x) = 1, x< ro
		// f(x) = 2x^3+3x^2+1, otherwise
		// theta= theta_o + \integral _theta_o ^ {theta+dtheta}  f(x) dx

		m_real ro=_RO;

		if(theta>ro || theta_o >ro )
			theta=theta_o+integral_f(theta_o, theta, ro);
	}



	quater q_delta2(currAngle-desiredAngle, axis_original);

	vector3 d23_after;
	d23_after.rotate(q_delta2, d23);

	vector3 d13_after=d12+d23_after;

	quater q_delta1;
	q_delta1.axisToAxis(d13_after, d1G);

	qq1=q_delta1*qq1;
	qq2=q_delta1*q_delta2*qq2;

	return d13_after.length();
}


void MotionUtil::setKneeDampingCoef_RO(double ro)
{
	_RO=ro;
}
double MotionUtil::getKneeDampingCoef_RO()
{
	return _RO;
}

#ifndef NO_DEBUG_GUI
#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/objectList.h"
#endif

int MotionUtil::limbIK_1DOFknee( const vector3& goal, const vector3& sh, const vector3& v1, const vector3& v2, const vector3& v3, const vector3& v4,
			quater& qq1, quater& qq2, vector3 const& axis, bool kneeDamping, m_real* lengthAdjust, double kneeDampingConstant)
{
	// 1은 shoulder, 2는 elbow, 3은 wrist
	vector3 d12=v3;
	vector3 d23=v4;
	vector3 d13=d12+d23;
	vector3 d1G=goal-sh;
	m_real d12_len=v1.length();	// original length before applying knee damping!
	m_real d23_len=v2.length(); // original length before applying knee damping!
	m_real d13_len=d13.length();
	m_real d1G_len=d1G.length();

#ifndef NO_DEBUG_GUI
			{
				double skinScale=2.54;
				RE::moveEntity(RE::createEntity("metric0","sphere1010.mesh"),vector3(2,1,1),sh*skinScale );
				RE::moveEntity(RE::createEntity("metric1","sphere1010.mesh"),vector3(2,1,1),(sh+v1)*skinScale);
				RE::moveEntity(RE::createEntity("metric2","sphere1010.mesh"),vector3(2,1,1),(sh+v1+v2)*skinScale);
				RE::moveEntity(RE::createEntity("metric4","sphere1010.mesh"),vector3(1,1,2),goal*skinScale);
			}
#endif
	// using the following assumptions, the problem become quite easy to solve.
	// Many of existing motion data do not satisfy the following assumption!
	// However, it is possible to define coordinate systems in which the following assumption holds without distoring the motion data too much. (see retargetMotToVrml.lua for details)
	//ASSERT(isSimilar(v1%axis,0));
	//ASSERT(isSimilar(v2%axis,0));


	m_real desiredAngle=angle(d12_len, d23_len, d1G_len).first;
#ifdef DEBUG_OUTPUT
	printf("desiredangle %f ->", desiredAngle);
#endif
	m_real currAngle;
	quater q2;
	q2.axisToAxis(d23, -d12);

	vector3 axis_original;
	q2.toAxisAngle(axis_original, currAngle);


	// knee damping suggested in Foot skate cleanup by kovar
	if(kneeDamping)
	{
		m_real theta_o=currAngle;
		m_real& theta=desiredAngle;

		m_real dtheta=theta-theta_o;

		// f(x) = 1, x< ro
		// f(x) = 2x^3+3x^2+1, otherwise
		// theta= theta_o + \integral _theta_o ^ {theta+dtheta}  f(x) dx

		//		m_real ro=TO_RADIAN(130);
		m_real ro=_RO;

#ifdef DEBUG_OUTPUT
		printf("before : %f %f \n", theta, theta_o);
#endif
		if(theta>ro || theta_o >ro )
			theta=theta_o+integral_f(theta_o, theta, ro);

#ifdef DEBUG_OUTPUT
		printf("after1 : %f %f \n", theta, theta_o);
#endif
		theta=theta_o+(theta-theta_o)*kneeDampingConstant;
#ifdef DEBUG_OUTPUT
		printf("after2 : %f %f \n", theta, theta_o);
#endif
	}
#ifdef DEBUG_OUTPUT
	printf("desiredangle %f \n", desiredAngle);
#endif

	if(1)
	{
		quater q_delta1, q_delta2;
		q_delta1.axisToAxis(d13, d1G);

		vector3 ld23;
		ld23=qq2.inverse()*d23;
		// local orientation at 2.
		quater lq2;
		if (1)
		{
			// offset V1 and V2 may not be straight so,
			// from qq2=qq1*lq2
			lq2.mult(qq1.inverse(), qq2);
			quater q_delta2(currAngle-desiredAngle, axis);
			lq2.leftMult(q_delta2);
		}
		else
			// 1은 shoulder, 2는 elbow, 3은 wrist
			lq2=quater(3.141592-desiredAngle, axis);

		qq2=qq1*lq2;
		vector3 d23_after=qq2*ld23;

		vector3 d13_after=d12+d23_after;
		q_delta2.axisToAxis(d13_after, d13);
		qq1=q_delta1*q_delta2*qq1;
		qq2=q_delta1*q_delta2*qq2;
	}
	else{
		// buggy
	// local orientation at 2.
	quater lq2;
	// from qq2=qq1*lq2
	lq2.mult(qq1.inverse(), qq2);

	quater q_delta2(currAngle-desiredAngle, axis);

#ifdef DEBUG_OUTPUT
	printf("lq2 %f %s %f %s\n",lq2.rotationAngle(),  lq2.output().ptr(), q_delta2.rotationAngle(), q_delta2.output().ptr());
#endif

	vector3 d23_after;

	if(lengthAdjust)
	{
		m_real r=increaseLengthAmt(d12_len, d23_len, d1G_len, cos(desiredAngle));
		vector3 dir;

		// prevent too much change.
		if( r/d12_len>0.05 )
			r=d12_len*0.05;
		else if(r/d12_len<-0.05)
			r=d12_len*-0.05;

		dir.normalize(d12);
		d12=d12+dir*r;

		dir.normalize(d23);
		d23=d23+dir*r;

		*lengthAdjust=r;
	}

	d23_after.rotate(qq2*q_delta2*qq2.inverse(), d23);

	vector3 d13_after=d12+d23_after;

	quater q_delta1;
	q_delta1.axisToAxis(d13_after, d1G);


	// qq1=q_delta1*qq1
	// qq2=q_delta1*qq2*q_delta2

	qq1=q_delta1*qq1;
	qq2=q_delta1*qq2*q_delta2;

	}
	return 0;
}
void MotionUtil::IKSolveAnalytic(const MotionLoader& skeleton, Bone& bone, vector3 input_goal, intvectorn& index, quaterN& delta_rot, bool bKneeDamping, bool bToeCorrection)
{
	vector3 goal;
	Bone* bone_lower;
	Bone* bone_middle;
	Bone* bone_upper;

	if(bToeCorrection)
		bone_lower=bone.parent();
	else
		bone_lower=&bone;

	bone_middle=bone_lower->parent();
	bone_upper=bone_middle->parent();

	index.setSize(3);

	index[0]=bone_upper->rotJointIndex();
	index[1]=bone_middle->rotJointIndex();
	index[2]=bone_lower->rotJointIndex();

#ifdef _DEBUG
	for(int i=0; i<index.size(); i++)
		ASSERT(index[i]!=-1);
#endif

	vector3 l1, l2;
	bone_middle->getOffset(l1);
	bone_lower->getOffset(l2);

	vector3 t1, t2, cv1, cv2;

	bone_upper->getTranslation(t1);
	bone_middle->getTranslation(t2);
	cv1.difference(t1, t2);

	bone_lower->getTranslation(t1);
	cv2.difference(t2, t1);

	quaterN origRot, rot;
	origRot.setSize(4);
	rot.setSize(4);
	for(int i=0; i<4; i++)
	{
		if(i==0)
			skeleton.getBoneByRotJointIndex(index[i]).parent()->getRotation(origRot[i]);
		else
			skeleton.getBoneByRotJointIndex(index[i-1]).getRotation(origRot[i]);
		rot[i]=origRot[i];
	}

	vector3 currToe;
	vector3 currAnkle;

	if(bToeCorrection)
	{
		// Foot skate cleanup by kovar, page(2)
		bone.getTranslation(currToe);
		bone_lower->getTranslation(currAnkle);
		goal=input_goal-currToe+currAnkle;
	}
	else
	{
		goal=input_goal;
	}

	// limb ik solver
	// goal is the goal position of wrist (global)
	// sh is the shoulder position (global)
	// v1 is the vector from the shoulder to the elbow (in neutral pose : offset)
	// v2 is the vector from the elbow to the wrist : offset
	// qq1 is the input and output shoulder joint angle (global)
	// qq2 is the input and output elbow joint angle (global)
	// ii is the importance value
	// axis is the elbow rotation axis
	vector3 sh;
	bone_upper->getTranslation(sh);

	MotionUtil::limbIK( goal, sh, l1, l2, cv1, cv2, rot[1], rot[2], 1.f, bKneeDamping);

	delta_rot.setSize(3);
	for(int i=3; i>=1; i--)
	{
		// Convert to local coordinate
		// hint: rot0*rot1*...*rotn=qqn
		// rot0=qq0
		// rot0*rot1=qq1
		// -> rot1=inv_qq0*qq1

		rot[i].leftMult(rot[i-1].inverse());
		origRot[i].leftMult(origRot[i-1].inverse());

		// calc displacement
		delta_rot[i-1].difference(origRot[i], rot[i]);
		delta_rot[i-1].align(quater(1,0,0,0));
	}
}
m_real MotionUtil::calcIKlength(const MotionLoader& skeleton, int con)
{
	vector3 goal;
	int index_upper, index_middle, index_lower;
	vector3 axis, currHip, currToe, currAnkle;
	setLimb(con, index_upper, index_middle, index_lower, axis, skeleton);

	m_real l1,l2;
	l1=skeleton.getBoneByRotJointIndex(index_middle).length();
	l2=skeleton.getBoneByRotJointIndex(index_lower).length();

	skeleton.getBoneByRotJointIndex(index_lower).getTranslation(currAnkle);
	skeleton.getBoneByRotJointIndex(index_upper).getTranslation(currHip);

	return currAnkle.distance(currHip)/(l1+l2);
}
bool MotionUtil::isIKpossible(const MotionLoader& skeleton, int con, const vector3& input_goal, m_real lengthGoal, m_real distGoal)
{
	vector3 goal;
	int index_upper, index_middle, index_lower;
	vector3 axis, currHip, currToe, currAnkle;
	setLimb(con, index_upper, index_middle, index_lower, axis, skeleton);

	m_real l1,l2;
	l1=skeleton.getBoneByRotJointIndex(index_middle).length();
	l2=skeleton.getBoneByRotJointIndex(index_lower).length();

	if(con==CONSTRAINT_LEFT_HEEL || con==CONSTRAINT_RIGHT_HEEL)
	{
		goal=input_goal;
	}
	else
	{
		// Foot skate cleanup by kovar, page(2)
		dep_GetBoneFromCon(skeleton, con).getTranslation(currToe);
		skeleton.getBoneByRotJointIndex(index_lower).getTranslation(currAnkle);
		goal=input_goal-currToe+currAnkle;
	}

	skeleton.getBoneByRotJointIndex(index_upper).getTranslation(currHip);

	// 다리가 너무 펴질것 같으면 하지 않는다.
	if(goal.distance(currHip)> (l1+l2)*lengthGoal) return false;
	// 너무 멀면, 할수 없다.
	if(currAnkle.distance(goal)> (l2)*distGoal) return false;
	return true;
}
void MotionUtil::setLimb(int con, int& index_up, int& index_mid, int& index_low, vector3& axis, const MotionLoader& skeleton)
{
	if(con==CONSTRAINT_LEFT_TOE || con==CONSTRAINT_LEFT_HEEL)
		con=CONSTRAINT_LEFT_FOOT;

	if(con==CONSTRAINT_RIGHT_TOE || con==CONSTRAINT_RIGHT_HEEL)
		con=CONSTRAINT_RIGHT_FOOT;

	if(con==CONSTRAINT_LEFT_FINGERTIP)
		con=CONSTRAINT_LEFT_HAND;
	if(con==CONSTRAINT_RIGHT_FINGERTIP)
		con=CONSTRAINT_RIGHT_HAND;

	if(con==CONSTRAINT_LEFT_HAND || con==CONSTRAINT_RIGHT_HAND)
		axis.setValue(-1,0,0);
	else
		axis.setValue(1,0,0);

	int voca_up, voca_mid, voca_low;
	if(con == CONSTRAINT_LEFT_FOOT)
	{
		voca_up=MotionLoader::LEFTHIP;
		voca_mid=MotionLoader::LEFTKNEE;
		voca_low=MotionLoader::LEFTANKLE;
	}
	else if(con == CONSTRAINT_RIGHT_FOOT)
	{
		voca_up=MotionLoader::RIGHTHIP;
		voca_mid=MotionLoader::RIGHTKNEE;
		voca_low=MotionLoader::RIGHTANKLE;
	}
	else if(con ==  CONSTRAINT_LEFT_HAND)
	{
		voca_up=MotionLoader::LEFTSHOULDER;
		voca_mid=MotionLoader::LEFTELBOW;
		voca_low=MotionLoader::LEFTWRIST;
	}
	else if(con == CONSTRAINT_RIGHT_HAND)
	{
		voca_up=MotionLoader::RIGHTSHOULDER;
		voca_mid=MotionLoader::RIGHTELBOW;
		voca_low=MotionLoader::RIGHTWRIST;
	}
	else TRACE("error in contraint type!");

	index_up=skeleton.getRotJointIndexByVoca(voca_up);		ASSERT(index_up!= -1);
	index_mid= skeleton.getRotJointIndexByVoca(voca_mid);	ASSERT(index_mid!= -1);
	index_low= skeleton.getRotJointIndexByVoca(voca_low);	ASSERT(index_low!= -1);
}

