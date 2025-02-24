#include "../../BaseLib/baselib.h"
#include <list>
#include "gjoint_quaternion.h"
#include "gjoint.h"
#include "gelement.h"
#include "liegroup.h"
#include "../Liegroup.inl"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"


static const double PI = 3.14159265358979;

//=============================================================
//                 GJointQuaternion
//=============================================================
GJointQuaternion::GJointQuaternion()
{
	jointType = GJOINT_QUAT;
	pCoordinates.push_back(&coordinates[0]);
	pCoordinates.push_back(&coordinates[1]);
	pCoordinates.push_back(&coordinates[2]);
	allocate_memory(3);
}

static void setS(RMatrix& S)
{
	// S = [   1,    0,   0
	//         0,    1,   0
	//         0,    0,   1 
	//         0,    0,   0
	//         0,    0,   0
	//         0,    0,   0 ];
	S.SetZero();
	S[0] = 1.0; 
	S[7] = 1.0; 
	S[14] = 1.0;
}
void GJointQuaternion::update_short()
{
	double q0, q1, q2, dq0, dq1, dq2, ddq0, ddq1, ddq2;

	q0 = coordinates[0].q; q1 = coordinates[1].q; q2 = coordinates[2].q;
	dq0 = coordinates[0].dq; dq1 = coordinates[1].dq; dq2 = coordinates[2].dq;
	ddq0 = coordinates[0].ddq; ddq1 = coordinates[1].ddq; ddq2 = coordinates[2].ddq;

	quater q(sqrt(1.0-q0*q0-q1*q1-q2*q2), q0, q1, q2);

	T=toGMBS(q, ::vector3(0,0,0));
	inv_T = SE3(~T.GetRotation());

	setS(S);

	if ( bReversed ) { assert(false);}
}

void GJointQuaternion::update()
{
	double q0, q1, q2, dq0, dq1, dq2, ddq0, ddq1, ddq2;

	q0 = coordinates[0].q; q1 = coordinates[1].q; q2 = coordinates[2].q;
	dq0 = coordinates[0].dq; dq1 = coordinates[1].dq; dq2 = coordinates[2].dq;
	ddq0 = coordinates[0].ddq; ddq1 = coordinates[1].ddq; ddq2 = coordinates[2].ddq;
			
	quater q(sqrt(1.0-q0*q0-q1*q1-q2*q2), q0, q1, q2);

	T=toGMBS(q, vector3(0,0,0));
	inv_T = SE3(~T.GetRotation());
			
	Sdq = se3(dq0, dq1, dq2, 0.0, 0.0, 0.0);
			
	dSdq = se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
			
	Sddq = se3(ddq0, ddq1, ddq2, 0.0, 0.0, 0.0);
			
	DSdqDt = Sddq + dSdq;
			
	setS(S);
			
	dS.SetZero();

	if ( bReversed ) { assert(false);}
}

RMatrix GJointQuaternion::get_DSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,3);
}

RMatrix GJointQuaternion::get_DdSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,3);
}

void GJointQuaternion::setMotion(const SO3 &R_, const RMatrix &dot_R_, const RMatrix &ddot_R_)
{
	RMatrix R(3,3,R_.GetArray());
	RMatrix Rt = ~R;	// transpose of R
	RMatrix W = Rt * dot_R_;
	Vec3 w(0.5*(W[5]-W[7]), 0.5*(W[6]-W[2]), 0.5*(W[1]-W[3]));	// unskew(W)
	RMatrix dW = Rt * ddot_R_ - W * W;
	Vec3 dw(0.5*(dW[5]-dW[7]), 0.5*(dW[6]-dW[2]), 0.5*(dW[1]-dW[3]));	// unskew(dW)

	setMotion(R_, w, dw);
}

void GJointQuaternion::setMotion(const SO3 &R_, const Vec3 &w_, const Vec3 &dot_w_)
{
	double q0, q1, q2, dq0, dq1, dq2, c0, c1, c2, s0, s1, s2;

	quater q=toBaseR(R_);

	// apply q, dq, ddq to coordinates[]->(q,dq,ddq)
	coordinates[0].q = q.x; coordinates[1].q = q.y; coordinates[2].q = q.z;
	coordinates[0].dq = w_.x; coordinates[1].dq = w_.y; coordinates[2].dq = w_.z;
	coordinates[0].ddq = dot_w_.x; coordinates[1].ddq = dot_w_.y; coordinates[2].ddq = dot_w_.z;
}

