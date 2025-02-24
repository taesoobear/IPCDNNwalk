#include <list>
#include <algorithm>
#include "gjoint_universal.h"
#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"


//=============================================================
//                 GJointUniversal
//=============================================================
GJointUniversal::GJointUniversal()
{
	jointType = GJOINT_UNIVERSAL;
	pCoordinates.push_back(&coordinates[0]);
	pCoordinates.push_back(&coordinates[1]);
	allocate_memory(2);
}

void GJointUniversal::getAxis(Vec3 &axis1_, Vec3 &axis2_)
{
	double q0, c0, s0;
	q0 = coordinates[0].q;
	c0 = cos(q0);
	s0 = sin(q0);

	axis1_ = Vec3(1,0,0);
	axis2_ = SO3(1,0,0,0,c0,s0,0,-s0,c0) * Vec3(0,1,0);
}

void GJointUniversal::update_short()
{
	double q0, q1, c0, c1, s0, s1;
	q0 = coordinates[0].q; q1 = coordinates[1].q;
	c0 = cos(q0); c1 = cos(q1);
	s0 = sin(q0); s1 = sin(q1);

	if ( bReversed ) {
		T = SE3(c1, 0, s1, s0*s1, c0, -s0*c1, -c0*s1, s0, c0*c1, 0, 0, 0);
		inv_T = SE3(~T.GetRotation());
		S[0] = -1; S[7] = -c0; S[8] = -s0;		

	} else {
		T = SE3(c1, s0*s1, -c0*s1, 0, c0, s0, s1, -s0*c1, c0*c1, 0, 0, 0);
		inv_T = SE3(~T.GetRotation());
		S[0] = c1; S[2] =  s1; S[7] = 1;
	}
}

void GJointUniversal::update()
{
	double q0, q1, dq0, dq1, ddq0, ddq1, c0, c1, s0, s1;
	q0 = coordinates[0].q; q1 = coordinates[1].q;
	dq0 = coordinates[0].dq; dq1 = coordinates[1].dq;
	ddq0 = coordinates[0].ddq; ddq1 = coordinates[1].ddq;
	c0 = cos(q0); c1 = cos(q1);
	s0 = sin(q0); s1 = sin(q1);

	if ( bReversed ) {
		T = SE3(c1, 0, s1, s0*s1, c0, -s0*c1, -c0*s1, s0, c0*c1, 0, 0, 0);
		inv_T = SE3(~T.GetRotation());
		Sdq = se3(-dq0, -c0*dq1, -s0*dq1, 0, 0, 0);
		dSdq = se3(0, s0*dq0*dq1, -c0*dq0*dq1, 0, 0, 0);
		Sddq = se3(-ddq0, -c0*ddq1, -s0*ddq1, 0, 0, 0);
		DSdqDt = Sddq + dSdq;
		S[0] = -1; S[7] = -c0; S[8] = -s0;
		dS[7] = s0*dq0; dS[8] = -c0*dq0;

	} else {
		T = SE3(c1, s0*s1, -c0*s1, 0, c0, s0, s1, -s0*c1, c0*c1, 0, 0, 0);
		inv_T = SE3(~T.GetRotation());
		Sdq = se3(c1*dq0, dq1, s1*dq0, 0, 0, 0);
		dSdq = se3(-s1*dq0*dq1, 0, c1*dq0*dq1, 0, 0, 0);
		Sddq = se3(c1*ddq0, ddq1, s1*ddq0, 0, 0, 0);
		DSdqDt = Sddq + dSdq;
		S[0] = c1; S[2] =  s1; S[7] = 1;
		dS[0] = -s1*dq1; dS[2] = c1*dq1; 
	}
}

RMatrix GJointUniversal::get_DSDq(GCoordinate *pCoordinate_)
{
	RMatrix DSDq;
	double c0, c1, s0, s1;
	int idx;

	DSDq.SetZero(6,2);

	if ( pCoordinate_ == &coordinates[0] ) {
		idx = 0;
	} else if ( pCoordinate_ == &coordinates[1] ) {
		idx = 1;
	} else {
		return DSDq;
	}

	if ( bReversed ) {
		switch ( idx ) {
			case 0:
				c0 = cos(coordinates[0].q);
				s0 = sin(coordinates[0].q);
				DSDq[7] = s0;
				DSDq[8] = -c0;
				break;
			case 1:
				break;
		}

	} else {
		switch ( idx ) {
			case 0:
				break;
			case 1:
				c1 = cos(coordinates[1].q);
				s1 = sin(coordinates[1].q);
				DSDq[0] = -s1; DSDq[2] = c1;
				break;
		}
	}

	return DSDq;
}

RMatrix GJointUniversal::get_DdSDq(GCoordinate *pCoordinate_)
{
	RMatrix DdSDq;
	double dq0, dq1, c0, c1, s0, s1;
	int idx;

	DdSDq.SetZero(6,2);

	if ( pCoordinate_ == &coordinates[0] ) {
		idx = 0;
	} else if ( pCoordinate_ == &coordinates[1] ) {
		idx = 1;
	} else {
		return DdSDq;
	}

	if ( bReversed ) {
		switch ( idx ) {
			case 0:
				dq0 = coordinates[0].dq;
				c0 = cos(coordinates[0].q);
				s0 = sin(coordinates[0].q);
				DdSDq[7] = c0*dq0; DdSDq[8] = s0*dq0;
				break;
			case 1:
				break;
		}
	} else {
		switch ( idx ) {
			case 0:
				break;
			case 1:
				dq1 = coordinates[1].dq;
				c1 = cos(coordinates[1].q);
				s1 = sin(coordinates[1].q);
				DdSDq[0] = -c1*dq1; DdSDq[2] = -s1*dq1;
				break;
		}
	}

	return DdSDq;
}