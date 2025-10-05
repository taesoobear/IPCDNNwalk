#include <algorithm>
#include "gjoint_free.h"
#include "gjoint.h"
#include "gjoint_spherical.h"
#include "gjoint_translational.h"
#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"



//=============================================================
//                 GJointFreeC
//=============================================================
GJointFreeC::GJointFreeC()
{
	compose(&spherical_joint, &translational_joint);
}

GJointFreeC2::GJointFreeC2()
{
	compose(&translational_joint, &spherical_joint);
}

//=============================================================
//                 GJointFree
//=============================================================
GJointFree::GJointFree()
{
	jointType = GJOINT_FREE;
	pCoordinates.push_back(&(spherical_joint.coordinates[0]));
	pCoordinates.push_back(&(spherical_joint.coordinates[1]));
	pCoordinates.push_back(&(spherical_joint.coordinates[2]));
	pCoordinates.push_back(&(translational_joint.coordinates[0]));
	pCoordinates.push_back(&(translational_joint.coordinates[1]));
	pCoordinates.push_back(&(translational_joint.coordinates[2]));
	allocate_memory(6);
}

void GJointFree::update_short()
{
	spherical_joint.update_short();
	translational_joint.update_short();

	Vec3 S1dq1w = spherical_joint.Sdq.GetW();
	Vec3 dS1dq1w = spherical_joint.dSdq.GetW();
	Vec3 S1ddq1w = spherical_joint.Sddq.GetW();
	RMatrix S1w = spherical_joint.S.Sub(0,2,0,2);
	RMatrix dS1w = spherical_joint.dS.Sub(0,2,0,2);
	Vec3 p = translational_joint.T.GetPosition();
	Vec3 S2dq2v = translational_joint.Sdq.GetV();
	Vec3 S2ddq2v = translational_joint.Sddq.GetV();

	// T1 = spherical_joint.T = SE3(R, 0)
	// T2 = translational_joint.T = SE3(eye, p)
	// T = T1*T2 = SE3(R, R*p)
	// inv_T = SE3(~R, -p)
	T = SE3(spherical_joint.T.GetRotation(), spherical_joint.T.GetRotation() * translational_joint.T.GetPosition());
	inv_T = SE3(spherical_joint.inv_T.GetRotation(), translational_joint.inv_T.GetPosition());

	S.Push(0, 0, S1w);
	S.Push(3, 0, -Cross(p, S1w));
	S.Push(0, 3, translational_joint.S);

	if ( bReversed ) { _update_short_for_reversed_joint(); }
}

void GJointFree::update()
{
	spherical_joint.update();
	translational_joint.update();

	Vec3 S1dq1w = spherical_joint.Sdq.GetW();
	Vec3 dS1dq1w = spherical_joint.dSdq.GetW();
	Vec3 S1ddq1w = spherical_joint.Sddq.GetW();
	RMatrix S1w = spherical_joint.S.Sub(0,2,0,2);
	RMatrix dS1w = spherical_joint.dS.Sub(0,2,0,2);
	Vec3 p = translational_joint.T.GetPosition();
	Vec3 S2dq2v = translational_joint.Sdq.GetV();
	Vec3 S2ddq2v = translational_joint.Sddq.GetV();

	// T1 = spherical_joint.T = SE3(R, 0)
	// T2 = translational_joint.T = SE3(eye, p)
	// T = T1*T2 = SE3(R, R*p)
	// inv_T = SE3(~R, -p)
	T = SE3(spherical_joint.T.GetRotation(), spherical_joint.T.GetRotation() * translational_joint.T.GetPosition());
	inv_T = SE3(spherical_joint.inv_T.GetRotation(), translational_joint.inv_T.GetPosition());

	Sdq = se3(S1dq1w, Cross(S1dq1w, p) + S2dq2v);
	dSdq = se3(dS1dq1w, Cross(dS1dq1w, p) + Cross(S1dq1w, S2dq2v));
	Sddq = se3(S1ddq1w, Cross(S1ddq1w, p) + S2ddq2v);
	DSdqDt = Sddq + dSdq;

	S.Push(0, 0, S1w);
	S.Push(3, 0, -Cross(p, S1w));
	S.Push(0, 3, translational_joint.S);

	dS.Push(0, 0, dS1w);
	dS.Push(3, 0, -(Cross(S2dq2v, S1w) + Cross(p, dS1w)));
	dS.Push(0, 3, translational_joint.dS);

	if ( bReversed ) { _update_for_reversed_joint(); }
}

RMatrix GJointFree::get_DSDq(GCoordinate *pCoordinate_)
{
	if ( find(pCoordinates.begin(), pCoordinates.end(), pCoordinate_) == pCoordinates.end() ) return Zeros(6, getDOF());

	Vec3 p = translational_joint.T.GetPosition();
	Vec3 S2iv = translational_joint.get_S(pCoordinate_).GetV();
	RMatrix DSDq, S1w, DS1Dqw;
	S1w = spherical_joint.S.Sub(0,2,0,2);
	DS1Dqw = spherical_joint.get_DSDq(pCoordinate_).Sub(0,2,0,2);

	DSDq.SetZero(6, getDOF());
	DSDq.Push(0, 0, DS1Dqw);
	DSDq.Push(3, 0, -(Cross(S2iv, S1w) + Cross(p, DS1Dqw)));
	DSDq.Push(0, 3, translational_joint.get_DSDq(pCoordinate_));

	if ( bReversed ) {
		DSDq = -Ad(inv_T, DSDq);
		DSDq -= ad(get_S(pCoordinate_), S);
	}

	return DSDq;
}

RMatrix GJointFree::get_DdSDq(GCoordinate *pCoordinate_)
{
	if ( find(pCoordinates.begin(), pCoordinates.end(), pCoordinate_) == pCoordinates.end() ) return Zeros(6, getDOF());

	Vec3 p = translational_joint.T.GetPosition();
	Vec3 DS2Dqdq2v = convert_to_Vec3(translational_joint.get_DSDq(pCoordinate_).Sub(3,5,0,2) * translational_joint.get_dq());
	Vec3 S2dq2v = translational_joint.Sdq.GetV();
	Vec3 S2iv = translational_joint.get_S(pCoordinate_).GetV();
	RMatrix DdSDq, S1w, dS1w, DS1Dqw, DdS1Dqw;
	S1w = spherical_joint.S.Sub(0,2,0,2);
	dS1w = spherical_joint.dS.Sub(0,2,0,2);
	DS1Dqw = spherical_joint.get_DSDq(pCoordinate_).Sub(0,2,0,2);
	DdS1Dqw = spherical_joint.get_DdSDq(pCoordinate_).Sub(0,2,0,2);

	DdSDq.SetZero(6, getDOF());
	DdSDq.Push(0, 0, DdS1Dqw);
	DdSDq.Push(3, 0, -(Cross(DS2Dqdq2v, S1w) + Cross(S2dq2v, DS1Dqw) + Cross(S2iv, dS1w) + Cross(p, DdS1Dqw)));
	DdSDq.Push(0, 3, translational_joint.get_DdSDq(pCoordinate_));

	if ( bReversed ) {
		RMatrix DSDq = get_DSDq(pCoordinate_);
		DdSDq = -Ad(inv_T, DdSDq);
		DdSDq -= ad(get_S(pCoordinate_), dS + ad(Sdq, S));
		DdSDq -= ad(DSDq*get_dq(), S) - ad(Sdq, DSDq);
	}

	return DdSDq;
}

void GJointFree::_update_short_for_reversed_joint()
{
	SE3 T_tmp = T;
	T = inv_T;
	inv_T = T_tmp;

	S = -Ad(inv_T, S);
}

void GJointFree::_update_for_reversed_joint()
{
	SE3 T_tmp = T;
	T = inv_T;
	inv_T = T_tmp;

	Sdq = -Ad(inv_T, Sdq);
	dSdq = -Ad(inv_T, dSdq);
	Sddq = -Ad(inv_T, Sddq);
	DSdqDt = Sddq + dSdq;
	S = -Ad(inv_T, S);
	dS = -Ad(inv_T, dS);
	dS -= ad(Sdq, S);
}

