#include <list>
#include "gjoint_prismatic.h"
#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"



//=============================================================
//                 GJointPrismatic
//=============================================================
GJointPrismatic::GJointPrismatic()
{
	jointType = GJOINT_PRISMATIC;
	pCoordinates.push_back(&coordinate);		// set coordinate pointer
	axis = Vec3(0,0,1);							// default axis
	allocate_memory(1);
}

void GJointPrismatic::setAxis(double x_, double y_, double z_)
{
	axis[0] = x_;
	axis[1] = y_;
	axis[2] = z_;
}

void GJointPrismatic::update_short()
{
	if ( bReversed ) {
		T = SE3(-axis*coordinate.q);
		inv_T = SE3(-T.GetPosition());
		S[3] = -axis[0]; S[4] = -axis[1]; S[5] = -axis[2];
	} else {
		T = SE3(axis*coordinate.q);
		inv_T = SE3(-T.GetPosition());
		S[3] = axis[0]; S[4] = axis[1]; S[5] = axis[2];
	}
}

void GJointPrismatic::update()
{
	//dS, dSdq are still zeros.

	if ( bReversed ) {
		T = SE3(-axis*coordinate.q);
		inv_T = SE3(-T.GetPosition());
		Sdq = se3(Vec3(0,0,0), -axis*coordinate.dq);
		Sddq = se3(Vec3(0,0,0), -axis*coordinate.ddq);
		DSdqDt = Sddq;
		S[3] = -axis[0]; S[4] = -axis[1]; S[5] = -axis[2];
	} else {
		T = SE3(axis*coordinate.q);
		inv_T = SE3(-T.GetPosition());
		Sdq = se3(Vec3(0,0,0), axis*coordinate.dq);
		Sddq = se3(Vec3(0,0,0), axis*coordinate.ddq);
		DSdqDt = Sddq;
		S[3] = axis[0]; S[4] = axis[1]; S[5] = axis[2];
	}
}

RMatrix GJointPrismatic::get_DSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,1);
}

RMatrix GJointPrismatic::get_DdSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,1);
}

