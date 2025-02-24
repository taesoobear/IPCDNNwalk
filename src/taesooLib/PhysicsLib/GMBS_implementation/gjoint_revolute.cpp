#include <list>
#include "gjoint_revolute.h"
#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"



//=============================================================
//                 GJointRevolute
//=============================================================
GJointRevolute::GJointRevolute()
{
	jointType = GJOINT_REVOLUTE;
	axis = Vec3(0,0,1);							// default axis
	pCoordinates.push_back(&coordinate);		// set coordinate pointer
	allocate_memory(1);
}

void GJointRevolute::setAxis(double x_, double y_, double z_)
{
	axis[0] = x_;
	axis[1] = y_;
	axis[2] = z_;
}

void GJointRevolute::update_short()
{
	if ( bReversed ) {
		T = SE3(Exp(-axis*coordinate.q), Vec3(0,0,0));
		inv_T = SE3(~T.GetRotation());
		S[0] = -axis[0]; S[1] = -axis[1]; S[2] = -axis[2];
	} else {
		T = SE3(Exp(axis*coordinate.q), Vec3(0,0,0));
		inv_T = SE3(~T.GetRotation());
		S[0] = axis[0]; S[1] = axis[1]; S[2] = axis[2];
	}
}

void GJointRevolute::update()
{
	//dS, dSdq are still zeros.

	if ( bReversed ) {
		T = SE3(Exp(-axis*coordinate.q), Vec3(0,0,0));
		inv_T = SE3(~T.GetRotation());
		Sdq = se3(-axis*coordinate.dq, Vec3(0,0,0));
		Sddq = se3(-axis*coordinate.ddq, Vec3(0,0,0));
		DSdqDt = Sddq;
		S[0] = -axis[0]; S[1] = -axis[1]; S[2] = -axis[2];
	} else {
		T = SE3(Exp(axis*coordinate.q), Vec3(0,0,0));
		inv_T = SE3(~T.GetRotation());
		Sdq = se3(axis*coordinate.dq, Vec3(0,0,0));
		Sddq = se3(axis*coordinate.ddq, Vec3(0,0,0));
		DSdqDt = Sddq;
		S[0] = axis[0]; S[1] = axis[1]; S[2] = axis[2];
	}
}

RMatrix GJointRevolute::get_DSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,1);
}

RMatrix GJointRevolute::get_DdSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,1);
}

