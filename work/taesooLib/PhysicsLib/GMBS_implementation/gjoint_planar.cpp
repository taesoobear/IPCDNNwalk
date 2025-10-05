#include <list>
#include "gjoint_planar.h"
#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"



//=============================================================
//                 GJointPlanar
//=============================================================
GJointPlanar::GJointPlanar()
{
	jointType = GJOINT_PLANAR;
	pCoordinates.push_back(&coordinates[0]);
	pCoordinates.push_back(&coordinates[1]);
	allocate_memory(2);
}

void GJointPlanar::update_short()
{
	if ( bReversed ) {
		T = SE3(1,0,0,0,1,0,0,0,1, -coordinates[0].q, -coordinates[1].q, 0);
		inv_T = SE3(1,0,0,0,1,0,0,0,1, coordinates[0].q, coordinates[1].q, 0);
		S[3] = S[10] = -1.0;
	} else {
		T = SE3(1,0,0,0,1,0,0,0,1, coordinates[0].q, coordinates[1].q, 0);
		inv_T = SE3(1,0,0,0,1,0,0,0,1, -coordinates[0].q, -coordinates[1].q, 0);
		S[3] = S[10] = 1.0;
	}
}

void GJointPlanar::update()
{
	//dS, dSdq are still zeros.

	if ( bReversed ) {
		T = SE3(1,0,0,0,1,0,0,0,1, -coordinates[0].q, -coordinates[1].q, 0);
		inv_T = SE3(1,0,0,0,1,0,0,0,1, coordinates[0].q, coordinates[1].q, 0);
		Sdq = se3(0, 0, 0, -coordinates[0].dq, -coordinates[1].dq, 0);
		Sddq = se3(0, 0, 0, -coordinates[0].ddq, -coordinates[1].ddq, 0);
		DSdqDt = Sddq;
		S[3] = S[10] = -1.0;
	} else {
		T = SE3(1,0,0,0,1,0,0,0,1, coordinates[0].q, coordinates[1].q, 0);
		inv_T = SE3(1,0,0,0,1,0,0,0,1, -coordinates[0].q, -coordinates[1].q, 0);
		Sdq = se3(0, 0, 0, coordinates[0].dq, coordinates[1].dq, 0);
		Sddq = se3(0, 0, 0, coordinates[0].ddq, coordinates[1].ddq, 0);
		DSdqDt = Sddq;
		S[3] = S[10] = 1.0;
	}
}

RMatrix GJointPlanar::get_DSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,2);
}

RMatrix GJointPlanar::get_DdSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,2);
}
