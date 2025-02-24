#include <list>
#include "gjoint_translational.h"
#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointTranslational
//=============================================================
GJointTranslational::GJointTranslational()
{
	jointType = GJOINT_TRANSLATIONAL;
	pCoordinates.push_back(&coordinates[0]);
	pCoordinates.push_back(&coordinates[1]);
	pCoordinates.push_back(&coordinates[2]);
	allocate_memory(3);
}

void GJointTranslational::update_short()
{
	if ( bReversed ) {
		T = SE3(1,0,0,0,1,0,0,0,1, -coordinates[0].q, -coordinates[1].q, -coordinates[2].q);
		inv_T = SE3(1,0,0,0,1,0,0,0,1, coordinates[0].q, coordinates[1].q, coordinates[2].q);
		S[3] = S[10] = S[17] = -1.0;
	} else {
		T = SE3(1,0,0,0,1,0,0,0,1, coordinates[0].q, coordinates[1].q, coordinates[2].q);
		inv_T = SE3(1,0,0,0,1,0,0,0,1, -coordinates[0].q, -coordinates[1].q, -coordinates[2].q);
		S[3] = S[10] = S[17] = 1.0;
	}
}

void GJointTranslational::update()
{
	//dS, dSdq are still zeros.

	if ( bReversed ) {
		T = SE3(1,0,0,0,1,0,0,0,1, -coordinates[0].q, -coordinates[1].q, -coordinates[2].q);
		inv_T = SE3(1,0,0,0,1,0,0,0,1, coordinates[0].q, coordinates[1].q, coordinates[2].q);
		
		Sdq = se3(0, 0, 0, -coordinates[0].dq, -coordinates[1].dq, -coordinates[2].dq);
		Sddq = se3(0, 0, 0, -coordinates[0].ddq, -coordinates[1].ddq, -coordinates[2].ddq);
		
		DSdqDt = Sddq;
		S[3] = S[10] = S[17] = -1.0;
	} else {
		T = SE3(1,0,0,0,1,0,0,0,1, coordinates[0].q, coordinates[1].q, coordinates[2].q);
		inv_T = SE3(1,0,0,0,1,0,0,0,1, -coordinates[0].q, -coordinates[1].q, -coordinates[2].q);

		Sdq = se3(0, 0, 0, coordinates[0].dq, coordinates[1].dq, coordinates[2].dq);
		Sddq = se3(0, 0, 0, coordinates[0].ddq, coordinates[1].ddq, coordinates[2].ddq);
		
		DSdqDt = Sddq;
		S[3] = S[10] = S[17] = 1.0;
	}
}

RMatrix GJointTranslational::get_DSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,3);
}

RMatrix GJointTranslational::get_DdSDq(GCoordinate *pCoordinate_)
{
	return Zeros(6,3);
}

