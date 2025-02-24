#include <list>
#include "gjoint_free_2d.h"
#include "gjoint.h"
#include "gjoint_revolute.h"
#include "gjoint_translational_xy.h"
#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"



//=============================================================
//                 GJointFree2D
//=============================================================
GJointFree2D::GJointFree2D()
{
	pCoordinates.push_back(&(rotational_z_joint.coordinate));
	pCoordinates.push_back(&(translational_xy_joint.coordinates[0]));
	pCoordinates.push_back(&(translational_xy_joint.coordinates[1]));
}

SE3 GJointFree2D::get_T()
{
	return rotational_z_joint.get_T() * translational_xy_joint.get_T();
}

se3 GJointFree2D::get_Sdq()
{
	return Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_Sdq()) + translational_xy_joint.get_Sdq();
}

se3 GJointFree2D::get_DSdqDt()
{
	return -ad(translational_xy_joint.get_Sdq(), Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_Sdq())) 
		+ Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_DSdqDt()) 
		+ translational_xy_joint.get_DSdqDt();
}

se3 GJointFree2D::get_dSdq()
{
	SE3 inv_T_translational = Inv(translational_xy_joint.get_T());

	return -ad(translational_xy_joint.get_Sdq(), Ad(inv_T_translational, rotational_z_joint.get_Sdq()))
		+ Ad(inv_T_translational, rotational_z_joint.get_dSdq())
		+ translational_xy_joint.get_dSdq();
}

se3 GJointFree2D::get_Sddq()
{
	return Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_Sddq()) + translational_xy_joint.get_Sddq();
}

RMatrix GJointFree2D::get_S()
{
	RMatrix S, S1, S2;
	S1 = Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_S());
	S2 = translational_xy_joint.get_S();
	S.SetZero(6,3);
	S.Push(0, 0, S1);
	S.Push(0, 1, S2);
	return S;
}

RMatrix GJointFree2D::get_dS()
{
	RMatrix dS, dS1, dS2;
	dS1 = - ad(translational_xy_joint.get_Sdq(), Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_S())) 
			+ Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_dS());
	dS2 = translational_xy_joint.get_dS();
	dS.SetZero(6,3);
	dS.Push(0, 0, dS1);
	dS.Push(0, 1, dS2);
	return dS;
}

se3 GJointFree2D::get_S(GCoordinate *pCoordinate_)
{
	if ( pCoordinate_ == &rotational_z_joint.coordinate ) {
		return Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_S(pCoordinate_));
	} else if ( pCoordinate_ == &translational_xy_joint.coordinates[0] || pCoordinate_ == &translational_xy_joint.coordinates[1] ) {
		return translational_xy_joint.get_S(pCoordinate_);
	} else {
		return se3(0,0,0,0,0,0);
	}
}

se3 GJointFree2D::get_dS(GCoordinate *pCoordinate_)
{
	if ( pCoordinate_ == &rotational_z_joint.coordinate ) {
		SE3 invTt = Inv(translational_xy_joint.get_T());
		return -ad(translational_xy_joint.get_Sdq(), Ad(invTt, rotational_z_joint.get_S(pCoordinate_))) + Ad(invTt, rotational_z_joint.get_dS(pCoordinate_));
	} else if ( pCoordinate_ == &translational_xy_joint.coordinates[0] || pCoordinate_ == &translational_xy_joint.coordinates[1] ) {
		return translational_xy_joint.get_dS(pCoordinate_);
	} else {
		return se3(0,0,0,0,0,0);
	}
}

RMatrix GJointFree2D::get_DSDq(GCoordinate *pCoordinate_)
{
	RMatrix DSDq, DSDq1, DSDq2;
	DSDq1 = -ad(translational_xy_joint.get_S(pCoordinate_), Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_S()))
			+ Ad(Inv(translational_xy_joint.get_T()), rotational_z_joint.get_DSDq(pCoordinate_));
	DSDq2 = translational_xy_joint.get_DSDq(pCoordinate_);
	DSDq.SetZero(6,3);
	DSDq.Push(0, 0, DSDq1);
	DSDq.Push(0, 1, DSDq2);
	return DSDq;
}

RMatrix GJointFree2D::get_DdSDq(GCoordinate *pCoordinate_)
{
	RMatrix DdSDq, DdSDq1, DdSDq2, Ss;
	SE3 invTt;
	invTt = Inv(translational_xy_joint.get_T());
	Ss = rotational_z_joint.get_S();
	DdSDq1 = -ad(translational_xy_joint.get_DSDq(pCoordinate_) * translational_xy_joint.get_dq(), Ad(invTt, Ss))
			+ ad(translational_xy_joint.get_Sdq(), ad(translational_xy_joint.get_S(pCoordinate_), Ad(invTt, Ss)))
			- ad(translational_xy_joint.get_Sdq(), Ad(invTt, rotational_z_joint.get_DSDq(pCoordinate_)))
			- ad(translational_xy_joint.get_S(pCoordinate_), Ad(invTt, rotational_z_joint.get_dS()))
			+ Ad(invTt, rotational_z_joint.get_DdSDq(pCoordinate_));
	DdSDq2 = translational_xy_joint.get_DdSDq(pCoordinate_);
	DdSDq.SetZero(6,3);
	DdSDq.Push(0, 0, DdSDq1);
	DdSDq.Push(0, 1, DdSDq2);
	return DdSDq;
}

