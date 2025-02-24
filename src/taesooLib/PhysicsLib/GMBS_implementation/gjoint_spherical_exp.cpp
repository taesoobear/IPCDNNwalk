#include <list>
#include "gjoint_spherical_exp.h"
#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"



const double PI = 3.14159265358979;

//=============================================================
//                 GJointSphericalExp
//=============================================================
GJointSphericalExp::GJointSphericalExp()
{
	pCoordinates.push_back(&coordinates[0]);
	pCoordinates.push_back(&coordinates[1]);
	pCoordinates.push_back(&coordinates[2]);
}

SE3 GJointSphericalExp::get_T()
{
	Vec3 q(coordinates[0].q, coordinates[1].q, coordinates[2].q);

	return SE3(Exp(q), Vec3(0,0,0));
}

se3 GJointSphericalExp::get_Sdq()
{
}

se3 GJointSphericalExp::get_dSdq()
{
}

se3 GJointSphericalExp::get_Sddq()
{
}

se3 GJointSphericalExp::get_DSdqDt()
{
}

RMatrix GJointSphericalExp::get_S()
{
	Vec3 q(coordinates[0].q, coordinates[1].q, coordinates[2].q);
	double a2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2];

	if ( a < 1E-12 ) return Eye(3,3);

	double a = sqrt(a2);
	double a3 = a2*a;
	RMatrix q_qt = matrix_a_at(q);
	RMatrix skew_q = matrix_skew(q);
	RMatrix skew_q_skew_q = matrix_skew_skew(q);
    
	return (1./a2)*q_qt - ((1.-cos(a))/a2)*skew_q - (sin(a)/a3)*skew_q_skew_q;
}

RMatrix GJointSphericalExp::get_dS()
{
}

se3 GJointSphericalExp::get_S(GCoordinate *pCoordinate_)
{
}

se3 GJointSphericalExp::get_dS(GCoordinate *pCoordinate_)
{
}

RMatrix GJointSphericalExp::get_DSDq(GCoordinate *pCoordinate_)
{
}

RMatrix GJointSphericalExp::get_DdSDq(GCoordinate *pCoordinate_)
{
}

