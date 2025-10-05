//================================================================================
//         SPHERICAL JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_SPHERICAL_EXPONENTIAL_COORDINATES_
#define _GMBS_GEOMETRIC_JOINT_SPHERICAL_EXPONENTIAL_COORDINATES_

#include "gjoint.h"
#include "gcoordinate.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointSphericalExp
//=============================================================
class GJointSphericalExp: public GJoint
{
public:
	GCoordinate coordinates[3];						// built-in coordinates

public:
	GJointSphericalExp();
	~GJointSphericalExp() {}

public: // virtual functions
	SE3 get_T();				
	se3 get_Sdq();		
	se3 get_DSdqDt();	
	se3 get_dSdq();
	se3 get_Sddq();

	RMatrix get_S();			
	RMatrix get_dS();

	se3 get_S(GCoordinate *pCoordinate_);
	se3 get_dS(GCoordinate *pCoordinate_);

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);

	bool isConstantScrew() { return false; }
};


#endif

