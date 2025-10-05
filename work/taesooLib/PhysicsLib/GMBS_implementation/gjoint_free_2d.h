//================================================================================
//         2D FREE JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_FREE_2D_
#define _GMBS_GEOMETRIC_JOINT_FREE_2D_

#include "gjoint.h"
#include "gjoint_revolute.h"
#include "gjoint_translational_xy.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointFree2D
//=============================================================
class GJointFree2D: public GJoint
{
public:
	GJointRevolute rotational_z_joint;
	GJointTranslationalXY translational_xy_joint;

public:
	GJointFree2D();
	~GJointFree2D() {}

public: // virtual functions
	bool reverse() { return false; }		// Not supported yet!

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

