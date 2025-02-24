//================================================================================
//         UNIVERSAL JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_UNIVERSAL_
#define _GMBS_GEOMETRIC_JOINT_UNIVERSAL_

#include "gjoint.h"
#include "gcoordinate.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointUniversal
//=============================================================
//
//  T = SE3(R(x,q0) * R(y,q1), Vec3(0,0,0)) 
//  where q0 = coordinates[0].q and q1 = coordinates[1].q 
//

class GJointUniversal: public GJoint
{
public:
	GCoordinate coordinates[2];	

public:
	GJointUniversal();
	~GJointUniversal() {}

public:
	void getAxis(Vec3 &axis1_, Vec3 &axis2_);	// return the x, y axes w.r.t. {joint left}
												// i.e., axis1_ = (1,0,0), axis2_ = R(x,q0)*Vec3(0,1,0)

public:
	bool isConstantScrew() { return false; }

	void update_short();
	void update();

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);
};


#endif

