//================================================================================
//         FIXED JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_FIXED_
#define _GMBS_GEOMETRIC_JOINT_FIXED_

#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointFixed
//=============================================================
class GJointFixed: public GJoint
{
public:
	GJointFixed();
	~GJointFixed() {}

public: 
	bool isConstantScrew() { return true; }
	
	void update_short() {}
	void update() {}
	
	RMatrix get_DSDq(GCoordinate *pCoordinate_) { return Zeros(6,0); }
	RMatrix get_DdSDq(GCoordinate *pCoordinate_) { return Zeros(6,0); }
};


#endif

