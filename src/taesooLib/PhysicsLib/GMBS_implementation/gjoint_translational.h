//================================================================================
//         TRANSLATIONAL(3-DOF) JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_TRANSLATIONAL_
#define _GMBS_GEOMETRIC_JOINT_TRANSLATIONAL_

#include "gjoint.h"
#include "gcoordinate.h"
#include "liegroup.h"


//=============================================================
//                 GJointTranslational
//=============================================================
class GJointTranslational: public GJoint
{
public:
	GCoordinate coordinates[3];				// built-in coordinates

public:
	GJointTranslational();
	~GJointTranslational() {}

public:
	bool isConstantScrew() { return true; }

	void update_short();
	void update();

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);
};


#endif

