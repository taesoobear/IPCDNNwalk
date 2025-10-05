//================================================================================
//         PRISMATIC JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_PRISMATIC_
#define _GMBS_GEOMETRIC_JOINT_PRISMATIC_

#include "gjoint.h"
#include "gcoordinate.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointPrismatic
//=============================================================
class GJointPrismatic: public GJoint
{
public:
	GCoordinate coordinate;						// built-in coordinate

	Vec3 axis;									// prismatic axis w.r.t. {joint left}

public:
	GJointPrismatic();
	~GJointPrismatic() {}

public:
	void setAxis(double x_, double y_, double z_);	// set axis
	void setAxis(Vec3 axis_) { axis = axis_; }
	Vec3 getAxis() { return axis; }

public:
	bool isConstantScrew() { return true; }

	void update_short();
	void update();

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);
};


#endif

