//================================================================================
//         PLANAR JOINT (2-DOF TRANSLATIONAL JOINT ON XY PLANE) FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_PLANAR_
#define _GMBS_GEOMETRIC_JOINT_PLANAR_

#include "gjoint.h"
#include "gcoordinate.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointPlanar
//=============================================================
//
//  T = SE3(SO3(), Vec3(q0,q1,0)) 
//  where q0 = coordinates[0].q and q1 = coordinates[1].q 
//

class GJointPlanar: public GJoint
{
public:
	GCoordinate coordinates[2];		// built-in coordinates

public:
	GJointPlanar();
	~GJointPlanar() {}

public:
	bool isConstantScrew() { return true; }

	void update_short();
	void update();

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);
};


#endif

