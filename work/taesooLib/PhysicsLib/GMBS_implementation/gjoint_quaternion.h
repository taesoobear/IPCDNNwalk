//================================================================================
//         SPHERICAL JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//                                                               taesoobear@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_QUATERNION
#define _GMBS_GEOMETRIC_JOINT_QUATERNION

#include "gjoint.h"
#include "gcoordinate.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointQuaternion
//=============================================================
class GJointQuaternion: public GJoint
{
public:
	GCoordinate coordinates[3];		// built-in coordinates

public:
	GJointQuaternion();
	~GJointQuaternion() {}

public:
	
	// set coordinates[]->(q,dq,ddq) with given set of (R, dot_R, ddot_R) or (R, w, dot_w)
	// R = SO3: {joint left} --> {joint right}
	// dot_R = time derivative of R (3x3 matrix)
	// ddot_R = time derivative of dot_R (3x3 matrix)
	// w = R^T dot_R = angular velocity of {joint right} w.r.t. {joint left} viewed in {joint left}
	// dot_w = R_^T ddot_R - R^T dot_R R^T dot_R = time derivative of w
	void setMotion(const SO3 &R, const RMatrix &dot_R, const RMatrix &ddot_R);
	void setMotion(const SO3 &R, const Vec3 &w, const Vec3 &dot_w);

public:
	bool isConstantScrew() { return false; }

	void update_short();
	void update();

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);
};


#endif

