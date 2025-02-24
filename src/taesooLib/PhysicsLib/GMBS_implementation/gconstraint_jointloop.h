//================================================================================
//         CONSTRAINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_CONSTRAINT_JOINTLOOP_
#define _GMBS_GEOMETRIC_CONSTRAINT_JOINTLOOP_

#include <list>
#include "gconstraint.h"
#include "liegroup.h"


class GJoint;
class GJoint;

enum JOINTLOOP_CONSTRAINT_TYPE
{
	JOINTLOOP_ORIENTATION_POSITION,		// both position and orientation considered
	JOINTLOOP_ORIENTATION_ONLY,			// only orientation considered
	JOINTLOOP_POSITION_ONLY,			// only position considered
};

//=============================================================
//                 GConstraintJointLoop
//=============================================================
class GConstraintJointLoop: public GConstraint
{
public:
	// constraint: f(joints)*M1 = f(joints2)*M2, if pJoints2.size() > 0
	//             f(joints)*M1 = T, if pJoints2.size() = 0
	//             Note: Joints should not be in both pJoints and pJoints2.
	//                   Cut-joint = pJoints[pJoints.size()-1]
	//					 M1 = M2 = T = Eye by default.
	std::list<GJoint *> pJoints, pJoints2;
	SE3 M1, M2;
	SE3 T;

	int num_coord, num_coord2;

	JOINTLOOP_CONSTRAINT_TYPE jointLoopConstraintType;	// JOINTLOOP_ORIENTATION_POSITION(default) | JOINTLOOP_ORIENTATION_ONLY | JOINTLOOP_POSITION_ONLY

	RMatrix jacobian, dotjacobian;	// jacobian = J, dotjacobian = dJdt when jointLoopConstraintType == JOINTLOOP_ORIENTATION_POSITION

public:
	GConstraintJointLoop();
	~GConstraintJointLoop() {}

public:
	bool setJoints(std::list<GJoint *> pjoints);
	bool setJoints(std::list<GJoint *> pjoints, std::list<GJoint *> pjoints2);
	bool setM(const SE3 &M1_, const SE3 &M2_ = SE3()) { M1 = M1_; M2 = M2_; return true; }
	bool setT(const SE3 &T_) { M2 = T_; return true; }

	void setJointLoopConstraintType(JOINTLOOP_CONSTRAINT_TYPE jointLoopConstraintType_);

	SE3 getLoopSE3();	// return f(joints)*M1

	RMatrix get_jacobian() { return jacobian; }
	RMatrix get_dotjacobian() { return dotjacobian; }
	RMatrix get_jacobian(int idx_) { return jacobian.Sub(0, jacobian.RowSize()-1, idx_, idx_); }
	RMatrix get_dotjacobian(int idx_) { return dotjacobian.Sub(0, dotjacobian.RowSize()-1, idx_, idx_); }

public:
	bool update_C();		// In addition to C, pJoints[]->(T,inv_T,S) will be updated in this function.
	bool update_J();		// pJoints[]->(inv_T, S) need to be updated before calling this function.
	bool update_dJdt();		// update_J() need to be called before calling this function.

	std::string getInfoStr();
};


#endif

