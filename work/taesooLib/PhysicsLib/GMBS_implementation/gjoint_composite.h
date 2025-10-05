//================================================================================
//         COMPOSITE OF TWO JOINTS FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_COMPOSITE_
#define _GMBS_GEOMETRIC_JOINT_COMPOSITE_

#include "gjoint.h"
#include "gcoordinate.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointComposite
//=============================================================
//
// T = pJoint1->T * pJoint2->T
//
// ** Computational speed of GJointComposite may be slower than that of an optimized single joint implementation.
// For e.g., a spherical joint can be implemented by GJointComposite(GJointComposite(GJointRevolute, GJointRevolute), GJointRevolute),
// and the calculation speed for update() is more than two times slower than that of GJointSpherical.
//
class GJointComposite: public GJoint
{
public:
	GJoint *pJoint1, *pJoint2;
	
public:
	GJointComposite();
	GJointComposite(GJoint *pjoint1_, GJoint *pjoint2_);
	~GJointComposite() {}

public:
	bool compose(GJoint *pjoint1_, GJoint *pjoint2_);	// compose two joints to make a composite joint
	
public:
	bool isConstantScrew() { return false; }

	void update_short();
	void update();

	void _update_short_for_reversed_joint();			// modify T, inv_T, S for reversed joint
	void _update_for_reversed_joint();					// modify T, inv_T, S, dS, Sdq, dSdq, Sddq, DSdqDt for reversed joint

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);
};


#endif

