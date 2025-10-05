//================================================================================
//         FREE JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_FREE_
#define _GMBS_GEOMETRIC_JOINT_FREE_

#include "gjoint_composite.h"
#include "gjoint_spherical.h"
#include "gjoint_translational.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GJointFreeC
//=============================================================
class GJointFreeC: public GJointComposite
{
public:
	GJointSpherical spherical_joint;
	GJointTranslational translational_joint;

public:
	GJointFreeC();
	~GJointFreeC() {}
};


class GJointFreeC2: public GJointComposite
{
public:
	GJointTranslational translational_joint;
	GJointSpherical spherical_joint;	

public:
	GJointFreeC2();
	~GJointFreeC2() {}
};


//=============================================================
//                 GJointFree
//=============================================================
class GJointFree: public GJoint
{
public:
	GJointSpherical spherical_joint;
	GJointTranslational translational_joint;

public:
	GJointFree();
	~GJointFree() {}

public:
	bool isConstantScrew() { return false; }

	void update_short();
	void update();

	void _update_short_for_reversed_joint();	// modify T, inv_T, S for reversed joint
	void _update_for_reversed_joint();			// modify T, inv_T, S, dS, Sdq, dSdq, Sddq, DSdqDt for reversed joint

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);
};

#endif

