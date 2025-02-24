//================================================================================
//         GEOMETRIC MULTIBODY SYSTEM WITH INVERSE KINEMATICS
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_MULTI_BODY_SYSTEM_WITH_INVERSE_KINEMATICS_
#define _GMBS_MULTI_BODY_SYSTEM_WITH_INVERSE_KINEMATICS_

#include <vector>
#include "gsystem.h"
#include "gbody_rigid.h"
#include "liegroup.h"


//=============================================================
//                 GSystemIK
//=============================================================
class GSystemIK: public GSystem
{
public:
	GSystemIK() {}
	~GSystemIK() {}

public:

	// build IK constraints J*dq = V where dq = pCoordinates[]->dq
	//	- IK constraints: V_target[i] = pbodies[i]'s (angular velocity, linear velocity at pos[i]) w.r.t. {global}
	//	- pos[i] is a position std::vector w.r.t. {pbodies[i]}
	//	- idxC[i]: index of the active constraints on the i-th body
	//	    e.g., by setting idxC[i] = [0, 1, 2, 5], all three components of the angular velocity 
	//		and the z-axis component of the linear velocity of pbodies[i] will be constrained.
	bool buildConstrIK_dq(
		RMatrix &J, RMatrix &V,
		std::vector<GBodyRigid*> pbodies, std::vector<Vec3> pos, std::vector<se3> V_target, std::vector< std::vector<int> > idxC);

	// solve dq = pCoordinates[]->dq satisfying the primary goal and minimizing the error on the secondary goal.
	//	- primary goal: V_primary[i] = pbodies_primary[i]'s (angular velocity, linear velocity at p_primary[i]) w.r.t. {global}
	//	- secondary goal: V_secondary[i] = pbodies_secondary[i]'s (angular velocity, linear velocity at p_secondary[i]) w.r.t. {global}
	//	- p_primary[i] and p_secondary[i] are position vectors w.r.t. {pbodies_primary[i]} and {pbodies_secondary[i]} respectively.
	//  - idxC_primary[i]: constraint setting for the i-th primary body
	//	- idxC_secondary[i]: constraint setting for the i-th secondary body
	//	    e.g., by setting idxC_primary[i] = [0, 1, 2, 5], all three components of the angular velocity 
	//		and the z-axis component of the linear velocity of pbodies_primary[i] will be constrained.
	//	- pcoords_prescribed: the coordinates whose velocities(dq) are already prescribed. (pcoords_prescribed[]->dq will be remained in the solution)
	//	- alpha: a parameter for singularity-robust inverse, srInv()
	bool solveIK_dq(
		RMatrix &dq, 
		std::vector<GBodyRigid*> pbodies_primary, std::vector<GBodyRigid*> pbodies_secondary, 
		std::vector<Vec3> p_primary, std::vector<Vec3> p_secondary,
		std::vector<se3> V_primary, std::vector<se3> V_secondary, 
		std::vector< std::vector<int> > idxC_primary, std::vector< std::vector<int> > idxC_secondary, 
		double alpha_primary = 0, double alpha_secondary = 0.001);
	bool solveIK_dq(
		RMatrix &dq, 
		std::vector<GBodyRigid*> pbodies_primary, std::vector<GBodyRigid*> pbodies_secondary, 
		std::vector<Vec3> p_primary, std::vector<Vec3> p_secondary,
		std::vector<se3> V_primary, std::vector<se3> V_secondary, 
		std::vector< std::vector<int> > idxC_primary, std::vector< std::vector<int> > idxC_secondary, 
		std::vector<GCoordinate*> pcoords_prescribed,
		double alpha_primary = 0, double alpha_secondary = 0.001);


	bool solveIK_dq(
		RMatrix &dq, 
		std::vector<GBodyRigid*> pbodies_primary, std::vector<GBodyRigid*> pbodies_secondary, 
		std::vector<Vec3> p_primary, std::vector<Vec3> p_secondary,
		std::vector<se3> V_primary, std::vector<se3> V_secondary, 
		std::vector< std::vector<int> > idxC_primary, std::vector< std::vector<int> > idxC_secondary, 
		std::vector<GCoordinate*> pcoords_prescribed,
		std::ofstream *pfout,
		double alpha_primary = 0, double alpha_secondary = 0.001);
};



#endif

