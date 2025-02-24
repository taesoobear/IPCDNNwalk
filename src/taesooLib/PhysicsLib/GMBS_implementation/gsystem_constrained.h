//================================================================================
//         GEOMETRIC MULTIBODY SYSTEM WITH CONSTRAINTS 
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_MULTI_BODY_SYSTEM_CONSTRAINED_
#define _GMBS_MULTI_BODY_SYSTEM_CONSTRAINED_

#include <list>
#include "rmatrix3j.h"
#include "liegroup.h"
#include "gsystem.h"
#include "gconstraint_jointloop.h"


class GConstraint;

//=============================================================
//                 GSystemConstrained
//=============================================================
class GSystemConstrained: public GSystem
{
public:	
	std::list<GConstraint *>	pConstraints;					// pointer to constraints

	std::list<GCoordinate *> pCoordinatesIndependent;			// pointer to independent coordinates
	std::list<GCoordinate *> pCoordinatesDependent;				// pointer to dependent coordinates

	std::list<GConstraintJointLoop> closedJointLoopConstraints;	// embedded closed joint-loop constraints

	std::list<GCoordinate *> pCutCoordinates;					// pointer to coordinates of cut-joints

public:
	RMatrix C;				// constraint eqns: C(q) = 0 where q = pSystem->pCoordinates[]->q
	RMatrix J, Ju, Jv;		// J = dc/dq, Ju = dc/dqu, Jv = dc/dqv where qu = pCoordinatesIndependent[]->q, qv = pCoordinatesDependent[]->q
	RMatrix dJdt;			// dJdt = dJ/dt

	double tolerance;		// tolerance limit in solving constraint equations(FNorm(del_q) < tolerance). (default = 1E-6)
	int max_iter_num;		// maximum iteration number in solving constraint equstions with Newton-Raphson method. (default = 50)

public:
	GSystemConstrained();
	~GSystemConstrained() {}

public:
	bool buildSystem(GBody *pGround_);

	// not implemented yet..
	void updateKinematics() {}
	void calcDynamics() {}
	void diffDynamics() {}

public:
	bool addConstraint(GConstraint *pConstraint_);
	bool removeConstraint(GConstraint *pConstraint_);
	void removeAllConstraints();

public:
	double setTolerance(double tolerance_);				// set tolerance for solving constraints and return the previous value
	int setMaximumIterationNumber(int max_iter_num_);	// set maximum iteration number for solving constraints and return the previous value

	bool setIndependentCoordinates(std::list<GCoordinate *> pIndependentCoordinates_);	// set independent coordinates directly

	bool updateDependentCoordinates();			// update displacement, velocity, acceleration of dependent coordinates.

	int getNumCoordinatesIndependent() { return int(pCoordinatesIndependent.size()); }
	int getNumCoordinatesDependent() { return int(pCoordinatesDependent.size()); }

public:	
	// sub-functions for buildSystem()
	bool _findClosedJointLoopConstraints();
	bool _findClosedJointLoop(GJoint *pCutJoint_, std::list<GJoint *> &loopjoints_);
	bool _findJointLoop(GJoint *pEndJoint_, std::list<GJoint *> &loopJoints_);

	// sub-functions for solving constraints, updateDependentCoordinates().
	// inefficient!! we don't have to consider coordinates not related to the constraints.
	int _getNC();						// get the number of all constraints
	
	bool _update_qv();					// update displacement of dependent coordinates
	bool _update_dqv();					// update velocity of dependent coordinates
	bool _update_ddqv();				// update acceleration of dependent coordinates
	
	bool _update_C();					// update C
	bool _update_J();					// update J
	bool _update_Ju();					// update Ju
	bool _update_Jv();					// update Jv
	bool _update_Ju_Jv();				// update Ju and Jv
	bool _update_J_Ju_Jv();				// update J, Ju, and Jv
	bool _update_dJdt();				// update dJdt

	// sub-functions for dynamics
	// inefficient!! we don't have to consider coordinates not related to the constraints.
	bool _calculateEquivalentIndependentCoordinatesForce(std::list<double> &tauu);	
};



#endif

