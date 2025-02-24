//================================================================================
//         GEOMETRIC MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_MULTI_BODY_SYSTEM_
#define _GMBS_MULTI_BODY_SYSTEM_

#include <list>
#include <vector>
#include <string>
#include "gelement.h"
#include "rmatrix3j.h"
#include "liegroup.h"


class GCoordinate;
class GBody;
class GJoint;
class GConstraint;

//=============================================================
//                 GSystem
//=============================================================
class GSystem: public GElement
{
public:	
	GBody *pGround;							// pointer to the ground body

	std::list<GBody *> pBodies;				// pointer to bodies (pGround is not included in pBodies! See buildSystem().)
	std::list<GJoint *> pJoints;			// pointer to joints

	std::vector<RMatrix> Jacobian;			// Jacobian[i] = Jacobian of pBodies[i]

public:
	GSystem();
	~GSystem() {}

public:
	// ---------- building system ----------------------------------------------------

	virtual bool buildSystem(GBody *pGround_);

	// ---------- gravity ------------------------------------------------------------

	bool setGravity(Vec3 g_);				// set gravity e.g. setGravity(Vec3(0,0,-9.81))
	Vec3 getGravity();

	// ---------- access to system elements ------------------------------------------

	GBody* getGround() { return pGround; }	// return pointer to the ground

	GBody* getBody(std::string name_);		// return pointer to a body whose name is name_ (return NULL if there is no such a body)
	GBody* getBody(int idx_);				// return pointer to a body whose index is idx_ (zero-based index assumed in pBodies) (return NULL if there is no such a joint)

	GJoint* getJoint(std::string name_);	// return pointer to a joint whose name is name_ (return NULL if there is no such a joint)
	GJoint* getJoint(int idx_);				// return pointer to a joint whose index is idx_ (zero-based index assumed in pJoints) (return NULL if there is no such a joint)

	int getIndexBody(std::string name_);	// return index of a body whose name is name_
	int getIndexJoint(std::string name_);	// return index of a joint whose name is name_

	// ---------- main functions for kinematics and dynamics -------------------------

	virtual void updateKinematics();		// update all kinematic variables of bodies, such as position, velocity, acceleration, 
											// with current pCoordinates[]->(q,dq,ddq).

	virtual void updateJacobian();			// update Jacobian
											// prerequisite: updateKinematics() or calcDynamics()

	virtual void initExternalForce();		// initializes external forces acting on bodies

	virtual void calcDynamics(bool update_global_location_ = true);			
											// calculate pCoordinates[]->(ddq|tau)
											// if pCoordinates[]->bPrescribed == true, then pCoordinates[]->tau will be updated.
											// if pCoordinates[]->bPrescribed == false, then pCoordinates[]->ddq will be updated.
											// (forward/inverse/hybrid dynamics can be covered by this function only.)
											// set update_global_location_ = false to skip updating global locations of bodies and joints
											// (You can use updateGlobalLocationsOfBodiesAndJoints() to manually update global locations of bodies and joints.)

	virtual void updateGlobalLocationsOfBodiesAndJoints();	
											// updates global locations of bodies and joints (pBodies[]->T_global, pJoints[]->T_global)
											// with current pBodies[]->T

	virtual void diffDynamics();			// calculate pCoordinates[]->(DddqDp|DtauDp)
											// prerequisite: setting differentiating variable p

	virtual void setDeriv_Dq(GCoordinate *pCoordinate_);	// set pCoordinate_->q as the differentiating variable
	virtual void setDeriv_Ddq(GCoordinate *pCoordinate_);	// set pCoordinate_->dq as the differentiating variable
	virtual void setDeriv_Dddq(GCoordinate *pCoordinate_);	// set pCoordinate_->ddq as the differentiating variable
	virtual void setDeriv_Dtau(GCoordinate *pCoordinate_);	// set pCoordinate_->tau as the differentiating variable
	virtual void setDeriv_DFe(GBody *pBody_, int idx_);		// set idx_-th element of the external force acting on pBody_ as the differentiating variable
	

	// ---------- mass, center of mass, momentum --------------------------------------

	double getMass();						// return total mass of the system
	
	Vec3 getPositionCOMGlobal();			// return the position of the center of mass of the system w.r.t. {global}

	Vec3 getVelocityCOMGlobal();			// return the velocity of the center of mass of the system w.r.t. {global}

	Vec3 getAccelerationCOMGlobal();		// return the acceleration of the center of mass of the system w.r.t. {global}
	
	dse3 getMomentumGlobal();				// return Hg = the momentum of the system w.r.t. {global}

	dse3 getMomentumCOM();					// return Hc = the momentum of the system w.r.t. {com}
											//      {com} = a moving coordinate frame whose origin is located at the center of mass of the system
											//              and whose orientation is always aligned with that of {global}.

	// ---------- derivatives of center of mass, momentum -----------------------------
	//
	// ** Local information of all joints needs to be updated before calling functions below.  
	// ** updateKinematics() or calcDynamics() will do this automatically.                     
	// ** To do this manually, use update_joint_local_info_short() or update_joint_local_info().

	void calcDerivative_PositionCOMGlobal_Dq(std::vector<GCoordinate *> pCoordinates_, RMatrix &DpDq_);
											// DpDq_ = the derivative of the position of the center of mass w.r.t. pCoordinates_[]->q
    
	void calcDerivative_PositionCOMGlobal_Dq(RMatrix &DpDq_);
											// DpDq_ = the derivative of the position of the center of mass w.r.t. GSystem::pCoordinates[]->q

	void calcDerivative_PositionCOMGlobal_Dq_2(RMatrix &DpDq_);
											// DpDq_ = the derivative of the position of the center of mass w.r.t. GSystem::pCoordinates[]->q
											// prerequisite: updateJacobian()

	void calcDerivative_MomentumGlobal_Dq_Ddq(std::vector<GCoordinate *> pCoordinates_, RMatrix &DHgDq_, RMatrix &DHgDdq_);
											// DHgDq_, DHgDdq_ = the derivatives of Hg w.r.t. pCoordinates_[]->q and pCoordinates_[]->dq

	void calcDerivative_MomentumGlobal_Dq_Ddq(RMatrix &DHgDq_, RMatrix &DHgDdq_);
											// DHgDq_, DHgDdq_ = the derivatives of Hg w.r.t. GSystem::pCoordinates[]->q and GSystem::pCoordinates[]->dq

	void calcDerivative_MomentumCOM_Dq_Ddq(std::vector<GCoordinate*> pCoordinates_, RMatrix &DHcDq_, RMatrix &DHcDdq_);
											// DHcDq_, DHcDdq_ = the derivatives of Hc w.r.t. pCoordinates_[]->q and pCoordinates_[]->dq

	void calcDerivative_MomentumCOM_Dq_Ddq(RMatrix &DHcDq_, RMatrix &DHcDdq_);
											// DHcDq_, DHcDdq_ = the derivatives of Hc w.r.t. GSystem::pCoordinates[]->q and GSystem::pCoordinates[]->dq

	void calcDerivative_MomentumCOM_Ddq( RMatrix &DHcDdq_); //taesoo
	// ---------- sub-functions ------------------------------------------------------
	
	// sub-functions for buildSystem()
	bool _scanBodiesAndJoints(GBody *pbody_);	// scan bodies and joints
	bool _scanCoordinates();					// scan coordinates for bodies and joints
 	virtual	int getIndexOfCoordinate(GCoordinate *pcoord_); // taesoo : gsystem class override this function to speed up the search.
	bool _findParentChildRelation();			// find pBodies[]->(pBaseJoint, pParentBody, pChildBodies)
	bool _findFwdJointLoopForBodies();			// find the forward joint loop for each body
	bool _getReady();							// get ready

	// sub-functions for updating local joint information
	void update_joint_local_info_short();		// updates part of local information of joints (T, inv_T, S)
	void update_joint_local_info();				// updates local information of joints

	// sub-function for updating body information
	void update_body_global_info();				// updates global location of bodies

	// sub-function for updating body Jacobian
	void update_Jacobian_child_bodies(GBody *pbody_, int idx_, RMatrix S_);	
												// update idx_-th column of pbody_->pChildBodies[]->Jacobian
												// S_ = idx_-th column of pbody_->Jacobian

	// sub-functions for Newton-Euler inverse dynamics
	void neFwdRecursion_a();
	void neBwdRecursion_b();

	// sub-functions for calcDynamics()
	void fsFwdRecursion_a();
	void fsBwdRecursion_b();
	void fsFwdRecursion_c();

	// sub-functions for diffDynamics()
	void fsFwdRecursion_DaDp();
	void fsBwdRecursion_DbDp();
	void fsFwdRecursion_DcDp();

	// sub-functions for the derivatives of the position of the center of mass and momentum
	Vec3 getDerivative_PositionCOMGlobal_Dq_2(GCoordinate *pCoordinate_);
	Vec3 getDerivative_PositionCOMGlobal_Dq(GCoordinate *pCoordinate_);
											// return the derivative of the position of the center of mass of the system w.r.t. pCoordinate_->q
											// Prerequisites: prerequisites for pBodies[]->getDerivative_PositionCOMGlobal_Dq(pCoordinate_)
											//                ( Usually, it requires that
											//					 1. pBodies[]->fJL.M1 = identity and pBodies[]->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION.
											//					 2. pBodies[]->fJL.update_J(); )
	dse3 getDerivative_MomentumGlobal_Dq(GCoordinate *pCoordinate_);
	dse3 getDerivative_MomentumGlobal_Ddq(GCoordinate *pCoordinate_);
											// return the derivatives of the momentum w.r.t. pCoordinate_->q and pCoordinate_->dq
											// Prerequisites: prerequisites for pBodies[]->getMomentumGlobalDerivative_[Dq,Ddq](pCoordinate_)
											//                ( Usually, it requires that
											//					 1. pBodies[]->fJL.M1 = identity and pBodies[]->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION.
											//					 2. pBodies[]->fJL.update_J(); )

	// --------- auxiliary functions ---------------------------------------------------

	std::string getInfoStr();
};



#endif

