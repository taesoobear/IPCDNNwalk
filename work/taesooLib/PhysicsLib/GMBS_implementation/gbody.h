//================================================================================
//         GEOMETRIC BODY FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_BODY_
#define _GMBS_GEOMETRIC_BODY_

#include <list>
#include "gelement.h"
#include "gconstraint_jointloop.h"
#include "liegroup.h"


class GJoint;

//=============================================================
//                 GBody
//=============================================================
class GBody: public GElement
{
public:
	std::list<GJoint *> pJoints;		// pointer to attached joints

	SE3 T_global;						// SE3: {global} -> {body}

	
	// --------- parent/child relation in the (virtual) tree topology system -------------

	GJoint *pBaseJoint;					// pBaseJoint->pRightBody = this
	GBody *pParentBody;					// pointer to parent body, pParentBody = pBaseJoint->pLeftBody
	std::list<GBody *> pChildBodies;	// pointers to child bodies, pChildBodies[]->pParentBody = this

	SE3 T, invT;						// T = pParentBody->{body} -> {body}, invT = Inv(T)

	RMatrix S, dS;						// properties of the base joint(pBaseJoint), but viewed in {body} frame
	se3 Sdq, dSdq, Sddq, DSdqDt;

	RMatrix Jacobian;					// Jacobian mapping system coordinates velocity to the body's velocity
	
	GConstraintJointLoop fJL;			// forward joint loop from ground to the body


	// --------- auxiliary ---------------------------------------------------------------

	bool bDpAlien;						// see set_bDpAlien(...) below.

public:
	GBody();
	~GBody() {}

public:

	virtual bool getReady();

	// --------- connection --------------------------------------------------------------

	bool addJoint(GJoint *pJoint_);
	bool removeJoint(GJoint *pJoint_);
	void removeAllJoints();


	// --------- get methods -------------------------------------------------------------

	virtual double getMass() = 0;				// return mass

	virtual Vec3 getPositionCOM() = 0;			// return the position of the body's c.o.m. w.r.t. {body}
	virtual Vec3 getPositionCOMGlobal() = 0;	// return the position of the body's c.o.m. w.r.t. {global}
	virtual Vec3 getVelocityCOMGlobal() = 0;	// return the velocity of the body at c.o.m. w.r.t. {global}
	virtual Vec3 getAccelerationCOMGlobal() = 0;// return the acceleration of the body at c.o.m. w.r.t. {global}

	virtual dse3 getMomentum() = 0;				// return the generalized momentum w.r.t. {body}
	virtual dse3 getMomentumGlobal() = 0;		// return the generalized momentum w.r.t. {global}
	
	virtual Vec3 getDerivative_PositionCOMGlobal_Dq(GCoordinate *pCoordinate_) = 0;
												// return the derivative of the position of the center of mass in {global} w.r.t. pCoordinate_->q

	virtual dse3 getDerivative_MomentumGlobal_Dq(GCoordinate *pCoordinate_) = 0;	
	virtual dse3 getDerivative_MomentumGlobal_Ddq(GCoordinate *pCoordinate_) = 0;
												// return the derivative of the generalized momentum in {global} w.r.t. pCoordinate_->q
												// return the derivative of the generalized momentum in {global} w.r.t. pCoordinate_->dq
												// Prerequisites: 1. make sure that fJL.M1 = identity, or fJL.setM(SE3());
												//                2. fJL.update_J();

	virtual std::string getInfoStr();

	// --------- methods for geometric dynamics and kinematics ---------------------------

	// import base joint information
	virtual void import_base_joint_info() = 0;

	se3& get_Sdq() { return Sdq; }
	se3& get_dSdq() { return dSdq; }
	se3& get_Sddq() { return Sddq; }
	se3& get_DSdqDt() { return DSdqDt; }

	RMatrix get_S() { return S; }
	RMatrix get_dS() { return dS; }

	// geometric kinematics
	virtual void set_dV(se3 dV_) = 0;

	virtual se3 get_V() = 0;					
	virtual se3 get_dV() = 0;
	virtual se3 get_DVDp() = 0;
	virtual se3 get_DdVDp() = 0;

	// geometric dynamics
	virtual void initExternalForce() = 0;

	virtual void neDynaRecursion_a() = 0;
	virtual void neDynaRecursion_b() = 0;

	virtual void fsDynaRecursion_a() = 0;
	virtual void fsDynaRecursion_b() = 0;
	virtual void fsDynaRecursion_c() = 0;

	virtual dse3 getTransformed_F() = 0;		
	virtual AInertia getTransformed_aI() = 0;	
	virtual dse3 getTransformed_aB() = 0;		

	// derivative of the dynamics
	virtual void neDynaRecursion_DaDp() = 0;
	virtual void neDynaRecursion_DbDp() = 0;

	virtual void fsDynaRecursion_DaDp() = 0;
	virtual void fsDynaRecursion_DbDp() = 0;
	virtual void fsDynaRecursion_DcDp() = 0;

	virtual dse3 getTransformed_DFDp() = 0;
	virtual AInertia getTransformed_DaIDp() = 0;
	virtual dse3 getTransformed_DaBDp() = 0;

	// set differentiating variable
	virtual void setDifferentiatingVariable_Dq(GCoordinate *pCoordinate_) = 0;
	virtual void setDifferentiatingVariable_Ddq(GCoordinate *pCoordinate_) = 0;
	virtual void setDifferentiatingVariable_Dddq(GCoordinate *pCoordinate_) = 0;
	virtual void setDifferentiatingVariable_Dtau(GCoordinate *pCoordinate_) = 0;
	virtual void setDifferentiatingVariable_DFe(GBody *pBody_, int idx_) {}

	void set_bDpAlien(bool bDpAlien_) { bDpAlien = bDpAlien_; }		
									// Set 'bDpAlien = true' for fast calculation of DddqDp or DtauDp in case that
									//  DVDp = DetaDp = DaIDp = DPsiDp = DPiDp = 0, DSDp = DdSDp = DhDp = DIDp = 0,
									//  and DFeDp, DdVDp, DFDp, DaBDp, DbetaDp are nonzero.

};


#endif

