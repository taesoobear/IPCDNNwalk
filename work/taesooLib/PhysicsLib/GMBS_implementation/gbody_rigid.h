//================================================================================
//         RIGID BODY FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_BODY_RIGID_
#define _GMBS_GEOMETRIC_BODY_RIGID_

#include "gbody.h"
#include "liegroup.h"
#include "rmatrix3j.h"


//=============================================================
//                 GBodyRigid
//=============================================================
class GBodyRigid: public GBody
{
public:
	// --------- geometric dynamics ------------------------------------------------------

	// quantities to be set by user
	Inertia I;				// generalized inertia (w.r.t. {body})
	dse3 Fe;				// generalized force acting on body from external environment (w.r.t. {body})

	// internal variables
	se3 V;					// generalized body velocity of {body} relative to {global} but viewed in {body}. V = inv(T_global)*d(T_global)/dt.
	se3 dV;					// generalized body acceleration. V = dV/dt.
	dse3 F;					// generalized force acting on body through pBaseJoint and viewed in {body}.
	AInertia aI;			// articulated body inertia 
	dse3 aB;				// bias force: F = aI*dV + aB
	se3 eta;
	RMatrix aI_S;
	RMatrix Psi;
	AInertia Pi;
	dse3 beta;

	// --------- derivatives of the geometric dynamics -----------------------------------

				// --------- derivative of the geometric dynamics w.r.t. an arbitrary scalar variable p ------
				// 'D' means 'partial derivative'
				// prerequisite:
				//               - set bnzD[q,dq]Dp
				//               - set pBaseJoint->pCoordinates[]->(DqDp,DdqDp)
				//               - set pBaseJoint->pCoordinates[]->DddqDp if pBaseJoint->isPrescribed() == true
				//               - set pBaseJoint->pCoordinates[]->DtauDp if pBaseJoint->isPrescribed() == false
				//               - set bnzD[S,h,Fe,I]Dp
				//				 - set DSDp, DdSDp, DhDp, DFeDp, DIDp
				// DSDp, DdSDp, DhDp are related with each other, and cannot be set with arbitrary values.
				// For example, if p == pBaseJoint->pCoordinates[i]->q then DhDp = S[i].

	// variables to be set by user
	RMatrix DSDp, DdSDp;	// DS/Dp, D(DS/Dt)/Dp
	se3 DhDp;				// invT*DT/Dp
	dse3 DFeDp;
	Inertia DIDp;

	bool bnzDqDp, bnzDdqDp;
	bool bnzDSDp, bnzDdSDp, bnzDhDp, bnzDFeDp, bnzDIDp;
	
	// internal variables
	se3 DVDp;
	se3 DdVDp;
	dse3 DFDp;
	AInertia DaIDp;
	dse3 DaBDp;
	se3 DetaDp;
	RMatrix DPsiDp;
	AInertia DPiDp;
	dse3 DbetaDp;

public:
	GBodyRigid();
	~GBodyRigid() {}

public: 

	virtual bool getReady();
	
	// --------- set methods -------------------------------------------------------------

	// mass properties
	void setMass(const double &mass_, const Vec3 &p_ = Vec3(0,0,0));	// point mass at p in {body}
	void setMass(const double &mass_, const double &ixx_, const double &iyy_, const double &izz_, const double &ixy_, const double &ixz_, const double &iyz_, const SE3 &T_ref_ = SE3());
	void addMass(const double &mass_, const double &ixx_, const double &iyy_, const double &izz_, const double &ixy_, const double &ixz_, const double &iyz_, const SE3 &T_ref_ = SE3());
	void extractMass(const double &mass_, const double &ixx_, const double &iyy_, const double &izz_, const double &ixy_, const double &ixz_, const double &iyz_, const SE3 &T_ref_ = SE3());
														// mass_ = mass of the body
														// inertia = [ixx_, ixy_, ixz_] = the 3 x 3 inertia matrix w.r.t. {ref}
														//           [ixy_, iyy_, iyz_]
														//           [ixz_, iyz_, izz_]
														// ** Be cautious on the ordering, (ixx_, iyy_, izz_, ixy_, ixz_, iyz_).
														// T_ref_ = SE3: {body} -> {ref}

	void moveMass(const SE3 &T_ref_new_);				// current mass and inertia of the body will be moved to {ref_new}
														// T_ref_new_ = SE3: {body} -> {ref_new}

	// external force
	void setExternalForceLocally(const dse3 &Fe_local_);		// Fe_local_ = external generalized force w.r.t. {body}
	void setExternalForceGlobally(const dse3 &Fe_global_);		// Fe_global_ = external generalized force w.r.t. {global}
	void setExternalForceGlobally(const Vec3 &p_, const Vec3 &fg_);	
														// set external force fg_ (w.r.t. {global}) acting on the body at p_ (w.r.t. {body})

	void addExternalForceLocally(const dse3 &Fe_local_);
	void addExternalForceGlobally(const dse3 &Fe_global_);
	void addExternalForceGlobally(const Vec3 &p_, const Vec3 &fg_);	

	// variables for differentiating dynamics
	void set_DSDp(const RMatrix &DSDp_) { DSDp = DSDp_; }
	void set_DdSDp(const RMatrix &DdSDp_) { DdSDp = DdSDp_; }
	void set_DhDp(const se3 &DhDp_) { DhDp = DhDp_; }
	void set_DFeDp(const dse3 &DFeDp_) { DFeDp = DFeDp_; }
	void set_DIDp(const Inertia &DIDp_) { DIDp = DIDp_; }

	void set_bnzAll(bool bnzAll_) { bnzDqDp = bnzDdqDp = bnzDSDp = bnzDdSDp = bnzDhDp = bnzDFeDp = bnzDIDp = bnzAll_; }

	void set_bnzDqDp(bool bnzDqDp_) { bnzDqDp = bnzDqDp_; }
	void set_bnzDdqDp(bool bnzDdqDp_) { bnzDdqDp = bnzDdqDp_; }
	void set_bnzDSDp(bool bnzDSDp_) { bnzDSDp = bnzDSDp_; }
	void set_bnzDdSDp(bool bnzDdSDp_) { bnzDdSDp = bnzDdSDp_; }
	void set_bnzDhDp(bool bnzDhDp_) { bnzDhDp = bnzDhDp_; }
	void set_bnzDFeDp(bool bnzDFeDp_) { bnzDFeDp = bnzDFeDp_; }
	void set_bnzDIDp(bool bnzDIDp_) { bnzDIDp = bnzDIDp_; }


	// --------- get methods -------------------------------------------------------------	

	// position and orientation
	Vec3 getPositionGlobal();							// return the position of {body}'s origin w.r.t. {global}
	Vec3 getPositionGlobal(const Vec3 &p_);				// return the position of the body at the point p_ w.r.t. {global}, where p_ is a relative position vector of the point w.r.t. {body}
	SO3 getOrientationGlobal();							// return the orientation of {body} w.r.t. {global}
	SO3 getOrientationGlobal(const SO3 &R_);			// return the orientation of {tmp} w.r.t. {global}, where R_ = SO(3): {body} --> {tmp}
	SE3 getPoseGlobal();								// return the position and orientation of {body} w.r.t. {global}
	SE3 getPoseGlobal(const SE3 &T_);					// return the position and orientation of {tmp} w.r.t. {global}, where T_ = SE(3): {body} --> {tmp}

	// velocity
	Vec3 getVelocityLinearGlobal();						// return the linear velocity of the body at the origin of {body} w.r.t. {global}
	Vec3 getVelocityLinearGlobal(const Vec3 &p_);		// return the linear velocity of the body at p_ w.r.t. {global}, where p_ is a relative position vector w.r.t {body}
	Vec3 getVelocityAngularGlobal();					// return the angular velocity of the body w.r.t. {global}
	se3 getVelocityGlobal();							// return se3(getVelocityAngularGlobal(), getVelocityLinearGlobal())
	se3 getVelocityGlobal(const Vec3 &p_);				// return se3(getVelocityAngularGlobal(), getVelocityLInearGlobal(p_))

	// acceleration                                     ** To get a relative acceleration of the body to the ground, the acceleration of the ground should be extracted. **
	Vec3 getAccelerationLinearGlobal();					// return the linear acceleration of the body at the origin of {body} w.r.t. {global}
	Vec3 getAccelerationLinearGlobal(const Vec3 &p_);	// return the linear acceleration of the body at p_ w.r.t. {global}, where p_ is a relative position vector w.r.t. {body}
	Vec3 getAccelerationAngularGlobal();				// return the angular acceleration of the body w.r.t. {global}
	se3 getAccelerationGlobal();						// return se3(getAccelerationAngularGlobal(), getAccelerationLinearGlobal())
	se3 getAccelerationGlobal(const Vec3 &p_);			// return se3(getAccelerationAngularGlobal(), getAccelerationLinearGlobal(p_))

	// mass, center of mass
	virtual double getMass();							// return mass
	
	virtual Vec3 getPositionCOM();						// return the position of the body's c.o.m. w.r.t. {body}
	virtual Vec3 getPositionCOMGlobal();				// return the position of the body's c.o.m. w.r.t. {global}
	virtual Vec3 getVelocityCOMGlobal();				// return the velocity of the body at c.o.m. w.r.t. {global}
	virtual Vec3 getAccelerationCOMGlobal();			// return the acceleration of the body at c.o.m. w.r.t. {global}

	// momentum
	virtual dse3 getMomentum();							// return the generalized momentum w.r.t. {body}
	virtual dse3 getMomentumGlobal();					// return the generalized momentum w.r.t. {global}

	// derivative of the center of mass
	virtual Vec3 getDerivative_PositionCOMGlobal_Dq(GCoordinate *pCoordinate_);
														// return the derivative of the position of the center of mass in {global} w.r.t. pCoordinate_->q
														// Prerequisites: 1. fJL.M1 = identity and fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION.
														//                2. fJL.update_J();

	// derivative of the momentum
	virtual dse3 getDerivative_MomentumGlobal_Dq(GCoordinate *pCoordinate_);
	virtual dse3 getDerivative_MomentumGlobal_Ddq(GCoordinate *pCoordinate_);
														// return the derivative of the generalized momentum in {global} w.r.t. pCoordinate_->q
														// return the derivative of the generalized momentum in {global} w.r.t. pCoordinate_->dq
														// Prerequisites: 1. fJL.M1 = identity and fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION.
														//                2. fJL.update_J();

	virtual std::string getInfoStr();


	// --------- methods for geometric kinematics and dynamics ---------------------------

	// import base joint information
	virtual void import_base_joint_info();	// import S, dS, Sdq, dSdq, Sddq, DSdqDt from pBaseJoint->(S,dS,...)

	// update base joint information
	void update_base_joint_info();			// update S, dS, Sdq, dSdq, Sddq, DSdqDt with pBaseJoint->(S,dS,...) if needed

	// geometric kinematics
	virtual void set_dV(se3 dV_) { dV = dV_; }

	virtual se3 get_V() { return V; }	
	virtual se3 get_dV() { return dV; }	
	virtual se3 get_DVDp() { return DVDp; }
	virtual se3 get_DdVDp() { return DdVDp; }

	// geometric dynamics
	virtual void initExternalForce() { Fe.SetZero(); }

	virtual void neDynaRecursion_a();
	virtual void neDynaRecursion_b();

	virtual void fsDynaRecursion_a();
	virtual void fsDynaRecursion_b();
	virtual void fsDynaRecursion_c();

	virtual dse3 getTransformed_F();		// return F viewed in pParentBody->{body}
	virtual AInertia getTransformed_aI();	// return aI viewed in pParentBody->{body}
	virtual dse3 getTransformed_aB();		// return aB viewed in pParentBody->{body}

	// derivative of the dynamics
	virtual void neDynaRecursion_DaDp();
	virtual void neDynaRecursion_DbDp();

	virtual void fsDynaRecursion_DaDp();
	virtual void fsDynaRecursion_DbDp();
	virtual void fsDynaRecursion_DcDp();

	virtual dse3 getTransformed_DFDp();
	virtual AInertia getTransformed_DaIDp();
	virtual dse3 getTransformed_DaBDp();

	// set differentiating variable
	virtual void setDifferentiatingVariable_Dq(GCoordinate *pCoordinate_);
	virtual void setDifferentiatingVariable_Ddq(GCoordinate *pCoordinate_);
	virtual void setDifferentiatingVariable_Dddq(GCoordinate *pCoordinate_);
	virtual void setDifferentiatingVariable_Dtau(GCoordinate *pCoordinate_);
	virtual void setDifferentiatingVariable_DFe(GBody *pBody_, int idx_);


	// --------- sub-functions for geometric kinematics and dynamics ---------------------

	virtual void update_T();
	virtual void update_V();				
	virtual void update_eta();
	virtual void update_dV(bool b_update_);			// set b_update_ = true to update dV with new pBaseJoint->get_ddq()
	virtual void update_F();
	virtual void update_F_fs();
	virtual void update_aI();
	virtual void update_aB();
	virtual void update_Psi();
	virtual void update_Pi();
	virtual void update_beta();
	virtual void update_tau();
	virtual void update_ddq();

	virtual void update_DVDp();				
	virtual void update_DetaDp();
	virtual void update_DdVDp();
	virtual void update_DFDp();
	virtual void update_DFDp_fs();
	virtual void update_DaIDp();
	virtual void update_DaBDp();
	virtual void update_DPsiDp();
	virtual void update_DPiDp();
	virtual void update_DbetaDp();
	virtual void update_DtauDp();
	virtual void update_DddqDp();

	se3 get_S(GCoordinate *pCoordinate_);			// return i-th column of S where pBaseJoint->pCoordinates[i] = pCoordinate_.
	se3 get_dS(GCoordinate *pCoordinate_);			// return i-th column of dS where pBaseJoint->pCoordinates[i] = pCoordinate_.
	
	RMatrix get_DSDq(GCoordinate *pCoordinate_);	// return DS/Dq where q = pCoordinate_->q.
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);	// return D(DS/Dt)/Dq where q = pCoordinate_->q.
	RMatrix get_DdSDdq(GCoordinate *pCoordinate_);	// return D(DS/Dt)/Ddq where dq = pCoordinate_->dq.

	bool isIncluded(GCoordinate *pCoordinate_);		// return true if pCoordinate_ is included pBaseJoint->pCoordinates.
};


#endif

