//================================================================================
//         GEOMETRIC JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_
#define _GMBS_GEOMETRIC_JOINT_

#include <vector>
#include "gelement.h"
#include "liegroup.h"
#include "rmatrix3j.h"


class GBody;

enum JointType { 
	GJOINT_NULL, 
	GJOINT_FIXED, 
	GJOINT_REVOLUTE, GJOINT_PRISMATIC, 
	GJOINT_PLANAR, GJOINT_UNIVERSAL, 
	GJOINT_TRANSLATIONAL, GJOINT_SPHERICAL, 
	GJOINT_FREE, 
	GJOINT_COMPOSITE ,
	GJOINT_QUAT
};

//=============================================================
//                 GJoint
//=============================================================
class GJoint: public GElement
{
public:
	JointType jointType;

	GBody *pLeftBody, *pRightBody;	// pointer to bodies connected

	SE3 T_left;						// SE3: pLeftBody->{body} --> {joint left}
	SE3 T_right;					// SE3: pRightBody->{body} -> {joint right}
	SE3 inv_T_right;				// Inv(T_right)
	SE3 inv_T_left;					// Inv(T_left)

	SE3 T_global;					// SE3: {global} --> {joint left}

	SE3 T;							// SE3: {joint left} --> {joint right}
	SE3 inv_T;						// inv_T = Inv(T)
	RMatrix S;						// S = [S1,...,Sn], the screw axis( or axes) w.r.t. {joint right}, where n = getDOF().
	RMatrix dS;						// dS = DS/Dt = [DS1/Dt,...,DSn/Dt]
	se3 Sdq;						// Sdq = S*dq = Inv(T)*DT/Dt where dq = pCoordinates[]->dq
	se3 dSdq;						// dSdq = dS*dq
	se3 Sddq;						// Sddq = S*ddq where ddq = pCoordinates[]->ddq
	se3 DSdqDt;						// DSdqDt = D(S*dq)/Dt = DS/Dt*dq + S*ddq

public:
	bool bReversed;					// bReversed = false if the left and right bodies specified by user are swapped internally.

	bool bCut;						// bCut = true if the joint is cut virtually for closed loop analysis
	
	bool bPrescribed;				// bPrescribed = true if pCoordinates[]->bPrescribed = true

public:
	GJoint();
	~GJoint() {}

public:
	virtual bool connectBodies(GBody *pLeftBody_, GBody *pRightBody_);
	virtual void disconnectBodies();

	void setPosition(const Vec3 &pL_, const Vec3 &pR_);
	void setOrientation(const SO3 &RL_, const SO3 &RR_);
	void setPositionAndOrientation(const SE3 &TL_, const SE3 &TR_);

	int getDOF() { return getNumCoordinates(); }

	void setPrescribed(bool b_);		// set pCoordinates[]->bPrescribed = b_
	bool isPrescribed() { return bPrescribed; }	

	bool isReversed() { return bReversed; }

	bool isCut() { return bCut; }

	std::string getInfoStr();

public:
	SE3 &get_T() { return T; }
	SE3 &get_inv_T() { return inv_T; }
	se3 &get_Sdq() { return Sdq; }
	se3 &get_dSdq() { return dSdq; }
	se3 &get_Sddq() { return Sddq; }
	se3 &get_DSdqDt() { return DSdqDt; }

	RMatrix &get_S() { return S; }
	RMatrix &get_dS() { return dS; }

	se3 get_S(int idx_);										// return idx_-th column of S
	se3 get_S(GCoordinate *pCoordinate_);						// return i-th column of S if pCoordinates[i] = pCoordinate_.
	se3 get_dS(int idx_);										// return idx_-th column of dS
	se3 get_dS(GCoordinate *pCoordinate_);						// return i-th column of dS if pCoordinates[i] = pCoordinate_.

	void allocate_memory(int n_);								// allocate memory for S, dS

public:	
	virtual bool reverse();										// reverse joint direction
	virtual bool isConstantScrew() { return false; }			// return false if S is a function of q.

	virtual void update_short() = 0;							// update T, inv_T, S (for joint loop Jacobian update)
	virtual void update() = 0;									// update T, inv_T, S, dS, Sdq, dSdq, Sddq, DSdqDt

	virtual RMatrix get_DSDq(GCoordinate *pCoordinate_) = 0;	// return DS/Dq where q = pCoordinate_->q
	virtual RMatrix get_DdSDq(GCoordinate *pCoordinate_) = 0;	// return D(dS)/Dq where q = pCoordinate_->q
};


#endif

