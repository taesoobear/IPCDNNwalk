#include <list>
#include <algorithm>
#include <stdio.h>

#include "gbody_rigid.h"
#include "gbody.h"
#include "gjoint.h"
#include "gcoordinate.h"

#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"

using namespace std;


//=============================================================
//                 GBodyRigid
//=============================================================
GBodyRigid::GBodyRigid()
{
	I.SetZero();
	Fe.SetZero();

	V.SetZero();
	dV.SetZero();
	F.SetZero();
	aI.SetZero();
	aB.SetZero();
	eta.SetZero();
	aI_S.SetZero(0,0);		// size may vary
	Psi.SetZero(0,0);		// size may vary
	Pi.SetZero();
	beta.SetZero();

	DSDp.SetZero(0,0);		// need to be redefined
	DdSDp.SetZero(0,0);		// need to be redefined
	DhDp.SetZero();
	DFeDp.SetZero();
	DIDp.SetZero();

	bnzDqDp = bnzDdqDp = false;
	bnzDSDp = bnzDdSDp = bnzDhDp = bnzDFeDp = bnzDIDp = false;

	DVDp.SetZero();
	DdVDp.SetZero();
	DFDp.SetZero();
	DaIDp.SetZero();
	DaBDp.SetZero();
	DetaDp.SetZero();
	DPsiDp.SetZero(0,0);	// size may vary
	DPiDp.SetZero();
	DbetaDp.SetZero();
}

bool GBodyRigid::getReady()
{
	if ( !GBody::getReady() ) return false;
	aI_S.SetZero(6, pBaseJoint->getDOF());
	Psi.SetZero(pBaseJoint->getDOF(), pBaseJoint->getDOF());
	DSDp.SetZero(6, pBaseJoint->getDOF());
	DdSDp.SetZero(6, pBaseJoint->getDOF());
	DPsiDp.SetZero(pBaseJoint->getDOF(), pBaseJoint->getDOF());
/*	if ( getMass() < 1E-8 ) 
	{
		printf("mass error\n");
		return false;
	}ddexter

	*/
	return true;
}

void GBodyRigid::setMass(const double &mass_, const Vec3 &p_)
{
	setMass(mass_, 0, 0, 0, 0, 0, 0, SE3(p_));
}

void GBodyRigid::setMass(const double &mass_, const double &ixx_, const double &iyy_, const double &izz_, const double &ixy_, const double &ixz_, const double &iyz_, const SE3 &T_ref_)
{
	Inertia I_;						// I_ = the generalized inertia w.r.t. {ref}
	I_.SetMass(mass_);
	I_.SetInertia(ixx_,iyy_,izz_,ixy_,ixz_,iyz_);

	I = I_.Transform(Inv(T_ref_));	// I = the generalized inertia w.r.t. {body}
}

void GBodyRigid::addMass(const double &mass_, const double &ixx_, const double &iyy_, const double &izz_, const double &ixy_, const double &ixz_, const double &iyz_, const SE3 &T_ref_)
{
	Inertia I_;									// I_ = the generalized inertia w.r.t. {ref}
	I_.SetMass(mass_);
	I_.SetInertia(ixx_,iyy_,izz_,ixy_,ixz_,iyz_);
	
	Inertia I_b = I_.Transform(Inv(T_ref_));	// I_b = the generalized inertia w.r.t. {body}

	// I += I_b
	for (int i=0; i<6; i++) { I._I[i] += I_b._I[i]; }
	for (int i=0; i<3; i++) { I._r[i] += I_b._r[i]; }
	I._m += I_b._m;
}

void GBodyRigid::extractMass(const double &mass_, const double &ixx_, const double &iyy_, const double &izz_, const double &ixy_, const double &ixz_, const double &iyz_, const SE3 &T_ref_)
{
	Inertia I_;									// I_ = the generalized inertia w.r.t. {ref}
	I_.SetMass(mass_);
	I_.SetInertia(ixx_,iyy_,izz_,ixy_,ixz_,iyz_);

	Inertia I_b = I_.Transform(Inv(T_ref_));	// I_b = the generalized inertia w.r.t. {body}

	// I -= I_b
	for (int i=0; i<6; i++) { I._I[i] -= I_b._I[i]; }
	for (int i=0; i<3; i++) { I._r[i] -= I_b._r[i]; }
	I._m -= I_b._m;
}

void GBodyRigid::moveMass(const SE3 &T_ref_new_)
{
	Inertia I_ref_new = I;
	I = I_ref_new.Transform(Inv(T_ref_new_));
}

Vec3 GBodyRigid::getPositionGlobal()
{
	return T_global.GetPosition();
}

Vec3 GBodyRigid::getPositionGlobal(const Vec3 &p_)
{
	return T_global * p_;
}

SO3 GBodyRigid::getOrientationGlobal()
{
	return T_global.GetRotation();
}

SO3 GBodyRigid::getOrientationGlobal(const SO3 &R_)
{
	return T_global.GetRotation() * R_;
}

SE3 GBodyRigid::getPoseGlobal()
{
	return T_global;
}

SE3 GBodyRigid::getPoseGlobal(const SE3 &T_)
{
	return T_global * T_;
}

Vec3 GBodyRigid::getVelocityLinearGlobal()
{
	return T_global.GetRotation() * V.GetV();
}

Vec3 GBodyRigid::getVelocityLinearGlobal(const Vec3 &p_)
{
	return T_global.GetRotation() * (Cross(V.GetW(),p_) + V.GetV());
}

Vec3 GBodyRigid::getVelocityAngularGlobal()
{
	return T_global.GetRotation() * V.GetW();
}

se3 GBodyRigid::getVelocityGlobal()
{
	SO3 R(T_global.GetRotation());
	return se3(R*V.GetW(), R*V.GetV());
}

se3 GBodyRigid::getVelocityGlobal(const Vec3 &p_)
{
	// Vp = Ad(SE3(R,0), Ad(Inv(SE3(Eye,p_)), V))
	//    = Ad(SE3(R,0) * Inv(SE3(Eye,p_)), V)
	//    = Ad(SE3(R, -R*p_), V)
	//    = se3(R*w, R*([w]*p + v))
	// where (w,v) = V, R = orientation of {body} w.r.t. {global}, and p_ = a position vector w.r.t. {body}

	SO3 R(T_global.GetRotation());
	Vec3 w(V.GetW()), v(V.GetV());
	return se3(R*w, R*(Cross(w,p_)+v));
}

Vec3 GBodyRigid::getAccelerationLinearGlobal()
{
	return T_global.GetRotation() * (Cross(V.GetW(), V.GetV()) + dV.GetV());
}

Vec3 GBodyRigid::getAccelerationLinearGlobal(const Vec3 &p_)
{
	SO3 R(T_global.GetRotation());
	Vec3 w(V.GetW()), v(V.GetV()), dw(dV.GetW()), dv(dV.GetV());
	return R*(Cross(w,Cross(w,p_)) + Cross(w,v) + Cross(dw,p_) + dv);
}

Vec3 GBodyRigid::getAccelerationAngularGlobal()
{
	return T_global.GetRotation() * dV.GetW();
}

se3 GBodyRigid::getAccelerationGlobal()
{
	SO3 R(T_global.GetRotation());
	Vec3 w(V.GetW()), v(V.GetV()), dw(dV.GetW()), dv(dV.GetV());
	return se3(R*dw, R*(Cross(w,v)+dv));
}

se3 GBodyRigid::getAccelerationGlobal(const Vec3 &p_)
{
	// Vp = Ad(T, V) where T = SE3(R, -R*p_) // see GBodyRigid::getVelociyGlobal
	// d(Vp)/dt = ad(dT/dt*Inv(T), Ad(T, V)) + Ad(T, dV/dt) where dV/dt = GBodyRigid::dV
	//          = se3( R*dw, R*([w]*[w]*p_ + [w]*v + [dw]*p_ + dv ) where (w,v) = V, (dw,dv) = dV/dt

	SO3 R(T_global.GetRotation());
	Vec3 w(V.GetW()), v(V.GetV()), dw(dV.GetW()), dv(dV.GetV());
	return se3(R*dw, R*(Cross(w,Cross(w,p_)) + Cross(w,v) + Cross(dw,p_) + dv));
}

double GBodyRigid::getMass()
{
	return I.GetMass();
}

Vec3 GBodyRigid::getPositionCOM()
{
	if ( getMass() > 1E-8 ) {
		return (1./getMass()) * I.GetOffDiag();
	} else {
		return Vec3(0,0,0);
	}
}

Vec3 GBodyRigid::getPositionCOMGlobal() 
{ 
	return T_global * getPositionCOM(); 
}

Vec3 GBodyRigid::getVelocityCOMGlobal()
{
	return getVelocityLinearGlobal(getPositionCOM());
}

Vec3 GBodyRigid::getAccelerationCOMGlobal()
{
	return getAccelerationLinearGlobal(getPositionCOM());
}

dse3 GBodyRigid::getMomentum()
{
	return  I*V;
}

dse3 GBodyRigid::getMomentumGlobal()
{
	return dAd(Inv(T_global), I*V);
}

Vec3 GBodyRigid::getDerivative_PositionCOMGlobal_Dq(GCoordinate *pCoordinate_)
{
	if ( find(fJL.pCoordinates.begin(), fJL.pCoordinates.end(), pCoordinate_) == fJL.pCoordinates.end() ) return Vec3(0,0,0);
	se3 Ji = convert_to_se3(fJL.get_J(pCoordinate_));
	return T_global.GetRotation() * ( Cross(Ji.GetW(), getPositionCOM()) + Ji.GetV() );
}

dse3 GBodyRigid::getDerivative_MomentumGlobal_Dq(GCoordinate *pCoordinate_)
{
	list<GCoordinate *>::iterator iter_pcoord;
	se3 DVDqi, Ji;

	if ( find(fJL.pCoordinates.begin(), fJL.pCoordinates.end(), pCoordinate_) == fJL.pCoordinates.end() ) return dse3(0,0,0,0,0,0);

	Ji = convert_to_se3(fJL.get_J(pCoordinate_));

	DVDqi.SetZero();
	for (iter_pcoord = fJL.pCoordinates.begin(); iter_pcoord != fJL.pCoordinates.end(); iter_pcoord++) {
		if ( *iter_pcoord == pCoordinate_ ) break;
		DVDqi += ad( convert_to_se3(fJL.get_J(*iter_pcoord)), convert_to_se3(fJL.get_J(pCoordinate_)) ) * (*iter_pcoord)->dq;
	}

	return dAd(Inv(T_global), I*DVDqi - dad(Ji, I*V));
}

dse3 GBodyRigid::getDerivative_MomentumGlobal_Ddq(GCoordinate *pCoordinate_)
{
	return dAd(Inv(T_global), I * convert_to_se3(fJL.get_J(pCoordinate_)));
}

void GBodyRigid::setExternalForceLocally(const dse3 &Fe_local_)
{
	Fe = Fe_local_;
}

void GBodyRigid::setExternalForceGlobally(const dse3 &Fe_global_)
{
	Fe = dAd(T_global, Fe_global_);
}

void GBodyRigid::setExternalForceGlobally(const Vec3 &p_, const Vec3 &fg_)
{
	Vec3 f = ~getOrientationGlobal() * fg_;
	Fe = dse3(Cross(p_, f), f);
}

void GBodyRigid::addExternalForceLocally(const dse3 &Fe_local_)
{
	Fe += Fe_local_;
}

void GBodyRigid::addExternalForceGlobally(const dse3 &Fe_global_)
{
	Fe += dAd(T_global, Fe_global_);
}

void GBodyRigid::addExternalForceGlobally(const Vec3 &p_, const Vec3 &fg_)
{
	Vec3 f = ~getOrientationGlobal() * fg_;
	Fe += dse3(Cross(p_, f), f);
}

void GBodyRigid::neDynaRecursion_a()
{
	update_base_joint_info();

	update_T();
	update_V();				
	update_eta();
	update_dV(false);
}

void GBodyRigid::neDynaRecursion_b()
{
	update_F();
	update_tau();
}

void GBodyRigid::fsDynaRecursion_a()
{
	update_base_joint_info();

	update_T();
	update_V();				
	update_eta();
}

void GBodyRigid::fsDynaRecursion_b()
{
	update_aI();
	update_aB();
	update_Psi();
	update_Pi();
	update_beta();
}

void GBodyRigid::fsDynaRecursion_c()
{
	if ( pBaseJoint->isPrescribed() ) {
		update_dV(false);
		update_F_fs();
		update_tau();
	} else {
		update_ddq();
		update_dV(true);
		update_F_fs();
	}
}

void GBodyRigid::neDynaRecursion_DaDp()
{
	update_DVDp();				
	update_DetaDp();
	update_DdVDp();
}

void GBodyRigid::neDynaRecursion_DbDp()
{
	update_DFDp();
	update_DtauDp();
}

void GBodyRigid::fsDynaRecursion_DaDp()
{
	update_DVDp();				
	update_DetaDp();
}

void GBodyRigid::fsDynaRecursion_DbDp()
{
	update_DaIDp();
	update_DaBDp();
	update_DPsiDp();
	update_DPiDp();
	update_DbetaDp();
}

void GBodyRigid::fsDynaRecursion_DcDp()
{
	if ( pBaseJoint->isPrescribed() ) {
		update_DdVDp();
		update_DFDp_fs();
		update_DtauDp();
	} else {
		update_DddqDp();
		update_DdVDp();
		update_DFDp_fs();
	}
}

dse3 GBodyRigid::getTransformed_F()
{
	return dAd(invT, F);
}

AInertia GBodyRigid::getTransformed_aI()
{
	return Pi.Transform(invT);
}

dse3 GBodyRigid::getTransformed_aB()
{
	return dAd(invT, beta);
}

dse3 GBodyRigid::getTransformed_DFDp()
{
	dse3 tmp(DFDp);

	if ( bnzDhDp ) {
		tmp -= dad(DhDp, F);
	}
	
	return dAd(invT, tmp);
}

AInertia GBodyRigid::getTransformed_DaIDp()
{
	AInertia tmp(DPiDp);

	if ( bnzDhDp ) {
		RMatrix tmp2 = dad(DhDp, convert_to_RMatrix(Pi));
		tmp -= AInertia((tmp2 + ~tmp2).GetPtr());
	}

	return tmp.Transform(invT);
}

dse3 GBodyRigid::getTransformed_DaBDp()
{
	dse3 tmp(DbetaDp);

	if ( bnzDhDp ) {
		tmp -= dad(DhDp, beta);
	}

	return dAd(invT, tmp);
}

void GBodyRigid::setDifferentiatingVariable_Dq(GCoordinate *pCoordinate_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(); iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DqDp = 0.0;
		(*iter_pcoord)->DdqDp = 0.0;
		if ( pBaseJoint->isPrescribed() ) {
			(*iter_pcoord)->DddqDp = 0.0;
		} else {
			(*iter_pcoord)->DtauDp = 0.0;	/////////////////////////////////////////////// in case of spring, this is wrong!
		}
	}
	set_bnzAll(false);

	// if pCoordinate_ is included in pBaseJoint->pCoordinates
	if ( isIncluded(pCoordinate_) ) {

		pCoordinate_->DqDp = 1.0;
		set_bnzDqDp(true);

		if ( !(pBaseJoint->isConstantScrew()) ) {
			set_DSDp(get_DSDq(pCoordinate_));
			set_DdSDp(get_DdSDq(pCoordinate_));
			set_bnzDSDp(true);
			set_bnzDdSDp(true);
		}

		set_DhDp(get_S(pCoordinate_));
		set_bnzDhDp(true);		// DhDp = S*DqDp
	}
}

void GBodyRigid::setDifferentiatingVariable_Ddq(GCoordinate *pCoordinate_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(); iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DqDp = 0.0;
		(*iter_pcoord)->DdqDp = 0.0;
		if ( pBaseJoint->isPrescribed() ) {
			(*iter_pcoord)->DddqDp = 0.0;
		} else {
			(*iter_pcoord)->DtauDp = 0.0;	////////////////////////////////////////////////// in case of spring, this is wrong!
		}
	}
	set_bnzAll(false);

	// if pCoordinate_ is included in pBaseJoint->pCoordinates
	if ( isIncluded(pCoordinate_) ) {
		pCoordinate_->DdqDp = 1.0;
		set_bnzDdqDp(true);

		if ( !(pBaseJoint->isConstantScrew()) ) {
			set_DdSDp(get_DdSDdq(pCoordinate_));
			set_bnzDdSDp(true);
		}
	}
}

void GBodyRigid::setDifferentiatingVariable_Dddq(GCoordinate *pCoordinate_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(); iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DqDp = 0.0;
		(*iter_pcoord)->DdqDp = 0.0;
		if ( pBaseJoint->isPrescribed() ) {
			(*iter_pcoord)->DddqDp = 0.0;
		} else {
			(*iter_pcoord)->DtauDp = 0.0;
		}
	}
	set_bnzAll(false);

	set_bDpAlien(true);

	// if pCoordinate_ is included in pBaseJoint->pCoordinates
	if ( isIncluded(pCoordinate_) ) {
		if ( pBaseJoint->isPrescribed() ) {
			pCoordinate_->DddqDp = 1.0;
		}
	}
}

void GBodyRigid::setDifferentiatingVariable_Dtau(GCoordinate *pCoordinate_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(); iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DqDp = 0.0;
		(*iter_pcoord)->DdqDp = 0.0;
		if ( pBaseJoint->isPrescribed() ) {
			(*iter_pcoord)->DddqDp = 0.0;
		} else {
			(*iter_pcoord)->DtauDp = 0.0;
		}
	}
	set_bnzAll(false);

	set_bDpAlien(true);

	// if pCoordinate_ is included in pBaseJoint->pCoordinates
	if ( isIncluded(pCoordinate_) ) {
		if ( !pBaseJoint->isPrescribed() ) {
			pCoordinate_->DtauDp = 1.0;
		}
	}
}

void GBodyRigid::setDifferentiatingVariable_DFe(GBody *pBody_, int idx_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(); iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DqDp = 0.0;
		(*iter_pcoord)->DdqDp = 0.0;
		if ( pBaseJoint->isPrescribed() ) {
			(*iter_pcoord)->DddqDp = 0.0;
		} else {
			(*iter_pcoord)->DtauDp = 0.0;
		}
	}
	set_bnzAll(false);

	set_bDpAlien(true);

	if ( pBody_ == this && idx_ >= 0 && idx_ < 6 ) {
		bnzDFeDp = true;
		DFeDp.SetZero();
		DFeDp[idx_] = 1.0;
	}
}

void GBodyRigid::import_base_joint_info()
{
	Sdq.Ad(pBaseJoint->T_right, pBaseJoint->get_Sdq());
	dSdq.Ad(pBaseJoint->T_right, pBaseJoint->get_dSdq());
	Sddq.Ad(pBaseJoint->T_right, pBaseJoint->get_Sddq());
	DSdqDt = Sddq;
	DSdqDt += dSdq;
	Ad(S, pBaseJoint->T_right, pBaseJoint->get_S());
	Ad(dS, pBaseJoint->T_right, pBaseJoint->get_dS());
}

void GBodyRigid::update_base_joint_info()
{
	Sdq.Ad(pBaseJoint->T_right, pBaseJoint->get_Sdq());
	dSdq.Ad(pBaseJoint->T_right, pBaseJoint->get_dSdq());
	Sddq.Ad(pBaseJoint->T_right, pBaseJoint->get_Sddq());
	DSdqDt = Sddq;
	DSdqDt += dSdq;

	if ( !(pBaseJoint->isConstantScrew()) ) {
		Ad(S, pBaseJoint->T_right, pBaseJoint->get_S());
		Ad(dS, pBaseJoint->T_right, pBaseJoint->get_dS());
	}
}

void GBodyRigid::update_T()
{
	list<GJoint*>::iterator iter_pjoint;

	T = pBaseJoint->T_left;
	T *= pBaseJoint->get_T();
	T *= pBaseJoint->inv_T_right;

	invT.SetInvOf(T);
}

void GBodyRigid::update_V()
{
	if ( pBaseJoint->getDOF() <= 0 ) {
		V.Ad(invT, pParentBody->get_V());
	} else {
		V.Ad(invT, pParentBody->get_V());
		V += Sdq;
	}
}

void GBodyRigid::update_eta()
{
	if ( pBaseJoint->getDOF() <= 0 ) {
		eta.SetZero();
	} else {
		eta.ad(V, Sdq);
		eta += dSdq;
	}
}

void GBodyRigid::update_dV(bool b_update_)
{
	if ( pBaseJoint->getDOF() <= 0 ) {
		dV.Ad(invT, pParentBody->get_dV());
	} else {
		dV.Ad(invT, pParentBody->get_dV());
		if ( b_update_ ) {
			//dV += convert_to_se3(S * pBaseJoint->get_ddq());	// uses new joint acceleration 
			//dV += (S * pBaseJoint->get_ddq()).GetPtr();	// uses new joint acceleration 
			int n = pBaseJoint->getDOF();
			double ddq_[6], Sddq_[6];
			pBaseJoint->get_ddq(ddq_);
			multAB(Sddq_, S.GetPtr(), ddq_, 6, n, n, 1);
			dV += Sddq_;
		} else {
			dV += Sddq;
		}
		dV += eta;
	}
}

void GBodyRigid::update_F()
{
	list<GBody *>::iterator iter_pbody_child;

	F = I * dV;			// inertial force
	F -= dad(V, I * V);	// Coriolis force
	F -= Fe;			// external force

	// force from child bodies
	for (iter_pbody_child = pChildBodies.begin(); iter_pbody_child != pChildBodies.end(); iter_pbody_child++) {
		F += (*iter_pbody_child)->getTransformed_F();
	}
}

void GBodyRigid::update_F_fs()
{
	F = aI * dV + aB;
}

void GBodyRigid::update_aI()
{
	list<GBody *>::iterator iter_pbody_child;

	aI = AInertia(I);

	for (iter_pbody_child = pChildBodies.begin(); iter_pbody_child != pChildBodies.end(); iter_pbody_child++) {
		aI += (*iter_pbody_child)->getTransformed_aI();
	}
}

void GBodyRigid::update_aB()
{
	list<GBody *>::iterator iter_pbody_child;

	aB = -dad(V, I*V);
	aB -= Fe;

	for (iter_pbody_child = pChildBodies.begin(); iter_pbody_child != pChildBodies.end(); iter_pbody_child++) {
		aB += (*iter_pbody_child)->getTransformed_aB();
	}
}

void GBodyRigid::update_Psi()
{
	if ( pBaseJoint->isPrescribed() ) {
		Psi.SetZero(0,0);
	} else {
		if ( pBaseJoint->getDOF() <= 0 ) {
			Psi.SetZero(0,0);
		} else {
			Mult_AInertia_se3(aI_S.GetPtr(), aI, S.GetPtr(), S.ColSize()); // aI_S = aI * S
			if ( pBaseJoint->getDOF() == 1 ) {
				Psi[0] = 1./(se3(S.GetPtr()).InnerProductWith(aI_S.GetPtr()));
			} else {
				RMatrix tmp(S.ColSize(), S.ColSize());
				AtMultB(tmp, S, aI_S);	// tmp = ~S * aI_S
				Psi = Inv(tmp);			// Psi = Inv(~S * aI * S)
			}
		}
	}
}
void GBodyRigid::update_Pi()
{
	if ( pBaseJoint->isPrescribed() ) {
		Pi = aI;
	} else {
		// Pi = aI - aI*S*Psi*~S*~aI, aI is symmetric.
		Pi = aI;
		if ( pBaseJoint->getDOF() > 0 ) {
			if ( pBaseJoint->getDOF() == 1 ) {
				Pi.SubstractAlphaSSt(Psi[0], aI_S.GetPtr());
			} else {
				Pi -= (aI_S*Psi*~aI_S).GetPtr(); //Pi -= AInertia((aI_S*Psi*~aI_S).GetPtr());	
			}
		}
	}
}

void GBodyRigid::update_beta()
{
	if ( pBaseJoint->isPrescribed() ) {
		if ( pBaseJoint->getDOF() <= 0 ) {
			beta = aB + aI * eta;
		} else {
			beta = aB + aI * (eta + Sddq);
		}
	} else {
		if ( pBaseJoint->getDOF() <= 0 ) {
			beta = aB + aI * eta;
		} else {
			//beta = aB + aI * ( eta + convert_to_se3( S * Psi * ( pBaseJoint->get_tau() - ~S * convert_to_RMatrix( aI * eta + aB ) ) ) );
			int n = pBaseJoint->getDOF();
			double tau_[6], Psi_tau_[6], S_Psi_tau_[6]; // with asuming n <= 6
			dse3 aI_eta_aB_ = aI * eta + aB;
			pBaseJoint->get_tau(tau_);
			for (int i=0; i<n; i++) { tau_[i] -= aI_eta_aB_.InnerProductWith(&S[6*i]); } //this is more fast than "multAB(tau2_, aI_eta_aB_.GetArray(), S.GetPtr(), 1, 6, 6, n); for (int i=0; i<n; i++) { tau_[i] -= tau2_[i]; }"
			multAB(Psi_tau_, Psi.GetPtr(), tau_, n, n, n, 1);
			multAB(S_Psi_tau_, S.GetPtr(), Psi_tau_, 6, n, n, 1);
			beta = aI_eta_aB_ + aI * se3(S_Psi_tau_);
		}
	}
}

void GBodyRigid::update_tau()
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	if ( pBaseJoint->getDOF() <= 0 ) return;

	//RMatrix tau_ = ~S * convert_to_RMatrix(F);
	int n = pBaseJoint->getDOF();
	double tau_[6];
	if(S.GetPtr())
		for (int i=0; i<n; i++) { tau_[i] = F.InnerProductWith(&S[6*i]); }
	else 
		for (int i=0; i<n; i++) tau_[i]=0.0;

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(), i=0; iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++, i++) {
		(*iter_pcoord)->tau = tau_[i];
	}
}

void GBodyRigid::update_ddq()
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	if ( pBaseJoint->getDOF() <= 0 ) return;

	//RMatrix ddq_ = Psi * ( pBaseJoint->get_tau() - ~S * convert_to_RMatrix( aI * ( Ad(invT, pParentBody->get_dV()) + eta ) ) - ~S * convert_to_RMatrix(aB) );
	int n = pBaseJoint->getDOF();
	double tau_[6], ddq_[6];
	se3 eta_ = Ad(invT, pParentBody->get_dV()) + eta;
	dse3 aB_ = aI * eta_ + aB;
	pBaseJoint->get_tau(tau_);
	for (int i=0; i<n; i++) { tau_[i] -= aB_.InnerProductWith(&S[6*i]); }
	multAB(ddq_, Psi.GetPtr(), tau_, n, n, n, 1);

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(), i=0; iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++, i++) {
		(*iter_pcoord)->ddq = ddq_[i];
	}
}

void GBodyRigid::update_DVDp()
{
	if ( bDpAlien ) {
		DVDp.SetZero();
		return;
	}

	DVDp.Ad(invT, pParentBody->get_DVDp());
	
	if ( bnzDhDp ) {
		DVDp -= ad(DhDp, Ad(invT, pParentBody->get_V()));
	}

	if ( bnzDSDp ) {
		DVDp += convert_to_se3(DSDp * pBaseJoint->get_dq());
	}

	if ( bnzDdqDp ) {
		DVDp += convert_to_se3(S * pBaseJoint->get_DdqDp());
	}
}

void GBodyRigid::update_DetaDp()
{
	if ( bDpAlien ) {
		DetaDp.SetZero();
		return;
	}

	RMatrix tmp = Zeros(6,1);

	DetaDp.ad(DVDp, Sdq);

	if ( bnzDSDp && bnzDdqDp ) {
		tmp += ad(V, DSDp * pBaseJoint->get_dq() + S * pBaseJoint->get_DdqDp());
	} else if ( bnzDSDp && !bnzDdqDp ) {
		tmp += ad(V, DSDp * pBaseJoint->get_dq());
	} else if ( !bnzDSDp && bnzDdqDp ) {
		tmp += ad(V, S * pBaseJoint->get_DdqDp());
	} else {
		;
	}

	if ( bnzDdSDp ) {
		tmp += DdSDp * pBaseJoint->get_dq();
	}

	if ( bnzDdqDp ) {
		tmp += dS * pBaseJoint->get_DdqDp();
	}

	DetaDp += convert_to_se3(tmp);
}

void GBodyRigid::update_DdVDp()
{
	if ( bDpAlien ) {
		DdVDp.Ad(invT, pParentBody->get_DdVDp());
		DdVDp += convert_to_se3(S * pBaseJoint->get_DddqDp());
		return;
	}

	DdVDp.Ad(invT, pParentBody->get_DdVDp());

	if ( bnzDhDp ) {
		DdVDp -= ad(DhDp, Ad(invT, pParentBody->get_dV()));
	}

	if ( bnzDSDp ) {
		DdVDp += convert_to_se3(DSDp * pBaseJoint->get_ddq());
	}

	DdVDp += convert_to_se3(S * pBaseJoint->get_DddqDp());
	DdVDp += DetaDp;
}

void GBodyRigid::update_DFDp()
{
	list<GBody *>::iterator iter_pbody_child;

	if ( bDpAlien ) {
		DFDp = I * DdVDp;
		if ( bnzDFeDp ) { DFDp -= DFeDp; }
		for (iter_pbody_child = pChildBodies.begin(); iter_pbody_child != pChildBodies.end(); iter_pbody_child++) {
			DFDp += (*iter_pbody_child)->getTransformed_DFDp();
		}
		return;
	}

	dse3 tmp = I * DVDp;

	DFDp = I * DdVDp - dad(DVDp, I*V);

	if ( bnzDIDp ) {
		DFDp += DIDp * dV;
		tmp += DIDp * V;
	}

	DFDp -= dad(V, tmp);

	if ( bnzDFeDp ) {
		DFDp -= DFeDp;
	}

	for (iter_pbody_child = pChildBodies.begin(); iter_pbody_child != pChildBodies.end(); iter_pbody_child++) {
		DFDp += (*iter_pbody_child)->getTransformed_DFDp();
	}
}

void GBodyRigid::update_DFDp_fs()
{
	if ( bDpAlien ) {
		DFDp = aI * DdVDp;
		DFDp += DaBDp;	
		return;
	}

	DFDp = DaIDp * dV;
	DFDp += aI * DdVDp;
	DFDp += DaBDp;	
}

void GBodyRigid::update_DaIDp()
{
	if ( bDpAlien ) {
		DaIDp.SetZero();
		return;
	}

	list<GBody *>::iterator iter_pbody_child;

	DaIDp.SetZero();

	if ( bnzDIDp ) {
		DaIDp += DIDp;
	}

	for (iter_pbody_child = pChildBodies.begin(); iter_pbody_child != pChildBodies.end(); iter_pbody_child++) {
		DaIDp += (*iter_pbody_child)->getTransformed_DaIDp();
	}
}

void GBodyRigid::update_DaBDp()
{
	list<GBody *>::iterator iter_pbody_child;

	if ( bDpAlien ) {
		DaBDp.SetZero();
		if ( bnzDFeDp ) { 
			DaBDp -= DFeDp; 
		}
		for (iter_pbody_child = pChildBodies.begin(); iter_pbody_child != pChildBodies.end(); iter_pbody_child++) {
			DaBDp += (*iter_pbody_child)->getTransformed_DaBDp();
		}
		return;
	}

	dse3 tmp = I*DVDp;

	DaBDp = -dad(DVDp, I*V);

	if ( bnzDIDp ) {
		tmp += DIDp * V;
	}

	DaBDp -= dad(V, tmp);

	if ( bnzDFeDp ) {
		DaBDp -= DFeDp;
	}

	for (iter_pbody_child = pChildBodies.begin(); iter_pbody_child != pChildBodies.end(); iter_pbody_child++) {
		DaBDp += (*iter_pbody_child)->getTransformed_DaBDp();
	}
}

void GBodyRigid::update_DPsiDp()
{
	if ( bDpAlien ) {
		DPsiDp.SetZero(0,0);
		return;
	}

	if ( pBaseJoint->isPrescribed() ) {
		DPsiDp.SetZero(0,0);
	} else {
		if ( pBaseJoint->getDOF() <= 0 ) {
			DPsiDp.SetZero(0,0);
		} else {
			RMatrix tmp = ~S * convert_to_RMatrix(DaIDp) * S;
			if ( bnzDSDp ) {
                RMatrix tmp2 = ~DSDp * convert_to_RMatrix(aI) * S;
				tmp += tmp2 + ~tmp2;
			}
			DPsiDp = -Psi*tmp*Psi;
		}
	}
}

void GBodyRigid::update_DPiDp()
{
	if ( bDpAlien ) {
		DPiDp.SetZero();
		return;
	}

	if ( pBaseJoint->isPrescribed() ) {
		DPiDp = DaIDp;
	} else {
		if ( pBaseJoint->getDOF() <= 0 ) {
			DPiDp = DaIDp;
		} else {
			RMatrix tmp = ( convert_to_RMatrix(DaIDp) * S ) * Psi * ~aI_S;
			DPiDp = DaIDp;
			DPiDp -= AInertia((aI_S * DPsiDp * ~aI_S).GetPtr());
			DPiDp -= AInertia((tmp + ~tmp).GetPtr());
			if ( bnzDSDp ) {
				RMatrix tmp2 = ( convert_to_RMatrix(aI) * DSDp ) * Psi * ~aI_S;
				DPiDp -= AInertia((tmp2 + ~tmp2).GetPtr());
			}
		}
	}
}

void GBodyRigid::update_DbetaDp()
{
	if ( bDpAlien ) {
		if ( pBaseJoint->isPrescribed() ) {
			DbetaDp = DaBDp;
			if ( pBaseJoint->getDOF() > 0 ) { DbetaDp += aI * convert_to_se3(S * pBaseJoint->get_DddqDp()); }
		} else {
			DbetaDp = DaBDp;
			if ( pBaseJoint->getDOF() > 0 ) {
				RMatrix S_Psi = S * Psi;
				RMatrix tmp3 = pBaseJoint->get_DtauDp() - ~S * convert_to_RMatrix(DaBDp);
				DbetaDp += aI * convert_to_se3(S_Psi * tmp3);
			}
		}
		return;
	}

	if ( pBaseJoint->isPrescribed() ) {
		if ( pBaseJoint->getDOF() <= 0 ) {
			DbetaDp = DaBDp;
			DbetaDp += DaIDp * eta;
			DbetaDp += aI * DetaDp;
		} else {
			se3 tmp(DetaDp);
			tmp += convert_to_se3(S * pBaseJoint->get_DddqDp());
			if ( bnzDSDp ) {
				tmp += convert_to_se3(DSDp * pBaseJoint->get_ddq());
			}
			DbetaDp = DaBDp;
			DbetaDp += DaIDp * (eta + Sddq);
			DbetaDp += aI * tmp;
		}
	} else {
		if ( pBaseJoint->getDOF() <= 0 ) {
			DbetaDp = DaBDp;
			DbetaDp += DaIDp * eta;
			DbetaDp += aI * DetaDp;
		} else {
			dse3 aI_eta_aB = aI * eta + aB;
			RMatrix S_Psi = S * Psi;
			RMatrix S_DPsiDp = S * DPsiDp;
			RMatrix tmp = pBaseJoint->get_tau() - ~S * convert_to_RMatrix(aI_eta_aB);
			RMatrix tmp2 = S_DPsiDp;
			RMatrix tmp3 = pBaseJoint->get_DtauDp() - ~S * convert_to_RMatrix(DaIDp * eta + aI * DetaDp + DaBDp);
			if ( bnzDSDp ) {
				tmp2 += DSDp * Psi;
				tmp3 -= ~DSDp * convert_to_RMatrix(aI_eta_aB);
			}
			DbetaDp = DaBDp;
			DbetaDp += DaIDp * (eta + convert_to_se3(S_Psi * tmp));
			DbetaDp += aI * (DetaDp + convert_to_se3(tmp2 * tmp + S_Psi * tmp3));
		}
	}
}

void GBodyRigid::update_DtauDp()
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	if ( pBaseJoint->getDOF() <= 0 ) return;

	RMatrix _DtauDp = ~S * convert_to_RMatrix(DFDp);
	if ( bnzDSDp ) {
		_DtauDp += ~DSDp * convert_to_RMatrix(F);
	}

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(), i=0; iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++, i++) {
		(*iter_pcoord)->DtauDp = _DtauDp[i];
	}
}

void GBodyRigid::update_DddqDp()
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	if ( pBaseJoint->getDOF() <= 0 ) return;

	if ( bDpAlien ) {
		RMatrix tmp1 = pBaseJoint->get_DtauDp() - ~S * convert_to_RMatrix(DaBDp);
		se3 tmp3; tmp3.Ad(invT, pParentBody->get_DdVDp());
		RMatrix _DddqDp = Psi * ( tmp1 - ~S * convert_to_RMatrix(aI * tmp3) );
		for (iter_pcoord = pBaseJoint->pCoordinates.begin(), i=0; iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++, i++) {
			(*iter_pcoord)->DddqDp = _DddqDp[i];
		}
		return;
	}

	se3 Ad_dV; Ad_dV.Ad(invT, pParentBody->get_dV());
	se3 Ad_dV_eta = Ad_dV + eta;

	// tmp1 = DtauDp - ~DSDp * aB - ~S * DaBDp;
	// tmp2 = (~DSDp * aI + ~S * DaIDp) * (Ad(invT, pParentBody->get_dV()) + eta);
	// tmp3 = Ad(invT, pParentBody->get_DdVDp()) - ad(DhDp, Ad(invT, pParentBody->get_dV())) + DetaDp;
	RMatrix tmp1 = pBaseJoint->get_DtauDp() - ~S * convert_to_RMatrix(DaBDp);
	RMatrix tmp2 = ~S * convert_to_RMatrix(DaIDp * Ad_dV_eta);
	se3 tmp3; tmp3.Ad(invT, pParentBody->get_DdVDp()); tmp3 += DetaDp;
	if ( bnzDSDp ) {
		tmp1 -= ~DSDp * convert_to_RMatrix(aB);
		tmp2 += ~DSDp * convert_to_RMatrix(aI * Ad_dV_eta);
	}
	if ( bnzDhDp ) {
		tmp3 -= ad(DhDp, Ad_dV);
	}

	// DddqDp
	RMatrix _DddqDp = DPsiDp * ( pBaseJoint->get_tau() - ~S * convert_to_RMatrix(aI * Ad_dV_eta + aB) );
	_DddqDp += Psi * ( tmp1 - tmp2 - ~S * convert_to_RMatrix(aI * tmp3) );

	for (iter_pcoord = pBaseJoint->pCoordinates.begin(), i=0; iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++, i++) {
		(*iter_pcoord)->DddqDp = _DddqDp[i];
	}
}

se3 GBodyRigid::get_S(GCoordinate *pCoordinate_)
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	for (i=0, iter_pcoord = pBaseJoint->pCoordinates.begin(); iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++, i++) {
		if ( *iter_pcoord == pCoordinate_ ) { return se3(S(0,i), S(1,i), S(2,i), S(3,i), S(4,i), S(5,i)); }
	}

	return se3(0,0,0,0,0,0);
}

se3 GBodyRigid::get_dS(GCoordinate *pCoordinate_)
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	for (i=0, iter_pcoord = pBaseJoint->pCoordinates.begin(); iter_pcoord != pBaseJoint->pCoordinates.end(); iter_pcoord++, i++) {
		if ( *iter_pcoord == pCoordinate_ ) { return se3(dS(0,i), dS(1,i), dS(2,i), dS(3,i), dS(4,i), dS(5,i)); }
	}

	return se3(0,0,0,0,0,0);
}

RMatrix GBodyRigid::get_DSDq(GCoordinate *pCoordinate_)
{
	if ( isIncluded(pCoordinate_) ) {
		return Ad(pBaseJoint->T_right, pBaseJoint->get_DSDq(pCoordinate_));
	} else {
		return Zeros(6, pBaseJoint->getDOF());
	}
}

RMatrix GBodyRigid::get_DdSDq(GCoordinate *pCoordinate_)
{
	if ( isIncluded(pCoordinate_) ) {
		return Ad(pBaseJoint->T_right, pBaseJoint->get_DdSDq(pCoordinate_));
	} else {
		return Zeros(6, pBaseJoint->getDOF());
	}
}

RMatrix GBodyRigid::get_DdSDdq(GCoordinate *pCoordinate_)
{
	return get_DSDq(pCoordinate_);
}

bool GBodyRigid::isIncluded(GCoordinate *pCoordinate_)
{
	if ( find(pBaseJoint->pCoordinates.begin(), pBaseJoint->pCoordinates.end(), pCoordinate_) != pBaseJoint->pCoordinates.end() ) 
		return true;
	else
		return false;
}

string GBodyRigid::getInfoStr()
{
	stringstream sstr;
	list<GJoint *>::iterator iter_pjoint;
	list<GBody *>::iterator iter_pbody;

	sstr << GBody::getInfoStr();
	sstr << "GBodyRigid::" << endl;
	sstr << "mass = " << getMass() << endl;
	sstr << "center of mass (local)  = " << getPositionCOM();
	sstr << "center of mass (global) = " << getPositionCOMGlobal();
	sstr << "generalized momentum (local)  = " << getMomentum();
	sstr << "generalized momentum (global) = " << getMomentumGlobal();
	sstr << "T_global = " << T_global;
	sstr << "I = " << I;
	sstr << "Fe = " << Fe;
	sstr << "T = " << T;
	sstr << "invT = " << invT;
	sstr << "V = " << V;
	sstr << "dV = " << dV;
	sstr << "F = " << F;
	sstr << "aI = " << aI;
	sstr << "aB = " << aB;
	sstr << "eta = " << eta;
	sstr << "Psi = " << Psi;
	sstr << "Pi = " << Pi;
	sstr << "beta = " << beta;

	return sstr.str();
}
