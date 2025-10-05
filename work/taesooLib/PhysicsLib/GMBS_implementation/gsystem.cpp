#include <algorithm>
#include "gelement.h"
#include "gsystem.h"
#include "gbody.h"
#include "gjoint.h"
#include "gcoordinate.h"
#include "gfunc.h"
#include "liegroup_rmatrix3_ext.h"

using namespace std;

//=============================================================
//                 GSystem
//=============================================================


GSystem::GSystem()
{
	pGround = NULL;
}


bool GSystem::setGravity(Vec3 g_)
{
	if ( pGround == NULL ) return false;

	pGround->set_dV(se3(Vec3(0,0,0),-g_));

	return true;
}

Vec3 GSystem::getGravity()
{
	return -pGround->get_dV().GetV();
}

GJoint* GSystem::getJoint(string name_)
{
	list<GJoint*>::iterator iter_pjoint;
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		if ( (*iter_pjoint)->getName() == name_ ) return *iter_pjoint;
	}
	return NULL;
}

GJoint* GSystem::getJoint(int idx_)
{
	if ( idx_ >= pJoints.size() ) return NULL;
	list<GJoint*>::iterator iter_pjoint = pJoints.begin();
	std::advance(iter_pjoint, idx_); 
	return *iter_pjoint;
}

GBody* GSystem::getBody(string name_)
{
	list<GBody*>::iterator iter_pbody;
	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		if ( (*iter_pbody)->getName() == name_ ) return *iter_pbody;
	}
	return NULL;
}

GBody* GSystem::getBody(int idx_)
{
	if ( idx_ >= pBodies.size() ) return NULL;
	list<GBody*>::iterator iter_pbody = pBodies.begin();
	std::advance(iter_pbody, idx_);
	return *iter_pbody;
}

int GSystem::getIndexJoint(string name_)
{
	int i;
	list<GJoint*>::iterator iter_pjoint;
	for (i=0, iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++, i++) {
		if ( (*iter_pjoint)->getName() == name_ ) return i;
	}
	return -1;
}

int GSystem::getIndexBody(string name_)
{
	int i;
	list<GBody*>::iterator iter_pbody;
	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++, i++) {
		if ( (*iter_pbody)->getName() == name_ ) return i;
	}
	return -1;
}

double GSystem::getMass()
{
	double mass = 0;
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		mass += (*iter_pbody)->getMass();
	//	printf("mass:%f\n", (*iter_pbody)->getMass());
	}
	
	return mass;
}

Vec3 GSystem::getPositionCOMGlobal()
{
	Vec3 p(0,0,0);
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		p += (*iter_pbody)->getMass() * (*iter_pbody)->getPositionCOMGlobal();
	}
	p *= (1./getMass());

	return p;
}

Vec3 GSystem::getVelocityCOMGlobal()
{
	Vec3 v(0,0,0);
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		v += (*iter_pbody)->getMass() * (*iter_pbody)->getVelocityCOMGlobal();
	}
	v *= (1./getMass());

	return v;
}

Vec3 GSystem::getAccelerationCOMGlobal()
{
	Vec3 a(0,0,0);
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		a += (*iter_pbody)->getMass() * (*iter_pbody)->getAccelerationCOMGlobal();
	}
	a *= (1./getMass());

	return a;
}

dse3 GSystem::getMomentumGlobal()
{
	dse3 H(0,0,0,0,0,0);
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		H += (*iter_pbody)->getMomentumGlobal();
	}

	return H;
}

dse3 GSystem::getMomentumCOM()
{
	return dAd(SE3(getPositionCOMGlobal()), getMomentumGlobal());
}

Vec3 GSystem::getDerivative_PositionCOMGlobal_Dq_2(GCoordinate *pCoordinate_)
{
	Vec3 DpDq(0,0,0), w, v, c;
	list<GBody *>::iterator iter_pbody;

	int idx = getIndexOfCoordinate(pCoordinate_);
	if ( idx < 0 ) return Vec3(0,0,0);

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {

		w[0] = (*iter_pbody)->Jacobian(0,idx);  v[0] = (*iter_pbody)->Jacobian(3,idx);
		w[1] = (*iter_pbody)->Jacobian(1,idx);	v[1] = (*iter_pbody)->Jacobian(4,idx);
		w[2] = (*iter_pbody)->Jacobian(2,idx);	v[2] = (*iter_pbody)->Jacobian(5,idx);

		// taesoo's comment: convert joint twist written with respect to the body frame 
		//          to the COM frame using Ad(c^-1)
		//         (  I  -c  ) *(skew(w)  v) *  (I c)
		//         (  0   1  )  (0        0)    (0 1)
		//        =  (skew(w) cross(w,c)+v)
		//           (  0           0    )

		// Or using notation type 2:
		// Ad(c^-1)
		//               (   I      0   )(w)
		//               ( skew(-c) I   )(v)

		// J_com+= mass_i*R_i* Ad(c_i^-1)*J_i
		if ( w[0] != 0 || w[1] != 0 || w[2] != 0 || v[0] != 0 || v[1] != 0 || v[2] != 0 ) {
			c = (*iter_pbody)->getPositionCOM();
			DpDq += (*iter_pbody)->getMass() * ( (*iter_pbody)->T_global.GetRotation() * ( Cross(w, c) + v ) );
		} 
	}

	DpDq *= (1./getMass());

	return DpDq;
}

Vec3 GSystem::getDerivative_PositionCOMGlobal_Dq(GCoordinate *pCoordinate_)
{
	Vec3 DpDq(0,0,0);
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		DpDq += (*iter_pbody)->getMass() * (*iter_pbody)->getDerivative_PositionCOMGlobal_Dq(pCoordinate_);
	}
	DpDq *= (1./getMass());

	return DpDq;
}

dse3 GSystem::getDerivative_MomentumGlobal_Dq(GCoordinate *pCoordinate_)
{
	dse3 DHDq(0,0,0,0,0,0);
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		DHDq += (*iter_pbody)->getDerivative_MomentumGlobal_Dq(pCoordinate_);
	}

	return DHDq;
}

dse3 GSystem::getDerivative_MomentumGlobal_Ddq(GCoordinate *pCoordinate_)
{
	dse3 DHDdq(0,0,0,0,0,0);
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		DHDdq += (*iter_pbody)->getDerivative_MomentumGlobal_Ddq(pCoordinate_);
	}

	return DHDdq;
}

void GSystem::calcDerivative_PositionCOMGlobal_Dq(vector<GCoordinate *> pCoordinates_, RMatrix &DpDq_)
{
	int i;
	list<GBody *>::iterator iter_pbody;
	vector<GCoordinate *>::iterator iter_pcoord;
	vector<SE3> M(pBodies.size());
	vector<JOINTLOOP_CONSTRAINT_TYPE> jlc_type(pBodies.size());
	Vec3 DpDqi;

	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {

		// save previous settings
		M[i] = (*iter_pbody)->fJL.M1;
		jlc_type[i] = (*iter_pbody)->fJL.jointLoopConstraintType;
		
		// prerequisites for (*iter_pbody)->getDerivative_PositionCOMGlobal_Dq(...)
		(*iter_pbody)->fJL.setM(SE3());
		(*iter_pbody)->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		(*iter_pbody)->fJL.update_J();
	}

	// calculate the derivative
	DpDq_.SetZero(3, int(pCoordinates_.size()));
	for (i=0, iter_pcoord = pCoordinates_.begin(); iter_pcoord != pCoordinates_.end(); i++, iter_pcoord++) {
		DpDqi = getDerivative_PositionCOMGlobal_Dq(*iter_pcoord);
		DpDq_.Push(0, i, convert_to_RMatrix(DpDqi));
	}

	// restore the previous settings
	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {
		(*iter_pbody)->fJL.setM(M[i]);
		(*iter_pbody)->fJL.setJointLoopConstraintType(jlc_type[i]);
	}
}

void GSystem::calcDerivative_PositionCOMGlobal_Dq(RMatrix &DpDq_)
{
	int i;
	list<GBody *>::iterator iter_pbody;
	list<GCoordinate *>::iterator iter_pcoord;
	vector<SE3> M(pBodies.size());
	vector<JOINTLOOP_CONSTRAINT_TYPE> jlc_type(pBodies.size());
	Vec3 DpDqi;

	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {

		// save previous settings
		M[i] = (*iter_pbody)->fJL.M1;
		jlc_type[i] = (*iter_pbody)->fJL.jointLoopConstraintType;
		
		// prerequisites for (*iter_pbody)->getDerivative_PositionCOMGlobal_Dq(...)
		(*iter_pbody)->fJL.setM(SE3());
		(*iter_pbody)->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		(*iter_pbody)->fJL.update_J();
	}

	// calculate the derivative
	DpDq_.SetZero(3, getNumCoordinates());
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); i++, iter_pcoord++) {
		DpDqi = getDerivative_PositionCOMGlobal_Dq(*iter_pcoord);
		DpDq_.Push(0, i, convert_to_RMatrix(DpDqi));
	}

	// restore the previous settings
	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {
		(*iter_pbody)->fJL.setM(M[i]);
		(*iter_pbody)->fJL.setJointLoopConstraintType(jlc_type[i]);
	}
}

void GSystem::calcDerivative_PositionCOMGlobal_Dq_2(RMatrix &DpDq_)
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;
	Vec3 DpDqi;

	DpDq_.SetZero(3, getNumCoordinates());
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); i++, iter_pcoord++) {
		DpDqi = getDerivative_PositionCOMGlobal_Dq_2(*iter_pcoord);
		DpDq_(0,i) = DpDqi[0];
		DpDq_(1,i) = DpDqi[1];
		DpDq_(2,i) = DpDqi[2];
	}
}

void GSystem::calcDerivative_MomentumGlobal_Dq_Ddq(vector<GCoordinate*> pCoordinates_, RMatrix &DHgDq_, RMatrix &DHgDdq_)
{
	int i;
	list<GBody *>::iterator iter_pbody;
	vector<GCoordinate *>::iterator iter_pcoord;
	vector<SE3> M(pBodies.size());
	vector<JOINTLOOP_CONSTRAINT_TYPE> jlc_type(pBodies.size());
	dse3 DHDqi, DHDdqi;

	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {

		// save previous settings
		M[i] = (*iter_pbody)->fJL.M1;
		jlc_type[i] = (*iter_pbody)->fJL.jointLoopConstraintType;
		
		// prerequisites for (*iter_pbody)->getDerivative_MomentumGlobal_Dq(...) and (*iter_pbody)->getDerivative_MomentumGlobal_Ddq(...)
		(*iter_pbody)->fJL.setM(SE3());
		(*iter_pbody)->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		(*iter_pbody)->fJL.update_J();
	}

	// calculate the derivatives
	DHgDq_.SetZero(6, int(pCoordinates_.size()));
	DHgDdq_.SetZero(6, int(pCoordinates_.size()));
	for (i=0, iter_pcoord = pCoordinates_.begin(); iter_pcoord != pCoordinates_.end(); i++, iter_pcoord++) {
		DHDqi = getDerivative_MomentumGlobal_Dq(*iter_pcoord);
		DHDdqi = getDerivative_MomentumGlobal_Ddq(*iter_pcoord);
		DHgDq_.Push(0, i, convert_to_RMatrix(DHDqi));
		DHgDdq_.Push(0, i, convert_to_RMatrix(DHDdqi));
	}

	// restore the previous settings
	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {
		(*iter_pbody)->fJL.setM(M[i]);
		(*iter_pbody)->fJL.setJointLoopConstraintType(jlc_type[i]);
	}
}

void GSystem::calcDerivative_MomentumGlobal_Dq_Ddq(RMatrix &DHgDq_, RMatrix &DHgDdq_)
{
	int i;
	list<GBody *>::iterator iter_pbody;
	list<GCoordinate *>::iterator iter_pcoord;
	vector<SE3> M(pBodies.size());
	vector<JOINTLOOP_CONSTRAINT_TYPE> jlc_type(pBodies.size());
	dse3 DHDqi, DHDdqi;

	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {

		// save previous settings
		M[i] = (*iter_pbody)->fJL.M1;
		jlc_type[i] = (*iter_pbody)->fJL.jointLoopConstraintType;
		
		// prerequisites for (*iter_pbody)->getDerivative_MomentumGlobal_Dq(...) and (*iter_pbody)->getDerivative_MomentumGlobal_Ddq(...)
		(*iter_pbody)->fJL.setM(SE3());
		(*iter_pbody)->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		(*iter_pbody)->fJL.update_J();
	}

	// calculate the derivatives
	DHgDq_.SetZero(6, getNumCoordinates());
	DHgDdq_.SetZero(6, getNumCoordinates());
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); i++, iter_pcoord++) {
		DHDqi = getDerivative_MomentumGlobal_Dq(*iter_pcoord);
		DHDdqi = getDerivative_MomentumGlobal_Ddq(*iter_pcoord);
		DHgDq_.Push(0, i, convert_to_RMatrix(DHDqi));
		DHgDdq_.Push(0, i, convert_to_RMatrix(DHDdqi));
	}

	// restore the previous settings
	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {
		(*iter_pbody)->fJL.setM(M[i]);
		(*iter_pbody)->fJL.setJointLoopConstraintType(jlc_type[i]);
	}
}

void GSystem::calcDerivative_MomentumCOM_Dq_Ddq(vector<GCoordinate*> pCoordinates_, RMatrix &DHcDq_, RMatrix &DHcDdq_)
{
	int i;
	list<GBody *>::iterator iter_pbody;
	vector<GCoordinate *>::iterator iter_pcoord;
	vector<SE3> M(pBodies.size());
	vector<JOINTLOOP_CONSTRAINT_TYPE> jlc_type(pBodies.size());

	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {

		// save previous settings
		M[i] = (*iter_pbody)->fJL.M1;
		jlc_type[i] = (*iter_pbody)->fJL.jointLoopConstraintType;
		
		// prerequisites for (*iter_pbody)->getDerivative_MomentumGlobal_Dq(...) and (*iter_pbody)->getDerivative_MomentumGlobal_Ddq(...)
		(*iter_pbody)->fJL.setM(SE3());
		(*iter_pbody)->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		(*iter_pbody)->fJL.update_J();
	}

	// calculate the derivatives
	Vec3 p = getPositionCOMGlobal();
	dse3 Hg = getMomentumGlobal();
	RMatrix DHgDq(6, int(pCoordinates_.size())), DHgDdq(6, int(pCoordinates_.size()));
	dse3 DHgDqi, DHgDdqi;
	for (i=0, iter_pcoord = pCoordinates_.begin(); iter_pcoord != pCoordinates_.end(); i++, iter_pcoord++) {
		DHgDqi = getDerivative_MomentumGlobal_Dq(*iter_pcoord);
		DHgDdqi = getDerivative_MomentumGlobal_Ddq(*iter_pcoord);
		DHgDq.Push(0, i, convert_to_RMatrix(DHgDqi));
		DHgDdq.Push(0, i, convert_to_RMatrix(DHgDdqi));
	}
	RMatrix DdAdDq_Hg(6, int(pCoordinates_.size()));
	Vec3 DpDqi;
	for (i=0, iter_pcoord = pCoordinates_.begin(); iter_pcoord != pCoordinates_.end(); i++, iter_pcoord++) {
		DpDqi = getDerivative_PositionCOMGlobal_Dq(*iter_pcoord);
		DdAdDq_Hg.Push(0, i, convert_to_RMatrix(dAd(SE3(p), dad(se3(Vec3(0,0,0),DpDqi), Hg))));
	}
	DHcDq_ = dAd(SE3(p), DHgDq) + DdAdDq_Hg;
	DHcDdq_ = dAd(SE3(p), DHgDdq);

	// restore the previous settings
	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {
		(*iter_pbody)->fJL.setM(M[i]);
		(*iter_pbody)->fJL.setJointLoopConstraintType(jlc_type[i]);
	}
}

static void RMat_push(RMatrix& inout,int irow, int icol, dse3 const& s)
{
	for(int i=0; i<6; i++)
		inout(irow+i, icol)=s[i];
}
void GSystem::calcDerivative_MomentumCOM_Ddq(RMatrix &DHcDdq_)
{
	int i;
	list<GBody *>::iterator iter_pbody;
	list<GCoordinate *>::iterator iter_pcoord;
	vector<SE3> M(pBodies.size());
	vector<JOINTLOOP_CONSTRAINT_TYPE> jlc_type(pBodies.size());

	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {

		// save previous settings
		M[i] = (*iter_pbody)->fJL.M1;
		jlc_type[i] = (*iter_pbody)->fJL.jointLoopConstraintType;
		
		// prerequisites for (*iter_pbody)->getDerivative_MomentumGlobal_Dq(...) and (*iter_pbody)->getDerivative_MomentumGlobal_Ddq(...)
		(*iter_pbody)->fJL.setM(SE3());
		(*iter_pbody)->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		(*iter_pbody)->fJL.update_J();
	}

	// calculate the derivatives
	Vec3 p = getPositionCOMGlobal();
	//dse3 Hg = getMomentumGlobal();
	RMatrix DHgDdq(6, getNumCoordinates());
	dse3 DHgDdqi;
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); i++, iter_pcoord++) {
		DHgDdqi = getDerivative_MomentumGlobal_Ddq(*iter_pcoord);
		//DHgDdq.Push(0, i, convert_to_RMatrix(DHgDdqi));
		RMat_push(DHgDdq, 0, i, DHgDdqi);
	}
	//RMatrix DdAdDq_Hg(6, getNumCoordinates());
	//Vec3 DpDqi;
	//for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); i++, iter_pcoord++) {
	//	DpDqi = getDerivative_PositionCOMGlobal_Dq(*iter_pcoord);
	//	//DdAdDq_Hg.Push(0, i, convert_to_RMatrix(dAd(SE3(p), dad(se3(Vec3(0,0,0),DpDqi), Hg))));
	//	RMat_push(DdAdDq_Hg,0,i,dAd(SE3(p), dad(se3(Vec3(0,0,0),DpDqi), Hg)));
	//}
	DHcDdq_ = dAd(SE3(p), DHgDdq);

	// restore the previous settings
	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {
		(*iter_pbody)->fJL.setM(M[i]);
		(*iter_pbody)->fJL.setJointLoopConstraintType(jlc_type[i]);
	}
}
void GSystem::calcDerivative_MomentumCOM_Dq_Ddq(RMatrix &DHcDq_, RMatrix &DHcDdq_)
{
	int i;
	list<GBody *>::iterator iter_pbody;
	list<GCoordinate *>::iterator iter_pcoord;
	vector<SE3> M(pBodies.size());
	vector<JOINTLOOP_CONSTRAINT_TYPE> jlc_type(pBodies.size());

	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {

		// save previous settings
		M[i] = (*iter_pbody)->fJL.M1;
		jlc_type[i] = (*iter_pbody)->fJL.jointLoopConstraintType;
		
		// prerequisites for (*iter_pbody)->getDerivative_MomentumGlobal_Dq(...) and (*iter_pbody)->getDerivative_MomentumGlobal_Ddq(...)
		(*iter_pbody)->fJL.setM(SE3());
		(*iter_pbody)->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		(*iter_pbody)->fJL.update_J();
	}

	// calculate the derivatives
	Vec3 p = getPositionCOMGlobal();
	dse3 Hg = getMomentumGlobal();
	RMatrix DHgDq(6, getNumCoordinates()), DHgDdq(6, getNumCoordinates());
	dse3 DHgDqi, DHgDdqi;
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); i++, iter_pcoord++) {
		DHgDqi = getDerivative_MomentumGlobal_Dq(*iter_pcoord);
		DHgDdqi = getDerivative_MomentumGlobal_Ddq(*iter_pcoord);
		DHgDq.Push(0, i, convert_to_RMatrix(DHgDqi));
		DHgDdq.Push(0, i, convert_to_RMatrix(DHgDdqi));
	}
	RMatrix DdAdDq_Hg(6, getNumCoordinates());
	Vec3 DpDqi;
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); i++, iter_pcoord++) {
		DpDqi = getDerivative_PositionCOMGlobal_Dq(*iter_pcoord);
		DdAdDq_Hg.Push(0, i, convert_to_RMatrix(dAd(SE3(p), dad(se3(Vec3(0,0,0),DpDqi), Hg))));
	}
	DHcDq_ = dAd(SE3(p), DHgDq) + DdAdDq_Hg;
	DHcDdq_ = dAd(SE3(p), DHgDdq);

	// restore the previous settings
	for (i=0, iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); i++, iter_pbody++) {
		(*iter_pbody)->fJL.setM(M[i]);
		(*iter_pbody)->fJL.setJointLoopConstraintType(jlc_type[i]);
	}
}

void GSystem::setDeriv_Dq(GCoordinate *pCoordinate_)
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->set_bDpAlien(false);
		(*iter_pbody)->setDifferentiatingVariable_Dq(pCoordinate_);
	}
}

void GSystem::setDeriv_Ddq(GCoordinate *pCoordinate_)
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->set_bDpAlien(false);
		(*iter_pbody)->setDifferentiatingVariable_Ddq(pCoordinate_);
	}
}

void GSystem::setDeriv_Dddq(GCoordinate *pCoordinate_)
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->set_bDpAlien(false);
		(*iter_pbody)->setDifferentiatingVariable_Dddq(pCoordinate_);
	}
}

void GSystem::setDeriv_Dtau(GCoordinate *pCoordinate_)
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->set_bDpAlien(false);
		(*iter_pbody)->setDifferentiatingVariable_Dtau(pCoordinate_);
	}
}

void GSystem::setDeriv_DFe(GBody *pBody_, int idx_)
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->set_bDpAlien(false);
		(*iter_pbody)->setDifferentiatingVariable_DFe(pBody_, idx_);
	}
}

bool GSystem::buildSystem(GBody *pGround_)
{
	pCoordinates.clear();
	pBodies.clear();
	pJoints.clear();
	
	pGround = pGround_;
	
	if ( !_scanBodiesAndJoints(pGround_) ) return false;

	pBodies.pop_front();	// pGround is popped out.

	if ( !_scanCoordinates() ) return false;

	if ( !_findParentChildRelation() ) return false;

	if ( !_findFwdJointLoopForBodies() ) return false;

	if ( !_getReady() ) return false;

	return true;
}

void GSystem::updateKinematics()
{
	update_joint_local_info();

	neFwdRecursion_a();

	updateGlobalLocationsOfBodiesAndJoints();
}

// body Jacobian matrices
void GSystem::updateJacobian()
{
	int idx;
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		//(*iter_pbody)->Jacobian.ReSize(6, getNumCoordinates());
		(*iter_pbody)->Jacobian.SetZero(6, getNumCoordinates());
	}

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		if ( (*iter_pbody)->pBaseJoint->getNumCoordinates() > 0 ) {
			idx = getIndexOfCoordinate(*((*iter_pbody)->pBaseJoint->pCoordinates.begin()));
			if ( idx >= 0 ) {
				(*iter_pbody)->Jacobian.Push(0, idx, (*iter_pbody)->get_S());
				update_Jacobian_child_bodies(*iter_pbody, idx, (*iter_pbody)->get_S());
			}
		}
	}
}

// taesoo: refer to Murray eqn (3.55). push all descendents' jacobians represented in the pbody frame.
void GSystem::update_Jacobian_child_bodies(GBody *pbody_, int idx_, RMatrix S_)
{
	list<GBody *>::iterator iter_pbody;
	RMatrix S2;
	for (iter_pbody = pbody_->pChildBodies.begin(); iter_pbody != pbody_->pChildBodies.end(); iter_pbody++) {
		S2 = Ad((*iter_pbody)->invT, S_);
		(*iter_pbody)->Jacobian.Push(0, idx_, S2);
		update_Jacobian_child_bodies(*iter_pbody, idx_, S2);	// recursive call until reaching end of branch
	}
}

void GSystem::calcDynamics(bool update_global_location_)
{
	update_joint_local_info();

	fsFwdRecursion_a();
	fsBwdRecursion_b();
	fsFwdRecursion_c();

	if ( update_global_location_ ) {
		updateGlobalLocationsOfBodiesAndJoints();
	}
}

void GSystem::diffDynamics()
{
	fsFwdRecursion_DaDp();
	fsBwdRecursion_DbDp();
	fsFwdRecursion_DcDp();
}

void GSystem::initExternalForce()
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->initExternalForce();
	}
}

void GSystem::update_joint_local_info_short()
{
	list<GJoint *>::iterator iter_pjoint;

	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		(*iter_pjoint)->update_short();
	}
}

void GSystem::update_joint_local_info()
{
	list<GJoint *>::iterator iter_pjoint;

	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		(*iter_pjoint)->update();
	}
}

void GSystem::updateGlobalLocationsOfBodiesAndJoints()
{
	list<GBody *>::iterator iter_pbody;
	list<GJoint *>::iterator iter_pjoint;

	// update global body location
	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->T_global = (*iter_pbody)->pParentBody->T_global;
		(*iter_pbody)->T_global *= (*iter_pbody)->T;
	}

	// update global joint location
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		(*iter_pjoint)->T_global = (*iter_pjoint)->pLeftBody->T_global;
		(*iter_pjoint)->T_global *= (*iter_pjoint)->T_left;
	}
}

void GSystem::neFwdRecursion_a()
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->neDynaRecursion_a();
	}
}

void GSystem::neBwdRecursion_b()
{
	list<GBody *>::reverse_iterator riter_pbody;

	for (riter_pbody = pBodies.rbegin(); riter_pbody != pBodies.rend(); riter_pbody++) {
		(*riter_pbody)->neDynaRecursion_b();
	}
}

void GSystem::fsFwdRecursion_a()
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->fsDynaRecursion_a();
	}
}

void GSystem::fsBwdRecursion_b()
{
	list<GBody *>::reverse_iterator riter_pbody;

	for (riter_pbody = pBodies.rbegin(); riter_pbody != pBodies.rend(); riter_pbody++) {
		(*riter_pbody)->fsDynaRecursion_b();
	}
}

void GSystem::fsFwdRecursion_c()
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->fsDynaRecursion_c();
	}
}

void GSystem::fsFwdRecursion_DaDp()
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->fsDynaRecursion_DaDp();
	}
}

void GSystem::fsBwdRecursion_DbDp()
{
	list<GBody *>::reverse_iterator riter_pbody;

	for (riter_pbody = pBodies.rbegin(); riter_pbody != pBodies.rend(); riter_pbody++) {
		(*riter_pbody)->fsDynaRecursion_DbDp();
	}
}

void GSystem::fsFwdRecursion_DcDp()
{
	list<GBody *>::iterator iter_pbody;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->fsDynaRecursion_DcDp();
	}

}

bool GSystem::_scanBodiesAndJoints(GBody *pbody_)
{
	GJoint *pjoint_;
	GBody *pbody_next_;
	list<GJoint*>::iterator iter_pjoint;
	
	if ( pbody_ == NULL ) return false;
	if ( find(pBodies.begin(), pBodies.end(), pbody_) != pBodies.end() ) return false;

	// add pbody_ to system
	pBodies.push_back(pbody_);
	
	for (iter_pjoint = pbody_->pJoints.begin(); iter_pjoint != pbody_->pJoints.end(); iter_pjoint++) {

		if ( (*iter_pjoint) == NULL ) return false;

		pjoint_ = *iter_pjoint;

		if ( find(pJoints.begin(), pJoints.end(), pjoint_) == pJoints.end() ) {
		
			// if needed, swap the bodies of pjoint_
			if ( pjoint_->pRightBody == pbody_ ) { 
				if ( !pjoint_->reverse() ) return false; 
			}

			// add pjoint_ to system
			pJoints.push_back(pjoint_);

			pbody_next_ = pjoint_->pRightBody;

			if ( find(pBodies.begin(), pBodies.end(), pbody_next_) == pBodies.end() ) {
				pjoint_->bCut = false;
				if ( !_scanBodiesAndJoints(pbody_next_) ) return false;
			} else {
				pjoint_->bCut = true;
			}
		}

	}

	return true;
}


bool GSystem::_scanCoordinates()
{
	list<GBody *>::iterator iter_pbody;
	list<GJoint *>::iterator iter_pjoint;
	list<GCoordinate *>::iterator iter_pcoord;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		for (iter_pcoord = (*iter_pbody)->pCoordinates.begin(); iter_pcoord != (*iter_pbody)->pCoordinates.end(); iter_pcoord++) {
			if ( *iter_pcoord == NULL ) return false;
			if ( find(pCoordinates.begin(), pCoordinates.end(), *iter_pcoord) != pCoordinates.end() ) return false;
			pCoordinates.push_back(*iter_pcoord);
		}
	}

	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		for (iter_pcoord = (*iter_pjoint)->pCoordinates.begin(); iter_pcoord != (*iter_pjoint)->pCoordinates.end(); iter_pcoord++) {
			if ( *iter_pcoord == NULL ) return false;
			if ( find(pCoordinates.begin(), pCoordinates.end(), *iter_pcoord) != pCoordinates.end() ) return false;
			pCoordinates.push_back(*iter_pcoord);
		}
	}

	for(iter_pcoord=pCoordinates.begin(); iter_pcoord!=pCoordinates.end(); iter_pcoord++)
	{
		(*iter_pcoord)->system_idx_coord=GElement::getIndexOfCoordinate(*iter_pcoord);
	}
	return true;
}

#include <stdexcept>
int GSystem::getIndexOfCoordinate(GCoordinate *pcoord_) // taesoo : gsystem class override this function to speed up the search.
{
	if(pcoord_->system_idx_coord==-1) throw std::runtime_error("coordinates not yet scanned");
	return pcoord_->system_idx_coord;
}

bool GSystem::_findParentChildRelation()
{
	list<GBody *>::iterator iter_pbody;
	list<GJoint *>::iterator iter_pjoint;

	// find the base joint, parent body, and child bodies of pBodies
	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {

		(*iter_pbody)->pChildBodies.clear();

		for (iter_pjoint = (*iter_pbody)->pJoints.begin(); iter_pjoint != (*iter_pbody)->pJoints.end(); iter_pjoint++) {

			if ( !(*iter_pjoint)->isCut() ) {

				if ( (*iter_pjoint)->pRightBody == (*iter_pbody) ) {

					(*iter_pbody)->pBaseJoint = *iter_pjoint;

					(*iter_pbody)->pParentBody = (*iter_pjoint)->pLeftBody;

				} else if ( (*iter_pjoint)->pLeftBody == (*iter_pbody) ) {

					(*iter_pbody)->pChildBodies.push_back((*iter_pjoint)->pRightBody);

				} else {

					return false;
				}
			}
		}
	}

	return true;
}

bool GSystem::_findFwdJointLoopForBodies()
{
	list<GBody *>::iterator iter_pbody;
	list<GJoint *> pjoints;

	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		pjoints.clear();
		if ( !gmbsTraceJointsBackward((*iter_pbody)->pBaseJoint, pGround, pjoints) ) return false;
		(*iter_pbody)->fJL.setJoints(pjoints);
	}

	return true;
}

bool GSystem::_getReady()
{
	list<GJoint *>::iterator iter_pjoint;
	list<GBody *>::iterator iter_pbody;

	// check joints and bodies (This should be placed after _findParentChildRelation().)
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		if ( !(*iter_pjoint)->getReady() ) return false;
	}
	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		if ( !(*iter_pbody)->getReady() ) return false;
	}

	//----- update kinematics ------
	// update local joint information
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		(*iter_pjoint)->update();
	}
	// load joint information on each body (forcefully)
	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		(*iter_pbody)->import_base_joint_info();	
	}
	// update kinematics (body pose, velocity, and acceleration)
	neFwdRecursion_a();
	// update global locations of bodies and joints
	updateGlobalLocationsOfBodiesAndJoints();

	return true;
}

string GSystem::getInfoStr()
{
	stringstream sstr;
	list<GBody *>::iterator iter_pbody;
	list<GJoint *>::iterator iter_pjoint;

	sstr << "GSystem:: " << GElement::getInfoStr();
	sstr << "    number of bodies included = " << int(pBodies.size()) << endl;
	sstr << "    number of joints included = " << int(pJoints.size()) << endl;
	sstr << "    number of coordinates = " << getNumCoordinates() << endl;
	sstr << "    ground body = " << pGround->getName() << endl;
	sstr << "--------------------------------------------------------------------" << endl;
	sstr << "         Bodies" << endl;
	sstr << "--------------------------------------------------------------------" << endl;
	for (iter_pbody = pBodies.begin(); iter_pbody != pBodies.end(); iter_pbody++) {
		sstr << (*iter_pbody)->getInfoStr();
		sstr << "----------" << endl;
	}
	sstr << "--------------------------------------------------------------------" << endl;
	sstr << "         Joints" << endl;
	sstr << "--------------------------------------------------------------------" << endl;
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		sstr << (*iter_pjoint)->getInfoStr();
		sstr << "----------" << endl;
	}

	return sstr.str();
}

