#include <list>
#include <string>
#include <sstream>
#include "gjoint.h"
#include "gelement.h"
#include "gbody.h"
#include "gcoordinate.h"
#include "liegroup.h"

using namespace std;


//=============================================================
//                 GJoint
//=============================================================
GJoint::GJoint()
{
	jointType = GJOINT_NULL;
	pLeftBody = pRightBody = NULL;
	T_left.SetIdentity();
	T_right.SetIdentity();
	inv_T_left.SetIdentity();
	inv_T_right.SetIdentity();
	T_global.SetIdentity();
	T.SetIdentity();
	inv_T.SetIdentity();
	Sdq.SetZero();
	dSdq.SetZero();
	Sddq.SetZero();
	DSdqDt.SetZero();
	bReversed = false;
	bCut = false;
	setPrescribed(false);
}

bool GJoint::connectBodies(GBody *pLeftBody_, GBody *pRightBody_)
{
	if ( pLeftBody_ == NULL || pRightBody_ == NULL ) return false;

	pLeftBody = pLeftBody_;
	pRightBody = pRightBody_;
	bReversed = false;
	bCut = false;
	if ( !pLeftBody->addJoint(this) ) return false;
	if ( !pRightBody->addJoint(this) ) return false;
	return true;
}

void GJoint::disconnectBodies()
{
	if ( pLeftBody != NULL ) {
		pLeftBody->removeJoint(this);
		pLeftBody = NULL;
	}
	if ( pRightBody != NULL ) {
		pRightBody->removeJoint(this);
		pRightBody = NULL;
	}
	bReversed = false;
	bCut = false;
}

void GJoint::setPosition(const Vec3 &pL_, const Vec3 &pR_)
{
	T_left.SetPosition(pL_);
	T_right.SetPosition(pR_);
	inv_T_left.SetInvOf(T_left);
	inv_T_right.SetInvOf(T_right);
}

void GJoint::setOrientation(const SO3 &RL_, const SO3 &RR_)
{
	T_left.SetRotation(RL_);
	T_right.SetRotation(RR_);
	inv_T_left.SetInvOf(T_left);
	inv_T_right.SetInvOf(T_right);
}

void GJoint::setPositionAndOrientation(const SE3 &TL_, const SE3 &TR_)
{
	T_left = TL_;
	T_right = TR_;
	inv_T_left.SetInvOf(T_left);
	inv_T_right.SetInvOf(T_right);
}

// set pCoordinates[]->bPrescribed
void GJoint::setPrescribed(bool b_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->bPrescribed = b_;
	}

	bPrescribed = b_;
}

bool GJoint::reverse()
{
	GBody *pbody_tmp;
	SE3 T_tmp;

	// swap pLeftBody and pRightBody
	pbody_tmp = pLeftBody;
	pLeftBody = pRightBody;
	pRightBody = pbody_tmp;

	// swap T_left and T_right
	T_tmp = T_left;
	T_left = T_right;
	T_right = T_tmp;

	// swap inv_T_left and inv_T_right
	T_tmp = inv_T_left;
	inv_T_left = inv_T_right;
	inv_T_right = T_tmp;

	// set flag
	bReversed = (bReversed == true ? false : true );

	return true;
}

string GJoint::getInfoStr()
{
	stringstream sstr;
	list<GCoordinate *>::iterator iter_pcoord;
	string body_name_1, body_name_2;
	if ( pLeftBody == NULL ) {
		body_name_1 = "NULL";
	} else {
		body_name_1 = pLeftBody->getName();
	}
	if ( pRightBody == NULL ) {
		body_name_2 = "NULL";
	} else {
		body_name_2 = pRightBody->getName();
	}

	sstr << GElement::getInfoStr();
	sstr << "GJoint:: " << endl;
	sstr << "    bodies connected = (" << body_name_1 << ", " << body_name_2 << ")" << endl;
	sstr << "    is reversed? " << (isReversed() ? "yes" : "no") << endl;
	sstr << "    is cut? " << (isCut() ? "yes" : "no") << endl;
	sstr << "    is prescribed? " << (isPrescribed() ? "yes" : "no") << endl;
	sstr << "    d.o.f. = " << getDOF() << endl;
	sstr << "    q = {";
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		sstr << (*iter_pcoord)->q << ", ";
	}
	sstr << "}" << endl;
	sstr << "    dq = {";
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		sstr << (*iter_pcoord)->dq << ", ";
	}
	sstr << "}" << endl;
	sstr << "    ddq = {";
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		sstr << (*iter_pcoord)->ddq << ", ";
	}
	sstr << "}" << endl;
	sstr << "    tau = {";
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		sstr << (*iter_pcoord)->tau << ", ";
	}
	sstr << "}" << endl;
	sstr << "T_global = " << T_global;
	sstr << "T_left = " << T_left;
	sstr << "T_right = " << T_right;
	sstr << "T = " << T << endl;
	sstr << "inv_T = " << inv_T << endl;
	sstr << "S = " << S << endl;
	sstr << "dS = " << dS << endl;
	sstr << "Sdq = " << Sdq << endl;
	sstr << "dSdq = " << dSdq << endl;
	sstr << "Sddq = " << Sddq << endl;
	sstr << "DSdqDt = " << DSdqDt << endl;
	sstr << endl;

	return sstr.str();
}

se3 GJoint::get_S(int idx_)
{
	if ( idx_ < 0 || idx_ >= getDOF() ) {
		return se3(0,0,0,0,0,0);
	} else {
		return se3(S(0,idx_),S(1,idx_),S(2,idx_),S(3,idx_),S(4,idx_),S(5,idx_));
	}
}

se3 GJoint::get_dS(int idx_)
{
	if ( idx_ < 0 || idx_ >= getDOF() ) {
		return se3(0,0,0,0,0,0);
	} else {
		return se3(dS(0,idx_),dS(1,idx_),dS(2,idx_),dS(3,idx_),dS(4,idx_),dS(5,idx_));
	}
}

se3 GJoint::get_S(GCoordinate *pCoordinate_)
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( *iter_pcoord == pCoordinate_ ) {
			return se3(S(0,i),S(1,i),S(2,i),S(3,i),S(4,i),S(5,i));
		}
	}

	return se3(0,0,0,0,0,0);
}

se3 GJoint::get_dS(GCoordinate *pCoordinate_)
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( *iter_pcoord == pCoordinate_ ) {
			return se3(dS(0,i),dS(1,i),dS(2,i),dS(3,i),dS(4,i),dS(5,i));
		}
	}

	return se3(0,0,0,0,0,0);
}

void GJoint::allocate_memory(int n_)
{
	S.SetZero(6,n_);
	dS.SetZero(6,n_);
}
