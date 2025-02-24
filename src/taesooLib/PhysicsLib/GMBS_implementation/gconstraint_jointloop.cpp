#include <list>

#include "gconstraint_jointloop.h"
#include "gconstraint.h"
#include "gjoint.h"
#include "gcoordinate.h"

#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"

using namespace std;


//=============================================================
//                 GConstraintJointLoop
//=============================================================
GConstraintJointLoop::GConstraintJointLoop()
{
	M1.SetIdentity();
	M2.SetIdentity();
//	T.SetIdentity();

	num_coord = 0;
	num_coord2 = 0;

	jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;

	jacobian.SetZero(0,0);
	dotjacobian.SetZero(0,0);
}

void GConstraintJointLoop::setJointLoopConstraintType(JOINTLOOP_CONSTRAINT_TYPE jointLoopConstraintType_)
{
	jointLoopConstraintType = jointLoopConstraintType_;

	switch ( jointLoopConstraintType )
	{
	case JOINTLOOP_ORIENTATION_POSITION:
		constrNum = 6;
		break;
	case JOINTLOOP_ORIENTATION_ONLY:
		constrNum = 3;
		break;
	case JOINTLOOP_POSITION_ONLY:
		constrNum = 3;
		break;
	}
}

bool GConstraintJointLoop::setJoints(list<GJoint *> pjoints)
{
	list<GJoint *>::iterator iter_pjoint;
	list<GCoordinate *>::iterator iter_pcoord;

	pCoordinates.clear();
	pJoints.clear();
	pJoints2.clear();
	num_coord = 0;
	num_coord2 = 0;

	for (iter_pjoint = pjoints.begin(); iter_pjoint != pjoints.end(); iter_pjoint++) {
		pJoints.push_back(*iter_pjoint);
		for (iter_pcoord = (*iter_pjoint)->pCoordinates.begin(); iter_pcoord != (*iter_pjoint)->pCoordinates.end(); iter_pcoord++) {
			pCoordinates.push_back(*iter_pcoord);
		}
		num_coord += (*iter_pjoint)->getDOF();
	}
	
	jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
	constrNum = 6;

	return true;
}

bool GConstraintJointLoop::setJoints(list<GJoint *> pjoints, list<GJoint *> pjoints2)
{
	list<GJoint *>::iterator iter_pjoint;
	list<GCoordinate *>::iterator iter_pcoord;

	pCoordinates.clear();
	pJoints.clear();
	pJoints2.clear();
	num_coord = 0;
	num_coord2 = 0;

	for (iter_pjoint = pjoints.begin(); iter_pjoint != pjoints.end(); iter_pjoint++) {
		pJoints.push_back(*iter_pjoint);
		for (iter_pcoord = (*iter_pjoint)->pCoordinates.begin(); iter_pcoord != (*iter_pjoint)->pCoordinates.end(); iter_pcoord++) {
			pCoordinates.push_back(*iter_pcoord);
		}
		num_coord += (*iter_pjoint)->getDOF();
	}

	for (iter_pjoint = pjoints2.begin(); iter_pjoint != pjoints2.end(); iter_pjoint++) {
		pJoints2.push_back(*iter_pjoint);
		for (iter_pcoord = (*iter_pjoint)->pCoordinates.begin(); iter_pcoord != (*iter_pjoint)->pCoordinates.end(); iter_pcoord++) {
			pCoordinates.push_back(*iter_pcoord);
		}
		num_coord2 += (*iter_pjoint)->getDOF();
	}

	if ( getNumCoordinates() != num_coord + num_coord2 ) return false;

	jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
	constrNum = 6;

	return true;
}

SE3 GConstraintJointLoop::getLoopSE3()
{
	list<GJoint *>::iterator iter_pjoint;
	SE3 re;

	re.SetIdentity();
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		re *= (*iter_pjoint)->T_left;
		re *= (*iter_pjoint)->get_T();
		re *= (*iter_pjoint)->inv_T_right;
	}
	re *= M1;
	
	return re;
}

bool GConstraintJointLoop::update_C()
{
	list<GJoint *>::iterator iter_pjoint;
	SE3 T_loop_left, T_loop_right; 
	RMatrix re(6,1);
	
	// left side of the loop
	T_loop_left.SetIdentity();
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		(*iter_pjoint)->update_short();
		T_loop_left *= (*iter_pjoint)->T_left;
		T_loop_left *= (*iter_pjoint)->get_T();
		T_loop_left *= (*iter_pjoint)->inv_T_right;
	}
	T_loop_left *= M1;

	// right side of the loop
	T_loop_right.SetIdentity();
	for (iter_pjoint = pJoints2.begin(); iter_pjoint != pJoints2.end(); iter_pjoint++) {
		(*iter_pjoint)->update_short();
		T_loop_right *= (*iter_pjoint)->T_right;
		T_loop_right *= (*iter_pjoint)->get_T();
		T_loop_right *= (*iter_pjoint)->inv_T_right;
	}
	T_loop_right *= M2;

	put_se3_to_matrix(re, InvSkew(Inv(T_loop_right)*T_loop_left), 0);		// InvSkew(G) = unskew(G-I), where G-I is assumed to be a 4x4 se3.
																// ** Modified in 2007.05.22: 
																//    InvSkew(Inv(T_right)*T_left) --> InvSkew(Inv(T_left)*T_right)
																// ** Modification canceled in 2007.05.22, i.e.,
																//    InvSkew(Inv(T_left)*T_right) --> InvSkew(Inv(T_right)*T_left)

	C.SetZero(constrNum, 1);
	switch ( jointLoopConstraintType )
	{
	case JOINTLOOP_ORIENTATION_POSITION:
		C = re;
		break;
	case JOINTLOOP_ORIENTATION_ONLY:
		C = re.Sub(0, 2, 0, 0);
		break;
	case JOINTLOOP_POSITION_ONLY:
		C = re.Sub(3, 5, 0, 0);
		break;
	}

	return true;
}

// body J of the joint loop constraint
// case f=g   : J = [ inv(f)*dot(f), -inv(g)*dot(g) ]
// case f*M=T : J = [ inv(f*M)*dot(f*M) ] = Ad( inv(M), [ inv(f)*dot(f) ] )
bool GConstraintJointLoop::update_J()
{
	int idx; 
	SE3 Ti;
	list<GJoint *>::reverse_iterator riter_pjoint;
	
	jacobian.SetZero(6, getNumCoordinates());

	idx = num_coord;
	Ti.SetInvOf(M1);
	for (riter_pjoint = pJoints.rbegin(); riter_pjoint != pJoints.rend(); riter_pjoint++) {
		idx -= (*riter_pjoint)->getDOF();
		Ti *= (*riter_pjoint)->T_right;
		//jacobian.Push(0, idx, Ad(Ti, (*riter_pjoint)->get_S()));
		Ad(&jacobian[6*idx], Ti, (*riter_pjoint)->get_S().GetPtr(), (*riter_pjoint)->getDOF());
		Ti *= (*riter_pjoint)->get_inv_T();
		Ti *= (*riter_pjoint)->inv_T_left;
	}

	idx = num_coord + num_coord2;
	Ti.SetInvOf(M2);
	for (riter_pjoint = pJoints2.rbegin(); riter_pjoint != pJoints2.rend(); riter_pjoint++) {
		idx -= (*riter_pjoint)->getDOF();
		Ti *= (*riter_pjoint)->T_right;
		//jacobian.Push(0, idx, -Ad(Ti, (*riter_pjoint)->get_S()));
		minus_Ad(&jacobian[6*idx], Ti, (*riter_pjoint)->get_S().GetPtr(), (*riter_pjoint)->getDOF());
		Ti *= (*riter_pjoint)->get_inv_T();
		Ti *= (*riter_pjoint)->inv_T_left;
	}
	
	J.SetZero(constrNum, getNumCoordinates());
	switch ( jointLoopConstraintType )
	{
	case JOINTLOOP_ORIENTATION_POSITION:
		J = jacobian;
		break;
	case JOINTLOOP_ORIENTATION_ONLY:
		J = jacobian.Sub(0, 2, 0, getNumCoordinates()-1);
		break;
	case JOINTLOOP_POSITION_ONLY:
		J = jacobian.Sub(3, 5, 0, getNumCoordinates()-1);
		break;
	}

	return true;
}

bool GConstraintJointLoop::update_dJdt()
{
	int i, j;
	RMatrix Jsum;
	list<GCoordinate *>::iterator iter_pcoord_i, iter_pcoord_j;

	dotjacobian.SetZero(6, getNumCoordinates());

	i=0;
	iter_pcoord_i = pCoordinates.begin();

	for ( ; i<num_coord; i++)
	{
		Jsum = Zeros(6,1);
		iter_pcoord_j = iter_pcoord_i;
		iter_pcoord_j++;
		for (j=i+1; j<num_coord; j++)
		{
			Jsum += ad(get_jacobian(i), get_jacobian(j)) * (*iter_pcoord_j)->dq;
			iter_pcoord_j++;
		}
		dotjacobian.Push(0, i, Jsum);
		iter_pcoord_i++;
	}

	for ( ; i<num_coord+num_coord2; i++)
	{
		Jsum = Zeros(6,1);
		iter_pcoord_j = iter_pcoord_i;
		iter_pcoord_j++;
		for (j=i+1; j<num_coord+num_coord2; j++)
		{
			Jsum += ad(get_jacobian(i), get_jacobian(j)) * (*iter_pcoord_j)->dq;
			iter_pcoord_j++;
		}
		dotjacobian.Push(0, i, -Jsum);
		iter_pcoord_i++;
	}

	dJdt.SetZero(constrNum, getNumCoordinates());
	switch ( jointLoopConstraintType )
	{
	case JOINTLOOP_ORIENTATION_POSITION:
		dJdt = dotjacobian;
		break;
	case JOINTLOOP_ORIENTATION_ONLY:
		dJdt = dotjacobian.Sub(0, 2, 0, getNumCoordinates()-1);
		break;
	case JOINTLOOP_POSITION_ONLY:
		dJdt = dotjacobian.Sub(3, 5, 0, getNumCoordinates()-1);
		break;
	}

	return true;
}

string GConstraintJointLoop::getInfoStr()
{
	stringstream sstr;
	list<GJoint*>::iterator iter_pjoint;

	sstr << GElement::getInfoStr();
	sstr << "jointLoopConstraintType = " << jointLoopConstraintType << endl;
	sstr << "pJoints = ";
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		sstr << (*iter_pjoint)->getName() << ", ";
	}
	sstr << endl;
	sstr << "pJoints2 = ";
	for (iter_pjoint = pJoints2.begin(); iter_pjoint != pJoints2.end(); iter_pjoint++) {
		sstr << (*iter_pjoint)->getName() << ", ";
	}
	sstr << endl;
	sstr << "M1 = " << M1 << "M2 = " << M2 << "T = " << T << endl;

	return sstr.str();
}