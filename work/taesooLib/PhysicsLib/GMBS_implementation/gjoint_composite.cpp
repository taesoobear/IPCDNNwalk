#include <list>
#include <algorithm>
#include "gjoint_composite.h"
#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"



//=============================================================
//                 GJointComposite
//=============================================================
GJointComposite::GJointComposite()
{
	jointType = GJOINT_COMPOSITE;
	pJoint1 = pJoint2 = NULL;
}

GJointComposite::GJointComposite(GJoint *pjoint1_, GJoint *pjoint2_)
{
	if ( pjoint1_ == NULL || pjoint2_ == NULL ) {
		pJoint1 = pJoint2 = NULL;
	} else {
		compose(pjoint1_, pjoint2_);
	}
}

bool GJointComposite::compose(GJoint *pjoint1_, GJoint *pjoint2_)
{
	std::list<GCoordinate*>::iterator iter_pcoord;

	if ( pjoint1_ == NULL || pjoint2_ == NULL ) return false;

	pJoint1 = pjoint1_;
	pJoint2 = pjoint2_;
	
	pCoordinates.clear();
	for (iter_pcoord = pJoint1->pCoordinates.begin(); iter_pcoord != pJoint1->pCoordinates.end(); iter_pcoord++) {
		pCoordinates.push_back(*iter_pcoord);
	}
	for (iter_pcoord = pJoint2->pCoordinates.begin(); iter_pcoord != pJoint2->pCoordinates.end(); iter_pcoord++) {
		pCoordinates.push_back(*iter_pcoord);
	}

	allocate_memory(getDOF());

	return true;
}

void GJointComposite::update_short()
{
	pJoint1->update_short();
	pJoint2->update_short();

	SE3 inv_T2 = pJoint2->get_inv_T();

	T = pJoint1->get_T();
	T *= pJoint2->get_T();

	inv_T.SetInvOf(T);

	// S = [Ad(inv_T2, pJoint1->get_S()), pJoint2->get_S()]
	S.SetZero(6, getDOF());
	Ad(S.GetPtr(), inv_T2, pJoint1->get_S().GetPtr(), pJoint1->getDOF()); //S.Push(0, 0, Ad(inv_T2, pJoint1->get_S()));
	S.Push(0, pJoint1->getDOF(), pJoint2->get_S());

	if ( bReversed ) { _update_short_for_reversed_joint(); }
}

void GJointComposite::update()
{
	pJoint1->update();
	pJoint2->update();

	SE3 inv_T2 = pJoint2->get_inv_T();

	T = pJoint1->get_T();
	T *= pJoint2->get_T();

	inv_T.SetInvOf(T);

	Sdq.Ad(inv_T2, pJoint1->get_Sdq());
	Sdq += pJoint2->get_Sdq();

	dSdq.ad(-pJoint2->get_Sdq(), Ad(inv_T2, pJoint1->get_Sdq()));
	dSdq += Ad(inv_T2, pJoint1->get_dSdq());
	dSdq += pJoint2->get_dSdq();

	Sddq.Ad(inv_T2, pJoint1->get_Sddq());
	Sddq += pJoint2->get_Sddq();

	DSdqDt = Sddq;
	DSdqDt += dSdq;

	// S = [Ad(inv_T2, pJoint1->get_S()), pJoint2->get_S()]
	S.SetZero(6, getDOF());
	//Ad(S.GetPtr(), inv_T2, pJoint1->get_S().GetPtr(), pJoint1->getDOF()); 
	S.Push(0, 0, Ad(inv_T2, pJoint1->get_S()));
	S.Push(0, pJoint1->getDOF(), pJoint2->get_S());

	dS.SetZero(6, getDOF());
	dS.Push(0, 0, - ad(pJoint2->get_Sdq(), Ad(inv_T2, pJoint1->get_S())) + Ad(inv_T2, pJoint1->get_dS()));
	dS.Push(0, pJoint1->getDOF(), pJoint2->get_dS());

	if ( bReversed ) { _update_for_reversed_joint(); }
}

RMatrix GJointComposite::get_DSDq(GCoordinate *pCoordinate_)
{
	if ( find(pCoordinates.begin(), pCoordinates.end(), pCoordinate_) == pCoordinates.end() ) return Zeros(6, getDOF());

	RMatrix DSDq, DS1Dq, DS2Dq;

	DS1Dq = -ad(pJoint2->get_S(pCoordinate_), Ad(pJoint2->get_inv_T(), pJoint1->get_S()));
	DS1Dq += Ad(pJoint2->get_inv_T(), pJoint1->get_DSDq(pCoordinate_));

	DS2Dq = pJoint2->get_DSDq(pCoordinate_);

	DSDq.SetZero(6, getDOF());
	DSDq.Push(0, 0, DS1Dq);
	DSDq.Push(0, pJoint1->getDOF(), DS2Dq);
	
	if ( bReversed ) {
		DSDq = -Ad(inv_T, DSDq);
		DSDq -= ad(get_S(pCoordinate_), S);
	}

	return DSDq;
}

RMatrix GJointComposite::get_DdSDq(GCoordinate *pCoordinate_)
{
	if ( find(pCoordinates.begin(), pCoordinates.end(), pCoordinate_) == pCoordinates.end() ) return Zeros(6, getDOF());

	RMatrix DdSDq, DdS1Dq, DdS2Dq, Ss;

	DdS1Dq = -ad(pJoint2->get_DSDq(pCoordinate_) * pJoint2->get_dq(), Ad(pJoint2->get_inv_T(), pJoint1->get_S()));
	DdS1Dq += ad(pJoint2->get_Sdq(), ad(pJoint2->get_S(pCoordinate_), Ad(pJoint2->get_inv_T(), pJoint1->get_S())));
	DdS1Dq -= ad(pJoint2->get_Sdq(), Ad(pJoint2->get_inv_T(), pJoint1->get_DSDq(pCoordinate_)));
	DdS1Dq -= ad(pJoint2->get_S(pCoordinate_), Ad(pJoint2->get_inv_T(), pJoint1->get_dS()));
	DdS1Dq += Ad(pJoint2->get_inv_T(), pJoint1->get_DdSDq(pCoordinate_));

	DdS2Dq = pJoint2->get_DdSDq(pCoordinate_);

	DdSDq.SetZero(6, getDOF());
	DdSDq.Push(0, 0, DdS1Dq);
	DdSDq.Push(0, pJoint1->getDOF(), DdS2Dq);

	if ( bReversed ) {
		RMatrix DSDq = get_DSDq(pCoordinate_);
		DdSDq = -Ad(inv_T, DdSDq);
		DdSDq -= ad(get_S(pCoordinate_), dS + ad(Sdq, S));
		DdSDq -= ad(DSDq*get_dq(), S);
		DdSDq -= ad(Sdq, DSDq);
	}

	return DdSDq;
}

void GJointComposite::_update_short_for_reversed_joint()
{
	SE3 T_tmp = T;
	T = inv_T;
	inv_T = T_tmp;

	S = -Ad(inv_T, S);
}

void GJointComposite::_update_for_reversed_joint()
{
	SE3 T_tmp = T;
	T = inv_T;
	inv_T = T_tmp;

	Sdq = -Ad(inv_T, Sdq);
	dSdq = -Ad(inv_T, dSdq);
	Sddq = -Ad(inv_T, Sddq);
	DSdqDt = Sddq + dSdq;
	S = -Ad(inv_T, S);
	dS = -Ad(inv_T, dS) - ad(Sdq, S);
}
