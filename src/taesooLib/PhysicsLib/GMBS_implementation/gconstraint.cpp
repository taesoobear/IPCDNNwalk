#include <list>
#include "gconstraint.h"
#include "gelement.h"
#include "gcoordinate.h"
#include "rmatrix3j.h"

using namespace std;


//=============================================================
//                 GConstraint
//=============================================================
GConstraint::GConstraint()
{
	constrNum = 0;
	C = Zeros(0,0);
	J = Zeros(0,0);
	dJdt = Zeros(0,0);
}

RMatrix GConstraint::get_C()
{
	return C;
}

RMatrix GConstraint::get_J()
{
	return J;
}

RMatrix GConstraint::get_J(int idx_)
{
	return J.Sub(0, J.RowSize()-1, idx_, idx_);
}

RMatrix GConstraint::get_J(GCoordinate *pCoordinate_)
{
	int idx;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(), idx = 0; iter_pcoord != pCoordinates.end(); iter_pcoord++, idx++) {
		if ( pCoordinate_ == *iter_pcoord ) break;
	}
	if ( idx >= getNumCoordinates() ) return Zeros(J.RowSize(), 1);

	return get_J(idx);
}

RMatrix GConstraint::get_J(vector<int> idx_)
{
	RMatrix Jsub(J.RowSize(), int(idx_.size()));
	for (int i=0; i<int(idx_.size()); i++) {
		Jsub.Push(0, i, get_J(idx_[i]));
	}
	return Jsub;
}

RMatrix GConstraint::get_J(vector<GCoordinate*> pCoordinates_)
{
	RMatrix Jsub(J.RowSize(), int(pCoordinates_.size()));
	for (int i=0; i<int(pCoordinates_.size()); i++) {
		Jsub.Push(0, i, get_J(pCoordinates_[i]));
	}
	return Jsub;
}

RMatrix GConstraint::get_dJdt()
{
	return dJdt;
}

RMatrix GConstraint::get_dJdt(int idx_)
{
	return dJdt.Sub(0, dJdt.RowSize()-1, idx_, idx_);
}

RMatrix GConstraint::get_dJdt(GCoordinate *pCoordinate_)
{
	int idx;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(), idx = 0; iter_pcoord != pCoordinates.end(); iter_pcoord++, idx++) {
		if ( pCoordinate_ == *iter_pcoord ) break;
	}
	if ( idx >= getNumCoordinates() ) return Zeros(dJdt.RowSize(), 1);

	return get_dJdt(idx);
}

RMatrix GConstraint::get_dJdt(vector<int> idx_)
{
	RMatrix dJdtsub(dJdt.RowSize(), int(idx_.size()));
	for (int i=0; i<int(idx_.size()); i++) {
		dJdtsub.Push(0, i, get_dJdt(idx_[i]));
	}
	return dJdtsub;
}

RMatrix GConstraint::get_dJdt(vector<GCoordinate*> pCoordinates_)
{
	RMatrix dJdtsub(dJdt.RowSize(), int(pCoordinates_.size()));
	for (int i=0; i<int(pCoordinates_.size()); i++) {
		dJdtsub.Push(0, i, get_dJdt(pCoordinates_[i]));
	}
	return dJdtsub;
}

