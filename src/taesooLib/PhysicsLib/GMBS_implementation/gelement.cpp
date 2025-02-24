#include <list>
#include <string>
#include <sstream>
#include "gelement.h"
#include "gcoordinate.h"
#include "liegroup.h"

using namespace std;


//=============================================================
//                 GElement
//=============================================================
void GElement::initialize()
{
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->initValues();
	}
}

vector<GCoordinate*> GElement::getPrescribedCoordinates()
{
	int i;
	vector<GCoordinate*> pcoords;
	list<GCoordinate *>::iterator iter_pcoord;
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( (*iter_pcoord)->bPrescribed ) {
			pcoords.push_back(*iter_pcoord);
		}
	}
	return pcoords;
}

vector<GCoordinate*> GElement::getUnprescribedCoordinates()
{
	int i;
	vector<GCoordinate*> pcoords;
	list<GCoordinate *>::iterator iter_pcoord;
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( !(*iter_pcoord)->bPrescribed ) {
			pcoords.push_back(*iter_pcoord);
		}
	}
	return pcoords;
}

vector<int> GElement::getIndexOfPrescribedCoordinates()
{
	int i;
	vector<int> idx;
	list<GCoordinate *>::iterator iter_pcoord;
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( (*iter_pcoord)->bPrescribed ) {
			idx.push_back(i);
		}
	}
	return idx;
}

vector<int> GElement::getIndexOfUnprescribedCoordinates()
{
	int i;
	vector<int> idx;
	list<GCoordinate *>::iterator iter_pcoord;
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( !(*iter_pcoord)->bPrescribed ) {
			idx.push_back(i);
		}
	}
	return idx;
}

int GElement::getIndexOfCoordinate(GCoordinate *pcoord_)
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;
	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( *iter_pcoord == pcoord_ ) return i;
	}
	return -1;
}

void GElement::set_q(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->q = x_[i++];
	}
}

void GElement::set_dq(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->dq = x_[i++];
	}
}

void GElement::set_ddq(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->ddq = x_[i++];
	}
}

void GElement::set_tau(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->tau = x_[i++];
	}
}

void GElement::set_DqDp(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DqDp = x_[i++];
	}
}

void GElement::set_DdqDp(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DdqDp = x_[i++];
	}
}

void GElement::set_DddqDp(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DddqDp = x_[i++];
	}
}

void GElement::set_DtauDp(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->DtauDp = x_[i++];
	}
}

void GElement::set_qLL(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->qLL = x_[i++];
	}
}

void GElement::set_dqLL(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->dqLL = x_[i++];
	}
}

void GElement::set_ddqLL(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->ddqLL = x_[i++];
	}
}

void GElement::set_tauLL(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->tauLL = x_[i++];
	}
}

void GElement::set_qUL(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->qUL = x_[i++];
	}
}

void GElement::set_dqUL(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->dqUL = x_[i++];
	}
}

void GElement::set_ddqUL(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->ddqUL = x_[i++];
	}
}

void GElement::set_tauUL(const double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->tauUL = x_[i++];
	}
}

void GElement::get_q(double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->q;
	}
}

void GElement::get_dq( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->dq;
	}
}

void GElement::get_ddq( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->ddq;
	}
}

void GElement::get_tau( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->tau;
	}
}

void GElement::get_DqDp( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->DqDp;
	}
}

void GElement::get_DdqDp( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->DdqDp;
	}
}

void GElement::get_DddqDp( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->DddqDp;
	}
}

void GElement::get_DtauDp( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->DtauDp;
	}
}

void GElement::get_qLL( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->qLL;
	}
}

void GElement::get_dqLL( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->dqLL;
	}
}

void GElement::get_ddqLL( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->ddqLL;
	}
}

void GElement::get_tauLL( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->tauLL;
	}
}

void GElement::get_qUL( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->qUL;
	}
}

void GElement::get_dqUL( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->dqUL;
	}
}

void GElement::get_ddqUL( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->ddqUL;
	}
}

void GElement::get_tauUL( double *x_)
{
	int i = 0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++) {
		x_[i++] = (*iter_pcoord)->tauUL;
	}
}

RMatrix GElement::get_q(std::vector<int> idx_)
{
	RMatrix qsub(int(idx_.size()), 1);
	RMatrix q = get_q();
	for (int i=0; i<int(idx_.size()); i++) {
		if ( idx_[i] < 0 || idx_[i] >= getNumCoordinates() ) {
			qsub[i] = 0;
		} else {
			qsub[i] = q[idx_[i]];
		}
	}
	return qsub;
}

RMatrix GElement::get_dq(std::vector<int> idx_)
{
	RMatrix dqsub(int(idx_.size()), 1);
	RMatrix dq = get_dq();
	for (int i=0; i<int(idx_.size()); i++) {
		if ( idx_[i] < 0 || idx_[i] >= getNumCoordinates() ) {
			dqsub[i] = 0;
		} else {
			dqsub[i] = dq[idx_[i]];
		}
	}
	return dqsub;
}

RMatrix GElement::get_ddq(std::vector<int> idx_)
{
	RMatrix ddqsub(int(idx_.size()), 1);
	RMatrix ddq = get_ddq();
	for (int i=0; i<int(idx_.size()); i++) {
		if ( idx_[i] < 0 || idx_[i] >= getNumCoordinates() ) {
			ddqsub[i] = 0;
		} else {
			ddqsub[i] = ddq[idx_[i]];
		}
	}
	return ddqsub;
}

RMatrix GElement::get_tau(std::vector<int> idx_)
{
	RMatrix tausub(int(idx_.size()), 1);
	RMatrix tau = get_tau();
	for (int i=0; i<int(idx_.size()); i++) {
		if ( idx_[i] < 0 || idx_[i] >= getNumCoordinates() ) {
			tausub[i] = 0;
		} else {
			tausub[i] = tau[idx_[i]];
		}
	}
	return tausub;
}

string GElement::getInfoStr()
{
	stringstream sstr;
	sstr << "GElement:: name = " << name << ", id = " << id << endl;
	return sstr.str();
}

