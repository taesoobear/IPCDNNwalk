//================================================================================
//         CONSTRAINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_CONSTRAINT_
#define _GMBS_GEOMETRIC_CONSTRAINT_

#include <vector>
#include "gelement.h"
#include "gcoordinate.h"
#include "rmatrix3j.h"


//=============================================================
//                 GConstraint
//=============================================================
class GConstraint: public GElement
{
public:
	int constrNum;		// the number of constraints

	RMatrix C;			// C = C(q,t) where q = pCoordinates[]->q, size(C) = (m,1)
	RMatrix J;			// J = dC/dq, size(J) = (m,n)
	RMatrix dJdt;		// dJdt = dJ/dt, size(dJdt) = (m,n)
						// where m = constrNum, n = size(pCoordinates)

public:
	GConstraint();
	~GConstraint() {}

public:
	int getNumConstraints() { return constrNum; }

	RMatrix get_C();

	RMatrix get_J();
	RMatrix get_J(int idx_);
	RMatrix get_J(GCoordinate *pCoordinate_);
	RMatrix get_J(std::vector<int> idx_);
	RMatrix get_J(std::vector<GCoordinate*> pCoordinates_);
	
	RMatrix get_dJdt();
	RMatrix get_dJdt(int idx_);
	RMatrix get_dJdt(GCoordinate *pCoordinate_);
	RMatrix get_dJdt(std::vector<int> idx_);
	RMatrix get_dJdt(std::vector<GCoordinate*> pCoordinates_);

public: // virtual functions
	virtual bool update_C() = 0;
	virtual bool update_J() = 0;
	virtual bool update_dJdt() = 0;
};


#endif

