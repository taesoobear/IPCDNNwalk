
#ifndef EXCLUDE_CLAPACK
extern "C"{
#include <f2c.h>
#include <clapack.h>
#undef abs
#undef max
#undef min
#undef complex
}
//#else
//#include "clapack_min.hpp"
//#endif

//#include <stdexcept>
//#include "../BaseLib/stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#ifndef __APPLE__
#include <malloc.h>
#endif
#include <stdexcept>
#include <memory.h>
#include <limits>
#include "../BaseLib/math/mathclass_minimum.h"
//#include "../BaseLib/math/mathclass.h"
#include "clapack_wrap.h"

class fortranIndexes: public _tvectorn<integer>
{
protected:
	fortranIndexes(integer* ptrr, int size, int stride):_tvectorn<integer>(ptrr,size,stride){}	// reference
public:
	fortranIndexes():_tvectorn<integer>(){}

	explicit fortranIndexes(int n):_tvectorn<integer>()				{setSize(n);}
	
	// copy constructor : 항상 카피한다.
	fortranIndexes(const _tvectorn<integer>& other):_tvectorn<integer>()		{ assign(other);}
	fortranIndexes(const fortranIndexes& other):_tvectorn<integer>()			{ assign(other);}
	
	// 값을 copy한다.
	fortranIndexes& operator=(const _tvectorn<integer>& other)		{ _tvectorn<integer>::assign(other);return *this;}
	fortranIndexes& operator=(const fortranIndexes& other)			{ assign(other);return *this;}
	
};


static double* getFortranArray(matrixn& a)
{
	double* buffer;
	int stride,n,m,on;
	a._getPrivate(buffer, stride, n,m,on);

	Msg::verify(m==stride,"getFortranArray");
	return buffer;
}
#include "GMBS_implementation/rmatrix3j.h"
static void RMat_transpose(RMatrix& at, matrixn const& a)
{
	double* buffer;
	int stride,n,m,on;
	a._getPrivate(buffer, stride, n,m,on);

	Msg::verify(m==stride,"getFortranArray");

	at.row=a.cols();
	at.col=a.rows();
	at.element=buffer;
	at.owner=false;
}

static void RMat_assign(RMatrix& ra, matrixn const& a)
{
	ra.ReNew(a.rows(), a.cols());
	
	for(int i=0; i<a.rows(); i++)
		for(int j=0; j<a.cols(); j++)
			ra(i,j)=a(i,j);
}

static void RMat_assign(matrixn & a, RMatrix const& ra)
{
	a.setSize(ra.row, ra.col);
	
	for(int i=0; i<a.rows(); i++)
		for(int j=0; j<a.cols(); j++)
			a(i,j)=ra(i,j);
}

void __balance(matrixn & a, matrixn& dd, char* opt)
{
	// all parameters are in column-major form. (transposed)
	
	integer n=(integer)a.rows();
	if (a.rows () != a.cols())    {
		Msg::error("AEPBALANCE requires square matrix");
		return;
	}

	vectorn scale(n);

	double *pscale = scale.dataPtr();

	double *p_balanced_mat = getFortranArray(a);


	integer ilo, ihi, info;

	dgebal_(opt, &n, p_balanced_mat, &n, &ilo, &ihi, pscale, &info);


	

	dd.resize(n,n);
	dd.setAllValue(0.0);
	dd.diag().setAllValue(1.0);

	
	double *p_balancing_mat = getFortranArray(dd);

	
	char* side = "R";

	dgebak_ (opt, side, &n, &ilo, &ihi, pscale, &n, p_balancing_mat, &n, &info);
}



static logical 
select_ana (const double& a, const double&)
{
	return (a < 0.0);
}


// a: inout
int __schur(matrixn& a, matrixn & unitary_mat, const char* ord)
{
	int a_nr = a.cols ();
	int a_nc = a.rows ();

	if (a_nr != a_nc)
		{
			Msg::error ("SCHUR requires square matrix");
			return -1;
		}

	bool calc_unitary=true;
	// Workspace requirements may need to be fixed if any of the
	// following change.

	char jobvs;
	char sense = 'N';
	char sort = 'N';

	if (calc_unitary)
		jobvs = 'V';
	else
		jobvs = 'N';

	char ord_char = ord[0];

	if (ord_char == 'A' || ord_char == 'D' || ord_char == 'a' || ord_char == 'd')
		sort = 'S';

	L_fp selector;
	if (ord_char == 'A' || ord_char == 'a')
		selector = (L_fp)select_ana;
	else if (ord_char == 'D' || ord_char == 'd')
		assert(false);
	//    selector = select_dig;
	else
		selector = 0;

	integer n = a_nc;
	integer lwork = 8 * n;
	integer liwork = 1;
	integer info;
	integer sdim;
	double rconde;
	double rcondv;


	if (calc_unitary)
		unitary_mat.resize (n, n);

	double *s = getFortranArray(a);
	double *q = getFortranArray(unitary_mat);

	vectorn wr(n);
	vectorn wi(n);
	vectorn work(lwork);
	double *pwr = wr.dataPtr();
	double *pwi = wi.dataPtr();
	double *pwork = work.dataPtr();

	// BWORK is not referenced for the non-ordered Schur routine.
	fortranIndexes bwork ((ord_char == 'N' || ord_char == 'n') ? 0 : n);
	integer *pbwork = bwork.dataPtr ();

	fortranIndexes iwork (liwork);
	integer *piwork = iwork.dataPtr();

	dgeesx_(&jobvs,&sort, selector, &sense,
		   &n, s, &n, &sdim, pwr, pwi, q, &n, &rconde, &rcondv,
		   pwork, &lwork, piwork, &liwork, pbwork, &info);

	return info;
}


void CLapack::balancedSchurDecomposition(matrixn const& H, matrixn& u)
{
	static matrixn H_colmajor;
	H_colmajor.transpose(H);

	//	"balance: used GEP, should have two output arguments"
	// Generalized eigenvalue problem. 

	matrixn dt,ut;
	__balance(H_colmajor, dt, "B");
	__schur(H_colmajor,ut, "A");

	u.multAtBt(dt, ut);
}


void CLapack::LUinvert(matrixn& out, const matrixn& in)
{
	RMatrix At;
	RMat_transpose(At,in);
	RMatrix _x;
	SolveAtxEqualB(At,_x,Eye(in.rows(), in.rows()));
	RMat_assign(out, _x);
}


	
#else
#include "../BaseLib/math/mathclass_minimum.h"
#include "clapack_wrap.h"

void CLapack::balancedSchurDecomposition(matrixn const& H, matrixn& u)
{
	Msg::error("reimplement balancedSchurDecomposition..");
}
void CLapack::LUinvert(matrixn& out, const matrixn& in)
{
	Msg::error("reimplement LUinvert..");
}
#endif
