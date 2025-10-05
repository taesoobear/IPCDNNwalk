#ifndef OPERATPR_NR_H
#define OPERATPR_NR_H
#pragma once

#include "math_macro.h"

namespace m
{
	// covariance * numData: usually called the matrix S.
	void covarianceN(vectorn& mean, matrixn& c, const matrixn& a) ;

	void LUinvert(matrixn& F, const matrixn& E) ;
	// det = e**log_det
	void LUinvert(matrixn& out, const matrixn& in, m_real& log_det);
	// det = det_mantissa * 10**det_exponent
	void LUinvert(matrixn& out, const matrixn& in, m_real & det_man, int& det_exp);

	void Diaginvert(vectorn& out, const vectorn& in, m_real& log_det);
	void SVinverse( matrixn& in_u, matrixn&mat ) ;
	void pseudoInverse(matrixn& out, const matrixn& in);

	// in=>u*diag(s)*vT. in is replaced by u.
	void SVdecompose(matrixn& in_u, vectorn & s, matrixn &v);
	void LUsolve(matrixn const & A, vectorn const& b, vectorn& x);

	class LDLT
	{
		void* _data;
		public:
		LDLT(matrixn const& A);
		virtual ~LDLT();
		vectorn solve(const vectorn& b);
	};

	class PartialPivLU
	{
		void* _data;
		public:
		PartialPivLU(matrixn const& A);
		virtual ~PartialPivLU();
		vectorn solve(const vectorn& b);
	};

	void PIsolve(matrixn const & A, vectorn const& b, vectorn& x);

	m_real determinant(const matrixn& E);

	// c: input e: eigen values, v: eigen vectors (each column).
	// c can be recovered from e and v using
	//  c.op2(m2::multABAt(), v, m::diag(e));
	//
	// method==0 : jacobi
	// method==1 : tqli, tred2 -> faster, but only for symmetric tridiagonal matrices.
	// eigen values and corresponding eigen vectors are sorted in decreasing eigen-values order.
	void eigenDecomposition(matrixn const& c, vectorn & e, matrixn & v, int method=1);
	void eigenVectors(matrixn& EVectors, const matrixn& mat, vectorn& eigenValues);
	void cofactor(matrixn& c, const matrixn& a);

}
#endif
