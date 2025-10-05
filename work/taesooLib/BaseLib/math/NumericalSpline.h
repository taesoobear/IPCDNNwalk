#ifndef NUMERICALSPLINE_H_
#define NUMERICALSPLINE_H_
#pragma once
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/math/Operator_NR.h"
class NumericalSpline
{
public:
	NumericalSpline(vectorn const& keytime, const matrixn& controlpoints){}
	~NumericalSpline(void){}

	// cubic spline이지만, c1 continuous까지만 보장하고, underdetermined와 overdetermined system을 모두 다룰수 있다.

	// X=at^3+bt^2+ct+d
	// Y=et^3+ft^2+gt+h

	// X'=3at^2+2bt+c
	// X''=6at+2b

	// minimize( integral (x'')^2 -> a와 b의 2차식.
	// such that 
	//   linear constraints... such as position, velocity, acceleration constraint: trivial
	//   tangential constraints, curvature constraintes... nonlinear.

};

class CubicPolynomial
{	
	matrixn coef;
public:
	class DOFsolver
	{
		// y=at^3+bt^2+ct+d
		//A(a,b,c,d)^T=b;
		matrixn A;					
		vectorn b;

		int mCurCon;
	public:
		DOFsolver(){A.setSize(4,4); b.setSize(4);mCurCon=0;}
		void conY(m_real t, m_real y)	{ A.row(mCurCon).setValues(4, CUBIC(t), SQR(t), t,1.0); b[mCurCon]=y; mCurCon++;}
		void conDY(m_real t, m_real y)	{ A.row(mCurCon).setValues(4, 3*SQR(t), 2*t,1.0,0.0); b[mCurCon]=y; mCurCon++;}
		void conDDY(m_real t, m_real y)	{ A.row(mCurCon).setValues(4, 6*t, 2.0,0.0,0.0); b[mCurCon]=y; mCurCon++;}
		void solve(vectorn& coef)		{ m::LUsolve(A,b,coef);}
	};

	std::vector<DOFsolver> mSolvers;

	CubicPolynomial(int dim){mSolvers.resize(dim);}

	void conY(m_real t, vectorn const& y)	{ for(unsigned int i=0; i<mSolvers.size(); i++) mSolvers[i].conY(t,y[i]);}
	void conDY(m_real t, vectorn const& dy)	{ for(unsigned int i=0; i<mSolvers.size(); i++) mSolvers[i].conDY(t,dy[i]);}
	void conDDY(m_real t, vectorn const& ddy){ for(unsigned int i=0; i<mSolvers.size(); i++) mSolvers[i].conDDY(t,ddy[i]);}

	void solve()
	{
		coef.setSize((int)mSolvers.size(), 4);
		for(int i=0; i<(int)mSolvers.size(); i++)
		{
			vectornView ri=coef.row(i);
			mSolvers[i].solve(ri);
		}
	}

	// time은 keytime[0]보다 크거나 같고, keytime[keytime.size()-1]보다 작거나 같은 범위에서만 동작한다.
	void getCurve(vectorn const& time, matrixn& points);
	void getFirstDeriv(vectorn const& time, matrixn& points);
	void getSecondDeriv(vectorn const& time, matrixn& points);
};
#endif
