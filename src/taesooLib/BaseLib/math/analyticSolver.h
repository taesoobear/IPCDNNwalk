#ifndef _ANALYTIC_SOLVER_H_
#define _ANALYTIC_SOLVER_H_
#pragma once

namespace analyticSolver
{
	class Function1D
	{
	public:
		Function1D(int s, int e):start(s), end(e){}
		m_real start;
		m_real end;
		virtual m_real y(m_real t)	{ throw "y"; return 0;}
		virtual Function1D* integralY(m_real startValue)=0;
		virtual Function1D* dY()=0;
	};

	namespace FunctionGenerator
	{
		class polySolver;
		class polyGenerator
		{
			polySolver* pSolver;
			m_real mStart, mEnd;
		public:
			polyGenerator(int degree, m_real start, m_real end);
			~polyGenerator();
			void conY(m_real t, m_real y);
			void conDY(m_real t, m_real y);
			Function1D* generate();
		};

		Function1D* linearFunction(m_real start, m_real end, m_real p1, m_real p2);
		Function1D* cubicFunction(m_real start, m_real end, m_real p1, m_real v1, m_real p2, m_real v2);
	};

	class Polynomial:public Function1D
	{
		// y=a+bx+cx^2+dx^3 -> coef=[a,b,c,d]
		mutable vectorn temp;	// temporary. (1, t, t^2, t^3, ...)
		public:
		vectorn coef;
		Polynomial(int s, int e, vectorn const& coeff):Function1D(s,e),coef(coeff){}

		// temporary. (1, t, t^2, t^3, ...)
		static void calcTemp(int maxDegree, m_real t, vectorn& temp);
		
		virtual m_real y(m_real t);
		virtual Function1D* integralY(m_real startValue);
		virtual Function1D* dY();
	};

	class PiecewiseFunction1D
	{		
	public:
		// piecewise function을 지원한다. 단 각 piece는 무한 미분 가능하다고 가정한다.
		// analytic 미분과 적분을 지원.
		PiecewiseFunction1D(vectorn const& domainControl);
		PiecewiseFunction1D(){}
		~PiecewiseFunction1D();

		void init(vectorn const& domainControl);

		int numPieces()	{return mDomainControl.size()-1;}

		m_real value(m_real domain);
		// domain이 증가순이라고 가정. 조금 더 효율적으로 구현가능.
		void values(vectorn const& domain, vectorn & values);
		// continous함수를 가정하고 적분한다. 
		void integral(PiecewiseFunction1D& func, m_real StartValue);
		void derivative(PiecewiseFunction1D& func);


		std::vector<Function1D*> mPieces;
	private:
		int piece(m_real domain);		
		vectorn mDomainControl;
	};

	void func_test();
}
#endif
