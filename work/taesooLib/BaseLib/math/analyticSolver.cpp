#include "stdafx.h"
#include "analyticSolver.h"
#include "Operator.h"
#include "Operator_NR.h"
//#include "GnuPlot.h"
#include <vector>
#include <algorithm>
namespace analyticSolver
{
	namespace FunctionGenerator
	{
		class polySolver
		{
			// y=at^0+bt+ct^2+dt^3
			//A(a,b,c,d)^T=b;
			matrixn A;					
			vectorn b;

			mutable vectorn temp;
			int mCurCon;

			void deriv(vectorn &temp, vectorn& dtemp)
			{
				dtemp.setSize(temp.size());
				dtemp[0]=0;
				for(int i=1; i<temp.size(); i++)
					dtemp[i]=temp[i-1]*i;
			}
		public:
			polySolver(int maxDegree){A.setSize(maxDegree+1,maxDegree+1); b.setSize(maxDegree+1);mCurCon=0;}
			void conY(m_real t, m_real y)	
			{
				Polynomial::calcTemp(A.rows()-1, t, temp);
				A.row(mCurCon)=temp;
				b[mCurCon]=y; mCurCon++;
			}
			
			void conDY(m_real t, m_real y)	
			{
				Polynomial::calcTemp(A.rows()-1, t, temp);
				vectornView a_i=A.row(mCurCon);
				deriv(temp, a_i);
				b[mCurCon]=y;
				mCurCon++;
			}
			
			void solve(vectorn& coef)		{ m::LUsolve(A,b,coef);}
		};

		polyGenerator::polyGenerator(int degree, m_real start, m_real end)
			:mStart(start), mEnd(end)
		{
			pSolver=new polySolver(degree);
		}
		polyGenerator::~polyGenerator(){delete pSolver;}
		void polyGenerator::conY(m_real t, m_real y)
		{
			pSolver->conY(t,y);
		}
		void polyGenerator::conDY(m_real t, m_real y)
		{
			pSolver->conDY(t,y);
		}

		Function1D* polyGenerator::generate()
		{
			vectorn coef;
			pSolver->solve(coef);
			return new Polynomial(mStart, mEnd, coef);
		}
		Function1D* linearFunction(m_real start, m_real end, m_real p1, m_real p2)
		{
			vectorn coef;
			polySolver p(1);
			p.conY(start, p1);
			p.conY(end, p2);
			p.solve(coef);
			return new Polynomial(start, end, coef);
		}
		Function1D* cubicFunction(m_real start, m_real end, m_real p1, m_real v1, m_real p2, m_real v2)
		{
			vectorn coef;
			polySolver p(3);
			p.conY(start, p1);
			p.conY(end, p2);
			p.conDY(start, 0);
			p.conDY(end, 0);
			p.solve(coef);
			return new Polynomial(start, end, coef);
		}
	};

	void Polynomial::calcTemp(int maxDegree, m_real t, vectorn& temp)
	{
		temp.setSize(maxDegree+1);
		temp[0]=1.0;
		for(int i=1; i<temp.size(); i++)
			temp[i]=t*temp[i-1];			
	}

	m_real Polynomial::y(m_real t)
	{
		calcTemp(coef.size()-1, t, temp);
		return coef%temp;
	}

	Function1D* Polynomial::integralY(m_real startValue)
	{
		// y=a+bx+cx^2+dx^3 -> coef=[a,b,c,d]
		vectorn coef2;
		coef2.setSize(coef.size()+1);
		coef2[0]=0.0;
		for(int i=0; i<coef.size(); i++)
			coef2[i+1]=coef[i]/m_real(i+1);

		Function1D* func=new Polynomial(start, end, coef2);
		((Polynomial*)func)->coef[0]+=startValue-func->y(start);
		return func;
	}

	Function1D* Polynomial::dY()
	{
		// y=a+bx+cx^2+dx^3 -> coef=[a,b,c,d]
		// -> b+2cx+3dx^2
		vectorn coef2;
		coef2.setSize(coef.size());
		
		for(int i=0; i<coef.size()-1; i++)
			coef2[i]=coef[i+1]*m_real(i+1);
		coef2[coef2.size()-1]=0.0;

		Function1D* func=new Polynomial(start, end, coef2);
		return func;
	}

	struct setNULL: public std::unary_function<Function1D*, void>
	{
	  void operator() (Function1D*& x) { x=NULL;}
	};

	struct deletePiece: public std::unary_function<Function1D*, void>
	{
	  void operator() (Function1D*& x) { delete x; x=NULL;}
	};

	PiecewiseFunction1D::PiecewiseFunction1D(vectorn const& domain)
		:mDomainControl(domain)
	{
		mPieces.resize(mDomainControl.size()-1);
		std::for_each(mPieces.begin(), mPieces.end(), setNULL());
	}
	
	PiecewiseFunction1D::~PiecewiseFunction1D()
	{
		std::for_each(mPieces.begin(), mPieces.end(), deletePiece());
	}

	void PiecewiseFunction1D::init(vectorn const& domain)
	{
		mDomainControl=domain;
		std::for_each(mPieces.begin(), mPieces.end(), deletePiece());
		mPieces.resize(mDomainControl.size()-1);
		std::for_each(mPieces.begin(), mPieces.end(), setNULL());
	}

	int PiecewiseFunction1D::piece(m_real domain)
	{
		int i;
		for(i=1; i<mDomainControl.size()-1; i++)
			if(domain<mDomainControl[i])
				return i-1;
		return i-1;
	}
	m_real PiecewiseFunction1D::value(m_real domain)
	{
		return mPieces[piece(domain)]->y(domain);
	}

	void PiecewiseFunction1D::values(vectorn const& domain, vectorn & values)
	{
		values.setSize(domain.size());

		for(int i=0; i<domain.size(); i++)
			values[i]=value(domain[i]);
	}

	void PiecewiseFunction1D::integral(PiecewiseFunction1D& func, m_real StartValue)
	{
		func.init(mDomainControl);
		m_real prevY=0.0;
		for(int i=0; i<func.numPieces(); i++)
		{
			func.mPieces[i]=mPieces[i]->integralY(prevY);
			prevY=func.mPieces[i]->y(func.mPieces[i]->end);
		}
	}

	void PiecewiseFunction1D::derivative(PiecewiseFunction1D& func)
	{
		func.init(mDomainControl);
		for(int i=0; i<func.numPieces(); i++)
			func.mPieces[i]=mPieces[i]->dY();
	}

	/*
	void func_test()
	{

		analyticSolver::PiecewiseFunction1D func(vectorn(3, 0, 20, 40));

		func.mPieces[0]=analyticSolver::FunctionGenerator::linearFunction(0, 20, 2, 17);
		func.mPieces[1]=analyticSolver::FunctionGenerator::cubicFunction(20, 40, 17, 0, 0, 0);

		matrixn data(40,2);
		data.column(0).colon(0, 1.0, 40);
		vectornView c0=data.column(0);
		vectornView c1=data.column(1);
		func.values(c0, c1);
		gnuPlot::plot2DSignal("funcTest", data);

		analyticSolver::PiecewiseFunction1D funcDeriv;
		func.derivative(funcDeriv);
		funcDeriv.values(c0, c1);
		gnuPlot::plot2DSignal("funcDerivTest", data);

		analyticSolver::PiecewiseFunction1D funcInt;
		func.integral(funcInt,0.0);
		funcInt.values(c0, c1);
		gnuPlot::plot2DSignal("funcIntTest", data);
	}
	*/
}
