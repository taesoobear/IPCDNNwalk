#ifndef _OPTIMIZE_H_
#define _OPTIMIZE_H_

#if _MSC_VER>1000
#pragma once
#endif

#include "OperatorStitch.h"
#ifdef USE_NR
#include "nr/nr.h"
#else
#include "Operator_NR.h"
#endif

#ifdef USE_NR

#else
typedef double  DP;
typedef vectorn Vec_I_DP;
typedef vectorn Vec_O_DP;
typedef vectorn Vec_IO_DP;
typedef vectorn Vec_DP;
#endif
class Optimize
{
	public:
	struct Opt_dimension
	{
		TString title;
		double curval;
		double max_step;
		double grad_step;
	};

	class Method
	{
		public:
		vectorn _opos;
		vectorn _x, _gradient;

		Method();
		virtual ~Method();
		virtual vectorn& getInout(){ return _x;}
		
		// implement optimize function using the following two functions.
		double func(const vectorn& x);
		double func_dfunc(const vectorn& x,vectorn& dx);

		protected:
		void prepareOptimization(Optimize& objectiveFunction);
		virtual void optimize(vectorn & initial )=0;
		void finalizeOptimization();
		friend class Optimize_impl;

		private:
		Optimize* _optimizer;
		double _oeval;
		void _getCurPos(vectorn &opos);
		void _normalize(vectorn& pos, vectorn const& opos);
		void _unnormalize(vectorn& opos, vectorn const& pos);
	};

	class ConjugateGradient: public Method
	{
		public:
		double tol, thr;
		int max_iter;
		// tol is tolerence for termination, thr is tolerence for linmin.
		ConjugateGradient(){max_iter=200, tol=0.001, thr=0.01;}
		ConjugateGradient(int maxiter){max_iter=maxiter, tol=0.001, thr=0.01;}
		protected:
		virtual void optimize(vectorn & initial);
	};

	//  a reference to the method object is stored, so do not delete the method object until optimization finishes.
	Optimize();
	void init(double stepSize, int ndim, double max_step, double grad_step, Method & method);

	Optimize(double stepSize, std::vector<Opt_dimension> const& dim, Method & method);
	Optimize(double stepSize, int ndim, double max_step, double grad_step, Method & method);
	virtual ~Optimize();
	virtual double objectiveFunction(vectorn const& pos)=0;
	virtual double gradientFunction(vectorn const& _pos, vectorn& gradient);
	
	void optimize(vectorn const& initialSolution);
	vectorn& getResult();

	bool _useNormalization;

	void _initSquareTerms(int ndim);
	double _updateSquareTermsGradient(vectorn const& pos, vectorn& gradient);
	void addSquared(intvectorn const& index, vectorn const& coef);

	struct Term {
		intvectorn indices;
		vectorn coeffs;
		double v; // cache the current value.
	};
	struct TermCoef {
		int termIndex;
		double termCoef;
	};
private:
	friend class Method;
	void* _data;
	std::vector<Opt_dimension> _opt_dimension;
	std::vector<Term> terms;
	std::vector<std::vector<TermCoef> > termCoeffs;
};

// IndexMapping can be many-to-one, or one-to-one mapping
// :: sizeRange should be smaller than or equal to sizeDomain

// in case of many-to-one mapping, inverseMapping stores only the first index.
// one-to-many mapping is also supported since inverse of one-to-many mapping is many-to-one mapping.
class IndexMapping
{
public:
	intvectorn mapping;
	intvectorn inverseMapping;

	IndexMapping(int max=0){init(max);}

	// sizeRange should be smaller than or equal to sizeDomain
	void init(int sizeDomain, int sizeRange=INT_MAX)
	{
		mapping.setSize(sizeDomain);
		if(sizeRange>sizeDomain)
			sizeRange=sizeDomain;
		inverseMapping.setSize(sizeRange);
		mapping.setAllValue(-1);	// denotes no mapping
		inverseMapping.setAllValue(-1);
	}

	int size() const {return mapping.size();}
	int sizeRange() const { return inverseMapping.size();}

	// set mapping such that map(i)==j
	void map(int i, int j)
	{
		ASSERT(mapping(i)==-1 || mapping(i)==j);
		mapping(i)=j;

		ASSERT(j>=0 && j< sizeRange());
		//if(inverseMapping(j)==-1 || inverseMapping(j)>i);
		inverseMapping(j)=i;
	}

	int operator()(int i) const	{ return mapping(i);}

	void operator()(intvectorn const& domain, intvectorn & range) const
	{
		range.reserve(domain.size());
		range.setSize(0);
		for(int i=0; i<domain.size(); i++)
		{
			int r=mapping(domain(i));
			if(r!=-1 && range.findFirstIndex(r)==-1)
				range.pushBack(r);
		}
	}

	int inverse(int j) const	{ return inverseMapping(j);}
};


// analytic constrained SQP solver
// - template argument baseSolver는
//   QuadraticFunction, SparseQuadraticFunction,
//   QuadraticFunctionHardConm, SparseQuadraticFunctionHardCon등이 사용될 수 있다.
// QuadraticFunctionHardCon이나 SparseQuadraticFunctionHardCon과는 달리
// equality constraint를 예외처리해서 실제 최적화과정에 사용되는 변수 개수를 줄인다.
// 사용법: QuadraticFunctionHardCon 이나 SparseQuadraticFunctionHardCon과 동일하지만,
// 항상 모든 addCon이 addSquared나 addSquaredWeighted보다 먼저 수행되어야한다.
template <class baseSolver>
class ConstrainedSQPsolver
{
	int mNumVar;
	int mNumCon;
	bitvectorn mEqCon;
	intvectorn mEqConIndex;
	vectorn mEqConValue;
	baseSolver* mSolver;
	IndexMapping toReorderedIndex;
	int mCurCon;
public:

	ConstrainedSQPsolver(int numVar, int numCon, int numCon2=0):mNumVar(numVar), mNumCon(numCon)
	{
		Msg::verify(numCon2==0, "linear equality constraints are not supported in this class yet - use sparseQuadraticFunctionHardCon instead.");

		mSolver=NULL;
		mEqCon.resize(numVar);
		mEqConIndex.setSize(numCon);
		mEqConValue.setSize(numCon);
		mCurCon=0;
	}
	virtual ~ConstrainedSQPsolver(){ delete mSolver;}

	void addCon(int n, m_real coef1, int index1, ...)
	{
		Msg::verify(n==1, "linear combination constraints are not supported in this class yet - use sparseQuadraticFunctionHardCon instead.");

		m_real coef2;
		va_list marker;
		va_start( marker, index1);
		coef2=va_arg(marker, m_real);
		va_end(marker);

		mEqConIndex[mCurCon]=index1;
		mEqConValue[mCurCon]=-1.0*coef2/coef1;	// ax+b=0 -> x=-b/a
		ASSERT(!mEqCon[index1]);
		mEqCon.setAt(index1);
		mCurCon++;

		if(mCurCon==mNumCon)
		{
			// index map을 구성한다.
			toReorderedIndex.init(mNumVar, mNumVar-mNumCon);

			int reorderedIndex=0;
			for(int i=0; i<mNumVar; i++)
			{
				if(mEqCon[i])
					toReorderedIndex.map(i, mEqConIndex.findFirstIndex(i));
				else
				{
					toReorderedIndex.map(i, reorderedIndex);
					reorderedIndex++;
				}
			}

			mSolver=new baseSolver(mNumVar-mNumCon);
		}
	}

	// 4*(3x+4y+5z+1)^2 를 추가하고 싶으면, addSquaredWeighted(4.0, 3, 3.0, x, 4.0, y, 5.0, z, 1.0); where x=0, y=1, z=2
	void addSquaredWeighted(m_real weight, int N, ...)
	{
		ASSERT(toReorderedIndex.size());
		QuadraticFunction::SquaredTerm* term=new QuadraticFunction::SquaredTerm;

		term->index.reserve(N);
		term->coef.reserve(N+1);


		va_list marker;
		va_start( marker, N);     /* Initialize variable arguments. */

		m_real constants=0.0;

		term->index.setSize(0);
		term->coef.setSize(0);
		int i;
		for(i=0; i<N; i++)
		{
			m_real coef=va_arg(marker, m_real);
			m_real index=va_arg(marker, int);


			if(mEqCon[index])
				constants+=coef*mEqConValue[toReorderedIndex(index)];
			else
			{
				term->coef.pushBack(coef);
				term->index.push_back(toReorderedIndex(index));
			}
		}

		term->coef.pushBack(va_arg(marker, m_real)+constants);
		va_end(marker);

		term->coef*=sqrt(weight);

		if(term->index.size())
			mSolver->mListSQTerms.push_back(term);
	}

	// (3x+4y+5z+1)^2 을 추가하고 싶으면, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
	void addSquared(int N, m_real coef1, int index1, ...)
	{

		QuadraticFunction::SquaredTerm* term=new QuadraticFunction::SquaredTerm;

		term->index.reserve(N);
		term->coef.reserve(N+1);

		va_list marker;
		va_start( marker, N);     /* Initialize variable arguments. */

		m_real constants=0.0;

		term->index.setSize(0);
		term->coef.setSize(0);
		int i;
		for(i=0; i<N; i++)
		{
			m_real coef=va_arg(marker, m_real);
			m_real index=va_arg(marker, int);


			if(mEqCon[index])
				constants+=coef*mEqConValue[toReorderedIndex(index)];
			else
			{
				term->coef.pushBack(coef);
				term->index.push_back(toReorderedIndex(index));
			}
		}

		term->coef.pushBack(va_arg(marker, m_real)+constants);
		va_end(marker);

		mSolver->mListSQTerms.push_back(term);

	}

	template <class MAT_TYPE>
	void buildSystem(MAT_TYPE & A, vectorn &b)
	{
		mSolver->buildSystem(A, b);
	}
	template <class MAT_TYPE>
	void solve(MAT_TYPE const& A, vectorn const& b, vectorn & x2)
	{
		vectorn x;
		mSolver->solve(A,b,x);

		x2.setSize(mNumVar);
		for(int i=0; i<mNumVar; i++)
		{
			if(mEqCon[i])
				x2[i]=mEqConValue[toReorderedIndex(i)];
			else
				x2[i]=x[toReorderedIndex(i)];
		}
	}
};

namespace _NRSolver
{
	// non-thread-safe
	extern void* mFunc;
}

// do not need to inherit this class. Just to demonstrate the interface.
class FuncAbstractClass
{
public:
	// return value should contain the initial solution.
	vectorn& getInout(){ ASSERT(false); return *((vectorn*)NULL);}
	inline m_real func(vectorn const& x){return 0.0;}
	inline void dfunc(vectorn const& x, vectorn& dx){}
};

// minimization of Func (conjugate gradient and gradient descent solvers are supported.)
// There also exists GSLsolver which has exactly the same interface with a lot more options.)

// usage:
// NRSolver<functionType> solver(pfunction);
// solver.conjugateGradientSolve();

template <class Func>	// Func should have three functions: getInout, func, dfunc
						//   vectorn& getInout()
						//   m_real func(vectorn const& x)
						//   void dfunc(vectorn const& x, vectorn& dx)
class NRSolver
{
public:
	NRSolver(Func* function)
	{
		_NRSolver::mFunc=(void*)function;
	}

	void conjugateGradientSolve( m_real tolerance=1.0e-6)
	{
		vectorn& x=((Func*)_NRSolver::mFunc)->getInout();
#ifdef USE_NR
		int iter; DP fret;
		NR::frprmn(x, tolerance, iter, fret, func, dfunc);
#else
		Msg::error("cgsolve");
#endif
	}

	void gradientDescentSolve( m_real ftol=1.0e-6)
	{
		int iter; DP fret;
		vectorn& x=((Func*)_NRSolver::mFunc)->getInout();
		ASSERT(0);
//		NR_OLD::gradient_descent(x, x.size(), ftol, iter, fret, func, dfunc);
	}

#ifdef USE_NR
	static DP func(Vec_I_DP & x)
	{
		return ((Func*)_NRSolver::mFunc)->func(x);
	}

	static void dfunc(Vec_I_DP &x, Vec_O_DP &dx)
	{
		((Func*)_NRSolver::mFunc)->dfunc(x, dx);
	}
#endif
};




// deprecated..
namespace NR_OLD
{
    void mnbrak(m_real&, m_real&, m_real&,
			m_real&, m_real&, m_real&, m_real (*func)(m_real)) ;

	m_real brent(m_real, m_real, m_real,
				m_real (*f)(m_real), m_real, m_real&) ;

	void frprmn(vectorn&, int, m_real, int&, m_real&,
				m_real (*func)(vectorn const&),
				m_real (*dfunc)(vectorn const&, vectorn&));

	void error( char* );

}

#include <stdio.h>
#include "../motion/IK_lbfgs/lbfgs.h"

using namespace LBFGS;
class LBFGS_METHOD: public Optimize::Method
{
protected:
    lbfgsfloatval_t *m_x;
	lbfgs_parameter_t param;
public:
    LBFGS_METHOD(double epsilon=1e-5) : m_x(NULL)
					 ,Optimize::Method()
    {
		/* Initialize the parameters for the L-BFGS optimization. */
		lbfgs_parameter_init(&param);
		param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
		param.epsilon=epsilon;
    }

    virtual ~LBFGS_METHOD()
    {
        if (m_x != NULL) {
            lbfgs_free(m_x);
            m_x = NULL;
        }
    }
protected:
	virtual void optimize(vectorn & initial)
    {

		int N=initial.size();
        lbfgsfloatval_t fx;
        lbfgsfloatval_t *m_x = lbfgs_malloc(N);

        if (m_x == NULL) {
            printf("ERROR: Failed to allocate a memory block for variables.\n");
            return ;
        }

        /* Initialize the variables. */
        for (int i = 0;i < N;i ++) {
            m_x[i] = initial[i];
        }


        /*
            Start the L-BFGS optimization; this will invoke the callback functions
            evaluate() and progress() when necessary.
         */
        int ret = lbfgs(N, m_x, &fx, _evaluate, _progress, this, &param);

#ifdef _DEBUG
        /* Report the result. */
        printf("L-BFGS optimization terminated with status code = %d\n", ret);
        //printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, m_x[0], m_x[1]);
#endif

        for (int i = 0;i < N;i ++) {
            initial[i]=m_x[i] ;
        }
    }

    static lbfgsfloatval_t _evaluate(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
        )
    {
        return reinterpret_cast<LBFGS_METHOD*>(instance)->evaluate(x, g, n, step);
    }

    lbfgsfloatval_t evaluate(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
        )
    {
        lbfgsfloatval_t fx = 0.0;

		vectornView mx((double*)x, n, 1);
		vectornView mg((double*)g, n, 1);

		return func_dfunc(mx, mg);
    }

    static int _progress(
        void *instance,
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
        return reinterpret_cast<LBFGS_METHOD*>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
    }

    int progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
#ifdef _DEBUG
        printf("Iteration %d:\n", k);
        printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
        printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
        printf("\n");
#endif
        return 0;
    }
};





#endif
