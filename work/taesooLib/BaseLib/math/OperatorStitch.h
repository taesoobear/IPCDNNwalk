#ifndef _OPERATORSTITCH_H_
#define _OPERATORSTITCH_H_

#if _MSC_VER>1000
#pragma once
#endif

// analytic constrained SQP solver utility written by Taesoo.
class HessianQuadratic
{
public:
	matrixn H;	// Hessian	(1차항 계수)
	vectorn R;	// Hessian residual	 (2차항 계수)
	HessianQuadratic(int dim);

	// (3x+4y+5z+1)^2 을 추가하고 싶으면, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
	void addSquared(int n, m_real coef1, int index1, ...);

	// 4*(3x+4y+5z+1)^2 를 추가하고 싶으면, addSquaredWeighted(4.0, 3, 3.0, x, 4.0, y, 5.0, z, 1.0); where x=0, y=1, z=2
	void addSquaredWeighted(m_real weight, int n, m_real coef1, int index1, ...);

	// (3x+4y+5z+1)^2 을 추가하고 싶으면, addSquared((0,1,2), (3,4,5,1))
	void addSquared(intvectorn const& index, vectorn const& value);
	void addSquaredH(intvectorn const& index, vectorn const& value);
	void addSquaredR(intvectorn const& index, vectorn const& value);
};

// numerical/analytic unconstrained SQP solver utility written by Taesoo.
class QuadraticFunction
{
public:

	QuadraticFunction(){}
	virtual ~QuadraticFunction();

	// (3x+4y+5z+1)^2 을 추가하고 싶으면, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
	void addSquared(int n, m_real coef1, int index1, ...);

	void addSquared(intvectorn const& index, vectorn const& value);

	// 4*(3x+4y+5z+1)^2 를 추가하고 싶으면, addSquaredWeighted(4.0, 3, 3.0, x, 4.0, y, 5.0, z, 1.0); where x=0, y=1, z=2
	void addSquaredWeighted(m_real weight, int n, m_real coef1, int index1, ...);

	m_real func(vectorn const& x);
	void dfunc(vectorn const& x, vectorn& dx);
	void dfunc(vectorn const& x, vectorn& dx, m_real scale);
	void buildSystem(int nvar, matrixn & A, vectorn &b);
	void buildSystem_A(int nvar, matrixn & A);
	void buildSystem_B(int nvar, vectorn & B);

	struct SquaredTerm
	{
		intvectorn index;
		vectorn coef;

		m_real fSquared(vectorn const& x);
		void dfSquared(vectorn const& x, vectorn& dx);
		void dfSquared(vectorn const& x, vectorn& dx, m_real scale);
	};

	std::list<SquaredTerm*> mListSQTerms;
	vectorn _values; // updated when buildSystem_A is called
};

class QuadraticFunctionSoftCon : public QuadraticFunction
{
	int mNumVar;
	int mNumCon;
	m_real mCoef;

public:
	QuadraticFunctionSoftCon (int numVar, int numCon, m_real conCoef=1000):mNumVar(numVar), mNumCon(numCon) { mCoef=sqrt(conCoef);}
	virtual ~QuadraticFunctionSoftCon (){}

	// (3x+4y+5z+1==0) 을 추가 하고 싶으면, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);
	void addCon(int n, m_real coef1, int index1, ...);
};


// analytic constrained SQP solver.
// numerical optimization(func/dfunc)은 제대로 동작 안함. - Lagrange multiplier알고리즘 때문인 듯.
class QuadraticFunctionHardCon : public QuadraticFunction
{
	
public:
	int mNumVar;
	int mNumCon;
	typedef matrixn MAT_TYPE;	// traits.
	struct Con
	{
		intvectorn index;
		vectorn coef;
		m_real func(vectorn const& x, int numVar_plus_conIndex);
		void dfunc(vectorn const& x, vectorn& dx, int numVar_plus_conIndex);
	};

	std::list<Con*> mListConTerms;

	QuadraticFunctionHardCon (int numVar, int numCon):mNumVar(numVar), mNumCon(numCon){}
	virtual ~QuadraticFunctionHardCon ();

	void addCon(int n, m_real coef1, int index1, ...);
	void addCon(intvectorn const& index, vectorn const& value);

	m_real func(vectorn const& x);
	void dfunc(vectorn const& x, vectorn& dx);
	void buildSystem(matrixn & A, vectorn &b);
	void updateSystem(const vectorn& con_values, vectorn &b);
};

class AnalyticGradient
{
public:
	struct Base
	{
		Base(){}
		virtual ~Base(){}
		virtual m_real func(vectorn const& x)								{ Msg::error("func should be reimplemented");return 0.0;}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0)	{ Msg::error("dfunc should be reimplemented");}
		virtual Base* copy () const											{ Msg::error("copy has not been reimplemented"); return NULL;}
	};

	struct QuadraticFunction : Base
	{
		::QuadraticFunction f;

		virtual m_real func(vectorn const& x){return f.func(x);}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0){f.dfunc(x,dx, scale);}
	};

	struct LinearTerm : Base
	{
		intvectorn index;
		vectorn coef;
		// (3x+4y+5z+1) -> LinearTerm(3,  3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
		LinearTerm(int n, m_real coef1, int index1, ...);

		virtual m_real func(vectorn const& x);
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);
		virtual Base* copy () const	{ return (Base*)new LinearTerm(*this); }
	};

	struct MultiplyTwo : Base
	{
		Base* f, * g;
		MultiplyTwo(Base* func1, Base* func2):f(func1), g(func2){}
		virtual ~MultiplyTwo()	{ delete f; delete g;}

		virtual m_real func(vectorn const& x){return f->func(x)*g->func(x);}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);
		virtual Base* copy () const	{ return (Base*)new MultiplyTwo(f->copy(), g->copy()); }
	};

	struct SQRT : Base
	{
		Base* f;
		SQRT(Base* func):f(func){}
		virtual ~SQRT(){delete f;}
		virtual m_real func(vectorn const& x){return sqrt(f->func(x));}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);
		virtual Base* copy () const	{ return (Base*)new SQRT(f->copy());}
	};

	struct Squared : Base
	{
		Base* f;
		Squared(Base* func):f(func){}
		virtual ~Squared(){delete f;}
		virtual m_real func(vectorn const& x){m_real v=f->func(x); return v*v;}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);
		virtual Base* copy () const	{ return (Base*)new Squared(f->copy());}
	};

	struct Pow : Base
	{
		Base* f;
		m_real exponent;
		Pow(Base* func, m_real exp):f(func), exponent(exp){}
		virtual ~Pow(){delete f;}
		virtual m_real func(vectorn const& x){m_real v=f->func(x); return pow(v, exponent);}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);
		virtual Base* copy () const	{ return (Base*)new Pow(f->copy(), exponent);}
	};

	struct LinearCombination : Base
	{
		LinearCombination() :Base(){mCoef=0.0;}
		struct Term
		{
			Term(m_real coef):mTerm(NULL), mCoef(coef){}
			~Term(){delete mTerm;}
			m_real mCoef;
			Base* mTerm;
		};

		void add(Base* pTerm, m_real coef=1.0);
		void addSqrt(Base* pTerm, m_real coef=1.0);
		void addSQR(Base* pTerm, m_real coef=1.0);
		void add(m_real coef){ mCoef+=coef;}

		virtual m_real func(vectorn const& x);
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);

		std::list<Term> mListTerms;
		m_real mCoef;

		virtual Base* copy() const ;
	};

	void add(Base* pTerm, m_real coef=1.0)		{ t.add(pTerm, coef);}
	void addSqrt(Base* pTerm, m_real coef=1.0)	{ t.addSqrt(pTerm, coef);}
	void addSQR(Base* pTerm, m_real coef=1.0)	{ t.addSQR(pTerm, coef);}


	m_real func(vectorn const& x)	{ return t.func(x);}

	inline void dfunc(vectorn const& x, vectorn &dx)
	{
		dx.setSize(x.size());
		dx.setAllValue(0.0);

		dfunc_noinit(x,dx);
	}

	// assumes that dx are already initizialized to zero vector
	void dfunc_noinit(vectorn const& x, vectorn& dx) { t.dfunc(x,dx);}


private:
	LinearCombination t;

};

#include "math_macro.h"

namespace m
{

	void _c0stitch(matrixn& c, int discontinuity, m_real strength=2.0);

	struct stitchOp
	{
		stitchOp(){}
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const=0;
	};
	struct linstitch: stitchOp
	{
		m_real mStrength;
		// strength가 클수록 c0stitch에 가까워진다.(overshooting확률이 적어진다.) 0보다 큰값을 대입해야한다.
		linstitch(m_real strength=5):mStrength(strength){}
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};
	// for multi dimensional signal. (each DOF is dependent)
	struct linstitchMulti: stitchOp 
	{
		m_real mStrength;
		bool mbConMid;
		// strength가 클수록 c0stitch에 가까워진다.(overshooting확률이 적어진다.) 0보다 큰값을 대입해야한다.
		linstitchMulti(m_real strength=5, bool bConMid=true):mStrength(strength), mbConMid(bConMid){}
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};


	struct c1stitchPreprocess : stitchOp
	{
		int mArow;
		int mBrow;
		bool mbConMid;
		matrixn Augmented;
		matrixn invAugmented;
		vectorn weight;

		// working space (temporary)
		mutable vectorn cc;
		mutable HessianQuadratic h;
		mutable vectorn d;
		mutable intvectorn index;
		mutable vectorn coef;
		mutable vectorn x;

		m_real mStrength;
		c1stitchPreprocess(int arow, int brow, m_real strength=2.0, bool bConMid=true);
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c1stitchPreprocessOnline: stitchOp

	{
		int mArow;
		int mBrow;
		matrixn Augmented;
		matrixn invAugmented;
		vectorn weight;

		// working space (temporary)
		mutable vectorn cc;
		mutable HessianQuadratic h;
		mutable vectorn d;
		mutable intvectorn index;
		mutable vectorn coef;
		mutable vectorn x;

		m_real mStrength;
		c1stitchPreprocessOnline(int arow, int brow, m_real strength=2.0);
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};



	struct linstitchPreprocessInc: stitchOp

	{
		int mArow;
		int mBrow;
		matrixn Augmented;
		matrixn invAugmented;
		vectorn weight;

		// working space (temporary)
		mutable vectorn cc,dd;
		mutable HessianQuadratic h;
		mutable vectorn d;
		mutable intvectorn index;
		mutable vectorn coef;
		mutable vectorn x;
		mutable matrixn aa,bb;


		int mMaxIter;
		int mNormalize;
		// strength가 클수록 c0stitch에 가까워진다.(overshooting확률이 적어진다.) 0보다 큰값을 대입해야한다.
		linstitchPreprocessInc(int arow, int brow, int nIter, int normalize=0,m_real strength=5);
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};


	struct linstitchPreprocess: stitchOp

	{
		int mArow;
		int mBrow;
		bool mbConMid;
		matrixn Augmented;
		matrixn invAugmented;
		vectorn weight;

		// working space (temporary)
		mutable vectorn cc;
		mutable HessianQuadratic h;
		mutable vectorn d;
		mutable intvectorn index;
		mutable vectorn coef;
		mutable vectorn x;


		// strength가 클수록 c0stitch에 가까워진다.(overshooting확률이 적어진다.) 0보다 큰값을 대입해야한다.
		linstitchPreprocess(int arow, int brow, m_real strength=5, bool bConMid=true);
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	// 시작점 끝점에서 위치, 속도를 constraint로 가속도 차이를 minimize.
	struct linstitch2: stitchOp

	{
		linstitch2(){}
		// a의 마지막 두 프레임이 b의 첫 두프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-2이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	// 시작점 끝점에서 위치, 속도를 constraint로 가속도 차이를 minimize.
	struct linstitchOnline: stitchOp
	{
		m_real mStrength;
		linstitchOnline(m_real strength=5.0):mStrength(strength){}
		// a의 마지막 프레임이 b의 첫 프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		// 단, a는 전혀 바뀌지 않는다. 즉 b만 수정해서 stitch
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	// 시작점 끝점에서 위치, 속도를 constraint로 가속도 차이를 minimize.
	struct linstitchForward : stitchOp
	{
		linstitchForward(){}
		// a의 마지막 프레임이 b의 첫 프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		// 단, b는 전혀 바뀌지 않는다. 즉 b만 수정해서 stitch
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c0concat: stitchOp
	{
		c0concat(){}
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 concat. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c0stitch: stitchOp

	{
		c0stitch(){}
		// a의 첫프레임과 b의 마지막 프레임은 변하지 않는다.
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c0stitchOnline: stitchOp
	{
		c0stitchOnline(){}
		// a의 첫프레임과 b의 마지막 프레임은 변하지 않는다.
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c0stitchPreserve2: stitchOp
	{
		c0stitchPreserve2(){}
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		// a의 첫두프레임과 b의 마지막 두프레임은 변하지 않는다.
		// linstitch도 첫 두프레임, 마지막 두프레임이 변하지 않기 위해 일관성을 위해 구현.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	// quaterN이 n개 붙어있는 matrix의 스티칭. (c는 a.rows()+b.rows()-1 by 4n matrix가 된다.)
	struct stitchQuaterNN : stitchOp
	{
		int mPreserveAmount;
		void (quaterN::*mFunc)(quaterN const&, quaterN const& );
		stitchQuaterNN (void (quaterN::*func)(quaterN const&, quaterN const& ), int preserveAmount=0)
			: mFunc(func),mPreserveAmount(preserveAmount){}
		virtual void calc(matrixn& c, const matrixn& a, const matrixn&b )const;
	};

	struct c1stitch: stitchOp
	{
		m_real mStrength;
		c1stitch(m_real strength=2.0):mStrength(strength){}
		// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};
}

void quater_linstitch(m::stitchOp const& op, quaterN& c, quaterN const& a, quaterN const& b);
// 여러개의 quaterN을 모아논 matrix를 stitch. (fast)
// quaternion align하느라 a와 b가 바뀔수 있음.
void quaterNN_linstitch(m::stitchOp const& op, matrixn& c, matrixn & a, matrixn & b);

#endif
