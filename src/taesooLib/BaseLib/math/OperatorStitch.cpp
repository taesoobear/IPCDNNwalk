#include "stdafx.h"
#include "mathclass.h"
#include "OperatorStitch.h"
#include "Operator.h"
#include "Operator_NR.h"
#include "Filter.h"
#include "BSpline.h"
#ifdef USE_NR
#include "nr/nr.h"
#else
#include "../../PhysicsLib/TRL/eigenSupport.h"
#endif
#include "conversion.h"
#include "../utility/operatorString.h"

#ifdef USE_NR
#define USE_LUDCMP
#endif

HessianQuadratic::HessianQuadratic(int dim)
{
	H.setSize(dim, dim);
	R.setSize(dim);
	H.setAllValue(0);
	R.setAllValue(0);
}

struct HessianQuadratic_term { m_real c; int i;};
void HessianQuadratic::addSquared(int N, m_real coef1, int index1, ...)
{
	const int MAX_TERM=10;
	static HessianQuadratic_term term[MAX_TERM+1];
	ASSERT(N<MAX_TERM);

	term[0].c=coef1;
	term[0].i=index1;

	va_list marker;
	va_start( marker, index1);     /* Initialize variable arguments. */

	int i;
	for(i=1; i<N; i++)
	{
		term[i].c=va_arg(marker, m_real);
		term[i].i=va_arg(marker, int);
	}

	term[i].c=va_arg(marker, m_real);
	va_end(marker);

	// update H
	for(int i=0; i<N; i++)
		H[term[i].i][term[i].i]+=2.0*SQR(term[i].c);
	for(int i=0; i<N; i++)
		for(int j=i+1; j<N; j++)
		{
			m_real prev=H[term[j].i][term[i].i];
			H[term[i].i][term[j].i]=H[term[j].i][term[i].i]=prev+2.0*term[i].c*term[j].c;
		}

	// update R
	for(int i=0; i<N; i++)
	{
		R[term[i].i]-=2.0*term[N].c*term[i].c;
	}
}

void HessianQuadratic::addSquaredWeighted(m_real weight, int N, m_real coef1, int index1, ...)
{
	const int MAX_TERM=10;
	static HessianQuadratic_term term[MAX_TERM+1];
	ASSERT(N<MAX_TERM);

	m_real sqWeight=sqrt(weight);


	term[0].c=coef1*sqWeight;
	term[0].i=index1;

	va_list marker;
	va_start( marker, index1);     /* Initialize variable arguments. */


	int i;
	for(i=1; i<N; i++)
	{
		term[i].c=va_arg(marker, m_real);
		term[i].c*=sqWeight;
		term[i].i=va_arg(marker, int);
	}

	term[i].c=va_arg(marker, m_real);
	term[i].c*=sqWeight;
	va_end(marker);


	// update H
	for(int i=0; i<N; i++)
		H[term[i].i][term[i].i]+=2.0*SQR(term[i].c);
	for(int i=0; i<N; i++)
		for(int j=i+1; j<N; j++)
		{
			m_real prev=H[term[j].i][term[i].i];
			H[term[i].i][term[j].i]=H[term[j].i][term[i].i]=prev+2.0*term[i].c*term[j].c;
		}

	// update R
	for(int i=0; i<N; i++)
	{
		R[term[i].i]-=2.0*term[N].c*term[i].c;
	}

}

// (3x+4y+5z+1)^2 을 추가하고 싶으면, addSquared((0,1,2), (3,4,5,1))
void HessianQuadratic::addSquared(intvectorn const& index, vectorn const& value)
{
	// update H
	addSquaredH(index, value.range(0, index.size()));
	// update R
	addSquaredR(index, value);
}


void HessianQuadratic::addSquaredH(intvectorn const& index, vectorn const& value)
{
	for(int i=0; i<index.size(); i++)
		H[index[i]][index[i]]+=2.0*SQR(value[i]);
	for(int i=0; i<index.size(); i++)
		for(int j=i+1; j<index.size(); j++)
		{
			m_real prev=H[index[j]][index[i]];
			H[index[i]][index[j]]=H[index[j]][index[i]]=prev+2.0*value[i]*value[j];
		}
}

void HessianQuadratic::addSquaredR(intvectorn const& index, vectorn const& value)
{
	for(int i=0; i<index.size(); i++)
	{
		R[index[i]]-=2.0*value[index.size()]*value[i];
	}
}


m_real QuadraticFunction::func(vectorn const& x)		
{
	double f=0.0;

	for(std::list<SquaredTerm*>::iterator i=mListSQTerms.begin(); i!=mListSQTerms.end(); ++i)
		f+=(*i)->fSquared(x);

	//	printf("%f ", f);
	//	fflush(stdout);
	return f;
}

void QuadraticFunction::dfunc(vectorn const& x, vectorn& dx)
{
	dx.setSize(x.size());
	dx.setAllValue(0);

	for(std::list<SquaredTerm*>::iterator i=mListSQTerms.begin(); i!=mListSQTerms.end(); ++i)
		(*i)->dfSquared(x, dx);
}

void QuadraticFunction::dfunc(vectorn const& x, vectorn& dx, m_real scale)
{
	for(std::list<SquaredTerm*>::iterator i=mListSQTerms.begin(); i!=mListSQTerms.end(); ++i)
		(*i)->dfSquared(x, dx, scale);
}

QuadraticFunction::~QuadraticFunction()
{
	for(std::list<SquaredTerm*>::iterator i=mListSQTerms.begin(); i!=mListSQTerms.end(); ++i)
		delete (*i);
}

m_real QuadraticFunction::SquaredTerm::fSquared(vectorn const& x)
{
	m_real f=0;
	for(int i=0; i<index.size(); i++)
		f+=coef[i]*x[index[i]];

	f+=coef[index.size()];
	return SQR(f);
}

void QuadraticFunction::SquaredTerm::dfSquared(vectorn const& x, vectorn& dx)
{
	//9x^2+ 24*xy + ... +6x+... 의 편미분.
	for(int i=0; i<index.size(); i++)
		dx[index[i]]+=2.0*SQR(coef[i])*x[index[i]];

	for(int i=0; i<index.size(); i++)
		for(int j=i+1; j<index.size(); j++)
		{
			dx[index[i]]+=2.0*coef[i]*coef[j]*x[index[j]];
			dx[index[j]]+=2.0*coef[i]*coef[j]*x[index[i]];
		}

	for(int i=0; i<index.size(); i++)
	{
		dx[index[i]]+=2.0*coef[index.size()]*coef[i];
	}
}

void QuadraticFunction::SquaredTerm::dfSquared(vectorn const& x, vectorn& dx, m_real scale)
{
	//9x^2+ 24*xy + ... +6x+... 의 편미분.
	for(int i=0; i<index.size(); i++)
		dx[index[i]]+=2.0*SQR(coef[i])*x[index[i]]*scale;

	for(int i=0; i<index.size(); i++)
		for(int j=i+1; j<index.size(); j++)
		{
			dx[index[i]]+=2.0*coef[i]*coef[j]*x[index[j]]*scale;
			dx[index[j]]+=2.0*coef[i]*coef[j]*x[index[i]]*scale;
		}

	for(int i=0; i<index.size(); i++)
	{
		dx[index[i]]+=2.0*coef[index.size()]*coef[i]*scale;
	}
}


void QuadraticFunction::addSquared(intvectorn const& index, vectorn const& value)
{
	SquaredTerm* term=new SquaredTerm;
	mListSQTerms.push_back(term);

	ASSERT(value.size()==index.size()+1);

	term->index=index;
	term->coef=value;
}

void QuadraticFunction::addSquared(int n, m_real coef1, int index1, ...)
{
	SquaredTerm* term=new SquaredTerm;
	mListSQTerms.push_back(term);

	term->index.setSize(n);
	term->coef.setSize(n+1);

	term->coef[0]=coef1;
	term->index[0]=index1;

	va_list marker;
	va_start( marker, index1);     /* Initialize variable arguments. */

	int i;
	for(i=1; i<n; i++)
	{
		term->coef[i]=va_arg(marker, m_real);
		term->index[i]=va_arg(marker, int);
	}

	term->coef[i]=va_arg(marker, m_real);
	va_end(marker);
}

void QuadraticFunction::addSquaredWeighted(m_real weight, int n, m_real coef1, int index1, ...)
{
	SquaredTerm* term=new SquaredTerm;
	mListSQTerms.push_back(term);

	term->index.setSize(n);
	term->coef.setSize(n+1);

	term->coef[0]=coef1;
	term->index[0]=index1;

	va_list marker;
	va_start( marker, index1);     /* Initialize variable arguments. */

	int i;
	for(i=1; i<n; i++)
	{
		term->coef[i]=va_arg(marker, m_real);
		term->index[i]=va_arg(marker, int);
	}

	term->coef[i]=va_arg(marker, m_real);
	va_end(marker);

	term->coef*=sqrt(weight);
}


void QuadraticFunctionSoftCon::addCon(int n, m_real coef1, int index1, ...)
{
	SquaredTerm* term=new SquaredTerm;
	mListSQTerms.push_back(term);

	term->index.setSize(n);
	term->coef.setSize(n+1);

	term->coef[0]=coef1;
	term->index[0]=index1;

	va_list marker;
	va_start( marker, index1);     /* Initialize variable arguments. */

	int i;
	for(i=1; i<n; i++)
	{
		term->coef[i]=va_arg(marker, m_real);
		term->index[i]=va_arg(marker, int);
	}

	term->coef[i]=va_arg(marker, m_real);
	va_end(marker);

	term->coef*=mCoef;
}

void AnalyticGradient::LinearCombination::add(Base* pTerm, m_real coef)
{
	mListTerms.push_back(Term(coef));
	mListTerms.back().mTerm=pTerm;
}

void AnalyticGradient::LinearCombination::addSqrt(Base* pTerm, m_real coef)
{
	mListTerms.push_back(Term(coef));
	mListTerms.back().mTerm=new SQRT(pTerm);
}

void AnalyticGradient::LinearCombination::addSQR(Base* pTerm, m_real coef)
{
	mListTerms.push_back(Term(coef));
	mListTerms.back().mTerm=new Squared(pTerm);
}

void AnalyticGradient::MultiplyTwo::dfunc(vectorn const& x, vectorn& dx, m_real scale)
{
	m_real fv=f->func(x);
	m_real gv=g->func(x);
	f->dfunc(x,dx, scale*gv);
	g->dfunc(x,dx, scale*fv);
}

AnalyticGradient::LinearTerm::LinearTerm(int n, m_real coef1, int index1, ...)
{
	index.setSize(n);
	coef.setSize(n+1);

	coef[0]=coef1;
	index[0]=index1;

	va_list marker;
	va_start( marker, index1);     /* Initialize variable arguments. */

	int i;
	for(i=1; i<n; i++)
	{
		coef[i]=va_arg(marker, m_real);
		index[i]=va_arg(marker, int);
	}

	coef[i]=va_arg(marker, m_real);
	va_end(marker);	
}

m_real AnalyticGradient::LinearTerm::func(vectorn const& x)
{
	m_real v=0.0;
	for(int i=0; i<index.size();i++)
	{
		v+=coef[i]*x[index[i]];
	}
	v+=coef[index.size()];
	return v;
}

void AnalyticGradient::LinearTerm::dfunc(vectorn const& x, vectorn& dx, m_real scale)
{
	for(int i=0; i<index.size(); i++)
		dx[index[i]]+=coef[i]*scale;
}

void AnalyticGradient::SQRT::dfunc(vectorn const& x, vectorn& dx, m_real scale)
{
	m_real rhcp=sqrt(f->func(x));
	f->dfunc(x, dx, 0.5*scale/rhcp);
}

void AnalyticGradient::Squared::dfunc(vectorn const& x, vectorn& dx, m_real scale)
{
	m_real v=f->func(x);
	f->dfunc(x, dx, scale*2.0*v);
}

void AnalyticGradient::Pow::dfunc(vectorn const& x, vectorn& dx, m_real scale)
{
	m_real v=f->func(x);
	f->dfunc(x, dx, scale*exponent*pow(v, exponent-1.0));
}

AnalyticGradient::Base* AnalyticGradient::LinearCombination::copy() const 
{
	LinearCombination* pNew=new LinearCombination(*this); 

	for(std::list<Term>::iterator i= pNew->mListTerms.begin(); i!=pNew->mListTerms.end(); i++)
	{
		(*i).mTerm=(*i).mTerm->copy();
	}

	return pNew;
}

m_real AnalyticGradient::LinearCombination::func(vectorn const& x)
{
	m_real f=0.0;

	for(std::list<Term>::iterator i=mListTerms.begin(); i!=mListTerms.end(); ++i)
	{
		Base* term=(*i).mTerm;
		f+=term->func(x)*(*i).mCoef;
	}
	f+=mCoef;
	return f;
}


void AnalyticGradient::LinearCombination::dfunc(vectorn const& x, vectorn& dx, m_real scale)
{		
	for(std::list<Term>::iterator i=mListTerms.begin(); i!=mListTerms.end(); ++i)
	{
		Base* term=(*i).mTerm;
		term->dfunc(x,dx, (*i).mCoef*scale);
	}
}

m_real QuadraticFunctionHardCon ::func(vectorn const& x)
{
	double f=QuadraticFunction::func(x);

	int icon=0;
	for(std::list<Con*>::iterator i=mListConTerms.begin(); i!=mListConTerms.end(); ++i)
	{
		f+=(*i)->func(x, mNumVar+icon);
		icon++;
	}
	ASSERT(icon==mNumCon);

	return f;
}

void QuadraticFunctionHardCon ::dfunc(vectorn const& x, vectorn& dx)
{
	QuadraticFunction::dfunc(x, dx);
	int icon=0;
	for(std::list<Con*>::iterator i=mListConTerms.begin(); i!=mListConTerms.end(); ++i)
	{
		(*i)->dfunc(x, dx, mNumVar+icon);
		icon++;
	}
	ASSERT(icon==mNumCon);
}

QuadraticFunctionHardCon::~QuadraticFunctionHardCon()
{
	for(std::list<Con*>::iterator i=mListConTerms.begin(); i!=mListConTerms.end(); ++i)
		delete (*i);
}

void QuadraticFunctionHardCon:: addCon(intvectorn const& index, vectorn const& value)
{
	Con* term=new Con;
	mListConTerms.push_back(term);
	term->index.assign(index);
	term->coef.assign(value);
}

void QuadraticFunctionHardCon ::addCon(int n, m_real coef1, int index1, ...)
{
	Con* term=new Con;
	mListConTerms.push_back(term);

	term->index.setSize(n);
	term->coef.setSize(n+1);

	term->coef[0]=coef1;
	term->index[0]=index1;

	va_list marker;
	va_start( marker, index1);     

	int i;
	for(i=1; i<n; i++)
	{
		term->coef[i]=va_arg(marker, m_real);
		term->index[i]=va_arg(marker, int);
	}

	term->coef[i]=va_arg(marker, m_real);
	va_end(marker);
}


m_real QuadraticFunctionHardCon ::Con::func(vectorn const& x, int numVar_plus_conIndex)
{
	m_real v, lambda;
	lambda=x[numVar_plus_conIndex];

	v=0;		
	for(int i=0; i<index.size(); i++)
		v+=x[index[i]]*coef[i];
	v+=coef[index.size()];

	return lambda*v;
}

void QuadraticFunctionHardCon ::Con::dfunc(vectorn const& x, vectorn& dx, int numVar_plus_conIndex)
{
	m_real lambda=x[numVar_plus_conIndex];

	m_real v=0;
	for(int i=0; i<index.size(); i++)
	{
		dx[index[i]]+=coef[i]*lambda;	
		v+=x[index[i]]*coef[i];
	}
	v+=coef[index.size()];

	dx[numVar_plus_conIndex]+=v;
}


void QuadraticFunction::buildSystem(int dim, matrixn & H, vectorn &R)
{
	H.setSize(dim, dim);
	R.setSize(dim);
	H.setAllValue(0);
	R.setAllValue(0);

	for(std::list<SquaredTerm*>::iterator i=mListSQTerms.begin(); i!=mListSQTerms.end(); ++i)
	{
		intvectorn& index=(*i)->index;
		vectorn& value=(*i)->coef;

		// updateH
		for(int i=0; i<index.size(); i++)
			H[index[i]][index[i]]+=2.0*SQR(value[i]);
		for(int i=0; i<index.size(); i++)
			for(int j=i+1; j<index.size(); j++)
			{
				m_real prev=H[index[j]][index[i]];
				H[index[i]][index[j]]=H[index[j]][index[i]]=prev+2.0*value[i]*value[j];
			}

		// update R
		for(int i=0; i<index.size(); i++)
		{
			R[index[i]]-=2.0*value[index.size()]*value[i];
		}
	}
}

void QuadraticFunctionHardCon ::buildSystem(matrixn & Augmented, vectorn &d)
{
	int nvar=mNumVar;
	int numCon=mNumCon;

	// LAGRANGIAN MULTIPLIER version
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)

	Augmented.setSize(nvar+numCon, nvar+numCon);

	d.setSize(nvar+numCon);

	matrixnView H=Augmented.range(0, nvar, 0, nvar);
	matrixnView A=Augmented.range(nvar, nvar+numCon, 0, nvar);
	matrixnView At=Augmented.range(0, nvar, nvar, nvar+numCon);

	vectornView d0=d.range(0, nvar);
	QuadraticFunction::buildSystem(nvar, H, d0);

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);

	int icon=0;
	for(std::list<Con*>::iterator i=mListConTerms.begin(); i!=mListConTerms.end(); ++i)
	{
		Con* con=*i;

		for(int i=0; i<con->index.size(); i++)
		{
			A[icon][con->index[i]]=con->coef[i];
			d[nvar+icon]=con->coef[con->index.size()]*-1.0;
		}		
		icon++;
	}
	ASSERT(icon==mNumCon);

	At.transpose(A);

	Augmented.range(nvar, nvar+numCon, nvar, nvar+numCon).setAllValue(0.0);
}

	m::c1stitchPreprocess::c1stitchPreprocess(int arow, int brow, m_real strength,bool bConMid)
:mArow(arow), mBrow(brow), h(arow+brow-1), mbConMid(bConMid), mStrength(strength)
{
	// objective function minimize
	// (X, Y, Z, ...)=x
	// minimize sum_0^(n-2) || x[i+1]-x[i] - cc[i]||^2

	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	int cRow=mArow+mBrow-1;

	Msg::verify(mArow>=3, "mArow=%d<3 in c1stitchPreprocess",mArow);
	Msg::verify(mBrow>=3, "mBrow=%d<3 in c1stitchPreprocess",mBrow);

	int nsample=cRow;

	int numCon;
	if(mbConMid)
		numCon=3;
	else
		numCon=2;

	Augmented.setSize(cRow+numCon, cRow+numCon);
	matrixnView H=Augmented.range(0,cRow, 0, cRow);
	matrixnView A=Augmented.range(cRow, cRow+numCon, 0, cRow);
	matrixnView At=Augmented.range(0, cRow, cRow, cRow+numCon);

	d.setSize(cRow+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-cc[i])^2} 의 hessian(=gradient의 계수).
	weight.setSize(nsample-1);

	m_real center=mArow-2;
	m_real distMax=MAX(nsample-3 - center, center);

	for(int i=0; i<nsample-1; i++)
	{
		index.setValues(2, i, i+1);
		coef.setValues(2, -1.0, 1.0);

		m_real dist;
		// dist=0 at i==center
		// dist=1 when i-center==distMax || center-i==distMax
		if(i<mArow-2) dist=m_real(center-i)/distMax;
		else dist=m_real(i-(center))/distMax;

		// dist=0 -> 1.0
		// dist=1 -> strength
		dist*=strength-1.0;
		dist+=1.0;
		weight[i]=sqrt(dist);
		coef*=weight[i];

		h.addSquaredH(index, coef);
	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][cRow-1]=1.0;
	if(mbConMid)
		A[2][mArow-1]=1.0;

	At.transpose(A);

	Augmented.range(cRow, cRow+numCon, cRow, cRow+numCon).setAllValue(0.0);
	m::LUinvert(invAugmented, Augmented);
}	

void m::c1stitchPreprocess::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	Msg::verify(a.rows()==mArow , "error! A row different %d %d", a.rows(), mArow);
	Msg::verify(b.rows()==mBrow, "error! B row different %d %d", b.rows(), mBrow);
	int nsample=a.rows()+b.rows()-1;
	c.setSize(nsample, a.cols());
	cc.setSize(c.rows()-1);

	for(int dim=0; dim<a.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

		for(int i=0; i<a.rows()-1; i++)
			cc[i]=fa(i+1)-fa(i);

		for(int i=0; i<b.rows()-1; i++)
			cc[i+a.rows()-1]=fb(i+1)-fb(i);

		matrixnView ccc=cc.column();
		m::_c0stitch(ccc, a.rows()-1);

		// set hessian residuals.
		h.R.setAllValue(0);
		for(int i=0; i<nsample-1; i++)
		{
			index.setValues(2, i, i+1);
			coef.setValues(3, -1.0, 1.0, -1.0*cc[i]);
			coef*=weight[i];
			h.addSquaredR(index, coef);
		}
		d.range(0, c.rows())=h.R;

		// set constraint
		d[c.rows()]=fa(0);
		d[c.rows()+1]=fb(b.rows()-1);
		if(mbConMid)
			//d[c.rows()+2]=(fa(a.rows()-1)+fb(0))*0.5;
			d[c.rows()+2]=sop::interpolate((m_real)a.rows()/(m_real)(a.rows()+b.rows()), 
					fa(a.rows()-1),fb(0));
		//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

		x.multmat(invAugmented, d);

		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}
}


	m::c1stitchPreprocessOnline::c1stitchPreprocessOnline(int arow, int brow, m_real strength)
:mArow(arow), mBrow(brow), h(arow+brow-1), mStrength(strength)
{
	// objective function minimize
	// (X, Y, Z, ...)=x
	// minimize sum_0^(n-2) || x[i+1]-x[i] - cc[i]||^2

	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	int cRow=mArow+mBrow-1;

	Msg::verify(mArow==2, "mArow=%d!=2 in c1stitchPreprocessOnline",mArow);
	Msg::verify(mBrow>=3, "mBrow=%d<3 in c1stitchPreprocessOnline",mBrow);

	int nsample=cRow;

	int numCon=2;

	Augmented.setSize(cRow+numCon, cRow+numCon);
	matrixnView H=Augmented.range(0,cRow, 0, cRow);
	matrixnView A=Augmented.range(cRow, cRow+numCon, 0, cRow);
	matrixnView At=Augmented.range(0, cRow, cRow, cRow+numCon);

	d.setSize(cRow+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-cc[i])^2} 의 hessian(=gradient의 계수).
	weight.setSize(nsample-1);

	m_real center=mArow-2;
	m_real distMax=MAX(nsample-3 - center, center);

	for(int i=0; i<nsample-1; i++)
	{
		index.setValues(2, i, i+1);
		coef.setValues(2, -1.0, 1.0);

		m_real dist;
		// dist=0 at i==center
		// dist=1 when i-center==distMax || center-i==distMax
		if(i<mArow-2) dist=m_real(center-i)/distMax;
		else dist=m_real(i-(center))/distMax;

		// dist=0 -> 1.0
		// dist=1 -> strength
		dist*=strength-1.0;
		dist+=1.0;
		weight[i]=sqrt(dist);
		coef*=weight[i];

		h.addSquaredH(index, coef);
	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][cRow-1]=1.0;

	At.transpose(A);

	Augmented.range(cRow, cRow+numCon, cRow, cRow+numCon).setAllValue(0.0);
	m::LUinvert(invAugmented, Augmented);
}	

void m::c1stitchPreprocessOnline::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	Msg::verify(a.rows()==mArow , "error! A row different %d %d", a.rows(), mArow);
	Msg::verify(b.rows()==mBrow, "error! B row different %d %d", b.rows(), mBrow);
	int nsample=a.rows()+b.rows()-1;
	c.setSize(nsample, a.cols());
	cc.setSize(c.rows()-1);

	for(int dim=0; dim<a.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

		for(int i=0; i<a.rows()-1; i++)
			cc[i]=fa(i+1)-fa(i);

		for(int i=0; i<b.rows()-1; i++)
			cc[i+a.rows()-1]=fb(i+1)-fb(i);

		m_real diff=cc[1]-cc[0];

		for(int i=0; i<b.rows()-1; i++)
			cc[i+a.rows()-1]+=diff*sop::map(i, -1, b.rows()-2, 0, 1);

		// set hessian residuals.
		h.R.setAllValue(0);
		for(int i=0; i<nsample-1; i++)
		{
			index.setValues(2, i, i+1);
			coef.setValues(3, -1.0, 1.0, -1.0*cc[i]);
			coef*=weight[i];
			h.addSquaredR(index, coef);
		}
		d.range(0, c.rows())=h.R;

		// set constraint
		d[c.rows()]=fa(0);
		d[c.rows()+1]=fb(b.rows()-1);

		//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

		x.multmat(invAugmented, d);

		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}
}

void m::_c0stitch(matrixn& c, int mDiscontinuity, m_real mStrength)
{
	//#define DEBUG_C0STITCH
#ifdef DEBUG_C0STITCH
	static int currS=0;

	c.op0(m0::drawSignals(sz1::format0("%4d.bmp", currS++), 0, 0, true));
#endif
	int discontinuity=mDiscontinuity;
	// I used cos function not to distort the start and end position.
	if(discontinuity<2)
	{
		printf("stitch impossible\n");
		return;
	}
	vectorn sv, av, center;
	sv.sub(c.row(discontinuity-1), c.row(discontinuity-2));
	av.sub(c.row(discontinuity), c.row(discontinuity+1));
	sv/=2.f;
	av/=2.f;    
	//center.op2(v2::mid(), c.row(discontinuity-1), c.row(discontinuity));
	m_real t=sop::clampMap(discontinuity, 0, c.rows()-1, 0.0, 1.0);
	v::interpolate(center, t, c.row(discontinuity-1), c.row(discontinuity));

	vectorn strans, atrans;
	strans.sub(center, c.row(discontinuity-1)+sv);
	atrans.sub(center, c.row(discontinuity)+av);

	for(int i=0; i<discontinuity; i++)
		c.row(i)+= strans*pow(double(i)/ ((double)discontinuity-0.5),mStrength);

	for(int i=discontinuity; i<c.rows(); i++)
		c.row(i)+= atrans*pow(1.f-(double(i-discontinuity)+0.5)/ ((double)(c.rows()-discontinuity)-0.5), mStrength);
#ifdef DEBUG_C0STITCH
	c.op0(m0::drawSignals(sz1::format0("%4d.bmp", currS++), 0, 0, true));
#endif

}

	m::linstitchPreprocessInc::linstitchPreprocessInc(int arow, int brow, int nIter, int normalize,m_real strength)
:mArow(arow), mBrow(brow), h(arow+brow+2),mMaxIter(nIter), mNormalize(normalize)
{
	// objective function minimize
	// (X, Y)=x
	// minimize sum_0^(n-3) || x[i]-2*x[i+1]+x[i+2] -cc[i]||^2
	//  -> (X[i]-2X[i+1]+X[i+2]-ccX[i], Y[i]-2Y[i+1]+Y[i+2]-ccY[i])^2
	//  -> X term과 Y term이 분리됨. 따라서 따로따로 optimize해도 optimal.

	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	int cRow=mArow+mBrow-1;

	ASSERT(mArow>=3);
	ASSERT(mBrow>=3);

	int nvar=mArow+mBrow+2;

	int numCon;
	numCon=6;

	Augmented.setSize(nvar+numCon, nvar+numCon);
	matrixnView H=Augmented.range(0,nvar, 0, nvar);
	matrixnView A=Augmented.range(nvar, nvar+numCon, 0, nvar);
	matrixnView At=Augmented.range(0, nvar, nvar, nvar+numCon);

	d.setSize(nvar+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-cc[i])^2} 의 hessian(=gradient의 계수).
	weight.setSize(mArow+mBrow-2);

	// array index
	// 01234
	// |...|.
	//    .|...|
	//     01234

	// var index
	// 012345   (==A.arrayIndex)
	//    678901  (==B.arrayIndex+a.rows()+2)

	// weight index
	//  0123	(==A.arrayIndex-1)
	//     4567 (==B.arrayIndex+a.rows()-1)

	// c index
	// 01234567
	for(int i=1; i<mArow; i++)
	{
		index.setValues(3, i-1, i, i+1);
		coef.setValues(3, 1.0, -2.0, 1.0);

		m_real dist;
		// dist=0 at i==a.rows()-1
		// dist=1 at i==1 
		dist=m_real(mArow-1-i)/m_real(mArow-2);
		dist*=strength-1.0;
		dist+=1.0;

		// weight index로 변환
		weight[i-1]=sqrt(dist);
		coef*=weight[i-1];

		h.addSquaredH(index, coef);
	}

	for(int i=0; i<mBrow-1; i++)
	{
		index.setValues(3, i-1, i, i+1);
		coef.setValues(3, 1.0, -2.0, 1.0);

		// var index로 변환
		index+=mArow+2;
		m_real dist;
		// dist=0 at i==0
		// dist=1 at i==b.rows()-2
		dist=1.0-m_real(mBrow-2-i)/m_real(mBrow-2);
		dist*=strength-1.0;
		dist+=1.0;

		// weight index로 변환
		weight[i+mArow-1]=sqrt(dist);
		coef*=weight[i+mArow-1];

		h.addSquaredH(index, coef);

	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][1]=1.0;
	A[2][nvar-2]=1.0;
	A[3][nvar-1]=1.0;
	A[4][mArow-1]=1.0;		// A[mArow-1]-B[0]==0
	A[4][mArow+2]=-1.0;

	// (A[mArow]-A[mArow-2])-(B[1]-B[-1])==0
	A[5][mArow]=1.0;
	A[5][mArow-2]=-1.0;		
	A[5][mArow+3]=-1.0;
	A[5][mArow+1]=1.0;

	At.transpose(A);

	Augmented.range(nvar, nvar+numCon, nvar, nvar+numCon).setAllValue(0.0);
	m::LUinvert(invAugmented, Augmented);
}	

void m::linstitchPreprocessInc::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	Msg::verify(a.rows()==mArow , "error! A row different %d %d", a.rows(), mArow);
	Msg::verify(b.rows()==mBrow, "error! B row different %d %d", b.rows(), mBrow);
	int nsample=a.rows()+b.rows()-1;
	c.setSize(nsample, a.cols());
	cc.setSize(a.rows()-1);
	dd.setSize(b.rows()-1);

	int nvar=mArow+mBrow+2;

	aa.setSize(a.rows()+1, a.cols());
	bb.setSize(b.rows()+1, b.cols());

	aa.range(0, a.rows())=a;
	bb.range(1, b.rows()+1)=b;

	// set acceleration at the discontinuity.
	for(int dim=0; dim<a.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

		int i;
		i=a.rows()-2;
		m_real accPrev=fa(i-1)-2.0*fa(i)+fa(i+1);
		i=1;
		m_real accNext=fb(i-1)-2.0*fb(i)+fb(i+1);

		m_real acc=(accPrev+accNext)/2.0;

		i=a.rows()-1;
		aa[i+1][dim]=acc-fa(i-1)+2.0*fa(i);
		i=0;
		bb[0][dim]=acc-fb(i+1)+2.0*fb(i);
	}

	/*
	   gnuPlotQueue q("linstitchIter", a.cols());
	   */

	for(int iter=0; iter<mMaxIter; iter++)
	{
		//q.plotScattered(aa, sz1::format("a%d",iter));
		//q.plotScattered(bb, sz1::format("b%d",iter));
		for(int dim=0; dim<a.cols(); dim++)
		{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

			for(int i=1; i<a.rows(); i++)
				cc[i-1]=aa[i-1][dim]-2.0*aa[i][dim]+aa[i+1][dim];

			for(int i=1; i<b.rows(); i++)
				dd[i-1]=bb[i-1][dim]-2.0*bb[i][dim]+bb[i+1][dim];

			h.R.setAllValue(0);

			// set hessian residuals.
			for(int i=1; i<mArow; i++)
			{
				index.setValues(3, i-1, i, i+1);
				coef.setValues(4, 1.0, -2.0, 1.0, -1.0*cc[i-1]);
				coef*=weight[i-1];

				h.addSquaredR(index, coef);
			}

			for(int i=0; i<mBrow-1; i++)
			{
				index.setValues(3, i-1, i, i+1);
				index+=mArow+2;
				coef.setValues(4, 1.0, -2.0, 1.0, -1.0*dd[i]);
				coef*=weight[i+mArow-1];
				h.addSquaredR(index, coef);
			}

			d.range(0, nvar)=h.R;


			// set constraint
			d[nvar]=fa(0);
			d[nvar+1]=fa(1);
			d[nvar+2]=fb(b.rows()-2);
			d[nvar+3]=fb(b.rows()-1);


			// aa[mArow-1]-bb[1]==0	when iter==mMaxIter-1
			d[nvar+4]=(aa[mArow-1][dim]-bb[1][dim])
				*sop::clampMap(iter, -1, mMaxIter-1, 1.0, 0.0);

			// (aa[mArow]-aa[mArow-2])-(bb[2]-bb[0])==0
			d[nvar+5]=((aa[mArow][dim]-aa[mArow-2][dim])-(bb[2][dim]-bb[0][dim]))
				*sop::clampMap(iter, -1, mMaxIter-1, 1.0, 0.0);


			x.multmat(invAugmented, d);

			// save results.
			for(int i=0; i<mArow+1; i++)
				aa[i][dim]=x[i];
			for(int i=0; i<mBrow+1; i++)
				bb[i][dim]=x[i+mArow+1];
		}

		if(mNormalize!=0)
		{
			ASSERT(aa.cols()%mNormalize==0);
			for(int j=0; j<aa.cols(); j+=mNormalize)
			{
				for(int i=0; i<aa.rows(); i++)
					aa.row(i).range(j, j+mNormalize).normalize();
				for(int i=0; i<bb.rows(); i++)
					bb.row(i).range(j, j+mNormalize).normalize();
			}
		}
	}

	// save final results.
	for(int dim=0; dim<a.cols(); dim++)
	{
		for(int i=0; i<mArow; i++)
			f(i)=aa[i][dim];
		for(int i=1; i<mBrow; i++)
			f(i+mArow-1)=bb[i+1][dim];
	}

	//q.plotScattered(c, "c");
}

	m::linstitchPreprocess::linstitchPreprocess(int arow, int brow, m_real strength, bool bConMid)
:mArow(arow), mBrow(brow), h(arow+brow-1), mbConMid(bConMid)
{
	// objective function minimize
	// (X, Y)=x
	// minimize sum_0^(n-3) || x[i]-2*x[i+1]+x[i+2] -cc[i]||^2
	//  -> (X[i]-2X[i+1]+X[i+2]-ccX[i], Y[i]-2Y[i+1]+Y[i+2]-ccY[i])^2
	//  -> X term과 Y term이 분리됨. 따라서 따로따로 optimize해도 optimal.

	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	int cRow=mArow+mBrow-1;

	ASSERT(mArow>=3);
	ASSERT(mBrow>=3);

	int nsample=cRow;

	int numCon;
	if(mbConMid)
		numCon=5;
	else
		numCon=4;

	Augmented.setSize(cRow+numCon, cRow+numCon);
	matrixnView H=Augmented.range(0,cRow, 0, cRow);
	matrixnView A=Augmented.range(cRow, cRow+numCon, 0, cRow);
	matrixnView At=Augmented.range(0, cRow, cRow, cRow+numCon);

	d.setSize(cRow+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-cc[i])^2} 의 hessian(=gradient의 계수).
	weight.setSize(nsample-2);
	for(int i=0; i<nsample-2; i++)
	{
		index.setValues(3, i, i+1, i+2);
		coef.setValues(3, 1.0, -2.0, 1.0);

		m_real dist;
		// dist=0 at i==a.rows()-2
		// dist=1 at i==0 && nsample-3
		if(i<mArow-2) dist=m_real(mArow-2-i)/m_real(mArow-2);
		else dist=m_real(i-(mArow-2))/m_real(nsample-3-(mArow-2));

		//		dist+=1.0/strength;
		//		weight[i]=dist;
		//		coef*=weight[i];

		// dist=0 -> 1.0
		// dist=1 -> strength

		dist*=strength-1.0;
		dist+=1.0;
		weight[i]=sqrt(dist);
		//weight[i]=dist;
		coef*=weight[i];

		h.addSquaredH(index, coef);
	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][1]=1.0;
	A[2][cRow-2]=1.0;
	A[3][cRow-1]=1.0;
	if(mbConMid)
		A[4][mArow-1]=1.0;

	At.transpose(A);

	Augmented.range(cRow, cRow+numCon, cRow, cRow+numCon).setAllValue(0.0);
	m::LUinvert(invAugmented, Augmented);
}	


void m::linstitchPreprocess::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	Msg::verify(a.rows()==mArow , "error! A row different %d %d", a.rows(), mArow);
	Msg::verify(b.rows()==mBrow, "error! B row different %d %d", b.rows(), mBrow);
	int nsample=a.rows()+b.rows()-1;
	c.setSize(nsample, a.cols());
	cc.setSize(c.rows()-2);



	for(int dim=0; dim<a.cols(); dim++)
	{

		for(int i=0; i<a.rows()-2; i++)
			cc[i]=fa(i)-2.0*fa(i+1)+fa(i+2);

		for(int i=0; i<b.rows()-2; i++)
			cc[i+a.rows()-1]=fb(i)-2.0*fb(i+1)+fb(i+2);

		// set acceleration at the discontinuity.
		cc[a.rows()-2]=0.5*(cc[a.rows()-3]+cc[a.rows()-1]);

		// set hessian residuals.
		h.R.setAllValue(0);
		for(int i=0; i<nsample-2; i++)
		{
			index.setValues(3, i, i+1, i+2);
			coef.setValues(4, 1.0, -2.0, 1.0, -1.0*cc[i]);
			coef*=weight[i];

			h.addSquaredR(index, coef);
		}
		d.range(0, c.rows())=h.R;


		// set constraint
		d[c.rows()]=fa(0);
		d[c.rows()+1]=fa(1);
		d[c.rows()+2]=fb(b.rows()-2);
		d[c.rows()+3]=fb(b.rows()-1);
		if(mbConMid)
			d[c.rows()+4]=(fa(a.rows()-1)+fb(0))*0.5;

		//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

		x.multmat(invAugmented, d);

		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}
}

void m::c0concat::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	// I used cos function not to distort the start and end position.
	if(a.rows()<2)
	{
		printf("stitch impossible\n");
		return;
	}
	vectorn center;
	center.add(a.row(a.rows()-1), b.row(0));
	center*=0.5;

	c.setSize(a.rows()+b.rows()-1, a.cols());
	int centerm=a.rows();
	for(int i=0; i<centerm-1; i++)
		c.row(i)=a.row(i);

	c.row(centerm-1)=center;

	for(int i=centerm; i<c.rows(); i++)
		c.row(i)=b.row(i-centerm+1);
}

// objective function minimize
// (X, Y)=x
// minimize sum_0^(n-3) || x[i]-2*x[i+1]+x[i+2] -cc[i]||^2
//  -> (X[i]-2X[i+1]+X[i+2]-ccX[i], Y[i]-2Y[i+1]+Y[i+2]-ccY[i])^2
//  -> X term과 Y term이 분리됨. 따라서 따로따로 optimize해도 optimal.

void m::linstitch::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	ASSERT(a.rows()>=3);
	ASSERT(b.rows()>=3);
	ASSERT(a.cols()==b.cols());

	c.setSize(a.rows()+b.rows()-1, a.cols());
	int nsample=c.rows();

	int numCon=5;
	matrixn Augmented(c.rows()+numCon, c.rows()+numCon);
	matrixnView H=Augmented.range(0,c.rows(), 0, c.rows());
	matrixnView A=Augmented.range(c.rows(), c.rows()+numCon, 0, c.rows());
	matrixnView At=Augmented.range(0, c.rows(), c.rows(), c.rows()+numCon);

	vectorn x, d(c.rows()+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-cc[i])^2} 의 hessian(=gradient의 계수).
	HessianQuadratic h(c.rows());
	intvectorn index;
	vectorn coef;
	vectorn weight(nsample-2);
	for(int i=0; i<nsample-2; i++)
	{
		index.setValues(3, i, i+1, i+2);
		coef.setValues(3, 1.0, -2.0, 1.0);

		m_real dist;
		// dist=0 at i==a.rows()-2
		// dist=1 at i==0 && nsample-3
		if(i<a.rows()-2) dist=m_real(a.rows()-2-i)/m_real(a.rows()-2);
		else dist=m_real(i-(a.rows()-2))/m_real(nsample-3-(a.rows()-2));

		dist*=mStrength-1.0;
		dist+=1.0;
		weight[i]=sqrt(dist);
		coef*=weight[i];

		h.addSquaredH(index, coef);
	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][1]=1.0;
	A[2][c.rows()-2]=1.0;
	A[3][c.rows()-1]=1.0;
	A[4][a.rows()-1]=1.0;

	At.transpose(A);

	Augmented.range(c.rows(), c.rows()+numCon, c.rows(), c.rows()+numCon).setAllValue(0.0);
#ifdef USE_LUDCMP
	DP p;
	matrixn LU(Augmented);
	Vec_INT indx(Augmented.rows());
	NR::ludcmp(LU,indx,p);
#else
	Eigen::MatrixXd _A=eigenView(Augmented);
	Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(_A);
	//matrixn invAugmented;
	//invAugmented.inverse(Augmented);
#endif
	vectorn cc(c.rows()-2);

	for(int dim=0; dim<a.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

		for(int i=0; i<a.rows()-2; i++)
			cc[i]=fa(i)-2.0*fa(i+1)+fa(i+2);

		for(int i=0; i<b.rows()-2; i++)
			cc[i+a.rows()-1]=fb(i)-2.0*fb(i+1)+fb(i+2);

		// set acceleration at the discontinuity.
		cc[a.rows()-2]=0.5*(cc[a.rows()-3]+cc[a.rows()-1]);

		// set hessian residuals.
		h.R.setAllValue(0);
		for(int i=0; i<nsample-2; i++)
		{
			index.setValues(3, i, i+1, i+2);
			coef.setValues(4, 1.0, -2.0, 1.0, -1.0*cc[i]);
			coef*=weight[i];

			h.addSquaredR(index, coef);
		}
		d.range(0, c.rows())=h.R;


		// set constraint
		d[c.rows()]=fa(0);
		d[c.rows()+1]=fa(1);
		d[c.rows()+2]=fb(b.rows()-2);
		d[c.rows()+3]=fb(b.rows()-1);
		d[c.rows()+4]=(fa(a.rows()-1)+fb(0))*0.5;
		//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
		x=d;
		NR::lubksb(LU,indx,x);
#else
		Eigen::VectorXd _x=dec.solve(eigenView(d));
		x=vecView(_x);
#endif
		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}
}

void m::c0stitch::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	// I used cos function not to distort the start and end position.
	if(a.rows()<2)
	{
		printf("stitch impossible\n");
		return;
	}
	vectorn center;
	center.add(a.row(a.rows()-1), b.row(0));
	center*=0.5;

	vectorn strans, atrans;
	strans.sub(center, a.row(a.rows()-1));
	atrans.sub(center, b.row(0));

	c.setSize(a.rows()+b.rows()-1, a.cols());
	int centerm=a.rows();
	for(int i=0; i<centerm; i++)	// i=0->0 , i=centerm-1 -> 1
		c.row(i)=a.row(i)+strans*pow(double(i)/ ((double)centerm-1.0),1.5);

	for(int i=centerm; i<c.rows(); i++) // i=centerm-1->0 , i=c.rows()-1 -> 1
		c.row(i)=b.row(i-centerm+1)+atrans*pow(1.f-double(i-centerm+1)/double(c.rows()-centerm), 1.5);

}

void m::c0stitchOnline::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	// I used cos function not to distort the start and end position.
	Msg::verify(a.rows()==2, "??");

	vectorn disc, atrans;
	disc.assign(a.row(a.rows()-1));

	atrans.sub(disc, b.row(0));

	c.setSize(a.rows()+b.rows()-1, a.cols());
	int centerm=a.rows();
	for(int i=0; i<centerm; i++)	// i=0->0 , i=centerm-1 -> 1
		c.row(i)=a.row(i);

	for(int i=centerm; i<c.rows(); i++) // i=centerm-1->0 , i=c.rows()-1 -> 1
		c.row(i)=b.row(i-centerm+1)+atrans*pow(1.f-double(i-centerm+1)/double(c.rows()-centerm), 1.5);

}

void m::c0stitchPreserve2::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	// I used cos function not to distort the start and end position.
	if(a.rows()<3 || b.rows()<3)
	{
		printf("stitch impossible\n");
		return;
	}

	c.setSize(a.rows()+b.rows()-1, a.cols());

	c.row(0)=a.row(0);
	c.row(c.rows()-1)=b.row(b.rows()-1);

	m::c0stitch t;
	t.calc(c.range(1, c.rows()-1).lval(), a.range(1, a.rows()), b.range(0, b.rows()-1));
}

void m::c1stitch::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	m::c1stitchPreprocess p(a.rows(), b.rows(), mStrength);
	p.calc(c, a, b);
}


void m::stitchQuaterNN ::calc(matrixn& c, const matrixn& a, const matrixn&b )const	
{
	c.setSize(a.rows()+b.rows()-1, a.cols());
	ASSERT(a.cols()==b.cols());
	ASSERT(a.cols()%4==0);

	if(mPreserveAmount==0)
	{
		for(int i=0; i< a.cols(); i+=4)
			(quatViewCol(c, i).*mFunc)(quatViewCol(a,i), quatViewCol(b, i));
	}
	else
	{
		for(int i=0; i<mPreserveAmount; i++)
		{
			c.row(i)=a.row(i);
			c.row(c.rows()-1-i)=b.row(b.rows()-1-i);
		}

		m::stitchQuaterNN t(mFunc);
		t.calc(c.range(mPreserveAmount, c.rows()-mPreserveAmount).lval(), 
				a.range(mPreserveAmount, a.rows()),
				b.range(0, b.rows()-mPreserveAmount));
	}
}

void m::linstitchForward::calc(matrixn& ddd, const matrixn& a, const matrixn& b) const
{
	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	ASSERT(a.rows()>=3);
	ASSERT(b.rows()>=3);
	ASSERT(a.cols()==b.cols());

	ddd.setSize(a.rows()+b.rows()-1, a.cols());
	ddd.range(a.rows()-1, ddd.rows())=b;

	// a의 마지막 두프레임만 이용.
	matrixnView c(ddd.range(0, a.rows()+1));

	ASSERT(c.rows()==2+a.rows()-1);

	int nsample=c.rows();

	int numCon=4;
	matrixn Augmented(c.rows()+numCon, c.rows()+numCon);
	matrixnView H=Augmented.range(0,c.rows(), 0, c.rows());
	matrixnView A=Augmented.range(c.rows(), c.rows()+numCon, 0, c.rows());
	matrixnView At=Augmented.range(0, c.rows(), c.rows(), c.rows()+numCon);

	vectorn x, d(c.rows()+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2])^2} 의 hessian(=gradient의 계수).
	H.setAllValue(0.0);
	for(int i=0; i<nsample; i++)
	{
		if(0<=i && i<nsample-2)
		{
			H[i][i]+=2.0;
			H[i][i+1]-=4.0;
			H[i][i+2]+=2.0;
		}
		if(1<=i && i<nsample-1)
		{
			H[i][i-1]-=4.0;
			H[i][i]+=8.0;
			H[i][i+1]-=4.0;
		}
		if(2<=i && i<nsample)
		{
			H[i][i-2]+=2.0;
			H[i][i-1]-=4.0;
			H[i][i]+=2.0;
		}
	}

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][1]=1.0;
	A[2][c.rows()-2]=1.0;
	A[3][c.rows()-1]=1.0;

	At.transpose(A);

	Augmented.range(c.rows(), c.rows()+numCon, c.rows(), c.rows()+numCon).setAllValue(0.0);
#ifdef USE_LUDCMP
	DP p;
	matrixn LU(Augmented);
	Vec_INT indx(Augmented.rows());
	NR::ludcmp(LU,indx,p);
#else
	Eigen::MatrixXd _A=eigenView(Augmented);
	Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(_A);
#endif
	vectorn cc(c.rows()-2);

	for(int dim=0; dim<a.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

		for(int i=0; i<a.rows()-2; i++)
			cc[i]=fa(i)-2.0*fa(i+1)+fa(i+2);

		// set acceleration at the discontinuity.
		cc[a.rows()-2]=cc[a.rows()-3];

		// set hessian residuals.
		for(int i=0; i<nsample; i++)
		{
			d[i]=0;
			if(0<=i && i<nsample-2)
				d[i]+=2.0*cc[i];
			if(1<=i && i<nsample-1)
				d[i]-=4.0*cc[i-1];
			if(2<=i && i<nsample)
				d[i]+=2.0*cc[i-2];
		}

		// set constraint
		d[c.rows()]=fa(0);
		d[c.rows()+1]=fa(1);
		d[c.rows()+2]=fb(0);
		d[c.rows()+3]=fb(1);
		//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
		x=d;
		NR::lubksb(LU,indx,x);
#else
		Eigen::VectorXd _x=dec.solve(eigenView(d));
		x=vecView(_x);
#endif
		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}
}
void m::linstitchOnline::calc(matrixn& ddd, const matrixn& a, const matrixn& b) const
{
	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	ASSERT(a.rows()>=3);
	ASSERT(b.rows()>=3);
	ASSERT(a.cols()==b.cols());


	ddd.setSize(a.rows()+b.rows()-1, a.cols());
	ddd.range(0, a.rows())=a;

	// a의 마지막 두프레임만 이용.
	matrixnView c(ddd.range(a.rows()-2, ddd.rows()));

	ASSERT(c.rows()==2+b.rows()-1);
	int nsample=c.rows();

	int numCon=4;
	matrixn Augmented(c.rows()+numCon, c.rows()+numCon);
	matrixnView H=Augmented.range(0,c.rows(), 0, c.rows());
	matrixnView A=Augmented.range(c.rows(), c.rows()+numCon, 0, c.rows());
	matrixnView At=Augmented.range(0, c.rows(), c.rows(), c.rows()+numCon);

	vectorn x, d(c.rows()+numCon);

	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-cc[i])^2} 의 hessian(=gradient의 계수).
	HessianQuadratic h(c.rows());
	intvectorn index;
	vectorn coef;
	vectorn weight(nsample-2);
	for(int i=0; i<nsample-2; i++)
	{
		index.setValues(3, i, i+1, i+2);
		coef.setValues(3, 1.0, -2.0, 1.0);

		m_real dist;
		// dist=0 at i==0
		// dist=1 at i=nsample-3
		dist=m_real(i)/m_real(nsample-3);
		dist*=mStrength-1.0;
		dist+=1.0;
		weight[i]=sqrt(dist);
		//weight[i]=dist;
		coef*=weight[i];

		h.addSquaredH(index, coef);
	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][1]=1.0;
	A[2][c.rows()-2]=1.0;
	A[3][c.rows()-1]=1.0;

	At.transpose(A);

	Augmented.range(c.rows(), c.rows()+numCon, c.rows(), c.rows()+numCon).setAllValue(0.0);
#ifdef USE_LUDCMP
	DP p;
	matrixn LU(Augmented);
	Vec_INT indx(Augmented.rows());
	NR::ludcmp(LU,indx,p);
#else
	Eigen::MatrixXd _A=eigenView(Augmented);
	Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(_A);
#endif
	vectorn cc(c.rows()-2);

	for(int dim=0; dim<a.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

		for(int i=0; i<b.rows()-2; i++)
			cc[i+2-1]=fb(i)-2.0*fb(i+1)+fb(i+2);

		// set acceleration at the discontinuity.
		cc[0]=cc[1];

		// set hessian residuals.
		h.R.setAllValue(0);
		for(int i=0; i<nsample-2; i++)
		{
			index.setValues(3, i, i+1, i+2);
			coef.setValues(4, 1.0, -2.0, 1.0, -1.0*cc[i]);
			coef*=weight[i];

			h.addSquaredR(index, coef);
		}
		d.range(0, c.rows())=h.R;

		// set constraint
		d[c.rows()]=fa(a.rows()-2);
		d[c.rows()+1]=fa(a.rows()-1);
		d[c.rows()+2]=fb(b.rows()-2);
		d[c.rows()+3]=fb(b.rows()-1);

		//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
		x=d;
		NR::lubksb(LU,indx,x);
#else
		Eigen::VectorXd _x=dec.solve(eigenView(d));
		x=vecView(_x);
#endif
		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}		
}

void m::linstitch2::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	ASSERT(a.rows()>=3);
	ASSERT(b.rows()>=3);
	ASSERT(a.cols()==b.cols());

	c.setSize(a.rows()+b.rows()-1, a.cols());
	int nsample=c.rows();

	int numCon=4;
	matrixn Augmented(c.rows()+numCon, c.rows()+numCon);
	matrixnView H=Augmented.range(0,c.rows(), 0, c.rows());
	matrixnView A=Augmented.range(c.rows(), c.rows()+numCon, 0, c.rows());
	matrixnView At=Augmented.range(0, c.rows(), c.rows(), c.rows()+numCon);

	vectorn x, d(c.rows()+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2])^2} 의 hessian(=gradient의 계수).
	H.setAllValue(0.0);
	for(int i=0; i<nsample; i++)
	{
		if(0<=i && i<nsample-2)
		{
			H[i][i]+=2.0;
			H[i][i+1]-=4.0;
			H[i][i+2]+=2.0;
		}
		if(1<=i && i<nsample-1)
		{
			H[i][i-1]-=4.0;
			H[i][i]+=8.0;
			H[i][i+1]-=4.0;
		}
		if(2<=i && i<nsample)
		{
			H[i][i-2]+=2.0;
			H[i][i-1]-=4.0;
			H[i][i]+=2.0;
		}
	}

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][1]=1.0;
	A[2][c.rows()-2]=1.0;
	A[3][c.rows()-1]=1.0;

	At.transpose(A);

	Augmented.range(c.rows(), c.rows()+numCon, c.rows(), c.rows()+numCon).setAllValue(0.0);
#ifdef USE_LUDCMP
	DP p;
	matrixn LU(Augmented);
	Vec_INT indx(Augmented.rows());
	NR::ludcmp(LU,indx,p);
#else
	matrixn invAugmented;
	m::LUinvert(invAugmented, Augmented);
#endif
	vectorn cc(c.rows()-2);

	for(int dim=0; dim<a.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

		for(int i=0; i<a.rows()-2; i++)
			cc[i]=fa(i)-2.0*fa(i+1)+fa(i+2);

		for(int i=0; i<b.rows()-2; i++)
			cc[i+a.rows()-2]=fb(i)-2.0*fb(i+1)+fb(i+2);

		// set hessian residuals.
		for(int i=0; i<nsample; i++)
		{
			d[i]=0;
			if(0<=i && i<nsample-2)
				d[i]+=2.0*cc[i];
			if(1<=i && i<nsample-1)
				d[i]-=4.0*cc[i-1];
			if(2<=i && i<nsample)
				d[i]+=2.0*cc[i-2];
		}

		// set constraint
		d[c.rows()]=fa(0);
		d[c.rows()+1]=fa(1);
		d[c.rows()+2]=fb(b.rows()-2);
		d[c.rows()+3]=fb(b.rows()-1);
		//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
		x=d;
		NR::lubksb(LU,indx,x);
#else
		x.multmat(invAugmented, d);
#endif
		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}
}


// objective function minimize
// (X, Y)=x
// minimize sum_0^(n-3) || x[i]-2*x[i+1]+x[i+2] -cc[i]||^2
//  -> (X[i]-2X[i+1]+X[i+2]-ccX[i], Y[i]-2Y[i+1]+Y[i+2]-ccY[i])^2
// linstitch와 동일한 결과를 훨씬 느린속도로 얻는다.-.-
// 하지만, 이 formulation은 quaternion stitching같이 디멘전간 dependency가 존재하는 경우로 확장이 가능하다.
void m::linstitchMulti::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	ASSERT(a.rows()>=3);
	ASSERT(b.rows()>=3);
	ASSERT(a.cols()==b.cols());

	c.setSize(a.rows()+b.rows()-1, a.cols());
	int nsample=c.rows();
	int ndim=c.cols();
	int nvar=nsample*ndim;

	int numSCon;
	if(mbConMid)
		numSCon=5;
	else
		numSCon=4;
	int numCon=numSCon*ndim;
	matrixn Augmented(nvar+numCon, nvar+numCon);
	matrixnView H=Augmented.range(0,nvar, 0, nvar);
	matrixnView A=Augmented.range(nvar, nvar+numCon, 0, nvar);
	matrixnView At=Augmented.range(0, nvar, nvar, nvar+numCon);

	vectorn x, d(nvar+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-cc[i])^2} 의 hessian(=gradient의 계수).
	HessianQuadratic h(nvar);
	intvectorn index;
	vectorn coef;
	vectorn weight(nsample-2);
	for(int i=0; i<nsample-2; i++)
	{
		for(int j=0; j<ndim; j++)
		{
			int offset=nsample*j;
			index.setValues(3, i+offset, i+1+offset, i+2+offset);
			coef.setValues(3, 1.0, -2.0, 1.0);

			if(j==0)
			{
				m_real dist;
				// dist=0 at i==a.rows()-2
				// dist=1 at i==0 && nsample-3
				if(i<a.rows()-2) dist=m_real(a.rows()-2-i)/m_real(a.rows()-2);
				else dist=m_real(i-(a.rows()-2))/m_real(nsample-3-(a.rows()-2));

				dist*=mStrength-1.0;
				dist+=1.0;
				weight[i]=sqrt(dist);
			}
			coef*=weight[i];

			h.addSquaredH(index, coef);
		}
	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	for(int j=0; j<ndim; j++)
	{
		int offset=nsample*j;
		int offsetCon=numSCon*j;
		A[0+offsetCon][0+offset]=1.0;
		A[1+offsetCon][1+offset]=1.0;
		A[2+offsetCon][nsample-2+offset]=1.0;
		A[3+offsetCon][nsample-1+offset]=1.0;

		if(mbConMid)
			A[4+offsetCon][a.rows()-1+offset]=1.0;
	}

	//printf("%s\n", H.output("%.0f").ptr());
	At.transpose(A);

	Augmented.range(nvar, nvar+numCon, nvar, nvar+numCon).setAllValue(0.0);
#ifdef USE_NR

	//printf("%s\n", Augmented.output("%.0f").ptr());
#define USE_LUDCMP
#ifdef USE_LUDCMP
	DP p;
	matrixn LU(Augmented);
	Vec_INT indx(Augmented.rows());
	NR::ludcmp(LU,indx,p);
#else
	matrixn invAugmented;
	m::LUinvert(invAugmented, Augmented);
#endif
	vectorn cc(nsample-2);

	h.R.setAllValue(0);
	for(int dim=0; dim<a.cols(); dim++)
	{
		int offset=nsample*dim;
#define fa(xxx)	a(xxx,dim)
#define fb(xxx)	b(xxx,dim)

		for(int i=0; i<a.rows()-2; i++)
			cc[i]=fa(i)-2.0*fa(i+1)+fa(i+2);

		for(int i=0; i<b.rows()-2; i++)
			cc[i+a.rows()-1]=fb(i)-2.0*fb(i+1)+fb(i+2);

		// set acceleration at the discontinuity.
		cc[a.rows()-2]=0.5*(cc[a.rows()-3]+cc[a.rows()-1]);

		// set hessian residuals.

		for(int i=0; i<nsample-2; i++)
		{
			index.setValues(3, i+offset, i+1+offset, i+2+offset);
			coef.setValues(4, 1.0, -2.0, 1.0, -1.0*cc[i]);
			coef*=weight[i];
			h.addSquaredR(index, coef);
		}

		int offsetCon=numSCon*dim;
		// set constraint
		d[nvar+offsetCon]=fa(0);
		d[nvar+1+offsetCon]=fa(1);
		d[nvar+2+offsetCon]=fb(b.rows()-2);
		d[nvar+3+offsetCon]=fb(b.rows()-1);
		if(mbConMid)
			d[nvar+4+offsetCon]=(fa(a.rows()-1)+fb(0))*0.5;
		//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);
	}

	d.range(0, nvar)=h.R;


#ifdef USE_LUDCMP
	x=d;
	NR::lubksb(LU,indx,x);
#else
	x.multmat(invAugmented, d);
#endif
	// save results.
	for(int dim=0; dim<a.cols(); dim++)
	{
		int offset=nsample*dim;
#define f(xxx) c(xxx,dim)
		for(int i=0; i<nsample; i++)
			f(i)=x[i+offset];
	}
#else
	Msg::error("ludcmp");
#endif
}

void quaterNN_linstitch(m::stitchOp const& op, matrixn& c, matrixn & a, matrixn & b)
{
	int njoint=a.cols()/4;
	c.setSize(a.rows()+b.rows()-1, a.cols());
	for(int j=0; j<njoint; j++)
	{
		quaterNView aa(a.range(0, a.rows(), j*4, (j+1)*4).toQuaterN());
		quaterNView bb(b.range(0, b.rows(), j*4, (j+1)*4).toQuaterN());
		aa.align();
		bb.row(0).align(aa.row(aa.rows()-1));
		bb.align();
	}

	op.calc(c,a,b);

	for(int j=0; j<njoint; j++)
	{
		quaterNView cc(c.range(0, c.rows(), j*4, (j+1)*4).toQuaterN());
		for(int i=0; i<cc.rows(); i++)
			cc.row(i).normalize();
	}
}
