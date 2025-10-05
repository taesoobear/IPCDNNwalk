#ifndef _VECTOR_N_H_
#define _VECTOR_N_H_
#pragma once
class matrixn;
class boolN;
#include "vector3.h"
#include "quater.h"
#include "transf.h"
#include "../utility/TypeString.h"
#include "../utility/util.h"

#include <typeinfo>
class matrixnView;
class matrixn;
class intvectorn;
class intvectornView;
class vectorn;
class vectornView;
class Metric;
#include "template_math.h"

#if _MSC_VER > 1000
//#pragma message("Compiling vector_n.h - this should happen just once per project.\n")
#endif

class intvectorn : public _tvectorn<int>
{
protected:
	intvectorn(int* ptrr, int size, int stride):_tvectorn<int>(ptrr,size,stride){}	// reference
public:
	intvectorn():_tvectorn<int>(){}

	explicit intvectorn(int n):_tvectorn<int>()				{setSize(n);}
	explicit intvectorn( int, int x, ...);	// n dimensional vector	(ex) : vectorn(3, 1, 2, 3);
	// copy constructor : 항상 카피한다.
	intvectorn(const _tvectorn<int>& other):_tvectorn<int>()		{ assign(other);}
	intvectorn(const intvectorn& other):_tvectorn<int>()			{ assign(other);}
	intvectorn(const intvectornView& other);

	// 값을 copy한다.
	intvectorn& operator=(const _tvectorn<int>& other)		{ _tvectorn<int>::assign(other);return *this;}
	intvectorn& operator=(const intvectorn& other)			{ assign(other);return *this;}
	intvectorn& operator=(const intvectornView& other);

	friend bool operator==(intvectorn const& a, intvectorn const& b)
	{
		if(a.size()!=b.size()) return false;
		for(int i=0; i<a.size(); i++)
			if(a[i]!=b[i]) return false;
		return true;
	}

	// L-value로 사용될수 있는, reference array를 만들어 return 한다.
	// ex) v.range(0,2).setValues(2, 1.0, 2.0);
	intvectornView	range(int start, int end, int step=1);
	const intvectornView	range(int start, int end, int step=1) const;

	intvectorn& assignBits(const boolN& bits);

	~intvectorn(){}

	int count(int (*s2_func)(int ,int ), int value, int start=0, int end=INT_MAX);
	intvectorn&  colon(int start, int end, int stepSize=1);
	intvectorn&  sortedOrder(vectorn const & input);
	intvectorn&  makeSamplingIndex(int nLen, int numSample);
	// 첫프레임과 마지막 프레임은 반드시 포함하고 나머지는 그 사이에서 uniform sampling
	intvectorn&  makeSamplingIndex2(int nLen, int numSample);

	intvectorn& findIndex(intvectorn const& source, int value);
	intvectorn& findIndex(boolN const& source, bool value, int start=0, int end=INT_MAX);
	// return -1 if not found.
	int findFirstIndex(int value) const;

	void parseString(int n_reserve, const std::string &source);
	intvectorn&  setAt( intvectorn const& columnIndex, _tvectorn<int> const& value);
	intvectorn&  setAt( intvectorn const& columnIndex, int value);

	void decode(const TString& input);

	int maximum() const;
	int minimum() const;
	int sum() const;

	vectorn toVectorn();

	TString output(const char* left="[", const char* typeString="%d", const char* seperator=",", const char* right="]") const;

	// all following functions are deprecated.
	inline void push_back(int x)						{ pushBack(x);}

	// use the corresponding functions in class intIntervals instead.
	void runLengthEncode(const intvectorn& source);
	void runLengthEncode(const boolN& source, int start=0, int end=INT_MAX);
	void runLengthDecode(boolN& out, int size);
	void runLengthEncodeCut(const boolN& cutState, int start=0, int end=INT_MAX);

	friend intvectorn operator*( intvectorn const& a, int b);
	friend intvectorn operator/( intvectorn const& a, int b);
	friend intvectorn operator+( intvectorn const& a, int b);
	friend intvectorn operator-( intvectorn const& a, int b);
	friend std::ostream& operator<< ( std::ostream& os, const intvectorn& u );
};

// reference로 받아온다.
class intvectornView :public intvectorn
{
public:
	// L-value로 사용될수 있는, reference array로 만든다.
	intvectornView (const int* ptrr, int size, int stride=1);
	// copy constructor : get reference.
	intvectornView(const _tvectorn<int>& other)				{ assignRef(other);}
	intvectornView(const intvectorn& other);//				{ assignRef(other);}
	intvectornView(const intvectornView& other)				{ assignRef(other);}
	~intvectornView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	intvectorn& operator=(const _tvectorn<int>& other)		{ _tvectorn<int>::assign(other);return *this;}
	intvectorn& operator=(const intvectorn& other);//		{ assign(other);return *this;}
	intvectorn& operator=(const intvectornView& other)		{ assign(other);return *this;}

	/* doesn't work in some gnu compilers.
    inline operator vectorn & () const { return (vectorn&)(*this);}
	*/
	// for directly passing to a parameter. 
	intvectorn& lval() const { return (intvectorn&)(*this);}  //!< ex> doSomeModification(a.range(1,3).lval());
};

namespace v0
{
	// v0 namespace에는 doSomething(c); 형태의 class나 function이 정의된다. (operatorTemplate)
	struct abstractClass 	// 상속할 필요 없음.
	{
		void operator()(vectorn& c){}
	};

	// deprecated
	struct _op
	{
		virtual void calc(vectorn& c) const {ASSERT(0);}
	};
}

namespace v1
{
	// v1 namespace에는 doSomething(c,a); 형태의 class나 function이 정의된다. (operatorTemplate)
	struct abstractClass 	// 상속할 필요 없음.
	{
		void operator()(vectorn& c, const vectorn& a){}
	};

	// deprecated
	struct _op
	{
		virtual void calc(vectorn& c, const vectorn& a) const {ASSERT(0);}
	};
}

namespace v2
{
	// v2 namespace에는 doSomething(c,a,b); 형태의 class나 function이 정의된다. (operatorTemplate)
	struct abstractClass	// 상속할 필요 없음.
	{
		void operator()(vectorn& c, const vectorn& a, const vectorn& b){}
	};
	// deprecated
	struct _op
	{
		virtual void calc(vectorn& c, const vectorn& a, const vectorn& b) const {ASSERT(0);}
	};
}

// deprecated
namespace sv2
{
	struct _op
	{
		virtual m_real calc(const vectorn& a, const vectorn& b) const {ASSERT(0);return 0.0;}
	};
}


class vectorn : public _tvectorn<m_real>
{
protected:
	vectorn(m_real* ptrr, int size, int stride):_tvectorn<m_real>(ptrr,size,stride){}
public:
	vectorn();
	vectorn(const vector3& other);
	vectorn(const quater& other);

	// 값을 카피해서 받아온다.
	vectorn(const _tvectorn<m_real>& other);
	vectorn(const vectorn& other);
	vectorn(const vectornView& other);

	explicit vectorn( int x):_tvectorn<m_real>() { setSize(x);}

	// n dimensional vector	(ex) : vectorn(3, 1.0, 2.0, 3.0);
	explicit vectorn( int n, m_real x);
	explicit vectorn( int n, m_real x, m_real y);
	explicit vectorn( int n, m_real x, m_real y, m_real z);
	explicit vectorn( int n, m_real x, m_real y, m_real z, m_real w, ...);	// n dimensional vector	(ex) : vectorn(3, 1.0, 2.0, 3.0);

	~vectorn(){}

	matrixnView column() const;	// return n by 1 matrix, which can be used as L-value (reference matrix)
	matrixnView row() const;	// return 1 by n matrix, which can be used as L-value (reference matrix)
	matrixnView matView(int nrow, int ncol);

	// L-value로 사용될수 있는, reference array를 만들어 return 한다.
	// ex) v.range(0,2).setValues(2, 1.0, 2.0);
	vectornView range(int start, int end, int step=1);
	const vectornView range(int start, int end, int step=1) const	;

	//////////////////////////////////////////////////////////////////////
	// binary operations.
	//////////////////////////////////////////////////////////////////////
	vectorn&  normalize(vectorn const& min, vectorn const& max);				//!< scale data so that min->0, max->1

	// slow binary operations, I recommend you to use _tvectorn<T>::add function, instead.
	friend vectorn operator+( vectorn const& a, vectorn const& b);
	friend vectorn operator-( vectorn const& a, vectorn const& b);
	friend vectorn operator*( vectorn const& a, vectorn const& b );
	friend vectorn operator/( vectorn const& a, vectorn const& b );
	friend vectorn operator+( vectorn const& a, m_real b);
	friend vectorn operator-( vectorn const& a, m_real b);
	friend vectorn operator*( vectorn const& a, m_real b);
	friend vectorn operator/( vectorn const& a, m_real b);
	friend vectorn operator*( matrixn const& a, vectorn const& b );


	//////////////////////////////////////////////////////////////////////
	// binary scalar operations
	//////////////////////////////////////////////////////////////////////
	m_real angle(vectorn const& other) const;			// calc angle between 0 to pi
	m_real angle2D(vectorn const& b) const;			// calc angle between 0 to 2pi
	m_real distance(vectorn const& other, Metric* pMetric=NULL) const;
	m_real cosTheta(vectorn const& other) const;			// cosTheta for every dimensional space
	m_real	sinTheta(vectorn const& b) const;				// sinTheta for only for 2D 
	bool   isSimilar(vectorn const& other,m_real thr=0.00001) const	{ if(distance(other)<=thr) return true; return false;}

	friend m_real operator%( vectorn const&, vectorn const& );		// dot product
	friend bool operator<(vectorn const& a, vectorn const& b);
	friend bool operator>(vectorn const& a, vectorn const& b);
	friend bool operator<=(vectorn const& a, vectorn const& b) { return !(a>b);};
	friend bool operator>=(vectorn const& a, vectorn const& b) { return !(a<b);};
	friend bool operator==(vectorn const& a, vectorn const& b) { return a.isSimilar(b);};

	//////////////////////////////////////////////////////////////////////
	// unary operations
	//
	// - binary operation이 unary로 사용가능하기에, 많이 생략되었음.
	//     ex) a.add(a,b)
	//////////////////////////////////////////////////////////////////////
	vectorn& assign(const vector3& other);
	vectorn& assign(const quater& other);
	vectorn& normalize(vectorn const& a);
	vectorn& assign(const vectorn& other) { _tvectorn<m_real>::assign(other);	return *this;}
	vectorn& resample(vectorn const& vec, int numSample);
	vectorn& concaten(vectorn const& a);

	// 카피해서 받아온다.

	// 카피해서 받아온다.

	vectorn& operator=(const _tvectorn<m_real>& other)	{ _tvectorn<m_real>::assign(other);return *this;}
	vectorn& operator=(const vectorn& other);//		{ assign(other);return *this;}
	vectorn& operator=(const vectornView& other);//	{ assign(other);return *this;}

    vectorn& operator=( vector3 const& other)		{ return assign(other);};
	vectorn& operator=( quater const& other)		{ return assign(other);};
	vectorn& derivative(vectorn const& a);
	vectorn Extract(const intvectorn& columns)	const { vectorn c; c.extract(*this, columns); return c;}
	vectorn& sort(vectorn const& source, intvectorn& sortedIndex);

	vectorn extractNonZeroValues(intvectorn& index, double thr=1e-9);
	// slow unary operations (negation)
	friend vectorn operator-( vectorn const& a);

	//////////////////////////////////////////////////////////////////////
	// void operations
	//////////////////////////////////////////////////////////////////////
	vectorn& normalize();
	vectorn& negate();

	//////////////////////////////////////////////////////////////////////
	// aggregate functions
	//////////////////////////////////////////////////////////////////////

	m_real aggregate(CAggregate::aggregateOP eOP) const;
	m_real length() const ;
	m_real minimum() const;
	m_real maximum()	const;
	m_real sum()	const;
	m_real squareSum() const;
	m_real avg() const;

	// matrix의 각 column vector들에 scalar unary를 적용한다. (결과 vector dim은 cols())
	// ex) v.aggregateEachColumn(vectorn::minimum, mat);
	void aggregateEachColumn(CAggregate::aggregateOP eOP, matrixn const& mat);
	// matrix의 각 row vector들에 scalar unary를 적용한다. (결과 vector dim은 rows())
	void aggregateEachRow(CAggregate::aggregateOP eOP, matrixn const& mat);
	void minimum(const matrixn& other) 	{ return aggregateEachColumn(CAggregate::MINIMUM, other);}
	void maximum(const matrixn& other) 	{ return aggregateEachColumn(CAggregate::MAXIMUM, other);}
	void mean(const matrixn& other) 	{ return aggregateEachColumn(CAggregate::AVG, other);}
	void lengths(matrixn const& in)   	{ return aggregateEachRow(CAggregate::LENGTH, in);}

	void findMax(m_real& max_v, int& max_index,int start=0,int end=INT_MAX) const;
	void findMin(m_real& min_v, int& min_index,int start=0,int end=INT_MAX) const;
	void colon(m_real start, m_real stepSize, int nSize=-1);
	void colon2(m_real start, m_real end, m_real stepSize=1);
	vectorn& linspace(m_real x1, m_real x2, int nSize=-1);
	//!< uniform sampling : sample centers of intervals in linspace of size n+1; eg> uniform (0,3, size=3) -> (  0.5, 1.5, 2.5 ).
	vectorn& uniform(m_real x1, m_real x2, int nSize=-1);


	///////////////////////////////////////////////////////////////////////
	// Utility functions
	//////////////////////////////////////////////////////////////////////

	TString output(const char* formatString="%.10g", int start=0, int end=INT_MAX) const;
	TString shortOutput() const;

	vectorn& fromMatrix(matrixn const& mat);

	void setVec3( int start, const vector3& src);
	void setQuater( int start, const quater& src);
	void setQuater6( int start, const quater& src);
	void setTransf(int start, const transf& t); // 7 dim format (p, (w,x,y,z))
	void setTransf9(int start, const transf& t); // 9 dim format (p, axis1, axis2) == YZ convention
												 
	// pytorch uses XY convention so...
	//  taesooLib uses axes y (column1) and z (column2) for 6d representation.
	//  but torch3d uses row0 and row1
	void convertAxesYZtoTorch6D(); // in-place
	void convertTorch6DtoAxesYZ(); // in-place
	bool isnan() const;

	vector3 toVector3(int startIndex=0)	const;
	quater toQuater(int startIndex=0) const;
	quater toQuater6(int startIndex=0) const;
	transf toTransf(int startIndex=0) const;
	transf toTransf9(int startIndex=0) const;

	friend class matrixn;

	inline void getVec3( int start, vector3& src) const	{ src=toVector3(start);}
	inline void getQuater( int start, quater& src) const	{ src=toQuater(start);}

	int getSize() const	{ return size();}


	void  setAt( intvectorn const& columnIndex, vectorn const& value);
	void  setAt( intvectorn const& columnIndex, double value);
	// deprecated - v::for_each, v::for_each1, v::for_each2 로 바꾸는 중.(operatorTemplate.hpp)

	// vector의 각 value들에 scalar binary를 적용한다.
	// ex) v.each(s2::MINIMUM, a, b);

	void each2(m_real (*s2_func)(m_real,m_real), vectorn const& a, vectorn const& b);
	void each2(const sv2::_op& op, const matrixn& a, const matrixn& b);
	void each2(const sv2::_op& op, const matrixn& a, const vectorn& b);
	void each1(void (*s1_func)(m_real&,m_real), vectorn const& a);
	void each1(void (*s1_func)(m_real&,m_real), m_real a);
	void each0(void (*s1_func)(m_real&,m_real))							{ return each1(s1_func, *this);}

	void op2(const v2::_op& op, const vectorn& a, const vectorn& b)			{ op.calc(*this, a,b);	}
	void op1(const v1::_op& op, const vectorn& a)							{ op.calc(*this, a);	}

	vectorn Each(void (*s1_func)(m_real&,m_real)) const;
	vectorn Each(m_real (*s2_func)(m_real,m_real), vectorn const& b) const;

	friend std::ostream& operator<< ( std::ostream& os, const vectorn& u );
	void parseString(int n_reserve, const std::string &source);
};



class vectornView :public vectorn
{
public:
	// L-value로 사용될수 있는, reference array로 만든다.
	vectornView (const m_real* ptrr, int size, int stride=1);
	// 값을 reference로 받아온다.
	vectornView(const _tvectorn<m_real>& other)		{ assignRef(other);}
	vectornView(const vectorn& other)				{ assignRef(other);}
	vectornView(const vectornView& other)			{ assignRef(other);}

	~vectornView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	vectorn& operator=(const vectorn & other)			{ assign(other);return *this;}
	vectorn& operator=(const _tvectorn<m_real>& other)	{ _tvectorn<m_real>::assign(other);return *this;}
	vectorn& operator=(const vectornView& other)		{ assign(other);return *this;}
	vectorn& operator=( vector3 const& other)		{ return assign(other);};
	vectorn& operator=( quater const& other)		{ return assign(other);};


	/* doesn't work in some gnu compilers.
    inline operator vectorn & () const { return (vectorn&)(*this);}
	*/
	// for directly passing to a parameter. 
	vectorn& lval() const { return (vectorn&)(*this);}  //!< ex> doSomeModification(a.range(1,3).lval());

	// cout << vec <<endl;
	friend std::ostream& operator<< ( std::ostream& os, const vectorn& u );
};



/*
namespace math
{
	void makeSamplingIndex(vectorn& out, int nLen, int numSample);		 //!< a simplified uniform sampling
	void colon(vectorn& out, m_real stepSize, int nSize=-1);//!< start부터 stepSize간격으로 size()만큼 채움. (start, start+stepSize, start+stepSize*2,...)
	void linspace(vectorn& out, m_real x1, m_real x2, int nSize=-1);		 //!< generates a vector of linearly equally spaced points between X1 and X2; eg> linspace(0,3, size=4) -> (0,   1,   2,   3)
	void uniform(vectorn& out, m_real x1, m_real x2, int nSize=-1);		 //!< uniform sampling : sample centers of intervals in linspace of size n+1; eg> uniform (0,3, size=3) -> (  0.5, 1.5, 2.5 ).
	void findMin(vectorn const& in, m_real& min, int& min_index) const;
	void findMax(m_real& max, int& max_index) const;
	int argMin(vectorn const& in)			{ m_real min; int min_index; findMin(min, min_index); return min_index;}
	int argMax(vectorn const& in)			{ m_real max; int max_index; findMax(max, max_index); return max_index;}
	int argNearest(vectorn const& in, m_real value) ;
	vectorn& interpolate(vectorn const& a, vectorn const& b, m_real t);

}*/


namespace v
{
	void eig(vectorn& eigenvalues, const matrixn& mat);
	void linspace(vectorn& out, m_real x1, m_real x2, int nSize=-1);		 //!< generates a vector of linearly equally spaced points
	void findMin(const vectorn& v, m_real& min_v, int& min_index) ;
	void findMax(const vectorn& v, m_real& max_v, int& max_index) ;
	intvectorn colon(int start, int end);
}

#endif
