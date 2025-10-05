#ifndef _FLOATVEC_N_H_
#define _FLOATVEC_N_H_
#pragma once
class boolN;
#include "vector3.h"
#include "quater.h"
#include "../utility/TypeString.h"
#include "../utility/util.h"

#include <typeinfo>
class floatvec;
class floatvecView;
#include "template_math.h"

class floatvec : public _tvectorn<float>
{
protected:
	floatvec(float* ptrr, int size, int stride):_tvectorn<float>(ptrr,size,stride){}
public:
	floatvec();
	floatvec(const vector3& other);
	floatvec(const quater& other);

	// 값을 카피해서 받아온다.
	floatvec(const _tvectorn<float>& other);
	floatvec(const floatvec& other);
	floatvec(const floatvecView& other);

	explicit floatvec( int x):_tvectorn<float>() { setSize(x);}

	// n dimensional vector	(ex) : floatvec(3, 1.0, 2.0, 3.0);
	explicit floatvec( int n, float x);
	explicit floatvec( int n, float x, float y);
	explicit floatvec( int n, float x, float y, float z);

	~floatvec(){}

	// L-value로 사용될수 있는, reference array를 만들어 return 한다.
	// ex) v.range(0,2).setValues(2, 1.0, 2.0);
	floatvecView range(int start, int end, int step=1);
	const floatvecView range(int start, int end, int step=1) const	;

	//////////////////////////////////////////////////////////////////////
	// unary operations
	//
	// - binary operation이 unary로 사용가능하기에, 많이 생략되었음.
	//     ex) a.add(a,b)
	//////////////////////////////////////////////////////////////////////
	floatvec& assign(const vector3& other);
	floatvec& assign(const quater& other);
	floatvec& assign(const floatvec& other) { _tvectorn<float>::assign(other);	return *this;}
	floatvec& assign(const vectorn& other);
	floatvec& concaten(floatvec const& a);

	// 카피해서 받아온다.

	// 카피해서 받아온다.

	floatvec& operator=(const _tvectorn<float>& other)	{ _tvectorn<float>::assign(other);return *this;}
	floatvec& operator=(const floatvec& other);//		{ assign(other);return *this;}
	floatvec& operator=(const floatvecView& other);//	{ assign(other);return *this;}

    floatvec& operator=( vector3 const& other)		{ return assign(other);};
	floatvec& operator=( quater const& other)		{ return assign(other);};


	///////////////////////////////////////////////////////////////////////
	// Utility functions
	//////////////////////////////////////////////////////////////////////

	TString output(const char* formatString="%f", int start=0, int end=INT_MAX) const;

	void setVec3( int start, const vector3& src);
	void setQuater( int start, const quater& src);

	vector3 toVector3(int startIndex=0)	const;
	quater toQuater(int startIndex=0) const;

	inline void getVec3( int start, vector3& src) const	{ src=toVector3(start);}
	inline void getQuater( int start, quater& src) const	{ src=toQuater(start);}

	friend std::ostream& operator<< ( std::ostream& os, const floatvec& u );
};



class floatvecView :public floatvec
{
public:
	// L-value로 사용될수 있는, reference array로 만든다.
	floatvecView (float* ptrr, int size, int stride);
	// 값을 reference로 받아온다.
	floatvecView(const _tvectorn<float>& other)		{ assignRef(other);}
	floatvecView(const floatvec& other)				{ assignRef(other);}
	floatvecView(const floatvecView& other)			{ assignRef(other);}

	~floatvecView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	floatvec& operator=(const floatvec & other)			{ assign(other);return *this;}
	floatvec& operator=(const _tvectorn<float>& other)	{ _tvectorn<float>::assign(other);return *this;}
	floatvec& operator=(const floatvecView& other)		{ assign(other);return *this;}
	floatvec& operator=( vector3 const& other)		{ return assign(other);};
	floatvec& operator=( quater const& other)		{ return assign(other);};

	// for directly passing to a parameter. 
	floatvec& lval() const { return (floatvec&)(*this);}  //!< ex> doSomeModification(a.range(1,3).lval());

	// cout << vec <<endl;
	friend std::ostream& operator<< ( std::ostream& os, const floatvec& u );
};


#endif
