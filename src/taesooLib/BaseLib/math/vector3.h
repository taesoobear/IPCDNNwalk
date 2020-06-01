#ifndef _VECTOR3_H_
#define _VECTOR3_H_
#if _MSC_VER > 1000
#pragma once
#endif
#include "../utility/TypeString.h"
#include <iostream>
class quater;
class matrix4;
class vectorn;
class matrixn;
/// never make virtual functions.
/** 이유: 이것의 주소는 direct하게 D3DXVECTOR3의 주소로 바뀔수 있다. virtual function을 만들면 VTable때문에 이것이 안된다.
	\ingroup group_math
*/

#if _MSC_VER > 1000
//#pragma message("Compiling vector.h - this should happen just once per project.\n")
#endif

class vector3
{
	public:

	double x, y, z;

    // constructors
    vector3() {}
	vector3( double v)				{ x=v; y=v; z=v;}
	vector3( double xx, double yy, double zz )				{ x=xx; y=yy; z=zz;}
    //vector3( double a[3] )								{ x=a[0]; y=a[1]; z=a[2]; }

	// binary operations
	// this가 output이다.
	void add(const vector3&, const vector3&);
	void sub(const vector3&, const vector3&);
	void multadd(const vector3&, double);	//!< c.multadd(b,w) -> c+=b*w;
	void mult(const vector3& ,double);
	void mult(const matrix4& mat, const vector3& v);
	void divide(const vector3& v, double a)	{ mult(v, 1/a);};
	void cross(const vector3&, const vector3&);
	vector3 cross(const vector3& other) const	{ vector3 c; c.cross(*this, other); return c;}
	vector3 mult(const vector3& o) const		{ return vector3(x*o.x, y*o.y, z*o.z);}
	void normalize( const vector3 & );
	vector3 dir()	{ vector3 temp; temp.normalize(*this); return temp; }
	void negate(const vector3&);
	void interpolate( double, vector3 const&, vector3 const& );
	void lerp( vector3 const& a, vector3 const& b, double t)	{ interpolate(t, a, b);}
	void blend(const vectorn& weight, const matrixn& aInputVector);
	//! quaternion ln
	/*!	A unit quaternion, is defined by:
	Q == (cos(theta), sin(theta) * v) where |v| = 1
	The natural logarithm of Q is, ln(Q) = (0, theta * v)*/

	void ln( const quater& q);
	void log( const quater& q)		{ ln(q);}

	// to avoid unnecessary copy, use q.exp(v) instead.
	quater exp() const;
	void rotate( const quater& q);
	void rotate( const matrix4& m); // applies only the linear part of m.
	void rotate( const quater& q, vector3 const& in);
	void angularVelocity( quater const& q1, quater const& q2);
	void linearVelocity(vector3 const& v1, vector3 const& v2);
	inline void difference(vector3 const& v1, vector3 const& v2)	{ linearVelocity(v1, v2);}

    double    operator%( vector3 const&) const;
    double    operator/( vector3 const&) const;
    vector3    operator+( vector3 const& ) const;
    vector3    operator-( vector3 const& ) const;

    vector3    operator*( double ) const;
    vector3    operator/( double ) const;
	friend vector3    operator*( double, vector3 const& ) ;

	// element multiplication (예전 버전에서는 crossproduct 로 정의되어 있었음. 주의할 것).
	vector3    operator*( vector3 const& ) const;
	void operator*=(vector3 const& );
	bool operator==(vector3 const& )const;

	// unary operations
	void add(double value);
	void add(const vector3& other);
	void sub(double value);
	void sub(const vector3& other);
	void column(int col, const matrix4& other);	//!< extract a column from the rotation part in a matrix
	void row(int col, const matrix4& other);		//!< extract a row from the rotation part in a matrix
	void translation(const matrix4& other);		//!< extract translation vector from a matrix
	void imaginaries(const quater& in);			//!< extract imagenaries from a quaternion
	double distance(const vector3& other) const;
	double squaredDistance(const vector3& other) const;
	void leftMult( const matrix4& mat);		//!< 당연히 this=mat*this; 로 정의된다.
	void rotationVector(const quater& in);
	quater quaternion() const;	//!< rotation vector를 quaternion으로 바꾼다.	== quater q; q.setRotation(*this); }

	void operator+=( vector3 const& );
    void operator-=( vector3 const& );
    void operator*=( double );
    void operator/=( double );
	vector3& operator=(vector3 const& );
	void hermite(const vector3& p1, const vector3& t1, const vector3& p2, const vector3& t2, double t);	//!< hermite interpolation of p1 and p2. 0<=t<=1
	vector3  operator-() const	{ vector3 c; c.negate(*this); return c;};

	inline void makeFloor( const vector3& cmp )
    {
        if( cmp.x < x ) x = cmp.x;
        if( cmp.y < y ) y = cmp.y;
        if( cmp.z < z ) z = cmp.z;
    }

    inline void makeCeil( const vector3& cmp )
    {
        if( cmp.x > x ) x = cmp.x;
        if( cmp.y > y ) y = cmp.y;
        if( cmp.z > z ) z = cmp.z;
    }

	// functions
	void normalize();
	void zero();
	double length() const;
	double squaredLength() const;
	// calc 0 to pi angle
    double angle( vector3 const& ) const;
	double cosTheta(vector3 const&) const;
	double sinTheta(vector3 const& b) const;
	// calc 0 to 2pi angle assuming z=0 (plane constraint)
	double angle2d(vector3 const& b) const;
	// calc -pi to pi angle assuming z=0 (plane constraint)
	double angle2ds(vector3 const& b) const;

	// calc -pi to pi angle assuming axis=0 (plane constraint): axis 0:x, 1: y, 2:z
	double angle2ds(vector3 const& b, int axis) const;

    double& operator[] (int i)						{ return (&x)[i];}
	const double& operator[] (int i) const			{ return (&x)[i];}
	double  getValue( int i ) const					{ return (&x)[i];}
    void   getValue( double d[3] )					{ d[0]=x; d[1]=y; d[2]=z;}

    void   setValue( double d[3] )					{ x=d[0]; y=d[1]; z=d[2]; }
	void   setValue(double xx, double yy, double zz )	{ x=xx; y=yy; z=zz;}
	void   setValue(const vectorn& other, int start=0);

	TString output() const;

	inline operator const double*() const		{ return &x;}
	inline operator double*()					{ return &x;}

	friend std::ostream& operator<< ( std::ostream& os, const vector3& u );

};

#endif
