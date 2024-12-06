#ifndef MATRIX3_H_
#define MATRIX3_H_

#pragma once

#include "quater.h"
class matrix3  
{
public:
	//m_real m_Mx[9];

	union {
        struct {
            m_real        _11, _12, _13;
            m_real        _21, _22, _23;
            m_real        _31, _32, _33;
        };
        m_real m[3][3];
    };

public:
	matrix3();
	matrix3(m_real v);
	matrix3(matrix3 const& b);
	matrix3(quater const& q){ setFromQuaternion(q);}
	~matrix3();
	bool operator==(matrix3 const& b) const;
	bool operator!=(matrix3 const& b) const	{ return !(this->operator==(b));}

	void add(matrix3 const& a,matrix3 const& b);
	void sub(matrix3 const& a,matrix3 const& b);
	void mult(matrix3 const& a,matrix3 const& b);
	void mult(matrix3 const& a, m_real b);
	inline vector3 mult(const vector3 &b) const
	{
		vector3 c;
		c.x=_11*b.x+_12*b.y+_13*b.z;
		c.y=_21*b.x+_22*b.y+_23*b.z;
		c.z=_31*b.x+_32*b.y+_33*b.z;
		return c;
	}
	inline vector3 multT(const vector3 &b) const
	{
		vector3 c;
		c.x=_11*b.x+_21*b.y+_31*b.z;
		c.y=_12*b.x+_22*b.y+_32*b.z;
		c.z=_13*b.x+_23*b.y+_33*b.z;
		return c;
	}
	inline quater toQuater() const { quater q; q.setRotation(*this); return q;}

	friend vector3 operator*(matrix3 const& a, vector3 const& b);
	friend vector3 operator*(vector3 const& b, matrix3 const& a);
	inline friend matrix3 operator*(matrix3 const& a, matrix3 const& b)	{ matrix3 c; c.mult(a,b); return c;}
	inline friend matrix3 operator*(double a, matrix3 const& b){ matrix3 c; c.mult(b,a); return c;}
	inline friend matrix3 operator*(matrix3 const& a, double b){ matrix3 c; c.mult(a,b); return c;}

	void setValue( m_real a00, m_real a01, m_real a02,
			  m_real a10, m_real a11, m_real a12, 
			  m_real a20, m_real a21, m_real a22 );
	
	void setValue( vector3 const&row1, vector3 const&row2, vector3 const&row3 );

	// set skew 
	void setTilde( m_real x, m_real y, m_real z );
	void setTilde( vector3 const &v );
	inline vector3 unskew() const { return vector3(_23*-1.0, _13, _12*-1.0);}
	void setFromOuterProduct( vector3 const &v1, vector3 const&v2 );
	void setFromQuaternion(quater const& q);

	inline vector3 &row(int iRow ) 	{	return *((vector3 *)(&m[iRow][0]));	}

	matrix3& operator=( matrix3 const&other );
	void operator+=(matrix3 const&other );
	void operator-=(matrix3 const&other );
	void operator*=(matrix3 const&other );
	void operator*=(m_real scalar);
	matrix3 operator+(matrix3 const& b)	const { matrix3 c; c.add(*this,b); return c;}
	matrix3 operator-(matrix3 const& b)	const { matrix3 c; c.sub(*this,b); return c;}
	matrix3 operator-()	const { matrix3 c(*this); c.negate(); return c;}

	void setRotation(const quater& q);
	void zero();
	void identity();
	void transpose( void );
	inline matrix3 transpose() const { matrix3 o(*this); o.transpose(); return o;}
	void negate( void );
	bool inverse(matrix3 const& a);
	bool inverse();
	void rotate(vector3 & inout) const;

	bool isSymmetric()const;
	bool isTranspose( matrix3 const& other )const;

	m_real& operator[](int i) ;	// indexed by 0,1,2,3,4,5,6,7,8
	m_real operator[](int i) const;	// indexed by 0,1,2,3,4,5,6,7,8

	inline m_real& operator()(int i, int j) {	return m[i][j];}
	inline m_real operator()(int i, int j) const {	return m[i][j];}


	// deprecated
	void PostMultiply( vector3 const &v, vector3 &Dst )const ;
	void PreMultiply( vector3 const &v, vector3 &Dst )const ;
	void CrossProduct( vector3 const &v, matrix3 &Dst )const ;

	
	// highly deprecated 
	
	void Multiply( matrix3 const &M, matrix3 &Dst )const ;
	void Add( matrix3 const &M, matrix3 &Dst ) const ;
	void Subtract( matrix3 const &M, matrix3 &Dst ) const ;
	void Multiply( m_real scale, matrix3 &Dst )const ;
	void Invert( matrix3 &Dst )const ;



	void Dump( char *szHeading = NULL );
	friend std::ostream& operator<< ( std::ostream& os, const matrix3& u );

};

namespace m
{
	void assign(matrixn & out, matrix3 const& M);
	void assign(matrix3 & out, matrixn const& M);
}

#endif
