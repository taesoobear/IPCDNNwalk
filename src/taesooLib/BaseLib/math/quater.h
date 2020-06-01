#ifndef _QUATER_H_
#define _QUATER_H_

#pragma once
#include "vector3.h"
class matrix3;
//! quaternion class
/*! 
	\ingroup group_math

	Definition:
	(axis*sin(theta/2), cos(theta/2))
	

주의 사항: 
quater클래스와 matrix4클래스는 openGL의 convention을 따른다. 가장 오른쪽에 있는것이 로컬 트랜스폼이다.
quaternion도 matrix와 마찬가지로 q1*q2*v형태로 정의상 오른쪽에 점 위치가 곱해진다고 생각해야 한다.
참고: D3DXMATRIX는 v*R1*R2형태로 점이 왼쪽에 곱해지고, 마찬가지로 D3DXQUATERNION은 q1*q2가 quater 정의의 q2*q1과 일치하도록 
곱하기순서가 바뀌어 정의 되어있다. 즉 D3DXMATRIX와 D3DXQUATERNION모두 왼쪽에 있는 것이 로컬, 가장 오른쪽에 있는 것이 글로벌이다.
*/

class quater
{
public:
	m_real w, x, y, z;
	
	//
    // constructors
	//
    quater() {}
    quater( m_real ww, m_real xx, m_real yy, m_real zz )			{ w=ww; x=xx; y=yy; z=zz;}
    quater( m_real a[4] )										{ w=a[0]; x=a[1]; y=a[2]; z=a[3]; }
	quater(const vector3& vec, m_real ww)						{ x=vec.x; y=vec.y; z=vec.z; w=ww;}
	quater(m_real angle, const vector3& axis)					{ setRotation(axis, angle);}
	quater(const vector3& vec)									{ x=vec.x; y=vec.y; z=vec.z; w=0;}
	quater(const quater& q); // copy constructor

	// binary operations
	// 이경우 lvalue가 output이 된다. 즉 c=a+b -> c.add(a,b);
	void add(quater const& a, quater const& b);
	void subtract(quater const& a, quater const& b);
	void mult(quater const& a, m_real b);
	void mult(quater const& a, quater const& b);// cross product
    void pow( vector3 const&, m_real );
    void slerp( quater const&, quater const&, m_real );
	void safeSlerp( quater const&, quater const&, m_real );// align이 안되어 있어도 동작
    void interpolate( m_real, quater const&, quater const& );
	void unitAxisToUnitAxis2(const vector3& vFrom, const vector3& vTo);
	void axisToAxis( const vector3& vFrom, const vector3& vTo);
	

	// Decompose a quaternion into q= rotAxis_Y * offset. (Same as the decomposeTwistTimesNoTwist function when rkAxis=(0,1,0))
	void decompose(quater& rotAxis_y, quater& offset) const;

	// Decompose a quaternion into q = q_twist * q_no_twist, where q is 'this'
    // quaternion.  If V1 is the input axis and V2 is the rotation of V1 by
    // q, q_no_twist represents the rotation about the axis perpendicular to
    // V1 and V2 (see Quaternion::Align), and q_twist is a rotation about V1.
    void decomposeTwistTimesNoTwist (const vector3& rkAxis, quater& rkTwist, quater& rkNoTwist) const;

    // Decompose a quaternion into q = q_no_twist * q_twist, where q is 'this'
    // quaternion.  If V1 is the input axis and V2 is the rotation of V1 by
    // q, q_no_twist represents the rotation about the axis perpendicular to
    // V1 and V2 (see Quaternion::Align), and q_twist is a rotation about V1.
    void decomposeNoTwistTimesTwist (const vector3& rkAxis, quater& rkNoTwist, quater& rkTwist) const;

	void scale(m_real s);
		
	// derivatives
	void difference(quater const& q1, quater const& q2);			//!< quaternion representation of "angular velocity w" : q2*inv(q1);
	void toLocal(quater const& parent, quater const& child);		//!< inv(parent)*child
	void derivative(quater const& q1, quater const& q2);	//!< q'(t)=(1/2) w q; or q'(t)=q2-q1; -> both produces almost same result
	
	void toAxisAngle(vector3& axis, m_real& angle) const;
	void blend(const vectorn& weight, matrixn& aInputQuater);
	void bezier(const quater& q0, const quater& q1, const quater& q2, const quater& q3, m_real t);
	void hermite(const quater& qa, const vector3& wa, const quater& qb, const vector3& wb, m_real t);
		
	m_real    operator%( quater const&) const;    // dot product
	m_real	  distance( quater const& ) const;	
    quater    operator+( quater const& b) const		{ quater c;c.add(*this,b); return c;}
    quater    operator-( quater const& b) const		{ quater c;c.subtract(*this,b); return c;}
    quater    operator-() const						{ quater c; c.negate(*this); return c;};
	quater    operator*( quater const& b) const		{ quater c;c.mult(*this,b); return c;}
	quater    operator*( m_real b) const			{ quater c;c.mult(*this,b); return c;}
	vector3	  operator*(vector3 const& b) const		{ vector3 c; c.rotate(*this, b); return c;}
	quater    operator/( m_real b) const			{ quater c;c.mult(*this,1/b);return c;}
	friend quater    operator*(m_real a, quater const& b) { quater c;c.mult(b,a); return c;}

	// unary operations
	//! quaternion exp
	/*!
	Given a pure quaternion defined by:
	Q = (0, theta/2 * v); -> this means theta/2 rotation vector

	This method calculates the exponential result.
	exp(Q) = (cos(theta/2), sin(theta/2) * v) -> theta rotation
	*/
    void exp( vector3 const& );
	vector3 log() const						{ vector3 o; o.ln(*this); return o;}
    void inverse(const quater& a)			{ w=a.w; x=a.x*-1.0; y=a.y*-1.0; z=a.z*-1.0; normalize();}
	quater   inverse() const				{ quater c; c.inverse(*this); return c;};
	void negate(quater const& a);
	void setRotation(const matrix4& a);
	void setRotation(const matrix3& a);
	void setRotation(const char* aChannel, m_real *aValue, bool bRightToLeft=false);	//!< from euler angle. aChannel="YXZ" or something like that.
	void getRotation(const char* aChannel, m_real *aValue, bool bRightToLeft=false) const;	//!< to euler angle. aChannel="YXZ" or something like that.
	void setRotation(const vector3& axis, m_real angle);
	void setRotation(const vector3& rotationVector)	{ exp(rotationVector/2.0); }
	// vecAxis should be a unit vector.
	void setAxisRotation(const vector3& vecAxis, const vector3& front, const vector3& vecTarget);
	void leftMult(quater const& a)			{ quater copy(*this); mult(a, copy);}
	void rightMult(quater const& a)			{ quater copy(*this); mult(copy,a);}
	void normalize(const quater& a);
	void align(const quater& other)			{ quater& c=*this; if(c%other<0) c*=-1;}
	void operator=(quater const& a)			{ setValue(a.w, a.x, a.y, a.z);} 
	void operator+=(quater const& a)		{ add(*this,a);}
	void operator-=(quater const& a)		{ subtract(*this,a);}
	void operator*=(quater const& a)		{ rightMult(a);}
	void operator*=(m_real a)				{ x*=a; y*=a; z*=a; w*=a;}
	void operator/=(m_real a)				{ m_real inv=1.0/a; (*this)*=inv;}

	vector3 rotationVector() const { vector3 c; c.rotationVector(*this); return c;}

	void normalize();
	void negate()	{ negate(*this);}
	void identity()	{ w=1.0; x=0.0; y=0.0; z=0.0;}
	m_real rotationAngle() const	{ vector3 temp; temp.rotationVector(*this); return temp.length();}
	m_real rotationAngle(const vector3& axis) const { vector3 temp; temp.rotationVector(*this); if(temp%axis<0) return temp.length()*-1.0; else return temp.length();}
	m_real rotationAngleAboutAxis(const vector3& axis) const;

	//
    // inquiry functions
	//
	m_real	real() const					{ return w; }
	vector3 imaginaries() const				{ vector3 out; out.x=x; out.y=y; out.z=z; return out;}
	m_real  length() const;
	m_real& operator[] (int i)				{ return (&w)[i];}
    const m_real& operator[] (int i) const 	{ return (&w)[i];}
	m_real getValue(int i)const				{ return (&w)[i];}
	void getValue( m_real d[4] )			{ d[0]=w; d[1]=x; d[2]=y; d[3]=z; }
    void setValue( m_real d[4] )			{ w=d[0]; x=d[1]; y=d[2]; z=d[3]; }
	void setValue( m_real ww,m_real xx, m_real yy, m_real zz ){ w=ww; x=xx; y=yy; z=zz;}
	void setValue( const vector3& vec, m_real ww)			  { x=vec.x; y=vec.y; z=vec.z; w=ww;}

	TString output(bool rotationVector=false);

	friend std::ostream &operator << (std::ostream &os, const quater &s);
};


#endif
