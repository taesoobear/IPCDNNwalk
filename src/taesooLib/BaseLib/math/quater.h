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
	double w, x, y, z;
	
	//
    // constructors
	//
    quater() {}
    quater( double ww, double xx, double yy, double zz )			{ w=ww; x=xx; y=yy; z=zz;}
    quater( double a[4] )										{ w=a[0]; x=a[1]; y=a[2]; z=a[3]; }
	quater(const vector3& vec, double ww)						{ x=vec.x; y=vec.y; z=vec.z; w=ww;}
	quater(double angle, const vector3& axis)					{ setRotation(axis, angle);}
	quater(const vector3& vec)									{ x=vec.x; y=vec.y; z=vec.z; w=0;}
	quater(const quater& q); // copy constructor

	// binary operations
	// 이경우 lvalue가 output이 된다. 즉 c=a+b -> c.add(a,b);
	void add(quater const& a, quater const& b);
	void subtract(quater const& a, quater const& b);
	void mult(quater const& a, double b);
	void mult(quater const& a, quater const& b);// cross product
    void pow( vector3 const&, double );

	// *this= ((1.0-t)*a + t*b); -> on the hyper-sphere.
    void slerp( quater const& a, quater const& b, double t);
	void safeSlerp( quater const& a, quater const& b, double t);// align이 안되어 있어도 동작
    void interpolate( double t, quater const& a, quater const& b);
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

	void scale(double s);
		
	// derivatives
	void difference(quater const& q1, quater const& q2);			//!< quaternion representation of "global angular velocity w" : q2*inv(q1);    
	void toLocal(quater const& parent, quater const& child);		//!< inv(parent)*child
	void derivative(quater const& q1, quater const& q2);	//!< q'(t)=(1/2) w q; or q'(t)=q2-q1; -> both produces almost same result
	
	void toAxisAngle(vector3& axis, double& angle) const;
	void blend(const vectorn& weight, matrixn& aInputQuater);
	void bezier(const quater& q0, const quater& q1, const quater& q2, const quater& q3, double t);
	void hermite(const quater& qa, const vector3& wa, const quater& qb, const vector3& wb, double t);
		
	double    operator%( quater const&) const;    // dot product
	double	  distance( quater const& ) const;	
    inline quater    operator+( quater const& b) const		{ quater c;c.add(*this,b); return c;}
    inline quater    operator-( quater const& b) const		{ quater c;c.subtract(*this,b); return c;}
    inline quater    operator-() const						{ quater c; c.negate(*this); return c;};
	inline quater    operator*( quater const& b) const		{ quater c;c.mult(*this,b); return c;}
	inline quater    operator*( double b) const			{ quater c;c.mult(*this,b); return c;}
	inline vector3	  operator*(vector3 const& b) const		{ vector3 c; c.rotate(*this, b); return c;}
	inline quater    operator/( double b) const			{ quater c;c.mult(*this,1/b);return c;}
	friend quater    operator*(double a, quater const& b) { quater c;c.mult(b,a); return c;}

	// unary operations
	//! quaternion exp
	/*!
	Given a pure quaternion defined by:
	Q = (0, theta/2 * v); -> this means theta/2 rotation vector

	This method calculates the exponential result.
	exp(Q) = (cos(theta/2), sin(theta/2) * v) -> theta rotation
	*/
    void exp( vector3 const& );
	inline vector3 log() const						{ vector3 o; o.ln(*this); return o;}
    inline void inverse(const quater& a)			{ w=a.w; x=a.x*-1.0; y=a.y*-1.0; z=a.z*-1.0; normalize();}
	inline quater   inverse() const				{ quater c; c.inverse(*this); return c;};
	inline quater rotationY() const 				{ quater rotY, offset; decompose(rotY, offset); return rotY;}
	void negate(quater const& a);
	void setRotation(const matrix4& a);
	void setRotation(const matrix3& a);
	void setRotation(const char* aChannel, double *aValue, bool bRightToLeft=false);	//!< from euler angle. aChannel="YXZ" or something like that.
	void getRotation(const char* aChannel, double *aValue, bool bRightToLeft=false) const;	//!< to euler angle. aChannel="YXZ" or something like that.
	void setRotation(const vector3& axis, double angle);
	inline void setRotation(const vector3& rotationVector)	{ exp(rotationVector/2.0); }
	// vecAxis should be a unit vector.
	void setAxisRotation(const vector3& vecAxis, const vector3& front, const vector3& vecTarget);
	inline void leftMult(quater const& a)			{ quater copy(*this); mult(a, copy);}
	inline void rightMult(quater const& a)			{ quater copy(*this); mult(copy,a);}
	void normalize(const quater& a);
	inline void align(const quater& other)			{ quater& c=*this; if(c%other<0) c*=-1;}
	inline void operator=(quater const& a)			{ setValue(a.w, a.x, a.y, a.z);} 
	inline void operator+=(quater const& a)		{ add(*this,a);}
	inline void operator-=(quater const& a)		{ subtract(*this,a);}
	inline void operator*=(quater const& a)		{ rightMult(a);}
	inline void operator*=(double a)				{ x*=a; y*=a; z*=a; w*=a;}
	inline void operator/=(double a)				{ double inv=1.0/a; (*this)*=inv;}

	inline vector3 rotationVector() const { vector3 c; c.rotationVector(*this); return c;}

	void normalize();
	inline quater normalized() const { quater q; q.normalize(*this); return q;}
	inline void negate()	{ negate(*this);}
	inline void identity()	{ w=1.0; x=0.0; y=0.0; z=0.0;}
	inline double rotationAngle() const	{ vector3 temp; temp.rotationVector(*this); return temp.length();}
	inline double rotationAngle(const vector3& axis) const { vector3 temp; temp.rotationVector(*this); if(temp%axis<0) return temp.length()*-1.0; else return temp.length();}
	double rotationAngleAboutAxis(const vector3& axis) const;

	//
    // inquiry functions
	//
	inline double	real() const					{ return w; }
	inline const vector3& imaginaries() const		{ return *((vector3*)&x);}
	double  length() const;
	double squaredLength() const 					{ return x*x + y*y + z*z + w*w;}
	inline double& operator[] (int i)				{ return (&w)[i];}
    inline const double& operator[] (int i) const 	{ return (&w)[i];}
	inline double getValue(int i)const				{ return (&w)[i];}
	inline void getValue( double d[4] )			{ d[0]=w; d[1]=x; d[2]=y; d[3]=z; }
    inline void setValue( double d[4] )			{ w=d[0]; x=d[1]; y=d[2]; z=d[3]; }
	inline void setValue( double ww,double xx, double yy, double zz ){ w=ww; x=xx; y=yy; z=zz;}
	inline void setValue( const vector3& vec, double ww)			  { x=vec.x; y=vec.y; z=vec.z; w=ww;}

	inline quater angvel2qdot(const vector3& omega) const
	{
		const quater& q=*this;
		quater out;
		// W(t)=(0, omega.x, omega.y, omega.z)
		// qdot = 0.5 * W(t) * q
		// Using quaternion multiplication rule,...
		// qdot =0.5*[ -qx -qy -qz; qw qz -qy ; -qz qw qx; qy -qx qw] *[ x y z]'
		// .. is different from the code below. The above W(t) is the global angular velocity.
		//
		// When omega is the body angular velocity
		// qdot = 0.5 * q * W(t) 
		// Using quaternion multiplication rule,...

		double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
		double x = omega.x, y=omega.y, z = omega.z;
		out.w = - 0.5 * (qx*x + qy*y + qz*z);
		out.x = 0.5 * (qw*x - qz*y + qy*z);
		out.y = 0.5 * (qz*x + qw*y - qx*z);
		out.z = 0.5 * (- qy*x + qx*y + qw*z);
		return out;
	}
	TString output(bool rotationVector=false);

	friend std::ostream &operator << (std::ostream &os, const quater &s);
};


#endif
