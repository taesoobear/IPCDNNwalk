//////////////////////////////////////////////////////////////////////////////////
//
//		LieGroup.h modified by junggon (junggon@gmail.com)
//
//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.h 
//						
//		version		:	v0.97 
//		author		:	Jinwook Kim (zinook@plaza.snu.ac.kr)
//		last update	:	2001.9.6
//
//		Note		:	
//						v0.95 library title changed : robotics.* -> liegroup.*
//						v0.95 is independent of RMatrix
//						v0.95 supports SO3 class
//						v0.95 Inertia class uses smaller number of member variables
//						v0.95 supports friend functions InvAd, InvdAd
//						v0.95 supports /, % operators in SE3 class
//						v0.97 supports articulated inertia class
//						v0.97 supports dad(V, J) - fast calc. of dad(V, J * V)
//
//////////////////////////////////////////////////////////////////////////////////


#ifndef _LIEGROUP_
#define _LIEGROUP_

#include "math.h"
#include <iostream>
#include <list>

class se3;
class dse3;
class SO3;
class SE3;
class Inertia;
class AInertia;

class Vec3
{
public:
	union { // taesoo added .x .y .z access
		struct { double x,y,z; };
		double _v[3];
	};
public:
	// constructors
	Vec3() {}
	Vec3(double d) { _v[0] = _v[1] = _v[2] = d; }
	Vec3(const double v[]) { _v[0] = v[0]; _v[1] = v[1]; _v[2] = v[2]; }
	Vec3(double v0, double v1, double v2) { _v[0] = v0; _v[1] = v1; _v[2] = v2; }
	Vec3(float *v) { _v[0] = v[0]; _v[1] = v[1]; _v[2] = v[2]; };
	// operators
	const Vec3 &operator + (void) const { return *this; }						// unary plus 
	Vec3 operator - (void) const { return Vec3(-_v[0],-_v[1],-_v[2]); }		// unary minus 
	double &operator [] (int i) { return _v[i]; }
	double operator [] (int i) const { return _v[i]; }
	Vec3 &operator = (const Vec3 &v) { _v[0] = v._v[0]; _v[1] = v._v[1]; _v[2] = v._v[2]; return *this; }
	Vec3 &operator = (const double v[]) { _v[0] = v[0]; _v[1] = v[1]; _v[2] = v[2]; return *this; }
	Vec3 &operator += (const Vec3 &v) { _v[0] += v._v[0]; _v[1] += v._v[1]; _v[2] += v._v[2];	return *this; }
	Vec3 &operator -= (const Vec3 &v)	{ _v[0] -= v._v[0]; _v[1] -= v._v[1]; _v[2] -= v._v[2]; return *this; }
	Vec3 &operator *= (double d) { _v[0] *= d; _v[1] *= d; _v[2] *= d; return *this; }
	Vec3 operator * (double d) const { return Vec3(d * _v[0], d * _v[1], d * _v[2]); }
	Vec3 operator + (const Vec3 &v) const { return Vec3(_v[0] + v._v[0], _v[1] + v._v[1], _v[2] + v._v[2]); }
	Vec3 operator - (const Vec3 &v) const { return Vec3(_v[0] - v._v[0], _v[1] - v._v[1], _v[2] - v._v[2]); }
	// methods
	void SetZero(void) { _v[0] = _v[1] = _v[2] = 0.0; }
	double Normalize() { double mag = Norm(*this); _v[0] /= mag; _v[1] /= mag; _v[2] /= mag; return mag; }
	double *GetArray(void) { return _v; }
	const double *GetArray(void) const { return _v; }
	// friend functions
	friend std::ostream &operator << (std::ostream &os, const Vec3 &v);	// std::ostream standard output
	friend Vec3 operator * (double d, const Vec3 &v) { return Vec3(d * v._v[0], d * v._v[1], d * v._v[2]); }
	friend double Norm(const Vec3 &v) { return sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]); }
	friend Vec3 Cross(const Vec3 &p, const Vec3 &q) { return Vec3(p._v[1] * q._v[2] - p._v[2] * q._v[1], p._v[2] * q._v[0] - p._v[0]*q._v[2], p._v[0] * q._v[1] - p._v[1]*q._v[0]); }
	friend double Inner(const Vec3 &p, const Vec3 &q) { return (p._v[0] * q._v[0] + p._v[1] * q._v[1] + p._v[2] * q._v[2]); }
	friend se3 Ad(const Vec3 &p, const se3 &s);	
	friend SO3 Exp(const Vec3 &w);
	friend SO3 EulerZYX(const Vec3 &x);
	friend SO3 EulerZYZ(const Vec3 &x);
	friend SO3 EulerXYZ(const Vec3 &x);
	friend SO3 EulerZXY(const Vec3 &x);
	// friend classes
	friend class se3;
	friend class dse3;
	friend class SO3;
	friend class SE3;
};

class se3
{
private:
	double _w[6];	// upper three : angular velocity,  lower three : linear velocity
public:
	// constructors
	se3() { }
	se3(double k) { _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = k; }
	se3(double w0, double w1, double w2, double w3, double w4, double w5) { _w[0] = w0;	_w[1] = w1;	_w[2] = w2; _w[3] = w3;	_w[4] = w4;	_w[5] = w5; }
	se3(const se3 &s) { _w[0] = s._w[0]; _w[1] = s._w[1]; _w[2] = s._w[2]; _w[3] = s._w[3]; _w[4] = s._w[4]; _w[5] = s._w[5]; }
	se3(double *s) { _w[0] = s[0]; _w[1] = s[1]; _w[2] = s[2]; _w[3] = s[3]; _w[4] = s[4]; _w[5] = s[5]; }
	se3(const Vec3 &w, const Vec3 &v) { _w[0] = w._v[0]; _w[1] = w._v[1]; _w[2] = w._v[2]; _w[3] = v._v[0]; _w[4] = v._v[1]; _w[5] = v._v[2]; }
	// operators
	const se3 &operator + (void) const { return *this; }
	se3 operator - (void) const { return se3(-_w[0], -_w[1], -_w[2], -_w[3], -_w[4], -_w[5]); }
	se3 &operator = (const se3 &s) { _w[0] = s._w[0]; _w[1] = s._w[1]; _w[2] = s._w[2]; _w[3] = s._w[3]; _w[4] = s._w[4]; _w[5] = s._w[5]; return *this; }
	se3 &operator = (double d) { _w[0] = d; _w[1] = d; _w[2] = d; _w[3] = d; _w[4] = d; _w[5] = d; return *this; }
	se3 &operator += (const se3 &s) { _w[0] += s._w[0]; _w[1] += s._w[1]; _w[2] += s._w[2]; _w[3] += s._w[3]; _w[4] += s._w[4]; _w[5] += s._w[5]; return *this; }
	se3 &operator += (const double *s) { _w[0] += s[0]; _w[1] += s[1]; _w[2] += s[2]; _w[3] += s[3]; _w[4] += s[4]; _w[5] += s[5]; return *this; }
	se3 &operator -= (const se3 &s) { _w[0] -= s._w[0]; _w[1] -= s._w[1]; _w[2] -= s._w[2]; _w[3] -= s._w[3]; _w[4] -= s._w[4]; _w[5] -= s._w[5];	return *this; }
	se3 &operator *= (double d) { _w[0] *= d; _w[1] *= d; _w[2] *= d; _w[3] *= d; _w[4] *= d; _w[5] *= d; return *this; }
	se3 &operator /= (double d) { d = 1.0 / d; _w[0] *= d; _w[1] *= d; _w[2] *= d; _w[3] *= d; _w[4] *= d; _w[5] *= d; return *this; }
	se3 operator + (const se3 &s) const { return se3(_w[0] + s._w[0], _w[1] + s._w[1], _w[2] + s._w[2], _w[3] + s._w[3], _w[4] + s._w[4], _w[5] + s._w[5]); }
	se3 operator - (const se3 &s) const { return se3(_w[0] - s._w[0], _w[1] - s._w[1], _w[2] - s._w[2], _w[3] - s._w[3], _w[4] - s._w[4], _w[5] - s._w[5]); }
	se3 operator * (double d) const { return se3(d * _w[0], d * _w[1], d * _w[2], d * _w[3], d * _w[4], d * _w[5]); }
	se3 operator / (double d) const { d = 1.0 / d; return se3(d * _w[0], d * _w[1], d * _w[2], d * _w[3], d * _w[4], d * _w[5]); }
	double &operator [] (int i) { return _w[i]; }
	double operator [] (int i) const { return _w[i]; }
	// methods
	void SetZero(void) { _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = 0.0; }
	double InnerProductWith(const double *s) { return (_w[0] * s[0] + _w[1] * s[1] + _w[2] * s[2] + _w[3] * s[3] + _w[4] * s[4] + _w[5] * s[5]); }
	Vec3 GetW() const { return Vec3(_w[0],_w[1],_w[2]); }
	Vec3 GetV() const { return Vec3(_w[3],_w[4],_w[5]); }
	void Ad(const SE3 &T, const se3 &s);	// *this = Ad(T,s)
	void ad(const se3 &s1, const se3 &s2);	// *this = ad(s1,s2)
	double *GetArray(void) { return _w; }
	const double *GetArray(void) const { return _w; }
	// friend functions
	friend std::ostream &operator << (std::ostream &os, const se3 &s);	// std::ostream standard output
	friend se3 operator * (double d, const se3 &s) { return se3(d * s._w[0], d * s._w[1], d * s._w[2], d * s._w[3], d * s._w[4], d * s._w[5]); }
	friend double operator * (const dse3 &t, const se3 &s);
	friend double operator * (const se3 &s, const dse3 &t);
	friend SE3 Exp(const se3 &s);
	friend SE3 Exp(const se3 &s, double theta);
	friend se3 Log(const SE3 &T);
	friend se3 Ad(const SE3 &T, const se3 &s);
//	friend se3 InvAd(const SE3 &T, const se3 &s);
	friend se3 Ad(const Vec3 &p, const se3 &s);
	friend se3 ad(const se3 &s1, const se3 &s2);
	friend dse3 dad(const se3 &s, const dse3 &t);
	friend dse3 dad(const se3 &V, const Inertia &J);
	friend se3 InvSkew(const SE3 &T); // invskew(T - I)
	friend double SquareSum(const se3 &s) { return s._w[0] * s._w[0] + s._w[1] * s._w[1] + s._w[2] * s._w[2] + s._w[3] * s._w[3] + s._w[4] * s._w[4] + s._w[5] * s._w[5]; }


	// friend classes
	friend class SE3;
	friend class Inertia;
	friend class AInertia;
};

class dse3
{
private:
	double _m[6];
public:
	// constructors
	dse3() { }
	dse3(double k) { _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = k; }
	dse3(double m0, double m1, double m2, double m3, double m4, double m5) { _m[0] = m0; _m[1] = m1; _m[2] = m2; _m[3] = m3; _m[4] = m4; _m[5] = m5; }
	dse3(const dse3 &t) { _m[0] = t._m[0]; _m[1] = t._m[1]; _m[2] = t._m[2];	_m[3] = t._m[3]; _m[4] = t._m[4]; _m[5] = t._m[5]; }
	dse3(const Vec3 &m, const Vec3 &f) { _m[0] = m._v[0]; _m[1] = m._v[1]; _m[2] = m._v[2];	_m[3] = f._v[0]; _m[4] = f._v[1]; _m[5] = f._v[2]; }
	// operators
	const dse3 &operator + (void) const { return *this; }
	dse3 operator - (void) const { return dse3(-_m[0], -_m[1], -_m[2], -_m[3], -_m[4], -_m[5]); }
	dse3 &operator = (const dse3 &t) { _m[0] = t._m[0]; _m[1] = t._m[1]; _m[2] = t._m[2]; _m[3] = t._m[3]; _m[4] = t._m[4]; _m[5] = t._m[5]; return *this; }
	dse3 &operator = (double d) { _m[0] = d; _m[1] = d; _m[2] = d; _m[3] = d; _m[4] = d; _m[5] = d; return *this; }
	dse3 &operator += (const dse3 &t) { _m[0] += t._m[0]; _m[1] += t._m[1]; _m[2] += t._m[2]; _m[3] += t._m[3]; _m[4] += t._m[4]; _m[5] += t._m[5]; return *this; }
	dse3 &operator -= (const dse3 &t) { _m[0] -= t._m[0]; _m[1] -= t._m[1]; _m[2] -= t._m[2]; _m[3] -= t._m[3]; _m[4] -= t._m[4]; _m[5] -= t._m[5]; return *this; }
	dse3 &operator *= (double d) { _m[0] *= d; _m[1] *= d; _m[2] *= d; _m[3] *= d; _m[4] *= d; _m[5] *= d; return *this; }
	dse3 &operator /= (double d) { d = 1.0 / d; _m[0] *= d; _m[1] *= d; _m[2] *= d; _m[3] *= d; _m[4] *= d; _m[5] *= d; return *this; }
	dse3 operator + (const dse3 &t) const { return dse3(_m[0] + t._m[0], _m[1] + t._m[1], _m[2] + t._m[2], _m[3] + t._m[3], _m[4] + t._m[4], _m[5] + t._m[5]); }	
	dse3 operator - (const dse3 &t) const { return dse3(_m[0] - t._m[0], _m[1] - t._m[1], _m[2] - t._m[2], _m[3] - t._m[3], _m[4] - t._m[4], _m[5] - t._m[5]); }	
	dse3 operator * (double d) const { return dse3(d * _m[0], d * _m[1], d * _m[2], d * _m[3], d * _m[4], d * _m[5]); }
	double &operator [] (int i) { return _m[i]; }
	double operator [] (int i) const { return _m[i]; }
	// methods
	void SetZero(void) { _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = 0.0; }
	double InnerProductWith(const double *s) { return (_m[0] * s[0] + _m[1] * s[1] + _m[2] * s[2] + _m[3] * s[3] + _m[4] * s[4] + _m[5] * s[5]); }
	double *GetArray(void) { return _m; }
	const double *GetArray(void) const { return _m; }
	// friend functions
	friend std::ostream &operator << (std::ostream &os, const dse3 &t);	// std::ostream standard output
	friend dse3 operator * (double d, const dse3 &t) { return dse3(d * t._m[0], d * t._m[1], d * t._m[2], d * t._m[3], d * t._m[4], d * t._m[5]); }
	friend double operator * (const dse3 &t, const se3 &s) { return (t._m[0] * s._w[0] + t._m[1] * s._w[1] + t._m[2] * s._w[2] + t._m[3] * s._w[3] + t._m[4] * s._w[4] + t._m[5] * s._w[5]); }
	friend double operator * (const se3 &s, const dse3 &t) { return (t * s); }
	friend dse3 dAd(const SE3 &T, const dse3 &t);
//	friend dse3 InvdAd(const SE3 &T, const dse3 &t);
	friend dse3 dad(const se3 &s, const dse3 &t);	
	friend dse3 dad(const se3 &V, const Inertia &J);
	friend AInertia KroneckerProduct(const dse3 &x, const dse3 &y);
	friend double SquareSum(const dse3 &t) { return t._m[0] * t._m[0] + t._m[1] * t._m[1] + t._m[2] * t._m[2] + t._m[3] * t._m[3] + t._m[4] * t._m[4] + t._m[5] * t._m[5]; }
	friend Vec3 GetM(const dse3 &t) { return Vec3(t._m[0], t._m[1], t._m[2]); }
	friend Vec3 GetF(const dse3 &t) { return Vec3(t._m[3], t._m[4], t._m[5]); }
	// friend class
	friend class Inertia;
	friend class AInertia;
};

class SO3
{
private:
	double _R[9];
public:
	// constructors
	SO3() { _R[0] = _R[4] = _R[8] = 1.0; _R[1] = _R[2] = _R[3] = _R[5] = _R[6] = _R[7] = 0.0; }
	SO3(const SO3 &R) { _R[0] = R._R[0]; _R[3] = R._R[3]; _R[6] = R._R[6]; _R[1] = R._R[1]; _R[4] = R._R[4]; _R[7] = R._R[7]; _R[2] = R._R[2]; _R[5] = R._R[5]; _R[8] = R._R[8]; }
	SO3(const double R[]) { _R[0] = R[0]; _R[3] = R[3]; _R[6] = R[6]; _R[1] = R[1]; _R[4] = R[4]; _R[7] = R[7]; _R[2] = R[2]; _R[5] = R[5]; _R[8] = R[8]; }
	SO3(double R0, double R1, double R2, double R3, double R4, double R5, double R6, double R7, double R8) { _R[0] = R0; _R[1] = R1; _R[2] = R2; _R[3] = R3; _R[4] = R4; _R[5] = R5; _R[6] = R6; _R[7] = R7; _R[8] = R8; }
	// operators
	double operator () (int i, int j) const { return _R[i+3*j]; }
	double &operator () (int i, int j) { return _R[i+3*j]; }
	double operator [] (int i) const { return _R[i]; }
	double &operator [] (int i) { return _R[i]; }
	SO3 &operator = (const SO3 &R)  { _R[0] = R._R[0]; _R[3] = R._R[3]; _R[6] = R._R[6]; _R[1] = R._R[1]; _R[4] = R._R[4]; _R[7] = R._R[7]; _R[2] = R._R[2]; _R[5] = R._R[5]; _R[8] = R._R[8]; return *this; }
	SO3 &operator *= (const SO3 &R);
	SO3 operator * (const SO3 &R) const;
	Vec3 operator * (const Vec3 &p) const { return Vec3(_R[0] * p._v[0] + _R[3] * p._v[1] + _R[6] * p._v[2], _R[1] * p._v[0] + _R[4] * p._v[1] + _R[7] * p._v[2], _R[2] * p._v[0] + _R[5] * p._v[1] + _R[8] * p._v[2]); }
	SO3 operator ~ (void) const { return SO3(_R[0], _R[3], _R[6], _R[1], _R[4], _R[7], _R[2], _R[5], _R[8]); }
	// methods
	void SetIdentity(void) { _R[0] = _R[4] = _R[8] = 1.0; _R[1] = _R[2] = _R[3] = _R[5] = _R[6] = _R[7] = 0.0; }
	double *GetArray(void) { return _R; }
	const double *GetArray(void) const { return _R; }
	// friend functions
	friend std::ostream &operator << (std::ostream &os, const SO3 &R);	// std::ostream standard output
	friend SO3 Inv(const SO3 &R) { return SO3(R._R[0], R._R[3], R._R[6], R._R[1], R._R[4], R._R[7], R._R[2], R._R[5], R._R[8]); }
	friend SO3 Exp(double w0, double w1, double w2);
	friend SO3 Exp(const Vec3 &w);
	friend Vec3 Log(const SO3 &R);
	friend SO3 RotX(double theta);
	friend SO3 RotY(double theta);
	friend SO3 RotZ(double theta);
	friend SO3 EulerZYX(const Vec3 &x);		// singularity : x[1] = -+ 0.5*PI
	friend Vec3 iEulerZYX(const SO3 &R);
	friend SO3 EulerZYZ(const Vec3 &x);		// singularity : x[1] = 0, PI
	friend Vec3 iEulerZYZ(const SO3 &R);
	friend SO3 EulerZXY(const Vec3 &x);
	friend Vec3 iEulerZXY(const SO3 &R);
	friend SO3 EulerXYZ(const Vec3 &x);
	friend Vec3 iEulerXYZ(const SO3 &R);
	friend SO3 Quat(double *quat);						// quaternion(quat[4]) --> SO3
	friend void iQuat(const SO3 &R, double *quat);		// SO3 --> quaternion(quat[4])
	// eps= 1E-6
	friend bool isSO3(double *R, double eps );	// is R[9] is a rotation matrix?
	// friend class
	friend class SE3;
};

class SE3
{
private:
	double _T[16];	// column order 4 X 4 homogeneous transformation matrix
public:
	// constructors
	SE3() { _T[0] = _T[5] = _T[10] = _T[15] = 1.0; _T[1] = _T[2] = _T[3] = _T[4] = _T[6] = _T[7] = _T[8] = _T[9] = _T[11] = _T[12] = _T[13] = _T[14] = 0.0; }
	SE3(const SE3 &T) { _T[0] = T._T[0]; _T[1] = T._T[1]; _T[2] = T._T[2]; _T[4] = T._T[4]; _T[5] = T._T[5]; _T[6] = T._T[6]; _T[8] = T._T[8]; _T[9] = T._T[9]; _T[10] = T._T[10]; _T[12] = T._T[12]; _T[13] = T._T[13]; _T[14] = T._T[14]; _T[3] = _T[7] = _T[11] = 0.0; _T[15] = 1.0; }
	SE3(double T0, double T1, double T2, double T4, double T5, double T6, double T8, double T9, double T10, double T12, double T13, double T14) { _T[0] = T0; _T[1] = T1; _T[2] = T2; _T[4] = T4; _T[5] = T5; _T[6] = T6; _T[8] = T8; _T[9] = T9; _T[10] = T10; _T[12] = T12; _T[13] = T13; _T[14] = T14; _T[3] = _T[7] = _T[11] = 0.0; _T[15] = 1.0; }
	SE3(const SO3 &R, const Vec3 &p) { _T[0] = R._R[0]; _T[4] = R._R[3]; _T[8] = R._R[6]; _T[1] = R._R[1]; _T[5] = R._R[4]; _T[9] = R._R[7]; _T[2] = R._R[2]; _T[6] = R._R[5]; _T[10] = R._R[8]; _T[12] = p._v[0]; _T[13] = p._v[1]; _T[14] = p._v[2]; _T[3] = _T[7] = _T[11] = 0.0; _T[15] = 1.0; }
	SE3(const SO3 &R) { _T[0] = R._R[0]; _T[4] = R._R[3]; _T[8] = R._R[6]; _T[1] = R._R[1]; _T[5] = R._R[4]; _T[9] = R._R[7]; _T[2] = R._R[2]; _T[6] = R._R[5]; _T[10] = R._R[8]; _T[3] = _T[7] = _T[11] = _T[12] = _T[13] = _T[14] = 0.0; _T[15] = 1.0; }
	SE3(const Vec3 &p) { _T[0] = _T[5] = _T[10] = _T[15] = 1.0; _T[1] = _T[2] = _T[3] = _T[4] = _T[6] = _T[7] = _T[8] = _T[9] = _T[11] = 0.0; _T[12] = p._v[0]; _T[13] = p._v[1]; _T[14] = p._v[2]; }
	SE3(const double T[]) { _T[0] = T[0]; _T[1] = T[1]; _T[2] = T[2]; _T[4] = T[4]; _T[5] = T[5]; _T[6] = T[6]; _T[8] = T[8]; _T[9] = T[9]; _T[10] = T[10]; _T[12] = T[12]; _T[13] = T[13]; _T[14] = T[14]; _T[3] = _T[7] = _T[11] = 0.0; _T[15] = 1.0; }
	// operators
	double operator () (int i, int j) const { return _T[i+4*j]; }
	double &operator () (int i, int j) { return _T[i+4*j]; }
	double operator [] (int i) const { return _T[i]; }
	double &operator [] (int i) { return _T[i]; }
	SE3 &operator = (const SE3 &T) { _T[0] = T._T[0]; _T[1] = T._T[1]; _T[2] = T._T[2]; _T[4] = T._T[4]; _T[5] = T._T[5]; _T[6] = T._T[6]; _T[8] = T._T[8]; _T[9] = T._T[9]; _T[10] = T._T[10]; _T[12] = T._T[12]; _T[13] = T._T[13]; _T[14] = T._T[14]; return *this; }
	SE3 operator * (const SE3 &T) const;
	SE3 operator / (const SE3 &T) const;	// *this * Inv(T), note that A / B * C != A / ( B * C )
	SE3 operator % (const SE3 &T) const;	// Inv(*this) * T, note that A * B % C != A * ( B % C )
	Vec3 operator * (const Vec3 &p) const;
	SE3 &operator *= (const SE3 &T);
	SE3 &operator /= (const SE3 &T);
	SE3 &operator %= (const SE3 &T);
	// methods
	void SetIdentity(void) { _T[0] = _T[5] = _T[10] = _T[15] = 1.0; _T[1] = _T[2] = _T[3] = _T[4] = _T[6] = _T[7] = _T[8] = _T[9] = _T[11] = _T[12] = _T[13] = _T[14] = 0.0; }
	void SetInvOf(const SE3 &T); // *this = Inv(T)
	SE3 &SetRotation(const SO3 &R);
	SE3 &SetPosition(const Vec3 &Pos);
	SE3 &Translate(const Vec3 &Pos);
	SE3 &Rotate(const SO3 &R);
	Vec3 GetPosition(void) const { return Vec3(_T[12], _T[13], _T[14]); }
	SO3 GetRotation(void) const { return SO3(_T[0], _T[1], _T[2], _T[4], _T[5], _T[6], _T[8], _T[9], _T[10]); }
	double *GetArray(void) { return _T; }
	const double *GetArray(void) const { return _T; }
	// friend functions
	friend std::ostream &operator << (std::ostream &os, const SE3 &T);	// std::ostream standard output
	friend SE3 Inv(const SE3 &T);
	friend SE3 Exp(const se3 &s);
	friend SE3 Exp(const se3 &s, double theta);
	friend se3 Log(const SE3 &T);
	friend se3 Ad(const SE3 &T, const se3& s);									// return Ad(T,s)
	friend void Ad(double *re, const SE3 &T, const double *s);					// re = Ad(T,s)
	friend void Ad(double *re, const SE3 &T, const double *s, int num);			// re = [Ad(T,s_0), ..., Ad(T,s_(num-1))] where s_i = (s[6*i+0],...,s[6*i+5])
	friend void minus_Ad(double *re, const SE3 &T, const double *s);			// re = -Ad(T,s)
	friend void minus_Ad(double *re, const SE3 &T, const double *s, int num);	// re = -[dAd(T,s_0), ..., dAd(T,s_(num-1))] where s_i = (s[6*i+0],...,s[6*i+5])
	friend dse3 dAd(const SE3 &T, const dse3 &t);								// return dAd(T,t)
	friend void dAd(double *re, const SE3 &T, const double *t);					// re = dAd(T,t)
	friend void dAd(double *re, const SE3 &T, const double *t, int num);		// re = [dAd(T,t_0), ..., dAd(T,t_(num-1))] where t_i = (t[6*i+0],...,t[6*i+5])
	friend void minus_dAd(double *re, const SE3 &T, const double *t);			// re = -dAd(T,t)
	friend void minus_dAd(double *re, const SE3 &T, const double *t, int num);	// re = -[Ad(T,t_0), ..., Ad(T,t_(num-1))] where t_i = (t[6*i+0],...,t[6*i+5])
//	friend se3 InvAd(const SE3 &T, const se3& s);
//	friend dse3 InvdAd(const SE3 &T, const dse3 &t);
	friend se3 InvSkew(const SE3 &T)  { return se3(0.5 * (T._T[6] - T._T[9]), 0.5 * (T._T[8] - T._T[2]), 0.5 * (T._T[1] - T._T[4]), T._T[12], T._T[13], T._T[14]); }	// invskew(T - I)
	// friend class
	friend class Inertia;
	friend class AInertia;
};

class Inertia
{
public:							// Inertia = [I, [r]; -[r], m1], junggon's comment
	double _I[6], _r[3], _m;	// _I[0] = Ixx, _I[1] = Iyy, _I[2] = Izz, _I[3] = Ixy, _I[4] = Ixz, _I[5] = Iyz
public:
	// constructors
	Inertia() { _I[0] = _I[1] = _I[2] = _I[3] = _I[4] = _I[5] = _r[0] = _r[1] = _r[2] = _m = 0.0; }
	Inertia(double m) { _m = _I[0] = _I[1] = _I[2] = m; _I[3] = _I[4] = _I[5] = _r[0] = _r[1] = _r[2] = 0.0; }
	Inertia(double mass, double Ixx, double Iyy, double Izz);
	Inertia(const Inertia &J) { _I[0] = J._I[0]; _I[1] = J._I[1]; _I[2] = J._I[2]; _I[3] = J._I[3]; _I[4] = J._I[4]; _I[5] = J._I[5]; _r[0] = J._r[0]; _r[1] = J._r[1]; _r[2] = J._r[2]; _m = J._m; }
	// operator
	dse3 operator * (const se3 &acceleration) const;
	// methods
	void SetZero(void) { _m = _I[0] = _I[1] = _I[2] = _I[3] = _I[4] = _I[5] = _r[0] = _r[1] = _r[2] = 0.0; }
	void SetMass(double mass) { _m = mass; }
	double GetMass(void) { return _m; }
	Vec3 GetOffDiag() { return Vec3(_r); }
	void SetInertia(double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz) { _I[0] = Ixx; _I[1] = Iyy; _I[2] = Izz; _I[3] = Ixy; _I[4] = Ixz; _I[5] = Iyz; }
	void SetOffDiag(const double r[]) { _r[0] = r[0]; _r[1] = r[1]; _r[2] = r[2]; }
	Inertia Transform(const SE3 &T) const;
	void ToArray(double I[]) const;
	// friend function
	friend std::ostream &operator << (std::ostream &os, const Inertia &iner);	// std::ostream standard output
	friend AInertia operator + (const Inertia &A, const AInertia &B);
	friend AInertia operator - (const Inertia &A, const AInertia &B);
	friend dse3 dad(const se3 &V, const Inertia &J);
	// freind class
	friend class AInertia;
};

// acticulated inertia class
class AInertia
{
private:						// AInertia = [_A, _B; ~_B, _C], junggon's comment
	double _A[9], _B[9], _C[9];	// _A, _C use only upper triangular entries  
public:
	// constructors
	AInertia() { _A[0] = _A[3] = _A[4] = _A[6] = _A[7] = _A[8] = _B[0] = _B[1] = _B[2] = _B[3] = _B[4] = _B[5] = _B[6] = _B[7] = _B[8] = _C[0] = _C[3] = _C[4] = _C[6] = _C[7] = _C[8] = 0.0; }
	AInertia(double d) { _A[0] = _A[3] = _A[4] = _A[6] = _A[7] = _A[8] = _B[0] = _B[1] = _B[2] = _B[3] = _B[4] = _B[5] = _B[6] = _B[7] = _B[8] = _C[0] = _C[3] = _C[4] = _C[6] = _C[7] = _C[8] = d; }
	AInertia(double a0, double a3, double a4, double a6, double a7, double a8, double b0, double b1, double b2, double b3, double b4, double b5, double b6, double b7, double b8,	double c0, double c3, double c4, double c6, double c7, double c8); 
	AInertia(const AInertia &J); 
	AInertia(const Inertia &J);
	AInertia(const double *M);	// M[36] is a single array representing column-wise 6 x 6 symmetric matrix.
	// operator
	const AInertia &operator + (void) const { return *this; }
	AInertia operator - (void) const;
	dse3 operator * (const se3 &a) const;
	AInertia operator + (const AInertia &J) const;
	AInertia operator + (const Inertia &J) const;
	AInertia operator - (const AInertia &J) const;
	AInertia operator - (const Inertia &J) const;
	AInertia &operator = (const AInertia &J);
	AInertia &operator = (const Inertia &J);
	AInertia &operator += (const AInertia &J);
	AInertia &operator += (const Inertia &J);
	AInertia &operator -= (const AInertia &J);
	AInertia &operator -= (const Inertia &J);
	AInertia &operator += (const double *M); // M[36] is a single array representing column-wise 6 x 6 symmetric matrix.
	AInertia &operator -= (const double *M); 
	// methods
	void SetZero(void) { _A[0] = _A[3] = _A[4] = _A[6] = _A[7] = _A[8] = _B[0] = _B[1] = _B[2] = _B[3] = _B[4] = _B[5] = _B[6] = _B[7] = _B[8] = _C[0] = _C[3] = _C[4] = _C[6] = _C[7] = _C[8] = 0.0; }
	AInertia Transform(const SE3 &T) const;
	void SubstractAlphaSSt(double alpha, const double *s);		// (*this) -= alpha*s*~s where s = 6-dimensional vector and alpha is a scalar 
	void ToArray(double I[]) const;
	// friend function	
	friend AInertia operator + (const Inertia &A, const AInertia &B);
	friend AInertia operator - (const Inertia &A, const AInertia &B);
	friend AInertia KroneckerProduct(const dse3 &x, const dse3 &y);
	friend std::ostream &operator << (std::ostream &os, const AInertia &J);
	friend void Mult_AInertia_se3(dse3 &re, const AInertia &J, const se3 &s);				// re = J*s
	friend void Mult_AInertia_se3(double *re, const AInertia &J, const double *s);			// re = J*s
	friend void Mult_AInertia_se3(double *re, const AInertia &J, const double *s, int num);	// re = [J*s_0, ..., J*s_(num-1)] where s_i = (s[6*i+0],...,s[6*i+5])
	// friend class
	friend class Inertia;
};

#include "liegroup.inl"

Inertia RectPrism(double density, double x, double y, double z);
Inertia Beam(double density, double rad, double len);
Inertia Ball(double density, double rad);

#endif
