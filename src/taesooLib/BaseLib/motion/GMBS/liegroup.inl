//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.cpp
//						
//		version		:	v0.97
//		author		:	Jinwook Kim (zinook@plaza.snu.ac.kr)
//		last update	:	2001.9.6
//
//////////////////////////////////////////////////////////////////////////////////


#ifndef LIEGROUP_INL
#define LIEGROUP_INL
#include "liegroup.h"

#define _PI_LIE		3.14159265358979
#define _SMALL_LIE	1E-9
#define _TINY_LIE	1E-12


inline void Ad(double *re, const SE3 &T, const double *s)
{
	double tmp[3] = { T._T[0] * s[0] + T._T[4] * s[1] + T._T[8] * s[2], 
					T._T[1] * s[0] + T._T[5] * s[1] + T._T[9] * s[2], 
					T._T[2] * s[0] + T._T[6] * s[1] + T._T[10] * s[2] };
	re[0] = tmp[0];
	re[1] = tmp[1];
	re[2] = tmp[2];
	re[3] = T._T[13] * tmp[2] - T._T[14] * tmp[1] + T._T[0] * s[3] + T._T[4] * s[4] + T._T[8] * s[5];
	re[4] = T._T[14] * tmp[0] - T._T[12] * tmp[2] + T._T[1] * s[3] + T._T[5] * s[4] + T._T[9] * s[5];
	re[5] = T._T[12] * tmp[1] - T._T[13] * tmp[0] + T._T[2] * s[3] + T._T[6] * s[4] + T._T[10] * s[5];
}

inline void minus_Ad(double *re, const SE3 &T, const double *s)
{
	double tmp[3] = { T._T[0] * s[0] + T._T[4] * s[1] + T._T[8] * s[2], 
					T._T[1] * s[0] + T._T[5] * s[1] + T._T[9] * s[2], 
					T._T[2] * s[0] + T._T[6] * s[1] + T._T[10] * s[2] };
	re[0] = tmp[0]; 
	re[1] = tmp[1]; 
	re[2] = tmp[2]; 
	re[3] = T._T[13] * tmp[2] - T._T[14] * tmp[1] + T._T[0] * s[3] + T._T[4] * s[4] + T._T[8] * s[5]; 
	re[4] = T._T[14] * tmp[0] - T._T[12] * tmp[2] + T._T[1] * s[3] + T._T[5] * s[4] + T._T[9] * s[5]; 
	re[5] = T._T[12] * tmp[1] - T._T[13] * tmp[0] + T._T[2] * s[3] + T._T[6] * s[4] + T._T[10] * s[5]; 

	re[0] *= -1.0; re[1] *= -1.0; re[2] *= -1.0; re[3] *= -1.0; re[4] *= -1.0; re[5] *= -1.0;
}

inline void Ad(double *re, const SE3 &T, const double *s, int num)
{
	for (int i=0; i<num; i++) {
		Ad(&re[6*i], T, &s[6*i]);
	}
}

inline void minus_Ad(double *re, const SE3 &T, const double *s, int num)
{
	for (int i=0; i<num; i++) {
		minus_Ad(&re[6*i], T, &s[6*i]);
	}
}

inline void dAd(double *re, const SE3 &T, const double *t)
{
	double tmp[3] = { t[0] - T._T[13] * t[5] + T._T[14] * t[4], 
					t[1] - T._T[14] * t[3] + T._T[12] * t[5], 
					t[2] - T._T[12] * t[4] + T._T[13] * t[3] };

	re[0] = T._T[0] * tmp[0] + T._T[1] * tmp[1] + T._T[2] * tmp[2];
	re[1] = T._T[4] * tmp[0] + T._T[5] * tmp[1] + T._T[6] * tmp[2];
	re[2] = T._T[8] * tmp[0] + T._T[9] * tmp[1] + T._T[10] * tmp[2];
	re[3] = T._T[0] * t[3] + T._T[1] * t[4] + T._T[2] * t[5];
	re[4] = T._T[4] * t[3] + T._T[5] * t[4] + T._T[6] * t[5];
	re[5] = T._T[8] * t[3] + T._T[9] * t[4] + T._T[10] * t[5];
}

inline void minus_dAd(double *re, const SE3 &T, const double *t)
{
	double tmp[3] = { t[0] - T._T[13] * t[5] + T._T[14] * t[4], 
					t[1] - T._T[14] * t[3] + T._T[12] * t[5], 
					t[2] - T._T[12] * t[4] + T._T[13] * t[3] };

	re[0] = T._T[0] * tmp[0] + T._T[1] * tmp[1] + T._T[2] * tmp[2];
	re[1] = T._T[4] * tmp[0] + T._T[5] * tmp[1] + T._T[6] * tmp[2];
	re[2] = T._T[8] * tmp[0] + T._T[9] * tmp[1] + T._T[10] * tmp[2];
	re[3] = T._T[0] * t[3] + T._T[1] * t[4] + T._T[2] * t[5];
	re[4] = T._T[4] * t[3] + T._T[5] * t[4] + T._T[6] * t[5];
	re[5] = T._T[8] * t[3] + T._T[9] * t[4] + T._T[10] * t[5];

	re[0] *= -1.0; re[1] *= -1.0; re[2] *= -1.0; re[3] *= -1.0; re[4] *= -1.0; re[5] *= -1.0;
}

inline void dAd(double *re, const SE3 &T, const double *t, int num)
{
	for (int i=0; i<num; i++) {
		dAd(&re[6*i], T, &t[6*i]);
	}
}

inline void minus_dAd(double *re, const SE3 &T, const double *t, int num)
{
	for (int i=0; i<num; i++) {
		minus_dAd(&re[6*i], T, &t[6*i]);
	}
}

// *this = T * s * Inv(T)
inline void se3::Ad(const SE3 &T, const se3 &s)
{
	double tmp[3] = { T[0] * s[0] + T[4] * s[1] + T[8] * s[2], 
					T[1] * s[0] + T[5] * s[1] + T[9] * s[2], 
					T[2] * s[0] + T[6] * s[1] + T[10] * s[2] };
	_w[0] = tmp[0];
	_w[1] = tmp[1];
	_w[2] = tmp[2];
	_w[3] = T[13] * tmp[2] - T[14] * tmp[1] + T[0] * s[3] + T[4] * s[4] + T[8] * s[5];
	_w[4] = T[14] * tmp[0] - T[12] * tmp[2] + T[1] * s[3] + T[5] * s[4] + T[9] * s[5];
	_w[5] = T[12] * tmp[1] - T[13] * tmp[0] + T[2] * s[3] + T[6] * s[4] + T[10] * s[5];
}

inline void se3::ad(const se3 &s1, const se3 &s2)
{
	_w[0] = s1[1] * s2[2] - s1[2] * s2[1];
	_w[1] = s1[2] * s2[0] - s1[0] * s2[2];
	_w[2] = s1[0] * s2[1] - s1[1] * s2[0];
	_w[3] = s1[1] * s2[5] - s1[2] * s2[4] - s2[1] * s1[5] + s2[2] * s1[4];
	_w[4] = s1[2] * s2[3] - s1[0] * s2[5] - s2[2] * s1[3] + s2[0] * s1[5];
	_w[5] = s1[0] * s2[4] - s1[1] * s2[3] - s2[0] * s1[4] + s2[1] * s1[3];
}

inline SE3 Exp(const se3 &S)
{
	double theta = sqrt(S._w[0] * S._w[0] + S._w[1] * S._w[1] + S._w[2] * S._w[2]), itheta = 1.0 / theta;
	double s[6] = { itheta * S._w[0], itheta * S._w[1], itheta * S._w[2], itheta * S._w[3], itheta * S._w[4], itheta * S._w[5] };
	double st = sin(theta), ct = cos(theta), vt = 1.0 - ct, ut = (theta - st) * (s[0] * s[3] + s[1] * s[4] + s[2] * s[5]), t0 = s[2] * st, t1 = s[1] * st, t2 = s[0] * st;

	if ( fabs(theta) < _SMALL_LIE ) return SE3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, S._w[3], S._w[4], S._w[5]);
	
	return SE3( s[0] * s[0] * vt + ct,
				s[0] * s[1] * vt + t0,
				s[0] * s[2] * vt - t1,
				s[0] * s[1] * vt - t0,
				s[1] * s[1] * vt + ct,
				s[1] * s[2] * vt + t2,
				s[0] * s[2] * vt + t1,
				s[1] * s[2] * vt - t2,
				s[2] * s[2] * vt + ct,
				st * s[3] + ut * s[0] + vt * (s[1] * s[5] - s[2] * s[4]),
				st * s[4] + ut * s[1] + vt * (s[2] * s[3] - s[0] * s[5]),
				st * s[5] + ut * s[2] + vt * (s[0] * s[4] - s[1] * s[3]) );
}

inline SE3 Exp(const se3 &s, double theta)
{
	double mag = s._w[0] * s._w[0] + s._w[1] * s._w[1] + s._w[2] * s._w[2];

	if ( fabs(theta) < _SMALL_LIE || mag < _TINY_LIE ) return SE3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, theta * s._w[3], theta * s._w[4], theta * s._w[5]);
	else if ( fabs(mag - 1.0) > _TINY_LIE ) return Exp(theta * s);

	double st = sin(theta), ct = cos(theta), vt = 1.0 - ct, ut = (theta - st) * (s._w[0] * s._w[3] + s._w[1] * s._w[4] + s._w[2] * s._w[5]), t0 = s._w[2] * st, t1 = s._w[1] * st, t2 = s._w[0] * st;

	return SE3( s._w[0] * s._w[0] * vt + ct,
				s._w[0] * s._w[1] * vt + t0,
				s._w[0] * s._w[2] * vt - t1,
				s._w[0] * s._w[1] * vt - t0,
				s._w[1] * s._w[1] * vt + ct,
				s._w[1] * s._w[2] * vt + t2,
				s._w[0] * s._w[2] * vt + t1,
				s._w[1] * s._w[2] * vt - t2,
				s._w[2] * s._w[2] * vt + ct,
				st * s._w[3] + ut * s._w[0] + vt * (s._w[1] * s._w[5] - s._w[2] * s._w[4]),
				st * s._w[4] + ut * s._w[1] + vt * (s._w[2] * s._w[3] - s._w[0] * s._w[5]),
				st * s._w[5] + ut * s._w[2] + vt * (s._w[0] * s._w[4] - s._w[1] * s._w[3]) );
}

inline se3 Log(const SE3 &T)
{
	double d = 0.5 * (T._T[0] + T._T[5] + T._T[10] - 1.0);
	if ( d > 1.0 ) { d = 1.0; }
	if ( d < -1.0 ) { d = -1.0; }
	double theta = acos(d);
	if ( fabs(theta) < _SMALL_LIE ) return se3(0.0, 0.0, 0.0, T._T[12], T._T[13], T._T[14]);
	double cof = theta / (2.0 * sin(theta));
	double x = cof * (T._T[6] - T._T[9]), y = cof * (T._T[8] - T._T[2]), z = cof * (T._T[1] - T._T[4]);
	theta = sqrt(x * x + y * y + z * z);
	cof = (2.0 * sin(theta) - theta * (1.0 + cos(theta))) / (2.0 * theta * theta * sin(theta));
	return se3( x, y, z,
				(1.0 - cof * (y * y + z * z)) * T._T[12] + (0.5 * z + cof * x * y) * T._T[13] + (cof * x * z - 0.5 * y) * T._T[14],
				(1.0 - cof * (x * x + z * z)) * T._T[13] + (0.5 * x + cof * z * y) * T._T[14] + (cof * x * y - 0.5 * z) * T._T[12],
				(1.0 - cof * (y * y + x * x)) * T._T[14] + (0.5 * y + cof * x * z) * T._T[12] + (cof * y * z - 0.5 * x) * T._T[13]);
}

// re = T * s * Inv(T)
inline se3 Ad(const SE3 &T, const se3 &s)
{
	double tmp[3] = { T._T[0] * s._w[0] + T._T[4] * s._w[1] + T._T[8] * s._w[2], 
					T._T[1] * s._w[0] + T._T[5] * s._w[1] + T._T[9] * s._w[2], 
					T._T[2] * s._w[0] + T._T[6] * s._w[1] + T._T[10] * s._w[2] };

	return se3(	tmp[0], tmp[1], tmp[2],
				T._T[13] * tmp[2] - T._T[14] * tmp[1] + T._T[0] * s._w[3] + T._T[4] * s._w[4] + T._T[8] * s._w[5],
				T._T[14] * tmp[0] - T._T[12] * tmp[2] + T._T[1] * s._w[3] + T._T[5] * s._w[4] + T._T[9] * s._w[5],
				T._T[12] * tmp[1] - T._T[13] * tmp[0] + T._T[2] * s._w[3] + T._T[6] * s._w[4] + T._T[10] * s._w[5]);
}

//// re = Inv(T) * s * T
//inline se3 InvAd(const SE3 &T, const se3 &s)
//{
//	double tmp[3] = { s._w[3] + s._w[1] * T._T[14] - s._w[2] * T._T[13], 
//					s._w[4] + s._w[2] * T._T[12] - s._w[0] * T._T[14], 
//					s._w[5] + s._w[0] * T._T[13] - s._w[1] * T._T[12] };
//
//	return se3(	T._T[0] * s._w[0] + T._T[1] * s._w[1] + T._T[2] * s._w[2],
//				T._T[4] * s._w[0] + T._T[5] * s._w[1] + T._T[6] * s._w[2],
//				T._T[8] * s._w[0] + T._T[9] * s._w[1] + T._T[10] * s._w[2],
//				T._T[0] * tmp[0] + T._T[1] * tmp[1] + T._T[2] * tmp[2],
//				T._T[4] * tmp[0] + T._T[5] * tmp[1] + T._T[6] * tmp[2],
//				T._T[8] * tmp[0] + T._T[9] * tmp[1] + T._T[10] * tmp[2]	);
//}

inline se3 Ad(const Vec3 &p, const se3 &s)
{
	return se3(	s._w[0], 
				s._w[1], 
				s._w[2], 
				p._v[1] * s._w[2] - p._v[2] * s._w[1] + s._w[3], 
				p._v[2] * s._w[0] - p._v[0] * s._w[2] + s._w[4], 
				p._v[0] * s._w[1] - p._v[1] * s._w[0] + s._w[5]	);
}

inline se3 ad(const se3 &s1, const se3 &s2)
{
	return se3(	s1._w[1] * s2._w[2] - s1._w[2] * s2._w[1],
				s1._w[2] * s2._w[0] - s1._w[0] * s2._w[2],
				s1._w[0] * s2._w[1] - s1._w[1] * s2._w[0],
				s1._w[1] * s2._w[5] - s1._w[2] * s2._w[4] - s2._w[1] * s1._w[5] + s2._w[2] * s1._w[4],
				s1._w[2] * s2._w[3] - s1._w[0] * s2._w[5] - s2._w[2] * s1._w[3] + s2._w[0] * s1._w[5],
				s1._w[0] * s2._w[4] - s1._w[1] * s2._w[3] - s2._w[0] * s1._w[4] + s2._w[1] * s1._w[3] );
}

inline dse3 dAd(const SE3 &T, const dse3 &t)
{
	double tmp[3] = { t._m[0] - T._T[13] * t._m[5] + T._T[14] * t._m[4], 
					t._m[1] - T._T[14] * t._m[3] + T._T[12] * t._m[5], 
					t._m[2] - T._T[12] * t._m[4] + T._T[13] * t._m[3] };

	return dse3(T._T[0] * tmp[0] + T._T[1] * tmp[1] + T._T[2] * tmp[2],
				T._T[4] * tmp[0] + T._T[5] * tmp[1] + T._T[6] * tmp[2],
				T._T[8] * tmp[0] + T._T[9] * tmp[1] + T._T[10] * tmp[2],
				T._T[0] * t._m[3] + T._T[1] * t._m[4] + T._T[2] * t._m[5],
				T._T[4] * t._m[3] + T._T[5] * t._m[4] + T._T[6] * t._m[5],
				T._T[8] * t._m[3] + T._T[9] * t._m[4] + T._T[10] * t._m[5]);
}

//inline dse3 InvdAd(const SE3 &T, const dse3 &t)
//{
//	double tmp[3] = { T._T[0] * t._m[3] + T._T[4] * t._m[4] + T._T[8] * t._m[5], 
//					T._T[1] * t._m[3] + T._T[5] * t._m[4] + T._T[9] * t._m[5], 
//					T._T[2] * t._m[3] + T._T[6] * t._m[4] + T._T[10] * t._m[5] };
//
//	return dse3(T._T[13] * tmp[2] - T._T[14] * tmp[1] + T._T[0] * t._m[0] + T._T[4] * t._m[1] + T._T[8] * t._m[2],
//				T._T[14] * tmp[0] - T._T[12] * tmp[2] + T._T[1] * t._m[0] + T._T[5] * t._m[1] + T._T[9] * t._m[2],
//				T._T[12] * tmp[1] - T._T[13] * tmp[0] + T._T[2] * t._m[0] + T._T[6] * t._m[1] + T._T[10] * t._m[2],
//				tmp[0], tmp[1], tmp[2]);
//}

inline dse3 dad(const se3 &s, const dse3 &t)
{
	return dse3(t._m[1] * s._w[2] - t._m[2] * s._w[1] + t._m[4] * s._w[5] - t._m[5] * s._w[4],
				t._m[2] * s._w[0] - t._m[0] * s._w[2] + t._m[5] * s._w[3] - t._m[3] * s._w[5],
				t._m[0] * s._w[1] - t._m[1] * s._w[0] + t._m[3] * s._w[4] - t._m[4] * s._w[3],
				t._m[4] * s._w[2] - t._m[5] * s._w[1],
				t._m[5] * s._w[0] - t._m[3] * s._w[2],
				t._m[3] * s._w[1] - t._m[4] * s._w[0]	);
}

// slightly efficient computation of dad(V, J * V)
inline dse3 dad(const se3 &V, const Inertia &J)
{
	double ww = V._w[0] * V._w[0] + V._w[1] * V._w[1] + V._w[2] * V._w[2];
	double wr = V._w[0] * J._r[0] + V._w[1] * J._r[1] + V._w[2] * J._r[2];
	double vr = V._w[3] * J._r[0] + V._w[4] * J._r[1] + V._w[5] * J._r[2];

	return dse3((J._I[3] * V._w[0] + J._I[1] * V._w[1] + J._I[5] * V._w[2]) * V._w[2] - (J._I[4] * V._w[0] + J._I[5] * V._w[1] + J._I[2] * V._w[2]) * V._w[1] - vr * V._w[0] + wr * V._w[3],
				(J._I[4] * V._w[0] + J._I[5] * V._w[1] + J._I[2] * V._w[2]) * V._w[0] - (J._I[0] * V._w[0] + J._I[3] * V._w[1] + J._I[4] * V._w[2]) * V._w[2] - vr * V._w[1] + wr * V._w[4],
				(J._I[0] * V._w[0] + J._I[3] * V._w[1] + J._I[4] * V._w[2]) * V._w[1] - (J._I[3] * V._w[0] + J._I[1] * V._w[1] + J._I[5] * V._w[2]) * V._w[0] - vr * V._w[2] + wr * V._w[5],
				ww * J._r[0] - wr * V._w[0] + J._m * (V._w[4] * V._w[2] - V._w[5] * V._w[1]),
				ww * J._r[1] - wr * V._w[1] + J._m * (V._w[5] * V._w[0] - V._w[3] * V._w[2]),
				ww * J._r[2] - wr * V._w[2] + J._m * (V._w[3] * V._w[1] - V._w[4] * V._w[0]));
}

inline SO3 SO3::operator * (const SO3 &R) const
{
	return SO3(	_R[0] * R._R[0] + _R[3] * R._R[1] + _R[6] * R._R[2], _R[1] * R._R[0] + _R[4] * R._R[1] + _R[7] * R._R[2], _R[2] * R._R[0] + _R[5] * R._R[1] + _R[8] * R._R[2],
				_R[0] * R._R[3] + _R[3] * R._R[4] + _R[6] * R._R[5], _R[1] * R._R[3] + _R[4] * R._R[4] + _R[7] * R._R[5], _R[2] * R._R[3] + _R[5] * R._R[4] + _R[8] * R._R[5],
				_R[0] * R._R[6] + _R[3] * R._R[7] + _R[6] * R._R[8], _R[1] * R._R[6] + _R[4] * R._R[7] + _R[7] * R._R[8], _R[2] * R._R[6] + _R[5] * R._R[7] + _R[8] * R._R[8]);
}

inline SO3 &SO3::operator *= (const SO3 &R)
{
	*this = *this * R;
	return *this;
}

inline SO3 Exp(double w0, double w1, double w2)
{
	double theta = sqrt(w0 * w0 + w1 * w1 + w2 * w2);
	if ( fabs(theta) < _TINY_LIE ) return SO3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
	
	w0 /= theta; w1 /= theta; w2 /= theta;
	double st = sin(theta), ct = cos(theta), vt = 1.0 - ct, t0 = w2 * st, t1 = w1 * st, t2 = w0 * st;

	return SO3( w0 * w0 * vt + ct, w0 * w1 * vt + t0, w0 * w2 * vt - t1,
				w0 * w1 * vt - t0, w1 * w1 * vt + ct, w1 * w2 * vt + t2,
				w0 * w2 * vt + t1, w1 * w2 * vt - t2, w2 * w2 * vt + ct );
}

inline SO3 Exp(const Vec3 &w)
{
	return Exp(w._v[0], w._v[1], w._v[2]);
}

inline Vec3 Log(const SO3 &R)
{
	double d = 0.5 * (R._R[0] + R._R[4] + R._R[8] - 1.0); 
	if ( d > 1.0 ) { d = 1.0; }
	if ( d < -1.0 ) { d = -1.0; }
	double theta = acos(d), cof = theta / (2.0 * sin(theta));

	if ( fabs(theta) < _SMALL_LIE ) return Vec3(0.0, 0.0, 0.0);

	return Vec3(cof * (R._R[5] - R._R[7]), cof * (R._R[6] - R._R[2]), cof * (R._R[1] - R._R[3]));
}

inline SO3 RotX(double theta)
{
	double c = cos(theta), s = sin(theta);
	return SO3(1, 0, 0, 0, c, s, 0, -s, c);
}

inline SO3 RotY(double theta)
{
	double c = cos(theta), s = sin(theta);
	return SO3(c, 0, -s, 0, 1, 0, s, 0, c);
}

inline SO3 RotZ(double theta)
{
	double c = cos(theta), s = sin(theta);
	return SO3(c, s, 0, -s, c, 0, 0, 0, 1);
}

inline SO3 EulerZYX(const Vec3 &x)
{
	double c0 = cos(x._v[0]), s0 = sin(x._v[0]), c1 = cos(x._v[1]), s1 = sin(x._v[1]), c2 = cos(x._v[2]), s2 = sin(x._v[2]);
	return SO3(c0 * c1, s0 * c1, -s1, c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2, c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2);
}

inline Vec3 iEulerZYX(const SO3 &R)
{
	return Vec3(atan2(R._R[1], R._R[0]), atan2(-R._R[2], sqrt(R._R[0] * R._R[0] + R._R[1] * R._R[1])), atan2(R._R[5], R._R[8]));
}

inline SO3 EulerZYZ(const Vec3 &x)
{
	double ca = cos(x._v[0]), sa = sin(x._v[0]), cb = cos(x._v[1]), sb = sin(x._v[1]), cg = cos(x._v[2]), sg = sin(x._v[2]);
	return SO3(ca * cb * cg - sa * sg, sa * cb * cg + ca * sg, -sb*cg,	-ca * cb * sg - sa * cg, ca * cg - sa * cb * sg, sb * sg, ca * sb, sa * sb, cb);
}
	
inline Vec3 iEulerZYZ(const SO3 &R)
{
	return Vec3(atan2(R._R[7], R._R[6]), atan2(sqrt(R._R[2] * R._R[2] + R._R[5] * R._R[5]), R._R[8]), atan2(R._R[5], -R._R[2]));
}

inline SO3 EulerZXY(const Vec3 &x)
{
	double c0 = cos(x._v[0]), s0 = sin(x._v[0]), c1 = cos(x._v[1]), s1 = sin(x._v[1]), c2 = cos(x._v[2]), s2 = sin(x._v[2]);
	return SO3(c0*c2 - s0*s1*s2, c2*s0 + c0*s1*s2, -c1*s2, -c1*s0, c0*c1, s1, c0*s2 + c2*s0*s1, s0*s2 - c0*c2*s1, c1*c2);
}

inline Vec3 iEulerZXY(const SO3 &R)
{
	return Vec3(atan2(-R._R[3], R._R[4]), atan2(R._R[5], sqrt(R._R[3]*R._R[3]+R._R[4]*R._R[4])), atan2(-R._R[2], R._R[8]));
}

inline SO3 EulerXYZ(const Vec3 &x)
{
	double c0 = cos(x._v[0]), s0 = sin(x._v[0]), c1 = cos(x._v[1]), s1 = sin(x._v[1]), c2 = cos(x._v[2]), s2 = sin(x._v[2]);
	return SO3(c1*c2, c0*s2 + c2*s0*s1, s0*s2 - c0*c2*s1, -c1*s2, c0*c2 - s0*s1*s2, c2*s0 + c0*s1*s2, s1, -c1*s0, c0*c1);
}

inline Vec3 iEulerXYZ(const SO3 &R)
{
	return Vec3(atan2(-R._R[7], R._R[8]), atan2(R._R[6], sqrt(R._R[7]*R._R[7]+R._R[8]*R._R[8])), atan2(-R._R[3], R._R[0]));
}

inline SO3 Quat(double *quat)
{
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm

	SO3 re;
	double sqw = quat[0]*quat[0];
	double sqx = quat[1]*quat[1];
	double sqy = quat[2]*quat[2];
	double sqz = quat[3]*quat[3];

	// invs (inverse square length) is only required if quaternion is not already normalized
	double invs = 1 / (sqx + sqy + sqz + sqw);
	re._R[0] = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
	re._R[4] = (-sqx + sqy - sqz + sqw)*invs ;
	re._R[8] = (-sqx - sqy + sqz + sqw)*invs ;

	double tmp1 = quat[1]*quat[2];
	double tmp2 = quat[3]*quat[0];
	re._R[1] = 2.0 * (tmp1 + tmp2)*invs ;
	re._R[3] = 2.0 * (tmp1 - tmp2)*invs ;

	tmp1 = quat[1]*quat[3];
	tmp2 = quat[2]*quat[0];
	re._R[2] = 2.0 * (tmp1 - tmp2)*invs ;
	re._R[6] = 2.0 * (tmp1 + tmp2)*invs ;
	tmp1 = quat[2]*quat[3];
	tmp2 = quat[1]*quat[0];
	re._R[5] = 2.0 * (tmp1 + tmp2)*invs ;
	re._R[7] = 2.0 * (tmp1 - tmp2)*invs ; 

	return re;
}

inline void iQuat(const SO3 &R, double *quat)
{
	// convert rotation matrix (R) to unit quaternion (quat)
	quat[0] = 0.5 * sqrt(1.+R._R[0]+R._R[4]+R._R[8]);
	quat[1] = (R._R[5]-R._R[7])/(4*quat[0]);
	quat[2] = (R._R[6]-R._R[2])/(4*quat[0]);
	quat[3] = (R._R[1]-R._R[3])/(4*quat[0]);
}

inline bool isSO3(double *R, double eps)
{
	if (fabs(R[0]*R[3] + R[3]*R[4] + R[6]*R[7]) > eps) return false;
	if (fabs(R[0]*R[2] + R[3]*R[5] + R[6]*R[8]) > eps) return false;
	if (fabs(R[1]*R[2] + R[4]*R[5] + R[7]*R[8]) > eps) return false;
	if (fabs(R[0]*R[0] + R[3]*R[3] + R[6]*R[6] - 1) > eps) return false;
	if (fabs(R[1]*R[1] + R[4]*R[4] + R[7]*R[7] - 1) > eps) return false;
	if (fabs(R[2]*R[2] + R[5]*R[5] + R[8]*R[8] - 1) > eps) return false;
	if (fabs(R[0]*R[4]*R[8] + R[3]*R[7]*R[2] + R[6]*R[1]*R[5] - R[0]*R[7]*R[5] - R[3]*R[1]*R[8] - R[6]*R[4]*R[2] -1) > eps) return false;
	return true;
}

inline SE3 &SE3::operator *= (const SE3 &T)
{
	double x0, x1, x2;
	
	_T[12] += _T[0] * T._T[12] + _T[4] * T._T[13] + _T[8] * T._T[14];
	_T[13] += _T[1] * T._T[12] + _T[5] * T._T[13] + _T[9] * T._T[14];
	_T[14] += _T[2] * T._T[12] + _T[6] * T._T[13] + _T[10] * T._T[14];
	
	x0 = _T[0] * T._T[0] + _T[4] * T._T[1] + _T[8] * T._T[2];
	x1 = _T[0] * T._T[4] + _T[4] * T._T[5] + _T[8] * T._T[6];
	x2 = _T[0] * T._T[8] + _T[4] * T._T[9] + _T[8] * T._T[10];
	_T[0] = x0; _T[4] = x1; _T[8] = x2;
	x0 = _T[1] * T._T[0] + _T[5] * T._T[1] + _T[9] * T._T[2];
	x1 = _T[1] * T._T[4] + _T[5] * T._T[5] + _T[9] * T._T[6];
	x2 = _T[1] * T._T[8] + _T[5] * T._T[9] + _T[9] * T._T[10];
	_T[1] = x0; _T[5] =x1; _T[9] = x2;
	x0 = _T[2] * T._T[0] + _T[6] * T._T[1] + _T[10] * T._T[2];
	x1 = _T[2] * T._T[4] + _T[6] * T._T[5] + _T[10] * T._T[6];
	x2 = _T[2] * T._T[8] + _T[6] * T._T[9] + _T[10] * T._T[10];
	_T[2] = x0; _T[6] = x1; _T[10] = x2;
	
	return  *this;
}

inline SE3 &SE3::operator /= (const SE3 &T)
{
	double tmp[9] = {	_T[0] * T._T[0] + _T[4] * T._T[4] + _T[8] * T._T[8],
					_T[1] * T._T[0] + _T[5] * T._T[4] + _T[9] * T._T[8],
					_T[2] * T._T[0] + _T[6] * T._T[4] + _T[10] * T._T[8],
					_T[0] * T._T[1] + _T[4] * T._T[5] + _T[8] * T._T[9],
					_T[1] * T._T[1] + _T[5] * T._T[5] + _T[9] * T._T[9],
					_T[2] * T._T[1] + _T[6] * T._T[5] + _T[10] * T._T[9],
					_T[0] * T._T[2] + _T[4] * T._T[6] + _T[8] * T._T[10],
					_T[1] * T._T[2] + _T[5] * T._T[6] + _T[9] * T._T[10],
					_T[2] * T._T[2] + _T[6] * T._T[6] + _T[10] * T._T[10] };
		
	_T[0] = tmp[0]; _T[1] = tmp[1]; _T[2] = tmp[2];
	_T[4] = tmp[3]; _T[5] = tmp[4]; _T[6] = tmp[5];
	_T[8] = tmp[6]; _T[9] = tmp[7]; _T[10] = tmp[8], 
	_T[12] -= tmp[0] * T._T[12] + tmp[3] * T._T[13] + tmp[6] * T._T[14];
	_T[13] -= tmp[1] * T._T[12] + tmp[4] * T._T[13] + tmp[7] * T._T[14];
	_T[14] -= tmp[2] * T._T[12] + tmp[5] * T._T[13] + tmp[8] * T._T[14];

	return *this;
}

inline SE3 &SE3::operator %= (const SE3 &T)
{
	double tmp[12] = { _T[0], _T[1], _T[2], _T[4], _T[5], _T[6], _T[8], _T[9], _T[10], T._T[12] - _T[12], T._T[13] - _T[13], T._T[14] - _T[14] };
	
	_T[0] = tmp[0] * T._T[0] + tmp[1] * T._T[1] + tmp[2] * T._T[2];
	_T[1] = tmp[3] * T._T[0] + tmp[4] * T._T[1] + tmp[5] * T._T[2];
	_T[2] = tmp[6] * T._T[0] + tmp[7] * T._T[1] + tmp[8] * T._T[2];
	_T[4] = tmp[0] * T._T[4] + tmp[1] * T._T[5] + tmp[2] * T._T[6];
	_T[5] = tmp[3] * T._T[4] + tmp[4] * T._T[5] + tmp[5] * T._T[6];
	_T[6] = tmp[6] * T._T[4] + tmp[7] * T._T[5] + tmp[8] * T._T[6];
	_T[8] = tmp[0] * T._T[8] + tmp[1] * T._T[9] + tmp[2] * T._T[10];
	_T[9] = tmp[3] * T._T[8] + tmp[4] * T._T[9] + tmp[5] * T._T[10];
	_T[10] = tmp[6] * T._T[8] + tmp[7] * T._T[9] + tmp[8] * T._T[10];
	_T[12] = tmp[0] * tmp[9] + tmp[1] * tmp[10] + tmp[2] * tmp[11];
	_T[13] = tmp[3] * tmp[9] + tmp[4] * tmp[10] + tmp[5] * tmp[11];
	_T[14] = tmp[6] * tmp[9] + tmp[7] * tmp[10] + tmp[8] * tmp[11];

	return *this;
}

inline SE3 SE3::operator * (const SE3 &T) const
{
	return SE3(	_T[0] * T._T[0] + _T[4] * T._T[1] + _T[8] * T._T[2],
				_T[1] * T._T[0] + _T[5] * T._T[1] + _T[9] * T._T[2],
				_T[2] * T._T[0] + _T[6] * T._T[1] + _T[10] * T._T[2],
				_T[0] * T._T[4] + _T[4] * T._T[5] + _T[8] * T._T[6],
				_T[1] * T._T[4] + _T[5] * T._T[5] + _T[9] * T._T[6],
				_T[2] * T._T[4] + _T[6] * T._T[5] + _T[10] * T._T[6],
				_T[0] * T._T[8] + _T[4] * T._T[9] + _T[8] * T._T[10],
				_T[1] * T._T[8] + _T[5] * T._T[9] + _T[9] * T._T[10],
				_T[2] * T._T[8] + _T[6] * T._T[9] + _T[10] * T._T[10],
				_T[12] + _T[0] * T._T[12] + _T[4] * T._T[13] + _T[8] * T._T[14],
				_T[13] + _T[1] * T._T[12] + _T[5] * T._T[13] + _T[9] * T._T[14],
				_T[14] + _T[2] * T._T[12] + _T[6] * T._T[13] + _T[10] * T._T[14] );
}

inline Vec3 SE3::operator * (const Vec3 &p) const
{
	return Vec3(_T[12] + _T[0] * p._v[0] + _T[4] * p._v[1] + _T[8] * p._v[2],
				_T[13] + _T[1] * p._v[0] + _T[5] * p._v[1] + _T[9] * p._v[2],
				_T[14] + _T[2] * p._v[0] + _T[6] * p._v[1] + _T[10] * p._v[2] );
}

inline SE3 SE3::operator / (const SE3 &T) const
{
	double tmp[9] = {	_T[0] * T._T[0] + _T[4] * T._T[4] + _T[8] * T._T[8],
					_T[1] * T._T[0] + _T[5] * T._T[4] + _T[9] * T._T[8],
					_T[2] * T._T[0] + _T[6] * T._T[4] + _T[10] * T._T[8],
					_T[0] * T._T[1] + _T[4] * T._T[5] + _T[8] * T._T[9],
					_T[1] * T._T[1] + _T[5] * T._T[5] + _T[9] * T._T[9],
					_T[2] * T._T[1] + _T[6] * T._T[5] + _T[10] * T._T[9],
					_T[0] * T._T[2] + _T[4] * T._T[6] + _T[8] * T._T[10],
					_T[1] * T._T[2] + _T[5] * T._T[6] + _T[9] * T._T[10],
					_T[2] * T._T[2] + _T[6] * T._T[6] + _T[10] * T._T[10] };
		
	return SE3(	tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], 
				_T[12] - tmp[0] * T._T[12] - tmp[3] * T._T[13] - tmp[6] * T._T[14],
				_T[13] - tmp[1] * T._T[12] - tmp[4] * T._T[13] - tmp[7] * T._T[14],
				_T[14] - tmp[2] * T._T[12] - tmp[5] * T._T[13] - tmp[8] * T._T[14] );
}

inline SE3 SE3::operator % (const SE3 &T) const
{
	return SE3(	_T[0] * T._T[0] + _T[1] * T._T[1] + _T[2] * T._T[2],
				_T[4] * T._T[0] + _T[5] * T._T[1] + _T[6] * T._T[2],
				_T[8] * T._T[0] + _T[9] * T._T[1] + _T[10] * T._T[2],
				_T[0] * T._T[4] + _T[1] * T._T[5] + _T[2] * T._T[6],
				_T[4] * T._T[4] + _T[5] * T._T[5] + _T[6] * T._T[6],
				_T[8] * T._T[4] + _T[9] * T._T[5] + _T[10] * T._T[6],
				_T[0] * T._T[8] + _T[1] * T._T[9] + _T[2] * T._T[10],
				_T[4] * T._T[8] + _T[5] * T._T[9] + _T[6] * T._T[10],
				_T[8] * T._T[8] + _T[9] * T._T[9] + _T[10] * T._T[10],
				_T[0] * (T._T[12] - _T[12]) + _T[1] * (T._T[13] - _T[13]) + _T[2] * (T._T[14] - _T[14]),
				_T[4] * (T._T[12] - _T[12]) + _T[5] * (T._T[13] - _T[13]) + _T[6] * (T._T[14] - _T[14]),
				_T[8] * (T._T[12] - _T[12]) + _T[9] * (T._T[13] - _T[13]) + _T[10] * (T._T[14] - _T[14]) );
}

inline void SE3::SetInvOf(const SE3 &T)
{
	_T[0] = T._T[0];	_T[4] = T._T[1];	_T[8] = T._T[2];
	_T[1] = T._T[4];	_T[5] = T._T[5];	_T[9] = T._T[6];
	_T[2] = T._T[8];	_T[6] = T._T[9];	_T[10] = T._T[10];

	_T[12] = -T._T[0] * T._T[12] - T._T[1] * T._T[13] - T._T[2] * T._T[14];
	_T[13] = -T._T[4] * T._T[12] - T._T[5] * T._T[13] - T._T[6] * T._T[14];
	_T[14] = -T._T[8] * T._T[12] - T._T[9] * T._T[13] - T._T[10] * T._T[14];
}

inline SE3 &SE3::SetRotation(const SO3 &R)
{
	_T[0] = R._R[0]; _T[4] = R._R[3]; _T[8] = R._R[6];
	_T[1] = R._R[1]; _T[5] = R._R[4]; _T[9] = R._R[7];
	_T[2] = R._R[2]; _T[6] = R._R[5]; _T[10] = R._R[8];
	return  *this;
}

inline SE3 &SE3::SetPosition(const Vec3 &Pos)
{
	_T[12] = Pos._v[0]; _T[13] = Pos._v[1]; _T[14] = Pos._v[2];
	return  *this;
}

inline SE3 &SE3::Translate(const Vec3 &Pos)
{
	_T[12] += Pos._v[0]; _T[13] += Pos._v[1]; _T[14] += Pos._v[2];
	return  *this;
}

inline SE3 &SE3::Rotate(const SO3 &R)
{
	double r1, r2, r3;
	r1 = _T[0] * R._R[0] + _T[4] * R._R[1] + _T[8] * R._R[2];
	r2 = _T[0] * R._R[3] + _T[4] * R._R[4] + _T[8] * R._R[5];
	r3 = _T[0] * R._R[6] + _T[4] * R._R[7] + _T[8] * R._R[8];
	_T[0] = r1; _T[4] = r2; _T[8] = r3;
	r1 = _T[1] * R._R[0] + _T[5] * R._R[1] + _T[9] * R._R[2];
	r2 = _T[1] * R._R[3] + _T[5] * R._R[4] + _T[9] * R._R[5];
	r3 = _T[1] * R._R[6] + _T[5] * R._R[7] + _T[9] * R._R[8];
	_T[1] = r1; _T[5] = r2; _T[9] = r3;
	r1 = _T[2] * R._R[0] + _T[6] * R._R[1] + _T[10] * R._R[2];
	r2 = _T[2] * R._R[3] + _T[6] * R._R[4] + _T[10] * R._R[5];
	r3 = _T[2] * R._R[6] + _T[6] * R._R[7] + _T[10] * R._R[8];
	_T[2] = r1; _T[6] = r2; _T[10] = r3;
	return  *this;
}

inline SE3 Inv(const SE3 &T)
{
	return SE3(	T._T[0], T._T[4], T._T[8], T._T[1], T._T[5], T._T[9], T._T[2], T._T[6], T._T[10],
				-T._T[0] * T._T[12] - T._T[1] * T._T[13] - T._T[2] * T._T[14],
				-T._T[4] * T._T[12] - T._T[5] * T._T[13] - T._T[6] * T._T[14],
				-T._T[8] * T._T[12] - T._T[9] * T._T[13] - T._T[10] * T._T[14]);
}


inline Inertia::Inertia(double mass, double Ixx, double Iyy, double Izz)
{
	_m = mass;
	_I[0] = Ixx; _I[1] = Iyy; _I[2] = Izz;
	_I[3] = _I[4] = _I[5] = _r[0] = _r[1] = _r[2] = 0.0;
}
	
inline Inertia Inertia::Transform(const SE3 &T) const
{
	Inertia J(_m);
	
	J._I[0] = _I[0] + _m * T._T[14] * T._T[14] + _m * T._T[13] * T._T[13] - 2.0 * _r[2] * T._T[14] - 2.0 * _r[1] * T._T[13];
	J._I[3] = _I[3] + T._T[13] * _r[0] + T._T[12] * _r[1] - _m * T._T[13] * T._T[12];
	J._I[4] = _I[4] + T._T[14] * _r[0] + T._T[12] * _r[2] - _m * T._T[14] * T._T[12];
	J._I[1] = _I[1] + _m * T._T[14] * T._T[14] + _m * T._T[12] * T._T[12] - 2.0 * _r[2] * T._T[14] - 2.0 * _r[0] * T._T[12];
	J._I[5] = _I[5] + T._T[14] * _r[1] + T._T[13] * _r[2] - _m * T._T[14] * T._T[13];
	J._I[2] = _I[2] + _m * T._T[13] * T._T[13] + _m * T._T[12] * T._T[12] - 2.0 * _r[1] * T._T[13] - 2.0 * _r[0] * T._T[12];

	// _tmp = Transpose of Rotation Part of T * J._I
	double tmp[9] = {	T._T[0] * J._I[0] + T._T[1] * J._I[3] + T._T[2] * J._I[4],
					T._T[4] * J._I[0] + T._T[5] * J._I[3] + T._T[6] * J._I[4],
					T._T[8] * J._I[0] + T._T[9] * J._I[3] + T._T[10] * J._I[4],
					T._T[0] * J._I[3] + T._T[1] * J._I[1] + T._T[2] * J._I[5],
					T._T[4] * J._I[3] + T._T[5] * J._I[1] + T._T[6] * J._I[5],
					T._T[8] * J._I[3] + T._T[9] * J._I[1] + T._T[10] * J._I[5],
					T._T[0] * J._I[4] + T._T[1] * J._I[5] + T._T[2] * J._I[2],
					T._T[4] * J._I[4] + T._T[5] * J._I[5] + T._T[6] * J._I[2],
					T._T[8] * J._I[4] + T._T[9] * J._I[5] + T._T[10] * J._I[2] };
	// J._I = tmp * Rotation Part of T
	J._I[0] = tmp[0] * T._T[0] + tmp[3] * T._T[1] + tmp[6] * T._T[2];
	J._I[3] = tmp[1] * T._T[0] + tmp[4] * T._T[1] + tmp[7] * T._T[2];
	J._I[4] = tmp[2] * T._T[0] + tmp[5] * T._T[1] + tmp[8] * T._T[2];
	J._I[1] = tmp[1] * T._T[4] + tmp[4] * T._T[5] + tmp[7] * T._T[6];
	J._I[5] = tmp[2] * T._T[4] + tmp[5] * T._T[5] + tmp[8] * T._T[6];
	J._I[2] = tmp[2] * T._T[8] + tmp[5] * T._T[9] + tmp[8] * T._T[10];

	J._r[0] = T._T[0] * (_r[0] - _m * T._T[12]) + T._T[1] * (_r[1] - _m * T._T[13]) + T._T[2] * (_r[2] - _m * T._T[14]);
	J._r[1] = T._T[4] * (_r[0] - _m * T._T[12]) + T._T[5] * (_r[1] - _m * T._T[13]) + T._T[6] * (_r[2] - _m * T._T[14]);
	J._r[2] = T._T[8] * (_r[0] - _m * T._T[12]) + T._T[9] * (_r[1] - _m * T._T[13]) + T._T[10] * (_r[2] - _m * T._T[14]);
	
	return J;
}

inline dse3 Inertia::operator * (const se3 &s) const
{
	return dse3(_I[0] * s._w[0] + _I[3] * s._w[1] + _I[4] * s._w[2] + _r[1] * s._w[5] - _r[2] * s._w[4],
				_I[3] * s._w[0] + _I[1] * s._w[1] + _I[5] * s._w[2] + _r[2] * s._w[3] - _r[0] * s._w[5],
				_I[4] * s._w[0] + _I[5] * s._w[1] + _I[2] * s._w[2] + _r[0] * s._w[4] - _r[1] * s._w[3],
				s._w[1] * _r[2] - s._w[2] * _r[1] + _m * s._w[3],
				s._w[2] * _r[0] - s._w[0] * _r[2] + _m * s._w[4],
				s._w[0] * _r[1] - s._w[1] * _r[0] + _m * s._w[5]);
}

inline void Inertia::ToArray(double I[]) const
{
	I[0]  =  _I[0];		I[6]  =  _I[3];		I[12] =  _I[4];		I[18] =    0.0;		I[24] = -_r[2];		I[30] =  _r[1];
	I[1]  =  _I[3];		I[7]  =  _I[1];		I[13] =  _I[5];		I[19] =  _r[2];		I[25] =    0.0;		I[31] = -_r[0];
	I[2]  =  _I[4];		I[8]  =  _I[5];		I[14] =  _I[2];		I[20] = -_r[1];		I[26] =  _r[0];		I[32] =    0.0;
	I[3]  =    0.0;		I[9]  =  _r[2];		I[15] = -_r[1];		I[21] =     _m;		I[27] =    0.0;		I[33] =    0.0;
	I[4]  = -_r[2];		I[10] =    0.0;		I[16] =  _r[0];		I[22] =    0.0;		I[28] =     _m;		I[34] =    0.0;
	I[5]  =  _r[1];		I[11] = -_r[0];		I[17] =    0.0;		I[23] =    0.0;		I[29] =    0.0;		I[35] =     _m;
}

inline AInertia::AInertia(const AInertia &J)
{
	_A[0] = J._A[0]; _A[3] = J._A[3]; _A[4] = J._A[4]; _A[6] = J._A[6]; _A[7] = J._A[7]; _A[8] = J._A[8]; 
	_B[0] = J._B[0]; _B[1] = J._B[1]; _B[2] = J._B[2]; _B[3] = J._B[3]; _B[4] = J._B[4]; _B[5] = J._B[5]; _B[6] = J._B[6]; _B[7] = J._B[7]; _B[8] = J._B[8];
	_C[0] = J._C[0]; _C[3] = J._C[3]; _C[4] = J._C[4]; _C[6] = J._C[6]; _C[7] = J._C[7]; _C[8] = J._C[8];
}

inline AInertia::AInertia(const Inertia &J)
{
	_A[0] = J._I[0];	_A[3] = J._I[3];	_A[4] = J._I[1];
	_A[6] = J._I[4];	_A[7] = J._I[5];	_A[8] = J._I[2];
	_B[0] = _B[4] = _B[8] = _C[3] = _C[6] = _C[7] = 0.0;
	_B[5] = J._r[0];	_B[7] = -J._r[0];
	_B[6] = J._r[1];	_B[2] = -J._r[1];
	_B[1] = J._r[2];	_B[3] = -J._r[2];
	_C[0] = _C[4] = _C[8] = J._m;
}

inline AInertia::AInertia(double a0, double a3, double a4, double a6, double a7, double a8, double b0, double b1, double b2, double b3, double b4, double b5, double b6, double b7, double b8, double c0, double c3, double c4, double c6, double c7, double c8)
{
	_A[0] = a0;		_A[3] = a3;		_A[4] = a4;
	_A[6] = a6;		_A[7] = a7;		_A[8] = a8;
	_B[0] = b0;		_B[1] = b1;		_B[2] = b2;
	_B[3] = b3;		_B[4] = b4;		_B[5] = b5;
	_B[6] = b6;		_B[7] = b7;		_B[8] = b8;
	_C[0] = c0;		_C[3] = c3;		_C[4] = c4;
	_C[6] = c6;		_C[7] = c7;		_C[8] = c8;
}

inline AInertia::AInertia(const double *M)
{
	_A[0] = M[0]; _A[3] = M[ 6]; _A[6] = M[12]; _B[0] = M[18]; _B[3] = M[24]; _B[6] = M[30];
	              _A[4] = M[ 7]; _A[7] = M[13]; _B[1] = M[19]; _B[4] = M[25]; _B[7] = M[31];
	                             _A[8] = M[14]; _B[2] = M[20]; _B[5] = M[26]; _B[8] = M[32];
	                                            _C[0] = M[21]; _C[3] = M[27]; _C[6] = M[33];
	                                                           _C[4] = M[28]; _C[7] = M[34];
	                                                                          _C[8] = M[35];
}

inline AInertia AInertia::operator - (void) const
{
	return AInertia(-_A[0], -_A[3], -_A[4], -_A[6], -_A[7], -_A[8], 
					-_B[0], -_B[1], -_B[2], -_B[3], -_B[4], -_B[5], -_B[6], -_B[7], -_B[8],
					-_C[0], -_C[3], -_C[4], -_C[6], -_C[7], -_C[8]);
}

inline dse3 AInertia::operator * (const se3 &a) const
{
	return dse3(_A[0] * a._w[0] + _A[3] * a._w[1] + _A[6] * a._w[2] + _B[0] * a._w[3] + _B[3] * a._w[4] + _B[6] * a._w[5],
				_A[3] * a._w[0] + _A[4] * a._w[1] + _A[7] * a._w[2] + _B[1] * a._w[3] + _B[4] * a._w[4] + _B[7] * a._w[5],
				_A[6] * a._w[0] + _A[7] * a._w[1] + _A[8] * a._w[2] + _B[2] * a._w[3] + _B[5] * a._w[4] + _B[8] * a._w[5],
				_B[0] * a._w[0] + _B[1] * a._w[1] + _B[2] * a._w[2] + _C[0] * a._w[3] + _C[3] * a._w[4] + _C[6] * a._w[5],
				_B[3] * a._w[0] + _B[4] * a._w[1] + _B[5] * a._w[2] + _C[3] * a._w[3] + _C[4] * a._w[4] + _C[7] * a._w[5],
				_B[6] * a._w[0] + _B[7] * a._w[1] + _B[8] * a._w[2] + _C[6] * a._w[3] + _C[7] * a._w[4] + _C[8] * a._w[5]);
}

inline AInertia AInertia::operator + (const AInertia &J) const
{
	return AInertia(_A[0] + J._A[0], _A[3] + J._A[3], _A[4] + J._A[4], _A[6] + J._A[6], _A[7] + J._A[7], _A[8] + J._A[8], 
					_B[0] + J._B[0], _B[1] + J._B[1], _B[2] + J._B[2], _B[3] + J._B[3], _B[4] + J._B[4], _B[5] + J._B[5], _B[6] + J._B[6], _B[7] + J._B[7], _B[8] + J._B[8],
					_C[0] + J._C[0], _C[3] + J._C[3], _C[4] + J._C[4], _C[6] + J._C[6], _C[7] + J._C[7], _C[8] + J._C[8]);
}

inline AInertia AInertia::operator + (const Inertia &J) const
{
	return AInertia(_A[0] + J._I[0], _A[3] + J._I[3], _A[4] + J._I[1], _A[6] + J._I[4], _A[7] + J._I[5], _A[8] + J._I[2],
					_B[0], _B[1] + J._r[2], _B[2]-J._r[1], _B[3]-J._r[2], _B[4], _B[5] + J._r[0], _B[6] + J._r[1], _B[7]-J._r[0], _B[8],
					_C[0] + J._m, _C[3], _C[4] + J._m, _C[6], _C[7], _C[8] + J._m);
}

inline AInertia AInertia::operator - (const AInertia &J) const
{
	return AInertia(_A[0] - J._A[0], _A[3] - J._A[3], _A[4] - J._A[4], _A[6] - J._A[6], _A[7] - J._A[7], _A[8] - J._A[8],
					_B[0] - J._B[0], _B[1] - J._B[1], _B[2] - J._B[2], _B[3] - J._B[3], _B[4] - J._B[4], _B[5] - J._B[5], _B[6] - J._B[6], _B[7] - J._B[7], _B[8] - J._B[8],
					_C[0] - J._C[0], _C[3] - J._C[3], _C[4] - J._C[4], _C[6] - J._C[6], _C[7] - J._C[7], _C[8] - J._C[8]);
}

inline AInertia AInertia::operator - (const Inertia &J) const
{
	return AInertia(_A[0] - J._I[0], _A[3] - J._I[3], _A[4] - J._I[1], _A[6] - J._I[4], _A[7] - J._I[5], _A[8] - J._I[2],
					_B[0], _B[1] - J._r[2], _B[2]+J._r[1], _B[3]+J._r[2], _B[4], _B[5] - J._r[0], _B[6] - J._r[1], _B[7]+J._r[0], _B[8],
					_C[0] - J._m, _C[3], _C[4] - J._m, _C[6], _C[7], _C[8] - J._m);
}

inline AInertia operator + (const Inertia &A, const AInertia &B)
{
	return AInertia(B._A[0] + A._I[0], B._A[3] + A._I[3], B._A[4] + A._I[1], B._A[6] + A._I[4], B._A[7] + A._I[5], B._A[8] + A._I[2],
					B._B[0], B._B[1] + A._r[2], B._B[2]-A._r[1], B._B[3]-A._r[2], B._B[4], B._B[5] + A._r[0], B._B[6] + A._r[1], B._B[7]-A._r[0], B._B[8],
					B._C[0] + A._m, B._C[3], B._C[4] + A._m, B._C[6], B._C[7], B._C[8] + A._m);
}

inline AInertia operator - (const Inertia &A, const AInertia &B)
{
	return AInertia(-B._A[0] + A._I[0], -B._A[3] + A._I[3], -B._A[4] + A._I[1], -B._A[6] + A._I[4], -B._A[7] + A._I[5], -B._A[8] + A._I[2], 
					-B._B[0], -B._B[1] + A._r[2], -B._B[2] - A._r[1], -B._B[3] - A._r[2], -B._B[4], -B._B[5] + A._r[0], -B._B[6] + A._r[1], -B._B[7] - A._r[0], -B._B[8],
					-B._C[0] + A._m, -B._C[3], -B._C[4] + A._m, -B._C[6], -B._C[7], -B._C[8] + A._m); 
}

inline AInertia &AInertia::operator = (const AInertia &J)
{
	_A[0] = J._A[0];	_A[3] = J._A[3];	_A[4] = J._A[4];	_A[6] = J._A[6];	_A[7] = J._A[7];	_A[8] = J._A[8];
	_B[0] = J._B[0];	_B[1] = J._B[1];	_B[2] = J._B[2];	_B[3] = J._B[3];	_B[4] = J._B[4];	_B[5] = J._B[5];	_B[6] = J._B[6];	_B[7] = J._B[7];	_B[8] = J._B[8];
	_C[0] = J._C[0];	_C[3] = J._C[3];	_C[4] = J._C[4];	_C[6] = J._C[6];	_C[7] = J._C[7];	_C[8] = J._C[8];
	return *this;
}

inline AInertia &AInertia::operator = (const Inertia &J)
{
	_A[0] = J._I[0];	_A[3] = J._I[3];	_A[4] = J._I[1];	_A[6] = J._I[4];	_A[7] = J._I[5];	_A[8] = J._I[2];
	_B[1] = J._r[2];	_B[2] -= J._r[1];	_B[3] -= J._r[2];	_B[5] = J._r[0];	_B[6] = J._r[1];	_B[7] -= J._r[0];
	_C[0] = J._m;		_C[4] = J._m;		_C[8] = J._m;
	return *this;
}

inline AInertia &AInertia::operator += (const double *M)
{
	_A[0] += M[0]; _A[3] += M[ 6]; _A[6] += M[12]; _B[0] += M[18]; _B[3] += M[24]; _B[6] += M[30];
	               _A[4] += M[ 7]; _A[7] += M[13]; _B[1] += M[19]; _B[4] += M[25]; _B[7] += M[31];
	                               _A[8] += M[14]; _B[2] += M[20]; _B[5] += M[26]; _B[8] += M[32];
	                                               _C[0] += M[21]; _C[3] += M[27]; _C[6] += M[33];
	                                                               _C[4] += M[28]; _C[7] += M[34];
	                                                                               _C[8] += M[35];
	return *this;
}

inline AInertia &AInertia::operator -= (const double *M)
{
	_A[0] -= M[0]; _A[3] -= M[ 6]; _A[6] -= M[12]; _B[0] -= M[18]; _B[3] -= M[24]; _B[6] -= M[30];
	               _A[4] -= M[ 7]; _A[7] -= M[13]; _B[1] -= M[19]; _B[4] -= M[25]; _B[7] -= M[31];
	                               _A[8] -= M[14]; _B[2] -= M[20]; _B[5] -= M[26]; _B[8] -= M[32];
	                                               _C[0] -= M[21]; _C[3] -= M[27]; _C[6] -= M[33];
	                                                               _C[4] -= M[28]; _C[7] -= M[34];
	                                                                               _C[8] -= M[35];
	return *this;
}

inline AInertia &AInertia::operator += (const AInertia &J)
{
	_A[0] += J._A[0];	_A[3] += J._A[3];	_A[4] += J._A[4];	_A[6] += J._A[6];	_A[7] += J._A[7];	_A[8] += J._A[8];
	_B[0] += J._B[0];	_B[1] += J._B[1];	_B[2] += J._B[2];	_B[3] += J._B[3];	_B[4] += J._B[4];	_B[5] += J._B[5];	_B[6] += J._B[6];	_B[7] += J._B[7];	_B[8] += J._B[8];
	_C[0] += J._C[0];	_C[3] += J._C[3];	_C[4] += J._C[4];	_C[6] += J._C[6];	_C[7] += J._C[7];	_C[8] += J._C[8];
	return *this;
}

inline AInertia &AInertia::operator -= (const AInertia &J)
{
	_A[0] -= J._A[0];	_A[3] -= J._A[3];	_A[4] -= J._A[4];	_A[6] -= J._A[6];	_A[7] -= J._A[7];	_A[8] -= J._A[8];
	_B[0] -= J._B[0];	_B[1] -= J._B[1];	_B[2] -= J._B[2];	_B[3] -= J._B[3];	_B[4] -= J._B[4];	_B[5] -= J._B[5];	_B[6] -= J._B[6];	_B[7] -= J._B[7];	_B[8] -= J._B[8];
	_C[0] -= J._C[0];	_C[3] -= J._C[3];	_C[4] -= J._C[4];	_C[6] -= J._C[6];	_C[7] -= J._C[7];	_C[8] -= J._C[8];
	return *this;
}

inline AInertia &AInertia::operator += (const Inertia &J)
{
	_A[0] += J._I[0];	_A[3] += J._I[3];	_A[4] += J._I[1];	_A[6] += J._I[4];	_A[7] += J._I[5];	_A[8] += J._I[2];
	_B[1] += J._r[2];	_B[2] -= J._r[1];	_B[3] -= J._r[2];	_B[5] += J._r[0];	_B[6] += J._r[1];	_B[7] -= J._r[0];
	_C[0] += J._m;		_C[4] += J._m;		_C[8] += J._m;
	return *this;
}

inline AInertia &AInertia::operator -= (const Inertia &J)
{
	_A[0] -= J._I[0];	_A[3] -= J._I[3];	_A[4] -= J._I[1];	_A[6] -= J._I[4];	_A[7] -= J._I[5];	_A[8] -= J._I[2];
	_B[1] -= J._r[2];	_B[2] += J._r[1];	_B[3] += J._r[2];	_B[5] -= J._r[0];	_B[6] -= J._r[1];	_B[7] += J._r[0];
	_C[0] -= J._m;		_C[4] -= J._m;		_C[8] -= J._m;
	return *this;
}

inline void AInertia::ToArray(double I[]) const
{
	I[0] = _A[0]; I[ 6] = _A[3]; I[12] = _A[6]; I[18] = _B[0]; I[24] = _B[3]; I[30] = _B[6];
	I[1] = _A[3]; I[ 7] = _A[4]; I[13] = _A[7]; I[19] = _B[1]; I[25] = _B[4]; I[31] = _B[7];
	I[2] = _A[6]; I[ 8] = _A[7]; I[14] = _A[8]; I[20] = _B[2]; I[26] = _B[5]; I[32] = _B[8];
	I[3] = _B[0]; I[ 9] = _B[1]; I[15] = _B[2]; I[21] = _C[0]; I[27] = _C[3]; I[33] = _C[6];
	I[4] = _B[3]; I[10] = _B[4]; I[16] = _B[5]; I[22] = _C[3]; I[28] = _C[4]; I[34] = _C[7];
	I[5] = _B[6]; I[11] = _B[7]; I[17] = _B[8]; I[23] = _C[6]; I[29] = _C[7]; I[35] = _C[8];
}

inline AInertia AInertia::Transform(const SE3 &T) const
{
	AInertia re;

	double _R[] = {	T._T[0], T._T[1], T._T[2], T._T[4], T._T[5], T._T[6], T._T[8], T._T[9], T._T[10] };

	double _D[] = {	_B[0] + T._T[14] * _C[3] - T._T[13] * _C[6], _B[1] - T._T[14] * _C[0] + T._T[12] * _C[6], _B[2] + T._T[13] * _C[0] - T._T[12] * _C[3],
					_B[3] + T._T[14] * _C[4] - T._T[13] * _C[7], _B[4] - T._T[14] * _C[3] + T._T[12] * _C[7], _B[5] + T._T[13] * _C[3] - T._T[12] * _C[4],
					_B[6] + T._T[14] * _C[7] - T._T[13] * _C[8], _B[7] - T._T[14] * _C[6] + T._T[12] * _C[8], _B[8] + T._T[13] * _C[6] - T._T[12] * _C[7] };

	double _E[] = {	_A[0] + T._T[14] * _B[3] - T._T[13] * _B[6] + _D[3] * T._T[14] - _D[6] * T._T[13], 0.0, 0.0,
					_A[3] + T._T[14] * _B[4] - T._T[13] * _B[7] - _D[0] * T._T[14] + _D[6] * T._T[12], _A[4] - T._T[14] * _B[1] + T._T[12] * _B[7] - _D[1] * T._T[14] + _D[7] * T._T[12], 0.0,
					_A[6] + T._T[14] * _B[5] - T._T[13] * _B[8] + _D[0] * T._T[13] - _D[3] * T._T[12], _A[7] - T._T[14] * _B[2] + T._T[12] * _B[8] + _D[1] * T._T[13] - _D[4] * T._T[12], _A[8] + T._T[13] * _B[2] - T._T[12] * _B[5] + _D[2] * T._T[13] - _D[5] * T._T[12]	};

	re._A[0] = (_R[0] * _E[0] + _R[1] * _E[3] + _R[2] * _E[6]) * _R[0] + (_R[0] * _E[3] + _R[1] * _E[4] + _R[2] * _E[7]) * _R[1] + (_R[0] * _E[6] + _R[1] * _E[7] + _R[2] * _E[8]) * _R[2];
	re._A[3] = (_R[0] * _E[0] + _R[1] * _E[3] + _R[2] * _E[6]) * _R[3] + (_R[0] * _E[3] + _R[1] * _E[4] + _R[2] * _E[7]) * _R[4] + (_R[0] * _E[6] + _R[1] * _E[7] + _R[2] * _E[8]) * _R[5];
	re._A[4] = (_R[3] * _E[0] + _R[4] * _E[3] + _R[5] * _E[6]) * _R[3] + (_R[3] * _E[3] + _R[4] * _E[4] + _R[5] * _E[7]) * _R[4] + (_R[3] * _E[6] + _R[4] * _E[7] + _R[5] * _E[8]) * _R[5];
	re._A[6] = (_R[0] * _E[0] + _R[1] * _E[3] + _R[2] * _E[6]) * _R[6] + (_R[0] * _E[3] + _R[1] * _E[4] + _R[2] * _E[7]) * _R[7] + (_R[0] * _E[6] + _R[1] * _E[7] + _R[2] * _E[8]) * _R[8];
	re._A[7] = (_R[3] * _E[0] + _R[4] * _E[3] + _R[5] * _E[6]) * _R[6] + (_R[3] * _E[3] + _R[4] * _E[4] + _R[5] * _E[7]) * _R[7] + (_R[3] * _E[6] + _R[4] * _E[7] + _R[5] * _E[8]) * _R[8];
	re._A[8] = (_R[6] * _E[0] + _R[7] * _E[3] + _R[8] * _E[6]) * _R[6] + (_R[6] * _E[3] + _R[7] * _E[4] + _R[8] * _E[7]) * _R[7] + (_R[6] * _E[6] + _R[7] * _E[7] + _R[8] * _E[8]) * _R[8];

	re._B[0] = (_R[0] * _D[0] + _R[1] * _D[1] + _R[2] * _D[2]) * _R[0] + (_R[0] * _D[3] + _R[1] * _D[4] + _R[2] * _D[5]) * _R[1] + (_R[0] * _D[6] + _R[1] * _D[7] + _R[2] * _D[8]) * _R[2];
	re._B[1] = (_R[3] * _D[0] + _R[4] * _D[1] + _R[5] * _D[2]) * _R[0] + (_R[3] * _D[3] + _R[4] * _D[4] + _R[5] * _D[5]) * _R[1] + (_R[3] * _D[6] + _R[4] * _D[7] + _R[5] * _D[8]) * _R[2];
	re._B[2] = (_R[6] * _D[0] + _R[7] * _D[1] + _R[8] * _D[2]) * _R[0] + (_R[6] * _D[3] + _R[7] * _D[4] + _R[8] * _D[5]) * _R[1] + (_R[6] * _D[6] + _R[7] * _D[7] + _R[8] * _D[8]) * _R[2];
	re._B[3] = (_R[0] * _D[0] + _R[1] * _D[1] + _R[2] * _D[2]) * _R[3] + (_R[0] * _D[3] + _R[1] * _D[4] + _R[2] * _D[5]) * _R[4] + (_R[0] * _D[6] + _R[1] * _D[7] + _R[2] * _D[8]) * _R[5];
	re._B[4] = (_R[3] * _D[0] + _R[4] * _D[1] + _R[5] * _D[2]) * _R[3] + (_R[3] * _D[3] + _R[4] * _D[4] + _R[5] * _D[5]) * _R[4] + (_R[3] * _D[6] + _R[4] * _D[7] + _R[5] * _D[8]) * _R[5];
	re._B[5] = (_R[6] * _D[0] + _R[7] * _D[1] + _R[8] * _D[2]) * _R[3] + (_R[6] * _D[3] + _R[7] * _D[4] + _R[8] * _D[5]) * _R[4] + (_R[6] * _D[6] + _R[7] * _D[7] + _R[8] * _D[8]) * _R[5];
	re._B[6] = (_R[0] * _D[0] + _R[1] * _D[1] + _R[2] * _D[2]) * _R[6] + (_R[0] * _D[3] + _R[1] * _D[4] + _R[2] * _D[5]) * _R[7] + (_R[0] * _D[6] + _R[1] * _D[7] + _R[2] * _D[8]) * _R[8];
	re._B[7] = (_R[3] * _D[0] + _R[4] * _D[1] + _R[5] * _D[2]) * _R[6] + (_R[3] * _D[3] + _R[4] * _D[4] + _R[5] * _D[5]) * _R[7] + (_R[3] * _D[6] + _R[4] * _D[7] + _R[5] * _D[8]) * _R[8];
	re._B[8] = (_R[6] * _D[0] + _R[7] * _D[1] + _R[8] * _D[2]) * _R[6] + (_R[6] * _D[3] + _R[7] * _D[4] + _R[8] * _D[5]) * _R[7] + (_R[6] * _D[6] + _R[7] * _D[7] + _R[8] * _D[8]) * _R[8];

	re._C[0] = (_R[0] * _C[0] + _R[1] * _C[3] + _R[2] * _C[6]) * _R[0] + (_R[0] * _C[3] + _R[1] * _C[4] + _R[2] * _C[7]) * _R[1] + (_R[0] * _C[6] + _R[1] * _C[7] + _R[2] * _C[8]) * _R[2];
	re._C[3] = (_R[0] * _C[0] + _R[1] * _C[3] + _R[2] * _C[6]) * _R[3] + (_R[0] * _C[3] + _R[1] * _C[4] + _R[2] * _C[7]) * _R[4] + (_R[0] * _C[6] + _R[1] * _C[7] + _R[2] * _C[8]) * _R[5];
	re._C[4] = (_R[3] * _C[0] + _R[4] * _C[3] + _R[5] * _C[6]) * _R[3] + (_R[3] * _C[3] + _R[4] * _C[4] + _R[5] * _C[7]) * _R[4] + (_R[3] * _C[6] + _R[4] * _C[7] + _R[5] * _C[8]) * _R[5];
	re._C[6] = (_R[0] * _C[0] + _R[1] * _C[3] + _R[2] * _C[6]) * _R[6] + (_R[0] * _C[3] + _R[1] * _C[4] + _R[2] * _C[7]) * _R[7] + (_R[0] * _C[6] + _R[1] * _C[7] + _R[2] * _C[8]) * _R[8];
	re._C[7] = (_R[3] * _C[0] + _R[4] * _C[3] + _R[5] * _C[6]) * _R[6] + (_R[3] * _C[3] + _R[4] * _C[4] + _R[5] * _C[7]) * _R[7] + (_R[3] * _C[6] + _R[4] * _C[7] + _R[5] * _C[8]) * _R[8];
	re._C[8] = (_R[6] * _C[0] + _R[7] * _C[3] + _R[8] * _C[6]) * _R[6] + (_R[6] * _C[3] + _R[7] * _C[4] + _R[8] * _C[7]) * _R[7] + (_R[6] * _C[6] + _R[7] * _C[7] + _R[8] * _C[8]) * _R[8];

	return re;
}

inline void AInertia::SubstractAlphaSSt(double a, const double *s)
{
	_A[0] -= a*s[0]*s[0];	_A[3] -= a*s[0]*s[1];	_A[6] -= a*s[0]*s[2];	_B[0] -= a*s[0]*s[3];	_B[3] -= a*s[0]*s[4];	_B[6] -= a*s[0]*s[5];
							_A[4] -= a*s[1]*s[1];	_A[7] -= a*s[1]*s[2];	_B[1] -= a*s[1]*s[3];	_B[4] -= a*s[1]*s[4];	_B[7] -= a*s[1]*s[5];
													_A[8] -= a*s[2]*s[2];	_B[2] -= a*s[2]*s[3];	_B[5] -= a*s[2]*s[4];	_B[8] -= a*s[2]*s[5];
																			_C[0] -= a*s[3]*s[3];	_C[3] -= a*s[3]*s[4];	_C[6] -= a*s[3]*s[5];
																									_C[4] -= a*s[4]*s[4];	_C[7] -= a*s[4]*s[5];
																															_C[8] -= a*s[5]*s[5];
}

// 0.5 * ( x * ~y + y * ~x)
inline AInertia KroneckerProduct(const dse3 &x, const dse3 &y)
{
	return AInertia(x._m[0] * y._m[0], 0.5 * (x._m[0] * y._m[1] + x._m[1] * y._m[0]), x._m[1] * y._m[1], 0.5 * (x._m[0] * y._m[2] + x._m[2] * y._m[0]), 0.5 * (x._m[1] * y._m[2] + x._m[2] * y._m[1]), x._m[2] * y._m[2],
					0.5 * (x._m[0] * y._m[3] + x._m[3] * y._m[0]), 0.5 * (x._m[1] * y._m[3] + x._m[3] * y._m[1]), 0.5 * (x._m[2] * y._m[3] + x._m[3] * y._m[2]), 0.5 * (x._m[0] * y._m[4] + x._m[4] * y._m[0]), 0.5 * (x._m[1] * y._m[4] + x._m[4] * y._m[1]), 0.5 * (x._m[2] * y._m[4] + x._m[4] * y._m[2]), 0.5 * (x._m[0] * y._m[5] + x._m[5] * y._m[0]), 0.5 * (x._m[1] * y._m[5] + x._m[5] * y._m[1]), 0.5 * (x._m[2] * y._m[5] + x._m[5] * y._m[2]),
					x._m[3] * y._m[3], 0.5 * (x._m[3] * y._m[4] + x._m[4] * y._m[3]), x._m[4] * y._m[4], 0.5 * (x._m[3] * y._m[5] + x._m[5] * y._m[3]), 0.5 * (x._m[4] * y._m[5] + x._m[5] * y._m[4]), x._m[5] * y._m[5]);
}

inline void Mult_AInertia_se3(dse3 &re, const AInertia &J, const se3 &s)
{
	re[0] = J._A[0] * s[0] + J._A[3] * s[1] + J._A[6] * s[2] + J._B[0] * s[3] + J._B[3] * s[4] + J._B[6] * s[5];
	re[1] = J._A[3] * s[0] + J._A[4] * s[1] + J._A[7] * s[2] + J._B[1] * s[3] + J._B[4] * s[4] + J._B[7] * s[5];
	re[2] = J._A[6] * s[0] + J._A[7] * s[1] + J._A[8] * s[2] + J._B[2] * s[3] + J._B[5] * s[4] + J._B[8] * s[5];
	re[3] = J._B[0] * s[0] + J._B[1] * s[1] + J._B[2] * s[2] + J._C[0] * s[3] + J._C[3] * s[4] + J._C[6] * s[5];
	re[4] = J._B[3] * s[0] + J._B[4] * s[1] + J._B[5] * s[2] + J._C[3] * s[3] + J._C[4] * s[4] + J._C[7] * s[5];
	re[5] = J._B[6] * s[0] + J._B[7] * s[1] + J._B[8] * s[2] + J._C[6] * s[3] + J._C[7] * s[4] + J._C[8] * s[5];
}

inline void Mult_AInertia_se3(double *re, const AInertia &J, const double *s)
{
	re[0] = J._A[0] * s[0] + J._A[3] * s[1] + J._A[6] * s[2] + J._B[0] * s[3] + J._B[3] * s[4] + J._B[6] * s[5];
	re[1] = J._A[3] * s[0] + J._A[4] * s[1] + J._A[7] * s[2] + J._B[1] * s[3] + J._B[4] * s[4] + J._B[7] * s[5];
	re[2] = J._A[6] * s[0] + J._A[7] * s[1] + J._A[8] * s[2] + J._B[2] * s[3] + J._B[5] * s[4] + J._B[8] * s[5];
	re[3] = J._B[0] * s[0] + J._B[1] * s[1] + J._B[2] * s[2] + J._C[0] * s[3] + J._C[3] * s[4] + J._C[6] * s[5];
	re[4] = J._B[3] * s[0] + J._B[4] * s[1] + J._B[5] * s[2] + J._C[3] * s[3] + J._C[4] * s[4] + J._C[7] * s[5];
	re[5] = J._B[6] * s[0] + J._B[7] * s[1] + J._B[8] * s[2] + J._C[6] * s[3] + J._C[7] * s[4] + J._C[8] * s[5];
}

inline void Mult_AInertia_se3(double *re, const AInertia &J, const double *s, int num)
{
	for (int i=0; i<num; i++) {
		Mult_AInertia_se3(&re[6*i], J, &s[6*i]);
	}
}
#endif