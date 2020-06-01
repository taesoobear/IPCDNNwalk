
#ifndef LIEGROUP_T_H_
#define LIEGROUP_T_H_ 
#include "../math/matrix3.h"

//Taesoo's  interface
// use these instead of those in BaseLib/motion/GMBS/*.h
namespace Liegroup
{

// SE3 : use matrix4 or transf
// SO3 : use matrix3 or quater

// do not make virtual function
class se3
{
	public:
		double _m[6];
		se3(){}
		~se3(){}

		se3(vector3 const& w, vector3 const& v){W()=w; V()=v;}
		se3(double m0, double m1, double m2, double m3, double m4, double m5) { _m[0] = m0; _m[1] = m1; _m[2] = m2; _m[3] = m3; _m[4] = m4; _m[5] = m5; }
		se3(const se3& t) { _m[0] = t._m[0]; _m[1] = t._m[1]; _m[2] = t._m[2];	_m[3] = t._m[3]; _m[4] = t._m[4]; _m[5] = t._m[5]; }
		se3(double k) { _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = k; }
		// operators
		const se3 &operator + (void) const { return *this; }
		se3 operator - (void) const { return se3(-_m[0], -_m[1], -_m[2], -_m[3], -_m[4], -_m[5]); }
		se3 &operator = (const se3 &t) { _m[0] = t._m[0]; _m[1] = t._m[1]; _m[2] = t._m[2]; _m[3] = t._m[3]; _m[4] = t._m[4]; _m[5] = t._m[5]; return *this; }
		se3 &operator = (double d) { _m[0] = d; _m[1] = d; _m[2] = d; _m[3] = d; _m[4] = d; _m[5] = d; return *this; }
		se3 &operator = (double* d) { for(int i=0; i<6; i++) _m[i] = d[i]; return *this;}
		se3 &operator += (const se3 &t) { _m[0] += t._m[0]; _m[1] += t._m[1]; _m[2] += t._m[2]; _m[3] += t._m[3]; _m[4] += t._m[4]; _m[5] += t._m[5]; return *this; }
		se3 &operator -= (const se3 &t) { _m[0] -= t._m[0]; _m[1] -= t._m[1]; _m[2] -= t._m[2]; _m[3] -= t._m[3]; _m[4] -= t._m[4]; _m[5] -= t._m[5]; return *this; }
		se3 &operator *= (double d) { _m[0] *= d; _m[1] *= d; _m[2] *= d; _m[3] *= d; _m[4] *= d; _m[5] *= d; return *this; }
		se3 &operator /= (double d) { d = 1.0 / d; _m[0] *= d; _m[1] *= d; _m[2] *= d; _m[3] *= d; _m[4] *= d; _m[5] *= d; return *this; }
		se3 operator + (const se3 &t) const { return se3(_m[0] + t._m[0], _m[1] + t._m[1], _m[2] + t._m[2], _m[3] + t._m[3], _m[4] + t._m[4], _m[5] + t._m[5]); }	
		se3 operator - (const se3 &t) const { return se3(_m[0] - t._m[0], _m[1] - t._m[1], _m[2] - t._m[2], _m[3] - t._m[3], _m[4] - t._m[4], _m[5] - t._m[5]); }	
		se3 operator * (double d) const { return se3(d * _m[0], d * _m[1], d * _m[2], d * _m[3], d * _m[4], d * _m[5]); }
		double &operator [] (int i) { return _m[i]; }
		double operator [] (int i) const { return _m[i]; }
		// methods
		void zero(void) { _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = 0.0; }
		double innerProduct(const se3& o) { const double *s=o._m; return (_m[0] * s[0] + _m[1] * s[1] + _m[2] * s[2] + _m[3] * s[3] + _m[4] * s[4] + _m[5] * s[5]); }
		double *ptr() { return _m; }
		const double *ptr() const { return _m; }
		double squaredLen() const { return _m[0] * _m[0] + _m[1] * _m[1] + _m[2] * _m[2] + _m[3] * _m[3] + _m[4] * _m[4] + _m[5] * _m[5]; }

		// angular velocity
		vector3 & W() const { return *((vector3*)(&_m[0]));}
		// velocity
		vector3 & V() const { return *((vector3*)(&_m[3]));}
		friend se3 operator * (double d, const se3 &t) { return se3(d * t._m[0], d * t._m[1], d * t._m[2], d * t._m[3], d * t._m[4], d * t._m[5]); }
		transf exp() const;
		void log(transf const& o);
		void Ad(transf const& T, se3 const& o);
		inline vectornView vec() const { return vectornView((m_real*)&_m[0], 6, 1);}
};

inline void zero(matrix4 & m){ m.setValue(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);}
inline matrix3 SO3(quater const& q) 				{ matrix3 m; m.setFromQuaternion(q); return m;}
inline matrix4 toSE3(vector3 const& p) 				{ matrix4 m; m.setTranslation(p, false); return m;}
inline vector3 position(matrix4 const& m) 		{ vector3 v; v.x=m._14; v.y=m._24; v.z=m._34; return v;}
inline matrix3 skew(vector3 const& w) 				{ matrix3 m; m.setTilde(w.x, w.y, w.z); return m;}
inline se3 mult(matrix3 const& r, se3 const& in) 	{ return se3(r*in.W(), r*in.V()); }
se3 twist(transf const& tf1, transf const& tf2, double timestep);

class dse3
{
	public:
		double _m[6];

	dse3() { }
	~dse3(){}
	dse3(double k) { _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = k; }
	dse3(double m0, double m1, double m2, double m3, double m4, double m5) { _m[0] = m0; _m[1] = m1; _m[2] = m2; _m[3] = m3; _m[4] = m4; _m[5] = m5; }
	dse3(const dse3 &t) { _m[0] = t._m[0]; _m[1] = t._m[1]; _m[2] = t._m[2];	_m[3] = t._m[3]; _m[4] = t._m[4]; _m[5] = t._m[5]; }
	dse3(const vector3 &m, const vector3 &f) { M()=m; F()=f;}

	// operators
	const dse3 &operator + (void) const { return *this; }
	dse3 operator - (void) const { return dse3(-_m[0], -_m[1], -_m[2], -_m[3], -_m[4], -_m[5]); }
	dse3 &operator = (const dse3 &t) { _m[0] = t._m[0]; _m[1] = t._m[1]; _m[2] = t._m[2]; _m[3] = t._m[3]; _m[4] = t._m[4]; _m[5] = t._m[5]; return *this; }
	dse3 &operator = (double d) { _m[0] = d; _m[1] = d; _m[2] = d; _m[3] = d; _m[4] = d; _m[5] = d; return *this; }
	dse3 &operator = (double* d) { for(int i=0; i<6; i++) _m[i] = d[i]; return *this;}
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
	void zero(void) { _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = 0.0; }
	double innerProduct(const dse3& o) { const double *s=o._m; return (_m[0] * s[0] + _m[1] * s[1] + _m[2] * s[2] + _m[3] * s[3] + _m[4] * s[4] + _m[5] * s[5]); }
	double *ptr() { return _m; }
	const double *ptr() const { return _m; }
	double squaredLen() const { return _m[0] * _m[0] + _m[1] * _m[1] + _m[2] * _m[2] + _m[3] * _m[3] + _m[4] * _m[4] + _m[5] * _m[5]; }
	
	vector3 & M() const { return *((vector3*)(&_m[0]));}
	vector3 & F() const { return *((vector3*)(&_m[3]));}
	void dAd(transf const& T, dse3 const& o);
	void inv_dAd(transf const& T, dse3 const& o);
	inline dse3 dAd(transf const& T) { dse3 temp; temp.dAd(T, *this); return temp;}
	inline dse3 inv_dAd(transf const& T) { dse3 temp; temp.inv_dAd(T, *this); return temp;}
	// friend functions
	friend dse3 operator * (double d, const dse3 &t) { return dse3(d * t._m[0], d * t._m[1], d * t._m[2], d * t._m[3], d * t._m[4], d * t._m[5]); }
	//friend double operator * (const dse3 &t, const se3 &s) { return (t._m[0] * s._w[0] + t._m[1] * s._w[1] + t._m[2] * s._w[2] + t._m[3] * s._w[3] + t._m[4] * s._w[4] + t._m[5] * s._w[5]); }
	//friend double operator * (const se3 &s, const dse3 &t) { return (t * s); }
	friend vector3 const& getM(const dse3 &t) { return *((vector3*)(&t._m[0]));}
	friend vector3 const& getF(const dse3 &t) { return *((vector3*)(&t._m[3]));}
};
class Inertia
{
	public: // Inertia = [I, [r]; -[r], m1], junggon's comment
	double _I[6], _r[3], _m;	// _I[0] = Ixx, _I[1] = Iyy, _I[2] = Izz, _I[3] = Ixy, _I[4] = Ixz, _I[5] = Iyz
	//
	// constructors
	Inertia() { _I[0] = _I[1] = _I[2] = _I[3] = _I[4] = _I[5] = _r[0] = _r[1] = _r[2] = _m = 0.0; }
	Inertia(double m) { _m = _I[0] = _I[1] = _I[2] = m; _I[3] = _I[4] = _I[5] = _r[0] = _r[1] = _r[2] = 0.0; }
	Inertia(double mass, double Ixx, double Iyy, double Izz);
	Inertia(const Inertia &J) { _I[0] = J._I[0]; _I[1] = J._I[1]; _I[2] = J._I[2]; _I[3] = J._I[3]; _I[4] = J._I[4]; _I[5] = J._I[5]; _r[0] = J._r[0]; _r[1] = J._r[1]; _r[2] = J._r[2]; _m = J._m; }
	// c= m*localpos
	Inertia(double m, const matrix3& I, const vector3& c)
	{
		setInertia(I._11, I._22, I._33, I._12, I._13, I._23);
		setOffDiag(c);
		setMass(m);
	}
	
	// operator
	dse3 operator * (const se3 &acceleration) const;
	// methods
	void zero(void) { _m = _I[0] = _I[1] = _I[2] = _I[3] = _I[4] = _I[5] = _r[0] = _r[1] = _r[2] = 0.0; }
	void setMass(double mass) { _m = mass; }
	double mass(void) { return _m; }
	vector3 getOffDiag() { return vector3(_r[0], _r[1], _r[2]); }
	vector3 getDiag() { return vector3(_I[0], _I[1], _I[2]); }
	vector3 getSymm() { return vector3(_I[3], _I[4], _I[5]); }
	void setInertia(double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz) { _I[0] = Ixx; _I[1] = Iyy; _I[2] = Izz; _I[3] = Ixy; _I[4] = Ixz; _I[5] = Iyz; }
	void setOffDiag(const vector3& r) { _r[0] = r.x; _r[1] = r.y; _r[2] = r.z; }
	Inertia transform(const transf &T) const;
	
	friend dse3 dad(const se3 &V, const Inertia &J);
};

// works for matrix3, matrix4, matrixn 
template <class MAT, class MAT2>
inline void assign33T(MAT & a, MAT2 const& b)
{
	for (int i=0; i<3; i++)
		for (int j=0; j<3;j++)
			a(i,j)=b(j,i);
}	
template <class MAT, class MAT2>
inline void assign33(MAT& a, MAT2 const& b)
{
	for (int i=0; i<3; i++)
		for (int j=0; j<3;j++)
			a(i,j)=b(i,j);
}	
inline void skew(matrixn& out, vector3 const& w)
{
	out(0,0)=0;
	out(0,1)=-1*w.z;
	out(0,2)=w.y;
	out(1,0)=w.z;
	out(1,1)=0;
	out(1,2)=-1*w.x;
	out(2,0)=-1*w.y;
	out(2,1)=w.x;
	out(2,2)=0;
}
inline void dAd(::matrixn& a, transf const& b)
{
	matrix3 br=SO3(b.rotation);
	assign33T(a.range(0,3,0,3).lval(),br);
	assign33T(a.range(3,6,3,6).lval(),br);
	a.range(3,6,0,3).setAllValue(0);
	matrixnView skew_b_T=a.range(3,6,0,3);
	skew(skew_b_T, b.translation*-1);
	a.range(0,3,3,6).mult(a.range(0,3,0,3),skew_b_T);
	skew_b_T.setAllValue(0);
}
inline matrix4 calcDotT(transf const& b, se3 const& V)// V=inv_b*dot_b
{
	// b= (R p) 
	//    (0 1)
	//    (R p)* (invR  -invR*p) = I    
	//    (0 1)  ( 0        1)
	//         V=(skew(w) v)=(invR  -invR*p)*(dotR  dotP)
	//           ( 0      0) ( 0        1  ) ( 0     0  )
	//
	matrix3  R=SO3(b.rotation);
	vector3 const& P=b.translation;
	// invR*dotR=skew(w)
	matrix3 dotR=R*skew(V.W());
	// v=invR*dotP
	vector3 dotP=R*V.V();
	matrix4 out;
	assign33(out, dotR);
	out.m[0][3]=dotP.x;
	out.m[1][3]=dotP.y;
	out.m[2][3]=dotP.z;
	out.m[3][0]=0;
	out.m[3][1]=0;
	out.m[3][2]=0;
	out.m[3][3]=0;
	
	return out;
}
inline void dot_dAd(::matrixn& out, transf const& b, matrix4 const& dotB) // V=inv_b*dot_b
{
	// b= (R p) 
	//    (0 1)
	// inv_b=(invR  -invR*p)
	//       ( 0        1  )
	// because
	//    (R p)* (invR  -invR*p) = I    
	//    (0 1)  ( 0        1)
	//
	// dAd(b)=( R' R'*skew(-p)) = (R          0 )'
	//        ( 0    R'    )      (skew(p)*R  R )
	//
	matrix3 R=SO3(b.rotation);
	const vector3& P=b.translation;
	matrix3 dotR;
	assign33(dotR, dotB);
	vector3 dotP=position(dotB);

	assign33T(out.range(0,3,0,3).lval(),dotR);
	// d(skew(b)*T)/dt=skew(dotP)*R+skew(p)*dotR
	assign33T(out.range(0,3,3,6).lval(),skew(dotP)*R+skew(P)*dotR);
	out.range(3,6,0,3).setAllValue(0);
	assign33T(out.range(3,6,3,6).lval(),dotR);
}

inline se3 to_se3(::vectorn const& v, int start=0)
{
	return se3(v(start+0), v(start+1), v(start+2),v(start+3), v(start+4), v(start+5)); 
}
}
//std::ostream &operator << (std::ostream &os, const vector3 &s);
std::ostream &operator << (std::ostream &os, const Liegroup::se3 &s);
std::ostream &operator << (std::ostream &os, const Liegroup::dse3 &s);
std::ostream &operator << (std::ostream &os, const Liegroup::Inertia &s);

#endif
