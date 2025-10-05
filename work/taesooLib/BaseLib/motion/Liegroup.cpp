#include "../baselib.h"
#include <vector>
#include <map>
#include <algorithm>
#include "Liegroup.h"
#include "GMBS/liegroup.h"
#include "GMBS/liegroup2.inl"

transf Liegroup::se3::exp() const
{
	SE3 tmp=Exp(*((::se3*)(this)));
	transf out;
	out.rotation=toBaseR(tmp);
	out.translation=toBase(tmp.GetPosition());
	return out;
}

void Liegroup::se3::log(transf const& o)
{
	::se3 l=Log(toGMBS(o.rotation, o.translation));
	(*((::se3*)this))=l;
}

void Liegroup::se3::Ad(transf const& T, Liegroup::se3 const& o)
{
	vector3 tmp=T.rotation*o.W();
	W()=tmp;
	vector3 tmp2;
	tmp2.cross(T.translation, tmp);
	tmp2+=T.rotation*o.V();
	V()=tmp2;
}
void Liegroup::se3::Ad(quater const& rot, Liegroup::se3 const& o)
{
	W()=rot*o.W();
	V()=rot*o.V();
}

void Liegroup::se3::Ad(vector3 const& trans, Liegroup::se3 const& o)
{
	W()=o.W();
	V()=o.V()+trans.cross(o.W());
}

void Liegroup::dse3::dAd(transf const& T, dse3 const& o)
{
	vector3 tmp=o.M()-T.translation.cross(o.F());
	quater invR;
	invR.inverse(T.rotation);
	M()=invR*tmp;
	F()=invR*o.F();
}
void Liegroup::dse3::inv_dAd(transf const& T, dse3 const& o)
{
	// needs to be optimized
	dAd(T.inverse(), o);
}

Liegroup::Inertia::Inertia(double mass, double Ixx, double Iyy, double Izz)
{
	_m = mass;
	_I[0] = Ixx; _I[1] = Iyy; _I[2] = Izz;
	_I[3] = _I[4] = _I[5] = _r[0] = _r[1] = _r[2] = 0.0;
}

Liegroup::dse3 Liegroup::Inertia::operator * (const Liegroup::se3 &s) const
{
	return Liegroup::dse3(_I[0] * s._m[0] + _I[3] * s._m[1] + _I[4] * s._m[2] + _r[1] * s._m[5] - _r[2] * s._m[4],
				_I[3] * s._m[0] + _I[1] * s._m[1] + _I[5] * s._m[2] + _r[2] * s._m[3] - _r[0] * s._m[5],
				_I[4] * s._m[0] + _I[5] * s._m[1] + _I[2] * s._m[2] + _r[0] * s._m[4] - _r[1] * s._m[3],
				s._m[1] * _r[2] - s._m[2] * _r[1] + _m * s._m[3],
				s._m[2] * _r[0] - s._m[0] * _r[2] + _m * s._m[4],
				s._m[0] * _r[1] - s._m[1] * _r[0] + _m * s._m[5]);
}
Liegroup::Inertia Liegroup::Inertia::transform(const transf &T) const
{
	::SE3 TT=toGMBS(T.rotation, T.translation);
	Liegroup::Inertia I;
	*((::Inertia*)&I)=((::Inertia*)this)->Transform(TT);
	return I;
}

using namespace std;
ostream &operator << (ostream &os, const Liegroup::Inertia &iner)
{
	os << "I : " << iner._I[0] << " " << iner._I[3] << " " << iner._I[4] << endl;
	os << "    " << iner._I[3] << " " << iner._I[1] << " " << iner._I[5] << endl;
	os << "    " << iner._I[4] << " " << iner._I[5] << " " << iner._I[2] << endl;
	os << "r : " << iner._r[0] << " " << iner._r[1] << " " << iner._r[2] << endl;
	os << "m : " << iner._m << endl;
    return os;
}

/*
ostream &operator << (ostream &os, const vector3 &v)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{	
		if ( v[i] >= 0.0 ) os << " ";
		os << v[i] << " ";
	}
	os << "];" << endl;
    return os;
}
*/





ostream &operator << (ostream &os, const Liegroup::se3 &s)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{	
		if ( s[i] >= 0.0 ) os << " ";
		os << s[i] << " ";
	}
	os << "];" << endl;
    return os;
}

ostream &operator << (ostream &os, const Liegroup::dse3 &t)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{	
		if ( t[i] >= 0.0 ) os << " ";
		os << t[i] << " ";
	}
	os << "];" << endl;
    return os;
}

Liegroup::se3 Liegroup::twist(transf const& self, transf const& tf2, double timestep)
{
	// body velocity: invR_ab*dR_ab (eqn 2.48)
	matrix4 t1, t2;
	t1.setRotation(self.rotation);
	t1.leftMultTranslation(self.translation);
	t2.setRotation(tf2.rotation);
	t2.leftMultTranslation(tf2.translation);
	matrix4 dotT;
	dotT=t2-t1;
	dotT*=(1.0/timestep);
	matrix4 v;
	matrix4 invT1;
	invT1.inverse(t1);
	v=invT1*dotT;
	return Liegroup::se3(vector3(v._23*-1, v._13, v._12*-1), 
						vector3(v._14, v._24, v._34));
}
Liegroup::se3 Liegroup::twist_nonlinear(transf const& tf1, transf const& tf2, double timestep)
{
	transf inv_tf1;
	inv_tf1=tf1.inverse();
	transf dotT_timestep;
	dotT_timestep.mult(tf2, inv_tf1);
	Liegroup::se3 dotT;
	dotT.log(dotT_timestep);
	dotT.W()*=1.0/timestep;
	dotT.V()*=1.0/timestep;

	// body-vel= exp(Ad_{inv_t1}*v) *theta_dot
	// where t2=exp(v theta)*t1
	Liegroup::se3 twist;
	twist.Ad(tf1.inverse(), dotT);
	return twist;
}
