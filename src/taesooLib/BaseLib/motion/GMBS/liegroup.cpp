#include "liegroup.h"

#define _PI		3.14159265358979

using namespace std;

ostream &operator << (ostream &os, const Vec3 &v)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{	
		if ( v._v[i] >= 0.0 ) os << " ";
		os << v._v[i] << " ";
	}
	os << "];" << endl;
    return os;
}

ostream &operator << (ostream &os, const se3 &s)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{	
		if ( s._w[i] >= 0.0 ) os << " ";
		os << s._w[i] << " ";
	}
	os << "];" << endl;
    return os;
}

ostream &operator << (ostream &os, const dse3 &t)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{	
		if ( t._m[i] >= 0.0 ) os << " ";
		os << t._m[i] << " ";
	}
	os << "];" << endl;
    return os;
}

ostream &operator << (ostream &os, const SO3 &R)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[" << endl;
	for ( int i = 0; i < 3; i++ ) 
	{
		os << "  ";
		for ( int j = 0; j < 3; j++ )
		{
			if ( R._R[i+j*3] >= 0.0 ) os << " ";
			os << R._R[i+j*3];
			if ( j == 2 ) os << " ;" << endl;
			else os << "  ";
		}
	}
	os << "];" << endl;
	return os;
}

ostream &operator << (ostream &os, const SE3 &T)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[" << endl;
	for ( int i = 0; i < 4; i++ )
	{
		os << "  ";
		for ( int j = 0; j < 4; j++ )
		{
			if ( T._T[i+j*4] >= 0.0 ) os << " ";
			os << T._T[i+j*4];
			if ( j == 3 ) os << " ;" << endl;
			else os << "  ";
		}
	}
	os << "];" << endl;
	return os;
}

ostream &operator << (ostream &os, const Inertia &iner)
{
	os << "I : " << iner._I[0] << " " << iner._I[3] << " " << iner._I[4] << endl;
	os << "    " << iner._I[3] << " " << iner._I[1] << " " << iner._I[5] << endl;
	os << "    " << iner._I[4] << " " << iner._I[5] << " " << iner._I[2] << endl;
	os << "r : " << iner._r[0] << " " << iner._r[1] << " " << iner._r[2] << endl;
	os << "m : " << iner._m << endl;
    return os;
}

Inertia RectPrism(double density, double x, double y, double z)
{
	double mass = density * x * y * z, ix = 1.0 / 12.0 * mass * (y * y + z * z), iy = 1.0 / 12.0 * mass * (x * x + z * z), iz = 1.0 / 12.0 * mass * (x * x + y * y);
	return Inertia(mass, ix, iy, iz);	
}

Inertia Beam(double density, double rad, double len)
{
	double mass = density * _PI * rad * rad * len, i1 = 1.0 / 12.0 * mass * (3.0 * rad * rad + len * len), i2 = 1.0 / 2.0 * mass * rad * rad;
	return Inertia(mass, i1, i1, i2);	
}

Inertia Ball(double density, double rad)
{
	double mass = density * _PI * rad * rad, i = 1.0 / 5.0 * mass * rad * rad;
	return Inertia(mass, i, i, i);	
}

ostream &operator << (ostream &os, const AInertia &I)
{
	os << I._A[0] << " " << I._A[3] << " " << I._A[6] << " " << I._B[0] << " " << I._B[3] << " " << I._B[6] << endl;
	os << I._A[3] << " " << I._A[4] << " " << I._A[7] << " " << I._B[1] << " " << I._B[4] << " " << I._B[7] << endl;
	os << I._A[6] << " " << I._A[7] << " " << I._A[8] << " " << I._B[2] << " " << I._B[5] << " " << I._B[8] << endl;
	os << I._B[0] << " " << I._B[1] << " " << I._B[2] << " " << I._C[0] << " " << I._C[3] << " " << I._C[6] << endl;
	os << I._B[3] << " " << I._B[4] << " " << I._B[5] << " " << I._C[3] << " " << I._C[4] << " " << I._C[7] << endl;
	os << I._B[6] << " " << I._B[7] << " " << I._B[8] << " " << I._C[6] << " " << I._C[7] << " " << I._C[8] << endl;
	return os;
}

