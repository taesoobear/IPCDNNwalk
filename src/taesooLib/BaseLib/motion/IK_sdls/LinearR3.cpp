/*
 *
 * RayTrace Software Package, release 1.0,  May 3, 2002.
 *
 * Mathematics Subpackage (VrMath)
 *
 * Author: Samuel R. Buss
 *
 * Software accompanying the book
 *		3D Computer Graphics: A Mathematical Introduction with OpenGL,
 *		by S. Buss, Cambridge University Press, 2003.
 *
 * Software is "as-is" and carries no warranty.  It may be used without
 *   restriction, but if you modify it, please change the filenames to
 *   prevent confusion between different versions.  Please acknowledge
 *   all use of the software in any publications or products based on it.
 *
 * Bug reports: Sam Buss, sbuss@ucsd.edu.
 * Web page: http://math.ucsd.edu/~sbuss/MathCG
 *
 */


#include "MathMisc.h"
#include "LinearR3.h"
//#include "Spherical.h"

// ******************************************************
// * VectorR3 class - math library functions				*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **

const VectorR3 UnitVecIR3(1.0, 0.0, 0.0);
const VectorR3 UnitVecJR3(0.0, 1.0, 0.0);
const VectorR3 UnitVecKR3(0.0, 0.0, 1.0);

const VectorR3 VectorR3::Zero(0.0, 0.0, 0.0);
const VectorR3 VectorR3::UnitX( 1.0, 0.0, 0.0);
const VectorR3 VectorR3::UnitY( 0.0, 1.0, 0.0);
const VectorR3 VectorR3::UnitZ( 0.0, 0.0, 1.0);
const VectorR3 VectorR3::NegUnitX(-1.0, 0.0, 0.0);
const VectorR3 VectorR3::NegUnitY( 0.0,-1.0, 0.0);
const VectorR3 VectorR3::NegUnitZ( 0.0, 0.0,-1.0);


double VectorR3::MaxAbs() const
{
	register double m;
	m = (x>0.0) ? x : -x;
	if ( y>m ) m=y;
	else if ( -y >m ) m = -y;
	if ( z>m ) m=z;
	else if ( -z>m ) m = -z;
	return m;
}



// *********************************************************************
// Rotation routines												   *
// *********************************************************************

// s.Rotate(theta, u) rotates s and returns s 
//        rotated theta degrees around unit vector w.
VectorR3& VectorR3::Rotate( double theta, const vector3& w) 
{
	double c = cos(theta);
	double s = sin(theta);
	double dotw = (x*w.x + y*w.y + z*w.z);
	double v0x = dotw*w.x;
	double v0y = dotw*w.y;		// v0 = provjection onto w
	double v0z = dotw*w.z;
	double v1x = x-v0x;
	double v1y = y-v0y;			// v1 = projection onto plane normal to w
	double v1z = z-v0z;
	double v2x = w.y*v1z - w.z*v1y;
	double v2y = w.z*v1x - w.x*v1z;	// v2 = w * v1 (cross product)
	double v2z = w.x*v1y - w.y*v1x;
	
	x = v0x + c*v1x + s*v2x;
	y = v0y + c*v1y + s*v2y;
	z = v0z	+ c*v1z + s*v2z;

	return ( *this );
}

// Rotate unit vector x in the direction of "dir": length of dir is rotation angle.
//		x must be a unit vector.  dir must be perpindicular to x.
VectorR3& VectorR3::RotateUnitInDirection ( const vector3& dir)
{	
	double theta = dir.squaredLength();
	if ( theta==0.0 ) {
		return *this;
	}
	else {
		theta = sqrt(theta);
		double costheta = cos(theta);
		double sintheta = sin(theta);
		vector3 dirUnit = dir/theta;
		*this = costheta*(*this) + sintheta*dirUnit;
		return ( *this );
	}
}


// ***************************************************************
//  Linear Algebra Utilities									 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

// Returns a righthanded orthonormal basis to complement vector u
void GetOrtho( const VectorR3& u,  VectorR3& v, VectorR3& w)
{
	if ( u.x > 0.5 || u.x<-0.5 || u.y > 0.5 || u.y<-0.5 ) {
		v.Set ( u.y, -u.x, 0.0 );
	}
	else {
		v.Set ( 0.0, u.z, -u.y);
	}
	v.Normalize();
	w = u;
	w *= v;
	w.Normalize();
	// w.NormalizeFast();
	return;
}

// Returns a vector v orthonormal to unit vector u
void GetOrtho( const VectorR3& u,  VectorR3& v )
{
	if ( u.x > 0.5 || u.x<-0.5 || u.y > 0.5 || u.y<-0.5 ) {
		v.Set ( u.y, -u.x, 0.0 );
	}
	else {
		v.Set ( 0.0, u.z, -u.y);
	}
	v.Normalize();
	return;
}

// ***************************************************************
//  Stream Output Routines										 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

ostream& operator<< ( ostream& os, const VectorR3& u )
{
	return (os << "<" << u.x << "," << u.y << "," << u.z << ">");
}

