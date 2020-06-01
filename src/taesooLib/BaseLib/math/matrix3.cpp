// matrix.cpp: implementation of the matrix class.
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "mathclass.h"
#include "matrix3.h"

matrix3::matrix3()
{

}
matrix3::matrix3(m_real v)
{
	_11=_12=_13=_21=_22=_23=_31=_32=_33=v;
}
void matrix3::operator*=(m_real scalar)
{
	_11*=scalar;
	_12*=scalar;
	_13*=scalar;
	_21*=scalar;
	_22*=scalar;
	_23*=scalar;
	_31*=scalar;
	_32*=scalar;
	_33*=scalar;
}

void matrix3::rotate(vector3 & v) const
{
	m_real x,y,z;	

	x=_11*v.x+_12*v.y+_13*v.z;
	y=_21*v.x+_22*v.y+_23*v.z;
	z=_31*v.x+_32*v.y+_33*v.z;

	v.x=x;
	v.y=y;
	v.z=z;
}
m_real& matrix3::operator[](int i) 
{
	switch(i)
	{
	case 0:
		return _11;
	case 1:
		return _12;
	case 2:
		return _13;
	case 3:
		return _21;
	case 4:
		return _22;
	case 5:
		return _23;
	case 6:
		return _31;
	case 7:
		return _32;
	case 8:
		return _33;
	}
	return _11;
}
m_real matrix3::operator[](int i) const	// indexed by 0,1,2,3,4,5,6,7,8
{
	switch(i)
	{
	case 0:
		return _11;
	case 1:
		return _12;
	case 2:
		return _13;
	case 3:
		return _21;
	case 4:
		return _22;
	case 5:
		return _23;
	case 6:
		return _31;
	case 7:
		return _32;
	case 8:
		return _33;
	}
	return _11;
}

bool matrix3::operator==(matrix3 const& b) const
{
	return	_11==b._11 &&
			_12==b._12 &&
			_13==b._13 &&
			_21==b._21 &&
			_22==b._22 &&
			_23==b._23 &&
			_31==b._31 &&
			_32==b._32 &&
			_33==b._33;
}

matrix3::~matrix3()
{

}

void matrix3::zero()
{
	for( int i=0; i<3; i++ )
		for( int j=0; j<3; j++ )
			m[i][j]= (m_real)0.0;
}

void matrix3::identity()
{
	_11 = _22 = _33 = (m_real)1.0;
	_12 = _13 = _21 = _23 = _31 = _32 = (m_real)0.0;
}

void matrix3::Multiply( matrix3 const&M, matrix3 &Dst )const
{
	Dst.mult(*this, M);
}

void matrix3::Add( matrix3 const&M, matrix3 &Dst )const
{
	Dst.add(*this,M);	
}

void matrix3::Subtract( matrix3 const&M, matrix3 &Dst )const
{
	Dst.sub(*this,M);
}

void matrix3::PostMultiply( vector3 const&v, vector3 &Dst )const
{
	Dst.x = _11 * v.x + _12 * v.y + _13 * v.z;
	Dst.y = _21 * v.x + _22 * v.y + _23 * v.z;
	Dst.z = _31 * v.x + _32 * v.y + _33 * v.z;
}

void matrix3::PreMultiply( vector3 const&v, vector3 &Dst )const
{
	Dst.x = v.x * _11 + v.y * _21 + v.z * _31;
	Dst.y = v.x * _12 + v.y * _22 + v.z * _32;
	Dst.z = v.x * _13 + v.y * _23 + v.z * _33;
}

void matrix3::mult(matrix3 const& a, m_real b)
{
	for( int i=0; i<3; i++ )
		for( int j=0; j<3; j++ )
			m[i][j]=a.m[i][j]*b;
}


void matrix3::Multiply( m_real scale, matrix3 &Dst )const
{
	for( int i=0; i<3; i++ )
		for( int j=0; j<3; j++ )
			Dst.m[i][j]=m[i][j]*scale;
}

void matrix3::Invert( matrix3 &Dst )const
{
	ASSERT(0);
}

bool matrix3::inverse()
{
	matrix3 t(*this);
	return inverse(t);
}

bool matrix3::inverse(matrix3 const& src) 
{
	
	// Invert a 3x3 using cofactors.  This is about 8 times faster than
	// the Numerical Recipes code which uses Gaussian elimination.

	m[0][0] = src(1,1)*src(2,2) -
		src(1,2)*src(2,1);
	m[0][1] = src(0,2)*src(2,1) -
		src(0,1)*src(2,2);
	m[0][2] = src(0,1)*src(1,2) -
		src(0,2)*src(1,1);
	m[1][0] = src(1,2)*src(2,0) -
		src(1,0)*src(2,2);
	m[1][1] = src(0,0)*src(2,2) -
		src(0,2)*src(2,0);
	m[1][2] = src(0,2)*src(1,0) -
		src(0,0)*src(1,2);
	m[2][0] = src(1,0)*src(2,1) -
		src(1,1)*src(2,0);
	m[2][1] = src(0,1)*src(2,0) -
		src(0,0)*src(2,1);
	m[2][2] = src(0,0)*src(1,1) -
		src(0,1)*src(1,0);

	m_real fDet =
		src(0,0)*m[0][0] +
		src(0,1)*m[1][0]+
		src(0,2)*m[2][0];

	m_real fTolerance=0.0000001;
	if ( ABS(fDet) <= fTolerance )
		return false;

	m_real fInvDet = 1.0/fDet;
	for (size_t iRow = 0; iRow < 3; iRow++)
        {
            for (size_t iCol = 0; iCol < 3; iCol++)
                m[iRow][iCol] *= fInvDet;
        }

	return true;
}
matrix3::matrix3(matrix3 const& a)
{
	_11=a._11;
	_12=a._12;
	_13=a._13;
	_21=a._21;
	_22=a._22;
	_23=a._23;
	_31=a._31; 
	_32=a._32;
	_33=a._33;
}

vector3 operator*(matrix3 const& a, vector3 const& b)
{
	vector3 c;
	c.x=a._11*b.x+a._12*b.y+a._13*b.z;
	c.y=a._21*b.x+a._22*b.y+a._23*b.z;
	c.z=a._31*b.x+a._32*b.y+a._33*b.z;
	return c;
}
vector3 operator*(vector3 const& a, matrix3 const& b)
{
	vector3 c;
	c.x=b._11*a.x+b._21*a.y+b._31*a.z;
	c.y=b._12*a.x+b._22*a.y+b._32*a.z;
	c.z=b._13*a.x+b._23*a.y+b._33*a.z;
	return c;
}
	
matrix3& matrix3::operator=( matrix3 const&a)
{
	_11=a._11;
	_12=a._12;
	_13=a._13;
	_21=a._21;
	_22=a._22;
	_23=a._23;
	_31=a._31;
	_32=a._32;
	_33=a._33;

	return *this;
}

void matrix3::setFromOuterProduct( vector3 const&v1, vector3 const&v2 )
{
	_11 = v1.x * v2.x; _12 = v1.x * v2.y; _13 = v1.x * v2.z;
	_21 = v1.y * v2.x; _22 = v1.y * v2.y; _23 = v1.y * v2.z;
	_31 = v1.z * v2.x; _32 = v1.z * v2.y; _33 = v1.z * v2.z;
}
void matrix3::setFromQuaternion(quater const& q)
{
	// jehee lee implementation
	matrix3& m=*this;
	m_real s, xs, ys, zs, wx, wy, wz, xx, xy, xz, yy, yz, zz;

	s  = 2.0/q.length();
	xs = q.x * s;  ys = q.y * s;  zs = q.z * s;
	wx = q.w * xs; wy = q.w * ys; wz = q.w * zs;
	xx = q.x * xs; xy = q.x * ys; xz = q.x * zs;
	yy = q.y * ys; yz = q.y * zs; zz = q.z * zs;

	m.m[0][0] = 1.0 - (yy + zz);
	m.m[0][1] = xy - wz;
	m.m[0][2] = xz + wy;
	m.m[1][0] = xy + wz;
	m.m[1][1] = 1.0 - (xx + zz);
	m.m[1][2] = yz - wx;
	m.m[2][0] = xz - wy;
	m.m[2][1] = yz + wx;
	m.m[2][2] = 1.0 - (xx + yy);
}

void matrix3::mult(matrix3 const& a,matrix3 const& M)
{
	matrix3& Dst=*this;

	Dst._11 = a._11 * M._11 + a._12 * M._21 + a._13 * M._31;
	Dst._12 = a._11 * M._12 + a._12 * M._22 + a._13 * M._32;
	Dst._13 = a._11 * M._13 + a._12 * M._23 + a._13 * M._33;
											  
	Dst._21 = a._21 * M._11 + a._22 * M._21 + a._23 * M._31;
	Dst._22 = a._21 * M._12 + a._22 * M._22 + a._23 * M._32;
	Dst._23 = a._21 * M._13 + a._22 * M._23 + a._23 * M._33;
											  
	Dst._31 = a._31 * M._11 + a._32 * M._21 + a._33 * M._31;
	Dst._32 = a._31 * M._12 + a._32 * M._22 + a._33 * M._32;
	Dst._33 = a._31 * M._13 + a._32 * M._23 + a._33 * M._33;
}

bool matrix3::isSymmetric()const
{
	if( (_12 != _21) || (_13 != _31) || (_23 != _32) )
		return false;
	return true;
}

bool matrix3::isTranspose( matrix3 const&other )const
{
	for( int i=0; i<3; i++ )
		for( int j=0; j<3; j++ )
			if( m[i][j] != other.m[j][i] )
				return false;
	return true;
}	

void matrix3::setValue( m_real a00, m_real a01, m_real a02,
		  m_real a10, m_real a11, m_real a12, 
		  m_real a20, m_real a21, m_real a22 )
{
	_11 = a00;
	_12 = a01;
	_13 = a02;
	_21 = a10;
	_22 = a11;
	_23 = a12;
	_31 = a20;
	_32 = a21;
	_33 = a22;
}

void matrix3::setValue( vector3 const&row1, vector3 const&row2, vector3 const&row3 )
{
	_11 = row1.x; _12 = row1.y; _13 = row1.z;
	_21 = row2.x; _22 = row2.y; _23 = row2.z;
	_31 = row3.x; _32 = row3.y; _33 = row3.z;
}

void matrix3::setTilde( m_real x, m_real y, m_real z )
{
	//  0 -z  y     a    -zb +yc
	//  z  0 -x  *  b  =  za -xc  == (x,y,z).cross(a, b, c)
	// -y  x  0     c    -ya +xb

	_11 = _22 = _33 = 0;
	_12 = -z; _13 = y;
	_21 = z; _23 = -x;
	_31 = -y; _32 = x;
}


void matrix3::transpose()
{
	m_real tmp;

	tmp = _12;
	_12 = _21; _21 = tmp;
	tmp = _13;
	_13 = _31; _31 = tmp;
	tmp = _23;
	_23 = _32; _32 = tmp;
}

//
// Do a cross product of each row with the vector
//
void matrix3::CrossProduct( vector3 const&v, matrix3 &Dst )const
{
	Dst._11 = _12 * v.z - _13 * v.y;
	Dst._12 = _13 * v.x - _11 * v.z;
	Dst._13 = _11 * v.y - _12 * v.x;
	Dst._21 = _22 * v.z - _23 * v.y;
	Dst._22 = _23 * v.x - _21 * v.z;
	Dst._23 = _21 * v.y - _22 * v.x;
	Dst._31 = _32 * v.z - _33 * v.y;
	Dst._32 = _33 * v.x - _31 * v.z;
	Dst._33 = _31 * v.y - _32 * v.x;
}


void matrix3::negate( )
{
	_11 = -_11;
	_12 = -_12;
	_13 = -_13;
	_21 = -_21;
	_22 = -_22;
	_23 = -_23;
	_31 = -_31;
	_32 = -_32;
	_33 = -_33;
}

void matrix3::operator*=(matrix3 const&other )
{
	matrix3 temp=*this;
	mult(temp, other);
}
void matrix3::operator+=(matrix3 const&b)
{
	_11+=b._11;
	_12+=b._12;
	_13+=b._13;
	_21+=b._21;
	_22+=b._22;
	_23+=b._23;
	_31+=b._31;
	_32+=b._32;
	_33+=b._33;
}

void matrix3::operator-=(matrix3 const&b)
{
	_11-=b._11;
	_12-=b._12;
	_13-=b._13;
	_21-=b._21;
	_22-=b._22;
	_23-=b._23;
	_31-=b._31;
	_32-=b._32;
	_33-=b._33;
}

void matrix3::add(matrix3 const& a,matrix3 const& b)
{
	_11=a._11+b._11;
	_12=a._12+b._12;
	_13=a._13+b._13;
	_21=a._21+b._21;
	_22=a._22+b._22;
	_23=a._23+b._23;
	_31=a._31+b._31;
	_32=a._32+b._32;
	_33=a._33+b._33;
}

void matrix3::sub(matrix3 const& a,matrix3 const& b)
{
	_11=a._11-b._11;
	_12=a._12-b._12;
	_13=a._13-b._13;
	_21=a._21-b._21;
	_22=a._22-b._22;
	_23=a._23-b._23;
	_31=a._31-b._31;
	_32=a._32-b._32;
	_33=a._33-b._33;

}

void matrix3::setTilde( vector3 const&v )
{
	setTilde( v.x, v.y, v.z );
}

void matrix3::Dump( char *szHeading )
{
	char szTemp[50];

	if( szHeading != NULL )
		Msg::print("%s\n", szHeading );

	for( int k=0; k<3; k++ )
	{
		sprintf( szTemp, "%8.4f %8.4f %8.4f\r\n", m[k][0], m[k][1], m[k][2]);
		Msg::print("%s\n", szTemp );
	}

}

void m::assign(matrixn & out, matrix3 const& M)
{
	ASSERT(out.rows()==3);
	ASSERT(out.cols()==3);

	out[0][0]=M._11;
	out[0][1]=M._12;
	out[0][2]=M._13;
	out[1][0]=M._21;
	out[1][1]=M._22;
	out[1][2]=M._23;
	out[2][0]=M._31;
	out[2][1]=M._32;
	out[2][2]=M._33;
}
void m::assign(matrix3 & M, matrixn const& in)
{
	ASSERT(in.rows()==3);
	ASSERT(in.cols()==3);

	M._11=in[0][0];
	M._12=in[0][1];
	M._13=in[0][2];
	M._21=in[1][0];
	M._22=in[1][1];
	M._23=in[1][2];
	M._31=in[2][0];
	M._32=in[2][1];
	M._33=in[2][2];
}
void matrix3::setRotation(const quater& q)
{
	// jehee lee implementation
    matrix3& m=*this;
    m_real s, xs, ys, zs, wx, wy, wz, xx, xy, xz, yy, yz, zz;

    s  = 2.0/q.length();
    xs = q.x * s;  ys = q.y * s;  zs = q.z * s;
    wx = q.w * xs; wy = q.w * ys; wz = q.w * zs;
    xx = q.x * xs; xy = q.x * ys; xz = q.x * zs;
    yy = q.y * ys; yz = q.y * zs; zz = q.z * zs;

    m.m[0][0] = 1.0 - (yy + zz);
    m.m[0][1] = xy - wz;
    m.m[0][2] = xz + wy;
    m.m[1][0] = xy + wz;
    m.m[1][1] = 1.0 - (xx + zz);
    m.m[1][2] = yz - wx;
    m.m[2][0] = xz - wy;
    m.m[2][1] = yz + wx;
    m.m[2][2] = 1.0 - (xx + yy);
}
std::ostream& operator<< ( std::ostream& os, const matrix3& u )
{
	return (os << "<" << u._11 << "," << u._12 << "," << u._13 << "\n"
				<< u._21 << "," << u._22 << "," << u._23 << "\n"
				<< u._31 << "," << u._32 << "," << u._33 << "\n"
			);
}
