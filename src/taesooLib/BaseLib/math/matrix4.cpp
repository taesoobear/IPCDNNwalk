// matrix.cpp: implementation of the matrix class.
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "mathclass.h"
#include "matrix4.h"
#include "matrix3.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

#ifdef DIRECT3D_VERSION
#ifdef MATH_SINGLE_PRECISION
#define USE_D3DFUNC
#endif
#endif
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

matrix4::matrix4()
{

}

matrix4::~matrix4()
{

}
matrix4::matrix4(const transf& transf)
{
	setRotation(transf.rotation);
	setTranslation(transf.translation);
}
void matrix4::operator*=(double b)
{
	_11*=b;
	_12*=b;
	_13*=b;
	_14*=b;
	_21*=b;
	_22*=b;
	_23*=b;
	_24*=b;
	_31*=b;
	_32*=b;
	_33*=b;
	_34*=b;
	_41*=b;
	_42*=b;
	_43*=b;
	_44*=b;
}

void matrix4::lookAtLH(const vector3& eye, const vector3& at, const vector3& up)
{	
#ifdef USE_D3DFUNC
	D3DXMATRIX dxmat;
	D3DXMatrixLookAtLH(&dxmat,eye,at,up);
	fromDXmat(dxmat);
#else
	printf("lookatlh\n");
	ASSERT(0);
	/*
	
	zaxis = normal(At - Eye)
	xaxis = normal(cross(Up, zaxis))
	yaxis = cross(zaxis, xaxis)

	 xaxis.x           xaxis.y           xaxis.z          -dot(xaxis, eye)
	 yaxis.x           yaxis.y           yaxis.z          -dot(yaxis, eye)
	 zaxis.x           zaxis.y           zaxis.z          -dot(zaxis, eye)
	 0                 0                 0                1
	*/
#endif
}

void matrix4::lookAtRH(const vector3& eye, const vector3& at, const vector3& up)
{
#ifdef USE_D3DFUNC
	D3DXMATRIX dxmat;
	D3DXMatrixLookAtRH(&dxmat,eye,at,up);
	fromDXmat(dxmat);
#else
/*
	zaxis = normal(Eye - At)
	xaxis = normal(cross(Up, zaxis))
	yaxis = cross(zaxis, xaxis)

	 xaxis.x           xaxis.y           xaxis.z          -dot(xaxis, eye)
	 yaxis.x           yaxis.y           yaxis.z          -dot(yaxis, eye)
	 zaxis.x           zaxis.y           zaxis.z          -dot(zaxis, eye)
	 0                 0                 0                1
	*/

	vector3 zaxis, xaxis, yaxis;
	zaxis.normalize(eye - at);

	xaxis.normalize(up.cross(zaxis));
	yaxis.cross(zaxis, xaxis);

	setValue(xaxis.x, xaxis.y, xaxis.z, 
			 yaxis.x, yaxis.y, yaxis.z,
			 zaxis.x, zaxis.y, zaxis.z);

	vector3 trans(xaxis%eye, yaxis%eye, zaxis%eye);
	setTranslation(trans*-1);

#endif
}

void matrix4::decomposeLH(vector3& eye, vector3& at, vector3& up) const
{
	printf("decomposelh\n");
	ASSERT(0);
}
void matrix4::decomposeRH(vector3& eye, vector3& at, vector3& up) const
{
	/*
	zaxis = normal(Eye - At)
	xaxis = normal(cross(Up, zaxis))
	yaxis = cross(zaxis, xaxis)

	 xaxis.x           xaxis.y           xaxis.z          -dot(xaxis, eye)
	 yaxis.x           yaxis.y           yaxis.z          -dot(yaxis, eye)
	 zaxis.x           zaxis.y           zaxis.z          -dot(zaxis, eye)
	 0                 0                 0                1
	*/

	vector3 xaxis(_11, _12, _13);
	up.setValue(_21, _22, _23);
	vector3 zaxis(_31, _32, _33);
	vector3 trans(_14*-1, _24*-1, _34*-1);
	matrix4 rot,invRot;
	rot.extractRot(*this);
	invRot.inverse(rot);
	eye.mult(invRot, trans);
	at.sub(eye,zaxis);
}

vector3 matrix4::operator*(vector3 const& a) const
{
	vector3 out;
	out.mult(*this, a);
	return out;
}

vector3 matrix4::rotate(vector3 const & v) const
{
	vector3 out;
	matrix4 const& mat=*this;
	out.x=mat._11*v.x+mat._12*v.y+mat._13*v.z;
	out.y=mat._21*v.x+mat._22*v.y+mat._23*v.z;
	out.z=mat._31*v.x+mat._32*v.y+mat._33*v.z;
	return out;
}
void matrix4::inverse(const matrix4& a)
{
#ifdef USE_D3DFUNC
	m_real fDet;
	D3DXMatrixInverse(*this, &fDet, a);
#else	
	adjoint(a);
	leftMult(1.0f / a.determinant());
#endif
}

void matrix4::extractRot(const matrix4& a)
{
	setValue(a._11,a._12,a._13, a._21, a._22, a._23, a._31, a._32, a._33);
}

void matrix4::leftMult(const matrix4& a)
{
	// this=a*this
	matrix4 temp;
	temp=*this;
	mult(a,temp);
}

void matrix4::operator*=(const matrix4& a)
{
	matrix4 temp;
	temp=*this;
	mult(temp,a);
}

void matrix4::mult(const matrix4& a, const matrix4& b)
{
#ifdef USE_D3DFUNC
	D3DXMatrixMultiply(*this, a,b);
#else
	if(&a==this)
	{
		matrix4 aa(a);
		mult(aa,b);
	}
	else if(&b==this || &a==&b)
	{
		matrix4 bb(b);
		mult(a,bb);
	}
	else
	{
		_11=a._11*b._11+a._12*b._21+a._13*b._31+a._14*b._41;
		_12=a._11*b._12+a._12*b._22+a._13*b._32+a._14*b._42;
		_13=a._11*b._13+a._12*b._23+a._13*b._33+a._14*b._43;
		_14=a._11*b._14+a._12*b._24+a._13*b._34+a._14*b._44;

		_21=a._21*b._11+a._22*b._21+a._23*b._31+a._24*b._41;
		_22=a._21*b._12+a._22*b._22+a._23*b._32+a._24*b._42;
		_23=a._21*b._13+a._22*b._23+a._23*b._33+a._24*b._43;
		_24=a._21*b._14+a._22*b._24+a._23*b._34+a._24*b._44;

		_31=a._31*b._11+a._32*b._21+a._33*b._31+a._34*b._41;
		_32=a._31*b._12+a._32*b._22+a._33*b._32+a._34*b._42;
		_33=a._31*b._13+a._32*b._23+a._33*b._33+a._34*b._43;
		_34=a._31*b._14+a._32*b._24+a._33*b._34+a._34*b._44;

		_41=a._41*b._11+a._42*b._21+a._43*b._31+a._44*b._41;
		_42=a._41*b._12+a._42*b._22+a._43*b._32+a._44*b._42;
		_43=a._41*b._13+a._42*b._23+a._43*b._33+a._44*b._43;
		_44=a._41*b._14+a._42*b._24+a._43*b._34+a._44*b._44;
	}

#endif
}
void matrix4::add(const matrix4& a, const matrix4& b)
{
	_11=a._11+b._11; _12=a._12+b._12; _13=a._13+b._13; _14=a._14+b._14;
	_21=a._21+b._21; _22=a._22+b._22; _23=a._23+b._23; _24=a._24+b._24;
	_31=a._31+b._31; _32=a._32+b._32; _33=a._33+b._33; _34=a._34+b._34;
	_41=a._41+b._41; _42=a._42+b._42; _43=a._43+b._43; _44=a._44+b._44;
}

void matrix4::sub(const matrix4& a, const matrix4& b)
{
	_11=a._11-b._11; _12=a._12-b._12; _13=a._13-b._13; _14=a._14-b._14;
	_21=a._21-b._21; _22=a._22-b._22; _23=a._23-b._23; _24=a._24-b._24;
	_31=a._31-b._31; _32=a._32-b._32; _33=a._33-b._33; _34=a._34-b._34;
	_41=a._41-b._41; _42=a._42-b._42; _43=a._43-b._43; _44=a._44-b._44;
}

void matrix4::mult(const matrix4& a, const quater& b)
{
	matrix4 temp;
	temp.setRotation(b);
	mult(a,temp);
}

void matrix4::mult(const quater& a, const matrix4& b)
{
	matrix4 temp;
	temp.setRotation(a);
	mult(temp,b);
}

void matrix4::leftMultRotation(const quater& b)
{
	matrix4 temp;
	temp.setRotation(b);
	leftMult(temp);
}

void matrix4::setTranslation(const vector3& vec, bool bPreserveCurrentRotation)
{
	if(!bPreserveCurrentRotation)
		setIdentityRot();

	_14=vec.x;
	_24=vec.y;
	_34=vec.z;
}

void matrix4::setValue( m_real x00, m_real x01, m_real x02,
				   m_real x10, m_real x11, m_real x12,		// 나머지는 0,1으로 초기화 
				   m_real x20, m_real x21, m_real x22 )	
	{ m[0][0]=x00, m[0][1]=x01, m[0][2]=x02,  m[1][0]=x10, m[1][1]=x11, m[1][2]=x12,  
m[2][0]=x20, m[2][1]=x21, m[2][2]=x22,  m[0][3]=0,m[1][3]=0,m[2][3]=0,m[3][0]=0,m[3][1]=0,m[3][2]=0,m[3][3]=1;}

void matrix4::setValue( m_real x00, m_real x01, m_real x02, m_real x03,
				   m_real x10, m_real x11, m_real x12, m_real x13,
				   m_real x20, m_real x21, m_real x22, m_real x23,
				   m_real x30, m_real x31, m_real x32, m_real x33)	
	{ m[0][0]=x00, m[0][1]=x01, m[0][2]=x02,  m[0][3]=x03,
	  m[1][0]=x10, m[1][1]=x11, m[1][2]=x12,  m[1][3]=x13,
	  m[2][0]=x20, m[2][1]=x21, m[2][2]=x22,  m[2][3]=x23,
	  m[3][0]=x30, m[3][1]=x31, m[3][2]=x32,  m[3][3]=x33;}

void matrix4::setTransform(const quater& rot, const vector3& trans)
{
	setRotation(rot);
	setTranslation(trans);
}

void matrix4::_discardTranslation()
{
	m[3][0]=0;
	m[3][1]=0;
	m[3][2]=0;
	m[3][3]=1;

	m[0][3]=0;
	m[1][3]=0;
	m[2][3]=0;
}

void matrix4::setRotation(const matrix3& m, bool bPreserveCurrentTranslation)
{
	_11=m._11;
	_12=m._12;
	_13=m._13;
	_21=m._21;
	_22=m._22;
	_23=m._23;
	_31=m._31;
	_32=m._32;
	_33=m._33;

	if(!bPreserveCurrentTranslation)
		_discardTranslation();
}


void matrix4::leftMultTranslation(const vector3& vec)
{
	//	 ( 1  x)
	//   (  1 y)*a
	//   (   1z)
	//   (    1)

	// I ASSUMED affine!!
	_14+=vec.x;
	_24+=vec.y;
	_34+=vec.z;
}

void matrix4::leftMultRotation(const vector3& axis, m_real angle)
{
	matrix4 temp;
	temp.setRotation(axis,angle);
	leftMult(temp);
}

void matrix4::leftMultScaling(m_real sx, m_real sy, m_real sz)
{
	matrix4 s;
	s.setScaling(sx,sy,sz);
	leftMult(s);
}

void matrix4::setRotation(const quater& q, bool bPreserveCurrentTranslation)
{
	// jehee lee implementation
    matrix4& m=*this;
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

	if(!bPreserveCurrentTranslation)
		_discardTranslation();

}


void matrix4::setIdentityRot()
{
#ifdef USE_D3DFUNC
	D3DXMatrixIdentity(*this);
#else
	setValue(1,0,0,0,1,0,0,0,1);
#endif
}

void matrix4::setRotation(const vector3& axis, m_real angle, bool bPreserveCurrentTranslation)
{
#ifdef USE_D3DFUNC
	D3DXMATRIX dxmat;
	D3DXMatrixRotationAxis(&dxmat,axis,angle);	
	fromDXmat(dxmat);
#else
	quater q;
	q.setRotation(axis, angle);
	setRotation(q,bPreserveCurrentTranslation);
#endif
}

void matrix4::setScaling(m_real sx, m_real sy, m_real sz)
{
#ifdef USE_D3DFUNC
	D3DXMatrixScaling(*this,sx,sy,sz);
#else
	setValue(sx,0,0,0,sy,0,0,0,sz);
#endif

}

void matrix4::transpose(const matrix4& a)
{
#ifdef USE_D3DFUNC
	D3DXMatrixTranspose(*this,a);
#else
	_11=a._11;
	_12=a._21;
	_13=a._31;
	_14=a._41;
	_21=a._12;
	_22=a._22;
	_23=a._32;
	_24=a._42;
	_31=a._13;
	_32=a._23;
	_33=a._33;
	_34=a._43;
	_41=a._14;
	_42=a._24;
	_43=a._34;
	_44=a._44;
#endif
}


void matrix4::setAxisRotation( const vector3& vecAxis, const vector3& front, const vector3& vecTarget)
{
	// front벡터를 vecAxis를 중심으로 회전해서 vecTarget과 vecAxis가 이루는 평면에 놓이도록 만드는 Matrix를 구한다.

	// Axis와 vecTarget이 이루는 평면의 normal을 계산한다.
	vector3 vecNormal, nVecNormal, vecProjVecTarget;
	vecNormal.cross(vecAxis, vecTarget);
	nVecNormal.normalize(vecNormal);
	vecProjVecTarget.cross(nVecNormal, vecAxis);

	// Axis와 front가 이루는 평면의 normal을 계산한다.
	vector3 vecNormal2,nVecNormal2, vecProjFront;
	vecNormal2.cross(vecAxis, front);
	nVecNormal2.normalize(vecNormal2);
	vecProjFront.cross(nVecNormal2, vecAxis);	

	// 이제 아래가 성립하는 mtrxAxisAlignedBillboard을 구한다.
	// 							   (vecAxis		 )T    ( vecAxis          )T
	//  mtrxAxisAlignedBillboard * (nVecNormal2  )  =  ( nVecNormal       )
	// 							   (vecProjFront )	   ( vecProjVecTarget )

	// 따라서							   vecAxis    T      	vecAxis    
	//		mtrxAxisAlignedBillboard =    (nVecNormal)	 *	(	nVecNormal2)
	//									 vecProjVecTarget		vecProjFront

	matrix4 mat2, mat1;
	
	mat2._11=vecAxis.x;			mat2._21=vecAxis.y;			mat2._31=vecAxis.z;			mat2._41=0;
	mat2._12=nVecNormal.x;		mat2._22=nVecNormal.y;		mat2._32=nVecNormal.z;		mat2._42=0;
	mat2._13=vecProjVecTarget.x;mat2._23=vecProjVecTarget.y;mat2._33=vecProjVecTarget.z;mat2._43=0; 
	mat2._14=0;					mat2._24=0;					mat2._34=0;					mat2._44=1;

	mat1._11=vecAxis.x;			mat1._12=vecAxis.y;		mat1._13=vecAxis.z;		mat1._14=0;
	mat1._21=nVecNormal2.x;		mat1._22=nVecNormal2.y; mat1._23=nVecNormal2.z; mat1._24=0;
	mat1._31=vecProjFront.x;	mat1._32=vecProjFront.y;mat1._33=vecProjFront.z;mat1._34=0; 
	mat1._41=0;					mat1._42=0;				mat1._43=0;				mat1._44=1;
	
	mult(mat2,mat1);
}

void matrix4::setRotationX(m_real A)
{	
#ifdef USE_D3DFUNC
	D3DXMATRIX dxmat;
	D3DXMatrixRotationX(&dxmat,A);
	fromDXmat(dxmat);
#else
	setValue( 1,  0,       0      ,
		0 , cos(A), -sin(A) ,
		0 , sin(A) , cos(A) );
#endif
}

void matrix4::setRotationY(m_real A)
{
#ifdef USE_D3DFUNC
	D3DXMATRIX dxmat;
	D3DXMatrixRotationY(&dxmat,A);
	fromDXmat(dxmat);
#else
	setValue(cos(A),  0,  sin(A) ,
			0      , 1,   0      ,
			-sin(A) , 0 ,  cos(A) );
#endif
}

void matrix4::setRotationZ(m_real A)
{	
#ifdef USE_D3DFUNC
	D3DXMATRIX dxmat;
	D3DXMatrixRotationZ(&dxmat,A);
	fromDXmat(dxmat);
#else
	setValue(cos(A),  -sin(A) ,  0   ,
			sin(A),   cos(A) ,  0   ,
			0     ,   0      ,  1   );
#endif
}

void matrix4::setRotation(const char* aChannel, m_real *aValue, bool bRightToLeft)
{
	//!< from euler angle. aChannel="YXZ" or something like that.

	matrix4 matTemp;
	matrix4& matRot=*this;
	matRot.setIdentityRot();
	for(int i=0; i<3; i++)
	{
		switch(aChannel[i])
		{
		case 'X':
			matTemp.setRotationX(aValue[i]);
			break;
		case 'Y':
			matTemp.setRotationY(aValue[i]);
			break;
		case 'Z':
			matTemp.setRotationZ(aValue[i]);
			break;
		default:
			ASSERT(0);
			continue;
		}

		if(bRightToLeft)
			matRot.mult(matTemp, matRot);
		else
			matRot.mult(matRot, matTemp);
			
	}
}

void matrix4::adjoint(const matrix4& a) 
{
	setValue( a.minor( 1, 2, 3, 1, 2, 3),
        -a.minor( 0, 2, 3, 1, 2, 3),
        a.minor( 0, 1, 3, 1, 2, 3),
        -a.minor( 0, 1, 2, 1, 2, 3),

        -a.minor( 1, 2, 3, 0, 2, 3),
        a.minor( 0, 2, 3, 0, 2, 3),
        -a.minor( 0, 1, 3, 0, 2, 3),
        a.minor( 0, 1, 2, 0, 2, 3),

        a.minor( 1, 2, 3, 0, 1, 3),
        -a.minor( 0, 2, 3, 0, 1, 3),
        a.minor( 0, 1, 3, 0, 1, 3),
        -a.minor( 0, 1, 2, 0, 1, 3),

        -a.minor( 1, 2, 3, 0, 1, 2),
        a.minor( 0, 2, 3, 0, 1, 2),
        -a.minor( 0, 1, 3, 0, 1, 2),
        a.minor( 0, 1, 2, 0, 1, 2));
}


m_real matrix4::determinant() const
{
    return m[0][0] * minor(1, 2, 3, 1, 2, 3) -
        m[0][1] * minor(1, 2, 3, 0, 2, 3) +
        m[0][2] * minor(1, 2, 3, 0, 1, 3) -
        m[0][3] * minor(1, 2, 3, 0, 1, 2);
}

void matrix4::leftMult(m_real scalar)
{
	m[0][0]*=scalar;
	m[1][0]*=scalar;
	m[2][0]*=scalar;
	m[3][0]*=scalar;
	m[0][1]*=scalar; 
	m[1][1]*=scalar; 
	m[2][1]*=scalar; 
	m[3][1]*=scalar; 
	m[0][2]*=scalar; 
	m[1][2]*=scalar; 
	m[2][2]*=scalar; 
	m[3][2]*=scalar; 
	m[0][3]*=scalar;
	m[1][3]*=scalar;
	m[2][3]*=scalar;
	m[3][3]*=scalar;
}

void matrix4::setTransform(const vector3& position, const vector3& scale, const quater& orientation)
{
    // Ordering:
    //    1. Scale
    //    2. Rotate
    //    3. Translate


    matrix4 rot3x3, scale3x3;
	rot3x3.setRotation(orientation);
	scale3x3.setScaling(scale.x, scale.y, scale.z);

    // Set up final matrix with scale, rotation and translation
	this->mult(rot3x3 , scale3x3);
	this->setTranslation(position);
}

m_real matrix4::minor(const size_t r0, const size_t r1, const size_t r2, 
								const size_t c0, const size_t c1, const size_t c2) const
{
    return m[r0][c0] * (m[r1][c1] * m[r2][c2] - m[r2][c1] * m[r1][c2]) -
        m[r0][c1] * (m[r1][c0] * m[r2][c2] - m[r2][c0] * m[r1][c2]) +
        m[r0][c2] * (m[r1][c0] * m[r2][c1] - m[r2][c0] * m[r1][c1]);
}
std::ostream& operator<<(std::ostream& os, const matrix4& arg)
{

	std::cout << "Matrix" << std::endl;

	std::cout << "{" << std::endl;

	for (int i = 0; i < 4; ++i)

	{
		std::cout << "  ";
		for(int j=0; j<4; ++j)
		{ 
			std::cout << arg.m[i][j];
			std::cout << ", ";
		}
		std::cout << std::endl;
	}

	std::cout << "}" << std::endl;

	return os;
}

void matrix4::setProjection(m_real fovx, m_real fovy, m_real Z_near, m_real Z_far)
{
	identity();
	_11=atan(1.5*fovx); // 2*near/(right - left)
	_22=atan(0.5*fovy); // 2*near/(top -bottom)
	_33=-(Z_far+Z_near)/(Z_far-Z_near);
	_34=-2.0*(Z_near*Z_far)/(Z_far-Z_near);
	_43=-1.0;
	_44=0.0;
}
