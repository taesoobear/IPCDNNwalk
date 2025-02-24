// written by TAESOO KWON
#include "stdafx.h"
#include "mathclass.h"
#include "quater.h"
#include "vector3.h"
#include "matrix3.h"
#include "Operator_NR.h"
static m_real eps = 1e-5;


void quater::negate(quater const& a)
{
	w=-a.w;
	x=-a.x;
	y=-a.y;
	z=-a.z;
}
quater::quater(const quater& q)
{ memcpy(this,&q,sizeof(quater)); }

void quater::add(quater const& a, quater const& b)
{
	x=a.x+b.x;
	y=a.y+b.y;
	z=a.z+b.z;
	w=a.w+b.w;
}

void quater::subtract(quater const& a, quater const& b)
{
	x=a.x-b.x;
	y=a.y-b.y;
	z=a.z-b.z;
	w=a.w-b.w;
}

void quater::mult(quater const& b, m_real a)
{
	x=a*b.x;
	y=a*b.y;
	z=a*b.z;
	w=a*b.w;
}


m_real quater::operator% (quater const& b)const
{
	quater const& a=*this;
	return a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
}

void quater::mult(quater const& a, quater const& b)
{
#ifdef USE_D3DFUNC
	// important issue: D3DXQuaternionMultiply function outputs the rotation representing the rotation Q1 followed by the rotation Q2 .
	// This is done so that D3DXQuaternionMultiply maintain the same semantics as D3DXMatrixMultiply because unit quaternions can be considered as another way to represent rotation matrices
	// however this quater class is done by definition. So I changed the multiplying order.
	D3DXQuaternionMultiply(*this, b, a);
#else
    w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    y = a.w*b.y + a.y*b.w + a.z*b.x - a.x*b.z;
    z = a.w*b.z + a.z*b.w + a.x*b.y - a.y*b.x;
#endif
}


/*
matrix Quater2Matrix( quater const& q )
{
    matrix m;
    m_real s, xs, ys, zs, wx, wy, wz, xx, xy, xz, yy, yz, zz;

    s  = 2.0/len(q);
    xs = q.x() * s;  ys = q.y() * s;  zs = q.z() * s;
    wx = q.w() * xs; wy = q.w() * ys; wz = q.w() * zs;
    xx = q.x() * xs; xy = q.x() * ys; xz = q.x() * zs;
    yy = q.y() * ys; yz = q.y() * zs; zz = q.z() * zs;

    m.p[0][0] = 1.0 - (yy + zz);
    m.p[1][0] = xy - wz;
    m.p[2][0] = xz + wy;
    m.p[0][1] = xy + wz;
    m.p[1][1] = 1.0 - (xx + zz);
    m.p[2][1] = yz - wx;
    m.p[0][2] = xz - wy;
    m.p[1][2] = yz + wx;
    m.p[2][2] = 1.0 - (xx + yy);

    return m;
}

quater Matrix2Quater( matrix const& m )
{
    quater q;

    m_real tr, s;
    int    i, j, k;
    static int next[3] = { 1, 2, 0 };

    tr = m.getValue(0,0) + m.getValue(1,1) + m.getValue(2,2);
    if ( tr > 0.0 )
    {
        s = sqrt( tr + 1.0 );
        q.p[0] = ( s * 0.5 );
        s = 0.5 / s;
        q.x = ( m.getValue(1,2) - m.getValue(2,1) ) * s;
        q.y = ( m.getValue(2,0) - m.getValue(0,2) ) * s;
        q.z = ( m.getValue(0,1) - m.getValue(1,0) ) * s;
    }
    else
    {
        i = 0;
        if ( m.getValue(1,1) > m.getValue(0,0) ) i = 1;
        if ( m.getValue(2,2) > m.getValue(i,i) ) i = 2;

        j = next[i];
        k = next[j];

        s = sqrt( (m.getValue(i,i)
					- (m.getValue(j,j) + m.getValue(k,k))) + 1.0 );
        q.p[i+1] = s * 0.5;
        s = 0.5 / s;
        q.w   = ( m.getValue(j,k) - m.getValue(k,j) ) * s;
        q.p[j+1] = ( m.getValue(i,j) + m.getValue(j,i) ) * s;
        q.p[k+1] = ( m.getValue(i,k) + m.getValue(k,i) ) * s;
    }

    return q;
}



quater EulerAngle2Quater( vector3 const& v )
{
	m_real x = v.x()/2.0;
	m_real y = v.y()/2.0;
	m_real z = v.z()/2.0;

	return quater( cos(x)*cos(y)*cos(z) + sin(x)*sin(y)*sin(z),
				   sin(x)*cos(y)*cos(z) - sin(x)*sin(y)*sin(z),
				   cos(x)*sin(y)*cos(z) + sin(x)*cos(y)*sin(z),
				   cos(x)*cos(y)*sin(z) - sin(x)*sin(y)*cos(z) );
}

*/

m_real
quater::length() const
{
    return (m_real)sqrt( w*w + x*x + y*y + z*z );
}

void quater::normalize(const quater& a)
{
#ifdef USE_D3DFUNC
	D3DXQuaternionNormalize(*this, a);
#else
	mult(a, 1.0/a.length());
#endif
}


void quater::normalize()
{
	quater c(*this);
	normalize(c);
}

/*
ostream& operator<<( ostream& os, quater const& a )
{
    os << "( " << a.w << " , " << a.x << " , " << a.y << " , " << a.z << " )";
    return os;
}

istream& operator>>( istream& is, quater& a )
{
	static char	buf[256];
	//is >> "(" >> a.p[0] >> "," >> a.x >> "," >> a.y >> "," >> a.z >> ")";
	is >> buf >> a.w >> buf >> a.x >> buf >> a.y >> buf >> a.z >> buf;
    return is;
}
*/
void quater::exp(vector3 const& ww)
{
#ifdef USE_D3DFUNC
	D3DXQuaternionExp(*this, quater(ww,0));
#else
	/*
	Given a pure quaternion defined by:
	q = (0, theta * v); 

	This method calculates the exponential result.
	exp(Q) = (cos(theta), sin(theta) * v)

	*/
	// jehee lee implementation
    m_real theta = (m_real)sqrt(ww % ww);
    m_real sc;

    if(theta < EPS) sc = 1;
    else sc = (m_real)sin(theta) / theta;

    vector3 v = (m_real)sc * ww;
    setValue((m_real)cos(theta), v.x, v.y, v.z);
	
#endif
}

void quater::pow(vector3 const& w, m_real a)
{
    exp((m_real)a * w);
}


void quater::slerp( quater const& a, quater const& b, m_real t )
{
#ifdef USE_D3DFUNC
	D3DXQuaternionSlerp(*this, a, b, t);
#else

	/* ogre implementation doesn't work
	m_real fCos = a%b;
    m_real fAngle ( acos(fCos) );

    if ( ABS(fAngle) < EPS )
        *this=a;

    m_real  fSin = sin(fAngle);
    m_real  fInvSin = 1.0/fSin;
    m_real  fCoeff0 = sin((1.0-t)*fAngle)*fInvSin;
    m_real  fCoeff1 = sin(t*fAngle)*fInvSin;
    
	if (fCos < 0.0f )// && shortestPath)
    {
        fCoeff0 = -fCoeff0;
        add(fCoeff0*a , fCoeff1*b);
		normalize();
    }
    else
    {
        add(fCoeff0*a , fCoeff1*b);
    }
	*/
 
	// jehee lee implementation
	// buggy
	m_real c = a % b;

	if ( 1.0+c > EPS )
	{
		if ( 1.0-c > EPS )
		{
			m_real theta = (m_real)acos( c );
			m_real sinom = (m_real)sin( theta );
			this->mult(( a*(m_real)sin((1-t)*theta) + b*(m_real)sin(t*theta) ), 1.0/ sinom);
		}
		else
			this->normalize((a*(1-t) + b*t));
	}
	else	this->add(a*(m_real)sin((0.5-t)*M_PI) , b*(m_real)sin(t*M_PI));
#endif
}

void quater::safeSlerp( quater const& a, quater const& b, m_real t)
{
	quater ta(a);
	if(( ta%b)<0.0f) ta.negate(ta);
	slerp(ta,b, t);
}

void quater::interpolate( m_real t, quater const& a, quater const& b )
{
	slerp( a, b, t );
}

m_real quater::distance( quater const& b ) const
{
	quater const& a=*this;
	vector3 c,d;
	c.ln( a.inverse()* b);
	d.ln( a.inverse()*-b);
	return MIN( c.length(),
				d.length() );
}

//-----------------------------------------------------------------------------
// Name: QuaternionUnitAxisToUnitAxis2
// Desc: Axis to axis quaternion double angle (no normalization)
//       Takes two points on unit sphere an angle THETA apart, returns
//       quaternion that represents a rotation around cross product by 2*THETA.
//-----------------------------------------------------------------------------
void quater::unitAxisToUnitAxis2(const vector3& vFrom, const vector3& vTo)
{
	vector3 vAxis;
	vAxis.cross(vFrom, vTo);
	setValue(vAxis, vFrom%vTo);
}

//-----------------------------------------------------------------------------
// Name: QuaternionAxisToAxis
// Desc: Axis to axis quaternion 
//       Takes two points on unit sphere an angle THETA apart, returns
//       quaternion that represents a rotation around cross product by theta.
//-----------------------------------------------------------------------------
void quater::axisToAxis( const vector3& vFrom, const vector3& vTo)
{
	vector3 vA, vB, vHalf;
	vA.normalize(vFrom);
	vB.normalize(vTo);
	//singular
	if(vA%vB<-0.999){
		vB*=-1;
		quater q;
		q.axisToAxis(vA, vB);
		// arbitrarily chosen axis (z-axis)
		mult(quater(TO_RADIAN(180), vector3(0,0,1)), q);
		return;
	}else{
		vHalf.add(vA,vB);
		vHalf.normalize();
	}
	unitAxisToUnitAxis2(vA, vHalf);
}

void quater::setRotation(const vector3& axis, m_real angle)
{
#ifdef USE_D3DFUNC
	D3DXQuaternionRotationAxis( *this, axis, angle);
#else
	m_real fHalfAngle=( 0.5*angle );
	m_real fSin = sin(fHalfAngle);
    w = cos(fHalfAngle);
    x = fSin*axis.x;
    y = fSin*axis.y;
    z = fSin*axis.z;
#endif
}

void quater::setRotation(const matrix4& m)
{
#ifdef USE_D3DFUNC
	D3DXMATRIX dxmat;
	m.toDXmat(dxmat);
	D3DXQuaternionRotationMatrix(*this,&dxmat);	
#else
	
	// jehee lee implementation
	quater &q=*this;

    m_real tr, s;
    int    i, j, k;
    static int next[3] = { 1, 2, 0 };

    tr = m.m[0][0] + m.m[1][1] + m.m[2][2];
    if ( tr > 0.0 )
    {
        s = sqrt( tr + 1.0 );
        q[0] = ( s * 0.5 );
        s = 0.5 / s;
        q.x = ( m.m[2][1] - m.m[1][2] ) * s;
        q.y = ( m.m[0][2] - m.m[2][0] ) * s;
        q.z = ( m.m[1][0] - m.m[0][1] ) * s;
    }
    else
    {
        i = 0;
        if ( m.m[1][1] > m.m[0][0] ) i = 1;
        if ( m.m[2][2] > m.m[i][i] ) i = 2;

        j = next[i];
        k = next[j];

        s = sqrt( (m.m[i][i]
					- (m.m[j][j] + m.m[k][k])) + 1.0 );
        q[i+1] = s * 0.5;
        s = 0.5 / s;
        q.w   = ( m.m[k][j] - m.m[j][k] ) * s;
        q[j+1] = ( m.m[j][i] + m.m[i][j] ) * s;
        q[k+1] = ( m.m[k][i] + m.m[i][k] ) * s;
    }


#endif
}
void quater::setRotation(const matrix3& m)
{
	
	// jehee lee implementation
	quater &q=*this;

    m_real tr, s;
    int    i, j, k;
    static int next[3] = { 1, 2, 0 };

    tr = m.m[0][0] + m.m[1][1] + m.m[2][2];
    if ( tr > 0.0 )
    {
        s = sqrt( tr + 1.0 );
        q[0] = ( s * 0.5 );
        s = 0.5 / s;
        q.x = ( m.m[2][1] - m.m[1][2] ) * s;
        q.y = ( m.m[0][2] - m.m[2][0] ) * s;
        q.z = ( m.m[1][0] - m.m[0][1] ) * s;
    }
    else
    {
        i = 0;
        if ( m.m[1][1] > m.m[0][0] ) i = 1;
        if ( m.m[2][2] > m.m[i][i] ) i = 2;

        j = next[i];
        k = next[j];

        s = sqrt( (m.m[i][i]
					- (m.m[j][j] + m.m[k][k])) + 1.0 );
        q[i+1] = s * 0.5;
        s = 0.5 / s;
        q.w   = ( m.m[k][j] - m.m[j][k] ) * s;
        q[j+1] = ( m.m[j][i] + m.m[i][j] ) * s;
        q[k+1] = ( m.m[k][i] + m.m[i][k] ) * s;
    }


}

void quater::setRotation(const char* aChannel, m_real *aValue, bool bRightToLeft)
{
	//!< from euler angle. aChannel="YXZ" or something like that.

	vector3 axis;
	quater temp;
	quater& qRot=*this;
	qRot.setValue(1, 0,0,0);

	int numChannels=strlen(aChannel);
	for(int i=0; i<numChannels; i++)
	{
		switch(aChannel[i])
		{
		case 'X':
		case 'x':
			axis=vector3(1,0,0);
			break;
		case 'Y':
		case 'y':
			axis=vector3(0,1,0);
			break;
		case 'Z':
		case 'z':
			axis=vector3(0,0,1);
			break;
		default:
			Msg::error("unknown channels");
			assert(0);
			continue;
		}

		temp.setRotation(axis, aValue[i]);

//		printf("set %d: %s %f\n", i, temp.output().ptr(), aValue[i]);
		if(bRightToLeft)
			qRot.leftMult(temp);
		else
		{
			quater copy(qRot); qRot.mult(copy, temp);			
		}

	}
}

// R=RZ*RY*RX
vector3 Quater2EulerAngleZYX( quater const& q )
{
	m_real s;
	m_real xs, ys, zs;
	m_real wx, wy, wz;
	m_real xx, xy, xz;
	m_real yy, yz, zz;
	m_real sinx, siny, sinz;
	m_real cosx, cosy, cosz;

	s  = 2.0f / q.length();
	xs = q.x * s;  ys = q.y * s;  zs = q.z * s;
	wx = q.w * xs; wy = q.w * ys; wz = q.w * zs;
	xx = q.x * xs; xy = q.x * ys; xz = q.x * zs;
	yy = q.y * ys; yz = q.y * zs; zz = q.z * zs;

	siny = wy - xz;
	cosy = sqrt( 1.0f - siny*siny );

	if ( cosy>EPS )
	{
		sinx = (yz + wx) / cosy;
		cosx = (1.0f - xx - yy) / cosy;

		sinz = (xy + wz) / cosy;
		cosz = (1.0 - yy - zz) / cosy;
	}
	else
	{
		sinx = wx - yz;
		cosx = 1.0 - xx - zz;

		sinz = 0.0;
		cosz = 1.0;
	}

	m_real x = atan2( sinx, cosx );
	m_real y = atan2( siny, cosy );
	m_real z = atan2( sinz, cosz );

	return vector3(x,y,z);
}

// R=RY*RX
vector3 Quater2EulerAngleYX( quater const& q )
{	
	matrix4 t;
	t.setRotation(q);

	double sinx=-1.0*t.m[1][2];
	double cosx=t.m[1][1];
	double cosy=t.m[0][0];
	double siny=-1.0*t.m[2][0];

	/*
	ASSERT(isSimilar(t.m[1][0],0));
	ASSERT(isSimilar(SQR(cosx)+SQR(sinx), 1.0));
	ASSERT(isSimilar(SQR(cosy)+SQR(siny), 1.0));

	ASSERT(isSimilar(siny*sinx, t.m[0][1]));
	ASSERT(isSimilar(cosx*siny, t.m[0][2]));
	ASSERT(isSimilar(cosy*sinx, t.m[2][1]));
	ASSERT(isSimilar(cosx*cosy, t.m[2][2]));
*/
	m_real x=atan2(sinx, cosx);
	m_real y=atan2(siny, cosy);

	return vector3(x,y,0);
}

inline vector3 Quater2EulerAngleZYX( quater const& q , int numChannels)
{
	switch(numChannels)
	{
	case 1:
	case 2:
		return Quater2EulerAngleYX(q);
	}
	return Quater2EulerAngleZYX(q);
}

// refactoring needed
m_real determinants(const matrix4& mat)
{
	return mat._11*mat._22*mat._33+mat._13*mat._21*mat._32+mat._12*mat._23*mat._31
		-mat._13*mat._22*mat._31-mat._11*mat._23*mat._32-mat._12*mat._21*mat._33;
}

void quater::getRotation(const char* aChannell, m_real *aValue, bool bRightToLeft) const
{
	//!< to euler angle. aChannel="YXZ" or something like that.
	vector3 euler;

	char aChannel[3];

	int numChannels=strlen(aChannell);

	if(bRightToLeft)
	{
		for(int i=0; i<numChannels; i++)
		{
			aChannel[i]=aChannell[i];
		}
	}
	else
	{
		for(int i=0; i<numChannels; i++)
		{
			aChannel[i]=aChannell[numChannels-i-1];
		}
	}

	for(int i=numChannels; i<3; i++)
	{
		aChannel[i]='X';

		for(int k=0; k<2; k++)
		{
			for(int j=0; j<i; j++)
				if(aChannel[i]==aChannel[j]) aChannel[i]++;		
		}
	}

	if(strncmp(aChannel, "XYZ", 3)==0 )
	{
		// R=RZ*RY*RX

		euler=Quater2EulerAngleZYX(*this, numChannels);
	}
	else 
	{
		matrix4 matT, matInvT;
		matT.setValue(0,0,0, 0,0,0, 0,0,0);
		for(int i=0; i<3; i++)
		{
			switch(aChannel[i])
			{
			case 'X':
				matT.m[i][0]=1.0;		
				break;
			case 'Y':
				matT.m[i][1]=1.0;
				break;
			case 'Z':
				matT.m[i][2]=1.0;
				break;
			default:
				assert(0);
				continue;
			}

		}
		matInvT.transpose(matT);

		matrix4 matRot2;
		matRot2.mult(matT, *this);
		matRot2.mult(matRot2, matInvT);

		quater q;
		q.setRotation(matRot2);
		euler=Quater2EulerAngleZYX(q, numChannels);
		//printf("det %f\n", determinants(matT));
		euler*=determinants(matT);
	}

	if(bRightToLeft)
	{
		aValue[0]=euler.x;
		aValue[1]=euler.y;
		aValue[2]=euler.z;
	}
	else
	{
		if(numChannels==3)
		{
			aValue[0]=euler.z;
			aValue[1]=euler.y;
			aValue[2]=euler.x;
		}
		else if(numChannels==2)
		{
			ASSERT(isSimilar(euler.z,0));
			aValue[0]=euler.y;
			aValue[1]=euler.x;
			aValue[2]=0.0;
		}
		else if(numChannels==1)
		{
			ASSERT(isSimilar(euler.z,0));
//			ASSERT(isSimilar(euler.y,0));
			aValue[0]=euler.x;
			aValue[1]=0.0;
			aValue[2]=0.0;
		}
	}

#ifdef _DEBUG

	quater qtest;
	qtest.setRotation(aChannell, aValue, bRightToLeft);

	qtest.align(*this);
	//ASSERT(isSimilar(qtest%*this, 1.0));
#endif
	//	printf("get %f %f %f\n", aValue[0], aValue[1], aValue[2]);

	// testing codes
/*	quater q(0.813, -0.368, 0.228, 0.387);
	q.normalize();
	quater qtest;
	m_real aValue[3];

	q.getRotation("YZX", aValue, true);
	qtest.setRotation("YZX", aValue, true);

	printf("error %f %s\n", TO_DEGREE(qtest.distance(q)), ((quater&)q).output().ptr());

	qtest.getRotation("YZX", aValue, true);
	q.setRotation("YZX", aValue, true);

	printf("error %f %s\n", TO_DEGREE(qtest.distance(q)), ((quater&)q).output().ptr());
*/

}	

void quater::setAxisRotation(const vector3& vecAxis, const vector3& front, const vector3& vecTarget)
{
	matrix4 temp;
	temp.setAxisRotation(vecAxis, front, vecTarget);
	this->setRotation(temp);
}

void quater::toAxisAngle(vector3& axis, m_real& angle) const
{
/*

#ifdef USE_D3DFUNC
	buggy.-.-
	D3DXQuaternionToAxisAngle(*this, axis, &angle);	
#else
	*/
	vector3 log;
	log.ln(*this);
	axis.normalize(log);
	angle=log.length()*2.0;

//#endif
}

void cumQTQ(matrixn& p, const vectorn& q, m_real w)
{
	p[0][0]+=w*q[0]*q[0];
	p[0][1]+=w*q[0]*q[1];
	p[0][2]+=w*q[0]*q[2];
	p[0][3]+=w*q[0]*q[3];
	p[1][0]+=w*q[1]*q[0];
	p[1][1]+=w*q[1]*q[1];
	p[1][2]+=w*q[1]*q[2];
	p[1][3]+=w*q[1]*q[3];
	p[2][0]+=w*q[2]*q[0];
	p[2][1]+=w*q[2]*q[1];
	p[2][2]+=w*q[2]*q[2];
	p[2][3]+=w*q[2]*q[3];
	p[3][0]+=w*q[3]*q[0];
	p[3][1]+=w*q[3]*q[1];
	p[3][2]+=w*q[3]*q[2];
	p[3][3]+=w*q[3]*q[3];
}

void quater::blend(const vectorn& weight, matrixn& q)
{
	int n=weight.size();

	quater& qx=*this;
	matrixn m(4,4);
	m.setAllValue(0);

	for( int i=0; i<n; i++ )
		cumQTQ(m, q.row(i), weight[i] );

	matrixn qstar;
	vectorn eigenV;
	m::eigenVectors(qstar, m, eigenV);
	
	for(int i=0; i<4; i++)
		qstar.row(i).normalize();

	// find the best quaternion from these eigen vectors
	vectorn SSD(4);	// sum of squared distance
	SSD.setAllValue(0);
	m_real t;
	for(int j=0; j<4; j++)
	{
		for(int i=0; i<n; i++)
		{
			t=qstar.row(j)%q.row(i);
			SSD[j]+=1-t*t;
		}
	}

	qx=qstar.row(SSD.argMin()).toQuater();
}

void quater::difference(quater const& q1, quater const& q22)
{
	quater q2(q22);
	q2.align(q1);

	// delta* q1 = q2 -->
	//-> delta =q2* inv(q1);
	// in other words, delta is as parent-local (as opposed to self-local)

	mult(q2, q1.inverse());    
	normalize();
}

void quater::toLocal(quater const& q1, quater const& q22)
{	
	quater q2(q22);
	q2.align(q1);

	mult(q1.inverse(), q2);	
	normalize();
}

void quater::derivative(quater const& q1, quater const& q2)
{
	//!< q'(t)=(1/2) w q; or q'(t)=q2-q1; -> both produces almost the same result when q1 and q2 are very similar.

	quater& out=*this;

	// numerical solution
	//out=q2-q1; 

	// analytic solution
	vector3 temp;
	temp.angularVelocity(q1, q2);
	out.mult(quater(temp), q1);
	out/=2.0;
}

/*
extern double combinationR(int n, int k);
m_real powR(m_real x, int k)
{
	m_real r=1;
	for(int i=0; i<k; i++)
		r*=x;
	return r;
}


void quater::bezier(matrixn aq, m_real t)
{
	const int n=3;
	m_real b[4];
	
	const int n=3;
	m_real b[4];

	for(int i=0; i<=n; i++)
	{
		b[i]=0;
		for(int j=i; j<=n; j++)
			b[i]+=combinationR(n,j)*powR(1-t, n-j)*powR(t, j);
	}

	*this=...
}*/

void quater::bezier(const quater& q0, const quater& q1, const quater& q2, const quater& q3, m_real t)
{
	// kim myung soo siggraph 96 paper
	// cumulative bases

	m_real b1, b2, b3;
	b1=1.0-CUBIC(1.0-t);
	b2=3.0*SQR(t)-2.0*CUBIC(t);
	b3=CUBIC(t);

	// This paper uses local angularVelocity w.
	// Furthermore, its length is twice the common definition. (This means that this paper is actually wrong.)
	// I modified it to use world angular velocity.
	// ->
	// q(t)=( PI_n^1 exp(2*wi*bi) ) * q0

	vector3 w1, w2, w3;
    w1.angularVelocity(q0, q1);
	w2.angularVelocity(q1, q2);
	w3.angularVelocity(q2, q3);

	this->mult( (w3*b3).quaternion()*(w2*b2).quaternion()*(w1*b1).quaternion() , q0);
	this->normalize();
}

void quater::hermite(const quater& qa, const vector3& wa, const quater& qb, const vector3& wb, m_real t)
{
	quater q0, q1, q2, q3;

	// p0=pa, p1=pa+va/3, p2=pb-vb/3, p3=pb
	// -> quaternion으로 바꾸면..	
	q0=qa;
	q1.mult((wa/3.0).quaternion(), qa);
	q2.mult((wb/3.0).quaternion().inverse(), qb);
	q3=qb;

	bezier(q0, q1, q2, q3, t);
}

void quater::scale(m_real s)
{
	vector3 rot;
	rot.rotationVector(*this);
	rot*=s;
	setRotation(rot);
}

void quater::decompose(quater& rotAxis_y, quater& offset) const
{
	/* buggy: vector3 axis(0,1,0);
	vector3 localFront(0,0,1);
	vector3 globalFront;

	globalFront.rotate(*this, localFront);
	rotAxis_y.setAxisRotation(axis, localFront, globalFront);
	//*this=rotAxis_y*offset;
	offset=rotAxis_y.inverse()*(*this);*/

    decomposeTwistTimesNoTwist(vector3(0,1,0), rotAxis_y, offset);
}

void quater::decomposeTwistTimesNoTwist (const vector3& rkAxis, quater& rkTwist, quater& rkNoTwist) const
{
	decomposeNoTwistTimesTwist(rkAxis, rkNoTwist, rkTwist);
	//*this=rkTwist*rkNoTwist;
	rkNoTwist.mult(rkTwist.inverse(), *this);

//	printf("A:%s BC:%s\n", rkTwist.output(true).ptr(), rkNoTwist.output(true).ptr());	
}

void quater::decomposeNoTwistTimesTwist (const vector3& rkAxis, quater& rkNoTwist, quater& rkTwist) const
{
	vector3 kRotatedAxis;
	kRotatedAxis.rotate((*this),rkAxis);
	rkNoTwist.axisToAxis(rkAxis,kRotatedAxis);	
	rkTwist.mult(rkNoTwist.inverse(),*this);
}

m_real quater::rotationAngleAboutAxis(const vector3& axis) const
{
	quater qoff, qaxis;
	decomposeNoTwistTimesTwist(axis, qoff, qaxis);
	return qaxis.rotationAngle(axis);
}

std::string quater::output(bool bRotationVector)
{	
	if(bRotationVector)
	{
		vector3 rv;
		rv.rotationVector(*this);
		return rv.output();
	}

	TString temp;
	temp.format("(%.10f %.10f %.10f %.10f)", w, x, y, z);
	return std::string(temp.ptr());
}
std::ostream &operator << (std::ostream &os, const quater &v)
{
	//os.setf(ios::scientific);
	//os.precision(8);
	os << "[ ";
	for ( int i = 0; i < 4; i++ )
	{	
		if ( v[i] >= 0.0 ) os << " ";
		os << v[i] << " ";
	}
	os << "];" << std::endl;
    return os;
}

vector3 quater::getFrameAxis(int icolumn) const
{
	vector3 v(0.0, 0.0, 0.0);
	v[icolumn]=1.0;
	return (*this)*v;
}

void quater::setFrameAxesYZ(vector3 const& _y, vector3 const& _z)
{
	vector3 nx, ny, nz;
	nx.cross(_y,_z);
	nx.normalize();
	ny.cross(_z,nx);
	ny.normalize();
	nz.cross(nx,ny);


	matrix3 m;
	m.setValue(
			nx.x, ny.x, nz.x,
			nx.y, ny.y, nz.y,
			nx.z, ny.z, nz.z);
	setRotation(m);
}
