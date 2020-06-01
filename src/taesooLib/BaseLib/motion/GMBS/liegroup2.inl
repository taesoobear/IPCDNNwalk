#ifndef LIEGROUP_T_INL
#define LIEGROUP_T_INL
inline se3 to_se3(::vectorn const& v, int start=0)
{
	return se3(v(start+0), v(start+1), v(start+2),v(start+3), v(start+4), v(start+5)); 
}
inline dse3 to_dse3(::vectorn const& v, int start=0)
{
	return dse3(v(start+0), v(start+1), v(start+2),v(start+3), v(start+4), v(start+5)); 
}
inline void radd(::vectorn & v, se3 const& vv)
{
	for (int i=0; i<6; i++)
		v(i)+=vv[i];
}
inline void radd(::vectorn & v, dse3 const& vv)
{
	for (int i=0; i<6; i++)
		v(i)+=vv[i];
}
inline void assign(::vectorn & v, dse3 const& vv)
{
	for (int i=0; i<6; i++)
		v(i)=vv[i];
}

inline Vec3 toVec3(::vectorn const& v, int startI)
{
	return Vec3(v(startI), v(startI+1), v(startI+2));
}


inline Vec3 toGMBS(::vector3 const& v)
{
	return Vec3 (v.x, v.y, v.z);
}
inline SE3 toGMBS(matrix4 const& v)
{
	SE3 out;
	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			out(i,j)=v.m[i][j];
	return out;
}

static inline se3 mult(SO3 const& r, se3 const& in)
{
	return se3(r*in.GetW(), r*in.GetV());
}
static inline dse3 mult(SO3 const& r, dse3 const& in)
{
	return dse3(r*GetM(in), r*GetF(in));
}
static inline double dot(double* a, dse3 const& b)
{
	double out=0;
	for (int i=0; i<6; i++)
		out+=a[i]*b[i];
	return out;
}
static inline dse3 mult(matrixn const& in, dse3 const& in2)
{
	dse3 out;
	for (int i=0; i<6; i++)
	{
		out[i]=dot(&in(i,0), in2);
	}	
	return out;
}
inline SO3 skew(Vec3 const& w)
{
	return SO3( 0, w.z, -w.y, -w.z,0, w.x, w.y, -w.x,0);
}
inline void skew(matrixn& out, Vec3 const& w)
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
inline SE3 toGMBS(::quater q, ::vector3 pos)
{
	SE3 out;
	m_real s, xs, ys, zs, wx, wy, wz, xx, xy, xz, yy, yz, zz;

    s  = 2.0/q.length();
    xs = q.x * s;  ys = q.y * s;  zs = q.z * s;
    wx = q.w * xs; wy = q.w * ys; wz = q.w * zs;
    xx = q.x * xs; xy = q.x * ys; xz = q.x * zs;
    yy = q.y * ys; yz = q.y * zs; zz = q.z * zs;

    out(0,0)= 1.0 - (yy + zz);
    out(0,1)= xy - wz;
    out(0,2)= xz + wy;
    out(1,0)= xy + wz;
    out(1,1)= 1.0 - (xx + zz);
    out(1,2)= yz - wx;
    out(2,0)= xz - wy;
    out(2,1)= yz + wx;
    out(2,2)= 1.0 - (xx + yy);
	
	out(0,3)=pos.x;
	out(1,3)=pos.y;
	out(2,3)=pos.z;

	return out;
}
inline ::vector3 toBase(Vec3 const& v)
{
	return ::vector3(v[0], v[1], v[2]);
}
template <class SE3>
inline void assign33(::matrixn & a, SE3 const& b)
{
	for (int i=0; i<3; i++)
		for (int j=0; j<3;j++)
			a(i,j)=b(i,j);
}	
inline void assign33(matrix4 & a, SE3 const& b)
{
	for (int i=0; i<3; i++)
		for (int j=0; j<3;j++)
			a.m[i][j]=b(i,j);
}

template <class SE3>
inline void assign33T(::matrixn & a, SE3 const& b)
{
	for (int i=0; i<3; i++)
		for (int j=0; j<3;j++)
			a(i,j)=b(j,i);
}	
inline void dAd(::matrixn& a, SE3 const& b)
{
	assign33T(a.range(0,3,0,3).lval(),b);
	assign33T(a.range(3,6,3,6).lval(),b);
	a.range(3,6,0,3).setAllValue(0);
	matrixnView skew_b_T=a.range(3,6,0,3);
	skew(skew_b_T, b.GetPosition()*-1);
	a.range(0,3,3,6).mult(a.range(0,3,0,3),skew_b_T);
	skew_b_T.setAllValue(0);
}
inline matrix4 calcDotT(SE3 const& b, se3 const& V)// V=inv_b*dot_b
{
	// b= (R p) 
	//    (0 1)
	//    (R p)* (invR  -invR*p) = I    
	//    (0 1)  ( 0        1)
	//         V=(skew(w) v)=(invR  -invR*p)*(dotR  dotP)
	//           ( 0      0) ( 0        1  ) ( 0     0  )
	//
	SO3 R=b.GetRotation();
	Vec3 P=b.GetPosition();
	// invR*dotR=skew(w)
	SO3 dotR=R*skew(V.GetW());
	// v=invR*dotP
	Vec3 dotP=R*V.GetV();
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
inline static SO3 add(SO3 const& a, SO3 const& b)
{
	SO3 out;
	for(int i=0;i<3;i++)
		for(int j=0; j<3; j++)
			out(i,j)=a(i,j)+b(i,j);
	return out;
}
inline static SE3 add(SE3 const& a, SE3 const& b)
{
	SE3 out;
	for(int i=0;i<4;i++)
		for(int j=0; j<4; j++)
			out(i,j)=a(i,j)+b(i,j);
	return out;
}
inline static SE3 sub(SE3 const& a, SE3 const& b)
{
	SE3 out;
	for(int i=0;i<4;i++)
		for(int j=0; j<4; j++)
			out(i,j)=a(i,j)-b(i,j);
	return out;
}
inline void dot_dAd(::matrixn& out, SE3 const& b, matrix4 const& dotB) // V=inv_b*dot_b
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
	SO3 R=b.GetRotation();
	Vec3 P=b.GetPosition();
	SE3 dotB2=toGMBS(dotB);
	SO3 dotR=dotB2.GetRotation();
	Vec3 dotP=dotB2.GetPosition();

	assign33T(out.range(0,3,0,3).lval(),dotR);
	// d(skew(b)*T)/dt=skew(dotP)*R+skew(p)*dotR
	assign33T(out.range(0,3,3,6).lval(),add(skew(dotP)*R,skew(P)*dotR));
	out.range(3,6,0,3).setAllValue(0);
	assign33T(out.range(3,6,3,6).lval(),dotR);
}

template <class SE3> // works for SE3, SO3 or RMatrix
inline quater toBaseR(SE3 const& m)
{
	/*	matrix4 out;
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			out.m[i][j]=m(i,j);

	quater q;
	q.setRotation(out);*/

	quater q;
	m_real tr, s;
    int    i, j, k;
    static int next[3] = { 1, 2, 0 };

    tr = m(0,0) + m(1,1) + m(2,2);
    if ( tr > 0.0 )
    {
        s = sqrt( tr + 1.0 );
        q[0] = ( s * 0.5 );
        s = 0.5 / s;
        q.x = ( m(2,1) - m(1,2) ) * s;
        q.y = ( m(0,2) - m(2,0) ) * s;
        q.z = ( m(1,0) - m(0,1) ) * s;
    }
    else
    {
        i = 0;
        if ( m(1,1) > m(0,0) ) i = 1;
        if ( m(2,2) > m(i,i) ) i = 2;

        j = next[i];
        k = next[j];

        s = sqrt( (m(i,i)
				   - (m(j,j) + m(k,k))) + 1.0 );
        q[i+1] = s * 0.5;
        s = 0.5 / s;
        q.w   = ( m(k,j) - m(j,k) ) * s;
        q[j+1] = ( m(j,i) + m(i,j) ) * s;
        q[k+1] = ( m(k,i) + m(i,k) ) * s;
    }

	return q;
}
inline ::matrix4 toBase(SE3 const& v)
{
	return matrix4(toBaseR(v), toBase(v.GetPosition()));
}
#endif
