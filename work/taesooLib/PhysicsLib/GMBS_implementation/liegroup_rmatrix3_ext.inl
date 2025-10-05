inline void Ad(RMatrix &re, const SE3 &T, const RMatrix &S)
{
	re.ReNew(6, S.ColSize());
	Ad(re.GetPtr(), T, S.GetPtr(), S.ColSize());
}

inline void minus_Ad(RMatrix &re, const SE3 &T, const RMatrix &S)
{
	re.ReNew(6, S.ColSize());
	minus_Ad(re.GetPtr(), T, S.GetPtr(), S.ColSize());
}

inline void dAd(RMatrix &re, const SE3 &T, const RMatrix &S)
{
	re.ReNew(6, S.ColSize());
	dAd(re.GetPtr(), T, S.GetPtr(), S.ColSize());
}

inline void minus_dAd(RMatrix &re, const SE3 &T, const RMatrix &S)
{
	re.ReNew(6, S.ColSize());
	minus_dAd(re.GetPtr(), T, S.GetPtr(), S.ColSize());
}

inline void multAB(double *re, const double *a, const double *b, int a_row, int a_col, int b_row, int b_col)
{
	int i, bc = b_col, k, ar;
	const double *tmpa, *tmpb = b;
	double sum;
	while ( bc-- )
	{
		ar = a_row;
		i = 0;
		while ( ar-- )
		{
			tmpa = a + (i++);
			sum = 0.0;
			k = a_col;
			while ( k-- )
			{
				sum += *tmpa * *tmpb;
				tmpa += a_row;
				tmpb++;
			}
			tmpb -= b_row;
			*(re++) = sum;				
		}
		tmpb += b_row;
	}
}

//inline void multASAt(double *re, const double *a, const double *s, int a_row, int a_col, int s_row)
//{
//	int i,j,k,l;
//	double sum;
//
//	for (i=0; i<a_row; i++) {
//		for (j=i; j<a_row; j++) {
//			sum = 0.0;
//			for (k=0; k<s_row; k++) {
//				sum += s[k+k*s_row] * a[i+k*a_row] * a[j+k*a_row];
//				for (l=k+1; l<s_row; l++) {
//					sum += s[k+l*s_row] * ( a[i+k*a_row] * a[j+l*a_row] + a[i+l*a_row] * a[j+k*a_row] );
//				}
//			}
//			re[i+j*a_row] = sum;
//		}
//	}
//	for (i=0; i<a_row; i++) {
//		for (j=0; j<i; j++) {
//			re[i+j*a_row] = re[j+i*a_row];
//		}
//	}
//}

inline RMatrix Ad(const SE3 &T, const RMatrix &J)
{
	se3 S;
	RMatrix re(J.RowSize(), J.ColSize());
	for (int i=0; i<J.ColSize(); i++)
	{
		S = se3(J[6*i], J[6*i+1], J[6*i+2], J[6*i+3], J[6*i+4], J[6*i+5]);
		put_se3_to_matrix(re, Ad(T,S), 6*i);
	}
	return re;
}

inline RMatrix dAd(const SE3 &T, const RMatrix &J)
{
	dse3 S;
	RMatrix re(J.RowSize(), J.ColSize());
	for (int i=0; i<J.ColSize(); i++)
	{
		S = dse3(J[6*i], J[6*i+1], J[6*i+2], J[6*i+3], J[6*i+4], J[6*i+5]);
		put_dse3_to_matrix(re, dAd(T,S), 6*i);
	}
	return re;
}

inline RMatrix ad(const se3 &S, const RMatrix &J)
{
	se3 S2;
	RMatrix re(J.RowSize(), J.ColSize());
	for (int i=0; i<J.ColSize(); i++)
	{
		S2 = se3(J[6*i], J[6*i+1], J[6*i+2], J[6*i+3], J[6*i+4], J[6*i+5]);
		put_se3_to_matrix(re, ad(S,S2), 6*i);
	}
	return re;
}

inline RMatrix ad(const RMatrix &S, const RMatrix &J)
{
	se3 SS(S[0], S[1], S[2], S[3], S[4], S[5]);
	return ad(SS, J);
}

inline RMatrix dad(const se3 &S, const RMatrix &J)
{
	dse3 S2;
	RMatrix JJ(J.RowSize(), J.ColSize());
	for (int i=0; i<J.ColSize(); i++)
	{
		S2 = dse3(J[6*i], J[6*i+1], J[6*i+2], J[6*i+3], J[6*i+4], J[6*i+5]);
		put_dse3_to_matrix(JJ, dad(S,S2), 6*i);
	}
	return JJ;
}

inline RMatrix Cross(const Vec3 &a, const RMatrix &B)
{
	Vec3 b;
	RMatrix re(B.RowSize(), B.ColSize());
	for (int i=0; i<B.ColSize(); i++) {
		b = Vec3(B[3*i], B[3*i+1], B[3*i+2]);
		put_Vec3_to_matrix(re, Cross(a,b), 3*i);
	}
	return re;
}

inline RMatrix convert_to_RMatrix(const Vec3 &p)
{
	RMatrix m(3,1);
	for (int i=0; i<3; i++) m[i] = p[i];
	return m;
}

inline RMatrix convert_to_RMatrix(const SO3 &R)
{
	RMatrix m(3,3);
	for (int i=0; i<9; i++) m[i] = R[i];
	return m;
}

inline RMatrix convert_to_RMatrix(const SE3 &T)
{
	RMatrix m(4,4);
	for (int i=0; i<16; i++) m[i] = T[i];
	return m;
}

inline RMatrix convert_to_RMatrix(const se3 &s)
{
	RMatrix m(6,1);
	for (int i=0; i<6; i++) m[i] = s[i];
	return m; 
}

inline RMatrix convert_to_RMatrix(const dse3 &s)
{
	RMatrix m(6,1);
	for (int i=0; i<6; i++) m[i] = s[i];
	return m; 
}

inline RMatrix convert_to_RMatrix(const Inertia &I)
{
	RMatrix m(6,6);	

	I.ToArray(m.GetPtr());

	return m;
}

inline RMatrix convert_to_RMatrix(const AInertia &aI)
{
	RMatrix m(6,6);

	aI.ToArray(m.GetPtr());

	return m;
}

inline se3 convert_to_se3(const RMatrix &s)
{
	if ( s.RowSize() * s.ColSize() != 6 ) return se3(0,0,0,0,0,0);
	return se3(s[0], s[1], s[2], s[3], s[4], s[5]);
}

inline dse3 convert_to_dse3(const RMatrix &m)
{
	if ( m.RowSize() * m.ColSize() != 6 ) return dse3(0,0,0,0,0,0);
	return dse3(m[0], m[1], m[2], m[3], m[4], m[5]);
}

inline Vec3 convert_to_Vec3(const RMatrix &v)
{
	if ( v.RowSize() * v.ColSize() != 3 ) return Vec3(0, 0, 0);
	return Vec3(v[0], v[1], v[2]);
}

inline RMatrix matrix_skew(const Vec3 &p)
{
	RMatrix m(3,3);

	m(0,0) = 0.0;		m(0,1) = -p[2];		m(0,2) = p[1];
	m(1,0) = p[2];		m(1,1) = 0.0;		m(1,2) = -p[0];
	m(2,0) = -p[1];		m(2,1) = p[0];		m(2,2) = 0.0;

	return m;
}

inline RMatrix matrix_skew_skew(const Vec3 &p)
{
	RMatrix m(3,3);

	m(0,0) = -p[2]*p[2]-p[1]*p[1];		m(0,1) = p[1]*p[0];					m(0,2) = p[2]*p[0];
	m(1,0) = p[1]*p[0];					m(1,1) = -p[2]*p[2]-p[0]*p[0];		m(1,2) = p[2]*p[1];
	m(2,0) = p[2]*p[0];					m(2,1) = p[2]*p[1];					m(2,2) = -p[1]*p[1]-p[0]*p[0];

	return m;
}
