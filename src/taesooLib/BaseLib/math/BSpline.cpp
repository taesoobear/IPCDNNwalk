#include "stdafx.h"
#include "mathclass.h"
//#ifdef _DEBUG
//#define new DEBUG_NEW
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
//#endif

#include "BSpline.h"

static void _tridag(const vectorn& a, const vectorn& b, const vectorn& c, const vectorn& r, vectorn& u)
{
	int j;
	m_real bet;

	int n=a.size();
	u.setSize(n);

	vectorn gam(n);	// one vector of workspace
	if(b[0]==0.0) ASSERT(0);
	// if this happens then you should rewrite your equations as a set of order N-1, with u1 trivially eliminated
	u[0]=r[0]/(bet=b[0]);

	// decomposition and forward substitution
	for(j=1; j<n; j++)
	{
		gam[j]=c[j-1]/bet;
		bet=b[j]-a[j]*gam[j];
		if(bet==0.0) ASSERT(0);	// algorithm fails
		u[j]=(r[j]-a[j]*u[j-1])/bet;
	}

	// backsubstitution
	for(j=n-2; j>=0; j--)
		u[j]-=gam[j+1]*u[j+1];
}
UniformSpline::UniformSpline(const matrixn& points, int start, int end)
{
	//Implementing http://mathworld.wolfram.com/CubicSpline.html by taesoo

	// Y(t)=a+bt+ct^2+dt^3

	if (start < 0) start=0;
	if (end > points.rows()) end=points.rows();

	intvectorn rows;
	rows.colon(start, end);
	//"colon" function is implemented in "taesooLib/BaseLib/math/vectorn.cpp"
	//"colon" : set vector size n=(end-start), set value [0]=start, [1]=start+1 ..., [n]=end
	y.extractRows(points, rows);
	//"extractRows" function is implemented in "taesooLib/BaseLib/math/template_matrix.h"
	//"extractRows" : copy "points" matrix to "y" from start row to end row (every columns)

	dim = y.cols();
	n = y.rows()-1;
	DT.setSize(dim,n+1); // DT size = y size

	// tridM is 3*n matrix
	tridM[0].setSize(n+1);
	tridM[1].setSize(n+1);
	tridM[2].setSize(n+1);
	tridM[0].setAllValue(1.f);
	tridM[1].setAllValue(4.f);
	tridM[1][0]=2.f;
	tridM[1][n]=2.f;
	tridM[2].setAllValue(1.f);

	vectorn r(n+1);
	for(int i=0; i<dim; i++) // i count columns
	{
		for(int j=1; j<n; j++)
			r[j] = 3.f * (y[j+1][i] - y[j-1][i]);		
		r[0] = 3.f * (y[1][i] - y[0][i]);
		r[n] = 3.f * (y[n][i] - y[n-1][i]);

        	vectornView row_i = DT.row(i);
		_tridag (tridM[0], tridM[1], tridM[2], r, row_i);
	}
}

void UniformSpline::getCurve(matrixn& points, int ndiv)
{
	m_real a, b, c, d, t;
	//n = points.rows() - 1
	points.setSize(n*ndiv+1,dim);
	for(int j=0; j<dim; j++)
	{
		for(int i=0; i<n; i++)
		{
			a = y[i][j];
			b = DT[j][i];
			c = 3.f * (y[i+1][j] - y[i][j]) - 2.f * DT[j][i] - DT[j][i+1];
			d = 2.f * (y[i][j] - y[i+1][j]) + DT[j][i] + DT[j][i+1];

			for(int k=0; k<ndiv; k++)
			{
				t=(m_real)k/(m_real)ndiv;

				points[i*ndiv+k][j]=a+b*t+c*SQR(t)+d*CUBIC(t);
			}
		}
		t=1.f;
		points[n*ndiv][j] = a + b * t + c * SQR(t) + d * CUBIC(t);
	}
}

void UniformSpline::getSecondDeriv(matrixn& points)
{
	m_real a, b, c, d, t;

	points.setSize(n+1, dim);
	for(int j=0; j<dim; j++)
	{
		int i;
		for(i=0; i<n; i++)
		{
			a = y[i][j];
			b = DT[j][i];
			c = 3.f * (y[i+1][j] - y[i][j]) - 2.f * DT[j][i] - DT[j][i+1];
			d = 2.f * (y[i][j] - y[i+1][j]) + DT[j][i] + DT[j][i+1];

			//first derivative:			points[i][j]=b+2.f*c*t+3*d*t*t;
			// second derivative:

			t = 0.f;
			points[i][j] = 2.f*c + 6.f * d * t;
		}
		t = 1.f;
		points[i][j] = 2.f * c + 6.f * d * t;
	}
}

NonuniformSpline::NonuniformSpline(vectorn const& pkeytime, const matrixn& points, const boundaryCondition &bc0, const boundaryCondition &bcn)
{
	__init(pkeytime, points, bc0, bcn);
}
NonuniformSpline::NonuniformSpline(vectorn const& keytime, const matrixn& controlpoints,NonuniformSpline:: boundaryCondition::bcT bc)
{
	boundaryCondition bc0(bc);
	boundaryCondition bc1(bc);
	__init(keytime, controlpoints, bc0, bc1);
}
NonuniformSpline::NonuniformSpline(vectorn const& keytime, const matrixn& controlpoints)
{
	__init(keytime, controlpoints, zeroAcc(), zeroAcc());
}

void NonuniformSpline::__init(vectorn const& pkeytime, const matrixn& points, const boundaryCondition &bc0, const boundaryCondition &bcn)
{
	//Adapting http://mathworld.wolfram.com/CubicSpline.html by taesoo

	// Y(t)=a+bt+ct^2+dt^3

	keytime=pkeytime;
	y=points;
	dim=y.cols();
	n=y.rows()-1;

	len.setSize(n);

	for(int i=0; i<n; i++)
		len[i]=keytime[i+1]-keytime[i];


	// let ni be len[i]
	// Yi(t)=ai+bi*t+ci*t^2+di*t^3, 0<=t<ni

	// from c0 continuity
	// yi=ai										- (1)
	// y{i+1}=ai+bi*ni+ci*ni^2+di*ni^3				- (2)

	// from c1 continuity
	// Di=bi										- (3)
	// D{i+1}=bi+2ci*ni+3di*ni^2					- (4)

	// from above equations
	// From (2)*2-(4)*ni,
	// 2*y{i+1} - ni*D{i+1}=2*yi+ni*Di - di*ni^3

	// Thus, di=(2*(yi-y{i+1}) + ni * (Di+D{i+1}))/(ni^3)

	// From (4)
	// D{i+1}-Di= 2ci*ni+ 6*(yi-y{i+1})/ni  + 3*(Di+D{i+1}))

	// Thus, ci= (3*(y{i+1}-yi)/ni -2*Di -D{i+1})/ni

	////////////////////////////////////////////////
	// boundary conditions
	///////////////////////////////////////////////

	// case1: acceleration=0

	// From Y0''(0)=0 -> c0=0
	// Thus, 2*D0 +D1 =3*(y1-y0)/n0


	// From Yn-1''(1)=0 -> c{n-1}+3*d{n-1}*n{n-1}=0
	// Thus, (3*(yn-y{n-1})/n{n-1} -2*D{n-1} -Dn)/n{n-1}+ 6*(y{n-1}-yn)/(n{n-1}^2)  + 3*(D{n-1}+Dn))/n{n-1} = 0
	// -> (1/n{n-1}, 2*1/n{n-1}) * ( D{n-1}, Dn) =3*(-y{n-1}+yn)/n{n-1}^2

	// case 2: velocity=0

	// From Y0'(0)=0 -> D0=0
	// From Yn-1'(1)=0 -> 	D{n-1}+ 2*c{n-1}*n{n-1}+3*d{n-1}*n{n-1}^2=0
	// Here, d[n-1]=(2*(y{n-1}-yn) + n{n-1} * (D{n-1}+Dn))/((n-1)^3)
	// and c[n-1]=(3*(y{n}-y{n-1})/n{n-1} -2*D{n-1} -Dn)/n{n-1}
	// -> D{n-1}+2*(3*(y{n}-y{n-1})/n{n-1} -2*D{n-1} -Dn) +3*(2*(y{n-1}-yn) + n{n-1} * (D{n-1}+Dn))/n{n-1}=0
	// -> Dn =0

	// case 3: velocity=v

	// From Y0'(0)=v -> D0=v
	// From Yn-1'(1)=0 -> 	D{n-1}+ 2*c{n-1}*n{n-1}+3*d{n-1}*n{n-1}^2=v
	// Here, d[n-1]=(2*(y{n-1}-yn) + n{n-1} * (D{n-1}+Dn))/((n-1)^3)
	// and c[n-1]=(3*(y{n}-y{n-1})/n{n-1} -2*D{n-1} -Dn)/n{n-1}
	// -> D{n-1}+2*(3*(y{n}-y{n-1})/n{n-1} -2*D{n-1} -Dn) +3*(2*(y{n-1}-yn) + n{n-1} * (D{n-1}+Dn))/n{n-1}=v
	// -> Dn =v

	////////////////////////////////////////////////
	// c2 continuity
	///////////////////////////////////////////////

	// From Yi''(1)==Yi+1''(0)
	// -> ci+3*di*ni = c{i+1}

	// Thus, (3*(y{i+1}-yi)/ni -2*Di -D{i+1})/ni+ (6*(yi-y{i+1}) + 3*ni * (Di+D{i+1}))/(ni^2) =
	//                        (3*(y{i+2}-y{i+1})/n{i+1} -2*D{i+1} -D{i+2})/n{i+1}

	// -> (3/n{i+1}^2-3/ni^2) y{i+1} +  (3/ni^2)* yi - (3/n{i+1}^2)*y{i+2}
	//	    +  ( 1/ni  )Di + (2/ni+ 2/n{i+1})*D{i+1} + (1/n{i+1})*D{i+2}


	// 정리하면   ( 1/ni  ,  2/ni+2/n{i+1}  ,  1/n{i+1} ) * (Di, D{i+1}, D{i+2})
	//						= 3 * ( -1/ni^2,  1/ni^2 - 1/n{i+1}^2, 1/n{i+1}^2) * (yi, y{i+1}, y{i+2})

	// tridiagonal system이 만들어졌음.
	//
	// 2/n0      1/n0                                ...      d0        3(   -y0/n0^2 + y1/n0^2                        )
	// 1/n0      2/n0+2/n1   1/n1                             d1        3(   -y0/n0^2 + y1/n0^2-y1/n1^2+  y2/n1^2      )
	//           1/n1        2/n1+2/n2     2/n2				  d2   =                        .
	// ...																					.
	//           1/n{n-2}   1/n{n-2}+1/n{n-1} 2*1/n{n-1}      d{n-1}
	//                           1/n{n-1}     2*1/n{n-1}      dn


	DT.setSize(dim,n+1);

	tridM[0].setSize(n+1);
	tridM[1].setSize(n+1);
	tridM[2].setSize(n+1);

	if(bc0.mType==boundaryCondition::ZERO_ACC)
	{
		tridM[1][0]=2.0/len[0];
		tridM[2][0]=1.0/len[0];
	}
	else	// ZERO_VEL
	{
		tridM[1][0]=1.0;
		tridM[2][0]=0.0;
	}

	for(int i=1; i<n; i++)
	{
		tridM[0][i]=1.0/len[i-1];
		tridM[1][i]=2.0/len[i-1]+2.0/len[i];
		tridM[2][i]=1.0/len[i];
	}

	if(bcn.mType==boundaryCondition::ZERO_ACC)
	{
		tridM[0][n]=1.0/len[n-1];
		tridM[1][n]=2.0/len[n-1];
	}
	else	// ZERO_VEL
	{
		tridM[0][n]=0;
		tridM[1][n]=1;
	}

	vectorn r(n+1);
	for(int i=0; i<dim; i++)
	{
		//= 3 * ( -1/ni^2,  1/ni^2 - 1/n{i+1}^2, 1/n{i+1}^2) * (yi, y{i+1}, y{i+2})
		for(int j=1; j<n; j++)
		{
			m_real factor1=1.0/SQR(len[j-1]);
			m_real factor2=1.0/SQR(len[j]);
			r[j]=3.f*(y[j+1][i]*factor2 - (factor2-factor1)*y[j][i] -factor1*y[j-1][i]);
		}

		if(bc0.mType==boundaryCondition::ZERO_ACC)
		{
			m_real factor2=1.0/SQR(len[0]);
			r[0]=3.f*(y[1][i]-y[0][i])*factor2;
		}
		else if(bc0.mType==boundaryCondition::ZERO_VEL)// ZERO_VEL
			r[0]=0.0;
		else
			r[0]=bc0.mConstraint[i];

		if(bcn.mType==boundaryCondition::ZERO_ACC)
		{
			m_real factor1=1.0/SQR(len[n-1]);
			r[n]=3.f*(y[n][i]-y[n-1][i])*factor1;
		}
		else if(bcn.mType==boundaryCondition::ZERO_VEL)// ZERO_VEL
			r[n]=0.0;
		else
			r[n]=bcn.mConstraint[i];

        vectornView row_i=DT.row(i);
		_tridag(tridM[0], tridM[1], tridM[2], r, row_i);
	}
}

void NonuniformSpline::getCurve(vectorn const& time, matrixn& points)
{
	Msg::verify(time[0]>=keytime[0] && time[time.size()-1]<=keytime[keytime.size()-1]+FERR, "invalid time array %s", time.output().ptr());

	m_real a, b, c, d, t;

	points.setSize(time.size(),dim);

	for(int j=0; j<dim; j++)
	{
		int curKeytime=0;
		for(int i=0; i<time.size(); i++)
		{

			while(time[i]>keytime[curKeytime+1]+0.001) curKeytime++;

			int iseg=curKeytime;

			// ci= (3*(y{i+1}-yi)/ni -2*Di -D{i+1})/ni
			// di=(2*(yi-y{i+1}) + ni * (Di+D{i+1}))/(ni^3)

			a=y[iseg][j];
			b=DT[j][iseg];
			c=(3.f*(y[iseg+1][j]-y[iseg][j])/len[iseg]-2.f*DT[j][iseg]-DT[j][iseg+1])/len[iseg];
			d=(2.f*(y[iseg][j]-y[iseg+1][j])+len[iseg]*(DT[j][iseg]+DT[j][iseg+1]))/CUBIC(len[iseg]);

			t=time[i]-keytime[curKeytime];
			points[i][j]=a+b*t+c*SQR(t)+d*CUBIC(t);
		}
	}
}

void NonuniformSpline::getFirstDeriv(vectorn const& time, matrixn& points)
{
	Msg::verify(time[0]>=keytime[0] && time[time.size()-1]<=keytime[keytime.size()-1]+FERR, "invalid time array");

	m_real a, b, c, d, t;

	points.setSize(time.size(),dim);

	for(int j=0; j<dim; j++)
	{
		int curKeytime=0;
		for(int i=0; i<time.size(); i++)
		{

			while(time[i]>keytime[curKeytime+1]+0.001) curKeytime++;

			int iseg=curKeytime;

			// ci= (3*(y{i+1}-yi)/ni -2*Di -D{i+1})/ni
			// di=(2*(yi-y{i+1}) + ni * (Di+D{i+1}))/(ni^3)

			a=y[iseg][j];
			b=DT[j][iseg];
			c=(3.f*(y[iseg+1][j]-y[iseg][j])/len[iseg]-2.f*DT[j][iseg]-DT[j][iseg+1])/len[iseg];
			d=(2.f*(y[iseg][j]-y[iseg+1][j])+len[iseg]*(DT[j][iseg]+DT[j][iseg+1]))/CUBIC(len[iseg]);

			t=time[i]-keytime[curKeytime];
			points[i][j]=b+2.0*c*t+3.0*d*SQR(t);
		}
	}
}

void NonuniformSpline::getSecondDeriv(vectorn const& time, matrixn& points)
{
	Msg::verify(time[0]>=keytime[0] && time[time.size()-1]<=keytime[keytime.size()-1]+FERR, "invalid time array");

	m_real a, b, c, d, t;

	points.setSize(time.size(),dim);

	for(int j=0; j<dim; j++)
	{
		int curKeytime=0;
		for(int i=0; i<time.size(); i++)
		{

			while(time[i]>keytime[curKeytime+1]+0.001) curKeytime++;

			int iseg=curKeytime;

			// ci= (3*(y{i+1}-yi)/ni -2*Di -D{i+1})/ni
			// di=(2*(yi-y{i+1}) + ni * (Di+D{i+1}))/(ni^3)

			a=y[iseg][j];
			b=DT[j][iseg];
			c=(3.f*(y[iseg+1][j]-y[iseg][j])/len[iseg]-2.f*DT[j][iseg]-DT[j][iseg+1])/len[iseg];
			d=(2.f*(y[iseg][j]-y[iseg+1][j])+len[iseg]*(DT[j][iseg]+DT[j][iseg+1]))/CUBIC(len[iseg]);

			t=time[i]-keytime[curKeytime];
			points[i][j]=2.0*c+6.0*d*t;
		}
	}
}

// constructor
Spline::Spline(const matrixn& points, int start, int end, int knotSpacingMethod)
{
	if(start<0) start=0;
	if(end>points.rows()) end=points.rows();

    NP = end-start;
	int dim=points.cols();
	P.setSize(NP, dim);
	A.setSize(NP, dim);
	B.setSize(NP, dim);
	C.setSize(NP, dim);
	k.setSize(NP);
	Mat[0].setSize(NP);
	Mat[1].setSize(NP);
	Mat[2].setSize(NP);

	for(int i=0;i<NP ;i++)
	{
		P.setRow(i, points.row(i+start));
	}

	switch(knotSpacingMethod)
	{
	case UNIFORM_KNOT:
		k.setAllValue(1.0f);
		break;
	case CHORD_LENGTH:
		{
			m_real AMag , AMagOld;
			// k
			AMagOld = A.row(0).length();
			for(int i=0 ; i<=NP-3 ; i++)
			{
				AMag = A.row(i+1).length();
				k[i] = AMagOld / AMag;
				AMagOld = AMag;
			}
			k[NP-2] = 1.0f;
		}
		break;
	}

	generate();
}

void Spline::generate()
{
	int dim=P.cols();

	// matrix A
	for(int i= 0 ; i<=NP-2 ; i++ )
	{
		A.row(i).sub(P.row(i+1), P.row(i));
	}

	// Matrix
	for(int i=1; i<=NP-2;i++)
	{
		Mat[0][i] = 1.0f;
		Mat[1][i] = 2.0f*k[i-1]*(1.0f + k[i-1]);
		Mat[2][i] = k[i-1]*k[i-1]*k[i];
	}
	Mat[1][0] = 2.0f;
	Mat[2][0] = k[0];
	Mat[0][NP-1] = 1.0f;
	Mat[1][NP-1] = 2.0f*k[NP-2];

	// matrix B
	for(int i=1; i<=NP-2;i++)
	{
		for(int j=0; j<dim; j++)
			B[i][j]= 3.0f*(A[i-1][j] + k[i-1]*k[i-1]*A[i][j]);
	}

	for(int j=0; j<dim; j++)
	{
		B[0][j] = 3.0f*A[0][j];
		B[NP-1][j] = 3.0f*A[NP-2][j];
	}

	//
	vectorn Bx;
	for(int j=0; j<dim; j++)
	{
		B.getColumn(j, Bx);
		MatrixSolve(Bx, Mat, NP);
		B.setColumn(j, Bx);
	}

	for(int i=0 ; i<=NP-2 ; i++ )
	{
		for(int j=0; j<dim; j++)
			C[i][j]=k[i]*B[i+1][j];
	}
}

void Spline::getCurve(matrixn& points, int ndiv)
{
	// GetCurveCount;
	Curve c;
	int count = 0;
	for(int i=0; i<NP-1 ; i++)
	{
		c.putCurve(A.row(i),B.row(i),C.row(i),ndiv);
		count += c.getCount();
	}

	points.setSize(count, P.cols());

	int PointCount=0;

	for(int i=0; i<NP-1 ; i++)
	{
		c.putCurve(A.row(i),B.row(i),C.row(i),ndiv);
		c.getCurve(P.row(i), points, PointCount);
	}
}

void Spline::MatrixSolve(vectorn& B, vectorn Mat[3], int NP)
{
    vectorn Work(NP);
	vectorn WorkB(NP);

	for(int i=0;i<=NP-1;i++)
	{
		Work[i] = B[i] / Mat[1][i];
		WorkB[i] = Work[i];
	}

	for(int j=0 ; j<10 ; j++)
	{ ///  need convergence judge
		Work[0] = (B[0] - Mat[2][0]*WorkB[1])/Mat[1][0];
		for(int i=1; i<NP-1 ; i++ )
		{
			Work[i] = (B[i]-Mat[0][i]*WorkB[i-1]-Mat[2][i]*WorkB[i+1])
						/Mat[1][i];
		}
		Work[NP-1] = (B[NP-1] - Mat[0][NP-1]*WorkB[NP-2])/Mat[1][NP-1];

		for(int i=0 ; i<=NP-1 ; i++ )
		{
			WorkB[i] = Work[i];
		}
	}
	for(int i=0 ; i<=NP-1 ; i++ )
	{
		B[i] = Work[i];
	}
}

void Spline::Curve::putCurve(const vectorn& a, const vectorn& b, const vectorn& c, int ndiv)
{
#define DIV_FACTOR 4.0 //adjust this factor to adjust the curve smoothness

	A=a;
	B=b;
	C=c;
	if(ndiv==0)
	{
		m_real max=A.maximum();
		m_real min=A.minimum();
		max=ABS(max);
		min=ABS(min);
		Ndiv = (int)(MAX(max, min)/DIV_FACTOR);
	}
	else
		Ndiv =ndiv;
}

void Spline::Curve::getCurve(const vectorn& p, matrixn& points, int& PointCount)
{
	int dim=p.size();
	m_real  t,f,g,h;
	if (Ndiv==0)
		Ndiv=1;

	//points[PointCount].assign(p);
	//PointCount++;

	printf("%d\n",Ndiv);
	for(int i=0; i<Ndiv ; i++)
	{
		t = (m_real)i / (m_real)(Ndiv);
		f = t*t*(3.0f-2.0f*t);
		g = t*(t-1.0f)*(t-1.0f);
		h = t*t*(t-1.0f);

		for(int j=0; j<dim; j++)
		{
			points[PointCount][j] = p[j] + A[j]*f + B[j]*g + C[j]*h;
		}
		PointCount++;
	}
}

int Spline::Curve::getCount()
{
	if (Ndiv==0)
		Ndiv=1;
	int PointCount = 0;

	for(int i=0; i<Ndiv ; i++)
	{
		PointCount++;
	}
	return PointCount;
}
/*
UniformBSpline::UniformBSpline(const vectorn & knot, const matrixn& points, int start, int end)
{
	if(start<0) start=0;
	if(end>points.rows()) end=points.rows();

	intvectorn rows;
	rows.colon(start, end);
	m_vSamples.extractRows(points, rows);

	m_vKnot=knot;
	m_real delta_knot=knot[1]-knot[0];


	// curve f_i(x_i)=SUM_k(CP_i[k]Basis[x_i-k]) where k in knot

	// linear system
	// f(0)=sample[0]
	// f(1)=sample[1]
	// f(2)=sample[2]
	//...
	// f(n-1)=sample[n-1]

	int k=m_vKnot.size();
	int n=m_vSamples.rows();

	// 첫점과 마지막 점은 control point 를 duplicate했다. (double vertices)
	// (B-1+B0(0) B1(0) B2(0) ... Bk-1+Bk(0) )  c0    2*sample[0][k-1]
	// (B-1+B0(1) B1(1) B2(1) ... Bk-1+Bk(1) )  c1    sample[1][k-1]
	// (                                             )  ..  =
	// (                                             )  ck    sample[n-1][k-1]
	// linear system for each dimension
	//     R                                  c_k  =  sample_k

    matrixn R;
	R.setSize(n,k);

	for(int i=0; i<n; i++)
	{
		for(int j=0; j<k; j++)
		{
			R[i][j]=basis(ABS(m_vKnot[j]-m_real(i)));
		}

		R[i][0]+=basis(ABS(m_vKnot[0]-delta_knot-m_real(i)));
		R[i][k-1]+=basis(ABS(m_vKnot[k-1]+delta_knot-m_real(i)));
	}

	AfxMessageBox("here1");
	matrixn invR;
	invR.pseudoInverse(R);

	AfxMessageBox("here2");
	matrixn doubleSample(m_vSamples);
	doubleSample[0]*=2.f;
	doubleSample[n-1]*=2.f;
	m_vCP.mult(invR, m_vSamples);

	m_vCP.op0(m0::drawSignals("test.bmp",0,0,true));
	AfxMessageBox("here3");
}

m_real UniformBSpline::basis(m_real d)
{
	// You can use any kind of basis. Currently cubic bspline bases
	if(d<0.f) d*=-1.f;

	if(d<1.f)
		return 2.f/3.f-SQR(d)*(2.f-d)/2.f;
	if(d<2.f)
		return CUBIC(2.f-d)/6.f;
	return 0.f;
}

*/

// Magic Software, Inc.
// http://www.magic-software.com
// http://www.wild-magic.com
// Copyright (c) 2004.  All Rights Reserved
//
// The Wild Magic Library (WML) source code is supplied under the terms of
// the license agreement http://www.magic-software.com/License/WildMagic.pdf
// and may not be copied or disclosed except in accordance with the terms of
// that agreement.

//----------------------------------------------------------------------------

BSpline::BSpline (const matrixn& akCtrlPoint, int iDegree, bool bLoop, bool bOpen)
    :
    m_bLoop(bLoop)
{
	int iNumCtrlPoints=akCtrlPoint.rows();

    assert( iNumCtrlPoints >= 2 );
    assert( 1 <= iDegree && iDegree <= iNumCtrlPoints-1 );

    m_iNumCtrlPoints = iNumCtrlPoints;
    m_iReplicate = ( bLoop ? (bOpen ? 1 : iDegree) : 0 );
    CreateControl(akCtrlPoint);
    m_kBasis.Create(m_iNumCtrlPoints+m_iReplicate,iDegree,bOpen);
}
//----------------------------------------------------------------------------

BSpline::BSpline (const matrixn& akCtrlPoint, int iDegree, bool bLoop, const vectorn& afKnot)
    :
    m_bLoop(bLoop)
{
	int iNumCtrlPoints=akCtrlPoint.rows();

    assert( iNumCtrlPoints >= 2 );
    assert( 1 <= iDegree && iDegree <= iNumCtrlPoints-1 );

    m_iNumCtrlPoints = iNumCtrlPoints;
    m_iReplicate = ( bLoop ? 1 : 0 );
    CreateControl(akCtrlPoint);
    m_kBasis.Create(m_iNumCtrlPoints+m_iReplicate,iDegree,true);
}
//----------------------------------------------------------------------------

BSpline::~BSpline ()
{
}
//----------------------------------------------------------------------------

void BSpline::CreateControl (const matrixn& akCtrlPoint)
{
    int iNewNumCtrlPoints = m_iNumCtrlPoints + m_iReplicate;

	m_akCtrlPoint.setSize(iNewNumCtrlPoints, akCtrlPoint.cols());
	m_akCtrlPoint.setValue(0,0, akCtrlPoint);
    for (int i = 0; i < m_iReplicate; i++)
        m_akCtrlPoint.setRow(m_iNumCtrlPoints+i, akCtrlPoint.row(i));
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------

void BSpline::SetControlPoint (int i, const vectorn& rkCtrl)
{
    if ( 0 <= i && i < m_iNumCtrlPoints )
    {
        // set the control point
		ASSERT(m_akCtrlPoint.cols()==rkCtrl.size());
        m_akCtrlPoint.row(i) = rkCtrl;

        // set the replicated control point
        if ( i < m_iReplicate )
            m_akCtrlPoint.row(m_iNumCtrlPoints+i) = rkCtrl;
    }
}
//----------------------------------------------------------------------------


//----------------------------------------------------------------------------

m_real& BSpline::Knot (int i)
{
    return m_kBasis.Knot(i);
}
//----------------------------------------------------------------------------

void BSpline::Get (m_real fTime, vectorn* pkPos,
    vectorn* pkDer1, vectorn* pkDer2, vectorn* pkDer3) const
{
    int i, iMin, iMax;
    if ( pkDer3 )
        m_kBasis.Compute(fTime,3,iMin,iMax);
    else if ( pkDer2 )
        m_kBasis.Compute(fTime,2,iMin,iMax);
    else if ( pkDer1 )
        m_kBasis.Compute(fTime,1,iMin,iMax);
    else
        m_kBasis.Compute(fTime,0,iMin,iMax);

    if ( pkPos )
    {
		(*pkPos).setSize(m_akCtrlPoint.cols());
		(*pkPos).setAllValue(0.f);
        for (i = iMin; i <= iMax; i++)
            *pkPos += m_akCtrlPoint.row(i)*m_kBasis.GetD0(i);
    }

    if ( pkDer1 )
    {
		(*pkDer1).setSize(m_akCtrlPoint.cols());
		(*pkDer1).setAllValue(0.f);
        for (i = iMin; i <= iMax; i++)
            *pkDer1 += m_akCtrlPoint.row(i)*m_kBasis.GetD1(i);
    }

    if ( pkDer2 )
    {
		(*pkDer2).setSize(m_akCtrlPoint.cols());
		(*pkDer2).setAllValue(0.f);
        for (i = iMin; i <= iMax; i++)
            *pkDer2 += m_akCtrlPoint.row(i)*m_kBasis.GetD2(i);
    }

    if ( pkDer3 )
    {
		(*pkDer3).setSize(m_akCtrlPoint.cols());
		(*pkDer3).setAllValue(0.f);
        for (i = iMin; i <= iMax; i++)
            *pkDer3 += m_akCtrlPoint.row(i)*m_kBasis.GetD3(i);
    }
}
//----------------------------------------------------------------------------

BSpline::Basis& BSpline::GetBasis ()
{
    return m_kBasis;
}

//----------------------------------------------------------------------------

// Magic Software, Inc.
// http://www.magic-software.com
// http://www.wild-magic.com
// Copyright (c) 2004.  All Rights Reserved
//
// The Wild Magic Library (WML) source code is supplied under the terms of
// the license agreement http://www.magic-software.com/License/WildMagic.pdf
// and may not be copied or disclosed except in accordance with the terms of
// that agreement.
//----------------------------------------------------------------------------

BSpline::Basis::Basis ()
{
}
//----------------------------------------------------------------------------

BSpline::Basis::Basis (int iNumCtrlPoints, int iDegree, bool bOpen)
{
    Create(iNumCtrlPoints,iDegree,bOpen);
}
//----------------------------------------------------------------------------

void BSpline::Basis::Create (int iNumCtrlPoints, int iDegree, bool bOpen)
{
    m_bUniform = true;

    int i, iNumKnots = Initialize(iNumCtrlPoints,iDegree,bOpen);
    m_real fFactor = ((m_real)1.0)/(m_iNumCtrlPoints-m_iDegree);
    if ( m_bOpen )
    {
        for (i = 0; i <= m_iDegree; i++)
            m_afKnot[i] = (m_real)0.0;

        for (/**/; i < m_iNumCtrlPoints; i++)
            m_afKnot[i] = (i-m_iDegree)*fFactor;

        for (/**/; i < iNumKnots; i++)
            m_afKnot[i] = (m_real)1.0;
    }
    else
    {
        for (i = 0; i < iNumKnots; i++)
            m_afKnot[i] = (i-m_iDegree)*fFactor;
    }
}
//----------------------------------------------------------------------------

BSpline::Basis::Basis (const vectorn& afKnot, int iDegree)
{
	Create(afKnot,iDegree);
}
//----------------------------------------------------------------------------

void BSpline::Basis::Create (const vectorn& afKnot, int iDegree)
{
	//afKnot.size() = m_iNumCtrlPoints - m_iDegree;
	int iNumCtrlPoints=afKnot.size()+iDegree;

    m_bUniform = false;

    int i, iNumKnots = Initialize(iNumCtrlPoints,iDegree,true);
    for (i = 0; i <= m_iDegree; i++)
        m_afKnot[i] = (m_real)0.0;

    for (int j = 0; i < m_iNumCtrlPoints; i++, j++)
        m_afKnot[i] = afKnot[j];

    for (/**/; i < iNumKnots; i++)
        m_afKnot[i] = (m_real)1.0;
}
//----------------------------------------------------------------------------

BSpline::Basis::~Basis ()
{
    Deallocate(m_aafBD0);
    Deallocate(m_aafBD1);
    Deallocate(m_aafBD2);
    Deallocate(m_aafBD3);
}

//----------------------------------------------------------------------------

m_real** BSpline::Basis::Allocate () const
{
    int iRows = m_iDegree + 1;
    int iCols = m_iNumCtrlPoints + m_iDegree;
    int iQuantity = iRows*iCols;

    m_real** aafArray = new m_real*[iRows];
    aafArray[0] = new m_real[iQuantity];
    memset(aafArray[0],0,iQuantity*sizeof(m_real));
    for (int i = 1; i < iRows; i++)
        aafArray[i] = &aafArray[0][i*iCols];

    return aafArray;
}
//----------------------------------------------------------------------------

void BSpline::Basis::Deallocate (m_real** aafArray)
{
    if ( aafArray )
    {
        delete[] aafArray[0];
        delete[] aafArray;
    }
}
//----------------------------------------------------------------------------

int BSpline::Basis::Initialize (int iNumCtrlPoints, int iDegree,
    bool bOpen)
{
    assert( iNumCtrlPoints >= 2 );
    assert( 1 <= iDegree && iDegree <= iNumCtrlPoints-1 );

    m_iNumCtrlPoints = iNumCtrlPoints;
    m_iDegree = iDegree;
    m_bOpen = bOpen;

    int iNumKnots = m_iNumCtrlPoints+m_iDegree+1;
	m_afKnot.setSize(iNumKnots);

    m_aafBD0 = Allocate();
    m_aafBD1 = NULL;
    m_aafBD2 = NULL;
    m_aafBD3 = NULL;

    return iNumKnots;
}
//----------------------------------------------------------------------------

m_real& BSpline::Basis::Knot (int i)
{
    if ( !m_bUniform )
    {
        // access only allowed to elements d+1 <= j <= n
        int j = i + m_iDegree + 1;
        if ( m_iDegree+1 <= j && j <= m_iNumCtrlPoints - 1 )
            return m_afKnot[j];
    }

	ASSERT(0);
    return m_afKnot[0];
}
//----------------------------------------------------------------------------

int BSpline::Basis::GetKey (m_real& rfTime) const
{
    if ( m_bOpen )
    {
        // open splines clamp to [0,1]
        if ( rfTime <= (m_real)0.0 )
        {
            rfTime = (m_real)0.0;
            return m_iDegree;
        }
        else if ( rfTime >= (m_real)1.0 )
        {
            rfTime = (m_real)1.0;
            return m_iNumCtrlPoints-1;
        }
    }
    else
    {
        // periodic splines wrap to [0,1]
        if ( rfTime < (m_real)0.0 || rfTime > (m_real)1.0 )
            rfTime -= floor(rfTime);
    }


    int i;

    if ( m_bUniform )
    {
        i = m_iDegree + (int)((m_iNumCtrlPoints-m_iDegree)*rfTime);
    }
    else
    {
        for (i = m_iDegree+1; i <= m_iNumCtrlPoints; i++)
        {
            if ( rfTime < m_afKnot[i] )
                break;
        }
        i--;
    }

    return i;
}
//----------------------------------------------------------------------------

void BSpline::Basis::Compute (m_real fTime, unsigned int uiOrder,
    int& riMinIndex, int& riMaxIndex) const
{
    // only derivatives through third order currently supported
    assert( uiOrder <= 3 );

    if ( uiOrder >= 1 )
    {
        if ( !m_aafBD1 )
            m_aafBD1 = Allocate();

        if ( uiOrder >= 2 )
        {
            if ( !m_aafBD2 )
                m_aafBD2 = Allocate();

            if ( uiOrder >= 3 )
            {
                if ( !m_aafBD3 )
                    m_aafBD3 = Allocate();
            }
        }
    }

    int i = GetKey(fTime);
    m_aafBD0[0][i] = (m_real)1.0;

    if ( uiOrder >= 1 )
    {
        m_aafBD1[0][i] = (m_real)0.0;
        if ( uiOrder >= 2 )
        {
            m_aafBD2[0][i] = (m_real)0.0;
            if ( uiOrder >= 3 )
                m_aafBD3[0][i] = (m_real)0.0;
        }
    }

    m_real fN0 = fTime-m_afKnot[i], fN1 = m_afKnot[i+1]-fTime;
    m_real fInvD0, fInvD1;
    int j;
    for (j = 1; j <= m_iDegree; j++)
    {
        fInvD0 = ((m_real)1.0)/(m_afKnot[i+j]-m_afKnot[i]);
        fInvD1 = ((m_real)1.0)/(m_afKnot[i+1]-m_afKnot[i-j+1]);

        m_aafBD0[j][i] = fN0*m_aafBD0[j-1][i]*fInvD0;
        m_aafBD0[j][i-j] = fN1*m_aafBD0[j-1][i-j+1]*fInvD1;

        if ( uiOrder >= 1 )
        {
            m_aafBD1[j][i] = (fN0*m_aafBD1[j-1][i]+m_aafBD0[j-1][i])*fInvD0;
            m_aafBD1[j][i-j] = (fN1*m_aafBD1[j-1][i-j+1]-m_aafBD0[j-1][i-j+1])
                *fInvD1;

            if ( uiOrder >= 2 )
            {
                m_aafBD2[j][i] = (fN0*m_aafBD2[j-1][i] +
                    ((m_real)2.0)*m_aafBD1[j-1][i])*fInvD0;
                m_aafBD2[j][i-j] = (fN1*m_aafBD2[j-1][i-j+1] -
                    ((m_real)2.0)*m_aafBD1[j-1][i-j+1])*fInvD1;

                if ( uiOrder >= 3 )
                {
                    m_aafBD3[j][i] = (fN0*m_aafBD3[j-1][i] +
                        ((m_real)3.0)*m_aafBD2[j-1][i])*fInvD0;
                    m_aafBD3[j][i-j] = (fN1*m_aafBD3[j-1][i-j+1] -
                        ((m_real)3.0)*m_aafBD2[j-1][i-j+1])*fInvD1;
                }
            }
        }
    }

    for (j = 2; j <= m_iDegree; j++)
    {
        for (int k = i-j+1; k < i; k++)
        {
            fN0 = fTime-m_afKnot[k];
            fN1 = m_afKnot[k+j+1]-fTime;
            fInvD0 = ((m_real)1.0)/(m_afKnot[k+j]-m_afKnot[k]);
            fInvD1 = ((m_real)1.0)/(m_afKnot[k+j+1]-m_afKnot[k+1]);

            m_aafBD0[j][k] = fN0*m_aafBD0[j-1][k]*fInvD0 + fN1*
                m_aafBD0[j-1][k+1]*fInvD1;

            if ( uiOrder >= 1 )
            {
                m_aafBD1[j][k] = (fN0*m_aafBD1[j-1][k]+m_aafBD0[j-1][k])*
                    fInvD0 + (fN1*m_aafBD1[j-1][k+1]-m_aafBD0[j-1][k+1])*
                    fInvD1;

                if ( uiOrder >= 2 )
                {
                    m_aafBD2[j][k] = (fN0*m_aafBD2[j-1][k] +
                        ((m_real)2.0)*m_aafBD1[j-1][k])*fInvD0 +
                        (fN1*m_aafBD2[j-1][k+1]- ((m_real)2.0)*
                        m_aafBD1[j-1][k+1])*fInvD1;

                    if ( uiOrder >= 3 )
                    {
                        m_aafBD3[j][k] = (fN0*m_aafBD3[j-1][k] +
                            ((m_real)3.0)*m_aafBD2[j-1][k])*fInvD0 +
                            (fN1*m_aafBD3[j-1][k+1] - ((m_real)3.0)*
                            m_aafBD2[j-1][k+1])*fInvD1;
                    }
                }
            }
        }
    }

    riMinIndex = i - m_iDegree;
    riMaxIndex = i;
}
//----------------------------------------------------------------------------
