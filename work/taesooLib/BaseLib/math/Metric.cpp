#include "stdafx.h"
#include "mathclass.h"
#include "./Metric.h"
#include "Operator.h"
#include "Operator_NR.h"
#include "conversion.h"

Metric::Metric(void)
{
}

Metric::~Metric(void)
{
}

m_real L2Metric::CalcDistance(const vectorn& a, const vectorn& b)
{
	ASSERT(a.size()==b.size());
	m_real c=0;	
	m_real dist;
	for(int i=0; i<a.size(); i++)
	{
		dist=a[i]-b[i];
		c+=dist*dist;
	}
	return sqrt(c);
}

Metric* L2Metric::Clone() const
{
	return new L2Metric();
}

m_real WeightedL2Metric::CalcDistance(const vectorn& a, const vectorn& b)
{
	ASSERT(a.size()==b.size());
	if(m_vWeight.size()!=a.size())
	{
		m_vWeight.setSize(a.size());
		m_vWeight.setAllValue(1.f);
	}

	m_real c=0;	
	m_real dist;
	for(int i=0; i<a.size(); i++)
	{
		dist=a[i]-b[i];
		c+=dist*dist*m_vWeight[i];
	}
	return sqrt(c);

}

Metric* WeightedL2Metric::Clone() const
{
	Metric* pOutput=new WeightedL2Metric();
	((WeightedL2Metric*)pOutput)->m_vWeight=m_vWeight;
	return pOutput;
}

m_real QuaterMetric::CalcDistance(const vectorn& a, const vectorn& b)
{
	ASSERT(a.size()==b.size());
	ASSERT(a.size()%4==0);

	m_real dist=0;
	for(int i=0; i<a.size(); i+=4)
	{
		m_real t=a.toQuater(i).distance(b.toQuater(i));
		dist+=SQR(t);				
	}
	return dist;
}

KovarMetric::KovarMetric(bool allowTranslationAlongAxis)
{
	m_allowTranslationAlongAxis=allowTranslationAlongAxis;
	m_axis.setValue(0,1,0);	
}

Metric* KovarMetric::Clone() const
{
	KovarMetric* pMetric=new KovarMetric();
	pMetric->m_weights.assign(m_weights);
	pMetric->m_axis=m_axis;
	pMetric->m_transfB=m_transfB;
	return pMetric;
}

m_real KovarMetric::CalcDistance(const vectorn& a, const vectorn& b)
{
	m_srcA.fromVector(a,3);
	m_srcB.fromVector(b,3);

	int n=m_srcA.rows();
	ASSERT(m_srcB.rows()==n);

	if(m_weights.size()!=n)
	{
		m_weights.setSize(n);
		m_weights.setAllValue(1.0/((m_real)n));
	}

	matrixn matA, matB;
	matA.setSameSize(m_srcA);
	matB.setSameSize(m_srcB);
	quater rot;
	rot.axisToAxis(m_axis, vector3(0,1,0));	// m_axis가 y축으로 돌아가는 로테이션.

	vector3 temp;
	for(int i=0; i<n; i++)
	{
		temp.rotate(rot, m_srcA.row(i).toVector3());		
		matA.row(i).assign(temp);
		temp.rotate(rot, m_srcB.row(i).toVector3());		
		matB.row(i).assign(temp);
	}
	
	// Refer SnapTogetherMotion.pdf by H.J.Shin
	// Basically, I will find best matching y_axis rotation
	m_real theta, x0, z0;
	
	// Calculate theta
	m_real xbar=0, x2bar=0, zbar=0, z2bar=0;
	enum {X=0, Y=1, Z=2};
	for(int i=0; i<n; i++)
	{
		m_real wi=m_weights[i];
		xbar+=wi*matA[i][X];
		x2bar+=wi*matB[i][X];
		zbar+=wi*matA[i][Z];
		z2bar+=wi*matB[i][Z];
	}

	m_real ybar=0, y2bar=0;
	if(m_allowTranslationAlongAxis)
	{		
		for(int i=0; i<n; i++)
		{
			m_real wi=m_weights[i];
			ybar+=wi*matA[i][Y];
			y2bar+=wi*matB[i][Y];		
		}
		ybar/=m_weights.sum();
		y2bar/=m_weights.sum();
	}
	m_real sumA=0, sumB=0;
	for(int i=0; i<n; i++)
	{
		m_real wi=m_weights[i];
		m_real xi=matA[i][X];
		m_real zi=matA[i][Z];
		m_real xi2=matB[i][X];
		m_real zi2=matB[i][Z];
		sumA+=wi*(xi*zi2-xi2*zi);
		sumB+=wi*(xi*xi2+zi*zi2);
	}
	
	sumA-=(xbar*z2bar-x2bar*zbar);
	sumB-=(xbar*x2bar+zbar*z2bar);
	theta=atan2(sumA,sumB);

	// calculate x0 and z0
	x0=xbar-x2bar*cos(theta)-z2bar*sin(theta);
	z0=zbar+x2bar*sin(theta)-z2bar*cos(theta);

	// calculate matching distance
	matrix4 transf;
	transf.setIdentityRot();
	transf.leftMultRotation(vector3(0,1,0), theta);
	transf.leftMultTranslation(vector3(x0, ybar-y2bar, z0));
	
//	ASSERT(x0==x0);
	// matA[i] ~similar to~ transf*matB[i]
	// Thus, rot*srcA[i] ~similar to~ transf*rot*srcB[i]
	// Thus, srcA[i] ~similar to~ invrot*transf*rot*srcB[i]
	// Thus, m_transfB <-invrot*transf*rot

	m_transfB.mult(transf, rot);
	m_transfB.leftMultRotation(rot.inverse());

	// transform and calc distance
	m_transformedB.setSameSize(m_srcB);
	
	for(int i=0; i<n; i++)
	{
		vector3 transformed;
		transformed.mult(m_transfB, m_srcB.row(i).toVector3());
		m_transformedB.row(i).assign(transformed);
	}
	
	m_real distance=0;
	for(int i=0; i<n; i++)
	{
		m_real temp=m_weights[i]*m_srcA.row(i).distance(m_transformedB.row(i));
		distance+=SQR(temp);
	}

	return distance;
}

Metric* NoRotMetric::Clone() const
{
	NoRotMetric* pMetric=new NoRotMetric();
	pMetric->m_weights.assign(m_weights);
	pMetric->m_axis=m_axis;
	pMetric->m_transfB=m_transfB;
	return pMetric;
}

m_real NoRotMetric::CalcDistance(const vectorn& a, const vectorn& b)
{
	m_srcA.fromVector(a,3);
	m_srcB.fromVector(b,3);

	int n=m_srcA.rows();
	ASSERT(m_srcB.rows()==n);

	if(m_weights.size()!=n)
	{
		m_weights.setSize(n);
		m_weights.setAllValue(1.0/((m_real)n));
	}

	matrixn matA, matB;
	matA.setSameSize(m_srcA);
	matB.setSameSize(m_srcB);
	quater rot;
	rot.axisToAxis(m_axis, vector3(0,1,0));	// m_axis가 y축으로 돌아가는 로테이션.

	vector3 temp;
	for(int i=0; i<n; i++)
	{
		temp.rotate(rot, m_srcA.row(i).toVector3());		
		matA.row(i).assign(temp);
		temp.rotate(rot, m_srcB.row(i).toVector3());		
		matB.row(i).assign(temp);
	}

	vector3 mean_A(0,0,0);
	vector3 mean_B(0,0,0);
	for(int i=0; i<n; i++)
	{
		mean_A+=matA.row(i).toVector3();
		mean_B+=matB.row(i).toVector3();
	}
	mean_A*=(1.0/double(n));
	mean_B*=(1.0/double(n));
	if(!m_allowTranslationAlongAxis)
	{
		mean_A.y=0; mean_B.y=0;
	}

	// calculate matching distance
	m_transfB.setIdentityRot();
	m_transfB.leftMultTranslation(-mean_B+mean_A);

	// transform and calc distance
	m_transformedB.setSameSize(m_srcB);
	
	for(int i=0; i<n; i++)
	{
		vector3 transformed;
		transformed.mult(m_transfB, m_srcB.row(i).toVector3());
		m_transformedB.row(i).assign(transformed);
	}
	
	m_real distance=0;
	for(int i=0; i<n; i++)
	{
		m_real temp=m_weights[i]*m_srcA.row(i).distance(m_transformedB.row(i));
		distance+=SQR(temp);
	}

	return distance;
}

Metric* PointCloudMetric::Clone() const {return new PointCloudMetric();}

m_real PointCloudMetric::CalcDistance(const vectorn& aa, const vectorn& bb)
{
	matrixnView r=matView(aa,3);
	matrixnView l=matView(bb,3);
	
	// l을 돌려서 r에 맞춘다.
	vectorn l_bar;
	l_bar.aggregateEachColumn(CAggregate::AVG, l);
	vectorn r_bar;
	r_bar.aggregateEachColumn(CAggregate::AVG, r);

	matrixn lp=l;
	matrixn rp=r;
	assert(lp.rows()==rp.rows());
	for(int i=0; i<lp.rows(); i++)
	{
		lp.row(i)-=l_bar;
		rp.row(i)-=r_bar;
	}

	// Please Refer to [Horn1986]
	// [Horn1986] Berthold K.P.Horn, Closed-form solution of absolute orientation using unit quaternions.
	vectornView xl=lp.column(0);
	vectornView yl=lp.column(1);
	vectornView zl=lp.column(2);
	vectornView xr=rp.column(0);
	vectornView yr=rp.column(1);
	vectornView zr=rp.column(2);
	
	m_real Sxx=xl%xr;
	m_real Sxy=xl%yr;
	m_real Sxz=xl%zr;

	m_real Syx=yl%xr;
	m_real Syy=yl%yr;
	m_real Syz=yl%zr;
	
	m_real Szx=zl%xr;
	m_real Szy=zl%yr;
	m_real Szz=zl%zr;

	matrixn N(4,4);
	vecView(N).setValues(16, Sxx+Syy+Szz,	Syz-Szy,	Szx-Sxz,		Sxy-Syx,
							 Syz-Szy,	Sxx-Syy-Szz,	Sxy+Syx,		Szx+Sxz,
							 Szx-Sxz,		Sxy+Syx,	-Sxx+Syy-Szz,	Syz+Szy,
							 Sxy-Syx,		Szx+Sxz,	Syz+Szy,	-Sxx-Syy+Szz);


	vectorn eigenValues;
	matrixn eigenVectors;

	try
	{
	m::eigenDecomposition(N, eigenValues, eigenVectors, 1);

	m_transfB.identity();
	m_transfB.leftMultTranslation(-l_bar.toVector3());
	m_transfB.leftMultRotation(eigenVectors.column(0).toQuater());
	m_transfB.leftMultTranslation(r_bar.toVector3());
	}
	catch(std::runtime_error & e)
	{
		m_transfB.identity();
		m_transfB.leftMultTranslation(-l_bar.toVector3());
		m_transfB.leftMultTranslation(r_bar.toVector3());
	}

	// transform and calc distance
	matrixn & matA=r;
	matrixn & matB=l;
	int n=matA.rows();
	m_transformedB.setSameSize(matB);
	
	for(int i=0; i<n; i++)
	{
		vector3 transformed;
		transformed.mult(m_transfB, matB.row(i).toVector3());
		m_transformedB.row(i).assign(transformed);
	}
	
	m_real distance=0;
	for(int i=0; i<n; i++)
	{
		m_real temp=matA.row(i).distance(m_transformedB.row(i));
		distance+=SQR(temp);
	}

	return sqrt(distance);
}


#include "DynamicTimeWarping.h"

DTWMetric::DTWMetric(int numColumn)
:m_nColumn(numColumn) 
{
	m_pDTW = new CDynamicTimeWarping();
}

DTWMetric::~DTWMetric()
{
	delete m_pDTW;
}


m_real DTWMetric::CalcDistance(const vectorn& a, const vectorn& b)
{
	m_srcA.fromVector(a, m_nColumn);
	m_srcB.fromVector(b, m_nColumn);

	return m_pDTW->CalcDTWDistance(m_srcA, m_srcB);
}

Metric* DTWMetric::Clone() const
{
	return new DTWMetric(m_nColumn);
}
/*
void CBoxerMetric ::State::calcDeltaState(const vectorn& vecParam) 
{
	State& delta=*this;
	delta.pos.x=vecParam[0];
	delta.pos.y=vecParam[1];
	delta.pos.z=vecParam[2];
	delta.orientation.x=vecParam[3];
	delta.orientation.y=vecParam[4];
	delta.orientation.z=vecParam[5];
	delta.orientation.w=vecParam[6];
	delta.targetDirection.x=vecParam[7];
	delta.targetDirection.y=vecParam[8];
	delta.targetDirection.z=vecParam[9];
	delta.orientation2.x=vecParam[10];
	delta.orientation2.y=vecParam[11];
	delta.orientation2.z=vecParam[12];
	delta.orientation2.w=vecParam[13];
}
m_real CBoxerMetric::CalcDistance(const vectorn& a, const vectorn& b)
{
	ASSERT(a.size()==b.size());
	ASSERT(a.size()==14);

	State d1,d2;
	d1.calcDeltaState(a);
	d2.calcDeltaState(b);

	//코드 고쳐야 함
	//임의의 상수를 줬음.

	m_real sum[4];
	for(int i=0;i<4;++i){
		sum[i]=0;
	}
	sum[0]=d1.pos.distance(d2.pos)*0.04;


	m_real dist;
	if((dist=d1.orientation%d2.orientation)>0)
		sum[1]= ACOS(dist);

	else
		sum[1]= ACOS(-1.0*dist);

	dist=d1.targetDirection%d2.targetDirection;
	sum[2]= ACOS(dist)*3.0;

	printf("%f %f %f\n",sum[0],sum[1],sum[2]);

	return sum[0]+sum[1]+sum[2]+sum[3];
}

*/


Metric* WeightedPointCloudMetric::Clone() const {return new WeightedPointCloudMetric(weights);}

m_real WeightedPointCloudMetric::CalcDistance(const vectorn& aa, const vectorn& bb)
{
	matrixnView r=matView(aa,3);
	matrixnView l=matView(bb,3);
	
	ASSERT(l.rows()==r.rows());
	// l을 돌려서 r에 맞춘다.
	vectorn l_bar(3), r_bar(3);
	l_bar.setAllValue(0);
	r_bar.setAllValue(0);

	for(int i=0; i<l.rows(); i++)
	{
		l_bar+=(l.row(i)*weights(i));
		r_bar+=(r.row(i)*weights(i));
	}

	matrixn lp=l;
	matrixn rp=r;
	for(int i=0; i<l.rows(); i++)
	{
		lp.row(i)-=l_bar;
		rp.row(i)-=r_bar;
	}

	for(int i=0; i<l.rows(); i++)
	{
		lp.row(i)*=weights(i);
		rp.row(i)*=weights(i);
	}
	// Please Refer to [Horn1986]
	// [Horn1986] Berthold K.P.Horn, Closed-form solution of absolute orientation using unit quaternions.
	vectornView xl=lp.column(0);
	vectornView yl=lp.column(1);
	vectornView zl=lp.column(2);
	vectornView xr=rp.column(0);
	vectornView yr=rp.column(1);
	vectornView zr=rp.column(2);
	
	m_real Sxx=xl%xr;
	m_real Sxy=xl%yr;
	m_real Sxz=xl%zr;

	m_real Syx=yl%xr;
	m_real Syy=yl%yr;
	m_real Syz=yl%zr;
	
	m_real Szx=zl%xr;
	m_real Szy=zl%yr;
	m_real Szz=zl%zr;

	matrixn N(4,4);
	vecView(N).setValues(16, Sxx+Syy+Szz,	Syz-Szy,	Szx-Sxz,		Sxy-Syx,
							 Syz-Szy,	Sxx-Syy-Szz,	Sxy+Syx,		Szx+Sxz,
							 Szx-Sxz,		Sxy+Syx,	-Sxx+Syy-Szz,	Syz+Szy,
							 Sxy-Syx,		Szx+Sxz,	Syz+Szy,	-Sxx-Syy+Szz);


	vectorn eigenValues;
	matrixn eigenVectors;
	errorOccurred=false;
	try
	{
	m::eigenDecomposition(N, eigenValues, eigenVectors, 1);

	m_transfB.identity();
	m_transfB.leftMultTranslation(-l_bar.toVector3());
	m_transfB.leftMultRotation(eigenVectors.column(0).toQuater());
	m_transfB.leftMultTranslation(r_bar.toVector3());
	}
	catch(std::runtime_error & e)
	{
		m_transfB.identity();
		m_transfB.leftMultTranslation(-l_bar.toVector3());
		m_transfB.leftMultTranslation(r_bar.toVector3());
		errorOccurred=true;
	}

	// transform and calc distance
	matrixn & matA=r;
	matrixn & matB=l;
	int n=matA.rows();
	m_transformedB.setSameSize(matB);
	
	for(int i=0; i<n; i++)
	{
		vector3 transformed;
		transformed.mult(m_transfB, matB.row(i).toVector3());
		m_transformedB.row(i).assign(transformed);
	}
	
	m_real distance=0;
	for(int i=0; i<n; i++)
	{
		m_real temp=matA.row(i).distance(m_transformedB.row(i));
		distance+=SQR(temp);
	}

	return sqrt(distance);

}
