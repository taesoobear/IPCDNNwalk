#include "stdafx.h"
#include "mathclass.h"
#include "Operator.h"
#include "Filter.h"
#include "BSpline.h"
#include "../utility/operatorString.h"
//#include "matrixnTransposedView.h"
namespace sop
{
	int interpolateInt(m_real t, int s, int e)
	{
		m_real f=m_real(s)*(1.0-t)+m_real(e)*t;
		return ROUND(f);
	}

	m_real interpolate(m_real t, m_real s, m_real e)
	{
		m_real f=s*(1.0-t)+e*t;
		return f;
	}

	m_real map(m_real t, m_real min, m_real max, m_real v1, m_real v2)
	{
		// return v1+((t-min)/(max-min))*(v2-v1)
		t=(t-min)/(max-min);
		return interpolate(t, v1, v2);
	}

	m_real clampMap(m_real t, m_real min, m_real max, m_real v1, m_real v2)
	{
		if(t<min) t=min;
		if(t>max) t=max;

		t=(t-min)/(max-min);
		return interpolate(t, v1, v2);
	}

	m_real smoothTransition(m_real a)
	{ return ((m_real)-2.0)*a*a*a+((m_real)3.0)*a*a;} // y=-2x^3+3x^2
													  //
	m_real sigmoid(m_real x)
	{
        return 1.0 / (1.0 + exp(-x));
	}
}

void  m::distanceMat(matrixn& out, matrixn const& a, Metric* pMetric)
{
	out.setSize(a.rows(), a.rows());
	for(int i=0; i<a.rows(); i++)
	{
		for(int j=i; j<a.rows(); j++)
		{
			out[j][i]=out[i][j]=a.row(i).distance(a.row(j), pMetric);
		}
	}
}
void  m::distanceMat(matrixn & out, matrixn const& a, matrixn const& b, Metric* pMetric)
{
	// 두 matrix의 각 원소 벡터 모든 조합 사이의 거리 구함.
	ASSERT(a.cols()==b.cols());
	out.setSize(a.rows(), b.rows());
	for(int i=0; i<a.rows(); i++)
	{
		for(int j=0; j<b.rows(); j++)
		{
			out[i][j]=a.row(i).distance(b.row(j),pMetric);
		}
	}
}
index2 m::argMin(matrixn const& a)
{
	index2 argMinV;
	m_real minV=DBL_MAX;
	for(int i=0; i<a.rows(); i++)
	{
		for(int j=0; j<a.cols(); j++)
		{
			if(a(i,j)<minV)
			{
				minV=a(i,j);
				argMinV=index2(i,j);
			}
		}
	}
	return argMinV;
}

static double _hermite(double p1, double t1, double p2, double t2, double s)
{
	//!< hermite interpolation of p1 and p2. 0<=t<=1
	m_real h1 =  2*CUBIC(s) - 3*SQR(s) + 1;          // calculate basis function 1
	m_real h2 = -2*CUBIC(s) + 3*SQR(s);              // calculate basis function 2
	m_real h3 =   CUBIC(s) - 2*SQR(s) + s;         // calculate basis function 3
	m_real h4 =   CUBIC(s) -  SQR(s);              // calculate basis function 4

	// multiply and sum all funtions together to build the interpolated point along the curve.
	return h1*p1 + h2*p2 + h3*t1 + h4*t2;
}

/**
 * ab와 cd를 연결하는 hermite curve를 만든다. 중간은 duration으로 주어진다.
 *   ab
 * ----
 *    -- duration --
 *                 +++++++
 *                 cd
 *    ************** hermite range (size=duration)
 */
void  v::hermite(vectorn& out, double a, double b, int duration, double c, double d)
{
	double vel1, vel2;
	vel1=b-a;
	vel2=d-c;
	double totalTime=duration+1;
	vel1/=(1.f/totalTime);	// 시간으로 나눠주었다. 단위는 1프레임
	vel2/=(1.f/totalTime);	// 시간으로 나눠주었다. 단위는 1프레임
	out.setSize(duration);

	for(int i=0; i<duration; i++)
	{
		out(i)=_hermite(a, vel1, d, vel2, (double)(i+1)/totalTime);
	}
}
#include <Eigen/Dense>
#include <Eigen/LU>

class QuinticPolynomial
{
	double _a0,_a1,_a2,_a3,_a4,_a5;
	public:

	QuinticPolynomial(double x0, double v0, double a0, double x1, double v1, double a1, double T)
	{
		Eigen::Matrix<double, 3, 3> A;
		double T2=T*T;
		double T3=T2*T;
		double T4=T3*T;
		A<< T3,  T3*T, T3*T2,
			3.0 * T2, 4.0 * T3, 5.0 * T4,
			6.0 * T, 12.0 * T2, 20.0 * T3;

		Eigen::Matrix<double,3,1> b;
		b<<x1 - x0 - v0 * T - a0 * T2 / 2.0,
			v1 - v0 - a0 * T,
			a1 - a0;

		//Eigen::Vector3d x=A.fullPivLU().solve(b);
		Eigen::PartialPivLU<Eigen::Ref<Eigen::MatrixXd> > lu(A);
		Eigen::Vector3d x=lu.solve(b);

		auto& self=*this;
		self._a0 = x0;
		self._a1 = v0;
		self._a2 = a0 / 2.0;
		self._a3 = x[0];
		self._a4 = x[1];
		self._a5 = x[2];


	}
    double calc_xt(double t){
		double t2=t*t;
		double t3=t2*t;
		double t4=t2*t2;
		double t5=t4*t;
        double xt = _a0 + _a1 * t + _a2 * t2 + _a3 * t3 + _a4 * t4 + _a5 * t5;

        return xt;
	}

    double calc_dxt(double t){
		double t2=t*t;
		double t3=t2*t;
		double t4=t2*t2;
        double dxt = _a1 + 2.0 * _a2 * t + 3.0 * _a3 * t2 + 4.0 * _a4 * t3 + 5.0 * _a5 * t4;

        return dxt;
	}

    double calc_ddxt(double t){
		double t2=t*t;
		double t3=t2*t;
        double ddxt = 2.0 * _a2 + 6.0 * _a3 * t + 12.0 * _a4 * t2 + 20.0 * _a5 * t3;

        return ddxt;
	}

	/*
    double calc_dddxt(double t){
        dddxt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2;

        return dddxt;
	}
	*/



};

void v::quintic(vectorn& out, double x0, double v0, double a0, double x1, double v1, double a1, double T)
{
	QuinticPolynomial q(x0, v0, a0, x1, v1, a1, T);
	int n=out.size();
	for(int i=0; i<n; i++)
	{
		double t=double(i)/double(n-1);
		t*=T;
		out[i]=q.calc_xt(t);
	}
}

	  void v::interpolate(vectorn & out, m_real t, vectorn const& a, vectorn const& b)
	  {
		out.setSize(a.size());
		ASSERT(a.size()==b.size());

		for (int i=0, ni=a.size(); i<ni; i++)
			out[i]=sop::interpolate(t,a(i), b(i));
	  }
    
	  m_real v::sample(vectorn const& in, m_real criticalTime)
    {
      m_real out;
      //!< 0 <=criticalTime<= numFrames()-1
      // float 0 이 정확하게 integer 0에 mapping된다.
      int a;
      float t;
      
      a=(int)floor(criticalTime);
      t=criticalTime-(float)a;
      
      if(t<0.005)
	out=in(a);
      else if(t>0.995)
	out=in(a+1);
      else
	{
	  if(a<0)
	    out=sop::interpolate(t-1.0,in(a+1), in(a+2));
	  else if(a+1>=in.size())
	    out=sop::interpolate(t+1.0, in(a-1), in(a));
	  else
	    out=sop::interpolate(t, in(a), in(a+1));
	}
      return out;
    }
    

void v::transition(vectorn & c, m_real mStart, m_real mEnd, int mnSize)
{
	c.setSize(mnSize);

	m_real totalTime=mnSize-1;
	m_real currTime;
	for(int i=0; i<mnSize; i++)
	{
		currTime=(float)i/totalTime;
		float t=-2.f*CUBIC(currTime)+3.f*SQR(currTime);
		c[i]=sop::interpolate(t, mStart, mEnd);
	}
}
int v::argMinRand(vectorn const& a, m_real thr, int start, int end)
{
	if (end>a.size()) 
		end=a.size();
	int argMinV=a.argMin();

	intvectorn indexes;

	m_real minV=a(argMinV)*thr;
	for(int i=start; i<end; i++)
	{
		if(a[i]<minV)
			indexes.push_back(i);
	}

	return indexes[rand()%indexes.size()];
}
index2 m::argMinRand(matrixn const& a, m_real thr)
{
	index2 argMinV=argMin(a);

	std::vector<index2> indexes;

	m_real minV=a(argMinV(0), argMinV(1))*thr;
	for(int i=0; i<a.rows(); i++)
	{
		for(int j=0; j<a.cols(); j++)
		{
			if(a(i,j)<minV)
			{
				indexes.push_back(index2(i,j));
			}
		}
	}

	return indexes[rand()%indexes.size()];
}

namespace m
{
	void multAB(matrixn& out, matrixn const& a, matrixn const& b, bool transposeA, bool transposeB)
	{
		if(transposeA)
		{
			if(transposeB)
				out.multAtBt(a,b);
			else
				out.multAtB(a,b);
		}
		else
		{
			if(transposeB)
				out.multABt(a,b);
			else
				out.mult(a,b);
		}
	}

	void multABC(matrixn& out, matrixn const& a, matrixn const& b, matrixn const& c, bool transposeA, bool transposeB, bool transposeC)
	{
		int rowA=a.rows();
		int colA=a.cols();
		int rowB=b.rows();
		int colB=b.cols();
		int rowC=c.rows();
		int colC=c.cols();

		if(transposeA)	SWAP<int>(rowA, colA);
		if(transposeB)	SWAP<int>(rowB, colB);
		if(transposeC)	SWAP<int>(rowC, colC);

		ASSERT(colA==rowB);
		ASSERT(colB==rowC);
		
		int aa=rowA;
		int bb=rowB;
		int cc=rowC;
		int dd=colC;

		if(aa*bb*cc+aa*cc*dd < bb*cc*dd+aa*bb*dd)
		{
			// case 1: (ab)c is efficient than a(bc)
			matrixn ab;
			m::multAB(ab, a, b, transposeA, transposeB);
			m::multAB(out, ab, c, false, transposeC);
		}
		else
		{
			// case 1: a(bc) is efficient than (ab)c
			matrixn bc;
			m::multAB(bc, b, c, transposeB, transposeC);
			m::multAB(out, a, bc, transposeA, false);
		}
	}

	void multAtB(vectorn& out, matrixn const& A, vectorn const& b)
	{
		ASSERT(A.cols()==b.size());
		// out.setSize(A.rows());
		// out.column().multAtB(A, b.column()); -> slow
		out.multmat(b,A);
	}




	matrixn diag(vectorn const& a)
	{
		matrixn c;
		c.setSize(a.size(),a.size());
		c.setAllValue(0.0);
		c.setDiagonal(a);
		return c;
	}




}





/*
void v0::each::calc(vectorn& c) const
{
	for(int i=0; i<c.size(); i++)
		m_cOperator.Calc(c[i], c[i]);
}


void v0::useUnary::calcInt(intvectorn& c) const
{
	intvectorn temp(c);
	m_cOperator.calcInt(c,temp);
}

v0::domain::domain(const bitvectorn& abIndex, const v0::Operator& op)
:m_op(op)
{
	m_aIndex.findIndex(abIndex,true);
}

v0::domain::domain(int start, int end, int stepSize, const v0::Operator& op)
:m_op(op)
{
	m_aIndex.colon(start, end, stepSize);
}

void v0::domain::calc(vectorn& c) const
{
	vectorn temp;
	temp.op1(v1::extract(m_aIndex), c);
	temp.op0(m_op);
	
	if(temp.size()!=m_aIndex.size())
		printf("Error in v0::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aIndex), temp);
}

void v0::domain::calcInt(intvectorn& c) const
{
	intvectorn temp;
	temp.op1(v1::extract(m_aIndex), c);
	temp.op0(m_op);
	
	if(temp.size()!=m_aIndex.size())
		printf("Error in v0::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aIndex), temp);
}

v1::domainRange::domainRange(const bitvectorn& abIndex, const v1::Operator& op)
:m_op(op)
{
	m_aInIndex.findIndex(abIndex,true);
	m_aOutIndex=m_aInIndex;

}

v1::domainRange::domainRange(int start, int end, int stepSize, const v1::Operator& op)
:m_op(op)
{
	m_aInIndex.colon(start, end, stepSize);
	m_aOutIndex=m_aInIndex;
}

void v1::domainRange::calc(vectorn& c, const vectorn& a) const
{
	vectorn tempc;
	vectorn tempa;
	
	tempc.op1(v1::extract(m_aOutIndex),c);
	tempa.op1(v1::extract(m_aInIndex),a);
	tempc.op1(m_op, tempa);
	
	if(m_aInIndex.size()!=m_aOutIndex.size())
		printf("Error in v1::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aOutIndex), tempc);
}

v1::domain::domain(int start, int end, const v1::Operator& op)
:m_op(op)
{
	m_aInIndex.colon(start, end);
}
void v1::domain::calc(vectorn& c, const vectorn& a) const
{
	vectorn tempa;

	tempa.op1(v1::extract(m_aInIndex),a);
	c.op1(m_op, tempa);
}

void v1::domainRange::calcInt(intvectorn& c, const intvectorn& a) const
{
	intvectorn tempc;
	intvectorn tempa;
	
	tempc.op1(v1::extract(m_aOutIndex),c);
	tempa.op1(v1::extract(m_aInIndex),a);
	tempc.op1(m_op, tempa);
	
	if(m_aInIndex.size()!=m_aOutIndex.size())
		printf("Error in v1::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aOutIndex), tempc);
}

void v1::domainRange::calc(vectorn& c, m_real a) const
{
	vectorn temp;
	temp.op1(v1::extract(m_aOutIndex), c);
	temp.op1(m_op, a);

	if(temp.size()!=m_aOutIndex.size())
		printf("Error in v1::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aOutIndex), temp);
}

void v1::domainRange::calcInt(intvectorn& c, int a) const
{
	intvectorn temp;
	temp.op1(v1::extract(m_aOutIndex), c);
	temp.op1(m_op, a);

	if(temp.size()!=m_aOutIndex.size())
		printf("Error in v1::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aOutIndex), temp);
}*/


#include "Filter.h"

/*
void v2::each::calc(vectorn& c, const vectorn& a, const vectorn& b) const
{
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=m_cOperator.Calc(a[i],b[i]);
}	

void v2::each::calc(vectorn& c, const vectorn& a, m_real b) const
{
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=m_cOperator.Calc(a[i],b);
}	

void v2::each::calcInt(intvectorn& c, const intvectorn& a, const intvectorn& b) const
{
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=m_cOperator.CalcInt(a[i],b[i]);
}

void v2::each::calcInt(intvectorn& c, const intvectorn& a, int b) const
{
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=m_cOperator.CalcInt(a[i],b);
}

*/




/*
/// c=ADD(c,d) 처럼 binaryop를 사용한다. eg) matrix1.op1(m1::useBinary(m2::...()))..

m0::domain::domain(int start, int end, int stepSize, const m0::Operator& op)
:m_op(op)
{
	m_aIndex.colon(start, end, stepSize);
}

void m0::domain::calc(matrixn& c) const
{
	matrixn temp;
	temp.op1(m1::extractRows(m_aIndex), c);
	temp.op0(m_op);
	
	if(temp.rows()!=m_aIndex.size())
		Msg::error("Error in m0::domain::calc\n");
	else
		c.op1(m1::assignRows(m_aIndex), temp);
}

m1::assignRows::assignRows(const intvectorn& index)
{
	m_vIndex.assign(index);
}

m1::assignRows::assignRows(int start, int end, int stepSize)
{
	m_vIndex.colon(start, end, stepSize);
}

void m1::assignRows::calc(matrixn& c, const matrixn& a) const
{
	for(int j=0; j<m_vIndex.size(); j++)
		c[m_vIndex[j]]=a[j];
}

m1::domainRange::domainRange(const bitvectorn& abIndex, const m1::Operator& op)
:m_op(op)
{
	m_aInIndex.findIndex(abIndex,true);
	m_aOutIndex=m_aInIndex;

}

m1::domainRange::domainRange(int start, int end, int stepSize, const m1::Operator& op)
:m_op(op)
{
	m_aInIndex.colon(start, end, stepSize);
	m_aOutIndex=m_aInIndex;
}

void m1::domainRange::calc(matrixn& c, const matrixn& a) const
{
	matrixn tempc;
	matrixn tempa;
	
	// unary operator 에서 c는 사용하지 않는 경우가 있다. 이경우 사용자가 c의 크기를 일일이 세팅하지 않는경우가 있다.
	// 이경우 양쪽 크기가 다르면 에러가 날수 있으므로 아래를 추가하였다. c와 a의 크기가 같은 일반적인 경우 영향없다.
	c.setSameSize(a);

	tempc.op1(m1::extractRows(m_aOutIndex),c);
	tempa.op1(m1::extractRows(m_aInIndex),a);
	tempc.op1(m_op, tempa);
	
	if(m_aInIndex.size()!=m_aOutIndex.size())
		Msg::error("Error in m1::domain::calcInt (in, out size are different!)");
	else
		c.op1(m1::assignRows(m_aOutIndex), tempc);
}
*/


void m::splineFit(int m_nDegree, matrixn& c, const matrixn& a) 
{
	BSpline sp(a, m_nDegree);
	c.setSameSize(a);
	for(int i=0; i<a.rows(); i++)
	{
		vectornView c_i=c.row(i);
		sp.GetPosition((float)i/(float)(a.rows()-1), c_i);
	}
}


#include "../image/imageclass.h"

void v::uniformSampling(vectorn& c, m_real x1, m_real x2, int nSize) 
{
	if(nSize!=-1)
		c.setSize(nSize);

	// simple sampling
	m_real len=x2-x1;
	m_real factor=1.f/((m_real)c.size());

	for(int i=0; i<c.size(); i++)
	{
		m_real position=((m_real)(i+(i+1)))*factor/2.f;
		c[i]=position*len+x1;
	}
}

void m::drawSignals(const char* m_strFilename, matrixn const& c, double m_fMin, double m_fMax, intvectorn m_vXplot)
{
	bool m_bMultiDimensional=true;
	bool m_bUseTheSameMinMax=true;

	CImage* pImage;
	if(m_bMultiDimensional)
	{
		matrixn t;
		t.transpose(c);

		if(m_bUseTheSameMinMax)
			pImage=CImageProcessor::DrawChart(t, CImageProcessor::LINE_CHART, m_fMin, m_fMax);
		else
		{
			vectorn aMin, aMax;
			aMin.minimum(c);
			aMax.maximum(c);
			for(int i=0; i<aMax.size(); i++) aMax(i)+=0.000001; 
			pImage=Imp::DrawChart(t, CImageProcessor::LINE_CHART, aMin, aMax);
		}
	}
	else
		pImage=CImageProcessor::DrawChart(c, CImageProcessor::LINE_CHART, m_fMin, m_fMax);

	CImagePixel cip(pImage);
	
	for(int i=0; i<m_vXplot.size(); i++)
	{
		cip.DrawVertLine(m_vXplot[i], 0, cip.Height(), CPixelRGB8(0,0,0),true);
	}

	CImageProcessor::SaveAndDeleteImage(pImage, m_strFilename);
}

void m::covariance(matrixn& c, const matrixn& a) 
{
	vectorn mean;
	mean.aggregateEachColumn(CAggregate::AVG, a);

	matrixn zeroMeaned;
	for(int i=0; i<zeroMeaned.rows(); i++)
		zeroMeaned.row(i).sub(a.row(i), mean);
	matrixn zeroMeaned_Columns;
	zeroMeaned_Columns.transpose(zeroMeaned);

	int dimension=a.cols();
	int numData=a.rows();

	c.setSize(dimension, dimension);

	for(int i=0; i<dimension; i++)
	{
		for(int j=0; j<dimension; j++)
		{
			c[i][j]=(zeroMeaned_Columns.row(i)%zeroMeaned_Columns.row(j))/(numData-1);
		}
	}
}

void m::filter(matrixn& inout, int kernelSize)
{
	matrixn a=inout;
	vectorn kernel;
	Filter::GetGaussFilter(kernelSize, kernel);	
	Filter::LTIFilter(1, kernel, a, inout);
}
void m::adaptiveFilter(matrixn& inout, vectorn const&  kernelSize, float frameTime)
{
	matrixn out(inout.rows(), inout.cols());
	for (int i=0; i<inout.cols(); i++)
		Filter::LTIFilter(1, frameTime, kernelSize, inout.column(i), out.column(i).lval());

	inout.assign(out);
}
	
void m::derivativeQuater(matrixn & out, matrixn const& rotations)	// ex) angularVelocities.delta(rotations);
{
	ASSERT(rotations.cols()==4);
	out.setSize(rotations.rows(), 3);
	ASSERT(out.rows()>2);
	for(int i=1; i<rotations.rows()-1; i++)
	{
		// w[i]*q[i-1]=q[i+1]
		// w[i]=q[i+1]*q[i-1]^-1
		quater invq;
		invq.inverse(rotations.row(i-1).toQuater());
		vector3 rotVector;
		rotVector.rotationVector(rotations.row(i+1).toQuater()*invq);
		out.row(i).assign(rotVector);
	}
	out.row(0)=out.row(1);
	out.row(rotations.rows()-1)=out.row(rotations.rows()-2);
}
void m::derivative(matrixn& c, const matrixn& a) 
{
	c.setSameSize(a);
	ASSERT(c.rows()>2);
	for(int i=1; i<a.rows()-1; i++)
	{
		c.row(i).sub(a.row(i+1), a.row(i-1));
		c.row(i)*=1/2.f;
	}
	c.setRow(0, c.row(1));
	c.setRow(a.rows()-1, c.row(a.rows()-2));
}
void  m::derivative(matrixn& c,matrixn const& positions, bitvectorn const& discontinuity)
{
	// ex) velocities.delta(positions);
	c.setSize(positions.rows(), positions.cols());

	intvectorn encoding;
	encoding.runLengthEncodeCut(discontinuity);

	int numSeg=encoding.size()/2;

	for(int seg=0; seg<numSeg; seg++)
	{
		int start=encoding[seg*2];
		int end=encoding[seg*2+1];
		m::derivative(c.range(start, end).lval(),positions.range(start, end));
	}
}

void m::superSampling(int nSuperSample, matrixn& c, const matrixn& a) 
{
	int m_nXn=nSuperSample;
	UniformSpline spl(a);
	matrixn temp;
	spl.getCurve(temp, m_nXn);

	// duplicate final vectors so that the number of rows simply becomes (end-start)*m_nXn
	c.resize(temp.rows()+m_nXn-1, temp.cols());
	c.range(0, temp.rows(), 0, temp.cols()).assign(temp);
	for(int i=0; i<m_nXn-1; i++)
	{
		c.row(c.rows()-m_nXn+1+i)=c.row(c.rows()-m_nXn);
	}
}

void m::downSampling(int m_nXn, matrixn& c, const matrixn& a) 
{
	int start=0;
	int end=a.rows();

	ASSERT((end-start)%m_nXn==0);

	c.setSize((end-start)/m_nXn, a.cols());

	c.setAllValue(0);
	for(int i=0; i<c.rows(); i++)
	{
		for(int j=0; j<m_nXn; j++)
		{
			c.row(i)+=a.row(i*m_nXn+j);
		}
		c.row(i)/=(m_real)m_nXn;
	}
}
void m::alignQuater(matrixn& c)
{
	for(int i=1; i<c.rows(); i++)
	{
		if(c.row(i-1)%c.row(i)<0)
			c.row(i)*=-1.f;
	}
}





void m::multA_diagB(matrixn& c, matrixn const& a, vectorn const& b)
{
	c.setSameSize(a);
	for(int i=0; i<c.cols(); i++)
		c.column(i).mult(a.column(i), b[i]);
}


m_real m::vMv(vectorn const& v, matrixn const& M)
{
	/* 깔끔한 버젼
	static vectorn vM;
	vM.multmat(v, M);
	return vM%v;*/

	// 빠른 구현.
	int b1,b2;
    m_real diff1,diff2;
    double sum;

    sum = 0;
	int nbands=v.size();
    for(b1=0; b1<nbands; b1++) 
    for(b2=0; b2<nbands; b2++)
    {
      diff1 = v[b1];
      diff2 = v[b2];
      sum += diff1*diff2*M[b1][b2];
    }
	return sum;
}

m_real m::sMs(vectorn const& a, vectorn const& b, matrixn const& M)
{
	/* 깔끔한 버젼
	static vectorn s;
	s.sub(a,b);
	return m::vMv(s,M);*/

	// 빠른 구현. (속도 차이 많이 남)
	int b1,b2;
    m_real diff1,diff2;
    double sum;

    sum = 0;
	int nbands=a.size();
    for(b1=0; b1<nbands; b1++) 
    for(b2=0; b2<nbands; b2++)
    {
      diff1 = a[b1]-b[b1];
      diff2 = a[b2]-b[b2];
      sum += diff1*diff2*M[b1][b2];
    }
	return sum;

}

m_real m::ss(vectorn const& a, vectorn const& b)
{
	m_real ss=0;
	for(int i=0; i<a.size(); i++)
	{
		ss+=SQR(a[i]-b[i]);
	}
	return ss;
}
m_real m::vDv(vectorn const& v, vectorn const& diagM)
{
    int b1;
    double diff1,diff2;
    double sum;

    sum = 0;
	int nbands=v.size();
    for(b1=0; b1<nbands; b1++) 
    {
      diff1 = v[b1];      
      sum += diff1*diff1*diagM[b1];
    }
	return sum;
}

m_real m::sDs(vectorn const& a, vectorn const& b, vectorn const& diagM)
{
    int b1;
    double diff1,diff2;
    double sum;

    sum = 0;
	int nbands=a.size();
    for(b1=0; b1<nbands; b1++) 
    {
      diff1 = a[b1]-b[b1];
      
      sum += diff1*diff1*diagM[b1];
    }

	return sum;
}

void m::multAtBA(matrixn& c, matrixn const& a, matrixn const& b) 
{
	c.multAtB(a, b*a);
}

void m::multABAt(matrixn& c, matrixn const& a, matrixn const& b) 
{
	c.multABt(a*b, a);
}

void m::hermite(matrixn& out, const vectorn& a, const vectorn& b, int duration, const vectorn& c, const vectorn& d)
{
	vectorn vel1, vel2;
	vel1.sub(b,a);
	vel2.sub(d,c);

	float totalTime=duration+1;
	vel1/=(1.f/totalTime);	// 시간으로 나눠주었다. 단위는 1프레임
	vel2/=(1.f/totalTime);	// 시간으로 나눠주었다. 단위는 1프레임

	out.setSize(duration, a.size());

	for(int i=0; i<duration; i++)
	{
		for(int j=0; j<a.size(); j++)
			out(i,j)=_hermite(a[j], vel1[j], d[j], vel2[j], (double)(i+1)/totalTime);
	}
}
void v::hermite(vectorn& out, double t, double T, const vectorn& a, const vectorn va, const vectorn& b,  const vectorn& vb)
{
	out.setSize(a.size());
	for(int j=0; j<a.size(); j++)
		out(j)=_hermite(a[j], va[j]*T, b[j], vb[j]*T, t/T);
}
/*

void vm1::curvature::calc(vectorn& c, const matrixn& pos) const
{
	//http://mathworld.wolfram.com/Curvature.html

	//k = 	(| r.^ x r^..|)/(| r^. |^3)

	matrixn vel;
	matrixn acc;

	vel.derivative(pos);
	acc.derivative(vel);

	c.setSize(pos.rows());

	for(int i=0; i<c.size(); i++)
	{
		c[i]= vel[i].toVector3().cross(acc[i].toVector3()).length();
		c[i]/=CUBIC(vel[i].length());
	}
}

void vm1::_calcUtility(const sv1::Operator& cOP, vectorn& c, const matrixn& a)
{
	// matrix의 각 column vector들을 aggregate한다. (결과 vector dim은 cols())
	vectorn column;
	c.setSize(a.cols());
	
	for(int i=0; i<a.cols();i++)
	{
		a.getColumn(i, column);
		c[i]=cOP.calc(column);
	}
}

void vm1::each::calc(vectorn& c, const matrixn& a) const
{
	// matrix의 각 row vector들을 aggregate한다. (결과 vector dim은 rows())
	const matrixn& mat=a;
	c.setSize(mat.rows());
	for(int i=0; i<mat.rows(); i++)
		c[i]=m_cOperator.calc(mat[i]);
}

// a에서 일부만 뽑아서 계산.
vm1::domain::domain(int start, int end, int stepSize, const vm1::Operator& op)
	:m_op(op)
{
	m_aIndex.colon(start, end, stepSize);
}

void vm1::domain::calc(vectorn& c, const matrixn& a) const
{
	matrixn temp;
	temp.extractRows(a, m_aIndex);
	m_op.calc(c, temp);
}

vm1::domainRange::domainRange(int start, int end, int stepSize, const vm1::Operator& op)
	:m_op(op)
{
	m_aInOutIndex.colon(start, end, stepSize);
}

void vm1::domainRange::calc(vectorn& c, const matrixn& a) const
{
	// unary operator 에서 c는 사용하지 않는 경우가 있다. 이경우 사용자가 c의 크기를 일일이 세팅하지 않는경우가 있다.
	// 이경우 양쪽 크기가 다르면 에러가 날수 있으므로 아래를 추가하였다. c와 a의 크기가 같은 일반적인 경우 영향없다.
	c.setSize(a.rows());

	matrixn tempa;
	vectorn tempc;
	tempa.extractRows(a, m_aInOutIndex);
	tempc.extract(c, m_aInOutIndex);
	m_op.calc(tempc, tempa);

	c.op1(v1::assign(m_aInOutIndex), tempc);
}

m_real sv1::domain::calc(const vectorn& c) const
{
	vectorn temp;
	temp.extract(c, m_aIndex);
	return temp.op1(m_op);
}

m_real sv1::_calcUtil(CAggregate::aggregateOP eOP, const vectorn& c) 
{
	CAggregate cOP(eOP);
	m_real cur=cOP.Init();

	for( int i=0; i<c.size(); i++ )
		cOP.Update(cur, c[i]);
	
	return cOP.Final(cur, c.size());
}



void NR_OLD::SVdecompose( matrixn &a , vectorn& w, matrixn& v )
{
	int m = a.rows();
	int n = a.cols();

	w.setSize( n );
	v.setSize( n, n );

	int flag, i, its, j, jj, k, l, nm;
	m_real anorm, c, f, g, h, s, scale, x, y, z;

  	static vectorn rv1; rv1.setSize( n );
	g = scale = anorm = 0.0;

	for( i=0; i<n; i++ )
	{
		l = i + 1;
		rv1[i] = scale * g;
		g = s = scale = 0.0;

		if ( i<m )
		{
			for ( k=i; k<m; k++ )
				scale += fabs(a[k][i]);

			if ( scale )
			{
				for ( k=i; k<m; k++ )
				{
					a[k][i] /= scale;
					s += a[k][i] * a[k][i];
				}

				f = a[i][i];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				a[i][i] = f - g;

				for( j=l; j<n; j++ )
				{
					for ( s=0.0, k=i; k<m; k++ )
						s += a[k][i] * a[k][j];
					f = s / h;

					for ( k=i; k<m; k++ )
						a[k][j] += f * a[k][i];
				}

				for( k=i; k<m; k++ )
					a[k][i] *= scale;
			}
		}

		w[i] = scale * g;
		g = s = scale = 0.0;

		if ( i<m && i != n-1)
		{
			for( k=l; k<n; k++)
				scale += fabs(a[i][k]);

			if ( scale )
			{
				for( k=l; k<n; k++ )
				{
					a[i][k] /= scale;
					s += a[i][k] * a[i][k];
				}

				f = a[i][l];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				a[i][l] = f - g;

				for ( k=l; k<n; k++ )
					rv1[k] = a[i][k] / h;

				for( j=l; j<m; j++ )
				{
					for( s=0.0, k=l; k<n; k++ )
						s += a[j][k] * a[i][k];

					for( k=l; k<n; k++ )
						a[j][k] += s * rv1[k];
				}

				for( k=l; k<n; k++ )
					a[i][k] *= scale;
			}
		}

		anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
	}

	for( i=n-1; i>=0; i-- )
	{
		if ( i<n-1 )
		{
			if ( g )
			{
				for( j=l; j<n; j++ )
					v[j][i] = (a[i][j] / a[i][l]) / g;

				for ( j=l; j<n; j++ )
				{
					for( s=0.0, k=l; k<n; k++ )
						s += a[i][k] * v[k][j];

					for( k=l; k<n; k++ )
						v[k][j] += s * v[k][i];
				}
			}

			for( j=l; j<n; j++ )
				v[i][j] = v[j][i] = 0.0;
		}

		v[i][i] = 1.0;
		g = rv1[i];
		l = i;
	}

	for( i=MIN(m, n)-1; i>=0; i-- )
	{
		l = i + 1;
		g = w[i];
		for( j=l; j<n; j++ )
		a[i][j] = 0.0;

		if ( g )
		{
			g = (m_real)1.0 / g;
			for( j=l; j<n; j++ )
			{
				for ( s=0.0, k=l; k<m; k++ )
					s += a[k][i] * a[k][j];

				f = (s / a[i][i]) * g;

				for( k=i; k<m; k++ )
					a[k][j] += f * a[k][i];
			}

			for( j=i; j<m; j++ )
				a[j][i] *= g;
		}
		else
			for( j=i; j<m; j++ )
				a[j][i] = 0.0;

		++a[i][i];
	}

	for( k=n-1; k>=0; k-- )
	{
		for( its=1; its<30; its++ )
		{
			flag = 1;
			for( l=k; l>=0; l-- )
			{
				nm = l - 1;
				if ((m_real) (fabs(rv1[l]) + anorm) == anorm)
				{
					flag = 0;
					break;
				}

				if ((m_real) (fabs(w[nm]) + anorm) == anorm) break;
			}

			if ( flag )
			{
				c = 0.0;
				s = 1.0;

				for( i=l; i<= k; i++ )
				{
					f = s * rv1[i];
					rv1[i] = c * rv1[i];

					if ((m_real) (fabs(f) + anorm) == anorm) break;

					g = w[i];
					h = pythag(f, g);
					w[i] = h;
					h = 1.0f / h;
					c = g * h;
					s = -f * h;

					for( j=0; j<m; j++ )
					{
						y = a[j][nm];
						z = a[j][i];
						a[j][nm] = y * c + z * s;
						a[j][i] = z * c - y * s;
					}
				}
			}

			z = w[k];
			if ( l == k )
			{
				if ( z < 0.0 )
				{
					w[k] = -z;
					for( j=0; j<n; j++ )
						v[j][k] = -v[j][k];
				}
				break;
			}

			if (its == 29)
				error("no convergence in 30 svdcmp iterations");

			x = w[l];
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0f * h * y);
			g = pythag(f, 1.0f);
			f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
			c = s = 1.0f;

			for( j=l; j<=nm; j++ )
			{
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;

				for( jj=0; jj<n; jj++ )
				{
					x = v[jj][j];
					z = v[jj][i];
					v[jj][j] = x * c + z * s;
					v[jj][i] = z * c - x * s;
				}

				z = pythag(f, h);
				w[j] = z;

				if ( z )
				{
					z = 1.0f / z;
					c = f * z;
					s = h * z;
				}

				f = c * g + s * y;
				x = c * y - s * g;

				for( jj=0; jj<m; jj++ )
				{
					y = a[jj][j];
					z = a[jj][i];
					a[jj][j] = y * c + z * s;
					a[jj][i] = z * c - y * s;
				}
			}

			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
	}
}

void NR_OLD::SVsubstitute( const matrixn& u, vectorn const& w, matrixn const& v,
            const vectorn& b, vectorn &x ) 
{
    assert( u.cols() == w.getSize() );
    assert( u.cols() == v.cols() );
    assert( u.cols() == v.rows() );
    assert( u.rows() == b.getSize() );
    assert( u.cols() == x.getSize() );

    int m = u.rows();
    int n = u.cols();

    int jj,j,i;
    m_real s;
    static vectorn tmp; tmp.setSize(n);

    for (j=0;j<n;j++) {
        s=0.0;
        if (w[j]>EPS) {
            for (i=0;i<m;i++) s += u[i][j]*b[i];
            s /= w[j];
        }
        tmp[j]=s;
    }
    for (j=0;j<n;j++) {
        s=0.0;
        for (jj=0;jj<n;jj++) s += v[j][jj]*tmp[jj];
        x[j]=s;
    }
}

void NR_OLD::SVinverse( matrixn& in_u, matrixn &mat ) 
{
    int m = in_u.rows();
    int n = in_u.cols();

    static matrixn V; V.setSize( n, n );
    static vectorn w; w.setSize( n );

    static vectorn b; b.setSize( m );
    static vectorn x; x.setSize( n );

	mat.setSize( n, m );

    SVdecompose( in_u, w, V );
    for( int j=0; j<m; j++ )
    {
        for( int i=0; i<m; i++ ) b[i] = 0;
        b[j] = 1.0;

        SVsubstitute(in_u, w, V, b, x );

        for( i=0; i<n; i++ )
            mat[i][j] = x[i];
    }
}

m_real NR_OLD::pythag( m_real a, m_real b )
{
	m_real pa = fabs( a );
	m_real pb = fabs( b );

	if ( pa > pb ) return pa * sqrt( 1.0f + SQR(pb / pa) );
	else return (pb==0.0f ? 0.0f : pb * sqrt(1.0f + SQR(pa / pb)));
}

void NR_OLD::tred2(matrixn& a, vectorn& d, vectorn& e)
{
	int l,k,j,i;
	double scale,hh,h,g,f;

	ASSERT(a.rows()==a.cols());

	int n=a.rows();
	d.setSize(n);
	e.setSize(n);

	for (i=n-1;i>0;i--) {
		l=i-1;
		h=scale=0.0;
		if (l > 0) {
			for (k=0;k<l+1;k++)
				scale += fabs(a[i][k]);
			if (scale == 0.0)
				e[i]=a[i][l];
			else {
				for (k=0;k<l+1;k++) {
					a[i][k] /= scale;
					h += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g=(f >= 0.0 ? -sqrt(h) : sqrt(h));
				e[i]=scale*g;
				h -= f*g;
				a[i][l]=f-g;
				f=0.0;
				for (j=0;j<l+1;j++) {
				// Next statement can be omitted if eigenvectors not wanted
					a[j][i]=a[i][j]/h;
					g=0.0;
					for (k=0;k<j+1;k++)
						g += a[j][k]*a[i][k];
					for (k=j+1;k<l+1;k++)
						g += a[k][j]*a[i][k];
					e[j]=g/h;
					f += e[j]*a[i][j];
				}
				hh=f/(h+h);
				for (j=0;j<l+1;j++) {
					f=a[i][j];
					e[j]=g=e[j]-hh*f;
					for (k=0;k<j+1;k++)
						a[j][k] -= (f*e[k]+g*a[i][k]);
				}
			}
		} else
			e[i]=a[i][l];
		d[i]=h;
	}
	// Next statement can be omitted if eigenvectors not wanted
	d[0]=0.0;
	e[0]=0.0;
	// Contents of this loop can be omitted if eigenvectors not
	//	wanted except for statement d[i]=a[i][i];
	for (i=0;i<n;i++) {
		l=i;
		if (d[i] != 0.0) {
			for (j=0;j<l;j++) {
				g=0.0;
				for (k=0;k<l;k++)
					g += a[i][k]*a[k][j];
				for (k=0;k<l;k++)
					a[k][j] -= g*a[k][i];
			}
		}
		d[i]=a[i][i];
		a[i][i]=1.0;
		for (j=0;j<l;j++) a[j][i]=a[i][j]=0.0;
	}
}


void NR_OLD::tqli(matrixn& z, vectorn& d, vectorn& e)
{
	int size=d.size();

	int m,l,iter,i,k;
	double s,r,p,g,f,dd,c,b;

	int n=size;
	for (i=1;i<n;i++) e[i-1]=e[i];
	e[n-1]=0.0;
	for (l=0;l<n;l++) {
		iter=0;
		do {
			for (m=l;m<n-1;m++) {
				dd=fabs(d[m])+fabs(d[m+1]);
				if (fabs(e[m])+dd == dd) break;
			}
			if (m != l) {
				if (iter++ == 30) {
//					MESSAGE("Too many iterations in tqli");
					return;
				}
				g=(d[l+1]-d[l])/(2.0*e[l]);
				r=pythag(g,1.0);
				g=d[m]-d[l]+e[l]/(g+SIGN(r,g));
				s=c=1.0;
				p=0.0;
				for (i=m-1;i>=l;i--) {
					f=s*e[i];
					b=c*e[i];
					e[i+1]=(r=pythag(f,g));
					if (r == 0.0) {
						d[i+1] -= p;
						e[m]=0.0;
						break;
					}
					s=f/r;
					c=g/r;
					g=d[i+1]-p;
					r=(d[i]-g)*s+2.0*c*b;
					d[i+1]=g+(p=s*r);
					g=c*r-b;
					// Next loop can be omitted if eigenvectors not wanted
					for (k=0;k<n;k++) {
						f=z[k][i+1];
						z[k][i+1]=s*z[k][i]+c*f;
						z[k][i]=c*z[k][i]-s*f;
					}
				}
				if (r == 0.0 && i >= l) continue;
				d[l] -= p;
				e[l]=g;
				e[m]=0.0;
			}
		} while (m != l);
	}
}*/



void s1::COS(m_real&b,m_real a)  {b= (m_real)cos(a);}
void s1::SIN(m_real&b,m_real a)  {b= (m_real)sin(a);}
void s1::EXP(m_real&b,m_real a)  {b= (m_real)exp(a);}
void s1::NEG(m_real&b,m_real a)  {b= -1*a;}
void s1::SQRT(m_real&b,m_real a)  { b= sqrt(a);}
void s1::SQUARE(m_real&b,m_real a)  { b= a*a;}
void s1::ASSIGN(m_real&b,m_real a)  { b= a;}
void s1::LOG(m_real&b,m_real a)  { b= log(a);}
void s1::abs(m_real&b,m_real a)  { b= (m_real)ABS(a);}
void s1::SMOOTH_TRANSITION(m_real&b,m_real a)  { b= ((m_real)-2.0)*a*a*a+((m_real)3.0)*a*a;} // y=-2x^3+3x^2
void s1::RADD(m_real&b,m_real a)  { b+=a;}
void s1::RDIV(m_real&b,m_real a)  { b/=a;}
void s1::RSUB(m_real&b,m_real a)  { b-=a;}
void s1::RMULT(m_real&b,m_real a)  { b*=a;}
void s1::BOUND(m_real&b, m_real a)  { b=CLAMP(b, -1*a, a);}
void s1::INVERSE(m_real&b, m_real a) { b=1.0/a;	}


m_real s2::ADD(m_real a, m_real b)  {return a+b;}
m_real s2::SUB(m_real a, m_real b)  {return a-b;}
m_real s2::MULT(m_real a, m_real b)  {return a*b;}
m_real s2::DIV(m_real a, m_real b)  {return a/b;}
m_real s2::POW(m_real a, m_real b)  {return pow(a,b);}
m_real s2::MINIMUM(m_real a, m_real b)  {return MIN(a,b);}
m_real s2::MAXIMUM(m_real a, m_real b)  {return MAX(a,b);}
m_real s2::GREATER(m_real a, m_real b)  { return (m_real) a>b;}
m_real s2::GREATER_EQUAL(m_real a, m_real b)  { return (m_real) a>=b;}
m_real s2::SMALLER(m_real a, m_real b)  { return (m_real) a<b;}
m_real s2::SMALLER_EQUAL(m_real a, m_real b)  { return (m_real) a<=b;}
m_real s2::EQUAL(m_real a, m_real b)  { return (m_real) a==b;}
m_real s2::AVG(m_real a, m_real b)  { return (a+b)/(m_real)2.0;}
m_real s2::BOUND(m_real a, m_real b)  { return CLAMP(a, -1*b, b);}
int s2::INT_NOT_EQUAL(int a, int b)		{ return a!=b;}
int s2::INT_EQUAL(int a, int b)		{ return a==b;}

m_real CAggregate::InitZero() const	{ return 0;};
m_real CAggregate::InitFMax() const	{ return FLT_MAX;};
m_real CAggregate::InitFMin() const	{ return -FLT_MAX;};
void CAggregate::UpdateSquareSum(m_real &cur, m_real v) const { cur+=v*v;}
void CAggregate::UpdateMin(m_real &cur, m_real v) const{ cur=MIN(cur,v);}
void CAggregate::UpdateMax(m_real &cur, m_real v) const{ cur=MAX(cur,v);}
void CAggregate::UpdateSum(m_real &cur, m_real v) const{ cur+=v;}
m_real CAggregate::FinalSqrt(m_real cur, int n) const	{ return sqrt(cur);}
m_real CAggregate::FinalCur(m_real cur, int n) const	{ return (cur);}
m_real CAggregate::FinalDivN(m_real cur, int n) const	{ return (cur/(m_real)n);}
