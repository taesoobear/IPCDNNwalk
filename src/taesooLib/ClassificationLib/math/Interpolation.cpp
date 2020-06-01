#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "Interpolation.h"
#include "../BaseLib/image/imageclass.h"
#include "../BaseLib/utility/util.h"
#include "statistics.h"
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/math/Operator_NR.h"
InterpolationNormalize::InterpolationNormalize(void)
{
}

InterpolationNormalize::~InterpolationNormalize(void)
{
}

void InterpolationNormalize::initialize(const matrixn& sourceSamples)
{
	if(m_cNormalizingInterval.size()==0)
		m_cNormalizingInterval.calcRange(sourceSamples);

	m_aNormalizedSamples.assign(sourceSamples);
	m_aNormalizedSamples.normalize(m_cNormalizingInterval.start(), m_cNormalizingInterval.end());

	m_vResultMin.minimum(m_aNormalizedSamples);
	m_vResultMax.maximum(m_aNormalizedSamples);

	m_vMean.setSize(m_aNormalizedSamples.cols());
	m_vStddev.setSize(m_aNormalizedSamples.cols());
	vectorn column;
	for(int i=0; i<m_aNormalizedSamples.cols(); i++)
	{
		m_aNormalizedSamples.getColumn(i,column);
		statistics::NormalDistribution n(column);
		m_vMean[i]=n.sampleMean();
		m_vStddev[i]=n.sampleStdDev();
	}
}

void InterpolationNormalize::calcWeight(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	vectorn nParameter;
	normalize(parameter, nParameter);
	calcWeightNormalized(nParameter, index, weight);
}

void InterpolationNormalize::normalize(const vectorn& unnormalizedSample, vectorn& normalizedSample) const
{
	m_cNormalizingInterval.uninterpolate(unnormalizedSample, normalizedSample);
}

void InterpolationNormalize::unnormalize(const vectorn& sample, vectorn& unnormalizedSample) const
{
	m_cNormalizingInterval.interpolate(sample, unnormalizedSample);
}

void InterpolationNormalize::calcNormalizedParameter(const vectorn& aZ, vectorn& normalizedParameter) const
{
	ASSERT(aZ.size()==dimension());
	normalizedParameter.setSize(aZ.size());


	for(int i=0; i<dimension(); i++)
	{
		statistics::NormalDistribution n(mean()[i], stddev()[i]);
		normalizedParameter[i]=n.X(aZ[i]);
	}
}

m_real radial_bases(m_real d, m_real sigma);
m_real radial_bases_rbf(m_real d, m_real sigma);

m_real InterpolationNormalize::score(const vectorn& vec) const
{
	m_real sigma;
	if(dimension()==2)
		sigma=0.1;
	else
		sigma=0.2;

	m_real score=0;
	// return minimum distance*-1
	for(int i=0; i< m_aNormalizedSamples.rows(); i++)
	{
		m_real distance=m_aNormalizedSamples.row(i).distance(vec);

		score+=radial_bases_rbf(distance, sigma);
	}

	score/=m_aNormalizedSamples.rows();
	return score;
	/*m_real minDist=FLT_MAX;

	// return minimum distance*-1
	for(int i=0; i< m_aNormalizedSamples.rows(); i++)
	{
		m_real distance=m_aNormalizedSamples[i].distance(vec);
		if(distance<minDist)
			minDist=distance;
	}

	return minDist*-1;*/
}

void InterpolationNormalize::Test(const vectorn& signal1D, const char* testID, int TestResolution)
{
	matrixn signal;
	signal.setSize(signal1D.size(), 1);
	signal.setColumn(0, signal1D);

	vectorn minv(1);
	vectorn maxv(1);
	minv[0]=0;
	maxv[0]=1;
	setMinMax(minv, maxv);
	initialize(signal);

	vectorn weight2;
	intvectorn index;
	matrixn weights2;
	weights2.setSize(signal.rows(), TestResolution);
	weights2.setAllValue(0);

	vectorn parameters;
	m_real min=signal1D.minimum();
	m_real max=signal1D.maximum();

	parameters.uniform(min-(max-min)*0.2, max+(max-min)*0.2, TestResolution);
	vectorn parameter(1);
	for(int i=0; i<TestResolution; i++)
	{
		parameter[0]=parameters[i];
		calcWeightNormalized(parameter, index, weight2);

		for(int j=0; j<index.size(); j++)
		{
			weights2[index[j]][i]=weight2[j];
		}
	}

	CImage* pImage=CImageProcessor::DrawChart(weights2, CImageProcessor::BAR_CHART);
	CImagePixel cip(pImage);

	for(int i=0; i<signal1D.size(); i++)
	{
		cip.DrawVertLine(parameters.argNearest(signal1D[i]), 0, cip.Height(), CPixelRGB8(0,0,0),true);
	}

	//CImageProcessor::SafeDelete(pImage, TString("weight_")+testID+TString(".bmp"));
}

/*
VoronoiInterpolation::VoronoiInterpolation()
:m_space(this)
{

}

void VoronoiInterpolation::initialize(const matrixn& sourceSamples)
{
	__super::initialize(sourceSamples);
	m_space.triangulate();
}

void VoronoiInterpolation::calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	// find appropriate simplex
	float min_distance=FLT_MAX;
	int arg_min;
	for(int i=0; i<m_space.numSimplex(); i++)
	{
		m_space.m_nCurrSimplex=i;
		m_space.calcWeight(parameter, weight);
		float distance=m_space.maxTdistance(weight);
		if(distance<min_distance)
		{
			min_distance=distance;
			arg_min=i;
		}
	}

	m_space.m_nCurrSimplex=arg_min;
	m_space.calcWeight(parameter, weight);
}
*/

void NoInterpolation::calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	Metric& metric=*m_pMetric;

	float minDist=FLT_MAX;
	int argMin=-1;
	for(int i=0; i<m_aNormalizedSamples.rows(); i++)
	{
		float dist=parameter.distance(m_aNormalizedSamples.row(i),&metric);
		if(dist<minDist)
		{
			argMin=i;
			minDist=dist;
		}
	}
	ASSERT(argMin!=-1);

	index.setSize(1);
	index[0]=argMin;
	weight.setSize(1);
	weight[0]=1.0;
}

KNearestInterpolation ::KNearestInterpolation (Metric* pMetric, int k, float power, float nw)
{
	m_pMetric=pMetric; m_nK=k; m_fK=power;m_fNW=nw;
}

void KNearestInterpolation::initialize(const matrixn& sourceSamples)
{
	InterpolationNormalize::initialize(sourceSamples);
}

void KNearestInterpolation::calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	vectorn distances;
	//	distances.op2(sv2::distance(), m_aNormalizedSamples, parameter);
	distances.setSize(m_aNormalizedSamples.rows());
	for(int i=0;i<distances.size();++i){
		distances[i]=m_pMetric->CalcDistance(m_aNormalizedSamples.row(i),parameter);
	}

	intvectorn sortedIndex;
	sortedIndex.sortedOrder(distances);

	int K=MIN(numSample()-1, m_nK);

	if(numSample()==1)
	{
		index.setSize(1); index[0]=0;
		weight.setSize(1); weight[0]=1;
	}
	else
	{
		do
		{
			index.setSize(K);
			weight.setSize(K);

			if(K==1)
			{
				weight[0]=1.f;
				index[0]=sortedIndex[0];
			}
			else if(K<0)
			{
				Msg::error("KNeearest???");
				break;
			}
			else
			{
				float maxDist=distances[sortedIndex[K-1]];
				for(int i=0; i<K; i++)
				{
					index[i]=sortedIndex[i];
					//weight[i]=1.f/MAX(pow(distances[sortedIndex[i]], (m_real)m_fK),FLT_EPSILON)-1.f/MAX(pow(distances[sortedIndex[K]], (m_real)m_fK) ,FLT_EPSILON);	// kovar siggraph 2004
					weight[i]=1.f/MAX(pow(distances[sortedIndex[i]]+maxDist*m_fNW, (m_real)m_fK),FLT_EPSILON)-1.f/MAX(pow(distances[sortedIndex[K]]+maxDist*m_fNW, (m_real)m_fK) ,FLT_EPSILON);	// kovar siggraph 2004
					//			weight[i]=1.f/MAX(SQR(distances[sortedIndex[i]]),0.001)-1.f/MAX(SQR(distances[sortedIndex[K]]) ,0.001);	// kovar siggraph 2004
				}

				weight/=weight.sum();
				K--;
			}
		}
		while(!isSimilar(weight.sum(), 1));
	}
}
/*
KNearestBoxerInterpolation ::KNearestBoxerInterpolation (int k, float power)
{
	m_nK=k; m_fK=power;
}

void KNearestBoxerInterpolation::initialize(const matrixn& sourceSamples)
{
	__super::initialize(sourceSamples);
	metric.m_vWeight.setSize(dimension());
	metric.m_vWeight.setAllValue(1.0);
}

void KNearestBoxerInterpolation::calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	vectorn distances;
	assert(m_aNormalizedSamples.cols()==parameter.size());
	distances.setSize(m_aNormalizedSamples.rows());
	for(int i=0;i<distances.size();++i){
		distances[i]=metric.CalcDistance(m_aNormalizedSamples[i],parameter);
	}
//	distances.op2(sv2::distance(), m_aNormalizedSamples, parameter);

	intvectorn sortedIndex;
	sortedIndex.sortedOrder(distances);

	int K=MIN(numSample()-1, m_nK);

	if(numSample()==1)
	{
		index.setSize(1); index[0]=0;
		weight.setSize(1); weight[0]=1;
	}
	else
	{
		do
		{
			index.setSize(K);
			weight.setSize(K);

			if(K==1)
			{
				weight[0]=1.f;
				index[0]=sortedIndex[0];
			}
			else
			{
				for(int i=0; i<K; i++)
				{
					index[i]=sortedIndex[i];
					weight[i]=1.f/MAX(pow(distances[sortedIndex[i]], (m_real)m_fK),FLT_EPSILON)-1.f/MAX(pow(distances[sortedIndex[K]], (m_real)m_fK) ,FLT_EPSILON);	// kovar siggraph 2004
					//			weight[i]=1.f/MAX(SQR(distances[sortedIndex[i]]),0.001)-1.f/MAX(SQR(distances[sortedIndex[K]]) ,0.001);	// kovar siggraph 2004
				}

				weight/=weight.sum();
				K--;
			}
		}
		while(!isSimilar(weight.sum(), 1));
	}
}*/

void IDWInterpolation::calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	vectorn distances;

	distances.each2(sv2::distance(), m_aNormalizedSamples, parameter);

	intvectorn sortedIndex;
	sortedIndex.sortedOrder(distances);

	index.colon(0, numSample());
	weight.setSize(numSample());
	for(int i=0; i<numSample(); i++)
		weight[i]=1.f/MAX(pow(distances[i], (m_real)m_fK),0.001);
	weight/=weight.sum();
}

void LCBInterpolation::initialize(const matrixn& sourceSamples)
{
	InterpolationNormalize::initialize(sourceSamples);

	m_interpolator.reset(dimension(), numSample());

	/*
	for(int i=0; i<numSample(); i++)
	{
		m_interpolator.addSample(sample(i));
	}*/

	// taesoo test
	vectorn temp;
	for(int i=0; i<numSample(); i++)
	{
		temp.assign(sample(i));
		temp.y()=0;
		m_interpolator.addSample(temp);
	}

	// m_interpolator automatically calculate R, but it is constant for all samples -.-
	// I will completely reimplement this !
}

void LCBInterpolation::calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	vectorn& allWeight=m_interpolator.eval(parameter);

	ASSERT(isSimilar(allWeight.sum(), 1));

	// full interpolation
	index.colon(0,allWeight.size());
	weight.assign(allWeight);
}

inline m_real B0(m_real u)
{
	return (m_real)(1.0/4.0*CUBIC(1-u));
}

inline m_real B1(m_real u)
{
	return (m_real)(1.0/4.0*(4-6*SQR(u)+3*CUBIC(u)));
}

m_real radial_bases(m_real d, m_real sigma)
{
	m_real x=d/sigma;
	if( x<0.0f ) x = -x;
	if( x>1.0f ) return 0.0f;
	return 1.0f       -6.0f*x*x +8.0f*x*x*x -3.0f*x*x*x*x;	// 4th order spline
}

m_real radial_bases_rbf(m_real d, m_real sigma)
{
	return exp(-1*SQR(d)/(2*SQR(sigma)));	// rbf kernel
}

m_real radial_bases_cubic(m_real d, m_real sigma)
{
	// cubic spline kernel
	m_real u,f;
	u = sqrt(d)/sigma;

	if (u<0.5 && u>=0)
		f = B1(2*u);
	else if (u>=0.5 && u<=1)
		f = B0(2*u-1);
	else
		f = 0;

	return f;
}

m_real radial_bases(const vectorn& center, m_real alpha, const vectorn& position)
{
	m_real distance=center.distance(position);
	return radial_bases(distance, alpha);
}

void LCBInterpolation2::initialize(const matrixn& sourceSamples)
{
	/*
	vectorn rbtest(10);
	for(int i=0; i<10; i++)
	{
		rbtest[i]=radial_bases((m_real(i-5)),3);
	}
	CString wstr;
	rbtest.output(wstr);
	MOutputToFile("weight.txt",("rbtest %s",wstr));
*/

	InterpolationNormalize::initialize(sourceSamples);

	// temporaries
	intvectorn iv;
	vectorn v;

	m_aLinearCoef.setSize(numSample(), dimension()+1);
	m_aRadialCoef.setSize(numSample(), dimension());

	// calculate dilation factor
	// heuristic : 2*min distance
	matrixn distMat;
	m::distanceMat(distMat, m_aNormalizedSamples);
	distMat.setDiagonal(FLT_MAX);

	if(distMat.rows()==1)
	{
		m_aDilationFactor.setSize(1);
		m_aDilationFactor[0]=1;
	}
	else
	{

	/*
		// heuristic : 2*min distance
		matrixn sortedDistMat;
		sortedDistMat.op2(OP_SORT, distMat, true);
		else
		{
			sortedDistMat.resize(sortedDistMat.rows(), MIN(sortedDistMat.cols()-1,3));
			m_aDilationFactor.aggregateEach(OP_AVG, sortedDistMat);
			m_aDilationFactor*=2;
		}*/

		// heuristic : max(min(distMat));
		vectorn min;
		min.minimum(distMat);
		m_aDilationFactor.setSize(numSample());
		m_aDilationFactor.setAllValue(min.minimum()*2);//.maximum());

	}

	// least square fitting of source parameters

	// parameter  linearCoef   weight for sample k
	// (x1 y1 z1 ... 1)  a1         1
	// (x2 y2 z2 ... 1)  a2         0
	// (x3 y3 z3 ... 1)  a3 =       0
	// ...
	// (xn yn zn ... 1)  d  =       0

	// linear system
	//  L               x(k)  =       e(k)

	matrixn L;
	L.setSize(numSample(), dimension()+1);
	L.setValue(0,0, m_aNormalizedSamples);
	L.setColumn(dimension(), 1);

	matrixn invL;
	// use pseudo inverse for the least square solution.
	m::pseudoInverse(invL,L);

	// construct radial basis

	// radial bases values                               radialCoef  residual
	// (R1(sample1) R2(sample1) R3(sample1) ... Rn(sample1))  h1    residual[1]
	// (R1(sample2) R2(sample2) R3(sample2) ... Rn(sample2))  h2    residual[2]
	// (                                                   )  .. =
	// (                                                   )  hn    residual[n]
	// linear system
	//     R                                                  h  =  redidual

	matrixn R(numSample(), numSample());
	matrixn invR;

	for(int i=0; i<numSample(); i++)
		for(int j=0; j<numSample(); j++)
			R[i][j]=radial_bases(sample(j), m_aDilationFactor[j], sample(i));

	m::pseudoInverse(invR,R);


	matrixn interpolant, residuals;
	interpolant.identity(numSample());

	vectorn residual(numSample());

	m_aRadialCoef.setSize(numSample(), invR.rows());
	for(int k=0; k<numSample(); k++)
	{
		// linear approximated position of sample k
		m_aLinearCoef.row(k).multmat(invL, interpolant.row(k));

		// compute residuals
		v.multmat(L, m_aLinearCoef.row(k));
		residual.sub(interpolant.row(k), v);

		m_aRadialCoef.row(k).multmat(invR, residual);
	}
}

void LCBInterpolation2::calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	index.colon(0,numSample());
	weight.setSize(numSample());

	vectorn parameter2;
	vectorn one(1);
	one[0]=1;
	for(int k=0; k<numSample(); k++)
	{
		parameter2.concat(parameter, one);
		weight[k]=parameter2 % m_aLinearCoef.row(k);

		// evaluate radial bases
		for(int kk=0; kk<numSample(); kk++)
			weight[k]+= radial_bases(sample(kk), m_aDilationFactor[kk], parameter)*m_aRadialCoef[k][kk];
	}

	m_real sum=weight.sum();
	m_real max=weight.maximum();
	//MOutputToFile("weight.txt", ("(%f,%f)",sum,max) );
	weight/=sum;


	//MOutputToFile("weight.txt", ("test start") );
}

void RBInterpolation::initialize(const matrixn& sourceSamples)
{
	InterpolationNormalize::initialize(sourceSamples);

	// temporaries
	intvectorn iv;
	vectorn v;

	m_aRadialCoef.setSize(numSample(), numSample());

	// calculate dilation factor
	// heuristic : 2*min distance
	matrixn distMat;
	m::distanceMat(distMat, m_aNormalizedSamples);
	distMat.setDiagonal(FLT_MAX);

	if(distMat.rows()==1)
	{
		m_aDilationFactor.setSize(1);
		m_aDilationFactor[0]=1;
	}
	else
	{

		/*
		// heuristic : 2*min distance
		matrixn sortedDistMat(distMat.rows(), distMat.cols());
		intvectorn sortedIndex;
		for(int i=0; i<distMat.rows(); i++)
			sortedDistMat.row(i).sort(distMat.row(i), sortedIndex);
		sortedDistMat.resize(sortedDistMat.rows(), MIN(sortedDistMat.cols()-1,3));
		m_aDilationFactor.aggregateEachColumn(CAggregate::AVG, sortedDistMat);
		m_aDilationFactor*=2;
		*/
		// heuristic : max(min(distMat));
		vectorn min;
		min.minimum(distMat);
		m_aDilationFactor.setSize(numSample());
		m_aDilationFactor.setAllValue(min.maximum());
	}

	// construct radial basis

	// radial bases values                                 radialCoef  weight for sample k
	// (R1(sample1) R2(sample1) R3(sample1) ... Rn(sample1))  h1          1
	// (R1(sample2) R2(sample2) R3(sample2) ... Rn(sample2))  h2          0
	// (                                                   )  ..     =
	// (                                                   )  hn          0
	// linear system
	//     R                                                  hk  =      ek

	matrixn R(numSample(), numSample());
	matrixn invR;

	for(int i=0; i<numSample(); i++)
		for(int j=0; j<numSample(); j++)
			R[i][j]=radial_bases(sample(j), m_aDilationFactor[j], sample(i));

	//invR.pseudoInverse(R);
	m::LUinvert(invR,R);

	matrixn interpolant;
	interpolant.identity(numSample());

	for(int k=0; k<numSample(); k++)
	{
		m_aRadialCoef.row(k).multmat(invR, interpolant.row(k));
	}

	//TString coef;
	//m_aRadialCoef.output(coef);
	//MOutputToFile("weight.txt", ("radial coef\n%s)",coef.ptr()) );


}

void RBInterpolation::calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
{
	index.colon(0,numSample());
	weight.setSize(numSample());

	for(int k=0; k<numSample(); k++)
	{
		// evaluate radial bases
		weight[k]=0;
		for(int kk=0; kk<numSample(); kk++)
			weight[k]+= radial_bases(sample(kk), m_aDilationFactor[kk], parameter)*m_aRadialCoef[k][kk];
	}

	m_real sum=weight.sum();
	m_real max=weight.maximum();
	//MOutputToFile("weight.txt", ("(%f,%f)",sum,max) );
	weight/=sum;


	//MOutputToFile("weight.txt", ("test start") );
}
