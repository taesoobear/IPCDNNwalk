#pragma once

#include "../../BaseLib/math/mathclass.h"
#include "../../BaseLib/math/intervalN.h"
#include "../../BaseLib/math/Metric.h"
/**
 * abstract class of interpolation classes
 *  Usage : initialize 후 calcWeight 또는 calcWeightNormalized를 call한다. (parameter가 normalize된 parameter인지 여부에 따라서)
 * \author taesoo
 *
 *
 */

namespace statistics
{
	class Interpolation
	{
	public:
		Interpolation(void);
		virtual ~Interpolation(void);

		/**
		* using input query, store all samples into m_aSamples
		* do additional initialization in the derived class
		* \param sourceSamples input query.
		*/
		virtual void initialize(const matrixn& sourceSamples);

		/// initialize 후에 사용할 수 있는 함수. 
		int dimension()	const						{ return m_aSamples.cols();}
		int numSample()	const						{ return m_aSamples.rows();}	
		vectornView sample(int i) const			{ return m_aSamples.row(i);}
		const matrixn& allSample() const			{ return m_aSamples;}

		virtual void calcWeight(const vectorn& parameter, intvectorn& index, vectorn& weight)=0;

		/// data들의 min
		const vectorn& minimum() const					{ return m_vResultMin;}
		/// data들의 max
		const vectorn& maximum() const					{ return m_vResultMax;}
		/// data들의 mean
		const vectorn& mean() const						{ return m_vMean;}
		/// date들의 stddev
		const vectorn& stddev()	const					{ return m_vStddev;}
	protected:
		matrixn m_aSamples;

		// 실제 data들의 min, max범위.
		vectorn m_vResultMin;	
		vectorn m_vResultMax;
		vectorn m_vMean;
		vectorn m_vStddev;
	};
}

// deprecated
class InterpolationNormalize
{
public:
	InterpolationNormalize(void);
	virtual ~InterpolationNormalize(void);

	/**
	 * 이걸 call하면, 주어진 min, max가 0, 1이 되도록 normalize한다.
	 * 즉, normalize하지 않으려면 min에 0, max에 1을 준다. 
	 * 이함수를 call하지 않으면, sourcesample의 min, max를 자동으로 추출하여 사용한다.
	 */
	void setMinMax(const vectorn& min, const vectorn& max) {m_cNormalizingInterval.assign(min, max);}
	/**
	 * using input query, store all normalize samples into m_aNormalizedSamples
	 * do additional initialization in the derived class
	 * \param sourceSamples input query.
	 */
	virtual void initialize(const matrixn& sourceSamples);

	/// initialize 후에 사용할 수 있는 함수. 
	int dimension()	const						{ return m_aNormalizedSamples.cols();}
	int numSample()	const						{ return m_aNormalizedSamples.rows();}	
	vectornView sample(int i) const			{ return m_aNormalizedSamples.row(i);}
    const matrixn& allSample() const			{ return m_aNormalizedSamples;}



	/// parameter가 normalized된경우
	virtual void calcWeightNormalized(const vectorn& normalizedParameter, intvectorn& index, vectorn& weight)=0;
	/// parameter가 normalized되지 않은 경우
	void calcWeight(const vectorn& parameter, intvectorn& index, vectorn& weight);

	void normalize(const vectorn& unnormalizedSample, vectorn& normalizedSample) const;
	void unnormalize(const vectorn& normalizedSample, vectorn& unnormalizedSample) const;

	virtual m_real score(const vectorn& vec) const;

	/// normalize된 data들의 min
	const vectorn& minimum() const					{ return m_vResultMin;}
	/// normalize된 data들의 max
	const vectorn& maximum() const					{ return m_vResultMax;}
	/// normalize된 date들의 mean
	const vectorn& mean() const						{ return m_vMean;}
	/// normalize된 date들의 stddev
	const vectorn& stddev()	const					{ return m_vStddev;}

	// normalize되지 않은 원래 space에서 값을 return
	m_real minimum(int dim) const					{ return m_cNormalizingInterval[dim].interpolate(m_vResultMin[dim]); }
	m_real maximum(int dim) const					{ return m_cNormalizingInterval[dim].interpolate(m_vResultMax[dim]); }

	/// standard normal distribution 에서의 z값을 입력으로 받는다. 즉 z가 0이면, mean이 출력된다.
	void calcNormalizedParameter(const vectorn& aZ, vectorn& normalizedParameter) const;
	
	/// 1차원에서 interpolation을 실험한다. 결과 weight시그널이 testID_weight.bmp에 저장된다.
	void Test(const vectorn& signal1D, const char* testID, int TestResolution=100);

	const intervalN& normalizingInterval() const	{ return m_cNormalizingInterval;}

protected:
	matrixn m_aNormalizedSamples;
	intervalN m_cNormalizingInterval;

	// m_vMin, m_vMax를 [0,1]범위로 scale했을때 실제 data들의 min, max범위.
	vectorn m_vResultMin;	
	vectorn m_vResultMax;
	vectorn m_vMean;
	vectorn m_vStddev;
};


/// Find closest sample
class NoInterpolation : public InterpolationNormalize
{
public:
	NoInterpolation(Metric* pMetric=NULL){ if(pMetric) m_pMetric=pMetric; else m_pMetric=new L2Metric();}
	virtual ~NoInterpolation(){ delete m_pMetric; }
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
	Metric* m_pMetric;
};

/// Find k closest samples. This is fast version of IDW interpolation.
class KNearestInterpolation : public InterpolationNormalize
{
public:
	KNearestInterpolation (Metric* pMetric, int k=4, float power=2.f, float noiseWeight=0.f);
	virtual ~KNearestInterpolation (){}
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
	
	/// weight= 1/distance^m_fK - 1/maxDistanceAmongK^m_fK
	int m_nK;
	float m_fK;
	float m_fNW;

	Metric* m_pMetric;	
};
/*
class KNearestBoxerInterpolation : public Interpolation
{
public:
	KNearestBoxerInterpolation (int k=4, float power=2.f);
	virtual ~KNearestBoxerInterpolation (){}
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);

	/// weight= 1/distance^m_fK - 1/maxDistanceAmongK^m_fK
	int m_nK;
	float m_fK;

	CBoxerMetric metric;
};*/

/// Inverse distance weighted interpolation by taesoo. This is the best interpolation class in most cases.
class IDWInterpolation : public InterpolationNormalize
{
public:
	IDWInterpolation(){m_fK=2.f;}
	virtual ~IDWInterpolation(){}
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
	
	/// weight= 1/distance^m_fK (default:2)
	float m_fK;
};

#include "LCBInterpolator.h"

/// Linear Cardinal Basis interpolation
class LCBInterpolation : public InterpolationNormalize
{
public:
	LCBInterpolation(){}
	virtual ~LCBInterpolation(){}
	
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
private:
	LCBInterpolator m_interpolator;
};

/// Linear Cardinal Basis interpolation2 by taesoo
class LCBInterpolation2 : public InterpolationNormalize
{
public:
	LCBInterpolation2(){}
	virtual ~LCBInterpolation2(){}
	
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
protected:	
	matrixn m_aLinearCoef;
	matrixn m_aRadialCoef;
	vectorn m_aDilationFactor;
};

/// Radial Basis interpolation by taesoo
class RBInterpolation : public InterpolationNormalize
{
public:
	RBInterpolation(){}
	virtual ~RBInterpolation(){}
	
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
private:	
	matrixn m_aRadialCoef;
	vectorn m_aDilationFactor;
};

class TestInterpolation: public InterpolationNormalize
{
public:
	TestInterpolation(){}
	TestInterpolation(const intvectorn& index, const vectorn& weight):m_index(index), m_weight(weight){}
    virtual ~TestInterpolation(){}

	void setTestVariable(const intvectorn& index, const vectorn& weight)
	{
		m_index=index;
		m_weight=weight;
	}

	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
	{
		index=m_index;
		weight=m_weight;
	}
	intvectorn m_index;
	vectorn m_weight;
};
