#pragma once

#include "Interpolation.h"
class Function
{
public:
	Function():mIsLearned(false){}
	virtual ~Function(){}

	// Test any function class using simple one or two dimensional samples.
	void test(const char* id);
	// numSample<20
	void test2(const char* id, int numSample);
	// plot을 해본후, 모델을 정해서 learning한다.
	void plot(const matrixn& sources, const matrixn& targets, const char* identifier) ;
	virtual void learn(const matrixn& sources, const matrixn& targets)	{mIsLearned=true; mDimSource=sources.cols();mDimTarget=targets.cols();}
	// PCA나 GPLVM같이 target space가 hidden인 경우 이함수를 상속해 사용한다.
	virtual void learn(const matrixn& sources)	{ mIsLearned=true; mDimSource=sources.cols(); mDimTarget=-1;}
	virtual bool isLearned() const { return mIsLearned;}
	int dimDomain() const		{ return mDimSource;}
	int dimRange() const		{ return mDimTarget;}

	TArray<TString> mSourceDimName;
	TArray<TString> mTargetDimName;
	virtual void mapping(const vectorn& source, vectorn& target) const =0;
	void mapping(const matrixn& sources, matrixn& targets) const;

	virtual m_real logProb(const vectorn& source) const							{Msg::error("logProb"); return 0.0;}

	// 일반적으로 function class는 하나의 source에 대해서 distribution을 계산해놓으면,
	// 다양한 target에 대해서 빠르게 확률을 계산할 수 있다. 즉 계산 속도를 최적화하기 위해 둘을 분리해놓았다.
	virtual void prepare(const vectorn& source) const							{ Msg::error("prepare"); }
	virtual m_real logProbPrepared(const vectorn& target) const					{ Msg::error("logprobprepaired");return 0.0;}

	// P(target|source)
	virtual m_real logProb(const vectorn& source, const vectorn& target) const
	{
		prepare(source);
		return logProbPrepared(target);
	}

	virtual void variance(const vectorn& source, vectorn& var) const			{var.setSize(0);}

	// following functions are deprecated
	// some function classes provide the probability estimation

	// max_{Y* in Y} P(Y*|source)
	//virtual m_real logProbMax(const vectorn& source) const						{Msg::error("logProbMax"); return 0.0;}
	// 입력으로 사용된 sample중 가장 확률이 높은것을 선택한다.
	//virtual void mappingNearest(const vectorn& source, vectorn& target) const { Msg::error("MappingNear");}

	//	virtual m_real prob(const vectorn& source) const							{Msg::error("prob"); return 1.0;}
//	virtual m_real prob(const vectorn& source, const vectorn& target) const		{Msg::error("prob2"); return 1.0;}

protected:
	int mDimSource;
	int mDimTarget;
	bool mIsLearned;
};

class BijectiveFunction : public Function
{
public:
	BijectiveFunction(void);
	virtual ~BijectiveFunction(void);

	virtual void inverseMapping(const vectorn& target, vectorn& source) const =0;
};

template <class OtherFunction>
class TClampFunction : public OtherFunction
{
public:
	TClampFunction(){}
	virtual ~TClampFunction(){}

	virtual void learn(const matrixn& sources, const matrixn& targets)
	{
		mRange.calcRange(sources);
		OtherFunction::learn(sources, targets);
	}

	virtual void mapping(const vectorn& source, vectorn& target) const
	{
		vectorn clampedSource;
		mRange.project(source, clampedSource);
		OtherFunction::mapping(clampedSource, target);
	}
	intervalN mRange;
};


class UniversalFunction : public Function
{
public:
	// parameters will be deleted in the destructor.
	UniversalFunction (Function* TrendEstimator, Function* ResidualEstimator)
		:mTrendEstimator(TrendEstimator), mResidualEstimator(ResidualEstimator) {}
	virtual ~UniversalFunction() { delete mTrendEstimator; delete mResidualEstimator;}

	virtual void learn(const matrixn & sources, const matrixn& targets);
	virtual void mapping(const vectorn& source, vectorn& target) const;

private:
	Function* mTrendEstimator;
	Function* mResidualEstimator;
	matrixn mTrends;
	matrixn mResiduals;
};

template <class TrendEstimator,class ResidualEstimator>
class TUniversalFunction : public Function
{
public:
	// parameters will be deleted in the destructor.
	TUniversalFunction ():mTrendEstimator(), mResidualEstimator() {}
	virtual ~TUniversalFunction() {}

	virtual void learn(const matrixn & sources, const matrixn& targets)
	{
		Function::learn(sources, targets);

		mTrendEstimator.learn(sources, targets);

		mTrends.setSameSize(targets);

		for(int i=0; i<targets.rows(); i++)
			mTrendEstimator.mapping(sources.row(i), mTrends.row(i).lval());

		mResiduals.subtract(targets, mTrends);

		mResidualEstimator.learn(sources, mResiduals);
	}

	virtual void mapping(const vectorn& source, vectorn& target) const
	{
		vectorn trend;
		mTrendEstimator.mapping(source, trend);
		mResidualEstimator.mapping(source, target);
		target+=trend;
	}

private:
	TrendEstimator mTrendEstimator;
	ResidualEstimator mResidualEstimator;
	matrixn mTrends;
	matrixn mResiduals;
};

class NormalizedFunction : public Function
{
public:
	NormalizedFunction (Function* other):mFunction(other){}
	virtual ~NormalizedFunction () {delete mFunction;}

	/**
	* 이걸 call하면, 주어진 min, max가 0, 1이 되도록 normalize한다.
	* 즉, normalize하지 않으려면 min에 0, max에 1을 준다.
	* 이함수를 call하지 않으면, sourcesample의 min, max를 자동으로 추출하여 사용한다.
	*/
	void setMinMax(const vectorn& min, const vectorn& max) {m_cNormalizingInterval.assign(min, max);}

	virtual void learn(const matrixn & sources, const matrixn& targets)
	{
		Function::learn(sources, targets);
		_initialize(sources);
		mFunction->learn(m_aNormalizedSamples, targets);
	}

	virtual void mapping(const vectorn& source, vectorn& target) const
	{
		vectorn normalizedSource;
		m_cNormalizingInterval.uninterpolate(source, normalizedSource);
		mFunction->mapping(normalizedSource, target);
	}

	/// _initialize 후에 사용할 수 있는 함수.
	int dimension()	const						{ return m_aNormalizedSamples.cols();}
	int numSample()	const						{ return m_aNormalizedSamples.rows();}
	vectornView sample(int i) const			{ return m_aNormalizedSamples.row(i);}
	const matrixn& allSample() const			{ return m_aNormalizedSamples;}

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

protected:
	void _initialize(const matrixn & sources);
	matrixn m_aNormalizedSamples;
	intervalN m_cNormalizingInterval;

	// m_vMin, m_vMax를 [0,1]범위로 scale했을때 실제 data들의 min, max범위.
	vectorn m_vResultMin;
	vectorn m_vResultMax;
	vectorn m_vMean;
	vectorn m_vStddev;

	Function* mFunction;
};

class LinearFunction : public BijectiveFunction
{
public:
	LinearFunction(){}
	virtual ~LinearFunction(){}

	void learn(const matrixn& sources, const matrixn& targets);

	virtual void mapping(const vectorn& source, vectorn& target) const;
	virtual void inverseMapping(const vectorn& target, vectorn& source) const;

	matrixn mLinearCoef;
	matrixn mInvLinearCoef;
};
class LeastSquareFitting : public Function
{
public:
	LeastSquareFitting(){}
	virtual ~LeastSquareFitting(){}

	void learn(const matrixn& sources, const matrixn& targets);

	virtual void mapping(const vectorn& source, vectorn& target) const;

	vector3 c;
};

class InterpolationFunction : public Function
{
public:
	InterpolationFunction (InterpolationNormalize* pint):mInterpolation(pint){}
	virtual ~InterpolationFunction (){delete mInterpolation;}

	virtual void learn(const matrixn& sources, const matrixn& targets);
	virtual void mapping(const vectorn& source, vectorn& target) const;

	InterpolationNormalize* mInterpolation;
	matrixn mTarget;
};

class NonlinearFunctionIDW : public Function
{
public:
	NonlinearFunctionIDW (int k=30, float power=2.f):Function(),mInterpolation(new L2Metric(), k, power){}
	NonlinearFunctionIDW (Metric* pMetric, int k=30, float power=2.f):Function(),mInterpolation(pMetric, k, power){}
	virtual ~NonlinearFunctionIDW (){}

	void changeMetric(Metric* pMetric){mInterpolation.m_pMetric=pMetric;}
	virtual void learn(const matrixn& sources, const matrixn& targets);
	virtual void mapping(const vectorn& source, vectorn& target) const;
	void mapping(const vectorn& source, vectorn& target, vectorn& weights) const;
	void mapping(const vectorn& source, vectorn& target, intvectorn & index, vectorn& weights) const;

	KNearestInterpolation mInterpolation;
	matrixn mTarget;
	intvectorn index;
	vectorn weight;
};

class tpros;
class TPros :public Function
{
public:
	TPros (const char* option="tpros spec/tspec");
	virtual ~TPros ();
	virtual void learn(const matrixn& sources, const matrixn& targets)	;
	virtual void mapping(const vectorn& source, vectorn& target) const ;

	m_real bestProb(const vectorn& source) const;

	virtual m_real prob(const vectorn& source) const	{		return bestProb(source);	}
	virtual m_real prob(const vectorn& source, const vectorn& target) const;

	void pack(BinaryFile& file);
	void unpack(BinaryFile& file);

	TString mOption;
	std::vector< tpros* > pGP;
	int mNumTrainingData;
	virtual bool isLearned() const { return pGP.size()!=0; }
};

class TProsFromFile:public TPros
{
public:
	TProsFromFile(const char* option="tpros spec/tspec"):TPros(option){}
	virtual ~TProsFromFile(){}
	virtual void learn(const matrixn& sources, const matrixn& targets);
	static bool mbSave;
	static BinaryFile* mpFile;
	static void testPackUnpack();
};

#include "PCA.H"

class FunctionUtil
{
public:
	FunctionUtil(){}
	virtual ~FunctionUtil(){}

	void addSample(const vectorn& source, const vectorn& target)
	{
		mSource.pushBack(source);
		mTarget.pushBack(target);
	}

	void learn(Function& func)
	{
		if(mName.length()) _dump(mSource, mTarget);
		func.learn(mSource, mTarget);
	}
	void learnInSubspace(Function& func, PCA& pca)
	{
		pca.Projection(mSource, mReducedSource);
		if(mName.length()) _dump(mReducedSource, mTarget);
		func.learn(mReducedSource, mTarget);
	}

	void learnInSubspace(Function& func, PCA& pca, PCA& tpca)
	{
		pca.Projection(mSource, mReducedSource);
		pca.Projection(mTarget, mReducedTarget);

		if(mName.length()) _dump(mReducedSource, mReducedTarget);
		func.learn(mReducedSource, mReducedTarget);
	}

	void plot(Function& func, const char* id) {func.plot(mSource, mTarget, id);}

	int numSample() const	{ return mSource.rows();}


	// utility funcitons.
	void enableDump(const char* filename)	{	mName=filename;	}
	void learnFromDumpfile(const char* filename, Function& func, int outdim=-1);
	matrixn const& source() const		{return mSource;}
	matrixn const& target() const		{return mTarget;}
private:
	void _dump(matrixn const& source, matrixn const& target);
	TString mName;
	matrixn mSource, mTarget;
	matrixn mReducedSource;
	matrixn mReducedTarget;
};
class BayesianLinearRegression :
	public Function
{
public:
	int m, d, N;
	m_real alpha, v;	// V=Identity(d)*v
	matrixn X,Y;		// data matrix
	matrixn XXt, InvXXt, YXt, YYt; 
	matrixn Sy_x, cross, YXtInvXXt;
	int mOrder;

	// prepare 
	mutable vectorn __mean;
	mutable m_real __stdev;

	BayesianLinearRegression(int order=1, m_real noisePrior=0.001);
	virtual ~BayesianLinearRegression(void);

	m_real calcEvidence();

	void mean(const vectorn& source, vectorn & x, vectorn& mean) const;

	/////////////////////////////////
	// reimplements Function 
	/////////////////////////////////

	virtual void learn(const matrixn& sources, const matrixn& targets);
	virtual void mapping(const vectorn& source, vectorn& target) const;
	virtual void variance(const vectorn& source, vectorn& var) const;

	virtual void prepare(const vectorn& source) const;
	virtual m_real logProbPrepared(const vectorn& target) const;

	//virtual void mappingNearest(const vectorn& source, vectorn& target) const;
	//virtual m_real logProb(const vectorn& source) const;
	//virtual m_real logProbMax(const vectorn& source) const;
	//virtual m_real logProb(const vectorn& source, const vectorn& target);
	//virtual m_real prob(const vectorn& source) const;
	//virtual m_real prob(const vectorn& source, const vectorn& target);
};
