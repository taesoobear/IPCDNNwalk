#ifndef BIJECTIVEFUNCTION_H_
#define BIJECTIVEFUNCTION_H_
#pragma once

class PCA;
#include "../math/Function.h"
#include "../math/statistics.h"
namespace bijectiveFunctions
{

class PCA : public BijectiveFunction
{	
public:
	::PCA* mPCA;
	PCA (m_real errorGoal=0.95);
	void setReducedDim(int dim);
	virtual ~PCA();
	virtual void learn(const matrixn& sources);
	virtual void mapping(const vectorn& source, vectorn& target) const;
	virtual void inverseMapping(const vectorn& target, vectorn& source) const;
};

class Normalization : public BijectiveFunction
{
	vectorn mSourceMinimum;
	vectorn mSourceMaximum;
	m_real mMinRange;
	m_real mMaxRange;
public:
	// 각 dimension의 min, max
	Normalization (m_real minRange, m_real maxRange):mMinRange(minRange), mMaxRange(maxRange){}
	virtual ~Normalization (){}
	virtual void learn(const matrixn& sources)	{ mSourceMinimum.minimum(sources); mSourceMaximum.maximum(sources);
		BijectiveFunction::learn(sources); mDimTarget=mDimSource; }
	virtual void mapping(const vectorn& source, vectorn& target) const;
	virtual void inverseMapping(const vectorn& target, vectorn& source) const;
};

class Standardization: public BijectiveFunction
{
	vectorn mean;
	vectorn stddev;
public:
	// 각 dimension을 z=(x-mu)/sigma 가 되게하는 변환.
	Standardization(){}
	virtual ~Standardization(){}
	virtual void learn(const matrixn& sources);
	virtual void mapping(const vectorn& source, vectorn& target) const;
	virtual void inverseMapping(const vectorn& target, vectorn& source) const;
};

class Identity: public BijectiveFunction
{
public:
	Identity(){}
	virtual ~Identity(){}
	virtual void learn(const matrixn& sources)	{BijectiveFunction::learn(sources); mDimTarget=mDimSource;}
	virtual void mapping(const vectorn& source, vectorn& target) const			{ target=source;}
	virtual void inverseMapping(const vectorn& target, vectorn& source) const	{ source=target;} 
};
}

class DensityEstimationInTransformedSpace : public statistics::DensityEstimation
{
	statistics::DensityEstimation* mPDF;
	BijectiveFunction* m_pSourceFunction;
	// temporary vectors
	mutable vectorn tsource;
public:
	DensityEstimationInTransformedSpace (DensityEstimation* other):mPDF(other){}
	virtual ~DensityEstimationInTransformedSpace (){}

	statistics::DensityEstimation* _densiyEstimator() const	{return mPDF;}

	void setFunction( BijectiveFunction* sourceFunction)
	{
		m_pSourceFunction=sourceFunction;
	}
	
	virtual void learn(const matrixn& sources)
	{
		matrixn tsources(sources.rows(), m_pSourceFunction->dimRange());

		for(int i=0; i<sources.rows(); i++)
			m_pSourceFunction->mapping(sources.row(i), tsources.row(i).lval());
		mPDF->learn(tsources);		
	}

	virtual int dim() const	{ return mPDF->dim();}

	virtual m_real pdf(const vectorn& source) const
	{
		m_pSourceFunction->mapping(source, tsource);
		return mPDF->pdf(tsource);		
	}

	virtual m_real logPdf(const vectorn& source) const
	{
		m_pSourceFunction->mapping(source, tsource);
		return mPDF->logPdf(tsource);		
	}
};
class FunctionInTransformedSpace : public Function
{
	
public:
	FunctionInTransformedSpace (Function* other):mFunction(other),m_pSourceFunction(NULL),m_pTargetFunction(NULL){}
	virtual ~FunctionInTransformedSpace () {}

	/**
	* 이걸 call하면, 주어진 min, max가 0, 1이 되도록 normalize한다.
	* 즉, normalize하지 않으려면 min에 0, max에 1을 준다. 
	* 이함수를 call하지 않으면, sourcesample의 min, max를 자동으로 추출하여 사용한다.
	*/
	void setFunction( BijectiveFunction* sourceFunction, BijectiveFunction* targetFunction)
	{
		m_pSourceFunction=sourceFunction;
		m_pTargetFunction=targetFunction;
	}
	
	virtual void learn(const matrixn & sources, const matrixn& targets)
	{
		Function::learn(sources, targets);
		matrixn tsources(sources.rows(), m_pSourceFunction->dimRange());
		matrixn ttargets(targets.rows(), m_pTargetFunction->dimRange());
	
		for(int i=0; i<sources.rows(); i++)
		{
			m_pSourceFunction->mapping(sources.row(i), tsources.row(i).lval());
			m_pTargetFunction->mapping(targets.row(i), ttargets.row(i).lval());
		}
		mFunction->learn(tsources, ttargets);
	}

	virtual void mapping(const vectorn& source, vectorn& target) const
	{
		m_pSourceFunction->mapping(source, tsource);
		mFunction->mapping(tsource, ttarget);
		m_pTargetFunction->inverseMapping(ttarget, target);
	}

	virtual void prepare(const vectorn& source) const							
	{
		m_pSourceFunction->mapping(source, tsource);
		mFunction->prepare(tsource);
	}

	virtual m_real logProbPrepared(const vectorn& target) const					
	{
		m_pTargetFunction->mapping(target, ttarget);
		return mFunction->logProbPrepared(ttarget);
	}
	
protected:
	Function* m_pSourceFunction;
	BijectiveFunction* m_pTargetFunction;
	Function* mFunction;

	// temporary vectors
	mutable vectorn tsource;
	mutable vectorn ttarget;

};
#endif
