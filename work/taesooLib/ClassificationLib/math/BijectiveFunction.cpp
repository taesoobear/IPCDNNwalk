#include "stdafx.h"
#include "Function.h"
#include "../BaseLib/math/Operator.h"
#include "BijectiveFunction.h"



bijectiveFunctions::PCA::PCA(m_real errorGoal)
{
	mPCA=new ::PCA(errorGoal);
}

bijectiveFunctions::PCA::~PCA()
{
	delete mPCA;
}
void bijectiveFunctions::PCA::setReducedDim(int dim)
{
	mPCA->setReducedDim(dim);
}
	
void bijectiveFunctions::PCA::learn(const matrixn& sources)
{
	mPCA->getPCA((matrixn&)sources);
}

void bijectiveFunctions::PCA::mapping(const vectorn& source, vectorn& target) const
{
	mPCA->Projection(source, target);
}

void bijectiveFunctions::PCA::inverseMapping(const vectorn& target, vectorn& source) const
{
	vectorn zeroMeaned;
	
	m::multAtB(zeroMeaned, mPCA->m_FeatureVector, target);
	source.add(zeroMeaned, mPCA->m_MeanData);
}

void bijectiveFunctions::Normalization::mapping(const vectorn& source, vectorn& target) const
{
	target.setSize(source.size());

	m_real alpha;
	for(int i=0; i<source.size(); i++)
	{
		alpha=(source[i]-mSourceMinimum[i])/(mSourceMaximum[i]-mSourceMinimum[i]);
		target[i]=mMinRange*(1.0-alpha)+mMaxRange*alpha;		
	}
}

void bijectiveFunctions::Normalization::inverseMapping(const vectorn& target, vectorn& source) const
{
	source.setSize(target.size());

	m_real alpha;
	for(int i=0; i<source.size(); i++)
	{
		//target[i]=mMinRange*(1.0-alpha)+mMaxRange*alpha;		
		// -> alpha(mMaxRange-mMinRange)+mMinRange=target;
		alpha=(target[i]-mMinRange)/(mMaxRange-mMinRange);

		//alpha=(source[i]-mSourceMinimum[i])/(mSourceMaximum[i]-mSourceMinimum[i]);
		source[i]=alpha*(mSourceMaximum[i]-mSourceMinimum[i])+mSourceMinimum[i];
	}
}

void bijectiveFunctions::Standardization::learn(const matrixn& sources)	
{
	mean.setSize(sources.cols());
	stddev.setSize(sources.cols());
	for(int i=0; i<sources.cols(); i++)
	{
		statistics::NormalDistribution n;
		n.learn(sources.column(i));
		mean[i]=n.sampleMean();
		stddev[i]=n.sampleStdDev();
	}

	BijectiveFunction::learn(sources); 
	mDimTarget=mDimSource; 
}

void bijectiveFunctions::Standardization::mapping(const vectorn& source, vectorn& target) const
{
	target.setSize(source.size());

	for(int i=0; i<source.size(); i++)
	{
		target[i]=(source[i]-mean[i])/stddev[i];
	}
}

void bijectiveFunctions::Standardization::inverseMapping(const vectorn& target, vectorn& source) const
{
	source.setSize(target.size());

	for(int i=0; i<source.size(); i++)
	{
		source[i]=target[i]*stddev[i]+mean[i];
	}
}
