#include "stdafx.h"
#include "../../BaseLib/math/mathclass.h"
#include "../../BaseLib/math/Operator.h"
#include "KNearestInterpolationFast.h"

statistics::KNearestInterpolationFast ::KNearestInterpolationFast ( int k, float power, float noiseWeight)
	:statistics::Interpolation() 
{
	m_nK=k; m_fK=power;
	m_fNW=noiseWeight;
	mKDtree=NULL;
}

void statistics::KNearestInterpolationFast::initialize(const matrixn& sourceSamples)
{
	Interpolation::initialize(sourceSamples);

	pointArray.resize(m_aSamples.rows());
	for(int i=0; i<pointArray.size(); i++)
	{
		pointArray[i]=&m_aSamples(i,0);
	}

	mKDtree=new ANNkd_tree(&pointArray[0], pointArray.size(), sourceSamples.cols());
}
void statistics::KNearestInterpolationFast::calcWeight(const vectorn& source, intvectorn & index, vectorn& weight) 
{
	vectorn dist;

	int numSample=m_aSamples.rows();
	int K=MIN(numSample, m_nK+1);
	index.resize(K);
	dist.resize(K);
	weight.resize(K);
	mKDtree->annkSearch(&source[0], K, &index[0], &dist[0]); 
	//printf("%s %s\n", index.output().ptr(), dist.output().ptr());
	for(int i=0; i<K; i++)
	{
		weight[i]=1.f/MAX(pow(dist[i], (m_real)m_fK),FLT_EPSILON)-1.f/MAX(pow(dist[K-1], (m_real)m_fK) ,FLT_EPSILON);	// kovar siggraph 2004
		// so that weight[K-1]==0
	}
	weight/=weight.sum();
}


statistics::WeightedKNearestInterpolationFast ::WeightedKNearestInterpolationFast ( const vectorn& weights, int k, float power, float noiseWeight)
	:statistics::KNearestInterpolationFast(k, power, noiseWeight)
{
	m_weights=weights;
}
void statistics::WeightedKNearestInterpolationFast::initialize(const matrixn& sourceSamples)
{
	matrixn temp;
	m::multA_diagB(temp, sourceSamples, m_weights);
	statistics::KNearestInterpolationFast::initialize(temp);
}
void statistics::WeightedKNearestInterpolationFast::calcWeight(const vectorn& source, intvectorn & index, vectorn& weight) 
{
	vectorn temp;
	temp.mult(source, m_weights);
	statistics::KNearestInterpolationFast::calcWeight(temp, index, weight) ;

}
KNearestInterpolationFast ::KNearestInterpolationFast ( int k, float power, float noiseWeight)
	:Function() 
{
	m_nK=k; m_fK=power;
	m_fNW=noiseWeight;
	mKDtree=NULL;
}

void KNearestInterpolationFast::learn(const matrixn& sources, const matrixn& targets)
{
	Function::learn(sources, targets);

	mSource=sources;
	mTarget=targets;

	pointArray.resize(mSource.rows());
	for(int i=0; i<pointArray.size(); i++)
	{
		pointArray[i]=&mSource(i,0);
	}

	mKDtree=new ANNkd_tree(&pointArray[0], pointArray.size(), sources.cols());
}

void KNearestInterpolationFast::mapping(const vectorn& source, vectorn& target) const
{

	intvectorn index;
	vectorn weight;
#if 1
	vectorn dist;

	int numSample=mTarget.rows();
	int K=MIN(numSample, m_nK+1);
	index.resize(K);
	dist.resize(K);
	weight.resize(K);
	mKDtree->annkSearch(&source[0], K, &index[0], &dist[0]); 
	//printf("%s %s\n", index.output().ptr(), dist.output().ptr());
	m_real maxDist=dist[K-1]; // amongK
	for(int i=0; i<K; i++)
	{
		if(index[i]==-1)
		{
			printf("%s\n",source.output().ptr());
			Msg::error("??? KNearestInterpolationFast::mapping source %s\n", source.output().ptr());
		}
		weight[i]=1.f/MAX(pow(dist[i]+maxDist*m_fNW, (m_real)m_fK),FLT_EPSILON)-1.f/MAX(pow(maxDist+maxDist*m_fNW, (m_real)m_fK) ,FLT_EPSILON);	// kovar siggraph 2004
		// so that weight[K-1]==0
	}
	weight/=weight.sum();
#else
	// brute-force

	vectorn distances;
	distances.setSize(mSource.rows());
	for(int i=0;i<distances.size();++i){
		distances[i]=mSource.row(i).distance(source);
	}

	intvectorn sortedIndex;
	sortedIndex.sortedOrder(distances);

	int numSample=mTarget.rows();
	int K=MIN(numSample-1, m_nK);

	if(numSample==1)
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
#endif
	target.setSize(mTarget.cols());
	target.setAllValue(0);


	for(int i=0; i<index.size(); i++)
	{
		target.multAdd(mTarget.row(index[i]), weight[i]);
	}
}

void KNearestInterpolationFast::mapping(const vectorn& source, vectorn& target, intvectorn & index, vectorn& weight) const
{

	//intvectorn index;
	//vectorn weight;
#if 1
	vectorn dist;

	int numSample=mTarget.rows();
	int K=MIN(numSample, m_nK+1);
	index.resize(K);
	dist.resize(K);
	weight.resize(K);
	mKDtree->annkSearch(&source[0], K, &index[0], &dist[0]); 
	//printf("%s %s\n", index.output().ptr(), dist.output().ptr());
	for(int i=0; i<K; i++)
	{
		weight[i]=1.f/MAX(pow(dist[i], (m_real)m_fK),FLT_EPSILON)-1.f/MAX(pow(dist[K-1], (m_real)m_fK) ,FLT_EPSILON);	// kovar siggraph 2004
		// so that weight[K-1]==0
	}
	weight/=weight.sum();
#else
	// brute-force

	vectorn distances;
	distances.setSize(mSource.rows());
	for(int i=0;i<distances.size();++i){
		distances[i]=mSource.row(i).distance(source);
	}

	intvectorn sortedIndex;
	sortedIndex.sortedOrder(distances);

	int numSample=mTarget.rows();
	int K=MIN(numSample-1, m_nK);

	if(numSample==1)
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
#endif
	target.setSize(mTarget.cols());
	target.setAllValue(0);


	for(int i=0; i<index.size(); i++)
	{
		target.multAdd(mTarget.row(index[i]), weight[i]);
	}
}

