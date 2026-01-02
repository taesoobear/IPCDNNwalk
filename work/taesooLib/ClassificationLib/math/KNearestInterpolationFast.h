#pragma once

#include "../../BaseLib/math/mathclass.h"
#include "../../BaseLib/math/intervalN.h"
#include "../../BaseLib/math/Metric.h"
#include "../../ClassificationLib/math/Interpolation.h"
#include "../../ClassificationLib/math/Function.h"
#include "ann_1.1.2/include/ANN/ANN.h"

// When L2 metric is used, use these classes because this is much faster.
namespace statistics
{
	class KNearestInterpolationFast: public statistics::Interpolation
	{
		public:
		KNearestInterpolationFast (int k=4, float power=2.f, float noiseWeight=0.f);
		virtual ~KNearestInterpolationFast (){ delete mKDtree;}
		virtual void initialize(const matrixn& sourceSamples);
		virtual void calcWeight(const vectorn& parameter, intvectorn& index, vectorn& weight);
	
		/// weight= 1/distance^m_fK - 1/maxDistanceAmongK^m_fK
		int m_nK;
		float m_fK;
		float m_fNW;

		std::vector<ANNpoint> pointArray;
		ANNkd_tree *mKDtree;
	};
	class WeightedKNearestInterpolationFast: public KNearestInterpolationFast
	{
		public:
		WeightedKNearestInterpolationFast (const vectorn& weights, int k=4, float power=2.f, float noiseWeight=0.f);
		virtual ~WeightedKNearestInterpolationFast (){ }
		virtual void initialize(const matrixn& sourceSamples);
		virtual void calcWeight(const vectorn& parameter, intvectorn& index, vectorn& weight);
	
		/// weight= 1/distance^m_fK - 1/maxDistanceAmongK^m_fK
		int m_nK;
		float m_fK;
		float m_fNW;
		vectorn m_weights;

		std::vector<ANNpoint> pointArray;
		ANNkd_tree *mKDtree;
	};
}
class KNearestInterpolationFast : public Function
{
public:
	KNearestInterpolationFast (int k=4, float power=2.f, float noiseWeight=0.f);
	virtual ~KNearestInterpolationFast (){ delete mKDtree;}

	// Function::
	virtual void learn(const matrixn& sources, const matrixn& targets);
	virtual void mapping(const vectorn& source, vectorn& target) const;
	void mapping(const vectorn& source, vectorn& target, intvectorn & index, vectorn& weights) const;
	//
	/// weight= 1/distance^m_fK - 1/maxDistanceAmongK^m_fK
	matrixn mSource;
	matrixn mTarget;
	intvectorn index;
	vectorn weight;
	int m_nK;
	float m_fK;
	float m_fNW;

	std::vector<ANNpoint> pointArray;
	ANNkd_tree *mKDtree;
};

