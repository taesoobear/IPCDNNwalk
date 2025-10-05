#pragma once
#include <vector>
namespace statistics
{

	// calculating likelihood
	
	class NormalDistribution
	{
	public:
		NormalDistribution(const vectorn& samples);
		NormalDistribution(m_real mean=0, m_real stddev=1.0);
		virtual ~NormalDistribution();

		void learn(const vectorn& samples);
		/// P(D|M)
		m_real  prob(m_real x) const;
		m_real  logProb(m_real x) const;
		m_real  sampleMean() const { return m_fMean;}
		m_real  sampleStdDev() const { return m_fStdDev;}
		m_real  Z(m_real x) const;
		m_real  X(m_real Z) const;
		
	private:
		m_real m_fMean;
		m_real m_fVar;
		m_real m_fStdDev;
	};

	class DensityEstimation
	{
	public:
		// works only for 2D scattered data.
		virtual void plot(const matrixn& sources, const char* identifier) ;

		virtual void learn(const matrixn& sources)=0;

		virtual int dim() const=0;

		virtual m_real logPdf(const vectorn& x) const=0;
		virtual m_real pdf(const vectorn& x) const=0;

		virtual void test(int n=10, const char* id="test");

	};

	// 각 차원이 independent하게 normal distribution으로 결정.
	class MVNormalDistribution : public DensityEstimation
	{
	public:
		virtual void learn(const matrixn& sources);

		virtual int dim() const;

		virtual m_real logPdf(const vectorn& x) const;
		virtual m_real pdf(const vectorn& x) const;

		std::vector<NormalDistribution> mNormalDistributions;
	};

	#include "PCA.H"
	class DistributionUtil
	{
	public:
		DistributionUtil(){}
		virtual ~DistributionUtil(){}

		void addSample(const vectorn& source)
		{
			mSource.pushBack(source);
		}

		int numSample()	
		{
			return mSource.rows();
		}

		void learn(DensityEstimation& dist)	{ dist.learn(mSource);}
		void learnInSubspace(DensityEstimation& dist, PCA& pca)
		{
			pca.Projection(mSource, mReducedSource);
			dist.learn(mReducedSource);
		}
		void plot(DensityEstimation& dist, const char* id) {dist.plot(mSource, id);}
	private:
		matrixn mSource;
		matrixn mReducedSource;
	};

	// return random value in [0,1]
	m_real randomSamplingUniform();
	// normal distribution
	m_real randomSamplingNormal(m_real sigma);
	int randomSampling(const vectorn& prob);
}