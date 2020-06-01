#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "statistics.h"
#include "GnuPlot.h"
#include "../BaseLib/image/DrawChart.h"
#include "../BaseLib/utility/operatorString.h"

using namespace statistics;

NormalDistribution::NormalDistribution(const vectorn& samples)
{
	m_fMean=samples.avg();
	vectorn zeroMeaned;
	zeroMeaned.sub(samples, m_fMean);
	m_fVar=zeroMeaned.aggregate(CAggregate::SQUARESUM)/samples.size();
	m_fStdDev=sqrt(m_fVar);
}

void NormalDistribution::learn(const vectorn& samples)
{
	m_fMean=samples.avg();
	vectorn zeroMeaned;
	zeroMeaned.sub(samples, m_fMean);
	m_fVar=zeroMeaned.aggregate(CAggregate::SQUARESUM)/samples.size();
	if(m_fVar==0)
		m_fVar=0.0001;
	m_fStdDev=sqrt(m_fVar);
}

NormalDistribution::NormalDistribution(m_real mean, m_real stddev)
{
	m_fMean=mean;
	m_fStdDev=stddev;
	m_fVar=SQR(stddev);
}

NormalDistribution::~NormalDistribution()
{
}

m_real NormalDistribution::prob(m_real x) const
{
	m_real probb=exp(-1.f*SQR(x-m_fMean)/ (2.f*m_fVar) ) / sqrt(2.f*M_PI*m_fVar);

	if(!(probb==probb))
	{
		printf("NaN: x %f, mean %f, var %f\n",x, m_fMean, m_fVar);
		assert(0);
	}
	return probb;
}

m_real NormalDistribution::logProb(m_real x) const
{
	m_real probb=(-1.f*SQR(x-m_fMean)/ (2.f*m_fVar) ) -log( sqrt(2.f*M_PI*m_fVar));

	if(!(probb==probb))
	{
		printf("NaN: x %f, mean %f, var %f\n",x, m_fMean, m_fVar);
		assert(0);
	}
	return probb;
}

m_real  NormalDistribution::Z(m_real x) const
{
	return (x-m_fMean)/m_fStdDev;
}

m_real  NormalDistribution::X(m_real Z) const
{
	//Z=(x-m_fMean)/m_fStdDev;
	return Z*m_fStdDev+m_fMean;
}

m_real statistics::randomSamplingUniform()
{
	int n=rand();
	return (m_real)n/((m_real)RAND_MAX+1.0);
}

m_real statistics::randomSamplingNormal(m_real sigma)
{
	m_real x, y, r2;

	do
	{
		/* choose x,y in uniform square (-1,-1) to (+1,+1) */

		x = -1 + 2 * statistics::randomSamplingUniform ();
		y = -1 + 2 * statistics::randomSamplingUniform ();

		/* see if it is in the unit circle */
		r2 = x * x + y * y;
	}
	while (r2 > 1.0 || r2 == 0);

	/* Box-Muller transform */
	return sigma * y * sqrt (-2.0 * log (r2) / r2);
}
int statistics::randomSampling(const vectorn& prob)
{
	vectorn costs=prob;

	for(int i=0; i<costs.size(); i++)
	{
		// if NaN
		if(!(costs[i]==costs[i]))
			costs[i]=0;
	}
	costs/=costs.sum();
	// make summed area table.
	for(int i=1; i<costs.size(); i++)
		costs[i]+=costs[i-1];

	m_real random=(m_real)rand()/(m_real)RAND_MAX;
	
	int i;
	for(i=0; i<costs.size()-1; i++)
		if(costs[i]>random)
			break;

	//printf("random %f [%d:%d] %s:%s\n", random, i, costs.size(), prob.output().ptr(), costs.output().ptr());

	return i;
}


void DensityEstimation::test(int n, const char* id)
{
	matrixn test;
	test.pushBack(vectorn(2,1.5,0.0));
	test.pushBack(vectorn(2,0.7,1.1));
	test.pushBack(vectorn(2,1.0,2.1));
	test.pushBack(vectorn(2,1.7,1.6));
	test.pushBack(vectorn(2,1.2,0.6));
	test.pushBack(vectorn(2,0.3,0.2));
	test.pushBack(vectorn(2,0.2,0.4));
	test.pushBack(vectorn(2,0.7,1.4));
	test.pushBack(vectorn(2,1.5,2.3));
	test.pushBack(vectorn(2,1.9,1.1));

	test.resize(n, 2);

	learn(test);
	plot(test, id);
}

void DensityEstimation::plot(const matrixn& sources, const char* identifier) 
{
#ifdef USE_GNU_PLOT
	{
		TString tid(identifier);

		matrixn const &x=sources;
		matrixn t(sources.rows(), 1);

		learn(x);
		for(int i=0; i<sources.rows(); i++)
			t[i][0]=pdf(sources.row(i));

		matrixn xtest;
		xtest.setSize(50*50,2);

		vectorn min, max, range;
		min.minimum(x);
		max.maximum(x);
		range.sub(max, min);

		vectorn d1;
		vectorn d2;
		d1.linspace(min[0], max[0], 50);
		d2.linspace(min[1], max[1], 50);

		int k=0;
		for (int i=0; i<50; i++)
			for(int j=0; j<50; j++)
			{
				xtest[k][0]=d1[i];
				xtest[k][1]=d2[j];
				k=k+1;
			}

			vectorn ytest(xtest.rows());
			for(int i=0; i<xtest.rows(); i++)
			{
				ytest[i]=pdf(xtest.row(i));
			}

			matrixn xx;
			gnuPlot::setYUP(false);
			xx.concatColumn(x, t);
			gnuPlot::plotScattered(tid+"source", xx, tid+"source", "x1", "x2", "t");

			matrixn xt;
			xt.concatColumn(xtest, ytest.cols());
			gnuPlot::plotScattered(tid+"test", xt, tid+"test", "xtest1", "xtest2", "ytest");

			TStrings filenames;
			filenames.pushBack(tid+"source");
			filenames.pushBack(tid+"test");

			gnuPlot::mergedPlotScattered(filenames, 3, tid+"all", "x1","x2","y");
	}
#else
	{
		TString tid(identifier);

		matrixn const &x=sources;
		matrixn t(sources.rows(), 1);

		learn(x);
		for(int i=0; i<sources.rows(); i++)
			t[i][0]=pdf(sources.row(i));


		vectorn min, max, range;
		min.minimum(x);
		max.maximum(x);
		range.sub(max, min);
		if(max==min)
			range.setAllValue(1.0);
		range*=0.5;

		vectorn d1;
		vectorn d2;
		d1.linspace(min[0]-range[0], max[0]+range[0], 50);
		d2.linspace(min[1]-range[1], max[1]+range[1], 50);

		vectorn xtest(2);
		matrixn ytest(50, 50);

		for (int i=0; i<50; i++)
			for(int j=0; j<50; j++)
			{
				xtest[0]=d1[i];
				xtest[1]=d2[j];
				ytest[i][j]=pdf(xtest);
			}

			DrawChart chart("x", "y", d1[0], d1[d1.size()-1], d2[0], d2[d2.size()-1]);
			chart.drawMatrix(ytest);
			chart.drawScatteredData(sources, CPixelRGB8(255,255,0), "../resource/default/pattern1.bmp");
			chart.save(sz1::format("../gnuplot/%s.bmp", identifier));
	}
#endif
	
}

void MVNormalDistribution ::learn(const matrixn& sources)
{
	mNormalDistributions.resize(sources.cols());

	vectorn col(sources.rows());
	for(int i=0; i<sources.cols(); i++)
	{
		sources.getColumn(i, col);
		mNormalDistributions[i].learn(col);
	}

}

int MVNormalDistribution ::dim() const
{
	return mNormalDistributions.size();
}

m_real MVNormalDistribution ::logPdf(const vectorn& x) const
{
	if(!dim() ) 
	{
		ASSERT(0);
		return -FLT_MAX;
	}

	m_real prob=0;
	for(int i=0; i<dim(); i++)
		prob+=mNormalDistributions[i].logProb(x[i]);
	return prob;
}

m_real MVNormalDistribution ::pdf(const vectorn& x) const
{
	if(!dim() ) 
	{
		ASSERT(0);
		return 0;
	}
	m_real prob=1;
	for(int i=0; i<dim(); i++)
		prob*=mNormalDistributions[i].prob(x[i]);
	return prob;
}

