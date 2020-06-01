#ifndef PCH_H_
#define PCH_H_
#pragma once

#include "../../BaseLib/math/mathclass.h"

class PCA
{
	public:
		PCA();
		PCA(double error_goal);	//!< error_goal=0.95 : 95% accuracy
		~PCA();
		/// OrgTemplateData?? noSamples * dim matrix?̴?.
		void getPCA(matrixn& OrgTemplateData, bool bOutput=true);
		void Projection(const vectorn& origVec, vectorn& reducedVec) const;
		void Projection(const matrixn& origVecs, matrixn& reducedVecs) const;
		void GetCovMatrix(double **InputData, int n, int m, double **SymData, double *mean);

		void GetReducedData(matrixn& reducedData);
		void GetReducedData(int i, vectorn& reducedVec);
		int GetNumData()	{ return m_noSamples;}
		
		void EigenData(matrixn& data);	// eigenVector*eigenValue+meanData

		void setReducedDim(int dim);
		bool forceReducedDim;
		int m_ReducedDim;
		int m_noSamples;
		vectorn m_MeanData;
		matrixn m_FeatureVector;


		int m_OriginalDim;
		double m_construction_error;
		double m_error_goal;
		bool m_bPercentThreshold;

		double * m_EigenValues;
    private:
		void tred2(double **a, double *d, double *e, int size);
		void tqli(double *d, double *e, double **z, int size);
		double pythag(const double a, const double b);
		double **m_ReducedData;

};
#endif
