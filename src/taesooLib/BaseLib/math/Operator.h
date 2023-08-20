#ifndef _OPERATOR_H_
#define  _OPERATOR_H_
#pragma once

#include "math_macro.h"
#include "../math/tvector.h"
class Metric;
class boolN;
namespace v
{
	int argMinRand(vectorn const& a, m_real thr=1.05, int start=0, int end=INT_MAX);
	void interpolate(vectorn & out, m_real t, vectorn const& a, vectorn const& b);
	void interpolateQuater(vectorn & out, m_real t, vectorn const& a, vectorn const& b);
	void transition(vectorn & out, m_real start, m_real end, int nSize);
	m_real sample(vectorn const& in, m_real criticalTime);
	void hermite(vectorn& out, double a, double b, int duration, double c, double d);
	// preserves out.size()
	void quintic(vectorn& out, double x0, double v0, double a0, double x1, double v1, double a1, double T);


}

namespace m
{
	// distances between each row of a.
	void distanceMat(matrixn & out, matrixn const& a, Metric* pMetric=NULL);
	void distanceMat(matrixn & out, matrixn const& a, matrixn const& b, Metric* pMetric=NULL);

	matrixn diag(vectorn const& a);

	// returns v^T * M * v
	m_real vMv(vectorn const& v, matrixn const& M);
	// returns v^T * Diag(M) * v
	m_real vDv(vectorn const& v, vectorn const& diagM);

	// returns (a-b)^T * M *(a-b)
	m_real sMs(vectorn const& a, vectorn const& b, matrixn const& M);

	// returns (a-b)^T * Diag(M) *(a-b)
	m_real sDs(vectorn const& a, vectorn const& b, vectorn const& diagM);

	// returns (a-b)^T * (a-b)  : squared distance
	m_real ss(vectorn const& a, vectorn const& b);

	void multAB(matrixn& out, matrixn const& a, matrixn const& b, bool transposeA=false, bool transposeB=false);
	void multABC(matrixn& out, matrixn const& a, matrixn const& b, matrixn const& c, bool transposeA=false, bool transposeB=false, bool transposeC=false);

	void multA_diagB(matrixn& c, matrixn const& a, vectorn const& b);
	void multAtB(vectorn& out, matrixn const& A, vectorn const& b);
	void multAtBA(matrixn& c, matrixn const& a, matrixn const& b);
	void multABAt(matrixn& c, matrixn const& a, matrixn const& b);

	index2 argMin(matrixn const& a);
	index2 argMinRand(matrixn const& a, m_real thr=1.05);
}

namespace sop
{
	int interpolateInt(m_real t, int s, int e);
	m_real interpolate(m_real t, m_real s, m_real e);

	// smoothTransition(0)=0, smoothTransition(1)=1, dy/dx= 0 at x=0 and 1
	m_real smoothTransition(m_real a);
	// if(t<min) return v1 elseif (t>max) return v2, otherwise, inbetween v1 and v2 linearly.
	m_real clampMap(m_real t, m_real min, m_real max, m_real v1=0.0, m_real v2=1.0);
	m_real map(m_real t, m_real min, m_real max, m_real v1=0.0, m_real v2=1.0);
	m_real sigmoid(m_real t);
}

namespace v
{
	/// uniform sampling : sample centers of intervals in linspace of size n+1; eg> uniform (0,3, size=3) -> (  0.5, 1.5, 2.5 ).
	void uniformSampling (vectorn& out, m_real xx1, m_real xx2, int nnSize=-1);
	void interpolate(m_real t, vectorn& out, vectorn const& a, vectorn const& b);
	void interpolateQuater(m_real t,vectorn& out, vectorn const& a, vectorn const& b);
	void hermite(vectorn& out, double t, double T, const vectorn& a, const vectorn va, const vectorn& b,  const vectorn& vb);
}
namespace m
{
    void derivative(matrixn& velocity, matrixn const& positions);	
	void derivative(matrixn& velocity, matrixn const& positions, boolN const& discontinuity);	// ex) velocities.delta(positions);
    void derivativeQuater(matrixn& velocity, matrixn const& positions);	
	void filter(matrixn& inout, int kernelSize);
	void adaptiveFilter(matrixn& inout, vectorn const& kernelSize, float frameTime);
	void superSampling(int nSuperSample, matrixn& out, matrixn const& in );
	void downSampling(int nDownSample, matrixn& out, matrixn const& in );
	void alignQuater(matrixn& inout);
	void splineFit(int nDegree, matrixn& c, const matrixn& a);
	// filename: aaa.bmp
	void drawSignals(const char* filename, matrixn const& in, double min=0, double max=0, intvectorn vXplot=intvectorn());
	void covariance(matrixn& c, const matrixn& a) ;
	/**
	 * ab와 cd를 연결하는 hermite curve를 만든다. 중간은 duration으로 주어진다.
	 *   ab
	 * ----
	 *    -- duration --
	 *                 +++++++
	 *                 cd
	 *    ************** hermite range (size=duration)
	 */
	void hermite(matrixn& out, const vectorn& a, const vectorn& b, int duration, const vectorn& c, const vectorn& d);

	/**
	 * ab를 연결하는 hermite curve를 만든다. 
	 * 0<=t<=T
	 */
}
namespace sv2
{
	struct dotProduct : public _op
	{
		dotProduct(){}
		virtual m_real calc(const vectorn& a, const vectorn& b) const { return a%b; }
	};
	struct distance : public _op
	{
		distance(Metric* pMetric=NULL):m_pMetric(pMetric){}
		virtual m_real calc(const vectorn& a, const vectorn& b) const { return a.distance(b, m_pMetric); }
		Metric* m_pMetric;
	};
}
#endif
