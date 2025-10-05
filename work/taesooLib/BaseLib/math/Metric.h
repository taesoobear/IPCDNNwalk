#pragma once

#include "vectorn.h"

class Metric
{
public:
	Metric(void);
	virtual ~Metric(void);

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b)=0;
	virtual Metric* Clone() const =0;
};

class L2Metric : public Metric
{
public:
	L2Metric(){}
	virtual ~L2Metric(){}

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};

class WeightedL2Metric : public Metric
{
public:
	WeightedL2Metric(){}
	virtual ~WeightedL2Metric(){}

	vectorn m_vWeight;
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};

class QuaterMetric : public Metric
{
public: 
	QuaterMetric (){}
	virtual ~QuaterMetric(){}

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const	{ return new QuaterMetric();}	
};

class KovarMetric : public Metric
{
public:
	KovarMetric(bool allowTranslationAlongAxis=false);
	virtual ~KovarMetric(){}

	// inputs
	vectorn m_weights;//!< default: (1,1,1,....,1)
	vector3 m_axis;	//!< default: (0,1,0)

	bool m_allowTranslationAlongAxis;
	// outputs
	matrix4 m_transfB;
	matrixn m_srcA, m_srcB, m_transformedB;	//!< Three n*3 matrixes.
	
	/**
	 * point cloud B를 m_axis축으로 m_transfB로 변환하면 weighted 최소 거리 제곱합이 되도록 matching된다. 그때의 L2 거리를 return
	 * 
	 * \param a a는 n*3 matrix로부터 변환된 것으로 가정한다. 즉 (x1,y1,z1,x2,y2,z2,...)
	 * \param b b는 n*3 matrix로부터 변환된 것으로 가정한다. 즉 (x1,y1,z1,x2,y2,z2,...)
	 * \return a와 b의 KovarDistance
	 */
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};

class NoRotMetric : public KovarMetric
{
public:
	NoRotMetric(bool allowTranslationAlongAxis=false):KovarMetric(allowTranslationAlongAxis) {}
	virtual ~NoRotMetric(){}

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};

class PointCloudMetric: public Metric
{
public:
	PointCloudMetric(){}
	virtual ~PointCloudMetric(){}

	// outputs
	matrix4 m_transfB;
	matrixn m_transformedB;	
	
	/**
	 * point cloud B를 y축으로 m_transfB로 변환하면 weighted 최소 거리 제곱합이 되도록 matching된다. 그때의 L2 거리를 return
	 * 
	 * \param a a는 n*3 matrix로부터 변환된 것으로 가정한다. 즉 (x1,y1,z1,x2,y2,z2,...)
	 * \param b b는 n*3 matrix로부터 변환된 것으로 가정한다. 즉 (x1,y1,z1,x2,y2,z2,...)
	 * \return a와 b의 KovarDistance
	 */
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};

class WeightedPointCloudMetric: public Metric
{
	vectorn weights;
public:
	WeightedPointCloudMetric(vectorn const& w):weights(w){weights/=weights.sum();}
	virtual ~WeightedPointCloudMetric(){}


	// outputs
	matrix4 m_transfB;
	matrixn m_transformedB;	
	bool errorOccurred;
	/**
	 * point cloud B를 y축으로 m_transfB로 변환하면 weighted 최소 거리 제곱합이 되도록 matching된다. 그때의 L2 거리를 return
	 * 
	 * \param a a는 n*3 matrix로부터 변환된 것으로 가정한다. 즉 (x1,y1,z1,x2,y2,z2,...)
	 * \param b b는 n*3 matrix로부터 변환된 것으로 가정한다. 즉 (x1,y1,z1,x2,y2,z2,...)
	 * \return a와 b의 KovarDistance
	 */
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};


class CDynamicTimeWarping;
class DTWMetric : public Metric
{
public:
	DTWMetric(int numColumn);
	virtual ~DTWMetric();

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
private:
	matrixn m_srcA, m_srcB;
	CDynamicTimeWarping* m_pDTW;
	int m_nColumn;
};

