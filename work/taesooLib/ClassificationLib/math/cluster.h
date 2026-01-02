#ifndef _CLUSTER_H_
#define  _CLUSTER_H_
#pragma once

#include "../../BaseLib/utility/TArray.h"

class matrixn;
class intvectorn;
class Metric;
/// wrapper class of CFuzzyCluster
/** 나중에는 필요한 기능만 남기고 CFuzzyCluster와 독립시킬 계획*/
class Cluster
{
public:
	Cluster(void);
	~Cluster(void);

	void ChangeDistanceMetric(const Metric& Metric);

	// cluster methods
	void DTWKmeanCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_thr, int& cluster_n, intvectorn& group_index, matrixn& aCenter, matrixn& refPattern);
	void ShinCluster(const matrixn &aInputVec, m_real inner_thr, int &GroupNum, intvectorn& aGroupIndex, matrixn& aCenter);
	void KMeanCluster(const matrixn& aInputvec, int cluster_n, intvectorn& groupIndex, matrixn& centers, const matrixn& input_centers);
private:

	Metric* m_pMetric;
	void CalcDTWDistanceMatrix(const matrixn& aInputVec, int numRow, int numColumn, matrixn& distMat);
	//pGroupIndex를 aVectorGroup형태로 바꾼다. 즉 몇번째 그룹에 뭐가 있는지 형태.
	static void MakeGroupIndex(CTArray<intvectorn>& aVectorGroup, int* pGroupIndex);
	static void CalcVectorGroup(int numInput, int numCluster, int* pGroupIndex, CTArray<intvectorn>& aVectorGroup);
	m_real CalcInnerClusterScore(const matrixn& aInputVec, const matrixn& aCenter, const CTArray<intvectorn>& aVectorGroup, int& maxGroupIndex, int& maxEltIndex, m_real& maxLength);
	m_real CalcInterClusterScore(const matrixn& aInputVec, const matrixn& aCenter, int numCluster, int& minGroup1, int& minGroup2, m_real& minLength);
	static bool CheckNonEmpty(const matrixn& aInputVec, int &numCluster, matrixn& aCenter, int *pGroupIndex);
};


/// Cluastering class의 interface를 통일시키는 것이 목적.
struct Clustering
{
	Clustering() {}
	virtual ~Clustering() {}
	/// aInputVec : array of input vectors. Each vector can have different length from others.
	virtual void cluster(const TArray<vectorn>& aInputVec) {}

	// query outputs
	int numGrp()				{ return m_nNumGroup;}
	int groupIndex(int elt)		{ return m_aGroupIndex[elt];}
	intvectorn& groupIndex()	{ return m_aGroupIndex;}

	// test routines
	void test(const char* fn);
	virtual void plot(matrixn const& samples,const char* fn);
	static void test();
protected:
	//	all outputs are save into members
	int m_nNumGroup;
	intvectorn m_aGroupIndex;
};

struct KMeanCluster : public Clustering
{
	KMeanCluster(int numCluster){m_nNumGroup=numCluster;}
	~KMeanCluster(){}

	matrixn centers;
	virtual void cluster(const TArray<vectorn>& aInputVec);
	const matrixn& getCenters(){ return centers;}

	static intvectorn findGroupIndex(matrixn const& source, matrixn const& centers);

};

namespace Clust
{
	class ClassSig;
};
struct GMMCluster: public Clustering
{
	enum Option { FULL, DIAG, SPHR};

	// Determine the optimal number of clusters
	GMMCluster(int init_num_of_subclasses, Option option1=FULL);
	// Number of clusters are predetermined.
	GMMCluster(Option option1, int numCluster);
	// Number of clusters are predetermined.
	GMMCluster(Option option1, int numCluster, int unused);

	virtual void cluster(const TArray<vectorn>& aInputVec);
	virtual void plot(matrixn const& samples,const char* fn);

	Option mOption;
	int mDesiredNumCluster;
	Clust::ClassSig* mSig;
};

struct ExactCluster : public Clustering
{
	ExactCluster() {}
	~ExactCluster()	{}

	virtual void cluster(const TArray<vectorn>& aInputVec);
};

struct FuzzyCluster : public Clustering
{
	/// use either thr or numCluster to adjust clustering
	FuzzyCluster(m_real thr=FLT_MAX, int numCluster=1):m_fRadii(thr), m_numCluster(numCluster) {}
	~FuzzyCluster() {}
	virtual void cluster(const TArray<vectorn>& aInputVec);
	int findNearestGroup(const vectorn& inputVec);
	m_real m_fRadii;
	matrixn m_aCenter;
	int m_numCluster;
};

struct AggloCluster : public Clustering
{
	enum { MIN_LINKAGE , AVERAGE_LINKAGE , MAX_LINKAGE };
	/// use either thr or numCluster to adjust clustering
	AggloCluster(const Metric& metric, int eLinkage, m_real thr /*=FLT_MAX*/, int numCluster /*=1*/);
	AggloCluster(const Metric& metric, int eLinkage, int numCluster);
	~AggloCluster();
	virtual void cluster(const TArray<vectorn>& aInputVec);
	void cluster_using_distances(const matrixn & distmat);
	int m_eLinkage;
	m_real m_fThr;
	Metric* m_pMetric;
	matrixn m_distMat;
	matrixn m_interDist;
	vectorn m_innerDist;

	struct Factory : public TFactory<Clustering>
	{
		Factory(Metric* pMetric, int eLinkage, m_real fThr=FLT_MAX, int numCluster=1):m_pMetric(pMetric), m_eLinkage(eLinkage), m_fThr(fThr), m_numCluster(numCluster) {}
		virtual~Factory(){ delete m_pMetric; }
		virtual Clustering* create(int index)	{ return new AggloCluster(*m_pMetric, m_eLinkage, m_fThr, m_numCluster);}
		Metric* m_pMetric;
		int m_eLinkage;
		m_real m_fThr;
		int m_numCluster;
	};
};

struct TwoPhaseClustering : public Clustering
{
	/// pC1, pC2F는 새로 생성해서 넣어줄것. TwoPhaseClustering이 끝날때 자동으로 삭제된다.
	TwoPhaseClustering(Clustering* pC1, TFactory<Clustering>* pC2F, int numC1Elt);
	~TwoPhaseClustering();

	/// aInputVec은 원소개수의 두배에 해당하는 array. 앞의 반은 C1에 쓰일 feature를 저장하고, 뒤의 반은 C2에서 쓰일 feature를 저장한다.
	virtual void cluster(const TArray<vectorn>& aInputVec);

	int m_numC1Elt;
	Clustering* m_pC1;
	TArray<Clustering> m_aC2;
	TArray<intvectorn> m_aTargetIndex;	//!< m_aC2의 i번째 cluster의 j번째 원소의 aInputVec인덱스는 m_aTargetIndex[i][j]
	intvectorn m_aGlobalGroupStartIndex;
};

#endif
