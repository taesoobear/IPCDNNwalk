#pragma once

#include <deque>
#include <vector>
#include "../BaseLib/utility/TArray.h"
typedef std::vector<int> intVector;

#include "../BaseLib/math/mathclass.h"

class CFuzzyCluster
{
public:
	CFuzzyCluster(void);
	~CFuzzyCluster(void);

	static void Subtractive(matrixn& x, int n, std::deque<vectorn>& centers);
	static void Subtractive(matrixn& x, int n, m_real radii, std::deque<vectorn>& centers);
	//맨 마지막 것이 out이고 radii가 vectorn type의 x 의 각 component에 대한 influence range (간단히 말해 threshold라고도 하지 --; )
	// n은 당욘 data 개수
	static void Subtractive(matrixn& x, int n, m_real* radii, std::deque<vectorn>& centers);
	static void Subtractive2(vectorn* x, int n, m_real radii, std::deque<vectorn>& centers);
	static void Subtractive2(vectorn* x, int n, m_real* radii, std::deque<vectorn>& centers);
private:
	static m_real min_value(vectorn& x);
	static m_real max_value(vectorn& x);
	static m_real min_value(m_real* x, int n, int& index);
	static m_real max_value(m_real* x, int n, int& index);
	static void min_value(vectorn* x, int n, vectorn& minX);
	static void max_value(vectorn* x, int n, vectorn& maxX);
	static m_real Distance(vectorn &v1, vectorn &v2, vectorn &factor);
	
public:
	static void Fcm(matrixn& x, int n, int cluster_n, std::deque<vectorn>& centers, int* group_index);
	static void Fcm(matrixn& x, int n, std::deque<vectorn>& centers, int* group_index);
	static void FcmRadii(matrixn& x, int n, m_real radii, std::deque<vectorn>& centers, int* group_index);
	static void InitFcm(matrixn& x, vectorn* U, int cluster_n, int expo, std::deque<vectorn>& centers);
	static m_real StepFcm(matrixn& x, vectorn* U, int cluster_n, int expo, std::deque<vectorn>& centers);
	
	// Followings are taesoo favorite interface version.

	static void Subtractive(const matrixn& inputvectors, m_real radii, std::deque<vectorn>& centers);
	static void FcmRadii(const matrixn& inputvectors,  m_real radii, matrixn& aCenter, int* group_index);
	// binary search로 원하는 거리를 thr범위 안에서 찾는다. 
	//즉 desired_ncenter+-thr범위에 센터 개수가 들어가면 멈춘다. 단 max_iteration=20이다.최종결과를 만든 radii를 return한다.
	static m_real FcmKCenter(const matrixn& aInputVec,  int numCluster, int thr, matrixn& aCenter, int* group_index, int max_iter=20);
	
	// bUseInputCenters가 true인 경우 입력된 centers가 initial center를 담고 있다고 가정
	static void KMeanCluster(const matrixn& inputvectors, int cluster_n, matrixn& centers, int *group_index, bool bUseInitialCenters=false);
	static void KMeanCluster(const matrixn& inputvectors, int cluster_n, matrixn& centers, int *group_index, const matrixn& input_centers);
	static void FcmKMeanCluster(const matrixn &inputvectors, int cluster_n, matrixn& centers, int *pGroupIndex);
	// kmean clustering후 empty cluster가 생기는 경우 아래 함수로 없애준다.
	static bool CheckNonEmpty(const matrixn& aInputVec, int &numCluster, matrixn& aCenter, int *pGroupIndex);

	static void GenerateOptimumCluster(const matrixn &aInputVec, int &GroupNum, int *pGroupIndex, m_real thr, matrixn& aCenter);

	//pGroupIndex를 aVectorGroup형태로 바꾼다. 즉 몇번째 그룹에 뭐가 있는지 형태.
	static void CalcVectorGroup(int numInput, int numCluster, int* pGroupIndex, CTArray<intVector>& aVectorGroup);
	static m_real CalcInnerClusterScore(const matrixn& aInputVec, const matrixn& aCenter, const CTArray<intVector>& aVectorGroup, int& maxGroupIndex, int& maxEltIndex, m_real& maxLength);
	static m_real CalcInterClusterScore(const matrixn& aInputVec, const matrixn& aCenter, int numCluster, int& minGroup1, int& minGroup2, m_real& minLength);

	static void CalcDTWDistanceMatrix(const matrixn& aInputVec, int numRow, int numColumn, matrixn& distMat);

	// 클러스터 개수가 cluster_n으로 될때까지 수행
	static void AggloCluster(const matrixn& distMat, int cluster_n, int* group_index);
	// inner cluster distance가 inner_cluster_thr을 넘지 않는 최소 클러스터.
	static void AggloCluster(const matrixn& distMat, m_real inner_cluster_thr, int& cluster_n, int* group_index);
	static void AggloClusterCore(const matrixn& distMat, matrixn& interDist, vectorn& innerDist, m_real inner_cluster_thr, int& cluster_n, int* group_index, int eLinkage=0);
	
	static void DTWAggloCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_cluster_thr, int& cluster_n, int* group_index, m_real *pfMaxInner=NULL, m_real* pfMinInter=NULL);
	static void DTWKmeanCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_cluster_thr, int& cluster_n, int* group_index, matrixn& aCenter, matrixn& refPattern, m_real *pfMaxInner=NULL, m_real* pfMinInter=NULL);
	
	// 완전히 동일한 벡터만 같은 클러스터에 들어갈 수 있다.
	static void LexicoCluster(const matrixn& distMat, int& cluster_n, int* group_index);

	// needed for Classify
	class SubClustering
	{
	public:
		void Init();
		void AddElement(int targetIndex, int GroupIndex, const matrixn& temporalFeature);
		void DelElement(int targetIndex);
		void RemoveEmpty();
		void Cluster(int nNumSample, int nSizePostureFeature, m_real maxInner);
		int CountSameGroupElements(int targetIndex);
		int FindCluster(matrixn& postureFeatures, m_real maxInner);
		matrixn aTemporal;
		matrixn aCenter;		  // clustering 방법에 따라 사용되지 않기도 한다.
		matrixn referencePattern; // clustering 방법에 따라 사용되지 않기도 한다.
		intvectorn aGroupIndex;
		intvectorn aTargetIndex;
		int numGroup;		
		int globalGroupStartIndex;
	};

	// 시간축 feature와 structural 정보를 concat한 벡터가 입력이다. structure 정보로 lexico cluster한 후, subcluster에서 dtw후 agglo clustering한다.
	static void Classify(const matrixn& aInputVec, int nNumSample, int nSizePostureFeature, m_real INNER_THR, int& numGroup, int* _aGroupIndex, CTArray<SubClustering>& subClusters, m_real *pfMaxInner=NULL, m_real* pfMinInter=NULL, bool bUseAgglo=false);	
};
