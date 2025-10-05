#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "./cluster.h"
#include "../BaseLib/image/imageclass.h"
#include "../BaseLib/image/DrawChart.h"
#include "FuzzyCluster.h"
#include <vector>
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/math/intervalN.h"
#include "../BaseLib/math/Metric.h"
#include "../BaseLib/math/DynamicTimeWarping.h"
#include "clust/clust.h"

void readData(const char* fname, matrixn & out);

void Clustering::test(const char* fn)
{
	// simplified code
	matrixn samples(500,2);
	readData("../../BaseLib/math/clust/example1/data", samples);

	TArray<vectorn> test;
	test.init(500);
	for(int i=0; i<test.size(); i++)
		test[i]=samples.row(i);

	cluster(test);

	plot(samples, fn);
}

void Clustering::plot(matrixn const& sources,const char* fn)
{
	matrixn const &x=sources;


	intervalN range;
	range.calcRange(sources);
	range.normalizeRange(vectorn(2, 1.0, 1.0));

	vectorn d1;
	vectorn d2;
	d1.linspace(range.start(0), range.end(0), 50);
	d2.linspace(range.start(1), range.end(1), 50);

	DrawChart chart("x", "y", d1[0], d1[d1.size()-1], d2[0], d2[d2.size()-1]);

	for(int i=0; i<m_nNumGroup; i++)
	{
		matrixn ppp;
		for(int j=0; j<sources.rows(); j++)
		{
			if(m_aGroupIndex[j]==i)
				ppp.pushBack(sources.row(j));
		}
		if(ppp.rows())
			chart.drawScatteredData(ppp, Imp::GetColor(i), "../resource/default/pattern1.bmp");
	}
	TString t;
	t.format("../gnuplot/cluster_%s.bmp", fn);
	chart.save(t);	
}

void Clustering::test()
{
	{
		KMeanCluster c(3);
		c.test("Kmean");
	}
	{
		GMMCluster c(10, GMMCluster::FULL);
		c.test("GMMFull");
	}

	{
		GMMCluster c(10, GMMCluster::DIAG);
		c.test("GMMDiag");
	}

	{
		GMMCluster c(10, GMMCluster::SPHR);
		c.test("GMMSphr");
	}

	{
		GMMCluster c(GMMCluster::SPHR, 3);
		c.test("GMMSphr_3");
	}

	{
		GMMCluster c(GMMCluster::DIAG, 3);
		c.test("GMMDiag_3");
	}

	{
		AggloCluster c(L2Metric(), AggloCluster ::AVERAGE_LINKAGE, FLT_MAX, 3);
		c.test("AggloAverage");
	}

}
Cluster::Cluster(void)
{
	m_pMetric=new L2Metric();
}

Cluster::~Cluster(void)
{
	delete m_pMetric;
}

void Cluster::DTWKmeanCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_cluster_thr, int& cluster_n, intvectorn& group_index, matrixn& aCenter, matrixn& refPattern)
{
	group_index.setSize(aInputVec.rows());

	int refPatternIndex=0;

	if(aInputVec.rows()>1)
	{
		static matrixn DTWdistMat;
		CalcDTWDistanceMatrix(aInputVec, numRow, numColumn, DTWdistMat);

		m_real MIN_SSE=FLT_MAX;
		// find reference pattern
		for(int i=0; i<aInputVec.rows(); i++)
		{
			m_real SSE=0;
			for(int j=0; j<aInputVec.rows(); j++)
			{
				if(j!=i)
					SSE+=DTWdistMat[i][j];
			}
			if(SSE<MIN_SSE)
			{
				MIN_SSE=SSE;
				refPatternIndex=i;
			}
		}
	}

	CDynamicTimeWarping dtw;
	matrixn sample;
	matrixn timeWarpedPattern;
	timeWarpedPattern.setSize(aInputVec.rows(), aInputVec.cols());

	refPattern.fromVector(aInputVec.row(refPatternIndex), numColumn);

	for(int i=0; i<aInputVec.rows(); i++)
	{
		if(i==refPatternIndex)
		{
			timeWarpedPattern.row(i)=aInputVec.row(i);
		}
		else
		{
			sample.fromVector(aInputVec.row(i), numColumn);
			dtw.TimeWarpToReferencePattern(refPattern, sample);
			vectornView ri=timeWarpedPattern.row(i);
			sample.toVector(ri);
		}
	}

	if(timeWarpedPattern.rows()==1)
	{
		cluster_n=1;
		group_index[0]=0;
		aCenter.assign(timeWarpedPattern);
	}
	else
		ShinCluster(timeWarpedPattern, inner_cluster_thr, cluster_n, group_index, aCenter);
}

void Cluster::CalcDTWDistanceMatrix(const matrixn& aInputVec, int numRow, int numColumn, matrixn& distMat)
{
	matrixn sample1, sample2;
	CDynamicTimeWarping dtw;
	m_real distance;
	distMat.setSize(aInputVec.rows(),aInputVec.rows());
	ASSERT(numRow*numColumn==aInputVec.cols());

	for(int i=0; i<aInputVec.rows(); i++)
	{
		for(int j=i; j<aInputVec.rows(); j++)
		{
			sample1.fromVector(aInputVec.row(i), numColumn);
			sample2.fromVector(aInputVec.row(j), numColumn);
			//sample1.save("sample1.txt");
			distance=dtw.CalcDTWDistance(sample1, sample2);
			distMat[i][j]=distMat[j][i]=distance;
		}
	}
}

void Cluster::ShinCluster(const matrixn &aInputVec, m_real inner_thr, int &GroupNum, intvectorn& aGroupIndex, matrixn& aCenter)
{
#ifdef OUTPUT_TO_FILE
	OutputToFile("optseg.txt","GenerateOptimumCluster");
#endif
	int i;
	/////////////////////////////////////////////////////////////////////
	//	Clustering by K-means algorithm
	//  clustering
	matrixn initial_center;
	int numCluster = 1;

	/*
	// binary search FCM-> make initial solution
	FcmKCenter(aInputVec, numCluster, 3, initial_center, aGroupIndex);	
	OutputToFile("optseg.txt","FCMKCenter Ended");
	*/
	vectorn temp;
	vectorn temp1;

	m_real max_length = 0.0f;int max_index1 = 0;	int max_index2 = 0;
	m_real min_length = FLT_MAX; int min_index1 = 0; int min_index2 = 1;
	
	m_real thresh_max = inner_thr;
	m_real thresh_min = inner_thr/2.f;
	
	int dimension = aInputVec.row(0).getSize();
	
	int max_iter = 100;

	CTArray<intvectorn> aVectorGroup;

	/////////////////////////////////////////////////////////////////////
	//	Clustering by K-means algorithm
	KMeanCluster(aInputVec, numCluster, aGroupIndex, aCenter, initial_center);
	CheckNonEmpty(aInputVec, numCluster, aCenter, aGroupIndex.dataPtr());
	for(int iter=0; iter<max_iter; iter++)
	{

		CalcVectorGroup(aInputVec.rows(), numCluster, aGroupIndex.dataPtr(), aVectorGroup);
		CalcInnerClusterScore(aInputVec, aCenter, aVectorGroup, max_index1, max_index2, max_length);
		CalcInterClusterScore(aInputVec, aCenter, numCluster, min_index1, min_index2, min_length);

		/////////////////////////////////////////////////////////////////////
		// check score

		int numCluster_old = numCluster;

		if(max_length>thresh_max)
		{
			/////////////////////////////////////////////////////////////////////
			// for cluster split
			ASSERT(max_index1<numCluster);
			initial_center.setSize(numCluster+1, aInputVec.cols());
			for(i=0; i<numCluster; i++)
				initial_center.row(i).assign(aCenter.row(i));
			initial_center.row(numCluster).assign(aInputVec.row(aVectorGroup[max_index1][max_index2]));
			numCluster ++;
		}
		else if(min_length<thresh_min)
		{
			/////////////////////////////////////////////////////////////////////
			// for cluster merge
			initial_center.setSize(numCluster-1, aInputVec.cols());
			ASSERT(min_index1 != min_index2);
			ASSERT(min_index1 < numCluster && min_index2 < numCluster);
			int ind = 0;
			for(i=0; i<numCluster; i++){
				if(i != min_index1 && i != min_index2){
					initial_center.row(ind) = aCenter.row(i);
					ind ++;
				}
			}
			ASSERT(ind == numCluster-2);
			m_real t1 = aVectorGroup[min_index1].size();
			m_real t2 = aVectorGroup[min_index2].size();

			temp.mult(aCenter.row(min_index1), t1/(t1+t2));
			temp1.mult(aCenter.row(min_index2), t2/(t1+t2));
			initial_center.row(numCluster-2).add(temp, temp1);
			
			numCluster --;
		}
		else
			break;
#ifdef OUTPUT_TO_FILE
		MOutputToFile("optseg.txt", ("numCluster %d  ", numCluster_old) );
		MOutputToFile("optseg.txt", ("max_len %6.3f min_len %6.3f \n", max_length, min_length) );
#endif
		/////////////////////////////////////////////////////////////////////
		//	Clustering by K-means algorithm
		KMeanCluster(aInputVec, numCluster, aGroupIndex, aCenter, initial_center);
		if(CheckNonEmpty(aInputVec, numCluster, aCenter, aGroupIndex.dataPtr()))
#ifdef OUTPUT_TO_FILE
			OutputToFile("optseg.txt","zero cluster occurred");
#else
		 ;		
#endif

		if(numCluster>=aInputVec.rows()) break;
		if(numCluster<2) break;
	}
	GroupNum = numCluster;
#ifdef OUTPUT_TO_FILE
	OutputToFile("optseg.txt", "OptSegEnded");
#endif
}

m_real Cluster::CalcInnerClusterScore(const matrixn& aInputVec, const matrixn& aCenter, const CTArray<intvectorn>& aVectorGroup, int& maxGroupIndex, int& maxEltIndex, m_real& maxLength)
{
	/////////////////////////////////////////////////////////////////////
	// Inner-Cluster Score;
	maxLength = 0.0f; maxGroupIndex = maxEltIndex = 0;
	m_real distance;
	int numCluster=aVectorGroup.Size();
	m_real cluster_score, score_inner=0.f;
	for(int i=0; i<numCluster; i++)
	{
		cluster_score = 0.0f;
		for(int j=0; j<aVectorGroup[i].size(); j++)
		{
			distance=m_pMetric->CalcDistance(aCenter.row(i),aInputVec.row(aVectorGroup[i][j]));
			cluster_score +=distance;
			if(maxLength<distance)
			{maxGroupIndex = i; maxEltIndex=j; maxLength = distance;}
		}
		score_inner += (cluster_score / (aVectorGroup[i].size()));
	}
	return score_inner/(m_real)numCluster;
}

m_real Cluster::CalcInterClusterScore(const matrixn& aInputVec, const matrixn& aCenter, int numCluster, int& minGroup1, int& minGroup2, m_real& minLength)
{
	/////////////////////////////////////////////////////////////////////
	// Inter-Cluster Score;
	minLength = FLT_MAX; minGroup1 = 1; minGroup2 = 0;
	m_real distance;
	m_real cluster_score=0.f;
	for(int i=0; i<numCluster; i++)
	{
		for(int j=i+1; j<numCluster; j++)
		{
			distance=m_pMetric->CalcDistance(aCenter.row(i),aCenter.row(j));
			cluster_score+=distance;
			if(minLength>distance)
			{minGroup1 = i; minGroup2 = j; minLength = distance;}
		}
	}
	return cluster_score/m_real(numCluster*numCluster);
}

void Cluster::CalcVectorGroup(int numInput, int numCluster, int* pGroupIndex, CTArray<intvectorn>& aVectorGroup)
{
	aVectorGroup.Init(numCluster);
	for(int i=0; i<numInput; i++)
	{
#ifdef _DEBUG
		ASSERT(0<=pGroupIndex[i]);
		ASSERT(pGroupIndex[i]<numInput);
#endif
		aVectorGroup[pGroupIndex[i]].pushBack(i);
	}
#ifdef _DEBUG
	for(int i=0; i<numCluster; i++)
		ASSERT(aVectorGroup[i].size() !=0);
#endif
}

bool Cluster::CheckNonEmpty(const matrixn& aInputVec, int &numCluster, matrixn& aCenter, int *pGroupIndex)
{
	bool bChanged=false;
	// Empty클러스터가 있으면 그거 없앤다.
	CTArray<intvectorn> aVectorGroup;
	aVectorGroup.Init(numCluster);
	for(int i=0; i<aInputVec.rows(); i++)
		aVectorGroup[pGroupIndex[i]].pushBack(i);

	matrixn aCenterCpy;
	for(int i=0; i<numCluster; i++)
	{
		if(aVectorGroup[i].size() ==0)
		{
			// delete empty class
			aVectorGroup.Swap(i,numCluster-1);
			aCenter.row(i)=aCenter.row(numCluster-1);
			aVectorGroup.Resize(numCluster-1);
			aCenter.setSize(numCluster-1, aCenter.cols());
			MakeGroupIndex(aVectorGroup, pGroupIndex);
			numCluster--;
			i--;
			bChanged=true;
		}
	}
	return bChanged;
}

void Cluster::MakeGroupIndex(CTArray<intvectorn>& aVectorGroup, int* pGroupIndex)
{
	for(int i=0; i<aVectorGroup.Size(); i++)
	{
		for(int j=0; j<aVectorGroup[i].size(); j++)
		{
			pGroupIndex[aVectorGroup[i][j]]=i;
		}
	}
}

void Cluster::KMeanCluster(const matrixn& inputvectors, int cluster_n, intvectorn& group_index, matrixn& centers, const matrixn& input_centers)
{
	ASSERT(cluster_n<=inputvectors.rows());
	bool again;						// stable?
	int i, j, l;
	
	m_real factor;
	int max_iter=1000;	// maximum number of iteration
	
	matrixn presult;	// result of previous iteration
	matrixn cresult;	// result of current iteration
	matrixn distMat;	// distance between inputvectors and centers
	m_real dist;			// distance from center, variables
	vectorn direction;
	m_real mindist, thresh;
	matrixn sum;		// used for calculating centers
	
	int num_input=inputvectors.rows();
	vectorn min;
	vectorn max;

	intvectorn groupNum;
	groupNum.setSize(cluster_n);

	min.minimum(inputvectors);
	max.maximum(inputvectors);
	
	m_real interval=m_pMetric->CalcDistance(min,max);
	
	if(inputvectors.rows() < 1)
		return;
	else
	{
		ASSERT(input_centers.rows()==0 || input_centers.cols()==inputvectors.cols());

		cresult.setSize(cluster_n, inputvectors.cols());

		int num_initial_centers=MIN(input_centers.rows(), cluster_n);

		for(int i=0; i< num_initial_centers; i++)
		{
			// copy from initial values
			cresult.row(i).assign(input_centers.row(i));
		}

		int randv=rand();
		factor=(m_real)inputvectors.rows()/(m_real)cluster_n;

		for(int i=num_initial_centers; i<cluster_n; i++)
		{
			// random select
			int data_index=(int(factor*i+0.5)+randv)%num_input;
			cresult.row(i).assign(inputvectors.row(data_index));
		}
	}

	centers.setSize(cluster_n, inputvectors.cols());
	for(i=0;i<max_iter;i++)
	{
		again=false;
		
		m::distanceMat(distMat, inputvectors, cresult, m_pMetric);
		for(j=0;j<num_input;j++)
		{
			// for each data point, 가장 가까운 cluster로 지정
			distMat.row(j).findMin(mindist, l);
			group_index[j]=l;
		}
		
		sum.setSize(cluster_n, inputvectors.cols());
		sum.setAllValue(0);
		groupNum.setAllValue(0);
		for(j=0; j<num_input; j++)
		{
			sum.row(group_index[j])+=inputvectors.row(j);
			groupNum[group_index[j]]++;
		}
		presult.assign(cresult);

		for(l=0; l<cluster_n; l++)
		{
			if(groupNum[l]!=0)
				cresult.row(l).div(sum.row(l), groupNum[l]);
		}
		for(l=0; l<cluster_n; l++)
		{
			if(groupNum[l]==0)
			{
				m_real max_dist = 0;
				int max_index = -1;
				//
				for ( j = 0; j < num_input; j++) {
					dist=m_pMetric->CalcDistance(inputvectors.row(j),cresult.row(group_index[j]));
					if (dist > max_dist && groupNum[group_index[j]]>1) {
						max_dist = dist;
						max_index = j;
					}
				}
				//
				ASSERT(max_index >= 0 && max_index < num_input);
				groupNum[group_index[max_index]] --;
				group_index[max_index] = l;
				cresult.row(l) = inputvectors.row(max_index);
			}
		}		
		
		thresh=interval/1000.f;
		for(l=0;l<cluster_n;l++)
		{
			if(m_pMetric->CalcDistance(presult.row(l),cresult.row(l))>thresh)
			{
				again=true;
				break;
			}
		}
		if(!again) break;
	}

	centers.assign(cresult);
}

void Cluster::ChangeDistanceMetric(const Metric& cMetric)
{
	delete m_pMetric; m_pMetric=cMetric.Clone();
}

void ExactCluster::cluster(const TArray<vectorn>& aInputVec)
{
	printf("start clustering\n");
	int maxColumn=0;
	for(int i=0; i<aInputVec.size(); i++)
	{
		if(aInputVec[i].size()>maxColumn)
			maxColumn=aInputVec[i].size();
	}

	matrixn aNonTemporal;
	aNonTemporal.setSize(aInputVec.size(), maxColumn);
	aNonTemporal.setAllValue(-1);

	for(int i=0; i< aNonTemporal.rows(); i++)
		aNonTemporal.row(i).range(0, aInputVec[i].size())= aInputVec[i];

	//CImageProcessor::SaveMatrix(aNonTemporal,"feature.bmp");

	m_aGroupIndex.setSize(aInputVec.size());
	CFuzzyCluster::LexicoCluster(aNonTemporal, m_nNumGroup, m_aGroupIndex.dataPtr());
}

void KMeanCluster::cluster(const TArray<vectorn>& aInputVec)
{
	matrixn aInputs(aInputVec.size(), aInputVec[0].size());

	for(int i=0; i<aInputs.rows(); i++)
		aInputs.row(i).assign(aInputVec[i]);


	Cluster c;
	matrixn input_centers;
	matrixn centers;
	m_aGroupIndex.setSize(aInputs.rows());
	c.KMeanCluster(aInputs, m_nNumGroup, m_aGroupIndex, centers, input_centers);
}

void FuzzyCluster::cluster(const TArray<vectorn>& aInputVec)
{
	int column=aInputVec[0].size();
	for(int i=1; i<aInputVec.size(); i++)	ASSERT(aInputVec[i].size()==column);

	matrixn aTemporal;
	aTemporal.setSize(aInputVec.size(), column);
	for(int i=0; i< aTemporal.rows(); i++)
		aTemporal.row(i).range(0, aInputVec[i].size())= aInputVec[i];

	m_aGroupIndex.setSize(aInputVec.size());
	
	if(m_numCluster==1)
		CFuzzyCluster::FcmRadii(aTemporal, m_fRadii, m_aCenter, m_aGroupIndex.dataPtr());
	else
		CFuzzyCluster::FcmKCenter(aTemporal,  m_numCluster, m_numCluster/10+1, m_aCenter, m_aGroupIndex.dataPtr());

	m_nNumGroup=m_aCenter.rows();
}

int FuzzyCluster::findNearestGroup(const vectorn& inputVec)
{
	vectorn dist;
	dist.each2(sv2::distance(), m_aCenter, inputVec);
	return dist.argMin();
}

AggloCluster::AggloCluster(const Metric& metric, int eLinkage, m_real thr, int numCluster)
{
	m_pMetric=metric.Clone();
	m_fThr=thr;
	m_nNumGroup=numCluster;
	m_eLinkage=eLinkage;
}
AggloCluster::AggloCluster(const Metric& metric, int eLinkage, int numCluster)
{
	m_pMetric=metric.Clone();
	m_fThr=FLT_MAX;
	m_nNumGroup=numCluster;
	m_eLinkage=eLinkage;
}


AggloCluster::~AggloCluster()
{
	delete m_pMetric;
}

void AggloCluster::cluster(const TArray<vectorn>& aInputVec)
{
	m_aGroupIndex.setSize(aInputVec.size());

	m_distMat.setSize(aInputVec.size(), aInputVec.size());

	for(int i=0; i<aInputVec.size(); i++)
	{
		for(int j=i; j<aInputVec.size(); j++)
		{
			m_distMat[i][j]=m_distMat[j][i]=m_pMetric->CalcDistance(aInputVec[i], aInputVec[j]);
		}
	}
	CFuzzyCluster::AggloClusterCore(m_distMat, m_interDist, m_innerDist, m_fThr, m_nNumGroup, m_aGroupIndex.dataPtr(), m_eLinkage);
}

void AggloCluster::cluster_using_distances(const matrixn & distmat)
{
	int n=distmat.rows();
	m_aGroupIndex.setSize(n);
	m_distMat.setSize(n,n);
	for(int i=0; i<n; i++)
		for(int j=i; j<n; j++)
			m_distMat[i][j]=m_distMat[j][i]=(distmat(i,j)+distmat(j,i))*0.5;

	CFuzzyCluster::AggloClusterCore(m_distMat, m_interDist, m_innerDist, m_fThr, m_nNumGroup, m_aGroupIndex.dataPtr(), m_eLinkage);
}
TwoPhaseClustering::TwoPhaseClustering(Clustering* pC1, TFactory<Clustering>* pC2F, int numC1Elt)
:m_pC1(pC1),m_numC1Elt(numC1Elt) 
{
	m_aC2.changeFactory(pC2F);
}

TwoPhaseClustering::~TwoPhaseClustering()
{
	delete m_pC1;
}

void TwoPhaseClustering::cluster(const TArray<vectorn>& aInputVec)
{
	// first clustering method
	intvectorn columns;

	TArray<vectorn> aFeature1;
	TArray<vectorn> aFeature2;

	m_aGroupIndex.setSize(aInputVec.size());
	m_aGroupIndex.setAllValue(-1);

	// extract features
	columns.colon(0, m_numC1Elt);
	aFeature1.extract(aInputVec, columns.size(), columns.dataPtr());
	columns.colon(m_numC1Elt, aInputVec.size());
	aFeature2.extract(aInputVec, columns.size(), columns.dataPtr());


	// clustering 1st phase.
	m_pC1->cluster(aFeature1);
	
	int nSubCluster=m_pC1->numGrp();
	intvectorn& aSubGroupIndex=m_pC1->groupIndex();

	m_aC2.init(nSubCluster);
	m_aTargetIndex.init(nSubCluster);
	m_aGlobalGroupStartIndex.setSize(nSubCluster);

	// agglo+DTW clustering in subClusters
	int numGroup=0;
	intvectorn aGroupIndex(aInputVec.size());
	TArray<vectorn> aTempFeature;

	int currGlobalGroupStartIndex=0;

	for(int i=0; i<nSubCluster; i++)
	{
		m_aTargetIndex[i].findIndex(aSubGroupIndex,i);
		// init subclusters
		aTempFeature.extract(aFeature2,m_aTargetIndex[i].size(), m_aTargetIndex[i].dataPtr());

		m_aC2[i].cluster(aTempFeature);
		ASSERT(m_aC2[i].numGrp()!=0);
		m_aGlobalGroupStartIndex[i]=currGlobalGroupStartIndex;
		currGlobalGroupStartIndex+=m_aC2[i].numGrp();
	}

	// set output
	m_nNumGroup=currGlobalGroupStartIndex;

	intvectorn temp;
	for(int i=0; i<nSubCluster; i++)
		m_aGroupIndex.setAt(m_aTargetIndex[i], temp.add(m_aC2[i].groupIndex(), m_aGlobalGroupStartIndex[i]));

	ASSERT(m_aGroupIndex.findFirstIndex(-1)==-1);
}



// Number of clusters are predetermined.
GMMCluster::GMMCluster(Option option1, int numCluster)
:mOption(option1)
{
	mDesiredNumCluster=numCluster;
	mSig=NULL;
}
GMMCluster::GMMCluster(Option option1, int numCluster, int unused)
:mOption(option1)
{
	mDesiredNumCluster=numCluster;
	mSig=NULL;
}

// Determine the optimal number of clusters
GMMCluster::GMMCluster(int init_num_of_subclasses, Option option1)
:mOption(option1)
{
	mDesiredNumCluster=init_num_of_subclasses*-1;
	mSig=NULL;
}

void GMMCluster::cluster(const TArray<vectorn>& aInputVec)
{
	TString option;
	if(mOption==FULL)
		option="full";
	else if(mOption==DIAG)
		option="diag";
	else 
		option="sphr";

	matrixn samples(aInputVec.size(), aInputVec[0].size());

	for(int i=0; i<samples.rows(); i++)
		samples.row(i)=aInputVec[i];
	
	if(mDesiredNumCluster<0)
		mSig=new Clust::ClassSig(-1*mDesiredNumCluster, samples, option);
	else
		mSig=new Clust::ClassSig(2*mDesiredNumCluster, samples, option, mDesiredNumCluster);

	m_nNumGroup=mSig->numGaussian();

	m_aGroupIndex.setSize(samples.rows());

	vectorn logprob;
	for(int i=0; i<samples.rows(); i++)
	{
		mSig->logLikelihoods(aInputVec[i], logprob);

		m_aGroupIndex[i]=logprob.argMax();
	}
}


void GMMCluster::plot(matrixn const& sources,const char* fn)
{
	matrixn const &x=sources;

	intervalN range;
	range.calcRange(sources);
	range.normalizeRange(vectorn(2, 1.0, 1.0));

	vectorn d1;
	vectorn d2;
	d1.linspace(range.start(0), range.end(0), 50);
	d2.linspace(range.start(1), range.end(1), 50);

	vectorn xtest(2);
	matrixn ytest(50, 50);

	for (int i=0; i<50; i++)
		for(int j=0; j<50; j++)
		{
			xtest[0]=d1[i];
			xtest[1]=d2[j];
			ytest[i][j]=exp(mSig->logLikelihood(xtest));
		}

	DrawChart chart("x", "y", d1[0], d1[d1.size()-1], d2[0], d2[d2.size()-1]);
	chart.drawMatrix(ytest);

	for(int i=0; i<m_nNumGroup; i++)
	{
		matrixn ppp;
		for(int j=0; j<sources.rows(); j++)
		{
			if(m_aGroupIndex[j]==i)
				ppp.pushBack(sources.row(j));
		}
		if(ppp.rows())
			chart.drawScatteredData(ppp, Imp::GetColor(i), "../resource/default/pattern1.bmp");
	}
	TString t;
	t.format("../gnuplot/cluster_%s.bmp", fn);
	chart.save(t);
	
}
