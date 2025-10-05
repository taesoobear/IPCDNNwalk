#pragma once

#include "../../BaseLib/math/mathclass.h"
#include "../../ClassificationLib/math/svm/svm.h"
#include <vector>
#include "../../BaseLib/utility/TArray.h"

struct SClass
{
	SClass(int data, int c)	{ hData=data; hClass=c;};
	int hData;
	int hClass;
};

class CSVMWrapOnline
{
public:

};

// Offline
class CSVMWrap
{
public:
	
	// 하나의 데이타가 data가 vector인경우.
	CSVMWrap(matrixn& data, bool bNormalize=true, int oneDataNumRow=0);	// 이 함수 안에서 data가 normalize되면서 내용이 바뀐다는 사실에 주의할 것.
	// 하나의 데이타가 data가 vector의나열 즉 matrix인경우. (DynamicTimeWarping kernel을 쓰는 경우가 대표적)
	CSVMWrap(CTArray<matrixn>& data, bool bNormalize=true);	// 이 함수 안에서 data가 normalize되면서 내용이 바뀐다는 사실에 주의할 것.
	
	virtual ~CSVMWrap();

	// utility functions
	void AddTrainingData(int hData, int hClass)			{ m_setTrainingData.push_back(SClass(hData,hClass));};
	int GetPredictedClass(int hData)					{ return m_sAllData.y[hData];};
	int GetUserSpecifiedClass(int hdata)	
	{
		for(int i=0; i<m_setTrainingData.size(); i++)	
			if(m_setTrainingData[i].hData==hdata) return m_setTrainingData[i].hClass; 
		return -1;
	}

	/// Train and predict
	/**
	m_sAllData는 모든 데이타 즉 사용자가 specify하지 않은 데이타까지 모두 포함하고 있다. 생성자에서 만들어진다.
	
	m_setTrainingData는 사용자가 몇번째 data의 클래스가 무엇인지 pair의 list를 갖고 있다. EG) (0,PUNCH)->(10, KICK)->...
	  :: TrainAndPredict를 수행하기 전에 반드시 만들어준다.

	결과는 m_sAllData의 y에 저장된다. EG) m_sAllData.y[0]<-PUNCH, m_sAllData.y[1]<-KICK...

	  Option: trainning option.
	    -s svm_type : set type of SVM (default 0)
			0 -- C-SVC
			1 -- nu-SVC
			2 -- one-class SVM
			3 -- epsilon-SVR
			4 -- nu-SVR
		-t kernel_type : set type of kernel function (default 2)
			0 -- linear: u'*v
			1 -- polynomial: (gamma*u'*v + coef0)^degree
			2 -- radial basis function: exp(-gamma*|u-v|^2)
			3 -- sigmoid: tanh(gamma*u'*v + coef0)
			4 -- GDTW: exp(-gamma*DistanceDTW) , this object should be created by CSVMWrap(CTArray<matrixn>& data..) constructor function.
			5 -- Kovar metric: exp(-gamma*kovardist)
			6 -- Quater metric: exp(-gamma*Quaterdist)
		-d degree : set degree in kernel function (default 3) or Number of elements in one data when using GDTW kernel.
		-g gamma : set gamma in kernel function (default 1/k)
		-r coef0 : set coef0 in kernel function (default 0)
		-c cost : set the parameter C of C-SVC, epsilon-SVR, and nu-SVR (default 1)
		-n nu : set the parameter nu of nu-SVC, one-class SVM, and nu-SVR (default 0.5)
		-p epsilon : set the epsilon in loss function of epsilon-SVR (default 0.1)
		-m cachesize : set cache memory size in MB (default 40)
		-e epsilon : set tolerance of termination criterion (default 0.001)
		-h shrinking: whether to use the shrinking heuristics, 0 or 1 (default 1)
		-wi weight: set the parameter C of class i to weight*C, for C-SVC (default 1)
		-v n: n-fold cross validation mode
	*/

	void Train(const char* option);
	int Predict(const vectorn & datum);

	// Train한후 모든 입력 데이타를 predict한다.
	void TrainAndPredict(const char* option);

	// pair of data index and its class. This should be set before TrainAndPredict.
	std::vector<SClass> m_setTrainingData;	

	// this is constructed in constructor funcition. Also result will be in m_sAllData.y
	svm_problem m_sAllData;

private:
	void Release();
	void ScaleData(matrixn& data);
	double fGamma;
	svm_node *m_xspace;
	int m_nOneDataNumRow;
	void Initialize(matrixn& data, bool bNormalize);
	static void GetMatrix(const svm_node* pNode, int nMatrixRow, matrixn& mat);
	static void GetVector(const svm_node* pNode, vectorn& vec);
	friend class Kernel;

	bool mbNormalize;

	svm_parameter param;
	svm_problem prob;
	svm_model *model;
	
};

