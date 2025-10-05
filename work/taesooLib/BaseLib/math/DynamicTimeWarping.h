#pragma once
#include "mathclass.h"
// CDynamicTimeWarpingdocument directory참고.
class CDynamicTimeWarping  
{
public:
	CDynamicTimeWarping();
	virtual ~CDynamicTimeWarping();

	m_real TimeWarpToReferencePattern(const matrixn& refP, matrixn& otherP);
	m_real CalcDTWDistance(const matrixn& a, const matrixn& b);

	enum { FROM_LEFT=0, FROM_BELOW, FROM_NOWHERE};

private:
	// cost matrix인덱스를 실제 입력 matrix의 인덱스로 바꾼다.
	inline int toAi(int i, int j)						{return i+2*j;}
	inline int toBi(int i, int j)						{return 2*i+j;}

	// calc euclidean distance between a[a_i] and b[b_i] (sum_j((a_ij-b_ij)^2))
	inline m_real Distance(const matrixn& a, const matrixn& b, int i, int j) 
	{
		int a_i=toAi(i,j);
		int b_i=toBi(i,j);
		return a_squaresum[a_i]+b_squaresum[b_i]-2*((a.row(a_i))%(b.row(b_i)));
	};

	matrixn cost;
	intmatrixn path;
	vectorn a_squaresum;	// vector of sum_j(aij^2)
	vectorn b_squaresum;	// vector of sum_j(bij^2)
};

