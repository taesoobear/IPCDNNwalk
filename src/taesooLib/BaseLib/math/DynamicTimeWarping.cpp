// CDynamicTimeWarping.cpp: implementation of the CDynamicTimeWarping class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "DynamicTimeWarping.h"
#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//#define _TRACE_PATH
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CDynamicTimeWarping::CDynamicTimeWarping()
{

}

CDynamicTimeWarping::~CDynamicTimeWarping()
{

}

void CalcSquareMatrix(const matrixn& source, vectorn& target)
{
	target.setSize(source.rows());
	for(int i=0; i<source.rows();i++)
		target[i]=source.row(i).length();
}

inline m_real findMin(m_real a, m_real b, int& arg_min)
{
	if(a<b)
	{
		arg_min=0;
		return a;
	}
	else
	{
		arg_min=1;
		return b;
	}
}

/*
m_real CDynamicTimeWarping::CalcDTWDistance(matrixn& a, matrixn& b)
{
}
*/

void TracePath(matrixn& cost, intmatrixn& path)
{
	int i,j;
	i=cost.rows()-1;
	j=cost.cols()-1;
	while(path[i][j]!=CDynamicTimeWarping::FROM_NOWHERE)
	{
		TRACE("(%d,%d)<-\n",i,j);
		switch(path[i][j])
		{
		case CDynamicTimeWarping::FROM_BELOW:
			i--;
			break;
		case CDynamicTimeWarping::FROM_LEFT:
			j--;
			break;
		}
	}
}

m_real CDynamicTimeWarping::TimeWarpToReferencePattern(const matrixn& refP, matrixn& otherP)
{
	m_real distance;
	distance=CalcDTWDistance(refP, otherP);

	// cost matrix인덱스를 실제 입력 matrix의 인덱스로 바꾼다.
	matrixn sourceP;
	sourceP.assign(otherP);

	intvectorn targetIndex(otherP.rows());
	targetIndex.setAllValue(-1);

	int i,j;
	i=cost.rows()-1;
	j=cost.cols()-1;
	targetIndex[toAi(i,j)]=toBi(i,j);
	while(path[i][j]!=CDynamicTimeWarping::FROM_NOWHERE)
	{
#ifdef _TRACE_PATH
		TRACE("(%d,%d)<-\n",i,j);
#endif
		switch(path[i][j])
		{
		case CDynamicTimeWarping::FROM_BELOW:
			i--;
			break;
		case CDynamicTimeWarping::FROM_LEFT:
			j--;
			break;
		}
		targetIndex[toAi(i,j)]=toBi(i,j);
	}
	targetIndex[toAi(0,0)]=toBi(0,0);
	for(i=0; i<sourceP.rows(); i++)
	{
		if(targetIndex[i]==-1)
			targetIndex[i]=targetIndex[i-1];
	}

	intvectorn softTargetIndex=targetIndex;
	// refine(soften)
	for(i=0; i<sourceP.rows()-3; i++)
	{
		// local하게 기울기가 1이면
		if(targetIndex[i+3]-targetIndex[i]==3)
		{
			//중간의 한프레임 흔들림은 없애준다.
			softTargetIndex[i+1]=softTargetIndex[i]+1;
			softTargetIndex[i+2]=softTargetIndex[i]+2;
		}
	}

#ifdef _TRACE_PATH
	targetIndex.trace("TargetIndex");
	softTargetIndex.trace("sTargetIndex");
#endif
	otherP.extractRows(sourceP, softTargetIndex);
	return distance;
}

m_real CDynamicTimeWarping::CalcDTWDistance(const matrixn& a, const matrixn& b)
{
	m_real distance;

	int len_a=a.rows();
	int len_b=b.rows();
	
	CalcSquareMatrix(a, a_squaresum);
	CalcSquareMatrix(b, b_squaresum);
	int mat_width,mat_height;
	mat_width=(2*len_a-len_b+2)/3;	// num_rows
	mat_height=(2*len_b-len_a+2)/3; // num_columns

	if(mat_width<1)	// 길이가 두배이상 차이나는 경우. 맞추는게 불가능해서 매트릭스가 나오지 않는다.이경우 무한거리를 준다.
		return (m_real) INT_MAX;
	if(mat_height<1)
		return (m_real) INT_MAX;

	ASSERT((mat_width-1)*2+(mat_height-1)*1<len_a);
	ASSERT((mat_width-1)*1+(mat_height-1)*2<len_b);


	cost.setSize(mat_height, mat_width);
	path.setSize(mat_height, mat_width);
	path.setAllValue(FROM_NOWHERE);

	// i means matrix row number, j means matrix column number
	int i=0,j=0;
	int arg_min;
	cost[i][j]=Distance(a,b,i,j);
	path[i][j]=FROM_NOWHERE;
	for(j=1; j<cost.cols(); j++)
	{
		cost[i][j]=cost[i][j-1]+Distance(a,b,i,j);
		path[i][j]=FROM_LEFT;
	}
	
	for(i=1; i<cost.rows(); i++)
	{
		j=0;
		cost[i][j]=cost[i-1][j]+Distance(a,b,i,j);
		path[i][j]=FROM_BELOW;
		
		for(j=1; j<cost.cols(); j++)
		{
			distance=Distance(a,b,i,j);

			cost[i][j]=findMin(cost[i][j-1]+distance,
								cost[i-1][j]+distance, arg_min);
			path[i][j]=arg_min;
		}
	}

#ifdef _TRACE_PATH
	TracePath(cost, path);
#endif
	distance=cost[cost.rows()-1][cost.cols()-1]/((m_real) ( (cost.rows()-1)+(cost.cols()-1) ) );
	return distance;
}
