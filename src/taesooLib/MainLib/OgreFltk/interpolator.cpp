// Interpolator.cpp: implementation of the Interpolator class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "interpolator.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Interpolator::Interpolator()
{
}

Interpolator::~Interpolator()
{
}

InterpolatorLinear::InterpolatorLinear()
{
	m_nNumKey=0;
}

InterpolatorLinear::~InterpolatorLinear()
{
}

int InterpolatorLinear::CalcFrameNumber(float cur_frame)
{
	int iframe;
	float length=(float)m_nNumKey-1.f;
	iframe=ROUND(cur_frame*length);
	if(iframe<0) iframe=0;
	if(iframe>=m_nNumKey) iframe=m_nNumKey-1;
	return iframe;
}

float InterpolatorLinear::CalcCurrFrame(int iframe)
{
	float length=(float)m_nNumKey-1.f;
	float cur_frame;
	cur_frame=((float)iframe)/length;
	if(cur_frame<0.f) cur_frame=0.f;
	if(cur_frame>1.f) cur_frame=1.f;
	return cur_frame;
}


InterpolatorPiecewiseLinear ::InterpolatorPiecewiseLinear ()
{
	m_nNumKey=0;
	m_aKey=NULL;
	m_nPrevI=0;
}

InterpolatorPiecewiseLinear ::~InterpolatorPiecewiseLinear ()
{
	// key는 공유한 값이므로 지우지 않는다. 
}

//! 현재의 cur_frame이 key[idx]와 key[idx+1]사이에 t위치에 존재함.
void InterpolatorPiecewiseLinear ::GetKeyPos(float cur_frame, int& i, float& t)
{
	if(m_aKey==NULL)
	{
		ASSERT(cur_frame>=0 && cur_frame<=1);

		// uniform
		float fframe;
		fframe=(cur_frame*((float)m_nNumKey-1));

		i=(int)fframe;
		ASSERT(i>=0);

		// fframe     0 1 2 3 4 5 6 7 8 9
		// curframe   0                 1
		if(i>=m_nNumKey-1)
		{
			i=m_nNumKey-2;
			t=1.f;
		}
		else
		{
			t=fframe-(float)i;
		}
	}
	else
	{
		int idx;
		float frac;
		GetKeyIdxFrac(cur_frame, &idx, &frac);
		i=idx-1;
		t=1.f-frac;
	}
}

void InterpolatorPiecewiseLinear::GetKeyIdxFrac(float cur_frame, int *pIdx, float *pFrac)
{
	ASSERT(cur_frame>=0 && cur_frame<=1);
	// 현재 temporal coherency를 이용하는 linear search이다.
	// binary search로 개선할 수 있다.

	for((*pIdx)=m_nPrevI;(*pIdx)<m_nNumKey;(*pIdx)++)
	{
		if(m_aKey[(*pIdx)]>cur_frame+FERR) break;
	}

	ASSERT((*pIdx)>0 );
	if(m_aKey[(*pIdx)-1]>cur_frame+FERR) 
	{
		// 다시 찾기 
		for((*pIdx)=0;(*pIdx)<m_nPrevI;(*pIdx)++)
		{
			if(m_aKey[(*pIdx)]>cur_frame+FERR) break;
		}
	}
	ASSERT((*pIdx)>0 );
	m_nPrevI=(*pIdx);

	if((*pIdx)>=m_nNumKey)
	{
		(*pIdx)=m_nNumKey-1;
		(*pFrac)=0.001f;
	}
	else 
	{
		(*pFrac)=(m_aKey[(*pIdx)]-cur_frame)/(m_aKey[(*pIdx)]-m_aKey[(*pIdx)-1]);
		if((*pFrac)<0.f+FERR) (*pFrac)=0.001f;
		if((*pFrac)>1.f-FERR) (*pFrac)=0.999f;
	}

	ASSERT((*pFrac)>0.f&& (*pFrac)<1.f);
}

/* sample usage
void InterpolatorPiecewiseLinear::Update(float cur_frame)
{
	int i=0;
	float frac;
	GetKeyIdxFrac(cur_frame, &i, &frac);
	*link=frac*keyvalue[i-1]+(1-frac)*keyvalue[i];
}
*/
