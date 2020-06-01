// Interpolator.h: interface for the Interpolator class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_Interpolator_H__47F07AC1_0179_11D4_B82A_00A024452D72__INCLUDED_)
#define AFX_Interpolator_H__47F07AC1_0179_11D4_B82A_00A024452D72__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//! 상속한 클래스들 참고, Key의 array가 있을때, 현재 frame이 key[i]와 key[i+1]사이에 얼마쯤에 위치하는지 계산한다.

class Interpolator
{
public:
	Interpolator();
	virtual ~Interpolator();
	
	virtual void Update(float cur_frame)=0;	//!< keyvalue에 따라 달라지기 때문에 상속해서 구현한다.
	
protected:
	
};

class InterpolatorLinear : public Interpolator
{
public:
	InterpolatorLinear ();
	virtual ~InterpolatorLinear ();

	void init(float fFrameTime) { m_fFrameTime=fFrameTime;}
	void init(float fFrameTime, int nNumKey) {m_fFrameTime=fFrameTime; m_nNumKey=nNumKey;}

	int CalcFrameNumber(float cur_frame);
	float CalcCurrFrame(int iframe);

	float GetNumFrame()						{ return (float)m_nNumKey;}
	float GetFrameTime()					{ return m_fFrameTime;}
	// length=numKey-1
	float GetCycleInterval()				{ return GetFrameTime()*(m_nNumKey-1);}
	virtual void Update(float cur_frame){}	//!< keyvalue에 따라 달라지기 때문에 상속해서 재구현한다.

protected:
	float m_fFrameTime;
	int m_nNumKey;	
};

class InterpolatorPiecewiseLinear : public Interpolator
{
public:
	InterpolatorPiecewiseLinear ();
	virtual ~InterpolatorPiecewiseLinear ();

	//! Key Array를 setting한다. Key Array가 [0,1]의 uniform인 경우 NULL로 세팅한다. Key Array는 reference로 가정한다. 즉 원본이 따로 있다고 생각하여 copy하지 않고, 나중에 free하지도 않는다.
	void SetKey(int nNumKey, float* key) { m_nNumKey=nNumKey; m_aKey=key; };
	//! 현재의 cur_frame이 key[idx]와 key[idx+1]사이에 t위치에 존재함.
	void GetKeyPos(float cur_frame, int& i, float& t);

	//! depricated api. 현재의 cur_frame이 key[idx]와 key[idx-1]사이에 frac의 비율 위치에 존재함을 알아낸다.
	void GetKeyIdxFrac(float cur_frame, int *idx, float *frac);

private:
	int m_nNumKey;
	float* m_aKey;	//!< 원본의 포인터를 공유한다.
	int m_nPrevI;

};

#endif // !defined(AFX_Interpolator_H__47F07AC1_0179_11D4_B82A_00A024452D72__INCLUDED_)
