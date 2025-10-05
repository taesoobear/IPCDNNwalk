// AlzzaPostureIP.h: interface for the AlzzaPostureIP class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_AlzzaPostureIP_H__F75EF1A1_0190_11D4_B82A_00A024452D72__INCLUDED_)
#define AFX_AlzzaPostureIP_H__F75EF1A1_0190_11D4_B82A_00A024452D72__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "interpolator.h"
#include "../../BaseLib/motion/my.h"
#include "../../BaseLib/utility/TArray.h"
class AlzzaCTransform;
class Posture;
class PLDPrimSkin;
class MotionLoader;
class Motion;
//! AlzzaCTransform 형태의 target을 Timer, Key값에 따라 interpolate한다.
/*! Rendering시에 사용된다. Interpolator,AlzzaCTransform 참고 */
class AlzzaPostureIP : public InterpolatorLinear
{
public:
	AlzzaPostureIP();
	virtual ~AlzzaPostureIP();
	virtual void Update(float cur_frame);

	void Init(int nJoint, float fFrameTime)	
	{
		m_nJoint=nJoint; 
		m_fFrameTime=fFrameTime; 
	}

	//! 원본데이타의 reference를 setting한다.
	void SetKeyValue(const Motion* pKeyValues)		{ m_nNumKey=pKeyValues->numFrames(); m_pKeyValues=pKeyValues;}
	//! 자신이 update해야 하는 벡터의 주소를 알아낸다.
	void SetTarget(PLDPrimSkin* pTarget)		{ m_pTarget=pTarget; };
	
	int GetFrameNumber()					{ return m_nNumKey;}	
	const Posture& pose(int iframe) const	{ return m_pKeyValues->pose(iframe);}

	const Motion& targetMotion() const		{ return *m_pKeyValues;}
private:
	int m_nJoint;
	// m_pKeyValues는 크기가 resize될수가 있으므로, 크기를 알고 싶을때는 m_nNumKey를 사용하시오.
	const Motion * m_pKeyValues;
	PLDPrimSkin* m_pTarget;
};

class MotionDOFInterpolator : public InterpolatorLinear
{
public:
	MotionDOFInterpolator():InterpolatorLinear(){}
	virtual ~MotionDOFInterpolator(){}
	virtual void Update(float cur_frame);
	void Init(float fFrameTime)	;
	void SetKeyValue(const MotionDOF* pKeyValues);
	void SetTarget(PLDPrimSkin* pTarget);
	int GetFrameNumber();
	const MotionDOF& targetMotion() const;
private:
	int m_nJoint;
	const MotionDOF* m_pKeyValues;
	PLDPrimSkin* m_pTarget;
};

#endif // !defined(AFX_AlzzaPostureIP_H__F75EF1A1_0190_11D4_B82A_00A024452D72__INCLUDED_)
