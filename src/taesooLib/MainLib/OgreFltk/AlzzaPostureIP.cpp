// AlzzaPostureIP.cpp: implementation of the AlzzaPostureIP class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "../BaseLib/motion/Motion.h"
#include "AlzzaPostureIP.h"
//#include "AlzzaCTransform.h"
#include "../BaseLib/motion/postureip.h"
#include "pldprimskin.h"
//#include "surfelClouds.h"
#include "../BaseLib/motion/MotionDOF.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

AlzzaPostureIP::AlzzaPostureIP() : InterpolatorLinear()
{
}

AlzzaPostureIP::~AlzzaPostureIP()
{	
}

void AlzzaPostureIP::Update(float cur_frame)
{
	int iframe=CalcFrameNumber(cur_frame);

	m_pTarget->setPose(*m_pKeyValues, iframe);
}


void MotionDOFInterpolator::Update(float cur_frame)
{
	int iframe=CalcFrameNumber(cur_frame);
	m_pTarget->setPoseDOF(m_pKeyValues->row(iframe), m_pKeyValues->mInfo);
}

void MotionDOFInterpolator::Init(float fFrameTime)	
{
	m_fFrameTime=fFrameTime; 
}

void MotionDOFInterpolator::SetKeyValue(const MotionDOF* pKeyValues)		{ m_nNumKey=pKeyValues->numFrames(); m_pKeyValues=pKeyValues;}
void MotionDOFInterpolator::SetTarget(PLDPrimSkin* pTarget)		{ m_pTarget=pTarget; };

int MotionDOFInterpolator::GetFrameNumber()					{ return m_nNumKey;}	

const MotionDOF& MotionDOFInterpolator::targetMotion() const		{ return *m_pKeyValues;}
