 // TimeSensor.cpp: implementation of the TimeSensor class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "timesensor.h"
#include "../BaseLib/utility/util.h"
#include "interpolator.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TimeSensor::TimeSensor() :Node()
{
	NodeType=TIMESENSOR;

	m_bPlaying=false;
	m_bLoop=false;
	m_pTrigger=NULL;
	m_fCycleInterval=1.f;
	m_fPrevFrame=-1.f;
	InitAnim(0.f, 1.f, 0.f);
}

TimeSensor::~TimeSensor()
{
	RemoveAllInterpolator();
}

void TimeSensor::RemoveAllInterpolator()
{
	std::list<Interpolator*>::iterator i;

    for (i = m_cInterpolatorList.begin(); i != m_cInterpolatorList.end(); ++i)
	{
		delete (*i);
	}
	m_cInterpolatorList.clear() ;
}

void TimeSensor::RemoveInterpolator(Interpolator* pInterpolator)
{
	m_cInterpolatorList.remove(pInterpolator);
}

void TimeSensor::Unpack(char *filename)
{
	FILE* temp;
	VERIFY(temp=fopen(filename,"r"));

	m_fCycleInterval=(float)atof(GetToken(temp));
	m_bLoop=atoi(GetToken(temp));
	VERIFY(GetToken(temp)==NULL);
	fclose(temp);
}

void TimeSensor::CloneFromAnother(TimeSensor* another)
{
	m_bPlaying=false;
	m_pTrigger=NULL;
	InitAnim(0.f, 1.f, 0.f);
	m_fCycleInterval=another->m_fCycleInterval;
	m_bLoop=another->m_bLoop;
}

void TimeSensor::AttachInterpolator(Interpolator* pInterpolator)
{
	m_cInterpolatorList.insert(m_cInterpolatorList.end(), pInterpolator);
}

int TimeSensor::FrameMove(float fElapsedTime)
{
	// update currframe
	if(m_bPlaying && m_bLoop)
	{
		m_fCurFrame+=fElapsedTime/m_fCycleInterval;

		if(m_fCurFrame>1) m_fCurFrame-=(float)floor(m_fCurFrame);
	}
	else if(m_bPlaying)
	{
		ASSERT(m_fEndFrame<=1.f);
		m_fCurFrame+=fElapsedTime/m_fCycleInterval;
		if(m_fCurFrame>m_fEndFrame)
		{
			m_bPlaying=false;
			
			if(m_pTrigger)
			{
				m_pTrigger->m_fElapsedTime=fElapsedTime;
				m_pTrigger->m_fOverTime=(m_fCurFrame-m_fEndFrame)*m_fCycleInterval;
				// 아래 문장은 필요하면 triggered안에서 할 수 있다. 
				//m_fCurFrame=m_fReturnFrame;
				// 멤버는 초기화를 해주고, 함수 call을 한다.
				Trigger* backup;
				backup=m_pTrigger;
				m_pTrigger=NULL;
				backup->Triggered(this);
				return FALSE;	// Triggered(this)안에서 this를 바꾼경우에도 문제가 없도록 바로 리턴한다. (멤버를 건드리지 않는다)
			}
			else
				m_fCurFrame=m_fReturnFrame;
		}
	}
	ASSERT(m_fCurFrame>=0-FERR);
	ASSERT(m_fCurFrame<=1+FERR);
	
	// 만약 바뀐 것이 없는 경우 update를 생략한다.
	if(m_fCurFrame==m_fPrevFrame) return FALSE;	
	update();
	return TRUE;
}

void TimeSensor::update()
{
	m_fPrevFrame=m_fCurFrame;

	std::list<Interpolator*>::iterator i;

	for (i = m_cInterpolatorList.begin(); i != m_cInterpolatorList.end(); ++i)
	{
		(*i)->Update(m_fCurFrame);
	}
}

void TimeSensor::TriggerSet(Trigger* pTrigger, float cur_frame, float end_frame, float return_frame)
{
	m_bLoop=false;
	InitAnim(cur_frame, end_frame, return_frame);

	ASSERT(m_pTrigger==NULL);
	m_pTrigger=pTrigger;
	StartAnim();	
}


void TimeSensor::InitAnim(int firstFrame, int lastFrame)
{
	int len=getNumFrameFromInterpolator()-1;

	if(lastFrame>len) lastFrame=len;
	if(firstFrame>lastFrame) firstFrame=lastFrame;

	float length=(float)len;
	if (length==0) length=0.000001;

	InitAnim(((float)firstFrame)/length, ((float)lastFrame)/length, ((float)lastFrame)/length);
}

void TimeSensor::TriggerSet(Trigger* pTrigger, int firstFrame, int lastFrame)
{
	int len=getNumFrameFromInterpolator()-1;

	if(lastFrame>len) lastFrame=len;
	float length=(float)len;

	
	TriggerSet(pTrigger, ((float)firstFrame)/length, ((float)lastFrame)/length, ((float)firstFrame)/length);
}

int TimeSensor::getCurFrameFromInterpolator()
{
	InterpolatorLinear* apip=((InterpolatorLinear*)(GetFirstInterpolator()));
	return apip->CalcFrameNumber(GetCurFrame());
}

float TimeSensor::getTotalTimeFromInterpolator()
{
	InterpolatorLinear* apip=((InterpolatorLinear*)(GetFirstInterpolator()));
	return apip->GetCycleInterval();
}

int TimeSensor::getEndFrameFromInterpolator()
{
	InterpolatorLinear* apip=((InterpolatorLinear*)(GetFirstInterpolator()));
	return apip->CalcFrameNumber(GetEndFrame());
}

int TimeSensor::getNumFrameFromInterpolator()
{
	InterpolatorLinear* apip=((InterpolatorLinear*)(GetFirstInterpolator()));
	return apip->GetNumFrame();	
}

int TimeSensor::getFrameFromInterpolator(float curFrame)
{
	InterpolatorLinear* apip=((InterpolatorLinear*)(GetFirstInterpolator()));
	return apip->CalcFrameNumber(curFrame);	
}

float TimeSensor::calcCurFrameFromInterpolator(int iframe)
{
	InterpolatorLinear* apip=((InterpolatorLinear*)(GetFirstInterpolator()));
	return apip->CalcCurrFrame(iframe);
}

FrameSensor::FrameSensor()
{
	m_fCurFrame=0.f;
	m_fFrameRate=0.f;
}

void FrameSensor::connect(Motion* pMotion, PLDPrimSkin* pSkin, bool bcontinuous)
{
	MotionTarget* pTarget=new MotionTarget (bcontinuous);
	pTarget->pMotion=pMotion;
	pTarget->pSkin=pSkin;
	
	if(pSkin->m_pTimer)
	{
		delete pSkin->m_pTimer;	// 이미 timer가 있다면 지운다.
		pSkin->m_pTimer=NULL;
	}
	connect(pTarget);
}

void FrameSensor::connect(FrameSensor::Target* pTarget)
{
	if(m_fFrameRate==0.f)
		m_fFrameRate=pTarget->frameRate();
	else
		Msg::verify(isSimilar(m_fFrameRate, pTarget->frameRate()), "Frame time should be the same");

	m_cTargetList.push_back(pTarget);

}

void FrameSensor::InitAnim(int curFrame)
{
	m_fCurFrame=int(curFrame);
}

void FrameSensor::triggerSet(Trigger* pTarget, float fTriggerTime)
{
	TriggerEvent ev;
	ev.pTrigger=pTarget;
	ev.fTriggerTime=fTriggerTime;

	// sort
	std::list<TriggerEvent>::iterator i;
	for(i=m_cTriggerList.begin(); i!=m_cTriggerList.end(); i++)
	{
		if((*i).fTriggerTime>fTriggerTime)
			break;
	}

	m_cTriggerList.insert(i, ev);
//#define TRACE_SORTED
#ifdef TRACE_SORTED
	for(i=m_cTriggerList.begin(); i!=m_cTriggerList.end(); i++)
	{
		printf("%f ", (*i).fTriggerTime);		
	}
	printf("\n");
#endif
}

void FrameSensor::MotionTarget::gotoFrame(m_real iframe)
{
	if(mbContinous)
	{
		Posture p;
		pMotion->samplePose(p, iframe);
		pSkin->SetPose(p, pMotion->skeleton());
	}
	else
		pSkin->setPose(*pMotion, (int)iframe);
}

float FrameSensor::MotionTarget::frameRate()
{
	return pMotion->frameRate();
}

int FrameSensor::FrameMove(float fElapsedTime)
{
	if(m_cTargetList.size())
	{
		ASSERT(m_fFrameRate!=0.0);

		m_fCurFrame+=fElapsedTime*m_fFrameRate;
				
		while (m_cTriggerList.size())
		{
			if(m_cTriggerList.front().fTriggerTime<=m_fCurFrame)
			{
				m_cTriggerList.front().pTrigger->Triggered(this);
				m_cTriggerList.pop_front();
			}
			else break;
		} 		

		for(std::list<Target*>::iterator i=m_cTargetList.begin(); i!=m_cTargetList.end(); i++)
		{
			(*i)->gotoFrame(m_fCurFrame);
		}
	}

	return TRUE;
}
