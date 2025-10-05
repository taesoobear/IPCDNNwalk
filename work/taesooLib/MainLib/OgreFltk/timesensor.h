// TimeSensor.h: interface for the TimeSensor class.
#pragma once

#include "../../BaseLib/motion/node.h"
#include <list>

class Interpolator;
class PLDPrimSkin;
//! 에니메이션의 시간진행을 관리한다.
/*!
TimeSensor는 해당 Interpolater의 리스트를 갖는다. TimeSensor의 FrameMove를 call하면 해당 interpolator가 target을 update한다.
*/

class TimeSensor : public Node  
{
public:
	class Trigger
	{
	public:
		Trigger(){m_fOverTime=0.f;}
		virtual ~Trigger(){}
		virtual void Triggered(TimeSensor* pTimer)=0;		
	protected:
		float m_fOverTime;	//!< Trigger가 실제 일어나야 하는 타이밍에서 몇초가 지났는지를 저장한다.
		float m_fElapsedTime; //!< 이전 프레임과의 차이. m_fOverTime과 함께 사용하면, 어디까지 플레이 되었는지를 정확히 알수 있다.
		friend class TimeSensor;
	};

	TimeSensor();
	virtual ~TimeSensor();
	
	void Unpack(char *fileName);

	/// 이함수를 직접 call하지 마시오. RenderPrimitive에서만 call할수 있습니다.
	int FrameMove(float fElapsedTime);
	
	inline void FirstInit(float cycleInterval, bool bloop)							{ m_fCycleInterval=cycleInterval; m_bLoop=bloop; }
	inline void loop(bool bloop)													{ m_bLoop=bloop;}
	inline void InitAnim(float curframe, float endframe, float returnframe)			{ ASSERT(m_pTrigger==NULL); m_fCurFrame=curframe; m_fEndFrame=endframe; m_fReturnFrame=returnframe; m_fPrevFrame=-1.f;}
	inline void StartAnim()															{ ASSERT(m_fCurFrame<=m_fEndFrame); m_bPlaying=true; }
	inline void StopAnim()															{ m_pTrigger=NULL;m_bPlaying=false; };
	inline bool	IsPlaying()															{ return m_bPlaying;};

	/// timer한테 Trigger를 setting한다. pTimer의 play가 시작된다. 끝난 후, pTrigger의 Triggered가 call된다.
	void TriggerSet(Trigger* pTrigger, float cur_frame, float end_frame, float return_frame);

	// utility for motion control
	void InitAnim(int firstFrame, int lastFrame=INT_MAX);
	void TriggerSet(Trigger* pTrigger, int firstFrame, int lastFrame=INT_MAX);
	int getCurFrameFromInterpolator();
	int getEndFrameFromInterpolator();	
	int getNumFrameFromInterpolator();
	float getTotalTimeFromInterpolator();
	int getFrameFromInterpolator(float curFrame);
	float calcCurFrameFromInterpolator(int iframe);


	void AttachInterpolator(Interpolator* pInterpolator);
	void RemoveAllInterpolator();	//!< delete interpolator까지 한다.
	void RemoveInterpolator(Interpolator* pInterpolator);	//!< list에서만 지운다.
	void CloneFromAnother(TimeSensor* another);
	inline float GetCurFrame()														{ return m_fCurFrame;};
	inline float GetEndFrame()														{ return m_fEndFrame;};
	inline float GetReturnFrame()													{ return m_fReturnFrame;};
	inline float GetCycleInterval()													{ return m_fCycleInterval;};	
	bool hasInterpolator() const { return !m_cInterpolatorList.empty();}
	Interpolator* GetFirstInterpolator();
	void update();
private:
	
	float m_fCycleInterval;
	float m_fCurFrame;			//!< 현재 play되고 있는 frame in [0, 1]
	float m_fEndFrame;			//!< animation을 끝내는 frame
	float m_fReturnFrame;		//!< animation이 끝난 후 보여줄 frame

	bool m_bLoop;				//!< true이면 animation은 무한 반복.
	bool m_bPlaying;			//!< loop 이 false인 경우 , 현재 animation이 진행중인지를 저장.

	float m_fPrevFrame;

	std::list<Interpolator*> m_cInterpolatorList;

	Trigger* m_pTrigger;

};

class FrameSensor : public FrameMoveObject
{
public:
	FrameSensor();
	virtual ~FrameSensor(){}

	class Trigger
	{
	public:
		Trigger(){m_fOverTime=0.f;}
		virtual ~Trigger(){}
		virtual void Triggered(FrameSensor* pTimer)=0;		
	protected:		
		float m_fOverTime;	//!< Trigger가 실제 일어나야 하는 타이밍에서 몇초가 지났는지를 저장한다.
		float m_fElapsedTime; //!< 이전 프레임과의 차이. m_fOverTime과 함께 사용하면, 어디까지 플레이 되었는지를 정확히 알수 있다.
		friend class TimeSensor;
	};

	class Target
	{
		double mFrameRate;
	public:
		Target(double frame_rate=30.0):mFrameRate(frame_rate){}
		virtual ~Target(){}
		virtual float frameRate(){return (float)mFrameRate;}
		// update pose or animations based on curframe.
		virtual void gotoFrame(m_real curframe)=0;
	};

	void connect(Motion* pMotion, PLDPrimSkin* pSkin, bool bcontinuous=false);
	void connect(Target* pTarget);
	void InitAnim(int curFrame);
	virtual int FrameMove(float fElapsedTime);

	// TriggerTime 이 30이면 동작의 pose(30)이 그려지기 직전에 trigger된다. (Triggered just before rendering the pose(30) if fTriggerTime=30)
	// 즉, Trigger를 받은 쪽에서 동작의 pose(30)부터 editing할 수 있다. 
	// (MotionTarget::mbContinuous==true인경우 pose(31)부터 editing할 수 있다. - pose(30)은 이미 inter-frame 보간에 사용되었기 때문.
	void triggerSet(Trigger* pTarget, float fTriggerTime);
	
	float curFrame() {return m_fCurFrame;}
private:
	float m_fCurFrame;	// 현재 play되고 있는 frame in [0, n]
	float m_fFrameRate;
	struct TriggerEvent
	{
		Trigger* pTrigger;
		float fTriggerTime;
	};
	std::list<TriggerEvent> m_cTriggerList;
	
	class MotionTarget : public Target
	{
		bool mbContinous;
	public:
		MotionTarget (bool bContinuous):mbContinous(bContinuous){}
		virtual ~MotionTarget(){}
		virtual float frameRate();
		virtual void gotoFrame(m_real iframe);
		Motion* pMotion;
		PLDPrimSkin* pSkin;
	};
	std::list<Target*> m_cTargetList;
};

