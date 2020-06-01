
#ifndef FLTK_MOTIONWINDOW_H_
#define FLTK_MOTIONWINDOW_H_
#include "../../MainLib/OgreFltk/framemoveobject.h"
class AnimationObject;
class FltkMotionWindow : public FrameMoveObject
{
	// Construction
public:
	class EventReceiver
	{
	public:
		EventReceiver(){}
		virtual ~EventReceiver(){}
		virtual void OnNext(FltkMotionWindow*)	{}
		virtual void OnPrev(FltkMotionWindow*)	{}
		virtual void OnFrameChanged(FltkMotionWindow*, int currFrame)	{}
	};

	FltkMotionWindow(int x, int y, int w);
	~FltkMotionWindow();

	void connect(EventReceiver& receiver);
	void disconnect(EventReceiver& receiver);

	void addSkin(AnimationObject* pSkin);
	void releaseAllSkin();
	void releaseSkin(AnimationObject* pSkin);
	void detachAllSkin();
	void detachSkin(AnimationObject* pSkin);
	// nskin이 음수일때는 -nskin개를 남겨놓고 나머지를 몽땅 지운다.
	void relaseLastAddedSkins(int nskins);

	int getCurrFrame()									{ return m_nCurrFrame;};
	int getNumFrame()									{ return m_numFrame;};
	int getNumSkin()									{ return (int)m_vecSkin.size();};
	AnimationObject* getSkin(int index)						{ return m_vecSkin[index];};

	void changeCurrFrame(int iframe);
	int playUntil(int iframe);
	int playFrom(int iframe);

	virtual int FrameMove(float fElapsedTime);
	// Implementation
protected:
	int m_nCurrFrame;
	int m_numFrame;
	std::vector<AnimationObject*> m_vecSkin;
	/// you can distinguish the caller by pWidget.
	void updateFrameNum();
	std::list<EventReceiver *> m_aEventReceiver;
};
#endif
