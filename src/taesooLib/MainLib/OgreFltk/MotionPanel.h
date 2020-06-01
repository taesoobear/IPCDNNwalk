#ifndef NO_GUI

#ifndef _MOTIONPANEL_H_
#define _MOTIONPANEL_H_

#if _MSC_VER>1000
#pragma once
#endif


class FltkMotionWindow;
class FltkScrollPanel;
class TraceManager;
class AnimationObject;
class TStrings;
#include "Loader.h"
#include "../../BaseLib/motion/MotionWrap.h"
#include <Fl/Fl_Double_Window.H>
// Loader, MotionWindow, Scrollpanel, Tracemanager을 세트로 갖고 있다. 아주 유용.
class MotionPanel :
	public Fl_Double_Window, public FlCallee, public Loader::EventReceiver
{
private:
	friend class MotionPanel_impl;

	FlMenu m_menuMotion;
	TArray<MotionWrap> m_motions;
	intvectorn mPairMotionIndex;
	Loader* m_loader;
	FltkMotionWindow* m_motionWin;
	FltkScrollPanel* m_scrollPanel;
	TraceManager* m_traceManager;
	FlMenu m_menuOp;
public:
	MotionPanel(int x, int y, int w, int h);
	virtual ~MotionPanel(void);

	FltkMotionWindow* motionWin()	{ return m_motionWin;}
	FltkScrollPanel* scrollPanel()	{ return m_scrollPanel;}
	Loader* loader()				{ return m_loader;}

	virtual void OnLoadStart(int numCharacter, std::vector<Motion*>& targetMotions);
	virtual void OnLoadStart(int numCharacter, std::vector<MotionDOFinfo const*>&aInfo, std::vector<MotionDOFcontainer*>& targetMotions);
	virtual void OnLoadEnd();
	virtual void onCallback(Fl_Widget* pWidget, int userdata);

	Motion& currMotion();
	MotionDOF& currMotionDOF();
	MotionDOFcontainer& currMotionDOFcontainer();
	MotionWrap& currMotionWrap();
	bool hasMotionDOF();
	bool hasPairMotion();
	Motion& currPairMotion();

	void changeCurrMotion(Motion const& mot);
	// copy.
	Motion const& registerMotion(Motion const& mot);
	// reference.
	void registerMotion(MotionDOF const& mot);
	void releaseMotions();

	int numMotion()	const		{return m_motions.size();}
	Motion& motion(int i) const	{return m_motions[i].mot();}
	MotionDOF& motiondof(int i) const	{return m_motions[i].motdof();}
};




#endif
#endif
