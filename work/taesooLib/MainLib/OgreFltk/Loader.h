#ifndef NO_GUI
#ifndef _LOADER_H_
#define _LOADER_H_

#if _MSC_VER>1000
#pragma once
#endif
#include "../WrapperLua/LUAwrapper.h"
#include "FltkAddon.h"
class Fl_Output;
class Fl_Group;
class Motion;
class MotionDOF;
class MotionDOFcontainer;
class MotionDOFinfo;
class MotionLoader;
class Loader : public LUAwrapper::Worker
{
public:

	LUAwrapper * L;
	virtual int work(TString const& workname, lunaStack& L);
	int _work(TString const& workname, TString const& arg);
	class EventReceiver
	{
	public:
		EventReceiver(){}
		virtual ~EventReceiver(){}
		// EventReceiver가 Loader한테 어디다 로딩할지 알려준다.
		virtual void OnLoadStart(int numCharacter, std::vector<Motion*>& targetMotions){}
		virtual void OnLoadStart(int numCharacter, std::vector<MotionDOFinfo const*>&aInfo, std::vector<MotionDOFcontainer*>& targetMotions){}
		virtual void OnLoadEnd(){}
	};

	Loader(int x, int y, int w, int h, EventReceiver* pEventReceiver=NULL);
	~Loader(void);

	void startManualLoad(int numCharacters, std::vector<Motion*>& motions);
	void endManualLoad(std::vector<Motion*> const& motions);

	bool mbAutoLoad;


	int numCharacter(int eMotion);


	void changeFactory(MotionLoader* skeleton, const char* type_name);
	static void constraintAutomaticMarking(Motion& mot);
	void onCallback(Fl_Widget* pWidget, int userData);
	Fl_Group* mGroup;
private:
	EventReceiver* m_pEventReceiver;
	void calcInterCon();
	Motion* mTargetMotion;
	Motion* mTargetMotion2;
	friend class MotionPanel;
};
#endif

#endif
