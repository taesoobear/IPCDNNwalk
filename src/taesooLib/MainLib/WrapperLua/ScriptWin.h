#ifndef RIGIDBODYINW_H
#define RIGIDBODYINW_H
#pragma once

#include "../OgreFltk/MotionPanel.h"
#include "../OgreFltk/FltkMotionWindow.h"
class VRMLloader;
#include "../OgreFltk/FltkRenderer.h"
#include "../OgreFltk/FlLayout.h"

#ifndef NO_GUI
class ScriptWin: public FlLayout, public FltkRenderer::Handler,public FrameMoveObject, public FltkMotionWindow::EventReceiver
#else
class ScriptWin: public FlLayout, public FrameMoveObject
#endif
{
public:	
	lua_State* L;
	int errorFunc;
	void checkErrorFunc(lunaStack&l);
	void luna_call(lunaStack& l,int numIn, int numOut);
	TString lastFunc;
protected:
	void _initLuaEnvironment();
	void _checkErrorFunc();
	void _loadScript(const char* script, const char* scriptstring=NULL);

	void getglobal(lunaStack& l, const char* func);
public:

	ScriptWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* defaultScript, const char* _defaultScriptFolder);
	ScriptWin(int w, int h,const char* title, const char* defaultScript, const char* _defaultScriptFolder);
	virtual ~ScriptWin(void);

	TString default_script;
	TString defaultScriptFolder;
	virtual void initLuaEnvironment();
	void loadScript(const char* script, const char* scriptstring=NULL);
	void dostring(const char* str);
	void dofile(const char* str);

	void releaseScript();
	FltkRenderer* m_renderer;
	MotionPanel* m_motionPanel;

#ifndef NO_GUI
	virtual int	handleRendererEvent(int ev) ;
	virtual int handleRendererMouseEvent(int ev, int x, int y, int button );
	// FltkMotionWindow::EventReceiver
	virtual void OnFrameChanged(FltkMotionWindow*, int currFrame);
#endif

	virtual int work(TString const& workname, lunaStack& L);
	TString _scriptFn;
	void setLabel(Fl_Button* b, TString const& name);
	TString getLabel(Fl_Button* b);

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);

	virtual void show();
	virtual void hide();
};
#endif
