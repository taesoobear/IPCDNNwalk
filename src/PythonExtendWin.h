#pragma once

//#include "../../MainLib/WrapperLua/ScriptWin.h" // if you want to detach PhysicsLib dependencies
#include <PhysicsLib/ScriptBaseWin.h>
class MotionPanel;
class FrameSensor;
#include <boost/python.hpp>
#include <Python.h>
// python extend win
class PythonExtendWin : public ScriptBaseWin
//public FlLayout, public FltkRenderer::Handler, public PLDPrimSkin::DrawCallback, public FrameMoveObject, public FrameSensor::Trigger 
{
public:
	boost::python::object m_main_module;
	boost::python::object m_main_namespace;
	PythonExtendWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~PythonExtendWin (void);


	// LUA interfacing.
	void __loadScript(const char* script);
	void __loadEmptyScript() { __loadScript(NULL);}
	void dostring(const char* script);
	void dofile(const char* pFilename );

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// PLDPrimSkin::DrawCallback
	virtual void draw(const Motion& mot, int iframe);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);

	// FrameSensor::Trigger 
	virtual void Triggered(FrameSensor* pTimer);

	// FltkRenderer::Handler
	virtual int handleRendererMouseEvent(int ev, int x, int y, int button);
	virtual int handleRendererKeyboardEvent(int ev, int key);
	virtual int	handleRendererEvent(int ev) ;

	virtual void OnFrameChanged(FltkMotionWindow*, int currFrame);
	virtual void initLuaEnvironment();

	void show();
	void hide();
	void firstInit();

	virtual int work(TString const& workname, lunaStack& L);
};




