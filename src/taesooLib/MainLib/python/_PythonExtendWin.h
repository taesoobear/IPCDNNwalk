#pragma once

//#include "../../MainLib/WrapperLua/ScriptWin.h" // if you want to detach PhysicsLib dependencies
#include "../../PhysicsLib/ScriptBaseWin.h"
class MotionPanel;
class FrameSensor;
#ifndef USE_PYBIND11
#define USE_BOOST_PYTHON
#endif

#ifdef USE_BOOST_PYTHON
#include <boost/python.hpp>
namespace WRAP_PY=boost::python;
#else
#include <pybind11/pybind11.h>
namespace WRAP_PY=pybind11;
#endif
#include <Python.h>
// python extend win
class PythonExtendWin : public ScriptBaseWin
//public FlLayout, public FltkRenderer::Handler, public PLDPrimSkin::DrawCallback, public FrameMoveObject, public FrameSensor::Trigger 
{
public:
	PythonExtendWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~PythonExtendWin (void);

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




