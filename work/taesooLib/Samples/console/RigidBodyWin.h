#pragma once

#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"
class FltkRenderer;
class VRMLloader;
#include "../../PhysicsLib/Liegroup.h"
#include "../../PhysicsLib/ScriptBaseWin.h"
FlLayout* createRigidBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);


class RigidBodyWin: public ScriptBaseWin
{
public:
	RigidBodyWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* defaultScript, const char* _defaultScriptFolder);
	virtual ~RigidBodyWin(void){}
	virtual void initLuaEnvironment();
	void firstInit();
};

