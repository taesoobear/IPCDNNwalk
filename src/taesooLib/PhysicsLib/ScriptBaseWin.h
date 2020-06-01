
#ifndef SCRIGIDBODYINW_H
#define SCRIGIDBODYINW_H

#include "../MainLib/WrapperLua/ScriptWin.h"
#include "Liegroup.h"
class ScriptBaseWin: public ScriptWin
{
	bool _autoLoaded;

	void createMenu();
public:
	ScriptBaseWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* defaultScript, const char* _defaultScriptFolder);
	ScriptBaseWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer,  TStrings const& scripts);

	TStrings scripts;

	void loadDefaultScript();
	
	virtual ~ScriptBaseWin(void);
	virtual void initLuaEnvironment();
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);
};

#endif
