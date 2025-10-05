
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

	// LUA interfacing.
	// after releasing previous script, this calles loadScript. Basically it sets g_luaScript, and then dofile it and calls ctor().
	void __loadScript(const char* scriptFile);

	// ctor() is not called
	void __loadEmptyScript() { __loadScript(NULL);}

	void dostring(const char* script);
	void dofile(const char* pFilename );
	
	virtual ~ScriptBaseWin(void);
	virtual void initLuaEnvironment();
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);
};

#endif
