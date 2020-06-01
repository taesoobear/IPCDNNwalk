#include "stdafx.h"
#include "GlobalUI.h"
#include "FlLayout.h"
#include "renderer.h"
#include "FltkRenderer.h"
#include "../MainLib/WrapperLua/LUAwrapper.h"
#include "../MainLib/WrapperLua/luna.h"
#include "../MainLib/WrapperLua/luna_baselib.h"
#include "../MainLib/WrapperLua/luna_mainlib.h"

#ifndef WIN32
#include <unistd.h>
#endif


void registerFlChoiceWins(lua_State* L);
// singleton class
static FlChoiceWins* _wins=NULL;
static GlobalUI* _GUI=NULL;

void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);

GlobalUI::GlobalUI(FlChoiceWins* wins,int argc, char* argv[])
{
	_wins=wins;
	_GUI=this;
	_param.resize(argc-1);

	for(int i=1; i<argc; i++)
		_param[i-1]=argv[i];

	_L=NULL;
	if(argc>1)
	{
		_L=new LUAwrapper();
		Register_baselib(_L->L);
		Register_mainlib(_L->L);
		lunaStack ls(_L->L);
		ls.set<TStrings>("param", &_param);
		ls.set<GlobalUI>("GUI", _GUI);
		ls.set<FlChoiceWins>("wins", _wins);
		_L->dofile(_param[0]);
	}
}

int GlobalUI::work(TString const& wn, lunaStack& L)
{
	if(wn=="showOgreTraceManager")
	{
		RE::FltkRenderer().onCallback(NULL, Hash("OgreTraceManager"));

	}

	return 0;
}
GlobalUI* getGlobalUI()
{
	return _GUI;
}
GlobalUI::~GlobalUI()
{
  if(_L) delete _L;
  _L=NULL;
}
