#include "stdafx.h"
#include "../OgreFltk/TraceManager.h"
#include "traceManager.h"


#ifdef NO_GUI

#include "../OgreFltk/renderer.h"
#include "../OgreFltk/RE.h"

namespace RE
{
	extern Globals* g_pGlobals;
}

TraceManager::TraceManager()
:FrameMoveObject()
{
	RE::g_pGlobals->g_traceManagers.push_back(this);
/*
	// start a new file
	FILE* file=fopen("trace.txt", "w");
	fclose(file);
*/
	RE::renderer().addFrameMoveObject(this);
}

TraceManager::~TraceManager(void)
{
	eraseAll();
//	RE::g_traceManagers.erase(std::find(RE::g_traceManagers.begin(), RE::g_traceManagers.end(), (AbstractTraceManager*)this));
	RE::renderer().removeFrameMoveObject(this);
}


void TraceManager::message(const char *id, const char *content)
{
	TraceBase::message(id, content);
}
int TraceManager::FrameMove(float fElapsedTime)
{
	draw();
	return 1;
}
void TraceManager::draw()
{

/*	
	namedmapTDM::iterator j;
	int y=20;
	int x=0;
	OutputToFile("trace.txt", "---------------------------------");

	TString temp;
	for(j = m_aMessage.begin(); j != m_aMessage.end(); ++j)
	{
		temp.format("%s\t%s", j->first.ptr(), j->second->mMessage.ptr());
		OutputToFile("trace.txt", temp.ptr());
	}
	*/
}

#endif
