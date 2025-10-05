
#ifdef NO_GUI
#ifndef CONSOLE_TRACE_MANAGER_H
#define CONSOLE_TRACE_MANAGER_H
#include "../OgreFltk/TraceManager.h"
class TraceManager: public TraceBase, FrameMoveObject
{
public:
	
	TraceManager();
	virtual ~TraceManager(void);

	virtual int FrameMove(float fElapsedTime);
	virtual void draw();

	virtual void message(const char *id, const char *content);
};



#endif
#endif
