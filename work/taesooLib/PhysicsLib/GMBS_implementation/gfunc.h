//================================================================================
//         GRAPH FUNCTIONS FOR MULTIBODY SYSTEM
//                                                                     release 0.1 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GRAPH_FUNCTIONS_
#define _GMBS_GRAPH_FUNCTIONS_

#include <list>
#include "gbody.h"
#include "gjoint.h"


bool gmbsTraceJointsBackward(GJoint *pEndJoint_, GBody *pLeftBodyOfEndJoint_, std::list<GJoint *> &pTracedJoints_);

#endif

