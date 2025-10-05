#include "gfunc.h"

using namespace std;

bool gmbsTraceJointsBackward(GJoint *pEndJoint_, GBody *pLeftBodyOfEndJoint_, list<GJoint *> &pTracedJoints_)
{
	GJoint *pjoint;
	list<GJoint *>::iterator iter_pjoint;

	pTracedJoints_.clear();

	if ( pEndJoint_ == NULL ) return false;

	pjoint = pEndJoint_;

	while (1) {

		if ( pjoint->pLeftBody == pLeftBodyOfEndJoint_ ) { break; }

		for (iter_pjoint = pjoint->pLeftBody->pJoints.begin(); iter_pjoint != pjoint->pLeftBody->pJoints.end(); iter_pjoint++) {
			if ( *iter_pjoint != NULL && !(*iter_pjoint)->isCut() && (*iter_pjoint)->pRightBody == pjoint->pLeftBody ) {
				pTracedJoints_.push_front(*iter_pjoint);
				break;
			}
		}

		if ( iter_pjoint == pjoint->pLeftBody->pJoints.end() ) { return false; }

		pjoint = *iter_pjoint;
	}

	pTracedJoints_.push_back(pEndJoint_);

	return true;
}
