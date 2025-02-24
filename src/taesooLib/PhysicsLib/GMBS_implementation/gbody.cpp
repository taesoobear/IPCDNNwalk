#include <list>
#include <algorithm>
#include <string>
#include <sstream>
#include "gbody.h"
#include "gelement.h"
#include "gjoint.h"
#include "rmatrix3j.h"

using namespace std;


//=============================================================
//                 GBody
//=============================================================
GBody::GBody()
{
	T_global.SetIdentity();
	pBaseJoint = NULL;
	pParentBody = NULL;

	T.SetIdentity();
	invT.SetIdentity();

	S.SetZero(0,0);			// may vary
	dS.SetZero(0,0);		// may vary
	Sdq.SetZero();
	dSdq.SetZero();
	Sddq.SetZero();
	DSdqDt.SetZero();

	Jacobian.SetZero(0,0);	// may vary

	bDpAlien = false;
}

bool GBody::getReady()
{
	// check if the base joint of the body exists
	if ( pBaseJoint == NULL ) return false;
	S.SetZero(6, pBaseJoint->getDOF());
	dS.SetZero(6, pBaseJoint->getDOF());
	return true;
}

bool GBody::addJoint(GJoint *pJoint_)
{
	if ( pJoint_ == NULL ) return false;
	if ( find(pJoints.begin(), pJoints.end(), pJoint_) != pJoints.end() ) return false;

	pJoints.push_back(pJoint_);

	return true;
}

bool GBody::removeJoint(GJoint *pJoint_)
{
	if ( pJoint_ == NULL ) return false;
	if ( find(pJoints.begin(), pJoints.end(), pJoint_) == pJoints.end() ) return false;

	pJoints.remove(pJoint_);

	return true;
}

void GBody::removeAllJoints()
{
	pJoints.clear();
}

string GBody::getInfoStr()
{
	stringstream sstr;
	list<GJoint *>::iterator iter_pjoint;
	list<GBody *>::iterator iter_pbody;

	sstr << GElement::getInfoStr();
	sstr << "GBody:: " << endl;
	sstr << "    number of joints attached = " << int(pJoints.size()) << endl;
	sstr << "    joints attached = ("; for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) { sstr << (*iter_pjoint)->getName() << ", "; } sstr << ")" << endl;
	sstr << "    base joint = " << pBaseJoint->getName() << endl;
	sstr << "    parent body = " << pParentBody->getName() << endl;
	sstr << "    child bodies = ("; for (iter_pbody = pChildBodies.begin(); iter_pbody != pChildBodies.end(); iter_pbody++) { sstr << (*iter_pbody)->getName() << ", "; } sstr << ")" << endl;
	sstr << "    forward joint loop = ("; for (iter_pjoint = fJL.pJoints.begin(); iter_pjoint != fJL.pJoints.end(); iter_pjoint++) { sstr << (*iter_pjoint)->getName() << ", "; } sstr << ")" << endl;

	return sstr.str();
}

