
#include "stdafx.h"
#include "LimbIKinfo.h"
#include "IKSolver.h"
#include "MotionLoader.h"
void MotionUtil::LimbIKinfo::init(Bone const& knee, Bone* anklebone, double axis_sign)
{
	MotionUtil::LimbIKinfo& info=*this;

	//Msg::verify(anklebone->getRotationalChannels().length()==3, "rotc");
	if(anklebone==knee.child())
	{
		info.knee=&knee;
		ASSERT(knee.getRotationalChannels().length()==1);
		ASSERT(knee.parent()->getRotationalChannels().length()==3);
		info.hip=knee.parent();
		info.ankle=anklebone;
	}
	else if(anklebone==knee.child()->child())
	{
		info.knee=&knee;
		ASSERT(knee.getRotationalChannels().length()==1);
		ASSERT(knee.parent()->getRotationalChannels().length()==0);
		ASSERT(knee.parent()->parent()->getRotationalChannels().length()==3);

		info.hip=knee.parent()->parent();
		info.ankle=anklebone;
	}
	else	
	{
		printf("%s %s\n", anklebone->NameId, knee.NameId);
		Msg::error("hybridIK error 1");
	}

	switch(info.knee->getRotationalChannels()[0])
	{
		case 'X':
			info.axis=vector3(1,0,0);
			break;
		case 'Y':
			info.axis=vector3(0,1,0);
			break;
		case 'Z':
			info.axis=vector3(0,0,1);
			break;
	}
	info.axis*=axis_sign;
}

void MotionUtil::LimbIKinfo::limbIK(double importance, quater const& conori, double kneeDampingConstant, bool useKneeDamping)
{
	q1=qo1;
	q2=qo2;
	double len=MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, axis, useKneeDamping, NULL, kneeDampingConstant);

	qt.difference(qo1, q1);
	qt.scale(importance);
	q1=qt*qo1;

	qt.difference(qo2, q2);
	qt.scale(importance);
	q2=qt*qo2;

	quater cq0;
	hip->parent()->getRotation(cq0);
	hip->_getLocalFrame().rotation=cq0.inverse()*q1;
	knee->_getLocalFrame().rotation=q1.inverse()*q2;
	ankle->_getLocalFrame().rotation=q2.inverse()*conori;
}
void MotionUtil::LimbIKinfo::prepare(quater const& conori, vector3 const& localpos)
{
	hip->parent()->getRotation(q0);
	hip->getTranslation(sh);
	hip->getRotation(q1);
	knee->getTranslation(elb);
	knee->getRotation(q2);
	ankle->getTranslation(wrist);
	if(knee->parent()==hip)
		knee->getOffset(v1);
	else
		v1=q1.inverse()*(elb-sh);

	if(ankle->parent()==knee)
		ankle->getOffset(v2);
	else
		v2=q2.inverse()*(wrist-elb);

	if (ankle->child() && ankle->child()->rotJointIndex()!=-1)
	{
		ankle->getRotation(qo1);
		ankle->child()->getRotation(qo2);
		qt.difference(qo1, qo2);
		// preserve original global ankle orientation.
		ankle->_getFrame().rotation=conori;
		ankle->child()->_getFrame().rotation.mult(qt, conori);
	}
	else
	{
		// preserve original global ankle orientation.
		ankle->_getFrame().rotation=conori;
	}
	hand=ankle->getFrame().toGlobalPos(localpos);
	v3.difference(sh, elb);
	v4.difference(elb, wrist);
	//v4.difference(elb,wrist);

	qo1=q1;
	qo2=q2;

	leg_delta.difference(sh, hand);
}
