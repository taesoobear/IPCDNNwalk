												\
#include "stdafx.h"
#include "../math/mathclass.h"
#include "../math/optimize.h"
#include "FullbodyIK_MotionDOF.h"
#include "Motion.h"
#include "MotionUtil.h"
#include "MotionLoader.h"
#include "MotionDOF.h"
#include "IKSolver.h"
#include "VRMLloader.h"
#include "COM_IKsolver.h"

//#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../../MainLib/OgreFltk/RE.h"
static void drawSphere(const char* name, const char* mat, double scale, vector3 const& pos)
{
	Ogre::SceneNode* pNode=RE::createEntity(name,"sphere1010.mesh");
	RE::setMaterialName(pNode, mat);
	RE::moveEntity(pNode, vector3(scale, scale, scale),pos);
}
#endif
												using namespace MotionUtil;

	COM_IKsolver::  COM_IKsolver(VRMLloader const& skel, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)
:Optimize(), mSkeleton(skel),mDofInfo(skel.dofInfo), mEffectors(effectors)
	,cgmethod(10)
{
	init(0.01, 3, 0.01, 0.005, cgmethod);
	mHipBones.resize(mEffectors.size());
	mKneeBones.resize(mEffectors.size());
	mAnkleBones.resize(mEffectors.size());
	mAxis.setSize(mEffectors.size());

	currPose.Init(skel.numRotJoint(), skel.numTransJoint());
	for(int i=0; i<mEffectors.size(); i++)
	{

		Msg::verify(mEffectors[i].bone->getRotationalChannels().length()==3, "rotc");
		Bone const& knee=mSkeleton.bone(knee_bone_indexes[i]);
		if(mEffectors[i].bone==knee.child())
		{
			mKneeBones[i]=&knee;
			ASSERT(knee.getRotationalChannels().length()==1);
			ASSERT(knee.parent()->getRotationalChannels().length()==3);
			mHipBones[i]=knee.parent();
			mAnkleBones[i]=mEffectors[i].bone;
			mbAdjustLen=false;
		}
		else if(mEffectors[i].bone==knee.child()->child())
		{
			mKneeBones[i]=&knee;
			ASSERT(knee.getRotationalChannels().length()==1);
			ASSERT(knee.parent()->getRotationalChannels().length()==0);
			ASSERT(knee.parent()->parent()->getRotationalChannels().length()==3);

			mHipBones[i]=knee.parent()->parent();
			mAnkleBones[i]=mEffectors[i].bone;
			mbAdjustLen=true;

		}
		else	
			Msg::error("??");

		switch(mKneeBones[i]->getRotationalChannels()[0])
		{
			case 'X':
				mAxis[i]=vector3(1,0,0);
				break;
			case 'Y':
				mAxis[i]=vector3(0,1,0);
				break;
			case 'Z':
				mAxis[i]=vector3(0,0,1);
				break;
		}
		mAxis[i]*=axis_sign[i];
	}
}

static void rotateChildren(Bone& b, quater const& q)
{
	b._getFrame().rotation.leftMult(q);
	for(Bone* hc=b.child(); hc; hc=hc->sibling())
		rotateChildren(*hc,q);
}

void COM_IKsolver::setSkeleton(vectorn & temp)
{
	if(mbAdjustLen)
	{

		for(int c=0; c<mEffectors.size();c++)
		{
			// reset knee translations to 0.
			temp[mDofInfo.startT(mKneeBones[c]->child()->treeIndex())]=0;
			// reset hip translations to 0.
			temp[mDofInfo.startT(mHipBones[c]->child()->treeIndex())]=0;
		}
	}
	mSkeleton.setPose(mDofInfo.setDOF(temp));
}

void COM_IKsolver::_prepare(int c)
{
	mHipBones[c]->parent()->getRotation(q0);
	mHipBones[c]->getTranslation(sh);
	mHipBones[c]->getRotation(q1);
	mKneeBones[c]->getTranslation(elb);
	mKneeBones[c]->getRotation(q2);
	mAnkleBones[c]->getTranslation(wrist);
	mKneeBones[c]->getOffset(v1);
	mAnkleBones[c]->getOffset(v2);
	//			mAnkleBones[c]->getRotation(test);

	// preserve original global ankle orientation.
	mAnkleBones[c]->_getFrame().rotation=ankleGlobal[c];

	hand=mAnkleBones[c]->getFrame().toGlobalPos(mEffectors[c].localpos);

	v3.difference(sh, elb);
	v4.difference(elb, mAnkleBones[c]->getTranslation());
}

void COM_IKsolver::_ikSolve(int c)
{
	goal=con[c]-hand+wrist;

	const bool useKneeDamping=false;
#ifdef DEBUG_DRAW
	if(c==1)
	{
		double skinScale=100;
		drawSphere("m0","red", 1, sh*skinScale );
		drawSphere("m1","red", 1, elb*skinScale );
		drawSphere("m2","red", 1, wrist*skinScale );
		drawSphere("g2","green", 3, goal*skinScale );
		
		RE::output("m0", "%s", sh.output().ptr());
		RE::output("m1", "%s", elb.output().ptr());
		RE::output("m2", "%s", wrist.output().ptr());
		RE::output("q1", "%s %f", q1.output().ptr(), q1.length());
		RE::output("v1","%s %s", v1.output().ptr(), (q1.inverse()*(elb-sh)).output().ptr());
		RE::output("v2","%s %s", v2.output().ptr(), (q2.inverse()*(wrist-elb)).output().ptr());
		RE::output("import","%f", importance[c]);
	}
#endif
	quater qo1=q1;
	quater qo2=q2;
	if(mbAdjustLen)
		MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, mAxis[c], useKneeDamping,&r[c]);
	else
		MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, mAxis[c], useKneeDamping);

	if(importance[c]!=1.0)
	{
		quater qt;
		qt.difference(qo1, q1);
		qt.scale(importance[c]);
		q1=qt*qo1;

		qt.difference(qo2, q2);
		qt.scale(importance[c]);
		q2=qt*qo2;
	}
	// q0
	// q0l1=qq1
	// q0l1l2=qq2			
	// q0l1l2l3=qq3
	BoneForwardKinematics & fk=mSkeleton.fkSolver();

	fk._local(*mHipBones[c]).rotation=q0.inverse()*q1;
	fk._local(*mKneeBones[c]).rotation=q1.inverse()*q2;
	fk._local(*mAnkleBones[c]).rotation=q2.inverse()*ankleGlobal[c];

	if(mbAdjustLen) {
		fk._local(*mHipBones[c]->child()).translation=vector3(0,r[c]*-1 ,0);
		fk._local(*mKneeBones[c]->child()).translation=vector3(0,r[c]*-1 ,0);
	}

	mSkeleton.fkSolver().setChain(*mAnkleBones[c]);		

#ifdef DEBUG_DRAW
	if(c==1)
	{
		double skinScale=100;
		drawSphere("ma0","green", 1, mHipBones[c]->getTranslation()*skinScale );
		drawSphere("ma1","green", 1, mKneeBones[c]->getTranslation()*skinScale );
		drawSphere("ma2","green", 1, mAnkleBones[c]->getTranslation()*skinScale );
	}
#endif
}

void COM_IKsolver::_iksolve(vector3 const& newPelvisPos)
{
	temp.setVec3(0, newPelvisPos);
	setSkeleton(temp);

	quater deltaHip[4];
	for (int i=0; i<mEffectors.size(); i++)
		deltaHip[i].identity();

	//		quater test;
	r.setSize(mEffectors.size());
	for (int i=0; i<mEffectors.size(); i++)
	{
		_prepare(i);
		_ikSolve(i);
	}

	totalMass=0;
	com.setValue(0,0,0);
	vector3 bcom;
	for(int b=1; b<mSkeleton.numBone(); b++){
		VRMLTransform& bone=mSkeleton.VRMLbone(b);
		totalMass+=bone.mass();
		bcom=bone.getFrame().toGlobalPos(bone.localCOM())*bone.mass();
		com+=bcom;
	}
	com/=totalMass;
}

void COM_IKsolver::calcPelvisPos(vectorn& origRootTF_, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& _con, vectorn const& _importance,vector3 const& _desiredCOM, vector3& pelvisPos)
{
	con=_con;
	desiredCom=_desiredCOM;
	importance=_importance;
	transf origRootTF=MotionDOF::rootTransformation(origRootTF_);

	{
		quater rotY, offset;
		origRootTF.rotation.decompose(rotY, offset);
		origRootTF.rotation.mult(currRotY, offset);
	}

	temp.assign(origRootTF_);
	MotionDOF::setRootTransformation(temp, origRootTF);
	setSkeleton(temp);

	for(int c=0; c<mEffectors.size();c++)
		ankleGlobal[c]=delta_foot[c]*mAnkleBones[c]->getFrame().rotation;

	MotionDOF::setRootTransformation(temp, newRootTF);

	_iksolve(newRootTF.translation);


	vectorn initialPos(3);
	initialPos.setVec3(0,newRootTF.translation);

	optimize(initialPos);

	pelvisPos=getResult().toVector3(0);
	if( pelvisPos.x!=pelvisPos.x)
	{
		printf("?\n");
		pelvisPos=newRootTF.translation;
	}
}

void COM_IKsolver::IKsolve(vectorn& origRootTF_, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& _con, vectorn const& _importance, vector3 const& _desiredCOM)
{
	vector3 pelvisPos;
	calcPelvisPos(origRootTF_, currRotY, newRootTF, delta_foot, _con, _importance, _desiredCOM, pelvisPos);

	_iksolve(pelvisPos);

	//printf("pelvisPos %s\n", pelvisPos.output().ptr());
	mSkeleton.fkSolver().getPoseFromLocal(tempp);
	//printf("ankleHeight %s %s %s %f %f\n", pelvisPos.output().ptr(), tempp.m_aTranslations[0].output().ptr(), mAnkleBones[0]->NameId,_con[0].y, mAnkleBones[0]->getFrame().translation.x);
	//printf("pelvisPos2 %s\n", tempp.m_aTranslations[0].output().ptr());
	/*		mSkeleton.setPose(tempp);

			quater tt1,tt2,tt3;
			int c=mEffectors.size()-1;
			tt1=mHipBones[c]->_getFrame().rotation;
			tt2=mKneeBones[c]->_getFrame().rotation;
			tt3=mAnkleBones[c]->_getFrame().rotation;*/
	mDofInfo.getDOF(tempp, origRootTF_);

}

m_real COM_IKsolver::objectiveFunction(vectorn const& x)
{
	_iksolve(x.toVector3(0));

	double e=com.distance(desiredCom);

	return e;
}

void COM_IKsolver::IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con)
{
	Msg::msgBox("not implemented yet!");
	transf origRootTF=MotionDOF::rootTransformation(temp);

	{
		quater rotY, offset;
		origRootTF.rotation.decompose(rotY, offset);
		origRootTF.rotation.mult(currRotY, offset);
	}

	MotionDOF::setRootTransformation(temp, origRootTF);
	setSkeleton(temp);

	quater  ankleGlobal[4];
	for(int c=0; c<mEffectors.size();c++)
		ankleGlobal[c]=delta_foot[c]*mAnkleBones[c]->getFrame().rotation;

	MotionDOF::setRootTransformation(temp, newRootTF);

	setSkeleton(temp);

	quater deltaHip[4];
	for(int c=0; c<mEffectors.size();c++)
		deltaHip[c].identity();

	//		quater test;
	vector3 goal, sh, elb, v1, v2, v3, v4, wrist,hand;
	quater q1, q2;

	vectorn r(mEffectors.size());

	for(int c=0; c<mEffectors.size();c++)
	{
		mHipBones[c]->getTranslation(sh);
		mHipBones[c]->getRotation(q1);
		mKneeBones[c]->getTranslation(elb);
		mKneeBones[c]->getRotation(q2);
		mKneeBones[c]->getOffset(v1);
		mAnkleBones[c]->getOffset(v2);
		mAnkleBones[c]->getTranslation(wrist);
		//			mAnkleBones[c]->getRotation(test);

		// preserve original global ankle orientation.
		mAnkleBones[c]->_getFrame().rotation=ankleGlobal[c];

		hand=mAnkleBones[c]->getFrame().toGlobalPos(mEffectors[c].localpos);
		goal=con[c]-hand+wrist;

		v3.difference(sh, elb);
		v4.difference(elb, mAnkleBones[c]->getTranslation());

		const bool useKneeDamping=true;

		if(mbAdjustLen)
			MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, mAxis[c], useKneeDamping,&r[c]);
		else
			MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, mAxis[c], useKneeDamping);


		/*
		   if(useKneeDamping)
		   IKSolver::limbIK(goal, sh, v1, v2, q1, q2, 1.0, mAxis[c], &v3, &v4);
		   else
		   IKSolver::limbIK(goal, sh, v1, v2, q1, q2, 1.0, mAxis[c]);
		   */


		///mHipBones[c]->_getFrame().rotation=q1;
		///mKneeBones[c]->_getFrame().rotation=q2;
		//if(mbAdjustLen)
		//{
		//	// update global orientations of translation joints too,
		//	// so that getPoseFromGlobal(...) works.
		//	mHipBones[c]->child()->_getFrame().rotation=q1;
		//	mKneeBones[c]->child()->_getFrame().rotation=q2;
		//}
		mHipBones[c]->_getLocalFrame().rotation=q0.inverse()*q1;
		mKneeBones[c]->_getLocalFrame().rotation=q1.inverse()*q2;
		mAnkleBones[c]->_getLocalFrame().rotation=q2.inverse()*ankleGlobal[c];
		if(mbAdjustLen)
			mSkeleton.fkSolver().setChain(*mAnkleBones[c]->child());
		else
			mSkeleton.fkSolver().setChain(*mAnkleBones[c]);

	}
	mSkeleton.getPose(tempp);
	/*		mSkeleton.setPose(tempp);

			quater tt1,tt2,tt3;
			int c=mEffectors.size()-1;
			tt1=mHipBones[c]->_getFrame().rotation;
			tt2=mKneeBones[c]->_getFrame().rotation;
			tt3=mAnkleBones[c]->_getFrame().rotation;*/
	mDofInfo.getDOF(tempp, temp);

	if(mbAdjustLen)
	{
		for(int c=0;c<mEffectors.size();c++)
		{
			temp[mDofInfo.startT(mHipBones[c]->child()->treeIndex())]=r[c]*-1;
			temp[mDofInfo.startT(mKneeBones[c]->child()->treeIndex())]=r[c]*-1;
		}
	}
}


#ifdef USE_LUABIND
#include "../MainLib/WrapperLua/BaselibLUA.h"
#include "../MainLib/WrapperLua/MainlibLUA.h"
#include <luabind/luabind.hpp>
#include <luabind/operator.hpp>
#include <luabind/out_value_policy.hpp>
#include <luabind/adopt_policy.hpp>
#include <luabind/error.hpp>

using namespace luabind;

void registerCOM_IKsolver(lua_State* L)
{
	luabind::module(L) 
		[
		class_<COM_IKsolver>("COM_IKsolver")
		.def(constructor<VRMLloader const&, std::vector<Effector>&, intvectorn const&, vectorn const&>() )
		.def("IKsolve", &COM_IKsolver::IKsolve)
		.def("IKsolve2", &COM_IKsolver::IKsolve2)
		.def("calcPelvisPos", &COM_IKsolver::calcPelvisPos)
		];
}
#endif
