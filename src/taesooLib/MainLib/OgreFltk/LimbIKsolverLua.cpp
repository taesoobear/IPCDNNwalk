#include "stdafx.h"
#include "../math/mathclass.h"
#include "../math/optimize.h"
//#include "../MainLib/gsl_addon/gsl_wrap.h"
#include "../BaseLib/motion/FullbodyIK_MotionDOF.h"
#include "../BaseLib/motion/Motion.h"
#include "../BaseLib/motion/MotionUtil.h"
#include "../BaseLib/motion/MotionLoader.h"
#include "../BaseLib/motion/MotionDOF.h"
#include "../BaseLib/motion/IKSolver.h"
#include "../../MainLib/OgreFltk/RE.h"

#include "../MainLib/WrapperLua/luna.h"
#include "../MainLib/WrapperLua/LUAwrapper.h"
#include "../MainLib/WrapperLua/luna_baselib.h"
#include "LimbIKsolverLua.h"
using namespace MotionUtil;

//#define DEBUG_DRAW
LimbIKsolverLua::LimbIKsolverLua(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& hip_bone_indexes, intvectorn const& knee_bone_indexes, vectorn const& axis_sign, lua_State* l)
		:Optimize(),FullbodyIK_MotionDOF3(dofInfo,effectors)
		,cgmethod(10)
		 ,L(l)
	{
		init(0.01, 3, 0.01, 0.005, cgmethod);
		Msg::verify(mEffectors.size()<=4, "too many effectors");
		for(int i=0; i<mEffectors.size(); i++)
		{
			LimbIKinfo &info=_limbikInfo[i];
			//Msg::verify(mEffectors[i].bone->getRotationalChannels().length()==3, "rotc");
			Bone const& hip=mSkeleton.bone(hip_bone_indexes[i]);
			Bone const& knee=mSkeleton.bone(knee_bone_indexes[i]);
			info.hip=&hip;
			info.knee=&knee;
			info.ankle=mEffectors[i].bone;

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
			info.axis*=axis_sign[i];
		}

		if(mEffectors.size()==1)
			mPelvis.resize(1);
		else
			mPelvis.resize(mEffectors.size()/2);
		for (int c=0; c<mEffectors.size(); c+=2)
		{
			LimbIKinfo &info=_limbikInfo[c];
			Bone* hip=info.hip->parent();
			{
				Bone* hip1;
				if(mEffectors.size()==1)
				{
					if (strcmp(hip->NameId,"LeftCollar")==0)
					{
						hip1=&mSkeleton.getBoneByName("RightCollar");
					}
					else if (strcmp(hip->NameId,"RightCollar")==0)
					{
						hip1=&mSkeleton.getBoneByName("LeftCollar");
					}
					else
						Msg::error("size error!");
				}
				else
					hip1=_limbikInfo[c+1].hip->parent();
				while(hip!=hip1 || hip->rotJointIndex()==-1)
				{
					Msg::verify(hip && hip1, "parent error!");
					hip=hip->parent();
					hip1=hip1->parent();
				}
			}
			mPelvis[c/2]=hip;
		}
	}
//knee_bone_indexes :  lknee , rknee
LimbIKsolverLua::LimbIKsolverLua(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign, lua_State* l)
		:Optimize(),FullbodyIK_MotionDOF3(dofInfo,effectors)
		,cgmethod(10)
		 ,L(l)
	{
		init(0.01, 3, 0.01, 0.005, cgmethod);
		Msg::verify(mEffectors.size()<=4, "too many effectors");
		for(int i=0; i<mEffectors.size(); i++)
		{
			LimbIKinfo &info=_limbikInfo[i];
			//Msg::verify(mEffectors[i].bone->getRotationalChannels().length()==3, "rotc");
			Bone const& knee=mSkeleton.bone(knee_bone_indexes[i]);
			if(mEffectors[i].bone==knee.child())
			{
				info.knee=&knee;
				ASSERT(knee.getRotationalChannels().length()==1);
				ASSERT(knee.parent()->getRotationalChannels().length()==3);
				info.hip=knee.parent();
				info.ankle=mEffectors[i].bone;
			}
			else if(mEffectors[i].bone==knee.child()->child())
			{
				info.knee=&knee;
				ASSERT(knee.getRotationalChannels().length()==1);
				ASSERT(knee.parent()->getRotationalChannels().length()==0);
				ASSERT(knee.parent()->parent()->getRotationalChannels().length()==3);

				info.hip=knee.parent()->parent();
				info.ankle=mEffectors[i].bone;
			}
			else	
			{
				printf("%s %s\n", mEffectors[i].bone->NameId, knee.NameId);
				Msg::error("LimbIKsolverLua error 1");
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
			info.axis*=axis_sign[i];
		}

		if(mEffectors.size()==1)
			mPelvis.resize(1);
		else
			mPelvis.resize(mEffectors.size()/2);
		for (int c=0; c<mEffectors.size(); c+=2)
		{
			LimbIKinfo &info=_limbikInfo[c];
			Bone* hip=info.hip->parent();
			{
				Bone* hip1;
				if(mEffectors.size()==1)
				{
					if (strcmp(hip->NameId,"LeftCollar")==0)
					{
						hip1=&mSkeleton.getBoneByName("RightCollar");
					}
					else if (strcmp(hip->NameId,"RightCollar")==0)
					{
						hip1=&mSkeleton.getBoneByName("LeftCollar");
					}
					else
						Msg::error("size error!");
				}
				else
					hip1=_limbikInfo[c+1].hip->parent();
				while(hip!=hip1)
				{
					Msg::verify(hip && hip1, "parent error!");
					hip=hip->parent();
					hip1=hip1->parent();
				}
			}
			mPelvis[c/2]=hip;
		}
	}


	static void rotateChildren(Bone& b, quater const& q)
	{
		b._getFrame().rotation.leftMult(q);
		for(Bone* hc=b.child(); hc; hc=hc->sibling())
			rotateChildren(*hc,q);
	}

	void LimbIKsolverLua::setSkeleton(vectorn & temp)
	{
		mSkeleton.setPose(mDofInfo.setDOF(temp));
	}

	void LimbIKsolverLua::_forwardKinematics(int c, quater const& theta)
	{
		// forward-kinematics
		int rotJointIndex=mPelvis[c/2]->rotJointIndex();
		if (rotJointIndex==0)
		{
			mPelvis[c/2]->_getFrame().rotation=mRootOri[c/2]*theta;
			mSkeleton.getPose(tempp);
			mSkeleton.setPose(tempp);
		}
		else
		{
			mSkeleton.fkSolver().getPoseFromLocal(tempp);
			// q_global=q0*q1*...*qn
			// q_local=(q0*q1*...*q_(n-1)).inverse()*q_global
			// q_local=parent.q_global.inverse()*q_global
			tempp.m_aRotations[rotJointIndex]=
				mPelvis[c/2]->parent()->getRotation().inverse()*(mRootOri[c/2]*theta);
			mSkeleton.setPose(tempp);
		}
	}
	
	m_real LimbIKsolverLua::objectiveFunction(vectorn const& x)
	{
		lunaStack l(this->L);
		l.getglobal("_objectiveFunction");
		l.push<LimbIKsolverLua>(this);
		l.push<vectorn>(x);
		l.call(2,1);
		double out;
		l>> out;
		return out;
	}

//sh : hip position
//q1 : hip angle
//elb : knee position
//q2 : knee angle
//v1 : hip-knee  nat
//v3 : hip-knee  cur
//wrist : ankle position
//v2 : knee-ankle nat
//v4 : knee-ankle cur
//goal : ankle
//

	void LimbIKsolverLua::_limbIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance)
	{
		for(int c=0; c<mEffectors.size();c++)
		{
			LimbIKinfo &info=_limbikInfo[c];
			vector3 csh;
			info.hip->getTranslation(csh);
			info.goal=conpos[c]-info.hand+info.wrist-csh+info.sh;
			_limbikInfo[c].limbIK(importance[c], conori[c]);
		}
		mSkeleton.fkSolver().forwardKinematics();
	}
	void LimbIKsolverLua::IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& _conori, vectorn const& importance)
	{
		MotionDOF::setRootTransformation(poseInout, newRootTF);
		setSkeleton(poseInout);
		mRootPos.resize(mEffectors.size()/2);
		mRootOri.resize(mEffectors.size()/2);
		for (int c=0; c<mEffectors.size(); c++)
		{
			LimbIKinfo &info=_limbikInfo[c];
			info.hipoffset=mPelvis[c/2]->getFrame().toLocalPos(info.hip->getTranslation());
		}

		for (int c=0; c<mEffectors.size(); c+=2)
		{
			mPelvis[c/2]->getTranslation(mRootPos[c/2]);
			//mPelvis[c/2]->getRotation(mRootOri[c/2]);
			mRootOri[c/2]=mPelvis[c/2]->_getLocalFrame().rotation;
#ifdef DEBUG_DRAW
			RE::moveEntity(RE::createEntity("qRoot", "axes.mesh"), vector3(3,3,3), mRootOri[c/2], vector3(0,-10,0)+ mPelvis[c/2]->getTranslation()*100);	
#endif
		}
		con=conpos;
		conori=_conori;
		impor=importance;


		for(int c=0; c<mEffectors.size();c++)
		{
			LimbIKinfo &info=_limbikInfo[c];
			info.prepare(conori[c]);
	
			info.hand=info.ankle->getFrame().toGlobalPos(mEffectors[c].localpos);
	//goal=conpos[c];
			info.goal=conpos[c]-info.hand+info.wrist;
		}
			
		lunaStack l(this->L);
		l.getglobal("_IKsolve");
		l.push<LimbIKsolverLua>(this);
		l.push<vectorn>(poseInout);
		l.push<transf>(newRootTF);
		l.push<vector3N>(conpos);
		l.push<quaterN>(conori);
		l.push<vectorn>(importance);
		l.call(6,0);
	}

	void LimbIKsolverLua::IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con)
	{
		Msg::error("not implmented");
	}

  void LimbIKsolverLua::IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con)
	{
		Msg::error("not implemented");
	}

	void LimbIKsolverLua::init_cg(int ndim, double grad_step, int max_iter, double tol, double thr)
	{
		cgmethod.max_iter=max_iter;
		cgmethod.tol=tol;
		cgmethod.thr=thr;
		init(0.01, ndim, 1, grad_step, cgmethod);
	}

void LimbIKsolverLua::LimbIKinfo::limbIK(double importance, quater const& conori)
{
	q1=qo1;
	q2=qo2;
	const bool useKneeDamping=true;
	MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, axis, useKneeDamping);

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
void LimbIKsolverLua::LimbIKinfo::prepare(quater const& conori)
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
	v3.difference(sh, elb);
	v4.difference(elb, wrist);
	//v4.difference(elb,wrist);

	qo1=q1;
	qo2=q2;
}


	LimbIKsolverHybrid::LimbIKsolverHybrid(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& hip_bone_indexes, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)
	:Optimize(),FullbodyIK_MotionDOF3(dofInfo,effectors)
	 ,cgmethod(10)
{
	init(0.01, 3, 0.01, 0.005, cgmethod);
	Msg::verify(mEffectors.size()<=4, "too many effectors");
	for(int i=0; i<mEffectors.size(); i++)
	{
		LimbIKinfo &info=_limbikInfo[i];
		//Msg::verify(mEffectors[i].bone->getRotationalChannels().length()==3, "rotc");
		Bone const& hip=mSkeleton.bone(hip_bone_indexes[i]);
		Bone const& knee=mSkeleton.bone(knee_bone_indexes[i]);
		info.hip=&hip;
		info.knee=&knee;
		info.ankle=mEffectors[i].bone;

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
		info.axis*=axis_sign[i];
	}

	if(mEffectors.size()==1)
		mPelvis.resize(1);
	else
		mPelvis.resize(mEffectors.size()/2);
	for (int c=0; c<mEffectors.size(); c+=2)
	{
		LimbIKinfo &info=_limbikInfo[c];
		Bone* hip=info.hip->parent();
		{
			Bone* hip1;
			if(mEffectors.size()==1)
			{
				if (strcmp(hip->NameId,"LeftCollar")==0)
				{
					hip1=&mSkeleton.getBoneByName("RightCollar");
				}
				else if (strcmp(hip->NameId,"RightCollar")==0)
				{
					hip1=&mSkeleton.getBoneByName("LeftCollar");
				}
				else
					Msg::error("size error!");
			}
			else
				hip1=_limbikInfo[c+1].hip->parent();
			while(hip!=hip1 || hip->rotJointIndex()==-1)
			{
				Msg::verify(hip && hip1, "parent error!");
				hip=hip->parent();
				hip1=hip1->parent();
			}
		}
		mPelvis[c/2]=hip;
	}
}
//knee_bone_indexes :  lknee , rknee
	LimbIKsolverHybrid::LimbIKsolverHybrid(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)
	:Optimize(),FullbodyIK_MotionDOF3(dofInfo,effectors)
	 ,cgmethod(10)
{
	init(0.01, 3, 0.01, 0.005, cgmethod);
	Msg::verify(mEffectors.size()<=4, "too many effectors");
	for(int i=0; i<mEffectors.size(); i++)
	{
		LimbIKinfo &info=_limbikInfo[i];
		//Msg::verify(mEffectors[i].bone->getRotationalChannels().length()==3, "rotc");
		Bone const& knee=mSkeleton.bone(knee_bone_indexes[i]);
		if(mEffectors[i].bone==knee.child())
		{
			info.knee=&knee;
			ASSERT(knee.getRotationalChannels().length()==1);
			ASSERT(knee.parent()->getRotationalChannels().length()==3);
			info.hip=knee.parent();
			info.ankle=mEffectors[i].bone;
		}
		else if(mEffectors[i].bone==knee.child()->child())
		{
			info.knee=&knee;
			ASSERT(knee.getRotationalChannels().length()==1);
			ASSERT(knee.parent()->getRotationalChannels().length()==0);
			ASSERT(knee.parent()->parent()->getRotationalChannels().length()==3);

			info.hip=knee.parent()->parent();
			info.ankle=mEffectors[i].bone;
		}
		else	
		{
			printf("%s %s\n", mEffectors[i].bone->NameId, knee.NameId);
			Msg::error("LimbIKsolverHybrid error 1");
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
		info.axis*=axis_sign[i];
	}

	if(mEffectors.size()==1)
		mPelvis.resize(1);
	else
		mPelvis.resize(mEffectors.size()/2);
	for (int c=0; c<mEffectors.size(); c+=2)
	{
		LimbIKinfo &info=_limbikInfo[c];
		Bone* hip=info.hip->parent();
		{
			Bone* hip1;
			if(mEffectors.size()==1)
			{
				if (strcmp(hip->NameId,"LeftCollar")==0)
				{
					hip1=&mSkeleton.getBoneByName("RightCollar");
				}
				else if (strcmp(hip->NameId,"RightCollar")==0)
				{
					hip1=&mSkeleton.getBoneByName("LeftCollar");
				}
				else
					Msg::error("size error!");
			}
			else
				hip1=_limbikInfo[c+1].hip->parent();
			while(hip!=hip1)
			{
				Msg::verify(hip && hip1, "parent error!");
				hip=hip->parent();
				hip1=hip1->parent();
			}
		}
		mPelvis[c/2]=hip;
	}
}



void LimbIKsolverHybrid::setSkeleton(vectorn & temp)
{
	mSkeleton.setPose(mDofInfo.setDOF(temp));
}

void LimbIKsolverHybrid::_forwardKinematics(int c, quater const& theta)
{
	// forward-kinematics
	int rotJointIndex=mPelvis[c/2]->rotJointIndex();
	if (rotJointIndex==0)
	{
		mPelvis[c/2]->_getFrame().rotation=mRootOri[c/2]*theta;
		mSkeleton.getPose(tempp);
		mSkeleton.setPose(tempp);
	}
	else
	{
		mSkeleton.fkSolver().getPoseFromLocal(tempp);
		// q_global=q0*q1*...*qn
		// q_local=(q0*q1*...*q_(n-1)).inverse()*q_global
		// q_local=parent.q_global.inverse()*q_global
		tempp.m_aRotations[rotJointIndex]=
			mPelvis[c/2]->parent()->getRotation().inverse()*(mRootOri[c/2]*theta);
		mSkeleton.setPose(tempp);
	}
}

m_real LimbIKsolverHybrid::objectiveFunction(vectorn const& x)
{
	std::vector<Effector>& eff=mEffectors;
	Bone* pelvis=getCenterBone(0);
	pelvis->_getLocalFrame().translation.add(mRootPos(0),x.toVector3(0));

	quater theta;
	theta.setRotation(x.toVector3(3));
	pelvis->_getLocalFrame().rotation.mult(mRootOri(0),theta);
	mSkeleton.fkSolver().forwardKinematics();
//
//
//	for(int c=0; c<mEffectors.size();c++)
//	{
//		LimbIKinfo &info=_limbikInfo[c];
//		info.prepare(conori[c]);
//
//		info.hand=info.ankle->getFrame().toGlobalPos(mEffectors[c].localpos);
//		//goal=conpos[c];
//		info.goal=con(c)-info.hand+info.wrist;
//	}
	_limbIK(con, conori, impor);

	double d=0;
	for(int c=0; c< mEffectors.size(); c++)
	{
		MotionUtil::Effector& eff=mEffectors[c];
		vector3 cpos=eff.bone->getFrame().toGlobalPos(eff.localpos);
		d=d+cpos.squaredDistance(con(c));


#ifdef DEBUG_DRAW
		TString temp,temp2;
		temp.format("ee%d", c);
		temp2.format("eee%d", c);
		RE::moveEntity(RE::createEntity(temp, "sphre1010.mesh"), vector3(13,13,3),cpos*100 );	
		RE::moveEntity(RE::createEntity(temp2, "sphre1010.mesh"), vector3(13,3,13),con(c)*100 );	
		RE::output(temp.ptr(), "%s",cpos.output().ptr());
		RE::output(temp2.ptr(), "%s",con(c).output().ptr());
#endif
	}

	for(int c=0; c< mOtherEffectors.size(); c++)
	{
		MotionUtil::Effector& eff=mOtherEffectors[c];
		vector3 cpos=eff.bone->getFrame().toGlobalPos(eff.localpos);
		d=d+cpos.squaredDistance(mOtherConPos(c))*mOtherImportance[c];


#ifdef DEBUG_DRAW
		TString temp,temp2;
		temp.format("oe%d", c);
		temp2.format("oee%d", c);
		RE::moveEntity(RE::createEntity(temp, "sphre1010.mesh"), vector3(13,13,3),cpos*100 );	
		RE::moveEntity(RE::createEntity(temp2, "sphre1010.mesh"), vector3(13,3,13),mOtherConPos(c)*100 );	
		RE::output(temp.ptr(), "%s",cpos.output().ptr());
		RE::output(temp2.ptr(), "%s",mOtherConPos(c).output().ptr());
#endif
	}
	//- skin scale 이 100인 경우에 적합하게 튜닝되어 있음.
	//- 
	double skinScale=100;
	double w=skinScale/100;
	double l=x.length();
	d=d*w*w+0.1*l*l;
	return d;
}

//sh : hip position
//q1 : hip angle
//elb : knee position
//q2 : knee angle
//v1 : hip-knee  nat
//v3 : hip-knee  cur
//wrist : ankle position
//v2 : knee-ankle nat
//v4 : knee-ankle cur
//goal : ankle
//

void LimbIKsolverHybrid::_limbIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance)
{
	for(int c=0; c<mEffectors.size();c++)
	{
		LimbIKinfo &info=_limbikInfo[c];
		vector3 csh;
		info.hip->getTranslation(csh);
		info.goal=conpos[c]-info.hand+info.wrist-csh+info.sh;
		_limbikInfo[c].limbIK(importance[c], conori[c]);
	}
	mSkeleton.fkSolver().forwardKinematics();
}
void LimbIKsolverHybrid::IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& _conori, vectorn const& importance)
{
	MotionDOF::setRootTransformation(poseInout, newRootTF);
	setSkeleton(poseInout);
	mRootPos.resize(mEffectors.size()/2);
	mRootOri.resize(mEffectors.size()/2);
	for (int c=0; c<mEffectors.size(); c++)
	{
		LimbIKinfo &info=_limbikInfo[c];
		info.hipoffset=mPelvis[c/2]->getFrame().toLocalPos(info.hip->getTranslation());
	}

	for (int c=0; c<mEffectors.size(); c+=2)
	{
		mPelvis[c/2]->getTranslation(mRootPos[c/2]);
		//mPelvis[c/2]->getRotation(mRootOri[c/2]);
		mRootOri[c/2]=mPelvis[c/2]->_getLocalFrame().rotation;
#ifdef DEBUG_DRAW
		RE::moveEntity(RE::createEntity("qRoot", "axes.mesh"), vector3(3,3,3), mRootOri[c/2], vector3(0,-10,0)+ mPelvis[c/2]->getTranslation()*100);	
#endif
	}
	con=conpos;
	conori=_conori;
	impor=importance;


	for(int c=0; c<mEffectors.size();c++)
	{
		LimbIKinfo &info=_limbikInfo[c];
		info.prepare(conori[c]);

		info.hand=info.ankle->getFrame().toGlobalPos(mEffectors[c].localpos);
		//goal=conpos[c];
		info.goal=conpos[c]-info.hand+info.wrist;
	}

	int dim=6;
	int max_iter=10;
	double tol=0.001; //-- used in frprmn termination condition
	double thr=1.0;/// -- used in NR_brek
	init_cg(dim, 0.005, max_iter, tol, thr);
	vectorn v(dim);
	v.setAllValue(0);
	optimize(v);
	vectorn& out=getResult();
	objectiveFunction( out);
	mSkeleton.getPoseDOF(poseInout);
}

void LimbIKsolverHybrid::IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con)
{
	Msg::error("not implmented");
}

void LimbIKsolverHybrid::IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con)
{
	Msg::error("not implemented");
}

void LimbIKsolverHybrid::init_cg(int ndim, double grad_step, int max_iter, double tol, double thr)
{
	cgmethod.max_iter=max_iter;
	cgmethod.tol=tol;
	cgmethod.thr=thr;
	init(0.01, ndim, 1, grad_step, cgmethod);
}

void LimbIKsolverHybrid::LimbIKinfo::limbIK(double importance, quater const& conori)
{
	q1=qo1;
	q2=qo2;
	const bool useKneeDamping=true;
	MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, axis, useKneeDamping);

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
void LimbIKsolverHybrid::LimbIKinfo::prepare(quater const& conori)
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
	v3.difference(sh, elb);
	v4.difference(elb, wrist);
	//v4.difference(elb,wrist);

	qo1=q1;
	qo2=q2;
}
