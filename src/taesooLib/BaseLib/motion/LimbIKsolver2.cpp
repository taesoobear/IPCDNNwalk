#include "stdafx.h"
#include "../math/mathclass.h"
#include "../math/optimize.h"
//#include "../MainLib/gsl_addon/gsl_wrap.h"
#include "../motion/FullbodyIK_MotionDOF.h"
#include "../motion/Motion.h"
#include "../motion/MotionUtil.h"
#include "../motion/MotionLoader.h"
#include "../motion/MotionDOF.h"
#include "../motion/IKSolver.h"
#include "../../MainLib/OgreFltk/RE.h"

#include "LimbIKsolver2.h"
using namespace MotionUtil;

double kneeDampingConstant=0.5;
#define ROTATION_SCALE_DOWN 1
//#define DEBUG_DRAW
//knee_bone_indexes :  lknee , rknee
LimbIKsolver2::LimbIKsolver2(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)
		:Optimize(),FullbodyIK_MotionDOF3(dofInfo,effectors)
		,cgmethod(10)
	{
		setOption(0);
		setValue(1,5,0.3, 5);
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
				Msg::error("LimbIKsolver2 error 1");
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

	void LimbIKsolver2::setSkeleton(vectorn & temp)
	{
		mSkeleton.setPose(mDofInfo.setDOF(temp));
	}

	m_real LimbIKsolver2::_prepare(int c)
	{
		LimbIKinfo &info=_limbikInfo[c];
		info.prepare(conori(c), mEffectors[c].localpos);
		return 0.0;
	}
	
	
	//root를 기준으로 골반을 회전한다.
	m_real LimbIKsolver2::_calcRootRotation()
	{
		_updateRootInfo();

		flagR=1;
		flagT=0;
		for(int c=0; c<mEffectors.size();c++)
			_prepare(c);	

		for(int c=0; c<mEffectors.size(); c+=2)
		{
			flagC=c/2;
			// 팔다리 둘다 지정한 경우, 팔 최적화, 다리 최적화 두번 진행
			init(0.01, 3, 0.01, 0.005, cgmethod);
			vectorn initial(3);
			initial.setVec3(0,vector3(0,0,0));
			optimize(initial);

#ifdef DEBUG_DRAW
			RE::moveEntity(RE::createEntity("theta", "axes.mesh"), vector3(3,3,3), theta, vector3(0,-10,0)+ mPelvis[0]->getTranslation()*100);	
#endif
			vector3 v=getResult().toVector3(0);
			theta.setRotation(v*ROTATION_SCALE_DOWN );

			_forwardKinematics(c, theta);
		}
		return 0.0;	
	}
	void LimbIKsolver2::_forwardKinematics(int c, quater const& theta)
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
	
	
	//root를 이동시킨다.
	m_real LimbIKsolver2::_calcRootTranslation()
	{
		flagR=0;
		flagT=1;
		for(int c=0; c<mEffectors.size();c++)
			_prepare(c);	
		mPelvis[0]->getTranslation(mRootPos[0]);
		mPelvis[0]->getRotation(mRootOri[0]);

		init(0.01, 3, 0.01, 0.005, cgmethod);
		vectorn initial(3);
		initial.setVec3(0,vector3(0,0,0));
		optimize(initial);
		
		quater iden;
#ifdef DEBUG_DRAW
		RE::moveEntity(RE::createEntity("theta", "axes.mesh"), vector3(3,3,3), theta, vector3(0,-10,0)+ mPelvis[0]->getTranslation()*100);	
#endif
		iden.setRotation(vector3(0,0,0));
		mPelvis[0]->_getFrame().translation=mRootPos[0];
		mPelvis[0]->_getFrame().rotation=mRootOri[0];
		vector3 setTranslation;
		setTranslation=vector3(0,getResult().toVector3(0).y,0);
		//setTranslation.setVec3(0,vector3(0,getResult().toVector3(0).y,0));
	//	setTranslation=get.setVec3(0,vector3(0,getResult().toVector3(0).y,0));
			
		//mPelvis[0]->_getFrame()=transf(iden,setTranslation)*mPelvis[0]->getFrame();
		mPelvis[0]->_getFrame()=transf(iden,getResult().toVector3(0))*mPelvis[0]->getFrame();

#ifdef DEBUG_DRAW
		RE::moveEntity(RE::createEntity("theta", "axes.mesh"), vector3(3,3,3), theta, vector3(0,-10,0)+ mPelvis[1]->getTranslation()*100);	
#endif
		mSkeleton.getPose(tempp);
		mSkeleton.setPose(tempp);
		return 0.0;
	}
	

	m_real LimbIKsolver2::objectiveFunction(vectorn const& x)
	{
		//Todo :root의 회전에 임의의(x) 방향들을 곱하여 에러율이 가장 작은 값을 찾는다.
		//에러율은 양발목과 양타겟의 거리를 제곱하여 계산한다.
		//
		//root의 방향: x 
		//에러율 : e
		double _gap[4];
		int numCon=mEffectors.size();
		vector3N hips;
		hips.resize(numCon);
		vector3 OptiRoot;	

		if(flagR)
		{
			theta.setRotation(ROTATION_SCALE_DOWN*x.toVector3(0));
			mPelvis[flagC]->_getFrame().translation=mRootPos[flagC];
			mPelvis[flagC]->_getFrame().rotation=mRootOri[flagC];
			mPelvis[flagC]->_getFrame()=mPelvis[flagC]->getFrame()*transf(theta,vector3(0,0,0));
			for (int c=0; c<mEffectors.size(); c++)
			{
				hips(c)=mPelvis[c/2]->getFrame().toGlobalPos(_limbikInfo[c].hipoffset);
			}
		}
		if(flagT)
		{
		
			for (int c=0; c<mEffectors.size(); c++)
			{
				hips(c)=mPelvis[c/2]->getFrame().toGlobalPos(_limbikInfo[c].hipoffset)+x.toVector3(0);
			}
			OptiRoot=mRootPos[0]+x.toVector3(0);
			//OptiRoot=mRootPos[0];
			//		OptiRoot.y=OptiRoot.y+x.toVector3(0).y;
		}
#ifdef DEBUG_DRAW
		RE::moveEntity(RE::createEntity("hips0", "sphere1010.mesh"), vector3(5,5,5), hips(0)*100);
		RE::moveEntity(RE::createEntity("hips1", "sphere1010.mesh"), vector3(5,5,5), hips(1)*100);
	
		RE::moveEntity(RE::createEntity("hips2", "sphere1010.mesh"), vector3(5,5,5), hips(2)*100);
		RE::moveEntity(RE::createEntity("hips3", "sphere1010.mesh"), vector3(5,5,5), hips(3)*100);
#endif
		for(int c=0; c<mEffectors.size();c++)
		{
			LimbIKinfo &info=_limbikInfo[c];
			double current_leg_len=info.leg_delta.length();
			double target_leg_len=hips(c).distance(con[c]);

			if(flagR)
				_gap[c]=ABS(target_leg_len-current_leg_len);
			else
				_gap[c]=MAX(target_leg_len-current_leg_len,0);
		}
		double e;
		
#ifdef DEBUG_DRAW
		RE::output("_gap[0]", "%f", _gap[0]);
		RE::output("_gap[1]", "%f", _gap[1]);
		RE::output("_gap[2]", "%f", _gap[2]);
		RE::output("_gap[3]", "%f", _gap[3]);
#endif
		if(flagT){
		//	printf("OptiRoot: %f,%f,%f ,scale : %f,%f, %f\n", OptiRoot.x, OptiRoot.y, OptiRoot.z);
	//		OrRoot[0].y=OrRoot[0].y*0.05;
		//OrRoot[0].x=OrRoot[0].x*1.2;
		//	OrRoot[0].z=OrRoot[0].z;
	//		OptiRoot.y=OptiRoot.y*0.05;
		//	OptiRoot.x=OptiRoot.x*1.2;
		//	OptiRoot.z=OptiRoot.z;
//RE::moveEntity(RE::createEntity("hips0", "sphere1010.mesh"), vector3(6,6,6), OrRoot[0]*100);

			double ee=0;
			for(int c=0; c<mEffectors.size();c++)
				ee+=valL*_gap[c]*_gap[c]*impor[c];
		   	e=ee+valN*(OrRoot[0]-OptiRoot).length()*(OrRoot[0]-OptiRoot).length();
		   	//e=valL*(_gap[0]*impor[0]+_gap[1]*impor[1]+_gap[2]*impor[2]+_gap[3]*impor[3])*(_gap[0]*impor[0]+_gap[1]*impor[1]+_gap[2]*impor[2]+_gap[3]*impor[3])+valN*(OrRoot[0]-testRoot).length()*(OrRoot[0]-testRoot).length();
#ifdef DEBUG_DRAW
			RE::moveEntity(RE::createEntity("hips0", "sphere1010.mesh"), vector3(6,6,6), OrRoot[0]*100);
			//RE::moveEntity(RE::createEntity("hips1", "sphere1010.mesh"), vector3(6,6,6), testRoot*100);
#endif
			//printf("Te: %f valN : %f\n",e,valN*(OrRoot[0]-testRoot).length()*(OrRoot[0]-testRoot).length());	
			}
//		   	}e=valL*(_gap[2]*_gap[2]+_gap[3]*_gap[3]);
		if(flagR)
		{
			if (mEffectors.size()==1)
				e=valL*(_gap[flagC*2+0]*_gap[flagC*2+0]*impor[flagC*2+0])+valM*x.toVector3(0).length()*x.toVector3(0).length();
			else
				e=valL*(_gap[flagC*2+0]*_gap[flagC*2+0]*impor[flagC*2+0]+_gap[flagC*2+1]*_gap[flagC*2+1]*impor[flagC*2+1])+valM*x.toVector3(0).length()*x.toVector3(0).length();
		}
		
#ifdef DEBUG_DRAW
		RE::output("angle.length", "%f", x.toVector3(0).length()*x.toVector3(0).length());
		RE::output("TranslRoot.length", "%f", (OrRoot[0]-OptiRoot).length());
		RE::output("opti_e:", "%f", e);
#endif
		return e;
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
	void LimbIKsolver2::setOption(const char* type, double val)
	{
		TString tid(type);
		if(tid=="optiR") // 0 or 1
			optiR=val;
		else if(tid=="optiT") // 0 or 1
			optiT=val;
		else if(tid=="iterik") // 0 or 1
			iterik=val;
	}
	void LimbIKsolver2::setOption(int option)
	{	if(option==0)
		{
		optiR=0;
		optiT=0;
		iterik=0;
		}
		else if(option==1)
		{
			optiR=!optiR;
		}else if(option==2)
		{
			iterik=!iterik;
		}
		else if(option==3)
		{
			optiT=!optiT;
		}
	}
	void LimbIKsolver2::setValue(double ValL, double ValM, double ValN,int IterNum)
	{
		valL=ValL;
		valM=ValM;
		valN=ValN;
		iterNum=IterNum;
	}

	void LimbIKsolver2::_limbIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance)
	{
		for(int c=0; c<mEffectors.size();c++)
		{
			LimbIKinfo &info=_limbikInfo[c];
			vector3 csh;
			info.hip->getTranslation(csh);
			info.goal=conpos[c]-info.hand+info.wrist-csh+info.sh;
			_limbikInfo[c].limbIK(importance[c], conori[c]);
			mSkeleton.fkSolver().setChain(*info.ankle);
		}
	}
	void LimbIKsolver2::IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& _conori, vectorn const& importance)
	{
		vector3 nextSh,diff[2],printL,printR;
		double target,gap[4]={10000,10000,10000,10000},preGap[4]={10000,10000,10000,10000},maxGap[4]={0,0,0,0},minGap[4]={10000,10000,10000,10000};
		MotionDOF::setRootTransformation(poseInout, newRootTF);
		setSkeleton(poseInout);
		mRootPos.resize(mEffectors.size()/2);
		mRootOri.resize(mEffectors.size()/2);
		for (int c=0; c<mEffectors.size(); c++)
		{
			LimbIKinfo &info=_limbikInfo[c];
			info.hipoffset=mPelvis[c/2]->getFrame().toLocalPos(info.hip->getTranslation());
		}
		_updateRootInfo();

		con=conpos;
		conori=_conori;
		impor=importance;
			
		for(int c=0; c<mEffectors.size();c++)
		{
			_prepare(c);	
			LimbIKinfo &info=_limbikInfo[c];
			info.goal=conpos[c]-info.hand+info.wrist;
		}
		//first ik
		_limbIK(conpos, conori, importance);

		for(int c=0; c<mEffectors.size();c++)
		{	
			vector3 hand=getAnkleBone(c)->getFrame().toGlobalPos(mEffectors[c].localpos);
			gap[c]=hand.distance(conpos[c]);
		}
//		printf("first L gap[0] = %f \n",gap[0]);
	//	printf("first R gap[1] = %f \n",gap[1]);
//		printf("%f\n",gap[0]);
//		printf("%f\n",gap[1]);
		//second optimize and ik
		if(optiT||iterik||optiR)
		{
			//for(int cnt=0 ;!(((gap[0]*gap[0]+gap[1]*gap[1])<0.0000001)||(preGap[0]*preGap[0]+preGap[1]*preGap[1])<(gap[0]*gap[0]+gap[1]*gap[1]))&&(cnt<100);cnt++ )
			for(int cnt=0 ;!((gap[0]*gap[0]+gap[1]*gap[1]+gap[2]*gap[2]+gap[3]*gap[3])<0.0000001)&&(cnt<iterNum);cnt++ )
			{
				if(optiR)
				{
					_calcRootRotation();
				}
				if(optiT)
				{
					OrRoot=mRootPos;
					_calcRootTranslation();
				}
				if(iterik)	
				{
					_limbIK(conpos, conori, importance);
				}
				for(int c=0; c<mEffectors.size();c++)
				{	
					vector3 hand=getAnkleBone(c)->getFrame().toGlobalPos(mEffectors[c].localpos);
					gap[c]=hand.distance(conpos[c]);
				}
			}//gap==0||preGap<gap
		}
		mSkeleton.fkSolver().forwardKinematics();
		mSkeleton.getPoseDOF(poseInout);
	}

	void LimbIKsolver2::IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con)
	{
		Msg::error("not implemented");
	}

  void LimbIKsolver2::IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con)
	{
		Msg::error("not implemented");
	}


void LimbIKsolver2::LimbIKinfo::limbIK(double importance, quater const& conori)
{
	q1=qo1;
	q2=qo2;
	const bool useKneeDamping=true;
	MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, axis, useKneeDamping, NULL, kneeDampingConstant);

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
void LimbIKsolver2::LimbIKinfo::prepare(quater const& conori, vector3 const& localpos)
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
void LimbIKsolver2::_updateRootInfo()
{
		for (int c=0; c<mEffectors.size(); c+=2)
		{
			mPelvis[c/2]->getTranslation(mRootPos[c/2]);
			mPelvis[c/2]->getRotation(mRootOri[c/2]);
			//mRootOri[c/2]=mPelvis[c/2]->_getLocalFrame().rotation;
#ifdef DEBUG_DRAW
			RE::moveEntity(RE::createEntity("mRootOri", "axes.mesh"), vector3(3,3,3), mRootOri[c/2], vector3(0,-10,0)+ mPelvis[c/2]->getTranslation()*100);	
#endif
		}
}
