#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "../BaseLib/math/optimize.h"
//#include "../MainLib/gsl_addon/gsl_wrap.h"
#include "FullbodyIK_MotionDOF.h"
#include "motion/Motion.h"
#include "motion/MotionUtil.h"
#include "motion/MotionLoader.h"
#include "motion/MotionDOF.h"
#include "motion/IKSolver.h"
//#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../../MainLib/OgreFltk/objectList.h"
static ObjectList * g_debugDraw=NULL;
#endif

using namespace MotionUtil;

FullbodyIK_MotionDOF3::FullbodyIK_MotionDOF3(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors)
	:mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),mEffectors(effectors)
{}
class FullbodyIKDOF_limbIK: public FullbodyIK_MotionDOF
{
public:
	std::vector<Effector> mEffectors;
	MotionDOFinfo const& mDofInfo;
	MotionLoader const& mSkeleton;

	std::vector<const Bone*> mHipBones;
	std::vector<const Bone*> mKneeBones;
	std::vector<const Bone*> mAnkleBones;
	vector3N mAxis;
	Posture tempp;
	bool bStraight;
	bool mbAdjustLen;

	FullbodyIKDOF_limbIK(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, Bone const& left_knee, Bone const& right_knee, bool bstraight=false, bool breversedAxis=false)
		:mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),mEffectors(effectors), 
		bStraight(bstraight)
	{

		Msg::verify(mEffectors.size()<=2, "eff size");
		mHipBones.resize(mEffectors.size());
		mKneeBones.resize(mEffectors.size());
		mAnkleBones.resize(mEffectors.size());
		mAxis.setSize(mEffectors.size());
		for(int i=0; i<mEffectors.size(); i++)
		{
		
			Msg::verify(mEffectors[i].bone->getRotationalChannels().length()==3, "rotc");
			if(mEffectors[i].bone==left_knee.child())
			{
				mKneeBones[i]=&left_knee;
				ASSERT(left_knee.getRotationalChannels().length()==1);
				ASSERT(left_knee.parent()->getRotationalChannels().length()==3);
				mHipBones[i]=left_knee.parent();
				mAnkleBones[i]=mEffectors[i].bone;
				mbAdjustLen=false;
			}
			else if(mEffectors[i].bone==right_knee.child())
			{
				mKneeBones[i]=&right_knee;
				ASSERT(right_knee.getRotationalChannels().length()==1);
				ASSERT(right_knee.parent()->getRotationalChannels().length()==3);
				mHipBones[i]=right_knee.parent();
				mAnkleBones[i]=mEffectors[i].bone;
				mbAdjustLen=false;

			}
			else if(mEffectors[i].bone==left_knee.child()->child())
			{
				mKneeBones[i]=&left_knee;
				ASSERT(left_knee.getRotationalChannels().length()==1);
				ASSERT(left_knee.parent()->getRotationalChannels().length()==0);
				ASSERT(left_knee.parent()->parent()->getRotationalChannels().length()==3);

				mHipBones[i]=left_knee.parent()->parent();
				mAnkleBones[i]=mEffectors[i].bone;
				mbAdjustLen=true;

			}
			else if(mEffectors[i].bone==right_knee.child()->child())
			{
			
				mKneeBones[i]=&right_knee;
				ASSERT(right_knee.getRotationalChannels().length()==1);
				ASSERT(right_knee.parent()->getRotationalChannels().length()==0);
				ASSERT(right_knee.parent()->parent()->getRotationalChannels().length()==3);

				mHipBones[i]=right_knee.parent()->parent();
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
			if(breversedAxis)
				mAxis[i]*=-1;
		}
	}

	static void rotateChildren(Bone& b, quater const& q)
	{
		b._getFrame().rotation.leftMult(q);
		for(Bone* hc=b.child(); hc; hc=hc->sibling())
			rotateChildren(*hc,q);
	}

	void setSkeleton(vectorn & temp)
	{
		if(mbAdjustLen)
		{
			// reset knee translations to 0.
			temp[mDofInfo.startT(mKneeBones[0]->child()->treeIndex())]=0;
			temp[mDofInfo.startT(mKneeBones[1]->child()->treeIndex())]=0;

			// reset hip translations to 0.
			temp[mDofInfo.startT(mHipBones[0]->child()->treeIndex())]=0;
			temp[mDofInfo.startT(mHipBones[1]->child()->treeIndex())]=0;
		}
		mSkeleton.setPose(mDofInfo.setDOF(temp));
	}

	vectorn temp;

	virtual void IKsolve(vectorn const& input, vectorn& output, vector3N const& constraintPositions)
	{
		vector3* con;
		temp=input;
		con=&constraintPositions[0];
		setSkeleton(temp);

		quater deltaHip[2];
		deltaHip[0].identity();
		deltaHip[1].identity();
		
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

			hand=mAnkleBones[c]->getFrame().toGlobalPos(mEffectors[c].localpos);
			goal=con[c]-hand+wrist;

			v3.difference(sh, elb);
			v4.difference(elb, mAnkleBones[c]->getTranslation());

			const bool useKneeDamping=true;

			if(bStraight) deltaHip[c]=q1;

			if(mbAdjustLen)
				MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, mAxis[c], useKneeDamping,&r[c]);
			else
				MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, mAxis[c], useKneeDamping);

			if(bStraight) deltaHip[c].difference(deltaHip[c],q1);

			/*
			if(useKneeDamping)
				IKSolver::limbIK(goal, sh, v1, v2, q1, q2, 1.0, mAxis[c], &v3, &v4);
			else
				IKSolver::limbIK(goal, sh, v1, v2, q1, q2, 1.0, mAxis[c]);
*/

			
			mHipBones[c]->_getFrame().rotation=q1;
			mKneeBones[c]->_getFrame().rotation=q2;

			if(mbAdjustLen)
			{
				// update global orientations of translation joints too,
				// so that getPoseFromGlobal(...) works.
				mHipBones[c]->child()->_getFrame().rotation=q1;
				mKneeBones[c]->child()->_getFrame().rotation=q2;
			}
		}

		if(bStraight)
		{
			quater t;
			t.interpolate(0.5, deltaHip[0], deltaHip[1]);
			for(Bone* hc=mSkeleton.bone(1).child(); hc; hc=hc->sibling())
			{
				if(hc!=mHipBones[0] && hc!=mHipBones[1])
					rotateChildren(*hc, t);
			}
		}
		mSkeleton.getPose(tempp);
/*		mSkeleton.setPose(tempp);

		quater tt1,tt2,tt3;
		int c=mEffectors.size()-1;
		tt1=mHipBones[c]->_getFrame().rotation;
		tt2=mKneeBones[c]->_getFrame().rotation;
		tt3=mAnkleBones[c]->_getFrame().rotation;*/
		mDofInfo.getDOF(tempp, output);

		if(mbAdjustLen)
		{
			for(int c=0;c<mEffectors.size();c++)
			{
				output[mDofInfo.startT(mHipBones[c]->child()->treeIndex())]=r[c]*-1;
				output[mDofInfo.startT(mKneeBones[c]->child()->treeIndex())]=r[c]*-1;
			}
		}
	}
};
void MotionUtil::setLimbIKParam_straight(FullbodyIK_MotionDOF* ik, bool bStraight)
{
	FullbodyIKDOF_limbIK* iik=(FullbodyIKDOF_limbIK* )ik;
	iik->bStraight=bStraight;
}

MotionUtil::FullbodyIK_MotionDOF* MotionUtil::createFullbodyIkDOF_limbIK(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee)
{
	return new FullbodyIKDOF_limbIK(info, effectors, left_knee, right_knee);
}

MotionUtil::FullbodyIK_MotionDOF* MotionUtil::createFullbodyIkDOF_limbIK(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee, bool bReversed)
{
	return new FullbodyIKDOF_limbIK(info, effectors, left_knee, right_knee, false, bReversed);
}

FullbodyIK_MotionDOF* MotionUtil::createFullbodyIkDOF_limbIK_straight(MotionDOFinfo const& info, std::vector<Effector>& effectors, Bone const& left_knee, Bone const& right_knee)
{
	return new FullbodyIKDOF_limbIK(info, effectors, left_knee, right_knee, true);
}



#include "IK_sdls/Node.h"
#include "IK_sdls/Tree.h"
#include "IK_sdls/Jacobian.h"
#include "IK_sdls/NodeWrap.h"

#define TEST_EULER_ROOT 0 // when set 0, quaternion integration is used.
#define TEST_FIXED_ROOTPOS 0

class FullbodyIK_MotionDOF_MultiTarget: public FullbodyIK_MotionDOF, public IK_sdls::LoaderToTree
{
	std::vector<MotionUtil::Effector> mEffectors;
	MotionDOFinfo const& mDofInfo;
	MotionLoader const& mSkeleton;

	
	IK_sdls::Jacobian *mJacob;

	
public:

	FullbodyIK_MotionDOF_MultiTarget(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, std::vector<RelativeConstraint>& constraints)
		:mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),mEffectors(effectors),
		 IK_sdls::LoaderToTree(dofInfo.skeleton(), effectors,constraints,
				 TEST_EULER_ROOT, TEST_FIXED_ROOTPOS)
	{
		if(mEffectors.size()>0)
		{
			mJacob = new IK_sdls::Jacobian(&mTree);
			mJacob->Reset();
		}
		else
			mJacob=NULL;
	}

	virtual~FullbodyIK_MotionDOF_MultiTarget(){ delete mJacob;}

	virtual void getAffectedDOF(intvectorn & index, intvectorn & trans_joint_index)
	{
		trans_joint_index.setSize(0);
		
		int con_size=mEffectors.size();
		index.reserve(mNode.size()-con_size);
		index.setSize(0);// excluding EFFECTOR

		for(int i=0; i<mNode.size(); i++)
		{
			Bone* bone=mNode[i].bone;
			int treeIndex=bone->GetIndex();

			if(mNode[i].node[0]->IsJoint())
				index.push_back(mSkeleton.getRotJointIndexByTreeIndex(treeIndex));
		}

		ASSERT(index.size()==mNode.size()-con_size);
	}

	virtual void IKsolve(vectorn const& pose, vectorn& output, vector3N const& con2)
	{
		

		vector3* con;
		int con_size;
		{
			con=&con2[0];
			con_size=con2.size();
		}

		ASSERT(mSkeleton.numTransJoint()<=1);
		// set tree. (position, orientation) : quaternion->euler

		setPoseDOF(mTree, mDofInfo, pose, mNode);
		mTree.Compute();

		// set target.
		mTree.target=con;
#if TEST_FIXED_ROOTPOS 
		for(int i=0; i<con_size; i++)
			mTree.target[i]-=pose.toVector3(0);
#endif

		mJacob->SetJendActive();

		double prevCost=DBL_MAX;
		for(int iter=0; iter<1000; iter++)
		{			
			mJacob->ComputeJacobian();

			// over-determined case에 대해서는 SDLS가 동작하지 않음.
			if(mJacob->ActiveJacobian().GetNumRows() > mJacob->ActiveJacobian().GetNumColumns() )
			//	mJacob->CalcDeltaThetasTranspose();		// Jacobian transpose method
				mJacob->CalcDeltaThetasDLS();			// Damped least squares method
			//	mJacob->CalcDeltaThetasPseudoinverse();	// Pure pseudoinverse method
			else
				mJacob->CalcDeltaThetasSDLS();			// Selectively damped least squares method

			//mJacob->CalcDeltaThetasTranspose();		// Jacobian transpose method
			//mJacob->CalcDeltaThetasDLS();			// Damped least squares method
						
			mJacob->UpdateThetas();						// Apply the change in the theta values
			mJacob->UpdatedSClampValue();

			if(iter%10==0)
			{
				// evaluate IK results.
				m_real cost=0;
		
				ASSERT(mEffectors.size()==con_size);
				for(int i=0; i<mEffectors.size(); i++)
				{
					IK_sdls::Node* node=mEffectorNode[i].node[0];
					double dist=(node->GetS()-mTree.target[i]).length();
					cost+=dist;		
				}
#ifdef _DEBUG
				Msg::print2("%d:%f", iter, cost);
#endif

				if(ABS(prevCost-cost)<0.000001) break;	// 수렴하면 멈춘다.
				prevCost=cost;
			}
		}

		getPoseDOF(mDofInfo, output, mNode);
	}
};
















FullbodyIK_MotionDOF* MotionUtil::createFullbodyIk_MotionDOF_MultiTarget(MotionDOFinfo const& info, std::vector<Effector>& effectors)
{
	std::vector<RelativeConstraint> constraints;
	return new FullbodyIK_MotionDOF_MultiTarget(info, effectors, constraints);
}

FullbodyIK_MotionDOF* MotionUtil::createFullbodyIk_MotionDOF_MultiTarget(MotionDOFinfo const& info, std::vector<Effector>& effectors, std::vector<RelativeConstraint>& constraints)
{
	return new FullbodyIK_MotionDOF_MultiTarget(info, effectors, constraints);
}


void MotionUtil::FullbodyIK_MotionDOF::IKsolve(vectorn& poseInOut, vector3N const& constraintPositions)
{
	static vectorn out;
	out.assign(poseInOut);
	IKsolve(poseInOut, out, constraintPositions);
	poseInOut=out;
}

