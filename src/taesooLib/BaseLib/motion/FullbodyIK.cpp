#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "../BaseLib/math/optimize.h"
//#include "../MainLib/gsl_addon/gsl_wrap.h"
#include "FullbodyIK.h"
#include "motion/Motion.h"
#include "motion/MotionUtil.h"
#include "motion/MotionLoader.h"
#include "motion/IKSolver.h"

using namespace MotionUtil;






Bone& dep_GetBoneFromCon(MotionLoader const& ml, int constraint) ;



void FullbodyIK::IKsolve(Posture& pose, vector3N const& con)
{
	static intvectorn joint_index, trans_joint_index;
	static quaterN delta_rot;
	static vector3N delta_trans;	
	IKsolve(pose, con, joint_index, delta_rot, trans_joint_index, delta_trans);

	for(int j=0; j<joint_index.size();j++)
		pose.m_aRotations[joint_index[j]].leftMult(delta_rot[j]);

	for(int j=0; j<trans_joint_index.size(); j++)
		pose.m_aTranslations[trans_joint_index[j]]+=delta_trans[j];
}

void FullbodyIK::IKsolve(Posture const& input, Posture& output, vector3N const& con)
{
	static intvectorn joint_index, trans_joint_index;
	static quaterN delta_rot;
	static vector3N delta_trans;	
	IKsolve(input, con, joint_index, delta_rot, trans_joint_index, delta_trans);
	
	ASSERT(output.numRotJoint()==input.numRotJoint());
	ASSERT(output.numTransJoint()==input.numTransJoint());
	for(int j=0; j<joint_index.size();j++)
		output.m_aRotations[joint_index[j]].mult(delta_rot[j], input.m_aRotations[joint_index[j]]);

	for(int j=0; j<trans_joint_index.size(); j++)
		output.m_aTranslations[trans_joint_index[j]].add(delta_trans[j],
		input.m_aTranslations[trans_joint_index[j]]);
}


int _GetConFromBone(MotionLoader const& ml, Bone* bone) 
{
	int aCon[]={CONSTRAINT_LEFT_HEEL, CONSTRAINT_RIGHT_HEEL, CONSTRAINT_LEFT_TOE, CONSTRAINT_RIGHT_TOE,
		CONSTRAINT_LEFT_HAND,CONSTRAINT_RIGHT_HAND,CONSTRAINT_LEFT_FINGERTIP,CONSTRAINT_RIGHT_FINGERTIP};
	
	for(int i=0; i<8; i++)
	{
		if(&dep_GetBoneFromCon(ml, aCon[i])== bone)
			return aCon[i];
	}

	return NUM_CONSTRAINT;
}

bool isToeBone(Bone* bone)
{
	int pvoca=bone->parent()->voca();

	if(pvoca==MotionLoader::LEFTANKLE ||
		pvoca==MotionLoader::RIGHTANKLE ||
		pvoca==MotionLoader::LEFTWRIST ||
		pvoca==MotionLoader::RIGHTWRIST)
		return true;

	return false;
}

class FullbodyIK_limbik: public FullbodyIK
{
	std::vector<Effector> mEffectors;
	MotionLoader& mSkeleton;
	bool mKneeDamping;
public:
	//IKSolver ik;
	FullbodyIK_limbik(MotionLoader& skeleton, std::vector<Effector>& effectors, bool useKneeDamping):mSkeleton(skeleton),mEffectors(effectors), mKneeDamping(useKneeDamping)
	{
/*#ifdef _DEBUG
		for(int i=0; i<mEffectors.size(); i++)
		{
			if(!isSimilar(mEffectors[i].localpos.length(), 0))
				Msg::error("LimbIK:: local pos!=0");
		}
#endif*/
	}
	virtual~FullbodyIK_limbik(){}

	void getAffectedDOF(intvectorn & index, intvectorn & trans_joint_index)
	{
		trans_joint_index.setSize(0);
		
		intvectorn index2;
		quaterN delta_rot2;

		index.reserve(mEffectors.size()*3);
		index.setSize(0);


		for(int i=0; i<mEffectors.size(); i++)
		{			
			bool bToeCorrection=isToeBone(mEffectors[i].bone);
		
			IKSolveAnalytic(mSkeleton, *mEffectors[i].bone, vector3(0,0,0), index2, delta_rot2, mKneeDamping, bToeCorrection);
			
			for(int id=0; id<3; id++)
				index.push_back(index2[id]);
		}
	}

	virtual void IKsolve(Posture const& pose, vector3N const& con, intvectorn & index, quaterN& delta_rot, intvectorn & trans_joint_index, vector3N& delta_trans)
	{
		trans_joint_index.setSize(0);
		delta_trans.setSize(0);

		intvectorn index2;
		quaterN delta_rot2;
		index.reserve(mEffectors.size()*3);
		delta_rot.reserve(mEffectors.size()*3);
		index.setSize(0);
		delta_rot.setSize(0);

		mSkeleton.setPose(pose);

		for(int i=0; i<mEffectors.size(); i++)
		{
			bool bToeCorrection=isToeBone(mEffectors[i].bone);
		
			vector3 goal=con[i];
			if(!isSimilar(mEffectors[i].localpos.length(), 0))
			{
				ASSERT(!bToeCorrection);
				vector3 currAnkle;
				vector3 currToe;
				mEffectors[i].bone->getTranslation(currAnkle);
				currToe=mEffectors[i].bone->getFrame()*mEffectors[i].localpos;

				goal=goal-currToe+currAnkle;
			}
			IKSolveAnalytic(mSkeleton, *mEffectors[i].bone, goal, index2, delta_rot2, mKneeDamping, bToeCorrection);
			
			for(int id=0; id<3; id++)
			{
				index.push_back(index2[id]);
				delta_rot.pushBack(delta_rot2[id]);
			}
		}
	}
};


#include "IK_sdls/Node.h"
#include "IK_sdls/Tree.h"
#include "IK_sdls/Jacobian.h"
#include "IK_sdls/NodeWrap.h"


class FullbodyIK_MultiTarget: public FullbodyIK
{
	std::vector< MotionUtil::Effector> mEffectors;
	MotionLoader& mSkeleton;
	
	IK_sdls::Tree mTree;
	IK_sdls::Jacobian *mJacob;

	std::vector<IK_sdls::NodeWrap> mNode;
	std::vector<IK_sdls::NodeWrap> mEffectorNode;
	bitvectorn mEffectorAttached;
	intvectorn mBoneToNode;
	bool rootPositionSpecified;
public:
	void copyTree(Bone* bone, IK_sdls::Node* parent)
	{
		if(bone->numChannels()==0)
		{
			return;
		}
		else if(mEffectorAttached[bone->GetIndex()])
		{
			if(parent==NULL)
			{
				mNode.push_back(IK_sdls::NodeWrap());

				mNode.back().createFreeNode(bone);
				mTree.InsertRoot(mNode.back().node[0]);
				mBoneToNode[bone->GetIndex()]=mNode.size()-1;
			}
			else
			{
				mNode.push_back(IK_sdls::NodeWrap());
				TString channels=bone->getRotationalChannels();
				mNode.back().createNodes(bone, channels);

				mTree.InsertChild_automatic(parent, mNode.back().node[0]);
				for(int c=1; c<channels.length(); c++)
					mTree.InsertLeftChild(mNode.back().node[c-1], mNode.back().node[c]);
				mBoneToNode[bone->GetIndex()]=mNode.size()-1;
			}


			if(bone->m_pChildHead)
			{
				copyTree((Bone*)bone->m_pChildHead, mNode[mBoneToNode[bone->GetIndex()]].back());
			}
		}

		if(bone->m_pSibling)
		{
			copyTree((Bone*)bone->m_pSibling, mNode[mBoneToNode[bone->m_pParent->GetIndex()]].back());
		}
	}

	FullbodyIK_MultiTarget(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& effectors):mSkeleton(skeleton),mEffectors(effectors)
	{
		skeleton.UpdateInitialBone();

		rootPositionSpecified=false;

		if(!mEffectors[0].bone->parent()->parent() &&
			isSimilar(mEffectors[0].localpos.length(),0))
		{
			rootPositionSpecified=true;
			mEffectors.erase(mEffectors.begin());
		}

		for(int i=0; i<mEffectors.size(); i++)
		{
			/*
			if(isSimilar(mEffectors[i].localpos.length(), 0))
			{
				mEffectors[i].bone->getOffset(mEffectors[i].localpos);
				mEffectors[i].bone=mEffectors[i].bone->parent();
			}
			*/
		}
		
		/*
		아래 주석 처리된 코드와 유사한 내용이 리커시브하게 수행된다.
		// pelvis.
		bone=&skeleton.getBoneByVoca(MotionLoader::HIPS);

		mRootPos=bone->getTranslation();
		mNode.push_back(NodeWrap());
		mNode.back().createNodes(bone, "ZXY");
		mTree.InsertRoot(mNode.back().node[0]);
		mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
		mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
		parent=mNode.back().node[2];

		// righthip
		bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTHIP);
		mNode.push_back(NodeWrap());
		mNode.back().createNodes(bone, "ZXY");
		mTree.InsertLeftChild(parent, mNode.back().node[0]);
		mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
		mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
		parent=mNode.back().node[2];

		// knee
		bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTKNEE);
		mNode.push_back(NodeWrap());
		mNode.back().createNodes(bone, "ZXY");
		mTree.InsertLeftChild(parent, mNode.back().node[0]);
		mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
		mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
		parent=mNode.back().node[2];

		// ankle
		bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTANKLE);

		mNode.push_back(NodeWrap());
		mNode.back().createNodes(bone, "EFFECTOR");
		mTree.InsertLeftChild(parent, mNode.back().node[0]);
	
		*/

		// Effector랑 관련이 없는 bone은 IK 트리만들때 제외하기 위해, 먼저 각 본이 effector와 관련이 되있는지 아닌지를 mEffectorAttached에 저장한다.
		mEffectorAttached.resize(skeleton.GetNumTreeNode());
		mBoneToNode.setSize(skeleton.GetNumTreeNode());
		mBoneToNode.setAllValue(-1);
		mEffectorAttached.clearAll();

		for(int i=0; i<mEffectors.size(); i++)
		{
			for(Bone* bone=mEffectors[i].bone; bone!=NULL; bone=bone->parent())
				mEffectorAttached.setAt(bone->GetIndex());
		}

		Bone* bone;
		vector3 zero (0,0,0);

		copyTree(&skeleton.getBoneByRotJointIndex(0), NULL);

		for(int i=0; i<mEffectors.size(); i++)
		{
			bone=mEffectors[i].bone;

			mEffectorNode.push_back(IK_sdls::NodeWrap());
			mEffectorNode.back().createEffector(bone, mEffectors[i].localpos);

			mTree.InsertEffector(mNode[mBoneToNode[bone->GetIndex()]].back(), (IK_sdls::Effector*)mEffectorNode.back().node[0]);
		}

		mTree.Init();
		mTree.Compute();
	
#ifdef _DEBUG
		mTree.Print();
		compareTrees(vector3(0,0,0));
#endif

		mJacob = new IK_sdls::Jacobian(&mTree);
		mJacob->Reset();


	}


	virtual~FullbodyIK_MultiTarget(){ delete mJacob;}

	void compareTrees(vector3 trans)
	{
		for(int i=0; i<mNode.size(); i++)
		{
			printf("bone %s (%d)\n", mNode[i].bone->NameId, i);
			
			if(mNode[i].node[0]->IsJoint())
			{
				printf("node %d:%d:%s\n", mNode[i].node[0]->GetJointNum(), mNode[i].node[0]->GetParentJointNum(), (mNode[i].node[0]->GetS()+trans).output().ptr());
				printf("NODE %d:%d:%s\n", mNode[i].back()->GetJointNum(), mNode[i].back()->GetParentJointNum(), (mNode[i].back()->GetS()+trans).output().ptr());
			}
			else
			{
				printf("efct %d:%d:%s\n", mNode[i].node[0]->GetEffectorNum(),mNode[i].node[0]->GetParentJointNum(), (mNode[i].node[0]->GetS()+trans).output().ptr());
			}
			printf("bone %d:%s\n", mNode[i].bone->GetIndex(), mNode[i].bone->getTranslation().output().ptr());
		}
	}

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

	virtual void IKsolve(Posture const& pose, vector3N const& con2, intvectorn & index, quaterN& delta_rot, intvectorn & trans_joint_index, vector3N& delta_trans)
	{
		vector3N con;

		vector3 rootPos=pose.m_aTranslations[0];

		if(rootPositionSpecified)
		{
			rootPos=con2[0];
			con.setSize(con2.size()-1);

			for(int i=0; i<con.size(); i++)
				con[i]=con2[i+1];
		}
		else
			con=con2;
		trans_joint_index.setSize(0);

		ASSERT(pose.numTransJoint()==1);
		// set tree. (position, orientation) : quaternion->euler

		index.reserve(mNode.size()-con.size());
		index.setSize(0);// excluding EFFECTOR

		for(int i=0; i<mNode.size(); i++)
		{
			Bone* bone=mNode[i].bone;
			int treeIndex=bone->GetIndex();

			if(mNode[i].node[0]->IsJoint())
			{				
				index.push_back(mSkeleton.getRotJointIndexByTreeIndex(treeIndex));
				m_real euler[3];
				TString channels=bone->getRotationalChannels();
				pose.m_aRotations[index.back()].getRotation(channels, euler);

				for(int c=0; c<channels.length(); c++)
					mNode[i].node[c]->SetTheta(euler[c]);
			}
		}

		ASSERT(index.size()==mNode.size()-con.size());
		
		delta_trans.setSize(0);

		delta_rot.reserve(mNode.size()-con.size());
		delta_rot.setSize(0);

		mTree.Compute();


		

		if(0){
			//mTree.Print();
			mSkeleton.setPose(pose);
			compareTrees(pose.m_aTranslations[0]);
		}
		// set target.
		mTree.target=con;

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

		
				ASSERT(mEffectors.size()==con.size());
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

		// set delta rot : euler->quaternion
		for(int i=0; i<mNode.size(); i++)
		{
			m_real euler[3];
			if(mNode[i].node[0]->IsJoint())
			{
				quater q;
				

				TString channels=mNode[i].bone->getRotationalChannels();

				for(int c=0; c<channels.length(); c++)
				{
					euler[c]=mNode[i].node[c]->GetTheta();
				}

				q.setRotation(channels, euler);

				q.difference(pose.m_aRotations[index[delta_rot.size()]], q);				

				delta_rot.pushBack(q);
			}
		}
	}
};













FullbodyIK* MotionUtil::createFullbodyIk_LimbIK(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& effectors, bool bUseKneeDamping)
{
	return new FullbodyIK_limbik(skeleton, effectors, bUseKneeDamping);
}


FullbodyIK* MotionUtil::createFullbodyIk_MultiTarget(MotionLoader& skeleton, std::vector<Effector>& effectors)
{
	return new FullbodyIK_MultiTarget(skeleton, effectors);
}


void MotionUtil::FullbodyIK::IKsolve(Posture const& input_pose, vector3N const& constraintPositions, intvectorn & rot_joint_index, quaterN& delta_rot)
{
	intvectorn trans_joint_index;
	vector3N delta_trans;
	IKsolve(input_pose, constraintPositions, rot_joint_index, delta_rot, trans_joint_index, delta_trans);
	ASSERT(trans_joint_index.size()==0);
}
