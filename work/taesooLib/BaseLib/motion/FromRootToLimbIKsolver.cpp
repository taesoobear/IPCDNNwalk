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

#include "LimbIKsolver.h"
using namespace MotionUtil;

LimbIKsolver2::LimbIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& knee_bone_indexes, vectorn const& axis_sign)
		:mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),mEffectors(effectors)
	{
		mHipBones.resize(mEffectors.size());
		mKneeBones.resize(mEffectors.size());
		mAnkleBones.resize(mEffectors.size());
		mAxis.setSize(mEffectors.size());
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
				Msg::error("LimbIKsolver error 1");

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

	// deprecated:
	LimbIKsolver2::LimbIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, Bone const& left_knee, Bone const& right_knee)
		:mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),mEffectors(effectors)
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
		}
	}

	static void rotateChildren2(Bone& b, quater const& q)
	{
		b._getFrame().rotation.leftMult(q);
		for(Bone* hc=b.child(); hc; hc=hc->sibling())
			rotateChildren(*hc,q);
	}

	void LimbIKsolver2::setSkeleton(vectorn & temp)
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


	void LimbIKsolver2::IKsolve3(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance)
	{
		MotionDOF::setRootTransformation(poseInout, newRootTF);

		setSkeleton(poseInout);

		quater deltaHip[4];
		for (int i=0; i<conpos.size(); i++)
			deltaHip[i].identity();
		
//		quater test;
		vector3 goal, sh, elb, v1, v2, v3, v4, wrist,hand;
		quater q1, q2;

		quater qo1, qo2,qt;
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

			if (mAnkleBones[c]->child() && mAnkleBones[c]->child()->rotJointIndex()!=-1)
			{
				mAnkleBones[c]->getRotation(qo1);
				mAnkleBones[c]->child()->getRotation(qo2);
				qt.difference(qo1, qo2);
				// preserve original global ankle orientation.
				mAnkleBones[c]->_getFrame().rotation=conori[c];
				mAnkleBones[c]->child()->_getFrame().rotation.mult(qt, conori[c]);
			}
			else
			{
				// preserve original global ankle orientation.
				mAnkleBones[c]->_getFrame().rotation=conori[c];
			}

			hand=mAnkleBones[c]->getFrame().toGlobalPos(mEffectors[c].localpos);
			goal=conpos[c]-hand+wrist;

			v3.difference(sh, elb);
			v4.difference(elb, mAnkleBones[c]->getTranslation());

			const bool useKneeDamping=true;

			qo1=q1;
			qo2=q2;
			if(mbAdjustLen)
				MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, mAxis[c], useKneeDamping,&r[c]);
			else
				MotionUtil::limbIK_1DOFknee(goal, sh, v1, v2, v3, v4, q1, q2, mAxis[c], useKneeDamping);

			qt.difference(qo1, q1);
			qt.scale(importance[c]);
			q1=qt*qo1;

			qt.difference(qo2, q2);
			qt.scale(importance[c]);
			q2=qt*qo2;

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

		mSkeleton.getPose(tempp);
/*		mSkeleton.setPose(tempp);

		quater tt1,tt2,tt3;
		int c=mEffectors.size()-1;
		tt1=mHipBones[c]->_getFrame().rotation;
		tt2=mKneeBones[c]->_getFrame().rotation;
		tt3=mAnkleBones[c]->_getFrame().rotation;*/
		mDofInfo.getDOF(tempp, poseInout);

		if(mbAdjustLen)
		{
			for(int c=0;c<mEffectors.size();c++)
			{
				poseInout[mDofInfo.startT(mHipBones[c]->child()->treeIndex())]=r[c]*-1;
				poseInout[mDofInfo.startT(mKneeBones[c]->child()->treeIndex())]=r[c]*-1;
			}
		}
	}
	void LimbIKsolver2::IKsolve(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con)
	{
		transf origRootTF=MotionDOF::rootTransformation(temp);
/*
		{
			quater rotY, offset, rotY2, offset2;
			origRootTF.rotation.decompose(rotY, offset);
			newRootTF.rotation.decompose(rotY2, offset2);

			origRootTF.rotation.mult(rotY2, offset);
		}

		MotionDOF::setRootTransformation(temp, origRootTF);*/

		{
			quater rotY, offset;
			origRootTF.rotation.decompose(rotY, offset);
			origRootTF.rotation.mult(currRotY, offset);
		}

		MotionDOF::setRootTransformation(temp, origRootTF);
		setSkeleton(temp);

		quater ankleGlobal[2];
		for(int c=0; c<mEffectors.size();c++)
			ankleGlobal[c]=mAnkleBones[c]->getFrame().rotation;
		
		MotionDOF::setRootTransformation(temp, newRootTF);

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

  void LimbIKsolver2::IKsolve2(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con)
	{
		transf origRootTF=MotionDOF::rootTransformation(temp);

		{
			quater rotY, offset;
			origRootTF.rotation.decompose(rotY, offset);
			origRootTF.rotation.mult(currRotY, offset);
		}

		MotionDOF::setRootTransformation(temp, origRootTF);
		setSkeleton(temp);

		quater  ankleGlobal[2];
		for(int c=0; c<mEffectors.size();c++)
			ankleGlobal[c]=delta_foot[c]*mAnkleBones[c]->getFrame().rotation;
		
		MotionDOF::setRootTransformation(temp, newRootTF);

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
/*

#ifdef USE_LUABIND
#include "../MainLib/WrapperLua/BaselibLUA.h"
#include "../MainLib/WrapperLua/MainlibLUA.h"
#include <luabind/luabind.hpp>
#include <luabind/operator.hpp>
#include <luabind/out_value_policy.hpp>
#include <luabind/adopt_policy.hpp>
#include <luabind/error.hpp>

using namespace luabind;

void registerLimbIKsolver2(lua_State* L)
{
	luabind::module(L)
	[
	 class_<MotionUtil::LimbIKsolver>("LimbIKsolver")
	 .def(constructor<MotionDOFinfo const&, std::vector<MotionUtil::Effector>&, Bone const&, Bone const&>())
		.def(constructor<MotionDOFinfo const&, std::vector<MotionUtil::Effector>&, intvectorn const&, vectorn const&>())
	 .def("IKsolve", &MotionUtil::LimbIKsolver::IKsolve)
			.def("IKsolve2", &MotionUtil::LimbIKsolver::IKsolve2)
			.def("IKsolve3", &MotionUtil::LimbIKsolver::IKsolve3)
	];

}
#endif*/
