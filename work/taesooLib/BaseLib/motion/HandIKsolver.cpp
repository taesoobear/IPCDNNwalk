#include "stdafx.h"
#include "../math/mathclass.h"
#include "../math/optimize.h"
#include "../math/Metric.h"
//#include "../MainLib/gsl_addon/gsl_wrap.h"
#include "../motion/FullbodyIK_MotionDOF.h"
#include "../motion/Motion.h"
#include "../motion/MotionUtil.h"
#include "../motion/MotionLoader.h"
#include "../motion/MotionDOF.h"
#include "../motion/IKSolver.h"
#include "../image/Image.h"
#include "../image/ImagePixel.h"

#define NO_DEBUG_GUI
#ifndef NO_DEBUG_GUI
#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/objectList.h"
#ifndef NO_GUI
#include <Ogre.h>
#include <Fl/Fl_Tile.H>
#endif
#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "../../MainLib/OgreFltk/FltkRenderer.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"
#include "../../MainLib/OgreFltk/fastCapture.h"
#include "../../MainLib/OgreFltk/MotionManager.h"
#include "../../MainLib/OgreFltk/Line3D.h"
#include "../../MainLib/OgreFltk/VRMLloader.h"
#endif

#include "HandIKsolver.h"
using namespace MotionUtil;

namespace RE_{
void renderOneFrame(bool check);

}
// 현재 global이지만 코드가 안정화 된 후에 member로 옮기면 됨.
struct HandIKsolver_options
{
	HandIKsolver_options(){ 
		max_iter=3;
		damp_ratio=0.8;
		damp_ratio_wrist=0.7;
		damp_ratio_finger=0.7;
		damp_ratio_fingerweight=0;
		degree_Min = TO_RADIAN(-80);
		degree_Max = TO_RADIAN(10);
		degree_Min_Thumb = TO_RADIAN(-80);
		degree_Max_Thumb = TO_RADIAN(10);
		degree_Min_horizon = TO_RADIAN(-10);
		degree_Max_horizon = TO_RADIAN(10);
		degree_Min_Thumb_horizon = TO_RADIAN(-30);
		degree_Max_Thumb_horizon = TO_RADIAN(80);
		debug_draw=false;
	}
	int max_iter;
	double damp_ratio;
	double damp_ratio_wrist;
	double damp_ratio_finger;
	double damp_ratio_fingerweight;
	double damp_ratio_fingerweight_reverse;
	double degree_Min,degree_Min_Thumb;
	double degree_Min_horizon,degree_Min_Thumb_horizon;
	double degree_Max,degree_Max_Thumb;
	double degree_Max_horizon,degree_Max_Thumb_horizon;
	bool debug_draw;
} ;

// static 함수로 해놓고 c++ 만 고치면, header까지 고치는 것에 비해 build time을 줄일수 있음.
static transf calcDelta(HandIKsolver const& self, vector3N const& conpos, bool noRot=false)
{
	vectorn nowpos_vec (self.mEffectors.size()*3);
	vectorn conpos_vec (self.mEffectors.size()*3);

	// loop for metric
	for(int j=0;j<self.mEffectors.size();j++)
	{
		vector3 Vec = self.mEffectors[j].bone->getFrame().toGlobalPos(self.mEffectors[j].localpos);
		nowpos_vec.setVec3(j*3,Vec);
		conpos_vec.setVec3(j*3,conpos[j]);
	}	
	double test1,test2;

	if (noRot)
	{

		NoRotMetric metricNoRot(true);
		test1=metricNoRot.CalcDistance(conpos_vec,nowpos_vec);
		return transf(metricNoRot.m_transfB);
	}
	PointCloudMetric metric;
	KovarMetric kovarmetric(true);// bool allowTranslationAlongYAxis

	test1 = metric.CalcDistance(conpos_vec,nowpos_vec);
	int n=self.mEffectors.size();
	test1=test1/(n*n); // distance metric not in the same space
	test2 = kovarmetric.CalcDistance(conpos_vec,nowpos_vec);

	transf transfSave (metric.m_transfB);
	transf transfSave2 (kovarmetric.m_transfB);

	//std::cout <<"transfSave.translation "<<transfSave.translation.x<<' '<<transfSave.translation.y<<' '<<transfSave.translation.z<<std::endl;
	//			std::cout<<'('<<i<<')'<<"transfSave2.translation "<<transfSave2.translation.x<<' '<<transfSave2.translation.y<<' '<<transfSave2.translation.z<<std::endl;

	//RE::moveEntity(RE::createEntity("wonderand","axes.mesh"),vector3(5,5,5),limbIK->self.mEffectors[0].bone->getFrame().rotation,limbIK->self.mEffectors[0].bone->getFrame().translation);
	if(test1>test2)
		//if(0)
	{
		//transf temp(transfSave.rotation,transfSave.translation);
		transfSave = transf(transfSave2.rotation,transfSave2.translation);
		//transfSave2 = transf(temp.rotation,temp.translation);
	}
	else
	{
	}
	return transfSave;

}
static void getFingerConfig(HandIKsolver const& self, int j, int index[4], int varindex[4]);
static HandIKsolver_options& options(HandIKsolver const& self)
{
	return *((HandIKsolver_options*)self._options);
}
static m_real  getDOF(HandIKsolver const& self, vectorn& dofTemp, vectorn const& x)
{
	dofTemp = self.dofOrigin;
	m_real cost = 0;
	int i;
	int dofindex[4], varindex[4];
	int reverse = 0;

	for(i=0;i<self.mHandBones.size();i++)
	{
		getFingerConfig(self, i, dofindex, varindex);
		for(int k=0; k<4; k++)
		{
			if (varindex[k]!=-1)
				dofTemp[dofindex[k]]+=x(varindex[k]);
		}
	}
	
	for(i=0;i<self.mHandBones.size();i++)
	{
		getFingerConfig(self, i, dofindex, varindex);
		for(int k=0; k<4; k++)
		{
			if(dofindex[k] == self.mDofInfo.DOFindex(self.mHandBones[i]->treeIndex(),0))
			{
				if(i!=0)
				{
					if(options(self).degree_Min_horizon > dofTemp[dofindex[k]])
						cost+=SQR(dofTemp[dofindex[k]]-options(self).degree_Min_horizon) * options(self).damp_ratio_fingerweight * 1000;
					else if(options(self).degree_Max_horizon <  dofTemp[dofindex[k]])
						cost+=SQR(dofTemp[dofindex[k]]-options(self).degree_Max_horizon) * options(self).damp_ratio_fingerweight * 1000;
				}
				else
				{
					if(options(self).degree_Min_Thumb_horizon > dofTemp[dofindex[k]])
						cost+=SQR(dofTemp[dofindex[k]]-options(self).degree_Min_Thumb_horizon) * options(self).damp_ratio_fingerweight * 1000;
					else if(options(self).degree_Max_Thumb_horizon < dofTemp[dofindex[k]])
						cost+=SQR(dofTemp[dofindex[k]]-options(self).degree_Max_Thumb_horizon) * options(self).damp_ratio_fingerweight * 1000;
				}
			}
			else
			{
				if(i!=0)
				{
					if(options(self).degree_Min > dofTemp[dofindex[k]])
						cost+=SQR(dofTemp[dofindex[k]]-options(self).degree_Min) * options(self).damp_ratio_fingerweight * 1000;
					else if(options(self).degree_Max < dofTemp[dofindex[k]])
						cost+=SQR(dofTemp[dofindex[k]]-options(self).degree_Max) * options(self).damp_ratio_fingerweight * 1000;
				}
				else
				{
					if(options(self).degree_Min_Thumb > dofTemp[dofindex[k]])
						cost+=SQR(dofTemp[dofindex[k]]-options(self).degree_Min_Thumb) * options(self).damp_ratio_fingerweight * 1000;
					else if(options(self).degree_Max_Thumb < dofTemp[dofindex[k]])
						cost+=SQR(dofTemp[dofindex[k]]-options(self).degree_Max_Thumb) * options(self).damp_ratio_fingerweight * 1000;
				}
			}
		}
	}

	return cost;
}

static void fingerIK(HandIKsolver & self)
{
	// finger IK
	self.init(0.01,self.dofNum,0.01,0.005,self.cgmethod);
	vectorn initial(self.dofNum);
	initial.setAllValue(0);
	self.optimize(initial);
	initial = self.getResult();

	vectorn dofTemp;
	getDOF(self, dofTemp, initial);
	//getDOF2(self, dofTemp, initial);
	for(int i=0; i<self.dofOrigin.size(); i++)
	{
		self.dofOrigin[i]=
			self.dofOrigin(i) * (1-options(self).damp_ratio) +
			dofTemp(i) * options(self).damp_ratio;//i*4]);
	}
}
static FullbodyIK_MotionDOF3 * createLimbIKsolver(MotionDOFinfo const& dofInfo,MotionLoader const& mSkeleton,intvectorn const& wrist_bone_indexes, bool bUseSolver2)
{
	FullbodyIK_MotionDOF3 *limbIK;
		//dof 의 갯수
		int wrist_num = wrist_bone_indexes.size();
		std::vector<Effector> wrist_effectors;// = MotionUtil->Effectors();
		vectorn wrist_axis_sign = vectorn(wrist_num);
		wrist_effectors.resize(wrist_num);
		
		
		for(int i=0;i<wrist_num;i++)
		{
			wrist_effectors[i].bone = mSkeleton.bone(wrist_bone_indexes[i]).child();
			wrist_effectors[i].localpos = vector3(0.0,0.0,0.0);

			if(i==0)
				wrist_axis_sign.set(i,-1);
			else
				wrist_axis_sign.set(i,1);
			
		}

		if (bUseSolver2)
		{
			limbIK = new LimbIKsolver2(dofInfo,wrist_effectors,wrist_bone_indexes,wrist_axis_sign);
			limbIK->setOption("optiR", 1);
			limbIK->setOption("iterik", 1);
			((LimbIKsolver2*)limbIK)->setValue(1,500,0.3,1);
		}
		else
		{
			limbIK = new LimbIKsolver(dofInfo,wrist_effectors,wrist_bone_indexes,wrist_axis_sign);
			limbIK->setOption("useKneeDamping", 0);
		}
		return limbIK;
}
//#define DEBUG_DRAW
HandIKsolver::~HandIKsolver()
{
	delete (HandIKsolver_options*)_options;
	_options=NULL;
}
HandIKsolver::HandIKsolver(MotionDOFinfo const& dofInfo, std::vector<Effector>& effectors, intvectorn const& hand_bone_indexes,intvectorn const& wrist_bone_indexes, vectorn const& axis_sign)
		:Optimize(),mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),mEffectors(effectors)
		,cgmethod(10)
		{	
		_wrist_bone_indexes=wrist_bone_indexes;
		_options=(void*)new HandIKsolver_options();
		wrist_num = wrist_bone_indexes.size();
		wrist_conpos = vector3N(wrist_num);
		wrist_conori = quaterN(wrist_num);
		wrist_impo = vectorn(wrist_num);

		limbIK = createLimbIKsolver(dofInfo,mSkeleton, wrist_bone_indexes, true);
		dofNum=15;
		//dofNum = 3;
		//init(0.01, 3, 0.01, 0.005, cgmethod);
		init(0.01, dofNum, 0.01, 0.005, cgmethod);
		mHandBones.resize(hand_bone_indexes.size());
		for(int i=0; i<mHandBones.size(); i++)
		{
			Bone const& hand=mSkeleton.bone(hand_bone_indexes[i]);
			mHandBones[i]=&hand;
			/*
			if(mEffectors[i].bone==hand.child()->child())
			{
				mHandBones[i]=&hand;
			}
			else	
			{
				printf("%s %s\n", mEffectors[i].bone->NameId, hand.NameId);
				Msg::error("HandIKsolver error 1");
			}
			*/
		}
	}

	m_real HandIKsolver::objectiveFunction(vectorn const& x)
	{
		vectorn dofTemp ;
		m_real cost=
		getDOF(*this, dofTemp, x);

		mSkeleton.setPoseDOF(dofTemp);
		transf conPlusVec=calcDelta(*this, con);	
		conPlusVec.scale(options(*this).damp_ratio_finger);

		// penalize distance
		for(int i=0;i<mEffectors.size();i++)
		{
			vector3 tempVec1 = conPlusVec*mEffectors[i].bone->getFrame().toGlobalPos(mEffectors[i].localpos);
			vector3 tempVec2 = con[i];
			vector3 test(0);
				
			test.difference(tempVec1,tempVec2);
			cost += test.length();
		}	

		return cost;
	}

	void HandIKsolver::setOption(const char* id, double val)
	{
		HandIKsolver& self=*this;
		TString tid(id);
		if(tid=="max_iter")
			options(self).max_iter=val;
		else if(tid=="damp_ratio")
			options(self).damp_ratio=val;
		else if(tid=="damp_ratio_wrist")
			options(self).damp_ratio_wrist=val;
		else if(tid=="damp_ratio_finger")
			options(self).damp_ratio_finger=val;
		else if(tid=="damp_ratio_fingerweight")
			options(self).damp_ratio_fingerweight=val;
		else if(tid=="damp_ratio_fingerweight_reverse")
			options(self).damp_ratio_fingerweight_reverse=val;
		else if(tid=="degree_Min")
			options(self).degree_Min = val;
		else if(tid=="degree_Max")
			options(self).degree_Max = val;
		else if(tid=="degree_Min_Thumb")
			options(self).degree_Min_Thumb = val;
		else if(tid=="degree_Max_Thumb")
			options(self).degree_Max_Thumb = val;
		else if(tid=="degree_Min_horizon")
			options(self).degree_Min_horizon = val;
		else if(tid=="degree_Max_horizon")
			options(self).degree_Max_horizon = val;
		else if(tid=="degree_Min_Thumb_horizon")
			options(self).degree_Min_Thumb_horizon = val;
		else if(tid=="degree_Max_Thumb_horizon")
			options(self).degree_Max_Thumb_horizon = val;
		else if(tid=="debug_draw")
			options(self).debug_draw = (bool)val;
		else if(tid=="use_solver2")
		{
			limbIK = createLimbIKsolver(mDofInfo,mSkeleton, _wrist_bone_indexes, val==1.0);
		}
	}
	static void getFingerConfig(HandIKsolver const& self, int j, int index[4], int varindex[4])
	{
		index[0] = self.mDofInfo.DOFindex(self.mHandBones[j]->treeIndex(),0);
		index[1] = self.mDofInfo.DOFindex(self.mHandBones[j]->treeIndex(),1);
		index[2] = self.mDofInfo.DOFindex(self.mHandBones[j]->child()->treeIndex(),0);
		index[3] = self.mDofInfo.DOFindex(self.mHandBones[j]->child()->child()->treeIndex(),0);
		if(self.dofNum==15){
			varindex[0]=j*3;
			varindex[1]=j*3+1;
			varindex[2]=j*3+2;
			varindex[3]=j*3+2;
		}
		else if(self.dofNum==5){
			if(j==0)
			{
				varindex[0]=j*3;
				varindex[1]=j*3+1;
				varindex[2]=j*3+2;
				varindex[3]=j*3+2;
			}
			else
			{
				varindex[0]=3;
				varindex[1]=4;
				varindex[2]=4;
				varindex[3]=4;
			}
		}
		else if(self.dofNum==2)
		{
			if(j==0)
			{
				varindex[0]=-1;
				varindex[1]=-1;
				varindex[2]=-1;
				varindex[3]=-1;
			}
			else
			{
				varindex[0]=0;
				varindex[1]=1;
				varindex[2]=1;
				varindex[3]=1;
			}
		}
		else Msg::error("unknown set of variables");

	}
#define NUM_DEBUG_SKIN 50
	static PLDPrimSkin* skins[NUM_DEBUG_SKIN]={NULL,NULL};
	static void drawDebugInfo(int i, HandIKsolver const& self, vectorn const& dofOrigin,bool debug_draw)
	{
#ifndef NO_DEBUG_GUI
		if(debug_draw == false)
			return;
		if(i<NUM_DEBUG_SKIN)
		{
			PLDPrimSkin*& skin=skins[i];

			if (skin==NULL) 
				skin=RE::createSkin(self.mSkeleton);
			skin->SetTranslation(0,0,50*i+50);
			skin->setPoseDOF(dofOrigin,self.mDofInfo);
		}
#endif
	}
	static void drawDebugFingerEE(HandIKsolver const& self, transf const& transfSave)
	{
#ifndef NO_DEBUG_GUI
	   	if(1)
			{
				RE::moveEntity(RE::createEntity("metric","sphere1010.mesh"),vector3(1,1,1), self.con[0]+vector3(1,0,0));
				RE::moveEntity(RE::createEntity("metric1","sphere1010.mesh"),vector3(1,1,1),self.con[1]+vector3(1,0,0));
				RE::moveEntity(RE::createEntity("metric2","sphere1010.mesh"),vector3(1,1,1),self.con[2]+vector3(1,0,0));
				RE::moveEntity(RE::createEntity("metric3","sphere1010.mesh"),vector3(1,1,1),self.con[3]+vector3(1,0,0));
				RE::moveEntity(RE::createEntity("metric4","sphere1010.mesh"),vector3(1,1,1),self.con[4]+vector3(1,0,0));
			}
			else
			{
				for(int j=0;j<self.mEffectors.size();j++)
				{
					vector3 Vec = self.mEffectors[j].bone->getFrame().toGlobalPos(self.mEffectors[j].localpos);
					TString id;
					id.format("metric%d", j);
					RE::moveEntity(RE::createEntity(id,"sphere1010.mesh"),vector3(1,1,1), transfSave*Vec+vector3(1,0,0));
				}
			}
#endif
	}
	void HandIKsolver::_handIK(vector3N const& conpos, quaterN const& conori, vectorn const& importance,  vectorn& r)
	{
		//std::cout<<"<<--------------[[_handIK start]]------------>>"<<std::endl;
		HandIKsolver& self=*this;

		mSkeleton.getPoseDOF(dofOrigin);
		double damp_ratio = options(self).damp_ratio; 
		int max_iter = options(self).max_iter;
		TFtemp = MotionDOF::rootTransformation(dofOrigin);

		//std::cout<<"<<-----[[damp loop START]]----->>"<<std::endl;
		int debugc=0;
		for(int i=0; i<max_iter ;i++)
		{
			mSkeleton.setPoseDOF(dofOrigin);
			quater wrist_save(mSkeleton.fkSolver().local(limbIK->mEffectors[0].bone->treeIndex()).rotation);

			// difference between the current pose and the contraint positions
			if (i==0)
			{
				transfSave=calcDelta(*this, conpos);	
				transfSave.scale(damp_ratio);

				transf newWrist=transfSave*limbIK->mEffectors[0].bone->getFrame();
				wrist_conpos(0)=newWrist.translation;
				wrist_conori(0)=newWrist.rotation;
			
				//RE::moveEntity(RE::createEntity("wonderand","axes.mesh"),vector3(0.5,0.5,0.5),newWrist.rotation, newWrist.translation+vector3(100,0,0));

				//RE::moveEntity(RE::createEntity("wonderand","axes.mesh"),vector3(5,5,5),limbIK->mEffectors[0].bone->getFrame().rotation,limbIK->mEffectors[0].bone->getFrame().translation);
				limbIK->IKsolve3(dofOrigin,TFtemp,wrist_conpos,wrist_conori,wrist_impo);
				mSkeleton.setPoseDOF(dofOrigin);
			}


			drawDebugInfo(debugc++, *this, dofOrigin,options(self).debug_draw);

			// reduce wrist tension
			for (int sub_iter=0; sub_iter<3; sub_iter ++)
			{
				quater q_wrist_current=mSkeleton.fkSolver()._local(limbIK->mEffectors[0].bone->treeIndex()).rotation;
				mSkeleton.fkSolver()._local(limbIK->mEffectors[0].bone->treeIndex()).rotation.safeSlerp(q_wrist_current, wrist_save, options(self).damp_ratio_wrist);
				mSkeleton.fkSolver().forwardKinematics();
				mSkeleton.fkSolver().getPoseDOFfromGlobal(dofOrigin);

				// resolve ik without rotating wrist
				transfSave=calcDelta(*this, conpos, true);	
				transfSave.scale(damp_ratio);
				transf newWrist=transfSave*limbIK->mEffectors[0].bone->getFrame();
				wrist_conpos(0)=newWrist.translation;
				wrist_conori(0)=newWrist.rotation;
				limbIK->IKsolve3(dofOrigin,TFtemp,wrist_conpos,wrist_conori,wrist_impo);
				mSkeleton.setPoseDOF(dofOrigin);
				drawDebugInfo(debugc++,*this, dofOrigin,options(self).debug_draw);
			}	

			if (1)
			{
				dofNum=2;
				fingerIK(*this);
				dofNum=5;
				fingerIK(*this);
				dofNum=15;
				fingerIK(*this);
			}
			drawDebugInfo(debugc++,*this, dofOrigin,options(self).debug_draw);
		}

		//std::cout<<"thumb finger degree "<<TO_DEGREE(dofOrigin(mDofInfo.DOFindex(mHandBones[0]->treeIndex(),0)))<<std::endl;
		//std::cout<<"index finger degree "<<TO_DEGREE(dofOrigin(mDofInfo.DOFindex(mHandBones[1]->child()->treeIndex(),0)))<<std::endl;
		//std::cout<<"middle finger degree "<<TO_DEGREE(dofOrigin(mDofInfo.DOFindex(mHandBones[2]->child()->treeIndex(),0)))<<std::endl;
		//std::cout<<"ring finger degree "<<TO_DEGREE(dofOrigin(mDofInfo.DOFindex(mHandBones[3]->child()->treeIndex(),0)))<<std::endl;
		//std::cout<<"little finger degree "<<TO_DEGREE(dofOrigin(mDofInfo.DOFindex(mHandBones[4]->child()->treeIndex(),0)))<<std::endl;
		//std::cout<<"<<-----[[damp loop END]]----->>"<<std::endl;
	
	//	mSkeleton.setPoseDOF(dofOrigin);
		//std::cout<<"<<--------------[[_handIK END]]------------>>"<<std::endl;
	}

	void HandIKsolver::IKsolve(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance)
	{
		vectorn importance_wrist(wrist_num);
		importance_wrist.setAllValue(1);
		IKsolve(poseInout, newRootTF,  conpos,  conori, importance,  importance_wrist);
	}
	void HandIKsolver::IKsolve(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance, vectorn const& importance_wrist)
	{
		//origRootTF = MotionDOF::rootTransformation(poseInout);
		vectorn r(mEffectors.size());
		
		for(int i=0;i<wrist_num;i++)
		{
			wrist_conpos(i) = limbIK->mEffectors[i].bone->getFrame().toGlobalPos(vector3(0,0,0));
			wrist_conori(i) = limbIK->mEffectors[i].bone->getFrame().rotation;
			wrist_impo(i)=importance_wrist(i);				
		}
	
		MotionDOF::setRootTransformation(poseInout,newRootTF);	
		mSkeleton.setPoseDOF(poseInout);

		poseTemp = poseInout;TFtemp = newRootTF;con=conpos;impor=importance;
	///
		_handIK(conpos, conori, importance,r);
	///	
		poseInout = dofOrigin;
	}

