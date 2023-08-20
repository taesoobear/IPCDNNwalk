// Motion.cpp: implementation of the Motion class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Motion.h"
#include "BVHLoader.h"
#include "postureip.h"
#include "../image/Image.h"
#include "../utility/TArray.h"
#include "../utility/configtable.h"
#include "MotionUtil.h"
#include "FootPrint.h"
#include "../utility/tfile.h"
#include "version.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

MotionView Motion::range(int start, int end, int step)
{
	int nSize;
	if(step==1)
		nSize=(end-start);
	else
		nSize=(end-start+1)/step;

	Posture** ptr;
	int stride, n, on;
	_getPrivate(ptr, stride, n, on);
	return MotionView(ptr+start*stride, nSize, step*stride, mInfo);
}

const MotionView Motion::range(int start, int end, int step) const
{
	int nSize;
	if(step==1)
		nSize=(end-start);
	else
		nSize=(end-start+1)/step;

	Posture** ptr;
	int stride, n, on;
	_getPrivate(ptr, stride, n, on);
	return MotionView(ptr+start*stride, nSize, step*stride, mInfo);
}


Motion::Motion(MotionLoader* pSource)
:_TVector<Posture>()
{
	Init(pSource);
}

Motion::Motion(const Motion& srcMotion, int startFrame, int endFrame)
{
	Init(srcMotion, startFrame, endFrame);
}
void Motion::__initFromMotionDOF(const MotionDOF& srcMotion, int startFrame, int endFrame)
{
	if(startFrame<0) startFrame=0;
	if(endFrame>srcMotion.numFrames()) endFrame=srcMotion.numFrames();

	float fFrameTime=1.0/srcMotion.mInfo.frameRate();
	MotionLoader * skel=(MotionLoader*)&srcMotion.mInfo.skeleton();
	InitEmpty(skel, endFrame-startFrame, fFrameTime);

	for(int i=startFrame; i<endFrame; i++){
		skel->setPoseDOF(srcMotion.row(i));
		skel->getPose(pose(i));
	}
}

Motion::Motion(const MotionDOF& srcMotion, int startFrame, int endFrame)
{
	__initFromMotionDOF(srcMotion, startFrame, endFrame);
}

Motion::Motion(const MotionDOFcontainer& srcMotion, int startFrame, int endFrame)
{
	__initFromMotionDOF(srcMotion.mot, startFrame, endFrame);
	MotionUtil::SetSignal ss(*this);
	ss.constraint(CONSTRAINT_LEFT_FOOT, srcMotion.conL);
	ss.constraint(CONSTRAINT_RIGHT_FOOT, srcMotion.conR);
	ss.constraint(IS_DISCONTINUOUS, srcMotion.discontinuity);
}


void Motion::setSkeleton(int iframe) const
{
	mInfo.m_pSkeleton->setPose(pose(iframe));
}


void Motion::InitSkeleton(MotionLoader* pSkeleton)
{
	if(mInfo.m_pSkeleton && !(mInfo.m_pSkeleton->numRotJoint()==pSkeleton->numRotJoint() &&
			mInfo.m_pSkeleton->numTransJoint()==pSkeleton->numTransJoint() &&
			mInfo.m_pSkeleton->numBone()==pSkeleton->numBone() ) && numFrames()!=0)
	{
		// has Incompatible motion data
		setSize(0);
	}

	if(pSkeleton)
		changeFactory(pSkeleton->factory()->clone(), false);
	mInfo.m_pSkeleton=pSkeleton;
}

void Motion::InitEmpty(MotionLoader* pSource, int numFrames, float fFrameTime)
{
	InitSkeleton(pSource);
	_Init(numFrames, pSource->numRotJoint(), pSource->numTransJoint(), fFrameTime);

#ifdef _DEBUG
	if(pSource->m_cPostureIP.numFrames())
		ASSERT(isSimilar(fFrameTime, pSource->m_cPostureIP.frameTime()));
#endif
}

void Motion::InitEmpty(const Motion& source, int numFrames)
{
	InitSkeleton(&source.skeleton());
	_Init(numFrames, source.NumJoints(), source.numTransJoints(), source.frameTime());
}

void Motion::Init(MotionLoader* pSource)
{
	InitSkeleton(pSource);

	int frames=pSource->m_cPostureIP.numFrames();
	if(frames)
	{
		_Init(frames, pSource->numRotJoint(), pSource->numTransJoint(), pSource->m_cPostureIP.frameTime());
		// dynamic typechecking을 하지 않는다. 자신의 factory를 사용하기 위해서임.
		cloneFrom(pSource->m_cPostureIP, 0, frames, 0, false);
	}
}

void Motion::Init(const Motion& srcMotion, int startFrame, int endFrame)
{
	if(startFrame<0) startFrame=0;
	if(endFrame>srcMotion.numFrames()) endFrame=srcMotion.numFrames();

	InitEmpty(srcMotion, endFrame-startFrame);
	cloneFrom(srcMotion, startFrame, endFrame,0);

	mInfo.m_strIdentifier=srcMotion.mInfo.m_strIdentifier;
}

void Motion::empty()
{
	setSize(0);
	mInfo.clear();
}

void Motion::Concat(const Motion* pAdd, int startFrame, int endFrame, bool bTypeCheck)
{
	ASSERT(pAdd != NULL);
	if(startFrame < 0 )
		startFrame = 0;
	if(endFrame > pAdd->numFrames())
		endFrame = pAdd->numFrames();

	ASSERT(endFrame>1);
	ASSERT(startFrame<(endFrame-1));

	if(numFrames()==0)
	{
		TString id=mInfo.m_strIdentifier;
		Init(*pAdd);
		mInfo.m_strIdentifier=id;
		return;
	}

	int numFrameOld = numFrames();
	int addMount = endFrame - startFrame;
	int reMount = numFrameOld+addMount;

	Resize(reMount);
	cloneFrom(*pAdd, startFrame, endFrame, numFrameOld,bTypeCheck);
}


int Motion::Parent(int jointIndex) const			{ return dep_GetParentJoint(*mInfo.m_pSkeleton, jointIndex);}
Posture& Motion::pose(int iframe) const
{
	RANGE_ASSERT(iframe>=0 && iframe<numFrames());
	return value(iframe);
}


/*
char* Motion::GetJointName(int jointIndex) const	{ return m_pSkeleton->getBoneByRotJointIndex(jointIndex).NameId;}
int Motion::getRotJointIndexByTreeIndex(char* jointname) const	{ return m_pSkeleton->getRotJointIndexByName(jointname);}
int Motion::getRotJointIndexByTreeIndex(int jointVoca) const		{ return m_pSkeleton->getRotJointIndexByVoca(jointVoca);}	//!< jointVoca는 MotionLoader.h에 enum으로 정의되어 있다.
*/

/*
void Motion::GetSitePositions(vector3N& aPos, int iJoint, bool bGlobal, int startFrame, int endFrame) const
{
	if(startFrame<0) startFrame=0;
	if(endFrame>numFrames()) endFrame=numFrames();

	aPos.setSize(endFrame-startFrame);
	for(int i=startFrame; i<endFrame; i++)
	{
		m_pSkeleton->setPose(pose(i));
		m_pSkeleton->GetSiteBone(iJoint).getTranslation(aPos[i-startFrame]);
	}
	ASSERT(bGlobal);
}
*/
/*
void Motion::GetJointVelocities(matrixn& aVel, int iJoint, bool bGlobal, int startFrame, int endFrame) const
{
	if(startFrame<0) startFrame=0;
	if(endFrame>numFrames()) endFrame=numFrames();

	matrixn aPos;
	GetJointPositions(aPos, iJoint, bGlobal, startFrame-1, endFrame+1);
	if(startFrame==0)
		aPos.pushFront(aPos[1]);

	if(endFrame==numFrames())
		aPos.pushBack(aPos[aPos.rows()-1]);

	ASSERT(aPos.rows()==endFrame-startFrame+2);	// 앞뒤로 하나의 포지션 벡터가 더 붙어있다.

	aVel.derivative(aPos);	// aVel역시 앞뒤로 하나 더 붙어있다.

#define toIndex(i) ((i)-startFrame+1)

	for(int i=startFrame; i<endFrame; i++)
	{
		if(isDiscontinuous(i))
		{
			if(i!=startFrame)
				aVel[toIndex(i-1)].assign(aVel[toIndex(i-2)]);
			aVel[toIndex(i)].assign(aVel[toIndex(i+1)]);
		}
	}
	aVel.popFront();
	aVel.popBack();
}
*/

/*
void Motion::GetJointAccelerations(matrixn& aAcc, int iJoint, bool bGlobal, int startFrame, int endFrame) const
{
	if(startFrame<0) startFrame=0;
	if(endFrame>numFrames()) endFrame=numFrames();

	matrixn aVel;
	GetJointVelocities(aVel, iJoint, bGlobal, startFrame-1, endFrame+1);
	if(startFrame==0)
		aVel.pushFront(aVel[1]);

	if(endFrame==numFrames())
		aVel.pushBack(aVel[aVel.rows()-1]);

	ASSERT(aVel.rows()==endFrame-startFrame+2);	// 앞뒤로 하나의 포지션 벡터가 더 붙어있다.

	aAcc.derivative(aVel);	// aVel역시 앞뒤로 하나 더 붙어있다.

#define toIndex(i) ((i)-startFrame+1)

	for(int i=startFrame; i<endFrame; i++)
	{
		if(isDiscontinuous(i))
		{
			if(i!=startFrame)
				aAcc[toIndex(i-1)].assign(aAcc[toIndex(i-2)]);
			aAcc[toIndex(i)].assign(aAcc[toIndex(i+1)]);
		}
	}
	aAcc.popFront();
	aAcc.popBack();
}
*/
/*


void Motion::GetJointPositions(hypermatrixn& aaPos, const intvectorn& aJointIndex, bool bGlobal
{
	if(startFrame<0) startFrame=0;
	if(endFrame>numFrames()) endFrame=numFrames();

	aaPos.setSize(aJointIndex.size(), endFrame-startFrame, 3);
	for(int i=0; i<aJointIndex.size(); i++)
	{
		GetJointPositions(aaPos[i],aJointIndex[i], bGlobal, startFrame, endFrame);
	}

}

void Motion::GetJointVelocities(hypermatrixn& aaVel, const intvectorn& aJointIndex, bool bGlobal
{
	if(startFrame<0) startFrame=0;
	if(endFrame>numFrames()) endFrame=numFrames();

	aaVel.setSize(aJointIndex.size(), endFrame-startFrame, 3);

	matrixn temp;
	for(int i=0; i<aJointIndex.size(); i++)
	{
		GetJointVelocities(temp,aJointIndex[i], bGlobal, startFrame, endFrame);
		aaVel[i].assign(temp);
	}
}
*/
/*
void Motion::GetSitePositions(hypermatrixn& aaPos, const intvectorn& jointIndex)
{
	aaPos.setSize(jointIndex.size(), numFrames(), 3);

	for(int i=0; i<numFrames(); i++)
	{
		m_pSkeleton->setPose(pose(i));
		for(int j=0; j<jointIndex.size(); j++)
		{
			vector3 site;
			m_pSkeleton->GetSiteBone(jointIndex[j]).getTranslation(site);
			aaPos[j][i].assign(site);
		}
	}
}*/

/*
void Motion::GetImportances(matrixn& aImp)
{
	PostureIP * pMD = m_pMotionData;
	aImp.setSize(pMD->numFrames(), 2);

	//	method 1
	bool* abCutState=new bool[pMD->numFrames()];
	CConstraintMarking cCM(abCutState, this);
	cCM.CalcImportance(3,7);
	cCM.m_pImportanceLeft->Save("limp3_6.bmp");
	delete[] abCutState;
	for(int i=0; i<pMD->numFrames(); i++)
	{
		aImp[i][0]=cCM.m_afImportanceLeft[i];
		aImp[i][1]=cCM.m_afImportanceRight[i];
	}

	// method 2
	for(int i=0; i<pMD->numFrames(); i++)
	{
		aImp[i][0]=(int)IsConstraint(i, CONSTRAINT_LEFT_FOOT);
		aImp[i][1]=(int)IsConstraint(i, CONSTRAINT_RIGHT_FOOT);
	}
}*/

/*
void Motion::GetConstraints(matrixn& aCon, const intvectorn& constraintIndex)
{
	PostureIP * pMD = m_pMotionData;
	aCon.setSize(pMD->numFrames(), constraintIndex.size());

	for(int i=0; i<pMD->numFrames(); i++)
	{
		for(int j=0; j<constraintIndex.size(); j++)
			aCon[i][j]=(int)IsConstraint(i,constraintIndex[j]);
	}
}

void Motion::getConstraints(bitvectorn& aCon, int constraint)
{
	PostureIP * pMD = m_pMotionData;
	aCon.setSize(pMD->numFrames());

	for(int i=0; i<pMD->numFrames(); i++)
	{
		aCon.setValue(i, IsConstraint(i,constraint));
	}
}*/

void Motion::_Init(int nkey, int numJoint, int numTransJoint, float fFrameTime)
{
	setSize(nkey);

	for(int i=0; i<nkey; i++)
	{
		pose(i).Init(numJoint, numTransJoint);
		setDiscontinuity(i, false);
	}
//	m_aDiscontinuity.setSize(nkey);
//	m_aDiscontinuity.clearAll();

	mInfo.m_fFrameTime=fFrameTime;
}

void Motion::Resize(int Frames)
{
	Msg::verify(isValid(), "setSkeleton first!");
	int numPosture = numFrames();
	int numJoint = mInfo.m_pSkeleton->numRotJoint();
	int numTransJoint= mInfo.m_pSkeleton->numTransJoint();

	resize(Frames);

	for(int i=numPosture; i<Frames; i++)
		pose(i).Init(numJoint, numTransJoint);

	for(int i=numPosture; i<Frames; i++)
		setDiscontinuity(i, false);
	setDiscontinuity(numPosture, true);
/*	if(numPosture<Frames)
	{
		keyvalue.resize(MIN(m_maxCapacity, Frames));
		m_numFrame=Frames;
		m_aDiscontinuity.resize(MIN(m_maxCapacity, Frames));

	}
	else if (numPosture>Frames)
	{
		keyvalue.resize(MIN(m_maxCapacity, Frames));
		m_numFrame=Frames;
		m_aDiscontinuity.resize(MIN(m_maxCapacity, Frames));
	}*/
}

void Motion::SamplePose(Posture& p, m_real criticalTime) const
{
	//!< 0.5 <=criticalTime< numFrames()-0.5
	// float 0.5 가 정확하게 integer 0에 mapping된다.
	// 즉 0.6등은 0과 1을 0에 가중치를 크게 둬서 섞은게 된다.
	// world좌표와 pixel좌표에 쓰는 float, integer converting scheme을 따랐다.

	int a;
	float t;

    a=(int)floor(criticalTime-0.5);
	t=criticalTime-0.5-(float)a;

    if(t<0.005)
		p=pose(a);
	else if(t>0.995)
		p=pose(a+1);
	else
	{
		if(a<0)
			//p=pose(0);
			p.Blend(pose(a+1), pose(a+2), t-1.0);
		else if(a+1>=numFrames())
			//pose=pose(numFrames()-1);
			p.Blend(pose(a-1), pose(a), t+1.0);
		else
			p.Blend(pose(a), pose(a+1), t);
	}
}

void Motion::samplePose(Posture& p, m_real criticalTime) const
{
	//!< 0 <=criticalTime<= numFrames()-1
	// float 0 이 정확하게 integer 0에 mapping된다.
	int a;
	float t;

	a=(int)floor(criticalTime);
	t=criticalTime-(float)a;

	if(t<0.005)
		p=pose(a);
	else if(t>0.995)
		p=pose(a+1);
	else
	{
		if(a<0)
			//pose=pose(0);
			p.Blend(pose(a+1), pose(a+2), t-1.0);
		else if(a+1>=numFrames())
			//p=pose(numFrames()-1);
			p.Blend(pose(a-1), pose(a), t+1.0);
		else
			p.Blend(pose(a), pose(a+1), t);
	}
}

/*
void Motion::Save(const char* filename)
{
	TFile wfile;
	wfile.OpenWriteFile(filename);
	wfile.PackStr(m_strIdentifier);
	Pack(&wfile, 1);
	wfile.CloseFile();
}

void Motion::Load(const char* filename, MotionLoader* pSkeleton)
{
	ASSERT(m_pSkeleton==NULL);
	ASSERT(m_pMotionData==NULL);

	m_pSkeleton=pSkeleton;
	TFile wfile;
	wfile.OpenReadFile(filename);
	char buffer[100];
	wfile.UnpackStr(buffer);
	m_strIdentifier=buffer;
	m_pMotionData=new PostureIP();
	VERIFY(wfile.UnpackInt()==POSTUREIP);
	Unpack(&wfile, 1);
	wfile.CloseFile();
}
*/

/*
void Motion::GetFootPrints(int start, int end, matrixn& aFootPositions, int constraint, intvectorn& aLeft, intvectorn& aRight) const
{
	intvectorn encoding;

	MotionUtil::CalcFootPrint cfp;
	//MotionUtil::GetFootPrint cfp;
	cfp.getFootPrints(*this, start, end, constraint, encoding, aFootPositions);

	int numGrp=encoding.size()/2;
	aLeft.setSize(numGrp);
	aRight.setSize(numGrp);
	for(int i=0; i<numGrp; i++)
	{
        aLeft[i]=encoding[i*2];
		aRight[i]=encoding[i*2+1];
	}
}*/



bool Motion::IsValid(int startFrame, int endFrame) const
{
	if(!IsValid(startFrame)) return false;
	for(int i=startFrame+1; i<endFrame; i++)
	{
		if(!IsValid(i)) return false;
		if(isDiscontinuous(i)) return false;
		if(isConstraint(i, POSE_IS_INVALID)) return false;
	}
	return true;
}

/*
void Motion::stitchOnline(const Motion &add, int startFrame , int endFrame)
{

	int numFrameOld=numFrames();

	MotionUtil::_reconstructConcat rc;
	MotionUtil::C1stitch st(rc);
	st.stitchOnline(100, *this, add);
}
*/

/*
void Motion::stitch(int nPrevSafe, const Motion &add, int startFrame , int endFrame )
{
	if(!m_pStitch)
		m_pStitch=new MotionUtil::StitchRetarget();

	m_pStitch->stitch(nPrevSafe, *this, add);



}
*/

/*void Motion::changeStitchMethod(MotionUtil::Stitch* pStitch) const
{
	if(m_pStitch)
		delete m_pStitch;
	m_pStitch=pStitch;
}*/

/*
void Motion::GetSegmentMass(vectorn& aSegMass)
{
	//HEAD, LUPPERARM, RUPPERARM, LLOWERARM, RLOWERARM, LHAND, RHAND, PELVIS, TORSO, LTHIGH, RTHIGH, LSHIN ,RSHIN, LFOOT, RFOOT
	// 7.1   3.3                     1.9                 0.6           15.3   22+4  10.5              6           1.5
	//float aMass[NUM_SEGMENT]={7.8, 2.7, 2.7, 2.3, 2.3, 0.6, 0.6, 46.84, 9.9, 9.9,4.6,4.6, 1.4, 1.4};
	float aMass[NUM_SEGMENT]={7.1, 3.3, 3.3, 1.9, 1.9, 0.6, 0.6, 15.3, 26, 10.5, 10.5, 6, 6, 1.5, 1.5};
	aSegMass.setSize(NUM_SEGMENT);
	aSegMass.setValue(aMass);
}

void Motion::GetSegmentLength(vectorn& aLength, hypermatrixn& aaJointPositions)
{
	aLength.setSize(NUM_SEGMENT);
	aLength[HEAD]=aaJointPositions[MotionLoader::HEAD][0].distance(aaJointPositions[MotionLoader::NECK][0])*2;
	aLength[LUPPERARM]=aaJointPositions[MotionLoader::LEFTSHOULDER][0].distance(aaJointPositions[MotionLoader::LEFTELBOW][0]);
	aLength[RUPPERARM]=aaJointPositions[MotionLoader::RIGHTSHOULDER][0].distance(aaJointPositions[MotionLoader::RIGHTELBOW][0]);
	aLength[LLOWERARM]=aaJointPositions[MotionLoader::LEFTWRIST][0].distance(aaJointPositions[MotionLoader::LEFTELBOW][0]);
	aLength[RLOWERARM]=aaJointPositions[MotionLoader::RIGHTWRIST][0].distance(aaJointPositions[MotionLoader::RIGHTELBOW][0]);
	aLength[LHAND]= 0;
	aLength[RHAND]= 0;
	aLength[PELVIS]=aLength[HEAD];
	aLength[TORSO]= aaJointPositions[MotionLoader::HIPS][0].distance(aaJointPositions[MotionLoader::NECK][0]);
	aLength[LTHIGH]= aaJointPositions[MotionLoader::LEFTHIP][0].distance(aaJointPositions[MotionLoader::LEFTKNEE][0]);
	aLength[RTHIGH]= aaJointPositions[MotionLoader::RIGHTHIP][0].distance(aaJointPositions[MotionLoader::RIGHTKNEE][0]);
	aLength[LSHIN]= aaJointPositions[MotionLoader::LEFTANKLE][0].distance(aaJointPositions[MotionLoader::LEFTKNEE][0]);
	aLength[RSHIN]= aaJointPositions[MotionLoader::RIGHTANKLE][0].distance(aaJointPositions[MotionLoader::RIGHTKNEE][0]);
	aLength[LFOOT]= 0;
	aLength[RFOOT]= 0;
}

void Motion::GetSegmentPositions(hypermatrixn& aaSegCOG, hypermatrixn& aaJointPositions)
{
	aaSegCOG.setSize(NUM_SEGMENT, aaJointPositions.rows(), aaJointPositions.cols());

	aaSegCOG[HEAD]=aaJointPositions[MotionLoader::HEAD];
	aaSegCOG[LUPPERARM].op2(m2::AVERAGE,aaJointPositions[MotionLoader::LEFTSHOULDER],aaJointPositions[MotionLoader::LEFTELBOW]);
	aaSegCOG[RUPPERARM].op2(m2::AVERAGE,aaJointPositions[MotionLoader::RIGHTSHOULDER],aaJointPositions[MotionLoader::RIGHTELBOW]);
	aaSegCOG[LLOWERARM].op2(m2::AVERAGE,aaJointPositions[MotionLoader::LEFTWRIST],aaJointPositions[MotionLoader::LEFTELBOW]);
	aaSegCOG[RLOWERARM].op2(m2::AVERAGE,aaJointPositions[MotionLoader::RIGHTWRIST],aaJointPositions[MotionLoader::RIGHTELBOW]);
	aaSegCOG[LHAND]=aaJointPositions[MotionLoader::LEFTWRIST];
	aaSegCOG[RHAND]=aaJointPositions[MotionLoader::RIGHTWRIST];
	aaSegCOG[PELVIS]=aaJointPositions[MotionLoader::HIPS];
	aaSegCOG[TORSO].op2(m2::AVERAGE, aaJointPositions[MotionLoader::CHEST],aaJointPositions[MotionLoader::NECK]);
	aaSegCOG[LTHIGH].op2(m2::AVERAGE, aaJointPositions[MotionLoader::LEFTHIP],aaJointPositions[MotionLoader::LEFTKNEE]);
	aaSegCOG[RTHIGH].op2(m2::AVERAGE, aaJointPositions[MotionLoader::RIGHTHIP],aaJointPositions[MotionLoader::RIGHTKNEE]);
	aaSegCOG[LSHIN].op2(m2::AVERAGE, aaJointPositions[MotionLoader::LEFTANKLE],aaJointPositions[MotionLoader::LEFTKNEE]);
	aaSegCOG[RSHIN].op2(m2::AVERAGE, aaJointPositions[MotionLoader::RIGHTANKLE],aaJointPositions[MotionLoader::RIGHTKNEE]);
	aaSegCOG[LFOOT]=aaJointPositions[MotionLoader::LEFTANKLE];
	aaSegCOG[RFOOT]=aaJointPositions[MotionLoader::RIGHTANKLE];
}*/


/*
void Motion::GetEnergy(vectorn& aEnergy, bool bSmooth)
{
	hypermatrixn aaJointPositions;
	hypermatrixn aaJointRotations;

	intvectorn aJointIndex(MotionLoader::NUM_JOINT_VOCA);
	for(int i=0; i<MotionLoader::NUM_JOINT_VOCA; i++)
	{
		aJointIndex[i]=skeleton().getRotJointIndexByVoca(i);
	}
	GetJointPositions(aaJointPositions, aJointIndex);
	GetJointRotations(aaJointRotations, aJointIndex);

	vectorn aSegMass;
	GetSegmentMass(aSegMass);
	vectorn aLength;
	GetSegmentLength(aLength, aaJointPositions);

	hypermatrixn aaSegCOG;
	GetSegmentPositions(aaSegCOG, aaJointPositions);

	hypermatrixn aaSegCOGvel;
	aaSegCOGvel.setSameSize(aaSegCOG);
	for(int i=0; i<aaSegCOG.page(); i++)	aaSegCOGvel[i].derivative(aaSegCOG[i]);
	matrixn aaSegCOGspeed(aaSegCOGvel.page(), aaSegCOGvel.rows());
	for(int i=0; i<aaSegCOG.page(); i++)
	{
		aaSegCOGspeed[i].lengths(aaSegCOGvel[i]);
		aaSegCOGspeed[i]*=0.5/frameTime();
	}

	hypermatrixn aaSegCOGangVel;
	aaSegCOGangVel.setSize(NUM_SEGMENT, aaJointPositions.rows(), aaJointPositions.cols());

	// quaternion smoothing
	if(bSmooth)
	{
		vectorn kernel;
		float fFrameTime=frameTime();
		int kernel_size=Filter::CalcKernelSize(0.6, fFrameTime);
		Filter::GetGaussFilter(kernel_size, kernel);

		for(int i=0; i<aaJointRotations.page(); i++)
			Filter::LTIFilterQuat(10, kernel, aaJointRotations.page(i));
	}

	aaSegCOGangVel[HEAD].derivativeQuater(aaJointRotations[MotionLoader::HEAD]);
	aaSegCOGangVel[LUPPERARM].derivativeQuater(aaJointRotations[MotionLoader::LEFTSHOULDER]);
	aaSegCOGangVel[RUPPERARM].derivativeQuater(aaJointRotations[MotionLoader::RIGHTSHOULDER]);
	aaSegCOGangVel[LLOWERARM].derivativeQuater(aaJointRotations[MotionLoader::LEFTELBOW]);
	aaSegCOGangVel[RLOWERARM].derivativeQuater(aaJointRotations[MotionLoader::RIGHTELBOW]);
	aaSegCOGangVel[LHAND].derivativeQuater(aaJointRotations[MotionLoader::LEFTWRIST]);
	aaSegCOGangVel[RHAND].derivativeQuater(aaJointRotations[MotionLoader::RIGHTWRIST]);
	aaSegCOGangVel[TORSO].derivativeQuater(aaJointRotations[MotionLoader::HIPS]);
	aaSegCOGangVel[LTHIGH].derivativeQuater(aaJointRotations[MotionLoader::LEFTHIP]);
	aaSegCOGangVel[RTHIGH].derivativeQuater(aaJointRotations[MotionLoader::RIGHTHIP]);
	aaSegCOGangVel[LSHIN].derivativeQuater(aaJointRotations[MotionLoader::LEFTKNEE]);
	aaSegCOGangVel[RSHIN].derivativeQuater(aaJointRotations[MotionLoader::RIGHTKNEE]);
	aaSegCOGangVel[LFOOT].derivativeQuater(aaJointRotations[MotionLoader::LEFTANKLE]);
	aaSegCOGangVel[RFOOT].derivativeQuater(aaJointRotations[MotionLoader::RIGHTANKLE]);

	for(int i=0; i<aaSegCOGangVel.page(); i++)
		aaSegCOGangVel.page(i)*=0.5/frameTime();

	// 0.6초 정도의 kernel size

	vectorn blurFilter;
	Filter::GetGaussFilter(Filter::CalcKernelSize(0.4, frameTime()), blurFilter);

	aEnergy.setSize(aaSegCOGspeed.cols());
	aEnergy.setAllValue(0);
	//aaSegCOGspeed.save("segCOGspeed.txt",false);
	//aaSegCOGangSpeed.save("segCOGangSpeed.txt",false);

	for(int i=0; i<NUM_SEGMENT; i++)
	{
		CString msg;
		aaSegCOGspeed[i].output(msg, "%f", 0, 20);
		TRACE(msg+"\n");
		if(bSmooth)
		{
			Filter::LTIFilter(1, blurFilter, aaSegCOGspeed[i]);
			Filter::LTIFilter(1, blurFilter, aaSegCOGangVel[i]);
		}

		// 1/2 mv^2
		aaSegCOGspeed[i].op1(v1::each(s1::SQUARE), aaSegCOGspeed[i]);
		aaSegCOGspeed[i]*=aSegMass[i]/2.f;

		aEnergy+=aaSegCOGspeed[i];

		// 1/2 Iw^2

		// I(t)=R(t)*I_body*R(t)^T
		// T= w*(I(t)w)/2

		for(int iframe=0; iframe<aaSegCOGangVel.rows(); iframe++)
		{
			quater qR;
			matrix4 I_body;

			// slender rod
			I_body.setIdentityRot();
			// Ixx
			I_body._11=1.f/12.f*aSegMass[i]*aLength[i]*aLength[i];
			// Iyy
			I_body._22=0;
			// Izz
			I_body._33=1.f/12.f*aSegMass[i]*aLength[i]*aLength[i];

			//aSegMass[i]
			vector3 w=aaSegCOGangVel.page(i).row(iframe).toVector3();
			switch(i)
			{
			case HEAD:qR=aaJointRotations.page(MotionLoader::HEAD).row(iframe).toQuater();break;
			case LUPPERARM:qR=aaJointRotations.page(MotionLoader::LEFTSHOULDER).row(iframe).toQuater();break;
			case RUPPERARM:qR=aaJointRotations.page(MotionLoader::RIGHTSHOULDER).row(iframe).toQuater();break;
			case LLOWERARM:qR=aaJointRotations.page(MotionLoader::LEFTELBOW).row(iframe).toQuater();break;
			case RLOWERARM:qR=aaJointRotations.page(MotionLoader::RIGHTELBOW).row(iframe).toQuater();break;
			case LHAND:qR=aaJointRotations.page(MotionLoader::LEFTWRIST).row(iframe).toQuater();break;
			case RHAND:qR=aaJointRotations.page(MotionLoader::RIGHTWRIST).row(iframe).toQuater();break;
			case TORSO:qR=aaJointRotations.page(MotionLoader::HIPS).row(iframe).toQuater();break;
			case LTHIGH:qR=aaJointRotations.page(MotionLoader::LEFTHIP).row(iframe).toQuater();break;
			case RTHIGH:qR=aaJointRotations.page(MotionLoader::RIGHTHIP).row(iframe).toQuater();break;
			case LSHIN:qR=aaJointRotations.page(MotionLoader::LEFTKNEE).row(iframe).toQuater();break;
			case RSHIN:qR=aaJointRotations.page(MotionLoader::RIGHTKNEE).row(iframe).toQuater();break;
			case LFOOT:qR=aaJointRotations.page(MotionLoader::LEFTANKLE).row(iframe).toQuater();break;
			case RFOOT:qR=aaJointRotations.page(MotionLoader::RIGHTANKLE).row(iframe).toQuater();break;
			}

			matrix4 matI;
			quater qRT;
			qRT.inverse(qR);
			matI.mult(I_body, qRT);
			matI.leftMultRotation(qR);
            vector3 Iw;
			Iw.mult(matI, w);
			aEnergy[iframe]+=(w%Iw)/2;
		}
	}
}
*/

/*
void Motion::GetCOGtrajectory(matrixn& aFullbodyCOG, bool bSmooth, int superSample)
{
	hypermatrixn aaJointPositions;

	intvectorn aJointIndex(MotionLoader::NUM_JOINT_VOCA);
	for(int i=0; i<MotionLoader::NUM_JOINT_VOCA; i++)
	{
		aJointIndex[i]=skeleton().getRotJointIndexByVoca(i);
	}
	GetJointPositions(aaJointPositions, aJointIndex);

	if(superSample!=1)
	{
		hypermatrixn aaJP(aaJointPositions);

		aaJointPositions.setSize(aaJP.page(), aaJP.rows()*superSample, aaJP.cols());

		matrixn temp;
		SegmentFinder seg(*this, 0, numFrames());

		for(int joint=0; joint<aaJP.page(); joint++)
		{
			for(int iseg=0; iseg< seg.numSegment(); iseg++)
			{
				int stt=seg.startFrame(iseg);
				int end=seg.endFrame(iseg);

				temp.op1(m1::superSampling(superSample, stt, end), aaJP.page(joint));
				ASSERT(temp.rows()==(end-stt)*superSample);
				aaJointPositions.page(joint).setValue(stt*superSample,0, temp);
			}
		}
	}

	vectorn aSegMass;
	GetSegmentMass(aSegMass);
	vectorn aLength;
	GetSegmentLength(aLength, aaJointPositions);

	hypermatrixn aaSegCOG;
	GetSegmentPositions(aaSegCOG, aaJointPositions);

	aFullbodyCOG.setSize(aaJointPositions.rows(), aaJointPositions.cols());
	aFullbodyCOG.setAllValue(0);

	// center of gravity 계산.
	matrixn aTemp;
	aTemp.setSameSize(aFullbodyCOG);
	float fTotalMass=aSegMass.sum();

	// 0.6초 정도의 kernel size
    int kernel_size=Filter::CalcKernelSize(0.6, frameTime());
	vectorn blurFilter;
	Filter::GetGaussFilter(kernel_size, blurFilter);

	for(int iseg=0; iseg<NUM_SEGMENT; iseg++)
	{
		if(bSmooth)
		{
			Filter::LTIFilter(1, blurFilter, aaSegCOG[iseg]);
		}

		aTemp.mult(aaSegCOG[iseg], aSegMass[iseg]/fTotalMass);
		aFullbodyCOG+=aTemp;
	}
}
*/



/*
void Motion::GetBonePositions(matrixn& aPos, const Bone& bone, bool bGlobal, int startFrame, int endFrame) const
{
	if(startFrame<0) startFrame=0;
	if(endFrame>numFrames()) endFrame=numFrames();

	SegmentFinder seg(*this, startFrame, endFrame);

	aPos.setSize(endFrame-startFrame, 3);

    for(int iseg=0; iseg< seg.numSegment(); iseg++)
	{
		int stt=seg.startFrame(iseg);
		int end=seg.endFrame(iseg);

		for(int i=stt; i<end; i++)
		{
			m_pSkeleton->setPose(pose(i));
			vector3 pos;
			bone.getTranslation(pos);
			aPos[i-startFrame].assign(pos);
		}
	}
}

void Motion::GetBoneVelocities(matrixn& aVel, const Bone& bone, bool bGlobal, int startFrame, int endFrame, float fSmoothKernel) const
{
	if(startFrame<0) startFrame=0;
	if(endFrame>numFrames()) endFrame=numFrames();

	SegmentFinder seg(*this, startFrame, endFrame);

	aVel.setSize(endFrame-startFrame, 3);

	matrixn aSegPos;
	matrixn aSegVel;
    for(int iseg=0; iseg< seg.numSegment(); iseg++)
	{
		int stt=seg.startFrame(iseg);
		int end=seg.endFrame(iseg);

		GetBonePositions(aSegPos, bone, bGlobal, stt, end);
		aSegVel.derivative(aSegPos);

		if(fSmoothKernel!=0.f)
			aSegVel.op0(m0::useUnary(m1::filter(KernelSize(fSmoothKernel))));

		aVel.setValue(stt-startFrame, 0, aSegVel);
	}
}

void Motion::GetBonePosVel(matrixn& aPos, matrixn& aVel, const Bone& bone, bool bGlobal, int startFrame, int endFrame, float fSmoothKernel) const
{
	if(startFrame<0) startFrame=0;
	if(endFrame>numFrames()) endFrame=numFrames();

	SegmentFinder seg(*this, startFrame, endFrame);

	aVel.setSize(endFrame-startFrame, 3);
	aPos.setSize(endFrame-startFrame, 3);

	matrixn aSegPos;
	matrixn aSegVel;
    for(int iseg=0; iseg< seg.numSegment(); iseg++)
	{
		int stt=seg.startFrame(iseg);
		int end=seg.endFrame(iseg);

		GetBonePositions(aSegPos, bone, bGlobal, stt, end);
		aSegVel.derivative(aSegPos);
		aSegVel/=2*frameTime();

		if(fSmoothKernel!=0.f)
			aSegVel.op0(m0::useUnary(m1::filter(KernelSize(fSmoothKernel))));

		aPos.setValue(stt-startFrame, 0, aSegPos);
		aVel.setValue(stt-startFrame, 0, aSegVel);
	}
}

*/

int Motion::isConstraint(int start, int end, int eConstraint) const
{
	for(int i=start; i<end; i++)
		if(isConstraint(i, eConstraint)) return i;
	return -1;
}

void Motion::exportMOT(const char* filename) const
{
	BinaryFile bf;
	bf.openWrite(filename, true);

	skeleton().pack(bf, MOT_RECENT_VERSION);
	_pack(bf, 5); // uses verion 5

	bf.close();
}
void Motion::exportBinary(const char* filename) const
{
	BinaryFile bf;
	bf.openWrite(filename, true);
	bf.packInt(MOT_RECENT_VERSION); // version 6
	_pack(bf, MOT_RECENT_VERSION);
	bf.close();
}

void Motion::importBinary(const char* filename) 
{
	BinaryFile bf;
	bf.openRead(filename);

	int version=bf.unpackInt();
	int type=bf.unpackInt();
	Msg::verify(type==POSTUREIP,"%s is not a valid mot2 file\n", filename);
	_unpack(bf, version);
	bf.close();

}
/*
void Motion::exportANIM( const char* filename) const
{
	BinaryFile bf(true, filename);
	bf.packInt(MOT_RECENT_VERSION);// version 1.
	_pack(bf, MOT_RECENT_VERSION);
	bf.close();
}
*/

void Motion::ChangeCoord(int eCoordinate)
{
	MotionLoader* pSkeleton=mInfo.m_pSkeleton;
	if(eCoordinate==FIRST_ARRANGED_COORD)
	{
		vector3 vec(0.0f,0.0f,0.0f);
		quater ori(1.0f,0.0f,0.0f,0.0f);
		pose(0).m_rotAxis_y = ori;
		pose(0).m_aRotations[0] = ori * pose(0).m_offset_q;
		pose(0).m_aTranslations[0] = vec;
		pose(0).m_aTranslations[0].y = pose(0).m_offset_y;
		ReconstructDataByDifference();
	}
	else if(eCoordinate==FIXED_COORD)
	{
		for(int i=0; i<numFrames(); i++)
		{
			quater q, rotY, rotXZ;
			pose(i).m_aRotations[0].decomposeTwistTimesNoTwist(vector3(0,1,0), rotY, rotXZ);
			pose(i).m_aTranslations[0].x=0;
			pose(i).m_aTranslations[0].z=0;
			pose(i).m_aRotations[0]=rotXZ;
		}
	}
	else if(eCoordinate==LOCAL_COORD)
	{
		for(int i=0; i<numFrames(); i++)
		{
			quater q, rotY, rotXZ;
			pose(i).m_aTranslations[0].x=0;
			pose(i).m_aTranslations[0].y=0;
			pose(i).m_aTranslations[0].z=0;
			pose(i).m_aRotations[0].identity();
		}
	}
	else if(eCoordinate==GLOBAL_COORD)
	{
		pose(0).m_aTranslations[0].y=pose(0).m_offset_y;
		ReconstructDataByDifference();
	}
	else if(eCoordinate==PELVIS_LOCAL_COORD || eCoordinate==PELVIS_FIXED_COORD)
	{
		Bone& pelvis=skeleton().getBoneByVoca(MotionLoader::HIPS);
		int pelvisIndex=skeleton().getRotJointIndexByVoca(MotionLoader::HIPS);
		for(int i=0; i<numFrames(); i++)
		{
			setSkeleton(i);
			for(int j=0; j<pelvisIndex; j++)
			{
				pose(i).m_aRotations[j].identity();
				pose(i).m_aTranslations[j].setValue(0,0,0);
			}

			if(eCoordinate==PELVIS_LOCAL_COORD)
			{
				pose(i).m_aRotations[pelvisIndex].identity();
				pose(i).m_aTranslations[pelvisIndex].setValue(0,0,0);
			}
			else
			{
				quater q, rotY, rotXZ;
				pelvis.getRotation(pose(i).m_aRotations[pelvisIndex]);
				pelvis.getTranslation(pose(i).m_aTranslations[pelvisIndex]);
				pose(i).m_aRotations[pelvisIndex].decomposeTwistTimesNoTwist(vector3(0,1,0), rotY, rotXZ);
				pose(i).m_aTranslations[pelvisIndex].x=0;
				pose(i).m_aTranslations[pelvisIndex].z=0;
				pose(i).m_aRotations[pelvisIndex]=rotXZ;
			}
		}
	}

}

void Motion::CalcInterFrameDifference(int stFrm)
{
	if(numFrames()==0)
		return;
	int i;

	ASSERT(stFrm<numFrames());
	ASSERT(stFrm>-1) ;

	///////////////////////////////////////////////////////////////////////////////
	//  calculation for frame stFrm
	///////////////////////////////////////////////////////////////////////////////
	pose(stFrm).decomposeRot();

	for(i=stFrm+1; i<numFrames(); i++)
	{
		///////////////////////////////////////////////////////////////////////////////
		//  calculation m_dv and offset_y
		///////////////////////////////////////////////////////////////////////////////
		pose(i).m_dv = (pose(i).m_aTranslations[0] - pose(i-1).m_aTranslations[0]);
		pose(i).m_dv.y = 0;

		quater inv_q;
		inv_q.inverse(pose(i-1).m_rotAxis_y);
		pose(i).m_dv.rotate(inv_q,pose(i).m_dv);
		pose(i).m_offset_y = pose(i).m_aTranslations[0].y;

		///////////////////////////////////////////////////////////////////////////////
		//  calculation rotAxis_y	pose(i).m_aRotations[0] = m_rotAxis_y*m_offset_q
		//							pose(i).m_dq * pose(i-1).m_rotAxis_y = pose(i).m_rotAxis_y
		//			  		  thus, pose(i).m_dq = pose(i).m_rotAxis_y * (pose(i-1).m_rotAxis_y)^(-1)
		///////////////////////////////////////////////////////////////////////////////

		pose(i).decomposeRot();




		pose(i).m_dq.mult(pose(i).m_rotAxis_y, inv_q);
		pose(i).m_dq.align(quater(1,0,0,0));	// m_dq should be small.

	}
}

void Motion::_reconstructRotByDifference(int iframe, bool bForward)
{
	ASSERT(bForward);

	if(iframe<0) iframe=0;
	if(iframe > numFrames()-1) return;
	int i;

//	pose(iframe).m_aRotations[0].mult(pose(iframe).m_rotAxis_y, pose(iframe).m_offset_q);
	pose(iframe).decomposeRot();

	for(i=iframe+1; i<numFrames(); i++)
	{
		//	pose(i).m_dq * pose(i-1).m_rotAxis_y = pose(i).m_rotAxis_y
		pose(i).m_rotAxis_y.mult(pose(i).m_dq, pose(i-1).m_rotAxis_y);
		pose(i).m_rotAxis_y.normalize();
		pose(i).m_aRotations[0].mult(pose(i).m_rotAxis_y, pose(i).m_offset_q);
		pose(i).m_aRotations[0].normalize();
	}
}

void Motion::_reconstructPosByDifference(int iframe, bool bForward)
{
	if(bForward)
	{
		if(iframe > numFrames()-1) return;
		int i;

		/* backup
		vector3 prev_v, dv;
		prev_v = keyvalue[iframe].m_aTranslations[0];

		for(i=iframe+1; i<numFrames(); i++)
		{
			dv.rotate(keyvalue[i-1].m_rotAxis_y, keyvalue[i].m_dv);
			prev_v = prev_v+dv;
			keyvalue[i].m_aTranslations[0] = prev_v;
			keyvalue[i].m_aTranslations[0].y = keyvalue[i].m_offset_y;
		}
		*/
		// simplified and optimized
		vector3 dv;
		for(i=iframe+1; i<numFrames(); i++)
		{
			dv.rotate(pose(i-1).m_rotAxis_y, pose(i).m_dv);
			pose(i).m_aTranslations[0].add(pose(i-1).m_aTranslations[0],dv);
			pose(i).m_aTranslations[0].y = pose(i).m_offset_y;
		}
	}
	else
	{
		// 위의 식을 거꾸로 푼다.
		if(iframe <0) return;
		int i;

		vector3 dv;
		for(i=iframe-1; i>=0; i--)
		{
			dv.rotate(pose(i).m_rotAxis_y, pose(i+1).m_dv);
			// pose(i+1).m_aTranslations[0] = pose(i).m_aTranslations[0]+dv; ->거꾸로
			pose(i).m_aTranslations[0].sub(pose(i+1).m_aTranslations[0] ,dv);
			pose(i).m_aTranslations[0].y = pose(i).m_offset_y;
		}
	}
}


/*
void PostureIP::Smooth(int numIter, bool bTrans)
{
	D3DXQUATERNION* aQuat = new D3DXQUATERNION[numFrames()];

	D3DXVECTOR3* aVec3 = new D3DXVECTOR3[numFrames()];

	for(int i=0; i<numFrames(); i++)
		aVec3[i]=pose(i)->m_aTranslations[0];

	if(bTrans)
	{
		Filter::Smoothing(numIter, numFrames(), aVec3);

		for(i=0; i<numFrames(); i++)
			pose(i)->m_aTranslations[0]=aVec3[i];
	}


	for(int j=0; j<m_numJoint; j++)
	{
		for(int i=0; i<numFrames(); i++)
			aQuat[i]=pose(i)->m_aRotations[j];

		Filter::AlignUnitQuaternions(numFrames(), aQuat);
		Filter::Smoothing(numIter, numFrames(), aQuat);

		for(i=0; i<numFrames(); i++)
			pose(i)->m_aRotations[j]=aQuat[i];
	}

	delete[] aQuat;
	delete[] aVec3;
}

void PostureIP::LTIFilter(int numIter, const vectorn& kernel)
{
	LTIFilter(numIter, kernel, numIter, kernel);

}

void PostureIP::LTIFilter(int numIterA, const vectorn& kernelA, int numIterB, const vectorn& kernelB)
{
	D3DXQUATERNION* aQuat = new D3DXQUATERNION[numFrames()];
	D3DXVECTOR3* aVec3 = new D3DXVECTOR3[numFrames()];

	for(int i=0; i<numFrames(); i++)
		aVec3[i]=pose(i)->m_aTranslations[0];

	Filter::LTIFilter(numIterA, kernelA, numFrames(), aVec3);

	for(i=0; i<numFrames(); i++)
		pose(i)->m_aTranslations[0]=aVec3[i];


	for(int j=0; j<m_numJoint; j++)
	{
		for(int i=0; i<numFrames(); i++)
			aQuat[i]=pose(i)->m_aRotations[j];

		Filter::AlignUnitQuaternions(numFrames(), aQuat);
		Filter::LTIFilter(numIterB, kernelB, numFrames(), aQuat);

		for(i=0; i<numFrames(); i++)
			pose(i)->m_aRotations[j]=aQuat[i];
	}

	delete[] aQuat;
	delete[] aVec3;
}
*/


/*
void PostureIP::Scale(float ratio)
{
	//!< MotionData의 root translation을 scale한다.
	for(int i=0; i<numFrames(); i++)
	{
		(pose(i)->m_aTranslations[0])*=ratio;
	}
}

void PostureIP::Translate(const vector3& trans)
{
	//!< MotionData 전체를 Translate한다. 즉 root position의 translation
	for(int i=0; i<numFrames(); i++)
	{
		pose(i)->m_aTranslations[0]+=trans;
	}
}

void PostureIP::translate(int start, int end, const vector3& trans)
{
	for(int i=start; i<end; i++)
	{
		pose(i).m_aTranslations[0]+=trans;
	}
}

void PostureIP::rotate(int start, int end, const quater& q) //!< MotionData 전체를 Rotate한다. root orientation quat앞에 곱해져서 제일 나중에
{
	for(int i=start; i<end; i++)
	{
		pose(i).m_aRotations[0].leftMult(q);
		pose(i).m_aTranslations[0].rotate(q, pose(i).m_aTranslations[0]);
	}
}
*/

void Motion::setConstraint(int fr, int con, bool bSet)
{
	if(fr<0 || fr>numFrames()-1) return;

	ASSERT(con>=0);
	ASSERT(con<NUM_CONSTRAINT);

	if(bSet)
		pose(fr).constraint.SetAt(con);	// Set Constraint
	else
		pose(fr).constraint.ClearAt(con);// Release Constraint
	return;
}

bool Motion::isConstraint(int fr, int con) const
{
	RANGE_ASSERT(fr >= 0 && fr<numFrames());

	return pose(fr).constraint[con];
}


void Motion::_pack(BinaryFile& bf, int version) const
{
	//Msg::verify(m_numFrame < m_maxCapacity, "Error _pack");

	bf.packInt(POSTUREIP);
	bf.pack(NULL);
	//(NameId);
	//Node::pack(bf, version);

	bf.packInt(numFrames());
	bf.packInt(NumJoints());
	bf.packFloat(mInfo.m_fFrameTime);

	bf.pack(getDiscontinuity());

	if(version<6)
	{
		for(int i=0; i<numFrames(); i++)
			pose(i).pack(bf, version);
	}
	else
	{
		bf.packInt(numRotJoints());
		bf.packInt(numTransJoints());
		for(int i=0; i<numFrames(); i++)
		{
			bf.pack(pose(i).m_aTranslations);
			bf.pack(pose(i).m_aRotations);
			bf.pack(pose(i).constraint);
			bf.pack(pose(i).m_additionalLinear);
			bf.pack(pose(i).m_additionalQuater);
		}
	}

	if(version<3)
	{
		for(int i=0; i<NumJoints(); i++)
			//bf.packInt(m_aTargetIndex[i]);
			bf.packInt(-1);
	}
}

void Motion::_unpack(BinaryFile& bf, int version)
{
	//Node::unpack(bf, version);
	TString temp;
	bf.unpack(temp);


	int nkey=bf.unpackInt();
	int njoint=bf.unpackInt();
	mInfo.m_fFrameTime=bf.unpackFloat();

	_Init(nkey, njoint, 1, mInfo.m_fFrameTime);

	bitvectorn discontinuity;
	bf.unpack(discontinuity);

	if(version<6)
	{
		for(int i=0; i<numFrames(); i++)
			pose(i).unpack(bf, version);
	}
	else
	{
		int nRotJoint=bf.unpackInt();
		int nTransJoint=bf.unpackInt();
		for(int i=0; i<numFrames(); i++)
		{
			pose(i).Init(nRotJoint, nTransJoint);
			bf.unpack(pose(i).m_aTranslations);
			bf.unpack(pose(i).m_aRotations);
			bf.unpack(pose(i).constraint);
			bf.unpack(pose(i).m_additionalLinear);
			bf.unpack(pose(i).m_additionalQuater);
		}
	}

	setDiscontinuity(discontinuity);

	if(version<3)
	{
	int temp;
	for(int i=0; i<njoint; i++)
		//m_aTargetIndex[i]=bf.unpackInt();
		temp=bf.unpackInt();
	}
}

//사용법: void Motion::cloneFrom(const Motion& other, int otherStartFrame, int otherEndFrame, int thisStart, bool bTypeCheck=true)
void Motion::cloneFrom(const Motion& other, int startFrame, int endFrame, int targetStart, bool bTypeCheck)
{
	int targetEnd=targetStart+endFrame-startFrame;

	ASSERT(numFrames()>=targetEnd);
	for(int i=targetStart; i<targetEnd; i++)
	{
		if(bTypeCheck)
			pose(i).Clone(&other.pose(i-targetStart+startFrame));
		else
			pose(i)._clone(&other.pose(i-targetStart+startFrame));

		if(other.isDiscontinuous(i-targetStart+startFrame))
			setDiscontinuity(i, true);
	}
}

void Motion::setPose(int iframe, const Posture& pose)
{
	ASSERT(pose.numRotJoint()==NumJoints());
	value(iframe)=pose;
}

/*
void Motion::changeFactory(TFactory<Posture>* pFactory)
{
	keyvalue.changeFactory(pFactory);
}*/

bool Motion::isDiscontinuous(int fr) const
{
	return pose(fr).constraint[IS_DISCONTINUOUS];
}
//			{ return m_aDiscontinuity[fr%m_maxCapacity];}
void Motion::setDiscontinuity(int fr, bool value)
{
	pose(fr).constraint.Assign(IS_DISCONTINUOUS, value);
}//	{ m_aDiscontinuity.setValue(fr%m_maxCapacity, value);}


boolN Motion::getDiscontinuity() const
{
	boolN t;
	t.resize(numFrames());
	for(int i=0; i<numFrames(); i++)
		t.set(i, isDiscontinuous(i)); return t;
}

void Motion::setDiscontinuity(bitvectorn const& bit)
{
	ASSERT(bit.size()==numFrames());
	for(int i=0; i<numFrames(); i++)
		setDiscontinuity(i, bit[i]);
}
