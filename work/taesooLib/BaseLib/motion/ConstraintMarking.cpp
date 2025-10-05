// ConstraintMarking.cpp: implementation of the CConstraintMarking class.
//
//////////////////////////////////////////////////////////////////////


#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "../BaseLib/math/hyperMatrixN.h"
#include "ConstraintMarking.h"
#include "MotionUtil.h"
#include "Motion.h"
#include "../BaseLib/image/Image.h"
#include "../BaseLib/image/ImagePixel.h"
#include "../BaseLib/image/ImageProcessor.h"
#include "../BaseLib/utility/configtable.h"
#include "../BaseLib/motion/MotionLoader.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


namespace MotionUtil
{
	//////////////////////////////////////////////////////////////////////
	// Construction/Destruction
	//////////////////////////////////////////////////////////////////////

	FootstepDetection::FootstepDetection(Motion* pMotion , bool bCleanup, bool bFillGap, int minConDuration, int reduceCon)
	{
		m_bCleanup=bCleanup;
		m_pAccurateMotion=pMotion;
		m_bFillGap=bFillGap;
		m_minConDuration=minConDuration;
		m_reduceCon=reduceCon;

		m_abCon.resize(m_pAccurateMotion->numFrames());
	}

	void FootstepDetection::calcConstraint(Bone& bone, int con, float height_thr, float speed_thr, const char* outputImage)
	{
		// importance계산
		int numJoint=m_pAccurateMotion->NumJoints();
		int numPosture=m_pAccurateMotion->numFrames();


		float fFrameTime=m_pAccurateMotion->frameTime();

		matrixn posL, velL;

		posL.setSize(numPosture, 3);
		velL.setSize(numPosture, 3);

		MotionLoader& skel=m_pAccurateMotion->skeleton();

		MotionUtil::GetSignals sig(*m_pAccurateMotion);
		sig.jointPosVel(bone, posL, velL);

		

		m_aSpeed.setSize(numPosture);
		m_aHeight.setSize(numPosture);
		

		for (int i=0; i<numPosture; i++)
			m_aSpeed[i]=velL.row(i).length();
		posL.getColumn(1, m_aHeight);

		m_real min_height=m_aHeight.minimum();

		height_thr+=min_height;

		calcConstraintSep(m_pAccurateMotion, height_thr, speed_thr, posL, m_aSpeed, m_abCon, m_bCleanup, m_minConDuration, m_reduceCon, m_bFillGap);

		
		if(outputImage)
		{
			matrixn speedHeight(2, m_aHeight.size());
			speedHeight.row(0)=m_aSpeed;
			speedHeight.row(1)=m_aHeight;

			vectorn min(2);
			vectorn max(2);
			vectorn aY(2);
			min[0]=0.f;
			max[0]=speed_thr* 2.f;
			min[1]=min_height;
			max[1]=min_height+(height_thr-min_height)*2.f;
			aY[0]=speed_thr;
			aY[1]=height_thr;
			Imp::ChangeChartPrecision(50);
			
			CImageProcessor::SaveAndDeleteImage(
				CImageProcessor::DrawChart(speedHeight, CImageProcessor::LINE_CHART, min, max, aY.dataPtr()), outputImage);			
			Imp::DefaultPrecision();

		}

		for(int i=0; i<m_abCon.size(); i++)
			m_pAccurateMotion->setConstraint(i, con, m_abCon[i]);
	}

	
	void FootstepDetection::fillGap(bitvectorn& ab, int numPosture, int interval)
	{
		bitvectorn original;
		original.resize(numPosture);

		for(int i=0; i<numPosture; i++)
		original.set(i,ab[i]);

		int j;
		int gapCount;
		// interval frame이하의 간격은 붙여준다.
		for(int i=0; i<numPosture; i++)
		{
			if(original[i]==false)
			{
				j=original.find(i);
				if(j==numPosture) break;
				gapCount=j-i;
				if(gapCount<interval)
				{
					for(int k=0; k<gapCount; k++)
					{
						ab.setAt(i+k);
					}
				}
				i=j-1;
			}
		}	
	}

	void FootstepDetection::fillGapUsingPos(bitvectorn &ab, int numPosture, const matrixn& pos, int interval, float thr)
	{
		bitvectorn original;
		original.resize(numPosture);

		for(int i=0; i<numPosture; i++)
		original.set(i,ab[i]);

		int gapCount,j;
		// interval frame이하의 간격은 붙여준다.
		for(int i=1; i<numPosture; i++)
		{
			if(original[i]==false)
			{
				j=original.find(i);
				if(j==numPosture) break;
				gapCount=j-i;
				if(gapCount<interval)
				{
					ASSERT(original[i-1] || i==1);

					if(pos.row(i-1).distance(pos.row(j)) < thr)
					{
						for(int k=0; k<gapCount; k++)
							ab.setAt(i+k);
					}
				}
				i=j-1;
			}
		}
	}

	
	void FootstepDetection::calcConstraintSep(Motion* pMot, float cut_value, float vel_cutvalue
										, const matrixn& pos, const vectorn& aSpeed, bitvectorn& abCon, bool bCleanup, int minConDuration, int reduceCon, bool bFillGap)
	{
		float fFrameTime=pMot->frameTime();

		int numPosture=pos.rows();


		abCon.clearAll();

		for(int i=0; i<numPosture; i++)
		{
			
		if(pos[i][1]<cut_value && aSpeed[i]<vel_cutvalue)
			abCon.setAt(i);

		}

		//보행동작에서 잘 동작하던건데 태권도에서 문제가 있어서 옵션으로 변경.
		// 이제 묶인것들을 보고 foot print를 계산한후, foot print센터와 1이상 멀어지는 경우는 제외해버린다.
		if(bCleanup)	
		{

			MotionUtil::SegmentFinder sf(*pMot, 0, pMot->numFrames());


			int aargMin=0;

			for(int iseg=0; iseg<sf.numSegment(); iseg++)
			{
				cleanup(abCon, sf.startFrame(iseg), sf.endFrame(iseg), aSpeed, pos);
			}

		}

		removeShortCon(abCon, minConDuration);
		shortenCon(abCon, reduceCon);
    	
		if(bFillGap)
		{
		// 발이 떨어졌다 바로 붙는 경우(0.1초  이하) 그 구간을 이어준다.
		fillGap(abCon, numPosture, ROUND(0.1/fFrameTime));

		// 0.16초의 이하의 인터벌인데, 위치이동이 별로 없는 경우 그 구간을 이어준다.
		fillGapUsingPos(abCon, numPosture, pos, ROUND(0.16/fFrameTime), 3.0);
		}

		
	}
	
	void FootstepDetection::removeShortCon(bitvectorn & abCon, int m_minConDuration)
	{
		
		if(m_minConDuration==1) return;
		intvectorn grp;

		grp.runLengthEncode(abCon);
		for(int i=0; i<grp.size()/2; i++)
		{
			int left=grp[i*2];
			int right=grp[i*2+1];

			if(right-left<m_minConDuration)
				for(int k=left; k<right; k++)
					abCon.clearAt(k);
		}
		
	}

	void FootstepDetection::shortenCon(bitvectorn & abCon, int m_reduceCon)
	{
		if(m_reduceCon==0) return;
		intvectorn grp;

		grp.runLengthEncode(abCon);
		for(int i=0; i<grp.size()/2; i++)
		{
			int left=grp[i*2];
			int right=grp[i*2+1];

			if(right-left<=m_reduceCon*2)
			{
				for(int k=left; k<right; k++)
					abCon.clearAt(k);
				abCon.setAt((left+right)/2);
			}
			else
			{
				for(int k=left; k<left+m_reduceCon; k++)
					abCon.clearAt(k);

				for(int k=right-m_reduceCon; k<right; k++)
					abCon.clearAt(k);
			}
		}
	}

	void FootstepDetection::cleanup(bitvectorn& abCon, int startFrame, int endFrame, const vectorn& aSpeed, const matrixn& pos )
	{
		intvectorn grp;

		intvectorn domain;
		vectorn footPrint(3);

		grp.runLengthEncode(abCon, startFrame, endFrame);
		for(int i=0; i<grp.size()/2; i++)
		{
			int left=grp[i*2];
			int right=grp[i*2+1];

			// foot print는 average position
			//footPrint.setAllValue(0);
			//for(int j=left; j<right; j++)
			//footPrint+=pos[j];
			//footPrint/=(m_real)(right-left);

			// foot print는 속도가 가장 작은점.
			domain.colon(left,right);
			int argMin=aSpeed.range(left, right).argMin()+left;
			footPrint=pos.row(argMin);

			for(int j=left; j<right; j++)
			{
				if(footPrint.distance(pos.row(j))>2.0)
					abCon.clearAt(j);
			}
		}
	}









	ConstraintMarking::ConstraintMarking(Motion* pMotion , bool bCleanup, bool bFillGap, int minConDuration, int reduceCon)
	{
		m_bCleanup=bCleanup;
		m_pAccurateMotion=pMotion;
		m_bFillGap=bFillGap;
		m_minConDuration=minConDuration;
		m_reduceCon=reduceCon;

		m_abLeftFoot.resize(m_pAccurateMotion->numFrames());
		m_abRightFoot.resize(m_pAccurateMotion->numFrames());

		for(int i=0; i<NUM_CON; i++)
		{
			m_abLeft[i].resize(m_pAccurateMotion->numFrames());
			m_abRight[i].resize(m_pAccurateMotion->numFrames());
		}	
	}

	ConstraintMarking::~ConstraintMarking()
	{
	}

	void ConstraintMarking::encodeCon(int con, Posture& pose, MotionLoader& skeleton, vector3 const& conPos)
	{

		if(con==CONSTRAINT_LEFT_TOE)
			skeleton.setChain(pose, skeleton.getRotJointIndexByVoca(MotionLoader::LEFTANKLE));
		else if(con==CONSTRAINT_RIGHT_TOE)
			skeleton.setChain(pose, skeleton.getRotJointIndexByVoca(MotionLoader::RIGHTANKLE));
		else
			Msg::error("Constraint is not con toe!");

		Bone& hips=skeleton.getBoneByVoca(MotionLoader::HIPS);

		quater q;
		hips.getRotation(q);

		vector3 p;
		hips.getTranslation(p);

		vector3 dv;
		dv.difference(p, conPos);

		quater inv_q;
		inv_q.inverse(q);

		if(con==CONSTRAINT_LEFT_TOE)
			pose.m_conToeL.rotate(inv_q, dv);
		else
			pose.m_conToeR.rotate(inv_q, dv);
	}

	vector3 ConstraintMarking::decodeCon(Motion const& mot, int iframe, int con)	
	{
		vector3 conPos;
		decodeCon(con, mot.pose(iframe), mot.skeleton(), conPos);
		return conPos;
	}

	void ConstraintMarking::decodeCon(int con, Posture& pose, MotionLoader& skeleton, vector3 & conPos)
	{
		skeleton.setChain(pose, skeleton.getRotJointIndexByVoca(MotionLoader::HIPS));


		Bone& hips=skeleton.getBoneByVoca(MotionLoader::HIPS);

		quater q;
		hips.getRotation(q);

		vector3 p;
		hips.getTranslation(p);

		vector3 dv;
		if(con==CONSTRAINT_LEFT_TOE)
			dv.rotate(q, pose.m_conToeL);
		else
			dv.rotate(q, pose.m_conToeR);

		conPos.add(p, dv);
	}
void ConstraintMarking::calcConstraintPos(int constraint)
	{
		MotionUtil::GetSignal getSignal(*m_pAccurateMotion);
		
		bitvectorn conToe;
		getSignal.constraint(constraint, conToe);

		intvectorn conInterval;

		MotionUtil::SegmentFinder sf(*m_pAccurateMotion, 0, m_pAccurateMotion->numFrames());

		intvectorn grp;
		intvectorn domain;
		vectorn footPrint(3);
		
		int aargMin=0;

		for(int iseg=0; iseg<sf.numSegment(); iseg++)
		{
			int startSeg=sf.startFrame(iseg);
			int endSeg=sf.endFrame(iseg);

			conInterval.runLengthEncode(conToe, startSeg, endSeg);
			int numConGrp=conInterval.size()/2;
			
			for(int grp=0; grp<numConGrp; grp++)
			{
				int start=conInterval[grp*2];
				int end=conInterval[grp*2+1];

				vector3 conPos(0,0,0);
				vector3 pos;
								
				for(int i=start; i<end; i++)
				{
					m_pAccurateMotion->skeleton().setPose(m_pAccurateMotion->pose(i));			
					Bone& bone=dep_GetBoneFromCon(m_pAccurateMotion->skeleton(),constraint);
						
					bone.getTranslation(pos);
					conPos+=pos;						
				}
				conPos/=float(end-start);

				for(int i=start; i<end; i++)
				{
					if(constraint==CONSTRAINT_LEFT_TOE)
						dep_toLocal(m_pAccurateMotion->pose(i), conPos, m_pAccurateMotion->pose(i).m_conToeL);
					else if(constraint==CONSTRAINT_RIGHT_TOE)
						dep_toLocal(m_pAccurateMotion->pose(i), conPos, m_pAccurateMotion->pose(i).m_conToeR);
					else Msg::error("Constraint is not con toe!");
				}
			}

			// fill gap (linearly blend constraint positions inbetween constrained frames.)
			if (numConGrp>0)
			for(int grp=0; grp<=numConGrp; grp++)
			{
				int prevEnd, nextStart;
				vector3 prevConPos, nextConPos;
				
				if(grp==0)
					prevEnd=startSeg;
				else
					prevEnd=conInterval[(grp-1)*2+1];
				
				if(grp==numConGrp)
					nextStart=endSeg;
				else
					nextStart=conInterval[grp*2];
				
#define CONTOE(y, x) if(constraint==CONSTRAINT_LEFT_TOE) y=m_pAccurateMotion->pose(x).m_conToeL;\
					else y=m_pAccurateMotion->pose(x).m_conToeR
				if(grp==0)
				{
					CONTOE(prevConPos, nextStart);
				}
				else
				{
					CONTOE(prevConPos, prevEnd-1);
				}
				
				if(grp==numConGrp)
				{
					CONTOE(nextConPos, prevEnd-1);
				}
				else
				{
					CONTOE(nextConPos, nextStart);
				}

				for(int i=prevEnd; i<nextStart; i++)
				{
					vector3 toepos;
					toepos.interpolate( (m_real)(i-prevEnd+1)/(m_real)(nextStart-prevEnd+1), prevConPos, nextConPos);

					if(constraint==CONSTRAINT_LEFT_TOE)
						m_pAccurateMotion->pose(i).m_conToeL=toepos;
					else
						m_pAccurateMotion->pose(i).m_conToeR=toepos;
				}
			}
		}
	}

	void ConstraintMarking::calcConstraint(float toe_height_thr, float toe_speed_thr, float heel_height_thr, float heel_speed_thr, bool bOutputImage, int eHowToChooseConstraint)
	{
		// importance계산
		int numJoint=m_pAccurateMotion->NumJoints();
		int numPosture=m_pAccurateMotion->numFrames();

		int jointL=m_pAccurateMotion->skeleton().getRotJointIndexByVoca(MotionLoader::LEFTANKLE);
		int jointR=m_pAccurateMotion->skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTANKLE);

		float fFrameTime=m_pAccurateMotion->frameTime();

		hypermatrixn posL, posR;
		hypermatrixn velL, velR;

		posL.setSize(NUM_CON, numPosture, 3);
		posR.setSize(NUM_CON, numPosture, 3);
		velL.setSize(NUM_CON, numPosture, 3);
		velR.setSize(NUM_CON, numPosture, 3);

		MotionLoader& skel=m_pAccurateMotion->skeleton();

		MotionUtil::GetSignals sig(*m_pAccurateMotion);
		sig.jointPosVel(dep_GetBoneFromCon(skel,CONSTRAINT_LEFT_TOE), posL[CON_TOE].lval(), velL[CON_TOE].lval());
		sig.jointPosVel(dep_GetBoneFromCon(skel, CONSTRAINT_RIGHT_TOE), posR[CON_TOE].lval(),   velR[CON_TOE].lval() );
		sig.jointPosVel(skel.getBoneByRotJointIndex(jointL), posL[CON_HEEL].lval(), velL[CON_HEEL].lval() );
		sig.jointPosVel(skel.getBoneByRotJointIndex(jointR), posR[CON_HEEL].lval(), velR[CON_HEEL].lval() );

		matrixn m_aaHeightLeft;	
		matrixn m_aaHeightRight;

		m_aaSpeedLeft.setSize(2,numPosture);
		m_aaSpeedRight.setSize(2,numPosture);
		m_aaHeightLeft.setSize(2,numPosture);
		m_aaHeightRight.setSize(2,numPosture);
		vectorn min_height(NUM_CON);
		vectorn height_thr(NUM_CON);
		vectorn speed_thr(NUM_CON);

		for(int con=0; con<NUM_CON; con++)
		{
			m_aaSpeedLeft.row(con).lengths( velL[con]);
			m_aaSpeedRight.row(con).lengths( velR[con]);
			posL[con].getColumn(1, m_aaHeightLeft.row(con).lval());
			posR[con].getColumn(1, m_aaHeightRight.row(con).lval());
			min_height[con]=MIN(m_aaHeightLeft.row(con).minimum(), m_aaHeightRight.row(con).minimum());
		}

		height_thr[CON_TOE]=toe_height_thr+min_height[CON_TOE];
		height_thr[CON_HEEL]=heel_height_thr+min_height[CON_HEEL];
		speed_thr[CON_TOE]=toe_speed_thr;
		speed_thr[CON_HEEL]=heel_speed_thr;

		for(int con=0; con<NUM_CON; con++)
		{
			FootstepDetection::calcConstraintSep(m_pAccurateMotion, height_thr[con], speed_thr[con], posL[con], 
				m_aaSpeedLeft.row(con), m_abLeft[con], m_bCleanup, m_minConDuration, m_reduceCon, m_bFillGap);
			FootstepDetection::calcConstraintSep(m_pAccurateMotion, height_thr[con], speed_thr[con], posR[con], 
				m_aaSpeedRight.row(con), m_abRight[con], m_bCleanup, m_minConDuration, m_reduceCon, m_bFillGap);
		}

		if(bOutputImage)
		{
			vectorn min_speed(NUM_CON);
			vectorn max_speed(NUM_CON);
			min_speed.setAllValue(0.f);
			max_speed.mult(speed_thr, 2.f);


			
			CImageProcessor::SaveAndDeleteImage(
				CImageProcessor::DrawChart(m_aaHeightLeft, CImageProcessor::LINE_CHART, min_height,min_height+(height_thr-min_height)*2.f,height_thr.dataPtr()), "foot_left_height.bmp");
			CImageProcessor::SaveAndDeleteImage(
				CImageProcessor::DrawChart(m_aaHeightRight, CImageProcessor::LINE_CHART, min_height,min_height+(height_thr-min_height)*2.f,height_thr.dataPtr()), "foot_right_height.bmp");
			CImageProcessor::SaveAndDeleteImage(
				CImageProcessor::DrawChart(m_aaSpeedLeft, CImageProcessor::LINE_CHART, min_speed, max_speed, speed_thr.dataPtr()), "foot_left_speed.bmp");
			CImageProcessor::SaveAndDeleteImage(
				CImageProcessor::DrawChart(m_aaSpeedRight, CImageProcessor::LINE_CHART, min_speed, max_speed, speed_thr.dataPtr()), "foot_right_speed.bmp");
		}

		for(int i=0; i<numPosture; i++)
		{
			switch(eHowToChooseConstraint)
			{
			case CHOOSE_AND:
				m_abLeftFoot.set(i,m_abLeft[CON_TOE][i]&&m_abLeft[CON_HEEL][i]);
				m_abRightFoot.set(i,m_abRight[CON_TOE][i]&&m_abRight[CON_HEEL][i]);
				break;
			case CHOOSE_OR:
				m_abLeftFoot.set(i,m_abLeft[CON_TOE][i]||m_abLeft[CON_HEEL][i]);
				m_abRightFoot.set(i,m_abRight[CON_TOE][i]||m_abRight[CON_HEEL][i]);
				break;
			case CHOOSE_TOE:
				m_abLeftFoot.set(i,m_abLeft[CON_TOE][i]);
				m_abRightFoot.set(i,m_abRight[CON_TOE][i]);
				break;
			case CHOOSE_HEEL:
				m_abLeftFoot.set(i,m_abLeft[CON_HEEL][i]);
				m_abRightFoot.set(i,m_abRight[CON_HEEL][i]);
				break;
			}
		}

		for(int i=0; i<m_abLeftFoot.size(); i++)
		{
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_LEFT_FOOT, m_abLeftFoot[i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_RIGHT_FOOT, m_abRightFoot[i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_LEFT_TOE, m_abLeft[CON_TOE][i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_RIGHT_TOE, m_abRight[CON_TOE][i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_LEFT_HEEL, m_abLeft[CON_HEEL][i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_RIGHT_HEEL, m_abRight[CON_HEEL][i]);
		}

		calcConstraintPos(CONSTRAINT_LEFT_TOE);
		calcConstraintPos(CONSTRAINT_RIGHT_TOE);
	}

}


