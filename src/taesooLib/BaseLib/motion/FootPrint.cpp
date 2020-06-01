
#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "FootPrint.h"
#include "motion/Motion.h"
#include "motion/MotionUtil.h"
#include "motion/MotionLoader.h"
#include "ConstraintMarking.h"
#include "../math/intervals.h"
#include "../math/Operator.h"
using namespace MotionUtil;
void FootPrint::getFootInterval(const Motion& mot, int iframe, int constraint, int& left, int& right) const
{
	if(mot.isConstraint(iframe, constraint))
	{
		bool bBrake=false;		
		for(left=iframe; !bBrake && left>=0 && mot.isConstraint(left, constraint); left--)
		{
			if(mot.isDiscontinuous(left))
				bBrake=true;
		}
		left++;

		for(right=iframe+1; right<mot.numFrames()  && !mot.isDiscontinuous(right) && mot.isConstraint(right, constraint); right++);

		ASSERT(right>left);
	}
	else
	{
		Msg::error("%d Is not constrained by con %d", iframe, constraint);
	}
}

void CalcFootPrint::getFootPrints(const Motion& mot, int start, int end, int constraint, intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	conInterval.runLengthEncode(conToe);
	conInterval+=start;

	int numConGrp=conInterval.size()/2;
	aFootPositions.setSize(numConGrp,3);
	
	for(int grp=0; grp<numConGrp; grp++)
	{
		int start=conInterval[grp*2];
		int end=conInterval[grp*2+1];

		vector3 conPos(0,0,0);
		vector3 pos;
						
		for(int i=start; i<end; i++)
		{
			mot.skeleton().setPose(mot.pose(i));			
			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
				
			bone.getTranslation(pos);
			conPos+=pos;						
		}
		conPos/=float(end-start);

		aFootPositions.row(grp).assign(conPos);		

	}
}

void GetFootPrint::getFootPrints(const Motion& mot, int start, int end, int constraint, intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	conInterval.runLengthEncode(conToe);
	conInterval+=start;

	int numConGrp=conInterval.size()/2;
	aFootPositions.setSize(numConGrp,3);
	
	for(int grp=0; grp<numConGrp; grp++)
	{
		int start=conInterval[grp*2];
		int end=conInterval[grp*2+1];

		vector3 conPos(0,0,0);
						
		for(int i=start; i<end; i++)
		{
			conPos+=MotionUtil::ConstraintMarking::decodeCon(mot, i, constraint);
		}
		conPos/=float(end-start);

		aFootPositions.row(grp).assign(conPos);		
	}	
}

GetFootPrintOnline::GetFootPrintOnline(m_real minHeight, m_real maxHeight, m_real lengthThr, m_real distThr)
:mHeightInterval(minHeight, maxHeight),
mLengthThr(lengthThr),
mDistThr(distThr)
{
	
	
}

void GetFootPrintOnline::getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	intIntervals cons;

	cons.runLengthEncode(conToe);
	cons.offset(start);

	for(int grp=0; grp<cons.size(); grp++)
	{
		int startCon=cons.start(grp);
		int endCon=cons.end(grp);
		for(int i=startCon; i<endCon; i++)
			Msg::verify(mot.isConstraint(i, constraint), "???");
	}

	aFootPositions.setSize(cons.size(),3);
	
	for(int grp=0; grp<cons.size(); grp++)
	{
		int startCon=cons.start(grp);
		int endCon=cons.end(grp);
		bool bContinuedCon=false;
		vector3 conPos(0,0,0);
		vector3 startPos;
		mot.skeleton().setPose(mot.pose(startCon));
		Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
		bone.getTranslation(startPos);

		if(startCon==start && start!=0 && mot.isConstraint(start-1, constraint))
		{
			//printf("continued con %d\n", start);

			/*
			mot.skeleton().setPose(mot.pose(start-1));

			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);*/
			
			// global 좌표계에서 저장되어있다고 가정한다.
			
			conPos=mot.pose(start-1).conPosition(constraint);
			bContinuedCon=true;
		}
		else
		{
			vector3 pos;
			vector3 startConPos;
			vector3 avgPos(0,0,0);
			
			for(int i=startCon; i<endCon; i++)
			{
				mot.skeleton().setPose(mot.pose(i));
				mot.pose(i).conPosition(constraint)=vector3(0,100,0);	// for debug
				Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
				bone.getTranslation(pos);

				if(i==startCon) 
				{
					startConPos=pos;
				}

				//avgPos+=MotionUtil::ConstraintMarking::decodeCon(mot, i, constraint);
				avgPos+=pos;
			}
			avgPos/=float(endCon-startCon);
					
			// 3 프레임 이상의 여유가 있으면, 그냥 avgPos사용
			m_real t=sop::clampMap(startCon, start, start+3);
			
            conPos.lerp(startConPos, avgPos, t);

			conPos=avgPos;
			conPos.y=mHeightInterval.project(conPos.y);
		
		}
		
		/*if(endCon<=startCon)//if(conPos.y>5 || endCon<=startCon)// || conPos.distance(startPos)>10)
		{
			cons.removeInterval(grp);
			grp--;			
		}
		else*/
		{
			aFootPositions.row(grp).assign(conPos);
		}
	}

	aFootPositions.resize(cons.size(),3);
	cons.encodeIntoVector(conInterval);

	/*
	// 바뀐바 없음.
	// save constraint (다음에 이어붙였을때 사용됨)
	for(int j=start; j<end; j++)
		((Motion&)mot).setConstraint(j, constraint, false);

	for(int i=0; i<cons.numInterval(); i++)
	{
		for(int j=cons.start(i); j< cons.end(i); j++)
			((Motion&)mot).setConstraint(j, constraint, true);
	}*/

}





void CalcFootPrintSpeed::getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	conInterval.runLengthEncode(conToe);
	conInterval+=start;

	int numConGrp=conInterval.size()/2;
	aFootPositions.setSize(numConGrp,3);
	
	for(int grp=0; grp<numConGrp; grp++)
	{
		int startCon=conInterval[grp*2];
		int endCon=conInterval[grp*2+1];

		vector3 conPos(0,0,0);

		if(startCon==start)
		{
			mot.skeleton().setPose(mot.pose(startCon));
			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);
		}
		else if(endCon==end)
		{
			mot.skeleton().setPose(mot.pose(endCon-1));
			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);
		}
		else
		{
			vector3 pos;
							
			for(int i=startCon; i<endCon; i++)
			{
				mot.skeleton().setPose(mot.pose(i));			
				Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
					
				bone.getTranslation(pos);
				conPos+=pos;						
			}
			conPos/=float(endCon-startCon);
		}

		aFootPositions.row(grp).assign(conPos);

	}
}

