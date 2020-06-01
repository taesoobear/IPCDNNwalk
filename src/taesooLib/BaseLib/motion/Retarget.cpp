#include "stdafx.h"
#include "../math/mathclass.h"
#include "../math/Operator.h"
#include "./Retarget.h"
#include "Motion.h"
#include "MotionUtil.h"
#include "IKSolver.h"
#include "../BaseLib/motion/MotionLoader.h"
#include "FootPrint.h"
#include "../math/intervals.h"
#include "FullbodyIK.h"
#include "../math/conversion.h"
#include "../utility/scoped_ptr.h"

using namespace MotionUtil;


void MotionUtil::calcFootPrints(const Motion& mot, bitvectorn const& con, const Bone& bone, intIntervals& cons, vector3N& footprints)
{
	cons.runLengthEncode(con);

	footprints.resize(cons.size());

	int numConGrp=cons.size();

	for(int grp=0; grp<numConGrp; grp++)
	{
		int start=cons.start(grp);
		int end=cons.end(grp);

		vector3 conPos(0,0,0);
		vector3 pos;

		for(int i=start; i<end; i++)
		{
			mot.skeleton().setPose(mot.pose(i));
			bone.getTranslation(pos);
			conPos+=pos;
		}
		conPos/=float(end-start);

		footprints(grp)=conPos;
	}
}

void MotionUtil::convertFootprintsToConstraints(intIntervals const& interval, vector3N const& footPrints,
										intvectorn & constraintFrames, vector3N& constraints)
{
	int totalFrames=0;
	for(int i=0; i<interval.size(); i++)
		totalFrames+=interval.end(i)-interval.start(i);

	constraintFrames.reserve(totalFrames);
	constraintFrames.resize(0);
	constraints.reserve(totalFrames);
	constraints.resize(0);

	for(int i=0; i<interval.size(); i++)
	{
		for(int f=interval.start(i); f<interval.end(i); f++)
		{
			constraintFrames.push_back(f);
			constraints.pushBack(footPrints(i));
		}
	}

}

void MotionUtil::calcConstraints(const Motion& source, int conIndex, intvectorn &constrainedFrames, vector3N& constraints)
{

	GetSignal gs(source);

	bitvectorn con;
	intIntervals interval;
	vector3N footprints;

	gs.constraint(conIndex, con);

	calcFootPrints(source, con, dep_GetBoneFromCon(source.skeleton(), conIndex), interval, footprints);

	convertFootprintsToConstraints(interval, footprints, constrainedFrames, constraints);
}

void MotionUtil::retargetEntireMotion(Motion& source, bool bRetargetFoot, bool bRetargetArm)
{
	std::vector<Bone*> bones;
	intvectorn cons;
	MotionLoader& skel=source.skeleton();

	if(bRetargetFoot)
	{
		bones.push_back(skel.getBoneByVoca(MotionLoader::LEFTANKLE).child());
		bones.push_back(skel.getBoneByVoca(MotionLoader::RIGHTANKLE).child());
		cons.push_back(CONSTRAINT_LEFT_TOE);
		cons.push_back(CONSTRAINT_RIGHT_TOE);
	}

	if(bRetargetArm)
	{
		bones.push_back(skel.getBoneByVoca(MotionLoader::LEFTWRIST).child());
		bones.push_back(skel.getBoneByVoca(MotionLoader::RIGHTWRIST).child());
		cons.push_back(CONSTRAINT_LEFT_FINGERTIP);
		cons.push_back(CONSTRAINT_RIGHT_FINGERTIP);
	}

	GetSignal gs(source);
	bitvectorn con;

	intIntervals interval;
	vector3N footprints;

	for(int bb=0; bb<bones.size(); bb++)
	{
		gs.constraint(cons[bb], con);
		calcFootPrints(source, con, *bones[bb], interval, footprints);

		scoped_ptr<FullbodyIK> iksolver;
		std::vector<MotionUtil::Effector> effectors;
		effectors.resize(1);
		effectors[0].bone=bones[bb];

		iksolver.reset(createFullbodyIk_LimbIK(source.skeleton(), effectors));

		intvectorn constrainedFrames;
		vector3N constraints;
		convertFootprintsToConstraints(interval, footprints, constrainedFrames, constraints);

		MotionRetarget mr(*iksolver);
		mr.retarget(source, constrainedFrames, matView(constraints), -30, source.numFrames()+30);
	}
}

quater MotionRetarget::quater_transition0(const quater& aa, const quater& bb, int i, int duration)
{
	quater a(aa);
	quater b(bb);
	quater qid;
	qid.identity();
	a.align(qid);
	b.align(qid);
	// kovar paper (prefer identity quaternion (more safe))

	m_real totalTime=(m_real)duration+1;
	m_real currTime;
	quater c, d, qi,out;
	qi.identity();

	currTime=(m_real)(i+1)/totalTime;
	m_real t=-2.f*CUBIC(currTime)+3.f*SQR(currTime);
	c.slerp(a, qi, t);
	d.slerp(qi, b, t);
	out.slerp(c, d, currTime);
	return out;
}

quater MotionRetarget::quater_transition(const quater& aa, const quater& b, int i, int duration)
{
	quater out;
	quater a(aa);
	a.align(b);

	m_real totalTime=duration+1;
	m_real currTime;

	currTime=(m_real)(i+1)/totalTime;
	m_real t=-2.f*CUBIC(currTime)+3.f*SQR(currTime);
	out.interpolate(t, a, b);
	return out;
}

vector3 MotionRetarget::vector3_transition(const vector3& a, const vector3& b, int i, int duration)
{
	vector3 out;
	m_real totalTime=duration+1;
	m_real currTime;

	currTime=(m_real)(i+1)/totalTime;
	m_real t=-2.f*CUBIC(currTime)+3.f*SQR(currTime);
	out.interpolate(t, a,b);
	return out;
}

quater MotionRetarget::quater_transition_auto(const quater& a, const quater& b, int i, int duration)
{
	if(duration>6)
		return quater_transition0(a,b, i, duration);
	else
		return quater_transition(a, b, i, duration);
}


void MotionUtil::Displacement::identity(int rot_size, int trans_size)
{
	rot.setSize(rot_size);
	trans.setSize(trans_size);
	rot.setAllValue(quater(1,0,0,0));
	trans.setAllValue(vector3(0,0,0));
}

void MotionUtil::Displacement::apply(Posture& inout, const intvectorn& rot_index, const intvectorn& trans_index)
{
	for(int i=0; i<rot_index.size(); i++)
		inout.m_aRotations[rot_index[i]].leftMult(rot[i]);

	for(int i=0; i<trans_index.size(); i++)
		inout.m_aTranslations[trans_index[i]]+=trans[i];
}


void Displacement::setSameSize(Displacement const& other)
{
	rot.setSize(other.rot.size());
	trans.setSize(other.trans.size());
}
void Displacement::transition(Displacement const& a, Displacement const& b, int i, int duration)
{
	setSameSize(a);
	for(int d=0; d<rot.size(); d++)
		//rot(d)=MotionRetarget::quater_transition_auto(a.rot[d], b.rot[d], i, duration);
		rot(d)=MotionRetarget::quater_transition(a.rot[d], b.rot[d], i, duration);

	for(int d=0; d<trans.size(); d++)
		trans(d)=MotionRetarget::vector3_transition(a.trans[d], b.trans[d], i, duration);
}

void MotionUtil::DisplacementMap_fillGap(DisplacementMap& value, int duration, Displacement const& prevDelta, Displacement const& delta)
{
	value.resize(duration);

	for(int i=0; i<duration; i++)
		value(i).transition(prevDelta, delta, i, duration);
}


MotionRetarget::MotionRetarget(FullbodyIK& iksolver)
:m_IKsolver(iksolver)
{
	intvectorn rot_joint_index, trans_joint_index;
	m_IKsolver.getAffectedDOF(rot_joint_index, trans_joint_index);
	leftBoundaryDisplacement.identity(rot_joint_index.size(), trans_joint_index.size());
	rightBoundaryDisplacement.identity(rot_joint_index.size(), trans_joint_index.size());
}

void MotionRetarget_IKonly::retarget(Motion const& input, intvectorn const& constrainedFrames, matrixn const& constraints, int firstFrame, int lastFrame, DisplacementMap& disp)
{
	intvectorn rot_index, trans_index;

	m_IKsolver.getAffectedDOF(rot_index, trans_index);

	disp.resize(lastFrame-firstFrame+1);

	for(int i=0; i<disp.size(); i++)
		disp[i].identity(rot_index.size(), trans_index.size());

	DisplacementMapView dispmap=vecViewOffset(disp, firstFrame);

	for(int i=0; i<constrainedFrames.size(); i++)
	{
		int f=constrainedFrames[i];
		// IK
		m_IKsolver.IKsolve(input.pose(f), vec3View(constraints.row(i)), rot_index, delta.rot, trans_index, delta.trans);

		dispmap(f)=delta;
	}
}

void MotionRetarget::retarget(Motion const& m_mot, intvectorn const& constrainedFrames, matrixn const& constraints, int firstFrame, int lastFrame, DisplacementMap& disp)
{
	disp.resize(lastFrame-firstFrame+1);

	DisplacementMapView dispmap=vecViewOffset(disp, firstFrame);

	intIntervals intervals;
	intervals.findConsecutiveIntervals(constrainedFrames);

	intvectorn rot_index, trans_index;

	prevDelta=leftBoundaryDisplacement;

	int numFootPrint=intervals.size();

	for(int footStep=0; footStep<numFootPrint; footStep++)
	{
		int left=constrainedFrames[intervals.start(footStep)];
		int right=constrainedFrames[intervals.end(footStep)-1]+1;

		// IK
		m_IKsolver.IKsolve(m_mot.pose(left), vec3View(constraints.row(intervals.start(footStep))), rot_index, delta.rot, trans_index, delta.trans);

		dispmap(left)=delta;

		// fill gap "before" or "inbetween" footsteps
		int start;

		if(footStep==0)
			start=firstFrame;
		else
			start=constrainedFrames[intervals.end(footStep-1)-1]+1;

		int end=left;

        DisplacementMapView dispmapView=dispmap.range(start, end);
		MotionUtil::DisplacementMap_fillGap(dispmapView, end-start, prevDelta, delta);

		// IK
		for(int f=left+1; f<right; f++)
		{
			m_IKsolver.IKsolve(m_mot.pose(f), vec3View(constraints.row(intervals.start(footStep)+f-left)), rot_index, delta.rot, trans_index, delta.trans);

			dispmap(f)=delta;
		}

		prevDelta=delta;

	}

	int start, end;
	if(numFootPrint)
	{
		// fill gap "after".
		start=constrainedFrames[intervals.end(numFootPrint-1)-1]+1;
		end=lastFrame+1;
	}
	else
	{
		start=firstFrame;
		end=lastFrame+1;
	}

	if(end-start>1)
	{
		delta=rightBoundaryDisplacement;
		DisplacementMapView dpv=dispmap.range(start, end);
		MotionUtil::DisplacementMap_fillGap(dpv, end-start, prevDelta, delta);
	}
}

void MotionRetarget::retarget(Motion& inout, intvectorn const& constrainedFrames, matrixn const& constraints, int firstFrame, int lastFrame)
{
	DisplacementMap disp;
	retarget(inout, constrainedFrames, constraints, firstFrame, lastFrame, disp);

	DisplacementMapView dispmap=vecViewOffset(disp, firstFrame);

	int start=MAX(0, firstFrame);
	int end=MIN(lastFrame+1, inout.numFrames());

	intvectorn rot_index, trans_index;
	m_IKsolver.getAffectedDOF(rot_index, trans_index);

	for(int i=start; i<end; i++)
		dispmap[i].apply(inout.pose(i), rot_index, trans_index);
}


namespace MotionUtil
{
	class _MotionRetargetOnline_data
	{
	public:
		_MotionRetargetOnline_data(Motion& m, FullbodyIK& iksolver):mot(m), retargetMethod(iksolver)
		{
			prevPlayEnd=0;
			bLastFrameConstrained=false;
			iksolver.getAffectedDOF(rot_joint_index, trans_joint_index);
			prevDelta.identity(rot_joint_index.size(), trans_joint_index.size());
		}

		Motion& mot;	// input and output motion.
		MotionRetarget retargetMethod;
		//MotionRetarget_IKonly retargetMethod;
		int prevPlayEnd;
		Displacement prevDelta;
		DisplacementMap disp;
		intvectorn rot_joint_index, trans_joint_index;
		bool bLastFrameConstrained;
		vectorn constraintLastFrame;
	};
}

void MotionRetargetOnline::notifyTransformation(const matrix4& tranf)
{
	if(mData->bLastFrameConstrained)
	{
		for(int i=0; i<mData->constraintLastFrame.size(); i+=3)
		{
			vector3 pos=mData->constraintLastFrame.toVector3(i);
			pos.leftMult(tranf);
			mData->constraintLastFrame.range(i, i+3).assign(pos);
		}
	}
}

MotionRetargetOnline::MotionRetargetOnline(Motion& mot, FullbodyIK& iksolver)
{
	mData=new MotionUtil::_MotionRetargetOnline_data(mot,iksolver);
}

MotionRetargetOnline::~MotionRetargetOnline()
{
	delete mData;
}

void MotionRetargetOnline::retarget(int start, int nextPlayEnd, intvectorn const& constrainedFrames, matrixn const& constraints)
{
	if(mData->bLastFrameConstrained)
	{
		int index=constrainedFrames.findFirstIndex(start);
		if(index!=-1)
		{
			// constrained position에 jump가 없도록 보장한다.
			vectorn delta;
			delta.sub(mData->constraintLastFrame, constraints.row(index));
			do
			{
				constraints.row(index)+=delta;
				index++;
			}
			while ( index<constrainedFrames.size() && constrainedFrames[index]==constrainedFrames[index-1]+1);
		}
	}

	int index;
	if((index=constrainedFrames.findFirstIndex(nextPlayEnd-1))==-1)
		mData->bLastFrameConstrained=false;
	else
	{
		mData->bLastFrameConstrained=true;
		mData->constraintLastFrame=constraints.row(index);
	}

	ASSERT(mData->prevPlayEnd==0 || mData->prevPlayEnd==start);

	mData->retargetMethod.leftBoundaryDisplacement=mData->prevDelta;
	mData->retargetMethod.retarget(mData->mot, constrainedFrames, constraints, start, nextPlayEnd+30, mData->disp);

	DisplacementMapView dispmap=vecViewOffset(mData->disp, start);


	if(constrainedFrames.size())
	{
		int lastValid=constrainedFrames[constrainedFrames.size()-1];
		for(int i=lastValid+1; i<dispmap.size(); i++)
			dispmap[i]=dispmap[lastValid];
	}

	int end=nextPlayEnd;
	for(int i=start; i<end; i++)
		dispmap[i].apply(mData->mot.pose(i), mData->rot_joint_index, mData->trans_joint_index);

//#define _DEBUG_DISPMAP
#ifdef _DEBUG_DISPMAP

	static matrixn temp;
	static intvectorn xplot;

	if(temp.rows()==0 && end<1000)
	{
		temp.setSize(1000,3);
		temp.setAllValue(0);
	}

	if(end>1000)
	{
		if(temp.rows())
		{
			TString fn;
			fn.format("dispmap.bmp", start, nextPlayEnd);
			//temp.op0(m0::drawSignals(fn, 0.0, 0.0, true, xplot));
			temp.op0(m0::drawSignals(fn, false, xplot));
			temp.setSize(0,0);
		}
	}
	else
	{
		for(int i=start; i<end; i++)
		{
			temp[i][0]=dispmap[i].rot(0).w;
			temp[i][1]=dispmap[i].trans(0).y;
		}

		temp.column(2).range(start, end).linspace(0,1);

		xplot.concat(xplot, constrainedFrames);
	}

	TString fn;
	fn.format("dispmat%05d_%05d.bmp", start, nextPlayEnd);
	vectorn tt(mData->disp.size());
	for(int i=0; i<tt.size(); i++)
		tt[i]=mData->disp[i].trans(0).y;

	tt.op0(v0::drawSignals(fn));

#endif
	mData->prevPlayEnd=nextPlayEnd;
	mData->prevDelta=dispmap[end-1];


}


void Retarget::retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval)
{
	intvectorn left, right;
	left.setSize(interval.size()/2);
	right.setSize(interval.size()/2+1);

	for(int i=0; i<left.size(); i++)
	{
		left[i]=interval[i*2];
		right[i]=interval[i*2+1];
	}

	right[right.size()-1]=-1;
	int currFootPrint=0;

	intvectorn iFootPrints;

	iFootPrints.setSize(endSafe-startSafe);
	iFootPrints.setAllValue(-1);	// means no constraint

#define setFootPrint(currFrame, currCon) iFootPrints[(currFrame)-startSafe]=(currCon)
#define getFootPrint(currFrame)	iFootPrints[(currFrame)-startSafe]

	for(int i=0; i<footPrints.rows(); i++)
	{
		for(int j=left[i]; j<right[i]; j++)
			setFootPrint(j, i);
	}

	intvectorn index;
	quaterN displacement[3];
	quaterN delta_rot;
	intvectorn displacementIndex(endSafe-startSafe);
	displacementIndex.setAllValue(-1);	// means no IK.

#define setDisplacement(currFrame, displacement) displacementIndex[(currFrame)-startSafe]=(displacement)
#define getDisplacement(currFrame) displacementIndex[(currFrame)-startSafe]

//#define SAVE_SIGNAL
#ifdef SAVE_SIGNAL
	matrixn saveSignal, signal2;
	saveSignal.setSize(endFrame-startFrame, 4);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1);
#endif
	int conCount=iFootPrints.count(s2::INT_NOT_EQUAL, -1);
	displacement[0].setSize(conCount+1);
	displacement[1].setSize(conCount+1);
	displacement[2].setSize(conCount+1);

	int count=0;

	// 일단 ik를 수행후 결과를 저장한다.
	for(int i=startFrame; i<endFrame; i++)
	{
		if(getFootPrint(i)!=-1)
		{
			m_mot.skeleton().setPose(m_mot.pose(i));

			MotionUtil::IKSolveAnalytic(m_mot.skeleton(), MotionUtil::conToBone(con, m_mot.skeleton()), footPrints.row(getFootPrint(i)).toVector3(), index, delta_rot);

			m_real angle1=(m_mot.pose(i).m_aRotations[index[2]]*delta_rot[2]).rotationAngle();
			m_real angle2=delta_rot[2].rotationAngle();

			//if(SQR(angle1)<0.1 && SQR(angle2) >0.1 )
			//if(SQR(angle1)<0.1 )|| ( SQR(angle1)<0.2 && SQR(angle2) >0.2))
			if(0)
			{
				//SetConstraint(i, con, false);

				// if leg is too straight, ignore IK
				int cur_footstep=getFootPrint(i);
				if(i==left[cur_footstep])
				{
					if(left[cur_footstep]+1<right[cur_footstep])
					{
						left[cur_footstep]++;
						setFootPrint(i,-1);
						continue;
					}
				}
				else
				{
					for(int k=i; k<right[cur_footstep]; k++)
						setFootPrint(k,-1);
					right[cur_footstep]=i;
					continue;
				}
			}
			setDisplacement(i, count);
			displacement[0][count]=delta_rot[1];
			displacement[1][count]=delta_rot[2];
			displacement[2][count]=delta_rot[3];
			count++;

			m_mot.pose(i).m_aRotations[index[1]].leftMult(delta_rot[1]);
			m_mot.pose(i).m_aRotations[index[2]].leftMult(delta_rot[2]);
			m_mot.pose(i).m_aRotations[index[3]].leftMult(delta_rot[3]);
#ifdef SAVE_SIGNAL
			saveSignal[i-startFrame].assign(delta_rot[3]);
			signal2[i-startFrame][0]=1.f;
#endif
		}
	}

	if(count==0) return ;

	return;
	displacement[0][count]=quater(1,0,0,0);
	displacement[1][count]=quater(1,0,0,0);
	displacement[2][count]=quater(1,0,0,0);

	for(int i=startSafe; i<endSafe; i++)
	{
		if(getDisplacement(i)==-1)
			setDisplacement(i, count);
	}

	int numFootPrint= left.size();

	quaterN displacementMap;


	for(int i=0; i<=numFootPrint; i++)
	{
		int start=(i==0)?startSafe:right[i-1]-1;
		int end=(i==numFootPrint)?endSafe-1:left[i];

		if(end<startFrame) continue;
		if(start>=endFrame) continue;

		//printf("[%d,%d)>[%d,%d)", start, end, startFrame, endFrame);
#ifdef SAVE_SIGNAL
		signal2[start-startFrame][2]=0;
		signal2[end-startFrame][2]=1;
#endif
		for(int j=0; j<3; j++)
		{
			if(end-start-1>0)
			{
				// transition version (c0 continuous)
				if(getDisplacement(start)==count||
					getDisplacement(end)==count)
					displacementMap.transition(displacement[j][getDisplacement(start)],
												displacement[j][getDisplacement(end)], end-start-1);
				else
                    displacementMap.transition0(displacement[j][getDisplacement(start)],
												displacement[j][getDisplacement(end)], end-start-1);

				for(int iframe=start+1; iframe<end; iframe++)
				{
					if(iframe<m_mot.numFrames())
						m_mot.pose(iframe).m_aRotations[index[j+1]].leftMult(displacementMap[iframe-(start+1)]);
#ifdef SAVE_SIGNAL
					saveSignal[iframe-startFrame].assign(displacementMap[iframe-(start+1)]);
					signal2[iframe-startFrame][1]=1.f;
#endif
				}


				// c1 continuous version
				/*quater a, b;
				if(start-startFrame-1>0 && displacementIndex[start-startFrame-1]!=-1)
					a=displacement[j][displacementIndex[start-startFrame-1]];
				else
					a=displacement[j][displacementIndex[start-startFrame]];

				if(end-startFrame+1<count && displacementIndex[end-startFrame+1]!=-1)
					b=displacement[j][displacementIndex[end-startFrame+1]];
				else
					b=displacement[j][displacementIndex[end-startFrame]];

				displacementMap.hermite0(a, displacement[j][displacementIndex[start-startFrame]], end-start+1,
										displacement[j][displacementIndex[end-startFrame]], b, 2.f);

				for(int iframe=start+1; iframe<end; iframe++)
				{
					pose(iframe).m_aRotations[index[j+1]].leftMult(displacementMap[iframe-(start)]);
#ifdef SAVE_SIGNAL
					saveSignal[iframe-startFrame].assign(displacementMap[iframe-(start)]);
					signal2[iframe-startFrame][1]=1.f;
#endif
				}
				*/

			}
		}
	}
#ifdef SAVE_SIGNAL
	saveSignal.op1(m1::each(v1::concat()), signal2);
	CString filename;
	filename.Format("saveSignal%d.bmp", con);
	saveSignal.op0(m0::drawSignals(filename,-1, 1, true));

	saveSignal.setSize(endFrame-startFrame, 4);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1.f);
#endif
}

void RetargetTest::retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval)
{
	intvectorn index;
	quaterN delta_rot;


	for(int footStep=0; footStep<interval.size()/2; footStep++)
	{
		int left=interval[footStep*2];
		int right=interval[footStep*2+1];

		vector3 conPos=footPrints.row(footStep).toVector3();
		for(int i=left; i<right; i++)
		{
			m_mot.skeleton().setPose(m_mot.pose(i));

			MotionUtil::IKSolveAnalytic(m_mot.skeleton(), MotionUtil::conToBone(con, m_mot.skeleton()), conPos, index, delta_rot);

			m_mot.pose(i).m_aRotations[index[1]].leftMult(delta_rot[1]);
			m_mot.pose(i).m_aRotations[index[2]].leftMult(delta_rot[2]);
			m_mot.pose(i).m_aRotations[index[3]].leftMult(delta_rot[3]);
		}
	}
}

void Retarget2::retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval)
{
	intvectorn index;
	quaterN prevDelta_rot;
	quaterN delta_rot;
	quaterN displacementMap;


	prevDelta_rot.setSize(4);
	prevDelta_rot[0].identity();
	prevDelta_rot[1].identity();
	prevDelta_rot[2].identity();
	prevDelta_rot[3].identity();

	intIntervals intervals;
	intervals.decodeFromVector(interval);


	int numFootPrint=intervals.size();

	for(int footStep=0; footStep<intervals.size(); footStep++)
	{
        if(intervals.start(footStep)>endFrame)
		{
			numFootPrint=footStep;
			break;
		}
		else if(intervals.end(footStep)>endFrame)
		{
			intervals.end(footStep)=endFrame;
		}
	}

	for(int footStep=0; footStep<numFootPrint; footStep++)
	{
		int left=intervals.start(footStep);
		int right=intervals.end(footStep);

		m_mot.skeleton().setPose(m_mot.pose(left));
		MotionUtil::IKSolveAnalytic(m_mot.skeleton(), MotionUtil::conToBone(con,m_mot.skeleton()), footPrints.row(footStep).toVector3(), index, delta_rot);
		m_mot.pose(left).m_aRotations[index[1]].leftMult(delta_rot[1]);
		m_mot.pose(left).m_aRotations[index[2]].leftMult(delta_rot[2]);
		m_mot.pose(left).m_aRotations[index[3]].leftMult(delta_rot[3]);

		// fill gap "before" or "inbetween" footsteps

		int start=(footStep==0)?startSafe:intervals.end(footStep-1);
		int end=left;

		for(int j=1; j<4; j++)
		{
			if(end-start-1>0)
			{
				if(end-start>6)
					displacementMap.transition0(prevDelta_rot[j], delta_rot[j], end-start);
				else
					displacementMap.transition(prevDelta_rot[j], delta_rot[j], end-start);

				for(int iframe=start; iframe<end; iframe++)
				{
					if(iframe<m_mot.numFrames())
						m_mot.pose(iframe).m_aRotations[index[j]].leftMult(displacementMap[iframe-start]);
				}
			}
		}
		// IK
		for(int i=left+1; i<right; i++)
		{
			m_mot.skeleton().setPose(m_mot.pose(i));

			MotionUtil::IKSolveAnalytic(m_mot.skeleton(),MotionUtil::conToBone( con,m_mot.skeleton()), footPrints.row(footStep).toVector3(), index, delta_rot);

			m_mot.pose(i).m_aRotations[index[1]].leftMult(delta_rot[1]);
			m_mot.pose(i).m_aRotations[index[2]].leftMult(delta_rot[2]);
			m_mot.pose(i).m_aRotations[index[3]].leftMult(delta_rot[3]);
		}

		prevDelta_rot=delta_rot;
	}

	if(numFootPrint)
	{
		// fill gap after.
		int start=intervals.end(numFootPrint-1);
		int end=endSafe;

		for(int j=1; j<4; j++)
		{
			if(end-start-1>0)
			{
				delta_rot[j].identity();
				if(end-start>6)
					displacementMap.transition0(prevDelta_rot[j], delta_rot[j], end-start);
				else
					displacementMap.transition(prevDelta_rot[j], delta_rot[j], end-start);

				for(int iframe=start; iframe<end; iframe++)
				{
					if(iframe<m_mot.numFrames())
						m_mot.pose(iframe).m_aRotations[index[j]].leftMult(displacementMap[iframe-start]);
				}
			}
		}
	}

}

/*
void RetargetOnline::retarget(int startFrame, int endFrame, const intvectorn& aConstraint, const CTArray<matrixn>& conPos, CTArray<intvectorn>&  left, CTArray<intvectorn>& right)

{
	int numCon=aConstraint.size();

	vector3 temp;

	static matrixn importance;
	static hypermatrixn footPosition;
	static bitvectorn undefined;
	static intvectorn undefinedGrp;
	static vectorn time;
	static vectorn signal;
	static quaterN delta_rot;
	static intvectorn index;

	importance.setSize(numCon, endFrame-startFrame);

	footPosition.setSize(numCon, endFrame-startFrame, 3);

	// calc importances and goal foot positions
	for(int con=0; con<numCon; con++)
	{
		undefined.setSize(endFrame-startFrame);
		undefined.setAll();

		// set first frame
		undefined.clearAt(0);
		importance[con][0]=0.f;

		skeleton().setPose(pose(startFrame));
		skeleton().GetSiteBone(skeleton().GetJointIndexFromConstraint(aConstraint[con])).getTranslation(temp);
		footPosition[con][0].assign(temp);

		// set keyframes (importance==1.f)
		for(int cur_footstep=0; cur_footstep<left[con].size(); cur_footstep++)
		{
			int start=left[con][cur_footstep];
			int end=right[con][cur_footstep];

			for(int i=start; i<end; i++)
			{
				undefined.clearAt(i-startFrame);
				importance[con][i-startFrame]=1.f;
				footPosition[con][i-startFrame]=conPos[con][cur_footstep];
			}
		}

		// set last frame
		if(undefined[endFrame-startFrame-1])
		{
			undefined.clearAt(endFrame-startFrame-1);
			importance[con][endFrame-startFrame-1]=0.f;

			skeleton().setPose(pose(endFrame-1));
			skeleton().GetSiteBone(skeleton().GetJointIndexFromConstraint(aConstraint[con])).getTranslation(temp);
			footPosition[con][endFrame-startFrame-1].assign(temp);
		}

		undefinedGrp.runLengthEncode(undefined);

		// fill undefined importance and foot positions
		for(int i=0; i<undefinedGrp.size()/2; i++)
		{
			int start=undefinedGrp[i*2];
			int end=undefinedGrp[i*2+1];

			vectorn& footStart=footPosition[con][start-1];
			vectorn& footEnd=footPosition[con][end];

			time.uniform(0, 1, end-start);
			signal.op1(v1::each(s1::SMOOTH_TRANSITION), time);

			for(int iframe=start; iframe<end; iframe++)
			{
				footPosition[con][iframe].interpolate(footStart, footEnd, signal[iframe-start]);
			}

			if(importance[con][start-1]==0 && importance[con][end]==0)
			{
				for(int iframe=start; iframe<end; iframe++)
					importance[con][iframe]=0;
			}
			else if(importance[con][start-1]==0)
			{
				for(int iframe=start; iframe<end; iframe++)
					importance[con][iframe]=ABS(time[iframe-start]);
			}
			else if(importance[con][end]==0)
			{
				for(int iframe=start; iframe<end; iframe++)
					importance[con][iframe]=ABS(1-time[iframe-start]);
			}
			else
			{
				for(int iframe=start; iframe<end; iframe++)
					importance[con][iframe]=ABS((time[iframe-start]-0.5)*2.f);
			}
		}
	}

	vector3 rootAdjust;
	for(int iframe=startFrame; iframe<endFrame; iframe++)
	{
		m_pSkeleton->setPose(pose(iframe));

		ImIKSolver::ApplyIK(*m_pSkeleton, footPosition[0][iframe-startFrame].toVector3(), footPosition[1][iframe-startFrame].toVector3(),
			importance[0][iframe-startFrame], importance[1][iframe-startFrame], rootAdjust, index, delta_rot);

		pose(iframe).m_aTranslations[0]+=rootAdjust;

		for(int i=0; i<index.size(); i++)
			pose(iframe).m_aRotations[index[i]].leftMult(delta_rot[i]);
	}
}
*/

/*
void Motion::adjustRootToConstraints(int startFrame, int endFrame, const intvectorn& aConstraint, const CTArray<matrixn>& aConPositions, CTArray<intvectorn>&  left, CTArray<intvectorn>& right)
{
	int numCon=aConstraint.size();
	for(int i=0; i<numCon; i++)
		right[i].push_back(-1);

	intvectorn curInterval(numCon);
	curInterval.setAllValue(0);

	CTArray<intvectorn> abConstraint;

	abConstraint.Init(numCon);
	for(int con=0; con<numCon; con++)
	{
		abConstraint[con].setSize(endFrame-startFrame);
		abConstraint[con].setAllValue(-1);	// means no constraint
	}

	// 매프레임마다 몇번째 foot print에 해당하는지를 저장한다.
	for(int i=startFrame; i<endFrame; i++)
	{
		for(int con=0; con<numCon; con++)
		{
			if(curInterval[con]<left[con].size())
			{
				if( i >= right[con][curInterval[con]]) curInterval[con]++;
				if(i<right[con][curInterval[con]] && left[con][curInterval[con]]<=i)
					abConstraint[con][i-startFrame]=curInterval[con];
			}
		}
	}

	for(int i=0; i<numCon; i++)
		VERIFY(right[i].popBack()==-1);

	matrixn rootDisplacement;
	rootDisplacement.setSize(0, 3);
	intvectorn displacementIndex(endFrame-startFrame);
	displacementIndex.setAllValue(-1);
	matrixn deltaConstraint;

//#define SAVE_SIGNAL
#ifdef SAVE_SIGNAL
	matrixn saveSignal, signal2;
	saveSignal.setSize(endFrame-startFrame, 3);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1);
#endif
	// for each constraint, retarget.
	int count=0;
	for(int i=startFrame; i<endFrame; i++)
	{
		m_pSkeleton->setPose(pose(i));
		deltaConstraint.setSize(0,3);

		for(int con=0; con<numCon; con++)
		{
			int cur_footstep;
			if((cur_footstep=abConstraint[con][i-startFrame])!=-1)
			{
				vectorn deltaPos;
				vector3 curPos;
				m_pSkeleton->GetSiteBone(m_pSkeleton->GetJointIndexFromConstraint(aConstraint[con])).getTranslation(curPos);
				deltaPos.sub(aConPositions[con][cur_footstep], curPos);
				deltaConstraint.pushBack(deltaPos);
			}
		}

		if(deltaConstraint.rows()==1)
		{
			rootDisplacement.resize(count+1,3);
			rootDisplacement.row(count).avg(deltaConstraint);
			pose(i).m_aTranslations[0]+=rootDisplacement.row(count).toVector3();

#ifdef SAVE_SIGNAL
			saveSignal[i-startFrame].assign(rootDisplacement.row(count));
			signal2[i-startFrame][0]=1;
#endif

			displacementIndex[i-startFrame]=count;
			count++;
		}
	}
	if(count==0) return;

	rootDisplacement.pushBack(vectorn(0,0,0));
	if(displacementIndex[0]==-1) displacementIndex[0]=count;
	if(displacementIndex[endFrame-startFrame-1]==-1) displacementIndex[endFrame-startFrame-1]=count;

	// root adjustment 구간을 만든다. left, right, numAdjust
	bitvectorn group;
	group.op(s2::EQUAL, displacementIndex, -1);
	intvectorn rootAdjust;
	rootAdjust.runLengthEncode(group);
	int numAdjust=rootAdjust.size()/2;
	vector3N displacementMap;

	for(int i=0; i<numAdjust; i++)
	{
		int start=rootAdjust[i*2]-1+startFrame;
		int end=rootAdjust[i*2+1]+startFrame;
#ifdef SAVE_SIGNAL
		signal2[start-startFrame][2]=0;
		signal2[end-startFrame][2]=1;
#endif
		if(end-start-1>0)
		{
			// transition
			displacementMap.transition(rootDisplacement[displacementIndex[start-startFrame]].toVector3(),
									rootDisplacement[displacementIndex[end-startFrame]].toVector3(), end-start-1);

			for(int iframe=start+1; iframe<end; iframe++)
			{
				pose(iframe).m_aTranslations[0]+=displacementMap[iframe-(start+1)];
#ifdef SAVE_SIGNAL
				saveSignal[iframe-startFrame].assign(displacementMap[iframe-(start+1)]);
				signal2[iframe-startFrame][1]=1.f;
#endif
			}

		}
	}
#ifdef SAVE_SIGNAL
	saveSignal.op1(m1::each(v1::concat()), signal2);
	saveSignal.op0(m0::drawSignals("saveRootSignal.bmp",-1, 1, true));
#endif
}
*/
/*
void Motion::retargetingConstraints(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, intvectorn& left, intvectorn& right)
{

	right.push_back(-1);
	int currFootPrint=0;

	intvectorn iFootPrints;

	iFootPrints.setSize(endSafe-startSafe);
	iFootPrints.setAllValue(-1);	// means no constraint

#define setFootPrint(currFrame, currCon) iFootPrints[(currFrame)-startSafe]=(currCon)
#define getFootPrint(currFrame)	iFootPrints[(currFrame)-startSafe]

	for(int i=0; i<footPrints.rows(); i++)
	{
		for(int j=left[i]; j<right[i]; j++)
			setFootPrint(j, i);
	}

	intvectorn index;
	quaterN displacement[3];
	quaterN delta_rot;
	intvectorn displacementIndex(endSafe-startSafe);
	displacementIndex.setAllValue(-1);	// means no IK.

#define setDisplacement(currFrame, displacement) displacementIndex[(currFrame)-startSafe]=(displacement)
#define getDisplacement(currFrame) displacementIndex[(currFrame)-startSafe]

//#define SAVE_SIGNAL
#ifdef SAVE_SIGNAL
	matrixn saveSignal, signal2;
	saveSignal.setSize(endFrame-startFrame, 4);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1);
#endif
	int conCount=iFootPrints.count(s2::NOTEQUAL, -1);
	displacement[0].setSize(conCount+1);
	displacement[1].setSize(conCount+1);
	displacement[2].setSize(conCount+1);


	int count=0;

	// 일단 ik를 수행후 결과를 저장한다.
	for(int i=startFrame; i<endFrame; i++)
	{
		if(getFootPrint(i)!=-1)
		{
			m_pSkeleton->setPose(pose(i));

			ImIKSolver::ApplyIK(*m_pSkeleton, con, true, footPrints[getFootPrint(i)].toVector3(), index, delta_rot);

			m_real angle1=(pose(i).m_aRotations[index[2]]*delta_rot[2]).rotationAngle();
			m_real angle2=delta_rot[2].rotationAngle();

			//if(SQR(angle1)<0.1 && SQR(angle2) >0.1 )
			//if(SQR(angle1)<0.1 )|| ( SQR(angle1)<0.2 && SQR(angle2) >0.2))
			if(0)
			{
				//SetConstraint(i, con, false);

				// if leg is too straight, ignore IK
				int cur_footstep=getFootPrint(i);
				if(i==left[cur_footstep])
				{
					if(left[cur_footstep]+1<right[cur_footstep])
					{
						left[cur_footstep]++;
						setFootPrint(i,-1);
						continue;
					}
				}
				else
				{
					for(int k=i; k<right[cur_footstep]; k++)
						setFootPrint(k,-1);
					right[cur_footstep]=i;
					continue;
				}
			}
			setDisplacement(i, count);
			displacement[0][count]=delta_rot[1];
			displacement[1][count]=delta_rot[2];
			displacement[2][count]=delta_rot[3];
			count++;

			pose(i).m_aRotations[index[1]].leftMult(delta_rot[1]);
			pose(i).m_aRotations[index[2]].leftMult(delta_rot[2]);
			pose(i).m_aRotations[index[3]].leftMult(delta_rot[3]);
#ifdef SAVE_SIGNAL
			saveSignal[i-startFrame].assign(delta_rot[3]);
			signal2[i-startFrame][0]=1.f;
#endif
		}
	}

	if(count==0) return ;

	displacement[0][count]=quater(1,0,0,0);
	displacement[1][count]=quater(1,0,0,0);
	displacement[2][count]=quater(1,0,0,0);

	for(int i=startSafe; i<endSafe; i++)
	{
		if(getDisplacement(i)==-1)
			setDisplacement(i, count);
	}

	int numFootPrint= left.size();

	quaterN displacementMap;


	for(int i=0; i<=numFootPrint; i++)
	{
		int start=(i==0)?startSafe:right[i-1]-1;
		int end=(i==numFootPrint)?endSafe-1:left[i];

		if(end<startFrame) continue;
		if(start>=endFrame) continue;

		//printf("[%d,%d)>[%d,%d)", start, end, startFrame, endFrame);
#ifdef SAVE_SIGNAL
		signal2[start-startFrame][2]=0;
		signal2[end-startFrame][2]=1;
#endif
		for(int j=0; j<3; j++)
		{
			if(end-start-1>0)
			{
				// transition version (c0 continuous)
				if(getDisplacement(start)==count||
					getDisplacement(end)==count)
					displacementMap.transition(displacement[j][getDisplacement(start)],
												displacement[j][getDisplacement(end)], end-start-1);
				else
                    displacementMap.transition0(displacement[j][getDisplacement(start)],
												displacement[j][getDisplacement(end)], end-start-1);

				for(int iframe=start+1; iframe<end; iframe++)
				{
					if(iframe<numFrames())
						pose(iframe).m_aRotations[index[j+1]].leftMult(displacementMap[iframe-(start+1)]);
#ifdef SAVE_SIGNAL
					saveSignal[iframe-startFrame].assign(displacementMap[iframe-(start+1)]);
					signal2[iframe-startFrame][1]=1.f;
#endif
				}



			}
		}
	}
#ifdef SAVE_SIGNAL
	saveSignal.op1(m1::each(v1::concat()), signal2);
	CString filename;
	filename.Format("saveSignal%d.bmp", con);
	saveSignal.op0(m0::drawSignals(filename,-1, 1, true));

	saveSignal.setSize(endFrame-startFrame, 4);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1.f);
#endif
}
*/

/*
// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
void Motion::retargetingConstraints(int startSafe, int startFrame, int endFrame, int endSafe)
{
	matrixn conPos;
	intvectorn left;
	intvectorn right;

	GetFootPrints(startSafe, endSafe, conPos, CONSTRAINT_LEFT_FOOT, left, right);
	retargetingConstraints(startSafe, startFrame, endFrame, endSafe, CONSTRAINT_LEFT_FOOT, conPos, left, right);

	GetFootPrints(startSafe, endSafe, conPos, CONSTRAINT_RIGHT_FOOT, left, right);
	retargetingConstraints(startSafe, startFrame, endFrame, endSafe, CONSTRAINT_RIGHT_FOOT, conPos, left, right);
}
*/

/// MOTION RETARGETTING


