#include "stdafx.h"
#include "Motion.h"
#include "MotionLoader.h"
#include "../image/ImageProcessor.h"
#include "./MotionUtil.h"
#include "../math/hyperMatrixN.h"
#include "../math/Operator.h"
#include "../math/Filter.h"
#include "../math/Metric.h"
using namespace MotionUtil;

#include <iostream>
using namespace std;
GetSignal::GetSignal(MotionWrap const& in)
	:m_Motion(in)
{}



GetSignal::GetSignal(const Motion& in)
	:m_Motion(in)
{
}
GetSignal::GetSignal(const MotionDOF& in)
	:m_Motion(in)
{
}

// retrieve joint translations	-> Only for translational joints!!!
// out: (end-start) by (ajoint.size()*3) matrix
// to retrieve the signal of the joint ajoint[i].
//  -> vec3ViewCol(out, i*3)
//  or out.range(startF, endF, i*3, (i+1)*3).toVector3N()
void GetSignal::transJointAll(const intvectorn& ajoint, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames())
		end=m_Motion.numFrames();

	out.setSize(end-start, 3*ajoint.size());

	Motion const& mot=m_Motion.mot();
	// get joints
	for(int k=0; k<ajoint.size();k++)
	{
		int j=ajoint[k];
		vector3NView outJ(out.range(0, out.rows(), k*3, (k+1)*3).toVector3N());
		for(int i=start; i<end; i++)
			outJ.row(i-start)=mot.pose(i).m_aTranslations[j];
	}
}

void GetSignal::jointAll(const intvectorn& ajoint, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames())
		end=m_Motion.numFrames();

	out.setSize(end-start, 4*ajoint.size());

	Motion const& mot=m_Motion.mot();
	// get joints
	for(int k=0; k<ajoint.size();k++)
	{
		int j=ajoint[k];
		quaterNView outJ(out.range(0, out.rows(), k*4, (k+1)*4).toQuaterN());
		for(int i=start; i<end; i++)
			outJ.row(i-start)=mot.pose(i).m_aRotations[j];
	}
}

void SetSignal::jointAll(const intvectorn& ajoint, matrixn const& in, int start)
{
	Motion const& mot=m_Motion;
	// set joints
	for(int k=0; k<ajoint.size();k++)
	{
		int j=ajoint[k];
		quaterNView inJ(in.range(0, in.rows(), k*4, (k+1)*4).toQuaterN());
		for(int i=0; i<in.rows(); i++)
		{
			mot.pose(i+start).m_aRotations[j]=inJ.row(i);
		}
	}
}

		// set joint translations (for translational joints only)
		// out: (end-start) by (ajoint.size()*3) matrix
		// to retrieve the signal of the joint ajoint[i].
		//   -> vec3ViewCol(out, i*3)
		//   or out.range(startF, endF, i*3, (i+1)*3).toVector3N() 
void SetSignal::transJointAll(const intvectorn& ajoint, matrixn const& in, int start)
{
	Motion const& mot=m_Motion;
	// set joints
	for(int k=0; k<ajoint.size();k++)
	{
		int j=ajoint[k];
		vector3NView inJ(in.range(0, in.rows(), k*3, (k+1)*3).toVector3N());
		for(int i=0; i<in.rows(); i++)
		{
			mot.pose(i+start).m_aTranslations[j]=inJ.row(i);
		}
	}
}

void GetSignal::jointGlobalAll(int eRootCoord, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames())
		end=m_Motion.numFrames();

	Motion const& mot=m_Motion.mot();
	
	out.setSize(end-start, 4*mot.NumJoints());

	// get root
	quaterNView root(out.range(0, out.rows(), 0, 4).toQuaterN());

	if(eRootCoord==GLOBAL_COORD)
	{
		for(int i=start; i<end; i++)
			root.row(i-start)=mot.pose(i).m_aRotations[0];
	}
	else
	{
		for(int i=start; i<end; i++)
			root.row(i-start).identity();
	}

	// get joints
	for(int j=1; j<mot.NumJoints(); j++)
	{
		int parentJoint=mot.Parent(j);
		quaterNView outJ(out.range(0, out.rows(), j*4, (j+1)*4).toQuaterN());
		quaterNView parentJ(out.range(0, out.rows(), parentJoint*4, (parentJoint+1)*4).toQuaterN());
		for(int i=start; i<end; i++)
			outJ.row(i-start).mult(parentJ.row(i-start), mot.pose(i).m_aRotations[j]);
	}
}
void SetSignal::jointGlobalAll(int eRootCoord, matrixn const& in, int start)
{
	// set root
	quaterNView root(in.range(0, in.rows(), 0, 4).toQuaterN());

	Motion const& mot=m_Motion;
	if(eRootCoord==GLOBAL_COORD)
	{
		for(int i=0; i<in.rows(); i++)
			mot.pose(i+start).m_aRotations[0]=root.row(i);
	}
	
	// set joints
	for(int j=1; j<mot.NumJoints(); j++)
	{
		int parentJoint=mot.Parent(j);
		quaterNView inJ(in.range(0, in.rows(), j*4, (j+1)*4).toQuaterN());
		quaterNView parentJ(in.range(0, in.rows(), parentJoint*4, (parentJoint+1)*4).toQuaterN());
		for(int i=0; i<in.rows(); i++)
		{
			mot.pose(i+start).m_aRotations[j].mult(parentJ.row(i).inverse(), inJ.row(i));
		}
	}
}

void GetSignal::root(matrixn& out, int start, int end)
{
	Motion const& mot=m_Motion.mot();
if(end>m_Motion.numFrames()) 
	end=m_Motion.numFrames();

out.setSize(end-start,3);
for(int i=start; i<end; i++)
	out.row(i-start).assign(mot.pose(i).m_aTranslations[0]);
}

		
void GetSignal::additionalLinear(matrixn& out, int start, int end)
{
	Motion const& mot=m_Motion.mot();
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start,mot.pose(start).m_additionalLinear.size());
	for(int i=start; i<end; i++)
		out.row(i-start).assign(mot.pose(i).m_additionalLinear);
}

void GetSignal::additionalQuater(matrixn& out, int start, int end)
{
	Motion const& mot=m_Motion.mot();
	if(end>m_Motion.numFrames()) 
	end=m_Motion.numFrames();

	out.setSize(end-start,mot.pose(start).m_additionalQuater.size());
	for(int i=start; i<end; i++)
		out.row(i-start).assign(mot.pose(i).m_additionalQuater);

}

void SetSignal::additionalLinear(matrixn const& in, int start)
{
	Motion & mot=m_Motion;
	int end=start+in.rows();
	for(int i=start; i<end; i++)
		mot.pose(i).m_additionalLinear=in.row(i-start);
}

void SetSignal::additionalQuater(matrixn const& in, int start)
{
	Motion & mot=m_Motion;
	int end=start+in.rows();
	for(int i=start; i<end; i++)
		mot.pose(i).m_additionalQuater=in.row(i-start);
}
void GetSignal::jointGlobal(int ijoint, matrixn& out, int start, int end)	
{
	jointGlobal(m_Motion.mot().skeleton().getBoneByRotJointIndex(ijoint), out, start, end);
}

void GetSignal::jointPos(int ijoint, matrixn& out, int start, int end)			
{
	jointPos(m_Motion.mot().skeleton().getBoneByRotJointIndex(ijoint), out, start, end);
}

void GetSignal::root(vector3N& out, int start, int end)
{
if(end>m_Motion.numFrames()) 
	end=m_Motion.numFrames();

Motion const& mot=m_Motion.mot();
out.setSize(end-start);
for(int i=start; i<end; i++)
	out[i-start]=mot.pose(i).m_aTranslations[0];
}

void GetSignal::joint(int ijoint, matrixn& out, int start, int end)
{
if(end>m_Motion.numFrames()) 
	end=m_Motion.numFrames();

Motion const& mot=m_Motion.mot();
out.setSize(end-start,4);
for(int i=start; i<end; i++)
	out.row(i-start).assign(mot.pose(i).m_aRotations[ijoint]);
}

void GetSignal::transJoint(int ijoint, matrixn& out, int start, int end)
{
if(end>m_Motion.numFrames()) 
	end=m_Motion.numFrames();

Motion const& mot=m_Motion.mot();
out.setSize(end-start,3);
for(int i=start; i<end; i++)
	out.row(i-start).assign(mot.pose(i).m_aTranslations[ijoint]);
}

void GetSignal::transJoint(int ijoint, vector3N& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
	end=m_Motion.numFrames();

Motion const& mot=m_Motion.mot();
out.setSize(end-start);
for(int i=start; i<end; i++)
	out.row(i-start)=mot.pose(i).m_aTranslations[ijoint];
}


void GetSignal::joint(int ijoint, quaterN& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

Motion const& mot=m_Motion.mot();
	out.setSize(end-start);
	for(int i=start; i<end; i++)
		out.row(i-start)=mot.pose(i).m_aRotations[ijoint];
}

void GetSignal::jointGlobal(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start,4);
	for(int i=start; i<end; i++)
	{
		m_Motion.skeleton().setPose(m_Motion.pose(i));
		quater rot;
		bone.getRotation(rot);
		out.row(i-start).assign(rot);
	}
}

void MotionUtil::GetSignal::constraintPositions(matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
	end=m_Motion.numFrames();

	out.setSize(end-start,6);

	for(int i=start; i<end; i++)
	{
		out.row(i-start).setVec3(0, m_Motion.mot().pose(i).m_conToeL);
		out.row(i-start).setVec3(3, m_Motion.mot().pose(i).m_conToeR);
	}
}


void GetSignal::jointFixed(int ijoint, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start,4);
	Posture pose;
	Motion const& mot=m_Motion.mot();
	for(int i=start; i<end; i++)
	{
		pose=mot.pose(i);
		pose.decomposeRot();
		pose.m_aTranslations[0]=vector3(0,pose.m_aTranslations[0].y, 0);
		pose.m_aRotations[0]=pose.m_offset_q;

		m_Motion.skeleton().setPose(pose);

		quater rot;

		m_Motion.skeleton().getBoneByRotJointIndex(ijoint).getRotation(rot);
		out.row(i-start).assign(rot);
	}
}

void GetSignal::jointPos(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start,3);
	for(int i=start; i<end; i++)
	{
		m_Motion.skeleton().setPose(m_Motion.pose(i));
		vector3 pos;
		bone.getTranslation(pos);
		out.row(i-start).assign(pos);
	}
}

void GetSignal::jointOri(const intvectorn& aJointIndex, hypermatrixn& aaPos, int eCoord, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	aaPos.setSize(aJointIndex.size(), end-start, 4);

	for(int i=0; i<aJointIndex.size(); i++)
	{
		switch(eCoord)
		{
		case GLOBAL_COORD:
			jointGlobal(aJointIndex[i], aaPos[i], start, end);
			break;
		case LOCAL_COORD:
			joint(aJointIndex[i], aaPos[i], start, end);
			break;
		case FIXED_COORD:
			jointFixed(aJointIndex[i], aaPos[i], start, end);
			break;
		default:
			ASSERT(0);
		}
	}
}

void GetSignal::jointPos(const intvectorn& aJointIndex, hypermatrixn& aaPos, int eCoord, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	aaPos.setSize(aJointIndex.size(), end-start, 3);

	for(int i=0; i<aJointIndex.size(); i++)
	{
		switch(eCoord)
		{
		case GLOBAL_COORD:
			jointPos(aJointIndex[i], aaPos[i], start, end);
			break;
		case LOCAL_COORD:
			jointPosLocal(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i], start, end);
			break;
		case FIXED_COORD:
			jointPosFixed(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i], start, end);
			break;
		case FIRST_FRAME_CENTERED_COORD:
			jointPosFirstCentered(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i], start, end);
			break;
		}
	}
}

void GetSignal::jointPosLocal(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start,3);
	for(int i=start; i<end; i++)
	{
		m_Motion.skeleton().setPose(m_Motion.pose(i));
		vector3 lpos, pos;
		bone.getTranslation(pos);
		pos-=m_Motion.mot().pose(i).m_aTranslations[0];
		lpos.rotate(m_Motion.mot().pose(i).m_aRotations[0].inverse(), pos);
		out.row(i-start).assign(lpos);
	}
}

void GetSignal::jointPosFirstCentered(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start,3);
	Posture pose;
	vector3 pos;

	quater q;
	vector3 trans;
	m_Motion.mot().pose(start).decomposeRot();
	q.difference(m_Motion.mot().pose(start).m_rotAxis_y , quater(1,0,0,0));
	trans.difference(m_Motion.mot().pose(start).m_aTranslations[0], vector3(0, m_Motion.mot().pose(start).m_aTranslations[0].y,0));

	for(int i=start; i<end; i++)
	{
		pose=m_Motion.mot().pose(i);
		pose.m_aTranslations[0]+=trans;
		pose.m_aTranslations[0].rotate(q);
		pose.m_aRotations[0].leftMult(q);
		
		m_Motion.skeleton().setPose(pose);
		bone.getTranslation(pos);
		out.row(i-start).assign(pos);
	}
}


void GetSignal::jointPosFixed(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start,3);
	for(int i=start; i<end; i++)
	{
		m_Motion.skeleton().setPose(m_Motion.pose(i));
		vector3 lpos, pos;
		bone.getTranslation(pos);
		pos-=m_Motion.mot().pose(i).m_aTranslations[0];

		quater q, rotY, rotXZ;
		m_Motion.mot().pose(i).m_aRotations[0].decomposeTwistTimesNoTwist(vector3(0,1,0), rotY, rotXZ);

		lpos.rotate(rotY.inverse(), pos);

		lpos.y+=m_Motion.mot().pose(i).m_aTranslations[0].y;
		out.row(i-start).assign(lpos);
	}
}

void GetSignal::constraint(int iconstraint, bitvectorn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.resize(end-start);

	for(int i=start; i<end; i++)
		out.set(i-start, m_Motion.mot().isConstraint(i, iconstraint));
}


void GetSignals::jointVel(const Bone& bone, matrixn& aVel, int eCoord, float fSmoothKernel)
{
	MotionUtil::SegmentFinder seg(m_Motion, 0, m_Motion.numFrames());

	GetSignal sig(m_Motion);
	aVel.setSize(m_Motion.numFrames(), 3);

	matrixn aSegPos;
	matrixn aSegVel;
    for(int iseg=0; iseg< seg.numSegment(); iseg++)
	{
		int stt=seg.startFrame(iseg);
		int end=seg.endFrame(iseg);

		switch(eCoord)
		{
		case GLOBAL_COORD:
			sig.jointPos(bone, aSegPos, stt, end);
			break;
		case LOCAL_COORD:
			sig.jointPosLocal(bone, aSegPos, stt, end);
			break;
		case FIXED_COORD:
			sig.jointPosFixed(bone, aSegPos, stt, end);
			break;
		}
		
		m::derivative(aSegVel,aSegPos);

		if(fSmoothKernel!=0.f)
			m::filter(aSegVel, Filter::CalcKernelSize(fSmoothKernel, m_Motion.frameTime()));
		
		aVel.setValue(stt, 0, aSegVel);
	}
}

void GetSignals::jointPosVel(const Bone& bone, matrixn& aPos, matrixn& aVel, int eCoord, float fSmoothKernel)
{
	MotionUtil::SegmentFinder seg(m_Motion, 0, m_Motion.numFrames());

	GetSignal sig(m_Motion);
	aPos.setSize(m_Motion.numFrames(), 3);
	aVel.setSize(m_Motion.numFrames(), 3);

	matrixn aSegPos;
	matrixn aSegVel;
    for(int iseg=0; iseg< seg.numSegment(); iseg++)
	{
		int stt=seg.startFrame(iseg);
		int end=seg.endFrame(iseg);

		switch(eCoord)
		{
		case GLOBAL_COORD:
			sig.jointPos(bone, aSegPos, stt, end);
			break;
		case LOCAL_COORD:
			sig.jointPosLocal(bone, aSegPos, stt, end);
			break;
		case FIXED_COORD:
			sig.jointPosFixed(bone, aSegPos, stt, end);
			break;
		}
		
		m::derivative(aSegVel,aSegPos);

		if(fSmoothKernel!=0.f)
			m::filter(aSegVel, Filter::CalcKernelSize(fSmoothKernel, m_Motion.frameTime()));
		
		aPos.setValue(stt, 0, aSegPos);
		aVel.setValue(stt, 0, aSegVel);
	}
}

void GetSignals::jointPos(const intvectorn& aJointIndex, hypermatrixn& aaPos, int eCoord)
{
	aaPos.setSize(aJointIndex.size(), m_Motion.numFrames(), 3);

	GetSignal sig(m_Motion);
	for(int i=0; i<aJointIndex.size(); i++)
	{
		switch(eCoord)
		{
		case GLOBAL_COORD:
			sig.jointPos(aJointIndex[i], aaPos[i]);
			break;
		case LOCAL_COORD:
			sig.jointPosLocal(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i]);
			break;
		case FIXED_COORD:
			sig.jointPosFixed(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i]);
			break;
		}
	}	
}

void GetSignals::jointVel(const intvectorn& aJointIndex, hypermatrixn& aaVel, int eCoord, float fSmooth)
{
	aaVel.setSize(aJointIndex.size(), m_Motion.numFrames(), 3);
	
	matrixn temp;
	for(int i=0; i<aJointIndex.size(); i++)
	{
		jointVel(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), temp, eCoord, fSmooth);
		aaVel[i].assign(temp);
	}
}

void SetSignal::root(const matrixn& in)
{
for(int i=0; i<m_Motion.numFrames(); i++)
	m_Motion.pose(i).m_aTranslations[0]=in.row(i).toVector3();
}

void SetSignal::constraint(int iconstraint, bitvectorn const& con, int start)
{
	for(int i=0; i<con.size(); i++)
		m_Motion.setConstraint(i+start, iconstraint, con[i]);
}

void SetSignal::root(const vector3N& in, int start)
{
for(int i=0; i<in.rows(); i++)
	m_Motion.pose(i+start).m_aTranslations[0]=in[i];
}

void SetSignal::joint(int ijoint, const matrixn& in)
{
for(int i=0; i<m_Motion.numFrames(); i++)
	m_Motion.pose(i).m_aRotations[ijoint]=in.row(i).toQuater();
}

void SetSignal::joint(int ijoint, const quaterN& in, int start)
{
for(int i=0; i<in.rows(); i++)
	m_Motion.pose(i+start).m_aRotations[ijoint]=in[i];
}

void SetSignal::transJoint(int ijoint, const matrixn& in)
{
	for(int i=0; i<in.rows(); i++)
		m_Motion.pose(i).m_aTranslations[ijoint]=in.row(i).toVector3();
}

void SetSignal::transJoint(int ijoint, const vector3N& in, int start)
{
	for(int i=0; i<in.rows(); i++)
		m_Motion.pose(i+start).m_aTranslations[ijoint]=in[i];
}


m_real MotionUtil::transitionCost(const Motion& mMotion, int from, int to, int interval)
{
	//!<  from까지 play하고 to+1부터 play하는 경우 transition cost
	
	// find valid left range 
	int i, left_i, right_i;
	for(i=0; i>=-1*interval ;i--)
	{
		if(!(i+from-1>=0 && i+from-1<mMotion.numFrames() &&	i+to-1>=0 && i+to-1<mMotion.numFrames())
		||	(mMotion.isDiscontinuous(i+from) || mMotion.isDiscontinuous(i+to)) )
			break;
	}
	left_i=i;

	// find valid right range
	for(i=-1; i<interval ;i++)
	{
		if(!(i+from+1>=0 && i+from+1<mMotion.numFrames() &&	i+to+1>=0 && i+to+1<mMotion.numFrames())
		||	(mMotion.isDiscontinuous(i+from+1) || mMotion.isDiscontinuous(i+to+1)) )
			break;
	}
	right_i=i;

	int size;
	size=right_i-left_i+1;

	if(size<2)	return FLT_MAX;

	vectorn pointsA, pointsB;
	
	intvectorn EEIndex;
	
	EEIndex.setSize(13);
	int ind=0;
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::HIPS);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTANKLE);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTANKLE);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTKNEE);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTKNEE);	
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTWRIST);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTWRIST);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTELBOW);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTELBOW);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTSHOULDER);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTSHOULDER);	
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::CHEST2);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexByVoca(MotionLoader::HEAD);

	intvectorn& jointIndex=EEIndex;

	static hypermatrixn aaPosA, aaPosB;

	MotionUtil::GetSignal sig(mMotion);
	
	sig.jointPos(jointIndex, aaPosA, true, left_i+from, right_i+from+1);
	sig.jointPos(jointIndex, aaPosB, true, left_i+to, right_i+to+1);
	
	static matrixn temp;
	pointsA.fromMatrix(temp.fromHyperMat(aaPosA));
	pointsB.fromMatrix(temp.fromHyperMat(aaPosB));

	static KovarMetric metric;
	m_real distance=metric.CalcDistance(pointsA, pointsB);
	//distance : 최소 L2Distance
	//normalize by size-> sqrt(distance*distance/size);
	return distance*sqrt(1/(m_real)size);
}


void MotionUtil::upsample(Motion& out, const Motion& in, int startFrame, int endFrame, int nSuperSample)
{
	out.SetIdentifier(in.GetIdentifier());
	// in has no discontinuity
	ASSERT(in.IsValid(startFrame, endFrame));

	GetSignal signal(in);
	SetSignal signalOut(out);

	int inNumFrame=endFrame-startFrame;
	out.InitEmpty(in, inNumFrame*nSuperSample);

	matrixn root(inNumFrame,3);
	hypermatrixn joint(in.NumJoints(), inNumFrame, 4);

	matrixn rootOut(out.numFrames(),3);
	hypermatrixn jointOut(in.NumJoints(), out.numFrames(), 4);

	signal.root(root, startFrame, endFrame);
	m::superSampling(nSuperSample, rootOut,root);
	signalOut.root(rootOut);

	for(int ijoint=0; ijoint<in.NumJoints(); ijoint++)
	{
		signal.joint(ijoint, joint[ijoint], startFrame, endFrame);
		m::alignQuater(joint[ijoint]);
		m::superSampling(nSuperSample,jointOut[ijoint], joint[ijoint]);
		for(int k=0; k<jointOut[ijoint].rows(); k++)
			jointOut[ijoint].row(k).normalize();
		signalOut.joint(ijoint, jointOut[ijoint]);
	}
	
}

void MotionUtil::upsample(Motion& out, const Motion& in, int nSuperSample)
{
	out.SetIdentifier(in.GetIdentifier());

	MotionUtil::SegmentFinder sf(in,0, in.numFrames());
	out.InitEmpty(in, 0);		
	out.frameTime(in.frameTime()/(float)nSuperSample);
	Motion temp;
	for(int iseg=0; iseg<sf.numSegment(); iseg++)
	{
		MotionUtil::upsample(temp, in, sf.startFrame(iseg), sf.endFrame(iseg), nSuperSample);
		out.Concat(&temp);	
	}

	ASSERT(out.numFrames()==in.numFrames()*nSuperSample);
}

void swap(quater& a, quater& b)
{
	quater c;
	c=a;
	a=b;
	b=c;
}

//mirror reflection
void MotionUtil::transpose(Motion& out, const Motion& in)
{
	// 생각보다 복잡하게 구현되었다. 간단하게는 그리는 코드에서 (1,1,-1) 스케일 매트릭스 곱해주는 것으로 가능하지만(space inversion),
	// 이는 quaternion으로 표현할 수 없는 변환이다.
	// 따라서 실제로는 quaternion은 mirror reflection시키고, 본을 좌우를 바꿔 달아주는 방법으로 구현하였다.
	out.InitEmpty(in, in.numFrames());

	Posture globalpose;
	globalpose.Init(in.NumJoints(), in.numTransJoints());

	quater rotAxisY;
	quater rotAxisX;
	quater offset_q;

	for(int i=0; i<in.numFrames(); i++)
	{
		globalpose=in.pose(i);
		in.skeleton().setPose(globalpose);
		
		for(int j=0; j<in.NumJoints(); j++)
			in.skeleton().getBoneByRotJointIndex(j).getRotation(globalpose.m_aRotations[j]);

		// transpose global pose.
		//  calculation rotAxis_y	global_rot = rotAxis_y*offset_q 
		// first transpose not-root joints
		for(int j=0; j<in.NumJoints(); j++)
		{
			quater& global_rot=globalpose.m_aRotations[j];

			global_rot.decomposeNoTwistTimesTwist(vector3(1,0,0), offset_q, rotAxisX);
			//y축으로 거꾸로 돌려 대칭 시킨다. 
			global_rot=offset_q.inverse()*rotAxisX;
		}

		// 아래 하드코딩 한 부분 고칠것- 엔드 이펙터에서 부터 시작해서 올라가면서 본이 같지 않은 동안만 바꿔주면 됨.
		// 그리고 위에 구현 틀린듯. 

		// 왼팔다리, 오른팔다리 바꿔끼우기.
		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTCOLLAR)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTCOLLAR)]);

		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTSHOULDER)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTSHOULDER)]);

		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTELBOW)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTELBOW)]);
		
		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTWRIST)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTWRIST)]);

		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTKNEE)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTKNEE)]);

		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTHIP)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTHIP)]);
		
		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::LEFTANKLE)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexByVoca(MotionLoader::RIGHTANKLE)]);

		out.pose(i).m_aRotations[0]=globalpose.m_aRotations[0];
		// convert to local pose.
		for(int j=1; j<in.NumJoints(); j++)
			out.pose(i).m_aRotations[j]=globalpose.m_aRotations[dep_GetParentJoint(in.skeleton(), j)].inverse()*globalpose.m_aRotations[j];

		out.pose(i).m_aTranslations[0]=in.pose(i).m_aTranslations[0];
		out.pose(i).m_aTranslations[0].x*=-1;

		if(in.isDiscontinuous(i))
			out.setDiscontinuity(i, true);
    }
}

// including self.
static void findChildren(MotionLoader& skeleton, int rootIndex, intvectorn& children)
{
	NodeStack stack;
	Node *src=skeleton.GetNode(rootIndex);	// dummy노드는 사용안함.
	children.resize(0);
	Bone* pBone=(Bone*)src;
	int treeIndex=pBone->treeIndex();
	children.pushBack(treeIndex);
	src=src->m_pChildHead;

	while(TRUE) 
	{
		while(src)
		{
			ASSERT(src->NodeType==BONE);
			Bone* pBone=(Bone*)src;
			cout <<pBone->name()<<endl;
			int treeIndex=pBone->treeIndex();
			children.pushBack(treeIndex);

			stack.Push(src);

			src=src->m_pChildHead;
		}
		stack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
	cout<<"children"<<children<<"\n";
}

intvectorn MotionUtil::findChildren(MotionLoader& skeleton, int rootIndex)
{
	intvectorn children;
	findChildren(skeleton, rootIndex, children);
	return children;
}
void MotionUtil::mirrorMotion(Motion& out, const Motion& in, intvectorn const& LrootIndices, intvectorn const& RrootIndices)
{
	out.InitEmpty(in, in.numFrames());

	Posture globalpose;
	globalpose.Init(in.NumJoints(), in.numTransJoints());

	quater rotAxisY;
	quater rotAxisX;
	quater offset_q;

	for(int i=0; i<in.numFrames(); i++)
	{
		globalpose=in.pose(i);
		in.skeleton().setPose(globalpose);
		
		for(int j=0; j<in.NumJoints(); j++)
			in.skeleton().getBoneByRotJointIndex(j).getRotation(globalpose.m_aRotations[j]);

		// transpose global pose.
		//  calculation rotAxis_y	global_rot = rotAxis_y*offset_q 
		// first transpose not-root joints
		for(int j=0; j<in.NumJoints(); j++)
		{
			quater& global_rot=globalpose.m_aRotations[j];

			/*
			global_rot.decomposeNoTwistTimesTwist(vector3(1,0,0), offset_q, rotAxisX);
			//y축으로 거꾸로 돌려 대칭 시킨다. 
			global_rot=offset_q.inverse()*rotAxisX;
			*/
			quater q=global_rot;
			global_rot=quater(q.w, q.x, -q.y, -q.z);
		}
		// convert to local pose.
		for(int j=1; j<in.NumJoints(); j++)
			out.pose(i).m_aRotations[j]=globalpose.m_aRotations[dep_GetParentJoint(in.skeleton(), j)].inverse()*globalpose.m_aRotations[j];

		out.pose(i).m_aTranslations[0]=in.pose(i).m_aTranslations[0];
		out.pose(i).m_aTranslations[0].x*=-1;
		out.pose(i).m_aRotations[0]=globalpose.m_aRotations[0];

		if(in.isDiscontinuous(i))
			out.setDiscontinuity(i, true);
	}

	// swap left and right bones
	for(int j=0;j<LrootIndices.size(); j++)
	{
		int Lroot=LrootIndices[j];
		int Rroot=RrootIndices[j];
		intvectorn Lchildren;
		intvectorn Rchildren;
		findChildren(in.skeleton(), Lroot, Lchildren);
		findChildren(in.skeleton(), Rroot, Rchildren);

		ASSERT(Lchildren.size()==Rchildren.size());

		for(int k=0; k<Lchildren.size(); k++)
		{
			int Lchild=Lchildren[k];
			int Rchild=Rchildren[k];

			for(int i=0; i<in.numFrames(); i++)
				// 왼팔다리, 오른팔다리 바꿔끼우기.
				swap(out.pose(i).m_aRotations[in.skeleton().getRotJointIndexByTreeIndex(Lchild)],
						out.pose(i).m_aRotations[in.skeleton().getRotJointIndexByTreeIndex(Rchild)]);
		}
    }
}

void MotionUtil::downsample(Motion& out, const Motion& in, int nDownSample)
{
	int len=0;
	for(int i=0; i<in.numFrames(); i+=nDownSample)
		len++;
    
	out.InitEmpty(in, len);
	out.frameTime(in.frameTime()*nDownSample);

	for(int i=0; i<in.numFrames(); i+=nDownSample)
	{
		out.pose(i/nDownSample)=in.pose(i);
		for(int j=0; j<nDownSample; j++)
		{
			if(i+j<in.numFrames() && in.isDiscontinuous(i+j))
			{
				out.setDiscontinuity(i/nDownSample, true);
				break;
			}
		}
	}
} 


static vector3 posVoca(MotionLoader const& s, int jointVoca) 
{ vector3 trans; s.getBoneByVoca(jointVoca).getTranslation(trans); return trans;}



void MotionUtil::smooth(Motion& out, const Motion& in, float kernelRoot, float kernelJoint)
{
	out.InitEmpty(in, in.numFrames());

	MotionUtil::SegmentFinder seg(in, 0, in.numFrames());

	matrixn rot, trans;
	vectorn rotKernel, transKernel;

	Filter::GetGaussFilter(Filter::KernelSize(in,kernelRoot), transKernel);
	Filter::GetGaussFilter(Filter::KernelSize(in,kernelJoint), rotKernel);

	int numIter=1;
	for(int iseg=0; iseg< seg.numSegment(); iseg++)
	{
		
		int stt=seg.startFrame(iseg);
		int end=seg.endFrame(iseg);

		MotionView input=in.range(stt, end);
		MotionView output=out.range(stt, end);

		// segment별로 따로 따로 스무딩해야한다.
		if(end-stt>MAX(transKernel.size(), rotKernel.size())+2)
		{
			GetSignal gs(input);
			SetSignal ss(output);
			
			for(int ijoint=0; ijoint<input.numRotJoints(); ijoint++)
			{
				gs.joint(ijoint, rot);
				Filter::LTIFilterQuat(numIter, rotKernel, rot);
				ss.joint(ijoint, rot);
			}

			for(int ijoint=0; ijoint<input.numTransJoints(); ijoint++)
			{
				gs.joint(ijoint, trans);
				Filter::LTIFilterQuat(numIter, transKernel, rot);
				ss.joint(ijoint, trans);
			}
		}
		else
		{
			output=input;
		}
	}
}

void MotionUtil::scale(Motion& inout, float ratio)
{
	//!< root translation을 scale한다.
	for(int i=0; i<inout.numFrames(); i++)
		inout.pose(i).m_aTranslations[0]*=ratio;
}

void MotionUtil::translate(Motion& inout, const vector3& trans, int start, int end)
{
	//!< Motion 전체를 Translate한다. 즉 root position의 translation
	if(start<0) start=0;
	if(end>inout.numFrames()) end=inout.numFrames();

	for(int i=start; i<end; i++)
	{
		inout.pose(i).m_aTranslations[0]+=trans;	
	}
}

void MotionUtil::rotate(Motion& inout, const quater& q, int start, int end )
{
	//!< MotionData 전체를 Rotate한다. root orientation quat앞에 곱해져서 제일 나중에 적용된다.
	if(start<0) start=0;
	if(end>inout.numFrames()) end=inout.numFrames();

	for(int i=start; i<end; i++)
	{
		inout.pose(i).m_aRotations[0].leftMult(q);
		inout.pose(i).m_aTranslations[0].rotate(q, inout.pose(i).m_aTranslations[0]);
	}
}

void MotionUtil::rigidTransform(Motion& inout, const transf& f, int start, int end)
{
	if(start<0) start=0;
	if(end>inout.numFrames()) end=inout.numFrames();

	for(int i=start; i<end; i++)
	{
		inout.pose(i).setRootTransformation(f*inout.pose(i).rootTransformation());
	}
}

void MotionUtil::timewarping(Motion& out, const Motion& srcMotion, const vectorn& timewarpFunction)
{
	// timewarp
	out.InitEmpty(srcMotion, timewarpFunction.size());

	for(int i=0; i<timewarpFunction.size(); i++)
	{
		// float 0.5 가 정확하게 integer 0에 mapping된다.
		// 즉 0.6등은 0과 1을 0에 가중치를 크게 둬서 섞은게 된다.
		// world좌표와 pixel좌표에 쓰는 float, integer converting scheme을 따랐다.

		srcMotion.SamplePose(out.pose(i), timewarpFunction[i]);
	}
}

void MotionUtil::timewarpingLinear(Motion& out, const Motion& srcMotion, const vectorn& timewarpFunction)
{
	// timewarp
	out.InitEmpty(srcMotion, timewarpFunction.size());

	for(int i=0; i<timewarpFunction.size(); i++)
	{
		srcMotion.samplePose(out.pose(i), timewarpFunction[i]);
	}
}

Motion* MotionUtil::untimewarpingLinear(const Motion& srcMotion, const vectorn& invtmwpFunction)
{
	Motion* out =new Motion();
	m_real minTime=invtmwpFunction[0];
	m_real maxTime=invtmwpFunction[invtmwpFunction.size()-1];

	ASSERT(srcMotion.numFrames()==invtmwpFunction.size());
	ASSERT(isSimilar(minTime,0.0));

	// 동작 길이가 정수가 되도록 rounding한다. 1을 더하는 이유는, 정수로 바꾸면 연속된 동작세그먼트간 1프레임 오버랩이 생기기 때문.
	int nDesiredLen=ROUND(maxTime)+1;

	vectorn invTmwpFunction(invtmwpFunction);
	invTmwpFunction*=1.0/maxTime*((m_real)nDesiredLen-1.0);
	
	ASSERT(nDesiredLen>=1);
	// timewarp
	out->InitEmpty(srcMotion, nDesiredLen);

#ifdef BUGGY
	int start, end;
	m_real criticalTime1;
	m_real criticalTime2;

	for(int i=0; i<invTmwpFunction.size()-1; i++)
	{
		criticalTime1=invTmwpFunction[i];
		criticalTime2=invTmwpFunction[i+1];
		
		start=ROUND(criticalTime1);
		end=ROUND(criticalTime2);

		int End;
		if(i==invTmwpFunction.size()-2)
			End=end+1;
		else 
			End=end;
        for(int j=start; j<End; j++)
		{
			m_real frac=((m_real)j-criticalTime1)/(criticalTime2-criticalTime1);
			//   0< frac+i <srcMotion.size()-1
			srcMotion.samplePose(out->pose(j), frac+i);
		}


	}

#ifdef _DEBUG

	for(int i=0; i<out->numFrames(); i++)
		ASSERT(out->pose(i).m_numJoint);
#endif
#else
#endif
	int curInterval=0;
	for(int i=0; i<nDesiredLen; i++)
	{
		m_real t=m_real (i);
		int j=curInterval+1;
		for(; j<invTmwpFunction.size(); j++)
		{
			if(invTmwpFunction[j]>=t-FERR)
				break;
		}

		curInterval=j-1;

		m_real frac=((m_real)i-invTmwpFunction[curInterval])/(invTmwpFunction[j]-invTmwpFunction[curInterval]);

		srcMotion.samplePose(out->pose(i), m_real(curInterval)+frac);
		

	}

	return out;
}

void MotionUtil::timewarping(Motion& inout, int playEnd, int criticalTimeBefore, int criticalTimeAfter)
{	
	//    playEnd         criticalTimeBefore
	// ---|---------------|-------|
	//    | adjustMot     |
	//                    |leftMot|


	// case 1:
	// ---|-----------|-------|
	//                criticalTimeAfter
	

	// case 2:
	// ---|-------------------|-------|
	//                criticalTimeAfter

	int leftMotionSize=inout.numFrames()-criticalTimeBefore;
	Motion temp2;
	vectorn timewarpFunction(criticalTimeAfter-playEnd);
	v::uniformSampling(timewarpFunction,(float)playEnd,(float)(criticalTimeBefore));
	MotionUtil::timewarping(temp2, inout, timewarpFunction);

	if(criticalTimeAfter>criticalTimeBefore)
	{
		// increasing size
		int numFrameOld=inout.numFrames();
		inout.Resize(inout.numFrames()+criticalTimeAfter-criticalTimeBefore);
		for(int i=numFrameOld-1; i>=criticalTimeBefore; i--)
			inout.pose(i-numFrameOld+inout.numFrames())=inout.pose(i);

		for(int i=playEnd; i<criticalTimeAfter; i++)
			inout.pose(i)=temp2.pose(i-playEnd);
	}
	else
	{
		// decreasing size
		for(int i=0; i<leftMotionSize; i++)
			inout.pose(criticalTimeAfter+i)=inout.pose(criticalTimeBefore+i);
		inout.Resize(inout.numFrames()+criticalTimeAfter-criticalTimeBefore);


		for(int i=playEnd; i<criticalTimeAfter; i++)
			inout.pose(i)=temp2.pose(i-playEnd);
	}
}

MotionUtil::SegmentFinder::SegmentFinder(const Motion& motion, int startFrame, int endFrame)
{
	if(startFrame<0) startFrame=0;
	if(endFrame>motion.numFrames()) endFrame=motion.numFrames();

	m_vStart.pushBack(startFrame);
	for(int i=startFrame+1; i<endFrame; i++)
	{
		if(motion.isDiscontinuous(i))
		{
			m_vEnd.pushBack(i);
			m_vStart.pushBack(i);
		}
	}
	m_vEnd.pushBack(endFrame);
	ASSERT(m_vStart.size()==m_vEnd.size());
}

void GetSignal::offsetQ(matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start, 4);

	// dv, drot
	for(int i=start; i<end; i++)
	{
		out.row(i-start)=m_Motion.mot().pose(i).m_offset_q;
	}
}

void MotionUtil::SetSignal::offsetQ(matrixn const& in, int start)
{
	int end=m_Motion.numFrames();

	if(end>in.rows()+start)
		end=in.rows()+start;

	// dv, drot
	for(int i=start; i<end; i++)
	{
		m_Motion.pose(i).m_offset_q=in.row(i-start).toQuater(0);
	}
}

void MotionUtil::GetSignal::interframeDifference(matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	// dv(2), dq(1), height (1), offset(4) total 8 dimensions

	out.setSize(end-start, 8);

	// dv, drot
	for(int i=start; i<end; i++)
	{
		Posture const& pose=m_Motion.mot().pose(i);
		out[i-start][0]=pose.m_dv.x;
		out[i-start][1]=pose.m_dv.z;
		out[i-start][2]=pose.m_dq.rotationAngle(vector3(0,1,0));
		out[i-start][3]=pose.m_offset_y;
		out.row(i-start).setQuater(4, pose.m_offset_q);
	}
}

void MotionUtil::SetSignal::interframeDifference(matrixn const& in, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	if(end>in.rows()+start)
		end=in.rows()+start;
	// dv, drot
	for(int i=start; i<end; i++)
	{
		m_Motion.pose(i).m_dv.x=in[i-start][0];
		m_Motion.pose(i).m_dv.z=in[i-start][1];
		m_Motion.pose(i).m_dq.setRotation(vector3(0,1,0), in[i-start][2]);
		m_Motion.pose(i).m_offset_y=in[i-start][3];		
		m_Motion.pose(i).m_offset_q=in.row(i-start).toQuater(4);
	}
}

void MotionUtil::SetSignal::constraintPositions(matrixn const& in, int start, int end)
{
	if(end>m_Motion.numFrames()) 
	end=m_Motion.numFrames();

	if(end>in.rows()+start)
		end=in.rows()+start;

	for(int i=start; i<end; i++)
	{
		m_Motion.pose(i).m_conToeL=in.row(i-start).toVector3(0);
		m_Motion.pose(i).m_conToeR=in.row(i-start).toVector3(3);
	}
}


void MotionUtil::exportBVH(Motion const& mot, const char* filename, int start, int end)
{
	if(start<0) start=0;
	if(end>mot.numFrames()) end=mot.numFrames();


	FILE* file;
	file=fopen(filename, "w");
	mot.skeleton().getBoneByRotJointIndex(0).packBVH(file, 0, &mot.skeleton());

	fprintf(file, "\nMOTION\nFrames: %d\n", end-start);
	fprintf(file, "Frame Time: %f\n", mot.frameTime());

	Bone* pBone;

	m_real eulerAngles[3];

	TString tc, channels;
	for(int iframe=start; iframe<end; iframe++)
	{

		for(int ibone=1; ibone<mot.skeleton().numBone(); ibone++)
		{
			pBone=&mot.skeleton().bone(ibone);
			tc=pBone->getTranslationalChannels();

			for(int i=0; i<tc.length(); i++)
			{
				vector3& v=mot.pose(iframe).m_aTranslations[pBone->transJointIndex()];
				fprintf(file, "%g ", v[tc[i]-'X']);
			}

			channels=pBone->getRotationalChannels();

			if(channels.length()>0)
			{
				mot.pose(iframe).m_aRotations[pBone->rotJointIndex()].getRotation(channels, eulerAngles);

				for(int c=0; c<channels.length(); c++)
					fprintf(file, "%g ", TO_DEGREE(eulerAngles[c]));

			}
		}
		fprintf(file, "\n");
	}

	fclose(file);
}

