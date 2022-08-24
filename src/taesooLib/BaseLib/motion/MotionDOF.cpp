// MotionDOF.cpp: implementation of the Motion class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MotionDOF.h"
#include "MotionLoader.h"
#include "../math/conversion.h"
#include "../math/OperatorStitch.h"
#include "../math/Operator.h"
#include "Liegroup.h"

//#include "../utility/TextFile.h"
//#include "../utility/operatorString.h"

double MotionDOFinfo::frameRate() const
{
  return mFrameRate;
}

void MotionDOFinfo::setFrameRate(double f)
{
  mFrameRate=f;
}

int MotionDOFinfo::numDOF() const
{
	return _sharedinfo->mTotalDOF;
}
MotionDOFinfo::MotionDOFinfo()
{
	_sharedinfo=NULL;
	mFrameRate=30;
}
MotionDOFinfo::MotionDOFinfo(const MotionDOFinfo& info)
	:_sharedinfo((SharedInfo*)info._sharedinfo)
{
	mFrameRate=30;
}

int MotionDOFinfo::parentIndex(int ibone) const 
{
	return skeleton().bone(ibone).parent()->treeIndex();
}
MotionDOFinfo::~MotionDOFinfo()
{
}

int MotionDOFinfo::numDOF(int ibone) const
{
	return endR(ibone)-startT(ibone);
}
int MotionDOFinfo::numActualDOF() const
{
  return numDOF()- numSphericalJoint();
}
//  int numBone() const;
//	return endDQ(ibone)-startDQ(ibone);

int MotionDOFinfo::DOFtype(int ibone, int offset) const
{
	int sT=startT(ibone);
	int sR=startR(ibone);
	int eR=endR(ibone);

	offset+=sT;
	if(offset<sR)
		return SLIDE;
	else if(eR-sR==4)
		return QUATERNION_W+offset-sR;
	return ROTATE;
}

int MotionDOFinfo::DOFindex(int ibone, int offset) const
{
	return startT(ibone)+offset;
}


void MotionDOFinfo::SharedInfo::init(MotionLoader const& l)
{
	mSkeleton=(MotionLoader*)&l;
	mTempPosture.Init(l.numRotJoint(), l.numTransJoint());

	bitvectorn useSpherical;
	useSpherical.resize(l.numBone());
	useSpherical.clearAll();
	useSpherical.setAt(1);
	_init(l,useSpherical);
}

int MotionDOFinfo::startT(int iBone) const	
{
	return _sharedinfo->mBoneInfo[iBone].startT;
}

int MotionDOFinfo::startDQ(int iBone) const	
{
	return _sharedinfo->mBoneInfo[iBone].startDQ;
}
int MotionDOFinfo::startR(int iBone) const	
{
	return _sharedinfo->mBoneInfo[iBone].startR;
}

int MotionDOFinfo::endR(int iBone) const	
{
	return _sharedinfo->mBoneInfo[iBone].endR;
}

int MotionDOFinfo::endDQ(int iBone) const	
{
	return _sharedinfo->mBoneInfo[iBone].endDQ;
}
bool MotionDOFinfo::hasTranslation(int iBone) const
{
	return _sharedinfo->mBoneInfo[iBone].startR!=_sharedinfo->mBoneInfo[iBone].startT;
}

bool MotionDOFinfo::hasQuaternion(int iBone) const
{
	return (_sharedinfo->mBoneInfo[iBone].endR-_sharedinfo->mBoneInfo[iBone].startR)==4;
}

bool MotionDOFinfo::hasAngles(int iBone) const
{
	return !hasQuaternion(iBone) && (_sharedinfo->mBoneInfo[iBone].endR-_sharedinfo->mBoneInfo[iBone].startR)!=0;
}

void MotionDOFinfo::getDOF(Posture const& p, vectorn& dof) const
{
	dof.resize(numDOF());

	MotionLoader* mSkeleton=_sharedinfo->mSkeleton;

	for(int i=1; i<mSkeleton->numBone(); i++)
	{
		Bone& bone=mSkeleton->bone(i);
		if(hasTranslation(i))
		{
			int start=startT(i);
			int ti=bone.transJointIndex();
			vector3 init_pos= bone.getOffsetTransform().translation;
			for(int c=0, nc=bone.getTranslationalChannels().length();
				c<nc; c++)
			{
				int xyz=bone.getTranslationalChannels()[c]-'X';
				dof[start+c]=p.m_aTranslations[ti][xyz]-init_pos[xyz]; // X or Y or Z
			}
		}

		int start=startR(i);
		int ri=bone.rotJointIndex();
		if(ri==-1) continue;

		if(hasQuaternion(i))
		{
			dof.range(start, start+4).assign(p.m_aRotations[ri]);
		}

		if(hasAngles(i))
		{
			if(bone.getRotationalChannels().findChar(0,'A')!= -1)
			{
				for(int i=0;i<bone.getRotationalChannels().length();i++)
				{
					vector3 axis;
					quater& qRot = p.m_aRotations[ri];

					int numChannels = strlen(bone.getRotationalChannels());
					for(int c=0;c<numChannels;c++)
					{
					//	printf("###%d %d###\n",start,c);
					/*	axis = bone.getAxisValue()[c];
						dof[start+c] = qRot.rotationAngle(axis);
						printf("dof %d : %lf\n",start+c,dof[start+c]);	*/
					}
				}
			}
			else
			{
				m_real aValue[3];

				p.m_aRotations[ri].getRotation(bone.getRotationalChannels(), aValue);
				/*
				quater q,qerr;
				q.setRotation(bone.getRotationalChannels(), aValue);
				qerr.difference(q, p.m_aRotations[ri]);
				qerr.align(quater(1,0,0,0));
				Msg::verify(qerr.rotationAngle()<0.001,"conversion error");
				*/
				
				for(int c=0,nc=bone.getRotationalChannels().length()
					; c<nc; c++)
					dof[start+c]=aValue[c];
			}
		}
	}
}

  

void MotionDOFinfo::setDOF(vectorn const& dof, Posture & p) const
{
	MotionLoader* mSkeleton=_sharedinfo->mSkeleton;
	for(int i=1; i<mSkeleton->numBone(); i++)
	{
		Bone& bone=mSkeleton->bone(i);
		if(hasTranslation(i))
		{
			int start=startT(i);
			int ti=bone.transJointIndex();

			bone.getLocalTrans(p.m_aTranslations[ti], &dof[start]);
		}

		int start=startR(i);
		int ri=bone.rotJointIndex();
		if(ri==-1) continue;

		if(hasQuaternion(i))
		{
			p.m_aRotations[ri]=dof.toQuater(start);
		}

		if(hasAngles(i))
		{
			bone.getLocalOri(p.m_aRotations[ri], &dof[start]);
		}
	}
}
void MotionDOFinfo::DOFtoDQ(vectorn const& dtheta, vectorn & dq)
{
	ASSERT(numActualDOF()==numDOF()-1); // other cases (many quaternion joints..) are not implemnted yet.
	ASSERT(dtheta.size()==numDOF());
	dq.setSize(numActualDOF());
	dq.range(0,3).assign(dtheta.range(4,7));// angular velocity goes first in DQ
	dq.range(3,6).assign(dtheta.range(0,3));
	dq.range(6,dq.size()).assign(dtheta.range(7,dtheta.size()));
}

void MotionDOFinfo::DQtoDOF(vectorn const& dq, vectorn & dtheta)
{
	ASSERT(numActualDOF()==numDOF()-1); // other cases (many quaternion joints..) are not implemnted yet.
	ASSERT(dq.size()==numActualDOF());
	dtheta.setSize(numDOF());
	dtheta.setVec3(4, dq.toVector3(0));// angular velocity goes first in DQ
	dtheta.setVec3(0, dq.toVector3(3));
	dtheta.range(7,dtheta.size()).assign(dq.range(6,dq.size()));
}
void MotionDOFinfo::blendBone(int ibone, vectorn & c, vectorn const& a, vectorn const& b, m_real t) const
{
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	int st=startT(ibone);
	int sr=startR(ibone);
	int nt=sr-st;
	int nr=endR(ibone)-sr;

	for(int i=0; i<nt; i++)
		c[i]=(1.0-t)*a[i]+t*b[i];

	if(nr==4){
		quater q;
		q.safeSlerp(a.toQuater(sr-st), b.toQuater(sr-st), t);
		c.range(nt, nt+4).assign(q);
	}
	else {
		for(int i=0; i<nr; i++)
			c[i+nt]=(1.0-t)*a[i+nt]+t*b[i+nt];
	}
}

void MotionDOFinfo::blend(vectorn & c, vectorn const& a, vectorn const& b, m_real t) const
{
	MotionLoader* mSkeleton=_sharedinfo->mSkeleton;
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	for(int i=0; i<c.size(); i++)
	{
		c[i]=(1.0-t)*a[i]+t*b[i];
	}
	for(int i=1; i<mSkeleton->numBone(); i++)
	{
		Bone& bone=mSkeleton->bone(i);
		if(hasQuaternion(i))
		{
			int start=startR(i);
			quater q;
			q.safeSlerp( a.toQuater(start), b.toQuater(start), t);
			c.range(start, start+4).assign(q);
		}
	}
}
void MotionDOFinfo::blendDelta(vectorn & c, vectorn const& a, vectorn const& b, m_real t) const
{
	MotionLoader* mSkeleton=_sharedinfo->mSkeleton;
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	for(int i=0; i<c.size(); i++)
	{
		c[i]=(1.0-t)*a[i]+t*b[i];
	}
	// root bone is excluded because it is delta encoded.
	for(int i=2; i<mSkeleton->numBone(); i++)
	{
		Bone& bone=mSkeleton->bone(i);
		if(hasQuaternion(i))
		{
			int start=startR(i);
			quater q;
			q.safeSlerp( a.toQuater(start), b.toQuater(start), t);
			c.range(start, start+4).assign(q);
		}
	}
}


Posture const& MotionDOFinfo::setDOF(vectorn const& dof) const
{
	setDOF(dof, _sharedinfo->mTempPosture);
	return _sharedinfo->mTempPosture;
}

void MotionDOFinfo::SharedInfo::init(MotionLoader const& l, bitvectorn const& useSpherical)
{
	mSkeleton=(MotionLoader*)&l;
	mTempPosture.Init(l.numRotJoint(), l.numTransJoint());

	_init(l,useSpherical);
}

int MotionDOFinfo::sphericalDOFindex(int isphericalJoint) const
{
	return _sharedinfo->mSphericalDofIndexes(isphericalJoint);
}
int MotionDOFinfo::numSphericalJoint() const
{
	return _sharedinfo->mSphericalDofIndexes.size();
}


void MotionDOFinfo::SharedInfo::_init(MotionLoader const& l, bitvectorn const& useSpherical)
{
	mBoneInfo.resize(l.numBone());
	mTotalDOF=0;
	mSphericalDofIndexes.resize(0);
	int mTotalActualDOF=0;
	for(int i=1; i<l.numBone(); i++)
	{
		Bone& bone=l.bone(i);
		
		mBoneInfo[i].startT=mTotalDOF;
		mBoneInfo[i].startDQ=mTotalActualDOF;
		mTotalDOF+=bone.getTranslationalChannels().length();
		mTotalActualDOF+= bone.getTranslationalChannels().length();
		mBoneInfo[i].startR=mTotalDOF;

		if(useSpherical[i])
		  {
			mSphericalDofIndexes.pushBack(mTotalDOF);
			mTotalDOF+=4;
			mTotalActualDOF+=3;
		  }
		else {
			int rc=bone.getRotationalChannels().length();
			mTotalDOF+=rc;
			mTotalActualDOF+=rc;
		}
		
		mBoneInfo[i].endR=mTotalDOF;
		mBoneInfo[i].endDQ=mTotalActualDOF;
	}

	DQtoBoneIndex.setSize(mTotalActualDOF);
	DOFtoBoneIndex.setSize(mTotalDOF);
	DOFtoDQ.setSize(mTotalDOF);
	DOFtoDQ.setAllValue(-1);
	DQtoDOF.setSize(mTotalActualDOF);


	for(int i=1; i<l.numBone(); i++)
	{
		DQtoBoneIndex.range(mBoneInfo[i].startDQ, mBoneInfo[i].endDQ).setAllValue(i);
		DOFtoBoneIndex.range(mBoneInfo[i].startT, mBoneInfo[i].endR).setAllValue(i);

		int startDQ=mBoneInfo[i].startDQ;
		int startDOF=mBoneInfo[i].startT;
		int num_param=mBoneInfo[i].endR-startDOF;
		//printf("bond : %d , %d %d %d\n", i, startDQ, startDOF, num_param);
		if(num_param==7)
		{
			// free joint
			for(int j=0; j<3; j++) // position
			{ // ignores w
				DOFtoDQ[startDOF+j]=startDQ+j+3; DQtoDOF[startDQ+j+3]=startDOF+j;
			}
			for(int j=0; j<3; j++) // orientation
			{ // ignores w
				DOFtoDQ[startDOF+j+4]=startDQ+j; DQtoDOF[startDQ+j]=startDOF+j+4;
			}
		}
		else if(mBoneInfo[i].endR-mBoneInfo[i].startR==4)
		{
			// ball joint
			for(int j=0; j<3; j++)
			{ 
				// ignores w
				DOFtoDQ[startDOF+j+1]=startDQ+j; DQtoDOF[startDQ+j]=startDOF+j+1;
			}
		}
		else
		{
			for(int j=0; j<num_param; j++)
			{
				DOFtoDQ[startDOF+j]=startDQ+j; DQtoDOF[startDQ+j]=startDOF+j;
			}
		}
	}
}

int MotionDOFinfo::numBone() const
{
	return _sharedinfo->mBoneInfo.size();
}

MotionDOF::MotionDOF(const MotionDOF& other)
:matrixn()	
{ assign(other); mInfo=other.mInfo;}

void MotionDOF::operator=(const MotionDOF& other)
{ assign(other); mInfo=other.mInfo; }


static void projectAngle(m_real& delta)	// represent angle in [-pi, pi]
{
	while(delta>M_PI+FERR)
		delta-=2.0*M_PI;
	while(delta<-1.0*M_PI-FERR)
		delta+=2.0*M_PI;

}

static void alignAngle(m_real prev, m_real& next)
{

	// next-prev는 [-PI,PI]에 있다고 보장을 못한다. 따라서 이범위에 들어오는 next-prev를 구한다.

	m_real delta=next-prev;
	
	projectAngle(delta);

	// 다시 원래의 prev범위로 되돌린다.
	next=prev+delta;
}

static void alignAngles(vectorn const& value, m_real x=0)
{
	alignAngle(x, value(0));
	for(int i=1; i<value.size(); i++)
		alignAngle(value(i-1), value(i));
}

void MotionDOF::set(const Motion& srcMotion)
{
	Msg::verify(mInfo._sharedinfo, "Error! mInfo._sharedinfo ==NULL ");
	Msg::verify(mInfo.numBone()==srcMotion.skeleton().numBone()
			|| (mInfo.skeleton().numTransJoint()==srcMotion.skeleton().numTransJoint() &&
				mInfo.skeleton().numRotJoint()==srcMotion.skeleton().numRotJoint())
			, "Error! numBone doesn't match");

	mInfo.mFrameRate=(double)srcMotion.frameRate();
	resize(srcMotion.numFrames());

	for(int i=0; i<srcMotion.numFrames(); i++)
		mInfo.getDOF(srcMotion.pose(i), row(i).lval());

	for(int i=0; i<mInfo.numBone() ;i++)
	{
		Bone& bone=srcMotion.skeleton().bone(i);

		int startT=mInfo.startT(i);
		int startR=mInfo.startR(i);
		int endR=mInfo.endR(i);
		if(mInfo.hasQuaternion(i))
		{
			quaterNView qv=quatViewCol(*this, startR);
			qv.align();
			for(int j=0; j<qv.size(); j++)
				qv[j].normalize();
		}
		else if(endR>startR)
		{
			for(int j=startR; j<endR; j++)
				alignAngles(column(j));
		}
	}
}

static void rotateBoneGlobal(Bone* bone, quater const& q_delta)
{
	//-- current global
	//-- q0*q1*lq= qo
	//--> parent_qo==q0*q1==qo*lq:inverse()
	quater qo=bone->getFrame().rotation;
	quater parent_qo=qo*bone->getLocalFrame().rotation.inverse();

	//--qo_new= q_delta*qo = parent_qo*lq_new
	//--lq_new= parent_qo:inverse()*q_delta*qo
	quater x=parent_qo.inverse()*q_delta*qo;
	x.normalize();
	bone->_getLocalFrame().rotation=x;
}
static void IKsolveSwivelAngle(Bone& sh_bone, Bone& elb_bone, Bone& wrist_bone, vector3 const& goal_elb)
{
	vector3 const& p0=sh_bone.getFrame().translation;
	vector3 const& p1=elb_bone.getFrame().translation;
	vector3 const& p2=wrist_bone.getFrame().translation;
	
	vector3 axis=p2-p0;
	axis.normalize();
	vector3 center=(p0+p2)/2;
	vector3 front=p1-center;
	vector3 target=goal_elb-center;
	quater q;
	q.setAxisRotation(axis, front, target);
	double angle=q.rotationAngleAboutAxis(axis);
	if (angle<-TO_RADIAN(30) )
		angle=-TO_RADIAN(30);
	else if (angle>TO_RADIAN(30) )
		angle=TO_RADIAN(30);
	double femurLen=(p1-p0).length();
	double importance=sop::clampMap(target.length(), 0.2*femurLen,0.4*femurLen, 0, 1);
	angle=angle*importance;
	q.setRotation(axis, angle);
	//q.setRotation(axis, TO_RADIAN(90));
	rotateBoneGlobal(&sh_bone,q);
}

void MotionDOF::set(const Motion& srcMotion, intvectorn const& treeIndicesShoulder, intvectorn const& treeIndicesElbow, intvectorn const& treeIndicesWrist)
{
	Msg::verify(mInfo._sharedinfo, "Error! mInfo._sharedinfo ==NULL ");
	Msg::verify(mInfo.numBone()==srcMotion.skeleton().numBone(), "Error! numBone doesn't match");

	mInfo.mFrameRate=(double)srcMotion.frameRate();
	resize(srcMotion.numFrames());

	vector3N shoulders(treeIndicesShoulder.size());
	vector3N wrist_before(treeIndicesShoulder.size());
	vector3N wrist_after(treeIndicesShoulder.size());
	vector3N elbow_before(treeIndicesShoulder.size());
	MotionLoader* skel=mInfo._sharedinfo->mSkeleton;
	for(int i=0; i<srcMotion.numFrames(); i++)
	{
		mInfo.getDOF(srcMotion.pose(i), row(i).lval());
		skel->setPose(srcMotion.pose(i));

		for(int j=0; j<treeIndicesShoulder.size(); j++)
		{
			int ti=treeIndicesShoulder(j);
			int tiw=treeIndicesWrist(j);
			int tie=treeIndicesElbow(j);
			shoulders(j)=skel->bone(ti).getFrame().translation;
			elbow_before(j)=skel->bone(tie).getFrame().translation;
			wrist_before(j)=skel->bone(tiw).getFrame().translation;
		}
		skel->setPoseDOF(row(i));
		for(int j=0; j<treeIndicesShoulder.size(); j++)
		{
			int ti=treeIndicesShoulder(j);
			int tiw=treeIndicesWrist(j);
			int tie=treeIndicesElbow(j);
			wrist_after(j)=skel->bone(tiw).getFrame().translation;
			//printf("error %d:%d:%f\n", i,j,( wrist_after(j)-wrist_before(j)).length());
			quater qdelta;	
			qdelta.axisToAxis(wrist_after(j)-shoulders(j), wrist_before(j)-shoulders(j));
			rotateBoneGlobal(&skel->bone(ti), qdelta);
			skel->fkSolver().setChain(skel->bone(tiw));
			IKsolveSwivelAngle(skel->bone(ti) , skel->bone(tie), skel->bone(tiw), elbow_before(j));
		}
		skel->fkSolver().forwardKinematics();
		skel->getPoseDOF(row(i).lval());
#ifdef DEBUGTRACE
		for(int j=0; j<treeIndicesShoulder.size(); j++)
		{
			int ti=treeIndicesShoulder(j);
			int tiw=treeIndicesWrist(j);
			wrist_after(j)=skel->bone(tiw).getFrame().translation;
			printf("error_after %d:%d:%f\n", i,j,( wrist_after(j)-wrist_before(j)).length());
		}
#endif
	}


	for(int i=0; i<mInfo.numBone() ;i++)
	{
		Bone& bone=srcMotion.skeleton().bone(i);

		int startT=mInfo.startT(i);
		int startR=mInfo.startR(i);
		int endR=mInfo.endR(i);
		if(mInfo.hasQuaternion(i))
		{
			quaterNView qv=quatViewCol(*this, startR);
			qv.align();
			for(int j=0; j<qv.size(); j++)
				qv[j].normalize();
		}
		else if(endR>startR)
		{
			for(int j=startR; j<endR; j++)
				alignAngles(column(j));
		}
	}
}
void MotionDOF::get(Motion& tgtMotion)
{
	tgtMotion.InitEmpty(
		mInfo._sharedinfo->mSkeleton,
		numFrames(),
		1.0/mInfo.mFrameRate);

	for(int i=0; i<tgtMotion.numFrames(); i++)
		mInfo.setDOF(row(i), tgtMotion.pose(i));
}

void MotionDOF::samplePose(m_real criticalTime, vectorn& out) const
{
	out.setSize(mInfo.numDOF());
	//!< 0 <=criticalTime<= numFrames()-1
	// float 0 이 정확하게 integer 0에 mapping된다.
	int a;
	m_real t;

	a=(int)floor(criticalTime);
	t=criticalTime-(m_real)a;

	if(t<0.005)
		out=row(a);
	else if(t>0.995)
		out=row(a+1);
	else
	{
		if(a<0)
			mInfo.blend(out, row(a+1), row(a+2), t-1.0);
			//pose=pose(0);
			
		else if(a+1>=numFrames())
			//p=pose(numFrames()-1);
			mInfo.blend(out, row(a-1), row(a), t+1.0);
		else
			mInfo.blend(out, row(a), row(a+1), t);
	}
}

void MotionDOF::sampleBone(int ibone, m_real criticalTime, vectorn& out) const
{
	out.setSize(mInfo.numDOF());
	//!< 0 <=criticalTime<= numFrames()-1
	// float 0 이 정확하게 integer 0에 mapping된다.
	int a;
	m_real t;

	a=(int)floor(criticalTime);
	t=criticalTime-(m_real)a;

	int st=mInfo.startR(ibone);
	int er=mInfo.endR(ibone);
	if(t<0.005)
		out=row(a).range(st,er);
	else if(t>0.995)
		out=row(a+1).range(st,er);
	else
	{
		if(a<0)
			mInfo.blendBone(ibone,out, row(a+1).range(st,er), row(a+2).range(st,er), t-1.0);
			//pose=pose(0);
			
		else if(a+1>=numFrames())
			//p=pose(numFrames()-1);
			mInfo.blendBone(ibone,out, row(a-1).range(st,er), row(a).range(st,er), t+1.0);
		else
			mInfo.blendBone(ibone,out, row(a).range(st,er), row(a+1).range(st,er), t);
	}
}

void quaterNN_linstitch(m::stitchOp const& op, matrixn& c, matrixn & a, matrixn & b);

// out.length() will become motA.length()+motB.length()
void MotionDOF::stitch( MotionDOF const& motA, MotionDOF const& motB)
{
	MotionDOF& out=*this;

	out.changeLength(motA.length()+motB.length());

	
	m::c1stitchPreprocess cs(motA.numFrames(), motB.numFrames(), 2, false);
	m::c0stitch cs0;
//	m::c0concat cs;

	InterframeDifference idA(motA);
	InterframeDifference idB(motB);

	InterframeDifference idOut;
	
	idOut.resize(motA.numFrames()+motB.numFrames()-1);
	idOut.startP=idA.startP;
	idOut.startRotY=idA.startRotY;

	// use c0 stitch for derivatives.
	cs0.calc(matView(idOut.dv).lval(), matView(idA.dv), matView(idB.dv));
	cs0.calc(matView(idOut.dq).lval(), matView(idA.dq), matView(idB.dq));

	cs.calc(idOut.offset_y.column().lval(), idA.offset_y.column(), idB.offset_y.column());
	quaterNN_linstitch(cs,matView(idOut.offset_q).lval(),matView(idA.offset_q).lval(), matView(idB.offset_q).lval());

	idOut.reconstruct(out, out.mInfo.frameRate());

	matrixn temp=matViewCol(motB, 7);

	MotionLoader const& skel=mInfo.skeleton();
	for(int i=1; i<skel.numBone(); i++)
	{
		Bone& bone=skel.bone(i);
		if(mInfo.hasAngles(i))
		{
			int startR=mInfo.startR(i);
			int endR=mInfo.endR(i);

			for(int j=startR; j<endR; j++)
				alignAngles(temp.column(j-7), motA(motA.numFrames()-1, j));
		}
	}

	cs.calc(matViewCol(out,7).lval(), matViewCol(motA, 7), temp);

	mInfo=motA.mInfo;
}
void MotionDOF::align( MotionDOF const& motA, MotionDOF const& motB)
{
	MotionDOF& out=*this;

	out.changeLength(motA.length()+motB.length());
	
	m::c0concat cs0;

	InterframeDifference idA(motA);
	InterframeDifference idB(motB);

	InterframeDifference idOut;
	
	idOut.resize(motA.numFrames()+motB.numFrames()-1);
	idOut.startP=idA.startP;
	idOut.startRotY=idA.startRotY;

	cs0.calc(matView(idOut.dv).lval(), matView(idA.dv), matView(idB.dv));
	cs0.calc(matView(idOut.dq).lval(), matView(idA.dq), matView(idB.dq));
	cs0.calc(idOut.offset_y.column().lval(), idA.offset_y.column(), idB.offset_y.column());
	cs0.calc(matView(idOut.offset_q).lval(),matView(idA.offset_q).lval(), matView(idB.offset_q).lval());

	idOut.reconstruct(out, out.mInfo.frameRate());
	if(numDOF()>7)
	{
		matrixn temp=matViewCol(motB, 7);

		MotionLoader const& skel=mInfo.skeleton();
		for(int i=1; i<skel.numBone(); i++)
		{
			Bone& bone=skel.bone(i);
			if(mInfo.hasAngles(i))
			{
				int startR=mInfo.startR(i);
				int endR=mInfo.endR(i);

				for(int j=startR; j<endR; j++)
					alignAngles(temp.column(j-7), motA(motA.numFrames()-1, j));
			}
		}

		cs0.calc(matViewCol(out,7).lval(), matViewCol(motA, 7), temp);
	}

	mInfo=motA.mInfo;
}
void MotionDOF::alignSimple( MotionDOF const& motA, MotionDOF const& motB)
{
	MotionDOF& out=*this;
	out.changeLength(motA.length()+motB.length());
	out.range(0, motA.rows())=motA;
	out.range(motA.rows(), out.rows())=motB.range(1, motB.rows());
	transf A=motA.rootTransformation(motA.rows()-1);
	A.rotation=A.rotation.rotationY();
	transf B=motB.rootTransformation(0);
	B.rotation=B.rotation.rotationY();
	transf delta;
	delta.difference(B, A);
	delta.translation.y=0;
	//std::cout <<delta<<std::endl;
	//std::cout<<out.row(motA.rows())[2]<<std::endl;
	out.range(motA.rows(), out.rows()).transform(delta);
	//std::cout<<out.row(motA.rows())[2]<<std::endl;
}


void MotionDOF::stitchDeltaRep( MotionDOF const& motA, MotionDOF const& motB)
{
	MotionDOF& out=*this;

	out.changeLength(motA.length()+motB.length());

	
	
//	m::c0concat cs;

	// use c0 stitch for derivatives.

	if(motA.numFrames()==2)
	{
		m::c0stitchOnline cs0;
		cs0.calc(matViewCol(out, 0,3).lval(), matViewCol(motA, 0, 3), matViewCol(motB, 0,3));
		m::c1stitchPreprocessOnline cs(motA.numFrames(), motB.numFrames(), 2);
		cs.calc(matViewCol(out,3).lval(), matViewCol(motA, 3), matViewCol(motB, 3));
	}
	else
	{
		m::c0stitch cs0;
		cs0.calc(matViewCol(out, 0,3).lval(), matViewCol(motA, 0, 3), matViewCol(motB, 0,3));
		m::c1stitchPreprocess cs(motA.numFrames(), motB.numFrames(), 2, false);
		cs.calc(matViewCol(out,3).lval(), matViewCol(motA, 3), matViewCol(motB, 3));
	}

	mInfo=motA.mInfo;
}



void MotionDOF::reconstructData(vector3 const& start_transf)
{
	reconstructData(start_transf, *this);
}

void MotionDOF::reconstructData(vector3 const& start_transf, matrixn& out) const
{
	InterframeDifference id;
	
	generateID(start_transf, id);
	if(out.rows()<numFrames())
		out.resize(numFrames(), 7);

	id.reconstruct(out, mInfo.frameRate());
}


void MotionDOF::reconstructOneFrame(vector3 const& prevRootTransf2D, vectorn const& deltaPose, vectorn & outpose) const
{
	/* basically the same as the following code.
	InterframeDifference id;
	
	generateID(prevRootTransf2D, id);
	outpose=deltaPose;

	id.reconstruct(outpose.row().lval(), mInfo.frameRate());
	*/
	vector3 startP=prevRootTransf2D;
	startP.y=0;
	quater startR;
	startR.setRotation(vector3(0,1,0), prevRootTransf2D.y);
	double dt=1.0/mInfo.frameRate();
	vector3 dv;
	dv.x=deltaPose(0);
	dv.z=deltaPose(1);
	dv*=dt;
	dv.rotate(startR);
	quater dq;
	dq.setRotation(vector3(0,1,0), deltaPose(2)*dt);
	quater offset_q;
	offset_q.setRotation(deltaPose.toVector3(4));

	outpose=deltaPose;
	setRootTransformation(outpose, transf(dq*startR*offset_q, startP+dv));
	outpose(1)=deltaPose(3); // offset_y
}
vector3 MotionDOF::convertToDeltaRep()
{
	InterframeDifference id(*this);
	return id.exportToDeltaRep(*this);
}

vector3 InterframeDifference::exportToDeltaRep(matrixn & output)
{
	InterframeDifference& id=*this;

	vector3 start_transf;
	start_transf.x=id.startP.x;
	start_transf.y=id.startRotY.rotationAngle(vector3(0,1,0));
	start_transf.z=id.startP.z;

	vector3 r;
	for(int i=0; i<numFrames(); i++)
	{
		m_real* row_i=&output.value(i,0);
		row_i[0]=id.dv[i].x;
		row_i[1]=id.dv[i].z;
		row_i[2]=id.dq[i].y;
		row_i[3]=id.offset_y[i];

		r.rotationVector(id.offset_q[i]);
		row_i[4]=r.x;
		row_i[5]=r.y;
		row_i[6]=r.z;
	}

	return start_transf;
}

void InterframeDifference::initFromDeltaRep(vector3 const& start_transf, matrixn const& input)
{
	InterframeDifference& id=*this;

	int numFrames=input.rows();
	id.resize(numFrames);

	id.startP.x=start_transf.x;
	id.startRotY.setRotation(vector3(0,1,0), start_transf.y);
	id.startP.z=start_transf.z;

	vector3 r;
	for(int i=0; i<numFrames; i++)
	{
		m_real* row_i=&input(i,0);
		id.dv[i].x=row_i[0];
		id.dv[i].z=row_i[1];
		id.dq[i]=vector3(0, row_i[2],0);
		id.offset_y[i]=row_i[3];

		r.x=row_i[4];
		r.y=row_i[5];
		r.z=row_i[6];

		id.offset_q[i].setRotation(r);
	}

}

void MotionDOF::generateID(vector3 const& start_transf, InterframeDifference& id) const
{
	id.initFromDeltaRep(start_transf, *this);
}



void MotionDOF::reconstructData(transf const& startTransf, matrixn& out) const
{
	reconstructData(startTransf.encode2D(), out);
}



MotionDOFview MotionDOF::range(int start, int end)
{
	return MotionDOFview (&value(start,0), end-start, cols(), stride, mInfo);
}
const MotionDOFview MotionDOF::range(int start, int end) const
{
	return MotionDOFview (&value(start,0), end-start, cols(), stride, mInfo);
}

MotionDOFview MotionDOF::range_c(int first, int last)
{
	return MotionDOFview (&value(first,0), last-first+1, cols(), stride, mInfo);
}

matrixnView MotionDOF::_matView()
{
	return ((matrixn*)this)->range(0, rows(), 0, cols());
	//return matrixnView(&value(0,0), rows(), cols(), stride);
}
/*
vectornView MotionDOF::row(int iframe)
{
	return ((matrixn*)this)->row(iframe);
}
*/

inline void decomposeRot(MotionDOF const& input, int iframe, quaterN& rotY, quaterN& offset_q)
{
	input.row(iframe).toQuater(3).decompose(rotY[iframe], offset_q[iframe]);
	offset_q[iframe].align(quater(1,0,0,0));
}

InterframeDifference::InterframeDifference(MotionDOF const& input)
{
	startP=input.row(0).toVector3(0);

	quaterN rotY;
	rotY.resize(input.numFrames());
	resize(input.numFrames());
	decomposeRot(input, 0, rotY, offset_q);
	

	m_real inv_dt=input.mInfo.frameRate();
	//printf("inv_dt=%f %x %g\n", inv_dt, input.mInfo, input.mInfo.frameRate());
	startRotY=rotY[0];

	quater inv_q, dqq, mid_rotY;

	for(int i=1; i<numFrames(); i++)
	{
		vector3 p=input.row(i).toVector3(0);
		vector3 p1=input.row(i-1).toVector3(0);
		dv[i]=p-p1;
		dv[i].y=0;
		dv[i]*=inv_dt;	// convert to speed
		
		

		decomposeRot(input, i, rotY, offset_q);

		mid_rotY.interpolate(0.5, rotY[i-1], rotY[i]);
		dv[i].rotate(mid_rotY.inverse());
		offset_y[i]=p.y;

		///////////////////////////////////////////////////////////////////////////////
		//  calculation rotAxis_y	pose(i).m_aRotations[0] = m_rotAxis_y*m_offset_q 
		//							pose(i).m_dq * pose(i-1).m_rotAxis_y = pose(i).m_rotAxis_y
		//			  		  thus, pose(i).m_dq = pose(i).m_rotAxis_y * (pose(i-1).m_rotAxis_y)^(-1)
		///////////////////////////////////////////////////////////////////////////////
		inv_q.inverse(rotY[i-1]);
		dqq.mult(rotY[i], inv_q);
		dqq.align(quater(1,0,0,0));	// m_dq should be small.
		dq[i].rotationVector(dqq);
		dq[i]*=inv_dt;	// convert to angVel
	}

	dq[0]=dq[1];
	dv[0]=dv[1];
	offset_y[0]=startP.y;
	startP.y=0;
}

void InterframeDifference::resize(int numFrames)
{
	dv.resize(numFrames);	// same length as input.
	dq.resize(numFrames);	// same length as input.
	offset_y.resize(numFrames);
	offset_q.resize(numFrames);
}

inline void setQ(matrixn& output, int i, quater const& q)
{
	m_real* output_i=output[i];
	output_i[3]=q.w;
	output_i[4]=q.x;
	output_i[5]=q.y;
	output_i[6]=q.z;
}

inline void setP(matrixn& output, int i, vector3 const& p, m_real py)
{
	m_real* output_i=output[i];
	output_i[0]=p.x;
	output_i[1]=py;
	output_i[2]=p.z;
}

void InterframeDifference::reconstruct(matrixn& output, m_real frameRate)
{
	if(numFrames()!=output.rows())
		throw std::runtime_error("InterframeDifference::reconstruct");
	// reconstruct rot
	setP(output, 0, startP, offset_y[0]);

	quater q,dqq;
	vector3 p,gdv;
	q.mult(startRotY,offset_q[0]);
	setQ(output, 0, q);

	vector3 prevP=startP;
	quater prevRotY=startRotY, rotY;
	m_real dt=1.0/frameRate;
	quater midRotY;
	//printf("dt=%g\n", dt);
	for(int i=1; i<numFrames(); i++)
	{
		// reconstruct rot
		dqq.setRotation(dq[i]*dt);
		rotY.mult(dqq, prevRotY);
		q.mult(rotY, offset_q[i]);
		setQ(output, i, q);

		midRotY.interpolate(0.5, prevRotY, rotY);
		// reconstruct trans
		//gdv.rotate(prevRotY, dv[i]*dt);
		gdv.rotate(midRotY, dv[i]*dt);
		setP(output, i, gdv+prevP, offset_y[i]);
		
		prevP=gdv+prevP;
		prevRotY=rotY;
	}
}



transf MotionDOF::rootTransformation(int i) const
{
	return MotionDOF::rootTransformation(row(i));
}

transf MotionDOF::rootTransformation(vectorn const& pose)
{
	return transf(pose.toQuater(3), pose.toVector3(0));
}

void MotionDOF::setRootTransformation(vectorn & pose, transf const& t)
{
	pose.setQuater(3, t.rotation);
	pose.setVec3(0, t.translation);
}



///////////////////////////////////////////
static void calcTangentRot(vector3 const& pos1, vector3 const& pos2, quater const& prot, m_real frameRate, quater& out)
{
	const m_real speedMin=1; //1m/s
	quater trot;
	vector3 diff;
	diff.difference(pos1, pos2);
	diff.y=0;

	Msg::verify(frameRate>10, "frameRateErr");
	m_real speed=diff.length()*frameRate;
	diff.normalize();

	trot.setAxisRotation(vector3(0,1,0), vector3(0,0,1), diff);

	out.interpolate(
		sop::smoothTransition(sop::clampMap(speed, speedMin/2, speedMin)), prot, trot);
}

static void decomposeRotC1(MotionDOF const& input, int iframe, quaterN& robustTangentRotY, quaterN& rotY, quaterN& offset_q)
{
	if(iframe==0)
	{
		decomposeRot(input, iframe, rotY, offset_q);
		robustTangentRotY[0]=rotY[0];
		return;
	}


	quater prot;

	vector3 pos1=input.row(iframe-1).toVector3(0);
	vector3 pos2=input.row(iframe).toVector3(0);
	pos1.y=0;
	pos2.y=0;

	quater root=input.row(iframe).toQuater(3);
	root.decompose(prot, offset_q[iframe]);
	offset_q[iframe].align(quater(1,0,0,0));

	calcTangentRot(pos1, pos2, prot, input.mInfo.frameRate(), robustTangentRotY[iframe]);
	
	rotY[iframe]=prot;
}

InterframeDifferenceC1::InterframeDifferenceC1(MotionDOF const& input)
{
	_frameRate=input.mInfo.frameRate();
	startPrevP=input.row(0).toVector3(0);
	startP=input.row(0).toVector3(0);

	quaterN tanRotY;
	quaterN rotY;
	tanRotY.resize(input.numFrames());
	rotY.resize(input.numFrames());
	resize(input.numFrames());
	decomposeRotC1(input, 0, tanRotY, rotY, offset_q);
	offset_qy[0].difference(tanRotY[0], rotY[0]);

	startRotY=rotY[0];

	quater inv_q, dqq;
	for(int i=1; i<numFrames(); i++)
	{
		vector3 p=input.row(i).toVector3(0);
		vector3 p1=input.row(i-1).toVector3(0);
		dv[i]=p-p1;
		dv[i].y=0;
		
		inv_q.inverse(tanRotY[i-1]);
		dv[i].rotate(inv_q);
		offset_y[i]=p.y;

		///////////////////////////////////////////////////////////////////////////////
		//  calculation rotAxis_y	pose(i).m_aRotations[0] = m_rotAxis_y*m_offset_q 
		//							pose(i).m_dq * pose(i-1).m_rotAxis_y = pose(i).m_rotAxis_y
		//			  		  thus, pose(i).m_dq = pose(i).m_rotAxis_y * (pose(i-1).m_rotAxis_y)^(-1)
		///////////////////////////////////////////////////////////////////////////////
		decomposeRotC1(input, i, tanRotY, rotY, offset_q);
		offset_qy[i].difference(tanRotY[i], rotY[i]);

		dqq.mult(tanRotY[i], inv_q);
		dqq.align(quater(1,0,0,0));	// m_dq should be small.
		dq[i].rotationVector(dqq);
	}

	dq[0]=dq[1];
	dv[0]=dv[1];
	offset_y[0]=startP.y;
	startP.y=0;
}

void InterframeDifferenceC1::resize(int numFrames)
{
	dv.resize(numFrames);	// same length as input.
	dq.resize(numFrames);	// same length as input.
	offset_y.resize(numFrames);
	offset_q.resize(numFrames);
	offset_qy.resize(numFrames);
}

void InterframeDifferenceC1::reconstruct(matrixn& output)
{
	if(numFrames()!=output.rows())
		throw std::runtime_error("InterframeDifference::reconstruct");

	// reconstruct rot
	setP(output, 0, startP, offset_y[0]);

	quater q, dqq;
	vector3 p, gdv;
	q.mult(startRotY,offset_q[0]);
	setQ(output, 0, q);

	vector3 prevPrevP=startPrevP;
	vector3 prevP=startP;
	quater prevRotY=startRotY, prevTangentRotY, tangentRotY, rotY;
	for(int i=1; i<numFrames(); i++)
	{
		// reconstruct rot
		dqq.setRotation(dq[i]);

		//printf("d%f ", prevPrevP.distance(prevP)*_frameRate);
		calcTangentRot(prevPrevP, prevP, prevRotY, _frameRate, prevTangentRotY);

		tangentRotY.mult(dqq, prevTangentRotY);
		rotY=offset_qy[i]*tangentRotY;
		q.mult(rotY, offset_q[i]);
		
		setQ(output, i, q);

		// reconstruct trans
		gdv.rotate(prevTangentRotY, dv[i]);
		setP(output, i, gdv+prevP, offset_y[i]);
		
		prevPrevP=prevP;
		prevP=gdv+prevP;
		
		prevRotY=rotY;
		prevRotY.normalize();
	}
}

void InterframeDifferenceC1::initFromDeltaRep(vectorn const& start_transf, matrixn const& input)
{
	InterframeDifferenceC1& id=*this;

	int numFrames=input.rows();
	id.resize(numFrames);

	id.startP.x=start_transf[0];
	id.startRotY.setRotation(vector3(0,1,0), start_transf[1]);
	id.startP.z=start_transf[2];
	id.startPrevP.x=start_transf[3];
	id.startPrevP.z=start_transf[4];

	vector3 r;
	for(int i=0; i<numFrames; i++)
	{
		m_real* row_i=&input(i,0);
		id.dv[i].x=row_i[0];
		id.dv[i].z=row_i[1];
		id.dq[i]=vector3(0, row_i[2],0);
		id.offset_y[i]=row_i[3];

		r.x=row_i[4];
		r.y=row_i[5];
		
		id.offset_q[i].setRotation("ZX", r);

		id.offset_qy[i].setRotation(vector3(0,1,0), row_i[6]);
	}
}

vectorn InterframeDifferenceC1::getTransformation(matrixn const& motionDOF, int iframe)
{
	vectorn start_transf(5);
	if(motionDOF.rows()==0)
	{
		start_transf.setAllValue(0);
		return start_transf;
	}
	vector3 root=MotionDOF::rootTransformation(motionDOF.row(iframe)).encode2D();
	start_transf[0]=root.x;
	start_transf[1]=root.y;
	start_transf[2]=root.z;

	if(iframe==0)
	{
		
		start_transf[3]=root.x;
		start_transf[4]=root.z;
	}
	else
	{
		start_transf[3]=motionDOF[iframe-1][0];
		start_transf[4]=motionDOF[iframe-1][2];
	}

	return start_transf;
}

vectorn InterframeDifferenceC1::exportToDeltaRep(matrixn & output)
{
	InterframeDifferenceC1& id=*this;

	vectorn start_transf(5);
	start_transf[0]=id.startP.x;
	start_transf[1]=id.startRotY.rotationAngle(vector3(0,1,0));
	start_transf[2]=id.startP.z;
	start_transf[3]=id.startPrevP.x;
	start_transf[4]=id.startPrevP.z;

	vector3 r;
	for(int i=0; i<numFrames(); i++)
	{
		m_real* row_i=&output.value(i,0);
		row_i[0]=id.dv[i].x;
		row_i[1]=id.dv[i].z;
		row_i[2]=id.dq[i].y;
		row_i[3]=id.offset_y[i];

		id.offset_q[i].getRotation("ZX",r);
		row_i[4]=r.x;
		row_i[5]=r.y;
		row_i[6]=id.offset_qy[i].rotationAngle(vector3(0,1,0));
	}

	return start_transf;
}
void MotionDOF::transform(transf const& t)
{
	for(int i=0; i<numFrames(); i++)
		setRootTransformation(row(i).lval(), t*rootTransformation(i));
}
void MotionDOF::scale(double t)
{
	for(int i=0; i<numFrames(); i++)
	{
		row(i).setVec3(0, row(i).toVector3(0)*t);
	}
}


void MotionDOF::setDOFinfo(MotionDOFinfo const& info)
{
	if(mInfo._sharedinfo && numFrames()>0)
	{
		//Msg::verify(mInfo.numBone()==info.numBone(), "Error! numBone doesn't match");
		Msg::verify(mInfo.numDOF()==info.numDOF(), "Error! numDOF doesn't match");
	}
	mInfo=info;

}
void MotionDOF::calcForwardDerivative(int i, vectorn & dpose, double frameRate) const
{
	transf T1=rootTransformation(i);
	transf T2=rootTransformation(i+1);

	dpose.setSize(row(i).size());
	dpose.sub(row(i+1), row(i)); //  central difference
	dpose*=frameRate;
	Msg::verify(mInfo.numSphericalJoint()==1, "calcForwardDerivative error!"); 
	// otherwise following code is incorrect
	transf T=rootTransformation(i);
	Liegroup::se3 V=Liegroup::twist(T, rootTransformation(i+1), 1.0/frameRate);
	dpose.setVec3(0, V.V());
	dpose.setVec3(4, V.W());
}
