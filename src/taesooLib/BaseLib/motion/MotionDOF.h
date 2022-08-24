#ifndef MOTIONDOF_H_
#define MOTIONDOF_H_
#pragma once

#include "postureip.h"

class MotionLoader;
class InterframeDifference;

#include "MotionDOFinfo.h"
class MotionDOFview;
class Motion;

class MotionDOF: public matrixn
{	
protected:
	MotionDOF(m_real* _ptr, int _n, int _m, int _stride) :matrixn(_ptr, _n, _m, _stride){}
	MotionDOF():matrixn(){}
public:
	MotionDOFinfo mInfo;	
	
	MotionDOF(const MotionDOFinfo& info):matrixn(), mInfo(info) {}
	
	// DEFAULT COPY CONSTRUCTOR.	
	MotionDOF(const MotionDOF& other);
	~MotionDOF(){}

	void transform(transf const& t);
	void scale(double f);
	void operator=(const MotionDOF& other);
	void setDOFinfo(MotionDOFinfo const& info);
	
	static transf rootTransformation(vectorn const& pose);
	static void setRootTransformation(vectorn & pose, transf const& t);	
	transf rootTransformation(int i) const;

	void calcForwardDerivative(int i, vectorn & dpose, double frameRate=120) const;
	
	MotionDOFview range(int start, int end);	// [start, end)
	const MotionDOFview range(int start, int end) const;	// [start, end)
	MotionDOFview range_c(int first, int last);	// [first, last] (closed interval)
	matrixnView _matView();
	int numFrames()	const			{ return rows(); }
	void resize(int numFrames)		{ matrixn::resize(numFrames, numDOF());}

	int numDOF() const				{ return mInfo.numDOF();}

	// 동작의 길이는 "키프레임 개수-1" 로 정의한다. 즉 인터프레임간 길이를 1로 계산한것.
	int length() const				{ return numFrames()-1;}
	void changeLength(int length)	{ matrixn::resize(length+1, numDOF()); }
		
	// 똑같은 모션을 만든다.(일부분을 따와서 만들 수도 있다.)
	void set(const Motion& srcMotion); 
	// knee, elbow DOF가 줄어드는 경우 이함수를 써야 포즈가 유지된다.
	void set(const Motion& srcMotion, intvectorn const& treeIndicesShoulder, intvectorn const& treeIndicesElbow, intvectorn const& treeIndicesWrist); 
	void get(Motion& tgtMotion);
	void samplePose(m_real criticalTime, vectorn& out) const;
	void sampleBone(int ibone, m_real criticalTime, vectorn& out) const;

	// for the following three functions: 
	// out.size() will become motA.size()+motB.size()-1
	// in other words, out.length() will become motA.length()+motB.length()
	void stitch(MotionDOF const& motA, MotionDOF const& motB);
	// modifies 1 frame in the middle. (Basically, a very abrupt transition.)
	void align(MotionDOF const& motA, MotionDOF const& motB);
	// does not modify any frame. (no stitch.) Just concat so that the last pose of motA is colocated with the first pose of motB
	void alignSimple(MotionDOF const& motA, MotionDOF const& motB);

	// Encode InterframeDifference.
	// The Delta representation is not compatible with MotionLoader.
	void stitchDeltaRep(MotionDOF const& motA, MotionDOF const& motB);
	
	vector3 convertToDeltaRep();	// see interframeDifference


	// following functions are valid only when converted to delta representation.
	void generateID(vector3 const& start_transf, InterframeDifference& out) const;
	void reconstructData(vector3 const & startTransf);
	void reconstructData(vector3 const& startTransf, matrixn& out) const;
	void reconstructData(transf const& startTransf, matrixn& out) const;
	// the following function does not modify self.
	void reconstructOneFrame(vector3 const& prevRootTransf2D, vectorn const& deltaPose, vectorn & outpose) const;

	vectornView dv_x() const	{ return column(0);}
	vectornView dv_z() const	{ return column(1);}
	vectornView dq_y() const	{ return column(2);}
	vectornView offset_y() const	{return column(3);}
};

class MotionDOFview: public MotionDOF
{
public:
	// copy constructors : get reference
	MotionDOFview(const MotionDOF& other)			{ _assignRef(other);	mInfo=other.mInfo;}
	MotionDOFview(const MotionDOFview& other)		{ _assignRef(other);	mInfo=other.mInfo;}
	MotionDOFview(m_real* ptr, int nrow, int ncol, int stride, const MotionDOFinfo& info):MotionDOF(ptr, nrow, ncol, stride) { mInfo=info; }

	// assignment operators: copy values
	void operator=(const MotionDOF& other)	{ _tmat<m_real>::assign(other);mInfo=other.mInfo;}
	void operator=(const MotionDOFview& other)	{ _tmat<m_real>::assign(other);mInfo=other.mInfo;}

};

namespace MotionUtil
{
	void noiseReduction(matrixn& inout, int nKernelSize, matrixn &velocity);
}
// local representation of the root trajectory.
class InterframeDifference
{
public:
	InterframeDifference(MotionDOF const& input);
	InterframeDifference(){}
	void resize(int numFrames);
	int numFrames()	{ return dv.size();}
	
	void initFromDeltaRep(vector3 const& start_transf, matrixn const& input);
	vector3 exportToDeltaRep(matrixn & output);
	
	// assumes that frame 0 is already fixed. (startP, startRotY is given)
	void reconstruct(matrixn& output, m_real frameRate);

	vector3 startP;		// planar (y==0) 
	quater startRotY;	
	vector3N dv;		// same length as input. DOF2
	vector3N dq;		// same length as input. DOF1
	vectorn offset_y;	// DOF1
	quaterN offset_q;	// DOF3
};

// local representation of the root trajectory. (C1-continuous)
class InterframeDifferenceC1
{
public:
	InterframeDifferenceC1(MotionDOF const& input);
	InterframeDifferenceC1(m_real frameRate):_frameRate(frameRate){}
	void resize(int numFrames);
	int numFrames()	{ return dv.size();}
	
	void initFromDeltaRep(vectorn const& start_transf, matrixn const& input);

	// returns startTransf
	vectorn exportToDeltaRep(matrixn & output);

	// assumes that frame 0 is already fixed. (startPrevP, startP, startRotY is given)
	void reconstruct(matrixn& output);	// output can be MotionDOF.
	
	static vectorn getTransformation(matrixn const& motionDOF, int iframe);
	vector3 startPrevP;	// planar (y==0) 
	vector3 startP;		// planar (y==0) 
	quater startRotY;	
	m_real _frameRate;
	vector3N dv;		// same length as input. DOF2
	vector3N dq;		// same length as input. DOF1
	vectorn offset_y;	// DOF1
	quaterN offset_qy;	// DOF1
	quaterN offset_q;	// DOF3
};
#endif
