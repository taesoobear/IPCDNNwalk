#pragma once
#include <set>
#include "../math/transf.h"
class Motion;
class matrixn;
class vector3N;
class quaterN;
class boolN;
class Bone;
#include "MotionWrap.h"

namespace MotionUtil
{
	typedef transf Coordinate ;
	
	enum {LOCAL_COORD, GLOBAL_COORD, FIXED_COORD, FIRST_FRAME_CENTERED_COORD};

	struct GetSignal	// 동작의 start 부터 end사이가 연속된 하나의 동작이라고 가정한다.
	{
		GetSignal(MotionWrap const& m);
		GetSignal(const Motion& in);
		GetSignal(const MotionDOF& in);
		
		// dv(2), dq(1), height (1), offset(4) total 8 dimensions
		void interframeDifference(matrixn& out, int start=0, int end=INT_MAX);
		void constraintPositions(matrixn& out, int start=0, int end=INT_MAX);
		// orientations
		void root(matrixn& out, int start=0, int end=INT_MAX);
		void root(vector3N& out, int start=0, int end=INT_MAX);
		void offsetQ(matrixn& out, int start=0, int end=INT_MAX);
		void joint(int ijoint, matrixn& out, int start=0, int end=INT_MAX);
		void joint(int ijoint, quaterN& out, int start=0, int end=INT_MAX);
		void transJoint(int ijoint, matrixn& out, int start=0, int end=INT_MAX);
		void transJoint(int ijoint, vector3N& out, int start=0, int end=INT_MAX);

		void additionalLinear(matrixn& out, int start=0, int end=INT_MAX);
		void additionalQuater(matrixn& out, int start=0, int end=INT_MAX);

		// retrieve joint orientations 
		// out: (end-start) by (ajoint.size()*4) matrix
		// to retrieve the signal of the joint ajoint[i].
		//  -> quatViewCol(out, i*3)
		//  or out.range(startF, endF, i*4, (i+1)*4).toQuaterN() 
		
		void jointAll(const intvectorn& ajoint, matrixn& out, int start=0, int end=INT_MAX);

		// retrieve joint translations	-> Only for translational joints!!!
		// out: (end-start) by (ajoint.size()*3) matrix
		// to retrieve the signal of the joint ajoint[i].
		//  -> vec3ViewCol(out, i*3)
		//  or out.range(startF, endF, i*3, (i+1)*3).toVector3N()
		void transJointAll(const intvectorn& ajoint, matrixn& out, int start=0, int end=INT_MAX);

		// calc global joint orientations (the most efficient way to do this)
		// out: (end-start) by (numJoint*4) matrix
		// to retrieve joint i.
		//   -> out.range(start, end, i*4, (i+1)*4).toQuaterN() 
		// currently eRootCoord should be either GLOBAL_COORD or LOCAL_COORD. 
		void jointGlobalAll(int eRootCoord, matrixn& out, int start=0, int end=INT_MAX);
		void jointGlobal(const Bone& bone, matrixn& out, int start=0, int end=INT_MAX);
		void jointGlobal(int ijoint, matrixn& out, int start=0, int end=INT_MAX);
		void jointFixed(int ijoint, matrixn& out, int start=0, int end=INT_MAX);

		// positions
		void jointPos(const Bone& bone, matrixn& out, int start=0, int end=INT_MAX);
		void jointPos(int ijoint, matrixn& out, int start=0, int end=INT_MAX);
		void jointPosLocal(const Bone& bone, matrixn& out, int start=0, int end=INT_MAX);
		void jointPosFixed(const Bone& bone, matrixn& out, int start=0, int end=INT_MAX);
		void jointPosFirstCentered(const Bone& bone, matrixn& out, int start=0, int end=INT_MAX);

		void constraint(int iconstraint, bitvectorn& con, int start=0, int end=INT_MAX);

		// utility functions
		void jointPos(const intvectorn& ajoint, hypermatrixn& aPos, int eCoord=GLOBAL_COORD, int start=0, int end=INT_MAX);
		void jointOri(const intvectorn& ajoint, hypermatrixn& aOri, int eCoord=LOCAL_COORD, int start=0, int end=INT_MAX);
		
		MotionWrap m_Motion;
	};

	struct SetSignal
	{
		SetSignal(Motion& out):m_Motion(out){}	

		void offsetQ(matrixn const& in, int start=0);
		void interframeDifference(matrixn const& in, int start=0, int end=INT_MAX);
		void constraintPositions(matrixn const & in, int start=0, int end=INT_MAX);
		void root(const matrixn& in);
		void root(const vector3N& in, int start=0);
		void joint(int ijoint, const matrixn& in);		
		void joint(int ijoint, const quaterN& in, int start=0);		
		void transJoint(int ijoint, const matrixn& in);		
		void transJoint(int ijoint, const vector3N& in, int start=0);		

		void additionalLinear(matrixn const& in, int start=0);
		void additionalQuater(matrixn const& in, int start=0);

		void constraint(int iconstraint, bitvectorn const& con, int start=0);

		// calc joint orientations 
		// out: (end-start) by (ajoint.size()*4) matrix
		// to retrieve the signal of the joint ajoint[i].
		//   -> quatViewCol(out, i*4)
		//   or out.range(startF, endF, i*4, (i+1)*4).toQuaterN() 
		void jointAll(const intvectorn& ajoint, matrixn const& in, int start=0);

		// set joint translations (for translational joints only)
		// out: (end-start) by (ajoint.size()*3) matrix
		// to retrieve the signal of the joint ajoint[i].
		//   -> vec3ViewCol(out, i*3)
		//   or out.range(startF, endF, i*3, (i+1)*3).toVector3N() 
		void transJointAll(const intvectorn& ajoint, matrixn const& in, int start=0);

		// calc local orientations from global joint orientations (the most efficient way to do this)
		// in: (end-start) by (numJoint*4) matrix
		// where in.range(start, end, i*4, (i+1)*4).toQuaterN() is global orientations for the joint i.
		// currently eRootCoord should be either GLOBAL_COORD or LOCAL_COORD. in case of LOCAL_COORD, root orientations will not be copied to m_Motion.
		void jointGlobalAll(int eRootCoord, matrixn const& in, int start=0);

		Motion& m_Motion;
	};


	// 동작전체에 대해 작동하고, 동작이 여러동작을 concatenate한 경우, 즉 discontinuity가 있는경우를 감안하여 작성된 코드들.
	struct GetSignals 
	{		
		GetSignals(const Motion& in):m_Motion(in){}
		void jointPosVel(const Bone& bone, matrixn& aPos, matrixn& aVel, int eCoord=GLOBAL_COORD, float fSmoothKernel=0.f);
		void jointVel(const Bone& bone, matrixn& out, int eCoord=GLOBAL_COORD, float fSmoothKernel=0.f);
		
		void jointPos(const intvectorn& aJointIndex, hypermatrixn& aaPos, int eCoord=GLOBAL_COORD);
		void jointOri(const intvectorn& aJointIndex, hypermatrixn& aaPos, int eCoord=LOCAL_COORD);
		void jointVel(const intvectorn& aJointIndex, hypermatrixn& aaVel, int eCoord=GLOBAL_COORD, float fSmoothKernel=0.f);
		const Motion& m_Motion;
	};

	intvectorn findChildren(MotionLoader& skeleton, int rootTreeIndex);

	m_real transitionCost(const Motion& in, int from, int to);	//!<  from까지 play하고 to+1부터 play하는 경우 transition cost, Kovar metric
	void scale(Motion& inout, float ratio);				//!< root translation을 scale한다.
	
	void translate(Motion& inout, const vector3& trans, int start=0, int end=INT_MAX);	//!< Motion 전체를 Translate한다. 즉 root position의 translation
	void rotate(Motion& inout, const quater& q, int start=0, int end=INT_MAX ); //!< MotionData 전체를 Rotate한다. root orientation quat앞에 곱해져서 제일 나중에 적용된다.
	void rigidTransform(Motion& inout, const transf& f, int start=0, int end=INT_MAX);

	// for continuous motion
	void upsample(Motion& out, const Motion& in, int startFrame, int endFrame, int nSuperSample);
	// for every motion
	void upsample(Motion& out, const Motion& in, int nSuperSample);
	// for every motion
	void downsample(Motion& out, const Motion& in, int nDownSample);


	// Timewarping함수
	/**
	 * 주어진 disiredLen길이의 timewarpFunction을 사용해 pointsampling방식으로 timewarping한다.
	 * \param timewarpFunction uniform resampling을 하려면, timewarpedFunction.linspace(0, numFrames, desiredLen+1)으로 채운다.
	 * timewarpingFunciton=[0, 1, 2, 3, 4]->[0,1,2,3,4] point sampling
	 * timewarpingFunciton=[0.5, 1.5, 2.5, 3.5]->[0-1,1-2,2-3,3-4] center samping
	 * timewarpingFunciton=[0, 2, 4, 8]-> twice faster
	 */	
	void timewarpingLinear(Motion& out, const Motion& in, const vectorn& timewarpFunction);
	// Assuming timewarpFunction[0]==0, untimewarp!
    Motion* untimewarpingLinear(const Motion& timewarpedMotion, const vectorn& timewarpFunction);

	// symmetric motion. mirroring.
	void mirrorMotion(Motion& out, const Motion& in, intvectorn const& LrootIndices, intvectorn const& RrootIndices);
	// symmetric motion. mirroring. (deprecated)
	void transpose(Motion& out, const Motion& in);

	void smooth(Motion& out, const Motion& in, float kernelRoot, float kernelJoint);

	/// 모션이 여러개의 짧은 모션클립을 concat한 긴 모션인 경우, 클립단위로 startFrame과 endFrame을 알아낸다.
	class SegmentFinder
	{
	public:
		SegmentFinder(const Motion& motion, int startFrame=0, int endFrame=INT_MAX);
		~SegmentFinder(){}
		int numSegment() const	{ return m_vStart.size();}
		int startFrame(int iSegment) const	{ return m_vStart[iSegment];}
		int endFrame(int iSegment) const	{ return m_vEnd[iSegment];}
	private:
		intvectorn m_vStart;
		intvectorn m_vEnd;
	};

	// 아래는 deprecated

	// Timewarping함수
	/**
	 * 주어진 disiredLen길이의 timewarpFunction을 사용해 pointsampling방식으로 timewarping한다.
	 * \param timewarpFunction uniform resampling을 하려면, timewarpedFunction.uniform(0, numFrames, desiredLen)으로 채운다.
	 * timewarpingFunciton=[0.5, 1.5, 2.5, 3.5, 4.5]->[0,1,2,3,4] point sampling
	 * timewarpingFunciton=[1, 2, 3, 4, 5]->[0-1,1-2,2-3,3-4,4-5] center samping
	 */
	void timewarping(Motion& out, const Motion& in, const vectorn& timewarpFunction);

	// playEnd 이후의 프레임을 조절할 수 있다. criticalTimeBefore에 해당하는 자세가 criticalTimeAfter에 나오도록 조절한다.
	void timewarping(Motion& inout, int playEnd, int criticalTimeBefore, int criticalTimeAfter);
	void exportBVH(Motion const& mot, const char* filename, int start=0, int end=100000);

}
