#ifndef _MOTION_H_
#define _MOTION_H_

#pragma once

//#include "MotionLoader.h"

#include "postureip.h"
#include "../utility/TVector.h"

class PLDPrimSkin;
class ModelLoader;
class MotionLoader;
class MotionDOF;
class MotionDOFcontainer;


//! 뼈대 정보와 해당 모션 정보를 갖는 클래스.
/*! 뼈대 정보는 m_pSkeleton에 저장되있다. 모션에 대한 HighLevel manipulation함수들은 멤버함수에 넣지 말고, MotionUtil namespace로 만들것.
*/

class MotionView;

class Motion: protected _TVector<Posture>
{
private:
	struct Info
	{
		Info()	{ clear(); }
		Info(const Info& other)	:m_fFrameTime(other.m_fFrameTime), m_strIdentifier(other.m_strIdentifier), m_pSkeleton(other.m_pSkeleton){}
		void clear()	{m_pSkeleton=NULL; m_fFrameTime=1.0f/30.0f;m_strIdentifier="";}
		void operator=(const Info& other)	{m_fFrameTime=other.m_fFrameTime;m_strIdentifier=other.m_strIdentifier;m_pSkeleton=other.m_pSkeleton;}

		float m_fFrameTime;
		TString m_strIdentifier;	//!< default로는 NULL string, 필요하면 SetIdentifier해서 사용하시오.
		MotionLoader* m_pSkeleton;	//!< 항상 reference이다. 즉 각 bvh별로 MotionLoader를 미리 생성해 놓고 포인터만 넘겨서 사용한다.
	};
	Info mInfo;

//	friend class Motion;
	friend class MotionView;
protected:
	Motion(Posture** ptrr, int size, int stride, Info const& info):_TVector<Posture>(ptrr,size,stride), mInfo(info){}

public:

	Motion():_TVector<Posture>() {}

	/**
	 * source로부터 motion을 생성한다.
	 * \param pSource Original motion을 갖고 있다.
	 * \param pNew NULL인 경우 m_pMotionData는 원본의 reference가 된다. new 만한 임의의 PostureIP를 넘기면, 내용을 original에서 복사한다.
	 * \return
	 */
	explicit Motion(MotionLoader* pSource);
	Motion(const Motion& srcMotion, int startFrame, int endFrame=INT_MAX);
	Motion(const MotionDOF& srcMotion, int startFrame=0, int endFrame=INT_MAX);
	Motion(const MotionDOFcontainer& srcMotion, int startFrame=0, int endFrame=INT_MAX);

	// DEFAULT COPY CONSTRUCTOR.
	Motion(const Motion& other):_TVector<Posture>()		{ Init(other); mInfo=other.mInfo;}

	virtual ~Motion(){}

	// L-value로 사용될수 있는, reference array를 만들어 return 한다.
	// ex) v.range(0,2)=otherMotion.range(4,6); -> selective assignment
	MotionView range(int start, int end, int step=1);
	const MotionView range(int start, int end, int step=1) const	;

	template <class VecType>
	void operator=(const VecType& other)	{ Init(other); mInfo=other.mInfo;}
	// templated assignment operator does not automatically define the default assignment operator.
	void operator=(const Motion& other)		{ Init(other); mInfo=other.mInfo;}

	inline MotionLoader& skeleton() const	{ return *mInfo.m_pSkeleton;};

	// same as skeleton().setPose(pose(iframe))
	void setSkeleton(int iframe) const;

	// 동작의 길이는 "키프레임 개수-1" 로 정의한다. 즉 인터프레임간 길이를 1로 계산한것.
	int length() const				{return numFrames()-1;}
	void changeLength(int length)	{ Resize(length+1); }

	// Init
	void empty();

	// skeleton만 초기화. 가지고 있는 MotionData가 skeleton과 incompatible한경우 MotionData는 release된다.
	void InitSkeleton(MotionLoader* pSource);

	void InitEmpty(MotionLoader* pSource, int numFrames, float fFrameTime=1.0/30.0);

	/// frame time과 skeleton을 source와 동일하게 한다.
	void InitEmpty(const Motion& source, int numFrames);

	// 똑같은 모션을 만든다.(일부분을 따와서 만들 수도 있다.)
	void Init(const Motion& srcMotion, int startFrame=0, int endFrame=INT_MAX);

	void SetIdentifier(const char* id)			{ mInfo.m_strIdentifier=id;};			//!< use as you want.
	const char* GetIdentifier() const			{ return mInfo.m_strIdentifier;};		//!< use as you want.

	Posture& pose(int iframe) const;
	inline Posture& operator[](int iframe) const	{return pose(iframe);}
	void setPose(int iframe, const Posture& pose);

	// 0 <= criticalTime <= numFrames()-1
	// 즉, 0일때 pose(0)이, numFrames()-1일때 pose(numFrames()-1)이 return된다.
	void samplePose(Posture& pose, m_real criticalTime) const;

	// skeleton과 모션을 모두 저장. ".mot"로 export. loading은 MotionLoader::loadAnimaion에서 가능하다.
	void exportMOT(const char* filename) const;

	//! pose어레이의 크기를 바꾼다. 길이가 늘어나는 경우 빈 pose들이 뒤쪽에 생긴다.
	void Resize(int frame);

	// numFrames becomes prevNumFrames+(endFrame-startFrame)
	void Concat(const Motion* pAdd, int startFrame=0, int endFrame=INT_MAX, bool bTypeCheck=true);

	int numRotJoints() const			{ return ((size()>0)?pose(0).numRotJoint():0);}
	int numTransJoints()	const		{ return ((size()>0)?pose(0).numTransJoint():0);}
	int numFrames() const				{ return size();}
	inline int size() const				{ return _TVector<Posture>::size();}

	float totalTime() const				{ return mInfo.m_fFrameTime*length();}
	// length+1 을 리턴한다.
	int numFrames(float second) const	{ return ROUND(second/frameTime())+1;}

	float frameTime() const				{ return mInfo.m_fFrameTime;}
	void frameTime(float ftime)			{ mInfo.m_fFrameTime=ftime;}
	int frameRate() const				{ return int(1.f/frameTime()+0.5f);}

	bool isConstraint(int fr, int eConstraint) const;
	void setConstraint(int fr, int con, bool bSet=true);

	bool isDiscontinuous(int fr) const;//			{ return m_aDiscontinuity[fr%m_maxCapacity];}
	bitvectorn getDiscontinuity() const;

	void setDiscontinuity(int fr, bool value);//	{ m_aDiscontinuity.setValue(fr%m_maxCapacity, value);}
	void setDiscontinuity(bitvectorn const& bit);

	enum { LOCAL_COORD, GLOBAL_COORD, FIXED_COORD, FIRST_ARRANGED_COORD, PELVIS_LOCAL_COORD, PELVIS_FIXED_COORD, NUM_COORD };

	void ChangeCoord(int eCoord);
	void CalcInterFrameDifference(int startFrame = 0);
	void ReconstructDataByDifference(int startFrame = 0, bool bForward=true)
			{ 	_reconstructRotByDifference(startFrame,bForward);	_reconstructPosByDifference(startFrame, bForward);}
	void _reconstructPosByDifference(int startFrame = 0, bool bForward=true);
	void _reconstructRotByDifference(int startFrame = 0, bool bForward=true);

	virtual void _unpack(BinaryFile& File, int nVersion);
	virtual void _pack(BinaryFile& File, int nVersion) const;

	/////////////////////////////////////////////////////////////////////
	// Do not use following deprecated functions.
	/////////////////////////////////////////////////////////////////////
	inline int NumJoints()	const				{ return numRotJoints();}


	int Parent(int jointIndex) const;//			{ return m_pSkeleton->GetParentJoint(jointIndex);};

	// skeleton을 초기화한 후, skeleton이 동작데이타(m_cPostureIP)를 가지고 있는경우 동작도 초기화.
	void Init(MotionLoader* pSource);
	void __initFromMotionDOF(const MotionDOF& srcMotion, int startFrame=0, int endFrame=INT_MAX);

	//! 똑같은 모션을 만든다.(일부분을 따와서 만들 수도 있다.)
	Motion* Clone(int startFrame=0, int endFrame=INT_MAX) const { return new Motion(*this, startFrame, endFrame);}


	void cloneFrom(const Motion& other, int otherStartFrame, int otherEndFrame, int thisStart, bool bTypeCheck=true);


	// 0.5 <=criticalTime< numFrames()-0.5
	// 즉, 0.5일때 pose(0)이, numFrames()-0.5일때 pose(numFrames()-1)이 return 된다.
	// 기존에는 동작 데이타를 discrete한 개념으로 보았기 때문에, 이 SamplePose함수가 적합했지만,
	// 2006년 6월 이후 부터는 동작데이터를 연속된 커브의 샘플링 값으로 취급하고 있다. 따라서 samplePose함수를 써야한다.
	void SamplePose(Posture& pose, m_real criticalTime) const;

	// totalTime과 TotalTime은 다른결과를 낸다는데 주의할것. -> TotalTime은 deprecated function.
	float TotalTime() const				{ return mInfo.m_fFrameTime*numFrames();}

	// If no frame in the interval is constrained, return -1. otherwise, return the indexed of first constrained frame;
	int isConstraint(int start, int end, int eConstraint) const;

	bool IsValid(int startFrame, int endFrame) const;
	bool IsValid(int iframe) const		{ return iframe>=0 && iframe<numFrames();}
	//bitvectorn& discontinuity()			{ Msg::verify(m_numFrame<m_maxCapacity, "discontinuity vector is invalid"); return m_aDiscontinuity;}
	bool IsContinuous(int iframe) const { return (iframe<numFrames() && !isDiscontinuous(iframe)); }

	// numFrames와 다른 결과를 낸다.
	int NumFrames(float fTime) const	{ return int(fTime/frameTime()+0.5);}

private:
	void _Init(int nkey, int numJoint, int numTransJoint, float fFrameTime);


};

// Note that MotionView is not std::container compatible.
class MotionView: public Motion
{
public:
	// L-value로 사용될수 있는, reference array로 만든다.
	MotionView (Posture** ptrr, int size, int stride, Motion::Info const& info):Motion(ptrr, size, stride, info){}
	// 값을 reference로 받아온다.
	template <class VecType>
	MotionView(const VecType& other)			{ assignRef(other); mInfo=other.mInfo;}
	// templated copy constructor does not automatically define the default copy constructor.
	MotionView(const MotionView& other)			{ assignRef(other); mInfo=other.mInfo;}

	~MotionView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	template <class VecType>
	void operator=(const VecType& other)		{ Motion::assign(other);}
	// templated assignment operator does not automatically define the default assignment operator.
	void operator=(const MotionView& other)	{ Motion::assign(other);}
};


#endif
