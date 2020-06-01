#pragma once

#include "../utility/TVector.h"
#include "../utility/scoped_ptr.h"
class Motion;
class Posture;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;
class boolN;
class intIntervals;
class vector3N;
class Bone;
namespace MotionUtil
{
	void calcFootPrints(const Motion& mot, bitvectorn const& con, const Bone& bone, intIntervals& interval, vector3N& footprints);
	void convertFootprintsToConstraints(intIntervals const& interval, vector3N const& footPrints, 
										intvectorn & constraintFrames, vector3N& constraints);
	void calcConstraints(const Motion& mot, int conIndex, intvectorn &constrainedFrames, vector3N& constraints);

	// refer to the implementation to understand how to use MotionRetarget class.
	void retargetEntireMotion(Motion& source, bool bRetargetFoot=true, bool bRetargetArm=true);

	class FullbodyIK;
	
	class Displacement
	{
	public:
		quaterN rot;
		vector3N trans;		
		Displacement(){}
		Displacement(Displacement const& other)	{ this->operator=(other);}
		void operator=(Displacement const& other)	{ rot=other.rot; trans=other.trans;}
		void identity(int rot_size, int trans_size);	
		void setSameSize(Displacement const& other);
		void apply(Posture& inout, intvectorn const& rot_joint_index, intvectorn const& trans_joint_index);
		void transition(Displacement const& a, Displacement const& b, int i, int duration);
	};

	typedef TVector<Displacement> DisplacementMap;
	typedef TVectorView<Displacement> DisplacementMapView;

	void DisplacementMap_fillGap(DisplacementMap& out, int duration, Displacement const& prevDelta, Displacement const& delta);	
	
	class MotionRetarget
	{
	protected:
		FullbodyIK& m_IKsolver;
		Displacement prevDelta, delta;		
	public:
		MotionRetarget(FullbodyIK& iksolver);
		virtual ~MotionRetarget(){}

		Displacement leftBoundaryDisplacement;
		Displacement rightBoundaryDisplacement;

		// 동작의 [firstFrame, lastFrame] 를 조정해서 constraint가 만족되도록 한다.
		//		
		// 동작의 firstFrame-1까지와 lastFrame+1이후는 바뀌지 않으며, firstFrame과 
		// lastFrame에서는 각각 leftBoundaryDisplacement 와 rightBoundaryDisplacement 가 적용된다. 
		// (default로는 identity 변환.)
		//
		// firstFrame, lastFrame은 실제 동작의 범위를 벗어나게 세팅될 수 있다. 
		//
		// ex) 동작 전체를 retarget하는 경우 동작의 첫프레임과 마지막프레임을 보존할 
		// 필요가 없는 경우, firstFrame=-30, lastFrame=motion.length()+30정도로 세팅하면, error propagation
		// 이 좀더 넓은 범위에서 수행될수 있어서 첫 프레임과 마지막 프레임 근처의 불필요한 discontinuity를 
		// 피할 수 있다.
		// constraints의 i-th row는 i-th constrained프레임의 constraintPositions의 vectorized version:
		//  ex) constraints.row(i)=vecView(i_th_cosPos)
		//  see FullbodyIK, baselib/math/conversion.h for more details.
		virtual void retarget(Motion const& input, intvectorn const& constrainedFrames, matrixn const& constraints, int firstFrame, int lastFrame, DisplacementMap& disp);

		// simplified interface.
		void retarget(Motion& inout, intvectorn const& constrainedFrames, matrixn const& constraints, int firstFrame, int lastFrame);

		// Utility functions
		static quater quater_transition0(const quater& aa, const quater& bb, int i, int duration);
		static quater quater_transition(const quater& aa, const quater& b, int i, int duration);
		static quater quater_transition_auto(const quater& a, const quater& b, int i, int duration);
		static vector3 vector3_transition(const vector3& a, const vector3& b, int i, int duration);		
	};

	class MotionRetarget_IKonly : public MotionRetarget
	{
	public:
		MotionRetarget_IKonly (FullbodyIK& iksolver):MotionRetarget(iksolver){}
		virtual ~MotionRetarget_IKonly (){}
		
		virtual void retarget(Motion const& input, intvectorn const& constrainedFrames, matrixn const& constraints, int firstFrame, int lastFrame, DisplacementMap& disp);
	};
	
	class _MotionRetargetOnline_data;
	class MotionRetargetOnline
	{		
		_MotionRetargetOnline_data* mData;
	public:
		MotionRetargetOnline(Motion& mot, FullbodyIK& iksolver);
		virtual ~MotionRetargetOnline();

		void notifyTransformation(const matrix4& tranf);
		// [start, nextPlayEnd) 영역만 retarget한다. 
		// 다음번 retarget이 호출될때 start는 이전 호출시의 nextPlayEnd랑 같아야한다.
		// displacement map:
		// step 1: |||||||||| - start, nextPlayEnd [0, 10)
		// step 2:          ||||||||||             [10, 20)
		// step 3:                   |||||||||     [20, 29)
		// step 4:                           ||||||||||| [29, 40) ...

		// This class guarantees that the applied displacement map at a step 
		// is continuously connected to the displacement map at the next step.
		void retarget(int start, int nextPlayEnd, intvectorn const& constrainedFrames, matrixn const& constraints);
	};

	// MotionRetarget과는 용도가 다르다. 가능하면 MotionRetarget 클래스를 쓰도록.
	class Retarget
	{
	public:
		Retarget(Motion& mot):m_mot(mot){}
		virtual ~Retarget(void){}
		// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);
		Motion& m_mot;
	};


	// IK가 잘 동작하는지 테스트 한다. (perframe IK only)
	class RetargetTest : public Retarget
	{
	public:
		RetargetTest(Motion& mot):Retarget(mot){}
		virtual ~RetargetTest(){}

		// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);

	};


	class Retarget2: public Retarget
	{
	public:
		Retarget2(Motion& mot):Retarget(mot){}
		virtual ~Retarget2(){}

		// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);

	};

}
