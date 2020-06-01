#ifndef MOTION_RETARGET_H_
#define MOTION_RETARGET_H_
#pragma once

class Motion;
class MotionDOF;
class matrixn;
class vector3N;
class quaterN;
class boolN;
class Bone;
class Path2D;

namespace MotionUtil
{
	class DeltaRep
	{
		static quater I;
		public:
			DeltaRep(){}
			virtual ~DeltaRep(){}
			virtual void decomposeRot(int iframe){}
			virtual void composeRot(int iframe){}
			virtual quater & rotY(int iframe) { return I;}
			virtual quater & dQ(int iframe) { return I;}
			virtual vector3 pos(int iframe) const{ return vector3(0,0,0);}
			virtual void setPos(int iframe, double px,double pz){}
			virtual int numFrames() const{ return 0;}
			virtual void calcInterFrameDifference(int startFrame){}
			virtual void reconstructPos(int startFrame){}
			virtual void reconstructAll(int startFrame){}
	};

	class RetargetOnline2D
	{
		int mStart;
		int mRetargetQMethod;
		matrixn mCurve;
		DeltaRep *mTarget;

		void getCurve(int start);
	public:
		enum {RETARGET_ROTY, RETARGET_DQ};

		// source는 smooth한 동작으로 가정한다. start를 포함하여 그 이후의 동작만 조정한다.
		RetargetOnline2D(Motion& target, int start, int eRetargetQMethod=RETARGET_ROTY);
		RetargetOnline2D(MotionDOF& target, int start, int eRetargetQMethod=RETARGET_ROTY);
		~RetargetOnline2D();
						
		// 가정: time>start
		void adjust(int time, quater const& oriY, vector3 const& pos2D);
		void adjust(int time, quater const& oriY);
		void adjust(int time, vector3 const& pos2D);
		void adjust(int time, m_real deltarot);
		// 마지막 프레임의 orientation을 변화시키지 않는다. 그 대신, time에 틀어진만큼 이후에 돌아오는 회전이 추가된다.
		void adjustSafe(int time, m_real deltarot);

		// path2D의 마지막 프레임에서 정확히 같도록 조정하되, path2D랑 오차가 너무 크지 않도록 중간중간 컨스트레인이 추가됨.
		void adjustToPath(Path2D & path, int frameThr=INT_MAX, m_real angleThr=FLT_MAX, m_real distThr=FLT_MAX);


		// currently only works for Motion class. (not MotionDOF)
		// time을 time2가 되도록 timewarping한다. times는 이전에 frame이 retarget후 어느 프레임에 해당하는지를 계산한다.
		// times은 증가순으로 소팅되어있다고 가정한다.
		void adjust(int time, int time2, intvectorn& times);	
	};


}
#endif
