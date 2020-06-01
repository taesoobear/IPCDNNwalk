// ConstraintMarking.h: interface for the CConstraintMarking class.
//
//////////////////////////////////////////////////////////////////////

#pragma once

class Motion;
class Posture;
class MotionLoader;
class Bone;
class CImage;

namespace MotionUtil
{
	class FootstepDetection
	{
	public:
		FootstepDetection(Motion* pMotion, bool bCleanup=false, bool bFillGap=false, int minConDuration=1, int reduceCon=0);
		virtual ~FootstepDetection(){}

		void calcConstraint(Bone& bone, int con, float height_thr, float speed_thr, const char* outputImage=NULL);

		bool m_bCleanup;
		bool m_bFillGap;
		int m_minConDuration;
		int m_reduceCon;
		Motion* m_pAccurateMotion;

		bitvectorn m_abCon;

		vectorn m_aSpeed;
		vectorn m_aHeight;

		// utility
		static void cleanup(bitvectorn& abCon, int startFrame, int endFrame, const vectorn& aSpeed, const matrixn& pos );
		static void fillGap(bitvectorn& ab, int numPosture, int interval);
		static void fillGapUsingPos(bitvectorn &ab, int numPosture, const matrixn& pos, int interval, float thr);
		static void calcConstraintSep(Motion* pMot, float cut_value, float vel_cutvalue, const matrixn& pos, const vectorn& aSpeed, bitvectorn& abCon, bool bCleanup, int minConDuration, int reduceCon, bool bFillGap);
		static void removeShortCon(bitvectorn & abCon, int minConDuration);
		static void shortenCon(bitvectorn & abCon, int shortenAmount);

	};

	// deprecated. use FootstepDetection
	class ConstraintMarking
	{
	public:
		ConstraintMarking(Motion* pMotion, bool bCleanup=false, bool bFillGap=false, int minConDuration=1, int reduceCon=0);
		virtual ~ConstraintMarking();

		void calcConstraint(float toe_height_thr, float toe_speed_thr, float heel_height_thr, float heel_speed_thr, bool bOutputImage=true, int eHowToChooseConstraint=CHOOSE_OR);

		bool m_bCleanup;
		bool m_bFillGap;
		int m_minConDuration;
		int m_reduceCon;
		Motion* m_pAccurateMotion;

		enum { CON_TOE, CON_HEEL, NUM_CON};
		enum { CHOOSE_AND, CHOOSE_OR, CHOOSE_TOE, CHOOSE_HEEL};

		bitvectorn m_abLeftFoot;
		bitvectorn m_abRightFoot;
		bitvectorn m_abLeft[2];
		bitvectorn m_abRight[2];


		void calcConstraintPos(int constraint);

		static void encodeCon(int con, Posture& pose, MotionLoader& skeleton, vector3 const& conPos);
		static void decodeCon(int con, Posture& pose, MotionLoader& skeleton, vector3 & conPos);
	
		//utility
		static vector3 decodeCon(Motion const& mot, int iframe, int con)	;

	private:
		matrixn m_aaSpeedLeft;	// LspeedVec, RspeedVec
		matrixn m_aaSpeedRight;

	
	
		

	
};
	
}

