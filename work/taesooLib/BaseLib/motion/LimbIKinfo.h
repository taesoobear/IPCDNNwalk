
#ifndef LIMBIKinfo_H_
#define LIMBIKinfo_H_

#include "FullbodyIK_MotionDOF.h"
namespace MotionUtil{
	struct LimbIKinfo {
		//sh : hip position
		//q1 : hip angle
		//elb : knee position
		//q2 : knee angle
		//v1 : hip-knee  nat
		//v3 : hip-knee  cur
		//wrist : ankle position
		//v2 : knee-ankle nat
		//v4 : knee-ankle cur
		//goal : ankle
		quater qo1, qo2,qt;
		quater q0, q1, q2;
		vector3 goal, sh, elb, v1, v2, v3, v4,v5, wrist,hand, leg_delta;
		vector3 hipoffset;
		const Bone *hip, *knee, *ankle;
		vector3 axis;
		void init(Bone const& knee, Bone* anklebone, double axis_sign);
		// get current hip, knee, ankle positions
		void prepare(quater const& conori, const vector3 & localpos);
		// output goes to (hip, knee, ankle)->_getLocalFrame().rotation
		void limbIK(double importance, quater const& conori, double kneeDampingConstant, bool useKneeDamping=true);
	};
}
#endif
