//================================================================================
//         SPHERICAL JOINT FOR MULTIBODY SYSTEM
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_GEOMETRIC_JOINT_SPHERICAL_
#define _GMBS_GEOMETRIC_JOINT_SPHERICAL_

#include "gjoint.h"
#include "gcoordinate.h"
#include "liegroup.h"
#include "rmatrix3j.h"


enum CoordinateChartForSphericalJoint { 
	EULER_ZYX = 0, 
	EULER_ZYZ = 1, 
	EULER_XYZ = 2, 
	EULER_ZXY = 3,
};

//=============================================================
//                 GJointSpherical
//=============================================================
class GJointSpherical: public GJoint
{
public:
	GCoordinate coordinates[3];		// built-in coordinates

	int coord_chart;				// type of coordinate chart 
									// (EULER_ZYX=0, EULER_ZYZ=1, EULER_XYZ=2, EULER_ZXY=3)

	bool b_fixed_coord_chart;		// set true to fix current coordinate chart, default = false.

	double min_det_J;				// minimal determinant of Jacobian to decide coordinate chart changing (Jacobian J = (GJoint::S).Sub(0,2,0,2))
									// default value = 0.5
									// ** If b_fixed_coord_chart == false, then the coordinate chart will be changed automatically 
									// to keep det(J) to be larger than min_det_J.
									// ** Note that 0 <= det(J) <= 1.

	double alpha_srInv;				// parameter for singularity-robust inverse of Jacobian
									// default value = 0.001

public:
	GJointSpherical();
	~GJointSpherical() {}

public:
	// coordinate chart
	int getCoordinateChart() { return coord_chart; }
	void setFixedCoordinateChart(CoordinateChartForSphericalJoint cc_);	// set a fixed coordinate chart
	void resetCoordinateChart() { coord_chart = EULER_ZYX; b_fixed_coord_chart = false; }

	// set coordinates[]->(q,dq,ddq) with given set of (R, dot_R, ddot_R) or (R, w, dot_w)
	// R = SO3: {joint left} --> {joint right}
	// dot_R = time derivative of R (3x3 matrix)
	// ddot_R = time derivative of dot_R (3x3 matrix)
	// w = R^T dot_R = angular velocity of {joint right} w.r.t. {joint left} viewed in {joint left}
	// dot_w = R_^T ddot_R - R^T dot_R R^T dot_R = time derivative of w
	// ** if b_fixed_coord_chart == false, the coordinate chart will be CHANGED if it is needed to avoid singularity. 
	// ** if b_fixed_coord_chart == true, singularity-robust inverse of Jacobian will be used to handle singularity instead of changing the coordinate chart.
	void setMotion(const SO3 &R, const RMatrix &dot_R, const RMatrix &ddot_R);
	void setMotion(const SO3 &R, const Vec3 &w, const Vec3 &dot_w);

public:
	bool isConstantScrew() { return false; }

	void update_short();
	void update();

	void _validateCoordinateChart();			// changes coordinate chart if needed (currently supports EULER_ZYX <--> EULER_ZYZ only)
	void _update_short_for_reversed_joint();	// modify T, inv_T, S for reversed joint
	void _update_for_reversed_joint();			// modify T, inv_T, S, dS, Sdq, dSdq, Sddq, DSdqDt for reversed joint

	RMatrix get_DSDq(GCoordinate *pCoordinate_);
	RMatrix get_DdSDq(GCoordinate *pCoordinate_);
};


#endif

