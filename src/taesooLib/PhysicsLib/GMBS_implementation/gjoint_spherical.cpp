#include <list>
#include "gjoint_spherical.h"
#include "gjoint.h"
#include "gelement.h"
#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"


static const double PI = 3.14159265358979;

//=============================================================
//                 GJointSpherical
//=============================================================
GJointSpherical::GJointSpherical()
{
	jointType = GJOINT_SPHERICAL;
	coord_chart = EULER_ZYX;
	b_fixed_coord_chart = false;
	min_det_J = 0.5;
	alpha_srInv = 0.001;
	pCoordinates.push_back(&coordinates[0]);
	pCoordinates.push_back(&coordinates[1]);
	pCoordinates.push_back(&coordinates[2]);
	allocate_memory(3);
}

void GJointSpherical::setFixedCoordinateChart(CoordinateChartForSphericalJoint cc_) 
{ 
	coord_chart = cc_; 
	b_fixed_coord_chart = true;
}

void GJointSpherical::update_short()
{
	_validateCoordinateChart();

	double q0, q1, q2, dq0, dq1, dq2, ddq0, ddq1, ddq2, c0, c1, c2, s0, s1, s2;

	q0 = coordinates[0].q; q1 = coordinates[1].q; q2 = coordinates[2].q;
	dq0 = coordinates[0].dq; dq1 = coordinates[1].dq; dq2 = coordinates[2].dq;
	ddq0 = coordinates[0].ddq; ddq1 = coordinates[1].ddq; ddq2 = coordinates[2].ddq;
	c0 = cos(q0); c1 = cos(q1); c2 = cos(q2);
	s0 = sin(q0); s1 = sin(q1); s2 = sin(q2);

	switch ( coord_chart ) {

		case EULER_ZYX:

			T = SE3(c0*c1, 
				s0*c1, 
				-s1, 
				c0*s1*s2 - s0*c2, 
				s0*s1*s2 + c0*c2, 
				c1*s2, 
				c0*s1*c2 + s0*s2, 
				s0*s1*c2 - c0*s2, 
				c1*c2, 
				0.0, 0.0, 0.0);

			inv_T = SE3(~T.GetRotation());

			// S = [   -s1,    0,   1
			//       s2*c1,   c2,   0
			//       c1*c2,  -s2,   0 
			//           0,    0,   0
			//           0,    0,   0
			//           0,    0,   0 ];
			S.SetZero();
			S[0] = -s1; S[1] = s2*c1; S[2] = c1*c2; 
			S[7] = c2; S[8] = -s2; 
			S[12] = 1.0;

			break;

		case EULER_ZYZ:

			T = SE3(c0*c1*c2 - s0*s2, 
				s0*c1*c2 + c0*s2, 
				-s1*c2,	
				-c0*c1*s2 - s0*c2, 
				c0*c2 - s0*c1*s2, 
				s1*s2, 
				c0*s1, 
				s0*s1, 
				c1, 
				0, 0, 0);

			inv_T = SE3(~T.GetRotation());

			// S = [-s1*c2,   s2,   0
			//       s1*s2,   c2,   0
			//          c1,    0,   1 
			//           0,    0,   0
			//           0,    0,   0
			//           0,    0,   0 ];
			S.SetZero();
			S[0] = -s1*c2; S[1] = s1*s2; S[2] = c1; 
			S[6] = s2; S[7] = c2; 
			S[14] = 1.0;

			break;

		case EULER_XYZ:
			
			T = SE3(c1*c2,
				c2*s0*s1 + c0*s2,
				-(c0*c2*s1) + s0*s2,
				-(c1*s2),
				c0*c2 - s0*s1*s2,
				c2*s0 + c0*s1*s2,
				s1,
				-(c1*s0),
				c0*c1,
				0.0, 0.0, 0.0);

			inv_T = SE3(~T.GetRotation());
			
			// S = [    c1*c2, s2,  0
			//       -(c1*s2), c2,  0
			//             s1,  0,  1
			//              0,  0,  0
			//              0,  0,  0
			//              0,  0,  0 ];
			S.SetZero();
			S[0] = c1*c2; S[1] = -(c1*s2); S[2] = s1; 
			S[6] = s2; S[7] = c2; 
			S[14] = 1;
			
			break;

		case EULER_ZXY:

			T = SE3(c0*c2 - s0*s1*s2,
				c2*s0 + c0*s1*s2,
				-(c1*s2),
				-(c1*s0),
				c0*c1,
				s1,
				c2*s0*s1 + c0*s2,
				-(c0*c2*s1) + s0*s2,
				c1*c2,
				0.0, 0.0, 0.0);

			inv_T = SE3(~T.GetRotation());
			
			// S = [ -(c1*s2), c2,  0
			//             s1,  0,  1
			//          c1*c2, s2,  0
			//              0,  0,  0
			//              0,  0,  0
			//              0,  0,  0 ];
			S.SetZero();
			S[0] = -(c1*s2); S[1] = s1; S[2] = c1*c2; 
			S[6] = c2; S[8] = s2;
			S[13] = 1;
			
			break;

		default:
			T.SetIdentity();
			inv_T.SetIdentity();
			S.SetZero();

			break;
	}

	if ( bReversed ) { _update_short_for_reversed_joint(); }
}

void GJointSpherical::update()
{
	_validateCoordinateChart();

	double q0, q1, q2, dq0, dq1, dq2, ddq0, ddq1, ddq2, c0, c1, c2, s0, s1, s2;

	q0 = coordinates[0].q; q1 = coordinates[1].q; q2 = coordinates[2].q;
	dq0 = coordinates[0].dq; dq1 = coordinates[1].dq; dq2 = coordinates[2].dq;
	ddq0 = coordinates[0].ddq; ddq1 = coordinates[1].ddq; ddq2 = coordinates[2].ddq;
	c0 = cos(q0); c1 = cos(q1); c2 = cos(q2);
	s0 = sin(q0); s1 = sin(q1); s2 = sin(q2);

	switch ( coord_chart ) {

		case EULER_ZYX:
			
			T = SE3(c0*c1, 
				s0*c1, 
				-s1, 
				c0*s1*s2 - s0*c2, 
				s0*s1*s2 + c0*c2, 
				c1*s2, 
				c0*s1*c2 + s0*s2, 
				s0*s1*c2 - c0*s2, 
				c1*c2, 
				0.0, 0.0, 0.0);

			inv_T = SE3(~T.GetRotation());
			
			Sdq = se3(-s1*dq0 + dq2,
				s2*c1*dq0 + c2*dq1,
				c1*c2*dq0 - s2*dq1,
				0.0, 0.0, 0.0);
			
			dSdq = se3(-c1*dq1*dq0,
				(c2*c1*dq2 - s2*s1*dq1)*dq0 - s2*dq2*dq1,
				(-s1*c2*dq1 - c1*s2*dq2)*dq0 - c2*dq2*dq1,
				0, 0, 0);
			
			Sddq = se3(-s1*ddq0 + ddq2,
				s2*c1*ddq0 + c2*ddq1,
				c1*c2*ddq0 - s2*ddq1,
				0, 0, 0);
			
			DSdqDt = Sddq + dSdq;
			
			// S = [   -s1,    0,   1
			//       s2*c1,   c2,   0
			//       c1*c2,  -s2,   0 
			//           0,    0,   0
			//           0,    0,   0
			//           0,    0,   0 ];
			S.SetZero();
			S[0] = -s1; S[1] = s2*c1; S[2] = c1*c2; 
			S[7] = c2; S[8] = -s2; 
			S[12] = 1.0;
			
			// dS = [               -c1*dq1,        0,   0
			//          c2*c1*dq2-s2*s1*dq1,  -s2*dq2,   0
			//         -s1*c2*dq1-c1*s2*dq2,  -c2*dq2,   0 
			//                            0,        0,   0
			//                            0,        0,   0
			//                            0,        0,   0 ];
			dS.SetZero();
			dS[0] = -c1*dq1; dS[1] = c2*c1*dq2 - s2*s1*dq1;	dS[2] = -s1*c2*dq1 - c1*s2*dq2;	
			dS[7] = -s2*dq2; dS[8] = -c2*dq2;

			break;

		case EULER_ZYZ:

			T = SE3(c0*c1*c2 - s0*s2, 
				s0*c1*c2 + c0*s2, 
				-s1*c2,	
				-c0*c1*s2 - s0*c2, 
				c0*c2 - s0*c1*s2, 
				s1*s2, 
				c0*s1, 
				s0*s1, 
				c1, 
				0, 0, 0);
			
			inv_T = SE3(~T.GetRotation());

			Sdq = se3(-s1*c2*dq0 + s2*dq1,
				s1*s2*dq0 + c2*dq1,
				c1*dq0 + dq2,
				0, 0, 0);
			
			dSdq = se3((-c1*c2*dq1 + s1*s2*dq2)*dq0 + c2*dq2*dq1,
				(c1*s2*dq1 + s1*c2*dq2)*dq0 - s2*dq2*dq1,
				-s1*dq1*dq0,
				0, 0, 0);
			
			Sddq = se3(-s1*c2*ddq0 + s2*ddq1,
				s1*s2*ddq0 + c2*ddq1,
				c1*ddq0 + ddq2,
				0, 0, 0);
			
			DSdqDt = Sddq + dSdq;
			
			// S = [-s1*c2,   s2,   0
			//       s1*s2,   c2,   0
			//          c1,    0,   1 
			//           0,    0,   0
			//           0,    0,   0
			//           0,    0,   0 ];
			S.SetZero();
			S[0] = -s1*c2; S[1] = s1*s2; S[2] = c1; 
			S[6] = s2; S[7] = c2; 
			S[14] = 1.0;
			
			// dS = [-c1*c2*dq1+s1*s2*dq2,   c2*dq2,   0
			//          c1*s2*dq1+s1*c2*dq2,  -s2*dq2,   0
			//                      -s1*dq1,        0,   0 
			//                            0,        0,   0
			//                            0,        0,   0
			//                            0,        0,   0 ];
			dS.SetZero();
			dS[0] = -c1*c2*dq1 + s1*s2*dq2; dS[1] = c1*s2*dq1 + s1*c2*dq2; dS[2] = -s1*dq1;
			dS[6] = c2*dq2; dS[7] = -s2*dq2;

			break;

		case EULER_XYZ:
			
			T = SE3(c1*c2,
				c2*s0*s1 + c0*s2,
				-(c0*c2*s1) + s0*s2,
				-(c1*s2),
				c0*c2 - s0*s1*s2,
				c2*s0 + c0*s1*s2,
				s1,
				-(c1*s0),
				c0*c1,
				0.0, 0.0, 0.0);

			inv_T = SE3(~T.GetRotation());
			
			Sdq = se3(dq0*c1*c2 + dq1*s2,
				dq1*c2 - dq0*c1*s2,
				dq2 + dq0*s1,
				0.0, 0.0, 0.0);
			
			dSdq = se3(dq1*dq2*c2 - dq0*(dq1*c2*s1 + dq2*c1*s2),
				-(dq1*dq2*s2) + dq0*(-(dq2*c1*c2) + dq1*s1*s2),
				dq0*dq1*c1,
				0, 0, 0);
			
			Sddq = se3(ddq0*c1*c2 + ddq1*s2,
				ddq1*c2 - ddq0*c1*s2,
				ddq2 + ddq0*s1,
				0, 0, 0);
			
			DSdqDt = Sddq + dSdq;

			// S = [    c1*c2, s2,  0
			//       -(c1*s2), c2,  0
			//             s1,  0,  1
			//              0,  0,  0
			//              0,  0,  0
			//              0,  0,  0 ];
			S.SetZero();
			S[0] = c1*c2; S[1] = -(c1*s2); S[2] = s1; 
			S[6] = s2; S[7] = c2; 
			S[14] = 1;
			
			// dS = [  -(dq1*c2*s1) - dq2*c1*s2,    dq2*c2,  0
			//         -(dq2*c1*c2) + dq1*s1*s2, -(dq2*s2),  0
			//                           dq1*c1,         0,  0
			//                                0,         0,  0
			//                                0,         0,  0
			//                                0,         0,  0 ];
			dS.SetZero();
			dS[0] = -(dq1*c2*s1) - dq2*c1*s2; dS[1] = -(dq2*c1*c2) + dq1*s1*s2; dS[2] = dq1*c1;
			dS[6] = dq2*c2; dS[7] = -(dq2*s2);

			break;

		case EULER_ZXY:

			T = SE3(c0*c2 - s0*s1*s2,
				c2*s0 + c0*s1*s2,
				-(c1*s2),
				-(c1*s0),
				c0*c1,
				s1,
				c2*s0*s1 + c0*s2,
				-(c0*c2*s1) + s0*s2,
				c1*c2,
				0.0, 0.0, 0.0);

			inv_T = SE3(~T.GetRotation());
			
			Sdq = se3(dq1*c2 - dq0*c1*s2,
				dq2 + dq0*s1,
				dq0*c1*c2 + dq1*s2,
				0.0, 0.0, 0.0);
			
			dSdq = se3(-(dq1*dq2*s2) + dq0*(-(dq2*c1*c2) + dq1*s1*s2),
				dq0*dq1*c1,
				dq1*dq2*c2 - dq0*(dq1*c2*s1 + dq2*c1*s2),
				0, 0, 0);
			
			Sddq = se3(ddq1*c2 - ddq0*c1*s2,
				ddq2 + ddq0*s1,
				ddq0*c1*c2 + ddq1*s2,
				0, 0, 0);
			
			DSdqDt = Sddq + dSdq;

			// S = [ -(c1*s2), c2,  0
			//             s1,  0,  1
			//          c1*c2, s2,  0
			//              0,  0,  0
			//              0,  0,  0
			//              0,  0,  0 ];
			S.SetZero();
			S[0] = -(c1*s2); S[1] = s1; S[2] = c1*c2; 
			S[6] = c2; S[8] = s2;
			S[13] = 1;
			
			// dS = [  -(dq2*c1*c2) + dq1*s1*s2, -(dq2*s2),  0
			//                           dq1*c1,         0,  0
			//         -(dq1*c2*s1) - dq2*c1*s2,    dq2*c2,  0
			//                                0,         0,  0
			//                                0,         0,  0
			//                                0,         0,  0 ];
			dS.SetZero();
			dS[0] = -(dq2*c1*c2) + dq1*s1*s2; dS[1] = dq1*c1; dS[2] = -(dq1*c2*s1) - dq2*c1*s2;
			dS[6] = -(dq2*s2); dS[8] = dq2*c2;

			break;

		default:
			T.SetIdentity();
			inv_T.SetIdentity();
			Sdq.SetZero();
			dSdq.SetZero();
			Sddq.SetZero();
			DSdqDt.SetZero();
			S.SetZero();
			dS.SetZero();

			break;
	}

	if ( bReversed ) { _update_for_reversed_joint(); }
}

RMatrix GJointSpherical::get_DSDq(GCoordinate *pCoordinate_)
{
	int idx;
	double c1, c2, s1, s2;
	RMatrix DSDq;
	
	if ( pCoordinate_ == &coordinates[0] ) {
		idx = 0;
	} else if ( pCoordinate_ == &coordinates[1] ) {
		idx = 1;
	} else if ( pCoordinate_ == &coordinates[2] ) {
		idx = 2;
	} else {
		return Zeros(6,3);
	}

	DSDq.SetZero(6,3);

	switch ( coord_chart ) {

		case EULER_ZYX:

			switch ( idx ) {
				case 0:
					break;

				case 1:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					// DsDq = [    -c1, 0, 0
					//          -s2*s1, 0, 0
					//          -s1*c2, 0, 0
					//               0, 0, 0
					//               0, 0, 0
					//               0, 0, 0 ];
					DSDq[0] = -c1;		
					DSDq[1] = -s2*s1;
					DSDq[2] = -s1*c2;
					break;

				case 2:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					// DsDq = [      0,   0, 0
					//           c2*c1, -s2, 0
					//          -c1*s2, -c2, 0
					//               0,   0, 0
					//               0,   0, 0
					//               0,   0, 0 ];
					DSDq[1] = c2*c1;		
					DSDq[2] = -c1*s2;
					DSDq[7] = -s2;
					DSDq[8] = -c2;
					break;
			}

			break;

		case EULER_ZYZ:

			switch ( idx ) {
				case 0:
					break;

				case 1:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					// DsDq = [ -c1*c2, 0, 0
					//           c1*s2, 0, 0
					//             -s1, 0, 0
					//               0, 0, 0
					//               0, 0, 0
					//               0, 0, 0 ];
					DSDq[0] = -c1*c2;
					DSDq[1] = c1*s2;
					DSDq[2] = -s1;
					break;

				case 2:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					// DsDq = [ s1*s2,  c2, 0
					//          s1*c2, -s2, 0
					//              0,   0, 0
					//              0,   0, 0
					//              0,   0, 0
					//              0,   0, 0 ];
					DSDq[0] = s1*s2;
					DSDq[1] = s1*c2;
					DSDq[6] = c2;
					DSDq[7] = -s2;
					break;
			}

			break;

		case EULER_XYZ:

			switch ( idx ) {
				case 0:
					break;

				case 1:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					// DsDq = [-(c2*s1),0, 0
					//           s1*s2, 0, 0
					//              c1, 0, 0
					//               0, 0, 0
					//               0, 0, 0
					//               0, 0, 0 ];
					DSDq[0] = -(c2*s1);
					DSDq[1] = s1*s2;
					DSDq[2] = c1;
					break;

				case 2:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					// DsDq = [ -(c1*s2),  c2, 0
					//          -(c1*c2), -s2, 0
					//                 0,   0, 0
					//                 0,   0, 0
					//                 0,   0, 0
					//                 0,   0, 0 ];
					DSDq[0] = -(c1*s2);
					DSDq[1] = -(c1*c2);
					DSDq[6] = c2;
					DSDq[7] = -s2;
					break;
			}

			break;

		case EULER_ZXY:

			switch ( idx ) {
				case 0:
					break;

				case 1:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					// DsDq = [  s1*s2, 0, 0
					//              c1, 0, 0
					//        -(c2*s1), 0, 0
					//               0, 0, 0
					//               0, 0, 0
					//               0, 0, 0 ];
					DSDq[0] = s1*s2;
					DSDq[1] = c1;
					DSDq[2] = -(c2*s1);
					break;

				case 2:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					// DsDq = [ -(c1*c2), -s2, 0
					//                 0,   0, 0
					//          -(c1*s2),  c2, 0
					//                 0,   0, 0
					//                 0,   0, 0
					//                 0,   0, 0 ];
					DSDq[0] = -(c1*c2);
					DSDq[2] = -(c1*s2);
					DSDq[6] = -s2;
					DSDq[8] = c2;
					break;
			}

			break;
	}

	if ( bReversed ) {
		DSDq = -Ad(inv_T, DSDq);
		DSDq -= ad(get_S(idx), S);
	}

	return DSDq;
}

RMatrix GJointSpherical::get_DdSDq(GCoordinate *pCoordinate_)
{
	int idx;
	double c1, c2, s1, s2, dq1, dq2;
	RMatrix DdSDq;

	if ( pCoordinate_ == &coordinates[0] ) {
		idx = 0;
	} else if ( pCoordinate_ == &coordinates[1] ) {
		idx = 1;
	} else if ( pCoordinate_ == &coordinates[2] ) {
		idx = 2;
	} else {
		return Zeros(6,3);
	}

	DdSDq.SetZero(6,3);

	switch ( coord_chart ) {

		case EULER_ZYX:

			switch ( idx ) {
				case 0:
					break;

				case 1:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					dq1 = coordinates[1].dq;
					dq2 = coordinates[2].dq;
					// DdSDq = [               s1*dq1, 0, 0
					//           -c2*s1*dq2-s2*c1*dq1, 0, 0
					//           -c1*c2*dq1+s1*s2*dq2, 0, 0
					//                              0, 0, 0
					//                              0, 0, 0
					//                              0, 0, 0
					//                              0, 0, 0 ];
					DdSDq[0] = s1*dq1;
					DdSDq[1] = -c2*s1*dq2-s2*c1*dq1;
					DdSDq[2] = -c1*c2*dq1+s1*s2*dq2;
					break;

				case 2:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					dq1 = coordinates[1].dq;
					dq2 = coordinates[2].dq;
					// DdSDq = [                    0,       0, 0
					//           -s2*c1*dq2-c2*s1*dq1, -c2*dq2, 0
					//            s1*s2*dq1-c1*c2*dq2,  s2*dq2, 0
					//                              0,       0, 0
					//                              0,       0, 0
					//                              0,       0, 0
					//                              0,       0, 0 ];
					DdSDq[1] = -s2*c1*dq2-c2*s1*dq1;	
					DdSDq[2] = s1*s2*dq1-c1*c2*dq2;
					DdSDq[7] = -c2*dq2;
					DdSDq[8] = s2*dq2;
					break;
			}

			break;

		case EULER_ZYZ:

			switch ( idx ) {
				case 0:
					break;

				case 1:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					dq1 = coordinates[1].dq;
					dq2 = coordinates[2].dq;
					// DdSDq = [  s1*c2*dq1+c1*s2*dq2, 0, 0
					//           -s1*s2*dq1+c1*c2*dq2, 0, 0
					//                        -c1*dq1, 0, 0
					//                              0, 0, 0
					//                              0, 0, 0
					//                              0, 0, 0 ];
					DdSDq[0] = s1*c2*dq1+c1*s2*dq2;
					DdSDq[1] = -s1*s2*dq1+c1*c2*dq2;
					DdSDq[2] = -c1*dq1;
					break;

				case 2:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					dq1 = coordinates[1].dq;
					dq2 = coordinates[2].dq;
					// DdSDq = [ c1*s2*dq1+s1*c2*dq2, -s2*dq2, 0
					//           c1*c2*dq1-s1*s2*dq2, -c2*dq2, 0
					//                             0,       0, 0
					//                             0,       0, 0
					//                             0,       0, 0
					//                             0,       0, 0 ];
					DdSDq[0] = c1*s2*dq1+s1*c2*dq2;
					DdSDq[1] = c1*c2*dq1-s1*s2*dq2;
					DdSDq[6] = -s2*dq2;
					DdSDq[7] = -c2*dq2;
					break;
			}

			break;

		case EULER_XYZ:

			switch ( idx ) {
				case 0:
					break;

				case 1:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					dq1 = coordinates[1].dq;
					dq2 = coordinates[2].dq;
					// DdSDq = [ -(dq1*c1*c2) + dq2*s1*s2, 0, 0
					//              dq2*c2*s1 + dq1*c1*s2, 0, 0
					//                          -(dq1*s1), 0, 0
					//                                  0, 0, 0
					//                                  0, 0, 0
					//                                  0, 0, 0 ];
					DdSDq[0] = -(dq1*c1*c2) + dq2*s1*s2;
					DdSDq[1] = dq2*c2*s1 + dq1*c1*s2;
					DdSDq[2] = -(dq1*s1);
					break;

				case 2:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					dq1 = coordinates[1].dq;
					dq2 = coordinates[2].dq;
					// DdSDq = [ -(dq2*c1*c2) + dq1*s1*s2, -(dq2*s2), 0
					//              dq1*c2*s1 + dq2*c1*s2, -(dq2*c2), 0
					//                                  0,         0, 0
					//                                  0,         0, 0
					//                                  0,         0, 0
					//                                  0,         0, 0 ];
					DdSDq[0] = -(dq2*c1*c2) + dq1*s1*s2;
					DdSDq[1] = dq1*c2*s1 + dq2*c1*s2;
					DdSDq[6] = -(dq2*s2);
					DdSDq[7] = -(dq2*c2);
					break;
			}

			break;

		case EULER_ZXY:

			switch ( idx ) {
				case 0:
					break;

				case 1:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					dq1 = coordinates[1].dq;
					dq2 = coordinates[2].dq;
					// DdSDq = [    dq2*c2*s1 + dq1*c1*s2, 0, 0
					//                          -(dq1*s1), 0, 0
					//           -(dq1*c1*c2) + dq2*s1*s2, 0, 0
					//                                  0, 0, 0
					//                                  0, 0, 0
					//                                  0, 0, 0 ];
					DdSDq[0] = dq2*c2*s1 + dq1*c1*s2;
					DdSDq[1] = -(dq1*s1);
					DdSDq[2] = -(dq1*c1*c2) + dq2*s1*s2;
					break;

				case 2:
					c1 = cos(coordinates[1].q);
					c2 = cos(coordinates[2].q);
					s1 = sin(coordinates[1].q);
					s2 = sin(coordinates[2].q);
					dq1 = coordinates[1].dq;
					dq2 = coordinates[2].dq;
					// DdSDq = [    dq1*c2*s1 + dq2*c1*s2, -(dq2*c2), 0
					//                                  0,         0, 0
					//           -(dq2*c1*c2) + dq1*s1*s2, -(dq2*s2), 0
					//                                  0,         0, 0
					//                                  0,         0, 0
					//                                  0,         0, 0 ];
					DdSDq[0] = dq1*c2*s1 + dq2*c1*s2;
					DdSDq[2] = -(dq2*c1*c2) + dq1*s1*s2;
					DdSDq[6] = -(dq2*c2);
					DdSDq[8] = -(dq2*s2);
					break;
			}

			break;
	}

	if ( bReversed ) {
		RMatrix DSDq = get_DSDq(pCoordinate_);
		DdSDq = -Ad(inv_T, DdSDq);
		DdSDq -= ad(get_S(idx), dS + ad(Sdq, S));
		DdSDq -= ad(DSDq*get_dq(), S);
		DdSDq -= ad(Sdq, DSDq);
	}

	return DdSDq;
}

void GJointSpherical::_validateCoordinateChart()
{
	if ( b_fixed_coord_chart ) return;

	int i;
	RMatrix q(3,1), q_(3,1);					// q: current Euler angles, q_: another Euler angles
	RMatrix dq(3,1), dq_(3,1);					// dq: time derivative of q, dq_: time derivative of q_
	RMatrix ddq(3,1), ddq_(3,1);				// ddq: time derivative of dq, ddq_: time derivative of dq_
	RMatrix s(3,1), c(3,1), s_(3,1), c_(3,1);	// s[i] = sin(q[i]), c[i] = cos(q[i]), s_[i] = sin(q_[i]), c_[i] = cos(q_[i])
	RMatrix J(3,3), J_(3,3), invJ_(3,3);
	RMatrix dJdt(3,3), dJdt_(3,3);

	// find q, dq, ddq
	for (i=0; i<3; i++)
	{
		q[i] = coordinates[i].q;
		dq[i] = coordinates[i].dq;
		ddq[i] = coordinates[i].ddq;
		s[i] = sin(q[i]);
		c[i] = cos(q[i]);
	}
	// set 0 <= q <= 2*PI for convenience 
	for (i=0; i<3; i++)
	{
		q[i] = fmod(q[i], 2.0*PI);
		if ( q[i] < 0.0 ) q[i] += 2.0*PI;
	}

	switch ( coord_chart ) {

		case EULER_ZYX:	// det(J) = -c[1]

			if ( fabs(c[1]) < min_det_J )
			{
				// calculate q_ from q
				q_[0] = atan2(s[0] * s[1] * c[2] - c[0] * s[2], c[0] * s[1] * c[2] + s[0] * s[2]);
				q_[1] = atan2(sqrt(1.0 - c[1] * c[1] * c[2] * c[2]), c[1] * c[2]);
				q_[2] = atan2(c[1] * s[2], s[1]);

				// calculate dq_
				// calculate sin(q_), cos(q_)
				for (i=0; i<3; i++)
				{
					s_[i] = sin(q_[i]);
					c_[i] = cos(q_[i]);
				}
				// calculate J: Jacobian of Euler_ZYX
				J[0] = -s[1];			J[3] = 0.0;			J[6] = 1.0;
				J[1] = s[2]*c[1];		J[4] = c[2];		J[7] = 0.0;
				J[2] = c[1]*c[2];		J[5] = -s[2];		J[8] = 0.0;
				// calculate J_: Jacobian of Euler_ZYZ
				J_[0] = -s_[1]*c_[2];	J_[3] = s_[2];		J_[6] = 0.0;
				J_[1] = s_[1]*s_[2];	J_[4] = c_[2];		J_[7] = 0.0;
				J_[2] = c_[1];			J_[5] = 0.0;		J_[8] = 1.0;
				// calculate dq_ = Inv(J_)*J*dq
				invJ_ = Inv(J_);
				dq_ = invJ_ * J * dq;

				// calculate ddq_
				// calculate dJdt: DotJacobian of Euler_ZYX
				dJdt[0] = -c[1]*dq[1];								dJdt[3] = 0.0;				dJdt[6] = 0.0;
				dJdt[1] = c[2]*c[1]*dq[2] - s[2]*s[1]*dq[1];		dJdt[4] = -s[2]*dq[2];		dJdt[7] = 0.0;
				dJdt[2] = -s[1]*c[2]*dq[1] - c[1]*s[2]*dq[2];		dJdt[5] = -c[2]*dq[2];		dJdt[8] = 0.0;
				// calculate dJdt_: DotJacobian of Euler_ZYZ
				dJdt_[0] = -c_[1]*c_[2]*dq_[1] + s_[1]*s_[2]*dq_[2];	dJdt_[3] = c_[2]*dq_[2];	dJdt_[6] = 0.0;
				dJdt_[1] = c_[1]*s_[2]*dq_[1] + s_[1]*c_[2]*dq_[2];		dJdt_[4] = -s_[2]*dq_[2];	dJdt_[7] = 0.0;
				dJdt_[2] = -s_[1]*dq_[1];								dJdt_[5] = 0.0;				dJdt_[8] = 0.0;
				// calculate ddq_ = Inv(J_)*( -dJdt_*dq_ + dJdt*dq + J*ddq )
				ddq_ = invJ_ * ( -dJdt_ * dq_ + dJdt * dq + J * ddq );

				// change coord_chart
				coord_chart = EULER_ZYZ;
				for (i=0; i<3; i++)
				{
					coordinates[i].q = q_[i];
					coordinates[i].dq = dq_[i];
					coordinates[i].ddq = ddq_[i];
				}
			}
			break;

		case EULER_ZYZ:	// det(J) = -s[1]

			if ( fabs(s[1]) < min_det_J )
			{
				// calculate q_ from q
				q_[0] = atan2(s[0] * c[1] * c[2] + c[0] * s[2], c[0] * c[1] * c[2] - s[0] * s[2]);
				q_[1] = atan2(s[1] * c[2], sqrt(1.0 + c[2] * c[2] * (c[1] * c[1] - 1.0)));
				q_[2] = atan2(s[1] * s[2], c[1]);

				// calculate dq_
				// calculate sin(q_), cos(q_)
				for (i=0; i<3; i++)
				{
					s_[i] = sin(q_[i]);
					c_[i] = cos(q_[i]);
				}
				// calculate J: Jacobian of Euler_ZYZ
				J[0] = -s[1]*c[2];		J[3] = s[2];		J[6] = 0.0;
				J[1] = s[1]*s[2];		J[4] = c[2];		J[7] = 0.0;
				J[2] = c[1];			J[5] = 0.0;			J[8] = 1.0;
				// calculate J_: Jacobian of Euler_ZYX
				J_[0] = -s_[1];			J_[3] = 0.0;		J_[6] = 1.0;
				J_[1] = s_[2]*c_[1];	J_[4] = c_[2];		J_[7] = 0.0;
				J_[2] = c_[1]*c_[2];	J_[5] = -s_[2];		J_[8] = 0.0;
				// calculate dq_ = Inv(J_)*J*dq
				invJ_ = Inv(J_);
				dq_ = invJ_*J*dq;

				// calculate ddq_
				// calculate dJdt: DotJacobian of Euler_ZYZ
				dJdt[0] = -c[1]*c[2]*dq[1] + s[1]*s[2]*dq[2];	dJdt[3] = c[2]*dq[2];		dJdt[6] = 0.0;
				dJdt[1] = c[1]*s[2]*dq[1] + s[1]*c[2]*dq[2];	dJdt[4] = -s[2]*dq[2];		dJdt[7] = 0.0;
				dJdt[2] = -s[1]*dq[1];							dJdt[5] = 0.0;				dJdt[8] = 0.0;
				// calculate dJdt_: DotJacobian of Euler_ZYX
				dJdt_[0] = -c_[1]*dq_[1];								dJdt_[3] = 0.0;				dJdt_[6] = 0.0;
				dJdt_[1] = c_[2]*c_[1]*dq_[2] - s_[2]*s_[1]*dq_[1];		dJdt_[4] = -s_[2]*dq_[2];	dJdt_[7] = 0.0;
				dJdt_[2] = -s_[1]*c_[2]*dq_[1] - c_[1]*s_[2]*dq_[2];	dJdt_[5] = -c_[2]*dq_[2];	dJdt_[8] = 0.0;
				// calculate ddq_ = Inv(J_)*( -dJdt_*dq_ + dJdt*dq + J*ddq )
				ddq_ = invJ_ * ( -dJdt_ * dq_ + dJdt * dq + J * ddq );

				// change coord_chart
				coord_chart = EULER_ZYX;
				for (i=0; i<3; i++)
				{
					coordinates[i].q = q_[i];
					coordinates[i].dq = dq_[i];
					coordinates[i].ddq = ddq_[i];
				}
			}
			break;
	}
}

void GJointSpherical::_update_short_for_reversed_joint()
{
	SE3 T_tmp = T;
	T = inv_T;
	inv_T = T_tmp;

	//S = -Ad(inv_T, S);

	SO3 iR = inv_T.GetRotation();
	Vec3 si;
	for (int i=0; i<3; i++) {
		si = iR * -Vec3(S(0,i),S(1,i),S(2,i));
		S(0,i) = si[0]; S(1,i) = si[1]; S(2,i) = si[2];
	}
}

void GJointSpherical::_update_for_reversed_joint()
{
	SE3 T_tmp = T;
	T = inv_T;
	inv_T = T_tmp;

	//Sdq = -Ad(inv_T, Sdq);
	//dSdq = -Ad(inv_T, dSdq);
	//Sddq = -Ad(inv_T, Sddq);
	//DSdqDt = Sddq + dSdq;
	//S = -Ad(inv_T, S);
	//dS = -Ad(inv_T, dS) - ad(Sdq, S);

	SO3 iR = inv_T.GetRotation();	// rotation matrix of the new inv_T
	Vec3 s, ds;

	Sdq = se3(iR * -Sdq.GetW(), Vec3(0,0,0));
	dSdq = se3(iR * -dSdq.GetW(), Vec3(0,0,0));
	Sddq = se3(iR * -Sddq.GetW(), Vec3(0,0,0));
	DSdqDt = Sddq + dSdq;

	for (int i=0; i<3; i++) {
		s = iR * -Vec3(S(0,i),S(1,i),S(2,i));
		ds = iR * -Vec3(dS(0,i),dS(1,i),dS(2,i)) - Cross(Sdq.GetW(), s);	// Sdq, s are the new ones, not the old ones!
		S(0,i) = s[0]; S(1,i) = s[1]; S(2,i) = s[2];
		dS(0,i) = ds[0]; dS(1,i) = ds[1]; dS(2,i) = ds[2];
	}
}

void GJointSpherical::setMotion(const SO3 &R_, const RMatrix &dot_R_, const RMatrix &ddot_R_)
{
	RMatrix R(3,3,R_.GetArray());
	RMatrix Rt = ~R;	// transpose of R
	RMatrix W = Rt * dot_R_;
	Vec3 w(0.5*(W[5]-W[7]), 0.5*(W[6]-W[2]), 0.5*(W[1]-W[3]));	// unskew(W)
	RMatrix dW = Rt * ddot_R_ - W * W;
	Vec3 dw(0.5*(dW[5]-dW[7]), 0.5*(dW[6]-dW[2]), 0.5*(dW[1]-dW[3]));	// unskew(dW)

	setMotion(R_, w, dw);
}

void GJointSpherical::setMotion(const SO3 &R_, const Vec3 &w_, const Vec3 &dot_w_)
{
	double q0, q1, q2, dq0, dq1, dq2, c0, c1, c2, s0, s1, s2;

	// calculate q
	Vec3 qv;
	switch ( coord_chart ) {
		case EULER_ZYX: qv = iEulerZYX(R_); break;
		case EULER_ZYZ: qv = iEulerZYZ(R_); break;
		case EULER_XYZ: qv = iEulerXYZ(R_); break;
		case EULER_ZXY: qv = iEulerZXY(R_); break;
	}
	q0 = qv[0]; q1 = qv[1]; q2 = qv[2];
	c0 = cos(q0); c1 = cos(q1); c2 = cos(q2);
	s0 = sin(q0); s1 = sin(q1); s2 = sin(q2);

	// calculate Jacobian
	RMatrix J(3,3);
	double detJ;
	switch ( coord_chart ) {
		case EULER_ZYX:
			J[0] = -s1;		J[3] = 0;		J[6] = 1;
			J[1] = s2*c1;	J[4] = c2;		J[7] = 0;
			J[2] = c1*c2;	J[5] = -s2;		J[8] = 0;
			detJ = -c1;
			break;
		case EULER_ZYZ:
			J[0] = -s1*c2;	J[3] = s2;		J[6] = 0;
			J[1] = s1*s2;	J[4] = c2;		J[7] = 0;
			J[2] = c1;		J[5] = 0;		J[8] = 1;
			detJ = -s1;
			break;
		case EULER_XYZ:
			J[0] = c1*c2;	J[3] = s2;		J[6] = 0;
			J[1] = -c1*s2;	J[4] = c2;		J[7] = 0;
			J[2] = s1;		J[5] = 0;		J[8] = 1;
			detJ = c1;
			break;
		case EULER_ZXY:
			J[0] = -c1*s2;	J[3] = c2;		J[6] = 0;
			J[1] = s1;		J[4] = 0;		J[7] = 1;
			J[2] = c1*c2;	J[5] = s2;		J[8] = 0;
			detJ = c1;
			break;
	}

	// calculate inverse of Jacobian
	RMatrix invJ;
	if ( b_fixed_coord_chart ) {
		invJ = srInv(J, alpha_srInv);	// singularity-robust inverse
	} else {
		// if singular, change the coordinate chart
		if ( fabs(detJ) < min_det_J ) {
			if ( coord_chart == EULER_ZYX || coord_chart == EULER_XYZ || coord_chart == EULER_ZXY ) {
				coord_chart = EULER_ZYZ;
				qv = iEulerZYZ(R_);
				q0 = qv[0]; q1 = qv[1]; q2 = qv[2];
				c0 = cos(q0); c1 = cos(q1); c2 = cos(q2);
				s0 = sin(q0); s1 = sin(q1); s2 = sin(q2);
				J[0] = -s1*c2;	J[3] = s2;		J[6] = 0;
				J[1] = s1*s2;	J[4] = c2;		J[7] = 0;
				J[2] = c1;		J[5] = 0;		J[8] = 1;
				detJ = -s1;
			} else if ( coord_chart == EULER_ZYZ ) {
				coord_chart = EULER_ZYX;
				qv = iEulerZYX(R_);
				q0 = qv[0]; q1 = qv[1]; q2 = qv[2];
				c0 = cos(q0); c1 = cos(q1); c2 = cos(q2);
				s0 = sin(q0); s1 = sin(q1); s2 = sin(q2);
				J[0] = -s1;		J[3] = 0;		J[6] = 1;
				J[1] = s2*c1;	J[4] = c2;		J[7] = 0;
				J[2] = c1*c2;	J[5] = -s2;		J[8] = 0;
				detJ = -c1;
			} else {
				;
			}
		}
		invJ = Inv(J);
	}

	// calculate dq
	RMatrix dqm = invJ * RMatrix(3,1,w_.GetArray());
	dq0 = dqm[0]; dq1 = dqm[1]; dq2 = dqm[2];

	// calculate dotJ_dq = dotJ * dq
	RMatrix dotJ_dq(3,1);
	switch ( coord_chart ) {
		case EULER_ZYX:
			dotJ_dq[0] = -c1*dq1*dq0;
			dotJ_dq[1] = (c2*c1*dq2 - s2*s1*dq1)*dq0 - s2*dq2*dq1;
			dotJ_dq[2] = (-s1*c2*dq1 - c1*s2*dq2)*dq0 - c2*dq2*dq1;
			break;
		case EULER_ZYZ:
			dotJ_dq[0] = (-c1*c2*dq1 + s1*s2*dq2)*dq0 + c2*dq2*dq1;
			dotJ_dq[1] = (c1*s2*dq1 + s1*c2*dq2)*dq0 - s2*dq2*dq1;
			dotJ_dq[2] = -s1*dq1*dq0;
			break;
		case EULER_XYZ:
			dotJ_dq[0] = dq1*dq2*c2 - dq0*(dq1*c2*s1 + dq2*c1*s2);
			dotJ_dq[1] = -(dq1*dq2*s2) + dq0*(-(dq2*c1*c2) + dq1*s1*s2);
			dotJ_dq[2] = dq0*dq1*c1;
			break;
		case EULER_ZXY:
			dotJ_dq[0] = -(dq1*dq2*s2) + dq0*(-(dq2*c1*c2) + dq1*s1*s2);
			dotJ_dq[1] = dq0*dq1*c1;
			dotJ_dq[2] = dq1*dq2*c2 - dq0*(dq1*c2*s1 + dq2*c1*s2);
			break;
	}

	// calculate ddq
	RMatrix ddqm = invJ * (RMatrix(3,1,dot_w_.GetArray()) - dotJ_dq);

	// apply q, dq, ddq to coordinates[]->(q,dq,ddq)
	coordinates[0].q = q0; coordinates[1].q = q1; coordinates[2].q = q2;
	coordinates[0].dq = dq0; coordinates[1].dq = dq1; coordinates[2].dq = dq2;
	coordinates[0].ddq = ddqm[0]; coordinates[1].ddq = ddqm[1]; coordinates[2].ddq = ddqm[2];
}

