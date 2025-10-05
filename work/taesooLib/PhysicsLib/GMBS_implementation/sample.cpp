#include <fstream>
#include <list>
#include "gbody_rigid.h"
#include "gjoint_revolute.h"
#include "gsystem.h"
#include "rmatrix3.h"
#include "liegroup.h"

using namespace std;
using namespace gmbs;

void main()
{
	ofstream fout("test.txt");
	GSystem mysystem;
	GBodyRigid ground, link1, link2;
	GJointRevolute joint1, joint2;

	//----------------------- temporary variables -----------------------

	int n = 1;				// dof of the joints, (revolute = 1)
	double *q1, *q2, *dq1, *dq2, *ddq1, *ddq2, *tau1, *tau2;
	q1 = new double[n]; q2 = new double[n];
	dq1 = new double[n]; dq2 = new double[n];
	ddq1 = new double[n]; ddq2 = new double[n];
	tau1 = new double[n]; tau2 = new double[n];
	q1[0] = 0.1; q2[0] = 0.2;
	dq1[0] = 0.1; dq2[0] = 0.2;
	ddq1[0] = 0.0; ddq2[0] = 0.0;
	tau1[0] = 0.0; tau2[0] = 0.0;

	//----------------------- system building -----------------------

	SE3 T_com1, T_com2;		// SE(3): {body} --> {com}
	T_com1 = SE3(SO3(), Vec3(0.5,0,0));	// In this case, orientation of {com} frame w.r.t {body} is identity.
	T_com2 = SE3(SO3(), Vec3(1.5,0,0));
	double mass1, mass2;
	mass1 = mass2 = 1;
	double ixx1, iyy1, izz1, ixy1, ixz1, iyz1;
	double ixx2, iyy2, izz2, ixy2, ixz2, iyz2;
	ixx1 = iyy1 = izz1 = ixy1 = ixz1 = iyz1 = 0;
	ixx2 = iyy2 = izz2 = ixy2 = ixz2 = iyz2 = 0;

	// mass properties
	link1.setMassProperty(T_com1, mass1, ixx1, iyy1, izz1, ixy1, ixz1, iyz1);
	link2.setMassProperty(T_com2, mass2, ixx2, iyy2, izz2, ixy2, ixz2, iyz2);

	// joints connect bodies(or links)
	joint1.connectBodies(&ground, &link1);
	joint2.connectBodies(&link1, &link2);

	// locations of the joints relative to the connected bodies
	joint1.setPosition(Vec3(0,0,0), Vec3(0,0,0));
	joint2.setPosition(Vec3(1,0,0), Vec3(1,0,0));

	// rotational axis of the revolute joints
	joint1.setAxis(0,0,1);
	joint2.setAxis(0,0,1);

	// system buliding
	mysystem.buildSystem(&ground);	// scan the system structure from ground
	mysystem.setGravity(Vec3(0,0,-9.81));

	//----------------------- forward kinematics -----------------------

	joint1.set_q(q1); joint2.set_q(q2);		// set joint displacement
	joint1.set_dq(q1); joint2.set_dq(q2);	// set joint velocity

	mysystem.updateKinematics();		// solve kinematics

	fout << link1.T_global << endl;		// print SE(3): {ground} --> {link1}
	fout << link2.T_global << endl; 	// print SE(3): {ground} --> {link2}
	fout << link1.T_global * link1.T_com << endl; // print SE(3): {ground} --> {com of link1}
	fout << (link1.T_global * link1.T_com).GetPosition() << endl;  // print global position of c.o.m. of link1
	fout << (link1.T_global * link1.T_com).GetRotation() << endl; // print orientation of {com of link1} w.r.t. {global}
	fout << GetV(link1.V) << endl; 	// print velocity of link1 at the origin of {link1} viewed in {link1}
	fout << GetW(link1.V) << endl; 	// print angular velocity of the body frame {link1} viewed in {link1}
	fout << link1.T_global.GetRotation() * GetV(link1.V) << endl; // print velocity of link1 at the origin of {link1} viewed in {ground}
	fout << link1.T_global.GetRotation() * GetW(link1.V) << endl; // print anglar velocity of {link1} viewed in {ground}

	//----------------------- inverse dynamics -----------------------

	// make sure that the joints whose torques need to be calclated are set to be 'prescribed'.
	joint1.setPrescribed(true);			// 'prescribed' means that the joint trajectory(position,velocity,acceleration) is prescribed by user.
	joint2.setPrescribed(true);

	// set joint trajectory
	joint1.coordinate.q = 0.1;			// you can also set the revolute joint coordinate in this way.
	joint1.coordinate.dq = 0.2;			// but, this is valid for revolute/prismatic joints only.
	joint1.coordinate.ddq = -0.1;		// so, we recommend to use the member functions such as set_q(...).
	joint2.set_q(q2); joint2.set_dq(dq2); joint2.set_ddq(ddq2);

	// calculate (inverse) dynamics
	mysystem.calcDynamics();			// since all joints are 'prescribed', calcDynamics() calculates torques corresponding to the joint trajectories.

	// print torque
	fout << joint1.get_tau() << endl;
	fout << joint2.get_tau() << endl;

	//----------------------- forward dynamics simulation -----------------------

	list<GCoordinate*>::iterator iter_pcoord;
	double h = 0.01;

	// make sure that joints whose accelerations need to be calculated are set to be 'unprescribed'.
	joint1.setPrescribed(false);
	joint2.setPrescribed(false);

	// simulate for 1 sec.
	tic();	// start a stopwatch timer
	for (double t=0; t<1; t+=h) {
		// set joint torque
		joint1.set_tau(Zeros(joint1.getDOF(),1).GetPtr());	// torque = 0 
		joint2.set_tau(Zeros(joint2.getDOF(),1).GetPtr());
		
		// calculate (forward) dynamics 
		mysystem.calcDynamics();	// since all joints are 'unprescribed', calcDynamics() calculates accelerations of the joints.
		
		// integrate with explicit Euler method
		for (iter_pcoord = mysystem.pCoordinates.begin(); iter_pcoord != mysystem.pCoordinates.end(); iter_pcoord++) {
			(*iter_pcoord)->q += h * (*iter_pcoord)->dq;
			(*iter_pcoord)->dq += h * (*iter_pcoord)->ddq;
		}
		
		// update kinematics
		mysystem.updateKinematics();
		
		// print global position of c.o.m of link1
		fout << t << "   " << (link1.T_global * link1.T_com).GetPosition() << endl;
	}
	fout << "elapsed time = " << toc() << " sec" << endl;	// print calculation time for simulation

	delete [] q1;
	delete [] q2;
	delete [] dq1;
	delete [] dq2;
	delete [] ddq1;
	delete [] ddq2;
	delete [] tau1;
	delete [] tau2;
}