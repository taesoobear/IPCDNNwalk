/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>

#include <rbdl/rbdl.h>
#include "tfile_standalone.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/** Compute the sum of the first terms of the Magnus expansion of \Omega such that
 * q' = q*exp(\Omega) is the quaternion obtained after applying a constant acceleration
 * rotation wD for a duration step, while starting with a velocity w
 */
Eigen::Vector3d magnusExpansion(const Eigen::Vector3d & w, const Eigen::Vector3d & wD, double step)
{
  double step2 = step * step;

  Eigen::Vector3d w1 = w + wD * step;
  Eigen::Vector3d O1 = (w + w1) * step / 2;
  Eigen::Vector3d O2 = w1.cross(w) * step2 / 12;
  Eigen::Vector3d O3 = wD.cross(O2) * step2 / 20;

  return O1 + O2 + O3;
}

void ConvertQAndQDotFromEmulated ( const VectorNd &q_emulated, const VectorNd &qdot_emulated, const Model &multdof3_model, 
		VectorNd *q_spherical, VectorNd *qdot_spherical) {
	for (unsigned int i = 1; i < multdof3_model.mJoints.size(); i++) 
	{
		unsigned int q_index = multdof3_model.mJoints[i].q_index;

		if (multdof3_model.mJoints[i].mJointType == JointTypeSpherical) 
		{
			Quaternion quat = Quaternion::fromZYXAngles ( Vector3d (
						q_emulated[q_index + 0], q_emulated[q_index + 1], q_emulated[q_index + 2]));
			multdof3_model.SetQuaternion (i, quat, (*q_spherical));

			Vector3d omega = angular_velocity_from_angle_rates (
					Vector3d (q_emulated[q_index], q_emulated[q_index + 1], q_emulated[q_index + 2]),
					Vector3d (qdot_emulated[q_index], qdot_emulated[q_index + 1], qdot_emulated[q_index + 2])
					);

			(*qdot_spherical)[q_index] = omega[0];
			(*qdot_spherical)[q_index + 1] = omega[1];
			(*qdot_spherical)[q_index + 2] = omega[2];
		} else {
			(*q_spherical)[q_index] = q_emulated[q_index];
			(*qdot_spherical)[q_index] = qdot_emulated[q_index];
		}
	}
}

int main (int argc, char* argv[]) {
	rbdl_check_api_version (RBDL_API_VERSION);
	Joint joint_rot_y = Joint (SpatialVector (0., 1., 0., 0., 0., 0.));
	Joint joint_spherical= Joint (JointTypeSpherical);
	Joint joint_fixed= Joint (JointTypeFixed);
	Body body;
	unsigned int sph_body_id, sph_child_id;
	Model multdof3_model;
	multdof3_model.gravity = Vector3d (0., 0., -9.81);

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;

	// copied from rbdl/tests/multidof*.cc

	body = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));



	/* test
	multdof3_model.AppendBody (Xtrans(Vector3d (0., 0., 0.)), joint_fixed, body);
	multdof3_model.AppendBody (Xtrans(Vector3d (0., 0., 0.)), joint_rot_y, body);
	sph_body_id = multdof3_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_spherical, body);
	*/
	/* test2
	multdof3_model.AppendBody (Xtrans(Vector3d (0., 0., 0.)), joint_fixed, body);
	sph_body_id = multdof3_model.AppendBody (Xtrans (Vector3d (0., 0., 0.)), joint_spherical, body);
	multdof3_model.AppendBody (Xtrans(Vector3d (1., 0., 0.)), joint_rot_y, body);
	*/
	multdof3_model.AppendBody (Xtrans(Vector3d (0., 0., 0.)), joint_rot_y, body);
	sph_body_id = multdof3_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_spherical, body);


	printf("%d\n", sph_body_id);
	Q = VectorNd::Zero ((size_t) multdof3_model.q_size);
	QDot = VectorNd::Zero ((size_t) multdof3_model.qdot_size);
	QDDot = VectorNd::Zero ((size_t) multdof3_model.qdot_size);
	Tau = VectorNd::Zero ((size_t) multdof3_model.qdot_size);

	VectorNd emuQ = VectorNd::Zero ((size_t) 4);
	VectorNd emuQDot = VectorNd::Zero ((size_t)4);
	emuQ[0] = 1.1;
	emuQ[1] = 1.2;
	emuQ[2] = 1.3;
	emuQ[3] = 1.4;

	emuQDot[0] = 0;
	emuQDot[1] = 0;
	emuQDot[2] = 0;
	emuQDot[3] = 0;

	ConvertQAndQDotFromEmulated (emuQ, emuQDot, multdof3_model, &Q, &QDot);

	std::cout << emuQ.transpose() << std::endl;
	std::cout << emuQDot.transpose() << std::endl;

	std::cout << ">>>" <<std::endl << Q.transpose() << std::endl;
	std::cout << QDot.transpose() << std::endl;


	BinaryFile t;
	t.openWrite("test_quat.dat");
	t.pack(Q);
	t.pack(QDot);

	double timestep=0.001;

	int niter=10000;
	t.packInt(niter);
	for(int iter=0; iter<niter; iter++){
		ForwardDynamics (multdof3_model, Q, QDot, Tau, QDDot);
		std::cout << Q.transpose() <<  ", " << QDot.transpose() << std::endl;


		// update velocity
		QDot[0]+=QDDot[0]*timestep;

		// update configuration
		Q[0]+=QDot[0]*timestep;


		if(0)
		{
			// doesn't work
			auto rel_R=multdof3_model.X_T[sph_body_id].E;
			Vector3d p_ang_acc=rel_R*QDDot.segment<3>(1);
			Vector3d p_ang_vel=rel_R*QDot.segment<3>(1);

			p_ang_vel+=timestep*p_ang_acc;

			Quaternion q_t0=multdof3_model.GetQuaternion(sph_body_id, Q);
			Quaternion q_t1=q_t0.timeStep(p_ang_vel, timestep);

			Q.segment<4>(1)=q_t1;
			QDot.segment<3>(1)=rel_R.transpose()*p_ang_vel;
		}
		else if(0)
		{
			// doesn't work
			Eigen::Quaterniond qi(Q(4), Q(1), Q(2), Q(3));
			Vector3d w=QDot.segment<3>(1);
			Vector3d wD=QDDot.segment<3>(1);

			// See https://cwzx.wordpress.com/2013/12/16/numerical-integration-for-rotational-dynamics/
			// the division by 2 is because we want to compute exp(O/2);
			Eigen::Vector3d O = magnusExpansion(w, wD, timestep) / 2;
			double n = O.norm();
			double s = std::sin(n);
			Eigen::Quaterniond qexp(std::cos(n), s * O.x(), s * O.y(), s * O.z());

			qi *= qexp;
			qi.normalize(); // This step should not be necessary but we keep it for robustness

			QDot.segment<3>(1)+=timestep*wD;
			Q(1)=qi.x();
			Q(2)=qi.y();
			Q(3)=qi.z();
			Q(4)=qi.w();
		}
		else
		{
			Quaternion q_t0=multdof3_model.GetQuaternion(sph_body_id, Q);
			auto rel_R=q_t0.toMatrix();
			Vector3d p_ang_acc=rel_R*QDDot.segment<3>(1);
			Vector3d p_ang_vel=rel_R*QDot.segment<3>(1);

			p_ang_vel+=timestep*p_ang_acc;

			Quaternion q_t1=q_t0.timeStep(p_ang_vel, timestep);

			Q.segment<4>(1)=q_t1;
			QDot.segment<3>(1)=rel_R.transpose()*p_ang_vel;
		}
		t.packInt(iter);
		t.pack(Q);
		t.pack(QDot);

		t.pack(CalcBodyToBaseCoordinates (multdof3_model, Q, 0, Vector3d(0,0,0), true));
		t.pack(CalcBodyToBaseCoordinates (multdof3_model, Q, 1, Vector3d(0,0,0), true));
		t.pack(CalcBodyToBaseCoordinates (multdof3_model, Q, 2, Vector3d(0,0,0), true));

		t.pack(CalcBodyToBaseCoordinates (multdof3_model, Q, 0, Vector3d(1,0,0), true));
		t.pack(CalcBodyToBaseCoordinates (multdof3_model, Q, 1, Vector3d(1,0,0), true));
		t.pack(CalcBodyToBaseCoordinates (multdof3_model, Q, 2, Vector3d(1,0,0), true));


		for (int j=0; j<3; j++)
		{
			Quaternion q;
			q=Quaternion::fromMatrix(multdof3_model.X_base[j].E);
			t.pack(q);
			t.pack(multdof3_model.X_base[j].r);
		}
	}

	t.close();

	return 0;
}

