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


int main (int argc, char* argv[]) {
	rbdl_check_api_version (RBDL_API_VERSION);
	Joint joint_rot_y = Joint (SpatialVector (0., 1., 0., 0., 0., 0.));
	Joint joint_eulerzyx = Joint (JointTypeEulerZYX);
	Body body;
	unsigned int eulerzyx_body_id, eulerzyx_child_id;
	Model eulerzyx_model;
    eulerzyx_model.gravity = Vector3d (0., 0., -9.81);

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;

	// copied from rbdl/tests/multidof*.cc

	// mass, com, inertia
	body = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));




    eulerzyx_model.AppendBody (Xtrans(Vector3d (0., 0., 0.)), joint_rot_y, body);
    eulerzyx_body_id = eulerzyx_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_eulerzyx, body);


    Q = VectorNd::Zero ((size_t) eulerzyx_model.q_size);
    QDot = VectorNd::Zero ((size_t) eulerzyx_model.qdot_size);
    QDDot = VectorNd::Zero ((size_t) eulerzyx_model.qdot_size);
    Tau = VectorNd::Zero ((size_t) eulerzyx_model.qdot_size);

	Q[0] = 1.1;
	Q[1] = 1.2;
	Q[2] = 1.3;
	Q[4] = 1.4;

	QDot[0] = 0;
	QDot[1] = 0;
	QDot[2] = 0;
	QDot[3] = 0;


	std::cout << Q.transpose() << std::endl;
	std::cout << QDot.transpose() << std::endl;


	BinaryFile t;
	t.openWrite("test.dat");
	t.pack(Q);
	t.pack(QDot);

	double timestep=0.001;

	int niter=10000;
	t.packInt(niter);
	for(int iter=0; iter<niter; iter++){
		ForwardDynamics (eulerzyx_model, Q, QDot, Tau, QDDot);
		std::cout << Q.transpose() <<  ", " << QDot.transpose() << std::endl;


		// update velocity
		QDot+=timestep*QDDot;

		// update configuration
		Q=Q+timestep*QDot;

		t.packInt(iter);
		t.pack(Q);
		t.pack(QDot);


		t.pack(CalcBodyToBaseCoordinates (eulerzyx_model, Q, 0, Vector3d(1,0,0), true));
		t.pack(CalcBodyToBaseCoordinates (eulerzyx_model, Q, 1, Vector3d(1,0,0), true));
		t.pack(CalcBodyToBaseCoordinates (eulerzyx_model, Q, 2, Vector3d(1,0,0), true));

		for (int j=0; j<3; j++)
		{
			Quaternion q;
			q=Quaternion::fromMatrix(eulerzyx_model.X_base[j].E);
			t.pack(q);
			t.pack(eulerzyx_model.X_base[j].r);
		}
	}

	t.close();

	return 0;
}

