#include "physicsLib.h"
#include "rbdlsim.h"

#include <ccd/ccd.h>
#include <ccd/quat.h>
#include <rbdl/Dynamics.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Model.h>
#include <rbdl/rbdl_math.h>

#include <iostream>
#include <vector>

#include "utils.h"
#include "../../TRL/eigenSupport.h"
#include "../../../MainLib/OgreFltk/objectList.h"
#include "../../../BaseLib/utility/QPerformanceTimer.h"

using namespace std;
namespace RE_{
void renderOneFrame(bool check);
}

inline double math_random() {return double(rand())/double(RAND_MAX);}
namespace RBDLSim {

void simplesim() {
	ObjectList dbg;
  World world;
  double restitution = 0.5;
  //SimBody sphere_body = CreateSphereBody(10., 0.1, restitution, Vector3d(0., 5.405, 0.), Vector3d::Zero());
  //world.mBodies.push_back(sphere_body);
  int num_bodies = 10;
  for (int i = 0; i < num_bodies; i++) {
    SimBody body;
     body = CreateSphereBody(
          1.,
          0.1,
          restitution,
          Vector3d(math_random()*0.01, math_random() *5., math_random()*0.01),
          Vector3d::Zero());

    world.mBodies.push_back(body);
  }

  SimShape ground_shape;
  ground_shape.mType = SimShape::Plane;
  ground_shape.pos.set(0., 0., 0.);
  ground_shape.orientation.set(0., 0., 0., 1.);
  ground_shape.scale.set(1.0, 1.0, 1.0);
  ground_shape.restitution = 1.0;

  world.mStaticShapes.push_back(ground_shape);

  world.mSimTime = 0.;
  cout << world.mBodies[0].q.transpose() << endl;

  double dt = 1.0/360.0;
  do {
	  RE_::renderOneFrame(true);
	  int niter=1.0/30.0/dt;
	  QPerformanceTimerCount2 timer;
	  QPerformanceTimerCount2 timer2;
	  timer.start();
	  for(int i=0; i<niter; i++)
	  {
		  world.calcUnconstrainedVelUpdate(dt);
		  timer2.start();
		  world.updateCollisionShapes();
		  world.detectCollisions();
		  world.resolveCollisions(dt, 20);
		  world.integrateWorld(dt);
		  timer2.pause();
	  }
	  RE::output("time", "%d %ld %ld", niter, timer.stop(), timer2.stop());

	// draw
	for (int i = 0; i < world.mBodies.size(); i++) {
		const SimBody& body = world.mBodies[i];
		for (int j = 0; j < body.mCollisionShapes.size(); j++) {
			const SimBody::BodyCollisionInfo& cinfo = body.mCollisionShapes[j];
			switch (cinfo.second.mType) {
				case SimShape::Box:
					break;
				case SimShape::Sphere:
					break;
				default:
					break;
			}
			transf tf;

			tf.translation=vector3(cinfo.second.pos[0], cinfo.second.pos[1], cinfo.second.pos[2]);
			RMatrix33 mat=cinfo.second.orientation.toMatrix();
			tf.rotation.setRotation(trlView(mat));
			tf.rotation=tf.rotation.inverse();

			TString id;
			id.format("body%d_%d", i, j);
			dbg.drawAxes(tf, id.ptr(), 2.0, 100.0);
			dbg.drawSphere(tf.translation*100, (id+"sphere").ptr(), "lightgrey_transparent", 0.1*50);
		}
	}


  } while (world.mSimTime < 100.01);
}

}  // namespace RBDLSim
