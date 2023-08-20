#ifndef RBDLSIM_RBDLSIM_H
#define RBDLSIM_RBDLSIM_H

#include <rbdl/Model.h>
#include <rbdl/rbdl_math.h>

#include <vector>

namespace RBDLSim {

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct SimShape;
struct SimBody;
struct CollisionInfo;

const double cCollisionEps = 1.0e-5;

struct SimShape {
  enum ShapeType { Box = 0, Sphere = 1, Plane = 2 };
  ShapeType mType;
  Vector3d pos;
  Quaternion orientation;
  Vector3d scale;
  double restitution = 1.0;
};

struct SimBody {
  VectorNd q, qdot, qddot, tau;
  Model mModel;
  bool mIsStatic = false;

  typedef std::pair<unsigned int, SimShape> BodyCollisionInfo;
  std::vector<BodyCollisionInfo> mCollisionShapes;

  void step(double ts);
  void resolveCollisions(double dt, std::vector<CollisionInfo>& collisions);
  void calcNextPositions(double dt, const VectorNd& in_qdot, VectorNd& out_q);
  void updateCollisionShapes();
};

struct CollisionInfo {
  SimBody* mBodyA = nullptr;
  SimBody* mBodyB = nullptr;
  const SimShape* mShapeA = nullptr;
  const SimShape* mShapeB = nullptr;
  int mBodyAIndex;
  int mBodyBIndex;
  Vector3d mManifoldPoints[8];
  int mNumManifoldPoints = 0;
  Vector3d posA = Vector3d::Zero();
  Vector3d posB = Vector3d::Zero();
  double biasVelocityA = 0.;
  double biasVelocityB = 0.;
  double accumImpulse = 0.;
  double deltaImpulse = 0.;
  Vector3d dir = Vector3d(0., 1., 0.);
  VectorNd jacA = VectorNd::Zero(1);
  VectorNd jacB = VectorNd::Zero(1);
  VectorNd MInvJacTA = VectorNd::Zero(1);
  VectorNd MInvJacTB = VectorNd::Zero(1);
  double GMInvGTA = 0.;
  double GMInvGTB = 0.;
  double accumFrictionImpulse[2] = {0.,0.};
  double deltaFrictionImpulse[2] = {0.,0.};
  Vector3d tangents[2] = {Vector3d::Zero(), Vector3d::Zero()};
  VectorNd tangentJacA[2] = {VectorNd::Zero(1), VectorNd::Zero(1)};
  VectorNd tangentJacB[2] = {VectorNd::Zero(1), VectorNd::Zero(1)};
  VectorNd tangentMInvJacTA[2] = {VectorNd::Zero(1), VectorNd::Zero(1)};
  VectorNd tangentMInvJacTB[2] = {VectorNd::Zero(1), VectorNd::Zero(1)};
  double tangentGMInvGTA[2] = {0., 0.};
  double tangentGMInvGTB[2] = {0., 0.};
  double effectiveRestitution = 1.0;
  double effectiveFriction = 0.2;

  double depth = 0.;
};

struct World {
  double mSimTime = 0.;
  std::vector<SimBody> mBodies;
  std::vector<SimShape> mStaticShapes;
  std::vector<CollisionInfo> mContactPoints;

  void calcUnconstrainedVelUpdate(double dt);
  void updateCollisionShapes();
  void detectCollisions();
  void resolveCollisions(double dt, int num_iter);
  bool integrateWorld(double dt);
};

bool CheckPenetration(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo);

bool CheckPenetrationSphereVsSphere(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo);

bool CheckPenetrationSphereVsPlane(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo);

bool CheckPenetrationBoxVsPlane (
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo);

bool CheckPenetrationAABBVsPlane(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo);

SimBody CreateSphereBody(
    double mass,
    double radius,
    double restitution,
    const Vector3d& pos,
    const Vector3d& vel);

SimBody CreateBoxBody(
    double mass,
    const Vector3d& size,
    double restitution,
    const Vector3d& pos,
    const Vector3d& vel);

void PrepareConstraintImpulse(
    double dt,
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo);

void CalcCollisions(
    SimBody& body_a,
    SimBody& body_b,
    std::vector<CollisionInfo>& collisions);

void CalcFrictionImpulse(
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo);

void ApplyFrictionImpulse(
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo);

void CalcConstraintImpulse(
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo);
void ApplyConstraintImpulse(
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo);

}  // namespace RBDLSim

#endif  //RBDLSIM_RBDLSIM_H
