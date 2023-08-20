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

using namespace std;

namespace RBDLSim {

void SimBody::step(double dt) {
  // Prerequisite: qdot has already been updated by resolveCollisions();
  calcNextPositions(dt, qdot, q);
}

void SimBody::calcNextPositions(
    double dt,
    const VectorNd& in_qdot,
    VectorNd& io_q) {
  for (int i = 1; i < mModel.mJoints.size(); ++i) {
    const Joint& joint = mModel.mJoints[i];
    if (joint.mJointType != JointTypeSpherical) {
      io_q.block(joint.q_index, 0, joint.mDoFCount, 1) +=
          dt * in_qdot.block(joint.q_index, 0, joint.mDoFCount, 1);
      continue;
    }

    // Integrate the Quaternion
    Quaternion q0 = mModel.GetQuaternion(i, io_q);
    Vector3d omega(in_qdot.block(joint.q_index, 0, 3, 1));
    Quaternion qd = q0.omegaToQDot(omega);
    Quaternion q1 = (q0 + qd * dt).normalized();
    assert(!isnan(q1.squaredNorm()));
    mModel.SetQuaternion(i, q1, io_q);
  }
}

/** Support function for box */
void SimShapeSupport(
    const void* _shape,
    const ccd_vec3_t* _dir,
    ccd_vec3_t* v) {
  SimShape* shape = (SimShape*)_shape;

  CCD_VEC3(pos, shape->pos[0], shape->pos[1], shape->pos[2]);
  CCD_QUAT(
      quat,
      shape->orientation[0],
      shape->orientation[1],
      shape->orientation[2],
      shape->orientation[3]);
  ccd_vec3_t dir;
  ccd_quat_t qinv;

  // apply rotation on direction vector
  ccdVec3Copy(&dir, _dir);
  ccdQuatInvert2(&qinv, &quat);
  ccdQuatRotVec(&dir, &qinv);

  // compute support point in specified direction
  if (shape->mType == SimShape::Box) {
    ccdVec3Set(
        v,
        ccdSign(ccdVec3X(&dir)) * shape->scale[0] * CCD_REAL(0.5),
        ccdSign(ccdVec3Y(&dir)) * shape->scale[1] * CCD_REAL(0.5),
        ccdSign(ccdVec3Z(&dir)) * shape->scale[2] * CCD_REAL(0.5));

  } else if (shape->mType == SimShape::Sphere) {
    ccd_real_t len;
    assert((shape->scale[0] - shape->scale[1]) < 1.0e-12);
    assert((shape->scale[2] - shape->scale[1]) < 1.0e-12);

    len = ccdVec3Len2(&dir);
    if (len - CCD_EPS > CCD_ZERO) {
      ccdVec3Copy(v, &dir);
      ccdVec3Scale(v, shape->scale[0] * CCD_REAL(0.5) / CCD_SQRT(len));
    } else {
      ccdVec3Set(v, CCD_ZERO, CCD_ZERO, CCD_ZERO);
    }
  } else {
    cerr << "Unknown shape type: " << shape->mType << endl;
    abort();
  }

  // transform support point according to position and rotation of object
  ccdQuatRotVec(v, &quat);
  ccdVec3Add(v, &pos);
}

static void sSwapCollisionInfoShapeOrder(CollisionInfo& cinfo) {
  cinfo.dir *= -1.;
  Vector3d temp_pos = cinfo.posA;
  cinfo.posA = cinfo.posB;
  cinfo.posB = temp_pos;
}

static void sCalcTangentVectors(const Vector3d &normal, Vector3d* tangent0, Vector3d* tangent1) {
  if (fabs(normal.dot(Vector3d(0., 0., 1.))) < 0.6) {
    *tangent0 = normal.cross(Vector3d(0., 0., 1.));
    *tangent1 = tangent0->cross(normal);
  } else {
    *tangent0 = normal.cross(Vector3d(1., 0., 0.));
    *tangent1 = tangent0->cross(normal);
  }

  assert (tangent0->squaredNorm() > cCollisionEps);
  assert (tangent1->squaredNorm() > cCollisionEps);
}

bool CheckPenetration(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo) {
  cinfo.mShapeA = &shape_a;
  cinfo.mShapeB = &shape_b;
  bool result = false;
  if (shape_a.mType == SimShape::Sphere && shape_b.mType == SimShape::Plane) {
    result = CheckPenetrationSphereVsPlane(shape_a, shape_b, cinfo);
    sCalcTangentVectors(cinfo.dir, &cinfo.tangents[0], &cinfo.tangents[1]);
    return result;
  } else if (
      shape_b.mType == SimShape::Sphere && shape_a.mType == SimShape::Plane) {
    result = CheckPenetrationSphereVsPlane(shape_b, shape_a, cinfo);
    sSwapCollisionInfoShapeOrder(cinfo);
    sCalcTangentVectors(cinfo.dir, &cinfo.tangents[0], &cinfo.tangents[1]);
    return result;
  } else if (
      shape_a.mType == SimShape::Sphere && shape_b.mType == SimShape::Sphere) {
    result = CheckPenetrationSphereVsSphere(shape_a, shape_b, cinfo);
    sCalcTangentVectors(cinfo.dir, &cinfo.tangents[0], &cinfo.tangents[1]);
    return result;
  } else if (
      shape_a.mType == SimShape::Box && shape_b.mType == SimShape::Plane) {
    result = CheckPenetrationBoxVsPlane(shape_a, shape_b, cinfo);
    sCalcTangentVectors(cinfo.dir, &cinfo.tangents[0], &cinfo.tangents[1]);
    return result;
  } else if (
      shape_a.mType == SimShape::Plane && shape_b.mType == SimShape::Box) {
    bool result = CheckPenetrationBoxVsPlane(shape_b, shape_a, cinfo);
    sSwapCollisionInfoShapeOrder(cinfo);
    sCalcTangentVectors(cinfo.dir, &cinfo.tangents[0], &cinfo.tangents[1]);
    return result;
  }

  ccd_t ccd;
  CCD_INIT(&ccd);
  ccd.support1 = SimShapeSupport;
  ccd.support2 = SimShapeSupport;
  ccd.max_iterations = 100;
  ccd.epa_tolerance = 0.0001;

  ccd_real_t depth;
  ccd_vec3_t dir, pos;
  int intersect =
      ccdGJKPenetration(&shape_a, &shape_b, &ccd, &depth, &dir, &pos);

  if (intersect == 0) {
    cinfo.posA.set(pos.v[0], pos.v[1], pos.v[2]);
    cinfo.posB.set(pos.v[0], pos.v[1], pos.v[2]);
    cinfo.dir.set(dir.v[0], dir.v[1], dir.v[2]);
    cinfo.depth = depth;
  }

  sCalcTangentVectors(cinfo.dir, &cinfo.tangents[0], &cinfo.tangents[1]);

  return !intersect;
}

bool CheckPenetrationSphereVsSphere(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo) {
  assert(shape_a.mType == SimShape::Sphere);
  assert(
      shape_a.scale[0] == shape_a.scale[1]
      && shape_a.scale[1] == shape_a.scale[2]);
  assert(shape_b.mType == SimShape::Sphere);
  assert(
      shape_b.scale[0] == shape_b.scale[1]
      && shape_b.scale[1] == shape_b.scale[2]);

  Vector3d diff_pos = shape_b.pos - shape_a.pos;
  double diff_pos_norm = diff_pos.norm();
  double distance = diff_pos_norm - (shape_a.scale[0] + shape_b.scale[0]) * 0.5;

  if (distance < cCollisionEps) {
    cinfo.dir = diff_pos / diff_pos_norm;
    cinfo.posA = shape_a.pos + cinfo.dir * shape_a.scale[0] * 0.5;
    cinfo.posB = shape_b.pos - cinfo.dir * shape_b.scale[0] * 0.5;
    cinfo.depth = fabs(distance);

    return true;
  }

  return false;
}

bool CheckPenetrationSphereVsPlane(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo) {
  assert(shape_a.mType == SimShape::Sphere);
  assert(shape_b.mType == SimShape::Plane);

  Vector3d plane_normal =
      shape_b.orientation.conjugate().rotate(Vector3d(0., 1., 0.));
  Vector3d plane_point = shape_b.pos;
  Vector3d sphere_point_to_plane =
      shape_a.pos - plane_normal * shape_a.scale[0] * 0.5;

  double sphere_center_height =
      plane_normal.transpose() * (sphere_point_to_plane - plane_point);
  if (sphere_center_height < cCollisionEps) {
    cinfo.dir = -plane_normal;
    cinfo.depth = sphere_center_height;

    cinfo.mManifoldPoints[cinfo.mNumManifoldPoints++] =
        sphere_point_to_plane - sphere_center_height * plane_normal;
    cinfo.posA = sphere_point_to_plane;
    cinfo.posB = sphere_point_to_plane - sphere_center_height * plane_normal;

    return 1;
  }

  return 0;
}

void CalcIntersectionLineSegmentPlane(
    const Vector3d& vA,
    const Vector3d& vB,
    const Vector3d& plane_point,
    const Vector3d& plane_normal,
    Vector3d& result) {
  double dA = (vA - plane_point).dot(plane_normal);
  double dB = (vB - plane_point).dot(plane_normal);

  double s = (-dA) / (dB - dA);
  assert(s >= 0);
  assert(s <= 1.);
  result = vA + s * (vB - vA);
}

bool CheckPenetrationBoxVsPlane(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo) {
  assert(shape_a.mType == SimShape::Box);
  assert(shape_b.mType == SimShape::Plane);

  SimShape aabb = shape_a;
  SimShape plane = shape_b;

  aabb.pos.setZero();
  aabb.orientation = Quaternion(0., 0., 0., 1.);

  const Matrix3d shape_a_rot = shape_a.orientation.toMatrix();
  plane.pos = shape_a_rot * (shape_b.pos - shape_a.pos);
  Quaternion rot_rel_box =
      shape_a.orientation.conjugate() * shape_b.orientation;
  plane.orientation = rot_rel_box;

  bool result = CheckPenetrationAABBVsPlane(aabb, plane, cinfo);

  if (isnan(cinfo.posA.squaredNorm())) {
    gLog("NaN error!");
  }

  assert(!isnan(cinfo.dir.squaredNorm()));
  assert(!isnan(cinfo.posA.squaredNorm()));
  assert(!isnan(cinfo.posB.squaredNorm()));

  cinfo.posA = shape_a_rot.transpose() * (cinfo.posA) + shape_a.pos;
  cinfo.posB = shape_a_rot.transpose() * (cinfo.posB) + shape_a.pos;
  cinfo.dir = shape_a_rot.transpose() * (cinfo.dir);

  // also transform all contact manifold points
  for (int i = 0; i < cinfo.mNumManifoldPoints; i++) {
    cinfo.mManifoldPoints[i] =
        shape_a_rot.transpose() * cinfo.mManifoldPoints[i] + shape_a.pos;
  }

  return result;
}

bool CheckPenetrationAABBVsPlane(
    const SimShape& shape_a,
    const SimShape& shape_b,
    CollisionInfo& cinfo) {
  assert(shape_a.mType == SimShape::Box);
  assert(shape_b.mType == SimShape::Plane);
  cinfo.mNumManifoldPoints = 0;

  Vector3d plane_normal =
      shape_b.orientation.conjugate().rotate(Vector3d(0., 1., 0.));
  Vector3d plane_pos = shape_b.pos;
  Vector3d dir_min, dir_max;

  for (int i = 0; i < 3; i++) {
    double sign = plane_normal[i] >= 0 ? -1. : 1.;

    dir_min[i] = sign * shape_a.scale[i] * 0.5;
    dir_max[i] = -sign * shape_a.scale[i] * 0.5;
  }

  double distance = (dir_min - plane_pos).dot(plane_normal);
  if (distance > cCollisionEps) {
    // early out: all points above plane
    return false;
  }

  // Separation direction clear: we would need to push the plane along its
  // negative plane normal for separation.
  cinfo.dir = -plane_normal;

  // If center is below plane, return that
  double center_distance = (Vector3d::Zero() - plane_pos).dot(plane_normal);
  if (center_distance < cCollisionEps) {
    cinfo.posA = Vector3d::Zero();
    cinfo.posB = Vector3d::Zero();
    cinfo.depth = center_distance - cCollisionEps;
    return true;
  }

  const Vector3d& scale = shape_a.scale;
  // clang-format off
  Vector3d vertices[8] = {
      Vector3d( scale[0] * 0.5, -scale[1] * 0.5,  scale[2] * 0.5),
      Vector3d( scale[0] * 0.5, -scale[1] * 0.5, -scale[2] * 0.5),
      Vector3d( scale[0] * 0.5,  scale[1] * 0.5,  scale[2] * 0.5),
      Vector3d( scale[0] * 0.5,  scale[1] * 0.5, -scale[2] * 0.5),
      Vector3d(-scale[0] * 0.5, -scale[1] * 0.5,  scale[2] * 0.5),
      Vector3d(-scale[0] * 0.5, -scale[1] * 0.5, -scale[2] * 0.5),
      Vector3d(-scale[0] * 0.5,  scale[1] * 0.5,  scale[2] * 0.5),
      Vector3d(-scale[0] * 0.5,  scale[1] * 0.5, -scale[2] * 0.5)
  };
  // clang-format on

  // Check whether any vertices are touching the plane.
  double distances[8];
  double max_depth = -cCollisionEps;
  for (int i = 0; i < 8; i++) {
    distances[i] = (vertices[i] - plane_pos).dot(plane_normal);
    if (distances[i] >= 0. && distances[i] < cCollisionEps) {
      cinfo.mManifoldPoints[cinfo.mNumManifoldPoints] = vertices[i];
      cinfo.mNumManifoldPoints++;
    }
    max_depth = distances[i] < max_depth ? distances[i] : max_depth;
  }

  if (cinfo.mNumManifoldPoints == 0) {
    // We have no vertex contacts so we have to compute points on the edges.
    const int num_edges = 12;
    char edge_pairs[num_edges][2] = {
        {0, 1},
        {0, 2},
        {0, 4},
        {1, 3},
        {1, 5},
        {2, 3},
        {2, 6},
        {3, 7},
        {4, 5},
        {4, 6},
        {5, 7},
        {6, 7}};

    for (int i = 0; i < num_edges; i++) {
      const double& d0 = distances[edge_pairs[i][0]];
      const double& d1 = distances[edge_pairs[i][1]];

      if (d0 * d1 < 0) {
        // we have an intersection on this edge, compute the contact point on
        // this edge.
        const Vector3d& v0 = vertices[edge_pairs[i][0]];
        const Vector3d& v1 = vertices[edge_pairs[i][1]];

        double s = (-d0) / (d1 - d0);
        assert(s >= 0);
        assert(s <= 1.);

        s = s < cCollisionEps ? 0. : s;
        s = s > 1.0 - cCollisionEps ? 1.0 : s;

        Vector3d vc = v0 + s * (v1 - v0);
        bool found_duplicate_point = false;
        for (int j = 0; j < cinfo.mNumManifoldPoints; j++) {
          if ((cinfo.mManifoldPoints[j] - vc).squaredNorm() < cCollisionEps) {
            gLog("Removing duplicate point");
            found_duplicate_point = true;
            break;
          }
        }

        if (!found_duplicate_point) {
          cinfo.mManifoldPoints[cinfo.mNumManifoldPoints++] =
              v0 + s * (v1 - v0);
        }

        if (cinfo.mNumManifoldPoints > 4) {
          gLog("Have %d manifold points?!", cinfo.mNumManifoldPoints);
        }
      }
    }
  }

  // Average manifold points to compute the central contact point.
  cinfo.posA.setZero();
  for (int i = 0; i < cinfo.mNumManifoldPoints; i++) {
    cinfo.posA += cinfo.mManifoldPoints[i];
  }
  cinfo.posA = cinfo.posA / (double)cinfo.mNumManifoldPoints;
  cinfo.posB = cinfo.posA;
  cinfo.depth = max_depth;

  return true;
}

void SimBody::updateCollisionShapes() {
  UpdateKinematicsCustom(mModel, &q, nullptr, nullptr);

  for (SimBody::BodyCollisionInfo& body_col_info : mCollisionShapes) {
    const unsigned int& body_id = body_col_info.first;
    SimShape& shape = body_col_info.second;

    shape.pos =
        CalcBodyToBaseCoordinates(mModel, q, body_id, Vector3d::Zero(), false);
    shape.orientation = Quaternion::fromMatrix(
        CalcBodyWorldOrientation(mModel, q, body_id, false));
  }
}

bool SolveGaussSeidelProj(
    MatrixNd& A,
    VectorNd& b,
    VectorNd& x,
    VectorNd& xlo,
    VectorNd& xhi,
    double tol = 1.0e-6,
    int maxiter = 100) {
  int n = A.cols();
  for (int iter = 0; iter < maxiter; iter++) {
    double residual = 0.;
    for (int i = 0; i < n; i++) {
      double sigma = 0.;
      for (int j = 0; j < i; j++) {
        sigma += A(i, j) * x[j];
      }
      for (int j = i + 1; j < n; j++) {
        sigma += A(i, j) * x[j];
      }
      double x_old = x[i];
      x[i] = (b[i] - sigma) / A(i, i);

      // Projection onto admissible values
      if (x[i] < xlo[i]) {
        x[i] = xlo[i];
      }
      if (x[i] > xhi[i]) {
        x[i] = xhi[i];
      }

      double diff = x[i] - x_old;
      residual += diff * diff;
    }

    if (residual < tol) {
      cout << "Numiter: " << iter << endl;
      return true;
    }

    if (iter > maxiter) {
      break;
    }
  }

  return false;
}

void CalcCollisions(
    SimBody& body_a,
    SimBody& body_b,
    std::vector<CollisionInfo>& collisions) {
  for (int i = 0, ni = body_a.mCollisionShapes.size(); i < ni; ++i) {
    const SimShape& shape_a = body_a.mCollisionShapes[i].second;

    for (int j = 0, nj = body_b.mCollisionShapes.size(); j < nj; ++j) {
      const SimShape& shape_b = body_b.mCollisionShapes[j].second;

      CollisionInfo cinfo;
      bool has_penetration = false;
      has_penetration = CheckPenetration(shape_a, shape_b, cinfo);
      cinfo.effectiveRestitution = shape_a.restitution * shape_b.restitution;

      if (has_penetration) {
        cinfo.mBodyA = &body_a;
        cinfo.mBodyAIndex = body_a.mCollisionShapes[i].first;
        cinfo.mBodyB = &body_b;
        cinfo.mBodyBIndex = body_b.mCollisionShapes[j].first;
        collisions.push_back(cinfo);
      }
    }
  }
}

void CalcImpulseVariables(
    double dt,
    SimBody* body,
    unsigned int body_index,
    const Vector3d& pos,
    const Vector3d& dir,
    const Vector3d* tangents,
    const double depth,
    VectorNd* MInvJacT,
    VectorNd* jac,
    double* G_MInv_GT,
    VectorNd* tangentJac,
    VectorNd* tangentMInvJacT,
    double* tangentGMInvGT,
    double* bias_vel,
    double restitution) {
  if (body == nullptr || body->mIsStatic) {
    jac->setZero();
    *G_MInv_GT = 0.;
    *bias_vel = 0.;
    return;
  }

  Model* model = &body->mModel;
  int ndof = model->qdot_size;
  const VectorNd& q = body->q;
  const VectorNd& qdot = body->qdot;

  assert(!isnan(q.squaredNorm()));

  // Calculate local coordinates of the contact point
  UpdateKinematicsCustom(*model, &q, nullptr, nullptr);
  Vector3d point_local_b =
      CalcBaseToBodyCoordinates(*model, q, body_index, pos, false);

  // Compute vectors and matrices of the contact system
  MatrixNd M(MatrixNd::Zero(ndof, ndof));
  CompositeRigidBodyAlgorithm(*model, q, M, false);

  MatrixNd G_constr(MatrixNd::Zero(3, ndof));
  CalcPointJacobian(*model, q, body_index, point_local_b, G_constr, false);
  (*jac) = dir.transpose() * G_constr;

  Eigen::LLT<MatrixNd> M_llt (M);
  (*MInvJacT) = M_llt.solve(jac->transpose());
  *G_MInv_GT = (*jac).transpose().dot (*MInvJacT);
  assert(!isnan(*G_MInv_GT));

  double beta = 0.01;
  double delta_slop = 0.05;
  *bias_vel = (*jac).transpose().dot(qdot) * restitution - beta / dt * std::max (0., -depth - delta_slop);

  tangentJac[0] = tangents[0].transpose() * G_constr;
  tangentJac[1] = tangents[1].transpose() * G_constr;
  tangentMInvJacT[0] = M_llt.solve(tangentJac[0].transpose());
  tangentMInvJacT[1] = M_llt.solve(tangentJac[1].transpose());
  tangentGMInvGT[0] = tangentJac[0].transpose().dot( tangentMInvJacT[0]);
  tangentGMInvGT[1] = tangentJac[1].transpose().dot( tangentMInvJacT[1]);

  assert (tangentGMInvGT[0] > 0.);
  assert (tangentGMInvGT[1] > 0.);
}

void PrepareConstraintImpulse(
    double dt,
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo) {
  //ZoneScoped;

  CalcImpulseVariables(
      dt,
      body_a,
      cinfo.mBodyAIndex,
      cinfo.posA,
      cinfo.dir,
      cinfo.tangents,
      cinfo.depth,
      &cinfo.MInvJacTA,
      &cinfo.jacA,
      &cinfo.GMInvGTA,
      cinfo.tangentJacA,
      cinfo.tangentMInvJacTA,
      cinfo.tangentGMInvGTA,
      &cinfo.biasVelocityA,
      cinfo.effectiveRestitution);

  CalcImpulseVariables(
      dt,
      body_b,
      cinfo.mBodyBIndex,
      cinfo.posB,
      cinfo.dir,
      cinfo.tangents,
      -cinfo.depth,
      &cinfo.MInvJacTB,
      &cinfo.jacB,
      &cinfo.GMInvGTB,
      cinfo.tangentJacB,
      cinfo.tangentMInvJacTB,
      cinfo.tangentGMInvGTB,
      &cinfo.biasVelocityB,
      cinfo.effectiveRestitution);
}

/// Calculates the impulse that we apply on body_b to resolve the contact.
void CalcFrictionImpulse(
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo) {
	//ZoneScoped;

  // Todo: add nonlinear effects * dt

  double rhs_tangent[2] = {0., 0.};
  if (body_a && !body_a->mIsStatic) {
    rhs_tangent[0] += cinfo.tangentJacA[0].transpose().dot( body_a->qdot);
    rhs_tangent[1] += cinfo.tangentJacA[1].transpose().dot( body_a->qdot);
  }
  if (body_b && !body_b->mIsStatic) {
    rhs_tangent[0] -= cinfo.tangentJacB[0].transpose().dot( body_b->qdot);
    rhs_tangent[1] -= cinfo.tangentJacB[1].transpose().dot( body_b->qdot);
  }


  for (int i = 0; i < 2; i++) {
    double denom = cinfo.tangentGMInvGTA[i] + cinfo.tangentGMInvGTB[i];
    assert (denom > cCollisionEps);
    double old_impulse = cinfo.accumFrictionImpulse[i];

    cinfo.deltaFrictionImpulse[i] = rhs_tangent[i] / denom;
    cinfo.accumFrictionImpulse[i] = cinfo.accumFrictionImpulse[i] + cinfo.deltaFrictionImpulse[i];
    if (cinfo.accumFrictionImpulse[i] >= cinfo.effectiveFriction * cinfo.accumImpulse) {
      cinfo.accumFrictionImpulse[i] = cinfo.effectiveFriction * cinfo.accumImpulse;
    }
    if (cinfo.accumFrictionImpulse[i] < -cinfo.effectiveFriction * cinfo.accumImpulse) {
      cinfo.accumFrictionImpulse[i] = -cinfo.effectiveFriction * cinfo.accumImpulse;
    }
    cinfo.deltaFrictionImpulse[i] = cinfo.accumFrictionImpulse[i] - old_impulse;

    assert (!isnan(cinfo.deltaFrictionImpulse[i]));
  }
}

void ApplyFrictionImpulse(
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo) {
	//ZoneScoped;

  if (body_a && !body_a->mIsStatic) {
    body_a->qdot +=
        cinfo.tangentMInvJacTA[0] * (-cinfo.deltaFrictionImpulse[0]);
    body_a->qdot +=
        cinfo.tangentMInvJacTA[1] * (-cinfo.deltaFrictionImpulse[1]);

    assert(!isnan(body_a->qdot.squaredNorm()));
  }

  if (body_b && !body_b->mIsStatic) {
    body_b->qdot +=
        -cinfo.tangentMInvJacTB[0] * (-cinfo.deltaFrictionImpulse[0]);
    body_b->qdot +=
        -cinfo.tangentMInvJacTB[1] * (-cinfo.deltaFrictionImpulse[1]);

    assert(!isnan(body_b->qdot.squaredNorm()));
  }
}

/// Calculates the impulse that we apply on body_b to resolve the contact.
void CalcConstraintImpulse(
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo) {
	//ZoneScoped;
  // Todo: add nonlinear effects * dt

  double rhs = 0.;
  if (body_a && !body_a->mIsStatic) {
    rhs += cinfo.jacA .transpose().dot( body_a->qdot )+ cinfo.biasVelocityA;
  }
  if (body_b && !body_b->mIsStatic) {
    rhs += -cinfo.jacB .transpose().dot( body_b->qdot) - cinfo.biasVelocityB;
  }

  double denom = cinfo.GMInvGTA + cinfo.GMInvGTB;
  assert(denom > cCollisionEps);

  double old_impulse = cinfo.accumImpulse;
  // TODO: is this really needed here??
  cinfo.deltaImpulse = rhs / denom;
  cinfo.accumImpulse = std::max(0., cinfo.accumImpulse + cinfo.deltaImpulse);
  cinfo.deltaImpulse = cinfo.accumImpulse - old_impulse;
}

void ApplyConstraintImpulse(
    SimBody* body_a,
    SimBody* body_b,
    CollisionInfo& cinfo) {
	//ZoneScoped;

  if (body_a && !body_a->mIsStatic) {
    body_a->qdot +=
        cinfo.MInvJacTA * (-cinfo.deltaImpulse);
    assert(!isnan(body_a->qdot.squaredNorm()));
  }

  if (body_b && !body_b->mIsStatic) {
    body_b->qdot += cinfo.MInvJacTB * (cinfo.deltaImpulse);
    assert(!isnan(body_b->qdot.squaredNorm()));
  }
}

void World::calcUnconstrainedVelUpdate(double dt) {
	//ZoneScoped;

  for (SimBody& body : mBodies) {
    assert(!isnan(body.q.squaredNorm()));

    ForwardDynamics(body.mModel, body.q, body.qdot, body.tau, body.qddot);

    // semi-implicit eulers
    body.qdot += dt * body.qddot;
  }
}

void World::updateCollisionShapes() {
  //ZoneScoped;

  for (SimBody& body : mBodies) {
    body.updateCollisionShapes();
  }
}

void World::detectCollisions() {
  //ZoneScoped;

  mContactPoints.clear();

  for (int i = 0, n = mBodies.size(); i < n; ++i) {
    SimBody& ref_body = mBodies[i];

    // Check collisions against moving bodies
    for (int j = i + 1; j < n; ++j) {
      SimBody& other_body = mBodies[j];
      CalcCollisions(ref_body, other_body, mContactPoints);
    }

    // Check collisions against static bodies
    for (SimBody::BodyCollisionInfo& body_col_info :
         ref_body.mCollisionShapes) {
      SimShape& ref_body_shape = body_col_info.second;

      // check against all static shapes
      for (SimShape& static_shape : mStaticShapes) {
        bool has_penetration = false;
        CollisionInfo cinfo;

        has_penetration = CheckPenetration(static_shape, ref_body_shape, cinfo);
        cinfo.effectiveRestitution = ref_body_shape.restitution;

        if (has_penetration) {
          if (isnan(cinfo.posA.squaredNorm())) {
            gLog("NaN error!");
          }

          cinfo.mBodyA = nullptr;
          cinfo.mBodyAIndex = -1;
          cinfo.mBodyB = &ref_body;
          cinfo.mBodyBIndex = body_col_info.first;
          assert(!isnan(cinfo.posA.squaredNorm()));
          assert(!isnan(cinfo.posB.squaredNorm()));

          for (int i = 0; i < cinfo.mNumManifoldPoints; i++) {
            CollisionInfo cpinfo(cinfo);
            cinfo.posA = cinfo.mManifoldPoints[i];
            cinfo.posB = cinfo.mManifoldPoints[i];
            mContactPoints.push_back(cinfo);
          }
        }
      }
    }
  }
}

void World::resolveCollisions(double dt, int num_iter) {
  //ZoneScoped;

  for (CollisionInfo& cinfo : mContactPoints) {
    PrepareConstraintImpulse(dt, cinfo.mBodyA, cinfo.mBodyB, cinfo);
  }

  for (int i = 0; i < num_iter; i++) {
    for (CollisionInfo& cinfo : mContactPoints) {
      CalcFrictionImpulse(cinfo.mBodyA, cinfo.mBodyB, cinfo);
      ApplyFrictionImpulse(cinfo.mBodyA, cinfo.mBodyB, cinfo);

      CalcConstraintImpulse(cinfo.mBodyA, cinfo.mBodyB, cinfo);
      ApplyConstraintImpulse(cinfo.mBodyA, cinfo.mBodyB, cinfo);
    }
  }
}

bool World::integrateWorld(double dt) {
	//ZoneScoped;

  for (SimBody& body : mBodies) {
    body.step(dt);
  }

  return true;
}

SimBody CreateSphereBody(
    double mass,
    double radius,
    double restitution,
    const Vector3d& pos,
    const Vector3d& vel) {
  SimBody result;

  Joint joint_trans_xyz(JointTypeTranslationXYZ);
  Joint joint_rot_quat(JointTypeSpherical);
  Matrix3d mat_zero=Matrix3d::Zero();
  Body nullbody(0., Vector3d(0., 0., 0.), mat_zero);

  Matrix3d I=Matrix3d::Identity() * 2. / 5. * mass * radius * radius;
  Body body(
      mass,
      Vector3d(0., 0., 0.),
      I);

  result.mModel.AppendBody(
      Xtrans(Vector3d(0., 0., 0.)),
      joint_trans_xyz,
      nullbody);
  unsigned int sphere_body = result.mModel.AppendBody(
      Xtrans(Vector3d(0., 0., 0.)),
      joint_rot_quat,
      body);

  result.q = VectorNd::Zero(result.mModel.q_size);
  result.q.block(0, 0, 3, 1) = pos;
  result.mModel.SetQuaternion(
      sphere_body,
      Quaternion(0., 0., 0., 1.),
      result.q);
  result.qdot = VectorNd::Zero(result.mModel.qdot_size);
  result.qddot = VectorNd::Zero(result.mModel.qdot_size);
  result.tau = VectorNd::Zero(result.mModel.qdot_size);

  SimShape shape;
  shape.mType = SimShape::Sphere;
  shape.pos = pos;
  shape.orientation.set(0., 0., 0., 1.);
  shape.scale.set(radius, radius, radius);
  shape.restitution = restitution;

  result.mCollisionShapes.push_back(
      SimBody::BodyCollisionInfo(sphere_body, shape));

  return result;
}

SimBody CreateBoxBody(
    double mass,
    const Vector3d& size,
    double restitution,
    const Vector3d& pos,
    const Vector3d& vel) {
  SimBody result;

  Joint joint_trans_xyz(JointTypeTranslationXYZ);
  Joint joint_rot_quat(JointTypeSpherical);
  Matrix3d matZero=Matrix3d::Zero();
  Body nullbody(0., Vector3d(0., 0., 0.), matZero);
  Matrix3d inertia(Matrix3d::Zero());
  inertia(0, 0) = (1. / 12.) * mass * (size[1] * size[1] + size[2] * size[2]);
  inertia(1, 1) = (1. / 12.) * mass * (size[0] * size[0] + size[2] * size[2]);
  inertia(2, 2) = (1. / 12.) * mass * (size[1] * size[1] + size[0] * size[0]);
  Body body(mass, Vector3d(0., 0., 0.), inertia);

  result.mModel.AppendBody(
      Xtrans(Vector3d(0., 0., 0.)),
      joint_trans_xyz,
      nullbody);
  unsigned int sphere_body = result.mModel.AppendBody(
      Xtrans(Vector3d(0., 0., 0.)),
      joint_rot_quat,
      body);

  result.q = VectorNd::Zero(result.mModel.q_size);
  result.q.block(0, 0, 3, 1) = pos;
  result.mModel.SetQuaternion(
      sphere_body,
      Quaternion(0., 0., 0., 1.),
      result.q);
  result.qdot = VectorNd::Zero(result.mModel.qdot_size);
  result.qddot = VectorNd::Zero(result.mModel.qdot_size);
  result.tau = VectorNd::Zero(result.mModel.qdot_size);

  SimShape shape;
  shape.mType = SimShape::Box;
  shape.pos = pos;
  shape.orientation.set(0., 0., 0., 1.);
  shape.scale.set(size[0], size[1], size[2]);
  shape.restitution = restitution;

  result.mCollisionShapes.push_back(
      SimBody::BodyCollisionInfo(sphere_body, shape));

  return result;
}

}  // namespace RBDLSim
