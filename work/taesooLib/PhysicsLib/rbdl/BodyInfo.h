
#include <rbdl/rbdl.h>
#include "../TRL/eigenSupport.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#define TEST_CONTACT_CACHE
namespace Trbdl {
	// for LCP solver
	struct LCPdata
	{
		bool visited;
#ifdef TEST_CONTACT_CACHE
		bool constrained;
		int numConstraints;
		int lastGlobalIndex;
#endif
		unsigned int lambda;
		unsigned int ndof;
		unsigned int q_index;
		Math::SpatialVector Ia_c;
		Math::SpatialVector pA0;
		Eigen::Matrix<double, 6, Eigen::Dynamic> U_Dinv;
	};
}
#ifdef EIGEN_CORE_H
// std::vectors containing any objects that have Eigen matrices or vectors
// as members need to have a special allocater. This can be achieved with
// the following macro.
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Trbdl::LCPdata)
#endif
namespace Trbdl {
	inline Quaternion toRBDL(quater const& q)
	{
		return Quaternion(q.x, q.y, q.z, q.w);
	}
	inline quater toBase(Quaternion const& q)
	{
		// quater w x y z  == Quaternion x y z w
		return quater(q[3], q[0], q[1], q[2]);
	}
	// Vector3d == eigenView(vector3)  or trlView(Vector3d)==vector3 (no copying required)
	inline Vector3d calcBaseToBodyCoordinates ( Model &model, unsigned int body_id, const Vector3d &point_base_coordinates) {
		// E corresponds to R.transpose()
		return model.X_base[body_id].E * (point_base_coordinates -model.X_base[body_id].r);
	}
	inline Vector3d calcPointVelocity ( Model &model, unsigned int body_id, const Vector3d &body_point)
	{
		SpatialVector point_spatial_velocity =
			SpatialTransform ( model.X_base[body_id].E.transpose(),
					body_point).apply(model.v[body_id]);

		return point_spatial_velocity.segment<3>(3);
	}
	inline Vector3d calcPointAcceleration ( Model &model, unsigned int reference_body_id, const Vector3d &body_point) 
	{
		// transformation at the contact point.
		SpatialTransform p_X_i ( model.X_base[reference_body_id].E.transpose(), body_point);
		SpatialVector p_v_i = p_X_i.apply(model.v[reference_body_id]);
		Vector3d a_dash = p_v_i.segment<3>(0).cross(p_v_i.segment<3>(3));
		SpatialVector p_a_i = p_X_i.apply(model.a[reference_body_id]);

		return p_a_i.segment<3>(3)+a_dash.segment<3>(0);
	}
	struct Link {
		Joint joint;
		Body body;
		int bodyId;// last body corresponding to a Bone
		int jointId;// spherical joint id for ball joints. the first joint for non-spherical multi-dof joints.
	};
	struct BodyInfo {
		Model model;

		VectorNd kps, kds;

		VectorNd Q;
		VectorNd QDot;
		VectorNd QDDot;
		VectorNd Tau;
		std::vector<Link> links;
		std::vector<SpatialVector> f_ext;
		void initialize();
		void clearExternalForces();
		void integrateQuater(int Qindex, int jointId, double _timeStep);
		inline void _getGlobalFrame(int bi, transf & out)
		{
			Quaternion q=Quaternion::fromMatrix(model.X_base[bi].E);
			out.rotation=toBase(q);
			out.translation=toBase(model.X_base[bi].r);
		}
		inline transf globalFrame(int bodyId) { transf out; _getGlobalFrame(bodyId, out); return out; }
		
		// only for free root joint.
		Joint joint0;
		Body body0;
		int body0Id;
		int joint0Id;
		Math::SpatialVector v0; // velocity of the root body (==0 usually).
		Math::SpatialVector a0; // acceleration of the root body (==0 usually).

		BodyInfo() { v0.setZero(); a0.setZero(); }
		std::vector<LCPdata> model_data;
		mutable bool lastSimulatedPoseCached;
		double _timestep;
	};
}
