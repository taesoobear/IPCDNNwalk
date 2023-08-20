// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hayang University.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 *
 * \author : Taesoo Kwon
 */
#ifndef NEW_DYNAMICSSIMULATOR_Trbdl_penalty_HEADER
#define NEW_DYNAMICSSIMULATOR_Trbdl_penalty_HEADER

/** @file DynamicsSimulator/server/DynamicsSimulator_Trbdl_penalty.h
 *
 */


#include "../CollisionDetector.h"
#include "../DynamicsSimulator_penaltyMethod.h"
#include "../../BaseLib/motion/VRMLloader.h"
#include "../../BaseLib/motion/Liegroup.h"


#include <rbdl/rbdl.h>
#include "../OpenHRPcommon.h"
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
	/**
	 * DynamicsSimulator_ class
	 */
	class DynamicsSimulator_Trbdl_penalty : public OpenHRP::DynamicsSimulator_penaltyMethod
	{
	public: // almost private
        std::vector<BodyInfo*> bodyInfoArray;

		double _timestep;
		double _currentTime;
		vector3 _gravity;

		void forwardKinamaticsAcc();
	public:

		DynamicsSimulator_Trbdl_penalty(const char* coldet);
		~DynamicsSimulator_Trbdl_penalty();

		virtual void getWorldVelocity(int ichara, VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& velocity) const;

		virtual void getWorldAcceleration(int ichara,VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& acc) const;

		virtual void getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const;

		// body force
		virtual void addForceToBone(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force) override final;
		virtual void addGlobalForceToBone(int ichara, int treeindex, ::vector3 const& globalpos, ::vector3 const& globalforce) override final;

		// torque around the world origin
		void addWorldTorqueToBone(int ichara, VRMLTransform* b, ::vector3 const& world_torque);

		virtual void _registerCharacter(const char *name, OpenHRP::CharacterInfo const& cinfo);

		virtual void init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);

		double currentTime() const;
		void setCurrentTime(double t);

		// _* functions uses rbdl native format
		// rbdl QDot: [v (world), w (body), dq]
		//      Tau : [fext (world), tauext (body), u]
		
		/* dw, dv, tauext, fext are w.r.t body coordinate unlike TRL_penalty or GMBS_penalty
 		  |       |   | _dv (world)  |   |    |   | fext (world)    |
		  | out_M | * | dw  (body)   | + | b1 | = | tauext  (body)  |
		  |       |   |ddq           |   |    |   | u               |
		  */
		void _calcMassMatrix(int ichara, matrixn& out, vectorn & b);
		void _calcJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos);
		// dot jacobian computation works only after enabling and calling initSimulation()
		void _enableDotJocobianComputation(int ichara);
		void _calcDotJacobianAt(int ichar, int ibone, matrixn& dotjacobian, vector3 const& localpos);
		virtual void _stepKinematic(int ichar, vectorn const& QDDot);

		inline vectornView _Q(int ichara) { return vecView(bodyInfo(ichara).Q);}
		inline vectornView _QDot(int ichara) { return vecView(bodyInfo(ichara).QDot);}
		inline vector3 _bodyW(int ichara, int treeIndex) const { auto& bi=bodyInfo(ichara); return toVector3(bi.model.v[bi.links[treeIndex].bodyId], 0);}
		inline vector3 _bodyV(int ichara, int treeIndex) const { auto& bi=bodyInfo(ichara); return toVector3(bi.model.v[bi.links[treeIndex].bodyId], 3);}
		

		virtual void setTimestep(double timeStep);
		virtual double getTimestep();

		virtual void initSimulation();
		
		virtual bool stepSimulation();


		virtual void setGVector(const ::vector3& wdata);
		virtual void getGVector(::vector3& wdata);

		virtual void _updateCharacterPose(); 

		inline Trbdl::BodyInfo& bodyInfo(int ichara) { return *bodyInfoArray[ichara];}
		inline const Trbdl::BodyInfo& bodyInfo(int ichara) const { return *bodyInfoArray[ichara];}
		inline Trbdl::Link& getLink(int ichara, int ibone) { return bodyInfoArray[ichara]->links[ibone];}

		// output is compatible to MotionDOF class.
		virtual void getLinkData(int ichara, LinkDataType t, vectorn& out);
		virtual void setLinkData(int ichara, LinkDataType t, vectorn const& in);

		virtual const vectorn & getLastSimulatedPose(int ichara=0) const { 
			auto& bi=bodyInfo(ichara);
			if (!bi.lastSimulatedPoseCached)
			{
				((DynamicsSimulator*)(this))->getPoseDOF(ichara, _getLastSimulatedPose(ichara));
				bi.lastSimulatedPoseCached=true;
			}
			return _getLastSimulatedPose(ichara);
		}

		void getBodyVelocity(int chara, VRMLTransform* b, Liegroup::se3& V) const ;
		// if model contains spherical joints, use getSphericalState* and setTau functions.
		/*
		 * our spherical state packing is different!!!
		 *
		 *  in our case, linear parts appear first, and then 3 DOF ball joints (YUP).
		 *         root                                     |     3           3         ...   
		 q_linear= [x, y, z, hinge1, hinge2, hinge3, hinge4]
		 q_quat  =                                          [qw,qx,qy,qz, qw2,qx2,qy2,qz2, qw3,qx3,qy3,qz3, ....]
		 q= [q_linear, q_quat]
		dq=	[dx,dy,dz,dhinge1,dhinge2,dhinge3,dhinge4,        wx,wy,wz,     wx2,wy2,wz2,     wx3,wy3,wz3, ...
		tau is packed in the same way with dq.

		dq : body velocity for all joints
		tau :  only for the root joint, external force/torque are around the world origin 
				all joint torques : self-local
		returns q_linear:size()
		*/
		int getSphericalState(int ichara, vectorn & q, vectorn& dq); // packing is different from setLinkData or setQ/setDQ
		void setSphericalState(int ichara, const vectorn& q, const vectorn& dq); // packing is different from setLinkData or setQ/setDQ
		void setTau(int ichara, const vectorn& tau); // packing is different from setLinkData or setU
		inline int q_quat_startIndex(int ichara){ 
			auto& d=skeleton(ichara).dofInfo; 
			return d.numDOF()-d.numSphericalJoint()*4; 
		}

		// for fixed joints
		void setNonStatePoseDOF(int ichara, vectorn const& pose);
		void setNonStateDQ(int ichara, vectorn const& dq);
		void setNonStateDDQ(int ichara, vectorn const& ddq);
		transf getNonStateRootQ(int ichara);
		

		// SDFAST style packing (x,y,z,qx,qy,qz,theta1,theta2,...,thetaN,qw), which is different from the MotionDOF/PoseDOF format  (x,y,z,qw,qx,qy,qz, theta1, ..., thetaN)
		// note that, for non-root spherical joints, exponential coordinates are used unlike getLinkData and getSphericalState!!!
		virtual void setQ(int ichara, vectorn const& v) override;
		virtual void getQ(int ichara, vectorn & v) const  override;
		//   (wx, wy, wz, vx, vy, vz, dtheta1, dtheta2, ...,dthetaN). w, v is in the global coordinate unlike dpose.
		virtual void setDQ(int ichara, vectorn const& v) override;
		virtual void getDQ(int ichara, vectorn& v) const override;
		virtual void setU(int ichara, vectorn const& v) override;


		void setStablePDparam(int ichara, const vectorn& kp, const vectorn& kd);
		void calculateStablePDForces(int ichara, const vectorn& desired_q, vectorn& tau, bool applyRootExternalForce=false);
	};


}


#endif
