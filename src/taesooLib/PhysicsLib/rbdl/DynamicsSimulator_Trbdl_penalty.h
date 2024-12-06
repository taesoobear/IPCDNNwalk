// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hayang /niversity.
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


#include "../OpenHRPcommon.h"

namespace Trbdl {
	struct Link;

	struct BodyInfo;
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

		// calc*Jacobian* functions uses taesooLib format
		virtual void calcJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos) override;
		virtual void calcDotJacobianAt(int ichar, int ibone, matrixn& DJ, vector3 const& localpos) override;
		void calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian);

		// _calc* functions uses rbdl native format
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

		vectornView _Q(int ichara);
		vectornView _QDot(int ichara);
		vector3 _bodyW(int ichara, int treeIndex) const ;
		vector3 _bodyV(int ichara, int treeIndex) const ;
		

		virtual void setTimestep(double timeStep);
		virtual double getTimestep();

		virtual void initSimulation();
		
		virtual bool stepSimulation();


		virtual void setGVector(const ::vector3& wdata);
		virtual void getGVector(::vector3& wdata);

		virtual void _updateCharacterPose(); 

		Trbdl::BodyInfo& bodyInfo(int ichara) ;
		const Trbdl::BodyInfo& bodyInfo(int ichara) const ;
		Trbdl::Link& getLink(int ichara, int ibone);

		// output is compatible to MotionDOF class.
		virtual void getLinkData(int ichara, LinkDataType t, vectorn& out);
		virtual void setLinkData(int ichara, LinkDataType t, vectorn const& in);

		virtual const vectorn & getLastSimulatedPose(int ichara=0) const ;

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
		

		/// SDFAST style packing (x,y,z,qx,qy,qz,theta1,theta2,...,thetaN,qw), which is different from the MotionDOF/PoseDOF format  (x,y,z,qw,qx,qy,qz, theta1, ..., thetaN)
		/// note that, for non-root spherical joints, exponential coordinates are used unlike getLinkData and getSphericalState!!!
		virtual void setQ(int ichara, vectorn const& v) override;
		virtual void getQ(int ichara, vectorn & v) const  override;
		//   (wx, wy, wz, vx, vy, vz, dtheta1, dtheta2, ...,dthetaN). w, v is in the global coordinate unlike dpose.
		virtual void setDQ(int ichara, vectorn const& v) override;
		virtual void getDQ(int ichara, vectorn& v) const override;
		virtual void setU(int ichara, vectorn const& v) override;

		/// sphericalState format
		void setStablePDparam(int ichara, const vectorn& kp, const vectorn& kd);
		void calculateStablePDForces(int ichara, const vectorn& desired_q, vectorn& tau, bool applyRootExternalForce=false);
		void calculateStablePDForces(int ichara, const vectorn& desired_q, const vectorn& desired_dq, vectorn& tau, bool applyRootExternalForce=false);
		inline void setStablePDparam_dof(int ichara, const vectorn& kp, const vectorn& kd)
		{
			vectorn _kp=poseToSphericalQ(ichara, kp);
			vectorn _kd=dposeToSphericalDQ(ichara, kd);
			setStablePDparam(ichara, _kp, _kd);
		}
		inline void calculateStablePDForces_dof(int ichara, const vectorn& desired_pose, const vectorn& desired_dpose, vectorn& tau, bool applyRootExternalForce=false)
		{
			vectorn desired_q=poseToSphericalQ(ichara, desired_pose);
			vectorn desired_dq=dposeToSphericalDQ(ichara, desired_dpose);
			calculateStablePDForces(ichara, desired_q, desired_dq, tau, applyRootExternalForce);
		}
	};


}


#endif
