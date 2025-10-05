// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hayang University.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 *
 * \author : Taesoo Kwon
 */
#ifndef NEW_DYNAMICSSIMULATOR_TRL_penalty_HEADER
#define NEW_DYNAMICSSIMULATOR_TRL_penalty_HEADER

/** @file DynamicsSimulator/server/DynamicsSimulator_TRL_penalty.h
 *
 */


#include "../CollisionDetector.h"
#include "../DynamicsSimulator_penaltyMethod.h"
#include "../../BaseLib/motion/VRMLloader.h"
#include "../../BaseLib/motion/Liegroup.h"

#include "World.h"


namespace OpenHRP {

	/**
	 * DynamicsSimulator_ class
	 */
	class DynamicsSimulator_TRL_penalty : public DynamicsSimulator_penaltyMethod
	{

	public: // almost private
		TRL::WorldBase world;

		void forwardKinamaticsAcc();
		

	public:
		inline TRL::Body& bodyInfo(int ichara) { return *world.body(ichara);}

		DynamicsSimulator_TRL_penalty(bool useSimpleColdet=true);
		DynamicsSimulator_TRL_penalty(const char* coldet);

		~DynamicsSimulator_TRL_penalty();

		virtual void getBodyVelocity(int chara, VRMLTransform* b, Liegroup::se3& V) const override;
		void calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian);

		virtual void getWorldVelocity(int ichara, VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& velocity) const override;

		virtual void getWorldAcceleration(int ichara,VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& acc) const override;

		virtual void getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const override;

		// body force
		void addForceToBone(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force);
		// torque around the world origin
		void addWorldTorqueToBone(int ichara, VRMLTransform* b, ::vector3 const& world_torque);

		virtual void _registerCharacter(const char *name, CharacterInfo const& cinfo) override;


		virtual void init(double timeStep,
						  OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);

		virtual double currentTime() const;
		virtual void setCurrentTime(double t) override;

		/* dw, dv, tauext, fext are w.r.t world coordinate
 		  |       |   | dw   |   |    |   | tauext    |
		  | out_M | * | dv   | + | b1 | = | fext      |
		  |       |   |ddq   |   |    |   | u         |
		  */
		virtual void calcMassMatrix(int ichara, matrixn& out, vectorn & b) override;

		virtual void setTimestep(double timeStep);
		virtual double getTimestep();

		virtual void initSimulation();
		
		virtual bool stepSimulation();


		virtual void setGVector(const ::vector3& wdata);
		virtual void getGVector(::vector3& wdata);

		virtual void _updateCharacterPose(); 

		// local jacobian
		virtual void calcBodyJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos);
		// calculates both j and dj for obtaining body velocity
		virtual void calcDotBodyJacobianAt(int ichar, int ibone, matrixn& jacobian, matrixn& dotjacobian, vector3 const& localpos);

		// global jacobian 
		virtual void calcJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos) override;
		virtual void calcDotJacobianAt(int ichar, int ibone, matrixn& dotjacobian, vector3 const& localpos) override;
		
		// spatial jacobian (when globalpos is vector3(0,0,0))
		void _calcJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& globalpos);
		void _calcDotJacobianAt(int ichar, int ibone, matrixn& dotjacobian, vector3 const& globalpos);

		TRL::Link* getLink(int ichara, int ibone, int idof=0);
		int getDQindex(int ichara, int ibone, int idof=0);
		// calculates a subblock S of Jacobian J (S=J[3:6, si:ei]) that directly corresponds to ibone
		// returns si (dqIndex of the first link in the i-th bone).
		int calcS(int ichara, int ibone, matrixn& S);

		// output is compatible to MotionDOF class.
		virtual void getLinkData(int ichara, LinkDataType t, vectorn& out) override;
		virtual void setLinkData(int ichara, LinkDataType t, vectorn const& in) override;

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
		*/
		void getSphericalState(int ichara, vectorn & q, vectorn& dq); // packing is different from setLinkData or setQ/setDQ
		void setSphericalState(int ichara, const vectorn& q, const vectorn& dq); // packing is different from setLinkData or setQ/setDQ
		void setTau(int ichara, const vectorn& tau); // packing is different from setLinkData or setU

		// conversion
		// input:
		// 		poseDOF		for setLinkData(ichara, DynamicsSimulator::JOINT_VALUE, cf) 
		// 		dposeDOF	for setLinkData(ichara, DynamicsSimulator::JOINT_VELOCITY, cf)  -- body velocity
		// 		cf 			for setLinkData(ichara, DynamicsSimulator::JOINT_FORCE, cf) -- differs depending on simulators
		// output: 
		// 		q for setQ(ichara, U);
		// 		dq for setDQ(ichara, U); -- world velocity, different packing
		// 		U for setU(ichara, U); -- world force, torque (around the world origin)
		void poseToQ(vectorn const& v, vectorn& out);
		void dposeToDQ(quater const& rootOri, vectorn const& v, vectorn& out);
		void torqueToU(const vectorn& v, vectorn& U);

		// different state packing from GMBS sim!
		void inverseDynamics(vectorn const& q, vectorn const& dq, vectorn const& ddq, vectorn& u);

		// inverse conversion.
		inline void QToPose(vectorn const& v, vectorn& out) { int rdof=v.size(); out.setSize(rdof); out.setVec3(0, v.toVector3(0)); out[3]=v[rdof-1]; out.setVec3(4, v.toVector3(3)); out.range(7, rdof)=v.range(6, rdof-1);}

		// SDFAST style packing (x,y,z,qx,qy,qz,theta1,theta2,...,thetaN,qw), which is different from the MotionDOF/PoseDOF format  (x,y,z,qw,qx,qy,qz, theta1, ..., thetaN)
		virtual void setQ(int ichara, vectorn const& v) override { assert(v.size()==rdof(ichara)); _setQ(ichara, &v(0)); }
		virtual void getQ(int ichara, vectorn & v) const  override{ v.setSize(rdof(ichara)); _getQ(ichara, &v(0));}

		virtual void setDQ(int ichara, vectorn const& v) override { assert(v.size()==dof(ichara)); if(v.size()>0) _setDQ(ichara, &v(0)); }
		virtual void getDQ(int ichara, vectorn& v) const override { v.setSize(dof(ichara)); if(v.size()>0) _getDQ(ichara, &v(0));}
		virtual void setU(int ichara, const vectorn& in) override;

		// for fixed joints
		void setNonStatePoseDOF(int ichara, vectorn const& pose);
		void setNonStateDQ(int ichara, vectorn const& dq);
		void setNonStateDDQ(int ichara, vectorn const& ddq);
		transf getNonStateRootQ(int ichara);


		// state = [q, dq]
		inline void setState(int ichara, vectorn const& v) { assert(v.size()==rdof(ichara )+dof(ichara)); _setState(ichara, &v(0)); }
		inline void getState(int ichara, vectorn & v) const { v.setSize(rdof(ichara)+dof(ichara)); _getState(ichara, &v(0));}
		inline void _getState(int ichara, double v[]) const { _getQ(ichara, &v[0]); _getDQ(ichara, &v[rdof()]);}
		inline void _setState(int ichara, double v[]) { _setQ(ichara, &v[0]);  _setDQ(ichara, &v[rdof()]); }
		inline void getU(int ichara, vectorn& v) const { v.setSize(dof(ichara)); _getU(ichara, &v[0]);}
		inline vectorn getU(int ichara) const  {vectorn v; v.setSize(dof(ichara)); _getU(ichara, &v[0]);return v;}
		void setDDQ(int ichara, vectorn const& v);

		// pointer access
		void _getQ(int ichara, double v[]) const; 
		void _setQ(int ichara, const double v[]);
		void _getDQ(int ichara, double v[]) const;
		void _setDQ(int ichara, const double v[]);
		void _getU(int ichara, double v[]) const; 

		// yaw (Z), pitch (Y), roll (X) assuming that up is Z, front is X.
		// e->[x,y,z,rz,ry,rx,theta1,... ,dx,dy,dz,drz,dry,drx,theta1,..]
		void stateToEulerZYX(vectorn const& q, vectorn const& dq, vectorn& eulerState) const;
		
		// yaw (Y), pitch (X), roll (Z) assuming that up is Y, front is Z.
		// e->[x,y,z,ry,rx,rz,theta1,... ,dx,dy,dz,dry,drx,drz,theta1,..]
		void stateToEulerYXZ(vectorn const& q, vectorn const& dq, vectorn& eulerState) const;

		void eulerZYXtoState(vectorn const& eulerState, vectorn& state) const;
		void eulerYXZtoState(vectorn const& eulerState, vectorn& state) const;

		void setStablePDparam(int ichara, const vectorn& kp, const vectorn& kd);
		void calculateStablePDForces(int ichara, const vectorn& desired_q, vectorn& tau, bool applyRootExternalForce=false);
	};


}


#endif
