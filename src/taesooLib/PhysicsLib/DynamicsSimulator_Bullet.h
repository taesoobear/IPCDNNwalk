// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * The University of Tokyo
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
#ifndef NEW_DYNAMICSSIMULATOR_UT_HEADER
#define NEW_DYNAMICSSIMULATOR_UT_HEADER

/** @file DynamicsSimulator_UT/server/DynamicsSimulator_impl.h
 *
 */

#include "OpenHRPcommon.h"
#include "CollisionDetector.h"
#include "DynamicsSimulator.h"
//#include "world.h"
namespace OpenHRP {


	class World_UT;
	/**
	 * DynamicsSimulator_UT class
	 */
	class DynamicsSimulator_UT : public DynamicsSimulator
	{
		OpenHRP::World_UT* world;

		bool isFirstSimulationLoop;

		CollisionSequence* collisions_frame0;
	public:

		DynamicsSimulator_UT(const char* coldettype="SIMPLE");

		~DynamicsSimulator_UT();

		OpenHRP::World_UT* getWorld() { return world;}
		virtual void registerCharacter(
				const char *name,
				CharacterInfo const& cinfo);


		void _registerCharacter(
				const char *name,
				CharacterInfo const& cinfo);
		enum JointDriveMode {
			HIGH_GAIN_MODE,
			TORQUE_MODE
		};

		virtual void init(
				double timeStep,
				OpenHRP::DynamicsSimulator_UT::IntegrateMethod integrateOpt);

		virtual double currentTime();
		
		virtual void getWorldVelocity(int ichara,VRMLTransform* b
			, vector3 const& localpos
			, vector3& velocity) const;

		virtual void getWorldAcceleration(int ichara,VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& acc) const;

		virtual void getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const;
		
		virtual void registerCollisionCheckPair(
				const char* char1, 
				const char* name1, 
				const char* char2,
				const char* name2,
				vectorn const& param);

		virtual void initSimulation();

		virtual bool stepSimulation();


		virtual void setGVector(const vector3& wdata);

		
		virtual void setCharacterAllJointModes(
				const char* characterName, 
				OpenHRP::DynamicsSimulator_UT::JointDriveMode jointMode);

		virtual void calcCharacterJacobian(
				const char* characterName,
				const char* baseLink,
				const char* targetLink,
				vectorn& jacobian);

		virtual void calcJacobian(int ichar, int ibone, matrixn& jacobian);
		virtual void calcDotJacobian(int ichar, int ibone, matrixn& dotjacobian);
		void calcCOMjacobian(int ichar, matrixn& jacobian);
		// output is compatible to MotionDOF class.
		virtual void getLinkData(int ichara, LinkDataType t, vectorn& out);
		virtual void setLinkData(int ichara, LinkDataType t, vectorn const& in);
		
	};

}

#endif
