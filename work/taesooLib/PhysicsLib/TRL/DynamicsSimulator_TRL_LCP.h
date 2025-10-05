// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hayang University.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 *
 * \author : Taesoo Kwon
 */
#ifndef NEW_DYNAMICSSIMULATOR_TRL_LCP_HEADER
#define NEW_DYNAMICSSIMULATOR_TRL_LCP_HEADER

/** @file DynamicsSimulator/server/DynamicsSimulator_TRL_LCP.h
 *
 */


#include "../CollisionDetector.h"
#include "DynamicsSimulator_TRL_penalty.h"

#include "World.h"
namespace TRL{
	class ContactForceSolver;
}


namespace OpenHRP {

	/**
	 * DynamicsSimulator_ class
	 */
	class DynamicsSimulator_TRL_LCP : public DynamicsSimulator_TRL_penalty
	{
	protected:
		mutable vectorn _f;
		mutable vector3N contactPos;
		TRL::ContactForceSolver* _contactForceSolver;
		double _MA;
		bool _velocityDamping; // turned off by default.
		double m_linearDamping, m_angularDamping;
	public:

		DynamicsSimulator_TRL_LCP(bool useSimpleColdet=true);
		DynamicsSimulator_TRL_LCP(const char* coldet);
		DynamicsSimulator_TRL_LCP(DynamicsSimulator_TRL_LCP const& other); // this copy constructor assumes that useSimpleColdet==false

		~DynamicsSimulator_TRL_LCP();
		void setParam_Epsilon_Kappa(double eps, double kap);
		void setParam_R_B_MA(double r, double b, double ma);
		void setDamping(double linearDamping=0.05, double angularDamping=0.85)
		{
			// turn on velocity damping (only when there are contacts)
			// velocity damping is non-physical, but used in all game physics engines such as bullet3d.
			_velocityDamping=true;
			m_linearDamping=linearDamping;
			m_angularDamping=angularDamping;
		}

		virtual void init(double timeStep,
						  OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);
		virtual void initSimulation();
		virtual bool stepSimulation();
		void stepSimulation_part1();
		void stepKinematic(int ichar, vectorn const& dq);
		void get_contact_jacob(int ichar, matrixn & M, vectorn& v_star, CollisionSequence& collisionSequence);
		void get_contact_pos(int ichar, vector3N& cpos, CollisionSequence& collisionSequence) const;
		virtual void drawLastContactForces(int ichara=0, ::vector3 const& draw_offset=::vector3(0,0,0)) const override final;
		virtual ::vector3 getContactForce(int ichar, int ibone) const override final;
		Liegroup::dse3 getCOMbasedContactForce(int ichar, int ibone) const; // returns (r x f, f)
		void registerCollisionCheckPair ( const char *charName1, const char *linkName1, const char *charName2, const char *linkName2, vectorn const& param);
		virtual void addRelativeConstraint(int ichara, Bone& bone1,::vector3 boneVector1,Bone& bone2,::vector3 boneVector2);
		virtual void removeRelativeConstraint(int ichara, Bone& bone1, Bone& bone2);

		virtual void _registerCharacter(const char *name, CharacterInfo const& cinfo);
	};
}


#endif
