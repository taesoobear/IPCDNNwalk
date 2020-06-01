// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hayang University.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 *
 * \author : Taesoo Kwon
 */
#ifndef NEW_DYNAMICSSIMULATOR_TRL_QP_HEADER
#define NEW_DYNAMICSSIMULATOR_TRL_QP_HEADER

/** @file DynamicsSimulator/server/DynamicsSimulator_TRL_QP.h
 *
 */


#include "../CollisionDetector.h"
#include "DynamicsSimulator_TRL_penalty.h"
#include "../DynamicsSimulator_QP.h"

#include "World.h"
namespace TRL{
	class ContactForceSolver;
}


namespace OpenHRP {

	/**
	 * DynamicsSimulator_ class
	 */
	class DynamicsSimulator_TRL_QP : public DynamicsSimulator_TRL_penalty, public DynamicsSimulator_QP
	{
		vectorn _f;
		vector3N contactPos;
		TRL::ContactForceSolver* _contactForceSolver;
		double _MA;
	public:

		DynamicsSimulator_TRL_QP(const char* coldet);
		DynamicsSimulator_TRL_QP(DynamicsSimulator_TRL_QP const& other); // this copy constructor assumes that useSimpleColdet==false
		~DynamicsSimulator_TRL_QP();
		void setParam_Epsilon_Kappa(double eps, double kap);
		void setParam_R_B_MA(double r, double b, double ma);

		void skipIntegration();
		virtual void init(double timeStep,
						  OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);
		virtual void initSimulation();
		virtual bool stepSimulation();
		void stepSimulation_part1();
		void stepKinematic(int ichar, vectorn const& dq);
		void stepKinematic2(int ichar, vectorn const& ddq);
		void get_contact_jacob(int ichar, matrixn & M, vectorn& v_star, CollisionSequence& collisionSequence);
		void get_contact_pos(int ichar, vector3N& cpos, CollisionSequence& collisionSequence);
		void drawLastContactForces(::vector3 const& draw_offset=::vector3(0,0,0));
		::vector3 getContactForce(int ichar, int ibone) const;
		void registerCollisionCheckPair ( const char *charName1, const char *linkName1, const char *charName2, const char *linkName2, vectorn const& param);
		int getNumAllLinkPairs() const;
	};
}


#endif
