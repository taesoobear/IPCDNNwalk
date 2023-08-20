
// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hayang University.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 *
 * \author : Taesoo Kwon
 */
#ifndef NEW_DYNAMICSSIMULATOR_Trbdl_QP_HEADER
#define NEW_DYNAMICSSIMULATOR_Trbdl_QP_HEADER

/** @file DynamicsSimulator/server/DynamicsSimulator_Trbdl_QP.h
 *
 */


#include "../CollisionDetector.h"
#include "DynamicsSimulator_Trbdl_penalty.h"
#include "../DynamicsSimulator_QP.h"



namespace Trbdl {

	/**
	 * DynamicsSimulator_ class
	 */
	class DynamicsSimulator_Trbdl_QP : public DynamicsSimulator_Trbdl_penalty, public OpenHRP::DynamicsSimulator_QP
	{
		vectorn _f;
		vector3N contactPos;
	public:

		DynamicsSimulator_Trbdl_QP(const char* coldet);
		~DynamicsSimulator_Trbdl_QP();
		virtual void init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);
		virtual void initSimulation();
		virtual void _stepKinematic(int ichar, vectorn const& QDDot);
	};
}


#endif
