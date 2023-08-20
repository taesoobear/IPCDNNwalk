// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hanyang university.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * Taesoo Kwon
 */
/** @file DynamicsSimulator/server/DynamicsSimulator_Trbdl_QP.cpp
 *
 */
#include "physicsLib.h"
#include "DynamicsSimulator.h"
#include "Body.h"
#include <vector>
#include <map>
#include <algorithm>
#include "../../BaseLib/math/Operator_NR.h"
#include "../../BaseLib/math/Operator.h"
#include "../../BaseLib/math/conversion.h"

#include "../TRL/eigenSupport.h"
#include "DynamicsSimulator_Trbdl_QP.h"
#include "../../BaseLib/utility/QPerformanceTimer.h"
#ifndef NO_OGRE
#include <OgreSceneNode.h>
#include "../../MainLib/OgreFltk/renderer.h"
#endif

inline vectornView vec(const double* v, int size)
{
	return vectornView((double*)v, size, 1);
}
//#include <sml.h>
//#include "pgs.h"
using namespace OpenHRP;
using namespace std;
using namespace Trbdl;

// #define INTEGRATOR_DEBUG
static const int debugMode = false;
static const bool enableTimeMeasure = false;

#if 1
// to disable profiling, set 1
#undef BEGIN_TIMER
#undef END_TIMER2
#define BEGIN_TIMER(x)
#define END_TIMER2(x)
#endif

DynamicsSimulator_Trbdl_QP::DynamicsSimulator_Trbdl_QP(const char* coldet)
:DynamicsSimulator_Trbdl_penalty(coldet),
DynamicsSimulator_QP(this)
{
}



DynamicsSimulator_Trbdl_QP::~DynamicsSimulator_Trbdl_QP()
{
}



void DynamicsSimulator_Trbdl_QP::init(double timeStep,
	  OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
{
	DynamicsSimulator_Trbdl_penalty::init(timeStep, integrateOpt);
}

void forwardKinematics (Trbdl:: BodyInfo& bi, Model &model, const VectorNd &Q, const VectorNd &QDot);
void DynamicsSimulator_Trbdl_QP::initSimulation()
{
	int n = numSkeleton();
	for(int i=0; i < n; ++i){
		auto &bi=bodyInfo(i);
		// necesary for QP
		_enableDotJocobianComputation(i);

		forwardKinematics(bi, bi.model, bi.Q, bi.QDot);
		bi.clearExternalForces();
	}

	_updateCharacterPose();

	// necessary for QP
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	_contacts->solve(collisionDetector->getCollisionPairs(),*collisions);
}



void DynamicsSimulator_Trbdl_QP::_stepKinematic(int ichar, vectorn const& QDDot)
{
	Trbdl::DynamicsSimulator_Trbdl_penalty::_stepKinematic(ichar, QDDot);
	// necessary for QP
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);

	_contacts->solve(collisionDetector->getCollisionPairs(),*collisions);
}
