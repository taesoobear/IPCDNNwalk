// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/** @file CollisionDetector/server/CollisionDetector_impl.h
 * Implementation of CollisionDetector_impl and CollisionDetectorFactory_impl
 */

#ifndef OPENHRP_COLISIONDETECTOR_IMPL_H_INCLUDED
#define OPENHRP_COLISIONDETECTOR_IMPL_H_INCLUDED

#include "../CollisionDetector.h"
#include "CollisionDetector_impl2.h"
class CdCache;
class CdScene;
class CdCharCache;
class CdCheckPair;
class CdJoint ;
#include <vector>

using namespace std;
namespace OpenHRP {

	/**
	 *
	 * CollisionDetector_impl class
	 * @version 0.3
	 *
	 */
	class CollisionDetector_impl : public OpenHRP::CollisionDetector, public CollisionDetector_impl_hidden
	{

	private:

        
	public:

		CollisionDetector_impl(CdCharCache* cache);

		virtual ~CollisionDetector_impl();

		virtual void addModel(const char* charName, CharacterInfo const& model);

		virtual void addCollisionPair( LinkPair const & colPair,
									  bool convexsize1,
									  bool convexsize2
									  );

		virtual void removeCollisionPair(const LinkPair& colPair);

		virtual bool queryContactDeterminationForDefinedPairs(std::vector<DynamicsSimulator::Character*> const& positions, CollisionSequence & collisions);

		virtual void setWorldTransformations(int charIndex, BoneForwardKinematics const& fk); 
		virtual bool testIntersectionsForDefinedPairs(CollisionSequence & collisions);

	private:

		void _contactDetermination(CdCheckPair* rPair, Collision& collision);

		void _setCharacterData(std::vector<DynamicsSimulator::Character*> const& characters);

		int _contactIntersection(CdCheckPair* rPair);

	};
}

#endif
