// Taesoo Kwon

#ifndef OPENHRP_COLISIONDETECTOR_gjk_H_INCLUDED
#define OPENHRP_COLISIONDETECTOR_gjk_H_INCLUDED

#include "Bullet/CollisionDetector_bullet.h"

namespace TRL {
	/**
	 * CollisionDetector_gjk class
	 */
	class CollisionDetector_gjk : public OpenHRP:: CollisionDetector_bullet
	{
	public:
		CollisionDetector_gjk();

		virtual ~CollisionDetector_gjk();

		virtual bool testIntersectionsForDefinedPairs( OpenHRP::CollisionSequence & collisions);
	};
}

#endif
