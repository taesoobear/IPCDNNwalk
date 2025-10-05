// Taesoo Kwon

#ifndef TRL_COLISIONDETECTOR_fcl_H_INCLUDED
#define TRL_COLISIONDETECTOR_fcl_H_INCLUDED

#include "../CollisionDetector.h"
#include "../../MainLib/Ogre/intersectionTest.h"


class fcl_ColObject;

namespace TRL {

	/**
	 * CollisionDetector_fcl class
	 */
	class CollisionDetector_fcl : public OpenHRP::CollisionDetector
	{
		void _addModel(VRMLloader* loader); // returns characterIndex
	public:

		// num_character by num_bone matrix
		std::vector<std::vector<fcl_ColObject*> > m_col_objects;


		CollisionDetector_fcl();

		virtual ~CollisionDetector_fcl();

		virtual int addModel(VRMLloader* loader); // returns characterIndex

		virtual void setWorldTransformations(int charIndex, BoneForwardKinematics const& fk); 
		virtual bool testIntersectionsForDefinedPairs(OpenHRP::CollisionSequence & collisions);
		virtual void addCollisionPair(
									  OpenHRP::LinkPair const & colPair,
									  bool convexsize1,
									  bool convexsize2
									  );

		virtual void removeCollisionPair(const OpenHRP::LinkPair& colPair);

		virtual void rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, RayTestResult& result);

	};
}

#endif
