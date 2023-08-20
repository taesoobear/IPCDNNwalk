// Taesoo Kwon

#ifndef OPENHRP_COLISIONDETECTOR_bullet_H_INCLUDED
#define OPENHRP_COLISIONDETECTOR_bullet_H_INCLUDED

#include "../../CollisionDetector.h"
#include "../BaseLib/motion/gjk/btCollisionObject.h"
#include "../MainLib/Ogre/intersectionTest.h"
namespace OpenHRP {

	/**
	 * CollisionDetector_bullet class
	 */
	class CollisionDetector_bullet : public OpenHRP::CollisionDetector
	{
		void _addModel(VRMLloader* loader); // returns characterIndex
	public:
		class ColObject // corresponds to a bone
		{
			public:
			std::vector<gjk::btCollisionObject* > co;	// one bone can have multiple convex collision shapes
			intersectionTest::AABB lb;	// local bounds
			intersectionTest::AABB gb;	// global bounds
			OBJloader::Geometry* mesh;
		};

		// num_character by num_bone matrix
		std::vector<std::vector<ColObject*> > m_col_objects;

		CollisionDetector_bullet();

		virtual ~CollisionDetector_bullet();

		virtual void addModel(const char* charName, CharacterInfo const& model);
		virtual int addModel(VRMLloader* loader); // returns characterIndex

		virtual void setWorldTransformations(int charIndex, BoneForwardKinematics const& fk); 
		virtual bool testIntersectionsForDefinedPairs(CollisionSequence & collisions);
		virtual void addCollisionPair(
									  LinkPair const & colPair,
									  bool convexsize1,
									  bool convexsize2
									  );

		virtual void removeCollisionPair(const LinkPair& colPair);

		virtual void rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, RayTestResult& result);
		virtual void rayTestBackside(int ichar, int ilink, vector3 const& from, vector3 const& to, RayTestResult& result);

		virtual bool isSignedDistanceSupported() { return true;}
		virtual double calculateSignedDistance(int iloader, int ibody, vector3 const& position, vector3& normal);
	};
}

#endif
