// Taesoo Kwon

#ifndef OPENHRP_COLISIONDETECTOR_libccd_H_INCLUDED
#define OPENHRP_COLISIONDETECTOR_libccd_H_INCLUDED

#include "../CollisionDetector.h"
#include "../../MainLib/Ogre/intersectionTest.h"

extern "C" {
#include "libccd/testsuites/support.h"
}
namespace OpenHRP {

	/**
	 * CollisionDetector_libccd class
	 */
	class CollisionDetector_libccd : public OpenHRP::CollisionDetector
	{
		void _addModel(VRMLloader* loader); // returns characterIndex
	public:
		struct ColObject // corresponds to a bone
		{
			OBJloader::Geometry* mesh;
			union OBJ_T{
				ccd_box_t box;
				ccd_sphere_t sphere;
				ccd_cyl_t cyl;
				ccd_cap_t cap;
				ccd_general_t gen;
			};

			struct Info{
				vector3N vertices;
				int elementType;
				vector3 elementSize;
				transf tf; // local coordinate for the element
				transf gtf; // global coordinate for the element
			};

			std::vector<OBJ_T> co;
			std::vector<Info> co_info;

			inline bool isLargeBox(int isubMesh)
			{
				// large enough to contain corner spheres (see CollisionDetector_libccd_LBS.cpp).
				auto& info=co_info[isubMesh];
				auto& esize=info.elementSize;
				return info.elementType==OBJloader::Element::BOX && esize.x>0.03 && esize.y>0.03 && esize.z>0.03;
			}
			// for broadphase
			intersectionTest::AABB lb;	// local bounds
			intersectionTest::AABB gb;	// global bounds
			vector3 center;
			double radius;
		};

		// num_character by num_bone matrix
		std::vector<std::vector<ColObject*> > m_col_objects;


		CollisionDetector_libccd();

		virtual ~CollisionDetector_libccd();

		virtual int addModel(VRMLloader* loader); // returns characterIndex

		virtual void setWorldTransformations(int charIndex, BoneForwardKinematics const& fk); 
		virtual bool testIntersectionsForDefinedPairs(CollisionSequence & collisions);
		bool CollisionCheckMesh(CollisionSequence & collisions, std::string chekmesh, std::string skipmesh);
		bool CollisionCheck(CollisionDetector &s, CollisionSequence & collisions, std::string chekmesh, std::string skipmesh);
		virtual void addCollisionPair(
									  LinkPair const & colPair,
									  bool convexsize1,
									  bool convexsize2
									  );

		virtual void removeCollisionPair(const LinkPair& colPair);

		virtual void rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, RayTestResult& result);

	};
}

#endif
