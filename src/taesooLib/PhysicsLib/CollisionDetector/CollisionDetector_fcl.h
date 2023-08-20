// Taesoo Kwon

#ifndef TRL_COLISIONDETECTOR_fcl_H_INCLUDED
#define TRL_COLISIONDETECTOR_fcl_H_INCLUDED

#include "../CollisionDetector.h"
#include "../../MainLib/Ogre/intersectionTest.h"

#include <Eigen/Dense>

#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"

namespace TRL {

	/**
	 * CollisionDetector_fcl class
	 */
	class CollisionDetector_fcl : public OpenHRP::CollisionDetector
	{
		void _addModel(VRMLloader* loader); // returns characterIndex
	public:
		struct ColObject // corresponds to a bone
		{
			struct OBJ_T{ 
				std::shared_ptr<fcl::CollisionGeometry<double>> geom;
				fcl::CollisionObject<double>* co;
				bool isMesh;
			};

			struct Info{
				vector3N vertices;
				int elementType;
				vector3 elementSize;
				transf tf; // local coordinate for the element
				transf gtf; // global coordinate for the element
			};
			//
			// one bone can have multiple convex collision shapes
			std::vector<OBJ_T> co;
			std::vector<Info> co_info;

			inline bool isLargeBox(int isubMesh, double thr)
			{
				// large enough to contain corner spheres (see CollisionDetector_fcl_LBS.cpp).
				auto& info=co_info[isubMesh];
				auto& esize=info.elementSize;
				return info.elementType==OBJloader::Element::BOX && esize.x>thr && esize.y>thr && esize.z>thr;
			}
			
			// for broadphase
			intersectionTest::AABB lb;	// local bounds
			intersectionTest::AABB gb;	// global bounds
			OBJloader::Geometry* mesh;
		};

		// num_character by num_bone matrix
		std::vector<std::vector<ColObject*> > m_col_objects;


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
