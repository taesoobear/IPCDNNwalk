/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H

#include "btTransform.h"
#include "btVector3.h"
#include "btMatrix3x3.h"

namespace gjk {
//#include "btPoint3.h"
//#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" //for the shape types

enum BroadphaseNativeTypes
{
// polyhedral convex shapes
	BOX_SHAPE_PROXYTYPE,
	TRIANGLE_SHAPE_PROXYTYPE,
	TETRAHEDRAL_SHAPE_PROXYTYPE,
	CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
	CONVEX_HULL_SHAPE_PROXYTYPE,
//implicit convex shapes
IMPLICIT_CONVEX_SHAPES_START_HERE,
	SPHERE_SHAPE_PROXYTYPE,
	MULTI_SPHERE_SHAPE_PROXYTYPE,
	CAPSULE_SHAPE_PROXYTYPE,
	CONE_SHAPE_PROXYTYPE,
	CONVEX_SHAPE_PROXYTYPE,
	CYLINDER_SHAPE_PROXYTYPE,
	UNIFORM_SCALING_SHAPE_PROXYTYPE,
	MINKOWSKI_SUM_SHAPE_PROXYTYPE,
	MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
//concave shapes
CONCAVE_SHAPES_START_HERE,
	//keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
	TRIANGLE_MESH_SHAPE_PROXYTYPE,
	///used for demo integration FAST/Swift collision library and Bullet
	FAST_CONCAVE_MESH_PROXYTYPE,
	//terrain
	TERRAIN_SHAPE_PROXYTYPE,
///Used for GIMPACT Trimesh integration
	GIMPACT_SHAPE_PROXYTYPE,
	
	EMPTY_SHAPE_PROXYTYPE,
	STATIC_PLANE_PROXYTYPE,
CONCAVE_SHAPES_END_HERE,

	COMPOUND_SHAPE_PROXYTYPE,

	SOFTBODY_SHAPE_PROXYTYPE,

	MAX_BROADPHASE_COLLISION_TYPES
};

///btCollisionShape provides interface for collision shapes that can be shared among btCollisionObjects.
class btCollisionShape
{

	void* m_userPointer;

public:

	btCollisionShape() : m_userPointer(0)
	{
	}
	virtual ~btCollisionShape()
	{
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const =0;

	virtual void	getBoundingSphere(btVector3& center,btScalar& radius) const;

	///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
	virtual btScalar	getAngularMotionDisc() const;


	///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result is conservative
	void calculateTemporalAabb(const btTransform& curTrans,const btVector3& linvel,const btVector3& angvel,btScalar timeStep, btVector3& temporalAabbMin,btVector3& temporalAabbMax) const;

#ifndef __SPU__


	SIMD_FORCE_INLINE bool	isConvex() const
	{
		return true;
	}


	virtual int		getShapeType() const=0;
	virtual void	setLocalScaling(const btVector3& scaling) =0;
	virtual const btVector3& getLocalScaling() const =0;
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const = 0;


//debugging support
	virtual const char*	getName()const =0 ;
#endif //__SPU__

	

	virtual void	setMargin(btScalar margin) = 0;
	virtual btScalar	getMargin() const = 0;

	
	///optional user data pointer
	void	setUserPointer(void* userPtr)
	{
		m_userPointer = userPtr;
	}

	void*	getUserPointer() const
	{
		return m_userPointer;
	}

};	

}
#endif //COLLISION_SHAPE_H


