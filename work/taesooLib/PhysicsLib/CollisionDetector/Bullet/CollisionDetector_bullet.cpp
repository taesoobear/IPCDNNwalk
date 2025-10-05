// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2018, CALAB.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * Taesoo Kwon
 */
#include <physicsLib.h>
 
///This low-level internal demo does intentionally NOT use the btBulletCollisionCommon.h header
///It needs internal access
#include "../../../BaseLib/motion/gjk/btTransform.h"
#include "../../../BaseLib/motion/gjk/btConvexHullShape.h"
#include "../../../BaseLib/motion/gjk/btGjkEpa2.h"
#include "../../../BaseLib/motion/gjk/btGjkEpa.h"
#include "../../../BaseLib/motion/gjk/btStackAlloc.h"
#include "../../../BaseLib/motion/gjk/btSphereShape.h"
#include "../../../BaseLib/motion/gjk/btVoronoiSimplexSolver.h"
#include "../../../BaseLib/motion/gjk/btVoronoiSimplexSolver.h"
#include "../../../BaseLib/motion/gjk/btSubSimplexConvexCast.h"

#include "CollisionDetector_bullet.h"
#include "../MainLib/OgreFltk/pldprimskin.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/motion/VRMLloader_internal.h"

using namespace std;
using namespace OpenHRP;
using namespace gjk;

#define USE_GJK_EPA_SOLVER_OLD
#define USE_BROADPHASE
//#define COL_DEBUG

inline btVector3 ToBullet(vector3 const& v)
{
	return btVector3(v.x, v.y, v.z);
}

inline vector3 ToBase(btVector3 const& v)
{
	return vector3(v.x(), v.y(), v.z());
}


static double collisionMargin=0.01;
CollisionDetector_bullet::CollisionDetector_bullet() 
	:OpenHRP::CollisionDetector()
{
	#ifdef COL_DEBUG
	FILE* fl=fopen("broadPhase.txt", "wt");
	fclose(fl);
#endif

}

CollisionDetector_bullet::~CollisionDetector_bullet()
{
	for(int i=0; i<m_col_objects.size(); i++)
	{
		auto& _object=m_col_objects[i];
		for(int j=0; j<_object.size(); j++)
		{
			if(!_object[j]) continue;
			std::vector<btCollisionObject*>& col=_object[j]->co;

			for(int k=0; k<col.size(); k++)
			{
				btCollisionObject* colobject=col[k];
				delete colobject->getCollisionShape();
				delete colobject;
			}
			if(_object[j]) delete _object[j];
		}
	}
}

int CollisionDetector_bullet::addModel(VRMLloader* loader)
{
	int ichar=	CollisionDetector::addModel(loader);
	_addModel(loader);
	return ichar;
}
void CollisionDetector_bullet::addModel(const char* charName,
                                      CharacterInfo const& cinfo)
{

	CollisionDetector::addModel(charName, cinfo);
}

void CollisionDetector_bullet::_addModel(VRMLloader* loader)
{
	m_col_objects.resize(m_col_objects.size()+1);
	std::vector<ColObject*>& _object=m_col_objects.back();
	
	int numBone=loader->numBone();
	_object.resize(numBone);
	_object[0]=NULL;
	for(int b=1; b<numBone; b++)
	{
		_object[b]=NULL;
		
		HRP_SHAPE* shape=loader->VRMLbone(b).mShape;
		if(shape)
		{
			_object[b]=new ColObject();
			
			int numSubMesh=shape->mesh.faceGroups.size();
			_object[b]->co.resize(numSubMesh);
			for(int k=0; k<numSubMesh; k++)
				_object[b]->co[k]=new btCollisionObject();

			  if (numSubMesh!=1) 
			    {
			      printf("%s numSubmesh %d\n", loader->VRMLbone(b).name().ptr(), numSubMesh);
			    }

#ifdef COL_DEBUG

			    
				  TString output;
				  output.format("%s numSubMesh %d, %d %d", loader->VRMLbone(b).name().ptr(), numSubMesh,
								shape->mesh.numVertex(), shape->mesh.faceGroups.end(0));
			  				
			   OutputToFile("broadPhase.txt", output.ptr());
#endif
			   _object[b]->mesh=&shape->mesh;

			for(int subMesh=0; subMesh<numSubMesh; subMesh++)
			{
				std::vector<gjk::btVector3> vertices;
				OBJloader::Element& e=shape->mesh.elements[subMesh];
				int sindex=shape->mesh.faceGroups.start(subMesh);
				int numFace=shape->mesh.faceGroups.end(subMesh)-sindex;
			

				if (e.elementType==OBJloader::Element::BOX)
				{
					vector3 shrinkSize=e.elementSize*0.5;

					if(shrinkSize.x>collisionMargin*3) shrinkSize.x-=collisionMargin*0.5;
					if(shrinkSize.y>collisionMargin*3) shrinkSize.y-=collisionMargin*0.5;
					if(shrinkSize.z>collisionMargin*3) shrinkSize.z-=collisionMargin*0.5;

					vector3N localpos(8);
					localpos.setAllValue(shrinkSize);
					localpos(4).x*=-1.0;
					localpos(5).x*=-1.0;
					localpos(6).x*=-1.0;
					localpos(7).x*=-1.0;
					localpos(1).y*=-1.0;
					localpos(3).y*=-1.0;
					localpos(5).y*=-1.0;
					localpos(7).y*=-1.0;
					localpos(0).z*=-1.0;
					localpos(1).z*=-1.0;
					localpos(4).z*=-1.0;
					localpos(5).z*=-1.0;

					vertices.resize(8);
					for(int i=0; i<8; i++)
					{
						vertices[i]=ToBullet(e.tf*localpos(i));
						_object[b]->lb.merge(ToBase(vertices[i]));
					}
				}
				else if (
						(e.elementType==OBJloader::Element::SPHERE || 
						(e.elementType==OBJloader::Element::ELLIPSOID && e.elementSize.x==e.elementSize.y && e.elementSize.z==e.elementSize.x)
						)&& e.tf.translation.length()<1e-3)
				{
					btCollisionShape* convexShape = new btSphereShape(e.elementSize.x);
					Msg::verify(convexShape, "convexShape");

					convexShape->setMargin(collisionMargin);

					btCollisionObject* colobject=_object[b]->co[subMesh];
					colobject->getWorldTransform().getBasis().setIdentity();
					colobject->getWorldTransform().setOrigin(btVector3(0,0,0));
					colobject->setCollisionShape(convexShape);
					_object[b]->lb.merge(e.elementSize);
					_object[b]->lb.merge(e.elementSize*-1);
					continue;
				}

				else
				{
					bitvectorn bvertices;
					bvertices.resize(shape->mesh.numVertex());

					bvertices.clearAll();
					for(int i=0; i<numFace; i++)
					{
						OBJloader::Face& f=shape->mesh.getFace(sindex+i);

						bvertices.setAt(f.vertexIndex(0));
						bvertices.setAt(f.vertexIndex(1));
						bvertices.setAt(f.vertexIndex(2));
					}


					vertices.resize(bvertices.count());

					Msg::verify(bvertices.size()>0 && vertices.size()>0, "vertices??");
					int c=0;
					for(int i=0; i<bvertices.size(); i++)
					{
						if(bvertices[i])
						{
							vector3 const& v=shape->mesh.getVertex(i);
							vertices[c++]=ToBullet(v);
							_object[b]->lb.merge(v);
						}
					}

					ASSERT(c==vertices.size());
				}

					
				btConvexHullShape* convexShape = new btConvexHullShape(&vertices[0].getX(), vertices.size());
				Msg::verify(convexShape, "convexShape");

				convexShape->setMargin(collisionMargin);

				btCollisionObject* colobject=_object[b]->co[subMesh];
				colobject->getWorldTransform().getBasis().setIdentity();
				colobject->getWorldTransform().setOrigin(btVector3(0,0,0));
				colobject->setCollisionShape(convexShape);


			}
		}
	}
}


void CollisionDetector_bullet::addCollisionPair(const LinkPair& colPair,
                                              bool convexsize1,
                                              bool convexsize2)
{
	CollisionDetector::addCollisionPair(colPair, convexsize1, convexsize2);

}

void CollisionDetector_bullet::removeCollisionPair(const LinkPair& colPair)
{
	ASSERT(0);
}




#include "../BaseLib/utility/QPerformanceTimer.h"

//QPerformanceTimerCount counter2(100, "collision detection");
#ifdef USE_GJK_EPA_SOLVER_OLD
static btStackAlloc g_stack(100000);
#endif

void CollisionDetector_bullet::setWorldTransformations(int charIndex, BoneForwardKinematics const& fk)
{
	int i=charIndex;
	std::vector<ColObject*>& col_objects=m_col_objects[i];
#ifdef USE_BROADPHASE
	matrix4 transf;
#endif
	for(int b=1, nb=mTrees[i]->numBone(); b<nb; b++)
	{
		if(col_objects[b])
		{
			ColObject& co=*col_objects[b];
#ifdef USE_BROADPHASE
			transf.setRotation(fk.global(b).rotation);
			transf.setTranslation(fk.global(b).translation);



			co.gb=co.lb;
			co.gb.transform(transf);
			co.gb.enlarge(collisionMargin);


#ifdef COL_DEBUG
			TString output;
			::transf f=fk.global(b);
			output.format("rot %s trans %s\n", f.rotation.output().ptr(),
					f.translation.output().ptr());
			OutputToFile("broadPhase.txt", output.ptr());
			output.format("lb  %s %s %s %s\n", co.lb.getMaximum().output().ptr(),
					co.lb.getMinimum().output().ptr(),
					co.lb.getMaximum().output().ptr(),
					co.lb.getMinimum().output().ptr());
			OutputToFile("broadPhase.txt", output.ptr());
			output.format("gb  %s %s %s %s\n", co.gb.getMaximum().output().ptr(),
					co.gb.getMinimum().output().ptr(),
					co.gb.getMaximum().output().ptr(),
					co.gb.getMinimum().output().ptr());
			OutputToFile("broadPhase.txt", output.ptr());
#endif
#endif

			quater rot=fk.global(b).rotation;
			btVector3 trans=ToBullet(fk.global(b).translation);


			for(int subMesh=0; subMesh<co.co.size(); subMesh++)
			{
				co.co[subMesh]->getWorldTransform().setRotation(rot);
				co.co[subMesh]->getWorldTransform().setOrigin(trans);
			}
		}
	}
}

/*
double CollisionDetector_bullet:: testSphereIntersection(int iloader, int ibody, vector3 const& position, double radius, vector3& normal)
{
	btSphereShape pointShape(btScalar(radius));
	pointShape.setMargin(collisionMargin);

	ColObject* colobject=m_col_objects[iloader][ibody];
	for(int subMesh=0; subMesh<colobject->co.size(); subMesh++)
	{
		auto * shape2=(btConvexHullShape*)(colobject->co[subMesh]->getCollisionShape());
		auto& T=colobject->co[subMesh]->getWorldTransform();
#ifdef USE_GJK_EPA_SOLVER_OLD

				Msg::error("not implemented yet");
				btGjkEpaSolver::sResults results;
				btScalar radialMargin = lp.margin;

				btGjkEpaSolver::Collide(
					(btConvexShape*)colobject[0]->co[subMesh1]->getCollisionShape(),
					colobject[0]->co[subMesh1]->	getWorldTransform(),
					(btConvexShape*)colobject[1]->co[subMesh2]->getCollisionShape(),
					colobject[1]->co[subMesh2]->	getWorldTransform(),
					radialMargin,
					&g_stack,
					results);
				double depth=results.depth;
				if (depth>0)
#else 
					ASSERT(false);
#endif
	}

}
*/
bool CollisionDetector_bullet:: getLocalBoundingBoxSize(int charIndex, int ibone, vector3& localSize)
{
	std::vector<ColObject*>& _object=m_col_objects[charIndex];
	RANGE_ASSERT(ibone<_object.size());
	if(!_object[ibone]) return false;
	auto& lb=_object[ibone]->lb;

	localSize=	lb.getMaximum()-lb.getMinimum();
	return true;
}
double CollisionDetector_bullet:: calculateSignedDistance(int iloader, int ibody, vector3 const& position, vector3& normal)
{
	ColObject* colobject=m_col_objects[iloader][ibody];
	double minDist=1e10;
	btGjkEpaSolver2::sResults	res;
	if(colobject)
	for(int subMesh=0; subMesh<colobject->co.size(); subMesh++)
	{
		auto * shape2=(btConvexHullShape*)(colobject->co[subMesh]->getCollisionShape());
		auto& T=colobject->co[subMesh]->getWorldTransform();

		btVector3 gp=ToBullet(position);
		btScalar radialMargin = collisionMargin;
		double dist=(btGjkEpaSolver2::SignedDistance(gp,radialMargin,shape2,T,res));
		if (dist<minDist) 
		{
			minDist=dist;

			normal=ToBase(res.normal);

		}
	}

	return minDist+collisionMargin;
}
bool CollisionDetector_bullet::testIntersectionsForDefinedPairs( CollisionSequence & collisions)
{
//	counter2.start();
	bool flag=false;

	collisions.seq.resize(mPairs.size());
	collisions.pairs=&mPairs;



	ColObject* colobject[2];

	for(int ipair=0; ipair<mPairs.size(); ipair++)
	{
		LinkPair& lp=mPairs[ipair];

		
		for(int i=0; i<2; i++)
			colobject[i]=m_col_objects[lp.charIndex[i]][lp.link[i]->treeIndex()];

//#define CAR_DEBUG
#ifdef CAR_DEBUG
		bool check=false;
		if(lp.charIndex[0]==1 && lp.charIndex[1]==2 && lp.link[0]->treeIndex()==1 && lp.link[1]->treeIndex()==1)
		{
			check=true;
			printf("check\n");
		}
		if(lp.charIndex[0]==2 && lp.charIndex[1]==1 && lp.link[0]->treeIndex()==1 && lp.link[1]->treeIndex()==1)
		{
			check=true;
			printf("check2\n");
		}

#endif
		if(colobject[0]==NULL) continue;
		if(colobject[1]==NULL) continue;

#ifdef USE_BROADPHASE
#ifdef COL_DEBUG
				TString output;
		output.format("broad phase %s %s %s %s\n", colobject[0]->gb.getMaximum().output().ptr(),
			   colobject[0]->gb.getMinimum().output().ptr(),
			   colobject[1]->gb.getMaximum().output().ptr(),
			   colobject[1]->gb.getMinimum().output().ptr());
			   OutputToFile("broadPhase.txt", output.ptr());
#endif

		// broad phase : error check
		if(colobject[0]->gb.getMaximum().x!=colobject[0]->gb.getMaximum().x)
		{
			collisions[ipair].points.resize(0);
			continue;
		}

		// broad phase.
		if(!colobject[0]->gb.intersects(colobject[1]->gb))
		{
			collisions[ipair].points.resize(0);
			continue;
		}
#endif

		collisions[ipair].points.resize(0);

		for(int subMesh1=0; subMesh1<colobject[0]->co.size(); subMesh1++)
			for(int subMesh2=0; subMesh2<colobject[1]->co.size(); subMesh2++)
			{
				//printf("co %d %d\n", subMesh1, subMesh2);
				// narrow phase.
#ifdef USE_GJK_EPA_SOLVER_OLD
				btGjkEpaSolver::sResults results;
				btScalar radialMargin = lp.margin;

				btGjkEpaSolver::Collide(
					(btConvexShape*)colobject[0]->co[subMesh1]->getCollisionShape(),
					colobject[0]->co[subMesh1]->	getWorldTransform(),
					(btConvexShape*)colobject[1]->co[subMesh2]->getCollisionShape(),
					colobject[1]->co[subMesh2]->	getWorldTransform(),
					radialMargin,
					&g_stack,
					results);
				double depth=results.depth;
				if (depth>0)
#else
				btGjkEpaSolver2::sResults results;
				btScalar radialMargin = lp.margin;

				btVector3 v;
				bool collide=btGjkEpaSolver2::Penetration(
					(btConvexShape*)colobject[0]->co[subMesh1].getCollisionShape(),
					colobject[0]->co[subMesh1].	getWorldTransform(),
					(btConvexShape*)colobject[1]->co[subMesh2].getCollisionShape(),
					colobject[1]->co[subMesh2].	getWorldTransform(),
					v,
					results);

				double depth=results.distance;
				if (collide)
#endif
				{
					flag=true;
					collisions[ipair].points.resize(collisions[ipair].points.size()+1);
					CollisionPoint& point=collisions[ipair].points.back();
					point.position=ToBase(results.witnesses[1]);
					point.normal=ToBase(results.normal)*-1;
					point.idepth=depth;
#ifdef CAR_DEBUG
					if(check)
					printf("check done\n");	
#endif // CAR_DEBUG
				}
			}
	}

	/*

	//////////////////////////////////
	// 제???? ?????? ??젼. OpenHRP?? ?????? ???? ?刻티? ????.
	collisions.resize(mPairs.size());
	bool flag=false;
	
	// 1. set world transforms.
	for(int i=0; i<characters.size(); i++)
	{
		std::vector<btCollisionObject*>& col_objects=m_col_objects[i];
		BoneForwardKinematics& fk=*characters[i]->chain;
		for(int b=1, nb=mTrees[i]->numBone(); b<nb; b++)
		{
			if(col_objects[b])
			{
				col_objects[b]->getWorldTransform().setRotation(ToBullet(fk.global(b).rotation));
				col_objects[b]->getWorldTransform().setOrigin(ToBullet(fk.global(b).translation));
			}
		}
	}

	m_collisionWorld->performDiscreteCollisionDetection();

	int numManifolds = m_collisionWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = m_collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	

		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();	
	}
*/
//	counter2.stop();

    return flag;
}


namespace gjk{
	struct	LocalShapeInfo
	{
		int	m_shapePart;
		int	m_triangleIndex;
		
		//const btCollisionShape*	m_shapeTemp;
		//const btTransform*	m_shapeLocalTransform;
	};
	struct	LocalRayResult
	{
		LocalRayResult(btCollisionObject*	collisionObject, 
			LocalShapeInfo*	localShapeInfo,
			const btVector3&		hitNormalLocal,
			btScalar hitFraction)
		:m_collisionObject(collisionObject),
		m_localShapeInfo(localShapeInfo),
		m_hitNormalLocal(hitNormalLocal),
		m_hitFraction(hitFraction)
		{
		}

		btCollisionObject*		m_collisionObject;
		LocalShapeInfo*			m_localShapeInfo;
		btVector3				m_hitNormalLocal;
		btScalar				m_hitFraction;

	};
	///RayResultCallback is used to report new raycast results
	struct	RayResultCallback
	{
		btScalar	m_closestHitFraction;
		btCollisionObject*		m_collisionObject;

		virtual ~RayResultCallback()
		{
		}
		bool	HasHit()
		{
			return (m_collisionObject != 0);
		}

		RayResultCallback()
			:m_closestHitFraction(btScalar(1.)),
			m_collisionObject(0)
		{
		}
		virtual	btScalar	AddSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace) = 0;
	};

	struct	ClosestRayResultCallback : public RayResultCallback
	{
		ClosestRayResultCallback()
		{
		}

		btVector3	m_hitNormalWorld;
			
		virtual	btScalar	AddSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace)
		{
			//caller already does the filter on the m_closestHitFraction
			btAssert(rayResult.m_hitFraction <= m_closestHitFraction);
			
			m_closestHitFraction = rayResult.m_hitFraction;
			m_collisionObject = rayResult.m_collisionObject;
			if (normalInWorldSpace)
			{
				m_hitNormalWorld = rayResult.m_hitNormalLocal;
			} else
			{
				///need to transform normal into worldspace
				m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
			}
			return rayResult.m_hitFraction;
		}
	};
	static void	rayTestSingle(const btTransform& rayFromTrans,const btTransform& rayToTrans,
					  btCollisionObject* collisionObject,
					  const btCollisionShape* collisionShape,
					  const btTransform& colObjWorldTransform,
					  RayResultCallback& resultCallback, short int collisionFilterMask=-1)
	{
		btSphereShape pointShape(btScalar(0.0));
		pointShape.setMargin(0.f);
		const btConvexShape* castShape = &pointShape;

		if (collisionShape->isConvex())
		{
			btConvexCast::CastResult castResult;
			castResult.m_fraction = resultCallback.m_closestHitFraction;

			btConvexShape* convexShape = (btConvexShape*) collisionShape;
			btVoronoiSimplexSolver	simplexSolver;
#define USE_SUBSIMPLEX_CONVEX_CAST 1
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
			btSubsimplexConvexCast convexCaster(castShape,convexShape,&simplexSolver);
#else
			//btGjkConvexCast	convexCaster(castShape,convexShape,&simplexSolver);
			//btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);
#endif //#USE_SUBSIMPLEX_CONVEX_CAST

			if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,colObjWorldTransform,colObjWorldTransform,castResult))
			{
				//add hit
				if (castResult.m_normal.length2() > btScalar(0.0001))
				{				
					if (castResult.m_fraction < resultCallback.m_closestHitFraction)
					{
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
						//rotate normal into worldspace
						castResult.m_normal = rayFromTrans.getBasis() * castResult.m_normal;
#endif //USE_SUBSIMPLEX_CONVEX_CAST

						castResult.m_normal.normalize();
						LocalRayResult localRayResult
							(
							 collisionObject, 
							 0,
							 castResult.m_normal,
							 castResult.m_fraction
							);

						bool normalInWorldSpace = true;
						resultCallback.AddSingleResult(localRayResult, normalInWorldSpace);

					}
				}
			}
		} else {
			Msg::error("not ported yet");
		}
	}
}
void CollisionDetector_bullet::rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result)
{
	ClosestRayResultCallback resultCallback ;

	ColObject* colobject=m_col_objects[ichar][ilink];
	if (colobject==NULL) return;
	btTransform rayFromTrans;
	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(ToBullet(from));
	btTransform rayToTrans;
	rayToTrans.setIdentity();
	rayToTrans.setOrigin(ToBullet(to));

	for(int subMesh=0; subMesh<colobject->co.size(); subMesh++)
	{ 
		btCollisionObject& co=*colobject->co[subMesh];
		rayTestSingle(rayFromTrans,rayToTrans,&co, co.getCollisionShape(),co.getWorldTransform(), resultCallback);
	}
	if (resultCallback.HasHit())
	{
		result.m_closestHitFraction=resultCallback.m_closestHitFraction;
		result.m_hitNormalWorld=ToBase(resultCallback.m_hitNormalWorld);
	}
}
void CollisionDetector_bullet::rayTestBackside(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result)
{
	ClosestRayResultCallback resultCallback ;

	ColObject* colobject=m_col_objects[ichar][ilink];
	if (colobject==NULL) return;
	btTransform rayFromTrans;
	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(ToBullet(from));
	btTransform rayToTrans;
	rayToTrans.setIdentity();
	rayToTrans.setOrigin(ToBullet(to));

	for(int subMesh=0; subMesh<colobject->co.size(); subMesh++)
	{ 
		btCollisionObject& co=*colobject->co[subMesh];

		// rayTestSingle 을 속기기 위하여 레이 방향을 뒤집음. 
		ClosestRayResultCallback tempResult;
		rayTestSingle(rayToTrans,rayFromTrans,&co, co.getCollisionShape(),co.getWorldTransform(), tempResult);

		if (tempResult.HasHit())
		{
			// 이제 레이 파라미터를 뒤집음.
			double newHitFraction=1.0- tempResult.m_closestHitFraction;

			if(newHitFraction	<= resultCallback.m_closestHitFraction)
			{
				// 가까우면.
				resultCallback.m_closestHitFraction=newHitFraction;
				resultCallback.m_hitNormalWorld=tempResult.m_hitNormalWorld;
				resultCallback.m_collisionObject=tempResult.m_collisionObject;
			}
		}
	}

	if (resultCallback.HasHit())
	{
		result.m_closestHitFraction=resultCallback.m_closestHitFraction;
		result.m_hitNormalWorld=ToBase(resultCallback.m_hitNormalWorld);
	}
}
namespace OpenHRP
{
CollisionDetector* createCollisionDetector_bullet()
{
	printf("bullet collision detector created!\n");
	return new CollisionDetector_bullet();
}
}
/*
 * btCollisionConfiguration* bt_collision_configuration;
btCollisionDispatcher* bt_dispatcher;
btBroadphaseInterface* bt_broadphase;
btCollisionWorld* bt_collision_world;

double scene_size = 500;
unsigned int max_objects = 16000;

bt_collision_configuration = new btDefaultCollisionConfiguration();
bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

btScalar sscene_size = (btScalar) scene_size;
btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);
//This is one type of broadphase, bullet has others that might be faster depending on the application
bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, true);  // true for disabling raycast accelerator

bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);
//Create two collision objects
btCollisionObject* sphere_A = new btCollisionObject();
btCollisionObject* sphere_B = new btCollisionObject();
//Move each to a specific location
sphere_A->getWorldTransform().setOrigin(btVector3((btScalar) 2, (btScalar) 1.5, (btScalar) 0));
sphere_B->getWorldTransform().setOrigin(btVector3((btScalar) 2, (btScalar) 0, (btScalar) 0));
//Create a sphere with a radius of 1
btSphereShape * sphere_shape = new btSphereShape(1);
//Set the shape of each collision object
sphere_A->setCollisionShape(sphere_shape);
sphere_B->setCollisionShape(sphere_shape);
//Add the collision objects to our collision world
bt_collision_world->addCollisionObject(sphere_A);
bt_collision_world->addCollisionObject(sphere_B);

//Perform collision detection
bt_collision_world->performDiscreteCollisionDetection();

int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
//For each contact manifold
for (int i = 0; i < numManifolds; i++) {
  btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
  btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
    btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
    contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
    int numContacts = contactManifold->getNumContacts();
    //For each contact point in that manifold
    for (int j = 0; j < numContacts; j++) {
      //Get the contact information
        btManifoldPoint& pt = contactManifold->getContactPoint(j);
        btVector3 ptA = pt.getPositionWorldOnA();
        btVector3 ptB = pt.getPositionWorldOnB();
        double ptdist = pt.getDistance();
    }
}
*/
