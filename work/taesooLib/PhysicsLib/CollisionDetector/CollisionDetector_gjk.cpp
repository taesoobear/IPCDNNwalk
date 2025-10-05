/** @file CollisionDetector/server/CollisionDetector_gjk.cpp
 * by taesoo
 */
#include "../../physicsLib.h"

#include "../../../BaseLib/motion/gjk/btTransform.h"
#include "../../../BaseLib/motion/gjk/btConvexHullShape.h"
#include "../../../BaseLib/motion/gjk/btGjkEpa2.h"
#include "../../../BaseLib/motion/gjk/btGjkEpa.h"
#include "../../../BaseLib/motion/gjk/btStackAlloc.h"
#include "../../../BaseLib/motion/gjk/btSphereShape.h"
#include "../../../BaseLib/motion/gjk/btVoronoiSimplexSolver.h"
#include "../../../BaseLib/motion/gjk/btVoronoiSimplexSolver.h"
#include "../../../BaseLib/motion/gjk/btSubSimplexConvexCast.h"
#include "CollisionDetector_gjk.h"
#include "../../MainLib/OgreFltk/pldprimskin.h"
#include "../../MainLib/OgreFltk/VRMLloader.h"
#include "../../BaseLib/motion/VRMLloader_internal.h"

using namespace std;
using namespace TRL;
using namespace OpenHRP;
using namespace gjk;
#define USE_GJK_EPA_SOLVER_OLD
#define USE_BROADPHASE // has to be same as that defined in CollisionDetector_bullet.cpp


//#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../../MainLib/OgreFltk/objectList.h"
static ObjectList g_objectList;
#endif
static double collisionMargin=0.01;

inline btVector3 ToBullet(vector3 const& v)
{
	return btVector3(v.x, v.y, v.z);
}

inline vector3 ToBase(btVector3 const& v)
{
	return vector3(v.x(), v.y(), v.z());
}

CollisionDetector_gjk::CollisionDetector_gjk() 
:	OpenHRP::CollisionDetector_bullet()
{
}

CollisionDetector_gjk::~CollisionDetector_gjk()
{
}


#ifdef USE_GJK_EPA_SOLVER_OLD
static btStackAlloc g_stack(1000000);
#endif

inline vector3 contactMidPos(
					OpenHRP::CollisionDetector::RayTestResult &r1, 
					OpenHRP::CollisionDetector::RayTestResult &r2, 
					vector3 const& midPoint, const vector3 & normal, double& depth )
{
	vector3 hitPos1;
	hitPos1.interpolate(r1.m_closestHitFraction, midPoint, normal*100);
	vector3 hitPos2;
	hitPos2.interpolate(r2.m_closestHitFraction, midPoint, normal*-100);

	depth=hitPos1.distance(hitPos2)-2*collisionMargin;
	return hitPos1*0.5+hitPos2*0.5;

}


static inline bool rayTest(CollisionDetector_gjk* cd, int iloader, int ibody, vector3 const& from, vector3 const& to, vector3& out)
{

	OpenHRP::CollisionDetector::RayTestResult r;
	cd->rayTestBackside(iloader, ibody, from, to, r);
	if(r.hasHit())
	{
		out.interpolate(r.m_closestHitFraction, from , to);
		return true;
	}
	return false;

}


bool CollisionDetector_gjk::testIntersectionsForDefinedPairs( CollisionSequence & collisions)
{
	bool flag=false;

	collisions.seq.resize(mPairs.size());
	collisions.pairs=&mPairs;



	ColObject* colobject[2];

	intvectorn  broadphase_cache(mPairs.size()); 
	broadphase_cache.setAllValue(0);

	intmatrixn  broadphase_cache_per_character(mTrees.size(), mTrees.size());
	broadphase_cache_per_character.setAllValue(0);

	for(int ipair=0; ipair<mPairs.size(); ipair++)
	{
		LinkPair& lp=mPairs[ipair];

		collisions[ipair].points.resize(0);
		
		for(int i=0; i<2; i++)
			colobject[i]=m_col_objects[lp.charIndex[i]][lp.link[i]->treeIndex()];

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
			continue;

		// broad phase.
		if(!colobject[0]->gb.intersects(colobject[1]->gb))
			continue;

		broadphase_cache[ipair]=1;
		broadphase_cache_per_character(lp.charIndex[0], lp.charIndex[1])++;
#endif
	}

	//for(int i=0; i<broadphase_cache_per_character.rows(); i++)
	//	cout<< i<<":"<<broadphase_cache_per_character.row(i)<<endl;
	for(int ipair=0; ipair<mPairs.size(); ipair++)
	{
		LinkPair& lp=mPairs[ipair];

		if (broadphase_cache[ipair]==0) continue;
		
		for(int i=0; i<2; i++)
			colobject[i]=m_col_objects[lp.charIndex[i]][lp.link[i]->treeIndex()];

		if(colobject[0]==NULL) continue;
		if(colobject[1]==NULL) continue;

		for(int subMesh1=0; subMesh1<colobject[0]->co.size(); subMesh1++)
			for(int subMesh2=0; subMesh2<colobject[1]->co.size(); subMesh2++)
			{
				//printf("co %d %d\n", subMesh1, subMesh2);
				// narrow phase.
#ifdef USE_GJK_EPA_SOLVER_OLD
				btGjkEpaSolver::sResults results;
				//btScalar radialMargin = lp.margin;
				btScalar radialMargin = collisionMargin;

				btGjkEpaSolver::Collide(
					(btConvexShape*)colobject[0]->co[subMesh1]->getCollisionShape(),
					colobject[0]->co[subMesh1]->	getWorldTransform(),
					(btConvexShape*)colobject[1]->co[subMesh2]->getCollisionShape(),
					colobject[1]->co[subMesh2]->	getWorldTransform(),
					radialMargin,
					&g_stack,
					results);
				double depth=results.depth-collisionMargin;
				if (depth>collisionMargin*0.5)
				{
					flag=true;
					vector3 normal=ToBase(results.normal)*-1;
#else
				btGjkEpaSolver2::sResults results;
				btScalar radialMargin = lp.margin;

				btVector3 v;
				bool collide=btGjkEpaSolver2::Penetration(
					(btConvexShape*)colobject[0]->co[subMesh1]->getCollisionShape(),
					colobject[0]->co[subMesh1]->	getWorldTransform(),
					(btConvexShape*)colobject[1]->co[subMesh2]->getCollisionShape(),
					colobject[1]->co[subMesh2]->	getWorldTransform(),
					v,
					results);

				double depth=std::max(0.0, results.depth-collisionMargin);
				if (collide)
				{
					flag=true;
					vector3 normal=ToBase(results.normal);
#endif
					vector3 position=ToBase(results.witnesses[1]);
					vector3 midPoint=position+normal*depth*0.5;


					int iloader1=lp.charIndex[0];
					int iloader2=lp.charIndex[1];
					int ibody1=lp.link[0]->treeIndex();
					int ibody2=lp.link[1]->treeIndex();

					double maxDepth=0;
					if (
							broadphase_cache_per_character(iloader1, iloader2)<4 &&
							colobject[0]->mesh->elements[subMesh1].elementType==OBJloader::Element::BOX
							&&colobject[1]->mesh->elements[subMesh2].elementType==OBJloader::Element::BOX)
					{
						// 점 접촉
						auto * shape1=(btConvexHullShape*)(colobject[0]->co[subMesh1]->getCollisionShape());
						auto * shape2=(btConvexHullShape*)(colobject[1]->co[subMesh2]->getCollisionShape());
						auto& points=collisions[ipair].points;
						auto& T1=colobject[0]->co[subMesh1]->getWorldTransform();
						auto& T2=colobject[1]->co[subMesh2]->getWorldTransform();

						{
							int np=shape1->getNumPoints();
							const auto* pp=shape1->getPoints();
							for(int i=0; i<np; i++) // for all corner points of a box (for example)
							{
								btGjkEpaSolver2::sResults	res;
								btVector3 gp=T1*pp[i];
								double dist=(btGjkEpaSolver2::SignedDistance(gp,radialMargin,shape2,T2,res));
								if(dist<0)
								{

									points.resize(points.size()+1);
									auto& point=points.back();
									point.idepth=std::max(dist*-1.0-collisionMargin, 0.0);
									point.position=ToBase(gp)+normal*(dist+collisionMargin);
									point.normal=normal;
									maxDepth=std::max(maxDepth, point.idepth);
								}
							}
						}

						{
							int np=shape2->getNumPoints();
							const auto* pp=shape2->getPoints();
							for(int i=0; i<np; i++)
							{
								btGjkEpaSolver2::sResults	res;
								btVector3 gp=T2*pp[i];
								double dist=(btGjkEpaSolver2::SignedDistance(gp,radialMargin,shape1,T1,res));
								if(dist<0)
								{

									points.resize(points.size()+1);
									auto& point=points.back();
									point.idepth=std::max(dist*-1.0-collisionMargin, 0.0);
									point.position=ToBase(gp)-normal*collisionMargin;
									point.normal=normal;
									maxDepth=std::max(maxDepth, point.idepth);
								}
							}
						}
					}
					else if(broadphase_cache_per_character(iloader1, iloader2)<4 &&
							colobject[0]->mesh->elements[subMesh1].elementType==OBJloader::Element::BOX
							&&colobject[1]->mesh->elements[subMesh2].elementType==OBJloader::Element::OBJ)
					{
						auto * shape1=(btConvexHullShape*)(colobject[0]->co[subMesh1]->getCollisionShape());
						auto * shape2=(btConvexHullShape*)(colobject[1]->co[subMesh2]->getCollisionShape());
						auto& points=collisions[ipair].points;
						auto& T1=colobject[0]->co[subMesh1]->getWorldTransform();
						auto& T2=colobject[1]->co[subMesh2]->getWorldTransform();
						{
							int np=shape1->getNumPoints();
							const auto* pp=shape1->getPoints();
							for(int i=0; i<np; i++)
							{
								btGjkEpaSolver2::sResults	res;
								btVector3 gp=T1*pp[i];
								double dist=(btGjkEpaSolver2::SignedDistance(gp,radialMargin,shape2,T2,res));
								if(dist<0)
								{
									points.resize(points.size()+1);
									auto& point=points.back();
									point.idepth=std::max(dist*-1.0-collisionMargin, 0.0);
									point.position=ToBase(gp)+normal*(dist+collisionMargin);
									point.normal=normal;
									maxDepth=std::max(maxDepth, point.idepth);
								}
							}
						}
						{
							int np=shape2->getNumPoints();
							const auto* pp=shape2->getPoints();
							for(int i=0; i<np; i++)
							{
								btGjkEpaSolver2::sResults	res;
								btVector3 gp=T2*pp[i];
								double dist=(btGjkEpaSolver2::SignedDistance(gp,radialMargin,shape1,T1,res));
								if(dist<0)
								{

									points.resize(points.size()+1);
									auto& point=points.back();
									point.idepth=std::max(dist*-1.0-collisionMargin, 0.0);
									point.position=ToBase(gp)-normal*collisionMargin;
									point.normal=normal;
									maxDepth=std::max(maxDepth, point.idepth);
								}
							}
						}
					}

					//if(c<3)
					//if(added==0)
					if(maxDepth<depth-collisionMargin-1e-3)
					{
						collisions[ipair].points.resize(collisions[ipair].points.size()+1);
						CollisionPoint& point=collisions[ipair].points.back();
						point.position=midPoint-normal*depth*0.5;
						point.normal=normal;
						point.idepth=std::max(depth-collisionMargin,0.0);
					}

				/*
				 *
				 * a test code which you can paste into testCollisionCheck_simpleGrom.lua. 

				local midPoint=b.position+b.normal*b.idepth*0.5

				
				local axis1=b.normal:cross(vector3(0,0,1)):unitVector()

				local function drawRayContactPoint(axis1, msg)
					local cd=mChecker.collisionDetector
					local r1=Physics.RayTestResult ()
					local r2=Physics.RayTestResult ()
					local ibody1=bases:getBone1(ilinkpair):treeIndex()
					local ibody2=bases:getBone1(ilinkpair):treeIndex()
					local target=midPoint+axis1*100
					cd:rayTestBackside(iloader1, ibody1, midPoint,target , r1)
					cd:rayTestBackside(iloader2, ibody2, midPoint, target, r2)

					local hitPos
					if r1.m_closestHitFraction<r2.m_closestHitFraction then
						if r1:hasHit() then
							hitPos=vector3()
							hitPos:interpolate(r1.m_closestHitFraction, midPoint, target)
						end
					else
						if r2:hasHit() then
							hitPos=vector3()
							hitPos:interpolate(r2.m_closestHitFraction, midPoint, target)
						end
					end
					if hitPos and hitPos:distance(midPoint)> b.idepth+0.01 then
						mDebugDraw:drawSphere(hitPos*config.skinScale, msg..tostring(b.ibody)..tostring(i), "red",1.0)
					end
				end
				drawRayContactPoint(axis1, 'axis1')
				drawRayContactPoint(-1*axis1, 'axis2')

				*/

				}
			}
	}



	/*
#ifdef DEBUG_DRAW
						TString nameid;
						nameid.format("%d %d %d %d", lp.charIndex[0], lp.link[0]->treeIndex(),
								lp.charIndex[1], lp.link[1]->treeIndex());
						if(intersect==0)
							g_objectList.drawSphere(ToBase(pos)*100,nameid.ptr(),"red", 5.0);
						else
						{
							g_objectList.erase(nameid.ptr());
							g_objectList.drawSphere(ToBase(pos)*100,(nameid+"last").ptr(),"blue", 5.0);
						}
#endif
*/


	return flag;
}
namespace OpenHRP
{
	CollisionDetector* createCollisionDetector_gjk()
	{
		printf("gjk collision detector created!\n");
		return new CollisionDetector_gjk();
	}
}
