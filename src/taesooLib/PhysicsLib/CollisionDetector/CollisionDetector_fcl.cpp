/** @file CollisionDetector/server/CollisionDetector_fcl.cpp
 * by taesoo
 */
#include "../../BaseLib/math/mathclass.h"
#include "../../BaseLib/motion/intersectionTest.h"
#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"

namespace OBJloader
{
	class Geometry;
}

struct fcl_ColObject // corresponds to a bone
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

	bool isLargeBox(int isubMesh, double thr);


	// for broadphase
	intersectionTest::AABB lb;	// local bounds
	intersectionTest::AABB gb;	// global bounds
	OBJloader::Geometry* mesh;
};

#include "../../physicsLib.h"
#include "CollisionDetector_fcl.h"
#include "../MainLib/OgreFltk/pldprimskin.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/motion/VRMLloader_internal.h"
#include "../TRL/eigenSupport.h"
#include "../BaseLib/motion/Terrain.h"

//#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../MainLib/OgreFltk/objectList.h"
static ObjectList* g_objectList=NULL;

#endif


using namespace std;
using namespace TRL;

//#define USE_BROADPHASE_CACHE

#ifdef USE_BRADPHASE_CACHE
#define USE_BROADPHASE
#endif


static double collisionMargin=0.01;


CollisionDetector_fcl::CollisionDetector_fcl() 
	:CollisionDetector()
{
#ifdef COL_DEBUG
	FILE* fl=fopen("broadPhase.txt", "wt");
	fclose(fl);
#endif

}

static void addPoint(OpenHRP::CollisionPointSequence& points,  OpenHRP::CollisionPoint& point , double coef=0.5)
{
	// to make the results identical to bullet detector:
	point.position=point.position-point.normal*point.idepth*coef;

	point.inode[0]=-1; // meaning a rigidbody point 
	point.inode[1]=-1; // meaning a rigidbody point 
	points.resize(points.size()+1);
	points.back()=point;
}

CollisionDetector_fcl::~CollisionDetector_fcl()
{
}

int CollisionDetector_fcl::addModel(VRMLloader* loader)
{
	int ichar=	CollisionDetector::addModel(loader);
	_addModel(loader);
	return ichar;
}

void CollisionDetector_fcl_init_ColObject(fcl_ColObject* pCol, HRP_SHAPE* shape)
{
	pCol->mesh=&shape->mesh;
	int numSubMesh=shape->mesh.faceGroups.size();
	pCol->co.resize(numSubMesh);
	pCol->co_info.resize(numSubMesh);

	if (numSubMesh!=1) 
	{
		//printf("%s numSubmesh %d\n", loader->VRMLbone(b).name().ptr(), numSubMesh);
		printf("numSubmesh %d\n", numSubMesh);
	}

#ifdef COL_DEBUG


	TString output;
	output.format("%s numSubMesh %d, %d %d", loader->VRMLbone(b).name().ptr(), numSubMesh,
			shape->mesh.numVertex(), shape->mesh.faceGroups.end(0));

	OutputToFile("broadPhase.txt", output.ptr());
#endif

	for(int subMesh=0; subMesh<numSubMesh; subMesh++)
	{
		OBJloader::Element& e=shape->mesh.elements[subMesh];

		// calc bounding box
		vector3N vertices;
		int sindex=shape->mesh.faceGroups.start(subMesh);
		int numFace=shape->mesh.faceGroups.end(subMesh)-sindex;

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
				vertices[c++]=v;
				pCol->lb.merge(v); // local bounding box

			}
		}

		ASSERT(c==vertices.size());

		auto& info=pCol->co_info[subMesh];
		info.vertices=vertices;
		info.elementType=e.elementType;
		info.elementSize=e.elementSize;
		info.tf=e.tf;
		pCol->co[subMesh].isMesh=false;
		if (e.elementType==OBJloader::Element::BOX)
		{
			pCol->co[subMesh].geom.reset(new fcl::Box<double>(toEigen(e.elementSize)));
			pCol->co[subMesh].co=new fcl::CollisionObject<double>(pCol->co[subMesh].geom, fcl::Transform3<double>::Identity());

			Msg::verify(vertices.size()==8,"???");
		}
		else if((e.elementType==OBJloader::Element::ELLIPSOID && isSimilar(e.elementSize.x, e.elementSize.y) && isSimilar(e.elementSize.z, e.elementSize.x))
				|| e.elementType==OBJloader::Element::SPHERE)
		{
			pCol->co[subMesh].geom.reset(new fcl::Sphere<double>(e.elementSize.x));
			pCol->co[subMesh].co=new fcl::CollisionObject<double>(pCol->co[subMesh].geom, fcl::Transform3<double>::Identity());
		}
		else if(e.elementType==OBJloader::Element::ELLIPSOID)
		{
			pCol->co[subMesh].geom.reset(new fcl::Ellipsoid<double>(toEigen(e.elementSize)));
			pCol->co[subMesh].co=new fcl::CollisionObject<double>(pCol->co[subMesh].geom, fcl::Transform3<double>::Identity());
		}
		else if(e.elementType==OBJloader::Element::CYLINDER)
		{
			pCol->co[subMesh].geom.reset(new fcl::Cylinder<double>(e.elementSize.x*0.5, e.elementSize.y));
			pCol->co[subMesh].co=new fcl::CollisionObject<double>(pCol->co[subMesh].geom, fcl::Transform3<double>::Identity());
		}
		else if(e.elementType==OBJloader::Element::CAPSULE)
		{
			pCol->co[subMesh].geom.reset(new fcl::Capsule<double>(e.elementSize.x*0.5, e.elementSize.y*0.5));
			pCol->co[subMesh].co=new fcl::CollisionObject<double>(pCol->co[subMesh].geom, fcl::Transform3<double>::Identity());
		}
		else if(e.elementType==OBJloader::Element::TRI)
		{
			pCol->co[subMesh].isMesh=true;
			int nv=shape->mesh.numVertex();
			int nT=shape->mesh.numFace();
			std::vector<fcl::Vector3<double>> points(nv);
			for(int i=0; i<nv; i++)
				points[i]<<toEigen( shape->mesh.getVertex(i));

			// fcl supports AABB, OBB, RSS, kIOS, OBBRSS, KDOP
			auto* model=new fcl::BVHModel<fcl::RSS<double> >();
			int result;
			result = model->beginModel();
			for(int i=0; i<nT; i++)
			{
				auto& f=shape->mesh.getFace(i);
				result = model->addTriangle(points[f.vi(0)], points[f.vi(1)], points[f.vi(2)]);
				Msg::verify(result== fcl::BVH_OK, "bvh not ok");
			}

			result = model->endModel();
			Msg::verify(result== fcl::BVH_OK, "bvh not ok");

			pCol->co[subMesh].geom.reset(model);
			pCol->co[subMesh].co=new fcl::CollisionObject<double>(pCol->co[subMesh].geom, fcl::Transform3<double>::Identity());
		}
		else 
		{
			Msg::error("ColDet_fcl does not support general convex yet. todo: use libqhull");
			//if(0)
			//{
			//CCD_BOX(box);
			//box.x=0.05;
			//box.y=0.05;
			//box.z=0.05;
			//pCol->co[subMesh].box=box;
			//pCol->co[subMesh].box._data=(void*)&info;
			//}
			//else
			{
				ASSERT(false);
				//ccd_general_t gen;
				//gen.type = CCD_OBJ_GENERAL;
				//copy(vector3(0,0,0), gen.pos);
				//copy(quater(1,0,0,0), gen.quat);
				//gen._data=NULL;

				//gen.ccdSupport=&ccdSupportConvex;
				//pCol->co[subMesh].gen=gen;
				//pCol->co[subMesh].gen._data=(void*)&info;
			}
		}
	}
}
void CollisionDetector_fcl::_addModel(VRMLloader* loader)
{
	m_col_objects.resize(m_col_objects.size()+1);
	std::vector<fcl_ColObject*>& _object=m_col_objects.back();

	int numBone=loader->numBone();
	_object.resize(numBone);
	_object[0]=NULL;
	for(int b=1; b<numBone; b++)
	{
		_object[b]=NULL;

		HRP_SHAPE* shape=loader->VRMLbone(b).mShape;
		if(shape)
		{
			_object[b]=new fcl_ColObject();
			CollisionDetector_fcl_init_ColObject(_object[b], shape);

		}
	}
}


void CollisionDetector_fcl::addCollisionPair(const OpenHRP::LinkPair& colPair,
		bool convexsize1,
		bool convexsize2)
{
	CollisionDetector::addCollisionPair(colPair, convexsize1, convexsize2);

}

void CollisionDetector_fcl::removeCollisionPair(const OpenHRP::LinkPair& colPair)
{
	ASSERT(0);
}




#include "../BaseLib/utility/QPerformanceTimer.h"

//QPerformanceTimerCount counter2(100, "collision detection");

void CollisionDetector_fcl::setWorldTransformations(int charIndex, BoneForwardKinematics const& fk)
{
	int i=charIndex;
	std::vector<fcl_ColObject*>& col_objects=m_col_objects[i];
#ifdef USE_BROADPHASE
	matrix4 gtransf;
#endif
	for(int b=1, nb=mTrees[i]->numBone(); b<nb; b++)
	{
		if(col_objects[b])
		{
			fcl_ColObject& co=*col_objects[b];

#ifdef DEBUG_DRAW
		if(!g_objectList) g_objectList=new ObjectList();
		TString name;
		name.format("axes %d %d", i, b);
		g_objectList->drawAxes(fk.global(b),name.ptr(), 1, 100);
#endif
#ifdef USE_BROADPHASE
			gtransf.setRotation(fk.global(b).rotation);
			gtransf.setTranslation(fk.global(b).translation);



			co.gb=co.lb;
			co.gb.transform(gtransf);
			co.gb.enlarge(collisionMargin);
#endif


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


			for(int subMesh=0; subMesh<co.co.size(); subMesh++)
			{

				fcl_ColObject::OBJ_T& obj=co.co[subMesh];
				fcl_ColObject::Info& info=co.co_info[subMesh];


				if(info.elementType==OBJloader::Element::OBJ
						||info.elementType==OBJloader::Element::ELLIPSOID // TODO: remove this. Currently ellipsoidal collision-checks are handled using their meshes.
						)
				{
					// all types that use "vertices" for collision detection
					info.gtf=fk.global(b);
				}
				else if(info.elementType==OBJloader::Element::CYLINDER
						||info.elementType==OBJloader::Element::CAPSULE 
						)
				{
					transf local_coord_conversion;
					local_coord_conversion.rotation.setRotation(vector3(1,0,0),TO_RADIAN(90.0));
					local_coord_conversion.translation=vector3(0,0,0);
					info.gtf=fk.global(b)* info.tf*local_coord_conversion;
					//info.gtf=fk.global(b)* info.tf;
				}
				else
					info.gtf.mult(fk.global(b), info.tf);
				vector3 const& t=info.gtf.translation;
				quater const& r=info.gtf.rotation;
				obj.co->setTransform(toEigen(r), toEigen(t));
			}
		}
	}
}

static void setHalfSpace(
		VRMLloader* tree,
		OBJloader::Terrain* terrain,
		vector3 const& position,
		std::shared_ptr<fcl::CollisionGeometry<double>>&geom,
		std::shared_ptr<fcl::CollisionObject<double>> &co)
{
	Msg::verify(terrain,"non-terrain mesh is not supported yet");
	auto terrainPos=tree->VRMLbone(1).getLocalFrame().translation;
	auto lpos=position-terrainPos;
	vector3 normal;
	double height= terrain->height(vector2(lpos.x, lpos.z), normal);

	//printf("%s .y==%f\n", position.output().ptr(), height+0.02);
	lpos.y=height;
	// nx=d
	double d=normal%(lpos+terrainPos);

	geom.reset(new fcl::Halfspace<double>(toEigen(normal), d));
	co.reset(new fcl::CollisionObject<double>(geom, fcl::Transform3<double>::Identity()));
}

static void checkCollision(fcl::CollisionObject<double>* co1, 
		fcl::CollisionObject<double>* co2, 
		std::vector<fcl::Contact<double>> &contacts_new)
{
	fcl::CollisionRequest<double> collisionRequest(1, true);
	collisionRequest.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
	fcl::CollisionResult<double> collisionResult;

	fcl::collide(co1, co2, collisionRequest, collisionResult);
	collisionResult.getContacts(contacts_new);
}
inline static void addPoint2(OpenHRP::CollisionPointSequence& points,  OpenHRP::CollisionPoint& point , std::vector<fcl::Contact<double>> &contacts_new)
{
	ASSERT (contacts_new.size()==1);

	auto& c=contacts_new[0];

	point.position=toBase(c.pos); // center pos.
	point.normal=toBase(c.normal);
	point.idepth=c.penetration_depth;
	addPoint( points, point);
}
		inline static void addCornerPoints(OpenHRP::CollisionPointSequence& points,  fcl_ColObject* pCol, int subMesh1,
		VRMLloader* tree,
		OBJloader::Terrain* terrain, bool isMesh1=false
		)
{
	if(pCol->mesh->elements[subMesh1].elementType==OBJloader::Element::BOX)
	{
		auto& info=pCol->co_info[subMesh1];
		auto& tf=info.gtf;
		//info.gtf=fk.global(b)* info.tf;
		auto bone_tf=tf*info.tf.inverse();
		OpenHRP::CollisionPoint point;
		for (int kk=0; kk<8; kk++)
		{
			auto position=bone_tf*info.vertices(kk);
			auto terrainPos=tree->VRMLbone(1).getLocalFrame().translation;
			auto lpos=position-terrainPos;
			vector3 normal;
			vector3 surfacePoint;
			bool res= terrain->findClosestSurfacePoint(lpos, normal, surfacePoint);
			//double height= terrain->height(vector2(lpos.x, lpos.z), normal);
			//if(height>lpos.y)
			if(res)
			{
				//lpos.y=height;
				//point.position=lpos+terrainPos;
				point.position=surfacePoint+terrainPos;
				point.idepth=point.position.distance(position); //height-lpos.y;
																//
				if(isMesh1)
				{
					// box is mesh1
					point.normal=normal*-1;
					addPoint(points, point, 0);
				}
				else
				{
					// box is mesh2
					point.normal=normal;
					addPoint(points, point);
				}
			}
			//printf("%s .y==%f\n", position.output().ptr(), height+0.02);

#ifdef DEBUG_DRAW
			TString nameid;
			nameid.format("ball%d %d", subMesh1, kk);
			g_objectList->drawSphere(position*100,nameid.ptr(),"red", 3.0);
			nameid.format("ball2%d %d", subMesh1, kk);
			g_objectList->drawSphere((surfacePoint+terrainPos)*100,nameid.ptr(),"green", 3.0);
#endif
		}

	}
}



bool CollisionDetector_fcl::testIntersectionsForDefinedPairs( OpenHRP::CollisionSequence & collisions)
{
	//	counter2.start();
	bool flag=false;

	collisions.seq.resize(mPairs.size());
	collisions.pairs=&mPairs;


	fcl_ColObject* colobject[2];

#ifdef USE_BROADPHASE_CACHE
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

		// broad phase : error check
		if(colobject[0]->gb.getMaximum().x!=colobject[0]->gb.getMaximum().x)
			continue;

		// broad phase.
		if(!colobject[0]->gb.intersects(colobject[1]->gb))
			continue;

		broadphase_cache[ipair]=1;
		broadphase_cache_per_character(lp.charIndex[0], lp.charIndex[1])++;
	}
#endif

	for(int ipair=0; ipair<mPairs.size(); ipair++)
	{
		OpenHRP::LinkPair& lp=mPairs[ipair];


#ifdef USE_BROADPHASE_CACHE
		if (broadphase_cache[ipair]==0) continue;
#endif
		for(int i=0; i<2; i++)
			colobject[i]=m_col_objects[lp.charIndex[i]][lp.link[i]->treeIndex()];

		if(colobject[0]==NULL) continue;
		if(colobject[1]==NULL) continue;


#ifdef COL_DEBUG
		TString output;
		output.format("broad phase %s %s %s %s\n", colobject[0]->gb.getMaximum().output().ptr(),
				colobject[0]->gb.getMinimum().output().ptr(),
				colobject[1]->gb.getMaximum().output().ptr(),
				colobject[1]->gb.getMinimum().output().ptr());
		OutputToFile("broadPhase.txt", output.ptr());
#endif


		collisions[ipair].points.resize(0);

		for(int subMesh1=0; subMesh1<colobject[0]->co.size(); subMesh1++)
			for(int subMesh2=0; subMesh2<colobject[1]->co.size(); subMesh2++)
			{
				ccd_real_t depth;
				ccd_vec3_t dir, pos;
				int intersect=-1;

				fcl::CollisionRequest<double> collisionRequest(1, true);
				collisionRequest.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
				fcl::CollisionResult<double> collisionResult;
				auto& co1=colobject[0]->co[subMesh1];
				auto& co2=colobject[1]->co[subMesh2];
				fcl::collide(co1.co, co2.co, collisionRequest, collisionResult);
				std::vector<fcl::Contact<double>> contacts;
				collisionResult.getContacts(contacts);


				//printf("co %d %d\n", subMesh1, subMesh2);
				// narrow phase.
				if (contacts.size()>0)
				{
					Msg::verify(contacts.size()==1, "fcl1???");
					auto& contact=contacts[0];

					OpenHRP::CollisionPoint point;
					point.position=toBase(contact.pos); // center pos.
					point.normal=toBase(contact.normal);
					point.idepth=contact.penetration_depth;

					if(co1.isMesh || co2.isMesh)
					{
						std::shared_ptr<fcl::CollisionGeometry<double>> geom;
						std::shared_ptr<fcl::CollisionObject<double>> co;
						std::vector<fcl::Contact<double>> contacts_new;
						if (co2.isMesh)
						{
							//use lp.charIndex[i], lp.link[i]->treeIndex()];
							VRMLloader* tree=mTrees[lp.charIndex[1]];
							setHalfSpace(tree, tree->_terrain, point.position, geom, co);
							auto& co1=colobject[0]->co[subMesh1];
							checkCollision(co1.co, co.get(), contacts_new);

							if (contacts_new.size()==1) {
								addPoint2(collisions[ipair].points, point, contacts_new);
								flag=true;
								addCornerPoints(collisions[ipair].points, colobject[0],  subMesh1, tree, tree->_terrain, true); 
							}
						}
						else
						{
							VRMLloader* tree=mTrees[lp.charIndex[0]];
							setHalfSpace(tree, tree->_terrain, point.position, geom, co);
							auto& co2=colobject[1]->co[subMesh1];
							checkCollision(co.get(), co2.co, contacts_new);

							if (contacts_new.size()==1) {
								addPoint2(collisions[ipair].points, point, contacts_new);
								flag=true;
								addCornerPoints(collisions[ipair].points, colobject[1],  subMesh2, tree, tree->_terrain, false); 
							}
						}
						continue;
					}

					addPoint( collisions[ipair].points, point);


					//Msg::output("intersect", "1");
					flag=true;

				}
				//Msg::output("intersect", "0");
#ifdef DEBUG_DRAW
				//if(!g_objectList) g_objectList=new ObjectList();

				//	TString nameid;
				//	nameid.format("%d %d %d %d", lp.charIndex[0], lp.link[0]->treeIndex(),
				//			lp.charIndex[1], lp.link[1]->treeIndex());
				//	if(intersect)
				//		g_objectList->drawSphere(ToBase(pos)*100,nameid.ptr(),"red", 2.0);
				//	else
				//		g_objectList->drawSphere(ToBase(pos)*100,(nameid+"last").ptr(),"blue", 2.0);
#endif
			}
	}


	return flag;
}


void CollisionDetector_fcl::rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result)
{
}
namespace OpenHRP
{
	CollisionDetector* createCollisionDetector_fcl()
	{
		printf("fcl collision detector created!\n");
		return new CollisionDetector_fcl();
	}
}

bool fcl_ColObject::isLargeBox(int isubMesh, double thr)
{
	// large enough to contain corner spheres (see CollisionDetector_fcl_LBS.cpp).
	auto& info=co_info[isubMesh];
	auto& esize=info.elementSize;
	return info.elementType==OBJloader::Element::BOX && esize.x>thr && esize.y>thr && esize.z>thr;
}
