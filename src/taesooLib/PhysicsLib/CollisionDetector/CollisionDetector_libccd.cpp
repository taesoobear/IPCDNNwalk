/** @file CollisionDetector/server/CollisionDetector_libccd.cpp
 * by taesoo
 */
#include "../../physicsLib.h"

#include "CollisionDetector_libccd.h"
#include "../MainLib/OgreFltk/pldprimskin.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/motion/VRMLloader_internal.h"

//#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../MainLib/OgreFltk/objectList.h"
static ObjectList* g_objectList=NULL;

#endif

//#define USE_BROADPHASE // does not work perfectly. turned off for now.
extern "C" {
#include "libccd/ccd/ccd.h"
}

using namespace std;
using namespace OpenHRP;

static double collisionMargin=0.01;
vector3 ToBase(ccd_vec3_t & v)
{
	return vector3(
			(double)v.v[0], 
			(double)v.v[1],
			(double)v.v[2]);
}
void copy(vector3 const& vv, ccd_vec3_t & v)
{
	v.v[0]=vv.x;
	v.v[1]=vv.y;
	v.v[2]=vv.z;
}
void copy(quater const& vv, ccd_quat_t & v)
{
	v.q[3]=vv.w;
	v.q[0]=vv.x;
	v.q[1]=vv.y;
	v.q[2]=vv.z;
}

void ccdSupportConvex(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);

int checkCollision(ccd_real_t& depth, 
				ccd_vec3_t &dir, 
				ccd_vec3_t &pos,
			CollisionDetector_libccd::ColObject* co1, int subMesh1,	
			CollisionDetector_libccd::ColObject* co2, int subMesh2)
{
	void* obj1=&co1->co[subMesh1].box;
	void* obj2=&co2->co[subMesh2].box;

	ccd_t ccd;
	CCD_INIT(&ccd);
	ccd.support1 = ccdSupport;
	ccd.support2 = ccdSupport;
	ccd.center1  = ccdObjCenter;
	ccd.center2  = ccdObjCenter;

	ccd.max_iterations = 1000;     // maximal number of iterations
	ccd.epa_tolerance  = 0.0001;  // maximal tolerance fro EPA part
	//ccd.max_iterations = 1000;     // maximal number of iterations
	//ccd.epa_tolerance  = 0.00001;  // maximal tolerance fro EPA part

	CollisionDetector_libccd::ColObject::Info* pinfo1=(CollisionDetector_libccd::ColObject::Info*)((ccd_box_t*)obj1)->_data;
	CollisionDetector_libccd::ColObject::Info* pinfo2=(CollisionDetector_libccd::ColObject::Info*)((ccd_box_t*)obj2)->_data;

	int intersect=-1;

	/*
	// libccd GJK does not work well when one of the element is too large. e.g. floor plane. 
	// when using MPRpenetration, this is unnecessary.
	if(pinfo1->elementType==OBJloader::Element::PLANE)
	{
	if(correctPlaneCollisionInfo(pinfo1, pinfo2, point))
	intersect=0;
	}
	else if(pinfo2->elementType==OBJloader::Element::PLANE)
	{
	if(correctPlaneCollisionInfo(pinfo2, pinfo1, point))
	{
	intersect=0;
	point.normal*=-1;
	}
	}
	else*/
		//intersect = ccdGJKPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
	intersect = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos); 
	return intersect;
}


CollisionDetector_libccd::CollisionDetector_libccd() 
{
#ifdef COL_DEBUG
	FILE* fl=fopen("broadPhase.txt", "wt");
	fclose(fl);
#endif

}

CollisionDetector_libccd::~CollisionDetector_libccd()
{
}

int CollisionDetector_libccd::addModel(VRMLloader* loader)
{
	int ichar=	CollisionDetector::addModel(loader);
	_addModel(loader);
	return ichar;
}

void CollisionDetector_libccd_init_ColObject(OpenHRP::CollisionDetector_libccd::ColObject* pCol, HRP_SHAPE* shape)
{
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

		if (e.elementType==OBJloader::Element::BOX)
		{
			ccd_box_t box;
			box.type = CCD_OBJ_BOX;
			copy(vector3(0,0,0), box.pos);
			copy(quater(1,0,0,0), box.quat);
			box._data=NULL;
			box.x=e.elementSize.x;
			box.y=e.elementSize.y;
			box.z=e.elementSize.z;
			pCol->co[subMesh].box=box;
			pCol->co[subMesh].box._data=(void*)&info;
		}
		else if((e.elementType==OBJloader::Element::ELLIPSOID && isSimilar(e.elementSize.x, e.elementSize.y) && isSimilar(e.elementSize.z, e.elementSize.x))
				|| e.elementType==OBJloader::Element::SPHERE)
		{
			info.elementType=OBJloader::Element::SPHERE;
			ccd_sphere_t sphere;
			sphere.type = CCD_OBJ_SPHERE;
			copy(vector3(0,0,0), sphere.pos);
			copy(quater(1,0,0,0), sphere.quat);
			sphere._data=NULL;
			sphere.radius=e.elementSize.x;
			pCol->co[subMesh].sphere=sphere;
			pCol->co[subMesh].sphere._data=(void*)&info;
		}
		else if(e.elementType==OBJloader::Element::CYLINDER)
		{
			ccd_cyl_t cylinder;
			cylinder.type = CCD_OBJ_CYL;
			copy(vector3(0,0,0), cylinder.pos);
			copy(quater(1,0,0,0), cylinder.quat);
			cylinder._data=NULL;
			cylinder.radius=e.elementSize.x*0.5;
			cylinder.height=e.elementSize.y;
			pCol->co[subMesh].cyl=cylinder;
			pCol->co[subMesh].cyl._data=(void*)&info;
		}
		else if(e.elementType==OBJloader::Element::CAPSULE)
		{
			ccd_cap_t capsule;
			capsule.type = CCD_OBJ_CAP;
			copy(vector3(0,0,0), capsule.pos);
			copy(quater(1,0,0,0), capsule.quat);
			capsule._data=NULL;
			capsule.radius=e.elementSize.x*0.5;
			capsule.height=e.elementSize.y*0.5; // finally fixed bug!!!
			pCol->co[subMesh].cap=capsule;
			pCol->co[subMesh].cap._data=(void*)&info;
		}
		else 
		{
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
				ccd_general_t gen;
				gen.type = CCD_OBJ_GENERAL;
				copy(vector3(0,0,0), gen.pos);
				copy(quater(1,0,0,0), gen.quat);
				gen._data=NULL;

				gen.ccdSupport=&ccdSupportConvex;
				pCol->co[subMesh].gen=gen;
				pCol->co[subMesh].gen._data=(void*)&info;
			}
		}
	}
}
void CollisionDetector_libccd::_addModel(VRMLloader* loader)
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
			CollisionDetector_libccd_init_ColObject(_object[b], shape);
		}
	}
}


void CollisionDetector_libccd::addCollisionPair(const LinkPair& colPair,
		bool convexsize1,
		bool convexsize2)
{
	CollisionDetector::addCollisionPair(colPair, convexsize1, convexsize2);

}

void CollisionDetector_libccd::removeCollisionPair(const LinkPair& colPair)
{
	ASSERT(0);
}




#include "../BaseLib/utility/QPerformanceTimer.h"

//QPerformanceTimerCount counter2(100, "collision detection");

void CollisionDetector_libccd::setWorldTransformations(int charIndex, BoneForwardKinematics const& fk)
{
	int i=charIndex;
	std::vector<ColObject*>& col_objects=m_col_objects[i];
#ifdef USE_BROADPHASE
	matrix4 gtransf;
#endif
	for(int b=1, nb=mTrees[i]->numBone(); b<nb; b++)
	{
		if(col_objects[b])
		{
			ColObject& co=*col_objects[b];

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

				ColObject::OBJ_T& obj=co.co[subMesh];
				CollisionDetector_libccd::ColObject::Info& info=co.co_info[subMesh];


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
				obj.box.pos.v[0]=t.x;
				obj.box.pos.v[1]=t.y;
				obj.box.pos.v[2]=t.z;
				obj.box.quat.q[0]=r.x;
				obj.box.quat.q[1]=r.y;
				obj.box.quat.q[2]=r.z;
				obj.box.quat.q[3]=r.w;
			}
		}
	}
}
bool CollisionDetector_libccd::testIntersectionsForDefinedPairs( CollisionSequence & collisions)
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

#ifdef USE_BROADPHASE
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
				ccd_real_t depth;
				ccd_vec3_t dir, pos;
				int intersect=-1;
				double radialMargin = lp.margin+collisionMargin;

				intersect=checkCollision(depth, dir, pos, colobject[0], subMesh1, colobject[1], subMesh2);
				//printf("co %d %d\n", subMesh1, subMesh2);
				// narrow phase.
				if (intersect ==0)
				{
					CollisionPoint point;
					point.position=ToBase(pos); // center pos.
					point.normal=ToBase(dir);
					point.idepth=depth;
					// to make the results identical to bullet detector:
					point.position=point.position-point.normal*point.idepth*0.5;
					point.inode[0]=-1; // meaning a rigidbody point 
					point.inode[1]=-1; // meaning a rigidbody point 


					//Msg::output("intersect", "1");
					flag=true;

					collisions[ipair].points.resize(collisions[ipair].points.size()+1);
					collisions[ipair].points.back()=point;
				}
				//Msg::output("intersect", "0");
#ifdef DEBUG_DRAW
				if(!g_objectList) g_objectList=new ObjectList();

					TString nameid;
					nameid.format("%d %d %d %d", lp.charIndex[0], lp.link[0]->treeIndex(),
							lp.charIndex[1], lp.link[1]->treeIndex());
					if(intersect)
						g_objectList->drawSphere(ToBase(pos)*100,nameid.ptr(),"red", 2.0);
					else
						g_objectList->drawSphere(ToBase(pos)*100,(nameid+"last").ptr(),"blue", 2.0);
#endif
			}
	}


	return flag;
}

bool CollisionDetector_libccd::CollisionCheckMesh(CollisionSequence& collisions, std::string chekmesh, std::string skipmesh)
{
	//	counter2.start();
	bool flag=false;

	collisions.seq.resize(mPairs.size());
	collisions.pairs=&mPairs;

	ColObject* colobject[2];

	for(int ipair=0; ipair<mPairs.size(); ipair++)
	{
		LinkPair& lp=mPairs[ipair];

		bool check = false;
		if ( chekmesh.compare(lp.charName1) == 0) 
		{
			if(skipmesh.compare(lp.charName2) != 0)
				check = true;
		}
		else if(chekmesh.compare(lp.charName2) == 0)
		{
			if(skipmesh.compare(lp.charName1) != 0)
				check = true;
		}

		if(check == true)
		{
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

#ifdef USE_BROADPHASE
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
				ccd_real_t depth;
				ccd_vec3_t dir, pos;
				int intersect=-1;
				double radialMargin = lp.margin+collisionMargin;
				intersect=checkCollision(depth, dir, pos, colobject[0], subMesh1, colobject[1], subMesh2);

				if (intersect ==0)
				{
					CollisionPoint point;

					point.position=ToBase(pos); // center pos.
					point.normal=ToBase(dir);
					point.idepth=depth;
					point.position=point.position-point.normal*point.idepth*0.5;
					point.inode[0]=-1; // meaning a rigidbody point 
					point.inode[1]=-1; // meaning a rigidbody point 
					flag=true;

					collisions[ipair].points.resize(collisions[ipair].points.size()+1);
					collisions[ipair].points.back()=point;
				}

#ifdef DEBUG_DRAW
				if(!g_objectList) g_objectList=new ObjectList();

					TString nameid;
					nameid.format("%d %d %d %d", lp.charIndex[0], lp.link[0]->treeIndex(),
							lp.charIndex[1], lp.link[1]->treeIndex());
					if(intersect)
						g_objectList->drawSphere(ToBase(pos)*100,nameid.ptr(),"red", 2.0);
					else
						g_objectList->drawSphere(ToBase(pos)*100,(nameid+"last").ptr(),"blue", 2.0);
#endif
			}
		}
	}
	return flag;
}

void CollisionDetector_libccd::rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result)
{
}
namespace OpenHRP
{
	CollisionDetector* createCollisionDetector_libccd()
	{
		printf("libccd collision detector created!\n");
		return new CollisionDetector_libccd();
	}
	bool CollisionCheck(CollisionDetector &s, CollisionSequence& collisions, std::string chekmesh, std::string skipmesh)
	{
		CollisionDetector_libccd& a = (CollisionDetector_libccd&)s;
		return a.CollisionCheckMesh(collisions, chekmesh, skipmesh);
	}
}

void ccdSupportConvex(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    ccd_general_t *obj = (ccd_general_t *)_obj;
    ccd_vec3_t dir, p;
    ccd_real_t maxdot, dot;
    ccd_quat_t qinv;
    size_t i;
    double *curp;

    ccdVec3Copy(&dir, _dir);
    ccdQuatInvert2(&qinv, &obj->quat);
    ccdQuatRotVec(&dir, &qinv);

    maxdot = -CCD_REAL_MAX;
	CollisionDetector_libccd::ColObject::Info* pinfo=(CollisionDetector_libccd::ColObject::Info*)obj->_data;
    curp = &pinfo->vertices[0].x;
	int pointcount=pinfo->vertices.size();
    for (i = 0; i < pointcount; i++, curp += 3){
        ccdVec3Set(&p, curp[0], curp[1], curp[2]);
        dot = ccdVec3Dot(&dir, &p);
        if (dot > maxdot){
            ccdVec3Copy(v, &p);
            maxdot = dot;
        }
    }
    // transform support vertex
    ccdQuatRotVec(v, &obj->quat);
    ccdVec3Add(v, &obj->pos);
}
/*
danfis says:
The ccdSupportConvex() function seems ok -- the support point is the vertex from the polyhedron that is furthest along the direction vector (dot product says you the length of projection on the direction vector)... But if you are unsure, try some literature.
*/
