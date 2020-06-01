// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
//
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/** @file CollisionDetector/server/CollisionDetector_impl.cpp
 * Implementation of CollisionDetector_impl and CollisionDetectorFactory_impl
 *
 * @version 0.2
 * @date 2002/01/15
 *
 */

#include "../physicsLib.h"
#include <time.h>
#include <iostream>
#include <string>
#include <vector>
#include "../MainLib/OgreFltk/pldprimskin.h"

#include "../BaseLib/math/tvector.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "CollisionDetector_impl.h"
using namespace std;
using namespace OpenHRP;


class CdModelCache;
class CdModelSet;


CdScene* createCdScene();
CdModelCache* createCdModelCache();
CdModelCache* getCachedModel(CdCharCache* cache_, const char* url);
CdModelSet* createModelSet(int i);
void addTriangle(CdModelSet*  modelSet, double* x, double* y, double* z);
void endModel(CdModelSet*  modelSet);
void _addModel(CdModelCache* cachedModel, const char* jName, CdModelSet* modelSet);
void addChar(CdCharCache* cache_, const char* url, CdModelCache* cachedModel);
void addChar(CdScene* scene_, const char* charName, CdModelCache* cachedModel);
//#define COLLISIONDETECTOR_DEBUG

CollisionDetector_impl::CollisionDetector_impl
(
  CdCharCache*         cache
 ) 
	:OpenHRP::CollisionDetector(),
	CollisionDetector_impl_hidden()
{
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetector_impl::CollisionDetector_impl()" << endl;
#endif
    cache_ = cache;
    scene_ = createCdScene();
}

CollisionDetector_impl::~CollisionDetector_impl()
{
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetector_impl::~CollisionDetector_impl()" << endl;
#endif
}

void CollisionDetector_impl::addModel(const char* charName,
                                      CharacterInfo const& cinfo)
{
	CollisionDetector::addModel(charName, cinfo);
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetector_impl::addModel() " << charName <<endl;
#endif

#define READTRI_COUNT 1000

    CdModelCache* cachedModel;

    TString url = cinfo.url;
	cachedModel=getCachedModel(cache_, url);
	if(!cachedModel)
	{
        cerr << "creating Cd object..." << endl;

        LinkInfoSequence const& jList = cinfo.links;
        cachedModel = createCdModelCache();

        double p[3][3];
        TString jName;


        for(unsigned int i=0; i < jList.size();++i){
            jName = jList[i].name;
            int ntri = 0;

			CdModelSet* modelSet = 0;
			OBJloader::Mesh* mesh=jList[i].mesh;
			
			if(mesh && mesh->numFace()){

                modelSet = createModelSet(i);
	
				vector3 v;
				index3 index;
				for(int tri=0; tri<mesh->numFace(); tri++)
				{
					index=mesh->getFace(tri).getIndex(OBJloader::Buffer::VERTEX);
					v=mesh->getVertex(index(0));
				    p[0][0]=v.x;
                    p[0][1]=v.y;
                    p[0][2]=v.z;
					v=mesh->getVertex(index(1));
                    p[1][0]=v.x;
                    p[1][1]=v.y;
                    p[1][2]=v.z;
					v=mesh->getVertex(index(2));
                    p[2][0]=v.x;
                    p[2][1]=v.y;
                    p[2][2]=v.z;
                    addTriangle(modelSet,p[0],p[1],p[2]);
					ntri++;
                }
                endModel(modelSet);
			}

			_addModel(cachedModel,jName, modelSet);
			
            cerr << jName << " has "<< ntri << " tris." << endl;
        }

        cerr << "finished." << endl;

        addChar(cache_, url,cachedModel);

    }

	addChar(scene_, charName,  cachedModel);
}

void CollisionDetector_impl::addCollisionPair(const LinkPair& colPair,
                                              bool convexsize1,
                                              bool convexsize2)
{
	CollisionDetector::addCollisionPair(colPair, convexsize1, convexsize2);

    const char* charName1 = colPair.charName1;
    const char* charName2 = colPair.charName2;
    const char* jointName1 = colPair.linkName1;
    const char* jointName2 = colPair.linkName2;
	_addCollisionPair(charName1, charName2, jointName1, jointName2);

}

void CollisionDetector_impl::removeCollisionPair(const LinkPair& colPair)
{
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetector_impl::removeCollisionPair()" << endl;
#endif
}



void CollisionDetector_impl::_setCharacterData(std::vector<DynamicsSimulator::Character*> const& characters)
{
    if (joints.size() == 0){ //first time
        for (unsigned int i=0; i < characters.size(); i++){
            CdChar *rChar = getChar(characters[i]->skeleton->name.ptr());
            if (rChar){
				for(int j=1, nj=characters[i]->skeleton->numBone(); j<nj; j++)
				{
					VRMLTransform& b=characters[i]->skeleton->VRMLbone(j);
                    CdJoint *rJoint = getJoint(rChar,b.HRPjointIndex(b.numHRPjoints()-1));	// the last joint in a bone has a collision shape.
                    if (rJoint){
                        joints.push_back(rJoint);
                    }else{
                        joints.push_back(NULL);
                    }
                }
            } else {
                cerr << "CollisionDetector_impl::_setCharacterData : Character not found(" << characters[i]->skeleton->name.ptr()<< ")" << endl;
            }
        }
    }

    vector<CdJoint *>::iterator it = joints.begin();
	
    for(unsigned int i = 0; i < characters.size(); i++){
		for(int j=1; j<characters[i]->skeleton->numBone(); j++, it++)
		{
			VRMLTransform& b=characters[i]->skeleton->VRMLbone(j);
			CdJoint* cdJoint = *it;
            if (cdJoint){

				matrix4 Ro=characters[i]->chain->global(b);
				setConfig(cdJoint, 
						Ro. _14, Ro. _24, Ro. _34, 
						Ro. _11, Ro. _12, Ro. _13,
						Ro. _21, Ro. _22, Ro. _23,
						Ro. _31, Ro. _32, Ro. _33);
			}
		}		
    }
}


#include "../BaseLib/utility/QPerformanceTimer.h"

QPerformanceTimerCount counter(100, "collision detection");


bool CollisionDetector_impl::queryContactDeterminationForDefinedPairs(std::vector<DynamicsSimulator::Character*> const& characters, CollisionSequence & collisions)
{
//	BEGIN_TIMER(query);

	counter.start();
	_setCharacterData(characters);

    long unsigned int numPair = getNumPair();
    
    collisions.resize(numPair);
    bool flag = false;
    for(long unsigned int pCount = 0 ; pCount < numPair ; ++pCount){
        CdCheckPair* rPair = getCheckPair(pCount);
        _contactDetermination(rPair, collisions[pCount]);
        if (collisions[pCount].points.size() > 0) flag = true;
    }

    //cerr << "CollisionDetector_impl::checkCollision(2)" << endl;

//	END_TIMER(query);
	counter.stop();

    return flag;
}

#ifdef INCLUDE_OPCODE_DETECTOR


CollisionDetectorFactory::CollisionDetectorFactory
()
{
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetectorFactory_impl::CollisionDetectorFactory_impl()" << endl;
#endif
	data=createCdCharCache();
}

CollisionDetectorFactory::~CollisionDetectorFactory()
{
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetectorFactory_impl::~CollisionDetectorFactory_impl()" << endl;
#endif
  deleteCdCharCache( data);
}

CollisionDetector*
CollisionDetectorFactory::create()
{
#ifdef COLLISIONDETECTOR_DEBUG
    cerr << "CollisionDetectorFactory_impl::create()" << endl;
#endif

    CollisionDetector* collisionDetectorImpl = new CollisionDetector_impl((CdCharCache*)data);
    return collisionDetectorImpl ;
}
#endif

		void CollisionDetector_impl::setWorldTransformations(int charIndex, BoneForwardKinematics const& fk)
{
	ASSERT(0);
}
		bool CollisionDetector_impl::testIntersectionsForDefinedPairs(CollisionSequence & collisions)
{
	ASSERT(0);
	return false;
}
#include "utilities.h"

void CollisionDetector_impl::_contactDetermination(CdCheckPair* rPair, Collision&  out_collision)
{
    int ret;

	collision_data *cdata;
    collide(rPair,&ret, &cdata);

    int npoints = 0;
    for (int i = 0; i < ret; i++) {
        for (int j = 0; j<cdata[i].num_of_i_points; j++){
            if (cdata[i].i_point_new[j]) npoints ++;
        }
    }
    if (npoints > 0){
        out_collision.points.resize(npoints);

        int idx = 0;
        for (int i = 0; i < ret; i++) {
			collision_data& cd = cdata[i];
            for (int j=0; j < cd.num_of_i_points; j++){
                if (cd.i_point_new[j]){
					CollisionPoint& point = out_collision.points[idx];
                    for(int k=0; k < 3; k++){
                        point.position[k] = cd.i_points[j][k];
					}
                    for(int k=0; k < 3; k++){
                        point.normal[k]   = cd.n_vector[k];
                    }
                    point.idepth = cd.depth;
                    idx++;
                }
            }
        }
    }
    //if (rPair->joint_[0]){
    //    out_collision.pair.charName1 = rPair->joint_[0]->parent_->name_.c_str();    // 긬긚뼹1
    //    out_collision.pair.linkName1 = rPair->joint_[0]->name_.c_str();    // 긬긚뼹1
    //}
    //if (rPair->joint_[1]){
    //    out_collision.pair.charName2 = rPair->joint_[1]->parent_->name_.c_str();    // 긬긚뼹2
    //    out_collision.pair.linkName2 = rPair->joint_[1]->name_.c_str();    // 긬긚뼹2
    //}
}
