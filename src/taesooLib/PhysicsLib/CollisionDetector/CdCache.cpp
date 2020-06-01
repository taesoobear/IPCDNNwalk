// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/** @file CollisionDetector/server/CdCache.cpp
 *
 *
 * @version 0.2
 * @date 2002/02/28
 * @version 0.3 support OPCODE
 * @date 2006/02/28
 * @version 0.4
 * @date 2006/06/30
 *
 */
#include "StdAfxColDet.h"

#include <iostream>
#include <string>
#include <map>
#include "utilities.h"
#include "CdCache.h"
#include "Opcode.h"

//#define CDCACHE_DEBUG

CdModelSet::CdModelSet()
{
	model_[0] = new Opcode::Model();
	model_[1] = new Opcode::Model();
  
	iMesh_ = new Opcode::MeshInterface();
  
	trisCount_ = 0;
	pointsCount_ = 0;
	trisCountAlloced_   = 3;
	pointsCountAlloced_ = trisCountAlloced_ * 3; 
	points_ = new Point[pointsCountAlloced_];
	tris_   = new IceMaths::IndexedTriangle[trisCountAlloced_];
}
CdModelSet::~CdModelSet()
{
    delete model_[0];
    delete model_[1];
    delete[] tris_;
    delete[] points_;
    delete iMesh_;
    trisCount_   = 0;
    pointsCount_ = 0;
}
void CdModelSet::addTriangle(const double *p1, 
							 const double *p2, 
							 const double *p3)
{
	if(pointsCount_ == pointsCountAlloced_){
		udword n = pointsCountAlloced_*2;
		if(n == 0) n = 3;
		Point *p = new Point[n];
		if(!p){puts("NOT ENOUGH MEMORY"); return;}
		udword i;
		for(i=0; i<pointsCount_; i++)
			p[i] = points_[i];
		delete[] points_;
		points_ = p;
		pointsCountAlloced_ = n;
	}
	points_[pointsCount_+0].Set((float)p1[0], (float)p1[1], (float)p1[2]);
	points_[pointsCount_+1].Set((float)p2[0], (float)p2[1], (float)p2[2]);
	points_[pointsCount_+2].Set((float)p3[0], (float)p3[1], (float)p3[2]);

	if(trisCount_ == trisCountAlloced_){
		udword n = trisCountAlloced_ *2;
		if(n == 0)   n = 1;
		IceMaths::IndexedTriangle *t = new IceMaths::IndexedTriangle[n];
		if(!t){puts("NOT ENOUGH MEMORY");  return;}
		udword i;
		for(i=0; i<trisCount_; i++)
			t[i] = tris_[i];
		delete[] tris_;
		tris_ = t;
		trisCountAlloced_ = n;
	}
	for(udword i=0; i<3; i++)
		tris_[trisCount_].mVRef[i] = pointsCount_+i;
	pointsCount_+=3;
	trisCount_++;
}

void CdModelSet::endModel()
{

	iMesh_->SetPointers(tris_, points_);
	iMesh_->SetNbTriangles(trisCount_);
	iMesh_->SetNbVertices(pointsCount_);
	OPCC_.mIMesh = iMesh_;
	OPCC_.mNoLeaf = false;
	OPCC_.mQuantized = false;
	OPCC_.mKeepOriginal = false;
	model_[0]->Build(OPCC_);
	model_[1]->Build(OPCC_);
}

CdModelCache::CdModelCache()
{
}

CdModelCache::~CdModelCache()
{
    cout << "CdModelCache::~CdModelCache()" <<endl;
    map<string,CdModelSet*>::iterator it;
    for(it = map_.begin(); it != map_.end(); ++it){
        delete it->second;
    }
    map_.clear();
}

int CdModelCache::addModel(const char* name,CdModelSet* obj)
{
#ifdef CDCACHE_DEBUG
    cout << "CdModelCache::addModel(" << name << ")" << endl;
#endif
    string name_s = name;
    map_[name] = obj;
    return 0;
}

int CdModelCache::removeModel(const char* name)
{
    cout << "CdModelCache::removeModel(" << name << ")" << endl;
    string name_s = name;
    map<string,CdModelSet*>::iterator it = map_.find(name);
    delete it->second;
    map_.erase(it);
    return 0;
}

CdModelSet* CdModelCache::getModel(const char* name)
{
    string name_s = name;
    return map_[name];
}

vector<string> CdModelCache::getNameList()
{
    vector<string> list;
    map<string,CdModelSet*>::iterator it;
    for(it = map_.begin(); it != map_.end(); ++it){
        list.push_back(it->first);
    }
    
    return list;
}

int CdModelCache::exist(const char* name)
{
    map<string,CdModelSet*>::iterator it;
    it = map_.find(name);
    return (it != map_.end());
}



CdCharCache::CdCharCache()
{
}

CdCharCache::~CdCharCache()
{
    cout << "CdCharCache::~CdCharCache()" <<endl;
    removeAllChar();
}

int CdCharCache::addChar(const char* name,CdModelCache* obj)
{
    cout << "CdCharCache::addChar(" << name << ")" << endl;
    string name_s = name;
    map_[name] = obj;
    return 0;
}

int CdCharCache::removeChar(const char* name)
{
    cout << "CdCharCache::removeChar(" << name << ")" << endl;
    if(exist(name)){
        string name_s = name;
        map<string,CdModelCache*>::iterator it = map_.find(name);
        delete it->second;
        map_.erase(it);
    }
    return 0;
}

int CdCharCache::removeAllChar()
{
    cout << "CdCharCache::removeAllChar() "<< endl;
    map<string,CdModelCache*>::iterator it;
    for(it = map_.begin(); it != map_.end(); ++it){
        removeChar(it->first.c_str());
    }
    map_.clear();
    return 0;
}

CdModelCache* CdCharCache::getChar(const char* name)
{
    string name_s = name;
    return map_[name];
}

int CdCharCache::exist(const char* name)
{
    map<string,CdModelCache*>::iterator it;
    it = map_.find(name);
    return (it != map_.end());
}
