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
/** @file CollisionDetector/server/CdScene.cpp
 *
 *
 * @version 0.2  
 * @date 2002/02/28
 * @version 0.3  support OPCODE
 * @date 2006/02/28
 * @version 0.4 
 * @date 2006/06/30
 *
 */
#include "StdAfxColDet.h"

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <math.h>

#include "CdCache.h"
#include "CdScene.h"

CdCheckPair::CdCheckPair(CdJoint* j1, CdJoint* j2)
{
    joint_[0] = j1;
    joint_[1] = j2;
}

CdCheckPair::~CdCheckPair()
{
}

// delete cPair when it becomes needless
int CdCheckPair::collide(int* num, collision_data** cPair, int flag)
{
    int ret = collide(num, flag);
    *cPair = CdWrapper::getCollisionData(); // get collision information
    return ret;
}

int CdCheckPair::collide(int* num,int flag)
{
    if (!joint_[0] || !joint_[1]) {
        *num = 0;
        return 0;
    }

    try {
        CdWrapper::cdCollide(
                             joint_[0]->rotation_,
                             joint_[0]->translation_,
                             joint_[0]->model_->model_[0],
                             joint_[1]->rotation_,
                             joint_[1]->translation_,
                             joint_[1]->model_->model_[1],
                             flag
                             );
        *num = CdWrapper::getNumContacts();   // number of colliding triangles
        return 0;
    } catch (CdWrapperException ex) {
        cout << "############CdWrapperException" << endl;
        return -1;
    }
}



CdScene::CdScene()
{
}

CdScene::~CdScene()
{
    cout << "CdScene::~CdScene()" <<endl;
    map<string, CdChar*>::iterator it;
    for(it = map_.begin(); it != map_.end(); it++){
        delete it->second;
    }
    map_.clear();
}

int CdScene::addChar(const char* name, CdChar* obj)
{
    cerr << "CdScene::addChar(" << name << ")" << endl;
    map_[name] = obj;
    return 0;
}

int CdScene::removeChar(const char* name)
{
    map_.erase(map_.find(name));
    //delete
    return 0;
}

CdChar* CdScene::getChar(const char* name)
{
    return map_[name];
}

int CdScene::exist(const char* name)
{
    map<string,CdChar*>::iterator it;
    it = map_.find(name);
    return (it != map_.end());
}

void CdScene::addCheckPair(CdCheckPair* cPair)
{
    pairs_.push_back(cPair);
}


CdCheckPair* CdScene::getCheckPair(int i)
{
    return pairs_[i];
}

int CdScene::getNumCheckPairs()
{
    return pairs_.size();
}


CdJoint::CdJoint(CdModelSet* model, const char* name, CdChar* parent)
{
    model_ = model;
    name_ = name;
    parent_ = parent;
    
    translation_[0] = 0;
    translation_[1] = 0;
    translation_[2] = 0;
    
    rotation_[0][0] = 1.0; rotation_[0][1] = 0.0; rotation_[0][2] = 0.0;
    rotation_[1][0] = 0.0; rotation_[1][1] = 1.0; rotation_[1][2] = 0.0;
    rotation_[2][0] = 0.0; rotation_[2][1] = 0.0; rotation_[2][2] = 1.0;
}

CdJoint::~CdJoint()
{
    cout << "CdJoint::~CdJoint("  << name_ << ")" << endl;
}

void CdJoint::setTransform(const double t[3],const double q[4])
{
    int i;
    for(i=0;i<3;i++) translation_[i] = t[i];

    double xx = q[1]*q[1];
    double yy = q[2]*q[2];
    double zz = q[3]*q[3];
    double wx = q[0]*q[1];
    double xy = q[1]*q[2];
    double yz = q[2]*q[3];
    double wy = q[0]*q[2];
    double xz = q[1]*q[3];
    double wz = q[0]*q[3];
    
    rotation_[0][0] = 1-2*(yy + zz);
    rotation_[0][1] =   2*(xy - wz);
    rotation_[0][2] =   2*(wy + xz);
    rotation_[1][0] =   2*(wz + xy);
    rotation_[1][1] = 1-2*(xx + zz);
    rotation_[1][2] =   2*(yz - wx);
    rotation_[2][0] =   2*(xz - wy);
    rotation_[2][1] =   2*(wx + yz);
    rotation_[2][2] = 1-2*(xx + yy);
}                     

CdChar::CdChar(CdModelCache* model, const char* name)
{
    
    cerr << "CdChar::CdChar(" << name << ")" << endl;
    
    vector<string> joints = model->getNameList();
	int n = joints.size();
	linkIndexToCdJoint.resize(n, 0);
    
    for(int i=0 ; i < n ; i++){
		CdJoint* joint = 0;
		CdModelSet* modelSet = model->getModel(joints[i].c_str());
		if(modelSet){
			joint = new CdJoint(modelSet, joints[i].c_str(), this);

			unsigned int linkIndex = modelSet->linkIndex;
			if(linkIndex >= linkIndexToCdJoint.size()){
				linkIndexToCdJoint.resize(linkIndex+1, 0);
			}
			linkIndexToCdJoint[linkIndex] = joint;
			
			cerr << "add Joint to CdChar : " <<  joints[i] << endl;
		}
        map_[joints[i]] = joint;
    }
    
    name_ = name;
    
}

CdChar::~CdChar()
{
    cout << "CdChar::~CdChar() " << name_ << endl;
    // delete joints
    map<string,CdJoint*>::iterator it;
    for(it = map_.begin(); it != map_.end(); it++){
        delete it->second;
    }
    map_.clear();
}

CdJoint* CdChar::getJoint(const char* name)
{
    return map_[name];
}

CdJoint* CdChar::getJoint(int linkIndex)
{
    return linkIndexToCdJoint[linkIndex];
}

