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
/** @file CollisionDetector/server/CdScene.h
 *
 * This file defines classes that represent relationship between current model states to OPCODE objects
 *
 * @version 0.2
 * @date 2002/02/28
 * @version 0.3  support OPCODE
 * @date 2006/02/28
 * @version 0.4  
 * @date 2006/06/30
 *
 */
#ifndef CD_SCENE_H
#define CD_SCENE_H

#include <string>
#include <vector>
#include <map>
#include "CdCache.h"
#include "CdWrapper.h"

class CdChar;

class CdJoint
{
public:
	DllExport CdJoint(CdModelSet* model, const char* name, CdChar* parent);
	DllExport ~CdJoint();
        
	void setTransform(const double t[3],const double q[4]);
	double translation_[3];
	double rotation_[3][3];
        
	string name_;
	CdModelSet* model_;
	CdChar* parent_;
};

class CdCheckPair
{
public:
	DllExport CdCheckPair(CdJoint* j1,CdJoint* j2);
	DllExport ~CdCheckPair();
	DllExport int collide(int* num, collision_data** cPair, int flag = CD_ALL_CONTACTS);
	DllExport int collide(int* num, int flag = CD_ALL_CONTACTS);
        
	CdJoint* joint_[2];
};

class CdChar
{
public:
	DllExport CdChar(CdModelCache* model, const char* name);
	DllExport ~CdChar();
	DllExport CdJoint* getJoint(const char* name);
	DllExport CdJoint* getJoint(int linkIndex);
	string name_;
private:
	vector<CdJoint*> linkIndexToCdJoint;
	map<string, CdJoint*> map_;
};


class CdScene
{
public:
	DllExport CdScene();
	DllExport ~CdScene();
	DllExport int addChar(const char* name, CdChar* obj);
	DllExport int removeChar(const char* name);
	DllExport CdChar* getChar(const char* name);
	int exist(const char* name);
	DllExport void addCheckPair(CdCheckPair* cPair);
        
	DllExport int getNumCheckPairs();
	DllExport CdCheckPair* getCheckPair(int i);
        
private:
	map<string,CdChar*> map_;
	vector<CdCheckPair*> pairs_;
};

#endif
