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
/** @file CollisionDetector/server/CdWrapper.cpp
 * wrapper implementation of collision detection package
 *
 * @version 0.1 (1999/12/13)
 * @version 0.2 (2000/03/15)    iostream.h -> iostream
 * @version 0.4 (2002/02/15)    remove garbages
 * @version 0.5 (2006/02/28)    support OPCODE
 * @version 0.6 (2006/06/30)    
 */


#include "StdAfxColDet.h"

#include "CdWrapper.h"
#include "Opcode.h"
#include "utilities.h"
#include <iostream>

//#define CDWRAPPER_DEBUG

using namespace std;

// ------------------------------------------------------------
// CdWrapperException
// ------------------------------------------------------------

CdWrapperException::CdWrapperException(
                                       int id,
                                       const char* msg
                                       ) : id_(id), msg_(msg) {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapperException::CdWrapperException()" << endl;
#endif
}

// ------------------------------------------------------------
CdWrapperException::~CdWrapperException() {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapperException::CdWrapperException()" << endl;
#endif
}

// ------------------------------------------------------------
// CdWrapper
// ------------------------------------------------------------

/**
   @if jp
   collision detection
   
   @param   R1 rotation of object1
   @param   T1 translation of object1
   @param   o1 CD_modelオブジェクト1
   @param   R2 回転成分2
   @param   T2 並進成分2
   @param   o2 CD_modelオブジェクト2
   @param   flag 接触チェックフラグ
   
   @endif
*/
void CdWrapper::cdCollide(
                          double R1[3][3], double T1[3], Opcode::Model* o1,
                          double R2[3][3], double T2[3], Opcode::Model* o2,
                          int flag
                          ) {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapper::cdCollide()" << endl;
#endif
    if(cdContact){
        delete[] cdContact;
        cdContact = 0;
        cdContactsCount = 0;
    }
    Opcode::AABBTreeCollider TC_;
    Opcode::BVTCache colCache;
    colCache.Model0 = o1;
    colCache.Model1 = o2;
    IceMaths::Matrix4x4 world0;
    IceMaths::Matrix4x4 world1;
    world0.Set((float)R1[0][0], (float)R1[1][0], (float)R1[2][0], 0.0,
			   (float)R1[0][1], (float)R1[1][1], (float)R1[2][1], 0.0,
			   (float)R1[0][2], (float)R1[1][2], (float)R1[2][2], 0.0,
			   (float)T1[0],    (float)T1[1],    (float)T1[2], 1.0);
    world1.Set((float)R2[0][0], (float)R2[1][0], (float)R2[2][0], 0.0,
			   (float)R2[0][1], (float)R2[1][1], (float)R2[2][1], 0.0,
			   (float)R2[0][2], (float)R2[1][2], (float)R2[2][2], 0.0,
			   (float)T2[0],    (float)T2[1],    (float)T2[2], 1.0);
    if(flag == CD_FIRST_CONTACT)
        TC_.SetFirstContact(true);
    bool isOK = TC_.Collide(colCache, &world0, &world1);
    cdBoxTestsCount = TC_.GetNbBVBVTests();
    cdTriTestsCount = TC_.GetNbPrimPrimTests();
    if (isOK != true)  {
        throw CdWrapperException(isOK, "CdWrapper::cdCollide()");
        return;
    }
}

/**
   @if jp
   collision detection
 
   @param   R1 rotation of object1
   @param   T1 translation of object1
   @param   o1 Opcode::Modelオブジェクト1
   @param   R2 rotation of object2
   @param   T2 translation of object2
   @param   o2 Opcode::Modelオブジェクト2
   @param   flag 接触チェックフラグ

   @endif
*/
void CdWrapper::cdCollide(
                          float R1[3][3], float T1[3], Opcode::Model* o1,
                          float R2[3][3], float T2[3], Opcode::Model* o2,
                          int flag
                          ) {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapper::cdCollide()" << endl;
#endif
    double Rf1[][3] = {
        {(double)R1[0][0], (double)R1[0][1], (double)R1[0][2]},
        {(double)R1[1][0], (double)R1[1][1], (double)R1[1][2]},
        {(double)R1[2][0], (double)R1[2][1], (double)R1[2][2]}
    };
    double Rf2[][3] = {
        {(double)R2[0][0], (double)R2[0][1], (double)R2[0][2]},
        {(double)R2[1][0], (double)R2[1][1], (double)R2[1][2]},
        {(double)R2[2][0], (double)R2[2][1], (double)R2[2][2]}
    };
    double Tf1[] = {(double)T1[0], (double)T1[1], (double)T1[2]};
    double Tf2[] = {(double)T2[0], (double)T2[1], (double)T2[2]};

    CdWrapper::cdCollide(Rf1, Tf1, o1, Rf2, Tf2, o2, flag);
}

/**
   @if jp
   collision detection
   
   @param   R1 rotation of object1
   @param   T1 translation of object1
   @param   s1 scale of object1
   @param   o1 Opcode::Modelオブジェクト1
   @param   R2 rotation of object2
   @param   T2 translation of object2
   @param   s2 scale of object2
   @param   o2 Opcode::Modelオブジェクト2
   @param   flag 接触チェックフラグ
   
   @endif
*/
void CdWrapper::cdCollide(
                          double R1[3][3], double T1[3], double s1, Opcode::Model* o1,
                          double R2[3][3], double T2[3], double s2, Opcode::Model* o2,
                          int flag
                          ) {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapper::cdCollide()" << endl;
#endif
    if(cdContact){
        cdContactsCount = 0;
    }
    Opcode::AABBTreeCollider TC_;
    Opcode::BVTCache colCache;
    colCache.Model0 = o1;
    colCache.Model1 = o2;
  
    IceMaths::Matrix4x4 world0;
    IceMaths::Matrix4x4 world1;
    world0.Set((float)R1[0][0], (float)R1[1][0], (float)R1[2][0], 0.0,
               (float)R1[0][1], (float)R1[1][1], (float)R1[2][1], 0.0,
               (float)R1[0][2], (float)R1[1][2], (float)R1[2][2], 0.0,
               (float)T1[0],    (float)T1[1],    (float)T1[2], 1.0);
    world1.Set((float)R2[0][0], (float)R2[1][0], (float)R2[2][0], 0.0,
               (float)R2[0][1], (float)R2[1][1], (float)R2[2][1], 0.0,
               (float)R2[0][2], (float)R2[1][2], (float)R2[2][2], 0.0,
               (float)T2[0],    (float)T2[1],    (float)T2[2], 1.0);
    if(flag == CD_FIRST_CONTACT)
        TC_.SetFirstContact(true); 
    int ret = TC_.Collide(colCache, &world0, &world1);
    cdBoxTestsCount = TC_.GetNbBVBVTests();
    cdTriTestsCount = TC_.GetNbPrimPrimTests();
    if (ret != CD_OK)  {
        throw CdWrapperException(ret, "CdWrapper::cdCollide()");
        return;
    }
}


/**
   @if jp
   * collision detection
   *
   * @param   R1 rotation of object1
   * @param   T1 translation of object1
   * @param   s1 scale of object1
   * @param   o1 Opcode::Modelオブジェクト1
   * @param   R2 rotation of object2
   * @param   T2 translation of object2
   * @param   s2 scale of object2
   * @param   o2 Opcode::Modelオブジェクト2
   * @param   flag 接触チェックフラグ

   @endif
*/
void CdWrapper::cdCollide(
                          float R1[3][3], float T1[3], float s1, Opcode::Model* o1,
                          float R2[3][3], float T2[3], float s2, Opcode::Model* o2,
                          int flag
                          ) {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapper::cdCollide()" << endl;
#endif
    double Rf1[][3] = {
        {(double)R1[0][0], (double)R1[0][1], (double)R1[0][2]},
        {(double)R1[1][0], (double)R1[1][1], (double)R1[1][2]},
        {(double)R1[2][0], (double)R1[2][1], (double)R1[2][2]}
    };
    double Rf2[][3] = {
        {(double)R2[0][0], (double)R2[0][1], (double)R2[0][2]},
        {(double)R2[1][0], (double)R2[1][1], (double)R2[1][2]},
        {(double)R2[2][0], (double)R2[2][1], (double)R2[2][2]}
    };
    double Tf1[] = {(double)T1[0], (double)T1[1], (double)T1[2]};
    double Tf2[] = {(double)T2[0], (double)T2[1], (double)T2[2]};
    double sf1 = (double)s1;
    double sf2 = (double)s2;

    CdWrapper::cdCollide(Rf1, Tf1, sf1, o1, Rf2, Tf2, sf2, o2, flag);
}


// ------------------------------------------------------------
int CdWrapper::getNumBoxTests() {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapper::getNumBoxTests()" << endl;
#endif
    return cdBoxTestsCount;
}

// ------------------------------------------------------------
int CdWrapper::getNumTriTests() {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapper::getNumTriTests()" << endl;
#endif
    return cdTriTestsCount;
}

// ------------------------------------------------------------
int CdWrapper::getNumContacts() {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapper::getNumContacts() " << cdContactsCount <<endl;
#endif
    return cdContactsCount;
}

// ------------------------------------------------------------
collision_data* CdWrapper::getCollisionData() {
#ifdef CDWRAPPER_DEBUG
    cout << "CdWrapper::getCollisionData()" << endl;
#endif
    return cdContact;
}

