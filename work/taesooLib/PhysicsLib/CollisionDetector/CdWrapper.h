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
/// @file CollisionDetector/server/CdWrapper.h
#ifndef CD_WRAPPER_H
#define CD_WRAPPER_H

#include "utilities.h"
#include "Opcode.h"

#ifdef _WIN32
#ifndef DLLEXPORT
#define DLLEXPORT __declspec(dllexport)
#endif /* DLLEXPORT */
#else
#define DLLEXPORT
#endif /* _WIN32 */


class DLLEXPORT CdWrapperException {

private:
    int id_;
    const char* msg_;

public:
    CdWrapperException(int id, const char* msg);
    ~CdWrapperException();
};


/**
   wrapper of collision detection package
*/
class DLLEXPORT CdWrapper {

public:

    static void cdCollide(
                          double R1[3][3], double T1[3], Opcode::Model* o1,
                          double R2[3][3], double T2[3], Opcode::Model* o2,
                          int flag = CD_ALL_CONTACTS
                          );

    static void cdCollide(
                          float R1[3][3], float T1[3], Opcode::Model* o1,
                          float R2[3][3], float T2[3], Opcode::Model* o2,
                          int flag = CD_ALL_CONTACTS
                          );

    static void cdCollide(
                          double R1[3][3], double T1[3], 
                          double s1, Opcode::Model* o1,
                          double R2[3][3], double T2[3], 
                          double s2, Opcode::Model* o2,
                          int flag = CD_ALL_CONTACTS
                          );

    static void cdCollide(
                          float R1[3][3], float T1[3], 
                          float s1, Opcode::Model* o1,
                          float R2[3][3], float T2[3], 
                          float s2, Opcode::Model* o2,
                          int flag = CD_ALL_CONTACTS
                          );
		
    static int getNumBoxTests();
		
    static int getNumTriTests();
		
    static int getNumContacts();
		
    static collision_data* getCollisionData();
};

#endif  /* CD_WRAPPER_H */
