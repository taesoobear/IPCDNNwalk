#include "physicsLib.h"
#include "Body.h"
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
/**
   @file DynamicsSimulator/server/World.cpp
   \author Shin'ichiro Nakaoka
*/

#include <string>

#include "Link.h"
#include "ForwardDynamicsABM.h"
#include "World.h"

using namespace std;
using namespace TRL;

static const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

static const bool debugMode = false;


WorldBase::WorldBase()
{
    currentTime_ = 0.0;
    timeStep_ = 0.005;

    g = vector3(0.0, 0.0, DEFAULT_GRAVITY_ACCELERATION);

    isEulerMethod =false;
	numRegisteredLinkPairs = 0;
}


WorldBase::~WorldBase()
{

}


int WorldBase::bodyIndex(const std::string& name)
{
    NameToIndexMap::iterator p = nameToBodyIndexMap.find(name);
    return (p != nameToBodyIndexMap.end()) ? p->second : -1;
}


BodyPtr WorldBase::body(int index)
{
	return bodyInfoArray[index].body; 
}
const BodyPtr WorldBase::body(int index) const
{
	return bodyInfoArray[index].body; 
}



BodyPtr WorldBase::body(const std::string& name)
{
    return bodyInfoArray[bodyIndex(name)].body;
}


void WorldBase::setTimeStep(double ts)
{
    timeStep_ = ts;
}


void WorldBase::setCurrentTime(double time)
{
    currentTime_ = time;
}


void WorldBase::setGravityAcceleration(const vector3& g)
{
    this->g = g;
   const int n = bodyInfoArray.size();

    for(int i=0; i < n; ++i)
		{
		BodyInfo& info = bodyInfoArray[i];
		if(info.forwardDynamics)
			info.forwardDynamics->setGravityAcceleration(g);
		}
}




void WorldBase::initialize()
{
   const int n = bodyInfoArray.size();

    for(int i=0; i < n; ++i){

		BodyInfo& info = bodyInfoArray[i];
		BodyPtr body = info.body;

		if(!info.forwardDynamics)
		{
			if(isEulerMethod)
				info.forwardDynamics=new ForwardDynamicsABM(body);
			else 
				info.forwardDynamics=new ForwardDynamicsABM_rungeKutta(body);
		}
	    info.forwardDynamics->setGravityAcceleration(g);
    	info.forwardDynamics->setTimeStep(timeStep_);
		info.forwardDynamics->initialize();
    }
}


void WorldBase::calcNextState()
{
	if(debugMode){
		cout << "World current time = " << currentTime_ << endl;
	}
    const int n = bodyInfoArray.size();

//#pragma omp parallel for num_threads(3) schedule(static)
#pragma omp parallel for num_threads(3) schedule(dynamic)
    for(int i=0; i < n; ++i){
        BodyInfo& info = bodyInfoArray[i];
        info.forwardDynamics->calcNextState();
    }
    currentTime_ += timeStep_;
}


int WorldBase::addBody(BodyPtr body)
{
    const string& name = body->name;

    if(!name.empty()){
        nameToBodyIndexMap[name] = bodyInfoArray.size();
    }

    BodyInfo info;

    info.body = body;
	info.forwardDynamics=NULL;

    bodyInfoArray.push_back(info);

    return bodyInfoArray.size() - 1;
}


void WorldBase::clearBodies()
{
    nameToBodyIndexMap.clear();
    bodyInfoArray.clear();
}


void WorldBase::setEulerMethod()
{
    isEulerMethod = true;
}


void WorldBase::setRungeKuttaMethod()
{
    isEulerMethod = false;
}


std::pair<int,bool> WorldBase::getIndexOfLinkPairs(Link* link1, Link* link2)
{
	int index = -1;
	int isRegistered = false;

    if(link1 != link2){

		LinkPairKey linkPair;
		if(link1 < link2){
			linkPair.link1 = link1;
			linkPair.link2 = link2;
		} else {
			linkPair.link1 = link2;
			linkPair.link2 = link1;
		}

		LinkPairKeyToIndexMap::iterator p = linkPairKeyToIndexMap.find(linkPair);

		if(p != linkPairKeyToIndexMap.end()){
			index = p->second;
			isRegistered = true;
		} else {
			index = numRegisteredLinkPairs++;
			linkPairKeyToIndexMap[linkPair] = index;
		}
	}

	return std::make_pair(index, isRegistered);
}


bool WorldBase::LinkPairKey::operator<(const LinkPairKey& pair2) const
{
	if(link1 < pair2.link1){
		return true;
	} else if(link1 == pair2.link1){
		return (link2 < pair2.link2);
	} else {
		return false;
	}
}
