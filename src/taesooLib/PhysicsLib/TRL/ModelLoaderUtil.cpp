#include "physicsLib.h"
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
/** @file DynamicsSimulator/server/convCORBAUtil.cpp
 *
 */

#include "OpenHRPcommon.h"
#include <stack>
#include "ModelLoaderUtil.h"
#include "Link.h"
#include "../MainLib/OgreFltk/VRMLloader.h"

using namespace TRL;
using namespace std;

static const bool debugMode = false;

	

static inline double getLimitValue(vectorn const& limitseq, double defaultValue)
{
	return (limitseq.size() == 0) ? defaultValue : limitseq[0];
}


static TRL::Link* createLink
(TRL::BodyPtr body, int index, OpenHRP::LinkInfoSequence const& iLinks, const matrix33& parentRs)
{
	OpenHRP::LinkInfo const& iLink = iLinks[index];

	int jointId = iLink.jointId;

	printf("%d index\n", index);
	TRL::Link* link = new Link;
	body->linkArray[index]=link;

	//link->name = iLink.name.ptr();

	//link->jointId = jointId-1;

	::vector3 b =iLink.translation;
	vector3 relPos(b.x, b.y, b.z);
	link->b = parentRs * relPos;

	matrix33 R=iLink.rotation;
	//link->Rs = (parentRs * R);
	//const matrix33& Rs = link->Rs;

	int jt=iLink.jointType;

	switch(jt)
	{
	case HRP_JOINT::FIXED:
	    link->jointType = Link::FIXED_JOINT;
		break;
	case HRP_JOINT::FREE:
	    link->jointType = Link::FREE_JOINT;
		break;
	case HRP_JOINT::ROTATE:
		link->jointType = Link::ROTATIONAL_JOINT;
		break;
	case HRP_JOINT::SLIDE:
		link->jointType = Link::SLIDE_JOINT;
		break;
	case HRP_JOINT::BALL:
		Msg::error("AIST doesn't support ball joints");
		break;
	default:
		link->jointType = Link::FREE_JOINT;
	}

	if(jointId < 0){
		if(link->jointType == Link::ROTATIONAL_JOINT || link->jointType == Link::SLIDE_JOINT){
			std::cerr << "Warning:  Joint ID is not given to joint " << link->index
					  << " of model " << body->modelName << "." << std::endl;
		}
	}

	link->a.zero();
	link->d.zero();

	vector3 a(0,0,0);
	if(iLink.jointAxis=="X")
		a[0]=1.0;
	else if(iLink.jointAxis=="Y")
		a[1]=1.0;
	else if(iLink.jointAxis=="Z")
		a[2]=1.0;
		
	//vector3 axis(Rs * a);
	vector3 axis(a);

	if(link->jointType == Link::ROTATIONAL_JOINT){
		link->a = axis;
	} else if(link->jointType == Link::SLIDE_JOINT){
		link->d = axis;
	}

	link->m             = iLink.mass;
	//link->Ir            = iLink.rotorInertia;
	link->Jm2	        = iLink.equivalentInertia; // diagonal element of the mass matrix
	//link->gearRatio		= iLink.gearRatio;
	////link->gearEfficiency = iLink.gearEfficiency;
	//link->gearEfficiency =1.0;
	//link->rotorResistor	= iLink.rotorResistor;
	//link->rotorResistor	= 0.0;//ohm
	//link->torqueConst	= iLink.torqueConst;
	//link->torqueConst	= 1.0;
	//link->rotorResistor = iLink.rotorResistor;
	//link->rotorResistor = 0.0;
	
	//if (link->Jm2 == 0){
	//	link->Jm2 = link->Ir * link->gearRatio * link->gearRatio;
	//}
	//link->encoderPulse	= iLink->encoderPulse();
	//link->encoderPulse	= 1.0;

	/*DblSequence_var ulimit  = iLink->ulimit();
	DblSequence_var llimit  = iLink->llimit();
	DblSequence_var uvlimit = iLink->uvlimit();
	DblSequence_var lvlimit = iLink->lvlimit();
*/
	vectorn ulimit, llimit, uvlimit, lvlimit;

	double maxlimit = numeric_limits<double>::max();

	/*
	link->ulimit  = getLimitValue(ulimit,  +maxlimit);
	link->llimit  = getLimitValue(llimit,  -maxlimit);
	link->uvlimit = getLimitValue(uvlimit, +maxlimit);
	link->lvlimit = getLimitValue(lvlimit, -maxlimit);
	*/

	const ::vector3& rc = iLink.centerOfMass;
	//link->c = Rs * vector3(rc.x, rc.y, rc.z);
	link->c = vector3(rc.x, rc.y, rc.z);

	matrix33 Io= iLink.inertia;

	/*
	matrix33 Imin;
	double massMin=0.1;
	OpenHRP::sphereInertia(massMin, Imin);
	if(link->m<massMin)
		link->m=massMin;

	if(Io._11<Imin._11) Io._11=Imin._11;
	if(Io._22<Imin._22) Io._22=Imin._22;
	if(Io._33<Imin._33) Io._33=Imin._33;
	*/

	//link->I = Rs * Io;
	link->I = Io;

	// a stack is used for keeping the same order of children
	std::stack<Link*> children;
	
	int childIndex = iLink.daughter;
	while(childIndex != -1){
	    //Link* childLink = createLink(body, childIndex, iLinks, Rs);
	    Link* childLink = createLink(body, childIndex, iLinks, parentRs);
	    if(childLink){
			children.push(childLink);
		}
	    childIndex = iLinks[childIndex].sister;
	}
	while(!children.empty()){
		link->addChild(children.top());
		children.pop();
	}
	return link;
}


BodyPtr TRL::loadBodyFromCharacterInfo(const OpenHRP::CharacterInfo* charaInfo)
{
	Body* body=new Body();

	body->modelName = charaInfo->name.ptr();

	int n = charaInfo->links.size();
	OpenHRP::LinkInfoSequence const& iLinks = charaInfo->links;

	int rootIndex = -1;

	for(int i=0; i < n; ++i){
		if(iLinks[i].mother < 0){
			if(rootIndex < 0){
				rootIndex = i;
			} else {
				body = 0; // more than one root !
			}
		}
	}
	if(rootIndex < 0){
		body = 0; // no root !
	}

	//std::vector<TRL::Link>* links=new std::vector<TRL::Link>();
	std::vector<TRL::Link*>* links=&body->linkArray;
	links->resize(n);
	//body->linkArray=links;

	if(body){
		matrix33 Rs(identity33());
		Link* rootLink = createLink(body, rootIndex, iLinks, Rs);
		body->setRootLink(rootLink);

		const ::vector3& p = iLinks[rootIndex].translation;
		vector3 pos(p.x, p.y, p.z);
		const ::matrix3& R = iLinks[rootIndex].rotation;
		body->setDefaultRootPosition(pos, R);

		body->initializeConfiguration();
	}

	return body;
}

