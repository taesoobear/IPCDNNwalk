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
 *
 * Copyright (c) 2017, Hanyang university.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 */

/**
   \file
   \brief Implementations of the LinkPath and JointPath classes
   \author Shin'ichiro Nakaoka
   \author Taesoo Kwon
*/
  

#include "Body.h"

#include "Link.h"
#include "LinkPath.h"

#include <algorithm>

using namespace std;
using namespace TRL;


LinkPath::LinkPath()
{

}


LinkPath::LinkPath(Link* root, Link* end)
{
    find(root, end);
}


LinkPath::LinkPath(Link* end)
{
    findPathFromRoot(end);
}


bool LinkPath::find(Link* root, Link* end)
{
    links.clear();
    numUpwardConnections = 0;
	bool found = findPathSub(root, 0, end, false);
	if(!found){
		links.clear();
	}
	return found;
}


bool LinkPath::findPathSub(Link* link, Link* prev, Link* end, bool isUpward)
{
    links.push_back(link);
    if(isUpward){
        ++numUpwardConnections;
    }
    
    if(link == end){
        return true;
    }

    for(Link* child = link->child; child; child = child->sibling){
        if(child != prev){
            if(findPathSub(child, link, end, false)){
                return true;
            }
        }
    }

    Link* parent = link->parent;
    if(parent && parent != prev){
        if(findPathSub(parent, link, end, true)){
            return true;
        }
    }

    links.pop_back();
    if(isUpward){
        --numUpwardConnections;
    }

    return false;
}


void LinkPath::findPathFromRoot(Link* end)
{
    links.clear();
    numUpwardConnections = 0;
    findPathFromRootSub(end);
    std::reverse(links.begin(), links.end());
}


void LinkPath::findPathFromRootSub(Link* link)
{
    links.push_back(link);
    if(link->parent){
        findPathFromRootSub(link->parent);
    }
}


JointPath::JointPath()
{
	initialize();
}

JointPath::JointPath(Link* root, Link* end) : 
	LinkPath(root, end), 
	joints(links.size())
{
	initialize();
    extractJoints();
}


JointPath::JointPath(Link* end) :
	LinkPath(end), 
	joints(links.size())
{
	initialize();
    extractJoints();
}


void JointPath::initialize()
{
	maxIkErrorSqr = 1.0e-6 * 1.0e-6;
}
	

JointPath::~JointPath()
{

}


bool JointPath::find(Link* root, Link* end)
{
    if(LinkPath::find(root, end)){
        extractJoints();
    }
	onJointPathUpdated();

	return (!joints.empty());
}


bool JointPath::findPathFromRoot(Link* end)
{
    LinkPath::findPathFromRoot(end);
	extractJoints();
	onJointPathUpdated();
	
    return !joints.empty();
}


void JointPath::extractJoints()
{
    numUpwardJointConnections = numUpwardConnections;

    int n = links.size();
    if(n <= 1){
        joints.clear();
    } else {
        int i = 0;
        if(isDownward(i)){
            i++;
        }
        joints.resize(n); // reserve size n buffer
        joints.clear();
        int m = n - 1;
        while(i < m){
            Link* link = links[i];
            if(link->jointType == Link::ROTATIONAL_JOINT || link->jointType == Link::SLIDE_JOINT){
                joints.push_back(link);
            } else if(!isDownward(i)){
                --numUpwardJointConnections;
            }
            ++i;
        }
        if(isDownward(m-1)){
			Link* link = links[m];
			if(link->jointType == Link::ROTATIONAL_JOINT || link->jointType == Link::SLIDE_JOINT){
				joints.push_back(link);
			}
        }
    }
}


void JointPath::onJointPathUpdated()
{

}


void JointPath::setMaxIKError(double e)
{
  maxIkErrorSqr = e * e;
}


void JointPath::calcJacobian(dmatrix& out_J) const
{
	const int n = joints.size();
	if(n > 0){
		Link* targetLink = LinkPath::endLink();
		calcJacobian(out_J, targetLink->p);
	}
}
void JointPath::calcJacobian(dmatrix& out_J, vector3 const& targetPos) const
{
	const int n = joints.size();
	out_J.resize(6, n);
	
	if(n > 0){
		
		Link* targetLink = LinkPath::endLink();
		
		for(int i=0; i < n; ++i){
			
			Link* link = joints[i];
			
			switch(link->jointType){
				
			case Link::ROTATIONAL_JOINT:
				{
					vector3 omega(link->R * link->a);
					vector3 arm(targetPos - link->p);
					if(!isJointDownward(i)){
						omega *= -1.0;
                    } 
					vector3 dp;
					dp.cross(omega, arm);
					setVector3(omega, out_J, 0, i);
					setVector3(dp,    out_J, 3, i);
				}
				break;
				
			case Link::SLIDE_JOINT:
				{
					vector3 dp(link->R * link->d);
					if(!isJointDownward(i)){
						dp *= -1.0;
					}
					out_J(0, i) = 0.0;
					out_J(1, i) = 0.0;
					out_J(2, i) = 0.0;
					setVector3(dp, out_J, 3, i);
				}
				break;
				
			default:
				for(int j=0; j < 6; ++j){
					out_J(j, i) = 0.0;
				}
			}
		}
	}
}
void JointPath::calcJdot(dmatrix& out_J, vector3 const& targetPos) const
{
	// vector3 dp=  cross(omega, arm) = skew (R * link->a) * (targetPos -link->p)
	// vector3 omega=R * link->a;
	//
	// Now, we need to compute d(dp)/dt, d(omega)/dt

	// dR/dt = R*skew(ww)  where ww is the body angular velocity
	// 			= skew(w)*R    where w is the global angular velocity
	// so d(omega)/dt = skew(w)* R* link->a
	//     d(dp)/dt = d(skew(omega)*arm)/dt
	//     			= skew ( d(omega)dt) * arm + skew(omega)* d(arm)/dt
	//

	const int n = joints.size();
	out_J.resize(6, n);

	if(n > 0){
		Link* targetLink = LinkPath::endLink();
		for(int i=0; i < n; ++i){


			Link* link = joints[i];
			assert(link->jointType==Link::ROTATIONAL_JOINT);
			Link* parent = link->parent;

			vector3 omega=link->R*link->a;
			if(!isJointDownward(i)){
				omega *= -1.0;
			} 
			// dw = d(omega)/dt
			vector3 dw; dw.cross(link->w, omega);
			vector3 arm=targetPos-link->p;
			vector3 dv ;
			dv.cross(dw, arm);
			//dv+=cross(omega, dotArm);
			dv+=cross(omega, targetLink->v + cross(targetLink->w, targetPos-targetLink->p) -link->v);
			setVector3(dw, out_J, 0, i);
			setVector3(dv, out_J, 3, i);
		}
	}
}

void JointPath::calcAngularJacobian(dmatrix& out_J) const
{
	const int n = joints.size();
	out_J.resize(3, n);
	
	if(n > 0){
		
		Link* targetLink = LinkPath::endLink();
		
		for(int i=0; i < n; ++i){
			
			Link* link = joints[i];
			
			switch(link->jointType){
				
			case Link::ROTATIONAL_JOINT:
				{
					vector3 omega(link->R * link->a);
					if(!isJointDownward(i)){
						omega *= -1.0;
                    } 
					setVector3(omega, out_J, 0, i);
				}
				break;
				
			case Link::SLIDE_JOINT:
				{
					out_J(0, i) = 0.0;
					out_J(1, i) = 0.0;
					out_J(2, i) = 0.0;
				}
				break;
				
			default:
				for(int j=0; j < 3; ++j){
					out_J(j, i) = 0.0;
				}
			}
		}
	}
}


std::ostream& operator<<(std::ostream& os, JointPath& path)
{
    path.putInformation(os);
    return os;
}


void JointPath::putInformation(std::ostream& os) const
{
    int n = numJoints();
    for(int i=0; i < n; ++i){
        Link* link = joint(i);
        os << link->index;
        if(i != n){
			os << (isDownward(i) ? " => " : " <= ");
        }
    }
    os << std::endl;
}
