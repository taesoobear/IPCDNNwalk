#include "physicsLib.h"
// -*- mode: c++; indent-tabs-mode: nil; tab-width: 4; c-basic-offset: 4; -*-
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
   \file
   \brief Implementations of Link class
   \author S.Nakaoka
*/


/**
   \ifnot jp
   \class TRL::Link
   A class for representing a rigid body that consists of an articulated body.
   
   \var int Link::index
   Index among all the links in the body.
   
   \endif
*/




#include "Body.h"
#include "Link.h"


using namespace std;
using namespace TRL;


Link::Link()
{
	dqIndex=-1;
    parent = 0;
	sibling = 0;
	child = 0;
	
}


Link::Link(const Link& org)
{
	copy(org); 
}

Link::~Link()
{
    Link* link = child;
    while(link){
        Link* linkToDelete = link;
        link = link->sibling;
        delete linkToDelete;
    }
}


namespace {
	void setBodyIter(Link* link, Body* body)
	{
		link->body = body;
		
		if(link->sibling){
			setBodyIter(link->sibling, body);
		}
		if(link->child){
			setBodyIter(link->child, body);
		}
	}
}


void Link::addChild(Link* link)
{
	if(link->parent){
		link->parent->detachChild(link);
	}

    link->sibling = child;
    link->parent = this;
    child = link;

	setBodyIter(link, body);
}

void Link::copy(const Link& org)
{
    jointType = org.jointType;
    a = org.a;
    d = org.d;
    b = org.b;
    c = org.c;
    m = org.m;
    I = org.I;
	/*
    ulimit = org.ulimit;
    llimit = org.llimit;
    uvlimit = org.uvlimit;
    lvlimit = org.lvlimit;
	*/

    parent = child = sibling = 0;

    if(org.child){
        for(Link* orgChild = org.child; orgChild; orgChild = orgChild->sibling){
            Link* newChild = new Link(*orgChild);
            newChild->parent = this;
            newChild->sibling = child;
            child = newChild;
        }
    }
}

/**
   A child link is detached from the link.
   The detached child link is *not* deleted by this function.
   If a link given by the parameter is not a child of the link, false is returned.
*/
bool Link::detachChild(Link* childToRemove)
{
	bool removed = false;

	Link* link = child;
	Link* prevSibling = 0;
	while(link){
		if(link == childToRemove){
			removed = true;
			if(prevSibling){
				prevSibling->sibling = link->sibling;
			} else {
				child = link->sibling;
			}
			break;
		}
		prevSibling = link;
		link = link->sibling;
	}

	if(removed){
		childToRemove->parent = 0;
		childToRemove->sibling = 0;
		setBodyIter(childToRemove, 0);
	}

	return removed;
}


std::ostream& operator<<(std::ostream &out, Link& link)
{
    link.putInformation(out);
    return out;
}

Link& Link::operator=(const Link& link)
{
	copy(link);
	return *this;
}


void Link::putInformation(std::ostream& os)
{
    os << "Link Index = " << index << ", dqIndex = " << dqIndex<<"\n";

    os << "Joint Type: ";

    switch(jointType) {
    case FREE_JOINT:
        os << "Free Joint\n";
        break;
    case FIXED_JOINT:
        os << "Fixed Joint\n";
        break;
    case ROTATIONAL_JOINT:
        os << "Rotational Joint\n";
        os << "Axis = " << a << "\n";
        break;
    case SLIDE_JOINT:
        os << "Slide Joint\n";
        os << "Axis = " << d << "\n";
        break;
    }

    os << "parent = " << (parent ? parent->index : -1) << "\n";

	os << "child = ";
	if(child){
		Link* link = child;
		while(true){
			os << link->index;
			link = link->sibling;
			if(!link){
				break;
			}
        	os << ", ";
		}
	} else {
		os << "null";
	}
    os << "\n";

    os << "b = "  << b << "\n";
    os << "c = "  << c << "\n";
    os << "m = "  << m << "\n";
    //os << "I = "  << I << "\n";
    //os << "ulimit = " << ulimit << "\n";
    //os << "llimit = " << llimit << "\n";
    //os << "uvlimit = " << uvlimit << "\n";
    //os << "lvlimit = " << lvlimit << "\n";

}
