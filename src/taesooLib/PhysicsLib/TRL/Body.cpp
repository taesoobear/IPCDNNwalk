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
/** \file
	\brief The implementation of the body class 
	\author S.NAKAOKA
*/

#include "physicsLib.h"
#include "Body.h"

#include <map>
#include <cstdlib>

#include "Link.h"
#include "LinkPath.h"
#include "TRL_common.h"
#include "../../BaseLib/motion/Liegroup.h"

using namespace TRL;
using namespace std;

static const bool PUT_DEBUG_MESSAGE = true;

static bool pluginsInDefaultDirectoriesLoaded = false;

#ifndef uint
typedef unsigned int uint;
#endif


Body::~Body()
{
	delete rootLink_;
}



void Body::initialize()
{

	
}
	

Body::Body() 
{
	initialize();
	
    rootLink_ = new Link;

    defaultRootPosition = vector3(0.0);
    defaultRootAttitude = identity33();
}


Body::Body(const Body& org) 
{
	initialize();
	
	modelName = org.modelName;

    setRootLink(new Link(*org.rootLink()));
	ASSERT(false);// need to set linkArray

	defaultRootPosition = org.defaultRootPosition;
	defaultRootAttitude = org.defaultRootAttitude;
}


void Body::setRootLink(Link* link)
{
    if(rootLink_){
		delete rootLink_;
    }
    rootLink_ = link;

    updateLinkTree();
}


void Body::setDefaultRootPosition(const vector3& pos, const matrix33& att)
{
    defaultRootPosition = pos;
    defaultRootAttitude = att;
}


void Body::updateLinkTree()
{
    //linkTraverse_.find(rootLink());

    int n = numLinks();
    
    for(int i=0; i < n; ++i){
        Link* l = link(i);
		l->body = this;
		l->index = i;
    }


    calcTotalMass();

    isStatic_ = (rootLink_->jointType == Link::FIXED_JOINT && numJoints() == 0);
}




void Body::initializeConfiguration()
{
    rootLink_->p = defaultRootPosition;
    rootLink_->R = defaultRootAttitude;

    rootLink_->v .zero();
    rootLink_->dv .zero();
    rootLink_->w .zero();
    rootLink_->dw .zero();
    rootLink_->vo .zero();
    rootLink_->dvo .zero();
    
    int n = numLinks();
    for(int i=0; i < n; ++i){
        Link* l = link(i);
        l->u = 0.0;
        l->q = 0.0;
        l->dq = 0.0;
        l->ddq = 0.0;
    }
 
    calcForwardKinematics(true, true);

	clearExternalForces();
}
 


double Body::calcTotalMass()
{
    totalMass_ = 0.0;

    int n = numLinks();
    for(int i=0; i < n; ++i){
        totalMass_ += link(i)->m;
    }

    return totalMass_;
}


vector3 Body::calcCM()
{
    totalMass_ = 0.0;
    
    vector3 mc(0.0);

    int n = numLinks();
    for(int i=0; i < n; i++){
        Link* link = (*this)[i];
		link->wc = link->p + link->R * link->c;
        mc += link->m * link->wc;
        totalMass_ += link->m;
    }

    return vector3(mc / totalMass_);
}

/**
   calculate the mass matrix using the unit vector method
   \todo replace the unit vector method here with
   a more efficient method that only requires O(n) computation time

   The motion equation (dv != dvo)
 		  |       |   | dw   |   |    |   | tauext      |
		  | out_M | * | dv   | + | b1 | = |  fext   |
		  |       |   |ddq   |   |    |   | u         |


		  assuming defined(SWAP_FORCE_AND_TORQUE)
		  which is the default option and is the only well-tested option.
*/
void Body::calcMassMatrix(dmatrix& out_M, dmatrix & b1, vector3 const& g)
{
	// buffers for the unit vector method
	vector3 dvoorg;
	vector3 dworg;
	vector3 root_w_x_v;

	uint nJ = numJoints();
	int totaldof = nJ;
	if( !isStatic_ ) totaldof += 6;

	out_M.resize(totaldof,totaldof);
	b1.resize(totaldof, 1);

	// preserve and clear the joint accelerations
	//dvector ddqorg; ddqorg.resize(nJ);
	//dvector uorg; uorg.resize(nJ);
#ifdef _MSC_VER
	ASSERT(nJ < 100);
	double ddqorg[100]; 
	double uorg[100];
#else
	double ddqorg[nJ]; // C++11. so compile with --std=c++0x option. faster than a dynamic allocation!
	double uorg[nJ];
#endif
	for(uint i = 0; i < nJ; ++i){
		Link* ptr = joint(i);
		//cout << i << ptr->name <<"\n";
		ddqorg[i] = ptr->ddq;
		uorg  [i] = ptr->u;
		ptr->ddq = 0.0;
	}

	// preserve and clear the root link acceleration
	dvoorg = rootLink_->dvo;
	dworg  = rootLink_->dw;
	root_w_x_v = cross(rootLink_->w, vector3(rootLink_->vo + cross(rootLink_->w, rootLink_->p)));
	rootLink_->dvo = g - root_w_x_v;   // dv = g, dw = 0
	rootLink_->dw  .zero();

	/*
	if(verifyEQ)
	{
		veqbackup=verifyEQ;
		std::cout << "wxv "<<root_w_x_v<<"dvo "<<rootLink_->dvo<<std::endl;
	}
	*/
	
	setColumnOfMassMatrix(b1, 0);


	if( !isStatic_ ){
		for(int i=0; i < 3; ++i){
			// dvo : 
			// j->dvo = dv - cross(j->dw, j->p) - cross(j->w, j->v);
			rootLink_->dvo[i] += 1.0;
#ifdef SWAP_FORCE_AND_TORQUE
			setColumnOfMassMatrix(out_M, i+3);
#else
			setColumnOfMassMatrix(out_M, i);
#endif
			rootLink_->dvo[i] -= 1.0;
		}
		for(int i=0; i < 3; ++i){
			rootLink_->dw[i] = 1.0;
			vector3 dw_x_p = cross(rootLink_->dw, rootLink_->p);	//  spatial acceleration caused by ang. acc.
			rootLink_->dvo -= dw_x_p;
#ifdef SWAP_FORCE_AND_TORQUE
			setColumnOfMassMatrix(out_M, i );
#else
			setColumnOfMassMatrix(out_M, i + 3);
#endif
			rootLink_->dvo += dw_x_p;
			rootLink_->dw[i] = 0.0;
		}
	}


	for(uint i = 0; i < nJ; ++i){
		Link* ptr = joint(i);
		ptr->ddq = 1.0;
		int j = i + 6;
		setColumnOfMassMatrix(out_M, j);
		out_M(j, j) += ptr->Jm2; // motor inertia
		ptr->ddq = 0.0;
    }

	// subtract the constant term
	for(size_t i = 0; i < out_M.cols(); ++i){
		out_M.column( i) -= b1.column(0);
	}

	// recover state
	for(uint i = 0; i < nJ; ++i){
		Link* ptr = joint(i);
		ptr->ddq  = ddqorg[i];
		ptr->u    = uorg  [i];
	}
	rootLink_->dvo = dvoorg;
	rootLink_->dw  = dworg;

}

void Body::setColumnOfMassMatrix(dmatrix& out_M, int column)
{
    vector3 f;
	vector3 tau;

    calcInverseDynamics(rootLink_, f, tau);

	if( !isStatic_ ){
		tau -= cross(rootLink_->p, f);
#ifdef SWAP_FORCE_AND_TORQUE
		setVector3(f,   out_M, 3, column);
		setVector3(tau, out_M, 0, column);
#else
		setVector3(f,   out_M, 0, column);
		setVector3(tau, out_M, 3, column);
#endif
	}

	int n = numJoints();
	for(int i = 0; i < n; ++i){
		Link* ptr = joint(i);
		out_M(i + 6, column) = ptr->u;
    }
}

/*
 *  see Kajita et al. Humanoid Robot Ohm-sha,  p.210
 */
void Body::calcInverseDynamics(Link* ptr, vector3& out_f, vector3& out_tau)
{	
    Link* parent = ptr->parent;
    if(parent){
		vector3 dsv,dsw,sv,sw;

		sw  = parent->R * ptr->a;
		sv  = cross(ptr->p, sw);
		dsv = cross(parent->w, sv) + cross(parent->vo, sw);
		dsw = cross(parent->w, sw);

		ptr->dw  = parent->dw  + dsw * ptr->dq + sw * ptr->ddq;
		ptr->dvo = parent->dvo + dsv * ptr->dq + sv * ptr->ddq;

		ptr->sw = sw;
		ptr->sv = sv;
    }
	
	vector3  c,P,L;
	matrix33 I,c_hat;

	c = ptr->R * ptr->c + ptr->p;
	I = ptr->R * ptr->I * trans(ptr->R);
	c_hat = hat(c);
	I += ptr->m * c_hat * trans(c_hat);
	P = ptr->m * (ptr->vo + cross(ptr->w, c));
	L = ptr->m * cross(c, ptr->vo) + I * ptr->w;

    out_f   = ptr->m * (ptr->dvo + cross(ptr->dw, c)) + cross(ptr->w, P);
    out_tau = ptr->m * cross(c, ptr->dvo) + I * ptr->dw + cross(ptr->vo,P) + cross(ptr->w,L);
	/*
	if(verifyEQ)
	{
		std::cout << "c "<<c<<" "<<I<<" "<<P<<" "<<L<<out_f<<out_tau<<std::endl;
	}
	*/

    if(ptr->child){
		vector3 f_c;
		vector3 tau_c;
		calcInverseDynamics(ptr->child, f_c, tau_c);
		out_f   += f_c;
		out_tau += tau_c;
    }

    ptr->u = dot(ptr->sv, out_f) + dot(ptr->sw, out_tau);

    if(ptr->sibling){
		vector3 f_s;
		vector3 tau_s;
		calcInverseDynamics(ptr->sibling, f_s, tau_s);
		out_f   += f_s;
		out_tau += tau_s;
    }
}


/**
   assuming Link::v,w is already computed by calcForwardKinematics(true);
   assuming Link::wc is already computed by calcCM();
*/
void Body::calcTotalMomentum(vector3& out_P, vector3& out_L)
{
	out_P = 0.0;
	out_L = 0.0;

	vector3 dwc;	// Center of mass speed in world frame
	vector3 P;		// Linear momentum of the link
	vector3 L;		// Angular momentum with respect to the world frame origin 
	vector3 Llocal; // Angular momentum with respect to the center of mass of the link

	int n = numLinks();
    for(int i=0; i < n; i++){
        Link* link = (*this)[i];

		dwc = link->v + cross(link->w, vector3(link->R * link->c));

		P   = link->m * dwc;

		//L   = cross(link->wc, P) + link->R * link->I * trans(link->R) * link->w; 
		Llocal = link->I * Mtx_prod(link->R, link->w);
		L      = cross(link->wc, P) + link->R * Llocal; 

		out_P += P;
		out_L += L;
    }
}

void Body::calcForwardKinematics(bool calcVelocity, bool calcAcceleration)
{
	LinkTraverse linkTraverse_;
	linkTraverse_.find(rootLink());
    linkTraverse_.calcForwardKinematics(calcVelocity, calcAcceleration);
}






void Body::clearExternalForces()
{
    int n = numLinks();
    for(int i=0; i < n; ++i){
        Link* link = (*this)[i];
        link->fext .zero();
        link->tauext .zero();
    }
}


void Body::putInformation(std::ostream &out)
{
    out << "Body: model name = " << modelName
		<< " name = " << name << "\n\n";

    int n = numLinks();
    for(int i=0; i < n; ++i){
        out << *link(i);
    }
    out << std::endl;
}




std::ostream& operator<< (std::ostream& out, Body& body)
{
    body.putInformation(out);
    return out;
}




