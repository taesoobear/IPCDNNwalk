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
	\brief Implementation of ContactForceSolver class
	\author S.NAKAOKA
	\author Taesoo Kwon
*/
#define ASSERT(x) 
#define RANGE_ASSERT(x) 

#include "physicsLib.h"
#include "Body.h"
#include "../../BaseLib/utility/QPerformanceTimer.h"
#if 1
// to disable profiling, set 1 above.
#undef BEGIN_TIMER
#undef END_TIMER2
#define BEGIN_TIMER(x)
#define END_TIMER2(x)
#endif
#ifdef __WIN32__
#define NOMINMAX
#endif

#include "World.h"
#include "Link.h"
#include "LinkTraverse.h"
#include "ForwardDynamicsABM.h"
#include "ContactForceSolver.h"

#include <limits>

#include "OpenHRPcommon.h"
#include "DynamicsSimulator.h"
#include "eigenSupport.h"


inline double norm2(const vector3& v) { return v.length();}

// settings

static const double VEL_THRESH_OF_DYNAMIC_FRICTION = 1.0e-4;

//static const bool ENABLE_STATIC_FRICTION = true;
//static const bool ONLY_STATIC_FRICTION_FORMULATION = (true );
static const bool STATIC_FRICTION_BY_TWO_CONSTRAINTS = true;
static const bool IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION = false;

static const bool ENABLE_TRUE_FRICTION_CONE =
	(true && STATIC_FRICTION_BY_TWO_CONSTRAINTS);
static const bool SKIP_REDUNDANT_ACCEL_CALC = true;
static const bool USE_PREVIOUS_LCP_SOLUTION = false;

static const bool ALLOW_SUBTLE_PENETRATION_FOR_STABILITY = true;
static const double ALLOWED_PENETRATION_DEPTH = 0.0001;
static const double NEGATIVE_VELOCITY_RATIO_FOR_ALLOWING_PENETRATION = 10.0; 


using namespace OpenHRP;
using namespace TRL;
using namespace std;

#include "World.h"
#include "Body.h"
#include "Link.h"
#include "LinkTraverse.h"
#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923
namespace TRL
{

#ifdef __WIN32__
	const double CFSImpl::PI   = 3.14159265358979323846;
	const double CFSImpl::PI_2 = 1.57079632679489661923;
#endif
};


bool ContactForceSolver::addCollisionCheckLinkPair
(int bodyIndex1, Link* link1, int bodyIndex2, Link* link2, double muStatic, double muDynamic, double epsilon)
{
    int index;
    int isRegistered;

    tie(index, isRegistered) = world.getIndexOfLinkPairs(link1, link2);

    if(index >= 0){

		int n = collisionCheckLinkPairs.size();
		if(index >= n){
			collisionCheckLinkPairs.resize(index+1);
		}

		LinkPair& linkPair = collisionCheckLinkPairs[index];

		linkPair.isSameBodyPair = (bodyIndex1 == bodyIndex2);
		linkPair.bodyIndex[0] = bodyIndex1;
		linkPair.link[0] = link1;
		linkPair.bodyIndex[1] = bodyIndex2;
		linkPair.link[1] = link2;
		linkPair.index = index;
		linkPair.muStatic = muStatic;
		linkPair.muDynamic = muDynamic;
		linkPair.epsilon = epsilon;
		linkPair.connection = 0;
    }

    return (index >= 0 && !isRegistered);
}



void ContactForceSolver::initialize(void)
{

	int numBodies = world.numBodies();

	bodiesData.resize(numBodies);

	connectedLinkPairs.clear();

	for(int bodyIndex=0; bodyIndex < numBodies; ++bodyIndex){

		BodyPtr body = world.body(bodyIndex);

		body->clearExternalForces();
		BodyData& bodyData = bodiesData[bodyIndex];
		bodyData.body = body;
		bodyData.linksData.resize(body->numLinks());
		bodyData.hasConstrainedLinks = false;
		bodyData.isTestForceBeingApplied = false;
		bodyData.isStatic = body->isStatic();


		LinkDataArray& linksData = bodyData.linksData;

		if(bodyData.isStatic ){
			int n = linksData.size();
			for(int j=0; j < n; ++j){
				LinkData& linkData = linksData[j];
				linkData.dw  = 0.0;
				linkData.dvo = 0.0;
			}
		}

		// initialize link data
		//const LinkTraverse& traverse = body->linkTraverse();
		const Body& traverse=*body;
		for(int j=0; j < traverse.numLinks(); ++j){
			Link* link = traverse[j];
			linksData[link->index].link = link;
			linksData[link->index].parentIndex = link->parent ? link->parent->index : -1;
		}

		// initialize link connection
		Body::LinkConnectionArray& connections = body->linkConnections;
		for(size_t j=0; j < connections.size(); ++j){

			connectedLinkPairs.push_back(LinkPair());
			LinkPair& linkPair = connectedLinkPairs.back();

			Body::LinkConnection& connection = connections[j];
			linkPair.connection = &connection;
			linkPair.isSameBodyPair = true;
			linkPair.constraintPoints.resize(connection.numConstraintAxes);

			for(int k=0; k < connection.numConstraintAxes; ++k){
				ConstraintPoint& constraint = linkPair.constraintPoints[k];
				constraint.numFrictionVectors = 0;
				constraint.globalFrictionIndex = numeric_limits<int>::max();
			}

			for(int k=0; k < 2; ++k){
				linkPair.bodyIndex[k] = bodyIndex;
				linkPair.bodyData[k] = &bodiesData[bodyIndex];
				ASSERT(linkPair.bodyData[k]);
				Link* link = connection.link[k];
				linkPair.link[k] = link;
				linkPair.linkData[k] = &(bodyData.linksData[link->index]);
			}
		}
	}

	int numLinkPairs = collisionCheckLinkPairs.size();
    for(int i=0; i < numLinkPairs; ++i){
        LinkPair& linkPair = collisionCheckLinkPairs[i];
		for(int j=0; j < 2; ++j){
			BodyData& bodyData = bodiesData[linkPair.bodyIndex[j]];
			linkPair.bodyData[j] = &bodyData;
			linkPair.linkData[j] = &(bodyData.linksData[linkPair.link[j]->index]);
			ASSERT(linkPair.bodyData[j]);
		}
	}

	prevGlobalNumConstraintVectors = 0;
	prevGlobalNumFrictionVectors = 0;
    numUnconverged = 0;

}


void ContactForceSolver::clearExternalForces()
{
    for(size_t i=0; i < bodiesData.size(); ++i){
		BodyData& bodyData = bodiesData[i];
		if(bodyData.hasConstrainedLinks){
			bodyData.body->clearExternalForces();
		}
    }
}


void ContactForceSolver::solve(CollisionSequence& corbaCollisionSequence)
{
    for(size_t i=0; i < bodiesData.size(); ++i){
		bodiesData[i].hasConstrainedLinks = false;
    }

	globalNumConstraintVectors = 0;
	globalNumFrictionVectors = 0;
	constrainedLinkPairs.clear();

	setConstraintPoints(corbaCollisionSequence);

	if(globalNumConstraintVectors > 0){
		const bool constraintsSizeChanged = ((globalNumFrictionVectors   != prevGlobalNumFrictionVectors) ||
											 (globalNumConstraintVectors != prevGlobalNumConstraintVectors));

		if(constraintsSizeChanged){
			initMatrices();
		}


		if(SKIP_REDUNDANT_ACCEL_CALC){
			setAccelCalcSkipInformation();
		}

		BEGIN_TIMER(sam1);
	    setDefaultAccelerationVector();
		END_TIMER2(sam1);
		BEGIN_TIMER(sam2);
	    setAccelerationMatrix();
		END_TIMER2(sam2);

		if(globalNumConstraintVectors - globalNumContactNormalVectors > 0){
			clearSingularPointConstraintsOfClosedLoopConnections();
		}
		
		setConstantVectorAndMuBlock();


		bool isConverged;
		if(!USE_PREVIOUS_LCP_SOLUTION || constraintsSizeChanged){
			solution.setAllValue(0.0);
		}
		// M ddq + c = Jt f
		// ddq + invM *c =  invM*Jt f
		//  J ddq + J *invM *c = J*invM*Jt   f
		// so 
		//  A == Mlcp = JinvMJ'  
		// Mlcp*f = b                   ----(5)
		
		// smoothed dynamics version:
		// (M+MA) dv + c = J'f + u, 
		// and (A+R) *f = (v_star - v_)/h  --- (6)
		//
		//  then, 
		//  dv = inv(M+MA)*(J'f + u-c)    --- (1)
		//
		//  let v' be v + dv*h. Then,
		//  
		//  v'= v + inv(M+MA)*(J'f + u-c)*h --- (2)
		//   
		//  let v^ be v+inv(M+MA) *(u-c)*h. Then
		//
		//  v' = v^ + inv(M+MA)*J'f*h        --- (3)
		//
		//  By multiplying J on both sides of (3)
		//
		//  v2 = v_ + Af *h,                  --- (4)
		//  where v2=Jv' and v_ = Jv^.
		//
		// from (4), (5) and the fact that v'=0,
		//    b = (-v_)/h
		//
		// to modify (5) to  (6),
		//
		//    b should change to   b - (-v_)/h +(v_star - v_)/h
		//                        = b + v_star/h
		// 
		//
		//  
		//	desired contact velocity v*:
		//		kp =   (1+epsilon) / (kappa*kappa);
		//		kd = 2*(1+epsilon) / kappa;
		//  	v* : (normal*(vn + h*(kp * (-0 + depth) - kd * vn)));
		//
		// A*F = v_star - Jv^ 
		// , where impulse F is f*dt
		//
		// or A * f = (v_star- v_)/dt   --- (2)
		//
		assert(Mlcp.rows()==Mlcp.cols());
		Mlcp.diag()+=_R;

		BEGIN_TIMER(mcp);
		solveMCPByProjectedGaussSeidel(Mlcp, B, solution);
		END_TIMER2(mcp);
		isConverged = true;

		if(!isConverged){
			++numUnconverged;
				//std::cout << "LCP didn't converge" << numUnconverged << std::endl;
		} else {

			addConstraintForceToLinks();
		}
	}

	prevGlobalNumConstraintVectors = globalNumConstraintVectors;
	prevGlobalNumFrictionVectors = globalNumFrictionVectors;
}


void ContactForceSolver::setConstraintPoints(CollisionSequence& collisions)
{
    for(size_t i=0; i < collisionCheckLinkPairs.size(); ++i){

        LinkPair& linkPair = collisionCheckLinkPairs[i];
		CollisionPointSequence& points = collisions[i].points;

		if(points.size() > 0){
			constrainedLinkPairs.push_back(&linkPair);
			setContactConstraintPoints(linkPair, points);
			Msg::verify(linkPair.bodyData[0] && linkPair.bodyData[1], "Error! you need to call init after registerCollisionPairs");
			linkPair.bodyData[0]->hasConstrainedLinks = true;
			linkPair.bodyData[1]->hasConstrainedLinks = true;
		}
    }

	globalNumContactNormalVectors = globalNumConstraintVectors;

	for(size_t i=0; i < connectedLinkPairs.size(); ++i){
        LinkPair& linkPair = connectedLinkPairs[i];
		constrainedLinkPairs.push_back(&linkPair);
		//check
		if(setConnectionConstraintPoints(linkPair)){
			linkPair.bodyData[0]->hasConstrainedLinks = true;
			linkPair.bodyData[1]->hasConstrainedLinks = true;
		} else {
			constrainedLinkPairs.pop_back();
		}
    }
	globalNumConnectionVectors = globalNumConstraintVectors - globalNumContactNormalVectors;
}


void ContactForceSolver::setContactConstraintPoints(LinkPair& linkPair, CollisionPointSequence& collisionPoints)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;
	constraintPoints.clear();
	int numExtractedPoints = 0;
	int numContactsInPair = collisionPoints.size();

	for(int j=0; j < numContactsInPair; ++j){

		CollisionPoint& collision = collisionPoints[j];
		constraintPoints.push_back(ConstraintPoint());
		ConstraintPoint& contact = constraintPoints.back();

		contact.point= collision.position;
		contact._normalTowardInside0=-collision.normal;
		contact.depth = collision.idepth;

		bool isNeighborhood = false;


		if(isNeighborhood){
			constraintPoints.pop_back();
		} else {
			numExtractedPoints++;
			contact.globalIndex = globalNumConstraintVectors++;

			// check velocities
			vector3 v[2];
			for(int k=0; k < 2; ++k){
				Link* link = linkPair.link[k];
				v[k] = link->vo + cross(link->w, contact.point);
			}
			contact.relVelocityOn0 = v[1] - v[0];
			contact.normalProjectionOfRelVelocityOn0 = dot(contact.normalTowardInside(1), contact.relVelocityOn0);

			vector3 v_tangent(contact.relVelocityOn0 - contact.normalProjectionOfRelVelocityOn0 * contact.normalTowardInside(1));

			contact.globalFrictionIndex = globalNumFrictionVectors;

			double vt_square = dot(v_tangent, v_tangent);
			static const double vsqrthresh = VEL_THRESH_OF_DYNAMIC_FRICTION * VEL_THRESH_OF_DYNAMIC_FRICTION;
			bool isSlipping = (vt_square > vsqrthresh);
			contact.mu = isSlipping ? linkPair.muDynamic : linkPair.muStatic;
			contact.numFrictionVectors = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? 2 : 4);
			setFrictionVectors(contact);
			globalNumFrictionVectors += contact.numFrictionVectors;
		}
	}
}


void ContactForceSolver::setFrictionVectors(ConstraintPoint& contact)
{
	vector3 u(0.0);
	int minAxis = 0;
	vector3 normal = contact.normalTowardInside(0);

	for(int i=1; i < 3; i++){
		if(fabs(normal[i]) < fabs(normal[minAxis])){
			minAxis = i;
		}
	}
	u[minAxis] = 1.0;

	vector3 t1(cross(normal, u));
	t1 /= norm2(t1);

#ifdef TWO_VECTORS
	contact._frictionVector0[0] = t1;
	contact._frictionVector0[1] =cross(normal, t1);
#else
	contact._frictionVector0 = t1;
#endif
}


bool ContactForceSolver::setConnectionConstraintPoints(LinkPair& linkPair)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	Body::LinkConnection* connection = linkPair.connection;

	Link* link0 = connection->link[0];
	Link* link1 = connection->link[1];

	vector3 point[2];
	point[0] = link0->p + link0->R * connection->point[0];
	point[1] = link1->p + link1->R * connection->point[1];
	vector3 midPoint((point[0] + point[1]) / 2.0);
	vector3 error(midPoint - point[0]);

	if(dot(error, error) > (0.04 * 0.04)){
		return false;
	}

	// check velocities
	vector3 v[2];
	for(int k=0; k < 2; ++k){
		Link* link = connection->link[k];
		if(link->jointType == Link::FIXED_JOINT){
			v[k] = 0.0;
		} else {
			v[k] = link->vo + cross(link->w, point[k]);
		}
	}
	vector3 relVelocityOn0(v[1] - v[0]);

	for(int i=0; i < connection->numConstraintAxes; ++i){
		ConstraintPoint& constraint = constraintPoints[i];
		constraint.point = midPoint;
		const vector3 axis(link0->R * connection->constraintAxes[i]);
		constraint._normalTowardInside0 =  axis;
		constraint.depth = dot(axis, error);
		constraint.globalIndex = globalNumConstraintVectors++;

		constraint.normalProjectionOfRelVelocityOn0 = dot(constraint.normalTowardInside(1), relVelocityOn0);
	}

	return true;
}





void ContactForceSolver::initMatrices()
{
	const int n = globalNumConstraintVectors;
	const int m = globalNumFrictionVectors;

	const int dimLCP = n + m;

	Mlcp.setSize(dimLCP, dimLCP);
	B.setSize(dimLCP);
	solution.setSize(dimLCP);


	{
		frictionIndexToContactIndex.resize(m);
	}

	an0.setSize(n);
	at0.setSize(m);

	MCPsolver::initWorkspace();
}


void ContactForceSolver::setAccelCalcSkipInformation()
{
	// clear skip check numbers
	for(size_t i=0; i < bodiesData.size(); ++i){
		BodyData& bodyData = bodiesData[i];
		if(bodyData.hasConstrainedLinks){
			LinkDataArray& linksData = bodyData.linksData;
			for(size_t j=0; j < linksData.size(); ++j){
				linksData[j].numberToCheckAccelCalcSkip = numeric_limits<int>::max();
			}
		}
	}

	// add the number of contact points to skip check numbers of the links from a contact target to the root
	int numLinkPairs = constrainedLinkPairs.size();
	for(int i=0; i < numLinkPairs; ++i){
		LinkPair* linkPair = constrainedLinkPairs[i];

		//int constraintIndex = linkPair->constraintPoints.front().globalIndex;
		//mod
		int constraintIndex;
		if(linkPair->constraintPoints.size()!=0)
			constraintIndex=linkPair->constraintPoints.front().globalIndex;
		else
			constraintIndex = 0;



		for(int j=0; j < 2; ++j){
			LinkDataArray& linksData = linkPair->bodyData[j]->linksData;
			int linkIndex = linkPair->link[j]->index;
			while(linkIndex >= 0){
				LinkData& linkData = linksData[linkIndex];
				if(linkData.numberToCheckAccelCalcSkip < constraintIndex){
					break;
				}
				linkData.numberToCheckAccelCalcSkip = constraintIndex;
				linkIndex = linkData.parentIndex;
			}
		}
	}
}


void ContactForceSolver::setDefaultAccelerationVector()
{
	// calculate accelerations with no constraint force
	for(size_t i=0; i < bodiesData.size(); ++i){
		BodyData& bodyData = bodiesData[i];
		if(bodyData.hasConstrainedLinks && ! bodyData.isStatic){

			{
				initABMForceElementsWithNoExtForce(bodyData);
				calcAccelsABM(bodyData, numeric_limits<int>::max());
			}
		}
	}

	// extract accelerations
	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

		for(size_t j=0; j < constraintPoints.size(); ++j){
			ConstraintPoint& constraint = constraintPoints[j];

			for(int k=0; k < 2; ++k){
				if(linkPair.bodyData[k]->isStatic){
					constraint.defaultAccel[k] = 0.0;
				} else {
					Link* link = linkPair.link[k];
					LinkData* linkData = linkPair.linkData[k];
					constraint.defaultAccel[k] =
						linkData->dvo - cross(constraint.point, linkData->dw) +
						cross(link->w, vector3(link->vo + cross(link->w, constraint.point)));
				}
			}

			vector3 relDefaultAccel(constraint.defaultAccel[1] - constraint.defaultAccel[0]);
			an0[constraint.globalIndex] = dot(constraint.normalTowardInside(1), relDefaultAccel);
			
			//mod
			if(at0.size()>0&&constraint.globalFrictionIndex!=numeric_limits<int>::max()){
					for(int k=0; k<constraint.numFrictionVectors; k++){
						at0[constraint.globalFrictionIndex+k]=dot(constraint.frictionVector(k,1), relDefaultAccel);
					}
			}

		}
	}
}


void ContactForceSolver::setAccelerationMatrix()
{
	const int n = globalNumConstraintVectors;
	const int m = globalNumFrictionVectors;	

	
	//mod
	matrixnView Knn=Mlcp.range(0,n,0,n);
	matrixnView Knt=Mlcp.range(n,n+m,0,n);

	//matrixnView Knn=Mlcp.range( 0,n, 0,n);
	//matrixnView Ktn=Mlcp.range( 0,n, n,n+m);
	//matrixnView Knt=Mlcp.range( n,n+m, 0,n);
	//matrixnView Ktt=Mlcp.range( n,n+m, n,n+m);

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		int numConstraintsInPair = linkPair.constraintPoints.size();

		for(int j=0; j < numConstraintsInPair; ++j){

			ConstraintPoint& constraint = linkPair.constraintPoints[j];
			int constraintIndex = constraint.globalIndex;

			// apply test normal force
			for(int k=0; k < 2; ++k){
				BodyData& bodyData = *linkPair.bodyData[k];
				if(!bodyData.isStatic){

					bodyData.isTestForceBeingApplied = true;
					const vector3& f = constraint.normalTowardInside(k);

					{
						vector3 tau(cross(constraint.point, f));
						calcABMForceElementsWithTestForce(bodyData, linkPair.link[k], f, tau);
						if(!linkPair.isSameBodyPair || (k > 0)){
							calcAccelsABM(bodyData, constraintIndex);
						}
					}
				}
			}
			extractRelAccelsOfConstraintPoints(Knn, Knt, constraintIndex, constraintIndex);


			// apply test friction force(modify)
			if(m>0)
			{
				matrixnView Ktn=Mlcp.range(0,n,n,n+m);
				matrixnView Ktt=Mlcp.range(n,n+m,n,n+m);
				for(int l=0; l < constraint.numFrictionVectors; ++l){
					for(int k=0; k < 2; ++k){
						BodyData& bodyData = *linkPair.bodyData[k];
						if(!bodyData.isStatic){
							const vector3& f = constraint.frictionVector(l,k);
							{
								vector3 tau(cross(constraint.point, f));
								calcABMForceElementsWithTestForce(bodyData, linkPair.link[k], f, tau);
								if(!linkPair.isSameBodyPair || (k > 0)){
									calcAccelsABM(bodyData, constraintIndex);
								}
							}	
						}	
					}

					//extractRelAccelsOfConstraintPoints(Ktn, Ktt, constraint.globalFrictionIndex + l, constraintIndex);
					//mod
					if(constraint.globalFrictionIndex!=numeric_limits<int>::max())
					{
						extractRelAccelsOfConstraintPoints(Ktn,Ktt,constraint.globalFrictionIndex + l, constraintIndex);
					}

				}
			}


			linkPair.bodyData[0]->isTestForceBeingApplied = false;
			linkPair.bodyData[1]->isTestForceBeingApplied = false;
		}
	}

}


void ContactForceSolver::initABMForceElementsWithNoExtForce(BodyData& bodyData)
{
	bodyData.dpf   = 0;
	bodyData.dptau = 0;

	std::vector<LinkData>& linksData = bodyData.linksData;
    //const LinkTraverse& traverse = bodyData.body->linkTraverse();
	const Body& traverse=*bodyData.body;
    int n = traverse.numLinks();

    for(int i = n-1; i >= 0; --i){
        Link* link = traverse[i];
        LinkData& data = linksData[i];

		data.pf0   = link->pf;
		data.ptau0 = link->ptau;

        for(Link* child = link->child; child; child = child->sibling){

			LinkData& childData = linksData[child->index];

			data.pf0   += childData.pf0;
			data.ptau0 += childData.ptau0;

			double uu_dd = childData.uu0 / child->dd;
			data.pf0    += uu_dd * child->hhv;
			data.ptau0  += uu_dd * child->hhw;
		}

		if(i > 0){
			data.uu0  = link->uu + link->u - (dot(link->sv, data.pf0) + dot(link->sw, data.ptau0));
			data.uu = data.uu0;
		}
    }
}


void ContactForceSolver::calcABMForceElementsWithTestForce
(BodyData& bodyData, Link* linkToApplyForce, const vector3& f, const vector3& tau)
{
	std::vector<LinkData>& linksData = bodyData.linksData;

	vector3 dpf  (-f);
	vector3 dptau(-tau);

	Link* link = linkToApplyForce;
	while(link->parent){
		LinkData& data = linksData[link->index];
		double duu = -(dot(link->sv, dpf) + dot(link->sw, dptau));
		data.uu += duu;
		double duudd = duu / link->dd;
		dpf   += duudd * link->hhv;
		dptau += duudd * link->hhw;
		link = link->parent;
	}

	bodyData.dpf   += dpf;
	bodyData.dptau += dptau;
}


void ContactForceSolver::calcAccelsABM(BodyData& bodyData, int constraintIndex)
{
	std::vector<LinkData>& linksData = bodyData.linksData;
	LinkData& rootData = linksData[0];
    Link* rootLink = rootData.link;

    if(rootLink->jointType == Link::FREE_JOINT){

		vector3 pf  (rootData.pf0   + bodyData.dpf);
		vector3 ptau(rootData.ptau0 + bodyData.dptau);

		CMatrix66 Ia;
		setMatrix33(rootLink->Ivv, Ia, 0, 0);
		setTransMatrix33(rootLink->Iwv, Ia, 0, 3);
		setMatrix33(rootLink->Iwv, Ia, 3, 0);
		setMatrix33(rootLink->Iww, Ia, 3, 3);

		CVector6 p;
		setVector3(pf,   p, 0);
		setVector3(ptau, p, 3);
		p *= -1.0;

		Eigen::Matrix<double, 6,1> x=Ia.ldlt().solve(p);

        getVector3(rootData.dvo, x, 0);
		getVector3(rootData.dw,  x, 3);

    } else {
        rootData.dw  = 0.0;
        rootData.dvo = 0.0;
    }

	// reset
	bodyData.dpf   = 0;
	bodyData.dptau = 0;

	int skipCheckNumber =  (numeric_limits<int>::max() - 1);
    int n = linksData.size();
    for(int linkIndex = 1; linkIndex < n; ++linkIndex){

		LinkData& linkData = linksData[linkIndex];

		if(!SKIP_REDUNDANT_ACCEL_CALC || linkData.numberToCheckAccelCalcSkip <= skipCheckNumber){

			Link* link = linkData.link;
			LinkData& parentData = linksData[linkData.parentIndex];

			linkData.ddq = (linkData.uu - (dot(link->hhv, parentData.dvo) + dot(link->hhw, parentData.dw))) / link->dd;
			linkData.dvo = parentData.dvo + link->cv + link->sv * linkData.ddq;
			linkData.dw  = parentData.dw  + link->cw + link->sw * linkData.ddq;

			// reset
			linkData.uu   = linkData.uu0;
		}
	}
}



void ContactForceSolver::extractRelAccelsOfConstraintPoints
(matrixn& Kxn, matrixn& Kxt, int testForceIndex, int constraintIndex)
{
	int maxConstraintIndexToExtract =  globalNumConstraintVectors;

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];

		BodyData& bodyData0 = *linkPair.bodyData[0];
		BodyData& bodyData1 = *linkPair.bodyData[1];

		if(bodyData0.isTestForceBeingApplied){
			if(bodyData1.isTestForceBeingApplied){
				extractRelAccelsFromLinkPairCase1(Kxn, Kxt, linkPair, testForceIndex, maxConstraintIndexToExtract);
			} else {
				extractRelAccelsFromLinkPairCase2(Kxn, Kxt, linkPair, 0, 1, testForceIndex, maxConstraintIndexToExtract);
			}
		} else {
			if(bodyData1.isTestForceBeingApplied){
				extractRelAccelsFromLinkPairCase2(Kxn, Kxt, linkPair, 1, 0, testForceIndex, maxConstraintIndexToExtract);
			} else {
				extractRelAccelsFromLinkPairCase3(Kxn, Kxt, linkPair, testForceIndex, maxConstraintIndexToExtract);
			}
		}
	}
}


void ContactForceSolver::extractRelAccelsFromLinkPairCase1
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;


		Link* link0 = linkPair.link[0];
		Link* link1 = linkPair.link[1];
		LinkData* linkData0 = linkPair.linkData[0];
		LinkData* linkData1 = linkPair.linkData[1];

		//! \todo Can the follwoing equations be simplified ?
		vector3 dv0(linkData0->dvo - cross(constraint.point, linkData0->dw) + cross(link0->w, vector3(link0->vo + cross(link0->w, constraint.point))));
		vector3 dv1(linkData1->dvo - cross(constraint.point, linkData1->dw) + cross(link1->w, vector3(link1->vo + cross(link1->w, constraint.point))));

		vector3 relAccel(dv1 - dv0);

		Kxn(constraintIndex, testForceIndex) =
			dot(constraint.normalTowardInside(1), relAccel) - an0(constraintIndex);

		for(int j=0; j < constraint.numFrictionVectors; ++j){
			const int index = constraint.globalFrictionIndex + j;

			//mod
			if(constraint.globalFrictionIndex!=numeric_limits<int>::max())
			{
				if(at0.size()>0){
					Kxt(index, testForceIndex) = dot(constraint.frictionVector(j,1), relAccel) - at0(index);
				}
			}	
			//Kxt(index, testForceIndex) = dot(constraint.frictionVector(j,1), relAccel) - at0(index);
		}
	}
}


void ContactForceSolver::extractRelAccelsFromLinkPairCase2
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int maxConstraintIndexToExtract)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;


		Link* link = linkPair.link[iTestForce];
		LinkData* linkData = linkPair.linkData[iTestForce];

		vector3 dv(linkData->dvo - cross(constraint.point, linkData->dw) + cross(link->w, vector3(link->vo + cross(link->w, constraint.point))));


		vector3 relAccel(constraint.defaultAccel[iDefault] - dv);

		Kxn(constraintIndex, testForceIndex) =
			dot(constraint.normalTowardInside(iDefault), relAccel) - an0(constraintIndex);

		for(int j=0; j < constraint.numFrictionVectors; ++j){
			const int index = constraint.globalFrictionIndex + j;

			//mod	
			if(at0.size()>0){
				Kxt(index, testForceIndex) =
					dot(constraint.frictionVector(j,iDefault), relAccel) - at0(index);
			}
		
			//Kxt(index, testForceIndex) =
				//dot(constraint.frictionVector(j,iDefault), relAccel) - at0(index);
		}

	}
}


void ContactForceSolver::extractRelAccelsFromLinkPairCase3
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;

		Kxn(constraintIndex, testForceIndex) = 0.0;

		for(int j=0; j < constraint.numFrictionVectors; ++j){
			Kxt(constraint.globalFrictionIndex + j, testForceIndex) = 0.0;
		}
	}
}



void ContactForceSolver::clearSingularPointConstraintsOfClosedLoopConnections()
{
	for(int i = globalNumContactNormalVectors; i < globalNumConstraintVectors; ++i){
		if(Mlcp(i, i) < 1.0e-4){
			for(size_t j=0; j < Mlcp.rows(); ++j){
				Mlcp(j, i) = 0.0;
			}
			Mlcp(i, i) = numeric_limits<double>::max();
		}
	}
}


void ContactForceSolver::setConstantVectorAndMuBlock()
{
	double dtinv = 1.0 / world.timeStep();
	const int block2 = globalNumConstraintVectors;
	const int block3 = globalNumConstraintVectors + globalNumFrictionVectors;

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		int numConstraintsInPair = linkPair.constraintPoints.size();

		for(int j=0; j < numConstraintsInPair; ++j){
			ConstraintPoint& constraint = linkPair.constraintPoints[j];
			int globalIndex = constraint.globalIndex;

			// set constant vector of LCP

			// constraints for normal acceleration

			if(linkPair.connection){
				// connection constraint

				const double& error = constraint.depth;
				double v;
				if(error >= 0){
					v = 0.1 * (-1.0 + exp(-error * 20.0));
				} else {
					v = 0.1 * ( 1.0 - exp( error * 20.0));
				}
					
				B(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 + v) * dtinv;

			} else {
				// contact constraint

				double v_star=0.0;
				if(USE_SMOOTHED_CONTACT_DYNAMICS)
				{
					double kp =   (1+epsilon) / (kappa*kappa);
					double kd = 2*(1+epsilon) / kappa;
					double h=world.timeStep();
					double vn=constraint.normalProjectionOfRelVelocityOn0;
					v_star= vn + h*(kp * (-0 + constraint.depth) - kd * vn);

					//printf("2: %f %f %f %f\n", vn, constraint.depth, h, v_star);
				}

				if(ALLOW_SUBTLE_PENETRATION_FOR_STABILITY && constraint.depth < ALLOWED_PENETRATION_DEPTH){

					double extraNegativeVel = (ALLOWED_PENETRATION_DEPTH - constraint.depth) * NEGATIVE_VELOCITY_RATIO_FOR_ALLOWING_PENETRATION;
					B(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 + extraNegativeVel) * dtinv;
				} else {
					B(globalIndex) = an0(globalIndex) + constraint.normalProjectionOfRelVelocityOn0 * dtinv;
				}
				// modify b to a smoothed dynamics version
				B(globalIndex)+= -v_star*dtinv;

				//printf("mu %f\n", constraint.mu);
				contactIndexToMu(globalIndex) = constraint.mu;

				int globalFrictionIndex = constraint.globalFrictionIndex;
				for(int k=0; k < constraint.numFrictionVectors; ++k){

					// constraints for tangent acceleration
					double tangentProjectionOfRelVelocity = dot(constraint.frictionVector(k,1), constraint.relVelocityOn0);

					B(block2 + globalFrictionIndex) = at0(globalFrictionIndex);
					if( !IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION ){
						B(block2 + globalFrictionIndex) += tangentProjectionOfRelVelocity * dtinv;
					}

					// for iterative solver
					frictionIndexToContactIndex[globalFrictionIndex] = globalIndex;

					++globalFrictionIndex;
				}
			}
		}
	}
}


void ContactForceSolver::addConstraintForceToLinks()
{
    int n = constrainedLinkPairs.size();
    for(int i=0; i < n; ++i){
		LinkPair* linkPair = constrainedLinkPairs[i];
		for(int j=0; j < 2; ++j){
			if(linkPair->link[j]->jointType != Link::FIXED_JOINT){
				addConstraintForceToLink(linkPair, j);
			}
		}
    }
}


void ContactForceSolver::addConstraintForceToLink(LinkPair* linkPair, int ipair)
{
    vector3 f_total(0.0);
    vector3 tau_total(0.0);

    ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
    int numConstraintPoints = constraintPoints.size();

    for(int i=0; i < numConstraintPoints; ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int globalIndex = constraint.globalIndex;

		vector3 f(solution(globalIndex) * constraint.normalTowardInside(ipair));

		for(int j=0; j < constraint.numFrictionVectors; ++j){
			//mod
			if(constraint.globalFrictionIndex!=numeric_limits<int>::max()){
				f += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
			}
			//f += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
		}

		f_total   += f;
		tau_total += cross(constraint.point, f);
    }

    Link* link = linkPair->link[ipair];
    link->fext   += f_total;
    link->tauext += tau_total;


#ifdef VERBOSE
		std::cout << "Constraint force to " << link->name << ": f = " << f_total << ", tau = " << tau_total << std::endl;
#endif
}





ContactForceSolver::ContactForceSolver(WorldBase& _world)
	:
	world(_world)
{
	USE_SMOOTHED_CONTACT_DYNAMICS = true;
	// recommended for low-freq control:
	// kappa=0.05; epsilon=0.1;
	// rigid:
	kappa=0.01; epsilon=0.01;
	_R=1.0e-5;
}


ContactForceSolver::~ContactForceSolver()
{
}

















void ContactForceSolver::setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError)
{
	ContactForceSolver	*impll=this;
	impll->maxNumGaussSeidelIteration = maxNumIteration;
	impll->numGaussSeidelInitialIteration = numInitialIteration;
	impll->gaussSeidelMaxRelError = maxRelError;
}
