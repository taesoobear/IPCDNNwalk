// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * Copyright (c) 2010, Hanyang University
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/**
   \file
   \originalauthor Shin'ichiro Nakaoka (AIST)
   \author Taesoo Kwon (HYU)
*/
#include "physicsLib.h"
#include "Body.h"
#include "ForwardDynamicsABM.h"
#include "Link.h"
#include "LinkTraverse.h"
#include "../../BaseLib/math/Operator_NR.h"


#include "eigenSupport.h"
using namespace TRL;
using namespace std;

void ForwardDynamicsABM::setTimeStep(double ts)
{
    timeStep = ts;
}


void ForwardDynamicsABM::setGravityAcceleration(const vector3& g)
{
    this->g = g;
}


void ForwardDynamicsABM::setEulerMethod()
{
    integrationMode = EULER_METHOD;
}


void ForwardDynamicsABM_rungeKutta::setRungeKuttaMethod()
{
    integrationMode = RUNGEKUTTA_METHOD;
}



/// function from Murray, Li and Sastry p.42
void ForwardDynamicsABM::SE3exp(vector3& out_p, matrix33& out_R,
							 const vector3& p0, const matrix33& R0,
							 const vector3& w, const vector3& vo, double dt)
{
    using ::std::numeric_limits;
	
    double norm_w = sqrt(w%w);
	
    if(norm_w < numeric_limits<double>::epsilon() ) {
		out_p = p0 + vo * dt;
		out_R = R0;
    } else {
		double th = norm_w * dt;
		vector3 w_n(w / norm_w);
		vector3 vo_n(vo / norm_w);
		matrix33 rot = rodrigues(w_n, th);
		
		out_p = rot * p0 + (identity33() - rot) * vector3(cross(w_n, vo_n)) + VVt_prod(w_n, w_n) * vo_n * th;
		out_R = rot * R0;
    }
}


	





ForwardDynamicsABM::ForwardDynamicsABM(BodyPtr body) :
	body(body)
{
    g .zero();
    timeStep = 0.005;


    integrationMode = EULER_METHOD;
}
ForwardDynamicsABM_rungeKutta::ForwardDynamicsABM_rungeKutta(BodyPtr body) :
	ForwardDynamicsABM(body),
	q0(body->numLinks()),
	dq0(body->numLinks()),
	dq(body->numLinks()),
	ddq(body->numLinks())
{
    integrationMode = RUNGEKUTTA_METHOD;
}

ForwardDynamicsABM_rungeKutta::~ForwardDynamicsABM_rungeKutta()
{

}


ForwardDynamicsABM::~ForwardDynamicsABM()
{

}



void ForwardDynamicsABM_rungeKutta::calcNextState()
{
	calcMotionWithRungeKuttaMethod();
	calcABMFirstHalf();
}
void ForwardDynamicsABM::calcNextState()
{
	// collect forces, calculate accelerations
	// and then update velocities and positions
	calcMotionWithEulerMethod();
	calcABMFirstHalf();
}

void ForwardDynamicsABM::_updatePositionEuler()
{
    Link* root = body->rootLink();
	bool rootDof=root->jointType != Link::FIXED_JOINT;

    if(rootDof){
        vector3 p;
        matrix33 R;
        SE3exp(p, R, root->p, root->R, root->w, root->vo, timeStep);
        root->p = p;
        root->R = R;
    }

	int n = body->numLinks();
    for(int i=1; i < n; ++i){
        Link* link = body->link(i);
        link->q  += link->dq  * timeStep;
    }
}


void ForwardDynamicsABM::_updatePositionAndVelocityEuler()
{
    Link* root = body->rootLink();
	bool rootDof=root->jointType != Link::FIXED_JOINT;

    if(rootDof){
        vector3 p;
        matrix33 R;
        SE3exp(p, R, root->p, root->R, root->w, root->vo, timeStep);
        root->p = p;
        root->R = R;

        root->vo += root->dvo * timeStep;
		root->w  += root->dw  * timeStep;
    }

	int n = body->numLinks();
    for(int i=1; i < n; ++i){
        Link* link = body->link(i);
        link->q  += link->dq  * timeStep;
        link->dq += link->ddq * timeStep;
    }
}


void ForwardDynamicsABM_rungeKutta::integrateRungeKuttaOneStep(double r, double dt)
{
    Link* root = body->rootLink();

    if(root->jointType != Link::FIXED_JOINT){

        SE3exp(root->p, root->R, p0, R0, root->w, root->vo, dt);
        root->vo = vo0 + root->dvo * dt;
		root->w  = w0 + root->dw  * dt;

		vo += r * root->vo;
		w += r * root->w;
		dvo += r * root->dvo;
		dw += r * root->dw;
    }

    int n = body->numLinks();
    for(int i=1; i < n; ++i){

        Link* link = body->link(i);

		link->q = q0[i] + dt * link->dq;
		link->dq = dq0[i] + dt * link->ddq;

		dq[i] += r * link->dq;
		ddq[i] += r * link->ddq;
    }
}


void ForwardDynamicsABM_rungeKutta::calcMotionWithRungeKuttaMethod()
{
    Link* root = body->rootLink();

    if(root->jointType != Link::FIXED_JOINT){
		p0  = root->p;
		R0  = root->R;
		vo0  = root->vo;
		w0   = root->w;
    }

    vo = 0;
    w = 0;
    dvo = 0;
    dw = 0;

    int n = body->numLinks();
    for(int i=1; i < n; ++i){
		Link* link = body->link(i);
        q0[i] = link->q;
		dq0[i] = link->dq;
		dq[i] = 0.0;
		ddq[i] = 0.0;
    }

	calcABMLastHalf();


    integrateRungeKuttaOneStep(1.0 / 6.0, timeStep / 2.0);
	calcABMPhase1();
	calcABMPhase2();
	calcABMPhase3();

    integrateRungeKuttaOneStep(2.0 / 6.0, timeStep / 2.0);
	calcABMPhase1();
	calcABMPhase2();
	calcABMPhase3();

    integrateRungeKuttaOneStep(2.0 / 6.0, timeStep);
	calcABMPhase1();
	calcABMPhase2();
	calcABMPhase3();

    if(root->jointType != Link::FIXED_JOINT){
        SE3exp(root->p, root->R, p0, R0, w0, vo0, timeStep);
        root->vo = vo0 + (dvo + root->dvo / 6.0) * timeStep;
		root->w  = w0 + (dw + root->dw / 6.0) * timeStep;
    }

    for(int i=1; i < n; ++i){
		Link* link = body->link(i);
		link->q = q0[i] + (dq[i] + link->dq / 6.0) * timeStep;
		link->dq = dq0[i] + (ddq[i] + link->ddq / 6.0) * timeStep;
    }
}


void ForwardDynamicsABM::calcPositionAndVelocityFK()
{
    //const LinkTraverse& traverse = body->linkTraverse();
    const Body& traverse = *body;
    int n = traverse.numLinks();

    for(int i=0; i < n; ++i){
        Link* link = traverse[i];
        Link* parent = link->parent;

		if(parent){

			switch(link->jointType){
			case Link::ROTATIONAL_JOINT:
				link->R  = parent->R * rodrigues(link->a, link->q);
				link->p  = parent->R * link->b + parent->p;
				link->sw = parent->R * link->a;
				link->sv = cross(link->p, link->sw);
				link->w  = link->dq * link->sw + parent->w;
				break;

			case Link::SLIDE_JOINT:
				link->p  = parent->R * (link->b + link->q * link->d) + parent->p;
				link->R  = parent->R;
				link->sw = 0.0;
				link->sv = parent->R * link->d;
				link->w  = parent->w;
				break;

			case Link::FIXED_JOINT:
			default:
				link->p  = parent->R * link->b + parent->p;
				link->R = parent->R;
				link->w  = parent->w;
				link->vo = parent->vo;
				link->v = parent->v;
				link->sw = 0.0;
				link->sv = 0.0;
				link->cv = 0.0;
				link->cw = 0.0;
				goto COMMON_CALCS_FOR_ALL_JOINT_TYPES;
			}

			// Common for ROTATE and SLIDE
			link->vo = link->dq * link->sv + parent->vo;
			vector3 dsv(cross(parent->w, link->sv) + cross(parent->vo, link->sw));
			vector3 dsw(cross(parent->w, link->sw));
			link->cv = link->dq * dsv;
			link->cw = link->dq * dsw;
		}

	COMMON_CALCS_FOR_ALL_JOINT_TYPES:
		link->v = link->vo + cross(link->w, link->p);

		link->wc = link->R * link->c + link->p;
		matrix33 Iw(matrix33(link->R * link->I)  * trans(link->R));
        matrix33 c_hat(hat(link->wc));
		link->Iww = link->m * (c_hat * trans(c_hat)) + Iw;
		link->Iwv = link->m * c_hat;

		vector3 P(link->m * (link->vo + cross(link->w, link->wc)));
		vector3 L(link->Iww * link->w + link->m * cross(link->wc, link->vo));

		link->pf   = cross(link->w,  P);
		link->ptau = cross(link->vo, P) + cross(link->w, L);
	}
}


void ForwardDynamicsABM::calcABMPhase2()
{
    //const LinkTraverse& traverse = body->linkTraverse();
	const Body& traverse=*body;
    int n = traverse.numLinks();

    for(int i = n-1; i >= 0; --i){
        Link* link = traverse[i];

		link->pf   -= link->fext;
		link->ptau -= link->tauext;

        for(Link* child = link->child; child; child = child->sibling){

			vector3 hhv_dd(child->hhv / child->dd);
			link->Ivv += child->Ivv - VVt_prod(child->hhv, hhv_dd);
			link->Iwv += child->Iwv - VVt_prod(child->hhw, hhv_dd);
			link->Iww += child->Iww - VVt_prod(child->hhw, vector3(child->hhw / child->dd));

			link->pf   += child->Ivv * child->cv + trans(child->Iwv) * child->cw + child->pf;
			link->ptau += child->Iwv * child->cv + child->Iww * child->cw + child->ptau;

			double uu_dd = child->uu / child->dd;
			link->pf   += uu_dd * child->hhv;
			link->ptau += uu_dd * child->hhw;
		}

		if(i > 0){
			link->hhv = link->Ivv * link->sv + trans(link->Iwv) * link->sw;
			link->hhw = link->Iwv * link->sv + link->Iww * link->sw;

			link->dd = dot(link->sv, link->hhv) + dot(link->sw, link->hhw) + link->Jm2;

			link->uu = link->u - (dot(link->hhv, link->cv) + dot(link->hhw, link->cw)
								  + dot(link->sv, link->pf) + dot(link->sw, link->ptau));
		}
    }
}


// A part of phase 2 (inbound loop) that can be calculated before external forces are given
void ForwardDynamicsABM::calcABMPhase2Part1()
{
    //const LinkTraverse& traverse = body->linkTraverse();
	const Body& traverse=*body;
    int n = traverse.numLinks();

    for(int i = n-1; i >= 0; --i){
        Link* link = traverse[i];

        for(Link* child = link->child; child; child = child->sibling){

			vector3 hhv_dd(child->hhv / child->dd);
			link->Ivv += child->Ivv - VVt_prod(child->hhv, hhv_dd);
			link->Iwv += child->Iwv - VVt_prod(child->hhw, hhv_dd);
			link->Iww += child->Iww - VVt_prod(child->hhw, vector3(child->hhw / child->dd));

			link->pf   += child->Ivv * child->cv + trans(child->Iwv) * child->cw;
			link->ptau += child->Iwv * child->cv + child->Iww * child->cw;
		}

		if(i > 0){
			link->hhv = link->Ivv * link->sv + trans(link->Iwv) * link->sw;
			link->hhw = link->Iwv * link->sv + link->Iww * link->sw;
			link->dd  = dot(link->sv, link->hhv) + dot(link->sw, link->hhw) + link->Jm2;
			link->uu  = - (dot(link->hhv, link->cv) + dot(link->hhw, link->cw));
		}
    }
}


// A remining part of phase 2 that requires external forces
void ForwardDynamicsABM::calcABMPhase2Part2()
{
    //const LinkTraverse& traverse = body->linkTraverse();
	const Body& traverse=*body;
    int n = traverse.numLinks();

    for(int i = n-1; i >= 0; --i){
        Link* link = traverse[i];

		link->pf   -= link->fext;
		link->ptau -= link->tauext;

        for(Link* child = link->child; child; child = child->sibling){
			link->pf   += child->pf;
			link->ptau += child->ptau;

			double uu_dd = child->uu / child->dd;
			link->pf   += uu_dd * child->hhv;
			link->ptau += uu_dd * child->hhw;
		}

		if(i > 0){
			link->uu += link->u - (dot(link->sv, link->pf) + dot(link->sw, link->ptau));
		}
    }
}


void ForwardDynamicsABM::calcABMPhase3()
{
    //const LinkTraverse& traverse = body->linkTraverse();
	const Body& traverse=*body;

    Link* root = traverse[0];

    if(root->jointType == Link::FREE_JOINT){

		// - | Ivv  trans(Iwv) | * | dvo | = | pf   |
		//   | Iwv     Iww     |   | dw  |   | ptau |

		//matrixn Ia(6,6);
		CMatrix66 Ia;
		setMatrix33(root->Ivv, Ia, 0, 0);
		setTransMatrix33(root->Iwv, Ia, 0, 3);
		setMatrix33(root->Iwv, Ia, 3, 0);
		setMatrix33(root->Iww, Ia, 3, 3);

		//vectorn p(6);
		CVector6 p;
		setVector3(root->pf, p, 0);
		setVector3(root->ptau, p, 3);
		p *= -1.0;

		//vectorn x; m::LUsolve(Ia, p, x);
		Eigen::Matrix<double, 6,1> x=Ia.ldlt().solve(p);

        getVector3(root->dvo, x, 0);
		getVector3(root->dw, x, 3);

    } else {
		// 기구학적으로 세팅 될 수도 있음.
        //root->dvo .zero();
        //root->dw .zero();
    }

    int n = traverse.numLinks();
    for(int i=1; i < n; ++i){

        Link* link = traverse[i];
        Link* parent = link->parent;

		link->ddq = (link->uu - (dot(link->hhv, parent->dvo) + dot(link->hhw, parent->dw))) / link->dd;
        link->dvo = parent->dvo + link->cv + link->sv * link->ddq;
        link->dw  = parent->dw  + link->cw + link->sw * link->ddq;
    }
}

void ForwardDynamicsABM::calcAccelFK()
{
    //const LinkTraverse& traverse = body->linkTraverse();
	const Body& traverse=*body;
    int n = traverse.numLinks();
    for(int i=1; i < n; ++i){
        Link* link = traverse[i];
        Link* parent = link->parent;
        link->dvo = parent->dvo + link->cv + link->sv * link->ddq;
        link->dw  = parent->dw  + link->cw + link->sw * link->ddq;
	}
}


