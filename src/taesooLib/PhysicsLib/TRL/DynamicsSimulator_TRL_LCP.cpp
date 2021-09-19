// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hanyang university.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * Taesoo Kwon
 */
/** @file DynamicsSimulator/server/DynamicsSimulator_TRL_LCP.cpp
 *
 */
#include "physicsLib.h"
#include "DynamicsSimulator.h"
#include "Body.h"
#include <vector>
#include <map>
#include <algorithm>
#include "../../BaseLib/math/Operator_NR.h"
#include "../../BaseLib/math/Operator.h"
#include "../../BaseLib/math/conversion.h"

#include "eigenSupport.h"
#include "DynamicsSimulator_TRL_LCP.h"
#include "Link.h"
#include "LinkTraverse.h"
#include "LinkPath.h"
#include "ModelLoaderUtil.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "ForwardDynamicsABM.h"
#include "TRL_common.h"
#include "ContactForceSolver.h"
#include "../../BaseLib/utility/QPerformanceTimer.h"
#ifndef NO_OGRE
#include <OgreSceneNode.h>
#include "../../MainLib/OgreFltk/renderer.h"
#endif

inline vectornView vec(const double* v, int size)
{
	return vectornView((double*)v, size, 1);
}
//#include <sml.h>
//#include "pgs.h"
using namespace OpenHRP;
using namespace std;
using namespace TRL;

// #define INTEGRATOR_DEBUG
static const int debugMode = false;
static const bool enableTimeMeasure = false;

#if 1
// to disable profiling, set 1
#undef BEGIN_TIMER
#undef END_TIMER2
#define BEGIN_TIMER(x)
#define END_TIMER2(x)
#endif

// copy constructor. 
DynamicsSimulator_TRL_LCP::DynamicsSimulator_TRL_LCP(DynamicsSimulator_TRL_LCP const& other)
:DynamicsSimulator_TRL_penalty(false)
{

	_contactForceSolver=new TRL::ContactForceSolver(world);
	int n = other._characters.size();
	for(int i=0; i<n; i++)
	{
		VRMLloader const& skel=other.skeleton(i);
		registerCharacter((VRMLloader*)&skel);
	}
	int nlinkpairs=other.collisionDetector->getCollisionPairs().size();
	for(size_t i=0; i < nlinkpairs; ++i)
	{
		LinkPair const& linkPair = other.collisionDetector->getCollisionPairs()[i];
		registerCollisionCheckPair(linkPair.charName1, linkPair.linkName1, linkPair.charName2, linkPair.linkName2, linkPair.param);
		//collisionDetector->addCollisionPair(linkPair, false, false);
	}
	setGVector(other.world.getGravityAcceleration());
	setParam_Epsilon_Kappa(other._contactForceSolver->epsilon, other._contactForceSolver->kappa);
	setParam_R_B_MA(other._contactForceSolver->_R, 0, other._MA);
	init(((DynamicsSimulator_TRL_LCP&)other).getTimestep(), OpenHRP::DynamicsSimulator::EULER);
}

DynamicsSimulator_TRL_LCP::DynamicsSimulator_TRL_LCP(bool useSimpleColdet)
:DynamicsSimulator_TRL_penalty(useSimpleColdet)
{
	// dshan comment:
	// k,  epsilon = 0.01, 0.01 -> rigid
	// k,  epsilon = 0.05, 0.10 -> recommended
	// k,  epsilon = 0.10, 1.00 -> smooth
	_MA=0;
	_contactForceSolver=new TRL::ContactForceSolver(world);
}
DynamicsSimulator_TRL_LCP::DynamicsSimulator_TRL_LCP(const char* coldet)
:DynamicsSimulator_TRL_penalty(coldet)
{
	// dshan comment:
	// k,  epsilon = 0.01, 0.01 -> rigid
	// k,  epsilon = 0.05, 0.10 -> recommended
	// k,  epsilon = 0.10, 1.00 -> smooth
	_MA=0;
	_contactForceSolver=new TRL::ContactForceSolver(world);
}



DynamicsSimulator_TRL_LCP::~DynamicsSimulator_TRL_LCP()
{
	delete _contactForceSolver;
}

#include "ContactForceSolver.h"
vector3 DynamicsSimulator_TRL_LCP::getContactForce(int ichar, int ibone) const
{
	vectorn const& f=_f;

    int n = _contactForceSolver->constrainedLinkPairs.size();
	if(n==0) return vector3 (0,0,0);

	intvectorn convLPindex(collisionDetector->getCollisionPairs().size());
	convLPindex.setAllValue(-1);
	int nn=0;
	assert(collisions->seq.size()==convLPindex.size() );
	for(int i=0; i<convLPindex.size(); i++)
	{
		CollisionPointSequence& points = (*collisions)[i].points;
		int n_point = points.size();
		if(n_point>0)
			convLPindex(nn++)=i;
	}

	assert(n==nn);
	vectorn& solution=_contactForceSolver->solution;
	int globalNumConstraintVectors=_contactForceSolver->globalNumConstraintVectors;

	vector3 contactForce(0,0,0);

    for(int i=0; i < n; ++i){
		TRL::ContactForceSolver::LinkPair* linkPair = _contactForceSolver->constrainedLinkPairs[i];
		const OpenHRP::LinkPair& linkPair2 = collisionDetector->getCollisionPairs()[convLPindex[i]];

		for(int j=0; j < 2; ++j){
			if(linkPair->link[j]->jointType != Link::FIXED_JOINT){
				int ipair=j;
				TRL::ContactForceSolver::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();

				for(int i=0; i < numConstraintPoints; ++i){
					TRL::ContactForceSolver::ConstraintPoint& constraint = constraintPoints[i];
					int globalIndex = constraint.globalIndex;

					assert(linkPair->bodyIndex[0]==linkPair2.charIndex[0]);
					assert(linkPair->bodyIndex[1]==linkPair2.charIndex[1]);

					vector3 ff(solution(globalIndex) * constraint.normalTowardInside(ipair));

					for(int j=0; j < 2; ++j){
						ff += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
					}

					if(linkPair->bodyIndex[0]==ichar && linkPair2.link[0]->treeIndex()==ibone)
					{
						contactForce+=ff;
					}
					if(linkPair->bodyIndex[1]==ichar && linkPair2.link[1]->treeIndex()==ibone)
					{
						contactForce+=ff;
					}
				}
			}
		}
	}
	return contactForce;
}
Liegroup::dse3 DynamicsSimulator_TRL_LCP::getCOMbasedContactForce(int ichar, int ibone) const
{
	Liegroup::dse3 out(0);

	vectorn const& f=_f;

    int n = _contactForceSolver->constrainedLinkPairs.size();
	if(n==0) return out;

	intvectorn convLPindex(collisionDetector->getCollisionPairs().size());
	convLPindex.setAllValue(-1);
	int nn=0;
	assert(collisions->seq.size()==convLPindex.size() );
	for(int i=0; i<convLPindex.size(); i++)
	{
		CollisionPointSequence& points = (*collisions)[i].points;
		int n_point = points.size();
		if(n_point>0)
			convLPindex(nn++)=i;
	}

	assert(n==nn);
	vectorn& solution=_contactForceSolver->solution;
	int globalNumConstraintVectors=_contactForceSolver->globalNumConstraintVectors;

	vector3 contactForce(0,0,0);
	vector3 contactTorque(0,0,0);

	VRMLloader* skel=_characters[ichar]->skeleton;
	VRMLTransform& bone=skel->VRMLbone(ibone);
	vector3 com=getWorldState(ichar).global(ibone)*bone.localCOM();

    for(int i=0; i < n; ++i){
		TRL::ContactForceSolver::LinkPair* linkPair = _contactForceSolver->constrainedLinkPairs[i];
		const OpenHRP::LinkPair& linkPair2 = collisionDetector->getCollisionPairs()[convLPindex[i]];
		CollisionPointSequence& points = (*collisions)[convLPindex[i]].points;

		for(int j=0; j < 2; ++j){
			if(linkPair->link[j]->jointType != Link::FIXED_JOINT){
				int ipair=j;
				TRL::ContactForceSolver::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();

				int n_point = points.size();
				Msg::verify(n_point==numConstraintPoints, "???");

				for(int i=0; i < numConstraintPoints; ++i){
					TRL::ContactForceSolver::ConstraintPoint& constraint = constraintPoints[i];
					int globalIndex = constraint.globalIndex;

					assert(linkPair->bodyIndex[0]==linkPair2.charIndex[0]);
					assert(linkPair->bodyIndex[1]==linkPair2.charIndex[1]);

					vector3 ff(solution(globalIndex) * constraint.normalTowardInside(ipair));

					for(int j=0; j < 2; ++j){
						ff += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
					}

					const vector3& pos=points[i].position;  // global
					if(linkPair->bodyIndex[0]==ichar && linkPair2.link[0]->treeIndex()==ibone)
					{
						contactForce+=ff;
						vector3 r=pos-com;
						contactTorque+=r.cross(ff);
					}
					if(linkPair->bodyIndex[1]==ichar && linkPair2.link[1]->treeIndex()==ibone)
					{
						contactForce+=ff;
						vector3 r=pos-com;
						contactTorque+=r.cross(ff);
					}
				}
			}
		}
	}
	out.M()=contactTorque;
	out.F()=contactForce;
	return out;
}

void DynamicsSimulator_TRL_LCP::drawLastContactForces(vector3 const& draw_offset)
{
	static ObjectList g_debugDraw;
	int ichar=0;
	get_contact_pos(ichar, contactPos, *collisions);

	vectorn& f=_f;

	f.setSize(contactPos.size()*3);

    int n = _contactForceSolver->constrainedLinkPairs.size();
	vectorn& solution=_contactForceSolver->solution;
	int globalNumConstraintVectors=_contactForceSolver->globalNumConstraintVectors;
	int cc=0;

    for(int i=0; i < n; ++i){
		TRL::ContactForceSolver::LinkPair* linkPair = _contactForceSolver->constrainedLinkPairs[i];
		for(int j=0; j < 2; ++j){
			if(linkPair->link[j]->jointType != Link::FIXED_JOINT){
				int ipair=j;
				TRL::ContactForceSolver::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();

				for(int i=0; i < numConstraintPoints; ++i){
					TRL::ContactForceSolver::ConstraintPoint& constraint = constraintPoints[i];
					int globalIndex = constraint.globalIndex;

					vector3 ff(solution(globalIndex) * constraint.normalTowardInside(ipair));

					for(int j=0; j < 2; ++j){
						ff += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
					}

					if(linkPair->bodyIndex[0]==ichar)
					{
						f.setVec3(cc*3, ff);
						cc++;
					}
					if(linkPair->bodyIndex[1]==ichar)
					{
						f.setVec3(cc*3, ff);
						cc++;
					}
				}
			}
		}
	}
	assert(cc==contactPos.size());

	if (f.size()>0){
		int noc=f.size()/3;
		double * ptr=&f(0);
		// draw forces
		vector3N lines;
		double _contactForceVis=0.001;
		for(int i = 0; i < noc; ++i) {
			//contact.set_contact_force(i, ptr+i*3);
			vector3 p=contactPos[i];
			vector3 f(ptr[i*3], ptr[i*3+1], ptr[i*3+2]);
			lines.pushBack(p*100);
			lines.pushBack((p+f*_contactForceVis)*100);
			//cout <<"f" << f <<endl;
		}
		g_debugDraw.registerObject("contactForce222", "LineList", "solidred", matView(lines));
#ifndef NO_OGRE
		g_debugDraw.getCurrRootSceneNode()->setPosition(ToOgre(draw_offset));
#endif
	}
	else
	{
		g_debugDraw.clear();
	}
}

void DynamicsSimulator_TRL_LCP::init(double timeStep,
	  OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
{
	DynamicsSimulator_TRL_penalty::init(timeStep, integrateOpt);
	_contactForceSolver->initialize();
	if(_contactForceSolver->collisionCheckLinkPairs.size()>0)
	{
		for(int i=0; i<_contactForceSolver->collisionCheckLinkPairs.size(); i++)
		{
			assert(_contactForceSolver->collisionCheckLinkPairs[i].bodyData[0]);
			assert(_contactForceSolver->collisionCheckLinkPairs[i].bodyData[1]);
		}
	}
}

void DynamicsSimulator_TRL_LCP::initSimulation()
{
#if 1
	world.initialize(); // moved to init (which is less frequently called)
	_contactForceSolver->initialize();
#else // buggy but faster.
	int n = _characters.size();
	for(int i=0; i<n; i++){
		TRL::ForwardDynamicsABM* fd=world.forwardDynamics(i);
		if(!fd) Msg::error("??? fd==NULL. did you call init(...)");
		fd->calcPositionAndVelocityFK();
	}
#endif
	_updateCharacterPose();
}

void DynamicsSimulator_TRL_LCP::setParam_Epsilon_Kappa(double eps, double kap)
{
	_contactForceSolver->epsilon=eps;
	_contactForceSolver->kappa=kap;
}
void DynamicsSimulator_TRL_LCP::setParam_R_B_MA(double r, double b, double ma)
{
	_contactForceSolver->_R=r;
	_MA=ma;
	for(int ichara=0; ichara<world.numBodies(); ichara++)
	{
		TRL::BodyPtr cinfo=world.body(ichara);
		TRL::Link* j;
		for(int i=-1, ni=cinfo->numJoints(); i<ni; i++)
		{
			j=cinfo->joint(i);
			j->Jm2=_MA;
		}
	}
}
void DynamicsSimulator_TRL_LCP::registerCollisionCheckPair
(
 const char *charName1,
 const char *linkName1,
 const char *charName2,
 const char *linkName2,
 vectorn const& param
 )
{
	int bodyIndex1 = world.bodyIndex(charName1);
	int bodyIndex2 = world.bodyIndex(charName2);
	Msg::verify(bodyIndex1!=-1, "TRL_LCP::registerCollisionCheckPair error1");
	Msg::verify(bodyIndex2!=-1, "TRL_LCP::registerCollisionCheckPair error2");
	double staticFriction=param[0];
	double slipFriction=param[1];

	const double epsilon = 0.0;


	if(bodyIndex1 >= 0 && bodyIndex2 >= 0){

		BodyPtr body1 = world.body(bodyIndex1);
		BodyPtr body2 = world.body(bodyIndex2);

		std::string emptyString = "";
		vector<Link*> links1;
		vector<std::string> linksName1;
		if(emptyString == linkName1){
			assert(false);
		} else {
			//links1.push_back(body1->link(linkName1));
			links1.push_back(getTRLlink(body1, ((VRMLTransform&)skeleton(bodyIndex1).getBoneByName(linkName1)).lastHRPjointIndex()));
			linksName1.push_back(linkName1);
		}

		vector<Link*> links2;
		vector<std::string> linksName2;
		if(emptyString == linkName2){
			assert(false);
		} else {
			//links2.push_back(body2->link(linkName2));
			links2.push_back(getTRLlink(body2, ((VRMLTransform&)skeleton(bodyIndex2).getBoneByName(linkName2)).lastHRPjointIndex()));
			linksName2.push_back(linkName2);
		}

		for(size_t i=0; i < links1.size(); ++i){
			for(size_t j=0; j < links2.size(); ++j){
				Link* link1 = links1[i];
				Link* link2 = links2[j];

				if(link1 && link2 && link1 != link2){
					bool ok = _contactForceSolver->addCollisionCheckLinkPair
						(bodyIndex1, link1, bodyIndex2, link2, staticFriction, slipFriction, epsilon);

					if(ok){
						LinkPair linkPair ;
						linkPair.charName1  = charName1;
						linkPair.linkName1 = linksName1[i].c_str();
						//link1->name.c_str();
						linkPair.charName2  =charName2;
						linkPair.linkName2 = linksName2[j].c_str();
						linkPair.param=param;
						//link2->name.c_str();

						printf("%s:%s-%s:%s added\n", linkPair.charName1.c_str(), linkPair.linkName1.c_str(), linkPair.charName2.c_str(), linkPair.linkName2.c_str());
						collisionDetector->addCollisionPair(linkPair, false, false);

					}
				}
			}
		}
	}
}

void DynamicsSimulator_TRL_LCP::stepSimulation_part1()
{
	BEGIN_TIMER(contact);
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	END_TIMER2(contact);
	BEGIN_TIMER(fd_abm);
	TRL::Body* cinfo=world.body(0);
	TRL::ForwardDynamicsABM* fd=world.forwardDynamics(0);
	// position, velocity fk is in initSimulation()
	fd->addGravity(); 
	//fd->calcABMPhase1(); -> initSimulation()+addGravity()
	// update inertia and velocity related terms
	fd->calcABMPhase2Part1();
	END_TIMER2(fd_abm);
	BEGIN_TIMER(conSol);
	// set external forces
	_contactForceSolver->solve(*collisions);
	END_TIMER2(conSol);
	{ 
		// calcNextState
		// collect forces, calculate accelerations, and then integrate them
		fd->calcMotionWithEulerMethod();
		TRL::Link *link=cinfo->joint(-1);
		link->v = link->vo + cross(link->w, link->p);
		//fd->calcABMPhase1();   // redundant
		//fd->calcABMPhase2Part1();// redundant
	}
	//_updateCharacterPose();// redundant with initSimulation

	for(int i=0; i<world.numBodies(); i++)
		world.body(i)->clearExternalForces();
}
bool DynamicsSimulator_TRL_LCP::stepSimulation()
{

	/*
	contact Time= 6 microseconds
	fd_abm Time= 8 microseconds
	conSol Time= 7 microseconds
	CalcNextState Time= 9 microseconds
	UPD Time= 3 microseconds
	*/
	BEGIN_TIMER(contact);
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	END_TIMER2(contact);
	BEGIN_TIMER(fd_abm);
	TRL::Body* cinfo=world.body(0);
	TRL::ForwardDynamicsABM* fd=world.forwardDynamics(0);
	// position, velocity fk
	fd->calcABMPhase1();
	// update inertia and velocity related terms
	fd->calcABMPhase2Part1();
	END_TIMER2(fd_abm);
	BEGIN_TIMER(conSol);
	// set external forces
	_contactForceSolver->solve(*collisions);
	END_TIMER2(conSol);
	// collect forces, calculate accelerations, and then integrate them
	BEGIN_TIMER(CalcNextState);
	world.calcNextState();	 // ABMPhase2-2, 3, integrate, ABMPhase1, 2-1
	END_TIMER2(CalcNextState);

	BEGIN_TIMER(UPD);
	_updateCharacterPose();

	for(int i=0; i<world.numBodies(); i++)
		world.body(i)->clearExternalForces();
	END_TIMER2(UPD);

	return true;
}
void DynamicsSimulator_TRL_LCP::stepKinematic(int ichar, vectorn const& dq)
{

	TRL::ForwardDynamicsABM* fd=world.forwardDynamics(ichar);
	setDQ(ichar, dq);
	fd->_updatePositionEuler();
	//fd->calcPositionAndVelocityFK();
	_updateCharacterPose();
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	world.body(ichar)->clearExternalForces();
}



void DynamicsSimulator_TRL_LCP::get_contact_pos(int ichar, vector3N & M, CollisionSequence& collisionSequence)
{
	VRMLloader& skel=skeleton(ichar);
	int noc =0;

	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		CollisionPointSequence& points = collisionSequence[i].points;
		if(linkPair.charIndex[0]==ichar )
			noc+=points.size();
		if(linkPair.charIndex[1]==ichar)
			noc+=points.size();
	}
	M.setSize(noc);
	int cc=0;
	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		CollisionPointSequence& points = collisionSequence[i].points;

		int n_point = points.size();
		if(n_point == 0) continue;
		if(linkPair.charIndex[0]!=ichar && linkPair.charIndex[1]!=ichar) continue;

		for(int j=0; j<n_point; j++)
		{
			const vector3& pos=points[j].position;  // global
			
			if(linkPair.charIndex[0]==ichar)
			{
				M[cc]=pos;
				cc++;
			}
			if(linkPair.charIndex[1]==ichar)
			{
				M[cc]=pos;
				cc++;
			}
		}
	}
	assert(cc==noc);
}
void DynamicsSimulator_TRL_LCP::get_contact_jacob(int ichar, matrixn & M, vectorn& v_star, CollisionSequence& collisionSequence)
{
	VRMLloader& skel=skeleton(ichar);
	int dof =skel.dofInfo.numActualDOF();
	int noc =0;

	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		CollisionPointSequence& points = collisionSequence[i].points;
		if(linkPair.charIndex[0]==ichar )
			noc+=points.size();
		if(linkPair.charIndex[1]==ichar)
			noc+=points.size();
	}

	M.setSize(3*noc, dof);
	matrixn J(6, dof);
	v_star.setSize(3*noc);

	int cc=0;
	double epsilon=_contactForceSolver->epsilon;
	double kappa=_contactForceSolver->kappa;
	double kp =   (1+epsilon) / (kappa*kappa);
	double kd = 2*(1+epsilon) / kappa;
	double h=getTimestep();
	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		CollisionPointSequence& points = collisionSequence[i].points;

		int n_point = points.size();
		if(n_point == 0) continue;
		if(linkPair.charIndex[0]!=ichar && linkPair.charIndex[1]!=ichar) continue;

		double k=linkPair.param[2];
		double d=linkPair.param[3];
		for(int j=0; j<n_point; j++)
		{
			const vector3& normal=points[j].normal;	 // global
			const vector3& pos=points[j].position;  // global
			double depth=points[j].idepth;

			transf const& frame1=getWorldState(linkPair.charIndex[0])._global(*linkPair.link[0]);
			transf const& frame2=getWorldState(linkPair.charIndex[1])._global(*linkPair.link[1]);

			vector3 lpos1, lpos2;
			lpos1=frame1.toLocalPos(pos);
			lpos2=frame2.toLocalPos(pos);
			vector3 vel1, vel2;
			getWorldVelocity(linkPair.charIndex[0],linkPair.link[0], lpos1, vel1);
			getWorldVelocity(linkPair.charIndex[1],linkPair.link[1], lpos2, vel2);

			vector3 relvel;
			relvel.sub(vel1, vel2);

			double vn=-normal%relvel;	// normal rel vel


			vector3 v(normal*(vn + h*(kp * (-0 + depth) - kd * vn)));

			if(linkPair.charIndex[0]==ichar)
			{
				//printf("vn2: %s %f %f %f\n", normal.output().ptr(), depth, vn, -v.z);
				calcJacobianAt(ichar, linkPair.link[0]->treeIndex(), J, lpos1);
				M.range(3*cc, 3*cc+3, 0, dof)=J.range(0, 3, 0, dof);
				v_star.setVec3(3*cc, v*-1);
				cc++;
			}
			if(linkPair.charIndex[1]==ichar)
			{
				//printf("vn2: %s %f %f %f\n", normal.output().ptr(), depth, vn, v.z);
				//printf("case 2 %s\n", v.output().ptr());
				calcJacobianAt(ichar, linkPair.link[1]->treeIndex(), J, lpos2);
				M.range(3*cc, 3*cc+3, 0, dof)=J.range(0, 3, 0, dof);
				v_star.setVec3(3*cc, v);
				cc++;
			}
		}
	}
	assert(cc==noc);
}

void DynamicsSimulator_TRL_LCP::_registerCharacter(const char *name, CharacterInfo const& cinfo)
{
	DynamicsSimulator_TRL_penalty::_registerCharacter(name, cinfo);
	if (cinfo.loader->constraints.size()>0)
	{
		auto& array_con=cinfo.loader->constraints;
		for(int i=0; i<array_con.size(); i++)
		{
			auto& con=array_con[i];
			addRelativeConstraint(
					_characters.size()-1, 
					cinfo.loader->bone(con.ibone1),
					con.localpos1,
					cinfo.loader->bone(con.ibone2),
					con.localpos2);
		}
	}

}
void DynamicsSimulator_TRL_LCP::addRelativeConstraint(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2)
{
	TRL::BodyPtr targetBody = world.body(ichara);
	TRL::Body::LinkConnection linkCon;
	linkCon.link[0]=getTRLlink(targetBody, ((VRMLTransform&)bone1).lastHRPjointIndex());
	linkCon.link[1]=getTRLlink(targetBody, ((VRMLTransform&)bone2).lastHRPjointIndex());
	linkCon.point[0]=boneVector1;
	linkCon.point[1]=boneVector2;
	linkCon.numConstraintAxes=3;
	linkCon.constraintAxes[0]=vector3(1,0,0);
	linkCon.constraintAxes[1]=vector3(0,1,0);
	linkCon.constraintAxes[2]=vector3(0,0,1);
    targetBody->linkConnections.push_back(linkCon);
	return;
}

void DynamicsSimulator_TRL_LCP::removeRelativeConstraint(int ichara, Bone& bone1, Bone& bone2)
{

	TRL::BodyPtr targetBody = world.body(ichara);
	TRL::Link* targetLink1 = getTRLlink(targetBody,((VRMLTransform&)bone1).lastHRPjointIndex());
	TRL::Link* targetLink2 = getTRLlink(targetBody,((VRMLTransform&)bone2).lastHRPjointIndex());
	TRL::Body::LinkConnectionArray& targetBodyLinks =  targetBody->linkConnections;

	std::vector<TRL::Body::LinkConnection>::iterator it;

	int i = 0;

	for (it = targetBodyLinks.begin(); it != targetBodyLinks.end();){
		if(it->link[0]->index == targetLink1->index || it->link[0]->index == targetLink2->index){
			if(it->link[1]->index == targetLink1->index || it->link[1]->index == targetLink2->index){
				it = targetBodyLinks.erase(it);
			}
			else{
				it++;
			}
		}
		else{
			it++;
		}
	}
	return;
}
