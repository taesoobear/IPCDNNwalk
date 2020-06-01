// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * The University of Tokyo
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/** @file DynamicsSimulator/server/World.cpp
 *
 */
#include "stdafx.h"
#include <string>

#include "World_UT.h"
#include "psim.h"
#include "Sensor.h"

using namespace OpenHRP;

static const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

static const bool debugMode = false;

#include <fstream>
//static std::ofstream logfile("world.log");
static std::ofstream logfile;

#include <sdcontact.h>

#if 0
bool World_UT::LinkPairKey::operator<(const LinkPairKey& pair2) const
{
	if((body1 == pair2.body1 && body2 == pair2.body2) ||
	   (body2 == pair2.body1 && body1 == pair2.body2 )){
		if(link1 < link2){
			if(pair2.link1 < pair2.link2){
				return (link1 < pair2.link1) ? true : (link2 < pair2.link2);
			} else {
				return (link1 < pair2.link2) ? true : (link2 < pair2.link1);
			}
		} else {
			if(pair2.link1 < pair2.link2){
				return (link2 < pair2.link1) ? true : (link1 < pair2.link2);
			} else {
				return (link2 < pair2.link2) ? true : (link1 < pair2.link1);
			}
		}
	} else {
		if(body1 < body2){
			if(pair2.body1 < pair2.body2){
				return (body1 < pair2.body1) ? true : (body2 < pair2.body2);
			} else {
				return (body1 < pair2.body2) ? true : (body2 < pair2.body1);
			}
		} else {
			if(pair2.body1 < pair2.body2){
				return (body2 < pair2.body1) ? true : (body1 < pair2.body2);
			} else {
				return (body2 < pair2.body2) ? true : (body1 < pair2.body1);
			}
		}
	}
}
#endif

World_UT_base::World_UT_base(): g(0.0, 0.0, DEFAULT_GRAVITY_ACCELERATION)
{
	currentTime_ = 0.0;
    timeStep_ = 0.005;

    isEulerMethod = false;
	chain = new pSim;

}

World_UT::World_UT(): World_UT_base()
{
    
#ifdef USE_SENSOR
	sensorsAreEnabled = false;
#endif
	numRegisteredLinkPairs = 0;

}

World_UT_base::~World_UT_base()
{
	if(chain) delete chain;
}

World_UT::~World_UT()
{
	
#ifdef USE_SENSOR

	int n_sensors = sensors.size();
	for(int i=0; i<n_sensors; i++)
	{
		OpenHRP::Sensor::destroy(sensors[i]);
	}
	sensors.clear();
#endif
	int n_pairs = contact_pairs.size();
	for(int i=0; i<n_pairs; i++)
	{
		delete contact_pairs[i];
	}
	contact_pairs.clear();
}


void World_UT_base::setTimeStep(double ts)
{
    timeStep_ = ts;
}


void World_UT_base::setCurrentTime(double time)
{
    currentTime_ = time;
}


void World_UT_base::setGravityAcceleration(const fVec3& _g)
{
    g.set(_g);
	if(chain->Root())
		chain->Root()->loc_lin_acc.set(_g);
}

#ifdef USE_SENSOR

void World_UT::enableSensors(bool on)
{
	sensorsAreEnabled = on;
}

#endif
void World_UT::initialize()
{
	chain->ClearExtForce();
	chain->in_create_chain = true;
	chain->clear_contact();
	chain->init_contact();
	chain->in_create_chain=false;
	// chain->BeginCreateChain();
	// chain->clear_contact();
	// chain->EndCreateChain();
	int n_pair = contact_pairs.size();
	// printf("n_pair: %d\n", n_pair);
	for(int i=0; i<n_pair; i++)
		contact_pairs[i]->Clear();
	World_UT_base::initialize();
}

void World_UT_base::initialize()
{

//	logfile << "initialize" << endl;
	chain->Schedule();
	chain->CalcPosition();
	chain->CalcVelocity();
//	logfile << "initialize end" << endl;
#if 0
	if(isEulerMethod){
		pForwardDynamics->setEulerMethod();
	} else {
		pForwardDynamics->setRungeKuttaMethod();
	}
	pForwardDynamics->setGravityAcceleration(g);
	pForwardDynamics->setTimeStep(timeStep_);
	pForwardDynamics->enableSensors(sensorsAreEnabled);
	pForwardDynamics->initialize();
#endif
}


void World_UT::calcNextState(CollisionDetector* collisionDetector, CollisionSequence& corbaCollisionSequence)
{
	if(debugMode){
		cout << "World current time = " << currentTime_ << endl;
	}
	
	// printf("cn \n");
	int n_pair = contact_pairs.size();
	for(int i=0; i<n_pair; i++)
	{
		LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		contact_pairs[i]->Clear();
		Collision& col = corbaCollisionSequence[i];
		int n_point = col.points.size();
		if(n_point == 0) continue;
		Joint* joint0 = contact_pairs[i]->GetJoint(0);
		static fVec3 pos0, norm0, pos, norm;
		for(int j=0; j<n_point; j++)
		{
			// printf("%d pints\n", n_point);
			CollisionPoint& point = col.points[j];
			const vector3& normal=point.normal;	
			const vector3& posi=point.position;
			double depth=point.idepth;

			// transf const& frame1=getWorldState(linkPair.charIndex[0])._global(*linkPair.link[0]);
			// transf const& frame2=getWorldState(linkPair.charIndex[1])._global(*linkPair.link[1]);

			// inertia frame -> joint0 frame : already done outside this function
			pos0(0) = posi[0];
			pos0(1) = posi[1];
			pos0(2) = posi[2];
			norm0(0) = normal[0];
			norm0(1) = normal[1];
			norm0(2) = normal[2];
#define COMMENT_OUT
#ifdef COMMENT_OUT
			pos0 -= joint0->abs_pos;
			pos.mul(pos0, joint0->abs_att);
			norm.mul(norm0, joint0->abs_att);
			contact_pairs[i]->AddPoint(pos.data(), norm.data(), point.idepth);
#else
			contact_pairs[i]->AddPoint(pos0.data(), norm0.data(), point.idepth);
#endif
		}
	}
	
	//
	if(isEulerMethod)
	{
		chain->Update(timeStep_, contact_pairs);
		chain->Integrate(timeStep_);
		chain->CalcPosition();
		chain->CalcVelocity();
	}
	else
	{
		chain->Update(timeStep_, contact_pairs);
		chain->IntegrateRK4(timeStep_, 0);
		chain->CalcPosition();
		chain->CalcVelocity();

		chain->Update();
		chain->IntegrateRK4(timeStep_, 1);
		chain->CalcPosition();
		chain->CalcVelocity();

		chain->Update();
		chain->IntegrateRK4(timeStep_, 2);
		chain->CalcPosition();
		chain->CalcVelocity();

		chain->Update();
		chain->IntegrateRK4(timeStep_, 3);
		chain->CalcPosition();
		chain->CalcVelocity();
	}
#ifdef USE_SENSOR
	
	// update sensors
	logfile << "update sensors ->" << endl;
	int n;
	n = numSensors();
	for(int i=0; i<n; i++)
	{
		if(sensors[i]->type == Sensor::FORCE)
		{
			ForceSensor* fs = (ForceSensor*)sensors[i];
			update_force_sensor(fs);
		}
		else if(sensors[i]->type == Sensor::RATE_GYRO)
		{
			RateGyroSensor* rgs = (RateGyroSensor*)sensors[i];
			update_rate_gyro_sensor(rgs);
		}
		else if(sensors[i]->type == Sensor::ACCELERATION)
		{
			AccelSensor* as = (AccelSensor*)sensors[i];
			update_accel_sensor(as);
		}
		else
		{
		}
	}
	logfile << "<- update sensors" << endl;
#endif
    currentTime_ += timeStep_;
}

#ifdef USE_SENSOR

void World_UT::update_force_sensor(ForceSensor* fs)
{
	logfile << "force sensor: " << fs->name << endl;
	logfile << "joint = " << fs->joint->name << endl;
	logfile << "f = " << fs->joint->joint_f << endl;
	logfile << "n = " << fs->joint->joint_n << endl;
	logfile << "localR =  " << fs->localR << endl;
	logfile << "localPos = " << fs->localPos << endl;
	fVec3 local_f, local_n, n;
	local_f.mul(fs->joint->joint_f, fs->localR);
	n.cross(fs->joint->joint_f, fs->localPos);
	n += fs->joint->joint_n;
	local_n.mul(n, fs->localR);
	fs->f.neg(local_f);
	fs->tau.neg(local_n);
}

void World_UT::update_rate_gyro_sensor(RateGyroSensor* rgs)
{
	rgs->w.mul(rgs->joint->loc_ang_vel, rgs->localR);
}

void World_UT::update_accel_sensor(AccelSensor* as)
{
	chain->CalcAcceleration();
	fVec3 acc_j, tmp1;
	tmp1.cross(as->joint->loc_ang_vel, as->localPos);
	acc_j.cross(as->joint->loc_ang_vel, tmp1);
	tmp1.cross(as->joint->loc_ang_acc, as->localPos);
	acc_j += tmp1;
	acc_j += as->joint->loc_lin_acc;
	as->dv.mul(acc_j, as->localR);
}
#endif

void World_UT_base::setEulerMethod()
{
    isEulerMethod = true;
}


void World_UT_base::setRungeKuttaMethod()
{
    isEulerMethod = false;
}


#if 0
std::pair<int,bool> World_UT::getIndexOfLinkPairs(BodyPtr body1, Link* link1, BodyPtr body2, Link* link2)
{
	int index = -1;
	int isRegistered = false;

    if(link1 != link2){

		LinkPairKey linkPair;
		linkPair.body1 = body1;
		linkPair.link1 = link1;
		linkPair.body2 = body2;
		linkPair.link2 = link2;

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
#endif
#ifdef USE_SENSOR
int World_UT::addSensor(Joint* _joint, int sensorType, int _id, const TString _name, const fVec3& _localPos, const fMat33& _localR)
{
	Sensor* sensor = Sensor::create(sensorType);
	sensor->joint = _joint;
	sensor->id = _id;
	sensor->name = _name;
	sensor->localPos.set(_localPos);
	sensor->localR.set(_localR);
	sensors.push_back(sensor);
	return 0;
}


Sensor* World_UT::findSensor(const char* sensorName, const char* charName)
{
	int n_sensors = sensors.size();
	for(int i=0; i<n_sensors; i++)
	{
		if(!strcmp(sensors[i]->name.ptr(), sensorName) && sensors[i]->joint &&
		   !strcmp(sensors[i]->joint->CharName(), charName))
		{
			return sensors[i];
		}
	}
	return 0;
}

int World_UT::numSensors(int sensorType, const char* charName)
{
	int count = 0;
	int n_sensors = sensors.size();
	for(int i=0; i<n_sensors; i++)
	{
		if(sensors[i]->type == sensorType &&
		   (!charName || !strcmp(sensors[i]->joint->CharName(), charName)))
		{
			count++;
		}
	}
	return count;
}
#endif

void World_UT_base::getAllCharacterData(const char* name, OpenHRP::DynamicsSimulator::LinkDataType type, vectorn& rdata)
{
	int index = -1, nchar = characters.size();
	for(int i=0; i<nchar; i++)
	{
		if(!strcmp(name, characters[i].name.ptr()))
		{
			index = i;
			break;
		}
	}
	if(index < 0) return;
	CharacterInfo& cinfo = characters[index];
	int n_joints = cinfo.n_joints, n_links = cinfo.links.size();
	rdata.resize(n_joints);
	for(int i=0; i<n_links; i++)
	{
		if(cinfo.jointIDs[i] >= 0)
		{
			_get_all_character_data_sub(cinfo.links[i], cinfo.jointIDs[i], type, rdata);
		}
	}
}

void World_UT_base::_get_all_character_data_sub(Joint* cur, int index, OpenHRP::DynamicsSimulator::LinkDataType type, vectorn& rdata)
{
	if(cur->j_type == ::JROTATE || cur->j_type == ::JSLIDE)
	{
		switch(type) {
		case OpenHRP::DynamicsSimulator::JOINT_VALUE:
			(rdata)[index] = cur->q;
			break;

		case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
			(rdata)[index] = cur->qd;
			break;

		case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
			(rdata)[index] = cur->qdd;
			break;

		case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
			(rdata)[index] = cur->tau;
			break;

		default:
//			cerr << "ERROR - Invalid type: " << getLabelOfGetLinkDataType(type) << endl;
			break;
		}
	}
}

void World_UT_base::setAllCharacterData(const char* name, OpenHRP::DynamicsSimulator::LinkDataType type, const vectorn& wdata)
{
	int index = -1, nchar = characters.size();
	for(int i=0; i<nchar; i++)
	{
		if(!strcmp(name, characters[i].name.ptr()))
		{
			index = i;
			break;
		}
	}
	if(index < 0) return;
	CharacterInfo& cinfo = characters[index];
	int n_joints = cinfo.n_joints, n_links = cinfo.links.size();
	for(int i=0; i<n_links; i++)
	{
		if(cinfo.jointIDs[i] >= 0)
		{
			_set_all_character_data_sub(cinfo.links[i], cinfo.jointIDs[i], type, wdata);
		}
	}
}

void World_UT_base::_set_all_character_data_sub(Joint* cur, int index, OpenHRP::DynamicsSimulator::LinkDataType type, const vectorn& wdata)
{
	if(index < wdata.length() &&
	   (cur->j_type == ::JROTATE || cur->j_type == ::JSLIDE))
	{
//		logfile << "set[" << index << "]: " << cur->name << ": " << wdata[index] << endl;
		switch(type) {
		case OpenHRP::DynamicsSimulator::JOINT_VALUE:
			cur->SetJointValue(wdata[index]);
			break;

		case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
			cur->SetJointVel(wdata[index]);
			break;

		case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
			cur->SetJointAcc(wdata[index]);
			break;

		case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
			cur->SetJointForce(wdata[index]);
			break;

		default:
//			cerr << "ERROR - Invalid type: " << getLabelOfGetLinkDataType(type) << endl;
			break;
		}
	}
}

static void vec3_to_array(const fVec3& vec, double* a, int offset = 0)
{
	a[offset++] = vec(0);
	a[offset++] = vec(1);
	a[offset] = vec(2);
}

static void vec3_to_array(const fVec3& vec, vectorn& a, int offset = 0)
{
	a[offset++] = vec(0);
	a[offset++] = vec(1);
	a[offset] = vec(2);
}

static void mat33_to_array(const fMat33& mat, matrix3& a)
{
	a[0] = mat(0,0);  a[1] = mat(0,1);  a[2] = mat(0,2);
	a[3] = mat(1,0);  a[4] = mat(1,1);  a[5] = mat(1,2);
	a[6] = mat(2,0);  a[7] = mat(2,1);  a[8] = mat(2,2);
}



#ifdef USE_SENSOR
void World_UT::getAllSensorStates(SensorStateSequence& all_sensor_states)
{
	ASSERT(0);
	/*
	int nchar = characters.size();
	for(int i=0; i<nchar; i++)
	{
		SensorState& state = all_sensor_states[i];
		// joint angles and torques
		CharacterInfo& cinfo = characters[i];
		int n_links = cinfo.links.size();
		for(int j=0; j<n_links; j++)
		{
			int index = cinfo.jointIDs[j];
			if(index >= 0)
			{
				Joint* joint = cinfo.links[j];
				if(joint->j_type == ::JROTATE || joint->j_type == ::JSLIDE){
					state.q[index] = joint->q;
					state.dq[index] = joint->qd;
					state.u[index] = joint->tau;
				}
			}
		}	
		// other sensors
		Joint* r = rootJoint(i);
		int n_sensors = sensors.size();
		for(int i=0; i<n_sensors; i++)
		{
			if(strcmp(sensors[i]->joint->CharName(), r->CharName())) continue;
			switch(sensors[i]->type)
			{
			case Sensor::FORCE:
			{
				ForceSensor* _fs = (ForceSensor*)sensors[i];
				vec3_to_array(_fs->f, state.force[sensors[i]->id], 0);
				vec3_to_array(_fs->tau, state.force[sensors[i]->id], 3);
				break;
			}

			case Sensor::RATE_GYRO:
			{
				RateGyroSensor* _gs = (RateGyroSensor*)sensors[i];
				vec3_to_array(_gs->w, state.rateGyro[sensors[i]->id]);
				break;
			}

			case Sensor::ACCELERATION:
			{
				AccelSensor* _as = (AccelSensor*)sensors[i];
				vec3_to_array(_as->dv, state.accel[sensors[i]->id]);
				break;
			}

			default:
				break;
			}
		}
	}*/
}
#endif
#if 0
void World_UT::_get_all_sensor_states_sub(Joint* cur, int& count, SensorState& state)
{
	if(!cur || cur->real) return;
	if(cur->j_type == ::JROTATE || cur->j_type == ::JSLIDE)
	{
		state.q[count] = cur->q;
		state.u[count] = cur->tau;
		count++;
	}
	_get_all_sensor_states_sub(cur->child, count, state);
	_get_all_sensor_states_sub(cur->brother, count, state);
}
#endif
void World_UT_base::calcCharacterJacobian(
		const char* characterName,
		const char* baseLink,
		const char* targetLink,
		fMat& J)
{
	Joint* basej = chain->FindJoint(baseLink, characterName);
	if(!basej) return;
	Joint* targetj = chain->FindJoint(targetLink, characterName);
	if(!targetj) return;

	fMat tempJ(6, chain->NumDOF());
	tempJ.zero();
	targetj->CalcJacobian(tempJ);

	int n_dof = 0;
	for(Joint* j=targetj; j!=basej; j=j->parent)
	{
		n_dof += j->n_dof;
	}
	
	J.resize(6, n_dof);
	J.zero();
	int count = 0;
	for(Joint* j=targetj; j!=basej; j=j->parent)
	{
		for(int m=0; m<j->n_dof; m++)
		{
			J(0, count+m) = tempJ(0, j->i_dof+m);
			J(1, count+m) = tempJ(1, j->i_dof+m);
			J(2, count+m) = tempJ(2, j->i_dof+m);
			J(3, count+m) = tempJ(3, j->i_dof+m);
			J(4, count+m) = tempJ(4, j->i_dof+m);
			J(5, count+m) = tempJ(5, j->i_dof+m);
		}
		count += j->n_dof;
	}
}


void World_UT::addCollisionCheckLinkPair(Joint* jnt1, Joint* jnt2, double staticFriction, double slipFriction, double epsilon)
{
	double default_spring = 1e5;
	double default_damper = 1e4;
	double default_slip_p = 2000.0;
	double default_slip_d = 700.0;
	double default_slip_func_coef_base = 0.1;
	SDContactPair* sd_pair = new SDContactPair(jnt1, jnt2, default_spring, default_damper, staticFriction, slipFriction, default_slip_p, default_slip_d, default_slip_func_coef_base);
	contact_pairs.push_back(sd_pair);
}

void World_UT_base::addCharacter(Joint* rjoint, const TString& name, LinkInfoSequence const& links)
{
	CharacterInfo cinfo(rjoint, name);
	int n_links = links.size();
	for(int i=0; i<n_links; i++)
	{
		const LinkInfo* linfo = &(links[i]);
		Joint* jnt = chain->FindJoint(linfo->name, name.ptr());
		if(jnt)
		{
			cinfo.links.push_back(jnt);
			cinfo.jointIDs.push_back(linfo->jointId);
			if(linfo->jointId >= 0) cinfo.n_joints++;
		}
	}
//	logfile << "character: " << name << ", root = " << rjoint->name << ", n_joints = " << cinfo.n_joints << endl;
	characters.push_back(cinfo);
}

Joint* World_UT_base::rootJoint(int index)
{
	return characters[index].root;
}
