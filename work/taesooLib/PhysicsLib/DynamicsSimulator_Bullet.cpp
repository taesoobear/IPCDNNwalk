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
/** @file DynamicsSimulator_UT/server/DynamicsSimulator_UT.cpp
 *
 */
#include "stdafx.h"
#include <vector>
#include <map>

#include "DynamicsSimulator_Bullet.h"
#include "ModelLoaderUtil_UT.h"
#include <psim.h>

using namespace OpenHRP;
using namespace std;

#define MACRO_LAPACK_TO_STRING(b) #b


#define MACRO_LAPACK_LIB(a) MACRO_LAPACK_TO_STRING(C:\\Program Files\\CLAPACK-3.1.1\\LIB\\Win32\\##a.lib)
//
//#ifdef _DEBUG
//#pragma comment(lib, MACRO_LAPACK_LIB(clapackd))
//#pragma comment(lib, MACRO_LAPACK_LIB(blasd))
//#pragma comment(lib, MACRO_LAPACK_LIB(libf2cd))
//#else
//#pragma comment(lib, MACRO_LAPACK_LIB(clapack))
//#pragma comment(lib, MACRO_LAPACK_LIB(blas))
//#pragma comment(lib, MACRO_LAPACK_LIB(libf2c))
//
//#endif


//#define INTEGRATOR_DEBUG
static const bool enableTimeMeasure = false;

//#include <fstream>
//static std::ofstream logfile("impl.log");
//static std::ofstream logfile;

DynamicsSimulator_UT::DynamicsSimulator_UT(const char* coldet)
:DynamicsSimulator(coldet)	//: orb_(CORBA::ORB::_duplicate(orb))
{
	collisions_frame0=new CollisionSequence;
	world=new World_UT();
	world->setCurrentTime(0.0);
	world->setRungeKuttaMethod();
}


DynamicsSimulator_UT::~DynamicsSimulator_UT()
{
	delete world;
	//delete collisionDetector;
	//delete collisions;
	delete collisions_frame0;
}


void DynamicsSimulator_UT::_registerCharacter(
		const char *name,
		CharacterInfo const& cinfo)
{
	registerCharacter(name, cinfo);
}
void DynamicsSimulator_UT::registerCharacter(
		const char *name,
		CharacterInfo const& chara)
{
//	logfile << "registerCharacter(" << name << ", " << chara->name() << ")" << endl;
	loadBodyFromCharacterInfo(world, name, &chara);	
	collisionDetector->addModel(name, chara);
	_characters.push_back(new DynamicsSimulator::Character(chara.loader));
//	logfile << "total dof = " << world->Chain()->NumDOF() << endl;
}

void DynamicsSimulator_UT::init(
		double _timestep,
		OpenHRP::DynamicsSimulator_UT::IntegrateMethod integrateOpt)
{
//	logfile << "init" << endl;

	world->setTimeStep(_timestep);
    world->setCurrentTime(0.0);

	if(integrateOpt == OpenHRP::DynamicsSimulator_UT::EULER){
		world->setEulerMethod();
	} else {
		world->setRungeKuttaMethod();
	}

	isFirstSimulationLoop = true;

}


static void joint_traverse_sub(Joint* cur, std::vector<Joint*>& jlist)
{
	if(!cur) return;
//	logfile << "joint_traverse_sub: " << cur->name << ", n_dof = " << cur->n_dof << endl;
	jlist.push_back(cur);
	joint_traverse_sub(cur->brother, jlist);
	joint_traverse_sub(cur->child, jlist);
}

static void joint_traverse(Joint* r, std::vector<Joint*>& jlist)
{
	jlist.push_back(r);
	joint_traverse_sub(r->child, jlist);
}


void DynamicsSimulator_UT::registerCollisionCheckPair(
		const char *charName1,
		const char *linkName1,
		const char *charName2,
		const char *linkName2,
		vectorn const& param
 )
{
	double staticFriction=param[0];
	double slipFriction=param[1];
	const double epsilon = 0.0;
//	logfile << "registerCollisionCheckPair" << endl;

	TString emptyString = "";
	std::vector<Joint*> joints1;
	std::vector<Joint*> joints2;
	pSim* chain = world->Chain();

	if(emptyString == linkName1)
	{
		Joint* r = chain->FindCharacterRoot(charName1);
		if(r) joint_traverse(r, joints1);
	}
	else
	{
		Joint* jnt1 = chain->FindJoint(linkName1, charName1);
		if(jnt1) joints1.push_back(jnt1);
	}
	if(emptyString == linkName2)
	{
		Joint* r = chain->FindCharacterRoot(charName2);
		if(r) joint_traverse(r, joints2);
	}
	else
	{
		Joint* jnt2 = chain->FindJoint(linkName2, charName2);
		if(jnt2) joints2.push_back(jnt2);
	}

	for(size_t i=0; i < joints1.size(); ++i)
	{
		Joint* j1 = joints1[i];
		for(size_t j=0; j < joints2.size(); ++j)
		{
			Joint* j2 = joints2[j];
			if(j1 && j2 && j1 != j2)
			{
// #define REVERSED
#ifdef REVERSED
//				logfile << "pair = " << j1->name << ", " << j2->name << endl;
				world->addCollisionCheckLinkPair(j2, j1, staticFriction, slipFriction, epsilon);
				LinkPair linkPair ;
				linkPair.charName1  = charName2;
				linkPair.linkName1 = j2->basename;
				linkPair.charName2  = charName1;
				linkPair.linkName2 = j1->basename;
				collisionDetector->addCollisionPair(linkPair, false, false);
#else
//				logfile << "pair = " << j1->name << ", " << j2->name << endl;
				world->addCollisionCheckLinkPair(j1, j2, staticFriction, slipFriction, epsilon);
				LinkPair linkPair ;
				linkPair.charName1  = charName1;
				linkPair.linkName1 = j1->basename;
				linkPair.charName2  = charName2;
				linkPair.linkName2 = j2->basename;
				collisionDetector->addCollisionPair(linkPair, false, false);
#endif
			}
		}
	}
}



static void vec3_to_seq(const fVec3& vec, vectorn& seq, size_t offset = 0)
{
	seq[(offset++)] = vec(0);
	seq[(offset++)] = vec(1);
	seq[(offset)] = vec(2);
}

static void seq_to_vec3(const vector3& seq, fVec3& vec)
{
	vec(0) = seq[0];
	vec(1) = seq[1];
	vec(2) = seq[2];
}

void DynamicsSimulator_UT::initSimulation()
{
	world->initialize();
	_updateCharacterPose();
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	// world->initialize();
	// _updateCharacterPose();
	// collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
}

bool DynamicsSimulator_UT::stepSimulation()
{
	/*
	// convert contact from inertial frame to frame0
	collisions_frame0->resize(collisions->size());
	assert(collisions->size()==collisionDetector->getCollisionPairs().size());
	for(int i=0; i<collisions->size(); i++) {
		LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		Collision& cf0=(*collisions_frame0)[i];
		Collision& c=(*collisions)[i];
		int n_point=c.points.size();
		cf0.points.resize(n_point);
		for (int j=0; j<n_point; j++){
			CollisionPoint& point0=cf0.points[j];
			CollisionPoint& point=c.points[j];
			
			const vector3& normal=point.normal;	
			const vector3& posi=point.position;
			double depth=point.idepth;

			transf const& frame1=getWorldState(linkPair.charIndex[0])._global((const Bone&)*linkPair.link[0]);
			transf const& frame2=getWorldState(linkPair.charIndex[1])._global((const Bone&)*linkPair.link[1]);
			vector3 lpos1=frame1.toLocalPos(posi);
			vector3 lnormal1=frame1.toLocalDir(normal);
			point0.position=lpos1;
			point0.normal=lnormal1;
			point0.idepth=depth;
		}
	}
	*/
	world->calcNextState(collisionDetector,*collisions);
	_updateCharacterPose();
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	return true;
}

/*
void DynamicsSimulator_UT::setCharacterLinkData(
		const char* characterName,
		const char* linkName,
		OpenHRP::DynamicsSimulator_UT::LinkDataType type,
		const vectorn& wdata)
{
//	logfile << "setCharacterLinkData(" << characterName << ", " << linkName << ")" << endl;
	Joint* joint = world->Chain()->FindJoint(linkName, characterName);
	if(!joint) return;

	switch(type) {

	case OpenHRP::DynamicsSimulator_UT::POSITION_GIVEN:
		joint->t_given = !(wdata[0] > 0.0);
		break;

	case OpenHRP::DynamicsSimulator_UT::JOINT_VALUE:
		joint->SetJointValue(wdata[0]);
		break;

	case OpenHRP::DynamicsSimulator_UT::JOINT_VELOCITY:
		joint->SetJointVel(wdata[0]);
		break;

	case OpenHRP::DynamicsSimulator_UT::JOINT_ACCELERATION:
		joint->SetJointAcc(wdata[0]);
		break;

	case OpenHRP::DynamicsSimulator_UT::JOINT_TORQUE:
		joint->SetJointForce(wdata[0]);
		break;

	case OpenHRP::DynamicsSimulator_UT::ABS_TRANSFORM:
	{
		fVec3 abs_pos(wdata[0], wdata[1], wdata[2]);
		fMat33 abs_att(wdata[3], wdata[4], wdata[5], wdata[6], wdata[7], wdata[8], wdata[9], wdata[10], wdata[11]);  // wdata is in row major order
		joint->SetJointValue(abs_pos, abs_att);
		break;
	}
	
	case OpenHRP::DynamicsSimulator_UT::ABS_VELOCITY:
	{
		fVec3 abs_lin_vel(wdata[0], wdata[1], wdata[2]);
		fVec3 abs_ang_vel(wdata[3], wdata[4], wdata[5]);
		joint->rel_lin_vel.mul(abs_lin_vel, joint->abs_att);
		joint->rel_ang_vel.mul(abs_ang_vel, joint->abs_att);
		break;
	}

	case OpenHRP::DynamicsSimulator_UT::EXTERNAL_FORCE:
	{
		// original: local frame?, around world origin
		// new: local frame, around joint origin
		joint->ext_force(0) = wdata[0];
		joint->ext_force(1) = wdata[1];
		joint->ext_force(2) = wdata[2];
		fVec3 n0(wdata[3], wdata[4], wdata[5]);
		fVec3 np;
		np.cross(joint->abs_pos, joint->ext_force);
		joint->ext_moment.sub(n0, np);
		break;
	}

	default:
		return;
	}

	needToUpdatePositions = true;
#ifdef USE_SENSOR
	needToUpdateSensorStates = true;
#endif
}


void DynamicsSimulator_UT::getCharacterLinkData(
		const char * characterName,
		const char * linkName,
		OpenHRP::DynamicsSimulator_UT::LinkDataType type,
		vectorn& out_rdata)
{
//	logfile << "getCharacterLinkData" << endl;
	Joint* joint = world->Chain()->FindJoint(linkName, characterName);
	assert(joint);

	vectorn* rdata = &out_rdata;

	switch(type) {

	case OpenHRP::DynamicsSimulator_UT::JOINT_VALUE:
		rdata->setSize(1);
		rdata[0] = joint->q;
		break;

	case OpenHRP::DynamicsSimulator_UT::JOINT_VELOCITY:
		rdata->setSize(1);
		rdata[0] = joint->qd;
		break;

	case OpenHRP::DynamicsSimulator_UT::JOINT_ACCELERATION:
		rdata->setSize(1);
		rdata[0] = joint->qdd;
		break;

	case OpenHRP::DynamicsSimulator_UT::JOINT_TORQUE:
		rdata->setSize(1);
		rdata[0] = joint->tau;
		break;

	case OpenHRP::DynamicsSimulator_UT::ABS_TRANSFORM:
	{
		fEulerPara ep;
		ep.set(joint->abs_att);
		rdata->setSize(7);
		rdata[0] = joint->abs_pos(0);
		rdata[1] = joint->abs_pos(1);
		rdata[2] = joint->abs_pos(2);
		rdata[3] = ep.Ang();
		rdata[4] = ep.Axis()(0);
		rdata[5] = ep.Axis()(1);
		rdata[6] = ep.Axis()(2);
		break;
	}

	case OpenHRP::DynamicsSimulator_UT::ABS_VELOCITY:
	{
		fVec3 v0, w0;
		v0.mul(joint->abs_att, joint->loc_lin_vel);
		w0.mul(joint->abs_att, joint->loc_ang_vel);
		rdata->setSize(6);
		rdata[0] = v0(0);
		rdata[1] = v0(1);
		rdata[2] = v0(2);
		rdata[3] = w0(0);
		rdata[4] = w0(1);
		rdata[5] = w0(2);
		break;
	}
	
	case OpenHRP::DynamicsSimulator_UT::EXTERNAL_FORCE:
	{
		// original: local frame, around joint origin
		// new: local frame?, around world origin
		rdata->setSize(6);
		rdata[0] = joint->ext_force(0);
		rdata[1] = joint->ext_force(1);
		rdata[2] = joint->ext_force(2);
		fVec3 np, n0;
		np.cross(joint->abs_pos, joint->ext_force);
		n0.add(joint->ext_moment, np);
		rdata[3] = n0(0);
		rdata[4] = n0(1);
		rdata[5] = n0(2);
		break;
	}
	
	default:
		break;
	}
}


void DynamicsSimulator_UT::getCharacterAllLinkData(
		const char * characterName,
		OpenHRP::DynamicsSimulator_UT::LinkDataType type,
		vectorn& rdata)
{
//	logfile << "getCharacterAllLinkData" << endl;
	world->getAllCharacterData(characterName, type, rdata);
}


void DynamicsSimulator_UT::setCharacterAllLinkData(
		const char * characterName,
		OpenHRP::DynamicsSimulator_UT::LinkDataType type,
		const vectorn & wdata)
{
//	logfile << "setCharacterAllLinkData: " << getLabelOfLinkDataType(type) << endl;
	world->setAllCharacterData(characterName, type, wdata);
}
*/

void DynamicsSimulator_UT::setGVector(
		const vector3& wdata)
{
//	assert(wdata.size() == 3);

//	logfile << "setGVector" << endl;
    fVec3 g;
	seq_to_vec3(wdata, g);
	world->setGravityAcceleration(g);

}


void DynamicsSimulator_UT::setCharacterAllJointModes(
		const char * characterName,
		OpenHRP::DynamicsSimulator_UT::JointDriveMode jointMode)
{
//	logfile << "setCharacterAllJointModes" << endl;
	bool isTorqueMode = (jointMode != OpenHRP::DynamicsSimulator_UT::HIGH_GAIN_MODE);

	world->Chain()->SetCharacterTorqueGiven(characterName, isTorqueMode);

}




/*
void DynamicsSimulator_UT::calcCharacterForwardKinematics(
		const char * characterName)
{
//	logfile << "calcCharacterForwardKinematics" << endl;
	world->Chain()->CalcPosition();

	needToUpdatePositions = true;
#ifdef USE_SENSOR
	needToUpdateSensorStates = true;
#endif
}


void DynamicsSimulator_UT::calcWorldForwardKinematics()
{
//	logfile << "calcWorldForwardKinematics" << endl;
	world->Chain()->CalcPosition();

	needToUpdatePositions = true;
#ifdef USE_SENSOR
	needToUpdateSensorStates = true;
#endif
}
*/


/*
bool DynamicsSimulator_UT::getCharacterCollidingPairs(
		const char *ccharacterName,
		LinkPairSequence& pairs)
{
	std::vector<unsigned int> locations;

	TString characterName=ccharacterName;
	for(unsigned int i=0; i < collisions->size(); ++i) {

		if(  characterName==(*collisions)[i].pair.charName1 ||
			 characterName==(*collisions)[i].pair.charName2 )
			locations.push_back(i);
	}

	pairs.resize(locations.size());

	unsigned long n=0;
	for(std::vector<unsigned int>::iterator iter = locations.begin();
		iter != locations.end(); ++iter) {

		pairs[n].charName1= (*collisions)[*iter].pair.charName1;
		pairs[n].charName2= (*collisions)[*iter].pair.charName2;
		pairs[n].linkName1= (*collisions)[*iter].pair.linkName1;
		pairs[n].linkName2= (*collisions)[*iter].pair.linkName2;
	}

	return true;
}
*/

void DynamicsSimulator_UT::calcCharacterJacobian(
		const char *characterName,
		const char *baseLink,
		const char *targetLink,
		vectorn& jacobian)
{
	fMat J;
	world->calcCharacterJacobian(characterName, baseLink, targetLink, J);

	int height = J.row();
	int width = J.col();

	jacobian.setSize(height * width);
	int i = 0;
	for(int r=0; r < height; ++r){
		for(int c=0; c < width; ++c){
			(jacobian)[i++] = J(r, c);
		}
	}
}

#include "../MainLib/OgreFltk/VRMLloader.h"
void DynamicsSimulator_UT::calcJacobian(int ichara, int ibone, matrixn& jacobian)
{
	OpenHRP::World_UT::CharacterInfo& cinfo=getWorld()->characters[ichara];
	fMat J(6, getWorld()->Chain()->NumDOF());
	J.zero();
	VRMLloader const& l=*_characters[ichara]->skeleton;
	VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
	int sj=b.mJoint->jointStartId;
	int sDOF=l.dofInfo.startT(ibone);
	int nDOF=l.dofInfo.endR(ibone)-sDOF;
	cinfo.links[b.mJoint->jointStartId+nDOF-1]->CalcJacobian(J);
	
	int height = J.row();
	int width = J.col();

	jacobian.setSize(height , width);
	int i = 0;
	for(int r=0; r < height; ++r){
		for(int c=0; c < width; ++c){
			(jacobian)(r,c) = J(r, c);
		}
	}
}
void DynamicsSimulator_UT::calcDotJacobian(int ichara, int ibone, matrixn& jacobian)
{
	printf("hihi %d %d %d\n", ichara, ibone, jacobian.rows());
	OpenHRP::World_UT::CharacterInfo& cinfo=getWorld()->characters[ichara];
	fMat J(6, getWorld()->Chain()->NumDOF());
	J.zero();
	VRMLloader const& l=*_characters[ichara]->skeleton;
	VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
	int sj=b.mJoint->jointStartId;
	int sDOF=l.dofInfo.startT(ibone);
	int nDOF=l.dofInfo.endR(ibone)-sDOF;
	/*
	 cinfo.links[b.mJoint->jointStartId+nDOF-1]->CalcJacobian2(&J);
	
	printf("hihi\n");
	int height = J.row();
	int width = J.col();

	jacobian.setSize(height , width);
	int i = 0;
	for(int r=0; r < height; ++r){
		for(int c=0; c < width; ++c){
			(jacobian)(r,c) = J(r, c);
		}
	}
	*/
	fVec jdot(getWorld()->Chain()->NumDOF());
	 cinfo.links[b.mJoint->jointStartId+nDOF-1]->CalcJdot(jdot);
	 
	 jacobian.setSize(jdot.size(),1);
	 for(int r=0; r < jacobian.rows(); ++r){
			(jacobian)(r,0) = jdot(r);
	 }

}

void DynamicsSimulator_UT::calcCOMjacobian(int ichara, matrixn& jacobian)
{
	OpenHRP::World_UT::CharacterInfo& cinfo=getWorld()->characters[ichara];
	fMat J;
	fVec3 com;
	getWorld()->Chain()->ComJacobian(J, com, cinfo.name.ptr());

	int height = J.row();
	int width = J.col();

	jacobian.setSize(height , width);
	int i = 0;
	for(int r=0; r < height; ++r){
		for(int c=0; c < width; ++c){
			(jacobian)(r,c) = J(r, c);
		}
	}
}

#include "UT_implementation/UT_common.h"

/*void DynamicsSimulator_UT::getWorldState(VRMLloader & ll)
{
	VRMLloader* l=&ll;
	OpenHRP::DynamicsSimulator&s=*this;
	OpenHRP::WorldState w;
	getWorldState(w);

	OpenHRP::LinkPositionSequence& lp=(*w.characterPositions)[0].linkPositions;


	for(int i=1; i<ll.numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform& )ll.bone(i);
		b.m_matCombined.setRotation(lp[b.mJoint->jointEndId-1].R);
		b.m_matCombined.setTranslation(lp[b.mJoint->jointEndId-1].p);
	}
}

void DynamicsSimulator_UT::setWorldState(VRMLloader const& ll)
{
	const VRMLloader* l=&ll;
	OpenHRP::DynamicsSimulator& s=*this;
	OpenHRP::World_UT::CharacterInfo& cinfo=getWorld()->characters[0];

	for(int i=1; i<l->numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform&)l->bone(i);
		quater q;
		vector3 t;
		q.setRotation(b.m_matRot);
		t.translation(b.m_matRot);
			
		if(b.mJoint->jointType==HRP_JOINT::ROTATE)
		{
			TString rotc=b.getRotationalChannels() ;
			m_real angles[3];
			q.getRotation(rotc, angles);

			int sj=b.mJoint->jointStartId;
			for(int jj=0, njj=rotc.length(); jj<njj; jj++)
			{
				int j=jj+sj;
				ASSERT(cinfo.jointIDs[j]==j);
				cinfo.links[j]->SetJointValue(angles[jj]);
			}
		}
		else if(b.mJoint->jointType==HRP_JOINT::FREE)
		{
			int j=b.mJoint->jointStartId;
			ASSERT(cinfo.jointIDs[j]==j);
			cinfo.links[j]->SetJointValue(toSDIMS(t), toSDIMS(q));
		}
	}
}
*/


void DynamicsSimulator_UT_getworldvel(Joint* joint, ::vector3 const& localpos, vector3& vel);
void DynamicsSimulator_UT_getworldacc(Joint* joint, ::vector3 const& localpos, vector3& vel);
void DynamicsSimulator_UT::getWorldVelocity(int ichara, VRMLTransform* b
											  , ::vector3 const& localpos
			, ::vector3& velocity) const
{

	OpenHRP::World_UT::CharacterInfo& cinfo=((DynamicsSimulator_UT*)this)->getWorld()->characters[ichara];
	Joint* joint=cinfo.links[b->HRPjointIndex(b->numHRPjoints()-1)];
	DynamicsSimulator_UT_getworldvel(joint,localpos,velocity);
}	

void DynamicsSimulator_UT::getWorldAcceleration(int ichara, VRMLTransform* b
											  , ::vector3 const& localpos
			, ::vector3& acc) const
{

	OpenHRP::World_UT::CharacterInfo& cinfo=((DynamicsSimulator_UT*)this)->getWorld()->characters[ichara];
	Joint* joint=cinfo.links[b->HRPjointIndex(b->numHRPjoints()-1)];
	DynamicsSimulator_UT_getworldacc(joint,localpos,acc);
}

void DynamicsSimulator_UT::getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const
{ 
	OpenHRP::World_UT::CharacterInfo& cinfo=((DynamicsSimulator_UT*)this)->getWorld()->characters[ichara];
	Joint* joint=cinfo.links[b->HRPjointIndex(b->numHRPjoints()-1)];
	fVec3 av;
	av.mul(joint->abs_att, joint->loc_ang_vel);

	angvel	=toBase(av);	
}

#include "UT_implementation/UT_common.h"

// output is compatible to MotionDOF class.
void DynamicsSimulator_UT::getLinkData(int ichara, LinkDataType t, vectorn& out)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	OpenHRP::DynamicsSimulator& s=*this;
	OpenHRP::World_UT::CharacterInfo& cinfo=getWorld()->characters[0];

	DynamicsSimulator_UT_getLinkData(l, cinfo, t, out);

}

/*
void DynamicsSimulator_UT_setLinkData(VRMLloader const& l, OpenHRP::World_UT::CharacterInfo& cinfo, DynamicsSimulator::LinkDataType t, vectorn const& in)
{
	ASSERT(in.size()==l.dofInfo.numDOF());

	fVec3 p;
	fMat33 r;
	double imag;
	fVec3 v;

	for(int i=1; i<l.numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(i);
			
		if(b.mJoint->jointType==HRP_JOINT::ROTATE)
		{
			int sj=b.mJoint->jointStartId;
			int sDOF=l.dofInfo.startT(i);
			int nDOF=l.dofInfo.endR(i)-sDOF;
			for(int jj=0; jj<nDOF; jj++)
			{
				ASSERT(cinfo.jointIDs[jj+sj]==jj+sj);
				ASSERT(l.dofInfo.hasAngles(i));

				switch(t)
				{
				case DynamicsSimulator::JOINT_VALUE:
					cinfo.links[sj+jj]->SetJointValue(in[sDOF+jj]);
					break;
				case DynamicsSimulator::JOINT_VELOCITY:
					cinfo.links[sj+jj]->SetJointVel(in[sDOF+jj]);
					break;
				case DynamicsSimulator::JOINT_ACCELERATION:
					cinfo.links[sj+jj]->SetJointAcc(in[sDOF+jj]);
					break;
				case DynamicsSimulator::JOINT_TORQUE:
					cinfo.links[sj+jj]->SetJointForce(in[sDOF+jj]);
					break;
				default:
					ASSERT(0);
				}
			}
		}
		else if(b.mJoint->jointType==HRP_JOINT::FREE)
		{
			int j=b.mJoint->jointStartId;
			ASSERT(cinfo.jointIDs[j]==j);

			ASSERT(l.dofInfo.hasTranslation(i));
			ASSERT(l.dofInfo.hasQuaternion(i));
			int sTDOF=l.dofInfo.startT(i);
			int sRDOF=l.dofInfo.startR(i);
			ASSERT(sRDOF-sTDOF==3);
			p(0)=in[sTDOF];
			p(1)=in[sTDOF+1];
			p(2)=in[sTDOF+2];
			imag=in[sRDOF];
			v(0)=in[sRDOF+1];
			v(1)=in[sRDOF+2];
			v(2)=in[sRDOF+3];

			switch(t)
			{
			case DynamicsSimulator::JOINT_VALUE:
				{
					quater q(imag,v(0), v(1), v(2));
					q.normalize();
					cinfo.links[j]->SetJointValue(p, toSDIMS(q));
				}
				break;
			case DynamicsSimulator::JOINT_VELOCITY:
				cinfo.links[j]->SetJointVel(p, v);
				imag=0.0;
				break;
			case DynamicsSimulator::JOINT_ACCELERATION:
				cinfo.links[j]->SetJointAcc(p, v);
				imag=0.0;
				break;
			case DynamicsSimulator::JOINT_TORQUE:
				cinfo.links[j]->SetJointForce(p, v);
				imag=0.0;
				break;
			default:
				ASSERT(0);
			}

			
		}
	}
}
*/

// output is compatible to MotionDOF class.
void DynamicsSimulator_UT::setLinkData(int ichara, LinkDataType t, vectorn const& in)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	OpenHRP::DynamicsSimulator& s=*this;
	OpenHRP::World_UT::CharacterInfo& cinfo=getWorld()->characters[0];
	DynamicsSimulator_UT_setLinkData(l, cinfo, t, in);

	
}

double DynamicsSimulator_UT::currentTime()
{
	return world->currentTime();
}
