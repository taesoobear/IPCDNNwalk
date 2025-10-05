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
/** @file DynamicsSimulator/server/convCORBAUtil.cpp
 *
 */
#include "stdafx.h"
#include "DynamicsSimulator_UT.h"
#include <stack>
#include "ModelLoaderUtil_UT.h"
#include "Sensor.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "World_UT.h"
#include "psim.h"

using namespace OpenHRP;
using namespace std;

//#include <fstream>
//static std::ofstream logfile("model.log");
//static std::ofstream logfile;

static void to_mat33(const matrix3& a, fMat33& mat)
{
	mat(0,0) = a._11;
	mat(0,1) = a._12;
	mat(0,2) = a._13;
	mat(1,0) = a._21;
	mat(1,1) = a._22;
	mat(1,2) = a._23;
	mat(2,0) = a._31;
	mat(2,1) = a._32;
	mat(2,2) = a._33;
}

static void to_vec3(const vector3& a, fVec3& vec)
{
	vec(0) = a[0];
	vec(1) = a[1];
	vec(2) = a[2];
}


/*taesoo static void createSensors(OpenHRP::World* world, Joint* jnt,  SensorInfoSequence& iSensors)
{
	int numSensors = iSensors->length();

	for(int i=0; i < numSensors; ++i)
	{
		SensorInfo_var iSensor = iSensors[i];

		int id = iSensor->id();

		if(id < 0){
			std::cerr << "Warning:  sensor ID is not given to sensor " << iSensor->name()
					  << "of character " << jnt->CharName() << "." << std::endl;
		} else {

			int sensorType = Sensor::COMMON;

			switch(iSensor->type()) {
			case ::FORCE_SENSOR:		sensorType = Sensor::FORCE;				break;
			case ::RATE_GYRO:			sensorType = Sensor::RATE_GYRO;			break;
			case ::ACCELERATION_SENSOR: sensorType = Sensor::ACCELERATION;		break;
			case ::PRESSURE_SENSOR:		sensorType = Sensor::PRESSURE;			break;
			case ::PHOTO_INTERRUPTER:	sensorType = Sensor::PHOTO_INTERRUPTER; break;
			case ::VISION_SENSOR:		sensorType = Sensor::VISION;			break;
			case ::TORQUE_SENSOR:		sensorType = Sensor::TORQUE;			break;
			}

			CORBA::String_var name0 = iSensor->name();
			string name(name0);
			vector3_var p = iSensor->translation();
			static fVec3 localPos;
			static fMat33 localR;
			to_vec3(p, localPos);
			matrix3_var rot = iSensor->rotation();
			to_mat33(rot, localR);
			world->addSensor(jnt, sensorType, id, name, localPos, localR);
		}
	}
}*/

static inline double getLimitValue(vectorn& limitseq, double defaultValue)
{
	return (limitseq.size()== 0) ? defaultValue : limitseq[0];
}


static Joint* createLink(Chain* _chain, const char* charname, int index, const LinkInfoSequence& iLinks, Joint* pjoint)
{
	const LinkInfo&iLink = iLinks[index];

	//logfile << "create: " << iLink->name() << ", jointId = " << iLink->jointId() << endl;
	TString const& name = iLink.name;
	std::string myname;
	char sep[2];
	sep[0] = charname_separator;
	sep[1] = '\0';
	myname = std::string(name) + std::string(sep) + std::string(charname);
	Joint* jnt = new Joint(myname.c_str());

	_chain->AddJoint(jnt, pjoint);

	int jointId = iLink.jointId;
	jnt->i_joint = jointId;

	int jt = iLink.jointType;

	switch(jt)
	{
	case HRP_JOINT::FIXED:
		jnt->j_type = ::JFIXED;
		break;
	case HRP_JOINT::FREE:
		jnt->j_type = ::JFREE;
		break;
	case HRP_JOINT::ROTATE:
		jnt->j_type = ::JROTATE;
		break;
	case HRP_JOINT::BALL:	
		jnt->j_type = ::JSPHERE;
		break;
	case HRP_JOINT::SLIDE:
		jnt->j_type = ::JSLIDE;
		break;
	default:
		jnt->j_type = ::JFREE;
	}

	if(jointId < 0)
	{
		if(jnt->j_type == ::JROTATE || jnt->j_type == ::JSLIDE || jnt->j_type == ::JSPHERE)
		{
			std::cerr << "Warning:  Joint ID is not given to joint " << jnt->name
					  << " of character " << charname << "." << std::endl;
		}
	}

	static fVec3 rel_pos;
	to_vec3(iLink.translation, rel_pos);

	static fMat33 rel_att;
	to_mat33(iLink.rotation, rel_att);

	// joint axis is always set to z axis; use init_att as the origin
	// of the joint axis
	if(jnt->j_type == ::JROTATE || jnt->j_type == ::JSLIDE)
	{
//	taesoo commented out	static fVec3 loc_axis;
//	taesoo commented out	to_vec3(iLink.jointAxis, loc_axis);
		//logfile << "loc_axis = " << loc_axis << endl;
		//logfile << "rel_att = " << rel_att << endl;
		//logfile << "rel_pos = " << rel_pos << endl;
#if 0
		static fMat33 init_att;
		static fVec3 p_axis;
		p_axis.mul(rel_att, loc_axis);  // joint axis in parent frame -> z axis
		static fVec3 x, y;
		x.set(1.0, 0.0, 0.0);
		y.set(0.0, 1.0, 0.0);
		double zx = p_axis*x;
		x -= zx * p_axis;
		double xlen = x.length();
		if(xlen > 1e-8)
		{
			x /= xlen;
			y.cross(p_axis, x);
		}
		else
		{
			double yz = y*p_axis;
			y -= yz * p_axis;
			double ylen = y.length();
			y /= ylen;
			x.cross(y, p_axis);
		}
		init_att(0,0) = x(0); init_att(1,0) = x(1); init_att(2,0) = x(2);
		init_att(0,1) = y(0); init_att(1,1) = y(1); init_att(2,1) = y(2);
		init_att(0,2) = p_axis(0); init_att(1,2) = p_axis(1); init_att(2,2) = p_axis(2);
		if(jnt->j_type == JROTATE)
			jnt->SetRotateJointType(rel_pos, init_att, AXIS_Z);
		else if(jnt->j_type == JSLIDE)
			jnt->SetSlideJointType(rel_pos, init_att, AXIS_Z);
//		logfile << "init_att = " << init_att << endl;
#else
		AxisIndex axis = AXIS_NULL;
		if(iLink.jointAxis=="X") axis = AXIS_X;
		else if(iLink.jointAxis=="Y") axis = AXIS_Y;
		else if(iLink.jointAxis=="Z") axis = AXIS_Z;
#if 0	// taesoo commented out
		if(loc_axis(0) > 0.95) 
		else if(loc_axis(1) > 0.95) axis = AXIS_Y;
		else if(loc_axis(2) > 0.95) axis = AXIS_Z;
#endif
		assert(axis != AXIS_NULL);
		if(jnt->j_type == JROTATE)
			jnt->SetRotateJointType(rel_pos, rel_att, axis);
		else if(jnt->j_type == JSLIDE)
			jnt->SetSlideJointType(rel_pos, rel_att, axis);
#endif
//		logfile << "n_dof = " << jnt->n_dof << endl;
	}
	else if(jnt->j_type == ::JSPHERE)
	{
		jnt->SetSphereJointType(rel_pos, rel_att);
	}
	else if(jnt->j_type == ::JFIXED)
	{
		jnt->SetFixedJointType(rel_pos, rel_att);
	}
	else if(jnt->j_type == ::JFREE)
	{
//		logfile << "rel_pos = " << rel_pos << endl;
//		logfile << "rel_att = " << rel_att << endl;
		jnt->SetFreeJointType(rel_pos, rel_att);
	}
	
	jnt->mass = iLink.mass;

	double equivalentInertia = iLink.equivalentInertia;

	if(equivalentInertia == 0.0){
		jnt->rotor_inertia = iLink.rotorInertia;
		jnt->gear_ratio = iLink.gearRatio;
	} else {
		jnt->rotor_inertia = equivalentInertia;
		jnt->gear_ratio = 1.0;
	}
		
//	link->Jm2	        = iLink.equivalentInertia();
//	link->torqueConst	= iLink.torqueConst();
//	if (link->Jm2 == 0){
//		link->Jm2 = link->Ir * link->gearRatio * link->gearRatio;
//	}
//	link->encoderPulse	= iLink->encoderPulse();

//	vectorn_var ulimit  = iLink->ulimit();
//	vectorn_var llimit  = iLink->llimit();
//	vectorn_var uvlimit = iLink->uvlimit();
//	vectorn_var lvlimit = iLink->lvlimit();

//	double maxlimit = numeric_limits<double>::max();

//	link->ulimit  = getLimitValue(ulimit,  +maxlimit);
//	link->llimit  = getLimitValue(llimit,  -maxlimit);
//	link->uvlimit = getLimitValue(uvlimit, +maxlimit);
//	link->lvlimit = getLimitValue(lvlimit, -maxlimit);

	static fVec3 com;
	to_vec3(iLink.centerOfMass, com);
	jnt->loc_com.set(com);

	static fMat33 inertia;
	to_mat33(iLink.inertia, inertia);
	jnt->inertia.set(inertia);

	int sindex = iLinks[index].sister;
	if(sindex >= 0) createLink(_chain, charname, sindex, iLinks, pjoint);

	int cindex = iLink.daughter;
	if(cindex >= 0) createLink(_chain, charname, cindex, iLinks, jnt);

//taesoo	createSensors(world, jnt, iLink->sensors());

	return jnt;
}


int OpenHRP::loadBodyFromCharacterInfo(World_UT_base* world, const char* _name, const CharacterInfo* charaInfo)
{
//	logfile << "loadBody(" << _name << ")" << endl;
	pSim* _chain = world->Chain();
	_chain->BeginCreateChain(true);
	// no root
	if(!_chain->Root())
	{
		_chain->AddRoot("space");
	}

	TString name = _name;

	int n = charaInfo->links.size();
	const LinkInfoSequence& iLinks = charaInfo->links;
	int failed = true;

	for(int i=0; i < n; ++i)
	{
		if(iLinks[i].mother < 0)  // root of the character
		{
			Joint* r = createLink((Chain*)world->Chain(), name, i, iLinks, _chain->Root());
			world->addCharacter(r, _name, charaInfo->links);
			failed = false;
			break;
		}
	}
#if 0
	int rootIndex = -1;
	if(body){
		matrix33 Rs(tvmet::identity<matrix33>());
		Link* rootLink = createLink(body, rootIndex, iLinks, Rs, importLinkShape);
		body->setRootLink(rootLink);

		vector3_var p = iLinks[rootIndex]->translation();
		vector3 pos(p[0u], p[1u], p[2u]);
		matrix3_var R = iLinks[rootIndex]->rotation();
		matrix33 att;
		getMatrix33FromRowMajorArray(att, R);
		body->setDefaultRootPosition(pos, att);

		body->installCustomizer();

		body->initializeConfiguration();
	}
#endif
	_chain->EndCreateChain();
//	logfile << "end of loadBody" << endl;
//	logfile << "total dof = " << _chain->NumDOF() << endl;
	return failed;
}


#include <ik.h>
void makeCharacterInfo(VRMLloader const& input, OpenHRP::CharacterInfo& output);
#include "UT_implementation/UT_common.h"
// void DynamicsSimulator_UT_setLinkData(VRMLloader const& l, OpenHRP::World_UT_base::CharacterInfo& cinfo, DynamicsSimulator::LinkDataType t, vectorn const& in);
// void DynamicsSimulator_UT_getLinkData(VRMLloader const& l, OpenHRP::World_UT_base::CharacterInfo& cinfo, DynamicsSimulator::LinkDataType t, vectorn& out);
// inline fVec3 toSDIMS(vector3 const& v)
// {
// 	return fVec3(v.x, v.y, v.z);
// }

// Abstract class. All IK solvers should reimplement IKsolve(...)
class FullbodyIK_UTPoser: public MotionUtil::FullbodyIK_MotionDOF
{
	World_UT_base::CharacterInfo cinfo;
	IK* _chain;
	VRMLloader const* _loader;
	std::vector<IKHandle*> _handles;
	IKCom* _com;

public:
	int numIter;
	double stepSize;
	virtual ~FullbodyIK_UTPoser(){delete _chain;}
	FullbodyIK_UTPoser(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors)
	{
		numIter=500;
		stepSize=0.05;
		_chain = new IK();
		_com=NULL;
		OpenHRP::CharacterInfo cinfotemp;
		_loader=&(const VRMLloader&)dofInfo.skeleton();

		VRMLloader const& loader=*_loader;
		makeCharacterInfo(loader, cinfotemp);
		OpenHRP::CharacterInfo *charaInfo=&cinfotemp;

		_chain->BeginCreateChain(true);
		// no root
		if(!_chain->Root())
		{
			_chain->AddRoot("space");
		}

		TString name = loader.name;

		int n = charaInfo->links.size();
		const LinkInfoSequence& iLinks = charaInfo->links;
		int failed = true;

		for(int i=0; i < n; ++i)
		{
			if(iLinks[i].mother < 0)  // root of the character
			{
				static fMat33 Rs;
				Rs.identity();
				Joint* r = createLink(_chain, name, i, iLinks, _chain->Root());

				cinfo.root=r;
				cinfo.name=loader.name;
				
				int n_links = iLinks.size();
				for(int i=0; i<n_links; i++)
				{
					const LinkInfo* linfo = &(iLinks[i]);
					Joint* jnt = _chain->FindJoint(linfo->name, name.ptr());
					if(jnt)
					{
						cinfo.links.push_back(jnt);
						cinfo.jointIDs.push_back(linfo->jointId);
						if(linfo->jointId >= 0) cinfo.n_joints++;
					}
				}
				failed = false;
				break;
			}
		}

		_handles.reserve(effectors.size());
		for(int i=0; i<effectors.size(); i++)
		{
			VRMLTransform* b=(VRMLTransform* )effectors[i].bone;

			if(b==NULL)
			{
				
				IK::ConstIndex cindex[3];
				for(int i=0; i<3; i++)
					cindex[i]=(effectors[i].localpos[i]==0.0)?IK::NO_CONSTRAINT:IK::HAVE_CONSTRAINT;

				ASSERT(_com==NULL);
				_com=new IKCom(_chain, name.ptr(), cindex, IK::LOW_PRIORITY,10);
				VERIFY(_chain->AddConstraint(_com)==i);
				_handles.push_back(NULL);
			}
			else
			{
				fVec3 _rel_pos=toSDIMS(effectors[i].localpos);
				_handles.push_back(_chain->AddMarker(RE::generateUniqueName().ptr(), b->name().ptr(), name.ptr(), _rel_pos));
			}
		}

		_chain->init();
	}


	// interface type 1. All derived classes should reimplement this interface.
	virtual void IKsolve(vectorn const& input, vectorn& output, vector3N const& constraintPositions)
	{
		DynamicsSimulator_UT_setLinkData(*_loader, cinfo, DynamicsSimulator::JOINT_VALUE, input);

		for(int i=0; i<constraintPositions.size(); i++)
		{
			if(_handles[i])
				_handles[i]->SetPos(toSDIMS(constraintPositions[i]));
			else
				_com->SetPos(toSDIMS(constraintPositions[i]));
		}

		for(int i=0; i<numIter; i++)
			_chain->Update(stepSize);

		DynamicsSimulator_UT_getLinkData(*_loader, cinfo, DynamicsSimulator::JOINT_VALUE, output);
	}
};

MotionUtil::FullbodyIK_MotionDOF* MotionUtil::createFullbodyIk_UTPoser(MotionDOFinfo const& info, std::vector<Effector>& effectors)
{
	return new FullbodyIK_UTPoser(info, effectors);
}
void MotionUtil::FullbodyIK_UTPoser_setParam(MotionUtil::FullbodyIK_MotionDOF* _solver, int numiter, double stepsize)
{
	FullbodyIK_UTPoser* solver=(FullbodyIK_UTPoser* ) _solver;

	solver->numIter=numiter;
	solver->stepSize=stepsize;
}


