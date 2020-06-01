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
#ifndef OPENHRP_WORLD_UT_H_INCLUDED
#define OPENHRP_WORLD_UT_H_INCLUDED

#include <vector>
#include <map>
#include <list>
#include <vector>
#include "sDIMS/fMatrix3.h"
#include "DynamicsSimulator_UT.h"
#include "hrpModelExportDef.h"

class pSim;
class Joint;
class SDContactPair;

namespace OpenHRP {

	class Sensor;
	class ForceSensor;
	class RateGyroSensor;
	class AccelSensor;
    
	class World_UT_base
	{
	protected:

		void _get_all_character_data_sub(Joint* cur, int index, OpenHRP::DynamicsSimulator::LinkDataType type, vectorn& rdata);
		void _set_all_character_data_sub(Joint* cur, int index, OpenHRP::DynamicsSimulator::LinkDataType type, const vectorn& wdata);

        double currentTime_;
        double timeStep_;
		 fVec3 g;

        bool isEulerMethod; // Euler or Runge Kutta ?
		pSim* chain;
	public:
		World_UT_base();
        virtual ~World_UT_base();


		 class CharacterInfo
		{
		public:
			CharacterInfo(Joint* _root, const TString& _name): name(_name)
			{
				root = _root;
				n_joints = 0;
			}
			CharacterInfo(){}
			TString name;
			std::vector<Joint*> links;
			std::vector<int> jointIDs;
			int n_joints;
			Joint* root;
			~CharacterInfo() {}
		};
		std::vector<CharacterInfo> characters;

		void setTimeStep(double);
		double timeStep(void) const { return timeStep_; }
	
		void setCurrentTime(double);
		double currentTime(void) const { return currentTime_; }
	
		void setGravityAcceleration(const fVec3& g);
		const fVec3& getGravityAcceleration() { return g; }

		void setEulerMethod();
		void setRungeKuttaMethod();

		virtual void initialize();
		virtual void calcNextState(CollisionDetector* collisionDetector, CollisionSequence& corbaCollisionSequence)=0;

//		std::pair<int,bool> getIndexOfLinkPairs(BodyPtr body1, Link* link1, BodyPtr body2, Link* link2);

		Joint* rootJoint(int index);
		

		void getAllCharacterData(const char* name, OpenHRP::DynamicsSimulator::LinkDataType type, vectorn& rdata);
		void setAllCharacterData(const char* name, OpenHRP::DynamicsSimulator::LinkDataType type, const vectorn& wdata);
		
		void calcCharacterJacobian(const char* characterName, const char* baseLink, const char* targetLink, fMat& J);

		pSim* Chain() {
			return chain;
		}
		void addCharacter(Joint* rjoint, const TString& _name, LinkInfoSequence const& links);
		int numLinks(int index) {
			return characters[index].links.size();
		}
		int numJoints(int index) {
			return characters[index].n_joints;
		}
		
		int numCharacter() {
			return characters.size();
		}

	};
	class World_UT: public World_UT_base
    {
    public:
       
		World_UT();
        ~World_UT();

		void clearCollisionPairs();

		

		void enableSensors(bool on);
		
		virtual void initialize();
		virtual void calcNextState(CollisionDetector*, CollisionSequence& corbaCollisionSequence);

		void addCollisionCheckLinkPair(Joint* jnt1, Joint* jnt2, double staticFriction, double slipFriction, double epsilon);

		
		
	protected:
		std::vector<SDContactPair*> contact_pairs;
	public:

	private:
		
		

		int numRegisteredLinkPairs;
		
       

		
	};


};

#endif
