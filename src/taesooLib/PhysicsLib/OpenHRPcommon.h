#pragma once

typedef double m_real;
#include "../BaseLib/math/matrix3.h"
#include "../MainLib/OgreFltk/Mesh.h"
class VRMLloader;
class VRMLTransform;
//#include <ModelLoader.h>
//#include <CollisionDetector.h>
//#include <DynamicsSimulator.h>

//#include "World.h"
namespace OpenHRP
{
	void sphereInertia(double mass, matrix3 & inertia);
//	class World;

	typedef _tvector<double, 6> vector6;

	/**
   * LinkInfo Interface
   * @author  Ichitaro Kohara, MSTC
   // taesoo commented out all variables that are not in use, and also the variables that don't seem very useful.
   */
  struct LinkInfo
  {
	  LinkInfo();
  
	::vector3 translation;
	matrix3 rotation;
	double	 mass;
	::vector3 centerOfMass;
	matrix3 inertia;
	//vectorn ulimit;
	//vectorn llimit;
	//vectorn uvlimit;
	//vectorn lvlimit;

    //--- for rotor inertia of servomotor  '01 Jun.29 s.kajita
	double rotorInertia;
	double gearRatio;
	double equivalentInertia;
	TString    name;
	int jointType;	// HRP_JOINT::jointType_T
	long      jointId;
	TString jointAxis;

	long      mother;     // index = -1 for the root.
	long      sister;     // index
	long      daughter;   // index

	// group of triangles
	OBJloader::Mesh* mesh;
  };
	typedef std::vector<LinkInfo> LinkInfoSequence;

	struct CharacterInfo
	{
		TString name;
		TString url;
		TStrings info;
		VRMLloader* loader;
		LinkInfoSequence links;
	};

	struct LinkPair {
		TString charName1;
		TString linkName1;
		TString charName2;
		TString linkName2;
		// param [0:staticFriction, 1:slipFriction, 2:penaltyForceStiffness, 3:penaltyForceDamp]
		vectorn param;
		double margin;

		// followings will be automatically set.
		int charIndex[2];	// character index.
		VRMLTransform* link[2];
	};

	struct CollisionPoint
	{
		::vector3 position; // unused for soft bodies
		::vector3 normal;
		double	  idepth;
		int inode[2]; // unused for articulated rigid bodies
	};

	typedef std::vector<CollisionPoint> CollisionPointSequence;
	
	struct Collision
	{
		CollisionPointSequence points;
	};

	struct CollisionSequence
	{
		std::vector<Collision> seq;
		std::vector<LinkPair> * pairs;
		CollisionSequence(){}
		intvectorn getCollisionLinkPairs();
		inline Collision& operator[](int i) { return seq[i];}
		inline CollisionPointSequence& getCollisionPoints(int ilinkpair){return seq[ilinkpair].points;}
		inline int getNumLinkPairs() {return seq.size();}
		inline int getCharacterIndex1(int ilinkpair){ return (*pairs)[ilinkpair].charIndex[0];}
		inline VRMLTransform* getBone1(int ilinkpair){ return (*pairs)[ilinkpair].link[0];}
		inline int getCharacterIndex2(int ilinkpair){ return (*pairs)[ilinkpair].charIndex[1];}
		inline VRMLTransform* getBone2(int ilinkpair){ return (*pairs)[ilinkpair].link[1];}
	};

}
