
#include "physicsLib.h"
#include "OpenHRPcommon.h"


void OpenHRP::sphereInertia(double mass, matrix3 & inertia)
{
	inertia.identity();

	// 0.524*(2*L)^3=mass/1000 assuming sphere with water density where L is the half radius
	double L=pow(mass/0.524/1000, 1/3)/2;
	inertia*=2.0/5.0*mass*L*L;
}
OpenHRP::LinkInfo::LinkInfo()
{
	translation=vector3(0,0,0);
	rotation.identity();
	
	centerOfMass=vector3(0,0,0);

#ifdef ZERO_MASS_SUPPORTED
	mass=0;
	inertia.zero();
#else
	mass=0.1;	// 100g
	OpenHRP::sphereInertia(mass, inertia);
#endif
	//vectorn ulimit;
	//vectorn llimit;
	//vectorn uvlimit;
	//vectorn lvlimit;

    //--- for rotor inertia of servomotor  '01 Jun.29 s.kajita
	rotorInertia=0;
	gearRatio=1;
	equivalentInertia=0;
	jointId=-1;

	mother=-1;     // index = -1 for the root.
	sister=-1;     // index
	daughter=-1;   // index

// taesoo    SensorInfoSequence  sensors;

	// group of triangles
	mesh=NULL;
}
intvectorn OpenHRP::CollisionSequence::getCollisionLinkPairs()
{
	intvectorn  temp;
	for(int i=0; i<getNumLinkPairs(); i++)
		if(getCollisionPoints(i).size()>0)
			temp.pushBack(i);
	return temp;
}
