
#ifndef DYNAMICSSIMULATOR_QP_HEADER
#define DYNAMICSSIMULATOR_QP_HEADER

#include "CollisionDetector.h"
#include "Liegroup.h"

namespace OpenHRP {
class ConstrainedPoints
{
	public:
	int globalNumConstraintVectors;
	int globalNumFrictionVectors ;
	struct ConstraintPoint {
		int globalIndex;
		::vector3 point;
		::vector3 normalTowardInside[2];
		::vector3 defaultAccel[2];
		double normalProjectionOfRelVelocityOn0;
		double depth; // position error in the case of a connection point

		double mu;
		::vector3 relVelocityOn0;
		::quater orientationOn0;
		int globalFrictionIndex;
		int numFrictionVectors;
		::vector3 frictionVector[4][2];
	};
	struct ConstrainedLinkPair {
		std::vector<ConstraintPoint> constraintPoints;
		LinkPair const* linkPair;
		int ilinkPair;
	};
	std::vector<ConstrainedLinkPair> constrainedLinkPairs;
	ConstrainedPoints(){}
	virtual void solve(std::vector<LinkPair> const& collisionCheckLinkPairs, CollisionSequence& collisions)=0;
};


class DynamicsSimulator_QP 
{
protected:
	ConstrainedPoints* _contacts;
public:
	DynamicsSimulator_QP(DynamicsSimulator* sim);
	virtual ~DynamicsSimulator_QP();
	struct ContactBasis
	{
		int ibody;
		int ibone;
		int ilinkpair;
		double depth;
		::vector3 globalpos;
		::vector3 relvel;
		::vector3 normal;
		vector3N frictionNormal;
		int globalIndex;
		int globalFrictionIndex;
	};
	void getContactBases(std::vector<ContactBasis>& basis , double invFrictionCoef=0.5) const;
	void calcContactBasisAll(matrixn& v_all, matrixn & dot_v_all, int link_pair_count, double invfrictionCoef=0.5);
	void calcContactBoneIndex(int link_pair_count, intvectorn& boneIndex);
	int getNumContactLinkPairs() const;
	
	// ys
	void getLinkPairBodiesBones(intvectorn& ibodies, intvectorn& ibones) const;
};
}

#endif
