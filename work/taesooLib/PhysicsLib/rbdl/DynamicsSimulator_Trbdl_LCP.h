#ifndef DS_TRBDL_LCP_H_
#define DS_TRBDL_LCP_H_

#pragma once
/** \file
	\author Taesoo Kwon
*/

#include "../../BaseLib/math/Metric.h"
//#include "../../BaseLib/motion/TransitionCost.h"
#include "../../BaseLib/motion/FullbodyIK.h"
#include "../../BaseLib/utility/scoped_ptr.h"
#include "../../BaseLib/math/Operator.h"
#include "../../MainLib/OgreFltk/Mesh.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"
#include <stdio.h> //printf debugging

#include "DynamicsSimulator_Trbdl_penalty.h"
#include "../../BaseLib/motion/Terrain.h"
//#include "../CollisionDetector/CollisionDetector_libccd_LBS.h"

#include "LCPsolver.h"
namespace Trbdl
{
	class GArticulatedBody;
	class GArticulatedBodyLink;
	class DynamicsSimulator_Trbdl_LCP;
	class LCPsolver;

class DynamicsSimulator_Trbdl_LCP : public DynamicsSimulator_Trbdl_penalty
{
protected:
	friend class GArticulatedBody;
	mutable vectorn _f;
	mutable vector3N contactPos;
	std::vector<GBody*> _bodies;
	void _addEmptyCharacter(const char* name);
	void _addBody(GBody* body);
	Trbdl::LCPsolver* _contactForceSolver;

public:
	double _MA;
	int numBodies() { return _bodies.size();}
	inline std::vector<GBody*>& bodiesData() {return _bodies;}

	inline GBody* body(int bodyIndex){ return _bodies[bodyIndex];}

	const matrixn& getMLCP() const;
	const vectorn& getMLCP_B() const;
	const vectorn& getMLCP_X() const;

	virtual bool stepSimulation();

	void get_contact_pos(int ichar, vector3N& cpos, OpenHRP::CollisionSequence& collisionSequence) const;
	DynamicsSimulator_Trbdl_LCP();
	DynamicsSimulator_Trbdl_LCP(const char* coldet);
	virtual ~DynamicsSimulator_Trbdl_LCP();

	void init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);
	virtual void initSimulation();

	virtual void _registerCharacter(const char *name, OpenHRP::CharacterInfo const& cinfo);

	void setParam_Epsilon_Kappa(double eps, double kap);
	void setParam_R_B_MA(double r, double b, double ma);
	void _registerCollisionCheckPair(int ibody1, int ibody2, int treeIndex1, int treeIndex2, vectorn const& param);
	void registerCollisionCheckPair( const char *charName1, const char *linkName1, const char *charName2, const char *linkName2, vectorn const& param);
	void registerCollisionCheckPair( const int bodyIndex1, const char* linkName1, const int bodyIndex2, const char *linkName2, vectorn const& param);
	void registerAllCollisionCheckPairs(int ibody1, int ibody2, vectorn const& param);

	void addRelativeConstraint(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2);
	void removeRelativeConstraint(int ichara, Bone& bone1, Bone& bone2);

	virtual void drawLastContactForces(int ichara=0, vector3 const& draw_offset=vector3(0,0,0)) const override final;
	virtual ::vector3 getContactForce(int ichar, int ibone) const override final;
	Liegroup::dse3 getCOMbasedContactForce(int ichar, int ibone) const; // returns (r x f, f)
private:
};
}
#endif
