#ifndef DS_TRBDL_IMPULSE_H_
#define DS_TRBDL_IMPULSE_H_

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
#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
#include <stdio.h> //printf debugging

#include "DynamicsSimulator_Trbdl_penalty.h"
#include "../../BaseLib/motion/Terrain.h"

namespace Trbdl
{
	class SimArticulatedBody;

class DynamicsSimulator_Trbdl_impulse : public DynamicsSimulator_Trbdl_penalty
{
protected:
	double _restitution;
	double _MA;

	std::vector<SimArticulatedBody*> _bodies;

	void clearJacCache();
	void resolveCollisions(double dt, int niter, bool noCollectContactForces);
	vector3N _contactForces; // indexed by ilink-pair
	vector3N _contactPos; // indexed by ilinkpair
public:

	inline SimArticulatedBody* body(int bodyIndex){ return _bodies[bodyIndex];}

	virtual bool stepSimulation();

	DynamicsSimulator_Trbdl_impulse(const char* coldet);
	virtual ~DynamicsSimulator_Trbdl_impulse();

	void init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);
	virtual void initSimulation();

	virtual void _registerCharacter(const char *name, OpenHRP::CharacterInfo const& cinfo);

	void setParam_restitution_MA(double r, double ma);
	virtual void drawLastContactForces(int ichar, ::vector3 const& draw_offset) const override final;
	virtual ::vector3 getContactForce(int ichar, int ibone) const override final;
private:
};
}
#endif
