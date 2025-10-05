#include "physicsLib.h"
#include "OpenHRPcommon.h"
#include "DynamicsSimulator_penaltyMethod.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/math/conversion.h"
#include "CollisionDetector.h"

void _setMaterial(Ogre::Renderable* ptr, const char* name);

// taesoo. global parameters.
namespace OpenHRP
{

static double _footSize=6;
static vector3 _trans=vector3(0,0.01,0);
static double _front=0;
static vector3 _contactForceVis(0.01, 0.01, 0.01);
DynamicsSimulator_penaltyMethod::DynamicsSimulator_penaltyMethod(bool useSimpleColdet):
	DynamicsSimulator(useSimpleColdet){}
DynamicsSimulator_penaltyMethod::DynamicsSimulator_penaltyMethod(const char* coldet):
	DynamicsSimulator(coldet){}

void DynamicsSimulator_penaltyMethod::setSimulatorParam(const char* string, vectorn const& value)
{
	TString id(string);

	if(id=="debugContact")
	{
		_footSize=value[0];
		_trans.x=value[1];
		_trans.y=value[2];
		_trans.z=value[3];
		_front=value[4];
	}
	else if(id=="penaltyDepthMax")
	{
		//_depthMax=value[0];
	}
	else if(id=="contactForceVis")
	{
		_contactForceVis.x=value[0];
		_contactForceVis.y=value[1];
		_contactForceVis.z=value[2];
	}
}
	


void DynamicsSimulator_penaltyMethod::registerCollisionCheckPair
(
 const char *charName1,
 const char *linkName1,
 const char *charName2,
 const char *linkName2,
 vectorn const& param
 )
{
	double staticFriction=param[0];
	double slipFriction=param[1];
	LinkPair linkPair ;
	linkPair.charName1  = charName1;
	linkPair.linkName1 = linkName1;
	linkPair.charName2  =charName2;
	linkPair.linkName2 = linkName2;
	linkPair.param=param;
	collisionDetector->addCollisionPair(linkPair, false, false);
}


void DynamicsSimulator_penaltyMethod::drawDebugInformation() 
{
#ifndef NO_GUI
	double scaleFactor=100;	// simulation: METER unit, rendering: CM unit.
	
	CollisionSequence& corbaCollisionSequence=*collisions;
	if (corbaCollisionSequence.seq.size()==0) return;
	
	mObjectContactVisualization.clear();
#define DRAW_ALL
#ifdef DRAW_ALL
	int nc=0;
	int nbc=0;
	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	  {
	    LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
	    int cqindex=(_boneToCQindex.size())?_boneToCQindex[linkPair.link[0]->treeIndex()]:-1;
	    if(cqindex!=-1 && _contactQueryBones[cqindex].coef<=0.0)
	      nbc+=corbaCollisionSequence[i].points.size();
	    else
	      nc+=corbaCollisionSequence[i].points.size();
	  }

	if(nc==0 && nbc==0) return;
	QuadList* box=new QuadList(vector3(0,1,0), _footSize);
	QuadList* box2=new QuadList(vector3(0,1,0), _footSize);

	if(nc!=0)
	  box->begin(nc);
	if(nbc!=0)
	  box2->begin(nbc);
	
	int c=0;
	int bc=0;

	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	  {
	    CollisionPointSequence& points = corbaCollisionSequence[i].points;
	    LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
	    int cqindex=(_boneToCQindex.size())?_boneToCQindex[linkPair.link[0]->treeIndex()]:-1;
	    if(cqindex!=-1 && _contactQueryBones[cqindex].coef<=0.0)
	    {
	      	int n_point = points.size();
		for(int j=0; j<n_point; j++)
		{
			const vector3& pos=points[j].position;
			box2->quad(bc++, pos*scaleFactor);
		}
	    }
	    else
	    { 
		int n_point = points.size();
		for(int j=0; j<n_point; j++)
		{
			const vector3& pos=points[j].position;
			box->quad(c++, pos*scaleFactor);
		}
	    }
	}
	ASSERT(c==nc);
	ASSERT(bc==nbc);
	if(nc!=0)
	  box->end();
	else
	{
	   delete box;
	   box=NULL;
	}
	if(nbc!=0)
	  box2->end();
	else
	{
	    delete box2;
		box2=NULL;
	}
	if (box)
	box->setMaterial( "redCircle");
	if(box2)
	box2->setMaterial( "blueCircle");
#else

	int nc=0;
	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		nc+=(corbaCollisionSequence[i].points.size()==0)?0:1;
	}

	if(nc==0) 
	{
		mObjectContactVisualization.clear();
		return;
	}
	QuadList* box=new QuadList(vector3(0,1,0), _footSize);
	box->setMaterial( "redCircle");

	box->begin(nc);
	
	int c=0;

	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		CollisionPointSequence& points = corbaCollisionSequence[i].points;

		int n_point = points.size();
		if(n_point==0) continue;
		vector3 avg(0,0,0);
		for(int j=0; j<n_point; j++)
		{
			const vector3& pos=points[j].position;
			avg+=pos*scaleFactor;
		}
		box->quad(c++, avg/n_point);
	}
	ASSERT(c==nc);

	box->end();
#endif

	
	vector3 dir(_trans);
	vector3 front;
	front.rotate(getWorldState(0).global(1).rotation, vector3(0,0,1));
	front.y=0;
	front.normalize();
	dir+=front*_front;		
	
	if(nc!=0)
	RE::moveEntity(mObjectContactVisualization.registerObject(RE::generateUniqueName(), box), dir*100);
	if(nbc!=0)
	RE::moveEntity(mObjectContactVisualization.registerObject(RE::generateUniqueName(), box2), dir*100);

	if(_contactForces.size())
	{
	vector3N lines;

	for( int i=0; i<_contactForces.size(); i++)
	{
		//RE::output("contactForce", "%s %s", _contactForces[i].p.output().ptr(), 
		//	_contactForces[i].f.output().ptr());

		if(_contactForces[i].chara==0)
		{
			transf const& frame1=getWorldState(_contactForces[i].chara)._global(*_contactForces[i].bone);

			vector3 p=frame1.toGlobalPos(_contactForces[i].p);
			vector3 f=frame1.toGlobalDir(_contactForces[i].f);
			lines.pushBack(p*100+dir*100);
			lines.pushBack((p+f*_contactForceVis)*100+dir*100);
		}
	}

	mObjectContactVisualization.registerObject("contactForce", "LineList", "solidblue", matView(lines));
	}
#endif
}






}

		

