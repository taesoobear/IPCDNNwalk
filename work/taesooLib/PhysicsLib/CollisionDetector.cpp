#include "physicsLib.h"
#include "CollisionDetector.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/motion/VRMLloader_internal.h"
#include "../MainLib/OgreFltk/MotionManager.h"
#include "../BaseLib/motion/Geometry.h"

void OpenHRP::CollisionDetector::addModel(const char* charName, CharacterInfo const& model)
{
	addModel(model.loader);
	mTreeNames.back()=charName; // overwrite name.
}


int OpenHRP::CollisionDetector::addObstacle(OBJloader::Geometry const& mesh)
{
	OpenHRP::CharacterInfo cinfo;
	VRMLloader* l=new VRMLloader(mesh, false);
	TString name=RE::generateUniqueName();
	l->name=name;
	l->url=name+".wrl";
	RE::motionManager().createMotionLoaderExt(name, (MotionLoader*)l);
	VRMLloader_updateMeshEntity(*l);
	addModel(l);
	return mTrees.size()-1;
}
int OpenHRP::CollisionDetector::addModel(VRMLloader* loader)
{
	Msg::verify(mTrees.size()==mTreeNames.size(), "CollisionDetector::addModel?");
	mTrees.push_back(loader);
	mTreeNames.push_back(std::string(loader->name.ptr()));
	return mTrees.size()-1;
}

bool OpenHRP::CollisionDetector::queryContactDeterminationForDefinedPairs(
		std::vector<DynamicsSimulator::Character*> const& characters, CollisionSequence & collisions)
{
	// 1. set world transforms.
	for(int i=0; i<characters.size(); i++)
	{
		BoneForwardKinematics& fk=*characters[i]->chain;
		setWorldTransformations(i, fk);
	}
	return testIntersectionsForDefinedPairs(collisions);
}
static int charaIndex(std::vector<std::string> const& _characterNames, const char* name)
{
	for(int i=0; i<_characterNames.size(); i++)
		if(_characterNames[i]==name)
			return i;
	return -1;
}
// deprecated
static int charaIndex(std::vector<VRMLloader*> const& _characters, VRMLloader* skel)
{
	for(int i=0; i<_characters.size(); i++)
		if(_characters[i]==skel)
			return i;
	return -1;
}

void OpenHRP::CollisionDetector::setMargin(int ilink, double margin)
{
	mPairs[ilink].margin=margin;
}

void OpenHRP::CollisionDetector::setMarginAll(vectorn const& margin)
{
  for(int i=0; i<mPairs.size(); i++)
	mPairs[i].margin=margin[i];
}
void OpenHRP::CollisionDetector::getMarginAll(vectorn & margin)
{
  margin.setSize(mPairs.size());
  for(int i=0; i<mPairs.size(); i++)
	margin[i]=mPairs[i].margin;
}
void OpenHRP::CollisionDetector::addCollisionPair(LinkPair const& colPair, bool convexsize1, bool convexsize2)
{
	mPairs.push_back(colPair);
	LinkPair& pair=mPairs.back();
	pair.charIndex[0]=charaIndex(mTreeNames, pair.charName1);
	pair.charIndex[1]=charaIndex(mTreeNames, pair.charName2);
	pair.margin=0;

	for(int i=0; i<2; i++)
	{
		if(pair.charIndex[i]==-1)
		{
			Msg::msgBox("addCollisionPair: character %s or %s not found", pair.charName1.ptr(), pair.charName2.ptr());
			mPairs.resize(mPairs.size()-1);
			return;
		}

		VRMLloader* body = mTrees[pair.charIndex[i]];
		int lindex;
		if(i==0)
			lindex=body->GetIndex(pair.linkName1);
		else
			lindex=body->GetIndex(pair.linkName2);

		if(lindex==-1)
		{
			Msg::msgBox("bone %s or %s not found", pair.linkName1.ptr(), pair.linkName2.ptr());
			mPairs.resize(mPairs.size()-1);
			return;
		}

		pair.link[i]=&body->VRMLbone(lindex);
	}
}
void OpenHRP::CollisionDetector::_addCollisionPair(int ichara1, int ibone1, int ichara2, int ibone2)
{

	mPairs.resize(mPairs.size()+1);
	LinkPair& pair=mPairs.back();
	pair.charIndex[0]=ichara1;
	pair.charIndex[1]=ichara2;
	pair.margin=0;

	for(int i=0; i<2; i++)
	{
		if(pair.charIndex[i]==-1)
		{
			Msg::msgBox("??? character not found %d", pair.charIndex[i]);
			mPairs.resize(mPairs.size()-1);
			return;
		}

		VRMLloader* body = mTrees[pair.charIndex[i]];
		int lindex;
		if(i==0)
			lindex=ibone1;
		else
			lindex=ibone2;

		if(lindex==-1)
			pair.link[i]=NULL; // not a bone.
		else
			pair.link[i]=&body->VRMLbone(lindex);
	}
}
// deprecated
void OpenHRP::CollisionDetector::addCollisionPair(VRMLloader* skel1, int ibone1, VRMLloader* skel2, int ibone2)
{

	mPairs.resize(mPairs.size()+1);
	LinkPair& pair=mPairs.back();
	pair.charIndex[0]=charaIndex(mTrees, skel1);
	pair.charIndex[1]=charaIndex(mTrees, skel2);
	pair.margin=0;

	for(int i=0; i<2; i++)
	{
		if(pair.charIndex[i]==-1)
		{
			Msg::msgBox("character (skel) not found");
			mPairs.resize(mPairs.size()-1);
			return;
		}

		VRMLloader* body = mTrees[pair.charIndex[i]];
		int lindex;
		if(i==0)
			lindex=ibone1;
		else
			lindex=ibone2;

		if(lindex==-1)
		{
			Msg::msgBox("bone not found");
			mPairs.resize(mPairs.size()-1);
			return;
		}

		pair.link[i]=&body->VRMLbone(lindex);
	}
}

/*
rayTest
void	btCollisionWorld::rayTestSingle(const btTransform& rayFromTrans,const btTransform& rayToTrans,
										btCollisionObject* collisionObject,
										const btCollisionShape* collisionShape,
										const btTransform& colObjWorldTransform,
										RayResultCallback& resultCallback)
{
	btCollisionObjectWrapper colObWrap(0,collisionShape,collisionObject,colObjWorldTransform);
	btCollisionWorld::rayTestSingleInternal(rayFromTrans,rayToTrans,&colObWrap,resultCallback);
}

*/

void OpenHRP::CollisionDetector::rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result)
{
}
