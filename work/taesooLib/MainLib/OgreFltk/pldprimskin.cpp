#include "stdafx.h"
#include "pldprimskin.h"
#include "../BaseLib/utility/util.h"
#include "../BaseLib/motion/MotionLoader.h"
#include "AlzzaPostureIP.h"
#include "../BaseLib/motion/Motion.h"
#include "timesensor.h"
//#include "alzzactransform.h"
#include "renderer.h"
#include "MotionManager.h"
#include "../BaseLib/utility/TextFile.h"
#ifndef NO_OGRE
#include "LineStrip.h"
//#include "SurfelClouds.h"
#include "MovableText.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreItem.h>
/*
#include <OgreSkeleton.h>
#include <OgreBone.h>
#include <OgreSkeletonInstance.h>
*/
#include "Line3D.h"
#endif
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/motion/ConstraintMarking.h"
#include "../BaseLib/motion/IKSolver.h"
#include "../BaseLib/utility/QPerformanceTimer.h"

using namespace std;
#include "pldprimskin_impl.h"
PLDPrimSkin::PLDPrimSkin()
:AnimationObject()
{
	mChain=NULL;
	mDrawCallback=NULL;
	mBeforeDrawCallback=NULL;
	mType="PLDPrimSkin";
}
void PLDPrimSkin::setSamePose(BoneForwardKinematics  const& in)
{
	Posture pose;
	in.getPoseFromLocal(pose);
	SetPose(pose, in.getSkeleton());
}
int PLDPrimSkin::UpdateBone()
{
	return 0;
}
void PLDPrimSkin::setSamePose(ScaledBoneKinematics  const& in)
{
	(*mChain)=in;
	UpdateBone();
}

void PLDPrimSkin::scale(double x, double y, double z)
{
#ifndef NO_OGRE
	const Ogre::Vector3 & scale=this->m_pSceneNode->getScale();
	setScale(scale.x*x, scale.y*y, scale.z*z);
#endif
}
void PLDPrimSkin::setScale(double x, double y, double z)
{
#ifndef NO_OGRE
	this->m_pSceneNode->setScale(x,y,z);
#endif
}
void PLDPrimSkin_impl::setScale(double x, double y, double z)
{
#ifndef NO_OGRE
	this->m_pSceneNode->setScale(x,y,z);
	mScale=x;
#endif
}

void PLDPrimSkin_impl::setDrawConstraint(int con, float radius, RE::Color c)
{
#ifndef NO_OGRE
#endif
}

void PLDPrimSkin_impl::setDrawOrientation(int ijoint)
{
  #ifndef NO_OGRE
	PLDPrimSkinAxes t;
	t.sceneNode=RE::createEntity(RE::generateUniqueName(), "axes.mesh");
	t.ijoint=ijoint;
	mAxes.push_back(t);
#endif
}

PLDPrimSkin::~PLDPrimSkin()
{
	delete mChain;
}
PLDPrimSkin_impl::PLDPrimSkin_impl()
{
#ifndef NO_OGRE
	mScale=1.0;
#endif
}
PLDPrimSkin_impl::~PLDPrimSkin_impl()
{

#ifndef NO_OGRE
#endif
}

// parameter 
void PLDPrimSkin_impl::__drawConstraints(const MotionLoader& skeleton)
{
#ifndef NO_OGRE
	if(mAxes.size())
	{
		for(int i=0; i<mAxes.size(); i++)
		{
			Bone& bone=skeleton.getBoneByRotJointIndex(mAxes[i].ijoint);
			quater q;
			vector3 p;
			bone.getRotation(q);
			bone.getTranslation(p);
			RE::moveEntity(mAxes[i].sceneNode, q, mScale*p);
		}
	}

#endif
}
void PLDPrimSkin_impl::_drawConstraints(const Posture& posture, const MotionLoader& skeleton)
{
	if(mDrawCallback)
		mDrawCallback->draw(posture, skeleton);

#ifndef NO_OGRE
	if(mAxes.size() || mCircles.size())
	{
		((MotionLoader&)skeleton).setPose(posture);
	}

	__drawConstraints(skeleton);
	if(mCircles.size())
	{
		vector3 pos;

		MotionLoader& skel=((MotionLoader&)skeleton);

		const bool drawJoints=false;

		if(drawJoints)
		{
			int index[3];
			vector3 axis;


			int i=CONSTRAINT_LEFT_TOE;
			MotionUtil::setLimb(i, index[0], index[1], index[2], axis, skel);

			for(int ii=0; ii<3; ii++)
			{
				vector3 pos_i;
				skel.getBoneByRotJointIndex(index[ii]).getTranslation(pos_i);
				Ogre::SceneNode* pNode=RE::createEntity(sz1::format("constraintsDebug%d_%02d",i, ii), "sphere1010.mesh");
				RE::moveEntity(pNode, vector3(2,2,2), pos_i);
			}
		}

	}
#endif
}



PLDPrimSkel::PLDPrimSkel(MotionLoader* pBVHL)
:PLDPrimSkin_impl()
{
	m_pTimer=NULL;
	mSkel=pBVHL;
	mChain=new BoneForwardKinematics(pBVHL);
	mChain->init();
}

PLDPrimSkel::~PLDPrimSkel()
{
}

int PLDPrimSkel::UpdateBone()
{
	return 1;
}


int PLDPrimSkel::ToInitialPose()
{

	mChain->init();
	UpdateBone();
	return 1;
}

void PLDPrimSkin::setPose(int iframe)
{
	/*
	ASSERT(m_pTimer);
	AlzzaPostureIP* pip=(AlzzaPostureIP*)m_pTimer->GetFirstInterpolator();
	ASSERT(pip);
	setPose(pip->targetMotion(), iframe);
	*/
	//InterpolatorLinear* pip=((InterpolatorLinear*)m_pTimer->GetFirstInterpolator());
	//float cur_frame=pip->CalcCurrFrame(iframe);
	//m_pTimer->InitAnim(cur_frame);
	//pip->Update((float)iframe);
	m_pTimer->InitAnim((float)iframe);
	m_pTimer->StopAnim();
}

void PLDPrimSkin::setPose(const Motion& mot, int iframe)
{
	if(mBeforeDrawCallback)
		mBeforeDrawCallback->draw(mot, iframe);
	SetPose(mot.pose(iframe), mot.skeleton());
	if(mDrawCallback)
		mDrawCallback->draw(mot, iframe);


}


void PLDPrimSkin::detachAnim()
{
	if(m_pTimer)
		delete m_pTimer;
	m_pTimer=NULL;
}
void PLDPrimSkin::applyAnim(const MotionDOF& motion)
{
	if(motion.numFrames()==0)
	{
		Msg::error("motion has 0 frames");
		return;
	}
	if(m_pTimer)	delete m_pTimer;

	MotionDOFInterpolator* node;

	node=new MotionDOFInterpolator();

	//printf("%f\n", motion.mInfo.frameRate());
	node->Init(1.0/motion.mInfo.frameRate());
	// reference (??료 공유)
	node->SetKeyValue(&motion);
	node->SetTarget(this);
	m_pTimer=new TimeSensor();
	m_pTimer->FirstInit(node->GetCycleInterval(), true);
	m_pTimer->AttachInterpolator(node);
	m_pTimer->StartAnim();
}
void PLDPrimSkin::ApplyAnim(const Motion& mot)
{
	if(mot.numFrames()==0)
	{
		Msg::error("motion has 0 frames");
		return;
	}
	if(m_pTimer)	delete m_pTimer;

	AlzzaPostureIP* node;

	node=new AlzzaPostureIP();
	node->Init(mot.NumJoints(), mot.frameTime());
	// reference (자료 공유)
	node->SetKeyValue(&mot);
	node->SetTarget(this);
	m_pTimer=new TimeSensor();
	m_pTimer->FirstInit(node->GetCycleInterval(), true);
	m_pTimer->AttachInterpolator(node);
	m_pTimer->StartAnim();
}


void PLDPrimSkin::setPoseDOF(const vectorn& poseDOF, MotionDOFinfo const& info)
{
  SetPose(info.setDOF(poseDOF), info.skeleton());
}

void PLDPrimSkel::SetPose(const Posture & posture, const MotionLoader& skeleton)
{
	MotionLoader const* pSkeleton=&skeleton;
	//printf("setpose test %x\n", mBeforeDrawCallback);
	if(mBeforeDrawCallback)
		mBeforeDrawCallback->draw(posture, skeleton);
	mChain->setPose(posture);
	_drawConstraints(posture, skeleton);
	UpdateBone();
}
void PLDPrimSkel::setPose(const Posture & posture)
{
	mChain->setPose(posture);
	UpdateBone();
}
void PLDPrimSkel::setPoseDOF(const vectorn& poseDOF)
{
	mChain->setPoseDOF(poseDOF);
	UpdateBone();
}
void PLDPrimSkel::setPoseDOFignoringTranslationalJoints(const vectorn& poseDOF)
{
	mChain->setPoseDOFignoringTranslationalJoints(poseDOF);
	UpdateBone();
}
void PLDPrimSkel::getPose(Posture & posture)
{
	mChain->getPoseFromGlobal(posture);
}


PLDPrimBone::PLDPrimBone(MotionLoader* pBVHL, const OgreRenderer& renderer)
:PLDPrimSkel(pBVHL)
{
#ifndef NO_OGRE
  m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();

	mSceneNodes.resize(mSkel->numBone());
	mSceneNodes[0]=NULL;
	mSceneNodes[1]=NULL;
	for(int i=2; i<mSceneNodes.size(); i++)
	{
		mSceneNodes[i]=m_pSceneNode->createChildSceneNode();
		createLimb(renderer, mSceneNodes[i]);


	}
#endif
	UpdateBone();

	if(m_pTimer) m_pTimer->StartAnim();
}

void PLDPrimBone::setMaterial(const char* mat)
{
#ifndef NO_OGRE
  for(int i=2; i<mSceneNodes.size(); i++)
		((Ogre::Item*)mSceneNodes[i]->getAttachedObject(0))->setDatablockOrMaterialName(mat);
#endif
}

PLDPrimBone::~PLDPrimBone()
{
}

void PLDPrimBone::createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode)
{
#ifndef NO_OGRE
  Ogre::Item *limbEntity =renderer.viewport().mScene->createItem("sphere1010.mesh");
//	limbEntity ->setNormaliseNormals(true);
	pNode->attachObject(limbEntity);
#endif
}

void PLDPrimBone::setLimb(Ogre::SceneNode *pNode, vector3 const& p0, vector3 const& p1, float thick)
{
#ifndef NO_OGRE
  vector3 center = 0.5*(p0+p1);
	vector3 displacement = 0.5*(p1-p0);
	vector3 origin(0,1,0);

	float dist = (float)displacement.length();

	pNode->setScale(thick/2,dist,thick/2);

	quater t;
	t.axisToAxis(vector3(0,1,0), displacement);
	pNode->setOrientation(ToOgre(t));
	pNode->setPosition(ToOgre(center));
#endif
}



int PLDPrimBone::UpdateBone()
{
	PLDPrimSkel::UpdateBone();
#ifndef NO_OGRE
	for(int i=2; i<mSkel->numBone(); i++)
	{
		vector3 from, to;
		from=mChain->global(*mSkel->bone(i).parent()).translation;
		to=mChain->global(i).translation;
		setLimb(mSceneNodes[i], from, to, from.distance(to)/5.f);
	}
#endif
/*
	NodeStack & m_TreeStack=mSkel->m_TreeStack;
	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot;
	int index=-1;

	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			index++;
			// do something for src
			// src의 index는 현재 index이다.
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
*/
	return 1;
}


#ifndef NO_OGRE
inline void setPosition(Ogre::Node* pBone, vector3 const& p)
{
	if(p.x==p.x )
		pBone->setPosition(ToOgre(p));
}
inline void setOrientation(Ogre::Node* pBone, quater const& q)
{
	if(q.x==q.x )
		pBone->setOrientation(ToOgre(q));
}

static void setTransform(Ogre::Node* pBone, transf const& tf)
{
	setOrientation(pBone, tf.rotation);
	setPosition(pBone, tf.translation);
}
#endif













PLDPrimPoint::PLDPrimPoint(MotionLoader* pBVHL, const OgreRenderer& renderer)
:PLDPrimSkel(pBVHL)
{
#ifndef NO_OGRE
  m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();

	mSceneNodes.resize(mSkel->numBone()*2);
	mSceneNodes[0]=NULL;
	mSceneNodes[1]=NULL;
	mSceneNodes[0+mSkel->numBone()]=NULL;
	mSceneNodes[1+mSkel->numBone()]=NULL;
	for(int i=2; i<mSkel->numBone(); i++)
	{
		mSceneNodes[i]=m_pSceneNode->createChildSceneNode();
		createLimb(renderer, mSceneNodes[i]);

		mSceneNodes[i+mSkel->numBone()]=m_pSceneNode->createChildSceneNode();

		if(pBVHL->getBoneByTreeIndex(i).NameId)
		{
			/*
			Ogre::NameValuePairList params;
			params["name"]=RE::generateUniqueName();
			params["fontName"]="BlueHighway";
			params["caption"]=pBVHL->getBoneByTreeIndex(i).NameId ;
			params["fontSize"]=5;

			auto* text=RE::ogreSceneManager()->createMovableObject(Ogre::MovableTextFactory::FACTORY_TYPE_NAME, RE::_objectMemoryManager(), & params);
			mSceneNodes[i+mSkel->numBone()]->attachObject(text);
			*/
		}
	}
#endif
	UpdateBone();

	if(m_pTimer) m_pTimer->StartAnim();
}

void PLDPrimPoint::setMaterial(const char* mat)
{
#ifndef NO_OGRE
  for(int i=2; i<mSkel->numBone(); i++)
	{
		((Ogre::Item*)mSceneNodes[i]->getAttachedObject(0))->setDatablockOrMaterialName(mat);
	}
#endif
}

PLDPrimPoint::~PLDPrimPoint()
{
}

void PLDPrimPoint::createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode)
{

  #ifndef NO_OGRE
  Ogre::Item *limbEntity =renderer.viewport().mScene->createItem("axes.mesh");
	//limbEntity ->setMaterialName("green");
//	limbEntity ->setNormaliseNormals(true);
	pNode->attachObject(limbEntity);

#endif
}


void PLDPrimPoint::setScale(double x, double y, double z) 
{
	PLDPrimSkel::setScale(x,y,z);
	_dist.resize(0);
	UpdateBone();
}

int PLDPrimPoint::UpdateBone()
{
	PLDPrimSkel::UpdateBone();
#ifndef NO_OGRE

	const double DIST_THR=10.0;;
	if(_dist.size()==0)
	{
		std::vector<double>_dist2;

		_dist2.resize(mSkel->numBone());
		_dist.resize(mSkel->numBone());
		_dist2[1]=DIST_THR*10;
		_dist[1]=DIST_THR*10;
		// pass 1
		for(int i=2; i<mSkel->numBone(); i++)
		{
			vector3 from, to;
			quater q;
			from=mChain->global(*mSkel->bone(i).parent()).translation;
			to=mChain->global(i).translation;
			auto* pbone=mSkel->bone(i).parent();
			double dist=from.distance(to)*m_pSceneNode->getScale().x;
			_dist2[i]=dist;
		}
		// pass2 (downward path)
		for(int i=2; i<mSkel->numBone(); i++)
		{
			double dist=_dist2[i];
			auto* pbone=mSkel->bone(i).parent();
			dist=MIN(dist,_dist2[pbone->treeIndex()]);
			_dist[pbone->treeIndex()]=MAX(dist, 2.0);
		}
		// pass3 (upward path)
		for(int i=mSkel->numBone()-1; i>1; i--)
		{
			double dist=_dist[i];
			auto* pbone=mSkel->bone(i).parent();
			dist=MIN(dist,_dist[pbone->treeIndex()]);
			_dist[i]=dist;
		}
	}
	for(int i=2; i<mSkel->numBone(); i++)
	{
		vector3 from, to;
		quater q;
		from=mChain->global(*mSkel->bone(i).parent()).translation;
		to=mChain->global(i).translation;
		//q=mChain->global(i).rotation;
		auto* pbone=mSkel->bone(i).parent();
		q=mChain->global(*pbone).rotation;

		Ogre::SceneNode* pNode=mSceneNodes[i];
		Ogre::Vector3 p0=ToOgre((to+from)/2);
		m_real scale=1.0/m_pSceneNode->getScale().x;
		double dist=_dist[i];
		if (dist<DIST_THR) 
			scale*=dist/DIST_THR;
															
		pNode->setScale(scale,scale,scale);
		pNode->setOrientation(ToOgre(q));
		pNode->setPosition(p0);


		Ogre::SceneNode* pNode2=mSceneNodes[i+mSkel->numBone()];
		pNode2->setScale(scale*2.0,scale*2.0,scale*2.0);
		pNode2->resetOrientation();
		pNode2->setPosition(ToOgre(to));
	}
#endif
	return 1;
}
//

PLDPrimBox::PLDPrimBox(MotionLoader* pBVHL, const OgreRenderer& renderer)
:PLDPrimSkel(pBVHL)
{
#ifndef NO_OGRE
  m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();

	mSceneNodes.resize(mSkel->numBone());
	mSceneNodes[0]=NULL;
	mSceneNodes[1]=NULL;
	for(int i=2; i<mSceneNodes.size(); i++)
	{
		mSceneNodes[i]=m_pSceneNode->createChildSceneNode();
		createLimb(renderer, mSceneNodes[i], pBVHL->getBoneByTreeIndex(i).GetNameId());
	}
#endif
	UpdateBone();

	if(m_pTimer) m_pTimer->StartAnim();
}

void PLDPrimBox::setMaterial(const char* mat)
{
#ifndef NO_OGRE
  for(int i=2; i<mSceneNodes.size(); i++)
		((Ogre::Item*)mSceneNodes[i]->getAttachedObject(0))->setDatablockOrMaterialName(mat);
#endif
}

PLDPrimBox::~PLDPrimBox()
{
}

void PLDPrimBox::createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode, const char* nameId)
{
#ifndef NO_OGRE
  Ogre::Item *limbEntity =renderer.viewport().mScene->createItem( "cube.mesh");
	limbEntity ->setDatablockOrMaterialName("green");
//	limbEntity ->setNormaliseNormals(true);
	pNode->attachObject(limbEntity);
#endif
}

int PLDPrimBox::UpdateBone()
{
	PLDPrimSkel::UpdateBone();
#ifndef NO_OGRE
	for(int i=2; i<mSkel->numBone(); i++)
	{
		vector3 v1, v2;
		transf const& parent=mChain->global(*mSkel->bone(i).parent());
		transf const& child=mChain->global(i);
		v1=child.translation;
		v2=parent.translation;

		vector3 offset;
		mSkel->bone(i).getOffset(offset);

		quater q, qglobal;
		qglobal=parent.rotation;
		q.axisToAxis(vector3(0,1,0), offset);

		double thick=3/m_pSceneNode->getScale().x;
		mSceneNodes[i]->setScale(0.01*thick,0.01*offset.length(), 0.01*thick);
		mSceneNodes[i]->setOrientation(ToOgre(qglobal*q));
		mSceneNodes[i]->setPosition(ToOgre((v1+v2)/2));
	}
#endif
	return 1;
}
////
void PLDPrimCyl::setThickness(float _thick)
{
	thick=_thick;
	UpdateBone();
}

PLDPrimCyl::PLDPrimCyl(MotionLoader* pBVHL, const OgreRenderer& renderer, bool lowPoly)
:PLDPrimSkel(pBVHL)
{
	thick=3;
#ifndef NO_OGRE
	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();

	mSceneNodes.resize(mSkel->numBone());
	mSceneNodes[0]=NULL;
	mSceneNodes[1]=NULL;
	for(int i=2; i<mSceneNodes.size(); i++)
	{
		mSceneNodes[i]=m_pSceneNode->createChildSceneNode();
		Ogre::Item *limbEntity ;
		if(lowPoly)
			//limbEntity =renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "cylinder_low.mesh");
			limbEntity =renderer.viewport().mScene->createItem( "cylinder_with_top.mesh");
		else
			limbEntity =renderer.viewport().mScene->createItem( "cylinder.mesh");
		limbEntity ->setDatablockOrMaterialName("green");
		//limbEntity ->setNormaliseNormals(true);
		mSceneNodes[i]->attachObject(limbEntity);
	}

	mSceneNodesPoint.resize(mSkel->numBone());
	mSceneNodesPoint[0]=NULL;

	for(int i=1; i<mSceneNodesPoint.size(); i++)
	{
		mSceneNodesPoint[i]=m_pSceneNode->createChildSceneNode();

		Ogre::Item *limbEntity ;
		if(lowPoly)
			limbEntity 	=renderer.viewport().mScene->createItem( "sphere1010_low.mesh");
		else
			limbEntity 	=renderer.viewport().mScene->createItem( "sphere1010.mesh");

		limbEntity ->setDatablockOrMaterialName("green");
		//limbEntity ->setNormaliseNormals(true);
		mSceneNodesPoint[i]->attachObject(limbEntity);
	}
#endif
	UpdateBone();

	if(m_pTimer) m_pTimer->StartAnim();
}

void PLDPrimCyl::setMaterial(const char* mat)
{
#ifndef NO_OGRE
  for(int i=2; i<mSceneNodes.size(); i++)
		((Ogre::Item*)mSceneNodes[i]->getAttachedObject(0))->setDatablockOrMaterialName(mat);

	for(int i=1; i<mSceneNodesPoint.size(); i++)
		((Ogre::Item*)mSceneNodesPoint[i]->getAttachedObject(0))->setDatablockOrMaterialName(mat);
#endif
}

PLDPrimCyl::~PLDPrimCyl()
{
}

void PLDPrimCyl::createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode)
{
#ifndef NO_OGRE
  ASSERT(0);
	Ogre::Item *limbEntity =renderer.viewport().mScene->createItem( "cylinder.mesh");
	limbEntity ->setDatablockOrMaterialName("green");
	//limbEntity ->setNormaliseNormals(true);
	pNode->attachObject(limbEntity);
#endif
}

void PLDPrimCyl::setLimb(Ogre::SceneNode *pNode, vector3 const& p0, vector3 const& p1, float thick)
{
#ifndef NO_OGRE
  vector3 center = 0.5*(p0+p1);
	vector3 displacement = 0.5*(p1-p0);
	vector3 origin(0,1,0);

	float dist = (float)displacement.length();

	pNode->setScale(thick/2,thick/2,thick/2);

	quater t;
	t.axisToAxis(vector3(0,1,0), displacement);
	pNode->setOrientation(ToOgre(t));
	pNode->setPosition(ToOgre(center));
#endif
}

#define MBEGIN_TIMER(x) BEGIN_TIMER(x)
#define MEND_TIMER(x) END_TIMER(x)
//#define MBEGIN_TIMER(x)
//#define MEND_TIMER(x)


int PLDPrimCyl::UpdateBone()
{

	PLDPrimSkel::UpdateBone();

#ifndef NO_OGRE
	vector3 from, to;

	to=mChain->global(1).translation;
	//tjtd::cout << "to"<<to<<std::endl;

	m_real _thick=thick/m_pSceneNode->getScale().x;
	//m_real _thick=thick;

	if(mSceneNodesPoint.size())
	{
		mSceneNodesPoint[1]->resetOrientation(); m_real f=_thick/2.0;
		mSceneNodesPoint[1]->setScale(f,f,f);
		mSceneNodesPoint[1]->setPosition(ToOgre(to));
	}

	for(int i=2; i<mSkel->numBone(); i++)
	{
		vector3 from, to;
		from=mChain->global(*mSkel->bone(i).parent()).translation;
		to=mChain->global(i).translation;
		setLimb(mSceneNodes[i], from, to, _thick  /*from.distance(to)/5.f*/);

		if(mSceneNodesPoint.size())
		{
			mSceneNodesPoint[i]->resetOrientation();m_real f=_thick/2.0;
			mSceneNodesPoint[i]->setScale(f,f,f);
			mSceneNodesPoint[i]->setPosition(ToOgre(to));
		}
	}
#endif
	return 1;
}

static vector3 greyblue(0.21*0.2, 0.36*0.2, 0.43*0.2);

PLDPrimThickLine::PLDPrimThickLine(MotionLoader* pBVHL, const OgreRenderer& renderer)
:PLDPrimSkel(pBVHL)
{
#ifndef NO_OGRE
	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();

	_thickness=3; // 3cm
	//_materialName="solidblue";
	_materialName="solidwhite";
	_lines=new ColorBillboardLineList (RE::generateUniqueID(), mSkel->numBone()-2, _thickness);
	for (int i=2; i<mSkel->numBone() ; i++)
	{
		vector3 start=mSkel->bone(i).getFrame().translation;
		vector3 end=mSkel->bone(i).parent()->getFrame().translation;
		_lines->line(i-2, start, end, greyblue );  // grey blue
	}

//	_lines->setDatablockOrMaterialName(_materialName, Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
	m_pSceneNode->attachObject(_lines);
#endif
	UpdateBone();

	if(m_pTimer) m_pTimer->StartAnim();
}

PLDPrimThickLine::~PLDPrimThickLine()
{
#ifndef NO_OGRE
#endif
}
int PLDPrimThickLine::FrameMove(float fElapsedTime)
{
#ifndef NO_OGRE
	UpdateBone();
#endif
	return PLDPrimSkel::FrameMove(fElapsedTime);
}

void PLDPrimThickLine::SetVisible(bool bVisible)
{
  #ifndef NO_OGRE
	m_pSceneNode->setVisible(bVisible);
#endif
}
void PLDPrimThickLine::setMaterial(const char* mat)
{
	_materialName=mat;
#ifndef NO_OGRE
	_lines->setDatablockOrMaterialName(_materialName, Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
#endif
}

void PLDPrimThickLine::setScale(double x, double y, double z)
{
#ifndef NO_OGRE
	this->m_pSceneNode->setScale(x,y,z);
	UpdateBone();
#endif
}
int PLDPrimThickLine::UpdateBone()
{
	PLDPrimSkel::UpdateBone();
#ifndef NO_OGRE
	double tu1=0.0;
	double tu2=1.0;
	double thickness=_thickness/m_pSceneNode->getScale().x;
	for(int i=2; i<mSkel->numBone(); i++)
	{
		vector3 from, to;
		from=mChain->global(*mSkel->bone(i).parent()).translation;
		to=mChain->global(i).translation;

		vector3 dir=to-from;
		dir.normalize();
		dir*=(thickness*0.25);
		from-=dir;
		to+=dir;

		auto& g=greyblue;

		_lines->updateChainElement(i-2,0, Ogre::v1::BillboardChain::Element(
					ToOgre(from), thickness, tu1, Ogre::ColourValue(g.x,g.y,g.z,1), Ogre::Quaternion(1,0,0,0)));
		_lines->updateChainElement(i-2,1, Ogre::v1::BillboardChain::Element(
					ToOgre(to), thickness, tu2, Ogre::ColourValue(g.x,g.y,g.z,1), Ogre::Quaternion(1,0,0,0)));


	}
#endif
	return 1;
}
