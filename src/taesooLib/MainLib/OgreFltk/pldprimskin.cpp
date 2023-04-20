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
#include "Circle.h"
//#include "SurfelClouds.h"
#include "MovableText.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreSkeleton.h>
#include <OgreBone.h>
#include <OgreSkeletonInstance.h>
#include "Line3D.h"
#endif
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/motion/ConstraintMarking.h"
#include "../BaseLib/motion/IKSolver.h"
#include "../BaseLib/utility/QPerformanceTimer.h"

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
	if(mCircles.size()==0)
	{
		mCircles.resize(32);
		for(int i=0; i<32; i++) mCircles[i]=NULL;
	}
	PLDPrimSkinCircle* pCircle;
	if(mCircles[con])
		pCircle=mCircles[con];
	else
	{
		pCircle=new PLDPrimSkinCircle;
		pCircle->pCircle=RE::createCircle();
		mCircles[con]=pCircle;
	}

	pCircle->c=c;
	pCircle->radius=radius;
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
  if(mCircles.size())
	{
		for(int i=0; i<32; i++)
		{
			if(mCircles[i])
			{
				PLDPrimSkinCircle* pCircle=mCircles[i];
				delete pCircle->pCircle;
				delete pCircle;
			}
		}
	}
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

		for(int i=0; i<32; i++)
			if(mCircles[i])
			{
				PLDPrimSkinCircle* pCircle=mCircles[i];
				if(i==WHICH_FOOT_IS_SUPPORTED )
				{
					pCircle->pCircle->SetColor(pCircle->c);
					if(posture.constraint[i])
						skel.getBoneByVoca(MotionLoader::LEFTANKLE).getTranslation(pos);
					else
						skel.getBoneByVoca(MotionLoader::RIGHTANKLE).getTranslation(pos);

					pos+=m_vTrans;
					pCircle->pCircle->SetCircle(pos, pCircle->radius, 10);
					pCircle->pCircle->SetVisible(true);
				}
				else if(posture.constraint[i])
				{

					pCircle->pCircle->SetColor(pCircle->c);
					//MotionUtil::ConstraintMarking::decodeCon(i, (Posture&)posture, (MotionLoader&)skeleton, pos);

					dep_GetBoneFromCon(skel, i).getTranslation(pos);
					if(pos.y<4) pos.y=4;
					pos+=m_vTrans;

					pCircle->pCircle->SetCircle(pos, pCircle->radius, 10);
					pCircle->pCircle->SetVisible(true);
				}
				else
				{
					pCircle->pCircle->SetVisible(false);
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
		((Ogre::Entity*)mSceneNodes[i]->getAttachedObject(0))->setMaterialName(mat);
#endif
}

PLDPrimBone::~PLDPrimBone()
{
}

void PLDPrimBone::createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode)
{
#ifndef NO_OGRE
  Ogre::Entity *limbEntity =renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "sphere1010.mesh");
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

	pNode->resetToInitialState();
	pNode->scale(thick/2,dist,thick/2);

	quater t;
	t.axisToAxis(vector3(0,1,0), displacement);
	pNode->rotate(ToOgre(t));
	pNode->translate(ToOgre(center));
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

PLDPrimLine::PLDPrimLine(MotionLoader* pBVHL, const OgreRenderer& renderer)
:PLDPrimSkel(pBVHL)
{
#ifndef NO_OGRE
  m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();

	mLines.resize(mSkel->numBone());
	mLines[0]=NULL;
	for(int i=1; i<mLines.size(); i++)
	{
		mLines[i]=RE::createThinLine();
	}
#endif
	UpdateBone();

	if(m_pTimer) m_pTimer->StartAnim();
}

PLDPrimLine::~PLDPrimLine()
{
#ifndef NO_OGRE
  for(int i=1; i<mLines.size(); i++)
	{
		delete mLines[i];
	}
#endif
}

void PLDPrimLine::setColor(RE::Color c)
{
#ifndef NO_OGRE
  for(int i=1; i<mLines.size(); i++)
	{
		mLines[i]->SetColor(c);
	}
#endif
}
void PLDPrimLine::SetVisible(bool bVisible)
{
  #ifndef NO_OGRE
	for(int i=1; i<mLines.size(); i++)
	{
		mLines[i]->SetVisible(bVisible);
	}
#endif
}

int PLDPrimLine::UpdateBone()
{
	PLDPrimSkel::UpdateBone();
#ifndef NO_OGRE
	for(int i=2; i<mSkel->numBone(); i++)
	{
		vector3 from, to;
		from=mChain->global(*mSkel->bone(i).parent()).translation;
		to=mChain->global(i).translation;
		mLines[i]->begin(2);
		mLines[i]->row(0).assign(from);
		mLines[i]->row(1).assign(to);
		mLines[i]->end();
	}
#endif
	return 1;
}

#ifndef NO_OGRE
static Ogre::Bone* getOgreBoneByName(Ogre::SkeletonInstance* pskel, const char* name2)
{
	Ogre::Bone* pBone=NULL;
	try
	{
		//pBone=pskel->getBone(Ogre::String(convTable[i]));--> 동작 안함. ex) HIPS로 찾으면 Hips가 안찾아짐.

		Ogre::Skeleton::BoneIterator iter = pskel->getBoneIterator();
		while(iter.hasMoreElements())
		{
			pBone=iter.getNext();
			if(fast_strcmp_upper(pBone->getName().c_str(), name2)==0)
				break;
		}
	}
	catch( Ogre::Exception& e )
	{
		Msg::error(e.getFullDescription().c_str());
	}
	return pBone;
}
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

static void setTransform(Ogre::Bone* pBone, transf const& tf)
{
	setOrientation(pBone, tf.rotation);
	setPosition(pBone, tf.translation);
}
#endif
PLDPrimOgreSkin::PLDPrimOgreSkin(MotionLoader* pLoader, Ogre::Entity* pEntity, const char* convfilename, const OgreRenderer& renderer, bool bCurrPoseAsBindPose, double scale)
:PLDPrimSkin_impl()
{
	m_pTimer=NULL;
#ifndef NO_OGRE
	m_pEntity=pEntity;
	mMotionScale=scale;

	// Add entity to the scene node
	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0,0,0));
	m_pSceneNode->attachObject(pEntity);
	//printf("movabletype:%s\n", pEntity->getMovableType().c_str());

	ASSERT(pEntity->getSkeleton());
	pEntity->getSkeleton()->_updateTransforms();

	TStrings convTable;
	convTable.resize(pLoader->numRotJoint());
	TStrings convTable2;
	convTable2.resize(pLoader->numTransJoint());

	if(convfilename)
	{
		CTextFile file;

		Msg::verify(file.OpenReadFile(convfilename), "file open error %s", convfilename);

		char *token;
		while(token=file.GetToken())
		{
			char* jointName=token;

			int iindex=pLoader->GetIndex(jointName);

			if(iindex==-1)
			{
				printf("warning %s not exist\n", jointName);
			}
			else
			{
				int ijoint=pLoader->getRotJointIndexByTreeIndex(iindex);
				ASSERT(ijoint<pLoader->numRotJoint());

				if(ijoint!=-1) //jinuk
				{
					convTable[ijoint]=file.GetToken();					
				}
			}
		}

		file.CloseFile();
		m_bMapPositionsToo=false;
	}
	else
	{
		m_bMapPositionsToo=true;
		// use joints name as ogre::bone name.
		for(int ijoint=0; ijoint<pLoader->numRotJoint(); ijoint++)
		{
			convTable[ijoint]=pLoader->GetName(pLoader->getTreeIndexByRotJointIndex(ijoint));
		}
		for(int ijoint=0; ijoint<pLoader->numTransJoint(); ijoint++)
		{
			convTable2[ijoint]=pLoader->GetName(pLoader->getTreeIndexByTransJointIndex(ijoint));

			//printf("%s\n", convTable2[ijoint].ptr());
		}
	}

	m_aTargetIndex.setSize(pLoader->numRotJoint());
	m_aRotOrigComb.setSize(pLoader->numRotJoint());
	m_aInvRotOrigComb.setSize(pLoader->numRotJoint());
	m_aLocalRotOrig.setSize(pLoader->numRotJoint());
	m_aBindPose.setSize(pLoader->numRotJoint()+1);
	m_aTargetIndexByTransJointIndex.setSize(pLoader->numTransJoint());
	m_aTargetIndexByTransJointIndex.setAllValue(-1);
	for(int i=0; i<pLoader->numRotJoint(); i++)
	{
		if(convTable[i].length())
		{
			Ogre::SkeletonInstance* pskel=m_pEntity->getSkeleton();
			Ogre::Bone* pBone=getOgreBoneByName(pskel, convTable[i]);
			m_aTargetIndex[i]=(int)(pBone->getHandle());
			Msg::verify(pBone!=NULL && pBone==pskel->getBone((unsigned short)(m_aTargetIndex[i])), "There is no Bone %s", convTable[i].ptr());
			pBone->setManuallyControlled(true);
			m_aRotOrigComb[i]=ToBase(pBone->_getDerivedOrientation());
			m_aInvRotOrigComb[i].inverse(ToBase(pBone->_getDerivedOrientation()));
			m_aLocalRotOrig[i]=ToBase(pBone->getOrientation());

			if(bCurrPoseAsBindPose)
				pLoader->getBoneByRotJointIndex(i).getRotation(m_aBindPose[i]);
			else
				m_aBindPose[i].identity();

		}
		else
			m_aTargetIndex[i]=-1;
	}

	if(m_bMapPositionsToo) {
		for(int i=0; i<pLoader->numTransJoint(); i++)
		{
			if(convTable2[i].length())
			{
				Ogre::SkeletonInstance* pskel=m_pEntity->getSkeleton();
				Ogre::Bone* pBone=getOgreBoneByName(pskel, convTable2[i]);
				m_aTargetIndexByTransJointIndex[i]=(int)(pBone->getHandle());
				Msg::verify(pBone!=NULL && pBone==pskel->getBone((unsigned short)(m_aTargetIndexByTransJointIndex[i])), "There is no Bone %s", convTable2[i].ptr());
				pBone->setManuallyControlled(true);
			}
			else
				m_aTargetIndexByTransJointIndex[i]=-1;
		}
	}

	if(bCurrPoseAsBindPose)
	{
		// bind root position저장.
		vector3 trans;
		pLoader->getBoneByRotJointIndex(0).getTranslation(trans);
		m_aBindPose[pLoader->numRotJoint()].setValue(0, trans.x, trans.y, trans.z);
	}
	else
		m_aBindPose[pLoader->numRotJoint()].setValue(0, 0,0,0);


	parentIdx.setSize(pLoader->numRotJoint());
	parentIdx.setAllValue(-1);

	for(int i=1; i<pLoader->numRotJoint(); i++)
	{
		// missing joint에 대한 고려를 해주어야 한다. 따라서 실제 사용된 조인트 만으로 hierarchy를 다시 만든다.
		int j=i;
		do
		{
			j=dep_GetParentJoint(*pLoader,j);
			if(j==-1) break;
		}
		while(m_aTargetIndex[j]==-1);

		parentIdx[i]=j;
	}
#endif
}

int PLDPrimOgreSkin::getOgreBoneIndex(int ibone) const { return m_aTargetIndex[ibone];}
int PLDPrimOgreSkin::parentRotJointIndex(int iRotJoint) const { return parentIdx[iRotJoint];}
PLDPrimOgreSkin::PLDPrimOgreSkin(const PLDPrimOgreSkin& other, const OgreRenderer& renderer, const char* nameid)
:PLDPrimSkin_impl()
{
#ifndef NO_OGRE
  m_pTimer=NULL;
	m_pEntity=other.m_pEntity->clone(nameid);

	// Add entity to the scene node
	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0,0,0));
	m_pSceneNode->attachObject(m_pEntity);


	m_pEntity->getSkeleton()->_updateTransforms();

	m_aTargetIndex=other.m_aTargetIndex;

	int numJoint=other.m_aRotOrigComb.size();
	m_aRotOrigComb.setSize(numJoint);
	m_aInvRotOrigComb.setSize(numJoint);
	m_aLocalRotOrig.setSize(numJoint);
	m_aTargetIndex=other.m_aTargetIndex;
	parentIdx=other.parentIdx;
	for(int i=0; i<numJoint; i++)
	{
		m_aRotOrigComb[i]=other.m_aRotOrigComb[i];
		m_aInvRotOrigComb[i]=other.m_aInvRotOrigComb[i];
		m_aLocalRotOrig[i]=other.m_aLocalRotOrig[i];
	}

	m_aBindPose.setSize(other.m_aBindPose.size());
	for(int i=0; i<m_aBindPose.size(); i++)
		m_aBindPose[i]=other.m_aBindPose[i];

	for(int i=0; i<numJoint; i++)
	{
		if(m_aTargetIndex[i]!=-1)
		{
			Ogre::SkeletonInstance* pskel=m_pEntity->getSkeleton();
			Ogre::Bone* pBone;

			try
			{
				pBone=pskel->getBone(m_aTargetIndex[i]);
			}
			catch( Ogre::Exception& e )
			{
				Msg::error(e.getFullDescription().c_str());
			}
			pBone->setManuallyControlled(true);
		}
	}
#endif
}

PLDPrimOgreSkin::~PLDPrimOgreSkin()
{
	//printf("dtorOgreSkin\n");

}



void PLDPrimOgreSkin::SetTranslation(float x, float y, float z)
{
#ifndef NO_OGRE
  vector3 pos;
	pos.x=m_pSceneNode->getPosition().x;
	pos.y=m_pSceneNode->getPosition().y;
	pos.z=m_pSceneNode->getPosition().z;

	pos-=m_vTrans;
	m_vTrans.setValue(x,y,z);

	pos+=m_vTrans;
	setPosition(m_pSceneNode, pos);
#endif
}


void PLDPrimOgreSkin::SetPose(const Posture & posture, const MotionLoader& skeleton)
{
#ifndef NO_OGRE
  quaterN aRotations(posture.numRotJoint());

	((MotionLoader&)skeleton).setPose(posture);

	for(int i=0; i<posture.numRotJoint(); i++)
	{
		skeleton.getBoneByRotJointIndex(i).getRotation(aRotations[i]);
	}

	Ogre::SkeletonInstance* skel=m_pEntity->getSkeleton();


	// root or root2 should not be missing. (for positioning)

	int curbone=0;

	const Ogre::Vector3 & scale=this->m_pSceneNode->getScale();
	double motionScale=scale.x*mMotionScale;
	if(m_aTargetIndex[0]!=-1)
	{
		Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndex[0]));



		//= L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2 ---------------2

		setOrientation(pBone, (
			m_aLocalRotOrig[0]* m_aInvRotOrigComb[0] * posture.m_aRotations[0]*m_aBindPose[0].inverse()* m_aRotOrigComb[0] ));
	//	skel->getRootBone()->setPosition(0,0,0);
	//	pBone->setPosition(0,0,0);
		quater& qbindTrans=m_aBindPose[m_aBindPose.size()-1];
		vector3 bindTrans(qbindTrans.x, qbindTrans.y, qbindTrans.z);
		setPosition(m_pSceneNode,(m_vTrans+posture.m_aTranslations[0]-bindTrans)*motionScale);

		curbone++;
	}
	else
	{
		printf("?\n");
		assert(m_aTargetIndex[1]!=-1 && posture.numTransJoint()>1);

		Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndex[1]));

		setOrientation(pBone, (
			m_aLocalRotOrig[1]* m_aInvRotOrigComb[1] * aRotations[1]*m_aBindPose[1].inverse()* m_aRotOrigComb[1] ));
	//	skel->getRootBone()->setPosition(0,0,0);
	//	pBone->setPosition(0,0,0);
		quater& qbindTrans=m_aBindPose[m_aBindPose.size()-1];
		vector3 bindTrans(qbindTrans.x, qbindTrans.y, qbindTrans.z);
		vector3 v;
		skeleton.getBoneByRotJointIndex(1).getTranslation(v);
		setPosition(m_pSceneNode, (m_vTrans+v-bindTrans)*motionScale);

		curbone=2;
	}

	//for(int i=curbone; i<posture.numRotJoint(); i++)
	for(int ii=curbone+1; ii<skeleton.numBone(); ii++)
	{
		//printf("%d\n", ii);
		int i=skeleton.bone(ii).rotJointIndex();

		if(i!=-1 && m_aTargetIndex[i]!=-1)
		{
			Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndex[i]));

			/////////////////////////////////////////////////////////////////////////////////////////
			// Bind pose는 임의의 포즈가 될수 있다. (MaxExporter특성상, 주로 frame0를 사용한다.)

			// 현재 오우거 노드가 부모로부터 3번째 노드라고 하면.
			//  노드의 global orientation CN2=LN0*LN1*LN2
			//  C2가 모델의 binding pose에서의 global orientation이라 하자.

			// 목표:
			// binding pose를 SetPose함수에 입력한 경우, CN2가 C2가 되도록 해야한다.
			// C2BB 가 동작 데이타 바인딩 포즈의 combined mat이라 하자.
			// CN2 = C2B * C2BB.inv * C2 가 되면 된다.
			// (즉 C2BB와 C2B(SetPose함수의 입력자세)가 동일한 경우, CN2==C2)

			// CN2=CN1*LN2 이므로
			// -> LN2=CN1.inv * CN2
			//       =CN1.inv * C2B * C2BB.inv * C2
			//       = ( C1B * C1BB.inv* C1).inv * C2B * C2BB.inv * C2 ---------------1
			//		 = C1.inv* C1BB * C1B.inv * C2B * C2BB.inv * C2
			//		 = L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2 ---------------2

			// 검산
			//  CN3= LN0 * lN1* LN2
			//  = (L0B * L0BB.inv * L0)*(L0.inv * L0BB * L0B.inv * C1B * C1BB.inv * C1)*(C1.inv* C1BB * C1B.inv * C2B * C2BB.inv * C2)
			//  =  C2B * C2BB.inv * C2

				setOrientation(pBone, (
					m_aLocalRotOrig[i]*m_aInvRotOrigComb[i]			// C1.inv
					* m_aBindPose[parentIdx[i]]				// C1BB
					* aRotations[parentIdx[i]].inverse() * aRotations[i]	// C1B.inv * C2B
					* m_aBindPose[i].inverse()				// C2BB.inv
					* m_aRotOrigComb[i]));					// C2

			//printf("%s\n", m_aBindPose[i].output().ptr()); // zero
			//printf("l:%s\n", m_aLocalRotOrig[i].output().ptr());// non-zero
			//printf("c:%s\n", m_aRotOrigComb[i].output().ptr());// non-zero

			// 고려사항 1. 양쪽의 link길이가 다른경우. 예를 들어 Ogre쪽은 link가 4개(0,1,@,2)이지만, 포즈쪽은 link가 3개(0,1,2)라 하자
			// 위식 1 대로 계산하면
			// LN2 = L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2;
			// LN@ = L@;
			// LN1 = L1 * C1.inv * C0BB * C0B.inv * C1B * C1BB.inv * C1
			// LN0 = C0B * C0BB.inv * L0;

			// 다곱하면.
			// CN2= C2B * C2BB.inv * C2; -> 문제 없음

			// 고려사항 2. 링크 구조가 다른경우. - 확실히 문제 생기는듯함. 수정요.
		}
		int j=skeleton.bone(ii).transJointIndex();
		if(j!=-1 && m_aTargetIndexByTransJointIndex[j]!=-1)
		{
			Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndexByTransJointIndex[j]));

			//posture.m_aTranslations[j] = if error value
			vector3 trans= posture.m_aTranslations[j] *motionScale;
			trans.rotate(quater(TO_RADIAN(90), vector3(0,1,0)));
			//if(j==1)
			//printf("%f %f %lf\n", trans.x,trans.y,trans.z);
			setPosition(pBone, -trans*2.4);
			//pBone->setPosition(ToOgre(vector3(trans.z,trans.x,trans.y)));
			/*
			if(j==1)
			{
			printf("%s\n", trans.output().ptr());
			printf("%f %f %lf\n", posture.m_aTranslations[j].x,posture.m_aTranslations[j].y,posture.m_aTranslations[j].z);
			printf("hihi%s %s %d\n",skeleton.bone(ii).NameId,pBone->getName().c_str(), j);
			}
			*/
		}
	}


#ifdef _DEBUG_MAT
	skel->_updateTransforms();

	for(int i=1; i<posture.numRotJoint(); i++)
	{
		// 검산. (본 구조가 다르면 틀릴때가 있는거 같음. 수정할 필요)
		if(m_aTargetIndex[i]!=-1)
		{
			Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndex[i]));
			quater CNi=ToBase(pBone->_getDerivedOrientation());
			quater Ci=m_aRotOrigComb[i];
			//CN2 = C2B * C2BB.inv * C2
			quater CNi2= aRotations[i]* m_aBindPose[i].inverse() * m_aRotOrigComb[i];

			RE::output(TString(sz0::format("c%d", i)), "%s = %s =0 %s", CNi.output().ptr(),CNi2.output().ptr(),Ci.output().ptr());
			//

		}
	}
#endif

	_drawConstraints(posture, skeleton);
#endif
}


void PLDPrimOgreSkin::ApplyAnim(const Motion& mot)
{


	AlzzaPostureIP* node;

	node=new AlzzaPostureIP();

	if(m_pTimer) delete m_pTimer;

	// target index가 reference가 아님
	node->Init(mot.NumJoints(),mot.frameTime());
	// reference (자료 공유)
	node->SetKeyValue(&mot);
	node->SetTarget(this);
	m_pTimer=new TimeSensor();
	m_pTimer->FirstInit(node->GetCycleInterval(), true);
	m_pTimer->AttachInterpolator(node);
	m_pTimer->StartAnim();

}



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
			Ogre::MovableText* text=new Ogre::MovableText(RE::generateUniqueName().ptr(), pBVHL->getBoneByTreeIndex(i).NameId, "BlueHighway", 5, Ogre::ColourValue::Black);
			text->showOnTop(true);
			mSceneNodes[i+mSkel->numBone()]->attachObject(text);
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
		((Ogre::Entity*)mSceneNodes[i]->getAttachedObject(0))->setMaterialName(mat);
	}
#endif
}

PLDPrimPoint::~PLDPrimPoint()
{
}

void PLDPrimPoint::createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode)
{

  #ifndef NO_OGRE
  Ogre::Entity *limbEntity =renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "axes.mesh");
	//limbEntity ->setMaterialName("green");
//	limbEntity ->setNormaliseNormals(true);
	pNode->attachObject(limbEntity);

#endif
}



int PLDPrimPoint::UpdateBone()
{
	PLDPrimSkel::UpdateBone();
#ifndef NO_OGRE
	for(int i=2; i<mSkel->numBone(); i++)
	{
		vector3 from, to;
		quater q;
		from=mChain->global(*mSkel->bone(i).parent()).translation;
		to=mChain->global(i).translation;
		//q=mChain->global(i).rotation;
		q=mChain->global(*mSkel->bone(i).parent()).rotation;

		Ogre::SceneNode* pNode=mSceneNodes[i];
		Ogre::Vector3 p0=ToOgre((to+from)/2);
		m_real scale=0.5/m_pSceneNode->getScale().x;
		pNode->resetToInitialState();
		pNode->scale(scale,scale,scale);
		pNode->rotate(ToOgre(q));
		pNode->translate(p0);


		Ogre::SceneNode* pNode2=mSceneNodes[i+mSkel->numBone()];
		pNode2->resetToInitialState();
		pNode2->scale(scale,scale,scale);
		pNode2->translate(ToOgre(to));
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
		((Ogre::Entity*)mSceneNodes[i]->getAttachedObject(0))->setMaterialName(mat);
#endif
}

PLDPrimBox::~PLDPrimBox()
{
}

void PLDPrimBox::createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode, const char* nameId)
{
#ifndef NO_OGRE
  Ogre::Entity *limbEntity =renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "cube.mesh");
	limbEntity ->setMaterialName("green");
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
		mSceneNodes[i]->resetToInitialState();
		mSceneNodes[i]->scale(0.01*thick,0.01*offset.length(), 0.01*thick);
		mSceneNodes[i]->rotate(ToOgre(qglobal*q));
		mSceneNodes[i]->translate(ToOgre((v1+v2)/2));
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
		Ogre::Entity *limbEntity ;
		if(lowPoly)
			//limbEntity =renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "cylinder_low.mesh");
			limbEntity =renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "cylinder_with_top.mesh");
		else
			limbEntity =renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "cylinder.mesh");
		limbEntity ->setMaterialName("green");
		//limbEntity ->setNormaliseNormals(true);
		mSceneNodes[i]->attachObject(limbEntity);
	}

	mSceneNodesPoint.resize(mSkel->numBone());
	mSceneNodesPoint[0]=NULL;

	for(int i=1; i<mSceneNodesPoint.size(); i++)
	{
		mSceneNodesPoint[i]=m_pSceneNode->createChildSceneNode();

		Ogre::Entity *limbEntity ;
		if(lowPoly)
			limbEntity 	=renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "sphere1010_low.mesh");
		else
			limbEntity 	=renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "sphere1010.mesh");

		limbEntity ->setMaterialName("green");
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
		((Ogre::Entity*)mSceneNodes[i]->getAttachedObject(0))->setMaterialName(mat);

	for(int i=1; i<mSceneNodesPoint.size(); i++)
		((Ogre::Entity*)mSceneNodesPoint[i]->getAttachedObject(0))->setMaterialName(mat);
#endif
}

PLDPrimCyl::~PLDPrimCyl()
{
}

void PLDPrimCyl::createLimb(const OgreRenderer& renderer, Ogre::SceneNode *pNode)
{
#ifndef NO_OGRE
  ASSERT(0);
	Ogre::Entity *limbEntity =renderer.viewport().mScene->createEntity(RE::generateUniqueName().ptr(), "cylinder.mesh");
	limbEntity ->setMaterialName("green");
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

	pNode->resetToInitialState();
	pNode->scale(thick/2,thick/2,thick/2);

	quater t;
	t.axisToAxis(vector3(0,1,0), displacement);
	pNode->rotate(ToOgre(t));
	pNode->translate(ToOgre(center));
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
		mSceneNodesPoint[1]->resetToInitialState(); m_real f=_thick/2.0;
		mSceneNodesPoint[1]->scale(f,f,f);
		mSceneNodesPoint[1]->translate(ToOgre(to));
	}

	for(int i=2; i<mSkel->numBone(); i++)
	{
		vector3 from, to;
		from=mChain->global(*mSkel->bone(i).parent()).translation;
		to=mChain->global(i).translation;
		setLimb(mSceneNodes[i], from, to, _thick  /*from.distance(to)/5.f*/);

		if(mSceneNodesPoint.size())
		{
			mSceneNodesPoint[i]->resetToInitialState();m_real f=_thick/2.0;
			mSceneNodesPoint[i]->scale(f,f,f);
			mSceneNodesPoint[i]->translate(ToOgre(to));
		}
	}
#endif
	return 1;
}

PLDPrimPreciseOgreSkin::PLDPrimPreciseOgreSkin(MotionLoader* pLoader, Ogre::Entity* pEntity, const OgreRenderer& renderer, double scale)
:PLDPrimSkin_impl()
{
	m_pTimer=NULL;
#ifndef NO_OGRE
	m_pEntity=pEntity;
	mMotionScale=scale;

	// Add entity to the scene node
	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0,0,0));
	m_pSceneNode->attachObject(pEntity);
	//printf("movabletype:%s\n", pEntity->getMovableType().c_str());

	ASSERT(pEntity->getSkeleton());
	pEntity->getSkeleton()->_updateTransforms();


	m_aTargetIndexByTreeIndex.setSize(pLoader->numBone());
	m_aTargetIndexByTreeIndex.setAllValue(-1);
	m_aRotOrigComb.setSize(pLoader->numBone());
	m_aInvRotOrigComb.setSize(pLoader->numBone());
	m_aLocalRotOrig.setSize(pLoader->numBone());

	m_aTransOrigComb.setSize(pLoader->numBone());
	m_aInvTransOrigComb.setSize(pLoader->numBone());
	m_aLocalTransOrig.setSize(pLoader->numBone());

	for(int i=1; i<pLoader->numBone(); i++)
	{
		const char* nameid=pLoader->bone(i).NameId;
		Ogre::SkeletonInstance* pskel=m_pEntity->getSkeleton();
		Ogre::Bone* pBone=getOgreBoneByName(pskel, nameid);
		m_aTargetIndexByTreeIndex[i]=(int)(pBone->getHandle());
		Msg::verify(pBone!=NULL && pBone==pskel->getBone((unsigned short)(m_aTargetIndexByTreeIndex[i])), "There is no Bone %s", nameid);
		pBone->setManuallyControlled(true);
		m_aRotOrigComb[i]=ToBase(pBone->_getDerivedOrientation());
		m_aLocalRotOrig[i]=ToBase(pBone->getOrientation());
		m_aTransOrigComb[i]=ToBase(pBone->_getDerivedPosition());
		m_aLocalTransOrig[i]=ToBase(pBone->getPosition());

		transf origcomb(m_aRotOrigComb[i], m_aTransOrigComb[i]);
		transf iorigcomb=origcomb.inverse();
		m_aInvRotOrigComb[i]=iorigcomb.rotation;
		m_aInvTransOrigComb[i]=iorigcomb.translation;
	}
#endif
}

int PLDPrimPreciseOgreSkin::getOgreBoneIndex(int ibone) const { return m_aTargetIndexByTreeIndex[ibone];}

PLDPrimPreciseOgreSkin::~PLDPrimPreciseOgreSkin()
{
	//printf("dtorOgreSkin\n");

}

void PLDPrimPreciseOgreSkin::SetTranslation(float x, float y, float z)
{
#ifndef NO_OGRE
  vector3 pos;
	pos.x=m_pSceneNode->getPosition().x;
	pos.y=m_pSceneNode->getPosition().y;
	pos.z=m_pSceneNode->getPosition().z;

	pos-=m_vTrans;
	m_vTrans.setValue(x,y,z);

	pos+=m_vTrans;
	setPosition(m_pSceneNode,pos);
#endif
}


void PLDPrimPreciseOgreSkin::SetPose(const Posture & posture, const MotionLoader& skeleton)
{
#ifndef NO_OGRE
	quaterN aRotations(skeleton.numBone());
	vector3N aTranslations(skeleton.numBone());

	((MotionLoader&)skeleton).setPose(posture);

	for(int i=1; i<skeleton.numBone(); i++)
	{
		Bone& bone=skeleton.bone(i);
		bone.getRotation(aRotations[i]);
		bone.getTranslation(aTranslations[i]);
	}

	Ogre::SkeletonInstance* skel=m_pEntity->getSkeleton();

	// root or root2 should not be missing. (for positioning)


	const Ogre::Vector3 & scale=this->m_pSceneNode->getScale();
	double motionScale=scale.x*mMotionScale;
//bygth -- because debug build error occur
//	ASSERT(m_aTargetIndex[1]!=-1);
	{
		// set root
		Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndexByTreeIndex[1]));

		//= L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2 ---------------2

		transf localOrig(m_aLocalRotOrig[1], m_aLocalTransOrig[1]);
		transf invOrigComb(m_aInvRotOrigComb[1], m_aInvTransOrigComb[1]);
		transf origComb(m_aRotOrigComb[1], m_aTransOrigComb[1]);
		transf roottf(posture.m_aRotations[0], vector3(0,0,0));

		transf ogre_roottf=localOrig*invOrigComb*roottf*origComb;

		setTransform(pBone, ogre_roottf);
		setPosition(m_pSceneNode, (m_vTrans+posture.m_aTranslations[0])*motionScale);
	}

	for(int i=2; i<skeleton.numBone(); i++)
	{

		if(m_aTargetIndexByTreeIndex[i]!=-1)
		{
			Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndexByTreeIndex[i]));
			Bone& bone=skeleton.bone(i);

			/////////////////////////////////////////////////////////////////////////////////////////
			// Bind pose는 임의의 포즈가 될수 있다. (MaxExporter특성상, 주로 frame0를 사용한다.)

			// 현재 오우거 노드가 부모로부터 3번째 노드라고 하면.
			//  노드의 global orientation CN2=LN0*LN1*LN2
			//  C2가 모델의 binding pose에서의 global orientation이라 하자.

			// 목표:
			// binding pose를 SetPose함수에 입력한 경우, CN2가 C2가 되도록 해야한다.
			// C2BB 가 동작 데이타 바인딩 포즈의 combined mat이라 하자.
			// CN2 = C2B * C2BB.inv * C2 가 되면 된다.
			// (즉 C2BB와 C2B(SetPose함수의 입력자세)가 동일한 경우, CN2==C2)

			// CN2=CN1*LN2 이므로
			// -> LN2=CN1.inv * CN2
			//       =CN1.inv * C2B * C2BB.inv * C2
			//       = ( C1B * C1BB.inv* C1).inv * C2B * C2BB.inv * C2 ---------------1
			//		 = C1.inv* C1BB * C1B.inv * C2B * C2BB.inv * C2
			//		 = L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2 ---------------2

			// 검산
			//  CN3= LN0 * lN1* LN2
			//  = (L0B * L0BB.inv * L0)*(L0.inv * L0BB * L0B.inv * C1B * C1BB.inv * C1)*(C1.inv* C1BB * C1B.inv * C2B * C2BB.inv * C2)
			//  =  C2B * C2BB.inv * C2

			transf localOrig(m_aLocalRotOrig[i], m_aLocalTransOrig[i]);
			transf invOrigComb(m_aInvRotOrigComb[i], m_aInvTransOrigComb[i]);
			transf origComb(m_aRotOrigComb[i], m_aTransOrigComb[i]);
			int pindex=bone.parent()->treeIndex();
			transf parentglobaltf(aRotations[pindex], aTranslations[pindex]);
			transf globaltf(aRotations[i], aTranslations[i]);
			transf localtf=parentglobaltf.inverse()*globaltf;
			localtf.translation*=motionScale;
			{
				// dirty hard coding
				localtf.translation.zero();
				if(bone.transJointIndex()!=-1)
				{
					localtf.translation=posture.m_aTranslations[bone.transJointIndex()]*1000;
				}
			}
			transf ogre_localtf=localOrig*invOrigComb*localtf*origComb;

			setTransform(pBone, ogre_localtf);
			//
			//printf("%s\n", m_aBindPose[i].output().ptr()); // zero
			//printf("l:%s\n", m_aLocalRotOrig[i].output().ptr());// non-zero
			//printf("c:%s\n", m_aRotOrigComb[i].output().ptr());// non-zero

			// 고려사항 1. 양쪽의 link길이가 다른경우. 예를 들어 Ogre쪽은 link가 4개(0,1,@,2)이지만, 포즈쪽은 link가 3개(0,1,2)라 하자
			// 위식 1 대로 계산하면
			// LN2 = L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2;
			// LN@ = L@;
			// LN1 = L1 * C1.inv * C0BB * C0B.inv * C1B * C1BB.inv * C1
			// LN0 = C0B * C0BB.inv * L0;

			// 다곱하면.
			// CN2= C2B * C2BB.inv * C2; -> 문제 없음

			// 고려사항 2. 링크 구조가 다른경우. - 확실히 문제 생기는듯함. 수정요.
		}
	}


#ifdef _DEBUG_MAT
	skel->_updateTransforms();
#endif

	_drawConstraints(posture, skeleton);
#endif
}


void PLDPrimPreciseOgreSkin::ApplyAnim(const Motion& mot)
{


	AlzzaPostureIP* node;

	node=new AlzzaPostureIP();

	if(m_pTimer) delete m_pTimer;

	// target index가 reference가 아님
	node->Init(mot.NumJoints(),mot.frameTime());
	// reference (자료 공유)
	node->SetKeyValue(&mot);
	node->SetTarget(this);
	m_pTimer=new TimeSensor();
	m_pTimer->FirstInit(node->GetCycleInterval(), true);
	m_pTimer->AttachInterpolator(node);
	m_pTimer->StartAnim();

}


PLDPrimThickLine::PLDPrimThickLine(MotionLoader* pBVHL, const OgreRenderer& renderer)
:PLDPrimSkel(pBVHL)
{
#ifndef NO_OGRE
	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();

	_thickness=3; // 3cm
	_materialName="solidblue";
	_lines=new BillboardLineList (RE::generateUniqueName(), mSkel->numBone()-2, _thickness);
	for (int i=2; i<mSkel->numBone() ; i++)
	{
		vector3 start=mSkel->bone(i).getFrame().translation;
		vector3 end=mSkel->bone(i).parent()->getFrame().translation;
		_lines->line(i-2, start, end); 
	}

	_lines->setMaterialName(_materialName);
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
	mSceneNode->setVisible(bVisible);
#endif
}
void PLDPrimThickLine::setMaterial(const char* mat)
{
	_materialName=mat;
#ifndef NO_OGRE
	_lines->setMaterialName(_materialName);
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

#if OGRE_VERSION_MINOR >= 8||OGRE_VERSION_MAJOR>=13 
		_lines->updateChainElement(i-2,0, Ogre::BillboardChain::Element(
					ToOgre(from), thickness, tu1, Ogre::ColourValue(1,1,1,1), Ogre::Quaternion(1,0,0,0)));
		_lines->updateChainElement(i-2,1, Ogre::BillboardChain::Element(
					ToOgre(to), thickness, tu2, Ogre::ColourValue(1,1,1,1), Ogre::Quaternion(1,0,0,0)));
#else
		_lines->updateChainElement(i-2,0, Ogre::BillboardChain::Element(
					ToOgre(from), thickness, tu1, Ogre::ColourValue(1,1,1,1)));
		_lines->updateChainElement(i-2,1, Ogre::BillboardChain::Element(
					ToOgre(to), thickness, tu2, Ogre::ColourValue(1,1,1,1)));
#endif


	}
#endif
	return 1;
}
