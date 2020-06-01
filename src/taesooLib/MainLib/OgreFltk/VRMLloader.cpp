
#include "stdafx.h"
#include "VRMLloader.h"
#include "../math/hyperMatrixN.h"
#include "../MainLib/OgreFltk/pldprimskin_impl.h"
#include "../MainLib/OgreFltk/renderer.h"
#include "../BaseLib/math/conversion.h"
#ifndef NO_OGRE
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#endif
#include "../BaseLib/motion/VRMLloader_internal.h"


class HRP_SHAPE_MESH_TO_ENTITY: public HRP_CUSTOM_SHAPE
{
	public:
	OBJloader::MeshToEntity* meshToEntity;
	HRP_SHAPE_MESH_TO_ENTITY(){
		meshToEntity=NULL;
	}
	virtual ~HRP_SHAPE_MESH_TO_ENTITY(){
		delete meshToEntity;
	}
};

static bool VRMLloader_isMeshEntityUpdated(VRMLloader  const& l)
{
	for(int i=1;i<l.numBone(); i++)
	{
		VRMLTransform* ll=((VRMLTransform*)&l.getBoneByTreeIndex(i));

		if(ll->mShape)
		{
			if(ll->mShape->customShape) return true;
			return false;
		}
	}
	return true;
}

// recreation of PLDPrimVRML is required to actually see the update mesh.
void VRMLloader_updateMeshEntity(VRMLloader& l)
{
  for(int i=1;i<l.numBone(); i++)
    {
      VRMLTransform* ll=((VRMLTransform*)&l.getBoneByTreeIndex(i));

      if(ll->mShape)
	{
	  if(ll->mShape->customShape)
	  {
		((HRP_SHAPE_MESH_TO_ENTITY*)ll->mShape->customShape)->meshToEntity->removeAllResources();
	    delete ll->mShape->customShape;
	  }
	  OBJloader::MeshToEntity::Option o;
	  o.useTexCoord=false;
	  o.buildEdgeList=true;
	  //o.buildEdgeList=false;

	  HRP_SHAPE_MESH_TO_ENTITY* oo=new HRP_SHAPE_MESH_TO_ENTITY() ;
	  oo->meshToEntity=new OBJloader::MeshToEntity(ll->mShape->mesh, RE::generateUniqueName(), o);
	  ll->mShape->customShape=oo;	
	}
    }
}


PLDPrimVRML::PLDPrimVRML(VRMLloader* pVRMLL, bool bDrawSkeleton, const OgreRenderer& renderer)
 :PLDPrimSkel(pVRMLL)
{
	pVRMLL->UpdateBone();
	if(!VRMLloader_isMeshEntityUpdated(*pVRMLL))
		VRMLloader_updateMeshEntity(*pVRMLL);
	mVRMLL=pVRMLL;
  
#ifndef NO_OGRE
  m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();
  mSceneNodes.resize(pVRMLL->numBone());
  for(int i=1; i<mSceneNodes.size(); i++)
    {
      VRMLTransform* ll=((VRMLTransform*)&mVRMLL->getBoneByTreeIndex(i));
      if(ll->mShape)
	{
	  mSceneNodes[i]=m_pSceneNode->createChildSceneNode();
	  mSceneNodes[i]->attachObject(
				       ((HRP_SHAPE_MESH_TO_ENTITY*)(ll->mShape->customShape))->meshToEntity->createEntity(RE::generateUniqueName(),
									      ll->mSegment->material));
	}
      else
	mSceneNodes[i]=NULL;

    }
  _updateEntities(mVRMLL->fkSolver());
  mDrawSkel=NULL;

  if(bDrawSkeleton)
    {
      //PLDPrimCyl(MotionLoader* pBVHL, const OgreRenderer& renderer, bool lowPoly=false);

      mDrawSkel=new PLDPrimCyl(pVRMLL, renderer, true);
	  mDrawSkel->setThickness(0.05);
      renderer.viewport().mScene->getRootSceneNode()->removeChild(mDrawSkel->m_pSceneNode);
      m_pSceneNode->addChild(mDrawSkel->m_pSceneNode);
    }
#endif
}

PLDPrimVRML::~PLDPrimVRML()
{
  #ifndef NO_OGRE
  if(mDrawSkel)
    {
      mDrawSkel->m_pSceneNode=NULL;
      delete mDrawSkel;
    }
  // mSceneNodes and their entities will be destroyed at ~AnimationObject();
#endif
}

void PLDPrimVRML::setPoseDOF(const vectorn& poseDOF)
{
  SetPose(mVRMLL->dofInfo.setDOF(poseDOF), *mVRMLL);
}

void PLDPrimVRML::setPose(BoneForwardKinematics const& in)
{
	*mChain=in;
	_updateEntities(*mChain);

  	/*
	static Posture pose;
	in.getPoseFromLocal(pose);
	SetPose(pose, *mVRMLL);*/
}

void PLDPrimVRML::SetPose(const Posture & posture, const MotionLoader& skeleton)
{
	PLDPrimSkel::SetPose(posture, skeleton);
#ifndef NO_OGRE
  //_updateEntities((MotionLoader*)&skeleton);
  _updateEntities(*mChain);

  if(mDrawSkel)
    mDrawSkel->SetPose(posture, skeleton);
#endif
}

void PLDPrimVRML::setThickness(float thick)
{
  #ifndef NO_OGRE
  if(mDrawSkel)
    mDrawSkel->setThickness(thick);
#endif
}

void PLDPrimVRML::_updateEntities(BoneForwardKinematics & fk)
{
#ifndef NO_OGRE
	for(int i=1; i<mSceneNodes.size(); i++)
	{
		VRMLTransform* ll=((VRMLTransform*)&mVRMLL->getBoneByTreeIndex(i));
		if(ll->mShape)
		{
			mSceneNodes[i]->resetToInitialState();
			mSceneNodes[i]->rotate(ToOgre(fk.global(i).rotation));
			mSceneNodes[i]->translate(ToOgre(fk.global(i).translation));
		}
	}
	__drawConstraints(*mVRMLL);
#endif
}

void PLDPrimVRML::setMaterial(const char* mat)
{
#ifndef NO_OGRE
  for(int i=1; i<mSceneNodes.size(); i++)
	  if(mSceneNodes[i])
		((Ogre::Entity*)mSceneNodes[i]->getAttachedObject(0))->setMaterialName(mat);
#endif
}

void PLDPrimVRML::setMaterial(int i,const char* mat)
{
#ifndef NO_OGRE
	  if(mSceneNodes[i] && mSceneNodes[i]->numAttachedObjects())
		((Ogre::Entity*)mSceneNodes[i]->getAttachedObject(0))->setMaterialName(mat);
#endif
}

PLDPrimVRML* RE::createVRMLskin(VRMLloader*pTgtSkel, bool bDrawSkeleton)
{
  PLDPrimVRML* pSkin;
  pSkin=new PLDPrimVRML(pTgtSkel, bDrawSkeleton, RE::renderer());
  RE::renderer().addFrameMoveObject(pSkin);
  return pSkin;
}



PhysicalProperties::PhysicalProperties(VRMLloader& skel)
 :mSkel(skel)
{
  mSegMass.reserve(skel.numBone());
  mSegIndex.reserve(skel.numBone());

  for(int i=1; i<skel.numBone(); i++)
    {
      HRP_SEGMENT* seg=skel.VRMLbone(i).mSegment;
      if(seg)
	{
	  mSegMass.pushBack(seg->mass);
	  mSegIndex.push_back(i);
	}
    }
}

void PhysicalProperties::segPositions(const MotionDOF& srcMotion,hypermatrixn& aaSegCOG)
{
  aaSegCOG.setSize(mSegMass.size(), srcMotion.numFrames(),3);

  for(int i=0; i<srcMotion.numFrames(); i++)
    {
      mSkel.setPoseDOF(srcMotion.row(i));
		
      for(int iseg=0; iseg< mSegMass.size(); iseg++)
	{
	  VRMLTransform& b=mSkel.VRMLbone(mSegIndex[iseg]);
	  aaSegCOG[iseg].row(i).assign(b.getFrame().toGlobalPos(b.mSegment->centerOfMass));
	}
    }

  mMotion=&srcMotion;

}

#include "../BaseLib/math/Operator.h"
#include "../BaseLib/math/Filter.h"
#include "../math/hyperMatrixN.h"

void PhysicalProperties::segVelocity(hypermatrixn& aaSegVel, const hypermatrixn& aaSegPos, float kernelSize)
{
  aaSegVel.setSameSize(aaSegPos);
  for(int page=0; page<aaSegVel.page(); page++)
	  m::derivative(aaSegVel[page], aaSegPos[page]);
  for(int page=0; page<aaSegVel.page(); page++)
    {
      double frameTime=1.0/mMotion->mInfo.frameRate();
      aaSegVel[page]/=frameTime;

      if(kernelSize!=0)
		  m::filter( aaSegVel[page],Filter::CalcKernelSize( kernelSize, frameTime));
    }
}

void PhysicalProperties::segAcceleration(hypermatrixn& aaSegAcc, const hypermatrixn& aaSegVel, float kernelSize)
{
  // calc derivative
  segVelocity(aaSegAcc, aaSegVel, kernelSize);   
}


// MeterPerUnit is required for calculating gravity G in the virtual unit system.
void PhysicalProperties::ZMP(matrixn& aZMP, const hypermatrixn& aaSegPos, const hypermatrixn& aaSegAcc)
{
  float g=9.8;		//meter/second^2
  g*=-1;
	
  int numFrame=aaSegPos.rows();
  aZMP.setSize(numFrame, 3);

  vector3 ri, ri__;
  m_real mi, A, B, C, D, E;
  int NUM_SEGMENT=mSegMass.size();
  for(int f=0; f<numFrame; f++)
    {
      A= B= C= D= E= 0.f;
      for(int i=0; i<NUM_SEGMENT; i++)
	{
	  ri=aaSegPos[i].row(f).toVector3();
	  ri__=aaSegAcc[i].row(f).toVector3();
	  mi=mSegMass[i];

	  A+=mi*(ri__.y-g  )*ri.x;
	  B+=mi*(ri__.x-0.f)*ri.y;
	  C+=mi*(ri__.y-g  );
	  D+=mi*(ri__.y-g  )*ri.z;
	  E+=mi*(ri__.z-0.f)*ri.y;
	}

      aZMP[f][0]=(A-B)/C;
      aZMP[f][1]=0.f;
      aZMP[f][2]=(D-E)/C;		
    }
}

vector3 VRMLloader::calcCOM() const
{
	VRMLloader* skel=(VRMLloader*)this;
	BoneForwardKinematics* chain=&fkSolver();

	vector3 com(0,0,0);

	m_real totalMass=0.0;
	for(int ibone=1; ibone<skel->numBone(); ibone++)
	{
		VRMLTransform& bone=skel->VRMLbone(ibone);
		if (bone.mSegment) {
			double mass=bone.mSegment->mass;
			com+=chain->global(bone).toGlobalPos(bone.mSegment->centerOfMass)*mass;
			totalMass+=mass;
		}
	}

	com/=totalMass;
	return com;

}

void VRMLloader::calcZMP(const MotionDOF& motion, matrixn & aZMP,double kernelSize)
{
  PhysicalProperties p(*this);
  hypermatrixn aaSegPos, aaSegVel, aaSegAcc;

  p.segPositions(motion, aaSegPos);
  p.segVelocity(aaSegVel, aaSegPos, kernelSize);
  p.segAcceleration(aaSegAcc, aaSegVel, kernelSize);


  p.ZMP(aZMP, aaSegPos, aaSegAcc);
}



void PLDPrimVRML::applyAnim(const MotionDOF& motion)
{
	if(m_pTimer)	delete m_pTimer;

	MotionDOFInterpolator* node;

	node=new MotionDOFInterpolator();
	//node->Init(1.0/120.0);
	node->Init(1.0/motion.mInfo.frameRate());
	// reference (??료 공유)
	node->SetKeyValue(&motion);
	node->SetTarget(this);
	m_pTimer=new TimeSensor();
	m_pTimer->FirstInit(node->GetCycleInterval(), true);
	m_pTimer->AttachInterpolator(node);
	m_pTimer->StartAnim();
}
class VRMLskinFactory : public TDefaultFactory<PLDPrimSkin>
{
	VRMLloader* _skel;
	public:
		VRMLskinFactory(VRMLloader*pTgtSkel)
		{
			_skel=pTgtSkel;
		}
		virtual PLDPrimSkin* create(int index=-1) const { return (PLDPrimSkin*)RE::createVRMLskin(_skel);}
};

TFactory<PLDPrimSkin>* RE::createVRMLskinFactory(VRMLloader*pTgtSkel)
{
	return (TFactory<PLDPrimSkin>*)new VRMLskinFactory(pTgtSkel);
}

void PhysicalProperties::calcZMP(VRMLloader& skel, const MotionDOF& srcMotion, vector3N& aZMP, float kernel_size)
{
	aZMP.setSize(srcMotion.numFrames());

	PhysicalProperties p(skel);
	hypermatrixn segPos, segVel, segAcc;
	p.segPositions(srcMotion, segPos);
	p.segVelocity(segVel, segPos, kernel_size);
	p.segAcceleration(segAcc, segVel, kernel_size);
	matrixnView aa=matView(aZMP);
	p.ZMP(aa, segPos, segAcc);
}
