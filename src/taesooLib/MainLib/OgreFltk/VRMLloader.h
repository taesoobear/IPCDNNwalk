// OgreMotionLoader.h: interface for the Ogre Skinned Mesh Character Skeleton
//     written by Taesoo Kwon.
//////////////////////////////////////////////////////////////////////

#ifndef VRMLLOADER_H_
#define VRMLLOADER_H_

#pragma once
#include "../../BaseLib/motion/VRMLloader.h"
#include "../../MainLib/OgreFltk/Mesh.h"
#include "pldprimskin.h"
#include "pldprimskin_impl.h"

void VRMLloader_updateMeshEntity(VRMLloader& l);

class PLDPrimCyl;
class OgreRenderer;
class PLDPrimVRML: public PLDPrimSkel
{
	VRMLloader* mVRMLL;
	PLDPrimCyl* mDrawSkel;
	void _updateEntities(BoneForwardKinematics&fk);
public:
	PLDPrimVRML(VRMLloader* pVRMLL, bool bDrawSkeleton, const OgreRenderer& renderer);
	virtual ~PLDPrimVRML();
	
	void setPoseDOF(const vectorn& poseDOF);
	void setPose(BoneForwardKinematics const& in);
	void applyAnim(const MotionDOF& motion);
	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton);
	virtual void setPoseDOF(const vectorn& poseDOF, MotionDOFinfo const& info) { setPoseDOF(poseDOF); }

	virtual void setThickness(float thick);

	void setMaterial(const char* mat);
	void setMaterial(int ibone, const char* mat);
	std::vector<Ogre::SceneNode*> mSceneNodes;
};

class PhysicalProperties
{
public:

  PhysicalProperties(VRMLloader& skel);	
  virtual ~PhysicalProperties(){}

  void segPositions(const MotionDOF& srcMotion, hypermatrixn& aaSegPos);
  void segVelocity(hypermatrixn& aaSegVel, const hypermatrixn& aaSegPos, float kernelSize);
  void segAcceleration(hypermatrixn& aaSegAcc, const hypermatrixn& aaSegVel, float kernelSize);

  void ZMP(matrixn& aZMP, const hypermatrixn& aaSegPos, const hypermatrixn& aaSegAcc);
  static void calcZMP(VRMLloader& skel,const MotionDOF& srcMotion, vector3N& aZMP, float kernel_size);

private: 
  VRMLloader& mSkel;

  const MotionDOF* mMotion;
  vectorn mSegMass;
  intvectorn mSegIndex;
};
namespace RE
{
	PLDPrimVRML* createVRMLskin(VRMLloader*pTgtSkel, bool bDrawSkeleton);
	TFactory<PLDPrimSkin>* createVRMLskinFactory(VRMLloader*pTgtSkel);
}
#endif
