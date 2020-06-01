
#ifndef MOTIONDOFINFO_H_
#define MOTIONDOFINFO_H_

class MotionDOFinfo
{
 public:
  struct BoneInfo
  {
    BoneInfo()	{startT=0;startR=0;endR=0;}
    int startT;	// linear DOF (translation)
    int startR, endR;	// spherical or angular DOF (spherical joint takes 4 numbers because of quaternion)
	  int startDQ, endDQ; // for storing dq (linear and angular velocity)
  };

  struct SharedInfo
  {
    std::vector<BoneInfo> mBoneInfo;
    int mTotalDOF;
    Posture mTempPosture;
    MotionLoader * mSkeleton;
	intvectorn mSphericalDofIndexes;
	intvectorn DQtoBoneIndex;
	intvectorn DOFtoBoneIndex;
	intvectorn DQtoDOF;
	intvectorn DOFtoDQ;
    void init(MotionLoader const& l);	// root joint에만 spherical joint사용.
    void init(MotionLoader const& l, bitvectorn const& useSpherical);
    void _init(MotionLoader const& l, bitvectorn const& useSpherical);
  };

  SharedInfo* _sharedinfo;
  double mFrameRate;
     
  MotionDOFinfo();
  MotionDOFinfo(const MotionDOFinfo& info);
  ~MotionDOFinfo();
   
  enum DOFtype_T { ROTATE, SLIDE, QUATERNION_W, QUATERNION_X, QUATERNION_Y, QUATERNION_Z};

  MotionLoader & skeleton() const	{return *_sharedinfo->mSkeleton;}
  int numDOF() const; // including quaternion's additional variables.
  int numActualDOF() const; // numDOF()- numSphericalJoint()
  int numBone() const;
  int numDOF(int ibone) const;
  int DOFtype(int ibone, int offset) const;
  int DOFindex(int ibone, int offset) const;
  int sphericalDOFindex(int isphericalJoint) const;
  int numSphericalJoint() const;
  double frameRate() const;
  void setFrameRate(double f);
  void getDOF(Posture const& p, vectorn& dof) const;
  void setDOF(vectorn const& dof, Posture& p) const;
  Posture const& setDOF(vectorn const& dof) const;	//!< non thread-safe
  bool hasTranslation(int iBone) const;
  bool hasQuaternion(int iBone) const;
  bool hasAngles(int iBone) const;
  int startT(int iBone) const;
  int startR(int iBone) const;
  int endR(int iBone) const;
  int startDQ(int iBone) const; 
  int endDQ(int iBone) const;
  inline int DQtoBone(int DQindex) const { return _sharedinfo->DQtoBoneIndex[DQindex];}
  inline int DOFtoBone(int DOFindex) const { return _sharedinfo->DQtoBoneIndex[DOFindex];}
  inline int DOFtoDQ(int DOFindex) const { return _sharedinfo->DOFtoDQ[DOFindex];}
  inline int DQtoDOF(int DOFindex) const { return _sharedinfo->DQtoDOF[DOFindex];}
  // two-different types of joint velocity packing
  // type 1:
  // DOFindex: (v.x, v.y,v.z, unused, w.x, w.y, w.z, ...) (quaterternion's w values are unused.)
  // 	-> size: numDOF()
  // 	-> convenient for setLinkData, getLinkData.
  // 	 	(compatible with MotionDOF)
  // type 2:
  // DQindex: (v.x, v.y,v.z, w.x, w.y, w.z, ...)  for storing tau, dq, ddq
  // 	-> size: numActualDOF()
  // 	-> convenient for jacobians, and matrix multiplications.
  // 	actual packing depends on simulators. The following two functions assume the gmbs simulator.
  // 	gmbs : (w.x, w.y, w.z, v.x, v.y, v.z, ...)  where v, w are body-local velocities
  // 	trl : (v.x, v.y, v.z, w.x, w.y, w.z, ...) where v, w are global velocities
  void DOFtoDQ(vectorn const& dtheta, vectorn & dq);
  void DQtoDOF(vectorn const& dq, vectorn & dtheta);
  void blend(vectorn & out, vectorn const& p1, vectorn const& p2, m_real t) const;
  void blendDelta(vectorn & out, vectorn const& p1, vectorn const& p2, m_real t) const;
  void blendBone(int ibone, vectorn & c, vectorn const& a, vectorn const& b, m_real t) const;
};
#endif
