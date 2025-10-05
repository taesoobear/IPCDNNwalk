#ifndef MAINLIBLUA_WRAP_H_
#define MAINLIBLUA_WRAP_H_
namespace RE_
{
  Viewpoint* getViewpoint();
  Viewpoint* getViewpoint(int n);
  std::string generateUniqueName();
  Ogre::Item* createPlane2(const char* id, m_real width, m_real height, int xsegment, int ysegment, int texSegx, int texSegy);
  Ogre::Item* createPlane(const char* id, m_real width, m_real height, int xsegment, int ysegment);
  void setBackgroundColour(m_real r, m_real g, m_real b);
  void remove(PLDPrimSkin* p);
  bool renderOneFrame(bool check);
  PLDPrimSkin* createSkin2(const MotionLoader& skel, int typet);
  PLDPrimSkin* createSkin3(const Motion& mot, int typet);
};
matrixn MotionDOF_calcDerivative(matrixn const& motionDOF, double frameRate );
matrixn MotionDOF_calcDerivative(MotionDOF const& motionDOF, double frameRate );
#endif
