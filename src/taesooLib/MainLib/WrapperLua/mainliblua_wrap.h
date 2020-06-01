#ifndef MAINLIBLUA_WRAP_H_
#define MAINLIBLUA_WRAP_H_
struct RE_
{
  static Viewpoint* getViewpoint();
  static Viewpoint* getViewpoint(int n);
  static std::string generateUniqueName();
  static Ogre::Entity* createPlane2(const char* id, m_real width, m_real height, int xsegment, int ysegment, int texSegx, int texSegy);
  static Ogre::Entity* createPlane(const char* id, m_real width, m_real height, int xsegment, int ysegment);
  static void setBackgroundColour(m_real r, m_real g, m_real b);
  static void remove(PLDPrimSkin* p);
  static void renderOneFrame(bool check);
  static PLDPrimSkin* createSkin2(const MotionLoader& skel, int typet);
  static PLDPrimSkin* createSkin3(const Motion& mot, int typet);
};
#endif
