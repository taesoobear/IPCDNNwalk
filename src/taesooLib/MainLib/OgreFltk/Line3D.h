#ifndef __LINE3D_H__ 
#define __LINE3D_H__ 
#ifndef NO_OGRE

#include <OgreSimpleRenderable.h>
#include <OgreBillboardChain.h>
#include <vector> 
//#include "LineStrip.h"

#define POSITION_BINDING 0 
#define TEXCOORD_BINDING 1 

#ifdef Real
#undef Real
#endif
class SimplerRenderable: public Ogre::SimpleRenderable
{
public:
	SimplerRenderable(){}
	virtual ~SimplerRenderable(){}

	virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera *cam) const; 
	virtual Ogre::Real getBoundingRadius(void) const; 

};

#include "dynamicRenderable.h"
#include <vector>

class DynamicLines : public DynamicRenderable
{
  typedef Ogre::Vector3 Vector3;
  typedef Ogre::Quaternion Quaternion;
  typedef Ogre::Camera Camera;
  typedef Ogre::Real Real;
  typedef Ogre::RenderOperation::OperationType OperationType;

public:
  /// Constructor - see setOperationType() for description of argument.
  DynamicLines(OperationType opType=Ogre::RenderOperation::OT_LINE_STRIP);
  virtual ~DynamicLines();

  /// Add a point to the point list
  void addPoint(const Ogre::Vector3 &p);
  /// Add a point to the point list
  void addPoint(Real x, Real y, Real z);

  /// Change the location of an existing point in the point list
  void setPoint(unsigned short index, const Ogre::Vector3 &value);

  /// Return the location of an existing point in the point list
  const Ogre::Vector3& getPoint(unsigned short index) const;

  /// Return the total number of points in the point list
  unsigned short getNumPoints(void) const;

  /// Remove all points from the point list
  void clear();

  /// Call this to update the hardware buffer after making changes.  
  void update();

  /** Set the type of operation to draw with.
   * @param opType Can be one of 
   *    - RenderOperation::OT_LINE_STRIP
   *    - RenderOperation::OT_LINE_LIST
   *    - RenderOperation::OT_POINT_LIST
   *    - RenderOperation::OT_TRIANGLE_LIST
   *    - RenderOperation::OT_TRIANGLE_STRIP
   *    - RenderOperation::OT_TRIANGLE_FAN
   *    The default is OT_LINE_STRIP.
   */
  void setOperationType(OperationType opType);
  OperationType getOperationType() const;

protected:
  /// Implementation DynamicRenderable, creates a simple vertex-only decl
  virtual void createVertexDeclaration();
  /// Implementation DynamicRenderable, pushes point list out to hardware memory
  virtual void fillHardwareBuffers();

  std::vector<Vector3> mPoints;
private:
  
  bool mDirty;
};

// you should attach this to a node to make it visible.
class LineList: public DynamicLines
{ 	
public: 
	LineList(void):DynamicLines(Ogre::RenderOperation::OT_LINE_LIST){}
	~LineList(void){}

	void begin(int n);
	void begin();	// not required for a new object.
	void line(int i, vector3 const& start, vector3 const& end);

#if OGRE_VERSION_MINOR>=12|| OGRE_VERSION_MAJOR>=13

	void setMaterial(const char* name);
#endif
	// after finishing add~~, end() must be called.
	void addLine(vector3 const& start, vector3 const& end);
	void addAxisAlignedBox(vector3 const& minCorner, vector3 const& maxCorner);	
	void addCircle(vector3 const& center, vector3 const& axis, m_real halfR);

	void end()	{ update();}

}; 


// usage : line=new BillboardLineList("...", 3); line->line(0,....);
class BillboardLineList : public Ogre::BillboardChain
{	
protected:
	int nSize;
	m_real thickness;
public:
	BillboardLineList(const char* name, int size, m_real thick=7.0): Ogre::BillboardChain(name, 2, size), thickness(thick){}
	~BillboardLineList (){}

	void line(int i, vector3 const& start, vector3 const & end, m_real tu1=0.0, m_real tu2=1.0);
};

class ColorBillboardLineList : public BillboardLineList
{	
public:
	ColorBillboardLineList(const char* name, int size, m_real thick=7.0): BillboardLineList(name, size, thick) {}
	~ColorBillboardLineList (){}

	void line(int i, vector3 const& start, vector3 const & end, vector3 const & rgbcolor, m_real tu1=0.0, m_real tu2=1.0);
};

class ColorWidthBillboardLineList : public BillboardLineList
{	
public:
	ColorWidthBillboardLineList(const char* name, int size, m_real thick=7.0): BillboardLineList(name, size, thick) {}
	~ColorWidthBillboardLineList (){}

	void line(int i, vector3 const& start, vector3 const & end, vector3 const & rgbcolor, m_real width, m_real tu1=0.0, m_real tu2=1.0);
};

class QuadList: public DynamicRenderable
{
	enum { POS_TEX_BINDING=0 };
public:
	QuadList(vector3 const& normal, m_real width); 
	~QuadList(void); 

	void begin(int n);
	void quad(int i, vector3 const& pos);
	void end();

#if OGRE_VERSION_MINOR>=12|| OGRE_VERSION_MAJOR>=13

	void setMaterial(const char* name);
#endif
	virtual void createVertexDeclaration();
	virtual void fillHardwareBuffers();

	vector3 mNormal;
	m_real mWidth;
	vector3N mPoints;
};

class ColorPointList: public DynamicRenderable
{
	enum { POS_COLOUR_BINDING=0 };
public:
	ColorPointList(); 
	~ColorPointList(void); 

	void begin(int n);
	void point(int i, vector3 const& color, vector3 const& pos);
	void end();

private:
	virtual void createVertexDeclaration();
	virtual void fillHardwareBuffers();
	vector3N mPoints;
	vector3N mColors;
};

// deprecated. use LineList.
class Line3D : public SimplerRenderable 
{ 
public: 

		
   Line3D(void); 
   ~Line3D(void); 

   void addPoint(const Ogre::Vector3 &p); 
   void removeAllpoints();
   const Ogre::Vector3 &getPoint(unsigned short index) const; 
   unsigned short getNumPoints(void) const; 
   void updatePoint(unsigned short index, const Ogre::Vector3 &value); 
   void drawLine(Ogre::Vector3 &start, Ogre::Vector3 &end); 
   void drawLines(void); 
   void SetColor(int i);

protected: 
   //void getWorldTransforms(Matrix4 *xform) const; 
   const Ogre::Quaternion &getWorldOrientation(void) const; 
   const Ogre::Vector3 &getWorldPosition(void) const; 

   std::vector<Ogre::Vector3> mPoints; 
   bool mDrawn; 
   int m_cLineColor;
}; 

void createSphere(const std::string& strName, const float r, const int nRings = 16, const int nSegments = 16);

#endif
#endif /* __LINE3D_H__ */ 
