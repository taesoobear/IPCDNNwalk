#ifndef __LINE3D_H__ 
#define __LINE3D_H__ 
#ifndef NO_OGRE
#ifdef None
#undef None
#endif

#include <OgreBillboardChain.h>
#include <OgreRenderOperation.h>
#include <Math/Array/OgreObjectMemoryManager.h>
#include <vector> 
//#include "LineStrip.h"

#define POSITION_BINDING 0 
#define TEXCOORD_BINDING 1 

#ifdef Real
#undef Real
#endif
class SimplerRenderable: public Ogre::v1::SimpleRenderable
{
public:
	SimplerRenderable():Ogre::v1::SimpleRenderable(RE::generateUniqueID(), RE::_objectMemoryManager(), RE::ogreSceneManager()){}
	virtual ~SimplerRenderable(){}

	virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera *cam) const; 
	virtual Ogre::Real getBoundingRadius(void) const; 

};
#include "dynamicRenderable.h"


// usage : line=new BillboardLineList("...", 3); line->line(0,....);
class BillboardLineList : public Ogre::v1::BillboardChain
{	
protected:
	int nSize;
	m_real thickness;
public:
	BillboardLineList(Ogre::IdType id, int size, m_real thick=7.0): Ogre::v1::BillboardChain(id, RE::_objectMemoryManager(), RE::ogreSceneManager(), 2, size), thickness(thick){}
	~BillboardLineList (){}

	void line(int i, vector3 const& start, vector3 const & end, m_real tu1=0.0, m_real tu2=1.0);
};

class ColorBillboardLineList : public BillboardLineList
{	
public:
	ColorBillboardLineList(Ogre::IdType id, int size, m_real thick=7.0): BillboardLineList(id, size, thick) {}
	~ColorBillboardLineList (){}

	void line(int i, vector3 const& start, vector3 const & end, vector3 const & rgbcolor, m_real tu1=0.0, m_real tu2=1.0);
};

class ColorWidthBillboardLineList : public BillboardLineList
{	
public:
	ColorWidthBillboardLineList(Ogre::IdType id, int size, m_real thick=7.0): BillboardLineList(id, size, thick) {}
	~ColorWidthBillboardLineList (){}

	void line(int i, vector3 const& start, vector3 const & end, vector3 const & rgbcolor, m_real width, m_real width2, m_real tu1=0.0, m_real tu2=1.0);
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


	void setMaterial(const char* name);
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



#endif
#endif /* __LINE3D_H__ */ 
