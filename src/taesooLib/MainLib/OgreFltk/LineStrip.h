//////////////////////////////////////////////////////////////////////

#ifndef __LINESTRIP_H__ 
#define __LINESTRIP_H__ 
#ifndef NO_OGRE
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include "RenderPrimitive.h"
#include "../../BaseLib/math/mathclass.h"
//#include "renderer.h"
//
class OgreRenderer;
#include "Line3D.h"



class LineStrip
{
public:
	
	//int Render(RenderStateManager* pRSM);
	// row: the numbef of sample points in a line segment, column:3, 

	void begin()			{ begin(0);}
	void begin(int n)		{ matLine.setSize(n,3); lineColor.setSize(n); lineColor.setAllValue(m_cLineColor); }
	inline vectornView row(int irow)	{ return matLine.row(irow);}
	RE::Color& color(int irow)	{ return (RE::Color&)lineColor[irow];}
	void end()				{ update();}

	
	void addCtrlPoint(const vector3& v, RE::Color c) { matLine.pushBack3(v); lineColor.pushBack(c); /*update();*/}
	void addCtrlPoint(const vector3& v)	{ addCtrlPoint(v, m_cLineColor);}
	int numCtrlPoint()		{ return matLine.rows();}

	/**
	 * Set drawing range of this line.
	 * \param the index of start points
	 * \param the index of end points
	 */
	void SetDrawRange(int start, int end)	{ m_nStartIndex=start; m_nEndIndex=end;}
	void SetThickness(float value) {m_fThickness = value; }
	void SetColor(RE::Color c);
	void SetVisible(bool value);

	
	void remove();
	LineStrip(const OgreRenderer& renderer);
	virtual ~LineStrip();

	const matrixn& mtrxLine() const	{ return matLine;}
	
protected:
	intvectorn lineColor;

	//std::vector<Ogre::SceneNode*> m_vecNodes;
	//std::vector<Ogre::Entity*> m_vecEntities;
	Line3D *m_line3d;	//!< default: NULL

	int m_nStartIndex;	//!< default: 0 
	int m_nEndIndex; //!< default: INT_MAX
	RE::Color m_cLineColor;
	
	bool m_bVisible;
	matrixn matLine;
	float m_fThickness;
	const OgreRenderer* renderer;
	Ogre::SceneNode *node_lines;	//!< default: NULL

	void update();
};


#endif
#endif // !defined(AFX_LINESEGMENT_H__27A79C05_504B_4299_9206_FCF61586F5A6__INCLUDED_)

