// LineSegment.h: interface for the LineSegment class.
//
//		vector3 from1(0,0,0);
//		vector3 from2(0,30,0);
//		vector3 from3(0,30,30);

//		LineSegment* temp = new LineSegment(renderer);
//		temp->SetThickness(10.0);
//		temp->matLine.setSize(2,3);
//		temp->matLine.row(0).assign(from1);
//		temp->matLine.row(1).assign(from2);
		
//		temp->update();

//		LineSegment* temp2 = new LineSegment(renderer);

//		temp2->SetThickness(3.0);
//		temp2->matLine.setSize(2,3);
//		temp2->matLine.row(0).assign(from3);
//		temp2->matLine.row(1).assign(from1);

//		temp2->update();
//
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LINESEGMENT_H__27A79C05_504B_4299_9206_FCF61586F5A6__INCLUDED_)
#define AFX_LINESEGMENT_H__27A79C05_504B_4299_9206_FCF61586F5A6__INCLUDED_
#ifndef NO_OGRE
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include "RenderPrimitive.h"
#include "../../BaseLib/math/mathclass.h"
//#include "renderer.h"
class OgreRenderer;


// deprecated. use LineList
class LineSegment 
{
public:

	enum Color { WHITE, BLACK, RED, GREEN, LIGHT_GREEN, BLUE};
	//int Render(RenderStateManager* pRSM);
	// row: the numbef of sample points in a line segment, column:3, 

	void begin(int n)		{ matLine.setSize(n,3); lineColor.setSize(n); lineColor.setAllValue(m_cLineColor); }
	inline vectornView row(int irow)	{ return matLine.row(irow);}
	Color& color(int irow)	{ return (Color&)lineColor[irow];}
	void end()				{ update();}

	
	void addCtrlPoint(const vector3& v, Color c) { matLine.pushBack3(v); lineColor.pushBack(c); update();}
	void addCtrlPoint(const vector3& v)	{ addCtrlPoint(v, m_cLineColor);}
	int numCtrlPoint()		{ return matLine.rows();}

	/**
	 * Set drawing range of this line.
	 * \param the index of start points
	 * \param the index of end points
	 */
	void SetDrawRange(int start, int end)	{ m_nStartIndex=start; m_nEndIndex=end; update();}
	void SetThickness(float value) {m_fThickness = value; }
	void SetColor(Color c) {m_cLineColor=c;}
	void SetVisible(bool value);

	
	void remove();

	LineSegment(); 
	LineSegment(const OgreRenderer& renderer);
	virtual ~LineSegment();

	const matrixn& mtrxLine() const	{ return matLine;}
	
protected:
	intvectorn lineColor;

	std::vector<Ogre::SceneNode*> m_vecNodes;
	std::vector<Ogre::Entity*> m_vecEntities;
	int m_nStartIndex;	//!< default: 0 
	int m_nEndIndex; //!< default: INT_MAX
	Color m_cLineColor;
	
	bool m_bVisible;
	matrixn matLine;
	float m_fThickness;
	const OgreRenderer* renderer;
	Ogre::SceneNode *node_lines;	//!< default: NULL

	void update();
};


// deprecated. use BillboardLineList
class ThickLineList
{
public:

	enum Color { WHITE, BLACK, RED, GREEN, LIGHT_GREEN, BLUE};
	//int Render(RenderStateManager* pRSM);
	// row: the numbef of sample points in a line segment, column:3, 

	void begin(int n)		{ matLine.setSize(n,6); lineColor.setSize(n); lineColor.setAllValue(m_cLineColor); }
	void line(int i, vector3 const& start, vector3 const & end)
	{
		matLine.row(i).setVec3(0, start);
		matLine.row(i).setVec3(3, end);
	}

	inline vectornView row(int irow)	{ return matLine.row(irow);}
	Color& color(int irow)	{ return (Color&)lineColor[irow];}
	void end()				{ update();}

	
	void SetThickness(float value) {m_fThickness = value; }
	void SetColor(Color c) {m_cLineColor=c;}
	void SetVisible(bool value);
	
	void remove();

	ThickLineList(); 
	ThickLineList(const OgreRenderer& renderer);
	virtual ~ThickLineList();
	
protected:
	intvectorn lineColor;

	std::vector<Ogre::SceneNode*> m_vecNodes;
	std::vector<Ogre::Entity*> m_vecEntities;
	Color m_cLineColor;
	
	bool m_bVisible;
	matrixn matLine;
	float m_fThickness;
	const OgreRenderer* renderer;
	Ogre::SceneNode *node_lines;	//!< default: NULL

	void update();
};

#endif
#endif // !defined(AFX_LINESEGMENT_H__27A79C05_504B_4299_9206_FCF61586F5A6__INCLUDED_)

