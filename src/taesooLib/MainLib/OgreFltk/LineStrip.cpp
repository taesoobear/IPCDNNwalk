// LineSegment.cpp: implementation of the LineSegment class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdrenderer.h"

#include "stdafx.h"
#ifndef NO_OGRE
#include "../BaseLib/math/mathclass.h"
#include "LineStrip.h"
//#include "RenderStateManager.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "renderer.h"
#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


LineStrip::LineStrip(const OgreRenderer& renderer)
{
	m_cLineColor=RE::WHITE;
	m_fThickness=1.0f;
	m_nStartIndex=0;
	m_nEndIndex=INT_MAX;
	m_bVisible=true;
	this->renderer = &renderer;
	m_line3d = NULL;
	
	node_lines = renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode();
	//node_lines->attachObject(&m_line3d);
}

LineStrip::~LineStrip()
{
	if(RE::rendererValid())
		renderer->viewport().mScene->getRootSceneNode()->removeAndDestroyChild(node_lines->getName());	
}

//lips
void LineStrip::remove()
{
	matLine.setSize(0,0);
	if(m_line3d != NULL)
	{
		delete(m_line3d);
		m_line3d = NULL;
	}
}

void LineStrip::update()
{	
	if(!m_bVisible) return;

	static int ientity=0;
	int StartIndex, EndIndex;
	Ogre::Vector3 p0;
	vectorn temp;

	StartIndex = (m_nStartIndex > 0) ? m_nStartIndex : 0 ;
	EndIndex = (m_nEndIndex < (matLine.rows()-1)) ? m_nEndIndex : (matLine.rows()-1) ;

	if(m_line3d != NULL)
	{
		node_lines->detachObject(m_line3d);
		delete(m_line3d);
		m_line3d = NULL;
	}


	if(EndIndex < 1)
		return;
	

	m_line3d = new Line3D();
	
	m_line3d->SetColor(m_cLineColor);

	for(int i=StartIndex;i<=EndIndex;i++)
	{
		matLine.getRow(i,temp);		p0.x = temp.getValue(0); p0.y = temp.getValue(1); p0.z = temp.getValue(2); 
		m_line3d->addPoint(p0);	
	}

	m_line3d->drawLines();       
	node_lines->attachObject(m_line3d);
}

void LineStrip::SetColor(RE::Color c) 
{
	m_cLineColor=c; 
	if(m_line3d)
		m_line3d->SetColor(c);
}
void LineStrip::SetVisible(bool value)
{
	if(value==m_bVisible) return;
	if(node_lines)
		node_lines->setVisible(value);

	m_bVisible=value;
	if(value)
		update();
	

}

#endif
