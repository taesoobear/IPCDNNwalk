// LineSegment.cpp: implementation of the LineSegment class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdrenderer.h"
#include "stdafx.h"
#ifndef NO_OGRE
#include "LineSegment.h"

#include "../BaseLib/math/mathclass.h"
//#include "RenderStateManager.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "pldprimskin_impl.h"
#include "renderer.h"
#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

/*LineSegment::LineSegment()
{
	m_wLineColor=D3DCOLOR_RGBA(255,255,255,255);
	m_fThickness=1.0f;
	m_nStartIndex=0;
	m_nEndIndex=INT_MAX;

//lips
	node_lines = (Ogre::SceneNode*)0;
//lips
}*/

//lips

LineSegment::LineSegment()
{
	renderer=&RE::renderer();
	Msg::verify(renderer, "renderer invalid");
	m_cLineColor=WHITE;
	m_fThickness=1.0f;
	m_nStartIndex=0;
	m_nEndIndex=INT_MAX;
	m_bVisible=true;
	
	node_lines = (this->renderer)->viewport().mScene->getRootSceneNode()->createChildSceneNode();
}

LineSegment::LineSegment(const OgreRenderer& renderer)
{
	m_cLineColor=WHITE;
	m_fThickness=1.0f;
	m_nStartIndex=0;
	m_nEndIndex=INT_MAX;
	m_bVisible=true;
	this->renderer = &renderer;
	
	node_lines = (this->renderer)->viewport().mScene->getRootSceneNode()->createChildSceneNode();
}

LineSegment::~LineSegment()
{
	if(RE::rendererValid())
		renderer->viewport().mScene->getRootSceneNode()->removeAndDestroyChild(node_lines->getName());
}

//lips
void LineSegment::remove()
{
	matLine.setSize(0,0);
	for(int i=0; i<m_vecNodes.size(); i++)
		m_vecNodes[i]->setVisible(false);
}

void LineSegment::update()
{	
	if(!m_bVisible) return;

	static int ientity=0;
	int StartIndex, EndIndex;
	Ogre::SceneNode *pNode;
	Ogre::Vector3 p0, p1;
	Ogre::Vector3 center;
	Ogre::Vector3 displacement;
	Ogre::Vector3 origin(0,1,0);
	Ogre::Vector3 toorigin;
	float dist;
	vectorn temp;

	for(int i=0; i<m_vecNodes.size(); i++)
		m_vecNodes[i]->setVisible(false);

	StartIndex = (m_nStartIndex > 0) ? m_nStartIndex : 0 ;
	EndIndex = (m_nEndIndex < (matLine.rows()-1)) ? m_nEndIndex : (matLine.rows()-1) ;

	int a = matLine.rows(); int b = matLine.cols();
	for(int i=StartIndex;i<EndIndex;i++)
	{


		int entityIndex=i-StartIndex;

		if(entityIndex<m_vecEntities.size())
		{
			;
		}
		else if(entityIndex==m_vecEntities.size())
		{
			pNode = node_lines->createChildSceneNode();
			m_vecNodes.push_back(pNode);
			Ogre::Entity *lineEntity = renderer->viewport().mScene->createEntity(pNode->getName(), "tritower.mesh");
			lineEntity->setCastShadows(false);
			m_vecEntities.push_back(lineEntity);
			pNode->attachObject(lineEntity);
		}
		else Msg::error("Bug?");

		pNode=m_vecNodes[entityIndex];
		pNode->setVisible(true);

		switch(lineColor[i+1])
		{
		case WHITE:
			m_vecEntities[entityIndex]->setMaterialName("solidwhite");
			break;
		case BLUE:
			m_vecEntities[entityIndex]->setMaterialName("solidblue");
			break;
		case GREEN:
			m_vecEntities[entityIndex]->setMaterialName("solidgreen");
			break;
		case RED:
			m_vecEntities[entityIndex]->setMaterialName("solidred");
			break;
		case BLACK:
			m_vecEntities[entityIndex]->setMaterialName("solidblack");
			break;
		case LIGHT_GREEN:
			m_vecEntities[entityIndex]->setMaterialName("solidlightgreen");
			break;

		default:
			printf("error color %d unknown\n");
			break;
		}
		
		matLine.getRow(i,temp);		p0.x = temp.getValue(0); p0.y = temp.getValue(1); p0.z = temp.getValue(2); 
		matLine.getRow(i+1,temp);	p1.x = temp.getValue(0); p1.y = temp.getValue(1); p1.z = temp.getValue(2);
		
		center = 0.5*(p0+p1);
		displacement = 0.25*(p1-p0);
		dist = (float)displacement.length();

		pNode->resetToInitialState();
		pNode->scale(m_fThickness/2,dist,m_fThickness/2);
	
		Ogre::Vector3 axis = origin.crossProduct(displacement);
		if(axis.length() != 0.0f) 
		{
			axis = axis/axis.length();
			float costheta = displacement.dotProduct(origin)/dist;
			float angle = acos(costheta);
			pNode->rotate(axis,(Ogre::Radian)angle);	

			toorigin.x = displacement.x * (-1.98034);
			toorigin.y = displacement.y * (-1.98034);
			toorigin.z = displacement.z * (-1.98034);
		}
		else
		{
			toorigin.x = 0;
			toorigin.y = -1.98034 * dist;
			toorigin.z = 0;
		}

		pNode->translate(toorigin);
		pNode->translate(center);
	}
}

void LineSegment::SetVisible(bool value)
{
	if(value==m_bVisible) return;
	if(node_lines)
		node_lines->setVisible(value);

	m_bVisible=value;
	update();
}





/////////////////////////////////////////
//thicklinelist

ThickLineList::ThickLineList()
{
	renderer=&RE::renderer();
	Msg::verify(renderer, "renderer invalid");
	m_cLineColor=WHITE;
	m_fThickness=1.0f;
	m_bVisible=true;
	
	node_lines = (this->renderer)->viewport().mScene->getRootSceneNode()->createChildSceneNode();
}

ThickLineList::ThickLineList(const OgreRenderer& renderer)
{
	m_cLineColor=WHITE;
	m_fThickness=1.0f;
	m_bVisible=true;
	this->renderer = &renderer;
	
	node_lines = (this->renderer)->viewport().mScene->getRootSceneNode()->createChildSceneNode();
}

ThickLineList::~ThickLineList()
{
	if(RE::rendererValid())
		RE::removeEntity(node_lines);
}

//lips
void ThickLineList::remove()
{
	matLine.setSize(0,0);
	for(int i=0; i<m_vecNodes.size(); i++)
		m_vecNodes[i]->setVisible(false);
}

void ThickLineList::update()
{	
	if(!m_bVisible) return;

	static int ientity=0;
	
	Ogre::SceneNode *pNode;
	Ogre::Vector3 p0, p1;
	Ogre::Vector3 center;
	Ogre::Vector3 displacement;
	Ogre::Vector3 origin(0,1,0);
	Ogre::Vector3 toorigin;
	float dist;
	vectorn temp;

	for(int i=0; i<m_vecNodes.size(); i++)
		m_vecNodes[i]->setVisible(false);
	
	for(int i=0; i<matLine.rows();i++)
	{

		int entityIndex=i;

		if(entityIndex<m_vecEntities.size())
		{
			;
		}
		else if(entityIndex==m_vecEntities.size())
		{
			pNode = node_lines->createChildSceneNode();
			m_vecNodes.push_back(pNode);
			//Ogre::Entity *lineEntity = renderer->viewport().mScene->createEntity(pNode->getName(), "tritower.mesh");
			Ogre::Entity *lineEntity = renderer->viewport().mScene->createEntity(pNode->getName(), "cylinder.mesh");
			lineEntity->setCastShadows(false);
			m_vecEntities.push_back(lineEntity);
			pNode->attachObject(lineEntity);
		}
		else Msg::error("Bug?");

		pNode=m_vecNodes[entityIndex];
		pNode->setVisible(true);

		switch(lineColor[i])
		{
		case WHITE:
			m_vecEntities[entityIndex]->setMaterialName("solidwhite");
			break;
		case BLUE:
			m_vecEntities[entityIndex]->setMaterialName("solidblue");
			break;
		case GREEN:
			m_vecEntities[entityIndex]->setMaterialName("solidgreen");
			break;
		case RED:
			m_vecEntities[entityIndex]->setMaterialName("solidred");
			break;
		case BLACK:
			//m_vecEntities[entityIndex]->setMaterialName("solidblackNoZwrite");
			m_vecEntities[entityIndex]->setMaterialName("solidblack");
			break;
		case LIGHT_GREEN:
			m_vecEntities[entityIndex]->setMaterialName("solidlightgreen");
			break;

		default:
			printf("error color %d unknown\n", lineColor[i]);
			break;
		}
		

		// cylinder.mesh 사용시.
		PLDPrimCyl ::setLimb(pNode, matLine.row(i).toVector3(0), matLine.row(i).toVector3(3), m_fThickness );

		// tritower.mesh 사용시.
		/*
		p0=ToOgre(matLine.row(i).toVector3(0));
		p1=ToOgre(matLine.row(i).toVector3(3));
		
		
		
		
		
		


		center = 0.5*(p0+p1);
		displacement = 0.25*(p1-p0); // tritower.mesh
		
		dist = (float)displacement.length();

		pNode->resetToInitialState();
		pNode->scale(m_fThickness/2,dist,m_fThickness/2);
	
		Ogre::Vector3 axis = origin.crossProduct(displacement);
		if(axis.length() != 0.0f) 
		{
			axis = axis/axis.length();
			float costheta = displacement.dotProduct(origin)/dist;
			float angle = acos(costheta);
			pNode->rotate(axis,(Ogre::Radian)angle);	

			toorigin.x = displacement.x * (-1.98034);
			toorigin.y = displacement.y * (-1.98034);
			toorigin.z = displacement.z * (-1.98034);
		}
		else
		{
			toorigin.x = 0;
			toorigin.y = -1.98034 * dist;
			toorigin.z = 0;
		}

		pNode->translate(toorigin);
		pNode->translate(center);*/
	}
}

void ThickLineList::SetVisible(bool value)
{
	if(value==m_bVisible) return;
	if(node_lines)
		node_lines->setVisible(value);

	m_bVisible=value;
	update();
}
#endif
