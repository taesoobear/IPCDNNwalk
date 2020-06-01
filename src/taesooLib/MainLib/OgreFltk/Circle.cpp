#include "stdafx.h"
#ifndef NO_OGRE

#include "../BaseLib/math/mathclass.h"
#include "Circle.h"
//#include "RenderStateManager.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
#endif

Circle::Circle(const OgreRenderer& renderer)
{
	circle = new LineStrip(renderer);	
}

Circle::~Circle()
{
	delete(circle);
}

void Circle::SetCircle(vector3& center, float radius, int n_controlpoint)
{
	if(radius <= 0 || n_controlpoint < 3)
		return ;

	circle->remove();
	circle->begin();

	int i;
	vector3 input;

	for(i=0;i<=n_controlpoint;i++)
	{
		input.x = center.x + radius * cos(((double)i/(double)n_controlpoint)*2.0*PI);
		input.y = center.y;
		input.z = center.z + radius * sin(((double)i/(double)n_controlpoint)*2.0*PI);
		circle->addCtrlPoint(input);
	}
	circle->end();
}

void Circle::SetVisible(bool value)
{
	circle->SetVisible(value);
}

void Circle::SetColor(RE::Color c)
{
	circle->SetColor(c);
}

#endif
