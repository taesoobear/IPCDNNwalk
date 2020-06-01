/* example
		vector3 from1(0,90,0);
        Circle *temp = new Circle(RE::renderer());
		
		temp->SetCircle(from1, 30, 30);
		temp->update();
*/

#ifndef __CIRCLE_H__ 
#define __CIRCLE_H__
#ifndef NO_OGRE
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000_H__ 

#include "LineStrip.h"

#define PI 3.141592

class Circle
{
public :

	Circle(const OgreRenderer& renderer);
	~Circle();

	void SetCircle(vector3& center, float radius, int n_controlpoint);
	void SetColor(RE::Color c);

	void SetVisible(bool value);

protected :
	LineStrip *circle;
};
#endif
#endif
