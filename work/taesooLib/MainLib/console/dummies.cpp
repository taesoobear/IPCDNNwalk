#include "stdafx.h"
#ifdef NO_GUI
#include "dummies.h"


#ifdef NO_OGRE
Ogre::ColourValue Ogre::ColourValue::White;
#endif



void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name)
{
}
int Fl_Button::value(int v) 
{
  v = v ? 1 : 0;
  if (value_ != v) {
    value_ = v;
    return 1;
  } else {
    return 0;
  }
}


int Fl_Menu_::value() const {return (int)(value_-menu_);}
 int Fl_Menu_::value(const Fl_Menu_Item* m){return 0;}
  int Fl_Menu_::value(int i) {return value(menu_+i);}
#endif
