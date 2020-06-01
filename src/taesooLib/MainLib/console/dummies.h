#ifndef DUMMIES_H_
#define DUMMIES_H_
#ifdef NO_GUI


class MotionPanel
{
public:
  MotionPanel(int x, int y, int w, int h){}


};

#ifdef NO_OGRE
namespace Ogre
{
class FrameListener
{
public:
};

class FrameEvent
{
public:
          m_real timeSinceLastFrame;

};
class Light
{
public:
};
class AnimationState
{
public:
};
  class SceneManager
  {
  public:
  };
class Node
{
	public:
		Ogre::Node* getParent() { return NULL;}
};
  class SceneNode : public Node
  {
    public:
  };
  class MovableObject
  {
  public:
  };
  class Entity:public MovableObject
  {
  public:
  };

  class ColourValue 
  {
  public:
    int c;
    static ColourValue White;
  };

  class SimpleRenderable 
  {
    public:
  };
}

#endif
class Fl_Group;

class Fl_Widget
{
public:  
  Fl_Widget (){}
  Fl_Widget(int,int,int,int, const char*a=0){}
  virtual~Fl_Widget(){}
  Fl_Group* parent_;
  void* user_data_;
  short x_,y_,w_,h_;

  Fl_Group* parent() const {return parent_;}
  void parent(Fl_Group* p) {parent_ = p;} // for hacks only, Fl_Group::add()
  void* user_data() const {return user_data_;}
  void user_data(void* v) {user_data_ = v;}
  
  virtual void resize(int,int,int,int){}
  bool visible() {return true;}
  void show(){}
  void hide(){}
  void redraw(){}
  void copy_label(const char* a){}

  void set_visible(){}
  void clear_visible(){}
  void x(int v) {x_ = (short)v;}
  void y(int v) {y_ = (short)v;}
  void w(int v) {w_ = (short)v;}
  void h(int v) {h_ = (short)v;}
  int x() const {return x_;}
  int y() const {return y_;}
  int w() const {return w_;}
  int h() const {return h_;}
};

class Fl_Group: public Fl_Widget 
{
public:
  Fl_Group(){}
  Fl_Group(int,int,int,int, const char* = 0){}
  Fl_Group(int,int, const char* = 0){}
  void begin(){}
  void end(){}
  
};

class Fl_Window : public Fl_Group 
{
public:
  Fl_Window(){}
  Fl_Window(int,int,int,int, const char* = 0){}
  Fl_Window(int,int, const char* = 0){}
};
class Fl_Double_Window : public Fl_Window
{

public:

  Fl_Double_Window(int,int,int,int, const char* = 0){}
  Fl_Double_Window(int,int, const char* = 0){}

};

class Fl_Button:public Fl_Widget
{
public:
  Fl_Button(){}
  Fl_Button(int,int,int,int, const char*a=0){}
  char value_;
  int value(int);
  char value() const {return value_;}
};
class Fl_Light_Button: public Fl_Button
{
public:
  Fl_Light_Button(){}
Fl_Light_Button(int,int,int,int, const char*a=0){}
  
};
class Fl_Check_Button:public Fl_Light_Button
{
public:
Fl_Check_Button(int,int,int,int, const char*a=0){}
};
class Fl_Valuator : public Fl_Widget
{
public:
  Fl_Valuator (){}
  double value_;
  double value() const {return value_;}
  int value(double v){if (v == value_) return 0;  value_ = v;  return 1;}
  void range(double a,double b){}
void step(double s){}

};
class Fl_Slider:public Fl_Valuator 
{
public:
  Fl_Slider(){}
  Fl_Slider(int X, int Y, int W, int H, const char* l=0){}
};
class Fl_Value_Slider:public Fl_Slider
{
public:
  Fl_Value_Slider(int X, int Y, int W, int H, const char* l=0){}
};
class Fl_Menu_Item: public Fl_Widget
{
public:
    const char *text;	// label()
   int flags;
};
class Fl_Menu_:public Fl_Widget 
{
  Fl_Menu_Item *menu_;
  const Fl_Menu_Item *value_;
public:
 int value() const ;
 int value(const Fl_Menu_Item* m);
  int value(int i);
};
class Fl_Choice:public Fl_Menu_
{
public:
  Fl_Choice(int x, int y, int w, int h, const char* l=0){}
};

enum { // values for flags:
  FL_MENU_INACTIVE = 1,
  FL_MENU_TOGGLE= 2,
  FL_MENU_VALUE = 4,
  FL_MENU_RADIO = 8,
  FL_MENU_INVISIBLE = 0x10,
  FL_SUBMENU_POINTER = 0x20,
  FL_SUBMENU = 0x40,
  FL_MENU_DIVIDER = 0x80,
  FL_MENU_HORIZONTAL = 0x100
};


enum Fl_Event {	// events
  FL_NO_EVENT		= 0,
  FL_PUSH		= 1,
  FL_RELEASE		= 2,
  FL_ENTER		= 3,
  FL_LEAVE		= 4,
  FL_DRAG		= 5,
  FL_FOCUS		= 6,
  FL_UNFOCUS		= 7,
  FL_KEYDOWN		= 8,
  FL_KEYUP		= 9,
  FL_CLOSE		= 10,
  FL_MOVE		= 11,
  FL_SHORTCUT		= 12,
  FL_DEACTIVATE		= 13,
  FL_ACTIVATE		= 14,
  FL_HIDE		= 15,
  FL_SHOW		= 16,
  FL_PASTE		= 17,
  FL_SELECTIONCLEAR	= 18,
  FL_MOUSEWHEEL		= 19,
  FL_DND_ENTER		= 20,
  FL_DND_DRAG		= 21,
  FL_DND_LEAVE		= 22,
  FL_DND_RELEASE	= 23
};
#endif
#endif
