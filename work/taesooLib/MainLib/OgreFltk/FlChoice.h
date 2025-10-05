#ifndef FLChoice_H_
#define FLChoice_H_
#include "FltkAddon.h"
#ifndef NO_GUI
#include <Fl/Fl_Choice.H>
#include <Fl/Fl_Window.H>
#include <Fl/Fl_Browser.H>
#endif
class FlChoice : public Fl_Choice
{
public:
	FlMenu menu;

	/*******************************************
	* Static allocation
	*	static Fl_Menu_Item menu[] = {
	*	// text, shortcut, callback_, user_data_, flags...
	*	{"left+top", 0, cbFunc, data1},
	*	{"left+bottom", 0, cbFunc, data2},
	*	{"right+top", 0, cbFunc, data3},
	*	{"right+bottom", 0, cbFunc, data4},
	*	{0}
	*	};
	*   (new Fl_Choice(0,0,100,20))->copy(menu); -> complicated code.
	*
	* Dynamic Allocation
	*	FlChoice* menu=new FlChoice(0,0,100,20);
	*
	*   menu->size(4);l
	*	menu->item(0, "left+top", 0, data1);
	*	menu->item(1, "left+bottom", 0, data2);
	*	menu->item(2, "right+top", 0, data3);
	*	menu->item(3, "right+bottom", 0, data4);
	*
	*
	* 필요할때마다 update하려면, size(4); item(0..3); redraw(); 이과정을 해주면 된다.
	*/

	FlChoice(int x, int y, int w, int h, const char* l=0)
		:Fl_Choice(x,y,w,h,l)
	{
		menu.init(this);
	}

	void size(int n)	{ menu.size(n);}
	int size()			{ return menu.size();}

	void item(int item, const char* text, int shortcut_=0,  int user_data_=0,  int flags=0,  unsigned char labeltype_=0,  unsigned char labelfont_=0, unsigned char labelsize_=0,  unsigned labelcolor_=0)
	{
		menu.item(item, text, shortcut_, user_data_, flags, labeltype_, labelfont_, labelsize_, labelcolor_);
	}

	Fl_Menu_Item& item(int item)
	{
		return menu.m_aMenuItem[item];
	}

	void beginSubMenu(int item, const char* text);
	void endSubMenu(int item);

};
/**
	This class should be instantiated dynamically so that the o->end() can be called after windows are created..
	o->end() will be automatically called when the last window is attached.

 o=new FlChoiceWins(....,3);
 o->window(0, new Fl_Window(...));
 o->window(1, new Fl_Window(...));
 o->window(2, new Fl_Window(...));

 */
class FlChoiceWins : public Fl_Group, public FlCallee
{
public:

	FlChoiceWins(int x, int y, int w, int h, int nWindow, const char* MainMenu=0);
	~FlChoiceWins(){}
	void window(int i, const char* winName, Fl_Group* pWnd);

	void show(int n);
	int numWindow()	const	{ return (int)aWnd.size();}
	Fl_Group* window(int i) { return aWnd[i];}
	virtual void resize(int x,int y,int w,int h);
	TStrings windowNames;
private:
	virtual void onCallback(Fl_Widget* pWidget, int userData);
	std::vector<Fl_Group*> aWnd;
	FlMenu m_menuWin;
	Fl_Group* m_pMenuWin;
	Fl_Scroll* m_pScroll;
	int m_nWindow;
	int m_nCurrShow;
};
/**
	This class should be instantiated dynamically so that the o->end() can be called after windows are created..
	o->end() will be automatically called when the last window is attached.

 o=new FlChoiceWins(....,3);
 o->window(0, new Fl_Window(...));
 o->window(1, new Fl_Window(...));
 o->window(2, new Fl_Window(...));

 */
class FlTreeChoiceWins : public Fl_Window, public FlCallee
{
public:

	FlTreeChoiceWins (int x, int y, int w, int h, int nWindow);
	~FlTreeChoiceWins (){}
	void window(int i, const char* winName, Fl_Window* pWnd);

	void show(int n);
	int numWindow()	const	{ return (int)aWnd.size();}
	Fl_Window* window(int i) { return aWnd[i];}
private:

	virtual void onCallback(Fl_Widget* pWidget, int userData);
	std::vector<Fl_Window*> aWnd;
	std::vector<Fl_Button*> aButton;
	FlMenu m_menuWin;
	int m_nWindow;
	int m_nCurrShow;
};

#endif
