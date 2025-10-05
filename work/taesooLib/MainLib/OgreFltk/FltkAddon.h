#ifndef _FLTKADDON_H_
#define _FLTKADDON_H_

#if _MSC_VER>1000
#pragma once

//#pragma message("Compiling FltkAddon.h - this should happen just once per project (when using precompiled header).\n")
#endif

class FlMenu;
class Fl_Scroll;
class Fl_Widget;
class Fl_Button;
class Fl_Check_Button;
class Fl_Light_Button;
class Fl_Slider;
class Fl_Menu_Item;
/// FlChooseFile("choose", "../resource/classification", "*.asf");
TString FlChooseFile(const char* message, const char* path, const char* Mask, bool bCreate=false);


/// Fltk EventReciver may inherit this class. I assumed that all widgets in the same group are connected to a single FlCallee.
class FlCallee
{
public:
	FlCallee(){};
	virtual~FlCallee(){};

	/** Usage: connect(new Fl_Button(0,0,80,20,"SomeButton"), 100);

	 parent가 갖고 있는 모든 widget과 메뉴는 같은 callee에 연결됨을 가정하였다.
	 실제적으로 구현할때는 parent자체가 callee가 되도록 하는게 가장 확실하다.
	 즉 parent는  Fl_Group(또는 Fl_Window 등)과 FlCallee를 동시상속.
	*/
	void connect(Fl_Widget* pWidget, int userData=-1);
	void connect(FlMenu& menu);

	// utility functions
	Fl_Widget* findWidget(int userData);
	Fl_Button* findButton(int userData) {return (Fl_Button*)(mWidgets[userData]);}
	Fl_Check_Button* findCheckButton(int userData);
	Fl_Light_Button* findLightButton(int userData)	{return (Fl_Light_Button*)(mWidgets[userData]);}
	Fl_Slider* findSlider(int userData)				{return (Fl_Slider*)(mWidgets[userData]);}

protected:
	/// you can distinguish the caller based on pWidget, pWidget->label() or userData.
	virtual void onCallback(Fl_Widget * pWidget, int userData)=0;

private:
	// map by userData
	std::map<int, Fl_Widget*> mWidgets;

	friend class FlMenu;
	static void cbFunc(Fl_Widget * pWidget, void *data);
};

/// Fltk EventReciver may inherit this class.
class Fl_Callee
{
public:
	Fl_Callee(){};
	virtual~Fl_Callee(){};

	/** Usage: connect(new Fl_Button(0,0,80,20,"SomeButton"), 0);

	FlCallee클래스와 달리 어떤 가정도 사용하지 않았다. 그대신 유저 데이타를 사용할 수 없다.
	편의상 콜백펑션을 미리 정의된 5개중에 한개를 사용할수 있다.
	(callBackID 는 {0,1,2,3,4} 중에 하나.)
	*/

	void connect(Fl_Widget* pWidget, int callBackID=0);

protected:
	/// you can distinguish the caller based on pWidget, pWidget->label(), or callBackID.
	virtual void onCallback(Fl_Widget * pWidget, int callBackID)=0;

private:
	static void cbFunc0(Fl_Widget * pWidget, void *data);
	static void cbFunc1(Fl_Widget * pWidget, void *data);
	static void cbFunc2(Fl_Widget * pWidget, void *data);
	static void cbFunc3(Fl_Widget * pWidget, void *data);
	static void cbFunc4(Fl_Widget * pWidget, void *data);
};

class Fl_Menu_;

/// Dynamically allocated menu. (Each instance can be connected to different FlCallee.
class FlMenu
{
public:
	FlMenu();
	~FlMenu();

	void init(Fl_Menu_* pMenu, int n);
	void init(Fl_Menu_* pMenu);
	void initChoice(int x, int y, int w, int h, const char* title=NULL);

	void size(int n);
	int size();

	void item(int item, const char* text, int shortcut_=0,  int user_data_=0,  int flags=0,  unsigned char labeltype_=0,  unsigned char labelfont_=0, unsigned char labelsize_=0,  unsigned labelcolor_=0);

	Fl_Menu_Item& item(int item);
	void beginSubMenu(int item, const char* text);
	void endSubMenu(int item);

	bool isValidItem(int item);

	Fl_Menu_& menu();
	TString const& label(int iItem) const;
	TString const& label()	const	;
	bool operator==(Fl_Widget* pWidget) const	;

	// get user data of the current choice.
	int userData();
	int userData(int n)	;
	void value(int n);
	int value()	const;
	void redraw();
private:
	friend class FlCallee;
	friend class FlLayout;
    int m_nMenu;
	TStrings m_aLabel;
	Fl_Menu_* m_pMenu;
	Fl_Menu_Item* m_aMenuItem;

	friend class FlChoice;
};

class CImage;
struct TRect;
void fl_draw_CImage(const CImage& imagee, const TRect& sourceRect, int x, int y);
#endif
