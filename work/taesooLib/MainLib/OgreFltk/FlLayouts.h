#ifndef NO_GUI
#pragma once

#include "FlLayout.h"
#include "FltkAddon.h"
#include "FL/Fl_Scrollbar.H"

// deprecated. FlLayout이 child Layout을 지원함.(아직은 임의의 레이아웃 상속클래스를 사용할 수 없다는 제약이 있지만..쉽게 추가할수 있으니까..)
class FlLayouts  : public Fl_Double_Window, public FlCallee
{	
public:
	FlLayouts (int w, int h, int nLayout, const char* title=NULL);
	FlLayouts (int x, int y, int w, int h, int nLayout, const char* title=NULL);
	~FlLayouts (void);
 
	/***********************************************************************
	usage

	o=new FlLayouts(....,3);
	 o->layout(0, new FlLayout(...));
	 o->layout(1, new FlLayout(...));
	 o->layout(2, new FlLayout(...));
	 */

	void layout(int i, const char* winName, FlLayout* pWnd);
	
	int numLayout()	const	{ return aWnd.size();}
	FlLayout* layout(int i) { return aWnd[i];}

	void updateLayouts();
	virtual void resize(int x,int y,int w,int h);

private:

	virtual void onCallback(Fl_Widget* pWidget, int userData);
	int m_nLayout;
	std::vector<FlLayout*> aWnd;
	std::vector<Fl_Check_Button*> aButton;
	std::vector<Fl_Group*> aBorder;	
	Fl_Scrollbar mScroll;
};
#endif
