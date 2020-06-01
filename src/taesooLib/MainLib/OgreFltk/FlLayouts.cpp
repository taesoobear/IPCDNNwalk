#include "stdafx.h"
#ifndef NO_GUI
#include "FlLayouts.h"

FlLayouts::FlLayouts (int w, int h, int nLayout, const char* title)
:Fl_Double_Window(w, h, title),m_nLayout(nLayout), mScroll(w-20,0, 20,h)
{	
}

FlLayouts ::FlLayouts (int x, int y, int w, int h, int nLayout, const char* title)
:Fl_Double_Window(x,y,w,h,title),m_nLayout(nLayout), mScroll(w-20,0, 20,h)
{
	connect(&mScroll,-1);
} 

FlLayouts ::~FlLayouts (void)
{
}
 
/***********************************************************************
usage

o=new FlLayouts(....,3);
 o->layout(0, new FlLayout(...));
 o->layout(1, new FlLayout(...));
 o->layout(2, new FlLayout(...));
 */

void FlLayouts ::layout(int i, const char* winName, FlLayout* pWnd)
{
	int pos=aWnd.size();
	if(pWnd->parent()!=(Fl_Group*)this) Msg::error(" Parent is not this (FlLayouts)");
	if(pos!=0 && pWnd->parent()!=aWnd[pos-1]->parent()) Msg::error(" Parent is different");
    
	pWnd->end();

    aWnd.push_back(pWnd);
	aBorder.push_back(new Fl_Group(10,10,w()-20, 20, winName));
	aBorder[aBorder.size()-1]->end();

	// small size
	Fl_Check_Button* o=new Fl_Check_Button(10, 10, w()-20, 20);
	o->labelsize(10);
	o->value(1);
	connect(o, pos);
	
	aButton.push_back(o);

	if(pos==m_nLayout-1)
	{
		end();
		updateLayouts();
	}
}

void FlLayouts ::updateLayouts()
{
	int totalHeight=0;
	for(int i=0; i<aWnd.size(); i++)
	{
		totalHeight+=30+aWnd[i]->minimumHeight()+10;
	}

	int totalWidth;
	if(totalHeight>h())
	{
		mScroll.set_visible();
		totalWidth=w()-10;
		mScroll.resize(w()-10,0,10, h());
		mScroll.range(0, totalHeight);

		int newValue=mScroll.value();
		newValue=CLAMP(newValue, 0, totalHeight-h());
		//mScroll.value(totalHeight/3, totalHeight/3, 0, totalHeight);//h());		

		mScroll.value(newValue, h(), 0, totalHeight);
	}
	else
	{
		mScroll.value(0, h(), 0, h());
		mScroll.clear_visible();
		totalWidth=w();
	}

	int curx=0;
	int cury=mScroll.value()*-1;
	
	for(int i=0; i<aWnd.size(); i++)
	{
		aBorder[i]->box(FL_THIN_UP_FRAME);
		
		int minH=aWnd[i]->minimumHeight();

		aBorder[i]->resize(curx+5, cury+25, totalWidth-10, minH+10);
		aBorder[i]->labelsize(11);
		aButton[i]->resize(totalWidth-50, cury+10, 15, 15);
		cury+=30;
		aWnd[i]->resize(curx+10, cury, totalWidth-20, minH); 
		cury+=minH+10;		
	}
	redraw();
}

void FlLayouts ::resize(int X,int Y,int W,int H)
{
	Fl_Double_Window::resize(X,Y,W,H); 
	updateLayouts();
}

void FlLayouts ::onCallback(Fl_Widget* pWidget, int userData)
{
	if(userData==-1)
	{
		updateLayouts();
	}
}
#endif