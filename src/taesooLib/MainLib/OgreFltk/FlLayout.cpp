#include "stdafx.h"
#include "FlLayout.h"
#ifndef NO_GUI
#include "FL/Fl_Scroll.H"
#include "FL/Fl_Adjuster.H"
#include "FL/Fl_Multi_Browser.H"
#include "FL/Fl_Select_Browser.H"
#include "FL/Fl_Multiline_Input.H"
#include "FL/Fl_Box.H"
#include "FL/Fl_Progress.H"
#endif
#include "FlChoice.h"

#ifdef NO_GUI
#define FL_VOID(x)
#else
#define FL_VOID(x) x
#endif

#include "../BaseLib/math/Operator.h"
#include "MotionPanel.h"
#include "FltkMotionWindow.h"
#include "../WrapperLua/luna.h"
#include "RE.h"


int mScaleFactor=1; // for backward compatibility.
float mfScaleFactor=1.f;
FlLayout ::Widget::Widget()
{
	mWidget=NULL;
	mType="None";
	mState.mButtonType=FlLayout::BUTTON;	// default type
	mState.mSliderType=FlLayout::VALUE_SLIDER; // default type
	mState.mGuideLines.linspace(0.0, 1.0, 4);
	mState.mStartSlot=0;
	mState.mEndSlot=INT_MAX;
	mState.mWidgetHeight=20*mfScaleFactor;
	mState.mLineSpace=5*mfScaleFactor;
	mState.mHorizSpace=5*mfScaleFactor;
}

FlLayout ::FlLayout ()
#ifdef __APPLE__
:Fl_Group(0,0,640,480)
#else
:Fl_Double_Window(640,480)
#endif
{
	Msg::error("do not use FlLayout's default constructor");
}

#ifdef __APPLE__
#define RIGHT_PADDING 0
#define SCROLLW 10
#define LEFT_MARGIN 5
#else
#define RIGHT_PADDING 0
#define SCROLLW 5
#define LEFT_MARGIN 5
#endif
FlLayout ::FlLayout (int ww, int hh, const char* title)
#ifdef __APPLE__
:Fl_Group(0,0,ww,hh)
#else
:Fl_Double_Window(ww, hh, title)
#endif
{
	_callbackRouter=this;
	mWidgets.resize(1);
	mWindow=new Fl_Double_Window(0,0,ww-SCROLLW*mfScaleFactor, hh);
	mWindow->end();
	begin();
#ifndef NO_GUI
	mScrollbar=new Fl_Scrollbar(ww-SCROLLW*mfScaleFactor,0, SCROLLW*mfScaleFactor, hh);
	mScrollbar->parent()->user_data((void*)this);
	mScrollbar->callback(cbFunc, NULL);
	mScrollbar->hide();
	end();
	resizable(mWindow);
#endif
}

FlLayout ::FlLayout (int x, int y, int ww, int hh, const char* title)
#ifdef __APPLE__
	:Fl_Group(x, y, ww, hh, title)
#else
	:Fl_Double_Window(x, y, ww, hh, title)
#endif
{
	_callbackRouter=this;
	mWidgets.resize(1);
#ifdef __APPLE__
	mWindow=new Fl_Double_Window(x,y,ww-SCROLLW*mfScaleFactor+RIGHT_PADDING, hh);
#else
	mWindow=new Fl_Double_Window(0,0,ww-SCROLLW*mfScaleFactor+RIGHT_PADDING, hh);
#endif
	mWindow->end();
	begin();
#ifndef NO_GUI
#ifdef __APPLE__
	mScrollbar=new Fl_Scrollbar(x+ww-SCROLLW*mfScaleFactor,0, SCROLLW*mfScaleFactor, hh);
#else
	mScrollbar=new Fl_Scrollbar(ww-SCROLLW*mfScaleFactor,0, SCROLLW*mfScaleFactor, hh);
#endif
	mScrollbar->parent()->user_data((void*)this);
	mScrollbar->callback(cbFunc, NULL);
	mScrollbar->hide();
	end();

	resizable(mWindow);
#endif
}

Fl_Widget* FlLayout::create(const char* type, const char* id, const char* title, int startSlot, int endSlot, int height)
{

	setWidgetPos(startSlot, endSlot);
	return createWidget(type, id, title);
}

FlLayout ::~FlLayout (void)
{

}

void FlLayout ::resize(int x,int y,int w,int h)
{
  #ifndef NO_GUI

#ifdef __APPLE__
	Fl_Group::resize(x,y,w,h);
#else
	Fl_Double_Window::resize(x,y,w,h);
#endif
#endif
	updateLayout();

}

vectorn& FlLayout::guideLines()
{
	return widgetRaw(1).mState.mGuideLines;
}

void FlLayout::setWidgetPos(int startSlot, int endSlot)
{
	if(endSlot>widgetRaw(1).mState.mGuideLines.size()-1)
		endSlot=widgetRaw(1).mState.mGuideLines.size()-1;
	widgetRaw(1).mState.mStartSlot=startSlot;
	widgetRaw(1).mState.mEndSlot=endSlot;
}

void FlLayout::setUniformGuidelines(int totalSlot)
{
	guideLines().linspace(0, 1.0, totalSlot+1);
}

void FlLayout::setWidgetPosUniform(int totalSlot, int slot)
{
	// 가로로 totalSlot등분을 해서, 몇번째에 넣어라.
	guideLines().linspace(0, 1.0, totalSlot+1);
	setWidgetPos(slot, slot+1);
}

Fl_Button* FlLayout::createButton(const char* id, const char* title)
{
	Fl_Button* b=NULL;
	switch(widgetRaw(1).mState.mButtonType)
	{
	case CHECK_BUTTON:
		b= (Fl_Button*) create("Check_Button", id, title);
		break;
	case LIGHT_BUTTON:
		b= (Fl_Button*) create("Light_Button", id, title);
		break;
	case BUTTON:
		b= (Fl_Button*) create("Button", id, title);
		break;
	}
	return b;
}
Fl_Slider* FlLayout::createSlider(const char* id, const char* title)
{
	switch(widgetRaw(1).mState.mSliderType)
	{
	case SLIDER:
		return (Fl_Slider*) create("Slider", id, title);
	case VALUE_SLIDER:
		return (Fl_Slider*) create("Value_Slider", id, title);
	case VALUE_VERT_SLIDER:
		return (Fl_Slider*) create("Vert_Slider", id, title);
	}
	return NULL;
}
#ifndef NO_GUI

class Fl_new_adjuster: public Fl_Adjuster
{
public:
	Fl_Box b;
	char buf2[100];
	Fl_new_adjuster(int X, int Y, int W, int H, const char* L=0)
		:Fl_Adjuster(X,Y,W/2,H, L)
		,b(FL_DOWN_BOX, X+W/2,Y, W/2,H,buf2)
	{
		b.labelsize(11*mfScaleFactor);

		align(FL_ALIGN_LEFT);
		step(1);
		value_damage();
	}

	virtual void resize(int x,int y,int w,int h)
	{
		b.resize(x+w/2,y,w/2,h);
		Fl_Adjuster::resize(x,y,w/2,h);
	}
	virtual void value_damage()
	{
		Fl_Adjuster::value_damage();

		if(value()<minimum())
			value(minimum());
		if(value()>maximum())
			value(maximum());
		format((char *)(b.label()));
		redraw();
	}

	virtual void redraw()
	{
		format((char *)(b.label()));
		b.redraw();
		Fl_Adjuster::redraw();
	}
};

class Fl_Frame_Adjuster: public Fl_Adjuster, public Fl_Callee
{
public:
	Fl_Box b;
	Fl_Button c;
	Fl_Button d;
	char buf2[100];
	Fl_Frame_Adjuster(int X, int Y, int W, int H, const char* L=0)
		:Fl_Adjuster(X,Y,W/2,H, L)
		,b(FL_DOWN_BOX, X+W/2,Y, W/2-40,H,buf2)
		,c(X+W-40,Y,20,H,"@<-")
		,d(X+W-20,Y,20,H,"@->")
	{
		b.labelsize(11*mfScaleFactor);

		align(FL_ALIGN_LEFT);
		step(1);
		value_damage();
		connect(&c, 0);
		connect(&d, 1);
	}
	virtual void onCallback(Fl_Widget * pWidget, int callBackID)
	{
		range(0, RE::motionPanel().currMotion().numFrames()-1);

		if(callBackID==0)
			value(RE::motionPanel().motionWin()->getCurrFrame());
		else
			RE::motionPanel().motionWin()->changeCurrFrame(value());
		redraw();
	}
	virtual void resize(int x,int y,int w,int h)
	{
		b.resize(x+w/2,y,w/2-40,h);
		c.resize(x+w-40,y,20,h);
		d.resize(x+w-20,y,20,h);
		Fl_Adjuster::resize(x,y,w/2,h);
	}
	virtual void value_damage()
	{
		Fl_Adjuster::value_damage();

		if(value()<minimum())
			value(minimum());
		if(value()>maximum())
			value(maximum());
		format((char *)(b.label()));
		redraw();
	}

	virtual void redraw()
	{
		format((char *)(b.label()));
		b.redraw();
		c.redraw();
		Fl_Adjuster::redraw();
	}
};


#endif

int FlLayoutGroup ::numLayouts()
{
	return layouts.size();
}

FlLayout* FlLayoutGroup ::layout(int ilayout)
{
	return layouts[ilayout];
}

void FlLayoutGroup ::showLayout(int ilayout)
{

	for(int i=0; i<numLayouts(); i++)
	{
		if(layout(i)->visible() && i!=ilayout)
			layout(i)->hide();
		if(!layout(i)->visible() && i==ilayout)
			layout(i)->show();
	}
	mCurrLayout=ilayout;
}

FlLayoutGroup ::FlLayoutGroup (int x, int y, int w, int h, const char* l)
	:Fl_Group(x,y,w,h,l)
{
#ifndef NO_GUI
  box(FL_THIN_UP_FRAME);
#endif
	layouts.resize(1);
	layouts[0]=new FlLayout(x,y,w,h);
#ifndef NO_GUI
	resizable(NULL);
#endif
	end();

	mCurrLayout=0;
	showLayout(0);
}

FlLayoutGroup ::FlLayoutGroup (int x, int y, int w, int h, FlLayout* layout, const char* l)
	:Fl_Group(x,y,w,h,l)
{
#ifndef NO_GUI
	box(FL_THIN_UP_FRAME);
#endif
	layouts.resize(1);
	layouts[0]=layout;
#ifndef NO_GUI
	insert(*layout, 0);
	resizable(NULL);
#endif
	end();

	mCurrLayout=0;
	showLayout(0);

}

FlLayoutGroup ::FlLayoutGroup (int x, int y, int w, int h, std::vector<FlLayout*> const& layout, const char* l)
	:Fl_Group(x,y,w,h,l)
{
#ifndef NO_GUI
	box(FL_THIN_UP_FRAME);
#endif
	layouts.resize(layout.size());
	for(int i=0; i<layouts.size(); i++)
	{
		layouts[i]=layout[i];
#ifndef NO_GUI
		insert(*layout[i], 0);
#endif
	}
#ifndef NO_GUI
	resizable(NULL);
#endif
	end();

	mCurrLayout=0;
	showLayout(0);

}
void FlLayoutGroup::updateLayout()
{
	for(int i=0; i<numLayouts() ;i++)
		layout(i)->updateLayout();
}

int FlLayoutGroup::minimumHeight()
{
	int minHeight=0;

	for(int i=0; i<numLayouts(); i++)
	{
		int mm=layout(i)->minimumHeight();
		minHeight=MAX(minHeight, mm);
	}
	return minHeight;
}
void FlLayoutGroup::resize(int x,int y,int w,int h)
{
#ifndef NO_GUI
	Fl_Group::resize(x,y,w,h);
	for(int i=0; i<numLayouts() ;i++)
		layout(i)->resize(x+LEFT_MARGIN*mfScaleFactor, y+5*mfScaleFactor, w-(LEFT_MARGIN+SCROLLW)*mfScaleFactor, h-10*mfScaleFactor);
#endif
}

void FlLayout::embedLayout(FlLayout* childLayout, const char* id, const char* title)
{
	begin();
	mWindow->begin();
	Fl_Widget* o=NULL;

	widgetRaw(1).mType="Layout";

	FlLayoutGroup * g = new FlLayoutGroup (0,0,80,20, childLayout);
	o=g;
	o->copy_label(title);
	_createWidget(id, o);
	mWindow->end();
}

void FlLayout::embedLayouts(std::vector<FlLayout*> const& childLayouts,const char* id, const char* title)
{
	begin();
	mWindow->begin();
	Fl_Widget* o=NULL;

	widgetRaw(1).mType="LayoutGroup";

	FlLayoutGroup * g = new FlLayoutGroup (0,0,80,20, childLayouts);
	o=g;
	o->copy_label(title);
	_createWidget(id, o);
	mWindow->end();
}

Fl_Widget* FlLayout::create(const char* type, const char* id, const char* title)
{
	begin();
	mWindow->begin();
	TString tid=type;
	Fl_Widget* o=NULL;

	if(tid=="Menu")
		tid="Choice";

	widgetRaw(1).mType=tid;

	if(tid=="Layout")
	{
		FlLayoutGroup * g = new FlLayoutGroup (0,0,80,20);
		g->layout()->_callbackRouter=_callbackRouter;
		o=g;
	}
	else if(tid=="LayoutGroup")
	{
		FlLayoutGroup* g= new FlLayoutGroup(0,0,80,20);
		o=g;
	}
	else if(tid=="Box")
	{
#ifndef NO_GUI
	  Fl_Box* b;
		b=new Fl_Box(0,0,80,20);
		o=b;
#else
		o=new Fl_Button(0,0,80,20);
#endif
	}
	else if(tid=="Input")
	{
#ifndef NO_GUI
	  Fl_Input* i;
		i=new Fl_Input(0,0,80,20);
		o=i;
#endif
	}
	else if(tid=="Adjuster")
	{
#ifndef NO_GUI
		Fl_new_adjuster* a;
		a=new Fl_new_adjuster(0,0,80,20);
		o=a;
#else
	    o=new Fl_Valuator ();
#endif
	}
	else if(tid=="Progress")
	{
#ifndef NO_GUI
		Fl_Progress* a;
		a=new Fl_Progress(0,0,80,20);
		o=a;
#else
	    o=new Fl_Valuator ();
#endif
	}
	else if(tid=="Frame_Adjuster")
	{
#ifndef NO_GUI
		o=new Fl_Frame_Adjuster(0,0,80,20);
#else
	    o=new Fl_Valuator ();
#endif
	}
	else if(tid=="Check_Button")
	{
		o=new Fl_Check_Button(0,0,80,20);
	}
	else if(tid=="Light_Button")
	{
		o=new Fl_Light_Button(0,0,80,20);
	}
	else if(tid=="Button")
	{
		o=new Fl_Button(0,0,80,20);
	}
	else if(tid=="Choice")
	{
		o=new FlChoice(0,0,80,20);
	}
	else if(tid=="Multi_Browser")
	{
		FL_VOID(o=new Fl_Multi_Browser(0,0,80,20));
	}
	else if(tid=="Select_Browser")
	{
		FL_VOID(o=new Fl_Select_Browser(0,0,80,20));
	}
	else if(tid=="Multiline_Input")
	{
		FL_VOID(o=new Fl_Multiline_Input(0,0,80,20));
	}
	else if(tid=="Slider")
	{
		o=new Fl_Slider(0,0,80,20);
#ifndef NO_GUI
		o->type(FL_HOR_SLIDER);
		o->align(FL_ALIGN_LEFT);
#endif
	}
	else if(tid=="Vert_Slider")
	{
		o=new Fl_Value_Slider(0,0,80, 20);
#ifndef NO_GUI
		o->type(FL_VERT_SLIDER);
		o->align(FL_ALIGN_LEFT);
#endif
	}
	else if(tid=="Value_Slider")
	{
		o=new Fl_Value_Slider(0,0,80,20);
#ifndef NO_GUI
		o->type(FL_HOR_SLIDER);
		o->align(FL_ALIGN_LEFT);
#endif
	}
	if(!o)	Msg::error("Unknown type (%s)",type );
	o->copy_label(title);
	_createWidget(id, o);
	mWindow->end();
	return o;
}

void FlLayout::newLine()
{
	setWidgetPos(0);
	_createWidget("__empty", NULL);
}

FlChoice * FlLayout::createMenu(const char* id, const char* title)
{
	return (FlChoice*)create("Choice",id, title);
}

void FlLayout::updateLayout()
{
	int cury=5*mfScaleFactor;

	mWindow->begin();
	intvectorn guidelines;
	for(int i=0; i<mWidgets.size()-1; i++)
	{
		guidelines.setSize(mWidgets[i].mState.mGuideLines.size());

		//printf("cury %d %d %d\n",i, cury, guidelines.size());

		for(int j=0; j<guidelines.size(); j++)
			guidelines[j]=sop::interpolateInt(mWidgets[i].mState.mGuideLines[j], 3, w()-SCROLLW*mfScaleFactor-2);

		// widget간 5픽셀 간격이 생기도록 한다.

		//int gap=5;
		int gap=mWidgets[i].mState.mHorizSpace;
#define _left(i)	(guidelines[mWidgets[i].mState.mStartSlot]+gap/2)
#define _right(i) (guidelines[MIN(mWidgets[i].mState.mEndSlot, guidelines.size()-1)]-(gap-gap/2))
#define _currWidth(i)	   (_right(i)-_left(i))
#define _curHeight(i)		(mWidgets[i].mState.mWidgetHeight)


		/*
		// title 만큼 자동으로 offset 하는 기능은 없앴음. 수작업으로 적당히 widget pos를 조정해서 할 것.
		if((mWidgets[i].mType=="Slider" ||
			mWidgets[i].mType=="Choice" ||
			mWidgets[i].mType=="Input"||
			mWidgets[i].mType.right(8)=="Adjuster") && mWidgets[i].widget()->label())
		{
			int offset=60;
			if(mWidgets[i].mType=="Slider" && mWidgets[i].mState.mSliderType==VALUE_VERT_SLIDER)
				offset=0;

			Fl_Widget* o=(Fl_Widget*)mWidgets[i].mWidget;
			o->resize(_left(i)+offset, cury, _currWidth(i)-offset, _curHeight(i));
		}
		else */
		if(mWidgets[i].mType=="Layout" || mWidgets[i].mType=="LayoutGroup")
		{
			FlLayoutGroup* o=(FlLayoutGroup*)mWidgets[i].mWidget;

			_curHeight(i)=o->minimumHeight()+25*mfScaleFactor;
			if(o)
				o->resize(_left(i), cury+15*mfScaleFactor, _currWidth(i), _curHeight(i)-15*mfScaleFactor);

			o->updateLayout();
		}
		else
		{
			Fl_Widget* o=(Fl_Widget*)mWidgets[i].mWidget;
			if(o)
				o->resize(_left(i), cury, _currWidth(i), _curHeight(i));
		}

		if(mWidgets[i].mState.mEndSlot>=mWidgets[i].mState.mGuideLines.size()-1)
		{
			cury+=_curHeight(i)+mWidgets[i+1].mState.mLineSpace;
		}
	}

	mWindow->end();
	end();

	m_minimumHeight=cury;
	//printf("minh %d %d\n", m_minimumHeight, h());
#ifndef NO_GUI

	if(m_minimumHeight>h())
	{
		mScrollbar->show();
		mScrollbar->value(0, h(), 0, m_minimumHeight);
		mScrollbar->redraw();
	}
	else
	{
		mScrollbar->value(0, h(), 0, m_minimumHeight);
		mScrollbar->hide();
	}
	// connect
	for(int i=0; i<mWidgets.size()-1; i++)
	{
		// parent가 갖고 있는 모든 widget과 메뉴는 같은 layout에 연결됨을 가정하였다.
		// 실제적으로 구현할때는 parent자체가 callee가 되도록 하는게 가장 확실하다.
		// 즉 parent는  Fl_Group(또는 Fl_Window 등)과 FlLayout를 동시상속.

		Fl_Widget*o=mWidgets[i].mWidget;


		if(mWidgets[i].mType=="Choice")
		{
			FlMenu* o=&(((FlChoice*)mWidgets[i].mWidget)->menu);
			for(int item=0; item<o->m_nMenu; item++)
				o->m_aMenuItem[item].callback(cbFunc);
		}
	}
	mWindow->redraw();
#endif
}

int FlLayout::minimumHeight()
{
	return m_minimumHeight;
}
int addItemRecursive(int curItem, bool bSubMenu, FlMenu & menu, lunaStack& ll, int tblindex)
{
	int arraySize=ll.arraySize(tblindex);
	for(int i=1; i<=arraySize; i++)
	{
		ll.gettable(tblindex, i);
		if(lua_type(ll.L, -1)!= LUA_TTABLE)
		{
			TString menuTitle;
			ll.setPop();
			ll>>menuTitle;
			if(bSubMenu )
				menu.item(curItem, menuTitle, 0,0, FL_SUBMENU);
			else
				menu.item(curItem, menuTitle);

			//printf("%d: %s\n", curItem, menuTitle.ptr());
			curItem++;
		}
		else
		{
			curItem=addItemRecursive(curItem, true, menu, ll, ll.gettop());
			ll.pop();
			menu.item(curItem, 0);	// submenu닫기.
			curItem++;
		}

		//ll.printStack(true);
		bSubMenu=false;
	}
	return curItem;
}

int FlLayout::work(TString const& workname, lunaStack& L)
{
	if(workname=="checkButton")
	{
		std::string bn;
		bool bValue;
		L>>bn>>bValue;
		findCheckButton(bn.c_str())->value(int(bValue));
	}
	else if(workname=="menuAddItem")
	{
		std::string bn;
		L>>bn;
		FlChoice* choice=findMenu(bn.c_str());
		int tblindex=4;
		int numMenuItems=L.treeSize(tblindex)-1;
		/*printf("%d %d %d  %d %d\n", 
				numMenuItems,L.treeSize(1), 
				L.treeSize(2), 
				L.treeSize(3),
				L.treeSize(4)
				);
				*/
		choice->size(numMenuItems);
		addItemRecursive(0, false, choice->menu, L, tblindex);
		choice->value(0);
		choice->redraw();
	}
	else if(workname=="menu")
	{
		std::string bn;
		int eValue;
		L>>bn>>eValue;
		findMenu(bn.c_str())->value(eValue);
	}
	else if(workname=="valuator")
	{
		std::string bn;
		m_real value;
		L>>bn>>value;
		findValuator(bn.c_str())->value(value);
	}
	return 0;
}

Fl_Widget* FlLayout::_createWidget(const char* id, Fl_Widget* o)
{
  #ifndef NO_GUI

	if(o)
	{
		// Auto-size
		o->labelsize(11*mfScaleFactor);

		// connect
		void* prev_user_data=o->parent()->user_data();
		Msg::verify(prev_user_data==NULL || prev_user_data==((void*)this), "FlLayout::_createWidget error");
		o->parent()->user_data((void*)this);
		o->callback(cbFunc, o->user_data());
	}
#endif
	mWidgets.resize(mWidgets.size()+1);
	// state는 이전 state를 유지한다.
	mWidgets[mWidgets.size()-1].mState=mWidgets[mWidgets.size()-2].mState;

	mWidgets[mWidgets.size()-2].mId=id;
	mWidgets[mWidgets.size()-2].mWidget=o;
	mNamedmapWidgets[mWidgets[mWidgets.size()-2].mId]=mWidgets.size()-2;
	return o;
}

FlLayout::Widget& FlLayout::widgetRaw(int n)
{
	int iwidget=mWidgets.size()-2+n;
	if(iwidget<0)
		return mWidgets[mWidgets.size()-1];

	return mWidgets[iwidget];
}

int FlLayout::widgetIndex(const char* id)
{
	for(int i=0; i<mWidgets.size()-1; i++)
	{
		if(mWidgets[i].mId==id)
			return i-(mWidgets.size()-2); // 0 represents the last widget.
	}

	// 1 means no such widget
	return 1;
}

void FlLayout::removeWidgets(int startWidgetIndex)
{
	int start=startWidgetIndex+mWidgets.size()-2;

	if(start==mWidgets.size()-1) return;

	for(int i=start; i<mWidgets.size()-1; i++)
	{
		mNamedmapWidgets.erase(mWidgets[i].mId);
#ifndef NO_GUI
		mWindow->remove(mWidgets[i].mWidget);
#endif
		mWidgets[i].mWidget=NULL;
	}

	mWidgets.resize(start+1);
	mWidgets[mWidgets.size()-1]=Widget();

	updateLayout();
}

FlLayout::Widget& FlLayout::findWidget(const char* id)
{
#ifdef USE_FULL_SEARCH
	for(int i=0; i<mWidgets.size()-1; i++)
	{
		if(mWidgets[i].mId==id)
			return mWidgets[i];
	}

	Msg::error("no widget %s", id);
	return mWidgets[mWidgets.size()-1];
#else
	if (mNamedmapWidgets.find(id)==mNamedmapWidgets.end())
		Msg::error("no widget %s", id);

	return mWidgets[mNamedmapWidgets[id]];
#endif
}

FlLayout::Widget& FlLayout::_findWidget(Fl_Widget* id)
{
	for(int i=0; i<mWidgets.size()-1; i++)
	{
		if(mWidgets[i].mWidget==id)
			return mWidgets[i];
	}

	Msg::error("no widget %x", id);
	return mWidgets[mWidgets.size()-1];
}

float FlLayout::Widget::progressValue() const
{
#ifndef NO_GUI
	return dynamic_cast<Fl_Progress*>(mWidget)->value();
#else
	return 0;
#endif
}
void FlLayout::Widget::progressValue(float v) 
{
#ifndef NO_GUI
	dynamic_cast<Fl_Progress*>(mWidget)->value(v);
#endif
}
Fl_Slider* FlLayout::Widget::slider()const
{
	return dynamic_cast<Fl_Slider*>(mWidget);
}
Fl_Button* FlLayout::Widget::button()const
{
	return dynamic_cast<Fl_Button*>(mWidget);
}
Fl_Light_Button* FlLayout::Widget::checkButton()const
{
	return dynamic_cast<Fl_Light_Button*>(mWidget);
}

Fl_Valuator* FlLayout::Widget::valuator() const
{
	return dynamic_cast<Fl_Valuator*>(mWidget);
}

FlChoice* FlLayout::Widget::menu()const
{
	return dynamic_cast<FlChoice*>(mWidget);
}

FlLayout* FlLayout::Widget::layout() const
{
	FlLayoutGroup* g=dynamic_cast<FlLayoutGroup*>(mWidget);
	return (g)?g->layout():NULL;
}

FlLayoutGroup* FlLayout::Widget::layoutGroup() const
{
	FlLayoutGroup* g=dynamic_cast<FlLayoutGroup*>(mWidget);
	return g;
}

FlChoice* FlLayout::menu(int n)
{
	return widgetRaw(n).menu();
}

FlLayout* FlLayout::layout(int n)
{
	return widgetRaw(n).layout();
}

FlLayout* FlLayout::findLayout(const char* id)
{
	return findWidget(id).layout();
}

FlLayoutGroup* FlLayout::findLayoutGroup(const char* id)
{
	return findWidget(id).layoutGroup();
}

FlChoice* FlLayout::findMenu(const char* id)
{
	return findWidget(id).menu();
}

Fl_Slider* FlLayout::slider(int n)
{
	return widgetRaw(n).slider();
}

Fl_Slider* FlLayout::findSlider(const char* id)
{
	return findWidget(id).slider();
}

Fl_Valuator* FlLayout::valuator(int n)
{
	return widgetRaw(n).valuator();
}

Fl_Valuator* FlLayout::findValuator(const char* id)
{
	return findWidget(id).valuator();
}

Fl_Button* FlLayout::button(int n)
{
	return widgetRaw(n).button();
}

Fl_Button* FlLayout::findButton(const char* id)
{
	return findWidget(id).button();
}

Fl_Light_Button* FlLayout::checkButton(int n)
{
	return widgetRaw(n).checkButton();
}

Fl_Light_Button* FlLayout::findCheckButton(const char* id)
{
	return findWidget(id).checkButton();
}

void FlLayout::callCallbackFunction(Widget& w)
{
	onCallback(w, w.widgetRaw(), (int)(reinterpret_cast<long long>(w.widgetRaw()->user_data())));
}

void FlLayout::cbFunc(Fl_Widget * pWidget, void *data)
{
  #ifndef NO_GUI

	FlLayout* pLayout=((FlLayout*)pWidget->parent()->user_data());

	if(pWidget==((Fl_Widget*)pLayout->mScrollbar))
	{
		printf("scroll\n");

#ifdef __APPLE__
		pLayout->mWindow->resize(pLayout->x()+0,pLayout->y()+-1*pLayout->mScrollbar->value(), pLayout->w()-SCROLLW*mfScaleFactor+RIGHT_PADDING, pLayout->mScrollbar->value()+pLayout->h());
#else
		pLayout->mWindow->resize(0,-1*pLayout->mScrollbar->value(), pLayout->w()-SCROLLW*mfScaleFactor+RIGHT_PADDING, pLayout->mScrollbar->value()+pLayout->h());
#endif
		return;
	}

	pLayout->onCallback(pLayout->_findWidget(pWidget), pWidget, (int)(reinterpret_cast<long long>(data)));
#endif
}

void FlLayout::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	// default: do nothing

	if(_callbackRouter!=this)
	{
#ifdef _MSC_VER
		printf("routing\n");
#endif
		_callbackRouter->onCallback(w, pWidget, userData);
	}
}

void FlLayout::setButtonClass(buttonType b)		{mWidgets[mWidgets.size()-1].mState.mButtonType=b;	}
void FlLayout::setSliderClass(sliderType s)		{mWidgets[mWidgets.size()-1].mState.mSliderType=s;	}
void FlLayout::setLineSpace(int l)				{mWidgets[mWidgets.size()-1].mState.mLineSpace=l*mfScaleFactor;}
void FlLayout::setHorizSpace(int h)				{mWidgets[mWidgets.size()-1].mState.mHorizSpace=h*mfScaleFactor;}
void FlLayout::setWidgetHeight(int h)			{mWidgets[mWidgets.size()-1].mState.mWidgetHeight=h*mfScaleFactor;}
void FlLayout::setUIscaleFactor(float s)
{
	mScaleFactor=(int)s;
	mfScaleFactor=s;
}

