#ifndef FL_LAYOUT_H_
#define FL_LAYOUT_H_

#pragma once


#include "../../BaseLib/utility/namedmapsupport.h"
#include "../WrapperLua/LUAwrapper.h"
class FlLayout;
class Fl_Button;
class Fl_Valuator;
class Fl_Slider;
class Fl_Light_Button;
class FlChoice;
#ifndef NO_GUI
#include <Fl/Fl_Group.H>
#include <Fl/Fl_Double_Window.H>
#endif
class FlLayoutGroup : public Fl_Group
{
	std::vector<FlLayout*> layouts;
	int mCurrLayout;
public:
	int numLayouts();
	FlLayout* layout(int ilayout=0);
	void showLayout(int ilayout);

	FlLayoutGroup (int x, int y, int w, int h, const char* l=0);
	FlLayoutGroup (int x, int y, int w, int h, FlLayout* layout, const char* l=0);
	FlLayoutGroup (int x, int y, int w, int h, std::vector<FlLayout*> const& layouts, const char* l=0);

	virtual void resize(int x,int y,int w,int h);

	void updateLayout();
	int minimumHeight();
};

class Fl_Scrollbar;
#include "../WrapperLua/LUAwrapper.h"
// 자동으로 layout을 해준다.
#ifdef __APPLE__
class FlLayout  : public Fl_Group, public LUAwrapper::Worker, noncopyable
#else
class FlLayout  : public Fl_Double_Window, public LUAwrapper::Worker, noncopyable
#endif
{
public:

	// buttontype, sliderType -> deprecated
	enum buttonType {BUTTON, CHECK_BUTTON, LIGHT_BUTTON};
	enum sliderType {SLIDER, VALUE_SLIDER, VALUE_VERT_SLIDER};

	FlLayout ();
	FlLayout (int w, int h, const char* title=NULL);
	FlLayout (int x, int y, int w, int h, const char* title=NULL);
	~FlLayout (void);

	/***********************************************************************
	usage

	TwoWin::TwoWin(int x, int y, int w, int h)
	: FlLayout  (x,y,w,h) ,
	{
		create("Button", "btn:attack1", "Attack 1");
		createSlider("Slider", "slider1", "Attack 1");
		slider(0).value(0,1);
		updateLayout();
	}

	TwoWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
	{
		if(w.mId=="btn:attack1")
			doSomething();
	}
	***********************************************************************/


	virtual int work(TString const& workname, lunaStack& L);

	// type-> FL_를 제외한 클래스 이름. ex) Input, Box, Adjuster, Check_Button, Menu, Choice, Slider, Frame_Adjuster, Layout...
	Fl_Widget* create(const char* type, const char* id, const char* title=0);

	// height가 0이면 widget의 default height를 사용한다.
	Fl_Widget* create(const char* type, const char* id, const char* title, int startSlot, int endSlot=INT_MAX, int height=0);

	void removeWidgets(int startWidgetIndex);

	void embedLayout(FlLayout* childLayout, const char* id, const char* title=0);
	void embedLayouts(std::vector<FlLayout*> const& childLayouts,const char* id, const char* title=0);

	void newLine(); // 다음줄로 옮긴다. (디폴트 widgetPosition에서는 자동으로 넘어가니 call안해주어도 됨.)

	// setState  (이후에 생성되는 모든 위젯은 가장 최근의 setState의 영향을 받는다.)
	void setLineSpace(int l=5);
	// space between two horizontally consecutive widgets
	void setHorizSpace(int h=5);

	void setWidgetHeight(int h=20);
	void setUniformGuidelines(int totalSlot); // 가로로 totalSlot등분한다.
	void setWidgetPos(int startSlot, int endSlot=INT_MAX); // guideline 따라 나누어져있는 영역에서 얼만큼 차지할지.
	void setWidgetPosUniform(int totalSlot, int slot);	// 가로로 totalSlot등분을 해서, 몇번째에 넣어라.
	void resetToDefault()	{setUniformGuidelines(3); setWidgetPos(0); setWidgetHeight(20);}
	vectorn& guideLines();

	// updateLayout를 마지막에 반드시 해주어야한다.
	void updateLayout();

	// scrollbar가 생기지 않는 minimum한 height를 return한다.
	int minimumHeight();

	class Widget
	{
	public:
		Widget();
		TString mId;
		TString mType;

		float progressValue() const;
		void progressValue(float v) ;

		Fl_Widget* widgetRaw() const {return mWidget;}
		template <class Fl_Widget_Type> Fl_Widget_Type* widget() const {return dynamic_cast<Fl_Widget_Type*>(widgetRaw());}

		// type casting utility
		Fl_Button* button() const;				//!< 모든 버튼에 사용.
		Fl_Valuator* valuator() const;			//!< slider나 adjuster에 사용.
		Fl_Slider* slider() const;
		Fl_Light_Button* checkButton() const;	//!< lightButton이나 checkButton에 사용
		FlChoice * menu() const;				//!< choice나 menu에 사용
		FlLayout* layout() const;
		FlLayoutGroup*layoutGroup() const;
	private:
		Fl_Widget* mWidget;

		// additional data
		union
		{
			int mValue;
		};

		struct State
		{
			// state variable
			buttonType mButtonType;	// 이 widget의 type
			sliderType mSliderType;	// 이 widget의 type

			vectorn mGuideLines; // 0부터 1사이. 소팅된 순서로. 가이드라인 [0,1], [1,2], [2,3].. 등이 widget이 차지할 슬랏이된다.
			int mStartSlot, mEndSlot;	// 시작 slot과 끝 slot 넘버. mRight==총슬랏수(mGuideLines.size()-1)이면 줄바꾸기됨.
			int mWidgetHeight;
			int mLineSpace;	// 이 widget위의 빈공간 (default=5).
			int mHorizSpace;
		};

		State mState;
		friend class FlLayout;
	};

	/// you can distinguish the caller based on w.mId, pWidget, pWidget->label() or userData.
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// n 이 0일때 마지막으로 생성된 위젯. 1일때 다음에 생성할 위젯, -1일때 끝에서 두번째 위젯, ...
	Widget& widgetRaw(int n);
	int widgetIndex(const char* id);	//!< 버튼 10개를 생성한후 첫번째 버튼의 index를 받아오면, 그 이후의 버튼은 index+1, index+2.등으로 접근 가능하다.
	Widget& findWidget(const char* id);

	template <class Fl_Widget> Fl_Widget* widget(int n)
	{
		return dynamic_cast<Fl_Widget*>(widgetRaw(n).widgetRaw());
	}

	template <class Fl_Widget> Fl_Widget* find(const char* id)
	{
		return dynamic_cast<Fl_Widget*>(findWidget(id).widgetRaw());
	}

	FlLayout* layout(int n);
	FlLayoutGroup* layoutGroup(int n);
	FlLayout* findLayout(const char* id);
	FlLayoutGroup* findLayoutGroup(const char* id);

	FlChoice * menu(int n);
	FlChoice * findMenu(const char* id);

	// utility functions :  findSlider(id)==find<Fl_Slider> (id), slider(0)==widgetT<Fl_Slider>(0)
	Fl_Slider* slider(int n);
	Fl_Slider* findSlider(const char* id);

	Fl_Valuator* valuator(int n);
	Fl_Valuator* findValuator(const char* id);

	Fl_Button* button(int n);
	Fl_Button* findButton(const char* id);

	// check button이나 light button을 찾아준다.
	Fl_Light_Button* checkButton(int n);
	Fl_Light_Button* findCheckButton(const char* id);

	void callCallbackFunction(Widget& w);

	virtual void resize(int x,int y,int w,int h);

	///////////////////////////////////////////////////////////////////
	//  아래 함수들은 모두 deprecated
	//
	//  use create(..) instead of createWidget(..)
	///////////////////////////////////////////////////////////////////
	Fl_Widget* createWidget(const char* type, const char* id, const char* title=0) { return create(type, id, title);}
	FlChoice * createMenu(const char* id, const char* title=0);
	Fl_Button* createButton(const char* id, const char* title=0);
	Fl_Slider* createSlider(const char* id, const char* title=0);

	// createButton, createSlider의 default class를 정해준다.
	void setButtonClass(buttonType b);
	void setSliderClass(sliderType s);

	static void setUIscaleFactor(float s);
private:
	int m_minimumHeight;
	Fl_Window* mWindow;
	Fl_Scrollbar* mScrollbar;
	std::vector<Widget> mWidgets;
	std::map<TString, int, cmpTString> mNamedmapWidgets;
	FlLayout* _callbackRouter;
//	friend class FlLayout;

	static void cbFunc(Fl_Widget * pWidget, void *data);
	Fl_Widget* _createWidget(const char* id, Fl_Widget* o);
	Widget& _findWidget(Fl_Widget* o);
};
#endif

