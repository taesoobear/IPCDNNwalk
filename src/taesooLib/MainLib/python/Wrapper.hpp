#pragma once
#include "../OgreFltk/FlChoice.h"

int FlGenShortcut(const char* s);
		class WidgetWrapper
		{
		public:
			static void checkButtonValue(FlLayout::Widget& w, int value) { w.checkButton()->value(value); }
			static void checkButtonValue3(FlLayout::Widget& w, bool value) { w.checkButton()->value(value); }
			static bool checkButtonValue2(FlLayout::Widget& w) { return w.checkButton()->value(); }
			static void menuSize(FlLayout::Widget& w,int nsize) { w.menu()->size(nsize); }
			static void menuItem(FlLayout::Widget& w,int i, const char* title) { w.menu()->item(i, title); }
			static void menuItem2(FlLayout::Widget& w,int i, const char* title, const char* shortc) { w.menu()->item(i, title, FlGenShortcut(shortc)); }
			static void menuValue(FlLayout::Widget& w, int v) { w.menu()->value(v); w.menu()->redraw(); }
			static std::string menuText(FlLayout::Widget& w, int v) {
#ifndef NO_GUI
				return w.menu()->text(v);
#else
				return "";
#endif
			}
			static std::string menuText2(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				return w.menu()->text();
#else
				return "";
#endif
			}
			static int menuValue2(FlLayout::Widget& w) { return w.menu()->value(); }
			static void setVisible(FlLayout::Widget& w) { w.widget<Fl_Widget>()->set_visible(); }
			static void userData(FlLayout::Widget& w, int userData) { w.widget<Fl_Widget>()->user_data(reinterpret_cast<void*>((long long)(userData))); }
			static int userData2(FlLayout::Widget& w) { return (int)reinterpret_cast<long long>(w.widget<Fl_Widget>()->user_data()); }
			static void clearVisible(FlLayout::Widget& w) { w.widget<Fl_Widget>()->clear_visible(); }
			static void sliderValue(FlLayout::Widget& w, double v) { w.slider()->value(v); }
			static void sliderStep(FlLayout::Widget& w, double v) {
#ifndef NO_GUI
				w.slider()->step(v);
#endif
			}
			static void sliderRange(FlLayout::Widget& w, double v1, double v2)
			{
#ifndef NO_GUI
				w.slider()->range(v1, v2);
#endif
			}
			static double sliderValue2(FlLayout::Widget& w) { return w.slider()->value(); }
			static void buttonShortcut(FlLayout::Widget& w, const char* s)
			{
#ifndef NO_GUI				
				w.button()->shortcut(FlGenShortcut(s));
				w.button()->tooltip(s);
#endif
			}
			static void buttonTooltip(FlLayout::Widget& w, const char* s)
			{
#ifndef NO_GUI				
				w.button()->tooltip(s);
#endif
			}
			static void buttonSetLabel(FlLayout::Widget& w, const char* s)
			{
#ifndef NO_GUI				
				w.widgetRaw()->copy_label(s);
#endif
			}
			static const char* buttonLabel(FlLayout::Widget& w)
			{
#ifndef NO_GUI				
				return w.widgetRaw()->label();
#else
				return "button";
#endif
			}
			static void redraw(FlLayout::Widget& w) { w.widget<Fl_Widget>()->redraw(); }
			static void deactivate(FlLayout::Widget& w)
			{
#ifndef NO_GUI				
				w.widget<Fl_Widget>()->deactivate();
#endif
			}
			static void activate(FlLayout::Widget& w)
			{
#ifndef NO_GUI				
				w.widget<Fl_Widget>()->activate();
#endif
			}
			static const char* id(FlLayout::Widget& w) { return w.mId; }
			static int browserSize(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				return browser->size();
#else
				return 0;
#endif
			}
			static bool browserSelected(FlLayout::Widget& w, int i)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				return browser->selected(i);
#else
				return false;
#endif
			}
			static int browserValue(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				return browser->value();
#else
				return 0;
#endif
			}
			static const char* browserText(FlLayout::Widget& w, int i)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				return browser->text(i);
#else
				return 0;
#endif
			}

			static void browserDeselect(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->deselect();
#endif
			}
			static void browserSelect(FlLayout::Widget& w,int i)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->select(i);
#endif
			}
			static void browserAdd(FlLayout::Widget& w, const char* name)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->add(name,NULL);
#endif
			}
			static void browserRemove(FlLayout::Widget& w, int i)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->remove(i);
#endif
			}
			static void browserClear(FlLayout::Widget& w)
			{
#ifndef NO_GUI
				Fl_Browser* browser=w.widget<Fl_Browser>();
				browser->clear();
#endif
			}
			static void inputValue1(FlLayout::Widget& w, const char* text)
			{
#ifndef NO_GUI
				Fl_Input* input=w.widget<Fl_Input>();
				input->value(text);
#endif
			}
			static std::string inputValue2(FlLayout::Widget& w)
			{
				std::string str;
#ifndef NO_GUI
				Fl_Input* input=w.widget<Fl_Input>();
				str=input->value();
#endif
				return str;
			}
static void inputType(FlLayout::Widget& w, const char* str)
			{
#ifndef NO_GUI
				if (TString(str)=="FL_MULTILINE_OUTPUT")
				{
					Fl_Input* ip=w.widget<Fl_Input>();
					ip->type(FL_MULTILINE_OUTPUT);
				}
#endif
			}
		};

