#include "stdafx.h"
#include "../BaseLib/utility/operatorString.h"
#ifndef NO_GUI
#include <FL/Fl_Color_Chooser.H>
#include <FL/Fl_PNG_Image.H>
#include <FL/Fl_JPEG_Image.H>
#include <FL/Fl_File_Chooser.H>
#endif
#include "FlChoice.h"

#ifndef NO_GUI

#if defined( WIN32) 
#define USE_NATIVE_FILE_CHOOSER
#include "Fl_Native_File_Chooser-0.84/FL/Fl_Native_File_Chooser.h"
#elif defined (__APPLE__)
#define USE_NATIVE_FILE_CHOOSER
#include <FL/Fl_Native_File_Chooser.H>
#endif
#else // NO_GUI

#ifdef WIN32
#include "windows.h"
#else
#include <unistd.h>
#endif
#endif // NO_GUI
#include "FltkAddon.h"

extern float mfScaleFactor;
TString getCurrentDirectory()
{
#ifdef WIN32
	char szCurrDirectory[512];
	::GetCurrentDirectory(512, szCurrDirectory);
	return TString(szCurrDirectory);
#else
	return "~tskwon/taesoo_cmu/OgreFltk/work"; // hard coded at the moment.
#endif
}
#ifdef __APPLE__
//std::string chooseFileMac(const char* path, const char* mask);
#endif

TString _getcwd()
{   
#ifdef _MSC_VER
  // utility. It opens the file dialog and choose file.
	char szCurrDirectory[512];
	GetCurrentDirectory(512, szCurrDirectory);
	return szCurrDirectory;
#else
   	const size_t chunkSize=255;
    const int maxChunks=10240; // 2550 KiBs of current path are more than enough

    char stackBuffer[chunkSize]; // Stack buffer for the "normal" case
    if(getcwd(stackBuffer,sizeof(stackBuffer))!=NULL)
        return TString(stackBuffer);
#endif
}
void _chdir(const char* szCurrDirectory)
{
#ifdef _MSC_VER
	SetCurrentDirectory(szCurrDirectory);
#else
	chdir(szCurrDirectory);
#endif
}
#ifndef NO_GUI
#if defined(__APPLE__) || !defined(_MSC_VER)
static TString _FlChooseFile_python(const char* message, const char* path, const char* Mask, bool bCreate)
{
	Fl::check(); // check input and redraw.

	// use filechooser.py on linux
	TString cmd;
#ifdef __APPLE__
	if(bCreate)
		cmd.format("python filechooser_mac.py '%s' '%s' '%s' SAVE", message, Mask, path);
	else
	{
			/*
		if (strlen(Mask)==5)
			cmd.format("./mac_openFileDialog %s %s", path, &Mask[2]);
			//std::string url=chooseFileMac(path, &Mask[2]);
			//printf("%s\n", url.c_str());
			//return TString(url.c_str());
		else
			*/
			cmd.format("python filechooser_mac.py '%s' '%s' '%s' OPEN", message, Mask, path);
	}
#else
	if(IsFileExist("/usr/bin/python"))
	{
		if(bCreate)
			cmd.format("python filechooser.py '%s' '%s' '%s' SAVE", message, Mask, path);
		else
			cmd.format("python filechooser.py '%s' '%s' '%s' OPEN", message, Mask, path);
	}
	else 
	{
		if(bCreate)
			cmd.format("python3 filechooser_wx.py '%s' '%s' '%s' SAVE", message, Mask, path);
		else
			cmd.format("python3 filechooser_wx.py '%s' '%s' '%s' OPEN", message, Mask, path);
	}
#endif
	printf("%s\n", cmd.ptr());
	FILE *lsofFile_p = popen(cmd.ptr(), "r");

	if (!lsofFile_p)
	{
		return TString();
	}

	char buffer[1024];
	char *line_p = fgets(buffer, sizeof(buffer), lsofFile_p);
	pclose(lsofFile_p);

	buffer[strlen(buffer)-1]=0;
	if (strcmp(buffer, "Closed, no files selected")==0)
		return TString();
	return TString(buffer);
}
#endif
static TString _FlChooseFile_fltk(const char* message, const char* path, const char* Mask, bool bCreate)
{
	int mask;
	if(bCreate)
		mask=Fl_File_Chooser::SINGLE | Fl_File_Chooser::CREATE;
	else
		mask=Fl_File_Chooser::SINGLE ;

#if defined( _MSC_VER) 
	char szCurrDirectory[512];
	GetCurrentDirectory(512, szCurrDirectory);
#endif
	Fl_File_Chooser fc(path, Mask, mask, message);
	fc.show();
	while (fc.visible())
		Fl::wait();

	if (fc.count() > 0)
	{
		printf("%s", fc.value(1));
#ifdef _MSC_VER
		SetCurrentDirectory(szCurrDirectory);
#endif
		return TString(fc.value(1));
	}

	return TString();
}
#endif
TString FlChooseFile(const char* message, const char* path, const char* Mask, bool bCreate)
{
#ifdef NO_GUI
  return TString();
#else
#if defined(_MSC_VER) 
  // use native file chooser on windows
#ifdef USE_NATIVE_FILE_CHOOSER

	TString szCurrDirectory=_getcwd();
	printf("curr:%s\n", szCurrDirectory.ptr());


	Fl_Native_File_Chooser *chooser = new Fl_Native_File_Chooser();
	if(bCreate)
		chooser->type(Fl_Native_File_Chooser::BROWSE_SAVE_FILE);
	else
		chooser->type(Fl_Native_File_Chooser::BROWSE_FILE);   // let user browse a single file

	chooser->directory(path);
	chooser->title(message);        // optional title
	chooser->filter(Mask);			// optional filter
	TString fn;
	switch ( chooser->show() ) {
		case -1:    // ERROR
		fprintf(stderr, "*** ERROR show() failed:%s\n", chooser->errmsg());
		break;
		case 1:     // CANCEL
		fprintf(stderr, "*** CANCEL\n");
		break;
		default:    // USER PICKED A FILE
		fn=chooser->filename();
		break;
	}

	_chdir(szCurrDirectory);
/*
	TString pdir=sz1::parentDirectory(szCurrDirectory);
	pdir.replace('\\', '/');
	// abs to relative.
	if(fn.left(pdir.length()).toUpper()==pdir.toUpper())
	{
		fn="../"+fn.right(pdir.length()*-1);
	}*/
	return fn;
#else // 
	return _FlChooseFile_fltk(message, path, Mask, bCreate);
#endif
#else // !defined(_MSC_VER)
#ifdef __APPLE__
	if(bCreate)
		return _FlChooseFile_fltk(message, path, Mask, bCreate);
#endif
	return _FlChooseFile_python(message, path, Mask, bCreate);
#endif
#endif
}

void FlChooseFiles(const char* message, const char* path, const char* Mask, TStrings& files)
{
#ifndef NO_GUI
#ifdef _MSC_VER
	// utility. It opens the file dialog and choose file.
	char szCurrDirectory[512];
	GetCurrentDirectory(512, szCurrDirectory);
#endif

	int mask;
	mask=Fl_File_Chooser::MULTI;

	Fl_File_Chooser fc(path, Mask, mask, message);
	fc.show();
	while (fc.visible())
		Fl::wait();

	if (fc.count() > 0)
	{
		files.resize(fc.count());
		for(int i=1; i<=fc.count(); i++)
		{
			printf("%s", fc.value(i));
			files[i-1]=fc.value(i);
		}
	}

#ifdef _MSC_VER
	SetCurrentDirectory(szCurrDirectory);
#endif
#endif
}



Fl_Widget* FlCallee::findWidget(int userData)
{
	std::map<int, Fl_Widget*>::iterator i;
	i=mWidgets.find(userData);

	if(i==mWidgets.end())
	{
		Msg::error("no such widget");
		return NULL;
	}
	else
		return i->second;
}

Fl_Check_Button* FlCallee::findCheckButton(int userData)
{
	Fl_Widget* pWidget=findWidget(userData);
	return (Fl_Check_Button*)(pWidget);

}
	/*
	왠지 모르겠지만 dynamic_cast가 동작 안함..-.-
	try {
		pCheckButton=dynamic_cast<Fl_Check_Button*>(pWidget);
	}
	catch (bad_cast) {
		Msg::error("Caught: bad_cast exception. A Shape is not a Circle.\n");
	}
	return pCheckButton;*/

	void FlCallee::cbFunc(Fl_Widget * pWidget, void *data)
	{
	  long long ddata=reinterpret_cast<long long>(data);
	  ((FlCallee*)pWidget->parent()->user_data())->onCallback(pWidget, int(ddata));
	}
void FlCallee::connect(Fl_Widget* pWidget, int userData)
{
#ifndef NO_GUI
	// Auto-size
	pWidget->labelsize(11*mfScaleFactor);

	// parent가 갖고 있는 모든 widget과 메뉴는 같은 callee에 연결됨을 가정하였다.
	// 실제적으로 구현할때는 parent자체가 callee가 되도록 하는게 가장 확실하다.
	// 즉 parent는  Fl_Group(또는 Fl_Window 등)과 FlCallee를 동시상속.
	void* prev_user_data=pWidget->parent()->user_data();
	Msg::verify(prev_user_data==NULL || prev_user_data==((void*)this), "FlCallee::connect error");
	pWidget->parent()->user_data((void*)this);
	pWidget->callback(cbFunc, (void*)(size_t)userData);
/*
	std::map<int, int> mapTest;
	mapTest[1243131232]=213;
	mapTest[1243232]=1213;
	mapTest[1242]=12133;
	mapTest[1242]=12134;
	mapTest[1243]=12135;

	printf("%d %d %d %d\n",mapTest[1243131232], mapTest[1243232],mapTest[1242],  mapTest[1243]);


	printf("%d %d %d %d userData\n", userData, (int)this, (int )pWidget, (int)&mWidgets);


	std::map<int, Fl_Widget*>::iterator i;
	i=mWidgets.find(userData);

	if(i==mWidgets.end())
	{
		printf("userdata overlaps\n");
		i->second==pWidget;
	}
	else*/
#endif
    mWidgets[userData]=pWidget;
}

void FlCallee::connect(FlMenu& menu)
{
#ifndef NO_GUI
  Fl_Widget* pWidget=menu.m_pMenu;
	// Auto-size
  pWidget->labelsize(11*mfScaleFactor);

	// parent가 갖고 있는 모든 widget과 메뉴는 같은 callee에 연결됨을 가정하였다.
	// 실제적으로 구현할때는 parent자체가 callee가 되도록 하는게 가장 확실하다.
	// 즉 parent는  Fl_Group(또는 Fl_Window 등)과 FlCallee를 동시상속.
	void* prev_user_data=menu.m_pMenu->parent()->user_data();
	Msg::verify(prev_user_data==NULL || prev_user_data==((void*)this), "FlCallee::connect error");
	menu.m_pMenu->parent()->user_data((void*)this);
	for(int item=0; item<menu.m_nMenu; item++)
		menu.m_aMenuItem[item].callback(cbFunc);
#endif
}

void Fl_Callee::connect(Fl_Widget* pWidget, int callBackID)
{
#ifndef NO_GUI
	// Auto-size
	pWidget->labelsize(11*mfScaleFactor);

	switch(callBackID)
	{
	case 0:
	pWidget->callback(cbFunc0, (void*)this);
	break;
	case 1:
	pWidget->callback(cbFunc1, (void*)this);
	break;
	case 2:
	pWidget->callback(cbFunc2, (void*)this);
	break;
	case 3:
	pWidget->callback(cbFunc3, (void*)this);
	break;
	case 4:
	pWidget->callback(cbFunc4, (void*)this);
	break;
	}
#endif
}

#ifdef NO_GUI
#define FL_VOID(x)
#else
#define FL_VOID(x) x
#endif

void Fl_Callee::cbFunc0(Fl_Widget * pWidget, void *data)
{
	FL_VOID(((Fl_Callee*)pWidget->user_data())->onCallback(pWidget, 0));
}
void Fl_Callee::cbFunc1(Fl_Widget * pWidget, void *data)
{
	FL_VOID(((Fl_Callee*)pWidget->user_data())->onCallback(pWidget, 1));
}
void Fl_Callee::cbFunc2(Fl_Widget * pWidget, void *data)
{
	FL_VOID(((Fl_Callee*)pWidget->user_data())->onCallback(pWidget, 2));
}
void Fl_Callee::cbFunc3(Fl_Widget * pWidget, void *data)
{
	FL_VOID(((Fl_Callee*)pWidget->user_data())->onCallback(pWidget, 3));
}
void Fl_Callee::cbFunc4(Fl_Widget * pWidget, void *data)
{
	FL_VOID(((Fl_Callee*)pWidget->user_data())->onCallback(pWidget, 4));
}

int FlMenu::userData()		
{
#ifndef NO_GUI
  return (int)reinterpret_cast<long long>((m_aMenuItem[m_pMenu->value()].user_data())); 
#else
  return 0;
#endif
}
Fl_Menu_Item& FlMenu:: item(int item)					{return m_aMenuItem[item];}
void  FlMenu::beginSubMenu(int item, const char* text)	{	this->item(item, text, 0, 0, FL_SUBMENU);}
void  FlMenu::endSubMenu(int item)						{	this->item(item, 0);}

bool  FlMenu::isValidItem(int item)						{ return (item>=0 && item<size() && m_aMenuItem[item].flags!=FL_SUBMENU && m_aMenuItem[item].text!=NULL);}

Fl_Menu_&  FlMenu::menu()						{return *m_pMenu;}
TString const&  FlMenu::label(int iItem) const	{return m_aLabel[iItem];}
TString const&  FlMenu::label()	const			{return label(value());}
bool  FlMenu::operator==(Fl_Widget* pWidget) const	{ return (Fl_Widget* )m_pMenu==pWidget;}

// get user data of the current choice.
int  FlMenu::userData(int n)	{ return (int)(reinterpret_cast<long long>(m_aMenuItem[n].user_data())); }

void FlMenu::value(int n)	{ m_pMenu->value(n);}
int FlMenu::value()	const	{ return m_pMenu->value(); }
void FlMenu::redraw()		{ FL_VOID(m_pMenu->redraw()); }

FlMenu::FlMenu()
{
	m_nMenu=0;
	m_pMenu=NULL;
}

FlMenu::~FlMenu()
{
	if(m_nMenu)
		delete [] m_aMenuItem;
	// m_pMenu will be automatically deleted in FLTK way.
}

void FlMenu::init(Fl_Menu_* pMenu, int n)
{
	init(pMenu);
	size(n);
}

void FlMenu::init(Fl_Menu_* pMenu)
{
	m_pMenu=pMenu;
#ifndef NO_GUI
	if(m_pMenu->h()<=20)
	{
		//o->labelsize(11);
		m_pMenu->textsize(11*mfScaleFactor);
	}
#endif
	if(m_nMenu)
		delete [] m_aMenuItem;

}

void FlMenu::initChoice(int x, int y, int w, int h, const char* title)
{
#ifndef NO_GUI
    Fl_Choice* o=new Fl_Choice(x, y, w, h, title);
	init(o);
#endif
}

void FlMenu::size(int n)
{
#ifndef NO_GUI
	if(m_nMenu)
		delete [] m_aMenuItem;

	m_aMenuItem=new Fl_Menu_Item[n+1];
	memset(m_aMenuItem, 0, sizeof(Fl_Menu_Item)*(n+1));
#endif
	m_aLabel.init(n);
	m_nMenu=n;

}

int FlMenu::size()	{return m_aLabel.size();}
void FlMenu::item(int item, const char* text, int shortcut_,  int user_data_,  int flags,  uchar labeltype_,  uchar labelfont_, uchar labelsize_,  unsigned labelcolor_)
{
	if(item >=m_nMenu)
	{
		printf("Error! Too many MenuItem \n");
		ASSERT(0);
		return;
	}
	if(text==0)
	{
		m_aLabel[item].format("__submenu_ended");
#ifndef NO_GUI

		memset(&m_aMenuItem[item], 0, sizeof(Fl_Menu_Item));
#endif
	}
	else
	{
		if(labelsize_==0)
			labelsize_=11;
		m_aLabel[item].format("%s",text);
#ifndef NO_GUI
		m_aMenuItem[item].label(m_aLabel[item].ptr());
		m_aMenuItem[item].shortcut(shortcut_);
		m_aMenuItem[item].user_data((void*)(size_t)user_data_);
		m_aMenuItem[item].flags=flags;
		m_aMenuItem[item].labeltype((Fl_Labeltype)labeltype_);
		m_aMenuItem[item].labelfont(labelfont_);
		m_aMenuItem[item].labelsize(labelsize_*mfScaleFactor);
		m_aMenuItem[item].labelcolor(labelcolor_);
#endif
	}
#ifndef NO_GUI

	if(item==m_nMenu-1)
	{
		m_pMenu->menu(m_aMenuItem);
	}
#endif
}
#ifndef NO_GUI

#include "FL/Fl_Scroll.H"

class Fl_Vert_Scroll : public Fl_Scroll
{
public:

  Fl_Vert_Scroll(int X,int Y,int W,int H,const char*l=0):Fl_Scroll(X,Y,W,H,l){type(Fl_Scroll::VERTICAL);}

// Insure the scrollbars are the last children:
void fix_scrollbar_order2() {
  Fl_Widget** a = (Fl_Widget**)array();
  if (a[children()-1] != &scrollbar) {
    int i,j; for (i = j = 0; j < children(); j++)
      if (a[j] != &hscrollbar && a[j] != &scrollbar) a[i++] = a[j];
    a[i++] = &hscrollbar;
    a[i++] = &scrollbar;
  }
}
  virtual	void resize(int X, int Y, int W, int H)
  {
	  fix_scrollbar_order2();
	  // move all the children:
	  Fl_Widget*const* a = array();
	  for (int i=children()-2; i--;) {
		Fl_Object* o = *a++;
		o->position(o->x()+X-x(), o->y()+Y-y());
		o->size(o->w()+W-w(), o->h());
	  }
	  Fl_Widget::resize(X,Y,W,H);
  }


};

#endif
FlChoiceWins::FlChoiceWins(int x, int y, int w, int h, int nWindow, const char* MainMenu)
:
Fl_Group(x,y,w,h),
m_nWindow(nWindow),
m_nCurrShow(-1)
{
#ifndef NO_GUI
	m_menuWin.init(new Fl_Choice(x+0, y+0, w-21*mfScaleFactor, 20*mfScaleFactor, ""), nWindow+1);
	Fl_Button* back=new Fl_Button(x+w-20*mfScaleFactor,y+1*mfScaleFactor,18*mfScaleFactor,18*mfScaleFactor,"@<-");
	back->shortcut(FL_BackSpace);
	connect(back,-1);
#endif
	windowNames.resize(nWindow);

#ifndef NO_GUI

	findWidget(-1)->hide();

	if(MainMenu)
		m_menuWin.item(0, MainMenu, 0, -1);
	else
		m_menuWin.item(0, "MainMenu", 0, -1);

#ifdef __APPLE__
	m_pMenuWin=new Fl_Window(0, 0*mfScaleFactor, w, h-0*mfScaleFactor);
#else
	m_pMenuWin=new Fl_Group(x+0, y+40*mfScaleFactor, w, h-40*mfScaleFactor);
#endif
	m_pScroll=new Fl_Vert_Scroll(x+0,y+20*mfScaleFactor,w,h-20*mfScaleFactor);
	m_pMenuWin->resizable(m_pScroll);

	if(m_pMenuWin->parent()!=(Fl_Group*)this) Msg::error("Menuwin's Parent is not this (FlChoiceWins)");
	m_pScroll->end();
	m_pMenuWin->end();
	resizable(m_pMenuWin);
#endif
}

void FlChoiceWins ::resize(int x,int y,int ww,int hh)
{
#ifndef NO_GUI
	Fl_Group::resize(x,y,ww,hh);
	m_pMenuWin->resize(x+0,y+40*mfScaleFactor,ww,hh-40*mfScaleFactor);
	m_pScroll->resize(x+0,y+20*mfScaleFactor,ww,hh-20*mfScaleFactor);

	for(int i=0; i<aWnd.size(); i++)
		aWnd[i]->resize(x+0, y+20*mfScaleFactor, w(), h()-20*mfScaleFactor);
#endif
}

void FlChoiceWins ::window(int i, const char* winName, Fl_Group* pWnd)
{
	int pos=aWnd.size();
#ifndef NO_GUI
	if(pWnd->parent()!=(Fl_Group*)this) Msg::error("%s's Parent is not this (FlChoiceWins) ", pWnd->label());
	if(pos!=0 && pWnd->parent()!=aWnd[pos-1]->parent()) Msg::error(" Parent is different");

	pWnd->end();
#endif
    aWnd.push_back(pWnd);
#ifndef NO_GUI

    pWnd->hide();

	pWnd->resize(x()+0,y()+20*mfScaleFactor,w(), h()-20*mfScaleFactor);

	m_menuWin.item(pos+1, winName, 0, pos);

	m_pMenuWin->begin();
	m_pScroll->begin();
#endif
	windowNames[i]=winName;
	TString temp;
	temp.format("%d. %s", i, winName);

#ifndef NO_GUI
	// small size
	Fl_Button* o=new Fl_Button(x()+10, y()+40*mfScaleFactor+ pos*25*mfScaleFactor, w()-20, 20*mfScaleFactor);
	o->copy_label(temp);
	o->labelsize(11*mfScaleFactor);
	o->align(FL_ALIGN_INSIDE | FL_ALIGN_LEFT);

	if(i<=9)
	{
		o->shortcut('0'+i);
		o->tooltip("shortcut: number");
	}

	connect(o, pos);

	m_pScroll->end();
	m_pMenuWin->end();

	if(pos==m_nWindow-1)
	{
		end();
		connect(m_menuWin);

		show(-1);	// hide all. (main menu)
	}
#endif
}

void FlChoiceWins ::show(int n)
{
#ifndef NO_GUI
	if(n==m_nCurrShow) return;
	if(m_nCurrShow==-1)
		m_pMenuWin->hide();
	else
		aWnd[m_nCurrShow]->hide();

	if(n==-1)
	{
		findWidget(-1)->hide();
		m_pMenuWin->show();
	}
	else
	{
		findWidget(-1)->show();
		aWnd[n]->show();
	}
	m_nCurrShow=n;

	m_menuWin.menu().value(n+1);
	redraw();
#endif
}

void FlChoiceWins ::onCallback(Fl_Widget * pWidget, int userData)
{
	show(userData);
}


void FlChoice::beginSubMenu(int item, const char* text)
{
	menu.item(item, text, 0, 0, FL_SUBMENU);
}

void FlChoice::endSubMenu(int item)
{
	menu.item(item, 0);
}

#ifndef NO_GUI
class FlPlot : public Fl_Widget {
protected:
	virtual void draw();

	Fl_Image* mSignal;
	std::vector<matrixn> mSignalSource;
public:
	FlPlot(int X, int Y, int W, int H, const char *l=0): Fl_Widget(X,Y,W,H,l), mSignal(NULL) {}
	virtual ~FlPlot() {delete mSignal;}
	void setSignal(matrixn const& scattered);
	void addSignal(matrixn const& scattered);

	// update mSignal and redraw.
	void update();
};
#endif
class Fl_Fast_Value_Slider : public Fl_Value_Slider
{
public:
	Fl_Fast_Value_Slider (int X, int Y, int W, int H, const char* l=0):Fl_Value_Slider(X,Y,W,H,l){}
	virtual ~Fl_Fast_Value_Slider (){}

    virtual int handle(int);
};
int Fl_Fast_Value_Slider ::handle(int ev)
{
#ifndef NO_GUI
	if(ev==FL_ENTER)
		take_focus();

	switch(ev)
	{
	case FL_FOCUS:
	case FL_UNFOCUS:
		return 1;
	}

	if(ev== FL_KEYBOARD)
	{
		int inc;

		printf("key up %c\r", Fl::event_key());
		if(Fl::event_ctrl())
			inc=10;
		else if(Fl::event_alt())
			inc=100;
		else inc=1;

		switch (Fl::event_key())
		{
		case '-':
			//if (horizontal()) return 0;
			handle_drag(clamp(increment(value(),-1*inc)));
			handle_release();
			return 1;
		case '+':
			//if (horizontal()) return 0;
			handle_drag(clamp(increment(value(),1*inc)));
			handle_release();
			return 1;
		default:
			return 0;
		}
    }
	return Fl_Value_Slider::handle(ev);
#else
  return 1;
	#endif

}

#ifndef NO_GUI
void fl_draw_CImage(const CImage& imagee, const TRect& sourceRect, int x, int y)
{

	CImage& image=(CImage&)imagee;
	CImagePixel ptr(&image);

	TRect srcRect(sourceRect.left, sourceRect.top, sourceRect.right, sourceRect.bottom);

	if(srcRect.left<0)
	{
		x-=srcRect.left;
		srcRect.left=0;
	}

	if(srcRect.top<0)
	{
		y-=srcRect.top;
		srcRect.top=0;
	}

	if(srcRect.right>image.GetWidth())
		srcRect.right=image.GetWidth();

	if(srcRect.bottom>image.GetHeight())
		srcRect.bottom=image.GetHeight();

	if(srcRect.Width()<=0 || srcRect.Height()<=0) return;

	static uchar *buffer=NULL;
	static int bufferWidth=0;

	int bufferSize=srcRect.Width()*3;
	if(bufferWidth<bufferSize)
	{
		delete buffer;
		buffer=new uchar[bufferSize];
		bufferWidth=bufferSize;
	}

	for(int yy=0; yy<srcRect.Height(); yy++)
	{
		CPixelRGB8* pColor=&ptr[srcRect.top+yy][srcRect.left];

		int w=srcRect.Width()*3;
		for(int i=0; i<w; i+=3)
		{
			buffer[i]=pColor->R;
			buffer[i+1]=pColor->G;
			buffer[i+2]=pColor->B;
			pColor++;
		}

		fl_draw_image(buffer, x, y+yy, srcRect.Width(), 1);
	}

}

// image의 sourceRect영역을 sampling ratio만큼 줄여서 화면의 x,y에 그린다.
// 즉 sampling ratio가 3이고, source와 dest의 x가 0인경우
// image의 0,1,2 가 0에 그려지고, 3,4,5가 1에 그려진다.
void fl_draw_CImage_scaleDown(int samplingRatio, const CImage& imagee, const TRect& sourceRect, int x, int y)
{
	CImage& image=(CImage&)imagee;
	CImagePixel ptr(&image);

	TRect srcRect(sourceRect.left, sourceRect.top, sourceRect.right, sourceRect.bottom);

	if(srcRect.left<0)
	{
		x-=srcRect.left/samplingRatio;
		srcRect.left=0;
	}

	if(srcRect.top<0)
	{
		y-=srcRect.top;
		srcRect.top=0;
	}

	int image_width=image.GetWidth();

	if(srcRect.right>image_width)
		srcRect.right=image_width;

	if(srcRect.bottom>image.GetHeight())
		srcRect.bottom=image.GetHeight();

	if(srcRect.Width()<=0 || srcRect.Height()<=0) return;

	static uchar *buffer=NULL;
	static int bufferWidth=0;


	int w=srcRect.Width()/samplingRatio*3;
	int bufferSize=w;
	if(bufferWidth<bufferSize)
	{
		delete buffer;
		buffer=new uchar[bufferSize];
		bufferWidth=bufferSize;
	}

	for(int yy=0; yy<srcRect.Height(); yy++)
	{
		CPixelRGB8* pColor=&ptr[srcRect.top+yy][srcRect.left];

#ifdef USE_DOWNSAMPLE
		for(int i=0; i<w; i+=3)
		{
			buffer[i]=pColor->R;
			buffer[i+1]=pColor->G;
			buffer[i+2]=pColor->B;
			pColor+=samplingRatio;
		}
#else
		for(int i=0; i<w; i+=3)
		{
			int R=0, G=0, B=0;
			for(int j=0; j<samplingRatio; j++)
			{
				R+=pColor->R;;
				G+=pColor->G;
				B+=pColor->B;
				pColor++;
			}
			buffer[i]=R/samplingRatio;
			buffer[i+1]=G/samplingRatio;
			buffer[i+2]=B/samplingRatio;
		}
#endif

		fl_draw_image(buffer, x, y+yy, srcRect.Width()/samplingRatio, 1);
	}
}


FlTreeChoiceWins ::FlTreeChoiceWins (int x, int y, int w, int h, int nWindow)
:Fl_Window(x,y,w,h),
m_nWindow(nWindow),
m_nCurrShow(-1)
{
	m_menuWin.init(new Fl_Choice(0, 0, w-21, 20, ""), nWindow+1);
	connect(new Fl_Button(w-20,1,18,18,"@<-"),-1);

	m_menuWin.item(0, "MainMenu", 0, -1);

	//resizable(false);
}

int defaultHeight(int i)
{
	return i*25+20+25;
}

void FlTreeChoiceWins ::window(int i, const char* winName, Fl_Window* pWnd)
{
	int pos=aWnd.size();
	if(pWnd->parent()!=(Fl_Group*)this) Msg::error(" Parent is not this (FlTreeChoiceWins)");
	if(pos!=0 && pWnd->parent()!=aWnd[pos-1]->parent()) Msg::error(" Parent is different");

	pWnd->end();

    aWnd.push_back(pWnd);
	pWnd->resize(0,20,w(), pWnd->h());
	pWnd->hide();

	m_menuWin.item(pos+1, winName, 0, pos);

	// small size
	Fl_Button* o=new Fl_Button(10, defaultHeight(pos), w()-20, 20, winName);
	o->labelsize(11*mfScaleFactor);

	connect(o, pos);

	aButton.push_back(o);

	if(pos==m_nWindow-1)
	{
		end();
		connect(m_menuWin);

		show(-1);	// hide all. (main menu)
	}
}

void FlTreeChoiceWins ::show(int n)
{
	if(n==m_nCurrShow)
	{
		if(m_nCurrShow==-1)
			return ;
		n=-1;
	}
	if(m_nCurrShow==-1)
		;
	else
		aWnd[m_nCurrShow]->hide();

	int curHeight=defaultHeight(n+1);

	static int defaultcolor=-1;
	if(defaultcolor==-1)
		defaultcolor=(int )aButton[0]->color();

	for(int i=0; i<numWindow(); i++)
	{
		aButton[i]->color((Fl_Color)defaultcolor);
		aButton[i]->resize(10, defaultHeight(i), w()-20, 20);
	}

	if(n!=-1)
	{
		aButton[n]->color(fl_rgb_color(200,255,225));
		aWnd[n]->resize(0,curHeight, w(), aWnd[n]->h());
		aWnd[n]->show();
		for(int i=n+1; i<numWindow(); i++)
			aButton[i]->resize(10, defaultHeight(i)+aWnd[n]->h(), w()-20, 20);
	}

	m_nCurrShow=n;

	m_menuWin.menu().value(n+1);
	redraw();
}

void FlTreeChoiceWins ::onCallback(Fl_Widget * pWidget, int userData)
{
	show(userData);
}


#ifndef NO_GUI
#include "Fl_Hor_Slider2.h"
Fl_Hor_Slider2::Fl_Hor_Slider2(int x,int y,int w,int h, const char *l )
:Fl_Slider(x+40,y, w-40, h, l)
{
	type(FL_HOR_SLIDER);
	range(0, 1.0);
	step(0.05);
	value(1.0);
	align(FL_ALIGN_LEFT);
}



#endif



#endif
