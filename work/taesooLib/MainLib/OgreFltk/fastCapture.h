#ifndef FASTCAPTURE_H_
#define FASTCAPTURE_H_
#ifndef NO_GUI
#pragma once


/* USAGE in FLTK
	Fl_Window* m_RenderView;

	int width=m_RenderView->w();
	int height=m_RenderView->h();
	
	
	// 32비트 화면 모드에서만 동작.
	
	FastCapture *mF=NULL;
	
	// start capture:
	mF=new FastCapture("../dump/dump.dat");

	mF->Screenshot(m_RenderView, width, height);

	m_RenderView->make_current();
	mF->Screenshot(m_RenderView, width, height);

	...

	// end capture:
	delete mF;
		
*/


class FastCapture
{
protected:
	TString fn;
	int last_x, last_y;
public:

	// opens a dump file
	FastCapture(const char* FileName);

	// optional. If called, a cursor will be drawn to the screenshots.
	virtual void setCursorPos(int x, int y) { last_x=x; last_y=y;}

	// add a screen shot to the dump file
	virtual bool Screenshot(Fl_Window* win, int Width, int Height){return false;}

	// closes the dump file after converting it to jpeg sequence
	virtual ~FastCapture(){}
	virtual void setAccumulation(int n){}
};


class FastCaptureDirect: public FastCapture
{
	unsigned char* lpvBits;	
	intvectorn cursorPos;
	int mWidth, mHeight;
	int m_nCount;
	int m_nCurrAccumulation;
	int m_nTotalAccumulation;
	CImage image;
	CImagePixel tempPixel;
public:

	// opens a dump file
	FastCaptureDirect(const char* FileName);

	virtual void setAccumulation(int n){ m_nCurrAccumulation=0; m_nTotalAccumulation=n;}

	// optional. If called, a cursor will be drawn to the screenshots.
	virtual void setCursorPos(int x, int y) { last_x=x; last_y=y;}

	// add a screen shot to the dump file
	virtual bool Screenshot(Fl_Window* win, int Width, int Height);

	// closes the dump file after converting it to jpeg sequence
	virtual ~FastCaptureDirect();
};
struct XRGB
{
	uchar B;
	uchar G;
	uchar R;
	uchar X;
};

struct RGB565
{
	uchar low;
	uchar high;
	inline uchar R()	{ return high&(~0x7);}
	inline uchar G()	{ return high<<5 | ((low>>5)<<2) ;}
	inline uchar B()	{ return low<<3;}
};

#ifdef _MSC_VER
#include <windows.h>
bool ScreenshotNew(LPCTSTR FileName, HDC SurfDC, int Width, int Height);
#endif
#endif
#endif
