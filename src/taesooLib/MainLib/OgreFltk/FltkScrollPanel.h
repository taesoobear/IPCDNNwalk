#ifndef _SCROLPANAL_H_
#define  _SCROLPANAL_H_

#ifndef NO_GUI


#if _MSC_VER>1000
#pragma once
#endif


class FltkMotionWindow;
class FltkScrollPanel;
class TraceManager;
class AnimationObject;
class TStrings;
class MotionPanel;

#include "../../BaseLib/image/Image.h"
#include "FltkMotionWindow.h"
class FltkScrollPanel : public Fl_Double_Window, public FltkMotionWindow::EventReceiver, public FlCallee
{
public:
	FltkScrollPanel(int x, int y, int w, int h, FltkMotionWindow* pTarget);
	virtual ~FltkScrollPanel();

	void addPanel(const char* filename);
	void addPanel(CImage* pSource);	//!< image will be copyed. You should release pSource
	CImage* createPanel();		//! Create a dynamic panel(you can edit the content of this panel). return empty image. you can create the image by calling CImage::Create(...)
	void addPanel(const bitvectorn& bits, CPixelRGB8 color);
	void addPanel(const bitvectorn& bits, CPixelRGB8 color, int startFrame);
	void addPanel(const intvectorn& indexes, const char* colormapfile=NULL,TStrings* translationTable=NULL);
	void addPanel(const vectorn& input);
	void addPanel(const vectorn& input, double fmin, double fmax);
	void addPanel(const matrixn& input);
	// set label to the last added panel
	void setLabel(const char* label);
	void changeLabel(const char* prevLabel, const char* newLabel);
	void sortPanels();
	void setLastPanelXOffset(int xoffset);

	// After you selected a panel by right clicking on the panel, this function returns the label of the selected panel.
	const char* selectedPanel()	{ return mSelectedPanel.ptr(); }
	void removeAllPanel();
	void removePanel(CImage* pImage);	//! Remove a dynamic panel.
	void changeXpos(CImage* pImage, int xpos);	//! moves a dynamic panel.
	void setCutState(const bitvectorn& abCutState) { m_abCutState=abCutState;}
	const bitvectorn& cutState()	{ return m_abCutState;}

	// Scroll panel의 화면을 드래깅해서 프레임을 선택하는 경우, 그 이벤트를 받는 클래스.
	class SelectUI
	{
	public:

		// double click 's' keys. (that is "ss")
		virtual void click(int iframe){}
		// select frames using a pair of 's' keystrokes.
		virtual void selected(int iframe, int endframe){}
		// select a panel using a right-click on the panel.
		virtual void panelSelected(const char* label, int iframe){}

		// drag an object drawn on a panel using mouse M-button.
		// return true if you want to receive dragging event.
		virtual bool startDragging(const char* label, int iframe, int& adjusedStart, int& adjustedEnd) { return false;}
		virtual void dragging(const char* label, int original_frame, int dragged_frame, int& adjusedStart, int& adjustedEnd){}
		virtual void finalize(const char* label, int original_iframe, int dragged_frame){}
	};

	void connectSelectUI(SelectUI& ui);
	void disconnectSelectUI(SelectUI& ui);
	// Fl_Window::
	virtual void draw();
	virtual int handle(int ev);

	// EventReceiver::
	virtual void OnNext(FltkMotionWindow*);
	virtual void OnPrev(FltkMotionWindow*);
	virtual void OnFrameChanged(FltkMotionWindow*,int currFrame);

	int currFrame() const	{ return m_nCurrFrame;}
	// Fl_Callee::
	virtual void onCallback(Fl_Widget * pWidget, int userData);

	struct Panel
	{
		Panel()										{xoffset=0; mImage=new CImage(); mIsDynamicPanel=false;}
		Panel(CImage* pImage, bool bDynamicPanel)	{xoffset=0; mImage=pImage; mIsDynamicPanel=bDynamicPanel;}
		void release()								{xoffset=0; delete mImage; mImage=NULL;}
		int xoffset;
		~Panel() { release();}
		CImage* mImage;
		bool mIsDynamicPanel;
		TString mLabel;
	};
private:

  /////////////////////////////////////////////////////
	// Frame selection 관련 시작.
	int getCurSel();
	void startSelection();
	void cancelSelection();
	void endSelection();
	void updateSelection();

	int _screenXtoFrame(int screenX);

	int _findPanel(int screenY, int* panelY=NULL);

	TRect mRect;
	void defineOverlayRect(int left, int top, int right, int bottom){ mRect.left =left, mRect.top=top, mRect.right=right, mRect.bottom=bottom;redraw();}
	void clearOverlayRect()	{mRect.left=-1;redraw();}
	// Frame selection 관련 끝.
	/////////////////////////////////////////////////////

	std::list<SelectUI*> m_aSelectUI;
	void drawPanel( int cur_frame, int y);
	void drawPanel( CImage* pSource, int xoffset, int screenY, int deltaY);
	void drawState(int cur_frame);
	int updateRange();

	FltkMotionWindow* mpTarget;
	// UI
	Fl_Slider m_sliderScroll;
	Fl_Check_Button m_bDrawState;
	int mSamplingRatio;
	TString mSelectedPanel;


	// panel
	std::vector<Panel*> m_aSource;
	TRect m_targetRect;
	int m_nCurrFrame;
	int mLeft;
	int getAxisPos();
	void setAxisPos(int );
	void projectAxisPos();

	boolN m_abCutState;
private:
	void _panelAdded();
};
#endif




class CImagePixel;
class FltkScrollSelectPanel: noncopyable
{
private:
	FltkScrollSelectPanel* _impl;
public:
	FltkScrollSelectPanel();
	virtual ~FltkScrollSelectPanel();
	virtual bool isCreated(){ return _impl && _impl->isCreated();}

	virtual int currMaxValue(){ return _impl->currMaxValue();}

	virtual void init(MotionPanel* pPanel, const char* label, int height=1, int maxValue=3);
	virtual void release(MotionPanel* pPanel){
		if(_impl)
			_impl->release(pPanel);
	}
	virtual void drawBoxColormap(int start, int end, int colormapValue)
	{
		_impl->drawBoxColormap(start, end, colormapValue);
	}
	virtual void drawBox(int start, int end, CPixelRGB8  color){
		_impl->drawBox(start,end,color);
	}
	virtual void drawFrameLines(intvectorn const& frames){
		_impl->drawFrameLines(frames);
	}
	virtual void drawTextBox(int start, int end, int colormapValue, const char* text){
		_impl->drawTextBox(start, end, colormapValue, text);
	}
	virtual void clear(int start, int end){
		_impl->clear(start, end);
	}
	virtual CImagePixel& getCIP() { return _impl->getCIP();}
};

#endif
