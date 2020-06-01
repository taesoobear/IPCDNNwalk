#include "stdafx.h"
#ifndef NO_GUI
#include "MotionPanel.h"
#include "FltkMotionWindow.h"
#include "FltkScrollPanel.h"
#include "FltkRenderer.h"
#include "RE.h"
#include "TraceManager.h"
#include "Loader.h"
#include "OgreMotionLoader.h"
#include "../BaseLib/motion/MotionUtil.h"
#include "MotionManager.h"
#include "../BaseLib/motion/MotionRetarget.h"
#include "../BaseLib/motion/FootPrint.h"
#include "../BaseLib/motion/ConstraintMarking.h"
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/math/Operator.h"
#include "FlLayout.h"
#include "../BaseLib/math/intervals.h"
#include "../BaseLib/motion/version.h"
#include "../Ogre/PLDPrimCustumSkin.h"
#include "FL/Fl_Float_Input.H"
#include <FL/Fl_File_Chooser.H>
#include "FlChoice.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreCamera.h>
#include "pldprimskin_impl.h"
#include "Panel.h"
#include "VRMLloader.h"

static int m_argMaxNumFrame=0;
extern int mScaleFactor;
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
class FltkScrollSelectPanel_impl: public FltkScrollSelectPanel
{
	public:
	FltkScrollSelectPanel_impl(){}
	virtual ~FltkScrollSelectPanel_impl();
	CImage* mImage;
	CImagePixel cip;
	CImage icolormap;
	CImagePixel colormap;
	intvectorn colormapIndex;

	virtual CImagePixel& getCIP() { return cip;}
	void create(CImage* pNewImage, int numFrame, int height, int maxValue, const char* colormapfile);
	virtual bool isCreated()
	{ return mImage!=NULL;}
	virtual int currMaxValue();

	virtual void init(MotionPanel* pPanel, const char* label, int height=1, int maxValue=3);
	virtual void release(MotionPanel* pPanel);
	virtual void drawBoxColormap(int start, int end, int colormapValue);
	virtual void drawBox(int start, int end, CPixelRGB8  color);
	virtual void drawFrameLines(intvectorn const& frames);
	virtual void drawTextBox(int start, int end, int colormapValue, const char* text);
	virtual void clear(int start, int end);

};
FltkScrollSelectPanel::FltkScrollSelectPanel()
{
	_impl=NULL;
}
FltkScrollSelectPanel::	~FltkScrollSelectPanel()
{
	delete _impl;
}
void FltkScrollSelectPanel::init(MotionPanel* pPanel, const char* label, int height, int maxValue)
{
	if(!_impl) _impl=new FltkScrollSelectPanel_impl();
	_impl->init(pPanel, label, height, maxValue);
}
/////////////////////////////////////////////////////////////////////////////
// FltkMotionWindow dialog

vector3 temp_v;

void saveViewpoint(FILE* file)
{
#ifdef NO_OGRE
	double fov=45;
	double zoom=1;
#else
	double fov=RE::renderer().viewport().mCam->getFOVy().valueDegrees();
	double zoom=RE::renderer().viewport().m_pViewpoint->getZoom();
#endif
	fprintf(file, "RE.viewpoint():setFOVy(%f)\n", fov);
	fprintf(file, "RE.viewpoint():setZoom(%f)\n", zoom);
	vector3 v=RE::renderer().viewport().m_pViewpoint->m_vecVPos;
	fprintf(file, "RE.viewpoint().vpos:set(%f, %f, %f)\n", v.x, v.y, v.z);
	v=RE::renderer().viewport().m_pViewpoint->m_vecVAt;
	fprintf(file, "RE.viewpoint().vat:set(%f, %f, %f)\n", v.x, v.y, v.z);
	fprintf(file, "RE.viewpoint():update()\n\n");
}
void cb(Fl_Widget *ob, void* userdata) {
	switch((int)reinterpret_cast<long long>(userdata))
	{
	case 0:
		temp_v.x=atof(((Fl_Float_Input*)ob)->value());
		break;
	case 1:
		temp_v.y=atof(((Fl_Float_Input*)ob)->value());
		break;
	case 2:
		temp_v.z=atof(((Fl_Float_Input*)ob)->value());
		break;
	}
}


void retargetEntireMotion(Motion& source, bool bRetargetOnline)
{
#ifdef USE_LUABIND
// constraint retargetting
	m_real minHeight=0.0, maxHeight=4.0, lengthThr=0.95, distThr=0.3;

	try
	{
		LUAwrapper L;
		L.setVal<const char*>("motionId", source.GetIdentifier());
		L.dofile("../resource/constraint.lua");


		L.getVal<double>( "desired_min_height", minHeight);
		L.getVal<double>( "desired_max_height", maxHeight);
	}
    catch(luabind::error& e)
    {
		printf("lua error %s\n", e.what());
    }

	intvectorn aConstraint(2);
	aConstraint[0]=CONSTRAINT_LEFT_TOE;
	aConstraint[1]=CONSTRAINT_RIGHT_TOE;

	CTArray<matrixn> conPos;
	CTArray<intvectorn> interval;

	conPos.Init(2);
	interval.Init(2);

	MotionUtil::FootPrint* pF;

//	if(bRetargetOnline)
		pF=new MotionUtil::GetFootPrintOnline (minHeight, maxHeight);
//	else
		//pF=new MotionUtil::GetFootPrint();-> deprecated

	MotionUtil::FootPrint& footprint=*pF;

	MotionUtil::Retarget2 rt(source);	// perform retargetting by smoothing out sudden inter-frame differences.
	//MotionUtil::RetargetTest rt(source);	// perform IK only for test

	for(int i=0; i<aConstraint.size(); i++)
		footprint.getFootPrints(source, 0, source.numFrames(), aConstraint[i], interval[i], conPos[i]);

	const bool drawFootprints=false;

	if(drawFootprints)
	{
		for(int i=0; i<conPos.Size(); i++)
		{
			//for(int j=0; j<conPos[i].rows(); j++)
			//	RE::moveEntity(RE::createEntity(sz1::format("footprints%d_%d", i,j), "sphere1010.mesh"), quater(1,0,0,0), conPos[i].row(j).toVector3());
			bitvectorn bits;
			bits.setSize(source.numFrames());
			for(int j=0; j<interval[i].size(); j+=2)
			{
				bits.setValue(interval[i][j], interval[i][j+1], true);
			}
			RE::scrollPanel()->addPanel(bits, CPixelRGB8 (255,255,0));
		}

	}
	for(int i=0; i<aConstraint.size(); i++)
		rt.retarget(0, 0, source.numFrames(), source.numFrames(), aConstraint[i], conPos[i], interval[i]);
#else
	assert(false);
#endif
}


class TranslateWin: public FlLayout
{
	MotionPanel* mMP;
public:
	TranslateWin(MotionPanel* mop):
		mMP(mop),
		FlLayout(400, 25*4, "Translate and press x")
	{
		createSlider("x","translate x");
		slider(0)->range(-1000, 1000);
		slider(0)->step(1.0);
		slider(0)->value(0);
		createSlider("y","translate y");
		slider(0)->range(-1000, 1000);
		slider(0)->step(1.0);
		slider(0)->value(0);
		createSlider("z","translate z");
		slider(0)->range(-1000, 1000);
		slider(0)->step(1.0);
		slider(0)->value(0);

		updateLayout();
	}

	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
	{
		if(w.mId=="x" || w.mId=="y" ||w.mId=="z" )
		{
			for(int i=0; i<mMP->motionWin()->getNumSkin(); i++)
				mMP->motionWin()->getSkin(i)->SetTranslation(slider(-2)->value(), slider(-1)->value(), slider(0)->value());
		}
	}
};

class ScaleWin: public FlLayout
{
	MotionPanel* mMP;
public:
	ScaleWin(MotionPanel* mop):
		mMP(mop),
		FlLayout(400, 25*4, "Scale and press x")
	{
		createSlider("scale","scale");
		slider(0)->range(0.01, 100);
		slider(0)->step(0.01);
		slider(0)->value(1.0);

		updateLayout();
	}

	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
	{
		if(w.mId=="scale" )
		{
		  
			double ss=slider(0)->value();
			for(int i=0; i<mMP->motionWin()->getNumSkin(); i++)
			{
				mMP->motionWin()->getSkin(i)->m_pSceneNode->setScale(ss,ss,ss);
/*
				followings are moved to the PLDprimskin.cpp

				PLDPrimSkin* pskin=dynamic_cast<PLDPrimSkin*>(mMP->motionWin()->getSkin(i));
				if(pskin)
				{
					printf("adjusting thickness\n");
					pskin->setThickness(3/ss);
				}*/ 
			}

		}
	}
};

class FltkMotionWindow_impl: public FltkMotionWindow, public Fl_Group, public FlCallee
{
	public:
	Fl_Light_Button m_bButtonPlay;
	Fl_Check_Button m_bButtonHide;
	Fl_Light_Button m_bButtonHideFoot;
	Fl_Button m_bButtonPrev;
	Fl_Button m_bButtonNext;
	Fl_Button m_bButtonHome;
	Fl_Button m_bButtonEnd;
	//	Fl_Fast_Value_Slider m_sliderFrame;
	Fl_Value_Slider m_sliderFrame;
	Fl_Int_Input m_gotoFrame;
	FltkMotionWindow* win;
	public:
	FltkMotionWindow_impl(int x, int y, int w) // standard constructor, height=20 (fixed)
: Fl_Group(x,y,w,40*mScaleFactor),
FltkMotionWindow(x,y,w),
m_bButtonPlay(	  0,0,   80*mScaleFactor,20*mScaleFactor,"@>"),
m_bButtonPrev(	  20*4*mScaleFactor,0,20*mScaleFactor,20*mScaleFactor,"@<|"),
m_bButtonNext(	  20*5*mScaleFactor,0,20*mScaleFactor,20*mScaleFactor,"@|>"),
m_bButtonHome( 	  20*6*mScaleFactor,0,20*mScaleFactor,20*mScaleFactor,"@<<"),
m_bButtonEnd(  	  20*7*mScaleFactor,0,20*mScaleFactor,20*mScaleFactor,"@>>"),
m_bButtonHide(    20*8*mScaleFactor,0,80*mScaleFactor,20*mScaleFactor,"Hide"),
m_bButtonHideFoot(20*8*mScaleFactor,0,80*mScaleFactor,20*mScaleFactor,"Foot"),
m_gotoFrame      (20*11  *mScaleFactor,0*20*mScaleFactor, 50*mScaleFactor, 20*mScaleFactor, "Goto"),
m_sliderFrame	 (0*80  *mScaleFactor,20*mScaleFactor,w,20*mScaleFactor,"Frame")
	{
		m_bButtonHide.hide();
		m_bButtonHideFoot.hide();

		// this (Fl_Window)
		end();
		FlCallee::connect(&m_bButtonPlay);
		m_bButtonPlay.shortcut(FL_CTRL+'p');
		m_bButtonPlay.tooltip("ctrl+p");
		m_bButtonPlay.when(FL_WHEN_CHANGED);
		FlCallee::connect(&m_bButtonPrev);
		m_bButtonPrev.shortcut(FL_ALT+'p');
		m_bButtonPrev.tooltip("prev (alt+P)");
		FlCallee::connect(&m_bButtonNext);
		m_bButtonNext.shortcut(FL_ALT+'n');
		m_bButtonNext.tooltip("next (alt+N)");
		FlCallee::connect(&m_bButtonHome);
		m_bButtonHome.tooltip("home");
		m_bButtonHome.shortcut(FL_CTRL+'h');
		FlCallee::connect(&m_bButtonEnd);
		m_bButtonEnd.tooltip("end");
		FlCallee::connect(&m_bButtonHide);
		FlCallee::connect(&m_bButtonHideFoot);
		FlCallee::connect(&m_sliderFrame);
		FlCallee::connect(&m_gotoFrame);
		m_gotoFrame.tooltip("ctrl+G");

		m_gotoFrame.when(FL_WHEN_ENTER_KEY|FL_WHEN_NOT_CHANGED);

		m_sliderFrame.type(FL_HOR_SLIDER);
		m_sliderFrame.range(0.0, 1.0);
		m_sliderFrame.step(1.0);
		m_sliderFrame.value(0.0);
		m_sliderFrame.align(FL_ALIGN_LEFT);

	}
	virtual void onCallback(Fl_Widget * pWidget, int);
	virtual int FrameMove(float fElapsedTime);
	virtual int handle(int ev);
};

/*
FltkMotionWindow::FltkMotionWindow()

{
	Msg::error("prohibited defaultConstructor");
}*/

FltkMotionWindow::~FltkMotionWindow()
{
}
FltkMotionWindow::FltkMotionWindow(int x, int y, int w)
{

	m_nCurrFrame=0;
	m_numFrame=0;
	RE::renderer().addAfterFrameMoveObject(this);
	//RE::renderer().addFrameMoveObject(this);
}

void PREPAIR_SKIN(const Motion & curMot, TString& meshFile, TString& mappingFile);

/////////////////////////////////////////////////////////////////////////////
// FltkMotionWindow message handlers
void FltkMotionWindow_impl::onCallback(Fl_Widget * pWidget, int userdata)
{
	if(pWidget==&m_gotoFrame)
	{
		int gotoFrame=atoi(m_gotoFrame.value());
		changeCurrFrame(gotoFrame);
	}
	else if(pWidget==&m_bButtonHide)
	{
		onCallback(NULL, Hash("hide skins"));
	}
	else if(userdata==Hash("hide skins"))
	{
		for(int i=0; i<m_vecSkin.size(); i++)
		{
			m_vecSkin[i]->SetVisible(!(bool)m_bButtonHide.value());
		}
	}
	else if(pWidget==&m_bButtonHideFoot)
	{
		for(int i=0; i<m_vecSkin.size(); i++)
		{
			//	m_vecSkin[i]->m_pFootPrint->SetVisible(!(bool)m_bButtonHideFoot.value());
		}
	}
	if(pWidget==&m_bButtonEnd)
	{
		m_bButtonPlay.value(0);
		for(int i=0; i<m_vecSkin.size(); i++)
		{
			m_vecSkin[i]->m_pTimer->InitAnim(1.0f,1.0f,1.0f);
			m_vecSkin[i]->m_pTimer->StopAnim();
		}
	}
	else if(pWidget==&m_bButtonNext)
	{
		if(!m_aEventReceiver.empty())
		{
			for(std::list<EventReceiver *>::iterator i=m_aEventReceiver.begin(); i!=m_aEventReceiver.end(); ++i)
				(*i)->OnNext(this);
		}
		else
			for(int i=0; i<m_vecSkin.size(); i++)
			{
				int iframe=m_vecSkin[i]->m_pTimer->getCurFrameFromInterpolator();
				changeCurrFrame(iframe+1);
			}
	}
	else if(pWidget==&m_bButtonPrev)
	{
		if(!m_aEventReceiver.empty())
		{
			for(std::list<EventReceiver *>::iterator i=m_aEventReceiver.begin(); i!=m_aEventReceiver.end(); ++i)
				(*i)->OnPrev(this);
		}
		else
			for(int i=0; i<m_vecSkin.size(); i++)
			{
				int iframe=m_vecSkin[i]->m_pTimer->getCurFrameFromInterpolator();
				changeCurrFrame(iframe-1);
			}

	}
	else if(pWidget==&m_bButtonHome)
	{
		m_bButtonPlay.value(0);
		for(int i=0; i<m_vecSkin.size(); i++)
		{
			m_vecSkin[i]->m_pTimer->InitAnim( 0.f,1.f,1.f);
			m_vecSkin[i]->m_pTimer->StopAnim();
		}
	}
	else if(pWidget==&m_bButtonPlay)
	{
		if(m_bButtonPlay.value())
		{
			for(int i=0; i<m_vecSkin.size(); i++)
			{
				m_vecSkin[i]->m_pTimer->FirstInit( m_vecSkin[i]->m_pTimer->getTotalTimeFromInterpolator(), false);
				m_vecSkin[i]->m_pTimer->StartAnim();
			}
		}
		else
		{
			for(int i=0; i<m_vecSkin.size(); i++)
			{
				m_vecSkin[i]->m_pTimer->StopAnim();
			}
		}
	}
	else if(pWidget==&m_sliderFrame)
	{
		m_bButtonPlay.value(0);
		for(int i=0; i<m_vecSkin.size(); i++)
		{
			m_vecSkin[i]->m_pTimer->InitAnim(m_sliderFrame.value());
			m_vecSkin[i]->m_pTimer->StopAnim();
		}
		m_sliderFrame.take_focus();
	}
	else return;

}

void FltkMotionWindow::addSkin(AnimationObject* pSkin)
{
	m_vecSkin.push_back(pSkin);
	if(pSkin->m_pTimer)
	{
		pSkin->m_pTimer->StopAnim();
		updateFrameNum();
	}
}

void FltkMotionWindow::connect(EventReceiver& receiver)
{
	m_aEventReceiver.remove(&receiver);
	m_aEventReceiver.push_back(&receiver);
}

void FltkMotionWindow::disconnect(EventReceiver& receiver)
{
	m_aEventReceiver.remove(&receiver);
}

void FltkMotionWindow::releaseAllSkin()
{
	for(int i=0; i<m_vecSkin.size(); i++)
	{
		RE::remove(m_vecSkin[i]);
	}

	detachAllSkin();
}

void FltkMotionWindow::detachAllSkin()
{
	m_vecSkin.clear();

	updateFrameNum();
}



#include "../BaseLib/utility/stdtemplate.h"
void FltkMotionWindow::releaseSkin(AnimationObject* pSkin)
{
	
	RE::remove(pSkin);

	detachSkin(pSkin);
}

void FltkMotionWindow::detachSkin(AnimationObject* pSkin)
{
	vector_remove<AnimationObject*>(m_vecSkin, pSkin);
	updateFrameNum();
}

void FltkMotionWindow::relaseLastAddedSkins(int nskins)
{
	if(nskins<0)
		nskins=m_vecSkin.size()+nskins;

	for(int i=0; i<nskins; i++)
	{
		if(m_vecSkin.size()==0) return;
		RE::remove(m_vecSkin.back());
		m_vecSkin.pop_back();
	}

	updateFrameNum();
}
int FltkMotionWindow::FrameMove(float fElapsedTime)
{ return 1;}
int FltkMotionWindow_impl::FrameMove(float fElapsedTime)
{
	if(m_vecSkin.size()!=0)
	{
		AnimationObject* pSkin=m_vecSkin[m_argMaxNumFrame];

		if(!pSkin->m_pTimer) return 0;
		int cur_frame=pSkin->m_pTimer->getCurFrameFromInterpolator();

		static int PrevFrame=-1;

		if(PrevFrame!=cur_frame)
		{
			m_sliderFrame.value(cur_frame);

			m_nCurrFrame = cur_frame;
			for(std::list<EventReceiver *>::iterator i=m_aEventReceiver.begin(); i!=m_aEventReceiver.end(); ++i)
				(*i)->OnFrameChanged(this,m_nCurrFrame);
		}
		PrevFrame=cur_frame;
	}

	return 1;
}


int FltkMotionWindow_impl::handle(int ev)
{
	switch(ev)
	{
	case FL_MOUSEWHEEL:
		changeCurrFrame(getCurrFrame()+Fl::event_dy());
		break;

	case FL_KEYDOWN:
		if(Fl::event_key()=='g')
		{
			if(Fl::event_ctrl())
			{
				m_gotoFrame.take_focus();
				return 1;
			}
		}
		break;
	//case FL_MOVE:
	//	take_focus();
	}
	return Fl_Group::handle(ev);
}

/*
#define INIT_ANIM(x, y, z, w)	if((x)) (x)->m_pTimer->InitAnim((y),(z),(w))
#define FIRST_INIT(x, y, z)		if((x)) (x)->m_pTimer->FirstInit((y),(z))
#define STOP_ANIM(x)			if((x)) (x)->m_pTimer->StopAnim()
#define START_ANIM(x)			if((x)) (x)->m_pTimer->StartAnim()
*/

void FltkMotionWindow::changeCurrFrame(int iframe)
{
	if(iframe<0) iframe=0;
	if(iframe>=m_numFrame) iframe=m_numFrame-1;

	((FltkMotionWindow_impl*)this)->m_bButtonPlay.value(0);
	for(int i=0; i<m_vecSkin.size(); i++)
	{
		if(m_vecSkin[i]->m_pTimer)
		{
			m_vecSkin[i]->m_pTimer->InitAnim(iframe);
			m_vecSkin[i]->m_pTimer->StopAnim();
		}
	}
}

void FltkMotionWindow::updateFrameNum()
{
	int max_framenum=0;

	for(int i=0; i<m_vecSkin.size(); i++)
	{
		if(m_vecSkin[i]->m_pTimer)
		{
			int num_frame=m_vecSkin[i]->m_pTimer->getNumFrameFromInterpolator();
			if(num_frame>max_framenum)
			{
				max_framenum=num_frame;
				m_argMaxNumFrame=i;
			}
		}
	}

	((FltkMotionWindow_impl*)this)->m_sliderFrame.range(0.0, max_framenum-1);
	m_numFrame=max_framenum;

#define DRAW_TIMELINE
#ifdef DRAW_TIMELINE

	static FltkScrollSelectPanel_impl mPanel;

	if(mPanel.isCreated())
		mPanel.release(&RE::motionPanel());

	mPanel.init(&RE::motionPanel(), "      Timeline");
	intvectorn frames;
	frames.colon(0, m_numFrame, 30);
	mPanel.drawFrameLines(frames);
#endif

}

int FltkMotionWindow::playUntil(int iframe)
{
	for(int i=0; i<m_vecSkin.size(); i++)
	{
		float end_frame=m_vecSkin[i]->m_pTimer->calcCurFrameFromInterpolator(iframe);

		m_vecSkin[i]->m_pTimer->FirstInit( m_vecSkin[i]->m_pTimer->getTotalTimeFromInterpolator(), false);
		m_vecSkin[i]->m_pTimer->InitAnim(m_vecSkin[i]->m_pTimer->GetCurFrame(), end_frame, end_frame);
		m_vecSkin[i]->m_pTimer->StartAnim();
	}
	return 1;
}

int FltkMotionWindow::playFrom(int iframe)
{
	for(int i=0; i<m_vecSkin.size(); i++)
	{
		m_vecSkin[i]->m_pTimer->FirstInit( m_vecSkin[i]->m_pTimer->getTotalTimeFromInterpolator(), true);
		m_vecSkin[i]->m_pTimer->InitAnim(iframe);
		m_vecSkin[i]->m_pTimer->StartAnim();
	}
	return 1;
}
FlMenu* FltkScrollPanel_mMenu;

FltkScrollPanel::FltkScrollPanel(int x, int y, int w, int h, FltkMotionWindow* pTarget)
: Fl_Double_Window(x,y,w,h),
//m_sliderScroll(w-20,20,20,h-20),
m_sliderScroll(w-20,0,20,h),
m_bDrawState(160,0,80,20,"Draw State"),
mSamplingRatio(1)
{
	mRect.left=-1;
	// this (Fl_Window) 그룹????났??
	resizable(this);

	FlMenu* o=new FlMenu();
	FltkScrollPanel_mMenu=o;
	o->initChoice(0,0, 20, 20);
	o->size(12);
	o->item(0, " ");
	o->item(1, "Add Panel",0, Hash("AdPn"));
	o->item(2, "Remove All", 0, Hash("Remv"));
	o->item(3, "Remove last added", 0, Hash("Remove last"));
	o->item(4, "Toggle draw state", 0, Hash("Toggle draw state"), FL_MENU_TOGGLE);
	o->item(5, "Mark discontinuities", 0, Hash("Mark discontinuities"));
	o->beginSubMenu(6, "change sampling ratio");
	o->item(7, "1x", 0, Hash("change sampling ratio"));
	o->item(8, "2x", 0, Hash("change sampling ratio"));
	o->item(9, "4x", 0, Hash("change sampling ratio"));
	o->item(10, "8x", 0, Hash("change sampling ratio"));
	o->endSubMenu(11);

	connect(*o);

	m_bDrawState.value(1);
	m_bDrawState.hide();
/*
	{
		FlMenu* o=new FlMenu();
		o->initChoice(80,0, 80, 20);
		o->size(4);
		o->item(0, "1x",0, Hash("change sampling ratio"));
		o->item(1, "2x",0, Hash("change sampling ratio"));
		o->item(2, "4x", 0, Hash("change sampling ratio"));
		o->item(3, "8x", 0, Hash("change sampling ratio"));
		o->value(0);
		mSamplingRatio=1;

		connect(*o);
	}
*/
	resizable(new Fl_Box(0, 20, w-20, h-20));
	end();
	m_sliderScroll.type(FL_VERT_SLIDER);
	m_sliderScroll.range(0.0, 100);
	m_sliderScroll.step(1.0);

	connect(&m_sliderScroll, Hash("SldS"));
	connect(&m_bDrawState, Hash("DrwS"));
	pTarget->connect(*this);
	mpTarget=pTarget;
	mUI=NULL;

	m_nCurrFrame=0;
	mLeft=-1*((w-20)/2);
}


FltkScrollPanel::~FltkScrollPanel()
{
	delete FltkScrollPanel_mMenu;
}


int selectionStart=-1;

int FltkScrollPanel::getCurSel()
{
	return m_nCurrFrame;
}

void FltkScrollPanel::startSelection()
{
  selectionStart=getCurSel();
	//make_current();
	defineOverlayRect(selectionStart, 0, selectionStart+1, h());

	Msg::print2("selection start");
}

void FltkScrollPanel::endSelection()
{
	printf("endSeclection called!!!\n"); // added by jae
	clearOverlayRect();

	int x1=selectionStart;
	int x2=getCurSel();
	updateRange();
	int minx=MIN(x1,x2);
	int maxx=MAX(x1,x2);

	if(mUI)
	{
		if(minx==maxx)
			mUI->click(minx);
		else
			mUI->selected(minx, maxx+1);
	}
	Msg::print2("selection [%d, %d] end", minx, maxx);
}

void FltkScrollPanel::cancelSelection()
{
	clearOverlayRect();
	redraw();
	Msg::print2("selection canceled");
}

void FltkScrollPanel::updateSelection()
{
	//make_current();

	defineOverlayRect(selectionStart, 0, getCurSel(), h());
}

int FltkScrollPanel::handle(int ev)
{
	static int push_x=-1;
	static int push_y=-1;
	static bool bButton1=false;
	static bool bButton2=false;
	static int currFrame;

	// for frame selection
	static int currLeft;

	// for domain selection using key stroke 's'
	enum { SELECTION_NONE, SELECTION_READY, SELECTING};
	static int selectionState=SELECTION_READY;

	switch(ev)
	{

	case FL_ACTIVATE:
	case FL_DEACTIVATE:
	case FL_HIDE:
	case FL_SHOW:
		// ??????지 경우??뭔?? ??줘????듯. ??단?? ??작??
		break;
	case FL_ENTER:
		//Msg::print2("enter");

		return 1;	// becomes belowmouse widget

	case FL_LEAVE:
		//Msg::print2("leave");
		break;

	case FL_KEYUP:
	//case FL_KEYDOWN:

		if(Fl::event_key()=='s' && !(Fl::event_state()&FL_CTRL) && !(Fl::event_state()&FL_ALT))
		{
			//Msg::print2("keyup s");
			if(selectionState==SELECTION_READY)
			{
				startSelection();
				selectionState=SELECTING;
			}
			else if(selectionState==SELECTING)
			{
				endSelection();
				selectionState=SELECTION_READY;
			}
			return 1;
		}
		break;
	case FL_PUSH:
		{
			RE::setGlobalMousePos(Fl::event_x()+x()+parent()->x(), Fl::event_y()+y()+parent()->y());

			push_x=Fl::event_x();
			push_y=Fl::event_y();
			if((push_y>20 || (push_y>0 && push_x> 80))&& push_x<w()-20 )
			{
				bButton1=(bool)(Fl::event_state()&FL_BUTTON1);
				bButton2=(bool)(Fl::event_state()&FL_BUTTON2);
				if(bButton2)
				{
					// middle button
					int panelY;
					int panel=_findPanel(Fl::event_y(), &panelY);
					if(panel!=-1 && mUI)
					{
						int adjustedStart, adjustedEnd;
						int iframe=_screenXtoFrame(push_x);
						bool res=mUI->startDragging(m_aSource[panel]->mLabel, iframe, adjustedStart, adjustedEnd);
						if(res)
						{
							defineOverlayRect(adjustedStart, panelY,
												adjustedEnd, panelY+m_aSource[panel]->mImage->GetHeight());
							return 1;
						}
					}
					return 0;
				}
				else if(bButton1)
				{
					currLeft=_screenXtoFrame(push_x);//m_nCurrFrame+=push_x
					mpTarget->changeCurrFrame(currLeft);
				}
				else
				{
					currLeft=mLeft;
				}
				currFrame=m_nCurrFrame;
				return 1;
			}

			break;
		}
	case FL_MOVE:
		{
			RE::setGlobalMousePos(Fl::event_x()+x()+parent()->x(), Fl::event_y()+y()+parent()->y());
			if(selectionState==SELECTING)
				updateSelection();
		}
		break;

	case FL_DRAG:
		{
			RE::setGlobalMousePos(Fl::event_x()+x()+parent()->x(), Fl::event_y()+y()+parent()->y());

			if(bButton1)
			{
				mpTarget->changeCurrFrame(currLeft+(Fl::event_x()-push_x)*mSamplingRatio);
			}
			else if(bButton2)
			{
				// middle button
				int panelY;
				int panel=_findPanel(Fl::event_y(), &panelY);
				Msg::verify(mUI, "sp???");
				int originalframe=_screenXtoFrame(push_x);
				int iframe=_screenXtoFrame(Fl::event_x());
				int adjustedStart, adjustedEnd;

				if(panel!=-1 )
				{
					mUI->dragging(m_aSource[panel]->mLabel, originalframe, iframe, adjustedStart, adjustedEnd);
					mRect.top=panelY;
					mRect.bottom=panelY+m_aSource[panel]->mImage->GetHeight();
				}
				else
					mUI->dragging(NULL, originalframe, iframe, adjustedStart, adjustedEnd);

				mRect.left=adjustedStart;
				mRect.right=adjustedEnd;
				redraw();
				return 1;
			}
			else
			{
				int delta=(Fl::event_x()-push_x)*mSamplingRatio;

				int newFrame=currFrame-delta;
				newFrame=CLAMP(newFrame, 0, mpTarget->getNumFrame()-1);
				mpTarget->changeCurrFrame(newFrame);
				m_nCurrFrame=newFrame;
				mLeft=currLeft-delta;
				projectAxisPos();
				redraw();
				return 1;
			}
			if(selectionState==SELECTING)
				updateSelection();

			return 1;
		}
	case FL_RELEASE:
		{


			if(bButton1)
			{

			}
			else if(bButton2)
			{
				// middle button
				int panelY;
				int panel=_findPanel(Fl::event_y(), &panelY);
				Msg::verify(mUI, "sp???");
				int originalframe=_screenXtoFrame(push_x);
				int iframe=_screenXtoFrame(Fl::event_x());

				if(panel!=-1 )
				{
					mUI->finalize(m_aSource[panel]->mLabel, originalframe, iframe);
				}
				else
					mUI->finalize(NULL, originalframe, iframe);

				clearOverlayRect();
				redraw();
				return 1;
			}
			else
			{
				if(push_x==Fl::event_x() && push_y==Fl::event_y())
				{
					// select a panel.
					if(m_aSource.size())
					{
						int j=_findPanel(Fl::event_y());
						if(j!=-1)
						{
							mSelectedPanel=m_aSource[j]->mLabel;
							if(mSelectedPanel.length())
								redraw();

							printf("panel selected\n");

							if(mUI) mUI->panelSelected(mSelectedPanel, _screenXtoFrame(push_x));
							return 1;
						}
					}
				}
			}
			return 1;
		}
		break;
	}
	return Fl_Double_Window::handle(ev);
}

void FltkScrollPanel::addPanel(CImage* pImage)
{
	m_aSource.push_back(new Panel(CImageProcessor::Clone(pImage), false));
	redraw();
}
static bool comparePanel(FltkScrollPanel::Panel* a, FltkScrollPanel::Panel* b)
{
	if (a->mLabel.length()==0 ) return true;
	if (b->mLabel.length()==0 ) return false;
	return strcmp(a->mLabel.ptr(), b->mLabel.ptr())<0;
}
#include <algorithm>
void FltkScrollPanel::sortPanels()
{
	std::sort(m_aSource.begin(), m_aSource.end(), comparePanel);
	redraw();
}

void FltkScrollPanel::removePanel(CImage* pImage)
{
  	for(int i=0; i<m_aSource.size(); i++)

	{
		if(m_aSource[i]->mImage==pImage)
		{
			delete m_aSource[i];
			for(int j=i+1; j<m_aSource.size(); j++)
				m_aSource[j-1]=m_aSource[j];
			m_aSource.resize(m_aSource.size()-1);
			redraw();
			break;
		}
	}
}

void FltkScrollPanel::changeXpos(CImage* pImage, int xpos)
{
  	for(int i=0; i<m_aSource.size(); i++)

	{
		if(m_aSource[i]->mImage==pImage)
		{
			m_aSource[i]->xoffset=xpos;
			redraw();
			break;
		}
	}
}

void FltkScrollPanel::setLabel(const char* label)
{
	m_aSource[m_aSource.size()-1]->mLabel=label;
}

void FltkScrollPanel::changeLabel(const char* prevLabel, const char* newLabel)
{
	for(int i=0; i<m_aSource.size(); i++)
	{
		if(m_aSource[i]->mLabel==prevLabel)
		{
			m_aSource[i]->mLabel=newLabel;
			break;
		}
	}
}

CImage* FltkScrollPanel::createPanel()
{
	m_aSource.resize(m_aSource.size()+1);
	m_aSource.back()=new Panel();
	m_aSource[m_aSource.size()-1]->mIsDynamicPanel=true;
	return m_aSource[m_aSource.size()-1]->mImage;
}

void FltkScrollPanel::removeAllPanel()
{
	int numDynamic=0;
	for(int i=0; i<m_aSource.size(); i++)
	{
		if(m_aSource[i]->mIsDynamicPanel)
		{
			m_aSource[numDynamic]=m_aSource[i];
			numDynamic++;
		}
		else
		{
			delete m_aSource[i];
		}
	}
	m_aSource.resize(numDynamic);
	redraw();
}
void FltkScrollPanel::addPanel(const char* filename)
{
  m_aSource.resize(m_aSource.size()+1);
  m_aSource.back()=new Panel();
	if(!m_aSource[m_aSource.size()-1]->mImage->Load(filename))
		Msg::error("add panel failed %s!", filename);
	redraw();
}

void FltkScrollPanel::addPanel(const vectorn& input)
{
	double minv=0;
	double maxv=0;
	m::drawSignals("__temp.bmp", input.column(), minv, maxv);	
	addPanel("__temp.bmp");
}
void FltkScrollPanel::addPanel(const vectorn& input, double minv, double maxv)
{
	m::drawSignals("__temp.bmp", input.column(), minv, maxv);	
	addPanel("__temp.bmp");
}
void FltkScrollPanel::setLastPanelXOffset(int xoffset)
{
	m_aSource.back()->xoffset=xoffset;
	redraw();
}
void FltkScrollPanel::addPanel(const matrixn& input)
{
	double minv=0;
	double maxv=0;
	m::drawSignals("__temp.bmp", input, minv, maxv);	
	addPanel("__temp.bmp");
}


void FltkScrollPanel::addPanel(const intvectorn& bits, const char* colormapfile, TStrings* translationTable)
{
	m_aSource.push_back(new Panel(CImageProcessor::DrawChart(bits,colormapfile), false));
	if(translationTable)
		m_aSource.push_back(new Panel(CImageProcessor::DrawChartText(bits, translationTable),false));
	redraw();
}



void FltkScrollPanel::addPanel(const bitvectorn& bits, CPixelRGB8 color)
{
	m_aSource.push_back(new Panel(CImageProcessor::DrawChart(bits, color),false));
	redraw();
}
void FltkScrollPanel::addPanel(const bitvectorn& bits, CPixelRGB8 color, int startFrame)
{
	Panel* p=new Panel(CImageProcessor::DrawChart(bits, color), false);
	p->xoffset=startFrame;
	m_aSource.push_back(p);
	redraw();
}

int FltkScrollPanel::_screenXtoFrame(int screenX)
{
	int currLeft=mLeft;
	return currLeft+screenX*mSamplingRatio;
}

int FltkScrollPanel::_findPanel(int mouseY, int* panelY)
{
	int nCurrPosition=0;
	int screenY=0;
	int y=m_sliderScroll.value();
	for(int i=0; i<m_aSource.size(); i++)
	{
		nCurrPosition+=m_aSource[i]->mImage->GetHeight();
		if(nCurrPosition>y)
		{
			nCurrPosition-=m_aSource[i]->mImage->GetHeight();
			int deltaY=y-nCurrPosition;

			for(int j=i; j<m_aSource.size(); j++)
			{
				if(screenY > m_targetRect.bottom) return 1;
				TRect rectDest=m_targetRect;
				rectDest.top=screenY+m_targetRect.top;
				rectDest.bottom=rectDest.top+m_aSource[i]->mImage->GetHeight();
				if(rectDest.top<=mouseY && mouseY <rectDest.bottom)
				{
					if(panelY) *panelY=rectDest.top;
					return j;
				}
				screenY+=m_aSource[j]->mImage->GetHeight()-deltaY;
				deltaY=0;
			}
		}
	}
	return -1;
}
void FltkScrollPanel::updateRange()
{
	m_targetRect.left=0;
	m_targetRect.right=w()-20;
	//m_targetRect.top=20;
	m_targetRect.top=0;
	m_targetRect.bottom=h();


	int sum=0;

	for(int i=0; i<m_aSource.size(); i++)
		sum+=m_aSource[i]->mImage->GetHeight();

	m_sliderScroll.range(0,MAX(0,sum-m_targetRect.Height()));

}

inline static int _getAxisPos(int currFrame, int mLeft, int mSamplingRatio)
{
	return (currFrame-mLeft)/mSamplingRatio;
}
int FltkScrollPanel::getAxisPos()
{
	return _getAxisPos(m_nCurrFrame,mLeft,mSamplingRatio);
}

void FltkScrollPanel::setAxisPos(int axisPos)
{
	mLeft=m_nCurrFrame-axisPos*mSamplingRatio;
}

void FltkScrollPanel::projectAxisPos()
{
	// check if axis pos is visible.
	int axisPos=getAxisPos();

	int minPos=(w()-20)/10;
	int maxPos=(w()-20)-minPos;
	if(axisPos<minPos)
		setAxisPos(minPos);

	if(axisPos>maxPos)
		setAxisPos(maxPos);

}
void FltkScrollPanel::OnFrameChanged(FltkMotionWindow*,int currFrame)
{
	// project currframe (when sampling ratio is not one.)
	currFrame=currFrame/mSamplingRatio*mSamplingRatio;
	m_nCurrFrame=currFrame;

	projectAxisPos();
	redraw();
}

void FltkScrollPanel::draw()
{
	updateRange();

	Fl_Double_Window::draw();

	int axis=getAxisPos();

	/*
	if(m_aSource.size())
	{
	draw_children();
	drawPanel(m_nCurrFrame, m_sliderScroll.value());

	int delta;

	fl_rectf(240,0, m_targetRect.Width(), 20, FL_BACKGROUND_COLOR	);
	if(m_nCurrFrame<axis)
	fl_rectf(0,20, axis-m_nCurrFrame, m_targetRect.Height(), FL_BACKGROUND_COLOR	);
	if((delta= m_aSource[0].GetWidth()-m_nCurrFrame)<axis)
	fl_rectf(axis+delta,20,axis-delta, m_targetRect.Height(), FL_BACKGROUND_COLOR	);
	}
	else
	Fl_Window::draw();


	*/

	if(m_aSource.size())
		drawPanel(m_nCurrFrame, m_sliderScroll.value());

	if(m_abCutState.size() && m_bDrawState.value() )
		drawState(m_nCurrFrame);

	Fl_Color col;
	col=fl_color();
#define DRAW_YELLOW_SCROLLLINE
#ifdef DRAW_YELLOW_SCROLLLINE
	fl_color(255,255,0);
	//fl_line_style(FL_DOT);
	fl_yxline(axis+m_targetRect.left, m_targetRect.top, m_targetRect.bottom);
#else
	fl_color(0,0,0);
	fl_yxline(axis+m_targetRect.left, m_targetRect.top, m_targetRect.bottom);
	fl_line_style(FL_DOT);
	fl_color(255,255,255);
	fl_yxline(axis+m_targetRect.left, m_targetRect.top, m_targetRect.top+20);
#endif
	if(mRect.left!=-1)
	{
		TRect newRect=mRect;
		if(newRect.left>newRect.right)
			std::swap(newRect.left, newRect.right);

		int axis=getAxisPos();
		int left=axis+(newRect.left-m_nCurrFrame)/mSamplingRatio;
		int right=axis+(newRect.right-m_nCurrFrame)/mSamplingRatio;
		fl_draw_box(FL_PLASTIC_UP_FRAME, left, newRect.top, right-left+1, newRect.Height(), FL_MAGENTA);

		/*fl_yxline(mRect.left, mRect.top, mRect.bottom-1);
		fl_yxline(mRect.right-1, mRect.top, mRect.bottom-1);
		fl_xyline(mRect.left, mRect.top, mRect.right-1);
		fl_xyline(mRect.left, mRect.bottom-1, mRect.right-1);*/
	}

	fl_line_style(0);
	fl_color(col);


	Fl_Double_Window::draw_children();
}



void FltkScrollPanel::drawPanel( int cur_frame, int y)
{
	int nCurrPosition=0;
	int screenY=0;
	for(int i=0; i<m_aSource.size(); i++)
	{
		nCurrPosition+=m_aSource[i]->mImage->GetHeight();
		if(nCurrPosition>y)
		{
			nCurrPosition-=m_aSource[i]->mImage->GetHeight();
			int deltaY=y-nCurrPosition;

			for(int j=i; j<m_aSource.size(); j++)
			{
				if(screenY > m_targetRect.bottom) return;
				drawPanel(m_aSource[j]->mImage, m_aSource[j]->xoffset, screenY, deltaY);


				if(m_aSource[j]->mLabel.length())
				{
					Fl_Color col;
					col=fl_color();


					intvectorn x(8, 1, 1, 1 , 0, 0 ,-1, -1, -1);
					intvectorn y(8, 1, 0, -1, 1, -1, 1, 0, -1);

					if(mSelectedPanel.length()&& mSelectedPanel==m_aSource[j]->mLabel)
						fl_color(col);
					else
						fl_color(255,255,255);

					for(int k=0; k<x.size(); k++)
						fl_draw(m_aSource[j]->mLabel, 5+x[k], screenY+m_targetRect.top-fl_descent()+fl_height()+y[k]);

					if(mSelectedPanel.length()&& mSelectedPanel==m_aSource[j]->mLabel)
						fl_color(255,255,255);
					else
						fl_color(col);
					fl_draw(m_aSource[j]->mLabel, 5, screenY+m_targetRect.top-fl_descent()+fl_height());
					fl_color(col);
				}

				screenY+=m_aSource[j]->mImage->GetHeight()-deltaY;
				deltaY=0;
			}


			//fl_rectf(0,screenY,m_targetRect.Width(), m_targetRect.Height()-screenY, FL_BACKGROUND_COLOR	);
			return;
		}
	}
}

void fl_draw_CImage_scaleDown(int samplingRatio, const CImage& imagee, const TRect& sourceRect, int x, int y);

void FltkScrollPanel::drawPanel( CImage* pSource, int xoffset, int screenY, int deltaY)
{
	TRect rectCrop=m_targetRect;
	TRect rectDest=m_targetRect;
	int axis=getAxisPos();

	// correspondences
	//   0,    axis,    Width()

	//  left  cur_frame    right

	rectCrop.left=mLeft-xoffset;
	rectCrop.right=rectCrop.left+m_targetRect.Width()*mSamplingRatio;

	int height=m_targetRect.Height();

	rectDest.top=screenY+m_targetRect.top;

	rectCrop.top=deltaY;
	rectCrop.bottom=deltaY+rectDest.Height();

	//fl_draw_CImage(*pSource, rectCrop, 0, rectDest.top);
	fl_draw_CImage_scaleDown(mSamplingRatio,*pSource, rectCrop, 0, rectDest.top);
}

void FltkScrollPanel::drawState(int cur_frame)
{
	bitvectorn& abState=m_abCutState;

	TRect rectCrop=m_targetRect;
	int axis=getAxisPos();

	rectCrop.left=cur_frame-axis;
	rectCrop.right=rectCrop.left+m_targetRect.Width();

	Fl_Color col;
	col=fl_color();
	fl_color(255,0,0);

	fl_line_style(FL_DOT);
	for(int i=rectCrop.left; i<rectCrop.right; i++)
	{
		if(i>=0 && i<abState.size())
		{
			if(abState[i])
			{
				fl_yxline(i-rectCrop.left+m_targetRect.left, m_targetRect.top, m_targetRect.bottom);
			}
		}
	}

	fl_line_style(0);
	fl_color(col);
}

void FltkScrollPanel::OnNext(FltkMotionWindow* pMW)
{
	if(m_abCutState.size())
	{
		int iframe=pMW->getCurrFrame();
		int num_frame=pMW->getNumFrame();

		for(int i=iframe+1; i<num_frame; i++)
		{
			if(m_abCutState[i])
			{
				pMW->playUntil(i);
				return;
			}
		}
	}
}

void FltkScrollPanel::OnPrev(FltkMotionWindow* pMW)
{
	// TODO: Add your control notification handler code here
	int iframe=pMW->getCurrFrame();

	if(m_abCutState.size()>0)
	{
		for(int i=iframe-1; i>=0; i--)
		{
			if(m_abCutState[i])
			{
				pMW->changeCurrFrame(i);
				return;
			}
		}
	}
}

void FltkScrollPanel::onCallback(Fl_Widget * pWidget, int userData)
{
	if(userData==Hash("Toggle draw state"))
	{
		if(m_bDrawState.value())
			m_bDrawState.value(0);
		else if(mSamplingRatio==1)
			m_bDrawState.value(1);
		else Msg::msgBox("draw state is not supported in supersampling mode");
		redraw();
	}
	else if(userData==Hash("Mark discontinuities"))
	{
		setCutState(RE::motionPanel().currMotionWrap().getDiscontinuity());
	}
	else if(userData==Hash("change sampling ratio"))
	{
		TString samplingRatio=((Fl_Choice* )pWidget)->text();

		int axisPos=getAxisPos();
		mSamplingRatio=atoi(samplingRatio.left(1));
		// project currframe (when sampling ratio is not one.)
		m_nCurrFrame=m_nCurrFrame/mSamplingRatio*mSamplingRatio;

		setAxisPos(axisPos);
		if(mSamplingRatio!=1)
			m_bDrawState.value(0);
		redraw();
	}
	if(pWidget==&m_sliderScroll )
		redraw();

	if(userData==Hash("AdPn"))
	{
		Fl_File_Chooser fc(".", "*.{bmp,bit}", Fl_File_Chooser::SINGLE, "Open BMP,BIT file");
		fc.show();
		while (fc.visible())
			Fl::wait();

		if (fc.count() > 0)
		{
			TString fn=fc.value();
			if(fn.right(3).toUpper()=="BMP")
				addPanel(fc.value());
			else
			{
				boolN bits;
				bits.load(cutState().size(), fn);
				setCutState(bits);
				addPanel(bits, CPixelRGB8 (0,0,0));
			}
		}
	}
	else if(userData==Hash("Remv"))
	{
		removeAllPanel();
	}
	else if(userData==Hash("Remove last"))
	{
		int i=m_aSource.size()-1;

		if(!m_aSource[i]->mIsDynamicPanel)
		{
			m_aSource[i]->release();
			m_aSource.resize(i);
			redraw();
		}
	}
}

namespace RE
{
	extern Globals* g_pGlobals;
}


MotionPanel::MotionPanel(int x, int y, int w, int h)
: Fl_Double_Window(x,y,w,h)
{
	bool layout1=true;
	if(w>350)
	{
	m_loader=new Loader(w-180*mScaleFactor,0,180*mScaleFactor,40*mScaleFactor, this);
	m_motionWin=new FltkMotionWindow_impl(0, 0, w-180*mScaleFactor);
	m_scrollPanel=new FltkScrollPanel(0, 40*mScaleFactor, w, h-40*mScaleFactor, m_motionWin);
	m_traceManager=new TraceManager(0, 40*mScaleFactor, w, h-40*mScaleFactor);
	}
	else{
	m_loader=new Loader(0,60*mScaleFactor,w,40*mScaleFactor, this);
	m_motionWin=new FltkMotionWindow_impl(0, 0, w);
	m_scrollPanel=new FltkScrollPanel(0, 80*mScaleFactor, w, h-40*mScaleFactor, m_motionWin);
	m_traceManager=new TraceManager(0, 80*mScaleFactor, w, h-40*mScaleFactor);
	layout1=false;
	}

	resizable(m_scrollPanel);
	resizable(m_traceManager);

	int crx=320-80-80;
	int cry=0;

	/*
	Fl_Widget* o=new Fl_Check_Button (crx, 0,100, 20,"Show output"); crx+=80;
	o->labelsize(11);
	((Fl_Light_Button *)o)->value(0);
	connect(o, Hash("Oupt"));
*/

	//crx+=100; //-> align to fltkToolkitRenderer
	if(layout1)
	{
		crx=w-(180+200)*mScaleFactor; // -> align to loader
		crx+=100*mScaleFactor;
	}
	else
	{
		crx=0;
		cry+=40*mScaleFactor;
	}

	m_menuOp.initChoice(crx,cry, 100*mScaleFactor,20*mScaleFactor, "");crx+=100*mScaleFactor;

	if(layout1)
		cry+=20*mScaleFactor;
	m_menuMotion.initChoice(crx,cry, (100+80)*mScaleFactor, 20*mScaleFactor);

	int nitem=76;
	m_menuOp.size(nitem);

	int item=0;
	m_menuOp.beginSubMenu(item++, "Show motions");
	m_menuOp.item(item++, "Show all motions",FL_CTRL+'M', Hash("Show all motions"));
	m_menuOp.item(item++, "Show all motions (low poly)",0, Hash("Show motions (lowpoly)"));
	m_menuOp.item(item++, "Show all motions (point)",0, Hash("Show motions (point)"));
	m_menuOp.item(item++, "Show all motions (elipsoid)",0, Hash("Show motions (elipsoid)"));
	m_menuOp.item(item++, "Show all motions (box)",0, Hash("Show motions (box)"));
	m_menuOp.item(item++, "Show skined motions",0, Hash("Show skined motions"));
	m_menuOp.item(item++, "Change skins",0, Hash("Change skins"));
	m_menuOp.item(item++, "Scale skins",0, Hash("Scale skins"));
	m_menuOp.item(item++, "Translate all skins",0, Hash("Translate skins"));
	m_menuOp.item(item++, "Hide skins", 0, Hash("hide skins"));
	m_menuOp.item(item++, "Release skins",0, Hash("Rlss"));
	m_menuOp.item(item++, "Release motions",0, Hash("Release motions"), FL_MENU_DIVIDER);
	m_menuOp.item(item++, "Show Bone",0, Hash("Bone"));
	m_menuOp.item(item++, "Show motion",0, Hash("Show"));
	m_menuOp.item(item++, "Show motion using skin",0, Hash("ShoS"));
	m_menuOp.item(item++, "Show motion using skin (dual quaternion)",0, Hash("ShoS2"));
	m_menuOp.item(item++, "Change skin (automatic)",0, Hash("SkinAuto"));
	m_menuOp.item(item++, "Change skin from a mapping table",0, Hash("Skin"));
	m_menuOp.item(item++, "Reset skin",0, Hash("RSSk"));
	m_menuOp.item(item++, "Change coordinate",0, Hash("Change coordinate"));
	m_menuOp.endSubMenu(item++);
	m_menuOp.beginSubMenu(item++, "Export");
	m_menuOp.item(item++, "Export motion",0, Hash("ExMo"));
	m_menuOp.item(item++, "Export BVH",0, Hash("Export BVH"));
	m_menuOp.item(item++, "Export BVH (ZXY)",0, Hash("Export BVH (ZXY)"));
	m_menuOp.item(item++, "Export all to BVH",0, Hash("Export all to BVH"));
	m_menuOp.item(item++, "Export pose",0, Hash("Export pose"));
	m_menuOp.item(item++, "Export constraints",0, Hash("Export constraints"));
	m_menuOp.endSubMenu(item++);
	m_menuOp.beginSubMenu(item++, "Constraints");
	m_menuOp.item(item++, "Import constraints",0, Hash("Import constraints"));
	m_menuOp.item(item++, "Calc constraint",0,Hash("Con"));
	m_menuOp.item(item++, "Calc constraint pos", 0, Hash("Calc constraint pos"));
	m_menuOp.item(item++, "Calc support foot",0,Hash("SprF"));
	m_menuOp.item(item++, "Show left constraint signals",0,Hash("LCon"));
	m_menuOp.item(item++, "Show right constraint signals",0,Hash("RCon"));
	m_menuOp.item(item++, "Show constraints (signal)",0, Hash("constraints"));
	m_menuOp.item(item++, "Save constraints (signal)",0, Hash("SaveConstraints"));
	m_menuOp.item(item++, "Show constraints (screen)",0, Hash("constraintsSc"));
	m_menuOp.item(item++, "Constraint retarget",0,Hash("Retarget"));
	m_menuOp.item(item++, "Constraint retarget (approx)",0,Hash("RetargetApprox"));
	m_menuOp.endSubMenu(item++);
	m_menuOp.beginSubMenu(item++, "Renderer");
	m_menuOp.item(item++, "Set playback speed",0, Hash("Set speed"));
	m_menuOp.item(item++, "Toggle background",0, Hash("Toggle background"));
	m_menuOp.item(item++, "Toggle skybox", 0, Hash("TgSb"));
	m_menuOp.item(item++, "Change background color", 0, Hash("ChBg"));
	m_menuOp.item(item++, "Change shadow technique", 0, Hash("ChSh"));
	m_menuOp.item(item++, "Toggle logo", 0, Hash("TgLg"));
	m_menuOp.item(item++, "Toggle cursor", 0, Hash("TgCs"));
	m_menuOp.item(item++, "Show bounding boxes", 0, Hash("Show bounding boxes"), FL_MENU_TOGGLE);
	m_menuOp.item(item-1).clear();
	m_menuOp.item(item++, "Capture (capture.jpg)", 0, Hash("Capt"));
	m_menuOp.item(item++, "Capture (jpeg sequence)", FL_CTRL+'c', Hash("capture"));
	m_menuOp.item(item++, "Convert captured data", 0, Hash("Convert captured data"));
	m_menuOp.item(item++, "Save current viewpoint", 0, Hash("Save current viewpoint"));
	m_menuOp.item(item++, "Show output (render window)", FL_CTRL+FL_ALT+'o', Hash("OgreTraceManager"));
	m_menuOp.item(item++, "SetCaptureFPS", 0, Hash("SetCaptureFPS"));
	m_menuOp.endSubMenu(item++);
	m_menuOp.beginSubMenu(item++, "Viewpoint");
	m_menuOp.item(item++, "Save view (slot 1)",FL_CTRL+FL_ALT+'1', Hash("Save view (slot 1)"));
	m_menuOp.item(item++, "Save view (slot 2)",FL_CTRL+FL_ALT+'2', Hash("Save view (slot 2)"));
	m_menuOp.item(item++, "Save view (slot 3)",FL_CTRL+FL_ALT+'3', Hash("Save view (slot 3)"));
	m_menuOp.item(item++, "Save view (slot 4)",FL_CTRL+FL_ALT+'4', Hash("Save view (slot 4)"));
	m_menuOp.item(item++, "Save view (slot 5)",FL_CTRL+FL_ALT+'5', Hash("Save view (slot 5)"));
	m_menuOp.item(item++, "Change view (slot 1)",FL_ALT+'1', Hash("Change view (slot 1)"));
	m_menuOp.item(item++, "Change view (slot 2)",FL_ALT+'2', Hash("Change view (slot 2)"));
	m_menuOp.item(item++, "Change view (slot 3)",FL_ALT+'3', Hash("Change view (slot 3)"));
	m_menuOp.item(item++, "Change view (slot 4)",FL_ALT+'4', Hash("Change view (slot 4)"));
	m_menuOp.item(item++, "Change view (slot 5)",FL_ALT+'5', Hash("Change view (slot 5)"));
	m_menuOp.endSubMenu(item++);
	m_menuOp.item(item++, "Plot ignored frames",0,Hash("Ignr"));
	m_menuOp.item(item++, "Add supersampled motion",0,Hash("Supr"));
	m_menuOp.item(item++, "Translate motion",0,Hash("Trans"));
	m_menuOp.item(item++, "Run scripts",FL_CTRL+'R', Hash("Run scripts"));
	m_menuOp.item(item++, "Show output", 0, Hash("Oupt"), FL_MENU_TOGGLE);
	connect(m_menuOp);
	assert(item==nitem);

	end();

	m_traceManager->hide();

	RE::g_pGlobals->pMotionPanel=this;
}

MotionPanel::~MotionPanel(void)
{
	RE::g_pGlobals->pMotionPanel=NULL;
}
#ifdef _MSC_VER
#ifdef USE_LUABIND
void FastCapture_convert(const char* filename);
#else
void FastCapture_convert(const char* filename){}
#endif
#endif

void FlChooseFiles(const char* message, const char* path, const char* Mask, TStrings& files);


void showBoundingBox(Ogre::SceneNode* node, bool bValue)
{
	if(node->getName()!="BackgroundNode")
	{
		Ogre::Node::ChildNodeIterator it=node->getChildIterator();

		while(it.hasMoreElements())
		{
			Ogre::Node* childnode=it.getNext();
			Ogre::SceneNode* childsceneNode;
			childsceneNode=dynamic_cast<Ogre::SceneNode*>(childnode);
			if(childsceneNode)
				showBoundingBox(childsceneNode, bValue);
		};
		node->showBoundingBox(bValue);
	}
}
void MotionPanel::onCallback(Fl_Widget* pWidget, int userdata)
{
	if(userdata==Hash("Show bounding boxes"))
	{
		Fl_Menu_* p=(Fl_Menu_* )pWidget;
		if((p->menu())[p->value()].value())
		{
			showBoundingBox(RE::ogreRootSceneNode(), true);
		}
		else
		{
			showBoundingBox(RE::ogreRootSceneNode(), false);
		}
	}
	else if(userdata==Hash("Save current viewpoint"))
	{
		TString fn=FlChooseFile("Input filename to create", "../Resource/scripts/","*.{lua,sim}", true);

		if(fn.length())
		{
			FILE* script=fopen(fn, "wt");
			saveViewpoint(script);
			fclose(script);
		}
	}
	else if(userdata==Hash("capture"))
	{
		RE::renderer().toggleScreenShotMode();
	}
#ifdef _MSC_VER
	else if(userdata==Hash("Convert captured data"))
	{
		TStrings fn;

		FlChooseFiles("Choose a file to convert", "../dump/", "*.dat", fn);

		for(int i=0; i<fn.size(); i++)
		{
			FastCapture_convert(fn[i]);
		}
	}
#endif
	else if(userdata==Hash("Ignr"))
	{
		intIntervals ignoreInterval;
		ignoreInterval.load(TString("../resource/")+currMotion().GetIdentifier()+".ignore");
		boolN bitv;
		ignoreInterval.toBitvector(bitv);
		m_scrollPanel->addPanel(bitv, CPixelRGB8 (255,0,255));
	}
	else if(userdata==Hash("Con"))
	{
		Imp::ChangeChartPrecision(50);
		Loader::constraintAutomaticMarking(currMotion());
		Imp::DefaultPrecision();

	}
	else if(userdata==Hash("ConNoXDX") || userdata==Hash("ConNoD"))
	{

		bool bConNoXDX=(userdata==Hash("ConNoXDX"));

		MotionUtil::SegmentFinder segFinder(currMotion(), 0, currMotion().numFrames());

		bitvectorn lcon;
		bitvectorn rcon;
		bitvectorn both;
		intIntervals interval;
		for(int iseg=0; iseg<segFinder.numSegment(); iseg++)
		{
			int start=segFinder.startFrame(iseg);
			int end=segFinder.endFrame(iseg);

			MotionUtil::GetSignal get(currMotion());
			get.constraint(CONSTRAINT_LEFT_TOE, lcon, start, end);
			get.constraint(CONSTRAINT_RIGHT_TOE, rcon, start, end);

			both._and(lcon, rcon);

			interval.runLengthEncode(both);
			for(int i=0; i<interval.size(); i++)
			{
				int startD=interval.start(i);
				int endD=interval.end(i);
				startD+=start;
				endD+=start;

				// constraint??된???찾기.
				int nconPrev=(startD-1>=0 && currMotion().isConstraint(startD-1, CONSTRAINT_LEFT_TOE))?CONSTRAINT_RIGHT_TOE:CONSTRAINT_LEFT_TOE;
				int nconNext=(endD<currMotion().numFrames()&& currMotion().isConstraint(endD, CONSTRAINT_LEFT_TOE))?CONSTRAINT_RIGHT_TOE:CONSTRAINT_LEFT_TOE;


				if(nconPrev == nconNext)
				{
					if(endD-startD>currMotion().NumFrames(0.3))
					{
						// LDL -> LRL???고침
						// RDR -> RLR???고침
						for(int fr=startD; fr<endD; fr++)
						{
							if(nconPrev==CONSTRAINT_LEFT_TOE)
								currMotion().setConstraint(fr, CONSTRAINT_RIGHT_TOE, false);
							else
								currMotion().setConstraint(fr, CONSTRAINT_LEFT_TOE, false);
						}
					}
					else
					{
						// LDL->L??? RDR->R???고침.
						for(int fr=startD; fr<endD; fr++)
						{
							if(nconPrev==CONSTRAINT_LEFT_TOE)
								currMotion().setConstraint(fr, CONSTRAINT_LEFT_TOE, false);
							else
								currMotion().setConstraint(fr, CONSTRAINT_RIGHT_TOE, false);
						}
					}
				}
				else if(!bConNoXDX)
				{
					// LDR -> LR
					// RDL -> RL
					int center=(startD+endD)/2;
					for(int fr=startD; fr<center; fr++)
						currMotion().setConstraint(fr, nconPrev, false);
					for(int fr=center; fr<endD; fr++)
						currMotion().setConstraint(fr, nconNext, false);
				}
			}
		}

		MotionUtil::GetSignal gs(currMotion());
		bitvectorn con;
		gs.constraint(CONSTRAINT_LEFT_TOE, con);
		m_scrollPanel->addPanel(con, CPixelRGB8 (255,0, 255));
		gs.constraint(CONSTRAINT_RIGHT_TOE, con);
		m_scrollPanel->addPanel(con, CPixelRGB8 (0,255, 255));
	}
	else if(userdata==Hash("constraints"))
	{
		MotionUtil::GetSignal gs(currMotion());
		bitvectorn con;
		gs.constraint(CONSTRAINT_LEFT_TOE, con);
		m_scrollPanel->addPanel(con, CPixelRGB8 (255,0, 255));
		gs.constraint(CONSTRAINT_RIGHT_TOE, con);
		m_scrollPanel->addPanel(con, CPixelRGB8 (255,0, 255));
	}
	else if(userdata==Hash("SaveConstraints"))
	{
		MotionUtil::GetSignal gs(currMotion());
		bitvectorn conL, conR;
		gs.constraint(CONSTRAINT_LEFT_TOE, conL);
		gs.constraint(CONSTRAINT_RIGHT_TOE, conR);

		printf("muaythai2 constraints saved!!");
		BinaryFile file;
		file.openWrite("muaythai2_con.con");
		file.pack(conL);
		file.pack(conR);
		file.close();
	}
	else if(userdata==Hash("constraintsSc"))
	{
		for(int i=0; i<m_motionWin->getNumSkin(); i++)
		{
			PLDPrimSkin* pSkin=((PLDPrimSkin*)m_motionWin->getSkin(i));
			pSkin->setDrawConstraint(CONSTRAINT_LEFT_TOE, 4, RE::RED);
			pSkin->setDrawConstraint(CONSTRAINT_RIGHT_TOE, 4, RE::RED);
			pSkin->setDrawOrientation(0);
		}
	}
	else if(userdata==Hash("LCon"))
	{
		m_scrollPanel->addPanel("foot_left_height.bmp");
		m_scrollPanel->addPanel("foot_left_speed.bmp");

		MotionUtil::GetSignal gs(currMotion());
		bitvectorn con;
		gs.constraint(CONSTRAINT_LEFT_FOOT, con);
		m_scrollPanel->addPanel(con, CPixelRGB8 (255,0, 255));
	}
	else if(userdata==Hash("RCon"))
	{
		m_scrollPanel->addPanel("foot_right_height.bmp");
		m_scrollPanel->addPanel("foot_right_speed.bmp");

		MotionUtil::GetSignal gs(currMotion());
		bitvectorn con;
		gs.constraint(CONSTRAINT_RIGHT_FOOT, con);
		m_scrollPanel->addPanel(con, CPixelRGB8 (0,255, 255));
	}
	else if(userdata==Hash("Rlss"))
	{
		m_motionWin->releaseAllSkin();
	}
	else if(userdata==Hash("Release motions"))
	{
		releaseMotions();
	}
	else if(userdata==Hash("Translate skins"))
	{
		(new TranslateWin(this))->show();

	}
	else if(userdata==Hash("Run scripts"))
	{
		try
		{

			LUAwrapper L;
			Register_baselib(L.L);
			Register_mainlib(L.L);

			TString file=FlChooseFile("Open LUA file", "../Resource/scripts/ui/MotionPanel/", "*.lua", false);

			if (file.length())
			{
				printf("%s", file.ptr());
				L.dofile(file);
			}
		}
		catch(std::exception& e)
		{
			Msg::msgBox("c++ error : %s", e.what());
			ASSERT(0);
		}
		catch (char* error)
		{
			fl_message("%s", error);
		}
		catch(...)
		{
			fl_message("some error");
			ASSERT(0);
		}

	}
	else if(userdata==Hash("Supr"))
	{
		std::vector<Motion*> motions;
		OnLoadStart(1, motions);
		int choice=fl_choice("How much?", "4x", "3x", "2x");//0,1,2
		int x=4-choice;
		MotionUtil::upsample(*motions[0],currMotion(),x);
		OnLoadEnd();
	}
	else if(userdata==Hash("ExMo"))
	{
		Fl_File_Chooser fc("../resource/motion", "*.mot", Fl_File_Chooser::SINGLE | Fl_File_Chooser::CREATE, "Export .MOT file");
		fc.show();
		while (fc.visible())
			Fl::wait();

		if (fc.count() > 0)
		{
			TString filename(fc.value(1));
			currMotion().exportMOT(fc.value(1));
		}
	}
	else if(userdata==Hash("Export skeleton"))
	{
		Fl_File_Chooser fc("../resource/motion", "*.skl", Fl_File_Chooser::SINGLE | Fl_File_Chooser::CREATE, "Export .SKL file");
		fc.show();
		while (fc.visible())
			Fl::wait();

		if (fc.count() > 0)
		{
			TString filename(fc.value(1));
			currMotion().skeleton().exportSkeleton(filename.left(-4)+".skl");
		}
	}
	else if(userdata==Hash("Export constraints"))
	{
		TString confn=FlChooseFile("Choose file", "../resource/motion/","*.bits",true);
		if(confn.length())
		{
			bitvectorn bitsL, bitsR;

			MotionUtil::GetSignal gs(currMotion());
			gs.constraint(CONSTRAINT_LEFT_TOE, bitsL);
			gs.constraint(CONSTRAINT_RIGHT_TOE, bitsR);

			BinaryFile f(true, confn);
			f.packInt(2);
			f.pack(bitsL);
			f.pack(bitsR);
			f.close();
		}
	}
	else if(userdata==Hash("Import constraints"))
	{
		TString confn=FlChooseFile("Choose file", "../resource/motion/","*.bits");
		if(confn.length())
		{
			bitvectorn bitsL, bitsR;

			BinaryFile f(false, confn);
			Msg::verify(f.unpackInt()==2, "con size error");
			f.unpack(bitsL);
			f.unpack(bitsR);
			f.close();


			MotionUtil::SetSignal ss(currMotion());
			ss.constraint(CONSTRAINT_LEFT_TOE, bitsL);
			ss.constraint(CONSTRAINT_RIGHT_TOE, bitsR);
		}
	}

	else if(userdata==Hash("Export pose"))
	{
		Fl_File_Chooser fc("./", "*.pose", Fl_File_Chooser::SINGLE | Fl_File_Chooser::CREATE, "Export BVH file");
		fc.show();
		while (fc.visible())
			Fl::wait();

		if (fc.count() > 0)
		{
			FlLayout o(400, 10+25+25+5, "Choose a frame to export, and press x");

			o.createSlider("frame","frame");
			o.slider(0)->range(0, currMotion().numFrames());
			o.slider(0)->step(5.0);
			o.slider(0)->value(0);
			o.updateLayout();

			o.show();
			while(o.visible())
				Fl::wait();

			savePose(currMotion().pose(o.slider(0)->value()), fc.value(1));
		}
	}
	else if(userdata==Hash("Export all to BVH"))
	{
		int numSkin=m_motionWin->getNumSkin();

		int c=0;
		for(int i=0; i<numSkin; i++)
		{
			if(m_motionWin->getSkin(i)->getType()=="PLDPrimSkin" )
			{
				PLDPrimSkin* pSkin=(PLDPrimSkin* )m_motionWin->getSkin(i);

				if(pSkin->m_pTimer)
				{
					AlzzaPostureIP* pInterpolator=(AlzzaPostureIP*)(pSkin->m_pTimer->GetFirstInterpolator());
					MotionUtil::exportBVH(pInterpolator->targetMotion(), sz1::format("export%0d.bvh", c++));
				}
			}

		}

	}
	else if(userdata==Hash("Export BVH") || userdata==Hash("Export BVH (ZXY)"))

	{
		Fl_File_Chooser fc("./", "*.bvh", Fl_File_Chooser::SINGLE | Fl_File_Chooser::CREATE, "Export BVH file");
		fc.show();
		while (fc.visible())
			Fl::wait();

		if (fc.count() > 0)
		{
			FlLayout o(400, 10+25+25+5, "Choose range to export, and press x");

			o.setUniformGuidelines(10);
			o.create("Value_Slider", "start","start",1);
			o.slider(0)->range(0, currMotion().numFrames());
			o.slider(0)->step(1.0);
			o.create("Value_Slider", "end", "end",1);
			o.slider(0)->range(0, currMotion().numFrames());
			o.slider(0)->step(1.0);
			o.slider(0)->value(currMotion().numFrames());
			o.updateLayout();

			o.show();
			while(o.visible())
				Fl::wait();

			printf("%f %f\n", o.slider(-1)->value(), o.slider(0)->value());


			if(userdata==Hash("Export BVH (ZXY)"))
			{
				for(int i=1; i<currMotion().skeleton().GetNumTreeNode(); i++)
				{
					Bone& bone=currMotion().skeleton().getBoneByTreeIndex(i);

					TString rot;
					TString trans;
					trans=bone.getTranslationalChannels();
					rot=bone.getRotationalChannels();
					if(rot.length())
						rot="ZXY";
					bone.setChannels(trans, rot);
				}
			}

			int start=int(o.slider(-1)->value());
			int end=int(o.slider(0)->value());

			if(Msg::confirm("Do you want to translate the first frame horizontally to the center?"))
			{
				vector3 trans=currMotion().pose(start).m_aTranslations[0];
				trans*=-1;
				trans.y=0;
				MotionUtil::translate(currMotion(), trans);
			}
			if(Msg::confirm("Do you want to rotate the first frame horizontally to face front?"))
			{
				quater rot;
				rot.setAxisRotation(vector3(0,1,0), currMotion().pose(start).front(), vector3(0,0,1));
				MotionUtil::rotate(currMotion(), rot);
			}

			MotionUtil::exportBVH(currMotion(), fc.value(1), start, end);
		}

	}
	else if(userdata==Hash("Show"))
	{
		m_motionWin->addSkin(RE::createSkin(currMotion()));
	}
	else if(userdata==Hash("Change coordinate"))
	{
		FlLayout o(400, 25*5, "Choose coordinate, and press x");
		o.createMenu("Coordinate", "Coordinate");
		o.menu(0)->size(3);
		o.menu(0)->item(0, "Local coordinate");
		o.menu(0)->item(1, "Aligned coordinate");
		o.menu(0)->item(2, "Fixed coordinate");
		o.menu(0)->value(1);
		o.updateLayout();

		o.show();
		while(o.visible())
			Fl::wait();

		int eCoord;
		switch(o.menu(0)->value())
		{
		case 0: eCoord=Motion::LOCAL_COORD;
			break;
		case 1: eCoord=Motion::FIRST_ARRANGED_COORD;
			break;
		case 2: eCoord=Motion::FIXED_COORD;
			break;
		}

		currMotion().ChangeCoord(eCoord);
	}
	else if(userdata==Hash("Show motions (point)"))
	{
		for(int i=0; i<m_motions.size(); i++)
		{
			PLDPrimSkin* pSkin=RE::createSkin(m_motions[i], RE::PLDPRIM_POINT);
			PLDPrimPoint* pBone=dynamic_cast<PLDPrimPoint*> (pSkin);
			if(i==1 && pBone)
				pBone->setMaterial("blue");
			m_motionWin->addSkin(pSkin);
		}

	}
	else if(userdata==Hash("Show motions (box)"))
	{
		for(int i=0; i<m_motions.size(); i++)
		{
			PLDPrimSkin* pSkin=RE::createSkin(m_motions[i], RE::PLDPRIM_BOX);
			PLDPrimPoint* pBone=dynamic_cast<PLDPrimPoint*> (pSkin);
			if(i==1 && pBone)
				pBone->setMaterial("blue");
			m_motionWin->addSkin(pSkin);
		}

	}

	else if(userdata==Hash("Show motions (elipsoid)"))
	{
		for(int i=0; i<m_motions.size(); i++)
		{
			PLDPrimSkin* pSkin=RE::createSkin(m_motions[i],RE::PLDPRIM_BLOB);
			PLDPrimBone* pBone=dynamic_cast<PLDPrimBone*> (pSkin);
			if(i==1 && pBone)
				pBone->setMaterial("green");
			m_motionWin->addSkin(pSkin);
		}
	}
	else if(userdata==Hash("Show all motions") || userdata==Hash("Show motions (lowpoly)"))
	{
		bool lowPoly=false;
		if(userdata==Hash("Show motions (lowpoly)"))
			lowPoly=true;
		for(int i=0; i<m_motions.size(); i++)
		{
			PLDPrimSkin* pSkin;
			if(lowPoly)
				pSkin=RE::createSkin(m_motions[i], RE::PLDPRIM_CYLINDER_LOWPOLY);
			else
				pSkin=RE::createSkin(m_motions[i], RE::PLDPRIM_CYLINDER);

			PLDPrimCyl* pBone=dynamic_cast<PLDPrimCyl*> (pSkin);
			if(i==1 && pBone)
				pBone->setMaterial("blue");
			m_motionWin->addSkin(pSkin);
		}
	}
	else if(userdata==Hash("ShoS2"))
	{
#ifdef INCLUDE_OGRESKINENTITY
		Motion& curMot=currMotion();

		PLDPrimSkin* pSkin=RE::createCustumSkin(curMot);
		m_motionWin->addSkin(pSkin);
#endif
	}
	else if(userdata==Hash("ShoS"))
	{
		Motion& curMot=currMotion();

		TString meshFile, mappingFile;
		PREPAIR_SKIN(curMot, meshFile, mappingFile);

		PLDPrimSkin* pSkin=RE::createOgreSkin(curMot.skeleton(),
			RE::renderer().viewport().mScene->createEntity(RE::generateUniqueName().ptr(), meshFile.ptr()),
			mappingFile);

		pSkin->ApplyAnim(currMotion());
		m_motionWin->addSkin(pSkin);
	}
	else if(userdata==Hash("Show skined motions"))
	{
		for(int i=0; i<m_motions.size(); i++)
		{
			Motion& curMot=m_motions[i];

			TString meshFile, mappingFile;
			PREPAIR_SKIN(curMot, meshFile, mappingFile);

			PLDPrimSkin* pSkin=RE::createOgreSkin(curMot.skeleton(),
				RE::renderer().viewport().mScene->createEntity(RE::generateUniqueName().ptr(), meshFile.ptr()),
				mappingFile);

			pSkin->ApplyAnim(curMot);
			m_motionWin->addSkin(pSkin);
		}
	}
	else if(userdata==Hash("SkinAuto"))
	{
		Motion& curMot=currMotion();

		TString meshFile, mappingFile;
		PREPAIR_SKIN(curMot, meshFile, mappingFile);

		RE::changeDefaultSkin(curMot.skeleton(),
			RE::renderer().viewport().mScene->createEntity(RE::generateUniqueName().ptr(), meshFile.ptr()),
				mappingFile);
	}
	else if(userdata==Hash("SkinAuto2"))
	{
		Motion& curMot=currPairMotion();

		TString meshFile, mappingFile;
		PREPAIR_SKIN(curMot, meshFile, mappingFile);

		RE::changeDefaultSkin(curMot.skeleton(),
			RE::renderer().viewport().mScene->createEntity(RE::generateUniqueName().ptr(), meshFile.ptr()),
				mappingFile);
	}
	else if(userdata==Hash("Change skins"))
	{
		for(int i=0; i<m_motions.size(); i++)
		{
			Motion& curMot=m_motions[i];


			TString meshFile, mappingFile;
			PREPAIR_SKIN(curMot, meshFile, mappingFile);

			RE::changeDefaultSkin(curMot.skeleton(),
				RE::renderer().viewport().mScene->createEntity(RE::generateUniqueName().ptr(), meshFile.ptr()),
				mappingFile);
		}

	}
	else if(userdata==Hash("Scale skins"))
	{
		(new ScaleWin(this))->show();
	}
	else if(userdata==Hash("Bone"))
	{
		m_motionWin->addSkin(RE::createSkin(currMotion().skeleton()));
	}
	else if(userdata==Hash("Oupt"))
	{

		Fl_Choice* p=(Fl_Choice* )pWidget;
		if(p->menu()[p->value()].value())
		{
			m_scrollPanel->hide();
			m_traceManager->show();
		}
		else
		{
			m_traceManager->hide();
			m_scrollPanel->show();
		}


	}
	else if(userdata==Hash("Skin"))
	{
		TString model;
		TString mapping;

		// choose skinmodel
		Fl_File_Chooser fc("../media/models", "*.mesh", Fl_File_Chooser::SINGLE, "Choose skin model");
		fc.show();
		while (fc.visible())
			Fl::wait();

		if (fc.count() > 0)
		{
			model=fc.value();

			// choose mapping table
			{
				Fl_File_Chooser fc("../resource/motion", "*.txt", Fl_File_Chooser::SINGLE, "Choose mapping table");
				fc.show();
				while (fc.visible())
					Fl::wait();

				if (fc.count() > 0)
				{
					currMotion().setSkeleton(0);

					TString name;
					static int nId=0;
					name.format("_%s%d", currMotion().GetIdentifier(), nId);
					nId++;
					RE::changeDefaultSkin(currMotion().skeleton(),
						RE::renderer().viewport().mScene->createEntity(name.ptr(), model.ptr()),
						fc.value() );
				}
			}

		}

	}
	else if(userdata==Hash("RSSk"))
	{
		RE::resetDefaultSkin(currMotion().skeleton());
	}
	else if(userdata==Hash("Trans"))
	{
		vector3 trans;
		Fl_Window *window=new Fl_Window(400,400);
		int y=0;
		{
			Fl_Widget* o=new Fl_Float_Input(70, y, 200, 30,"X:"); y+=35;
			o->callback(cb, 0);
			o=new Fl_Float_Input(70, y, 200, 30,"Y:"); y+=35;
			o->callback(cb, (void*)1);
			o=new Fl_Float_Input(70, y, 200, 30,"Z:"); y+=35;
			o->callback(cb, (void*)2);
			window->end();
			window->show();
			while (window->visible())
				Fl::wait();

			Msg::msgBox("translation %s", temp_v.output().ptr());

			MotionUtil::translate(currMotion(), temp_v);
		}
	}
	else if(userdata==Hash("Calc constraint pos"))
	{
		Motion& mot=(Motion&)currMotion();
		MotionUtil::ConstraintMarking cm((Motion*)(&mot), false);
		cm.calcConstraintPos(CONSTRAINT_LEFT_TOE);
		cm.calcConstraintPos(CONSTRAINT_RIGHT_TOE);
	}
	else if(userdata==Hash("Retarget") || userdata==Hash("RetargetApprox"))
	{

		Motion& source=(Motion&)currMotion();

		retargetEntireMotion(source, userdata==Hash("RetargetApprox"));

		Msg::msgBox("Done! If results look strange, it's possibly because constraint positions were not pre-calculated");
	}
	else RE::FltkRenderer().onCallback(pWidget, userdata);
}

void MotionPanel::OnLoadStart(int numCharacter, std::vector<Motion*>& targetMotions)
{
	Motion* mot1, *mot2;
	m_motions.resize(m_motions.size()+numCharacter);
	mPairMotionIndex.resize(m_motions.size()+numCharacter);

	if(numCharacter==1)
	{
		mot1=&m_motions[m_motions.size()-1].mot();
		mot2=NULL;
		mPairMotionIndex[m_motions.size()-1]=-1;
		targetMotions.push_back(mot1);
	}
	else if (numCharacter==2)
	{
		mot1=&m_motions[m_motions.size()-2].mot();
		mot2=&m_motions[m_motions.size()-1].mot();
		mPairMotionIndex[m_motions.size()-2]=m_motions.size()-1;
		mPairMotionIndex[m_motions.size()-1]=m_motions.size()-2;
		targetMotions.push_back(mot1);
		targetMotions.push_back(mot2);
	}
}
void MotionPanel::OnLoadStart(int numCharacter, std::vector<MotionDOFinfo const*>&aInfo, std::vector<MotionDOFcontainer*>& targetMotions)
{
	MotionDOFcontainer* mot1, *mot2;
	m_motions.resize(m_motions.size()+numCharacter);
	mPairMotionIndex.resize(m_motions.size()+numCharacter);

	if(numCharacter==1)
	{
		m_motions[m_motions.size()-1].setMotionDOF(*aInfo[0]);
		mot1=&m_motions[m_motions.size()-1].motdofc();
		mot2=NULL;
		mPairMotionIndex[m_motions.size()-1]=-1;
		targetMotions.push_back(mot1);
	}
	else if (numCharacter==2)
	{
		m_motions[m_motions.size()-2].setMotionDOF(*aInfo[0]);
		m_motions[m_motions.size()-1].setMotionDOF(*aInfo[1]);
		mot1=&m_motions[m_motions.size()-2].motdofc();
		mot2=&m_motions[m_motions.size()-1].motdofc();
		mPairMotionIndex[m_motions.size()-2]=m_motions.size()-1;
		mPairMotionIndex[m_motions.size()-1]=m_motions.size()-2;
		targetMotions.push_back(mot1);
		targetMotions.push_back(mot2);
	}
}


Motion const& MotionPanel::registerMotion(Motion const& mot)
{
	std::vector<Motion*> targetMotions;
	OnLoadStart(1, targetMotions);
	*targetMotions[0]=mot;

	OnLoadEnd();
	return *targetMotions[0];
}
void MotionPanel::registerMotion(MotionDOF const& mot)
{
	m_motions.resize(m_motions.size()+1);
	int imot=m_motions.size()-1;
	// there is no pair motion.
	mPairMotionIndex.resize(imot+1);
	mPairMotionIndex[imot]=-1;
	m_motions[imot].setMotionDOF((MotionDOF&)mot);

	OnLoadEnd();
}

void MotionPanel::releaseMotions()
{
	//m_motionWin->releaseAllSkin();
	m_motions.resize(0);
	mPairMotionIndex.resize(0);
	m_menuMotion.size(1);
	m_menuMotion.item(0,0);
	m_menuMotion.value(0);
	connect(m_menuMotion);
	m_menuMotion.redraw();
}

void MotionPanel::OnLoadEnd()
{
	m_menuMotion.size(m_motions.size());
	for(int i=0; i<m_motions.size(); i++)
	{
        m_menuMotion.item(i, m_motions[i].getIdentifier());
	}

	m_menuMotion.value(0);
    connect(m_menuMotion);
	m_menuMotion.redraw();
}

class MotionPanel_impl
{
	public:
		static void checkMotionMenu(MotionPanel& self)
		{
			if(self.m_menuMotion.value()>=self.m_motions.size())
			{
				Msg::print("Warning! Motion is not loaded\n");
				Msg::print("Default motion is loading\n");

				self.loader()->onCallback(NULL, Hash("LOAD"));
			}
		}
};
Motion& MotionPanel::currMotion()
{
	MotionPanel_impl::checkMotionMenu(*this);
	return m_motions[m_menuMotion.value()];
}
MotionDOF& MotionPanel::currMotionDOF()
{
	MotionPanel_impl::checkMotionMenu(*this);
	return m_motions[m_menuMotion.value()];
}
void MotionPanel::changeCurrMotion(Motion const& mot)
{
	int found=-1;
	for(int i=0; i<m_motions.size(); i++)
	{
		if (!m_motions[i].hasMotionDOF() && &m_motions[i].mot()==&mot)
		{
			found=i;
			break;
		}
	}
	if(found!=-1)
	{
		m_menuMotion.value(found);
		m_menuMotion.redraw();
	}
}
MotionDOFcontainer& MotionPanel::currMotionDOFcontainer()
{
	MotionPanel_impl::checkMotionMenu(*this);
	return m_motions[m_menuMotion.value()].motdofc();
}
MotionWrap& MotionPanel::currMotionWrap()
{
	MotionPanel_impl::checkMotionMenu(*this);
	return m_motions[m_menuMotion.value()];
}
bool MotionPanel::hasMotionDOF()
{
	MotionPanel_impl::checkMotionMenu(*this);
	return m_motions[m_menuMotion.value()].hasMotionDOF();
}

bool MotionPanel::hasPairMotion()
{
	if(m_menuMotion.value()>=m_motions.size())
	{
		throw(std::runtime_error("Motion is not loaded!"));
	}
	return mPairMotionIndex[m_menuMotion.value()]!=-1;
}

Motion& MotionPanel::currPairMotion()
{
	if(m_menuMotion.value()>=m_motions.size())
	{
		throw(std::runtime_error("Motion is not loaded!"));
	}

	return m_motions[mPairMotionIndex[m_menuMotion.value()]];
}


void Panel::setMaxValue(int maxValue)
{
	colormapIndex.makeSamplingIndex2(icolormap.GetWidth(), maxValue+1);
}

void Panel::createPanel(CImage *pNewImage, int numFrame, int maxLen, int maxValue, const char* colormapfile)
{
#define FONT_HEIGHT 16
	create(pNewImage, numFrame, 20+FONT_HEIGHT*maxLen, maxValue, colormapfile);
}

#ifdef DrawText
#undef DrawText
#endif
void Panel::drawSegmentText(int start, int end, int value, TString const & text)
{
	for(int i=start; i<end; i++)
	{
		cip.DrawVertLine(i, 0, 20, colormap.GetPixel(colormapIndex[value],0));
		cip.DrawVertLine(i, 20, cip.Height()-20, CPixelRGB8 (0,0,0));
	}

	for(int i=0; i<text.length(); i++)
		cip.DrawText(start,20+FONT_HEIGHT*i, text.subString(i,i+1));
}

void Panel::drawBox(int start, int end, int colormapValue)
{
	for(int i=start; i<end; i++)
		cip.DrawVertLine(i, 0, cip.Height(), colormap.GetPixel(colormapIndex[colormapValue],0));
}

void Panel::drawTextBox(int start, int end, int colormapValue, const char* text)
{
	for(int i=start; i<end; i++)
		cip.DrawVertLine(i, 0, cip.Height(), colormap.GetPixel(colormapIndex[colormapValue],0));
	cip.DrawText(start, 0, text, true, CPixelRGB8 (255,255,255));
}

void Panel::clear(int start, int end)
{
	if(start<0) start=0;
	if(end>cip.Width()) end=cip.Width();
	if(start<end)
		cip.DrawBox(TRect(start, 0, end, cip.Height()), CPixelRGB8 (0,0,0));
}



void FltkScrollSelectPanel_impl::create(CImage* pNewImage, int numFrame, int height, int maxValue, const char* colormapfile)
{
	mImage=pNewImage;
	mImage->Create(numFrame, height);//, 24);

	cip.Init(mImage);

	icolormap.Load(colormapfile);
	colormap.Init(&icolormap);
	colormapIndex.makeSamplingIndex2(icolormap.GetWidth(), maxValue+1);
}


int FltkScrollSelectPanel_impl::currMaxValue()
{
	return colormapIndex.size()-1;
}

void FltkScrollSelectPanel_impl::init(MotionPanel* pPanel, const char* label, int height, int maxValue)
{
#ifdef _DEBUG
	static MotionPanel* mPanel=NULL;
	ASSERT(mPanel==NULL || mPanel==pPanel);
	mPanel=pPanel;
#endif
	if(isCreated())	release(pPanel);
	mImage=pPanel->scrollPanel()->createPanel();
	pPanel->scrollPanel()->setLabel(label);
	create(mImage, pPanel->motionWin()->getNumFrame(), 16*height, maxValue, "../Resource/default/colormap.bmp");
	clear(0, pPanel->motionWin()->getNumFrame());
}

void FltkScrollSelectPanel_impl::release(MotionPanel* pPanel)
{
	pPanel->scrollPanel()->removePanel(mImage);
	mImage=NULL;
}

FltkScrollSelectPanel_impl::~FltkScrollSelectPanel_impl()
{
	if(mImage && RE::motionPanelValid()){
		release(&RE::motionPanel());
	}
}
void FltkScrollSelectPanel_impl::drawBoxColormap(int start, int end, int colormapValue)
{
	for(int i=start; i<end; i++)
		cip.DrawVertLine(i, 0, cip.Height(), colormap.GetPixel(colormapIndex[colormapValue],0));
}

void FltkScrollSelectPanel_impl::drawBox(int start, int end, CPixelRGB8 color)
{
	for(int i=start; i<end; i++)
		cip.DrawVertLine(i, 0, cip.Height(), color);
}

void FltkScrollSelectPanel_impl::drawFrameLines(intvectorn const& frames)
{
	clear(0, cip.Width());

	for(int i=0; i<frames.size(); i++)
	{
		cip.DrawVertLine(frames(i),0, cip.Height(), CPixelRGB8 (255,255,255));
	}
}

void FltkScrollSelectPanel_impl::drawTextBox(int start, int end, int colormapValue, const char* text)
{
	for(int i=start; i<end; i++)
		cip.DrawVertLine(i, 0, cip.Height(), colormap.GetPixel(colormapIndex[colormapValue],0));
	cip.DrawText(start, 0, text, true, CPixelRGB8 (255,255,255));
}

void FltkScrollSelectPanel_impl::clear(int start, int end)
{
	if(start<0) start=0;
	if(end>cip.Width()) end=cip.Width();
	if(start<end)
		cip.DrawBox(TRect(start, 0, end, cip.Height()), CPixelRGB8 (0,0,0));
}

#else
void saveViewpoint(FILE* file)
{
}
#endif
