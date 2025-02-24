
#include "stdafx.h"

#if defined NO_GUI && defined NO_OGRE 
#include "../console/traceManager.h"
#else
#include "TraceManager.h"
#endif
#include "renderer.h"

namespace RE
{
	namespace _private
	{
		extern bool g_bOutput;
	}
	std::vector<AbstractTraceManager*> g_traceManagers;
}

extern float mfScaleFactor;

bool RE::_private::g_bOutput=true;

void RE::outputState(bool bOutput)
{
	_private::g_bOutput=bOutput;
}
void RE::output(const char* key, const char* pszFormat, ...)
{
	if(!_private::g_bOutput) return;
	if(g_traceManagers.size()>0)
	{
		AbstractTraceManager* pTraceManager=g_traceManagers.back();
		TString temp;
		va_list argptr ;
		va_start(argptr, pszFormat);
		temp._format(pszFormat, argptr);
		pTraceManager->message(key, temp);
	}
}

void RE_dumpOutput(TStrings& output, int itracemanager)
{
	if(!RE::_private::g_bOutput) return;
	if(RE::g_traceManagers.size()>=itracemanager)
	{
		(*(RE::g_traceManagers.end()-itracemanager))->dumpMessage(output);
	}
}
void RE_outputRaw(const char* key, const char* output, int itracemanager)
{
	if(!RE::_private::g_bOutput) return;
	if(RE::g_traceManagers.size()>=itracemanager)
	{
		(*(RE::g_traceManagers.end()-itracemanager))->message(key, output);
	}
}

void RE_outputEraseAll(int itracemanager)
{
	if(!RE::_private::g_bOutput) return;
	if(RE::g_traceManagers.size()>=itracemanager)
	{
		((TraceBase*) (*(RE::g_traceManagers.end()-itracemanager)))->eraseAll();
	}
}
void TraceBase::erase(const char *id)
{
	namedmapTDM::iterator i;
	if((i=m_aMessage.find(id))!=m_aMessage.end())
	{
		// 존재하는 경우 지워준다.
		delete i->second;
		m_aMessage.erase(i);
	}
}

void TraceBase::dumpMessage(TStrings& out)
{
	namedmapTDM::iterator j;
	int c=0;
	for(j = m_aMessage.begin(); j != m_aMessage.end(); ++j)
		c=c+2;

	out.resize(c);
	c=0;
	for(j = m_aMessage.begin(); j != m_aMessage.end(); ++j)
	{
#ifdef OLD_COMPILERS
		out[c++]=j->first;
#else
		out[c++]=j->first.c_str();
#endif
		out[c++]=j->second->mMessage;
	}
}
void TraceBase::eraseAll()
{
	namedmapTDM::iterator i,prev;
	for(i = m_aMessage.begin(); i != m_aMessage.end();)
	{
		delete i->second;
		prev=i;
		++i;
		m_aMessage.erase(prev);
	}
}

void TraceBase::message(const char *id, const char *content)
{
	if(!content) return;
	erase(id);

	DynamicMessage* pDM=new DynamicMessage(content);
	m_aMessage[id]=pDM;
}

#ifndef NO_GUI


TraceManager::TraceManager(int x, int y, int w, int h)
: Fl_Double_Window(x,y,w,h)
{
	end();
	RE::g_traceManagers.push_back(this);
}

TraceManager::~TraceManager(void)
{
	eraseAll();
//	RE::g_traceManagers.remove(this);
}


void TraceManager::message(const char *id, const char *content)
{
	TraceBase::message(id, content);
	redraw();
}

void TraceManager::draw()
{
	Fl_Double_Window::draw();

	Fl_Color col;
	col=fl_color();
	fl_color(0,0,0);

	namedmapTDM::iterator j;
	int y=20;
	int x=0;
	for(j = m_aMessage.begin(); j != m_aMessage.end(); ++j)
	{
#ifdef OLD_COMPILERS
		fl_draw(j->first.ptr(), x, y);
#else
		fl_draw(j->first.c_str(), x, y);
#endif
		fl_draw(j->second->mMessage.ptr(), x+300, y);
		y+=20;
		if(y>h())
		{
			y=20;
			x=w()/2;
		}
	}
	fl_color(col);
}

#endif
#ifndef NO_OGRE
#include <OgreOverlay.h>
#include <OgreOverlayManager.h>
#include <OgreOverlayContainer.h>
#include <OgreOverlayElement.h>
#include <OgreStringConverter.h>
namespace Ogre
{
	Ogre::v1::OverlayContainer* createContainer(int x, int y, int w, int h, const char* name) {

		v1::OverlayContainer* container = (v1::OverlayContainer*)
			v1::OverlayManager::getSingleton().createOverlayElement(
			"BorderPanel", name);
		container->setMetricsMode(v1::GMM_PIXELS);
		container->setHeight(h);
		container->setWidth(w);
		container->setParameter("border_size", "0 0 0 0");
		container->setParameter("border_material", "Core/StatsBlockBorder");
		container->setParameter("border_topleft_uv", "0.0000 1.0000 0.0039 0.9961");
		container->setParameter("border_top_uv", "0.0039 1.0000 0.9961 0.9961");
		container->setParameter("border_topright_uv", "0.9961 1.0000 1.0000 0.9961");
		container->setParameter("border_left_uv","0.0000 0.9961 0.0039 0.0039");
		container->setParameter("border_right_uv","0.9961 0.9961 1.0000 0.0039");
		container->setParameter("border_bottomleft_uv","0.0000 0.0039 0.0039 0.0000");
		container->setParameter("border_bottom_uv","0.0039 0.0039 0.9961 0.0000");
		container->setParameter("border_bottomright_uv","0.9961 0.0039 1.0000 0.0000");
		container->setLeft(x);
		container->setTop(y);

		return container;

	}

	Ogre::v1::OverlayElement* createTextArea(const String& name, Real width, Real height, Real top, Real left,
		uint fontSize, const String& caption, bool show) {


		v1::OverlayElement* textArea =
				v1::OverlayManager::getSingleton().createOverlayElement("TextArea", name);
			textArea->setMetricsMode(v1::GMM_PIXELS);
			textArea->setWidth(width);
			textArea->setHeight(height);
			textArea->setTop(top);
			textArea->setLeft(left);
			//textArea->setParameter("font_name", "TrebuchetMSBold");
			textArea->setParameter("font_name", "BlueHighway");
			//textArea->setParameter("font_name", "SdkTrays/Value");
			textArea->setParameter("char_height", StringConverter::toString(fontSize));
			textArea->setCaption(caption);
			textArea->setParameter("colour_top", "0 0.5 0");
			textArea->setParameter("colour_bottom", "0 0.2 0");

			if (show) {
				textArea->show();
			}
			else {
				textArea->hide();
			}

			return textArea;

	}



}

OgreTraceManager::OgreTraceManager(int x, int y, int w, int h)
	:FrameMoveObject()
{
	RE::g_traceManagers.push_back(this);
	RE::renderer().addAfterFrameMoveObject(this);
	tid="  ";
}
int OgreTraceManager::FrameMove(float fElapsedTime)
{
	namedmapTDM::iterator j;
	tid="  ";
	TString nd("namedDraw_");
	for(j = m_aMessage.begin(); j != m_aMessage.end(); ++j)
	{
		TString temp=j->first.c_str();
		if (temp.length()>10 && temp.subString(0,10)==nd)
		{
			// too many information to be on screen.
		}
		else
		{
			temp.replace('\n', ' ');
			tid+=temp;
			tid+="\t";
			temp=j->second->mMessage;
			temp.replace('\n', ' ');
			temp.replace('\t', ' ');
			tid+=temp+"\n";
		}
	}
	return 0;
}


#endif
