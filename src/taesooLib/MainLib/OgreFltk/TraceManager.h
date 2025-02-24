
#ifndef TRACE_MANAGER_H
#define TRACE_MANAGER_H
#pragma once


#include "../../BaseLib/utility/namedmapsupport.h"
#if (defined(_MSC_VER)&& _MSC_VER<1700) || (!defined(_MSC_VER) &&__cplusplus < 201103L)
#define OLD_COMPILERS
#endif

#ifdef OLD_COMPILERS
#ifdef _WIN32
// VS2010
#include <hash_map>
class T_stringhasher : public stdext::hash_compare <TString>
{
public:
  size_t operator() (TString const &in) const { 
    size_t h = 0; const char* p;
    for(p = in.ptr(); *p != 0; ++p)
		h = 31 * h + (*p);
    return h;
  }
  bool operator() (TString const& s1,TString const& s2) const { return strcmp(s1.ptr(), s2.ptr())<0; }
};
#else
#include <ext/hash_map>
// allow the gnu hash_map to work on TString
namespace __gnu_cxx {
	template<> struct hash< TString > {
		size_t operator()(const TString& s) const {
			return hash< const char* >()( s.ptr() );
		}
	}; /* gcc.gnu.org/ml/libstdc++/2002-04/msg00107.html */
}

#endif
#else
#include <unordered_map>
#endif
#include "RE.h"
class TraceBase : public AbstractTraceManager
{
public:

	TraceBase(){}
	virtual ~TraceBase(){}
	class DynamicMessage
	{
	public:
		DynamicMessage(const char* content):mMessage(content){}
		~DynamicMessage(){}
		TString mMessage;
	};
#ifdef OLD_COMPILERS
#ifdef _WIN32
	//typedef std::map<TString, DynamicMessage*, cmpTString> namedmapTDM ;
	typedef stdext::hash_map<TString, DynamicMessage*, T_stringhasher> namedmapTDM ;
#else
	typedef	__gnu_cxx::hash_map<TString, DynamicMessage*> namedmapTDM;
#endif
#else
	typedef std::unordered_map<std::string, DynamicMessage*> namedmapTDM;
#endif

	void erase(const char *id);
	virtual void eraseAll();
	virtual void dumpMessage(TStrings& out);

	virtual void message(const char *id, const char *content);

	namedmapTDM m_aMessage;
};

#ifndef NO_GUI


class TraceManager : public Fl_Double_Window, public TraceBase
{
public:
	
	TraceManager(int x, int y, int w, int h);
	virtual ~TraceManager(void);

	virtual void draw();

	virtual void message(const char *id, const char *content);

	
};

#endif
#ifndef NO_OGRE


#include "framemoveobject.h"
class OgreTraceManager : public TraceBase, public FrameMoveObject
{
public:
	
	TString tid, tc;
	OgreTraceManager(int x, int y, int w, int h);
	virtual ~OgreTraceManager(void){}

	virtual void draw(){}

	void showOutputs(){}
	void hideOutputs(){}

    void setVisible( int iElement, bool visible){}                      // 296
	
	virtual int FrameMove(float fElapsedTime);
};
#endif
#endif
