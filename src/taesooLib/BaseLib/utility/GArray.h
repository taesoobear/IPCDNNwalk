#ifndef _GARRAY_H_
#define _GARRAY_H_

#if _MSC_VER>1000
#pragma once
#endif

#include "TArray.h"
#include <vector>
#include <map>
#include <list>

class GElt
{
public:
	enum { TE_INTEGER, TE_FLOAT, TE_STRING, TE_NO_DATA };

	GElt(const char* szData)					{ m_eType=TE_NO_DATA; strData(szData);};
	GElt(int nData)								{ m_eType=TE_NO_DATA; intData(nData);};
	GElt(float fData)							{ m_eType=TE_NO_DATA; floatData(fData);};
	GElt()										{ m_eType=TE_NO_DATA;};
	~GElt()										{ clear(); };

	void clear()										{ if(m_eType==TE_STRING) delete[] m_szData; m_eType=TE_NO_DATA;};

	// data query
	int type() const									{ return m_eType;};

	void strData(const char* szData)					{ clear(); m_eType=TE_STRING; m_szData=new char[strlen(szData)+1]; strcpy(m_szData,szData);};
	void intData(int nData)								{ clear(); m_eType=TE_INTEGER; m_nData=nData;};
	void floatData(float fData)							{ clear(); m_eType=TE_FLOAT; m_fData=fData;};

	char* strData() const								{ ASSERT(m_eType==TE_STRING); return m_szData;};
	int intData() const									{ ASSERT(m_eType==TE_INTEGER); return m_nData;};
	float floatData() const								{ ASSERT(m_eType==TE_FLOAT); return m_fData;};

	bool operator==(const GElt& other) const;
	void operator=(const GElt& other);
private:
	int m_eType;
	union
	{
		char * m_szData;
		int m_nData;
		float m_fData;
	};
};

/**
 * 각 원소가 string또는 float또는 interger를 갖는 general array.
 * 나는 일반적으로 name taesoo age 27 .. 하는 식으로 id, value조합으로 많이 사용한다.
 */
class GArray : public TArray<GElt>
{
public:
	GArray(int size);
	virtual ~GArray();

	int Find(const GElt& other);
};
#endif
