// GArray.cpp: implementation of the GArray class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
//#include "stdtemplate.h"
#include "GArray.h"
#include <list>

bool GElt::operator==(const GElt& other) const
{
	if(m_eType!=other.m_eType) return false;

	switch(m_eType)
	{
	case TE_INTEGER:
		return m_nData==other.m_nData;
	case TE_FLOAT:
		return m_fData==other.m_fData;
	case TE_STRING:
		return strcmp(m_szData, other.m_szData)==0;
	}
	return false;
}

void GElt::operator=(const GElt& other)
{
	clear();
	m_eType=other.type();
	switch(other.type())
	{
	case TE_INTEGER:
		m_nData=other.m_nData;
	case TE_FLOAT:
		m_fData=other.m_fData;
	case TE_STRING:
		m_szData=new char[strlen(other.strData())+1]; strcpy(m_szData,other.strData());
	}
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GArray::GArray(int size)
:TArray<GElt>(size)
{
}

GArray::~GArray()
{
}


int GArray::Find(const GElt& other)
{
	for(int i=0; i<size(); i++)
		if(data(i)==other) return i;
	return -1;
}
