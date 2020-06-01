// NameTable.cpp: implementation of the NameTable class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "NameTable.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


using namespace utility;
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

NameTable::NameTable()
{

}

NameTable::~NameTable()
{
	Clear();
}

void NameTable::Insert(const char* name)
{
	char* str=Copy(name);
	ASSERT(m_namedmap.find(str)==m_namedmap.end());
	m_namedmap[str]=m_aNames.size();
	m_aNames.push_back(str);
}

int NameTable::operator[](const char* key)
{
	ASSERT(m_namedmap.find((char*)key)!=m_namedmap.end());
	return m_namedmap[(char*)key];
}

char* NameTable::operator[](int keyvalue)
{
	return m_aNames[keyvalue];
}

void NameTable::Clear()
{
	std::vector<char*>::iterator i;

	for(i=m_aNames.begin(); i!=m_aNames.end(); i++)
	{
		delete[] (*i);
	}

	m_aNames.clear();
	m_namedmap.clear();
}


bool NameTable::find(const char* name)
{
	if(name==NULL) return false;
    return (m_namedmap.find((char*)name)!=m_namedmap.end());
}
