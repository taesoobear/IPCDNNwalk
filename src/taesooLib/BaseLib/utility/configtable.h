// configtable.h: interface for the ConfigTable class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CONFIGTABLE_H__BDBC6D61_7A07_11D5_9D95_0050BF107BFC__INCLUDED_)
#define AFX_CONFIGTABLE_H__BDBC6D61_7A07_11D5_9D95_0050BF107BFC__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define CT_MAX_ITEM 100
#define CT_NBUCKET 100
#define CT_NCOUNT 30
#define MAX_ITEM 100

#include "namedmapsupport.h"

//#include <tchar.h>
typedef std::map<std::string , char *, ltstr> namedmapLPTCHAR;

//! macro function .. usage : CTReadInt(m_ConfigTable, m_bFullScreen); 의미: m_bFullScreen을 config file에서 읽어와서 setting한다.
#define CTReadInt(cname,vname)			vname=cname.GetInt(#vname)
#define CTReadFloat(cname,vname)		vname=cname.GetFloat(#vname)
#define CTReadString(cname,vname)		cname.GetString(vname,#vname)

//! 파일에서 설정을 저장하거나 읽어오는 클래스 Key와 Content의 두 string으로 관리된다.
/*!
a.txt
_________________________
key1 13
key2 15
_________________________
라 하면,
GetInt(string("key1"))==13 이 성립
Find("key1") 하면 "13"이 return 된다.

*/
class ConfigTable
{
public:
	ConfigTable();
	ConfigTable(const char *szFileName);
	ConfigTable(const char *szFileName1, const char* szFileName2 );
	virtual ~ConfigTable();

	void load(const char* szFileName1, const char* szFileName2);

	char *Find(std::string const&name);
	char *Find(const char * name);

	int GetInt(const char* szName);
	float GetFloat(const char* szName);
	void GetString(char* output, const char* szName);

	//! key에 해당하는 값의 내용을 바꾼후, 파일에 저장한다.
	void WriteInt(char szName[100], int num);
	void WriteFloat(char szName[100], float num);

	void WriteTable();
	void Initialize(const char *szFileName);
private:
	void Release();
	void Insert(std::string &key, int value);

	namedmapLPTCHAR m_namedmapContent;

	char m_szFileName[100];
};

#endif // !defined(AFX_CONFIGTABLE_H__BDBC6D61_7A07_11D5_9D95_0050BF107BFC__INCLUDED_)
