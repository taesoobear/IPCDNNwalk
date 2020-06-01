// ConfigTable.cpp: implementation of the ConfigTable class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "configtable.h"
#include "util.h"
#include "TextFile.h"
#include <string>

#include <stdio.h>
#include <stdlib.h>

//ConfigTable g_ConfigTable("config.txt");
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
ConfigTable::ConfigTable()
{
}

ConfigTable::ConfigTable(const char *szFileName)
{
	Initialize(szFileName);
}

ConfigTable::ConfigTable(const char* testFn, const char *szFileName)
{
	load(testFn, szFileName);
}

void ConfigTable::load(const char* testFn, const char *szFileName)
{
	Release();
	FILE* test=fopen(testFn,"r");
	if (test)
	{
		fclose(test);
		Initialize(testFn);
	}
	else if(szFileName)
		Initialize(szFileName);
}

ConfigTable::~ConfigTable()
{
	Release();
}

void ConfigTable::WriteTable()
{
	FILE *configFile;
	VERIFY(configFile=fopen(m_szFileName,"w"));
	if (configFile==NULL) printf("Error %s not found\n", m_szFileName);
	namedmapLPTCHAR::iterator iList;
	for(iList = m_namedmapContent.begin(); iList != m_namedmapContent.end(); iList++)
		fprintf(configFile, "%s %s\n", iList->first.c_str(), iList->second);

	fclose(configFile);
}

//#include <unistd.h>
void ConfigTable::Initialize(const char *szFileName)
{
	Release();
	CTextFile configFile;
	if(!configFile.OpenReadFile(szFileName))
		Msg::error("Config file %s open error", szFileName);
	strcpy(m_szFileName,szFileName);

	char* token;
	TString value;
	while(token=configFile.GetToken())
	{
		char szKey[100];
		strcpy(szKey,token);
		token=configFile.GetLine();
		value=token;
		value.trimLeft(" \t");
		value.trimRight(" \t\n");
		char* szValue=new char[value.length()+1];
		strcpy(szValue,value);

		ASSERT(m_namedmapContent.find(szKey) == m_namedmapContent.end());
		m_namedmapContent[szKey] = szValue;
	}
	configFile.CloseFile();

/*	FILE *configFile;

	VERIFY(configFile=fopen(szFileName,"r"));
	strcpy(m_szFileName,szFileName);

	char buff[4096];
	char seps[10];
	char *token;
	while(fgets(buff,4096,configFile))
	{
		if(buff[0]=='#') continue;

		char szKey[100];
		strcpy(seps," \t\n");
		token=strtok(buff,seps);
		if(!token) continue;
		strcpy(szKey,token);
		strcpy(seps,"\n");
		token=strtok(NULL,seps);
		char *szValue=(char*)malloc((strlen(token)+1)*sizeof(char));
		strcpy(szValue,token);

		ASSERT(m_namedmapContent.find(szKey) == m_namedmapContent.end());
		m_namedmapContent[szKey] = szValue;
	}

	fclose(configFile);*/
}

void ConfigTable::Insert(std::string& name, int value)
{
	ASSERT(0);
	/*int bucket;
	bucket=HashFunction(name);
	table[bucket][m_nCount[bucket]]=value;
	m_nCount[bucket]++;
	ASSERT(m_nCount[bucket]<CT_NCOUNT);*/
}

void ConfigTable::Release()
{
	namedmapLPTCHAR::iterator iList;
	for(iList = m_namedmapContent.begin(); iList != m_namedmapContent.end(); iList++){
		char * lpTemp = iList->second;
		delete[] lpTemp;
	}
	m_namedmapContent.clear();
}

char* ConfigTable::Find(const char* name)
{
	return Find(std::string(name));
}

char* ConfigTable::Find(std::string const& name)
{
#ifdef ALL_CHAR_TOUPPER_CONFIGKEY
	for(int i = 0; i < name.length(); i++)
		name[i] = toupper(name[i]);
#endif

	namedmapLPTCHAR::iterator i=m_namedmapContent.find(name);
	if(i==m_namedmapContent.end())
		return NULL;


	return i->second;
}

void ConfigTable::WriteInt(char szName[100], int num)
{
	if(m_namedmapContent[szName])
		free(m_namedmapContent[szName]);

	char *buff=(char*)malloc(sizeof(char)*10);

	sprintf(buff, "%d", num);
	ASSERT(strlen(buff) < 512);
	m_namedmapContent.erase(m_namedmapContent.find(szName));
	m_namedmapContent[szName] = buff;
	WriteTable();
}

void ConfigTable::WriteFloat(char szName[100], float num)
{
	if(m_namedmapContent[szName])
		free(m_namedmapContent[szName]);

	char *buff=(char*)malloc(sizeof(char)*10);

	sprintf(buff, "%g", num);

	ASSERT(strlen(buff) < 512);
	m_namedmapContent.erase(m_namedmapContent.find(szName));
	m_namedmapContent[szName] = buff;
	WriteTable();
}

int ConfigTable::GetInt(const char* szName)
{
	char* str=Find(std::string(szName));
	if(!str)
	{
		Msg::error("%s not found in config file", szName);
		return 0;
	}
	return (int)atoi(str);
}

float ConfigTable::GetFloat(const char* szName)
{
	char* str=Find(std::string(szName));
	if(!str)
	{
		Msg::error("%s not found in config file", szName);
		return 0.f;
	}
	return (float)atof(str);
}

void ConfigTable::GetString(char* output, const char* szName)
{
	char* str=Find(std::string(szName));
	if(!str)
	{
		Msg::error("%s not found in config file", szName);
		return;
	}
	strcpy(output,str);
}
