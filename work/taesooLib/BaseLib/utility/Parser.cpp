//
//////////////////////////////////////////////////////////////////////
 
#include "stdafx.h"
#include "stdtemplate.h"
#include "Parser.h"

Parser::Parser(const char* filename, const char* seperator, bool bToUpper)
:mSeperator(seperator),
m_bToUpper(bToUpper),
token (NULL)
{
	if(filename)
		m_pFile=fopen(filename,"rt");
	else
		m_pFile=NULL;
}

#ifdef _MSC_VER
FILE *fmemopen(void *buf, size_t size, const char *opentype);
#endif
void Parser::openFile(const char* filename)
{
	if(m_pFile) fclose(m_pFile);
	m_pFile=fopen(filename,"rt");
}
void Parser::openFromMemory(const char* text)
{
	m_pFile=fmemopen((void*)(text),strlen(text)+1, "rt");
	if(!m_pFile){
	   	throw std::runtime_error("openmemory");
	}
}
Parser::~Parser()
{
	if(m_pFile)
		fclose(m_pFile);
}

TString Parser::getToken()
{
	TString strToken;
	if(token){
		token = strtok(NULL, mSeperator.ptr());
	}

	if(m_pFile==NULL)
		return strToken;

	while(!token)
	{
		if(fgets(buff, 4096, m_pFile)){
			if(buff[0] != '#')
				token = strtok(buff, mSeperator.ptr());
		}
		else {
			Msg::error("Parser???");
			fclose(m_pFile);
			m_pFile=NULL;
			return strToken;
		}
	}

	strToken=token;

	if(m_bToUpper)
		strToken.makeUpper();

	return strToken;
}

FILE* Parser::getFilePointer()
{
	return this->m_pFile;
}
