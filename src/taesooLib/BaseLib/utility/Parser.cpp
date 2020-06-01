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
	m_pFile=fopen(filename,"rt");
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
