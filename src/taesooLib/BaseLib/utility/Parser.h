
#pragma once
 
#include "TypeString.h"
class Parser
{
public:
	/// bTextMode인 경우, text file형태로 저장한다.
	Parser(const char* filename, const char* seperator=" ,\r\t\n", bool bToUpper=false);
	virtual ~Parser();

	// seperator로 구분된 token을 return한다. #으로 시작하는 line은 주석으로 간주한다.
	TString getToken();
	FILE* getFilePointer();
private:
	FILE *m_pFile;
	char buff[4096];
	bool m_bToUpper;	
	TString mSeperator;
	char* token;
};

