// TextFile.cpp: implementation of the CTextFile class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "TextFile.h"
#include <ctype.h>
#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

static bool iswhitespace(TString const& token)
{
	for(int i=0; i<token.length(); i++)
	{
		if(!isspace(token[i]))
			return false;
	}
	return true;
}
CTextFile::CTextFile()
{
#ifdef _MSC_VER
	m_strSeps= " ,\t\n";
#else
	m_strSeps= " ,\t\n\r";
#endif

	m_bToUpper=false;
	Init();
}

TString const& CTextFile::getSeperators() const
{
	return m_strSeps;
}

void CTextFile::setSeperators(const char* seps)
{
	m_strSeps=seps;
}

void CTextFile::Init()
{
	m_pFile=NULL;

	for(int i=0; i<NUM_UNDO; i++)
		m_aStates[i].Init();
	
	m_nCurrState=0;
	m_nEmptyLine=0;
}

CTextFile::~CTextFile()
{
	if(m_pFile) fclose(m_pFile);
}
static bool IsOneOf(char c, const char* seps)
{
	for(int i=0; seps[i]; i++)
		if(c==seps[i]) return true;
	return false;
}

static bool ReadFromFile(FILE*& file, char* buff, const char* seps)
{
	// #으로 시작하는 line은 제거
	bool bEOF;
	if(file==NULL) return true;
	while(1)
	{
		bEOF=(fgets(buff, 4095, file)==NULL);
		if(bEOF)
		{
			fclose(file);
			file=NULL;
			return bEOF;
		}
		if(buff[0] != '#')
		{
			bool bValidLine=false;
			for(int ii=1; buff[ii]; ii++)
			{
				if(buff[ii]=='#')
				{
					buff[ii]=0;
					break;
				}
			}

			for(int ii=0; buff[ii]; ii++)
				if(!IsOneOf(buff[ii], seps))
					bValidLine=true;

			if(bValidLine) return bEOF;
		}
	}
}
FILE *fmemopen(void *buf, size_t size, const char *opentype)
{
	FILE *f;

	assert(strcmp(opentype, "r") == 0);

	f = tmpfile();
	fwrite(buf, 1, size, f);
	rewind(f);

	return f;
}

bool CTextFile::OpenMemory(const char *text)
{

	Init();
	ASSERT(m_pFile==NULL);
	void* buf=(void*)text;
	m_pFile=fmemopen(buf,strlen(text)+1, "rt");
	if(!m_pFile){
	   	throw std::runtime_error("openmemory");
	}
	//if(!m_pFile) return false;
	ReadOneLine();

	return true;
}
bool CTextFile::OpenReadFile(const char *fileName)
{
	Init();
	ASSERT(m_pFile==NULL);
	m_pFile=fopen(fileName,"rt");
	if(!m_pFile){
	   	throw std::runtime_error(fileName);
	}
	//if(!m_pFile) return false;
	ReadOneLine();
	return true;
}

void CTextFile::CloseFile()
{
	if(m_pFile)
		fclose(m_pFile);
	Init();
}


char* copy(int left, int right, char* buff, char* output)
{
	int i;
	for(i=left; i<right; i++)
		output[i-left]=buff[i];
	output[i-left]=0;
	return output;
}
TString const& CTextFile::getSingleCharacterTokens() const
{
	return m_strSingleCharTokens;
}

void CTextFile::setSingleCharacterTokens(const char* sct)
{
	m_strSingleCharTokens=sct;
}
char* CTextFile::Strtok()
{
	int& cur_index=m_aStates[m_nCurrState].nIndex;
	int cur_line=m_aStates[m_nCurrState].nLine;
		
	int i;
	for(i=cur_index; m_aszLine[cur_line][i] && IsOneOf(m_aszLine[cur_line][i], m_strSeps); i++);
	int left=i;
	ASSERT(left<4096);

	int right;

	if(m_strSingleCharTokens.length())
	{
		if(IsOneOf(m_aszLine[cur_line][i],m_strSingleCharTokens))
		{
			right=i+1;
		}
		else
		{
			char c;
			for(; (c=m_aszLine[cur_line][i]) 
				&& !IsOneOf(c, m_strSeps) 
				&& !IsOneOf(c, m_strSingleCharTokens); i++);
			right=i;
		}
	}
	else
	{
		for(; m_aszLine[cur_line][i] && !IsOneOf(m_aszLine[cur_line][i], m_strSeps); i++);
		right=i;
	}

	ASSERT(right<4096);
	cur_index=right;
	if(left==right) return NULL;
	return copy(left, right, m_aszLine[cur_line], m_szOutput);
}

bool CTextFile::ReadOneLine()
{
	m_aStates[m_nCurrState].nLine=(m_aStates[m_nCurrState].nLine+1)%NUM_LINES;
	m_aStates[m_nCurrState].nIndex=0;
	int curline=m_aStates[m_nCurrState].nLine;
	
	if(curline==m_nEmptyLine)
	{
		// CASE 1: first time reading without UNDO
		m_abEOF[curline]=ReadFromFile(m_pFile, m_aszLine[curline], m_strSeps);
		m_nEmptyLine=(m_nEmptyLine+1)%NUM_LINES;
	}
	else 
	{
		// CASE 2: re-reading after UNDO.
	}
	return m_abEOF[curline];
}

void CTextFile::SaveUndoState()
{
	m_aStates[(m_nCurrState+1)%NUM_UNDO].Clone(m_aStates[m_nCurrState]);
	m_nCurrState=(m_nCurrState+1)%NUM_UNDO;	
}

char CTextFile::GetChar()
{
	SaveUndoState();

	int& cur_index=m_aStates[m_nCurrState].nIndex;
	int cur_line=m_aStates[m_nCurrState].nLine;
	

	if(m_aszLine[cur_line][cur_index]=='\n')
	{
		return '\n';
		// need to increase cur_line.. but..
	}
	else
	{
		cur_index++;
		return m_aszLine[cur_line][cur_index-1];
	}

}

char* CTextFile::GetQuotedText(char quotationmark)
{
	TString seps=getSeperators();
	TString backup_sct=m_strSingleCharTokens;
	TString token=GetToken();
	if(token.length()==2 && token[0]==quotationmark && token[1]==quotationmark)
	{
		m_szOutput[0]=0;
		return m_szOutput;
	}
	else
	{
		Undo();
		setSeperators("\"\n");
		m_strSingleCharTokens="\"";
		token=GetToken();
		if(iswhitespace(token))
			token=GetToken();
#ifdef _DEBUG
		printf("getQuotedText2:%s:\n", token.ptr());
#endif
		setSeperators(seps);
		GetToken();
		m_strSingleCharTokens=backup_sct;
		
		strcpy(m_szOutput, token);

		return m_szOutput;
	}
}

char* CTextFile::GetToken(bool& bLineChange)
{
	bLineChange=false;
	SaveUndoState();

	if(m_abEOF[m_aStates[m_nCurrState].nLine]) return NULL;

	char* token;
	token=Strtok();
	
	while(!token)
		if(!ReadOneLine())
		{
			bLineChange=true;
			token = Strtok();
		}
		else 
			return NULL;

	if(m_bToUpper)
		for(int i = 0; i < strlen(token); i++)
			token[i] = toupper(token[i]);
//#define VERVOSE
#ifdef VERVOSE
		printf(":%s:\n", token);
#endif
	return token;
}

char* CTextFile::GetLine()
{
	SaveUndoState();
	int curline=m_aStates[m_nCurrState].nLine;
	if(m_abEOF[curline]) return NULL;
	char* token;	
	int len=strlen(m_aszLine[curline]);
	token=copy(m_aStates[m_nCurrState].nIndex, len, m_aszLine[curline], m_szOutput);
	ReadOneLine();
	return token;
}

void CTextFile::Undo()
{
	m_nCurrState=(m_nCurrState+NUM_UNDO-1)%NUM_UNDO;
}
