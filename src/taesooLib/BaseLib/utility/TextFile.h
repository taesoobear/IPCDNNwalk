// TextFile.h: interface for the CTextFile class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_TEXTFILE_H__1F48B92D_AA9E_4B20_8F24_4F72D9B0C87E__INCLUDED_)
#define AFX_TEXTFILE_H__1F48B92D_AA9E_4B20_8F24_4F72D9B0C87E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// 항상 NUM_UNDO번 이내의 undo가 가능한 클래스. 즉 GetToken()=="ROOT"였다면, 이를 undo 후 다시 GetToken하면 다시 "ROOT"가 나온다. GetLine()후 undo후 GetToken()도 가능하다.
#include "TypeString.h"

class CTextFile  
{
public:
	CTextFile();
	virtual ~CTextFile();
	bool OpenReadFile(const char *fileName);
	void CloseFile();

	char* GetToken(bool& bLineChanged);
	char* GetToken()						{ bool bLineChanged; return GetToken(bLineChanged);};
	char* GetLine();
	char* GetQuotedText(char quotationmark='\"');
	char GetChar();
	void Undo();
	TString const& getSeperators() const;
	void setSeperators(const char* seps);
	// { } 등 한글자단위로 철저하게 분리하고 싶은 기호가 있으면 아래함수를 호출할 것. e.g. sct="{}[]"
	void setSingleCharacterTokens(const char* sct);
	TString const& getSingleCharacterTokens() const;
private:
	enum { NUM_UNDO=3, NUM_LINES};
	void Init();
	char* Strtok();
	bool ReadOneLine();
	void SaveUndoState();
	FILE *m_pFile;
	TString m_strSeps;
	TString m_strSingleCharTokens;
	
	char m_szOutput[4096];
	
	char* m_pToken;
	bool m_bToUpper;
	
	class State
	{
	public:
		void Init()	{nIndex=0; nLine=-1;}
		void Clone(const State& other)	{ nIndex=other.nIndex; nLine=other.nLine;};
		int nIndex;
		int nLine;
	};

	int m_nCurrState;
	State m_aStates[NUM_UNDO];

	int m_nEmptyLine;
	char m_aszLine[NUM_LINES][4096];
	bool m_abEOF[NUM_LINES];
};

#endif // !defined(AFX_TEXTFILE_H__1F48B92D_AA9E_4B20_8F24_4F72D9B0C87E__INCLUDED_)
