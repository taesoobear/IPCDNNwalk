// TypeString.cpp: implementation of the TypeString class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "util.h"
#include "TypeString.h"
#include <stdarg.h>
#include <ctype.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TString::TString(const char* str, int i)
{
	initData();
	format("%s%d",str,i);
}

TString::TString()
{
	initData();
}

void TString::updateLength()						{ m_nDataLen=(int)strlen(m_psData); }
void TString::makeUpper()
{
	for(int i=0; i<length(); i++)
		value(i)=toupper(value(i));
}

TString TString::toUpper()
{
	TString temp=*this;
	temp.makeUpper();
	return temp;
}

void TString::initData()
{
	m_nDataLen = 0;
	m_nAllocLen = 0;

	m_psData = NULL;
}

void TString::freeData()
{
	if(m_psData != NULL){
		free(m_psData);
		initData();
	}

}

int TString::isEmpty() const
{
	return (m_nDataLen == 0);
}

void TString::empty()
{
	m_nDataLen = 0;
	if(m_nAllocLen)
		m_psData[0] = '\0';
}

TString::TString(const sz0::Operator& op)
{
	initData();
	op(*this);
}

TString::TString(const char* strString)
{
	initData();
	alloc( strString);
}

TString::TString(const TString &string)
{
	initData();
	alloc(string);
}

TString::~TString()
{
	freeData();
}

void TString::alloc(const TString& Src, int start, int end)
{
	if(start<0)	start=0;
	if(end > Src.length())	end = Src.length();

	m_nDataLen = end - start ;

	if(m_nDataLen<=0)
		empty();
	else{
		reserve(m_nDataLen + 1);

		memcpy(m_psData,Src.ptr(start), m_nDataLen*sizeof(char));
		m_psData[m_nDataLen] = '\0';
	}
	return;
}

void TString::alloc(const char* strData, int nLen)
{
	if(nLen == -1)
		nLen = (strData==NULL)? 0 : strlen(strData);
	m_nDataLen = nLen;


	if(m_nDataLen!=0) {
		reserve(m_nDataLen + 1);
		memcpy(m_psData,strData,m_nDataLen*sizeof(char));
	}

	if(m_nAllocLen!=0){
		m_psData[m_nDataLen] = '\0';
	}
}

void TString::replace(char a, char b)
{
	for(int i=0; i<length(); i++)
		if(value(i)==a) value(i)=b;
}


void TString::replace(const char* a, const char* b)
{
	TString aa(a);

	TString output;

//	ASSERT(aa.length()==bb.length());
	for(int i=0; i<length()-aa.length(); i++)
	{
		if(subString(i,i+aa.length())==aa)
		{
			output.add("%s", b);
			i+=aa.length()-1;
		}
		else
			output.add("%c", (*this)[i]);
	}
	output.add("%s",this->right(aa.length()).ptr());
	*this=output;
}
/*
void TString::replace(const char* a, const char* b)
{
	TString aa(a);
	TString bb(b);

	ASSERT(aa.length()==bb.length());
	for(int i=0; i<length()-aa.length(); i++)
	{
		if(subString(i,i+aa.length())==aa)
		{
			for(int j=0; j<bb.length(); j++)
			{
				(*this)[i+j]=bb[j];
			}

			i+=aa.length()-1;
		}
	}
}
*/

int TString::strcmp(const TString &str2) const
{
	if(length()==0) return str2.length()!=0;
	return ::strcmp(m_psData,str2.ptr());
}

int TString::strcmp(const char* str2) const
{
	if(length()==0) return str2[0]!=0;
	return ::strcmp(m_psData,str2);
}


int TString::strncmp(const TString &str2, int n) const
{
	if(n==-1)
		return ::strcmp( m_psData, str2.ptr() );
	else
		return ::strncmp( m_psData, str2.ptr(),n);

}

void TString::concat(const TString & str2, int n)
{
	concat( str2.ptr(), n );
}

void TString::concat(const char* str2, int n )
{
	if(str2==NULL)
		return;
	int appLen = strlen(str2) - n;
	int concatLen = m_nDataLen + appLen;

	if(appLen<0) return;

	reserve(concatLen+1);

	memcpy(m_psData+m_nDataLen, str2+n, appLen);
	m_nDataLen = concatLen;
	m_psData[m_nDataLen] = '\0';
}


TString& TString::add(const char* pszFormat, ...)
{
	va_list argptr ;
	va_start(argptr, pszFormat);
	TString temp;
	temp._format(pszFormat, argptr);
	concat(temp);
	return *this;
}

//%s,%d,%i,%x, and  %c, %s ...
// %3s등의 format지원해야함..
TString& TString::format(const char* pszFormat, ...)
{
	va_list argList;
	va_start(argList,pszFormat);
	_format(pszFormat, argList);
	return *this;
}

void TString::reserve(int AllocLen)
{
	if(m_nAllocLen < AllocLen){
		m_nAllocLen = AllocLen;
		//m_psData = (LPSTR) realloc((void *)m_psData, m_nAllocLen*sizeof(char) );


		char* prevData=m_psData;
		m_psData=(char*) malloc(m_nAllocLen*sizeof(char));

		if(prevData)
		{
		    #ifdef _MSC_VER
			strncpy_s(m_psData, m_nAllocLen, prevData, m_nDataLen);
			#else
			strncpy(m_psData, prevData, m_nDataLen);
			#endif
			free(prevData);
		}
	}
}

void TString::_format(const char* pszFormat, va_list& argList)
{
    #ifdef _MSC_VER
	int nMaxLen =_vscprintf(pszFormat, argList);
	reserve(nMaxLen+1);
	vsprintf(m_psData, pszFormat, argList);
	m_nDataLen = strlen(m_psData);
	#else
	char temp[500];
	vsnprintf(temp, 499, pszFormat, argList);
	int nMaxLen=strlen(temp);
	reserve(nMaxLen+1);
	strcpy(m_psData, temp);
	m_nDataLen=nMaxLen;
	#endif
}

//find
int TString::findChar(int start, char one) const
{
	for(int i=start; i<m_nDataLen; i++){
		if(m_psData[i]==one)
			return i;
	}
	return -1;
}

int TString::findCharRight(char one, int start) const
{
	if(start>=m_nDataLen) start=m_nDataLen-1;
	for(int i=start; i>=0; i--){
		if(m_psData[i]==one)
			return i;
	}
	return -1;
}

int TString::findStr(int start, const char* src, int Len) const
{
	int srcLen = (src==NULL)? 0: (Len==-1)? strlen(src): Len;
	if(srcLen == 0)
		return -1;
	for(int i=start; i<m_nDataLen-srcLen+1; i++){
		if(!::strncmp(m_psData+i,src,srcLen))
			return i;
	}
	return -1;
}

int TString::find(const char* src) const
{
	return findStr(0, src);
}
//substring
TString TString::subString( int start, int end) const
{
	TString Result;

	Result.alloc(*this,start,end);
	return Result;
}

void TString::trimLeft(const char* delimiter)
{
	int nchar=strlen(delimiter);

	int i;
	for(i=0; i<length(); i++)
	{
		int j;
		for(j=0; j<nchar; j++)
			if(value(i)==delimiter[j])	break;
		if(j==nchar) break;
	}

	if(i!=0)
	{
		TString substring;
		substring=subString(i);
		alloc(substring);
	}
}

void TString::trimRight(const char* delimiter)
{
	int nchar=strlen(delimiter);

	int i;
	for(i=length()-1; i>=0; i--)
	{
		int j;
		for(j=0; j<nchar; j++)
			if(value(i)==delimiter[j]) break;

		if(j==nchar) break;
	}

	if(i!=length()-1)
	{
		TString substring;
		substring=subString(0, i+1);
		alloc(substring);
		return;
	}

}

bool TString::isOneOf(int i, const TString& src) const
{
	if(i>=length()) return false;
	int srcLen = src.length();
	for(int j=0; j<srcLen; j++)
		if(src[j]==m_psData[i])
		{
			return true;
		}
	return false;
}


// loop를 돌면서 start가 end가 되면 멈출것.
void TString::token(int &start, const TString& delimiter, TString& token) const
{

	int i ;
	for(i=start; i<length(); i++)
	{
		if(	isOneOf(i, delimiter))
		{
			token=subString(start, i);
			while(isOneOf(i,delimiter)) i++;
			start=i;
			return;
		}
	}
	token=subString(start, i);
	start=i;
}

//Extract
TString TString::token( char src, int index) const
{
	TString Result;
	int startpos=-1;
	int endpos=-1;
	int currin=0;

	for(int i = 0; i<m_nDataLen; i++){
		if(m_psData[i]==src){
			currin ++;
			if(currin==index-1)
				startpos = i;
			else if(currin==index){
				endpos = i;
				break;
			}
		}
	}
	if(startpos!=-1 || index == 1)
		Result.alloc(*this,startpos+1,endpos);
	return Result;
}

TString TString::token(const char* src, int index) const
{
	TString Result;
	int startpos=-1;
	int endpos=-1;
	int currin=0;
	int i,j;
	int srcLen = strlen(src);

	for(i = 0; i<m_nDataLen; i++){
		for(j = 0; j<srcLen; j++)
		{
			if(m_psData[i]==src[j]){
				currin ++;
				if(currin==index-1)
					startpos = i;
				else if(currin==index){
					endpos = i;
					break;
				}
			}
		}
	}

	if(startpos!=-1 || index == 1)
		Result.alloc(*this,startpos+1,endpos);
	return Result;
}

//OPERATOR "="
const TString& TString::operator =(const TString& Str )
{
	alloc(Str);
	return *this;
}

const TString& TString::operator =( const char* mbstrStr)
{
	alloc(mbstrStr);
	return *this;
}

//OPerator "+="
const TString& TString::operator +=( const TString& Str)
{
	concat( Str );
	return *this;
}

const TString& TString::operator +=( const char* mbstrStr)
{
	concat( mbstrStr );
	return *this;
}


TString TString::operator +(int a) const
{
	const TString &Str1=*this;
	TString Result;
	Result.format("%s%d", Str1.ptr(), a);
	return Result;
}
// OPerator "+"
TString  TString::operator +( const TString &Str2) const
{
	const TString &Str1=*this;
	TString Result;
	Result = Str1;
	Result += Str2;
	return Result;
}

TString  TString::operator +( const char* Str2) const
{
	const TString &Str1=*this;
	TString Result;
	Result = Str1;
	Result += Str2;
	return Result;
}

TString  operator +(const char*  Str1, const TString &Str2)
{
	TString Result;
	Result = Str1;
	Result += Str2;
	return Result;
}

//operator "=="
bool  TString::operator ==( const TString &Str2) const
{
	if(length()!=Str2.length()) return false;
	const TString &Str1=*this;
	return !(Str1.strcmp(Str2));
}

bool  TString::operator ==( const char* Str2) const
{
	const TString &Str1=*this;
	if(Str1.length()==0) return (Str2==NULL || Str2[0]==0);
	if(Str2==NULL) return false;
	return !(::strcmp(Str1.ptr(), Str2));
}
bool  operator ==(const char* Str1, const TString &Str2)
{
	return !(Str2.strcmp(Str1));
}

//operator "!="
bool  TString::operator !=(const TString &Str2) const
{
	const TString &Str1=*this;

	if(Str1.strcmp(Str2))
		return true;
	else
		return false;
}

bool  TString::operator !=(const char* Str2) const
{
	const TString &Str1=*this;

	if(Str1.strcmp(Str2))
		return true;
	else
		return false;
}

bool  operator !=(const char* Str1, const TString &Str2)
{
	if(Str2.strcmp(Str1))
		return true;
	else
		return false;
}


TString TStrings::prefix() const	// 모든 문자열이 같은 prefix로 시작하는 경우 해당 prefix return.
{
	TStrings const& other=*this;
	// 모든 문자열이 같은 prefix로 시작하는 경우 잘라준다.
	int prefixEnd=0;
	for(; prefixEnd< other[0].length(); prefixEnd++)
	{
		char c=other[0][prefixEnd];

		int i;
		for(i=1; i<other.size(); i++)
		{
			if(other[i].length()<prefixEnd+1 || c!=other[i][prefixEnd])
				break;
		}
		if (i!=other.size()) break;
	}
	return other[0].subString(0, prefixEnd);
}
void TStrings::trimSamePrefix(const TStrings& other)
{
	// 모든 문자열이 같은 prefix로 시작하는 경우 잘라준다.
	int prefixEnd=0;
	for(; prefixEnd< other[0].length(); prefixEnd++)
	{
		char c=other[0][prefixEnd];

		int i;
		for(i=1; i<other.size(); i++)
		{
			if(other[i].length()<prefixEnd+1 || c!=other[i][prefixEnd])
				break;
		}
		if (i!=other.size()) break;
	}

	init(other.size());

	for(int i=0; i<size(); i++)
        data(i)=other[i].subString(prefixEnd);
}

void TStrings::init(int n)
{
	mStrings.resize(n);
}

void TStrings::setStrings( int n, const char* s1, ... )
{
	va_list marker;
	va_start( marker, s1);     /* Initialize variable arguments. */

	init(n);
	data(0)=s1;
	for(int i=1; i<n; i++)
	{
		data(i)=va_arg( marker, const char*);
	}
	va_end( marker );              /* Reset variable arguments.      */
}

int TStrings::find(const char* other) const
{
	int i;
	for(i=0; i<size(); i++)
		if(data(i)==other)
			break;
	return i;
}
/*
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TypeString::TypeString()
{
	InitData();

}

void TypeString::InitData()
{
	m_nDataLen = 0;
	m_nAllocLen = 0;

	m_psData = NULL;
}

void TypeString::FreeData()
{
	if(m_psData != NULL){
		free(m_psData);
		m_psData = NULL;
	}

}

int TypeString::IsEmpty() const
{
	return (m_nDataLen == 0);
}

void TypeString::Empty()
{
	m_nDataLen = 0;
	if(m_nAllocLen)
		m_psData[0] = '\0';
}

TypeString::TypeString(LPCSTR mbstrString)
{
	InitData();
	AllocMBSTR( mbstrString);
}


TypeString::TypeString(LPCWSTR wstrString)
{
	InitData();
	AllocMBSTR( wstrString );
}

TypeString::TypeString(const TypeString &string)
{
	InitData();
	AllocMBSTR(string);
}


TypeString::~TypeString()
{
	FreeData();
}

void TypeString::AllocMBSTR(const TypeString& Src, int start, int end)
{
	if(end == -1)
		end = Src.GetDataLen();

	m_nDataLen = end - (start + 1);

	if(m_nDataLen<=0)
		Empty();
	else{
		Allocate(m_nDataLen + 1);

		memcpy(m_psData,Src.GetpsData(start+1), m_nDataLen*sizeof(CHAR));
		m_psData[m_nDataLen] = '\0';
	}
	return;
}

void TypeString::AllocMBSTR(LPCSTR strData, int nLen)
{
	if(nLen == -1)
		nLen = (strData==NULL)? 0 : strlen(strData);
	m_nDataLen = nLen;


	if(m_nDataLen!=0) {
		Allocate(m_nDataLen + 1);
		memcpy(m_psData,strData,m_nDataLen*sizeof(CHAR));
	}

	if(m_nAllocLen!=0){
		m_psData[m_nDataLen] = '\0';
	}
}

void TypeString::AllocMBSTR(LPCWSTR wstrData, int nLen)
{
	if(wstrData == NULL)
		return;

	if(nLen == -1)
		m_nDataLen = wcslen(wstrData)*2;

	m_psData = AllocWideToMultiByte(wstrData, m_nDataLen);

	m_nAllocLen = m_nDataLen+1;
	m_nDataLen = strlen(m_psData);
}


BSTR TypeString::SysAllocString()
{
	return( SysAllocString(m_psData, m_nDataLen) );

}


//static
BSTR TypeString::SysAllocString(LPCSTR strData, int nLen)
{

	if(nLen == -1)
		 nLen = strlen(strData);

	int nWLen = MultiByteToWideChar(CP_ACP, 0, strData,	nLen, NULL, NULL);
	BSTR bstr = ::SysAllocStringLen(NULL, nWLen);
	if (bstr == NULL)
		return L"SysAllocError";

	MultiByteToWideChar(CP_ACP, 0, strData, nLen,	bstr, nWLen);

	return bstr;
}


//static
LPSTR TypeString::AllocWideToMultiByte(LPCWSTR wstrData, int nLen)
{
	char *Data;
	if(nLen == -1)
		nLen = wcslen(wstrData)*2;
	Data =  (LPSTR)calloc( (nLen+1), sizeof(char) );

	WideCharToMultiByte(CP_ACP,0,wstrData,-1,Data,(nLen+1),NULL,NULL);

	return Data;
}

//static
LPWSTR TypeString::AllocMultiByteToWide(LPCSTR strCon, int nLen)
{
	wchar_t *wData;
	if(nLen == -1)
		nLen = strlen(strCon);
	wData = (LPWSTR)calloc( (nLen+1), sizeof(wchar_t) );

	MultiByteToWideChar(CP_ACP, 0, strCon,-1, wData, (nLen+1) );

	return wData;
}



int TypeString::RightLength(int pos) const
{
	return (m_nDataLen - pos + 1);
}


int TypeString::StringCmp(const TypeString &str2) const
{
	return lstrcmp(m_psData,str2.GetpsData());
}


int TypeString::StringNCmp(const TypeString &str2, int n) const
{
	if(n==-1)
		return strcmp( m_psData, str2.GetpsData() );
	else
		return strncmp( m_psData, str2.GetpsData(),n);

}

void TypeString::ConCatNMBSTR(const TypeString & str2, int n)
{
	ConCatNMBSTR( str2.GetpsData(), n );
}

void TypeString::ConCatNMBSTR(LPCSTR str2, int n )
{
	int appLen = strlen(str2) - n;
	int concatLen = m_nDataLen + appLen;

	if(appLen<0) return;

	Allocate(concatLen+1);

	memcpy(m_psData+m_nDataLen, str2+n, appLen);
	m_nDataLen = concatLen;
	m_psData[m_nDataLen] = '\0';
}


//%s,%d,%i,%x, and  %c, %s ...
// %3s등의 format지원해야함..
void TypeString::FormatTS(LPCSTR pszFormat, ...)
{

#define FORCE_ANSI      0x10000
#define FORCE_UNICODE   0x20000
#define FORCE_INT64     0x40000
#define TCHAR_ARG   TCHAR
#define WCHAR_ARG   WCHAR
#define CHAR_ARG    char
#define DOUBLE_ARG  double

//	ASSERT(AfxIsValidString(lpszFormat));

	va_list argList;
	va_start(argList,pszFormat);
	va_list argListSave = argList;

	// make a guess at the maximum length of the resulting string
	int nMaxLen = 0;

	for (LPCTSTR lpsz = pszFormat; *lpsz != '\0'; lpsz = _tcsinc(lpsz))
	{
		// handle '%' character, but watch out for '%%'
		if (*lpsz != '%' || *(lpsz = _tcsinc(lpsz)) == '%')
		{
			nMaxLen += _tclen(lpsz);
			continue;
		}

		int nItemLen = 0;

		// handle '%' character with format
		int nWidth = 0;
		for (; *lpsz != '\0'; lpsz = _tcsinc(lpsz))
		{
			// check for valid flags
			if (*lpsz == '#')
				nMaxLen += 2;   // for '0x'
			else if (*lpsz == '*')
				nWidth = va_arg(argList, int);
			else if (*lpsz == '-' || *lpsz == '+' || *lpsz == '0' ||
				*lpsz == ' ')
				;
			else // hit non-flag character
				break;
		}
		// get width and skip it
		if (nWidth == 0)
		{
			// width indicated by
			nWidth = _ttoi(lpsz);
			for (; *lpsz != '\0' && _istdigit(*lpsz); lpsz = _tcsinc(lpsz))
				;
		}

//		ASSERT(nWidth >= 0);

		int nPrecision = 0;
		if (*lpsz == '.')
		{
			// skip past '.' separator (width.precision)
			lpsz = _tcsinc(lpsz);

			// get precision and skip it
			if (*lpsz == '*')
			{
				nPrecision = va_arg(argList, int);
				lpsz = _tcsinc(lpsz);
			}
			else
			{
				nPrecision = _ttoi(lpsz);
				for (; *lpsz != '\0' && _istdigit(*lpsz); lpsz = _tcsinc(lpsz))
					;
			}
//			ASSERT(nPrecision >= 0);
		}


		// should be on type modifier or specifier
		int nModifier = 0;
		if (_tcsncmp(lpsz, _T("I64"), 3) == 0)
		{
			lpsz += 3;
			nModifier = FORCE_INT64;
#if !defined(_X86_) && !defined(_ALPHA_)
			// __int64 is only available on X86 and ALPHA platforms
			ASSERT(FALSE);
#endif

		}
		else
		{
			switch (*lpsz)
			{
			// modifiers that affect size
			case 'h':
				nModifier = FORCE_ANSI;
				lpsz = _tcsinc(lpsz);
				break;
			case 'l':
				nModifier = FORCE_UNICODE;
				lpsz = _tcsinc(lpsz);
				break;

			// modifiers that do not affect size
			case 'F':
			case 'N':
			case 'L':
				lpsz = _tcsinc(lpsz);
				break;
			}
		}



		// now should be on specifier
		switch (*lpsz | nModifier)
		{
		// single characters
		case 'c':
		case 'C':
			nItemLen = 2;
			va_arg(argList, TCHAR_ARG);
			break;
		case 'c'|FORCE_ANSI:
		case 'C'|FORCE_ANSI:
			nItemLen = 2;
			va_arg(argList, CHAR_ARG);
			break;
		case 'c'|FORCE_UNICODE:
		case 'C'|FORCE_UNICODE:
			nItemLen = 2;
			va_arg(argList, WCHAR_ARG);
			break;

		// strings
		case 's':
			{
				LPCTSTR pstrNextArg = va_arg(argList, LPCTSTR);
				if (pstrNextArg == NULL)
				   nItemLen = 6;  // "(null)"
				else
				{
				   nItemLen = strlen(pstrNextArg);
				   nItemLen = max(1, nItemLen);
				}
			}
			break;

		case 'S':
			{
#ifndef _UNICODE
				LPWSTR pstrNextArg = va_arg(argList, LPWSTR);
				if (pstrNextArg == NULL)
				   nItemLen = 6;  // "(null)"
				else
				{
				   nItemLen = wcslen(pstrNextArg);
				   nItemLen = max(1, nItemLen);
				}
#else
				LPCSTR pstrNextArg = va_arg(argList, LPCSTR);
				if (pstrNextArg == NULL)
				   nItemLen = 6; // "(null)"
				else
				{
				   nItemLen = strlenA(pstrNextArg);
				   nItemLen = max(1, nItemLen);
				}
#endif
			}
			break;

		case 's'|FORCE_ANSI:
		case 'S'|FORCE_ANSI:
			{
				LPCSTR pstrNextArg = va_arg(argList, LPCSTR);
				if (pstrNextArg == NULL)
				   nItemLen = 6; // "(null)"
				else
				{
				   nItemLen = lstrlenA(pstrNextArg);
				   nItemLen = max(1, nItemLen);
				}
			}
			break;

		case 's'|FORCE_UNICODE:
		case 'S'|FORCE_UNICODE:
			{
				LPWSTR pstrNextArg = va_arg(argList, LPWSTR);
				if (pstrNextArg == NULL)
				   nItemLen = 6; // "(null)"
				else
				{
				   nItemLen = wcslen(pstrNextArg);
				   nItemLen = max(1, nItemLen);
				}
			}
			break;
		}


		// adjust nItemLen for strings
		if (nItemLen != 0)
		{
			if (nPrecision != 0)
				nItemLen = min(nItemLen, nPrecision);
			nItemLen = max(nItemLen, nWidth);
		}
		else
		{
			switch (*lpsz)
			{
			// integers
			case 'd':
			case 'i':
			case 'u':
			case 'x':
			case 'X':
			case 'o':
				if (nModifier & FORCE_INT64)
					va_arg(argList, __int64);
				else
					va_arg(argList, int);
				nItemLen = 32;
				nItemLen = max(nItemLen, nWidth+nPrecision);
				break;

			case 'e':
			case 'g':
			case 'G':
				va_arg(argList, DOUBLE_ARG);
				nItemLen = 128;
				nItemLen = max(nItemLen, nWidth+nPrecision);
				break;

			case 'f':
				{
					double f;
					LPTSTR pszTemp;

					// 312 == strlen("-1+(309 zeroes).")
					// 309 zeroes == max precision of a double
					// 6 == adjustment in case precision is not specified,
					//   which means that the precision defaults to 6
					//pszTemp = (LPTSTR)_alloca(max(nWidth, 312+nPrecision+6));
					char temp[324];
					pszTemp=temp;

					f = va_arg(argList, double);
					_stprintf( pszTemp, _T( "%*.*f" ), nWidth, nPrecision+6, f );
					nItemLen = _tcslen(pszTemp);
				}
				break;

			case 'p':
				va_arg(argList, void*);
				nItemLen = 32;
				nItemLen = max(nItemLen, nWidth+nPrecision);
				break;

			// no output
			case 'n':
				va_arg(argList, int*);
				break;

			default:
//				ASSERT(FALSE);  // unknown formatting option
				break;
			}
		}

		// adjust nMaxLen for output nItemLen

		nMaxLen += nItemLen;
	}


	Allocate(nMaxLen);
//	VERIFY(_vstprintf(m_psData, pszFormat, argListSave) <= GetAllocLength());
	_vstprintf(m_psData, pszFormat, argListSave);
	m_nDataLen = lstrlen(m_psData);
//	ReleaseBuffer();

	va_end(argListSave);

}

void TypeString::Allocate(int AllocLen)
{
	if(m_nAllocLen < AllocLen){
		m_nAllocLen = AllocLen;
		m_psData = (LPSTR) realloc((void *)m_psData, m_nAllocLen*sizeof(char) );
	}
}

//find
int TypeString::FindChar(int start, char one) const
{
	for(int i=start; i<m_nDataLen; i++){
		if(m_psData[i]==one)
			return i;
	}
	return -1;
}

int TypeString::FindStr(int start, LPCSTR src, int Len) const
{
	int srcLen = (src==NULL)? 0: (Len==-1)? lstrlen(src): Len;
	if(srcLen == 0)
		return -1;
	for(int i=start; i<m_nDataLen-srcLen+1; i++){
		if(!strncmp(m_psData+i,src,srcLen))
			return i;
	}
	return -1;
}

int TypeString::FindStr(int start, const TypeString &src, int Len) const
{
	return FindStr(start, src.GetpsData(), Len);
}

//substring
TypeString TypeString::SubString( int start, int end) const
{
	TypeString Result;

	Result.AllocMBSTR(*this,start,end);
	return Result;
}


//Extract
TypeString TypeString::ExtractDel( char src, int index) const
{
	TypeString Result;
	int startpos=-1;
	int endpos=-1;
	int currin=0;

	for(int i = 0; i<m_nDataLen; i++){
		if(m_psData[i]==src){
			currin ++;
			if(currin==index-1)
				startpos = i;
			else if(currin==index){
				endpos = i;
				break;
			}
		}
	}
	if(startpos!=-1 || index == 1)
		Result.AllocMBSTR(*this,startpos,endpos);
	return Result;
}

TypeString TypeString::ExtractDel(LPCSTR src, int index) const
{
	TypeString Result;
	int startpos=-1;
	int endpos=-1;
	int currin=0;
	int i,j;
	int srcLen = lstrlen(src);

	for(i = 0; i<m_nDataLen; i++){
		for(j = 0; j<srcLen; j++)
		{
			if(m_psData[i]==src[j]){
				currin ++;
				if(currin==index-1)
					startpos = i;
				else if(currin==index){
					endpos = i;
					break;
				}
			}
		}
	}

	if(startpos!=-1 || index == 1)
		Result.AllocMBSTR(*this,startpos,endpos);
	return Result;
}

TypeString TypeString::ExtractDel(const TypeString& src, int index) const
{
	return ExtractDel(GetpsData(),index);
}

//OPERATOR "="
const TypeString& TypeString::operator =(const TypeString& Str )
{
	AllocMBSTR(Str);
	return *this;
}

const TypeString& TypeString::operator =( LPCSTR mbstrStr)
{
	AllocMBSTR(mbstrStr);
	return *this;
}

const TypeString& TypeString::operator =( LPCWSTR wstrStr)
{
	AllocMBSTR(wstrStr);
	return *this;
}

//OPerator "+="
const TypeString& TypeString::operator +=( const TypeString& Str)
{
	ConCatNMBSTR( Str );
	return *this;
}

const TypeString& TypeString::operator +=( LPCSTR mbstrStr)
{
	ConCatNMBSTR( mbstrStr );
	return *this;
}

char TypeString::operator []( int index ) const
{
	return m_psData[index];
}

// OPerator "+"
TypeString __stdcall operator +(const TypeString &Str1, const TypeString &Str2)
{
	TypeString Result;
	Result = Str1;
	Result += Str2;
	return Result;
}

TypeString __stdcall operator +(const TypeString &Str1, LPCSTR Str2)
{
	TypeString Result;
	Result = Str1;
	Result += Str2;
	return Result;
}

TypeString __stdcall operator +(LPCSTR Str1, const TypeString &Str2)
{
	TypeString Result;
	Result = Str1;
	Result += Str2;
	return Result;
}

//operator "=="
bool __stdcall operator ==(const TypeString &Str1, const TypeString &Str2)
{
	return !(Str1.StringCmp(Str2));
}

bool __stdcall operator ==(const TypeString &Str1, LPCSTR Str2)
{
	return !(Str1.StringCmp(Str2));
}
bool __stdcall operator ==(LPCSTR Str1, const TypeString &Str2)
{
	return !(Str2.StringCmp(Str1));
}

//operator "!="
bool __stdcall operator !=(const TypeString &Str1, const TypeString &Str2)
{
	if(Str1.StringCmp(Str2))
		return true;
	else
		return false;
}

bool __stdcall operator !=(const TypeString &Str1, LPCSTR Str2)
{
	if(Str1.StringCmp(Str2))
		return true;
	else
		return false;
}

bool __stdcall operator !=(LPCSTR Str1, const TypeString &Str2)
{
	if(Str2.StringCmp(Str1))
		return true;
	else
		return false;
}

*/


	void TStrings::set( int i, const char* b)
	{
		(*this)[i]=b;
	}
	void TStrings::set( int i)
	{
		(*this)[i]="";
	}
	std::string TStrings::get(int i)const
	{
		return std::string((*this)[i].ptr());
	}
	void TStrings::pushBack(const char* i)
	{
		TString temp(i);
		pushBack(temp);
	}
