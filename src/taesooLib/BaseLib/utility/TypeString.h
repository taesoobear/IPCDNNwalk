#ifndef _TSTRING_H_
#define _TSTRING_H_

#pragma once
#include <typeinfo>
#include <limits.h>
#include <cstdarg>
#include "checkError.h"
#include <string>
class TString;
namespace sz0
{
	struct Operator
	{
		Operator(){}
		virtual ~Operator(){}
		virtual void operator()(TString& c) const;
	};
}

// string class mainly for internal use.
//
// examples:
// to convert TString aaa to std::string -->          std::string(aaa.ptr())
// to convert std::string to TString -->              TString(aaa.c_str()) 
//
/// Do not use for printf input directly! O: printf("%s", a.ptr()); X: printf("%s", a)
class TString
{
public:
	TString();
	TString(const char* str);
	TString(const char* str, int i);
	// if(str=="aaa" && i=0) -> this="aaa0"
	TString(const TString& str);
	TString(const sz0::Operator& op);
	~TString();

	const char* ptr(int n=0) const			{ if(n>=length()) return NULL; return m_psData+n; }
	inline operator const char*()	const	{ return ptr();}
	inline const char* c_str() const		{ return ptr(0);}
	inline std::string tostring() const { return std::string(ptr());}
	int length() const						{ return m_nDataLen; }
	void replace(char a, char b);
	void replace(const char* a, const char* b);

	/// Almost private functions. 직접 access. Do not forget to updateLength!!!
	char* ptr_ref(int n=0)					{ return m_psData+n; }
	void reserve(int AllocLen);
	void alloc(const char* strData, int nLen = -1);
	void alloc(const TString& Src, int start = 0, int end = INT_MAX);
	void _format(const char* pszFormat, va_list& argList);

	void updateLength();

	//find
	//start부터 nLen을 비교하여 찾는다.(0 - indexing)
	int findChar(int start, char one) const;
	int findCharRight(char one, int start=INT_MAX) const;

	// returns -1 when not found
	int findStr(int start, const char* src, int Len=-1) const;
	int find(const char* src) const;

	void makeUpper();
	TString toUpper();
	// i번째 character가 src의 문자중 하나인지?
	bool isOneOf(int i, const TString& src) const;

	// loop를 돌면서 start가 end가 되면 멈출것.
	void token(int &start, const TString& delimiter, TString& token) const;

	//delimiter로 구분되는 index번째 토큰 (1 - indexing) ex) "abc def".token(" ,",1)=="abc"
	TString token(char delimiter, int index = 1) const;
	TString token(const char* delimiter, int index = 1) const;

	//substring : [start, end] 사이의 string
	TString subString( int start = 0, int end = INT_MAX) const;

	/// n<0 인경우 반대쪽부터 카운트.
	TString left(int n) const				{ if(n<0) return subString(0, length()+n); return subString(0, n);}
	/// n<0 인경우 반대쪽부터 카운트.
	TString right(int n) const				{ if(n<0) return TString(ptr(-1*n)); int start=length()-n; return subString(start);}

	void trimLeft(const char* delimiter);
	void trimRight(const char* delimiter);

	const char* right2(int n) const			{ int start=length()-n; return ptr(start);}

	//compare
	int strncmp(const TString &str, int n=-1) const;
	int strcmp(const TString &str) const;
	int strcmp(const char* str) const;

	int isEmpty() const;
	void empty();

	void op0(const sz0::Operator& op)	{op(*this);}

	//sprintf
	TString& add(const char* pszFormat, ...);
	TString& format(const char* pszFormat, ...);

	//OPerater "="
	const TString& operator =( const TString& Str );
	const TString& operator =( const char* strStr);
	//OPerator "+="
	const TString& operator +=( const TString& Str );
	const TString& operator +=( const char* mbstrStr );
	//OPerator "[]"
	char& operator []( int index ) const	{	return m_psData[index];	}
	char& value(int index) const			{	return m_psData[index];	}

	//Concat
	void concat(const TString &str, int n = 0);
	void concat(const char*  str, int n =0);

	//operator "+"
	TString operator +(const TString &Str2) const;
	TString operator +(const char* Str2) const;
	TString operator +(int a) const;
	friend TString operator +(const char* Str1, const TString &Str2);

	//operator "=="
	bool operator ==(const TString &Str2) const;
	bool operator ==(const char* Str2) const;
	friend bool  operator ==(const char* Str1, const TString &Str2);

	//operator "!="
	bool operator !=(const TString &Str2) const;
	bool operator !=(const char* Str2) const;
	friend bool operator !=(const char* Str1, const TString &Str2);

private:

	void initData();
	void freeData();

	char* m_psData;

	//data,alloc Len
	int m_nDataLen;
	int m_nAllocLen;



};


#include <vector>
class TStrings
{
	std::vector<TString> mStrings;
public:
	TStrings(){}
	TStrings(int n){  init(n);}
	virtual~TStrings(){}

	TString& operator[](int i)				{ RANGE_ASSERT(i>=0 && i<size());return mStrings[i];}
	TString& data(int i)					{ RANGE_ASSERT(i>=0 && i<size());return mStrings[i];}
	TString& back()							{ RANGE_ASSERT(size()>0); return data(size()-1);}

	TString const& operator[](int i)const	{ RANGE_ASSERT(i>=0 && i<size());return mStrings[i];}
	TString const& data(int i)const			{ RANGE_ASSERT(i>=0 && i<size());return mStrings[i];}
	TString const& back() const				{ RANGE_ASSERT(size()>0);return data(size()-1);}
	void init(int n);
	// eg) setStrings(3, "hihi", "hehe", "hohoho");
	void setStrings( int n, const char* s1, ... );

	int size() const						{ return (int)mStrings.size();}
	void resize(int n)						{ mStrings.resize(n);}
	void pushBack(const TString& other)		{ mStrings.push_back(other);}

	TString prefix() const;	// 모든 문자열이 같은 prefix로 시작하는 경우 해당 prefix return.
    void trimSamePrefix(const TStrings& other);	// 모든 문자열이 같은 prefix로 시작하는 경우 잘라준다.
	// 못찾으면 size() return;
	int find(const char* other) const;
	void set( int i, const char* b);
	void set( int i);
	std::string get(int i) const;
	void pushBack(const char* i);
};
#endif
