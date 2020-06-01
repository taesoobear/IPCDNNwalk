#ifndef __NAMEDTYPES_H__
#define __NAMEDTYPES_H__

#pragma warning (disable: 4786)
#include <map>
#include <string>
#include <functional>

#pragma warning (disable: 4800)

struct ltstr: std::binary_function<const std::string&, const std::string&  , bool>
{
	bool operator()(const std::string &_X, const std::string &_Y) const
	{
		return _X.compare(_Y)<0;
	}
};

struct ltsz: std::binary_function<char* const &, char* const &, bool>
{
	bool operator()(char* const & _X, char* const & _Y) const;
};

struct ltcsz: std::binary_function<const char* , const char* , bool>
{
	bool operator()(const char* _X, const char* _Y) const;
};

struct ltint: std::binary_function<const int&, const int&  , bool>
{
	bool operator()(const int &_X, const int &_Y) const
	{
		return _X<_Y;
	}
};

class TString;
struct cmpTString: std::binary_function<TString const &, TString const &, bool>
{
	bool operator()(TString const & _X, TString const & _Y) const;
};


#pragma warning (disable: 4786)
/// char*으로 부터 int를 얻어낼때 사용
typedef std::map<char*, int, ltsz> namedmapInt2;
/// string으로 부터 void를 얻어낼때 사용
typedef std::map<std::string , void *, ltstr> namedmapVoidData;
/// string으로 부터 int를 얻어낼때 사용
typedef std::map<std::string , int, ltstr> namedmapInt;
/// int로 부터 LPVOID를 얻어낼때 사용
typedef std::map<int, void*, ltint> intmapVoidData;

#endif //__NAMEDTYPES_H__
