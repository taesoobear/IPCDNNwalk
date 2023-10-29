#ifndef _CHECK_ERROR_H__
#define _CHECK_ERROR_H__

#if _MSC_VER >1000
#pragma once
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 
#endif

#if _MSC_VER > 1000
#pragma warning(disable:4819) // utf-8 without BOM is fine, isn't it M$?
#pragma warning( disable : 4018) // signed, unsigned mismatch warning 을 없앱니다.
#pragma warning( disable : 4267) // size_t to int, possible lost of data warning을 없앱니다.
#endif

#else
void __noop(...);
#endif

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef __APPLE_CC__
#include <malloc.h>
#endif
#include <stdexcept>
#include <memory.h>

#ifndef ASSERT
#ifdef _DEBUG
#define ASSERT(x) assert(x)
#define RANGE_ASSERT(x) do {if(!(x)) throw std::runtime_error("range_error");} while(false)
#define VERIFY(x) assert(x)
#define TRACE	Msg::print
#else
#define ASSERT(x) 
#define RANGE_ASSERT(x) do {if(!(x)) throw std::runtime_error("range_error");} while(false)
//#define RANGE_ASSERT(x) assert(x)
//#define RANGE_ASSERT(x) 
#define VERIFY(x)	(x)
#define TRACE	__noop
#endif
#endif

namespace Msg
{
	void verify(bool bExpression, const char*, ...);// release에서도 없어지지 않는 verify를 하고 싶은경우 사용.
	void print(const char*,...);
	void print2(const char*,...);
	void error(const char*,...);
	void msgBox(const char*,...);
	bool confirm(const char*,...);
	void flush();
	void output(const char* key, const char* formatString, ...);
	void outputState(bool bOutput);

	class Base
	{
	public:
		Base(){m_bOutput=true;}
		virtual ~Base(){}
		// inherit and implement followings
		virtual void print(const char* msg);
		virtual void print2(const char* msg);
		virtual void flush();
		virtual void error(const char* msg);
		virtual void msgBox(const char* msg);
		virtual void output(const char* key, const char* msg);
		// 아직 구현안됨. 사용자에게 yes or no물어보는 기능.
		virtual bool confirm(const char* msg);
		bool m_bOutput;

	};

	extern Base* g_pMsgUtil;
}

#endif
