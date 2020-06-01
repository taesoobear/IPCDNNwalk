#ifndef _BASELIB_H_
#define _BASELIB_H_

#ifdef _MSC_VER
// UTF-8 is fine, isn't it MS?
#pragma warning( disable : 4819)
#endif

#include "utility/checkError.h"


#include <vector>
#include <list>
#include <map>

#define USE_MATHCLASS
#define ALL_CHAR_TOUPPER

//! Resource Handle관련 type
typedef int ResourceHandle;
//! Resource Handle 상수
#define HANDLE_NONE -1

#include "utility/tfile.h"
#include "math/mathclass.h"


#include "utility/util.h"
#define FERR 0.0001f
inline double IsZero(double f) { return ((f) < FERR && (f) >-FERR);}

//  Private copy constructor and copy assignment ensure classes derived from
//  class noncopyable cannot be copied.

//  Contributed by Dave Abrahams

class noncopyable
{
protected:
	noncopyable() {}
	~noncopyable() {}
private:  // emphasize the following members are private
	noncopyable( const noncopyable& );
	const noncopyable& operator=( const noncopyable& );
};


/*
typedef unsigned long       DWORD;
#ifndef VOID
#define VOID void
typedef char CHAR;
typedef short SHORT;
typedef long LONG;
#endif

#define _ULONGLONG_
#if (!defined (_MAC) && (!defined(MIDL_PASS) || defined(__midl)) && (!defined(_M_IX86) || (defined(_INTEGRAL_MAX_BITS) && _INTEGRAL_MAX_BITS >= 64)))
typedef __int64 LONGLONG;
typedef unsigned __int64 ULONGLONG;

#define MAXLONGLONG                      (0x7fffffffffffffff)
#else

#if defined(_MAC) && defined(_MAC_INT_64)
typedef __int64 LONGLONG;
typedef unsigned __int64 ULONGLONG;

#define MAXLONGLONG                      (0x7fffffffffffffff)
#else
typedef double LONGLONG;
typedef double ULONGLONG;
#endif //_MAC and int64

#endif

typedef union _LARGE_INTEGER {
    struct {
        DWORD LowPart;
        LONG HighPart;
    };
    struct {
        DWORD LowPart;
        LONG HighPart;
    } u;
    LONGLONG QuadPart;
} LARGE_INTEGER;

#endif
*/

#endif

