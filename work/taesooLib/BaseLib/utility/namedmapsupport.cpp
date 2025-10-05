#include "stdafx.h"
#include "namedmapsupport.h"
#include <stdio.h>
#include <string.h>
#include "TypeString.h"
		bool ltsz::operator()(char* const & _X, char* const & _Y) const
{
		return strcmp(_X, _Y)<0;
}
	bool ltcsz::operator()(const char* _X, const char* _Y) const
	{
		return strcmp(_X, _Y)<0;
	}
	bool cmpTString::operator()(TString const & _X, TString const & _Y) const
	{
		return _X.strcmp(_Y)<0;
	}
