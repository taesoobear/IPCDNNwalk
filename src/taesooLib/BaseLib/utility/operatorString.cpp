// TypeString.cpp: implementation of the TypeString class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "util.h"
#include "TypeString.h"
#include "operatorString.h"
#include <stdio.h>
#include <string.h>

void sz0::replace::operator()(TString& c) const
{
	TString out;
	
	bitvectorn index;
	index.resize(c.length());
	index.clearAll();

	for(int i=0; i<c.length()-mPattern.length()+1; i++)
	{
		if(c.subString(i, i+mPattern.length())==mPattern)
		{
			index.setAt(i);
			i+=mPattern.length()-1;
		}
	}

	int count=index.count();

	out.reserve(c.length()-mPattern.length()*count+mReplacepattern.length()*count+1);
	out.empty();

	char character[2];
	character[1]=0;
	int i;
	for(i=0; i<c.length()-mPattern.length()+1; i++)
	{
		if(index[i])
		{
			out.add(mReplacepattern);
			i+=mPattern.length()-1;
		}
		else
		{
			character[0]=c[i];
			out.concat(character);				
		}
	}

	if(mPattern.length()>1)
		out.add(c.subString(i));

	c=out;
}

void sz0::filename ::operator()(TString& c) const
{
	int index1, index2;
	index1=c.findCharRight('\\');
	index2=c.findCharRight('/');
	int index=(index1>index2)?index1:index2;
	if(index!=-1)
		c=c.right(-1*index-1);
}

void sz0::Operator::operator()(TString& c) const	{ Msg::error("v1::%s::operator()(vr) not implemented!!!\n", typeid( *this).name());}
void sz0::zero::operator()(TString& c) const
{
	m_op.operator()(c);
	c.replace(' ','0');
}

sz0::format::format(const char* pszFormat,...)
{
	va_list argptr ;
	va_start(argptr, pszFormat);
	m_str._format(pszFormat, argptr);
}

sz0::format0::format0(const char* pszFormat,...)
{
	va_list argptr ;
	va_start(argptr, pszFormat);
	m_str._format(pszFormat, argptr);
	m_str.replace(' ','0');
}

void sz0::format::operator()(TString& c) const
{
	c=m_str;
}

void sz0::format0::operator()(TString& c) const
{
	c=m_str;
}

TString sz1::format(const char* pszFormat, ...) 
{
	TString m_str;
	va_list argptr ;
	va_start(argptr, pszFormat);
	m_str._format(pszFormat, argptr);
	return m_str;
}

TString sz1::format0(const char* pszFormat, ...) 
{
	TString m_str;
	va_list argptr ;
	va_start(argptr, pszFormat);
	m_str._format(pszFormat, argptr);
	m_str.replace(' ','0');
	return m_str;
}

// extract filename: eg) fn="c:\a\b.bmp" -> return "b.bmp", where dir="c:\a"
TString sz1::filename(TString const& fn, TString& dir)
{
	TString lfn;
	// split fn into dir+lfn
	int currDir;
	if((currDir=fn.findCharRight('/'))!=-1)
	{
		lfn=fn.right(-1*currDir-1);
		dir=fn.left(currDir);
	}
	else
	{
		dir.empty();
		lfn=fn;
	}

	return lfn;
}

TString sz1::extension(TString const& fn)
{
	int pos;
	if((pos=fn.findCharRight('.'))!=-1)
	{
		return fn.right(-1*pos-1);
	}
	TString temp;
	temp.empty();
	return temp;
}

TString sz1::parentDirectory(TString const& childDirectory)
{
	for(int i=1,ni=childDirectory.length(); i<ni; i++)
	{
		if(childDirectory[ni-i-1]=='\\' || childDirectory[ni-i-1]=='/')
		{
			return childDirectory.left(-1*i);
		}
	}
	return TString();
}
