
#include "stdafx.h"
#include "mathclass.h"
#include "float.h"
#include "Metric.h"
#include "floatvec.h"


floatvec& floatvec::operator=(const floatvec& other)		{ assign(other);return *this;}	
floatvec& floatvec::operator=(const floatvecView& other)	{ assign(other);return *this;}


floatvecView ::floatvecView (float* ptrr, int size, int str)
:floatvec(ptrr,size,str)
{
}

/////////////////////////////////////////////////////////////////////////////////


floatvec::floatvec( int n, float x)
:_tvectorn<float>()
{
	ASSERT(n==1);
	setSize(n);
	value(0)=x;
}

floatvec::floatvec( int n, float x, float y)
:_tvectorn<float>()
{
	ASSERT(n==2);
	setSize(n);
	value(0)=x;
	value(1)=y;
}
floatvec::floatvec( int n, float x, float y, float z)
:_tvectorn<float>()
{
	ASSERT(n==3);
	setSize(n);
	value(0)=x;
	value(1)=y;
	value(2)=z;
}


floatvecView floatvec::range(int start, int end, int step)
{
	return _range<floatvecView >(start,end,step);
}

const floatvecView floatvec::range(int start, int end, int step) const	{ return ((floatvec*)this)->range(start, end, step);}



floatvec& floatvec::concaten(floatvec const& a)					
{
	floatvec& c=*this;
	int prev_size=c.size();
	c.resize(c.size()+a.size());
	c.range(prev_size, prev_size+a.size())= a;
	return c;
}


/////////////////////////////////////////////////////////////////////////////////
floatvec::floatvec()
:_tvectorn<float>()
{
}


floatvec::floatvec(const vector3& other)
:_tvectorn<float>()
{
	assign(other);
}

floatvec::floatvec(const quater& other)
:_tvectorn<float>()
{
	assign(other);
}

// 값을 카피해서 받아온다.	
floatvec::floatvec(const _tvectorn<float>& other)
:_tvectorn<float>()
{
	_tvectorn<float>::assign(other);
}	

floatvec::floatvec(const floatvec& other)
:_tvectorn<float>()
{
	assign(other);
}	

floatvec::floatvec(const floatvecView& other)
:_tvectorn<float>()
{
	assign(other);
}	


floatvec& floatvec::assign(const vector3& other)
{
	setSize(3);
	for(int i=0; i<3; i++)
		value(i)=other.getValue(i);
	return *this;
}
floatvec& floatvec::assign(const vectorn& other)
{
	setSize(3);
	for(int i=0; i<other.size(); i++)
		value(i)=other.getValue(i);
	return *this;
}


floatvec& floatvec::assign(const quater& other)
{
    setSize(4);
	for(int i=0; i<4; i++)
		value(i)=other.getValue(i);
	return *this;
}






TString floatvec::output(const char* formatString, int start, int end) const
{
	TString id;
	if(end>size()) end=size();
	id+="[";
	for(int i=start; i<end; i++)
	{
		id.add(formatString, (*this)[i]);
		id+=",";
	}
	id+="]";

	return id;
}




void floatvec::setVec3( int start, const vector3& src)	{RANGE_ASSERT(start+3<=size()); for(int i=start; i<start+3; i++) (*this)[i]=src.getValue(i-start);}
void floatvec::setQuater( int start, const quater& src)	{RANGE_ASSERT(start+4<=size());for(int i=start; i<start+4; i++) (*this)[i]=src.getValue(i-start);}
vector3 floatvec::toVector3(int startIndex)	const	{RANGE_ASSERT(startIndex+3<=size());vector3 out; for(int i=0; i<3; i++) out[i]=getValue(i+startIndex); return out;};
quater floatvec::toQuater(int startIndex) const		{RANGE_ASSERT(startIndex+4<=size()); quater out; for(int i=0; i<4; i++) out[i]=getValue(i+startIndex); return out;};

std::ostream& operator<< ( std::ostream& os, const floatvec& u )
{
	return (os << u.output().ptr());
}
