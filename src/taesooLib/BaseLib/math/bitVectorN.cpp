#include "stdafx.h"
#include "mathclass.h"
#include "./bitVectorN.h"

class vectorn;
class intvectorn;
#include "../utility/TArray.h"
// used only in this file.
class _boolN_worker
{
	public:
		_boolN_worker(void);
		_boolN_worker(const _boolN_worker& other);
		~_boolN_worker(void);

		void setSize(int size);
		void resize(int size);

		//! b=a[10] 처럼 사용할 수 있다.
		bool operator[](int nIndex) const;
		bool operator()(int nIndex) const;
		bool getValue(int nIndex) const	;
		bool value(int nIndex) const;

		//! SetAt(1)은 a[1]=true에 해당
		void setAt(int nIndex);
		void setAll();

		void toggle(int nIndex);

		//! ClearAt(1)은 a[1]=false에 해당
		void clearAt(int nIndex);
		void clearAll();

		void setValue(int nIndex, bool bit);
		void setValue(int start, int end, bool bit);
		void setAllValue(bool bit);

		int count()	const;
		int size() const;

		_boolN_worker& operator=( _boolN_worker const& );

		/// if there is no bValue return end;
		int find(int start, int end, bool bValue=true) const;
		int find(int start, bool bValue=true) const;


		// deprecated.
		void output(TString& id, int start=0, int end=INT_MAX) const;
		TString output(int start=0, int end=INT_MAX) const;

		inline int calcBitArrayPos(int nIndex) const			{ return nIndex/32;};
		inline int calcBitArrayIndex(int nIndex) const			{ return nIndex%32;};
		int m_nSize;
		std::vector<BitArray> m_aBitArrays;
};

_boolN_worker::_boolN_worker()
{
	m_nSize=0;
}

_boolN_worker::_boolN_worker(const _boolN_worker& other)
{
	m_nSize=0;
	this->operator =(other);
}

boolN::boolN(const vectorn& other)
{
	_own();
	this->operator =(other);
}

_boolN_worker::~_boolN_worker()
{
}

void _boolN_worker::setSize(int size)
{
	int BAsize=calcBitArrayPos(size)+1;
	if(BAsize>m_aBitArrays.size())
	{
		m_aBitArrays.resize(BAsize);
	}
	m_nSize=size;
}

void _boolN_worker::resize(int size)
{
	int prevSize=m_nSize;
	setSize(size);
	setValue(prevSize,size, false);
}

_boolN_worker& _boolN_worker::operator=( _boolN_worker const& a)
{
	_boolN_worker &c = (*this);
	c.setSize( a.size() );

	for( int i=0; i<c.size(); i++ )
		c.setValue(i, a[i]);

	return c;
}

void boolN::operator=( vectorn const& a)
{
	boolN &c = (*this);
	c.resize( a.size() );

	for( int i=0; i<c.size(); i++ )
		c.set(i, (!isSimilar(a[i],0.0) ));

}
bool boolN::operator==(boolN const& other) const
{
	if(size()!=other.size()) return false;
	for(int i=0; i<size(); i++)
		if((*this)[i]!=other[i])
			return false;
	return true;
}

void boolN::findZeroCrossing(const vectorn& vec, zeroCrossingMode mode)
{
	int i;
	resize(vec.size());
	clearAll();
	switch(mode)
	{
		case ZC_MIN:
			for(i=0; i < size()-1; i++)
				if(vec[i]<0 && vec[i+1]> 0)
					setAt(i);
			break;
		case ZC_MAX:
			for(i=0; i < size()-1; i++)
				if(vec[i]>0 && vec[i+1]<0)
					setAt(i);

			for(i=1; i < size()-1; i++)
				if(vec[i-1]>0 && vec[i+1]< 0)
				{
					if(!value(i-1) && !value(i))
					{
						printf("%d\n", i);
						setAt(i);
					}
				}

			break;
		case ZC_ALL:
			for(i=0; i < size()-1; i++)
				if(vec[i]*vec[i+1]<0)
					setAt(i);
			break;
	}
}

void boolN::findLocalOptimum(const vectorn& vec, zeroCrossingMode mode)
{
	if(vec.size()<=2)
	{
		resize(vec.size());
		clearAll();
		return;
	}
	vectorn deriv;
	deriv.derivative(vec);
	findZeroCrossing(deriv, mode);
}

/*void _boolN_worker::refineLocalOptimum(const vectorn& signal, const _boolN_worker& localOptimum, int windowSize, zeroCrossingMode mode)
  {
  setSize(signal.size());
  clearAll();
  ASSERT(signal.size()==localOptimum.size());

  int halfWindow=windowSize/2;
  intvectorn aIndex;
  for(int i=0; i<signal.size(); i++)
  {
  aIndex.findIndex(localOptimum, true, i-halfWindow, i+halfWindow);
  switch(mode)
  {
  case ZC_MIN:
  setAt(signal.Extract(aIndex).argMin());
  break;
  case ZC_MAX:
  setAt(signal.Extract(aIndex).argMax());
  break;
  default:
  ASSERT(0);
  }

  }
  }
  */
int _boolN_worker::find(int start, int end, bool bValue) const
{
	int j;
	for(j=start; j<end; j++)
	{
		if((*this)[j]==bValue)
			return j;
	}
	return j;	// return end;
}

int _boolN_worker::find(int start, bool bValue) const
{
	int j;
	for(j=start; j<size(); j++)
	{
		if((*this)[j]==bValue)
			return j;
	}
	return j;	// return size();
}

int boolN::findPrev(int start, bool bValue) const
{
	int j;
	for(j=start-1; j>=0; j--)
	{
		if((*this)[j]==bValue)
			return j;
	}
	return j;	// return -1;
}


void boolN::_or(const boolN& a, const boolN& b)
{
	boolN &c = (*this);
	assert( a.size()==b.size() );
	c.resize( a.size() );

	for( int i=0; i<a.size(); i++ )
		c.set(i, a[i] || b[i] );
}

void boolN::_and(const boolN& a, const boolN& b)
{
	boolN &c = (*this);
	assert( a.size()==b.size() );
	c.resize( a.size() );

	for( int i=0; i<a.size(); i++ )
		c.set(i, a[i] && b[i] );
}


void boolN::setAt(const intvectorn& aIndex)
{
	for(int i=0; i<aIndex.size(); i++)
	{
		setAt(aIndex[i]);
	}
}

void boolN::clearAt(const intvectorn& aIndex)
{
	for(int i=0; i<aIndex.size(); i++)
	{
		clearAt(aIndex[i]);
	}
}

void boolN::makeJumpIndex(intvectorn& jumpIndex) const
{
	jumpIndex.setSize(size());
	int curr=0;
	while(1)
	{
		bool bValue=get(curr);
		int next=find(curr+1, !bValue);
		for(int i=curr; i<next; i++)
			jumpIndex[i]=next;
		if(next==size()) break;
		curr=next;
	}
}

void boolN::centers(int start, int end, const intvectorn& jumpIndex, const boolN& bits, bool bLocal)
{
	//! output 크기는 (end-start)*2-1이다. true인 군집의 center위치에 true로 assign한다.
	/// input    1 0 1 1 1 0 1 1 0 0
	/// centers  1000001000000100000
	resize((end-start)*2-1);
	clearAll();

	int left, right;
	left=start;
	if(!bits[left])
		left=jumpIndex[left];
	else
	{
		if(!bLocal)
			left=bits.findPrev(left, false)+1;
	}
	ASSERT(left>=end || bits[left]);

	while(left<end)
	{
		right=jumpIndex[left]-1;
		if(bLocal && right>end-1)
			right=end-1;

		int centerIndex=left+right-2*start;
		if(centerIndex>=0 && centerIndex<size())
			setAt(centerIndex);
		if(right+1>=jumpIndex.size()) break;
		left=jumpIndex[right+1];
	}

}

void _boolN_worker::output(TString& id, int start, int end) const
{
	if(end>size()) end=size();
	id+="[";
	for(int i=start; i<end; i++)
	{
		id.add("%1d", (*this)[i]);
	}
	id+="]";

}

TString _boolN_worker::output(int start, int end) const
{
	if(end>size()) end=size();
	TString id;
	id+="[";
	for(int i=start; i<end; i++)
	{
		id.add("%1d",(int)(*this)[i]);
	}
	id+="]";
	return id;
}

void _boolN_worker::setValue(int start, int end, bool bit)
{
	for(int i=start; i<end; i++)
		setValue(i,bit);
}

void boolN::op(int (*s2_func)(int,int), const intvectorn& source, int value, int start, int end)
{
	if(start<0) start=0;
	if(end>source.size()) end=source.size();

	resize(end-start);
	clearAll();

	for(int i=start; i<end; i++)
		if(s2_func(source[i], value)) setAt(i-start);

}

void boolN::op(m_real (*s2_func)(m_real ,m_real), const vectorn& source, m_real value, int start, int end)
{
	if(start<0) start=0;
	if(end>source.size()) end=source.size();

	resize(end-start);
	clearAll();

	for(int i=start; i<end; i++)
		if(s2_func	(source[i], value)) setAt(i-start);

}

#include "../utility/TextFile.h"

void boolN::save(const char* filename)
{
	// text file에 쓰기.ex) a.txt = 1 3 6 12
	FILE* file;
	file=fopen(filename, "wt");

	if(file)
	{
		for(int i=0; i<size(); i++)
		{
			if((*this)[i])
				fprintf(file, "%d ", i);
		}
		fclose(file);
	}
	else
	{
		Msg::print("file open error %s", filename);
	}
}

void boolN::load(int numFrame, const char* filename)
{
	resize(numFrame);
	clearAll();
	CTextFile stopMarking;
	if(stopMarking.OpenReadFile(filename))
	{
		char* token;
		while(token=stopMarking.GetToken())
		{
			int index=atoi(token);
			if(index>size())
			{
				Msg::print("Warning! _boolN_worker loading = invalid index at %d\n", index);
				resize(index+1);
			}
			setAt(index);
		}
	}
	else
	{
		Msg::print("file open error %s", filename);
	}
}

int boolN::findNearest(float i, bool bValue) const
{
	int start=int(floor(i));
	if(i-floor(i) > 0.5)
	{
		// +쪽 부터 뒤져야 한다.
		for(int i=0; i<size(); i++)
		{
			int right=start+i+1;
			int left=start-i;
			if(right<size() && value(right)==bValue) return right;
			if(left>=0 && value(left)==bValue) return left;
			if(left<0 || right>=size())
				return -1;
		}
	}
	else
	{
		// -쪽 부터 뒤져야 한다.
		for(int i=0; i<size(); i++)
		{
			int right=start+i+1;
			int left=start-i;
			if(left>=0 && value(left)==bValue) return left;
			if(right<size() && value(right)==bValue) return right;
			if(left<0 || right>=size())
				return -1;
		}
	}
	return -1;
}

float boolN::distanceToNearest(float i, bool bValue) const
{
	int n=findNearest(i,bValue);
	if(n==-1) return FLT_MAX;
	return ABS(i-(float)n);
}

bool _boolN_worker::operator[](int nIndex) const
{
	ASSERT(nIndex<m_nSize);
	return m_aBitArrays[calcBitArrayPos(nIndex)][calcBitArrayIndex(nIndex)];
}

boolN boolN::operator|( boolN const& b) const
{
	boolN const& a=*this;
	boolN c; c._or(a,b); return c;
}


boolN boolN::operator&(boolN const& b) const
{
	boolN const& a=*this;
	boolN c; c._and(a,b); return c;
}

boolN& boolN::operator|=(boolN const& a)
{
	(*this)._or((*this),a); return *this;
}
boolN& boolN::operator&=(boolN const& a)
{ (*this)._and((*this),a); return *this;};


bool _boolN_worker::operator()(int nIndex) const
{
	return (*this)[nIndex];
}

bool _boolN_worker::getValue(int nIndex) const
{
	return this->operator [](nIndex);
}
bool _boolN_worker::value(int nIndex) const
{
	return this->operator [](nIndex);
}

//! SetAt(1)은 a[1]=true에 해당
void _boolN_worker::setAt(int nIndex)
{
	ASSERT(nIndex<m_nSize);
	m_aBitArrays[calcBitArrayPos(nIndex)].SetAt(calcBitArrayIndex(nIndex));
}

void _boolN_worker::toggle(int nIndex)
{
	if(value(nIndex))
		clearAt(nIndex);
	else
		setAt(nIndex);
}
//! ClearAt(1)은 a[1]=false에 해당
void _boolN_worker::clearAt(int nIndex)
{
	ASSERT(nIndex<m_nSize);
	m_aBitArrays[calcBitArrayPos(nIndex)].ClearAt(calcBitArrayIndex(nIndex));
}


void _boolN_worker::setValue(int nIndex, bool bit)
{
	if(bit) setAt(nIndex);
	else clearAt(nIndex);
}

void _boolN_worker::setAllValue(bool bit)
{
	if(bit) setAll();
	else clearAll();
}

void _boolN_worker::setAll()
{
	setValue(0, size(), true);
}

void _boolN_worker::clearAll()
{
	int index=calcBitArrayPos(m_nSize)+1;
	for(int i=0; i<index; i++) m_aBitArrays[i].ClearAll();
}
int _boolN_worker::count()	const
{
	if(m_nSize==0) return 0;
	int index=calcBitArrayPos(m_nSize)+1;
	int count=0; for(int i=0; i<index; i++) count+=m_aBitArrays[i].GetCount();
	return count;
}
int _boolN_worker::size() const
{
	return m_nSize;
}

/*
   _boolN_worker boolN::bit()
   {
   _boolN_worker t;
   t.setSize(size());
   for(int i=0; i<size(); i++)
   t.setValue(i, (*this)[i]);
   return t;
   }
   */
void boolN::_own()
{
	_vec=new _boolN_worker();
	_start=0;
	_end=0;
	_owner=true;
}
void boolN::assignBit(const _boolN_worker& other)
{
	Msg::verify(_owner, "boolN::assignBit..");
	(*_vec)=(other);
	_start=0;
	_end=_vec->size();
}
boolN::~boolN()
{
	ASSERT(!_vec || _owner);
	delete _vec;
}
void boolN::resize(int n)
{
	ASSERT(_owner);
	_vec->resize(n);
	_start=0;
	_end=_vec->size();
}
int boolN::count()	const
{
	int count=0;
	for (int i=0; i<size(); i++)
		if((*this)(i)) count++;
	return count;
}
void boolN::assign(const boolN& other) 
{

	if(other._owner && _owner)
	{
		(*_vec)=(*other._vec);
		_start=0;
		_end=_vec-> size();
	}
	else
	{
		resize(other.size());
		for(int i=0; i<size(); i++)
		{
			set(i, other[i]);
		}
	}
}
bool boolN::operator[](int i) const
{
	RANGE_ASSERT(0<=i && i<_end-_start);
	return _vec->operator[](i+_start);
}
int boolN::find(int start, int end, bool bValue) const
{
	int j;
	for(j=start; j<end; j++)
	{
		if((*this)[j]==bValue)
			return j;
	}
	return j;	// return end;
}
int boolN::find(int start, bool bValue) const
{
	int j;
	for(j=start; j<size(); j++)
	{
		if((*this)[j]==bValue)
			return j;
	}
	return j;	// return size();
}
void boolN::set(int i, bool b)
{
	RANGE_ASSERT(0<=i && i<_end-_start);
	_vec->setValue(i+_start,b);
}
void boolN::setValue(int start, int end, bool bit)
{
	for(int i=start; i<end; i++)
		set(i,bit);
}
void boolN::setAllValue(bool b)
{
	for(int i=0; i<size(); i++)
		set(i,b);
}
TString boolN::output() const
{
	TString id;
	id+="[";
	for(int i=0; i<size(); i++)
	{
		id.add("%1d", ((int)(*this)[i]));
	}
	id+="]";
	return id;
}
void boolN::getRawData(intvectorn& out) const
{
	_boolN_worker vec;
	vec.setSize(size());
	for (int i=0; i<size(); i++)
		vec.setValue(i, (*this)[i]);

	int nBitArray;
	nBitArray=vec.calcBitArrayPos(vec.size())+1;
	out.resize(nBitArray);
	for(int i=0; i<nBitArray; i++)
	{
		out[i]=int(vec.m_aBitArrays[i].m_Bits);
	}
}
int boolN::calcRawDataSize()
{
	_boolN_worker temp;
	return temp.calcBitArrayPos(size())+1;
}
void boolN::setRawData(intvectorn const& in, int nsize)
{
	_boolN_worker vec;
	vec.setSize(nsize);
	int nBitArray=in.size();
	for(int i=0; i<nBitArray; i++)
	{
		vec.m_aBitArrays[i].m_Bits=(unsigned int)in[i];
	}
	resize(nsize);
	for (int i=0; i<size(); i++)
		set(i, vec[i]);
}
std::ostream& operator<< ( std::ostream& os, const boolN& u )
{
	return (os << u.output().ptr());
}
boolNView boolN::range(int start, int end) const
{
	auto& a=*this;
	return boolNView (a._vec, a._start+start, a._start+end);
}
