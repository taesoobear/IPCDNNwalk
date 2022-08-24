#ifndef __boolN_H_
#define __boolN_H_
#pragma once

namespace BaseLib {
//! BitArray의 Fast Version
/*! 다른 종류의 BitArray는 BitProxy class를 사용하여 a[10]=0 처럼 lvalue로 사용되는 것이 가능하지만,
이 BitArray는 lvalue로 사용되는 경우 Assign 또는 SetAt, ClearAt을 사용하여야 한다.
또한 32보다 큰 크기의 array로 사용할 수 없다.
그 대신 직접 bitwise operation을 수행하는 것에 비해 전혀 performance overhead가 없다.
*/
class BitArray
{
public:
	BitArray(const BitArray& other) { m_Bits=other.m_Bits; };
	BitArray() { m_Bits=0; };
	~BitArray()	{};
	//! b=a[10] 처럼 사용할 수 있다.
	inline bool operator[](int nIndex) const				{ return (m_Bits>>nIndex &1);}
	//! SetAt(1)은 a[1]=true에 해당
	inline void SetAt(int nIndex)							{ m_Bits|=1<<nIndex; }
	inline void SetAt(int nIndex,int value)			{ if(value)m_Bits|=1<<nIndex;else m_Bits&=~(1<<nIndex); }
	//! ClearAt(1)은 a[1]=false에 해당
	inline void ClearAt(int nIndex)							{ m_Bits&=~(1<<nIndex); }

	inline void Assign(int nIndex, bool bit)				{ if(bit) SetAt(nIndex); else ClearAt(nIndex);}

	inline void ClearAll()									{ m_Bits=0;}
	int GetCount() const				{	int count=0; for(int i=0; i<32; i++) if(m_Bits>>i&1)	count++; return count; }

	// bitwise or
	friend BitArray operator|(const BitArray& a, const BitArray& b)	{ BitArray c; c.m_Bits=a.m_Bits|b.m_Bits; return c;}
	// bitwise and
	friend BitArray operator&(const BitArray& a, const BitArray& b)	{ BitArray c; c.m_Bits=a.m_Bits&b.m_Bits; return c;}

	BitArray& operator|=( const BitArray & other)					{ m_Bits|=other.m_Bits; return *this;}
	BitArray& operator&=( const BitArray & other)					{ m_Bits&=other.m_Bits; return *this;}
	void operator=(const BitArray& other)							{ m_Bits=other.m_Bits;}
	unsigned int m_Bits;
	
};
}

// boolN (range operation is possible)
class _boolN_worker; // only for internal use.
class boolNView;
class boolN
{
protected:
	boolN(_boolN_worker* v):_vec(v),_owner(false){}

	void _own();
public:
  _boolN_worker* _vec;
  int _start, _end;
  bool _owner;

  boolN()
  {
	  _own();
  }
  boolN(int n)
  { 
	  _own();
	  resize(n);
  }
  boolN(const boolN& other)
  {
	_own();
	assign(other);
  }
  boolN(const boolNView& other);
  boolN( vectorn const& a);
  virtual ~boolN();
  virtual void resize(int n);
  
  int size()const{return _end-_start;}

  void assignBit(const _boolN_worker& other);

  // always copy values
  void assign(const boolN& other);

  void operator=(const boolN& other)			{ assign(other);}
  void operator=(const boolNView& other)			{ assign(other);}
  void operator=( vectorn const& a);

  bool operator[](int i) const;
  inline bool operator()(int i) const { return (*this)[i];}
  inline bool value(int i) const { return (*this)[i];}

  //! 일종의 windowed global optimum이다. 단 candidate는 localOptimum이다. 소스코드 참고.
  //void refineLocalOptimum(const vectorn& signal, const _boolN_worker& localOptimum, int windowSize, zeroCrossingMode mode=ZC_MIN);
  void makeJumpIndex(intvectorn& jumpIndex) const;
  //! output 크기는 (end-start)*2-1이다. true인 군집의 center위치에 true로 assign한다.
  /// input    1 0 1 1 1 0 1 1 0 0
  /// centers  1000001000000100000
  void centers(int start, int end, const intvectorn& jumpIndex, const boolN& bits, bool bLocal=true);
	/// if there is no bValue return size;
	int find(int start, int end, bool bValue=true) const;
	int find(int start, bool bValue=true) const;
	int findPrev(int start, bool bValue=true) const;
		// find bValue which is nearest to i. if fail, return -1;
		int findNearest(float i, bool bValue=true) const;
		float distanceToNearest(float i, bool bValue=true) const;

	void op(int (*s2_func)(int,int), const intvectorn& source, int value, int start=0, int end=INT_MAX);
	void op(m_real (*s2_func)(m_real,m_real), const vectorn& source, m_real value, int start=0, int end=INT_MAX);
  void set(int i, bool b);
  bool get(int i) const { return value(i);}
  void setAt(int i) 	{ set(i, true);}
  void clearAt(int i) 	{ set(i, false);}

  void setAt(const intvectorn& aIndex);
  void clearAt(const intvectorn& aIndex);
  void setValue(int start, int end, bool bit);
	void setAllValue(bool b);
	void clearAll() 	{ setAllValue(false);}
	int count() const;
  TString output() const;
  boolNView range(int start, int end) const;
  void getRawData(intvectorn& out) const;
  int calcRawDataSize();
  void setRawData(intvectorn const& in, int size);
	void save(const char* filename);	// text file에 쓰기.ex) a.txt = 1 3 6 12
	void load(int size, const char* filename);	// text file에서 읽기.ex) a.txt = 1 3 6 12



	// primary functions
	// binary OP
	void _or(const boolN& a, const boolN& b);
	void _and(const boolN& a, const boolN& b);
	boolN operator|(boolN const& b) const;
	boolN operator&(boolN const& b) const;

	// unary OP
	boolN& operator|=(boolN const& a);

	boolN& operator&=(boolN const& a);

	bool operator==(boolN const& other) const;
	bool operator!=(boolN const& other) const	{ return !operator==(other);};
	// utility functions
	enum zeroCrossingMode { ZC_MIN, ZC_MAX, ZC_ALL};
	//! zero-crossing을 찾는다.	ZC_MIN: negative to positive crossing, ZC_MAX: positive to negative crossing
	void findZeroCrossing(const vectorn& signal, zeroCrossingMode mode=ZC_ALL);
	//! local optimum을 찾는다. ZC_MIN: local minimum, ZC_MAX: local maximum
	void findLocalOptimum(const vectorn& signal, zeroCrossingMode mode=ZC_MIN);
	friend std::ostream& operator<< ( std::ostream& os, const boolN& u );
};

class boolNView : public boolN
{
public:
  boolNView(_boolN_worker* vec, int start, int end):boolN(vec)
	  {_start=start,_end=end;}
  boolNView(const boolNView& other):boolN(other._vec)
  {
	_start=other._start;
	_end=other._end;
  }
  boolNView(const boolN& other):boolN(other._vec)
  {
	_start=other._start;
	_end=other._end;
  }
  virtual ~boolNView(){_vec=NULL;}
  
  virtual void resize(int n)
  {
	  RANGE_ASSERT(size()==n);
  }
  void operator=(const boolN& other)			{ assign(other);}
  void operator=(const boolNView& other)			{ assign(other);}
  
};

typedef boolN bitvectorn; // for backward compatibility
#endif
