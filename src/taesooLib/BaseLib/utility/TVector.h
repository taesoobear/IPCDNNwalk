#ifndef _UTIL_TVECTOR_H_
#define _UTIL_TVECTOR_H_

#if _MSC_VER>1000
#pragma once
#endif

#include "stdtemplate.h"

///////////////////////////////////////////////
//
//  T*의 어레이.
// slicing이 가능하다.
// ex. TVector<int> a(3), b;
// a[0]=0;
// a[1]=2;
// a[2]=3;
// b.assignRef(a);
// b[0]=3;
// print(a[0]); -> prints 3.

// TVector<int> c=a.range(0,2);

template <class T> class _TMatrix;

template <class T>
class _TVector
{
	typedef T* T_PTR;

	std::vector<T*> _buffer;	// valid only if owner==true.

	TFactory<T>* m_pFactory;
	T_PTR* ptr;
	int stride; // space between two succesive elements
	int n, on;	// size of memory allocated.
	bool owner;	// ptr is allocated by this, so will be freed by this.

protected:

	void initElt(int i)
	{
		if(_buffer[i]==NULL)
			_buffer[i]=m_pFactory->create(i);
	}

	void deinitElt(int i)
	{
		ASSERT(_buffer[i]!=NULL);
		m_pFactory->release(_buffer[i]);
		_buffer[i]=NULL;
	}

	// disable default constructors (Do not use this class directly!)
	_TVector (T_PTR* ptrr, int size, int stride);	// reference
	_TVector (const _TVector<T>& other);
	_TVector ();

	// L-value로 사용될수 있는, reference array로 만든다.
	void assignRef(const _TVector<T>& other);

	void changeFactory(TFactory<T>* pF, bool bErrorCheck=true)		{	if(bErrorCheck) Msg::verify(n==0 && on==0, "array may become inhomogeneous"); delete m_pFactory; m_pFactory=pF; }

	template <class vecViewType>
	vecViewType _range(int start, int end, int step)
	{
		int nSize;
		if(step==1)
			nSize=(end-start);
		else
			nSize=(end-start+1)/step;

		return vecViewType(ptr+start*stride, nSize, step*stride);
	}

	template <class matViewType>
	matViewType _column() const
	{
		// return n by 1 matrix, which can be used as L-value (reference matrix)
		matViewType c(ptr, size(), 1, stride);//sizeof(m_real));
		return c;
	}

	template <class matViewType>
	matViewType _row() const		// return 1 by n matrix, which can be used as L-value (reference matrix)
	{
		matViewType c(ptr, 1, size(), size());
		return c;
	}

public:
	~_TVector();

	bool isReference() const	{ return !owner;}

	void _getPrivate(T_PTR*& ptr, int& stride, int& n, int& on) const;

	// reference가 value로 카피된다.
	_TVector<T>& assign(const _TVector<T>& other);

	// a.value(0)==a[0]==a(0)
	inline T&   value(int i) const							{ ASSERT(i>=0 && i<n); return **(ptr+i*stride);}
	inline T&   operator[](int i) const						{ return value(i); }
	inline T&   operator()(int i) const						{ return value(i);}

	int  size() const										{ return n; }
    void setSize( int x );
	int getStride() const                                  { return stride;}
	void resize(int nsize);		// always preserves data, and fills empty entries by 0.
	void reserve( int rr)								{ if(rr>on) { int prev_n=n; resize(rr); n=prev_n; }}
	void swap(int i, int j)								{ T temp=(*this)[i]; (*this)[i]=(*this)[j]; (*this)[j]=temp;};
};

template <class T>
_TVector<T>::_TVector()
{
	on = n = 0;
	stride = 1;
	owner=true;
	m_pFactory=new TDefaultFactory<T>();
}

template <class T>
_TVector<T>::_TVector(T_PTR* ptrr, int size, int str)
{
	on=0;
	ptr=ptrr;
	n=size;
	stride=str;
	owner=false;
	m_pFactory=new TDefaultFactory<T>();
}

template <class T>
_TVector<T>::_TVector(const _TVector<T>& other)
{
	on = n = 0;
	stride = other.getStride();
	owner=true;
	m_pFactory=new TDefaultFactory<T>();
	assign(other);
}

template <class T>
void _TVector<T>::_getPrivate(T_PTR*& ptr2, int& stride2, int& n2, int& on2) const
{
	ptr2=ptr;
	stride2=stride;
	n2=n;
	on2=on;
}


template <class T>
_TVector<T>::~_TVector()
{
	for(int i=0; i<_buffer.size(); i++)
		deinitElt(i);

	delete m_pFactory;
}



template <class T>
void _TVector<T>::setSize( int x )
{
	if(n==x)
		return;

	if(!owner)
		Msg::error("setSize called, but not an owner");

#ifdef _DEBUG
	if(stride!=1)
		Msg::error("setSize called, but stride!=1");
#endif

	if ( on<x )
	{
		// 아래 assert가 fail이 날수 있는 경우는 on==0인데 n!=x인 경우이다.
		ASSERT(on>=n);

		_buffer.resize(x);
		ptr= &_buffer[0];

		for(int i=on; i<x; i++)
		{
			_buffer[i]=NULL;
			initElt(i);
		}
		on = x;
	}

	//_buffer.resize(x);
	n = x;
}


template <class T>
void _TVector<T>::resize(int nsize)
{
	if(n==nsize)
		return;

	if(!owner)
		Msg::error("resize called, but not an owner");

#ifdef _DEBUG
	if(stride!=1)
		Msg::error("setSize called, but stride!=1");
#endif

	if(nsize<=n)
		setSize(nsize);
	else if(nsize<=on)
	{
		int prev_size=n;
		setSize(nsize);
	}
	else
	{
		int capacity=MAX(on,10);
		// capacity가 nsize를 포함할때까지 doubling
		while(capacity<nsize)	capacity*=2;

		ASSERT(_buffer.size()==on);

		_buffer.resize(capacity);
		ptr=&_buffer[0];

		for(int i=on; i<capacity; i++)
		{
			_buffer[i]=NULL;
			initElt(i);
		}
		on=capacity;
		int prevSize=n;
		n=nsize;
	}
}


template <class T>
_TVector<T>& _TVector<T>::assign(const _TVector<T>& other)
{
	setSize(other.size());
	for( int i=0; i<n; i++ )
		value(i) = other[i];
	return *this;
}

template <class T>
void _TVector<T>::assignRef(const _TVector<T>& other)
{
	other._getPrivate(ptr, stride, n, on);
	on=0;
	owner=false;
}


template <class T> class TVectorView;

template <class T>
class TVector: public _TVector<T>
{
protected:
	TVector<T>(T** ptrr, int size, int stride):_TVector<T>(ptrr,size,stride){}
public:
	TVector<T>():_TVector<T>(){}

	// 값을 카피해서 받아온다.
	template <class VecType>
	TVector<T>(const VecType& other)	{ assign(other);}
	// templated copy constructor does not automatically define the default copy constructor.
	TVector<T>(const TVector<T>& other)	{ assign(other);}

	explicit TVector<T>( int x):_TVector<T>() { _TVector<T>::setSize(x);}
	~TVector<T>(){}

	// L-value로 사용될수 있는, reference array를 만들어 return 한다.
	// ex) v.range(0,2).setValues(2, 1.0, 2.0);
	TVectorView<T> range(int start, int end, int step=1);
	const TVectorView<T> range(int start, int end, int step) const	;


	template <class VecType>
	void operator=(const VecType& other)	{ _TVector<T>::assign(other);}
	// templated assignment operator does not automatically define the default assignment operator.
	void operator=(const TVector<T>& other)	{ _TVector<T>::assign(other);}
};


// Note that TVectorView is not std::container compatible.
template <class T>
class TVectorView:public TVector<T>
{
public:
	// L-value로 사용될수 있는, reference array로 만든다.
	TVectorView (T** ptrr, int size, int stride):TVector<T>(ptrr, size, stride){}
	// 값을 reference로 받아온다.
	template <class VecType>
	TVectorView (const VecType& other)			{ assignRef(other);}
	// templated copy constructor does not automatically define the default copy constructor.
	TVectorView(const TVectorView<T>& other)	{ TVector<T>::assignRef(other);}

	~TVectorView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	template <class VecType>
	void operator=(const VecType& other)		{ _TVector<T>::assign(other);}
	// templated assignment operator does not automatically define the default assignment operator.
	void operator=(const TVectorView<T>& other)	{ _TVector<T>::assign(other);}
};

template <class T>
TVectorView<T> vecViewOffset(TVector<T> const& a, int start)
{
	T**ptr;
	int stride, n, on;
	a._getPrivate(ptr, stride, n, on);

	ptr=ptr-start*stride;
	return TVectorView<T>(ptr, n+start, stride);
}


template <class T>
TVectorView<T> TVector<T>::range(int start, int end, int step)
{


//	return _TVector<T>::_range< TVectorView<T> > (start,end,step);
		int nSize;
		if(step==1)
			nSize=(end-start);
		else
			nSize=(end-start+1)/step;

	T**ptr;
	int stride, n, on;
	TVector<T>::_getPrivate(ptr, stride, n, on);

	return TVectorView<T> (ptr+start*stride, nSize, step*stride);
}

template <class T>
const TVectorView<T> TVector<T>::range(int start, int end, int step) const
{
	return ((const TVector<T>*)this)->range(start, end, step);
}
#endif
