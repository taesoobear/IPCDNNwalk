#ifndef _TARRAY_H_
#define _TARRAY_H_

#if _MSC_VER>1000
#pragma once
#endif

#include "stdtemplate.h"
#if _MSC_VER > 1000
#pragma warning (disable: 4786)
#endif
#include "stdlib.h"

// factory 를 어떤거를 쓰는가에 따라 메모리 관리 방식이 달라진다. 메모리 관리는 스스로 책임지기 바람.


// deprecated. std::vector<boost::shared_ptr<...>> 조합 또는 TVector를 사용할 것.
template <class T>
class TArray
{
	/**
	* 기본적인 사용법은 CArray나 std::vector등과 유사하다. 가장 큰 차이점은 Factory에 따라 원소의 초기화를 다르게 할수 있다는 점이다.
	* 특히 TDefaultFactory를 사용한경우(bReference=false인 경우) 모든 원소의 생성자가 call 된다. 이는 모든 원소가 factory를 통해 생성되기 때문에 가능하다.
	* 내부적 구현은 *의 array로, std::vector와 다르다. 만약 bReference==false 이면, new를 call하지 않고, 원소가 NULL로 초기화된다.
	* \Todo
	* n을 입력받는 생성자 함수 두개로 나눌생각. (int, bool) (int, TFactory<T>*) 로...
	*/
public:
	TArray(int n, bool bReference=false);	//!< Use default factory if pF==NULL. if(bReference=true) this class do not "new" or "delete" elements.
	TArray(int n, TFactory<T>* pF);
	TArray(bool bReference=false);	//!< use TDefaultFactory if bReference=false, else use TFactory
	TArray(TFactory<T>* pF);
	TArray(const TArray& other);	//!< copy constructor. (size가 0이 아니면 동작하지 않는다.)
	virtual ~TArray();

	void init(int nsize);
	void resize(int nsize);
	void release();
	int size() const;
	T& operator[](int nIndex) const;
	T& data(int nIndex) const				{ return (*this)[nIndex]; }
	T& back() const							{ return (*this)[size()-1]; }
	T* ptr(int nIndex) const				{ return m_apElement[nIndex];}
	void swap(int i, int j);
	void replace(int i, T* pElement);
	void pushBack(T* pElement)				{ resize(size()+1); replace(size()-1, pElement);}

	//! compare func는 T의 **를 입력으로 받는다. 출력은 element1과 element2의 크기를 비교해서 1,0,-1을 각각 클때, 같을때, 작을때 return한다.
	void sort(int start, int end, int (*compareFunc)(const void** ppElement1, const void** ppElement2));
	void changeFactory(TFactory<T>* pF)		{ASSERT(m_nSize==0); release(); delete m_pFactory; m_pFactory=pF; }
	void changeFactory(TArray<T>& other) const	{ other.changeFactory(m_pFactory->clone());	}
	void assign(const TArray<T>& other);
	void extract(const TArray<T>& other, int nElt, int* aiElt);
	void pushBack(const TArray<T>& other);
	void pushBack(const TArray<T>& other, int nElt, int* aiElt);

	// end이하는 end-start만큼 위로 올라간다. 즉 크기가 end-start만큼 작아진다.
	void remove(int start, int end);

protected:
	T* initElt(int index);
	void deinitElt(int index);
	TFactory<T>* m_pFactory;

	T **m_apElement;
	// m_apElement[0]부터 m_apElement[m_nSize-1]까지는 NULL이 아닌 pointer를 갖는다.
	// m_apElement[m_nSize]부터 m_apElement[m_nCapacity]까지는 NULL pointer를 갖는다.
	// 그 이후를 사용하려하면 m_apElement자체가 doubling된다.
	int m_nSize;
	int m_nCapacity;
};

//#include "stdafx.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

template <class T>
TArray<T>::TArray(int n, bool bReference)
{
	m_nSize=0;
	m_nCapacity=0;
	if(bReference)
		m_pFactory=new TFactory<T>();
	else
		m_pFactory=new TDefaultFactory<T>();
	init(n);
}
template <class T>
TArray<T>::TArray(const TArray& other)
{
	ASSERT(other.m_nSize==0);

	m_nSize=0;
	m_nCapacity=0;

	m_pFactory=other.m_pFactory->clone();
	init(0);
}

template <class T>
void TArray<T>::remove(int start, int end)
{
	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다.
	int numCols=end-start;

	for(int i=start; i<end; i++)
			deinitElt(i);

	for(int i=end; i<size(); i++)
	{
		m_apElement[i-numCols]=m_apElement[i];
	}

	int newSize=size()-numCols;

	for(int i=newSize; i<size(); i++)
		m_apElement[i]=NULL;
	m_nSize=newSize;
}

template <class T>
TArray<T>::TArray(int n, TFactory<T>* pF)
{
	m_nSize=0;
	m_nCapacity=0;
	m_pFactory=pF;
	init(n);
}

template <class T>
TArray<T>::TArray(bool bReference)
{
	m_nSize=0;
	m_nCapacity=0;
	if(bReference)
		m_pFactory=new TFactory<T>();
	else
		m_pFactory=new TDefaultFactory<T>();
}

template <class T>
TArray<T>::TArray(TFactory<T>* pF)
{
	m_nSize=0;
	m_nCapacity=0;
	m_pFactory=pF;
}

template <class T>
T* TArray<T>::initElt(int index)
{
	return m_pFactory->create(index);
}

template <class T>
void TArray<T>::deinitElt(int index)
{
	m_pFactory->release(m_apElement[index]);
	m_apElement[index]=NULL;
}


template <class T>
TArray<T>::~TArray()
{
	release();
	delete m_pFactory;
}

template <class T>
int TArray<T>::size() const
{
	return m_nSize;
}

template <class T>
void TArray<T>::swap(int i, int j)
{
	T* temp;
	temp=m_apElement[i];
	m_apElement[i]=m_apElement[j];
	m_apElement[j]=temp;
}

template <class T>
void TArray<T>::replace(int i, T* pElement)
{
	ASSERT(i<m_nSize);
	deinitElt(i);
	m_apElement[i]=pElement;
}

template <class T>
void TArray<T>::resize(int nsize)
{
	ASSERT(m_nSize<=m_nCapacity);
	if(nsize<=m_nSize)
	{
		for(int i=nsize; i<m_nSize; i++)
			deinitElt(i);
		m_nSize=nsize;
	}
	else if(nsize<=m_nCapacity)
	{
		for(int i=m_nSize; i<nsize; i++)
		{
			m_apElement[i]=initElt(i);
		}
		m_nSize=nsize;
	}
	else
	{
		ASSERT(nsize>m_nCapacity && nsize>m_nSize);
		if(m_nCapacity==0)
		{
			init(nsize);
			return;
		}

		// m_nCapacity가 nsize를 포함할때까지 doubling
		for(;m_nCapacity<nsize;)	m_nCapacity*=2;

		T** apTempElement=new T*[m_nCapacity];
		for(int i=0; i<m_nSize; i++)
			// copy
			apTempElement[i]=m_apElement[i];
		for(int i=m_nSize; i<nsize; i++)
			apTempElement[i]=initElt(i);
		for(int i=nsize; i<m_nCapacity; i++)
			apTempElement[i]=NULL;

		delete[] m_apElement;
		m_apElement=apTempElement;

		m_nSize=nsize;
	}
}

template <class T>
void TArray<T>::init(int nsize)
{
	release();

	m_nSize=nsize;
	m_nCapacity=nsize;
	m_apElement=new T*[nsize];
	for(int i=0; i<m_nSize; i++)
	{
		m_apElement[i]=initElt(i);
	}
}

template <class T>
void TArray<T>::release()
{
	if(m_nSize==0) return;

	for(int i=0; i<m_nSize; i++)
		deinitElt(i);

	delete[] m_apElement;

	m_nSize=0;
	m_nCapacity=0;
}

template <class T>
T& TArray<T>::operator[](int nIndex) const
{
	ASSERT(nIndex>=0 && nIndex<size());
	return *m_apElement[nIndex];
}

template <class T>
void TArray<T>::sort(int start, int end, int (*compareFunc)(const void** ppElement1, const void** ppElement2))
{
	qsort((void*)&(m_apElement[start]), end-start, sizeof(T*), (int (*)(const void*, const void*))compareFunc);
}


template <class T>
void TArray<T>::extract(const TArray<T>& other, int nelt, int* aielt)
{
	changeFactory(new TFactory<T>());	// reference
	resize(nelt);

	for(int j=0; j<nelt; j++)
	{
		ASSERT(ptr(j)==NULL);
		replace(j, other.ptr(aielt[j]));
	}
}

template <class T>
void TArray<T>::assign(const TArray<T>& other)
{
	changeFactory(new TFactory<T>()); // reference
	resize(other.size());

	for(int j=0; j<other.size(); j++)
	{
		ASSERT(ptr(j)==NULL);
		replace(j, other.ptr(j));
	}
}

template <class T>
void TArray<T>::pushBack(const TArray<T>& other)
{
	for(int j=0; j<other.size(); j++)
	{
		pushBack(other.ptr(j));
	}

}
template <class T>
void TArray<T>::pushBack(const TArray<T>& other, int nElt, int* aiElt)
{
	for(int j=0; j<nElt; j++)
	{
		pushBack(other.ptr(aiElt[j]));
	}
}

/// CTArray class is deprecated. do not use this class
template <class T>
class CTArray
{
public:
	CTArray(int n);
	CTArray();
	CTArray(T* (*factory_func)(void* param, int index));
	virtual ~CTArray();

	void Init(int nsize, void* param=NULL);	//!< param은 factory의 parameter로 들어간다. param==NULL이면 default param사용
	void Resize(int nsize, void* param=NULL);//!< 줄이거나 늘리는것 모두 가능. param==NULL이면 default param사용
	void Release();
	int Size() const;
	T& operator[](int nIndex) const;
	void Swap(int i, int j);
	void Replace(int i, T* pElement);
	void pushBack(T* pElement)		{ Resize(Size()+1); Replace(Size()-1, pElement);}
	//! compare func는 T의 **를 입력으로 받는다. 출력은 element1과 element2의 크기를 비교해서 1,0,-1을 각각 클때, 같을때, 작을때 return한다.
	void Sort(int start, int end, int (*compareFunc)(const void** ppElement1, const void** ppElement2));
	void ChangeFactory(T* (*factory_func)(void* param, int index))	{ASSERT(m_nSize==0); Release(); m_factoryFunc=factory_func;	}
	void ChangeDefaultParam(void* param)	{ m_defaultParam=param;};

protected:
	T* (*m_factoryFunc)(void* param, int index);
	static T* default_factory(void* param, int index)	{ return new T();}
	void* m_defaultParam;

	T **m_apElement;
	// m_apElement[0]부터 m_apElement[m_nSize-1]까지는 NULL이 아닌 pointer를 갖는다.
	// m_apElement[m_nSize]부터 m_apElement[m_nCapacity]까지는 NULL pointer를 갖는다.
	// 그 이후를 사용하려하면 m_apElement자체가 doubling된다.
	int m_nSize;
	int m_nCapacity;
};

//#include "stdafx.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

template <class T>
CTArray<T>::CTArray(int n)
{
	m_nSize=0;
	m_nCapacity=0;
	m_defaultParam=NULL;
	Init(n);
}

template <class T>
CTArray<T>::CTArray()
{
	m_nSize=0;
	m_nCapacity=0;
	m_defaultParam=NULL;
	m_factoryFunc=default_factory;
}

template <class T>
CTArray<T>::CTArray(T* (*factory_func)(void* param, int index))
{
	m_nSize=0;
	m_nCapacity=0;
	m_defaultParam=NULL;
	m_factoryFunc=factory_func;
}

template <class T>
CTArray<T>::~CTArray()
{
	Release();
}

template <class T>
int CTArray<T>::Size() const
{
	return m_nSize;
}

template <class T>
void CTArray<T>::Swap(int i, int j)
{
	T* temp;
	temp=m_apElement[i];
	m_apElement[i]=m_apElement[j];
	m_apElement[j]=temp;
}

template <class T>
void CTArray<T>::Replace(int i, T* pElement)
{
	ASSERT(i<m_nSize);
	ASSERT(pElement!=NULL);
	delete m_apElement[i];
	m_apElement[i]=pElement;
}

template <class T>
void CTArray<T>::Resize(int nsize, void* param)
{
	if(param==NULL) param=m_defaultParam;

	ASSERT(m_nSize<=m_nCapacity);
	if(nsize<=m_nSize)
	{
		for(int i=nsize; i<m_nSize; i++)
		{
			delete m_apElement[i];
			m_apElement[i]=NULL;
		}
		m_nSize=nsize;
	}
	else if(nsize<=m_nCapacity)
	{
		for(int i=m_nSize; i<nsize; i++)
		{
			m_apElement[i]=m_factoryFunc(param, i);
		}
		m_nSize=nsize;
	}
	else
	{
		ASSERT(nsize>m_nCapacity && nsize>m_nSize);
		if(m_nCapacity==0)
		{
			Init(nsize);
			return;
		}

		// m_nCapacity가 nsize를 포함할때까지 doubling
		for(;m_nCapacity<nsize;)	m_nCapacity*=2;

		T** apTempElement=new T*[m_nCapacity];
		for(int i=0; i<m_nSize; i++)
			// copy
			apTempElement[i]=m_apElement[i];
		for(int i=m_nSize; i<nsize; i++)
			apTempElement[i]=m_factoryFunc(param, i);
		for(int i=nsize; i<m_nCapacity; i++)
			apTempElement[i]=NULL;

		delete[] m_apElement;
		m_apElement=apTempElement;

		m_nSize=nsize;
	}
}

template <class T>
void CTArray<T>::Init(int nsize, void* param)
{
	if(param==NULL) param=m_defaultParam;
	Release();

	m_nSize=nsize;
	m_nCapacity=nsize;
	m_apElement=new T*[nsize];
	for(int i=0; i<m_nSize; i++)
	{
		m_apElement[i]=m_factoryFunc(param, i);
	}
}

template <class T>
void CTArray<T>::Release()
{
	if(m_nSize==0) return;

	for(int i=0; i<m_nSize; i++)
	{
		delete m_apElement[i];
	}
	delete[] m_apElement;

	m_nSize=0;
	m_nCapacity=0;
}

template <class T>
T& CTArray<T>::operator[](int nIndex) const
{
	ASSERT(nIndex>=0 && nIndex<Size());
	return *m_apElement[nIndex];
}

template <class T>
void CTArray<T>::Sort(int start, int end, int (*compareFunc)(const void** ppElement1, const void** ppElement2))
{
	qsort((void*)&(m_apElement[start]), end-start, sizeof(T*), (int (*)(const void*, const void*))compareFunc);
}

#endif
