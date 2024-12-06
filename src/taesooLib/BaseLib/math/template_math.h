#ifndef _TEMPLATE_MATH_H_
#define _TEMPLATE_MATH_H_

#if _MSC_VER>1000
#pragma once
#endif

///////////////////////////////////////////////
//
//  array of type T. type T should not have any virtual functions.
//  ex) int, m_real, quater, vector3 ...
//  vector3 can be seen as an array of m_real.
//  so vector3N inherits _tvectorn<vector3, m_real>.
// template class _tvectorn also does not have any virtual functions.
//

template <class T> class _tmat;
template <class T, class T_base> class _tvectorn;
template <class T, class T_base>
struct _tvec_setter
{
	_tvectorn<T, T_base>& _a;
	int _c;
	_tvec_setter(_tvectorn<T, T_base>& a):_a(a),_c(0){}
	_tvec_setter& operator,(T b){_a(_c)=b; _c=_c+1; return *this;}
};
template <class T, class T_base=T>
class _tvectorn
{
#ifdef _DEBUG
	// For convenient use of debugger.
	std::vector<T_base> _arrayDEBUG;	// valid only if owner==true in debug mode.
#endif
	T_base *ptr;
	int stride;	// space between two succesive elements in terms of sizeof(T_base)
	int     n,	on;	// size of memory allocated.
	bool owner;	// ptr is allocated by this, so will be freed by this.

	friend class _tmat<T_base>;

	T* getStride1Pointer() const;
protected:

	// disable default constructors (Do not use this class directly!)
	_tvectorn(T_base* ptrr, int size, int stride);	// reference
	_tvectorn(const _tvectorn<T,T_base>& other);
	_tvectorn();
	// L-value로 사용될수 있는, reference array로 만든다.
	void assignRef(const _tvectorn<T,T_base>& other);
	template <class vecViewType>
	vecViewType _range(int start, int end, int step)
    {
        RANGE_ASSERT(start>=0);
        RANGE_ASSERT(end<=n);
        int nSize;

        if (start > end)
		{
			RANGE_ASSERT(step<0);
            nSize=(end-start+ step+1)/step;
		}
        else
            nSize=(end-start+ step-1)/step;

        RANGE_ASSERT(nSize>=0);
        return vecViewType(ptr+start*stride, nSize, step*stride);
    }

	template <class matViewType>
	matViewType _column() const
	{
		// return n by 1 matrix, which can be used as L-value (reference matrix)
		matViewType c(ptr, size(), 1, stride);
		return c;
	}

	template <class matViewType>
	matViewType _row() const		// return 1 by n matrix, which can be used as L-value (reference matrix)
	{
		ASSERT(stride==1);
		matViewType c(ptr, 1, size(), size());
		return c;
	}

	/*
	template <class vecViewType>
	vecViewType _row(int i)const			{ assert(i>=0 && n>i); return vecViewType (&subValue(i,0), T_N, 1); }
	*/

	inline T_base& _subValue(int i, int j) const				{ RANGE_ASSERT(i>=0 && i<n); RANGE_ASSERT(j<stride); return *(ptr+i*stride+j);}

	template <class vecViewType>
	vecViewType _column(int i)const			{ return vecViewType (&_subValue(0,i), size(), stride); }

public:
	~_tvectorn();

	bool isReference() const	{ return !owner;}

	void _getPrivate(T_base*& ptr, int& stride, int& n, int& on) const;
	int _getStride() const	{ return stride;}
	T* dataPtr() const
	{
		ASSERT(stride==sizeof(T)/sizeof(T_base));
		return (T*)ptr;
	}

	inline T&	x() const	{ return value(0);};
	inline T&	y() const	{ return value(1);};
	inline T&	z() const	{ return value(2);};
	inline T&	w() const	{ return value(3);};


	T		getValue( int i ) const						{ RANGE_ASSERT(i>=0 && i<n); return value(i); }
	
	void	set( int i, T d )						{ RANGE_ASSERT(i>=0 && i<n); value(i) = d; }
	void	setValue( int i, T d )						{ RANGE_ASSERT(i>=0 && i<n); value(i) = d; }

	void	setAllValue(T d)							{ for(int i=0; i<n; i++) value(i)=d;}
	void	setAllValue(int n, T d)						{ setSize(n); for(int i=0; i<n; i++) value(i)=d;}
	// C-style setValues (non type-safe, slightly more effiencient)
	void	setValues(int n, T x,...);					//!< setValues(3,4.3, 2.3, 4.3);


	inline _tvec_setter<T, T_base> setValues(int n) { setSize(n); return _tvec_setter<T,T_base>(*this);}  //!< setValues(3), 0,0.1,1.1;
	inline _tvec_setter<T, T_base> setValues() { return _tvec_setter<T,T_base>(*this);}  //!< setValues(), 0,0.1,1.1;
	void	setValues(T const* values);
	void	clear(int start, int end);

	void	extract(_tvectorn<T,T_base> const& in, _tvectorn<int> const & columns)	;

	void	assignSelective(_tvectorn<int> const & columns, _tvectorn<T,T_base> const& in);
	void	assignSelective(_tvectorn<int> const & columns, T in);
	// eg. assignSelective(3, index1, value1, index2, value2, index3, value3, ...)  where value is of type T
	void	assignSelective(int n, ...);


	// reference가 value로 카피된다.
	_tvectorn<T,T_base>& assign(const _tvectorn<T,T_base>& other);

	// a.value(0)==a[0]==a(0)
	inline T&   value(int i) const							{ RANGE_ASSERT(i>=0 && i<n); return *((T*)(ptr+i*stride));}
	// value + error checking + python-like backward indexing. default in LUA.
	inline T&   at(int i) const								{ 
		if(i<0) i=size()+i;
		RANGE_ASSERT(i>=0 && i<n); return *((T*)(ptr+i*stride));
	}
	inline T&   operator[](int i) const						{ return value(i); }
	inline T&   operator()(int i) const						{ return value(i);}

	int  size() const										{ return n; }
	void setSize( int );		// preserves data iff size decreases or doesn't change.
	void resize(int nsize);		// always preserves data, and fills empty entries by 0.
	void reserve( int rr)	{ if(rr>on) { int prev_n=n; resize(rr); n=prev_n; }}
	void swap(int i, int j)								{ T temp=(*this)[i]; (*this)[i]=(*this)[j]; (*this)[j]=temp;};

	void pushBack(T x) { resize(size()+1); setValue(size()-1,x);}
	void pushFront(T x);

	T popFront();
	T popBack();
	T& back(int i=0) const	{ return value(size()-1+i);}

	// end이하는 end-start만큼 당겨진다. 즉 vector크기가 end-start만큼 작아진다.
	void bubbleOut(int start, int end);
	// nrow이하는 nbubble만큼 우측으로 옮겨진다. 즉 vector크기가 nbubble만큼 커지고, 빈칸이 생긴다.
	void bubbles(int nrow, int nbubbles);

	//////////////////////////////////////////////////////////////////////
	// binary operations.
	//////////////////////////////////////////////////////////////////////
	// e.g > c.add(a,b) , c.cross(a,b), c=a.distance(b)..
	// (unary operation으로도 사용가능 ex) c.add(c,b);m

	_tvectorn<T,T_base>& add( _tvectorn<T,T_base> const&, _tvectorn<T,T_base> const& );
	_tvectorn<T,T_base>& add( T a, _tvectorn<T,T_base> const& b);
	_tvectorn<T,T_base>& add( _tvectorn<T,T_base> const& a, T b);
	_tvectorn<T,T_base>& sub( _tvectorn<T,T_base> const&, _tvectorn<T,T_base> const& );
	_tvectorn<T,T_base>& sub( T a, _tvectorn<T,T_base> const& b);
	_tvectorn<T,T_base>& sub( _tvectorn<T,T_base> const& a, T b);
	_tvectorn<T,T_base>& mult( _tvectorn<T,T_base> const&, T );
	_tvectorn<T,T_base>& mult( _tvectorn<T,T_base> const& a, _tvectorn<T,T_base> const& b);
	_tvectorn<T,T_base>& div( _tvectorn<T,T_base> const& a, T b)				{ return mult(a, 1/b);}
	_tvectorn<T,T_base>& div( _tvectorn<T,T_base> const& a, _tvectorn<T,T_base> const& b);
	_tvectorn<T,T_base>& multAdd(const _tvectorn<T,T_base>& a, const _tvectorn<T,T_base>& b) ;
	_tvectorn<T,T_base>& multAdd(const _tvectorn<T,T_base>& a, T b) ;
	_tvectorn<T,T_base>& concat(_tvectorn<T, T_base> const& a, _tvectorn<T, T_base> const& b);

	void multmat( _tmat<T> const&, _tvectorn<T, T_base> const& );
	void multmat( _tvectorn<T, T_base> const&, _tmat<T> const& );

	//////////////////////////////////////////////////////////////////////
	// unary operations
	//
	// - binary operation이 unary로 사용가능하기에, 많이 생략되었음.
	//     ex) a.add(a,b)
	//////////////////////////////////////////////////////////////////////
	_tvectorn<T, T_base>& operator+=( _tvectorn<T, T_base> const& other);
	_tvectorn<T, T_base>& operator+=( T value )	;
	_tvectorn<T, T_base>& operator-=( _tvectorn<T, T_base> const& other);
	_tvectorn<T, T_base>& operator-=( T value );
	_tvectorn<T, T_base>& operator*=( _tvectorn<T, T_base> const& other);
	_tvectorn<T, T_base>& operator*=( T value );
	_tvectorn<T, T_base>& operator/=( T value );

	int argMin() const;
	int argMax() const;
	int argNearest(T value) const;

	/// deprecated
	//void	  getValue(int start, int end, _tvectorn<T, T_base, T_N>& value);
	//void	  setValue( int start, int end, T d)			{ for(int i=start; i<end; i++) value(i)=d;}
	//void	  setValue( int start, int end, _tvectorn<T, T_base, T_N> const& value);	//!< (*this)[start]=value[0]..
	//void	  setValuePtr( int start, int end, T* v);	//!< (*this)[start]=value[0]..
};

template <class T, class T_base>
_tvectorn<T, T_base>::_tvectorn()
{
	on = n = 0;
	stride = sizeof(T)/sizeof(T_base);
	owner=true;
}

template <class T, class T_base>
_tvectorn<T, T_base>::_tvectorn(T_base* ptrr, int size, int str)
{
	on=0;
	ptr=ptrr;
	n=size;
	stride=str;
	owner=false;
}

template <class T, class T_base>
_tvectorn<T, T_base>::_tvectorn(const _tvectorn<T, T_base>& other)
{
	on = n = 0;
	stride = sizeof(T)/sizeof(T_base);
	owner=true;

	assign(other);
}

template <class T, class T_base>
void _tvectorn<T, T_base>::_getPrivate(T_base*& ptr2, int& stride2, int& n2, int& on2) const
{
	ptr2=ptr;
	stride2=stride;
	n2=n;
	on2=on;
}


template <class T, class T_base>
_tvectorn<T, T_base>::~_tvectorn()
{
	//if ( owner && on>0 ) delete[] ptr;
#ifndef _DEBUG
	if ( owner && on>0 ) free(ptr);
#endif
}



template <class T, class T_base>
void _tvectorn<T, T_base>::setSize( int x )
{
	if(n==x)
		return;

	if(!owner)
		Msg::error("setSize called, but not an owner");

	int T_N=sizeof(T)/sizeof(T_base);

#ifdef _DEBUG
	if(stride!=T_N)
		Msg::error("setSize called, but stride!=1");
#endif

	if ( on<x )
	{
		// 아래 assert가 fail이 날수 있는 경우는 on==0인데 n!=x인 경우이다.
		ASSERT(on>=n);

#ifdef _DEBUG
		_arrayDEBUG.reserve(x*T_N);
		_arrayDEBUG.resize(x*T_N);
		ptr= &_arrayDEBUG[0];
#else
		//if ( on>0 ) delete[] ptr;
		if ( on>0 ) free(ptr);
		//ptr = new T_base [x*T_N];
		ptr= (T_base*)malloc(size_t(sizeof(T_base)*x*T_N));
		Msg::verify(ptr!=NULL, "malloc (size %d) failed", sizeof(T_base)*x*T_N );
#endif
		on = x;
	}
#ifdef _DEBUG
	_arrayDEBUG.resize(x*T_N);
	if(x>0)
		ptr= &_arrayDEBUG[0];
#endif
	n = x;
}

template <class T, class T_base>
void _tvectorn<T, T_base>::clear(int start, int end)
{
	for(int i=start; i<end; i++)
		value(i)=baselib::traits<T, T_base>::zero();
}

template <class T, class T_base>
void _tvectorn<T, T_base>::resize(int nsize)
{
	RANGE_ASSERT(nsize>=0);
	if(n==nsize)
		return;

	if(!owner)
		Msg::error("resize called, but not an owner");

	int T_N=sizeof(T)/sizeof(T_base);

#ifdef _DEBUG
	if(stride!=T_N)
		Msg::error("setSize called, but stride!=1");
#endif

	if(nsize<=n)
		setSize(nsize);
	else if(nsize<=on)
	{
		int prev_size=n;
		setSize(nsize);
		clear(prev_size, nsize);
	}
	else
	{
		int capacity=MAX(on,10);
		// capacity가 nsize를 포함할때까지 doubling
		while(capacity<nsize)	capacity*=2;


		/* previous implementation is based on data copy.
		int prev_size=n;
		_tvectorn<T, T_base, T_N> backup;
		backup.assign(*this);
		setSize(capacity);
		setSize(nsize);
		for(int i=0; i<prev_size; i++) value(i)=backup(i);
		clear(prev_size, nsize);*/

#ifdef _DEBUG
		_arrayDEBUG.reserve(capacity*T_N);
		_arrayDEBUG.resize(nsize*T_N);
		ptr=&_arrayDEBUG[0];
#else
		// new implementation based on realloc
		if(on==0)
			ptr=(T_base*)malloc(sizeof(T_base)*capacity*T_N);
		else
			ptr=(T_base*)realloc(ptr, sizeof(T_base)*capacity*T_N);
		Msg::verify(ptr!=NULL, "realloc (size %d) failed", sizeof(T_base)*capacity*T_N );
#endif
		on=capacity;
		int prevSize=n;
		n=nsize;
		clear(prevSize, nsize);

	}
}

template <class T, class T_base>
void _tvectorn<T, T_base>::setValues( int n, T x, ... )
{
	va_list marker;
	va_start( marker, x);     /* Initialize variable arguments. */

	setSize(n);
	setValue(0, x);
	for(int i=1; i<n; i++)
	{
		setValue(i, va_arg( marker, T));
	}
	va_end( marker );              /* Reset variable arguments.      */
}

template <class T, class T_base>
void _tvectorn<T, T_base>::setValues(T const* values)
{
	for(int i=0; i<size(); i++)
		value(i)=values[i];
}


template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::assign(const _tvectorn<T, T_base>& other)
{
	setSize(other.size());
	for( int i=0; i<n; i++ )
		value(i) = other[i];
	return *this;
}

template <class T, class T_base>
void _tvectorn<T, T_base>::assignRef(const _tvectorn<T, T_base>& other)
{
	other._getPrivate(ptr, stride, n, on);
	on=0;
	owner=false;
}

template <class T, class T_base>
void _tvectorn<T, T_base>::pushFront(T x)
{
	resize(size()+1);
	for(int i=size()-1; i>0; i--)
		value(i)=value(i-1);
	value(0)=x;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::mult( _tvectorn<T, T_base> const& b, T a )
{
	_tvectorn<T, T_base> &c = (*this);
	c.setSize( b.size() );

	for( int i=0; i<c.size(); i++ )
		c[i] = b[i]*a;
	return c;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::mult( _tvectorn<T, T_base> const& a, _tvectorn<T, T_base> const& b)
{
	_tvectorn<T, T_base> &c = (*this);
	c.setSize( b.size() );

	for( int i=0; i<c.size(); i++ )
		c[i] = a[i]*b[i];
	return c;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::div( _tvectorn<T, T_base> const& a, _tvectorn<T, T_base> const& b)
{
	_tvectorn<T, T_base> &c = (*this);
	c.setSize( b.size() );

	for( int i=0; i<c.size(); i++ )
		c[i] = a[i]/b[i];
	return c;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::add(_tvectorn<T, T_base> const& a, _tvectorn<T, T_base> const& b)
{
	_tvectorn<T, T_base> &c = (*this);
	c.setSize( b.size() );

	for( int i=0; i<c.size(); i++ )
		c[i] = a[i]+b[i];
	return c;
}
template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::sub(_tvectorn<T, T_base> const& a, _tvectorn<T, T_base> const& b)
{
	_tvectorn<T, T_base> &c = (*this);
	c.setSize( b.size() );

	for( int i=0; i<c.size(); i++ )
		c[i] = a[i]-b[i];
	return c;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::concat(_tvectorn<T, T_base> const& a, _tvectorn<T, T_base> const& b)
{
	if(&a==this)
	{
		_tvectorn<T, T_base> aa(a);
		return concat(aa,b);
	}

	if(&b==this)
	{
		_tvectorn<T, T_base> bb(b);
		return concat(a,bb);
	}

	_tvectorn<T, T_base> &c = (*this);
	c.setSize(a.size()+b.size());

	for(int i=0; i<a.size(); i++)
		c[i]=a[i];

	for(int i=0; i<b.size(); i++)
		c[i+a.size()]=b[i];

	return c;
}

// macro functions

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::add( _tvectorn<T, T_base> const& a, T b)
{
	_tvectorn<T, T_base>& c=*this;
	c.setSize( a.size() );

	for( int i=0; i<a.size(); i++ )
		c[i] = a[i] + b;
	return c;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::add( T a, _tvectorn<T, T_base> const& b)
{
	return add(b, a);
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::sub( T a, _tvectorn<T, T_base> const& b)
{
	_tvectorn<T, T_base>& c=*this;
	c.setSize( b.size() );

	for( int i=0; i<b.size(); i++ )
		c[i] = a - b[i];
	return c;
}
template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::sub( _tvectorn<T, T_base> const& a, T b)
{
	_tvectorn<T, T_base>& c=*this;
	c.setSize( a.size() );

	for( int i=0; i<a.size(); i++ )
		c[i] = a[i] - b;
	return c;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::multAdd(const _tvectorn<T, T_base>& a, const _tvectorn<T, T_base>& b)
{
	_tvectorn<T, T_base>& c=*this;
	RANGE_ASSERT(c.size()==a.size() && c.size()==b.size());

	for(int i=0; i<c.size(); i++)
	{
		c[i]+=a[i]*b[i];
	}
	return c;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::multAdd(const _tvectorn<T, T_base>& a, T b)
{
	_tvectorn<T, T_base>& c=*this;
	RANGE_ASSERT(c.size()==a.size() );

	for(int i=0; i<c.size(); i++)
	{
		c[i]+=a[i]*b;
	}
	return c;
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::operator+=( _tvectorn<T, T_base> const& other)
{
	return add(*this, other);
}

template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::operator+=( T value )
{
	return add(*this, value);
}
template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::operator-=( _tvectorn<T, T_base> const& other)
{
	return sub(*this, other);
}
template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::operator-=( T value )
{
	return sub(*this, value);
}
template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::operator*=( _tvectorn<T, T_base> const& other)
{
	return mult(*this, other);
}
// component wise multiplication
template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::operator*=( T value )
{
	return mult(*this, value);
}
template <class T, class T_base>
_tvectorn<T, T_base>& _tvectorn<T, T_base>::operator/=( T value )
{
	return mult(*this, 1.0/value);
}

template <class T, class T_base>
T _tvectorn<T, T_base>::popBack()
{
	T out=(*this)[size()-1];
	resize(size()-1);
	return out;
}

template <class T, class T_base>
T _tvectorn<T, T_base>::popFront()
{
	T out=(*this)[0];

	for(int i=0; i<size()-1; i++)
	{
		(*this)[i]=(*this)[i+1];
	}
	resize(size()-1);
	return out;
}

template <class T, class T_base>
void _tvectorn<T, T_base>::bubbleOut(int start, int end)
{
	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다.
	int numCols=end-start;

	for(int i=end; i<size(); i++)
	{
		(*this)[i-numCols]=(*this)[i];
	}

	resize(size()-numCols);
}

template <class T, class T_base>
void _tvectorn<T, T_base>::bubbles(int nrow, int nbubbles)
{
	// nrow이하는 nbubble만큼 아래로 내린다. 즉 matrix크기가 nbubble만큼 세로로 커지고, 빈칸이 생긴다.
	int prev_row=size();
	resize(size()+nbubbles);

	for(int i=prev_row-1; i>=nrow; i--)
			(*this)[i+nbubbles]=(*this)[i];

	for(int i=nrow; i<nrow+nbubbles; i++)
			(*this)[i]=0;
}

template <class T, class T_base>
void  _tvectorn<T, T_base>::extract(_tvectorn<T, T_base> const& in, _tvectorn<int> const& columns)
{
	setSize(columns.size());

	for(int j=0; j<columns.size(); j++)
	{
		(*this)[j]=in[columns[j]];
	}
}

template <class T, class T_base>
void  _tvectorn<T, T_base>::assignSelective(_tvectorn<int> const & columns, _tvectorn<T, T_base> const& in )
{
	for(int j=0; j<columns.size(); j++)
		(*this)[columns[j]]=in[j];
}


template <class T, class T_base>
void  _tvectorn<T, T_base>::assignSelective(_tvectorn<int> const & columns, T in )
{
	for(int j=0; j<columns.size(); j++)
		(*this)[columns[j]]=in;
}

template <class T, class T_base>
void _tvectorn<T, T_base>::assignSelective(int N, ...)
{
	va_list marker;
	va_start( marker, N);     /* Initialize variable arguments. */

	int i;
	for(i=0; i<N; i++)
	{
		int index=va_arg(marker, int);
		value(index)=va_arg(marker, T);
	}
	va_end(marker);

}

template <class T, class T_base>
T* _tvectorn<T, T_base>::getStride1Pointer() const
{
	// 한 벡터가 여러번 반복해서 읽어지는 경우, stride가 1이 아닌경우 1인 놈에 카피해서 읽는것이 더 효율적임.
	T* bb;
	if(stride==1)
		bb=ptr;
	else
	{
		static _tvectorn<T, T_base> copy;
		copy.setSize(size());
		bb=copy.ptr;
		for(int i=0; i<size(); i++)
			bb[i]=value(i);
	}
	return bb;
}

template <class T, class T_base>
void _tvectorn<T, T_base>::multmat( _tmat<T> const& a, _tvectorn<T, T_base> const& b )
{
    _tvectorn<T, T_base> &c = (*this);
    RANGE_ASSERT( a.cols()==b.size() );
    c.setSize( a.rows() );

	T* bb=b.getStride1Pointer() ;
	for( int i=0; i<a.rows(); i++ )
    {
        T c_i= 0.0;
		T* a_i=a[i];
        for( int k=0; k<b.size(); k++ )
            c_i+= a_i[k] * bb[k];
		c[i] =c_i;
    }
}

template <class T, class T_base>
void _tvectorn<T, T_base>::multmat( _tvectorn<T, T_base> const& b, _tmat<T> const& a )
{
    _tvectorn<T, T_base> &c = (*this);
    RANGE_ASSERT( a.rows()==b.size() );
    c.setSize( a.cols() );


	T* bb=b.getStride1Pointer() ;
    for( int i=0; i<a.cols(); i++ )
    {
        T c_i= 0.0;
        for( int k=0; k<b.size(); k++ )
             c_i+= bb[k] * a[k][i];
		c[i]=c_i;
    }
}

template <class T, class T_base>
int _tvectorn<T,T_base>::argMin() const
{
	T min_v=std::numeric_limits<T>::max();

	int min_index=0;

	for(int i=0; i<size(); i++)
	{
		T v=value(i);
		if(v<min_v)
		{
			min_v=v;

			min_index=i;
		}
	}
	return min_index;
}

template <class T, class T_base>
int _tvectorn<T,T_base>::argMax() const
{
	T max_v=-1*std::numeric_limits<T>::max();
	int max_index;

	for(int i=0; i<size(); i++)
	{
		T v=value(i);
		if(v>max_v)
		{
			max_v=v;
			max_index=i;
		}
	}
	return max_index;
}

template <class T, class T_base>
int _tvectorn<T,T_base>::argNearest(T v) const
{
	int argNearest=-1;
	T_base min=std::numeric_limits<T_base>::max();

	T_base dist;
	for(int i=0; i<size(); i++)
	{
		if((dist=baselib::traits<T,T_base>::distance(value(i), v))< min)
		{
			min=dist;
			argNearest=i;
		}
	}
	return argNearest;
}

// deprecated
/*
template <class T, class T_base>
void _tvectorn<T, T_base>::setValue( int start, int end, _tvectorn<T, T_base, T_N> const& vec)
{
	for(int i=start; i<end; i++)
	{
		value(i)=vec.value(i-start);
	}
}

template <class T, class T_base>
void _tvectorn<T, T_base, T_N>::setValuePtr( int start, int end, T* values)
{
	for(int i=start; i<end; i++)
	{
		value(i)=values[i-start];
	}
}

template <class T, class T_base>
void _tvectorn<T, T_base, T_N>::getValue(int start, int end, _tvectorn<T, T_base, T_N>& vv)
{
	vv.setSize(end-start);

	for(int i=start; i<end; i++)
	{
		vv[i-start]=value(i);
	}
}

*/

#endif
