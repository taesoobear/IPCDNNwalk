#ifndef _TEMPLATE_MATRIX_H_
#define _TEMPLATE_MATRIX_H_

#if _MSC_VER>1000
#pragma once
#endif

#ifdef _DEBUG
//#define _DEBUG_INFO	-> 버그 있음.
#endif

#include "template_math.h"
template <class T>
struct _tmat_setter
{
	_tmat<T>& _a;
	int _crow,_ccol;
	_tmat_setter(_tmat<T>& a):_a(a),_crow(0),_ccol(0){}
	_tmat_setter& operator,(T b){_a(_crow,_ccol)=b; _crow+=(_ccol +1)/_a.cols(); _ccol=(_ccol+1)%_a.cols();return *this;}
};
template <class T>
class _tmat
{
protected:

#ifdef _DEBUG_INFO
	// For convenient use of debugger.
	std::vector<T> _arrayDEBUG;	// valid only if owner==true in debug mode.
#endif

	int      n,	m,
			stride,	// stride between each row.
			on;	// size of memory allocated( which is size of buffer). Not the number of row. (by taesoo)
	T* buffer;

	friend class _tvectorn<T, T>;
protected:

	// disable default copy constructor
	_tmat(const _tmat<T>& other) { ASSERT(0);}
	_tmat();
	_tmat(T* _ptr, int _n, int _m, int _stride) { buffer=_ptr; n=_n; m=_m; stride=_stride; on=-1;	} // reference

	// reference로 받아온다.
	void _assignRef( _tmat<T> const&);

	template <class matViewType>
	matViewType _range(int startRow, int endRow, int startColumn, int endColumn)
	{
		RANGE_ASSERT(startRow>=0 && endRow<=rows() && startColumn<cols());
		if(endColumn>cols()) endColumn=cols();
		return matViewType(((buffer+startRow*stride)+startColumn), endRow-startRow, endColumn-startColumn, stride);
	}

	template <class matViewType>
	const matViewType _range(int startRow, int endRow, int startColumn, int endColumn) const
	{
		RANGE_ASSERT(startRow>=0 && endRow<=rows() && startColumn<cols());
		if(endColumn>cols()) endColumn=cols();
		return matViewType(((buffer+startRow*stride)+startColumn), endRow-startRow, endColumn-startColumn, stride);
	}

	// L-value로 사용될수 있는 reference vector 를 return한다.

	template <class vecViewType>
	vecViewType _row(int i)const			{ RANGE_ASSERT(i>=0 && n>i); return vecViewType (&value(i,0), cols(), 1); }
	template <class vecViewType>
	vecViewType _column(int i)const			{ RANGE_ASSERT(i>=0 && m>i); return vecViewType (&buffer[i], rows(), stride); }
	template <class vecViewType>
	vecViewType _diag()const			{ int k=MIN(n,m); return vecViewType (&value(0,0), k, stride+1); }

public:
	virtual ~_tmat();

	// use BinaryFile instead
	//void load(const char* filename);
	//void save(const char* filename) const;

	bool	isReference() const				{ return on==-1;}
	// inquiry functions
	T    getValue( int, int ) const;

	int	rows()  const			{ return n; }
	int	nrows()  const			{ return n; }
	int	cols() const			{ return m; }
	int	ncols() const			{ return m; }

	void	setDiagonal(T value);
	void	setDiagonal(const _tvectorn<T>& );

	// set value
	void    setValue( int, int, T );
	// C-style setValue. (non type-safe, slightly more efficient)
	void	setValues(int m, int n, T x00, ...);					//!< setValues(3,1, 0.0, 0.1, 1.1);

	// C++ style setValue 
	inline _tmat_setter<T> setValues(int m, int n) { setSize(m,n); return _tmat_setter<T>(*this);}  //!< setValues(3,1), 0,0.1,1.1;
	inline _tmat_setter<T> setValues() { return _tmat_setter<T>(*this);}  //!< setValues(), 0,0.1,1.1;

	void    set( int, int, T );
	void	setAllValue(T);
	void	setAllValue(int nrow, int ncol, T value)			{ setSize(nrow, ncol); setAllValue(value);}


	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다.
	void deleteRows(int start, int end);
	// end이하는 end-start만큼 왼쪽으로 당겨진다. 즉 matrix크기가 end-start만큼 가로로 작아진다.
	void deleteCols(int start, int end);
	// nrow이하는 nbubble만큼 아래로 내린다. 즉 matrix크기가 nbubble만큼 세로로 커지고, 빈칸이 생긴다.
	void bubbles(int nrow, int nbubbles);
	// ncolumn우측은 nbubble만큼 우측으로 민다. 즉 matrix크기가 nbubble만큼 가로로 커지고, 빈칸이 생긴다.
	void bubbleColumns(int ncol, int nbubbles);

	void	  pushFront(const _tvectorn<T>& rowVec);	//!< slow operation, hypermatrix의 원소를 pushFront하면 문제가 생길수 있음
	void	  popFront();
	void	  popFront(_tvectorn<T>& rowVec);		//!< slow operation, hypermatrix의 원소를 popFront하면 문제가 생길수 있음

	void	  pushBack(const _tvectorn<T>& rowVec);
	void	  popBack();
	void	  popBack(_tvectorn<T>& rowVec);

	void extractRows(_tmat<T> const& mat, _tvectorn<int> const& rows);
	void extractColumns(_tmat<T> const& mat, _tvectorn<int> const& columns);
	void assignColumns(_tmat<T> const& mat, _tvectorn<int> const& columns);
	void assignRows(_tmat<T> const& mat, _tvectorn<int> const& columns);

#ifdef _DEBUG
	template <class TT> class RangeCheck
	{
	public:
		RangeCheck(TT* p, int col):mPtr(p), mCol(col){}
		TT& operator[](int i) const	{ ASSERT(i>=0 && i<mCol); return *(mPtr+i);}
		inline operator TT*() const	{ return mPtr;}
		TT* mPtr;
		int mCol;
	};
	RangeCheck<T> operator[](int i) const		{ ASSERT(i<rows() ); return RangeCheck<T>((T*)(buffer+i*stride), cols());}
#else
	T*  operator[](int i) const					{ RANGE_ASSERT(i<rows() ); return (T*)(buffer+i*stride);}
#endif
	inline T&  value(int i, int j) const		{ ASSERT(i<rows() && j<cols()); return *((buffer+i*stride)+j);}
	// value + error checking + python-like backward indexing. default in LUA.
	inline T&  at(int i, int j) const		{ 
		if(i<0) i=rows()+i;
		if(j<0) j=cols()+j;
		RANGE_ASSERT(i<rows() && j<cols()); return *((buffer+i*stride)+j);
	}
	inline T&	 operator()(int i, int j) const			{ return value(i,j);}


	void	setSize( int, int );  // 원래 데이타 유지 보장 전혀 없음.
	void    setSameSize( const _tmat<T>& other)	{ setSize(other.rows(), other.cols());};  // 원래 데이타 유지 보장 전혀 없음.
	void    resize( int, int );	// 빈자리는 0으로 채움 , 원래 데이타 유지.
	void _getPrivate(T*& buffer2, int & stride2, int& n2, int& m2, int& on2) const;
	inline int _getStride() const { return stride;}
	inline T* _data() const { return buffer;}

	// reference가 value로 카피된다. reference를 받아오고 싶으면, assignRef를 사용할 것.
	_tmat<T>&  assign( _tmat<T> const&);


	// unary operations
	void transpose( _tmat<T> const& );
    void operator+=( _tmat<T> const& );
    void operator-=( _tmat<T> const& );
	void operator*=( T );
	void operator/=( T );
	void concatRow( _tmat<T> const&);

	// binary operations
	void mult( _tmat<T> const&, _tmat<T> const& );
	void multABt(_tmat<T> const& a, _tmat<T> const& b); //!< a*b^T
	void multAtB(_tmat<T> const& a, _tmat<T> const& b); //!< a^T*b
	void multAtBt(_tmat<T> const& a, _tmat<T> const& b); //!< a^T*b^T
	void mult( _tmat<T> const& a, T b );
	void add( _tmat<T> const&, _tmat<T> const& );
	void add( _tmat<T> const&, T);
	void add( T, _tmat<T> const&);
	void subtract( _tmat<T> const&, _tmat<T> const& );
	void subtract( _tmat<T> const&, T);
	void subtract( T, _tmat<T> const&);

	void concatRow( _tmat<T> const&, _tmat<T> const&);
	void concatColumn( _tmat<T> const&, _tmat<T> const&);


	// 아래는 모두 deprecated
	// row(i), col(i), range(rowstart, rowend, columnstart, columnend) 등을 사용할 것.
	void      getRow( int, _tvectorn<T>& ) const;
	void	  getColumn( int, _tvectorn<T>& ) const;

	void	  setRow(int i, T value);
	void      setRow( int i, const _tvectorn<T>& );
	void	  setColumn(int j, T value);
	void	  setColumn( int j, const _tvectorn<T>& );

	void	  setValue(int rowstart, int rowend, int columnstart, int columnend, T);
	void	  setValue(int row, int column, _tmat<T> const& mat);	// this의 row,column부터 row+mat.rows(), column+mat.cols()까지 채움

};


template <class T>
_tmat<T>::_tmat()
{
	on = n = m= stride=0;
}

template <class T>
_tmat<T>::~_tmat()
{
	//if( on>0) delete[] buffer;
#ifndef _DEBUG_INFO
	if( on>0) free(buffer);
#endif
}

template <class T>
T _tmat<T>::getValue( int row, int column ) const
{
	const _tmat<T>& v=*this;
	return v[row][column];
}

template <class T>
void _tmat<T>::setValues( int m, int n, T x, ... )
{
	va_list marker;
	va_start( marker, x);     /* Initialize variable arguments. */

	setSize(m,n);
	setValue(0, 0,x);
	int i=0;
	int j=1;

	for(int j=1, nj=n; j<nj; j++)
		setValue(i,j,va_arg( marker, T));

	if(m>1)
	{
		for(int i=1, ni=m; i<ni; i++)
		{
			for(int j=0, nj=n; j<nj; j++)
			{
				setValue(i,j,va_arg( marker, T));
			}
		}
	}

	va_end( marker );              /* Reset variable arguments.      */
}

template <class T>
void _tmat<T>::setValue(int rowstart, int rowend, int columnstart, int columnend, T d)
{
	_tmat<T>& v=*this;
	for(int i=rowstart; i<rowend; i++)
	{
		T* v_i=v[i];
		for(int j=columnstart; j<columnend; j++)
			v_i[j]=d;
	}
}

template <class T>
void _tmat<T>::set(int i,int j, T value)
{
	(*this)[i][j]=value;
}

template <class T>
void _tmat<T>::setAllValue(T value)
{
	_tmat<T>& v=*this;
	for(int i=0; i<rows(); i++)
	{
		T* v_i=v[i];
		for(int j=0; j<cols(); j++)
			v_i[j]=value;
	}
}



template <class T>
void _tmat<T>::setValue( int row, int column, T value )
{
	_tmat<T>& v=*this;
	v[row][column]=value;
}

template <class T>
void _tmat<T>::setSize( int nRow, int nColumn )
{
	if(n==nRow && m==nColumn) return;

	if(isReference())
		Msg::error("setSize called, but not an owner");

	if(on<nRow*nColumn)	// buffer가 모자랄때만 다시 생성
	{
		int capacity;
		if(on)
		{
#ifndef _DEBUG_INFO
			free(buffer);
#endif
			capacity=on;
			// capacity가 nsize를 포함할때까지 doubling
			while(capacity<nRow*nColumn)	capacity*=2;
		}
		else
			capacity=nRow*nColumn;

		// 한번에 크게 할당한다. 기존의 모든 벡터마다 loop를 돌면서 할당하는 방법은 대략 좋지않다.
		// 이유는 new operator는 상당히 시간이 오래걸리는 연산이기 때문이다. (delete도 마찬가지)
		//buffer=new T [nRow*nColumn];
#ifdef _DEBUG_INFO
		_arrayDEBUG.reserve(capacity);
		_arrayDEBUG.resize(nRow*nColumn);
		buffer=&_arrayDEBUG[0];
#else
		buffer=(T*)malloc(sizeof(T)*capacity);
		Msg::verify(buffer!=NULL, "malloc (size %d) failed", sizeof(T)*capacity );
#endif

		on=capacity;
	}
#ifdef _DEBUG_INFO
	_arrayDEBUG.resize(nRow*nColumn);
#endif

	n = nRow;
	m = nColumn;
	stride=nColumn;
}


template <class T>
void _tmat<T>::resize(int nRow, int nColumn)
{
	if(n==nRow && m==nColumn) return;

	if(isReference())
		Msg::error("setSize called, but not an owner");

	if(nColumn==cols())
	{
		if(nRow*stride<on)
		{
			int prev_row=rows();
			n = nRow;
			setValue(prev_row, nRow, 0, cols(), 0);
			return;
		}
		else if(stride==cols())
		{
			// need to reallocate memory
			// buffer를 넉넉하게 잡는다. (잦은 재할당을 막는다.)
			int capacity=MAX(on,50);
			// capacity가 nsize를 포함할때까지 doubling
			while(capacity<nRow*nColumn)	capacity*=2;

#ifdef _DEBUG_INFO
			_arrayDEBUG.reserve(capacity);
			_arrayDEBUG.resize(nRow*nColumn);
			buffer=&_arrayDEBUG[0];
#else
			// realloc is possible
			if(on)
				buffer=(T*)realloc(buffer, sizeof(T)*capacity);
			else
				buffer=(T*)malloc(sizeof(T)*capacity);

			Msg::verify(buffer!=NULL, "malloc (size %d) failed", sizeof(T)*capacity );
#endif

			int prevRow=n;
			n=nRow;
			on=capacity;

			for(int i=prevRow; i<rows(); i++)
			{
				T* v_i=(*this)[i];
				for(int j=0; j<cols(); j++)
					v_i[j]=0;
			}

			return;
		}
	}
	else if(nRow*stride<on)
	{
		if(nColumn<=stride)
		{
			int prev_row=rows();
			int prev_col=cols();
			n = nRow;
			m = nColumn;
			setValue(prev_row, nRow, 0, cols(), 0);
			setValue(0, rows(), prev_col, nColumn, 0);
			return;
		}
		else if(nRow*nColumn<on)
		{
			ASSERT(nColumn>stride);
			// enough capacity
			_tmat<T> backup;
			backup._assignRef(*this);	// get reference (which is valid only when the data is not overwritten)

			// backward copy is safe (데이타가 항상 오른쪽으로 밀리니까 뒤에서부터 카피한다.).
			int minRow=MIN(nRow, backup.rows());
			int minColumn=MIN(nColumn, backup.cols());
			n = nRow;
			m = nColumn;
			stride=nColumn;
			for(int i=minRow-1; i>=0; i--)
			{
				T* v_i=(*this)[i];
				T* backup_i=backup[i];
				for(int j=minColumn-1; j>=0; j--)
					v_i[j]=backup_i[j];
			}

			setValue(minRow, nRow, 0, cols(), 0);
			setValue(0, rows(), minColumn, cols(), 0);
			return;
		}
	}

	// default: backup, malloc and copy!

	// buffer를 넉넉하게 잡는다. (잦은 재할당을 막는다.)
	int capacity=MAX(on,50);
	// capacity가 nsize를 포함할때까지 doubling
	while(capacity<nRow*nColumn)	capacity*=2;

	// data copy is needed since shape is modified.
#ifdef _DEBUG_INFO
	_tmat<T> backup;
	backup.assign(*this);
	_arrayDEBUG.reserve(capacity);
	_arrayDEBUG.resize(nRow*nColumn);
	buffer=&_arrayDEBUG[0];
	on=capacity;
	setSize(nRow, nColumn);
#else
	_tmat<T> backup;

	T* bufferBackup=(on)? buffer : NULL;

	backup._assignRef(*this);	// get reference (which is still valid until bufferBackup is freed.)

	// 한번에 크게 할당한다. 기존의 모든 벡터마다 loop를 돌면서 할당하는 방법은 대략 좋지않다.
	// 이유는 new operator는 상당히 시간이 오래걸리는 연산이기 때문이다. (delete도 마찬가지)
	//buffer=new T[capacity];

	buffer=(T*)malloc(sizeof(T)*capacity);
	Msg::verify(buffer!=NULL, "malloc (size %d) failed", sizeof(T)*capacity );
	on=capacity;

	setSize(nRow, nColumn);
#endif

	int backupRow=MIN(backup.rows(), nRow);
	int backupColumn=MIN(backup.cols(), nColumn);

	this->setAllValue(0);
	for(int i=0; i<backupRow; i++)
	{
		T* v_i=(*this)[i];
		for(int j=0; j<backupColumn; j++)
			v_i[j]=backup[i][j];
	}

#ifndef _DEBUG_INFO
	if(bufferBackup) free(bufferBackup);
#endif

}


template <class T>
void _tmat<T>::_getPrivate(T*& buffer2, int & stride2, int& n2, int& m2, int& on2) const
{
	buffer2=buffer;
	stride2=stride;
	n2=n;
	m2=m;
	on2=on;
}

template <class T>
void _tmat<T>::_assignRef( _tmat<T> const& a )
{
	a._getPrivate(buffer, stride, n, m, on);
	on=-1;
}

template <class T>
_tmat<T>& _tmat<T>::assign( _tmat<T> const& a )
{
	_tmat<T> &c = (*this);
	c.setSize( a.rows(), a.cols() );

	int new_size=a.rows()*a.cols();


	if(new_size>0)
	{
		for(int i=0; i<rows(); i++)
		{
			T* c_i=c[i];
			T* a_i=a[i];
			for(int j=0; j<cols(); j++)
				c_i[j]=a_i[j];
		}
	}

	return c;
}


template <class T>
void _tmat<T>::setValue(int r, int c, _tmat<T> const& mat)
{
	RANGE_ASSERT(rows()>=r+mat.rows());
	RANGE_ASSERT(cols()>=c+mat.cols());

	for(int i=0; i<mat.rows(); i++)
		for(int j=0; j<mat.cols(); j++)
			(*this)[i+r][j+c]=mat[i][j];
}


template <class T>
void _tmat<T>::deleteRows(int start, int end)
{
	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다.
	int numRows=end-start;

	for(int i=end; i<rows(); i++)
	{
		T* ti_numRows=(*this)[i-numRows];
		T* ti=(*this)[i];
		for(int j=0; j<cols(); j++)
			ti_numRows[j]=ti[j];
	}

	resize(rows()-numRows, cols());
}

template <class T>
void _tmat<T>::deleteCols(int start, int end)
{
	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다.
	int numCols=end-start;

	for(int j=0; j<rows(); j++)
	{
		T* this_j=(*this)[j];
		for(int i=end; i<cols(); i++)
			this_j[i-numCols]=this_j[i];
	}

	resize(rows(), cols()-numCols);
}

template <class T>
void _tmat<T>::bubbles(int nrow, int nbubbles)
{
	// nrow이하는 nbubble만큼 아래로 내린다. 즉 matrix크기가 nbubble만큼 세로로 커지고, 빈칸이 생긴다.
	int prev_row=rows();
	resize(rows()+nbubbles, cols());

	for(int i=prev_row-1; i>=nrow; i--)
		for(int j=0; j<cols(); j++)
			(*this)[i+nbubbles][j]=(*this)[i][j];

	for(int i=nrow; i<nrow+nbubbles; i++)
		for(int j=0; j<cols(); j++)
			(*this)[i][j]=0;
}

template <class T>
void _tmat<T>::bubbleColumns(int ncolumn, int nbubbles)
{
	// ncolumn우측은 nbubble만큼 우측으로 민다. 즉 matrix크기가 nbubble만큼 가로로 커지고, 빈칸이 생긴다.
	int prev_col=cols();
	resize(rows(), cols()+nbubbles);

	for(int i=0; i<rows(); i++)
	{
		for(int j=prev_col-1; j>=ncolumn; j--)
			(*this)[i][j+nbubbles]=(*this)[i][j];

		for(int j=ncolumn; j<ncolumn+nbubbles; j++)
			(*this)[i][j]=0;
	}
}

template <class T>
void _tmat<T>::setRow( int x, const _tvectorn<T>& vec )
{
	_tmat<T>& v=*this;
	RANGE_ASSERT(vec.size()==m);
	for( int i=0; i<m; i++ )
		v[x][i] = vec[i];
}

template <class T>
void _tmat<T>::getRow( int row, _tvectorn<T>& out ) const
{
	const _tmat<T>& v=*this;
	out.setSize(cols());
	out.setValues(v[row]);
}

template <class T>
void _tmat<T>::setColumn( int x, const _tvectorn<T>& vec )
{
	_tmat<T>& v=*this;
	RANGE_ASSERT(vec.size()==n);
	for (int i=0; i<n; i++)
		v[i][x] = vec[i];
}

template <class T>
void _tmat<T>::getColumn( int col, _tvectorn<T>& out) const
{
	out.setSize(rows());
	for(int i=0; i<n; i++)
		out[i]=(*this)[i][col];
}

template <class T>
void _tmat<T>::pushFront(const _tvectorn<T>& rowVec)
{
	bubbles(0,1);
	setRow(0, rowVec);
}

template <class T>
void _tmat<T>::popFront()
{
	deleteRows(0,1);
}


template <class T>
void _tmat<T>::popFront(_tvectorn<T>& out)
{
	out.assign(_row<_tvectorn<T> >(0));

	deleteRows(0,1);
}

template <class T>
void _tmat<T>::pushBack(const _tvectorn<T>& rowVec)
{
	RANGE_ASSERT(rows()==0 || rowVec.size()==cols());
	resize(rows()+1, rowVec.size());
	setRow(rows()-1, rowVec);
}

template <class T>
void _tmat<T>::popBack()
{
	resize(rows()-1, cols());
}

template <class T>
void _tmat<T>::setColumn(int j, T value)
{
	for(int i=0; i<rows(); i++)
		(*this)[i][j]=value;
}

template <class T>
void _tmat<T>::setRow(int i, T value)
{
	for(int j=0; j<cols(); j++)
		(*this)[i][j]=value;
}

template <class T>
void _tmat<T>::setDiagonal(T value)
{
	RANGE_ASSERT(rows()==cols());
	for(int i=0; i<n ;i++)
		(*this)[i][i]=value;
}

template <class T>
void _tmat<T>::setDiagonal(const _tvectorn<T>& v)
{
	RANGE_ASSERT(rows()==cols());
	RANGE_ASSERT(rows()==v.size());
	for(int i=0; i<n ;i++)
		(*this)[i][i]=v[i];
}

template <class T>
void _tmat<T>::extractRows(_tmat<T> const& mat, _tvectorn<int> const& rows)
{
	setSize(rows.size(), mat.cols());

	for(int i=0; i<rows.size(); i++)
	{
		for(int j=0; j<cols();j++)
			value(i,j)=mat.value(rows[i], j);
	}
}
template <class T>
void _tmat<T>::assignRows(_tmat<T> const& mat, _tvectorn<int> const& rows)
{

	for(int i=0; i<rows.size(); i++)
	{
		for(int j=0; j<cols();j++)
			value(rows[i], j)=mat(i,j);
	}
}

template <class T>
void _tmat<T>::extractColumns(_tmat<T> const& mat, _tvectorn<int> const& columns)
{
	setSize(mat.rows(), columns.size());

	for(int i=0; i<mat.rows(); i++)
	{
		for(int j=0; j<columns.size();j++)
			value(i,j)=mat.value(i, columns[j]);
	}
}
template <class T>
void _tmat<T>::assignColumns(_tmat<T> const& mat, _tvectorn<int> const& columns)
{

	for(int i=0; i<mat.rows(); i++)
	{
		for(int j=0; j<columns.size();j++)
			value(i, columns[j])=mat(i,j);
	}
}


template <class T>
void _tmat<T>::transpose( _tmat<T> const& a )
{
    _tmat<T> &c = (*this);
    c.setSize( a.cols(), a.rows() );

    for( int i=0; i<a.rows(); i++ )
	{
		T* a_i=a[i];
		for( int j=0; j<a.cols(); j++ )
			c(j,i)=a_i[j];
	}
}


template <class T>
void _tmat<T>::mult( _tmat<T> const& a, _tmat<T> const& b )
{
	if(&a==this)
	{
		_tmat<T> aa;
		aa.assign(a);
		return mult(aa,b);
	}

	_tmat<T> bT;
	bT.transpose(b);
	return multABt(a,bT);
}

template <class T>
void _tmat<T>::multABt(_tmat<T> const& a, _tmat<T> const& b)
{
	_tmat<T> &c = (*this);
    RANGE_ASSERT( a.cols()==b.cols() );
    c.setSize( a.rows(), b.rows() );

    for( int i=0; i<a.rows(); i++ )
	{
		T* a_i=a[i];
		for( int j=0; j<b.rows(); j++ )
		{
			T subSum=0;

			T* b_j=b[j];

			for( int k=0; k<a.cols(); k++ )
				subSum += a_i[k] * b_j[k];

			c[i][j] = subSum;
		}
	}
}
template <class T>
void _tmat<T>::multAtB(_tmat<T> const& a, _tmat<T> const& b)
{
	_tmat<T> aT, bT;
	aT.transpose(a);
	bT.transpose(b);
	multABt(aT,bT);	// 이렇게 계산하는게 더 빠름. (cache coherent)

/*	incorrect // A^T B= (B A^T)^T

	_tmat<T> &c = (*this);
    assert( a.cols()==b.cols() );
    c.setSize( a.rows(), b.rows() );

    for( int i=0; i<b.rows(); i++ )
	{
		T* b_i=a[i];
		for( int j=0; j<a.rows(); j++ )
		{
			T subSum=0;

			T* a_j=b[j];

			for( int k=0; k<b.cols(); k++ )
				subSum += b_i[k] * a_j[k];

			c[j][i] = subSum;
		}
	}*/
}

template <class T>
void _tmat<T>::multAtBt(_tmat<T> const& a, _tmat<T> const& b)
{
	if(&b==this)
	{
		_tmat<T> bb;
		bb.assign(b);
		return multAtBt(a,bb);
	}

	_tmat<T> aT;
	aT.transpose(a);
	return multABt(aT,b);
}

#define __private_tmat_for_each2(op)\
	_tmat<T> &c = (*this);\
    RANGE_ASSERT( a.rows()==b.rows() );\
	RANGE_ASSERT( a.cols()==b.cols() );\
	c.setSize(a.rows(), a.cols());\
	int a_rows=a.rows(); int a_cols=a.cols();\
	for(int i=0; i<a_rows; i++)\
	{\
		T* c_i=c[i];\
		T* a_i=a[i];\
		T* b_i=b[i];\
		for(int j=0; j<a_cols; j++)\
			c_i[j]=a_i[j] op b_i[j];\
	}

template <class T>
void  _tmat<T>::add( _tmat<T> const& a, _tmat<T> const& b)
{
	__private_tmat_for_each2(+);
}


#define __private_tmat_for_each2_1(op)\
	_tmat<T> &c = (*this);\
    c.setSize(a.rows(), a.cols());\
	int a_rows=a.rows(); int a_cols=a.cols();\
	for(int i=0; i<a_rows; i++)\
	{\
		T* c_i=c[i];\
		T* a_i=a[i];\
		for(int j=0; j<a_cols; j++)\
			c_i[j]=a_i[j] op b;\
	}

#define __private_tmat_for_each1_2(op)\
	_tmat<T> &c = (*this);\
    c.setSize(b.rows(), b.cols());\
	int c_rows=c.rows(); int c_cols=c.cols();\
	for(int i=0; i<c_rows; i++)\
	{\
		T* c_i=c[i];\
		T* b_i=b[i];\
		for(int j=0; j<c_cols; j++)\
			c_i[j]=a op b_i[j];\
	}

template <class T>
void  _tmat<T>::add( _tmat<T> const& a, T b)
{
	__private_tmat_for_each2_1(+);
}

template <class T>
void  _tmat<T>::add( T a, _tmat<T> const& b)
{
	__private_tmat_for_each1_2(+);
}

template <class T>
void  _tmat<T>::subtract( _tmat<T> const& a, T b)
{
	__private_tmat_for_each2_1(-);
}

template <class T>
void  _tmat<T>::subtract( T a, _tmat<T> const& b)
{
	__private_tmat_for_each1_2(-);
}

template <class T>
void  _tmat<T>::subtract( _tmat<T> const& a, _tmat<T> const& b)
{
	__private_tmat_for_each2(-);
}

#define __private_tmat_for_each1(op)\
	_tmat<T> &c = (*this);\
	RANGE_ASSERT(c.rows()==a.rows());\
	RANGE_ASSERT(c.cols()==a.cols());\
	int c_rows=c.rows(); int c_cols=c.cols();\
	for(int i=0; i<c_rows; i++)\
	{\
		T* c_i=c[i];\
		T* a_i=a[i];\
		for(int j=0; j<c_cols; j++)\
			c_i[j] op;\
	}

template <class T>
void _tmat<T>::operator+=( _tmat<T> const& a )
{
	__private_tmat_for_each1(+= a_i[j])
}

template <class T>
void _tmat<T>::operator-=( _tmat<T> const& a )
{
	__private_tmat_for_each1(-= a_i[j])
}

template <class T>
void _tmat<T>::mult( _tmat<T> const& a, T b )
{
	setSize(a.rows(), a.cols());
	__private_tmat_for_each1(= a_i[j]*b)
}



#define __private_tmat_for_each1_scalar(op)\
	_tmat<T> &c = (*this);\
	int c_rows=c.rows(); int c_cols=c.cols();\
	for(int i=0; i<c_rows; i++)\
	{\
		T* c_i=c[i];\
		for(int j=0; j<c_cols; j++)\
			c_i[j] op a;\
	}

template <class T>
void _tmat<T>::operator*=( T a )
{
	__private_tmat_for_each1_scalar(*=)
}

template <class T>
void _tmat<T>::operator/=( T aa )
{
	T a=1.0/aa;
	__private_tmat_for_each1_scalar(*=)
}

template <class T>
void _tmat<T>::concatRow( _tmat<T> const& a, _tmat<T> const& b)
{
	RANGE_ASSERT(a.cols()==b.cols());
	setSize(a.rows()+b.rows(), a.cols());

	for(int i=0; i<a.rows(); i++)
	{
		_row<_tvectorn<T> >(i)=a._row<_tvectorn<T> >(i);
	}
	for(int i=0; i<b.rows(); i++)
	{
		_row<_tvectorn<T> >(i+a.rows())=b._row<_tvectorn<T> >(i);
	}
}

template <class T>
void _tmat<T>::concatColumn( _tmat<T> const& a, _tmat<T> const& b)
{
	RANGE_ASSERT(a.rows()==b.rows());
	setSize(a.rows(), a.cols()+b.cols());

	for(int i=0; i<a.rows(); i++)
	{
		_row<_tvectorn<T> >(i).concat(a._row<_tvectorn<T> >(i),b._row<_tvectorn<T> >(i));
	}
}

template <class T>
void _tmat<T>::concatRow( _tmat<T> const& b)
{
	_tmat<T>& a=*this;
	RANGE_ASSERT(a.rows()==0 || a.cols()==b.cols());
	int prevRow=a.rows();
	resize(a.rows()+b.rows(), b.cols());

	for(int i=0; i<b.rows(); i++)
	{
		_row<_tvectorn<T> >(i+prevRow)=b._row<_tvectorn<T> >(i);
	}
}


#endif
