#pragma once



class intmat3D
{
protected:
	int pages, nrows, columns;
	int* data;
	std::vector<intmatrixnView*> m_pages;
public:
	intmat3D(void);
	intmat3D(int pages, int rows, int columns);
	intmat3D(const intmat3D& other);
	~intmat3D(void);

	int	page() const	{return pages; }
	int rows() const		{return nrows; }
	int cols() const	{return columns; }

	void setSize( int, int, int);  //!< 원래 데이타 유지 보장 전혀 없음.
	void setSameSize(intmat3D const& other)	{ setSize(other.page(), other.rows(), other.cols());};
	intmatrixnView& page(int index) const				{ return *m_pages[index];}
	intmatrixnView& operator[](int index) const		{ return page(index);}

	intmat3D&  assign(intmat3D const& other);
	intmat3D&  operator=(intmat3D const& other)	{ return assign(other);};
};

//! 3D matrix 클래스
/*! conventions:
1. 그 자체로 3D 매트릭스를 표현하거나,
2. 2D matrix의 시간축 나열 형태 (각 page가 2D matrix, 즉, 시간축이 page 방향이 된다. )
3. multi-dimension signal 여러개. (시간축이 row방향이 된다.)
사실 2와 3은 구별이 명확하지 않지만, 서로 독립적인 matrix는 보통 3번 convention을 택하는 것이 각각 독립적으로
smoothing하거나 그래프를 그릴수 있어 편한 반면(ex: 왼손 velocity signal, 오른손 velocity signal)
비디오 시퀀스 같은 독립적이지 않은 signal은 matrix의 나열인 2번 convention을 택하는 것이 좋겠다.

\ingroup group_math
*/

class hypermatrixn
{
protected:
	int npages, nrows, columns;
	matrixn data;
public:
	inline int _getStride1() const { return data._getStride();}
	inline int _getStride2() const { return columns;}
	inline int _getStride3() const { return 1;}

	hypermatrixn(void);
	hypermatrixn(int pages, int nrows, int columns);
	hypermatrixn(const hypermatrixn& other);
	~hypermatrixn(void);

	inline int	page() const	{return npages; } // deprecated
	inline int	pages() const	{return npages; }
	inline int rows() const		{return nrows; }
	inline int cols() const	{return columns; }

	void setSize( int, int, int);  //!< 원래 데이타 유지 보장 전혀 없음.
	void setSameSize(hypermatrixn const& other)	{ setSize(other.page(), other.rows(), other.cols());};
	void resize( int , int, int); // nrows, ncolumns만 안바뀌면 data 유지. 바뀐영역 0으로 초기화는 하지 않음.
	void pushBack(const matrixn& mat);
	inline matrixnView page(int index) const			{ 
		RANGE_ASSERT(index>=0 && index<pages()); 
		return  data.row(index).matView(rows(), cols());
	}
	inline double& operator()(int index, int j, int k) 
	{
		RANGE_ASSERT(index>=0 && index<pages()); 
		return data.row(index)(j*cols()+k);
	}
	inline const double& operator()(int index, int j, int k)  const
	{
		RANGE_ASSERT(index>=0 && index<pages()); 
		return data.row(index)(j*cols()+k);
	}
	// extract
	inline matrixn row(int index) const {
		matrixn out(pages(), cols());
		for(int i=0; i<pages(); i++)
			for(int j=0; j<cols(); j++)
				out(i,j)=(*this)(i,index, j);

		return out;
	}
	inline void setRow(int index, matrixn const& in) {
		matrixn out(pages(), cols());
		for(int i=0; i<pages(); i++)
			for(int j=0; j<cols(); j++)
				(*this)(i,index, j)=in(i,j);
	}
	// extract
	inline matrixn column(int index) const {
		matrixn out(pages(), rows());
		for(int i=0; i<pages(); i++)
			for(int j=0; j<rows(); j++)
				out(i,j)=(*this)(i,j,index);

		return out;
	}
	inline void setColumn(int index, matrixn const& in) {
		matrixn out(pages(), rows());
		for(int i=0; i<pages(); i++)
			for(int j=0; j<rows(); j++)
				(*this)(i,j,index)=in(i,j);
	}
	matrixnView operator[](int index) const		{ return page(index);}

	hypermatrixn&  assign(hypermatrixn const& other);
	hypermatrixn&  operator=(hypermatrixn const& other)	{ return assign(other);};

	void each(const m1::_op& op, const hypermatrixn& other);
	matrixn weightedAverage(const vectorn & page_weights) const;
};



/*

class matrixn;

#include "../utility/TArray.h"
#include <typeinfo.h>


namespace h2
{
	//enum { ADD, SUB};

	/// Operator를 쓰고 싶으면, 예를들어 Operator(ADD) 대신 그냥 ADD 이라고 쓰면 됨.
	struct Operator
	{
		Operator()				{ m_eOperator=-1;}
		Operator(int op)	{ m_eOperator=(int)op;}
		virtual ~Operator()		{}
		virtual void calc(hypermatrixn& c, const hypermatrixn& a, const hypermatrixn& b) const
		{	Msg::error("h2::%s::calc(vv) not implemented!!!\n", typeid( *this).name()); ASSERT(0);	}
		int m_eOperator;
	};
}

namespace h1
{
	//enum { };

	/// Operator를 쓰고 싶으면, 예를들어 Operator(ADD) 대신 그냥 ADD 이라고 쓰면 됨.
	struct Operator
	{
		Operator()		{ m_eOperator=-1;}
		Operator(int op)	{ m_eOperator=(int)op;}
		virtual ~Operator()		{}
		virtual void calc(hypermatrixn& c, const hypermatrixn& a) const
		{	Msg::error("h1::%s::calc(vv) not implemented!!!\n", typeid( *this).name()); ASSERT(0);}
		int m_eOperator;
	};

}

namespace sh1
{
	enum { LENGTH, MINIMUM, MAXIMUM, SUM, AVERAGE, RMS , SQUARESUM } ;

	/// Operator를 쓰고 싶으면, 예를들어 Operator(LENGTH)대신 그냥 LENGTH 라고 쓰면 됨.
	struct Operator
	{
		Operator()	{}
		Operator(int op);
		virtual ~Operator(){};
		virtual m_real calc(const hypermatrixn& c) const;
		int m_eOP;
	};
};

class hypermatrixn :
	protected  CTArray<matrixn>
{
private:
	friend class matrixn;
	int pages, rows, columns;
	bool m_bDirty;	// 원소중에 하나에서 resize가 수행된경우에 m_bDirty가 set된다. 즉 모든 페이지가 같은 크기가 아닐수 있다는 뜻이 된다.
public:
	hypermatrixn(void);
	hypermatrixn(int pages, int rows, int columns);
	hypermatrixn(const hypermatrixn& other);
	~hypermatrixn(void);

	bool dirty();
	void clean();	// 원소 크기가 다른 경우 안전한 더 큰 크기로 바꾸어준다.

    int	page() const	{return pages; }
    int rows() const		{return rows; }
    int cols() const	{return columns; }

	void setSize( int, int, int);  //!< 원래 데이타 유지 보장 전혀 없음.
	void resize(int, int, int);		//!< 원래 데이타 최대한 유지. 빈자리는 0으로(느림)
	void setSameSize(hypermatrixn const& other)	{ setSize(other.page(), other.rows(), other.cols());};
	matrixn& page(int index) const				{ return CTArray<matrixn>::operator [](index);};
	matrixn& operator[](int index) const		{ return page(index);};

	void fromMatrix(const matrixn& mat, int column);

	// unary operations
	m_real op1(const sh1::Operator& op) const;
	hypermatrixn&  op1(const s1::Operator&, m_real);
	hypermatrixn&  op1(const h1::Operator&, hypermatrixn const&);
	hypermatrixn&  operator=(hypermatrixn const& other)	{ return assign(other);};
	hypermatrixn&  assign(hypermatrixn const& other);

	// binary operations
	hypermatrixn&  op2(const h2::Operator&, hypermatrixn const&, hypermatrixn const&);

	void	  load(const char* filename, bool bLoadFromBinaryFile=false);
	void      save(const char* filename, bool bSaveIntoBinaryFile=false);

};
*/

