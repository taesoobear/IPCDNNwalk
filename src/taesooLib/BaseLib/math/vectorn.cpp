
#include "stdafx.h"
#include "mathclass.h"
#include "float.h"
#include "Metric.h"

//m_real* allocate(int n);
//void deallocate(m_real *b);

intvectornView::intvectornView(const intvectorn& other)
{
	assignRef(other);
}

intvectornView intvectorn::range(int start, int end, int step)
{
	return _range<intvectornView >(start, end, step);
}

const intvectornView intvectorn::range(int start, int end, int step) const	{ return ((intvectorn*)this)->range(start, end, step);}

intvectorn& intvectornView::operator=(const intvectorn& other){assign(other);return *this;}	

intvectorn& intvectorn ::operator=(const intvectornView& other){ assign(other);return *this;}

vectorn& vectorn::operator=(const vectorn& other)		{ assign(other);return *this;}	
vectorn& vectorn::operator=(const vectornView& other)	{ assign(other);return *this;}

intvectorn&
intvectorn::assignBits( bitvectorn const& a )
{
	intvectorn &c = (*this);
	c.setSize( a.size() );

	for( int i=0; i<c.size(); i++ )
		c[i] = (int)a[i];
	return c;
}


intvectorn::intvectorn(const intvectornView& other)
:_tvectorn<int>()
{	
	assign(other);
}


intvectornView ::intvectornView (const int* ptrr, int size, int str)
:intvectorn((int*)ptrr,size,str)
{
}

void intvectorn::decode(const TString& input)
{
	int i=0;
	TString token;
	setSize(0);
	while(i!=input.length())
	{
		input.token(i, TString(" ,\n"), token);
		pushBack(atoi(token));
	}
}


TString intvectorn::output(const char* left, const char* typeString, const char* seperator, const char* right) const
{
	TString id;
	id+=left;
	for(int i=0; i<size(); i++)
	{
		id.add(typeString, (*this)[i]);

		if(i!=size()-1)
			id.add("%s", seperator);
	}
	id+=right;

	return id;
}


intvectorn::intvectorn( int n, int x, ...)	// n dimensional vector	(ex) : vectorn(3, 1.0, 2.0, 3.0);
:_tvectorn<int>()
{
	va_list marker;
	va_start( marker, x);     /* Initialize variable arguments. */

	setSize(n);
	setValue(0, x);
	for(int i=1; i<n; i++)
	{
		setValue(i, va_arg( marker, int));
	}
	va_end( marker );              /* Reset variable arguments.      */	
}

intvectorn&  intvectorn::setAt(intvectorn const& columnIndex, _tvectorn<int> const& value)
{
	ASSERT(value.size()==columnIndex.size());
	for(int i=0; i<columnIndex.size(); i++)
	{
		(*this)[columnIndex[i]]=value[i];
	}
	return *this;
}

intvectorn&  intvectorn::setAt( intvectorn const& columnIndex, int value)
{
	for(int i=0; i<columnIndex.size(); i++)
	{
		(*this)[columnIndex[i]]=value;
	}
	return *this;
}

vectorn intvectorn::toVectorn()
{
	vectorn c;
	c.setSize(size());

	for(int i=0; i<size(); i++)
		c[i]=(m_real)value(i);
	return c;
}


void intvectorn::runLengthEncodeCut(const bitvectorn& cutState, int start, int end)
{
	if(start<0) start=0;
	if(end>cutState.size()) end=cutState.size();

	setSize(0);

	pushBack(start);  // start
	int i;
	for(i=start+1; i<end; i++)
	{
		if(cutState[i])
		{
			pushBack(i);	// end
			pushBack(i);   // start
		}
	}

	pushBack(i);	// end
}

void intvectorn::runLengthEncode(const intvectorn& source)
{
	/**
	* RunLength encoding을 구한다.
	* [1 1 1 3 3 3 3 4 4] -> [0 1 3 3 7 4 9]
	* 0 부터 1이 3까지 나오고, 3이 7까지 나오고 4가 9까지 나온다는 뜻.
	* \param source 
	*/

	setSize(0);
	pushBack(0);
	int curValue=source[0];

	int  i;
	for(i=1; i<source.size(); i++)
	{
		if(curValue!=source[i])
		{
			pushBack(curValue);
			pushBack(i);
			curValue=source[i];
		}
	}
	pushBack(curValue);
	pushBack(i);
}

int intvectorn::minimum() const
{
	const intvectorn& v=*this;
	int min=v[0];
	for(int i=1; i<size(); i++)
	{
		if(v[i]<min) min=v[i];
	}
	return min;
}

int intvectorn::maximum() const
{
	const intvectorn& v=*this;
	int max=v[0];
	for(int i=1; i<size(); i++)
	{
		if(v[i]>max) max=v[i];
	}
	return max;
}

int intvectorn::sum() const
{
	const intvectorn& v=*this;
	int sum=0;
	for(int i=0; i<size(); i++)
	{
		sum+=v[i];
	}
	return sum;	
}

intvectorn&  intvectorn::makeSamplingIndex(int nLen, int numSample)
{
	setSize(numSample);

	// simple sampling
	float len=(float)nLen;

	float factor=1.f/(float)numSample;
	for(int i=0; i<numSample; i++)
	{
		float position=((float)i+0.5f)*factor;
		(*this)[i]=(int)(position*len);
	}
	return *this;
}

intvectorn&  intvectorn::makeSamplingIndex2(int nLen, int numSample)
{
	// 첫프레임과 마지막 프레임은 반드시 포함하고 나머지는 그 사이에서 uniform sampling
	if(numSample<2 || nLen<3)
		return makeSamplingIndex(nLen, numSample);

	setSize(numSample);
	(*this)[0]=0;
	(*this)[numSample-1]=nLen-1;

	if(numSample>2)
	{
		intvectorn samplingIndex;
		samplingIndex.makeSamplingIndex(nLen-2, numSample-2);
		samplingIndex+=1;

		for(int i=1; i<numSample-1; i++)
		{
			this->value(i)=samplingIndex[i-1];
		}
	}
	return *this;
}


intvectorn&  intvectorn::colon(int start, int end, int stepSize)
{
	int nSize;

	/* buggy
	if(stepSize==1)
		nSize=(end-start);
	else
		nSize=(end-start+1)/stepSize;
	*/

	nSize=0;
	for(int i=start; i<end; i+=stepSize)
		nSize++; // nSize = end - start (if stepSize = 0)

	setSize(nSize);
	int currPos=0;
	for(int i=start; i<end; i+=stepSize)
	{
		(*this)[currPos]=i;
		currPos++;
	}
	ASSERT(currPos==nSize);
	return *this;
}

class info
{
public:
	info(){}
	info(const vectorn * pIn, int index){m_pInput=pIn; m_index=index;};
	~info(){}
	const vectorn * m_pInput;
	int m_index;
	static int compareInfo(const void** ppA, const void** ppB)
	{
		info& a=*((info*)*ppA);
		info& b=*((info*)*ppB);
		m_real valA=a.m_pInput->getValue(a.m_index);
		m_real valB=b.m_pInput->getValue(b.m_index);
		if(valA<valB)
			return -1;
		if(valA==valB)
			return 0;
		return 1;
	}
	static info* factory(void* param, int index)	{ return new info((const vectorn *) param, index);};
};



intvectorn&  intvectorn::sortedOrder(vectorn const & input)
{
	//!< input[0]<input[2]<input[1]<input[3]인경우 결과는 [0213]

	TArray<info> aSort;
	aSort.init(input.size());
	for(int i=0; i<input.size(); i++)
	{
		aSort[i].m_pInput=&input;
		aSort[i].m_index=i;
	}

	aSort.sort(0, input.size(), info::compareInfo);

	setSize(input.size());
	for(int i=0; i<input.size(); i++)
	{
		(*this)[i]=aSort[i].m_index;
	}
	return *this;
}

int intvectorn::count(int (*s2_func)(int,int), int value, int start, int end)
{
	int count=0;
	if(start<0) start=0;
	if(end>size()) end=size();

	for(int i=start; i<end; i++)
		if(s2_func((*this)[i], value)) count++;

	return count;
}

void intvectorn::runLengthEncode(const bitvectorn& source, int start, int end)
{
	if(start<0) start=0;
	if(end>source.size()) end=source.size();

	setSize(0);

	bool bFindTrue=false;
	int i;
	for(i=start; i<end; i++)
	{
		if(bFindTrue)
		{
			if(!source[i])
			{
				pushBack(i);	// end
				bFindTrue=false;
			}
		}
		else
		{
			if(source[i])
			{
				pushBack(i);	// start
				bFindTrue=true;
			}
		}
	}

	if(bFindTrue)
		pushBack(i);	// end
}

void intvectorn::runLengthDecode(boolN& out, int size)
{
	out.resize(size);
	out.clearAll();

	for(int i=0; i< this->size()/2; i++)
	{
		int start=(*this)[i*2];
		int end=(*this)[i*2+1];

		out.setValue(start, end, true);
	}	
}


vectornView ::vectornView (const m_real* ptrr, int size, int str)
:vectorn((m_real*)ptrr,size,str)
{
}

/////////////////////////////////////////////////////////////////////////////////


vectorn::vectorn( int n, m_real x)
:_tvectorn<m_real>()
{
	ASSERT(n==1);
	setSize(n);
	value(0)=x;
}

vectorn::vectorn( int n, m_real x, m_real y)
:_tvectorn<m_real>()
{
	ASSERT(n==2);
	setSize(n);
	value(0)=x;
	value(1)=y;
}
vectorn::vectorn( int n, m_real x, m_real y, m_real z)
:_tvectorn<m_real>()
{
	ASSERT(n==3);
	setSize(n);
	value(0)=x;
	value(1)=y;
	value(2)=z;
}


vectorn::vectorn( int n, m_real x, m_real y, m_real z, m_real w,...)	// n dimensional vector	(ex) : vectorn(3, 1.0, 2.0, 3.0);
:_tvectorn<m_real>()
{
	va_list marker;
	va_start( marker, w);     /* Initialize variable arguments. */

	setSize(n);
	setValue(0, x);
	setValue(1, y);
	setValue(2, z);
	setValue(3, w);
	for(int i=4; i<n; i++)
	{
		setValue(i, va_arg( marker, m_real));
	}
	va_end( marker );              /* Reset variable arguments.      */	
}

vectornView vectorn::range(int start, int end, int step)
{
	return _range<vectornView >(start,end,step);
}

const vectornView vectorn::range(int start, int end, int step) const	{ return ((vectorn*)this)->range(start, end, step);}



vectorn& vectorn::sort(vectorn const& source, intvectorn& sortedIndex)
{
	sortedIndex.sortedOrder(source);
	extract(source, sortedIndex);
	return *this;
}


vectorn& vectorn::concaten(vectorn const& a)					
{
	vectorn& c=*this;
	int prev_size=c.size();
	c.resize(c.size()+a.size());
	c.range(prev_size, prev_size+a.size())= a;
	return c;
}


/////////////////////////////////////////////////////////////////////////////////
vectorn::vectorn()
:_tvectorn<m_real>()
{
}


vectorn::vectorn(const vector3& other)
:_tvectorn<m_real>()
{
	assign(other);
}

vectorn::vectorn(const quater& other)
:_tvectorn<m_real>()
{
	assign(other);
}

// 값을 카피해서 받아온다.	
vectorn::vectorn(const _tvectorn<m_real>& other)
:_tvectorn<m_real>()
{
	_tvectorn<m_real>::assign(other);
}	

vectorn::vectorn(const vectorn& other)
:_tvectorn<m_real>()
{
	assign(other);
}	

vectorn::vectorn(const vectornView& other)
:_tvectorn<m_real>()
{
	assign(other);
}	

matrixnView vectorn::column() const
{
	return _column<matrixnView >();
}

matrixnView vectorn::row() const		// return 1 by n matrix, which can be used as L-value (reference matrix)
{
	return _row<matrixnView >();
}

matrixnView vectorn::matView(int nrow, int ncol)
{
	resize(nrow* ncol);
	ASSERT(_getStride()==1);
	return matrixnView(dataPtr(), nrow, ncol, ncol);
}

vectorn& vectorn::assign(const vector3& other)
{
	setSize(3);
	for(int i=0; i<3; i++)
		value(i)=other.getValue(i);
	return *this;
}

vectorn& vectorn::assign(const quater& other)
{
    setSize(4);
	for(int i=0; i<4; i++)
		value(i)=other.getValue(i);
	return *this;
}


void vectorn::each1(void (*cOP)(m_real&,m_real), vectorn const& a)
{
	vectorn& c=*this;
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		cOP(c[i], a[i]);
}

void vectorn::each1(void (*cOP)(m_real&,m_real), m_real a)
{
	vectorn& c=*this;
	for(int i=0; i<c.size(); i++)
		cOP(c[i], a);
}

void vectorn::each2(const sv2::_op& op, const matrixn& a, const matrixn& b)
{
	vectorn& c=*this;
	c.setSize(a.rows());
	for(int i=0; i<a.rows(); i++)
		c[i]=op.calc(a.row(i), b.row(i));
}

void vectorn::each2(const sv2::_op& op, const matrixn& a, const vectorn& b)
{
	vectorn& c=*this;
	c.setSize(a.rows());
	for(int i=0; i<a.rows(); i++)
		c[i]=op.calc(a.row(i), b);
}

vectorn&
vectorn::negate()
{
    vectorn &c = (*this);
    for( int i=0; i<c.size(); i++ )
        c[i] = -c[i];
    return c;
}

m_real
operator%( vectorn const& a, vectorn const& b )
{
    assert( a.size()==b.size() );

    m_real c=0;
    for( int i=0; i<a.size(); i++ )
        c += a[i] * b[i];
    return c;
}



bool operator<(vectorn const& a, vectorn const& b)
{
	for(int i=0; i<a.size(); i++)
	{
		if(a[i]>=b[i]) return false;
	}
	return true;
}

bool operator>(vectorn const& a, vectorn const& b)
{
	for(int i=0; i<a.size(); i++)
	{
		if(a[i]<=b[i]) return false;
	}
	return true;
}


m_real
vectorn::length() const
{
    m_real c=0;
    for( int i=0; i<size(); i++ )
        c += this->value(i)*this->value(i);
    return sqrt(c);
}

m_real vectorn::distance(vectorn const& other, Metric* pMetric) const
{
	if(pMetric)
	{
		return pMetric->CalcDistance(*this, other);
	}

	ASSERT(size()==other.size());
	m_real c=0;	
	m_real dist;
	for(int i=0; i<size(); i++)
	{
		dist=value(i)-other[i];
		c+=dist*dist;
	}
	return sqrt(c);
}


vectorn&
vectorn::normalize()
{
    vectorn &c = (*this);

    m_real invl = 1/this->length();
    (*this)*=invl;
    return c;
}

vectorn& vectorn::normalize(vectorn const& a)
{
	vectorn &c = (*this);

    m_real invl = 1/a.length();
    c.mult(a, invl);
    return c;
}

vectorn& vectorn::normalize(vectorn const& min, vectorn const& max)
{
	ASSERT(size()==min.size());
	ASSERT(size()==max.size());

	for(int j=0; j<size(); j++)
	{
		m_real value=getValue(j);
		value=(value-min[j])/(max[j]-min[j]);
		setValue(j,value);
	}
	return *this;
}



/*
ostream& operator<<( ostream& os, vectorn const& a )
{
    os << "( ";
    for( int i=0; i< a.size()-1; i++ )
        os << a.v[i] << " , ";
    os << a.v[a.size()-1] << " )";
    return os;
}

istream& operator>>( istream& is, vectorn& a )
{
	static char	buf[256];
    //is >> "(";
	is >> buf;
    for( int i=0; i< a.size()-1; i++ )
	{
		//is >> a.v[i] >> ",";
		is >> a.v[i] >> buf;
	}
	//is >> a.v[a.size()-1] >> ")";
	is >> a.v[a.size()-1] >> buf;
    return is;
}
*/

m_real	vectorn::cosTheta(vectorn const& b) const
{
	vectorn const& a=*this;
	// a dot b= |a||b|cosTheta
	return (a)%(b)/(a.length()*b.length());
}

// calc angle between 0 to pi
m_real	vectorn::angle(vectorn const& b) const 
{
	vectorn const& a=*this;
	return (m_real)(ACOS(a.cosTheta(b)));
}

vectorn& vectorn::fromMatrix(matrixn const& mat)
{
	mat.toVector(*this); 
	return *this;
}

TString vectorn::output(const char* formatString, int start, int end) const
{
	TString id;
	if(end>size()) end=size();
	id+="[";
	for(int i=start; i<end; i++)
	{
		double d=(*this)[i];
		if(d<1e-10 && d>-1e-10)
			id+="0,";
		else
		{
			id.add(formatString, (*this)[i]);
			id+=",";
		}
	}
	id+="]";

	return id;
}




void  vectorn::each2(m_real (*s2_func)(m_real,m_real), vectorn const& a, vectorn const& b)
{
	vectorn& c=*this;
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=s2_func(a[i],b[i]);
}


vectorn& vectorn::derivative(vectorn const& a)					
{
	vectorn& c=*this;
	c.setSize(a.size());
	ASSERT(c.size()>2);
	for(int i=1; i<a.size()-1; i++)
	{
		c[i]= (a[i+1]- a[i-1])/2.f;
	}
	c[0]=c[1];
	c[a.size()-1]=c[a.size()-2];
	return c;
}

m_real vectorn::aggregate(CAggregate::aggregateOP eOP) const
{
	const vectorn& c=*this;
	CAggregate cOP(eOP);
	m_real cur=cOP.Init();

	for( int i=0; i<c.size(); i++ )
		cOP.Update(cur, c[i]);

	return cOP.Final(cur, c.size());
}
void vectorn::aggregateEachRow(CAggregate::aggregateOP eOP, matrixn const& mat)
{
	setSize(mat.rows());
	for(int i=0; i<mat.rows(); i++)
	{
		value(i)=mat.row(i).aggregate(eOP);
	}
}


intvectorn operator+( intvectorn const& a, int b)			{ intvectorn c; c.add(a,b); return c;};
intvectorn operator-( intvectorn const& a, int b)			{ intvectorn c; c.add(a,-b); return c;};

vectorn operator+( vectorn const& a, vectorn const& b)	{ vectorn c; c.add(a,b); return c;}
vectorn operator-( vectorn const& a, vectorn const& b)	{ vectorn c; c.sub(a,b); return c;}
vectorn operator*( vectorn const& a, vectorn const& b )	{ vectorn c; c.mult(a,b); return c;}
vectorn operator/( vectorn const& a, vectorn const& b )	{ vectorn c; c.div(a,b); return c;}
vectorn operator+( vectorn const& a, m_real b)			{ vectorn c; c.add(a,b); return c;};
vectorn operator-( vectorn const& a, m_real b)			{ vectorn c; c.sub(a,b); return c;};
vectorn  operator/( vectorn const& a, m_real b)			{ vectorn c; c.div(a,b); return c;};
vectorn operator*( vectorn const& a, m_real b )			{ vectorn c; c.mult(a,b); return c;};

vectorn operator*( matrixn const& a, vectorn const& b )		{ vectorn c; c.multmat(a,b); return c;};

void vectorn::aggregateEachColumn(CAggregate::aggregateOP eOP, matrixn const& mat)
{
	setSize(mat.cols());
	for(int i=0; i<mat.cols(); i++)
	{
		value(i)=mat.column(i).aggregate(eOP);
	}
}
m_real vectorn::minimum() const			
{
	return aggregate(CAggregate::MINIMUM);
}
m_real vectorn::maximum()	const		
{
	return aggregate(CAggregate::MAXIMUM);
}

m_real vectorn::sum()	const			
{
	return aggregate(CAggregate::SUM);
}
m_real vectorn::squareSum() const		
{
	return aggregate(CAggregate::SQUARESUM);
}
m_real vectorn::avg() const	
{
	return aggregate(CAggregate::AVG);
}
// calc angle between 0 to 2pi
m_real	vectorn::angle2D(vectorn const& b) const
{
	vectorn const& a=*this;

	m_real rad=a.angle(b);
	// length 가 0인경우 발생.
	ASSERT(rad==rad);
	if(a.sinTheta(b)<0)
		return (m_real)(2.0*M_PI-rad);

	return rad;
}

m_real	vectorn::sinTheta(vectorn const& b) const
{
	vectorn const& a=*this;

	// |a cross b| = |a||b|sinTheta
	ASSERT(a.size()==2);
	ASSERT(b.size()==2);

	vectorn a3,b3;
	a3.setSize(3);
	b3.setSize(3);

	a3.x()=a.getValue(0);
	a3.y()=a.getValue(1);
	a3.z()=0;
	b3.x()=b.getValue(0);
	b3.y()=b.getValue(1);
	b3.z()=0;
	m_real sinTheta;
	if(a%b>0)
	{
		vector3 crs;
		crs.cross(a3.toVector3(),b3.toVector3());
		sinTheta=crs.z/(a.length()*b.length());
	}
	else
	{
		vector3 crs;
		crs.cross(b3.toVector3(),-a3.toVector3());
		sinTheta=crs.z/(a.length()*b.length());
	}
	ASSERT(-1<=sinTheta);
	ASSERT(sinTheta<=1);

	return sinTheta;
}
vectorn&  vectorn::resample(vectorn const& vec, int numSample)
{
	setSize(numSample);

	intvectorn iv;
	iv.makeSamplingIndex(vec.size(), numSample);

	for(int i=0; i<numSample; i++)
	{
		(*this)[i]=vec[iv[i]];
	}
	return *this;
}

// namespace vectorUtil

/*

int vectorn::argNearest(m_real value) const
{
	int argNearest=-1;
	m_real min=FLT_MAX;

	m_real dist;
	for(int i=0; i<size(); i++)
	{
		if((dist=(m_real)(ABS((*this)[i]-value))) < min)
		{
			min=dist;
			argNearest=i;
		}
	}
	return argNearest;
}

int vectorn::argMax(int_vectorn const& columns) const
{
	int argMax=-1;
	m_real max=-FLT_MAX;
	for(int i=0; i<columns.size(); i++)
	{
		if((*this)[columns[i]]>max)
		{
			max=(*this)[columns[i]];
			argMax=columns[i];
		}
	}
	return argMax;
}


vectorn&  vectorn::normalizeSignal(m_real min, m_real max)
{
m_real sigMin=minimum();
m_real sigMax=maximum();

for(int i=0; i<size(); i++)
value(i)=(value(i)-sigMin)/(sigMax-sigMin)*(max-min)+min;

return *this;
}

vectorn&
vectorn::solve( matrixn const& a, vectorn const& b, int num, m_real tolerance, m_real damp )
{
vectorn &c = (*this);
assert( a.rows()==a.cols() );
assert( a.rows()==b.size() );
c.setSize( b.size() );

int flag = TRUE;
for( int i=0; i<num && flag; i++ )
{
flag = FALSE;
for( int j=0; j<a.rows(); j++ )
{
m_real r = b[j] - a[j]%c;
c[j] += damp*r/a[j][j];
if ( r>tolerance ) flag = TRUE;
}
}

return c;
}


vectorn&
vectorn::solve( matrixn const& a, vectorn const& b )
{
vectorn &c = (*this);
assert( a.rows()==a.cols() );
assert( a.rows()==b.size() );

int n = b.size();
c.setSize( n );
c.assign( b );

static matrixn mat; mat.setSize( n, n );
mat.assign( a );

static int* index;
static int index_count = 0;
if ( index_count<n )
{
if ( index_count>0 ) delete[] index;
index_count = n;
if ( index_count>0 ) index = new int[index_count];
}

mat.LUdecompose( index );
mat.LUsubstitute( index, c );

return c;
}


vectorn&
vectorn::solve( matrixn const& a, vectorn const& b, m_real tolerance )
{
int m = a.rows();
int n = a.cols();

assert( m >= n );
assert( b.size()==m );

vectorn &c = (*this);
c.setSize( n );

static matrixn u; u.setSize( m, n );
static vectorn w; w.setSize( n );
static matrixn v; v.setSize( n, n );

u.assign( a );
NR_OLD::SVdecompose( u, w, v );

int i, j;
m_real s;
static vectorn tmp; tmp.setSize( n );

m_real wmax = 0.0f;
for( j=0; j<n; j++ )
if ( w[j] > wmax ) wmax = w[j];

for( j=0; j<n; j++ )
if ( w[j] < wmax * tolerance ) w[j] = 0.0f;

for( j=0; j<n; j++ )
{
s = 0.0f;
if ( w[j] )
{
for( i=0; i<m; i++ )
s += u[i][j] * b[i];
s /= w[j];
}
tmp[j] = s;
}

for ( j=0; j<n; j++ )
{
s = 0.0;
for ( i=0; i<n; i++ )
s += v[j][i] * tmp[i];
c[j] = s;
}

return c;
}


vectorn&  vectorn::colon(m_real start, m_real stepSize, int nSize)
{
if(nSize!=-1)
setSize(nSize);

m_real cur=start;
for(int i=0; i<size(); i++)
{
v[i]=cur;
cur+=stepSize;
}
return *this;
}

vectorn& vectorn::linspace(m_real x1, m_real x2, int nSize)
{
if(nSize!=-1)
setSize(nSize);

// simple sampling
m_real len=x2-x1;

m_real factor=1.f/((m_real)size()-1);
for(int i=0; i<size(); i++)
{
// position : increases from 0 to 1
m_real position=((m_real)i)*factor;
(*this)[i]=position*len+x1;
}
return *this;
}

vectorn&  vectorn::uniform(m_real x1, m_real x2, int nSize)
{
if(nSize!=-1)
setSize(nSize);

// simple sampling
m_real len=x2-x1;
m_real factor=1.f/((m_real)size());

for(int i=0; i<size(); i++)
{
m_real position=((m_real)(i+(i+1)))*factor/2.f;
(*this)[i]=position*len+x1;
}
return *this;
}


vectorn&  vectorn::makeSamplingIndex(int nLen, int numSample)
{
setSize(numSample);

// simple sampling
m_real len=(m_real)nLen;

m_real factor=1.f/(m_real)numSample;
for(int i=0; i<numSample; i++)
{
m_real position=((m_real)i+0.5f)*factor;
(*this)[i]=(m_real)(position*len);
}
return *this;
}

vectorn& vectorn::interpolate(vectorn const& a, vectorn const& b, m_real t)
{
ASSERT(a.size()==b.size());
setSize(a.size());

for(int i=0; i<n; i++)
{
(*this)[i]=a[i]*(1-t)+b[i]*t;
}
return *this;
}


vectorn& vectorn::sort(vectorn const& source, int_vectorn& sortedIndex)
{
sortedIndex.sortedOrder(source);
return extract(source, sortedIndex);
}

*/
intvectorn& intvectorn::findIndex(intvectorn const& source, int value)
{
	int count=0;
	for(int i=0; i<source.size(); i++)
	{
		if(source[i]==value)
			count++;
	}
	setSize(count);
	count=0;
	for(int i=0; i<source.size(); i++)
	{
		if(source[i]==value)
		{
			(*this)[count]=i;
			count++;
		}
	}
	return *this;
}

int intvectorn::findFirstIndex(int value) const
{
	for(int i=0; i<size(); i++)
		if((*this)[i]==value)
			return i;
	return -1;
}



intvectorn& intvectorn::findIndex(bitvectorn const& source, bool value, int start, int end)
{
	if(start<0) start=0;
	if(end>source.size()) end=source.size();

	int count=0;
	for(int i=start; i<end; i++)
	{
		if(source[i]==value)
			count++;
	}
	setSize(count);
	count=0;
	for(int i=start; i<end; i++)
	{
		if(source[i]==value)
		{
			(*this)[count]=i;
			count++;
		}
	}
	return *this;
}

void vectorn::colon2(m_real start, m_real end, m_real stepSize)
{
	int nsize=0;
	for(m_real v=start; v<end; v+=stepSize)
		nsize++;

	setSize(nsize);

	int c=0;
	for(m_real v=start; v<end; v+=stepSize)
	{
		value(c++)=v;
	}
}

void  vectorn::colon(m_real start, m_real stepSize, int nSize)
{
	vectorn& v=*this;
	if(nSize!=-1)
		setSize(nSize);

	m_real cur=start;
	for(int i=0; i<size(); i++)
	{
		v[i]=cur;
		cur+=stepSize;
	}
}

vectorn& vectorn::linspace(m_real x1, m_real x2, int nSize)
{
	if(nSize!=-1)
		setSize(nSize);

	// simple sampling
	m_real len=x2-x1;

	m_real factor=1.f/((m_real)size()-1);
	for(int i=0; i<size(); i++)
	{
		// position : increases from 0 to 1
		m_real position=((m_real)i)*factor;
		(*this)[i]=position*len+x1;
	}
	return *this;
}

vectorn&  vectorn::uniform(m_real x1, m_real x2, int nSize)
{
	if(nSize!=-1)
		setSize(nSize);

	// simple sampling
	m_real len=x2-x1;
	m_real factor=1.f/((m_real)size());

	for(int i=0; i<size(); i++)
	{
		m_real position=((m_real)(i+(i+1)))*factor/2.f;
		(*this)[i]=position*len+x1;
	}
	return *this;
}

void vectorn::findMin(m_real& min_v, int& min_index,int start,int end) const
{
	const vectorn& v=*this;
	min_v=FLT_MAX;

	min_index=0;
	if(end>size())end=size();
	assert(start>=0&&end>start);
	for(int i=start; i<end; i++)
	{
		if(v[i]<min_v)
		{
			min_v=v[i];
			min_index=i;
		}
	}
}


void  vectorn::findMax(m_real& max_v, int& max_index,int start,int end) const
{
	const vectorn& v=*this;
	max_v=-FLT_MAX;
	if(end>size())end=size();
	assert(start>=0&&end>start);
	for(int i=start; i<end; i++)
	{
		if(v[i]>max_v)
		{
			max_v=v[i];
			max_index=i;
		}
	}
}


void v::linspace(vectorn& out, m_real x1, m_real x2, int nSize)
{
	if(nSize!=-1)
	out.setSize(nSize);

	// simple sampling
	m_real len=x2-x1;

	m_real factor=1.f/((m_real)out.size()-1);
	for(int i=0; i<out.size(); i++)
	{
		// position : increases from 0 to 1
		m_real position=((m_real)i)*factor;
		out[i]=position*len+x1;
	}
}


void v::findMin(const vectorn& v, m_real& min_v, int& min_index) 
{
	min_v=FLT_MAX;

	min_index=0;

	for(int i=0; i<v.size(); i++)
	{
		if(v[i]<min_v)
		{
			min_v=v[i];
			min_index=i;
		}
	}
}

void v::findMax(const vectorn& v, m_real& max_v, int& max_index) 
{
	max_v=-FLT_MAX;

	for(int i=0; i<v.size(); i++)
	{
		if(v[i]>max_v)
		{
			max_v=v[i];
			max_index=i;
		}
	}
}

intvectorn v::colon(int start, int end)
{
	intvectorn c; c.colon(start, end); return c;	
}


vectorn operator-( vectorn const& a)							{ vectorn neg(a);neg.negate();return neg;}
vectorn vectorn::Each(void (*cOP)(m_real&,m_real)) const					{ vectorn c; c.assign(*this); c.each1(cOP,*this); return c;}
vectorn vectorn::Each(m_real (*cOP)(m_real,m_real), vectorn const& b)	const { vectorn c; c.each2(cOP, *this,b); return c;}
void vectorn::setVec3( int start, const vector3& src)	{RANGE_ASSERT(start+3<=size()); for(int i=start; i<start+3; i++) (*this)[i]=src.getValue(i-start);}
void vectorn::setQuater( int start, const quater& src)	{RANGE_ASSERT(start+4<=size());for(int i=start; i<start+4; i++) (*this)[i]=src.getValue(i-start);}
void vectorn::setTransf( int start, const transf& src)	{
	RANGE_ASSERT(start+7<=size());
	for(int i=start; i<start+3; i++) (*this)[i]=src.translation.getValue(i-start);
	start+=3;
	for(int i=start; i<start+4; i++) (*this)[i]=src.rotation.getValue(i-start);
}
vector3 vectorn::toVector3(int startIndex)	const	{RANGE_ASSERT(startIndex+3<=size());vector3 out; for(int i=0; i<3; i++) out[i]=getValue(i+startIndex); return out;};
quater vectorn::toQuater(int startIndex) const		{RANGE_ASSERT(startIndex+4<=size()); quater out; for(int i=0; i<4; i++) out[i]=getValue(i+startIndex); return out;};
transf vectorn::toTransf(int startIndex) const		{
	RANGE_ASSERT(startIndex+7<=size()); 
	transf t;
	{
		vector3& out=t.translation; for(int i=0; i<3; i++) out[i]=getValue(i+startIndex);
	}
	startIndex+=3;
	{
		quater &out=t.rotation; for(int i=0; i<4; i++) out[i]=getValue(i+startIndex); 
	}
	return t;
};

std::ostream& operator<< ( std::ostream& os, const vectorn& u )
{
	return (os << u.output().ptr());
}
std::ostream& operator<< ( std::ostream& os, const intvectorn& u )
{
	return (os << u.output().ptr());
}
