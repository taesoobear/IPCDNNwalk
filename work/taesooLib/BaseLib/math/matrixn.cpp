#include "stdafx.h"
#include "mathclass.h"
#include "../utility/tfile.h"
#include "../utility/operatorString.h"
#include "hyperMatrixN.h"


bool matrixn::isnan() const
{
	for (int i=0;i< rows(); i++)
	{
		if (row(i).isnan())
			return true;
	}
	return false;
}
matrixn& matrixn::operator=(const matrixnView& other)		{ assign(other); return *this;}

intmatrixn& intmatrixn::operator=(const intmatrixnView& other)		{ assign(other); return *this;}

intmatrixnView intmatrixn::range(int startRow, int endRow, int startColumn, int endColumn)
{
	return _range<intmatrixnView >(startRow, endRow, startColumn, endColumn);
}

const intmatrixnView intmatrixn::range(int startRow, int endRow, int startColumn, int endColumn) const
{
	return _range<intmatrixnView >(startRow, endRow, startColumn, endColumn);
}

intmatrixnView::intmatrixnView(int* ptr, int nrow, int ncol, int stride2)
:intmatrixn(ptr, nrow, ncol, stride2)
{
}

intmatrixnView::~intmatrixnView()
{
}

int intmatrixn::findRow(intvectorn const& rowv) const
{
	for(int target=0; target<rows(); target++)
	{
		if(row(target)==rowv)
		{
			return target;
		}
	}

	return -1;
}
/*
void intmatrixn::pushBack(const intvectorn& rowVec)
{
	ASSERT(rows()==0 || rowVec.size()==cols());
	resize(rows()+1, rowVec.size());
	setRow(rows()-1, rowVec);
}

void intmatrixn::popBack(intvectorn* pOut)
{
	if(pOut)
		pOut->assign(row(rows()-1));

	resize(rows()-1, cols());
}*/

////////////////////////////////////////////////////////////////////////////////

matrixn ::matrixn(const matrixnView& other)
:_tmat<m_real>()
{
	assign(other);
}

matrixnView::matrixnView(m_real* ptr, int nrow, int ncol, int stride2)
:matrixn(ptr, nrow, ncol, stride2)
{
}

matrixnView::~matrixnView()
{
}
////////////////////////////////////////////////////////////////////////////////
matrixn ::matrixn ( int x, int y)
:_tmat<m_real>()
{
	setSize(x,y);
}

matrixn ::~matrixn ()
{
}
matrixn& matrixn::fromHyperMat(const hypermatrixn& mat)
{
	//!< columnwise concat all pages of mat into one large matrix
	setSize(mat.rows(), mat.cols()*mat.page());

	for(int i=0; i<mat.page(); i++)
	{
		(*this).setValue(0, i*mat.cols(), mat.page(i));
	}

	return *this;
}

////////////////////////////////////////////////////////////////////////////////









bool matrixn::isValid() const
{
	for(int i=0; i<rows() ;i++)
		for(int j=0; j<cols(); j++)
		{
			if(value(i,j)!=value(i,j))
				return false;

			if(value(i,j)>DBL_MAX/2.0)
				return false;
			if(value(i,j)<-DBL_MAX/2.0)
				return false;
		}
	return true;
}

matrixn&  matrixn::assign( vector3N const& a)
{
	matrixn& c=(*this);
	c.setSize(a.rows(), 3);

	for(int i=0; i<a.rows(); i++)
		c.row(i).assign(a.row(i));
	return c;
}

matrixn&  matrixn::assign( quaterN const& a)
{
	matrixn& c=(*this);
	c.setSize(a.rows(), 4);

	for(int i=0; i<a.rows(); i++)
		c.row(i).assign(a.row(i));
	return c;
}





/*
ostream& operator<<( ostream& os, matrixn const& a )
{
    for( int i=0; i< a.rows(); i++ ) os << a.v[i] << endl;
    return os;
}

istream& operator>>( istream& is, matrixn& a )
{
    for( int i=0; i< a.rows(); i++ ) is >> a.v[i];
    return is;
}
*/

void matrixn::normalize(const vectorn &min, const vectorn&max)
{
	ASSERT(cols()==min.size());
	ASSERT(cols()==max.size());
	for(int i=0; i<rows(); i++)
	{
		row(i).normalize(min,max);
	}
}

void matrixn::toVector(vectorn& vec) const
{
	// concat all column vector of this matrix into one large vector.
	vec.setSize(rows()*cols());

	for(int i=0; i<rows(); i++)
	{
		memcpy(&(vec[i*cols()]),operator[](i), sizeof(m_real)*cols() );
	}
}

matrixn& matrixn::fromVector(const vectorn& vec, int column)
{
	int row=vec.size()/column;
	ASSERT(vec.size()%column==0);

	setSize(row, column);

	for(int i=0; i< row; i++)
		for(int j=0; j<column; j++)
			value(i,j)=vec(i*column+j);

	return *this;
}


m_real matrixn::distance(matrixn const& other, Metric* pMetric) const	// matrix 두개 사이의 거리, 정의는 구현 참고
{
	m_real distance=0;
	ASSERT(rows()==other.rows());
	ASSERT(cols()==other.cols());

	for(int i=0; i<rows(); i++)
	{
		distance+=row(i).distance(other.row(i),pMetric);
	}
	return distance;
}
m_real matrixn::op1(CAggregate::aggregateOP eOP) const
{
	CAggregate cOP(eOP);
	m_real cur=cOP.Init();

	for( int i=0; i<rows();i++ )
		for(int j=0; j<cols(); j++)
			cOP.Update(cur, value(i,j));

	return cOP.Final(cur, rows()*cols());
}
matrixnView matrixn::range(int startRow, int endRow, int startColumn, int endColumn)
{
	return _range<matrixnView >(startRow,  endRow,  startColumn,  endColumn);
}


const matrixnView matrixn::range(int startRow, int endRow, int startColumn, int endColumn) const
{
	return _range<matrixnView >( startRow,  endRow,  startColumn,  endColumn);
}

matrixnView matrixn::slice(int srow,int erow,int scol,int ecol)
{
	if( srow<0 ) srow=rows()+srow;
	if( erow<=0 ) erow=rows()+erow;
	if( scol<0 ) scol=cols()+scol;
	if( ecol<=0 ) ecol=cols()+ecol;

	return _range<matrixnView >(srow,  erow,  scol,  ecol);
}
const matrixnView matrixn::slice(int srow,int erow,int scol,int ecol) const
{
	if( srow<0 ) srow=rows()+srow;
	if( erow<=0 ) erow=rows()+erow;
	if( scol<0 ) scol=cols()+scol;
	if( ecol<=0 ) ecol=cols()+ecol;

	return _range<matrixnView >(srow,  erow,  scol,  ecol);
}
matrixn&  matrixn::concatColumns( std::list<matrixn*> matrixes)
{
	int total_columns=0;
	int rows=0;

	std::list<matrixn*>::iterator i;
	for(i=matrixes.begin(); i!=matrixes.end(); i++)
	{
		ASSERT(rows==0 || rows==(*i)->rows());
		rows=(*i)->rows();
		total_columns+=(*i)->cols();
	}

	setSize(rows, total_columns);
	int curr_column=0;
	for( i=matrixes.begin(); i!=matrixes.end(); i++)
	{
		this->setValue(0,curr_column, **i);
		curr_column+=(*i)->cols();
	}

	return *this;
}


/*
void matrixn::bubbles(int nrow, int nbubbles)
{
	// nrow이하는 nbubble만큼 아래로 내린다. 즉 matrix크기가 nbubble만큼 세로로 커지고, 빈칸이 생긴다.
	int prev_row=rows();
	resize(rows()+nbubbles, cols());

	for(int i=prev_row-1; i>=nrow; i--)
		row(i+nbubbles).assign(row(i));

	for(i=nrow; i<nrow+nbubbles; i++)
		row(i).setAllValue(0);
}*/

/*void matrixn::deleteRows(int start, int end)
{
	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다.
	int numRows=end-start;

	for(int i=end; i<rows(); i++)
		row(i-numRows).assign(row(i));

	resize(rows()-numRows, cols());
}*/


void matrixn::pushBack3(const vector3& rowVec)
{
	ASSERT(rows()==0 || cols()==3);
	resize(rows()+1, 3);
	row(rows()-1).assign(rowVec);
}


matrixn&  matrixn::assign( matrix4 const& mat, bool bOnly3x3)
{
	if(bOnly3x3)
		setSize(3,3);
	else
		setSize(4,4);

	matrixn& c=*this;
	c[0][0]=mat._11;
	c[0][1]=mat._12;
	c[0][2]=mat._13;
	c[1][0]=mat._21;
	c[1][1]=mat._22;
	c[1][2]=mat._23;
	c[2][0]=mat._31;
	c[2][1]=mat._32;
	c[2][2]=mat._33;

	if(!bOnly3x3)
	{
		c[3][0]=mat._41;
		c[3][1]=mat._42;
		c[3][2]=mat._43;
		c[3][3]=mat._44;
		c[0][3]=mat._14;
		c[1][3]=mat._24;
		c[2][3]=mat._34;
	}

	return c;
}




matrixn&  matrixn::identity(int n)
{
	matrixn& ret=*this;
	setSize(n,n);
	setAllValue(0.0);
	for(int i=0; i<n; i++)
		ret[i][i]=1.0;
	return ret;
}


TString matrixn::shortOutput() const
{
	TString out;
	out.add("{\n");
	auto& a=*this;
	if (a.rows()<10 )
	{
		for (int i=0; i<a.rows(); i++)
			out.add(" {[%d]=%s\n", i, a.row(i).shortOutput().ptr());
	}
	else
	{
		for (int i=0; i<5; i++)
			out.add(" {[%d]=%s\n", i,a.row(i).shortOutput().ptr());
		out.add("\n ...\n");
		for (int i=a.rows()-5; i< a.rows(); i++)
			out.add(" {[%d]=%s\n", i, a.row(i).shortOutput().ptr());
	}
	out.add("}\n");

	//printf("%s\n", out.ptr());
	return out;
}

TString matrixn::output(const char* formatString, int start, int end) const
{
	TString	id;
	if(end>rows()) end=rows();

	TString temp;
	id+="[\n";

	for(int i=start; i<end; i++)
	{
		temp.format("%d ", i);
		temp+=row(i).output(formatString);
		id+=temp;
		id+="\n";
	}
	id+="]";
	return id;
}



m_real matrixn::trace() const
{
	int nn=MIN(rows(), cols());
	m_real tr=0;
	for(int i=0; i<nn; i++)
		tr+=(*this)[i][i];
	return tr;
}
/*
namespace matrixUtil

void matrixn::eigenVectors(const matrixn& input, vectorn& eigenValues)
{
matrixn symData(input);
vectorn e;
NR_OLD::tred2(symData, eigenValues, e);
NR_OLD::tqli(symData, eigenValues, e);

int dim=eigenValues.size();
matrixn& EVectors=*this;

EVectors.setSize(dim,dim);

// Now each row contains EigenVector
for (int i = 0; i < dim; i++) {
for (int j = 0; j < dim; j++) {
EVectors[i][j] = symData[j][i];
}
}
}


*/

#include "Operator.h"
matrixn&  matrixn::resample(matrixn const& mat, int numSample)
{
#if 0
	setSize(numSample, mat.cols());

	intvectorn iv;
	iv.makeSamplingIndex(mat.rows(), numSample);

	for(int i=0; i<numSample; i++)
	{
		(*this)[i]=mat[iv[i]];
	//	(*this)=mat;
	}
#else
	setSize(numSample, mat.cols());

	m_real f;
	m_real fl;
	m_real t;
	int ifl;
	for(int i=0; i<numSample; i++)
	{
		f=sop::map(i, 0, numSample-1, 0, mat.rows()-1);
		fl=floor(f);
		t=(f-fl);
		ifl=int(fl);
		if(ifl<mat.rows()-1)
			v::interpolate(row(i).lval(), t, mat.row(ifl), mat.row(ifl+1));
		else
			row(i).assign(mat.row(ifl));
	}
#endif
	return *this;
}
/*
matrixn&  matrixn::linearResample(matrixn const& mat, vectorn const& samplingIndex)
{
	int numSample=samplingIndex.size();
	setSize(numSample, mat.cols());

	m_real fl;
	m_real t;
	int ifl;
	for(int i=0; i<numSample; i++)
	{
		fl=floor(samplingIndex[i]);
		t=(samplingIndex[i]-fl);
		ifl=int(fl);
		(*this)[i].interpolate(mat[ifl], mat[ifl+1], t);
	}
	return *this;
}

void matrixn::resample(int nSample)
{
	matrixn temp(*this);
	resample(temp, nSample);
}

void
matrixn::LUdecompose( int* index )
{
assert( this->rows() == this->cols() );

int n = this->rows();
int i, j, k, imax;
m_real big, dum, sum, temp;

static vectorn vv; vv.setSize( n );
matrixn &a = (*this);

for ( i=0; i<n; i++ )
{
big = 0.0f;
for ( j=0; j<n; j++ )
if ((temp = fabs(a[i][j])) > big)
big = temp;

if (big == 0.0f)
{
NR_OLD::error("Singular matrix in routine LUdecompose" );
assert( FALSE );
}

vv[i] = 1.0f / big;
}

for ( j=0; j<n; j++ )
{
for ( i=0; i<j; i++ )
{
sum = a[i][j];
for ( k=0; k<i; k++ )
sum -= a[i][k] * a[k][j];
a[i][j] = sum;
}

big = 0.0;
for ( i=j; i<n; i++ )
{
sum = a[i][j];
for ( k=0; k<j; k++ )
sum -= a[i][k] * a[k][j];
a[i][j] = sum;
if ((dum = vv[i] * fabs(sum)) >= big)
{
big = dum;
imax = i;
}
}

if ( j!=imax )
{
for ( k=0; k<n; k++ )
{
dum = a[imax][k];
a[imax][k] = a[j][k];
a[j][k] = dum;
}
vv[imax] = vv[j];
}

index[j] = imax;
if (a[j][j] == 0.0f) a[j][j] = (m_real)EPS;

if ( j!=n )
{
dum = 1.0f / a[j][j];
for ( i=j+1; i<n; i++ )
a[i][j] *= dum;
}
}
}

void
matrixn::LUsubstitute( int* index, vectorn &b )
{
assert( this->rows() == this->cols() );

int n = this->rows();
matrixn &a = (*this);

int i, ii = -1, ip, j;
m_real sum;

for ( i=0; i<n; i++ )
{
ip = index[i];
sum = b[ip];
b[ip] = b[i];

if (ii>-1)
for ( j=ii; j<i; j++ )
sum -= a[i][j] * b[j];
else
if (sum)
ii = i;

b[i] = sum;
}

for ( i=n-1; i>=0; i-- )
{
sum = b[i];
for ( j=i+1; j<n; j++ )
sum -= a[i][j] * b[j];
b[i] = sum / a[i][i];
}
}

void
matrixn::LUinverse( matrixn &mat )
{
assert( this->rows() == this->cols() );

int n = this->rows();

static int* index;
static int index_count = 0;
if ( index_count<n )
{
if ( index_count>0 ) delete[] index;
index_count = n;
if ( index_count>0 ) index = new int[index_count];
}

static vectorn b;
b.setSize( n );

mat.setSize( n, n );

LUdecompose( index );
for( int j=0; j<n; j++ )
{
for( int i=0; i<n; i++ ) b[i] = 0;
b[j] = 1.0;

LUsubstitute( index, b );

for( i=0; i<n; i++ )
mat[i][j] = b[i];
}
}

matrixn&
matrixn::mergeUpDown( matrixn const& a, matrixn const& b )
{
assert( a.cols()==b.cols() );
matrixn &c = (*this);
c.setSize( a.rows()+b.rows(), a.cols() );

for( int j=0; j<a.cols(); j++ )
{
for( int i=0; i<a.rows(); i++ )
c[i][j] = a[i][j];

for( i=0; i<b.rows(); i++ )
c[i+a.rows()][j] = b[i][j];
}

return c;
}

matrixn&
matrixn::mergeLeftRight( matrixn const& a, matrixn const& b )
{
assert( a.rows()==b.rows() );
matrixn &c = (*this);
c.setSize( a.rows(), a.cols()+b.cols() );

for( int i=0; i<a.rows(); i++ )
{
for( int j=0; j<a.cols(); j++ )
c[i][j] = a[i][j];

for( j=0; j<b.cols(); j++ )
c[i][j+a.cols()] = b[i][j];
}

return c;
}

void
matrixn::splitUpDown( matrixn& a, matrixn& b )
{
assert( this->rows()%2 == 0 );
matrixn &c = (*this);
a.setSize( c.rows()/2, c.cols() );
b.setSize( c.rows()/2, c.cols() );

for( int j=0; j<a.cols(); j++ )
{
for( int i=0; i<a.rows(); i++ )
a[i][j] = c[i][j];

for( i=0; i<b.rows(); i++ )
b[i][j] = c[i+a.rows()][j];
}
}

void
matrixn::splitLeftRight( matrixn& a, matrixn& b )
{
assert( this->cols()%2 == 0 );
matrixn &c = (*this);
a.setSize( c.rows(), c.cols()/2 );
b.setSize( c.rows(), c.cols()/2 );

for( int i=0; i<a.rows(); i++ )
{
for( int j=0; j<a.cols(); j++ )
a[i][j] = b[i][j];

for( j=0; j<b.cols(); j++ )
b[i][j] = c[i][j+a.cols()];
}
}

void
matrixn::splitUpDown( matrixn& a, matrixn& b, int num )
{
assert( this->rows()>num );
matrixn &c = (*this);
a.setSize( num, c.cols() );
b.setSize( c.rows()-num, c.cols() );

for( int j=0; j<a.cols(); j++ )
{
for( int i=0; i<a.rows(); i++ )
a[i][j] = c[i][j];

for( i=0; i<b.rows(); i++ )
b[i][j] = c[i+a.rows()][j];
}
}

void
matrixn::splitLeftRight( matrixn& a, matrixn& b, int num )
{
assert( this->cols()>num );
matrixn &c = (*this);
a.setSize( c.rows(), num );
b.setSize( c.rows(), c.cols()-num );

for( int i=0; i<a.rows(); i++ )
{
for( int j=0; j<a.cols(); j++ )
a[i][j] = b[i][j];

for( j=0; j<b.cols(); j++ )
b[i][j] = c[i][j+a.cols()];
}
}


*/





matrixn operator+( matrixn const& a, matrixn const& b)	{matrixn c; c.add(a,b); return c;};
matrixn operator-( matrixn const& a, matrixn const& b)	{matrixn c; c.subtract(a,b); return c;};
matrixn operator*( matrixn const& a, matrixn const& b)	{matrixn c; c.mult(a,b); return c;};
matrixn operator/( matrixn  const& a, m_real b)		{matrixn  c; c.mult(a,1.0/b); return c;};
matrixn operator*( matrixn const& a, m_real b )		{matrixn c;c.mult(a,b);return c;}
matrixn operator*( m_real b , matrixn const& a)		{matrixn c;c.mult(a,b);return c;}

quaterNView matrixn::toQuaterN() const
{
	return _column<quaterNView >(0);
}
vector3NView matrixn::toVector3N() const
{
	return _column<vector3NView >(0);
}

void matrixn::sampleRow( m_real criticalTime, vectorn& out) const
{
matrixn const& in=*this;
				out.setSize(in.cols());
				//!< 0 <=criticalTime<= numFrames()-1
				// float 0 이 정확하게 integer 0에 mapping된다.
				int a;
				float t;

				a=(int)floor(criticalTime);
				t=criticalTime-(float)a;

				if(t<0.005)
					out=in.row(a);
				else if(t>0.995)
					out=in.row(a+1);
				else {
					if(a<0)
						v::interpolate(out, t-1.0, in.row(a+1), in.row(a+2));
					else if(a+1>=in.rows())
						v::interpolate(out, t+1.0, in.row(a-1), in.row(a));
					else
						v::interpolate(out, t, in.row(a), in.row(a+1));
					}
				}
std::ostream& operator<< ( std::ostream& os, const matrixn& u )
{
	return (os << u.output().ptr());
}
matrixn matrixn::derivative(double frame_rate) const
{
   matrixn dsrc;
   
   dsrc.setSize(rows(), cols());
   
   for (int i=1; i< rows()-1; i++)
   {
	   dsrc.row(i).sub(row(i+1),row(i-1));
	   dsrc.row(i)*=(frame_rate/2.0);
   }
   
   // fill in empty rows
   dsrc.row(0).assign(dsrc.row(1));
   dsrc.row(dsrc.rows()-1).assign(dsrc.row(dsrc.rows()-2));
   return dsrc;
}
matrixn matrixn::derivative_forward(double frame_rate) const
{
   matrixn dsrc;
   
   dsrc.setSize(rows(), cols());
   
   for (int i=0; i< rows()-1; i++)
   {
	   dsrc.row(i).sub(row(i+1),row(i));
	   dsrc.row(i)*=frame_rate;
   }
   
   // fill in empty rows
   dsrc.row(dsrc.rows()-1).assign(dsrc.row(dsrc.rows()-2));
   return dsrc;
}
