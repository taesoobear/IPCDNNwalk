#include "stdafx.h"
#include "LCBInterpolator.h"
#include "../BaseLib/math/Operator_NR.h"


LCBInterpolator::LCBInterpolator( int dim, int num )
{
	locked = FALSE;
	d=dim;
	n=num;		// Number of the samples
	count=0;
	r=10000;
	alloc();
}

void
LCBInterpolator::clear( void )
{
	locked = FALSE;
	count  = 0;
}

void
LCBInterpolator::alloc( void )
{
	if( locked ) clear();
	if( d==0 || n==0 )	return;
	
	locked = FALSE;
	count  = 0;
	matS.setSize(n, d);

	matA.setSize( n,  d+1 );
	matR.setSize( n,  n );
	r=10000;
}

LCBInterpolator::~LCBInterpolator( void )
{
	clear();
}


void
LCBInterpolator::reset( int dim, int num )
{
	clear();
	d=dim;
	n=num;
	alloc();
}


int
LCBInterpolator::addSample( const vectorn& v )
{
	if( locked ) return count;
	if( count>=n ) return count;
	if( v.getSize() != d ) return count;

	matS.row(count)=v;
	count++;
	if( count==n ) compute();
	return count;
}

void
LCBInterpolator::compute( void )
{
	int i, j;

	for( i=0; i<n; i++ )
		for( j=i+1; j<n; j++ )
		{
			float tempd = matS.row(i).distance(matS.row(j))*2;
			if( tempd< r ) r=tempd;
		}

	static matrixn aa;
	static matrixn aainv;
	static matrixn ww;
	static matrixn aA;
	static matrixn rr;
	static matrixn rrinv;

	aa.setSize( d+1, n );
	ww.setSize( n, n );
	rr.setSize( n, n );

	for( i=0; i<n; i++ )
	{	for( j=0; j<d; j++ )	aa[j][i]=matS[i][j];
		aa[d][i]=1;
	}

	//aa.SVinverse( aainv );
	m::pseudoInverse(aainv,aa);


	for( i=0; i<n; i++ )	for( j=0; j<n; j++ )	ww[i][j]=0;
	for( i=0; i<n; i++ )		ww[i][i]=1;
	matA.mult( ww, aainv );

// reinitialize aa for compute residual
	for( i=0; i<n; i++ )
	{	for( j=0; j<d; j++ )	aa[j][i]=matS[i][j];
		aa[d][i]=1;
	}


	aA.mult( matA, aa );


	ww-=aA;			// now ww is residual matrix q
	for( i=0; i<n; i++ )
	{
		rr[i][i]=1;
		for( j=i+1; j<n; j++ )
			rr[i][j]=rr[j][i]=CBasis( r, matS.row(i).distance(matS.row(j)));
	}
	m::LUinvert(rrinv,rr);
	matR.mult( ww, rrinv );
	locked=TRUE;

}

static vectorn t1;
float
LCBInterpolator::eval( const vectorn& x, int k )
{
	if( !locked ) return 0;
	int i;
	float w = 0;
	t1.setSize( n );

	for( i=0; i<n; i++ )
		w+= matR[k][i]*CBasis( r, x.distance(matS.row(i)) );
	for( i=0; i<d; i++ )
		w+=matA[k][i]*x[i];
	w+=matA[k][d];
	return w;
}

static vectorn t2;
static vectorn t3;

vectorn&
LCBInterpolator::eval( const vectorn& x )
{
	t2.setSize( n );
	t3.setSize( n );

	int i, j;
	if( !locked )
	{
		for( i=0; i<n; i++ ) t3[i]=0; return t3;
	}

	for( i=0; i<n; i++ )
		t2[i] = CBasis( r, matS.row(i).distance(x) );
	t3.multmat(matR,t2);
	for( i=0; i<n; i++ )
	{
		for( j=0; j<d; j++ )	t3[i]+=matA[i][j]*x[j];
		t3[i]+=matA[i][d];
	}
	return t3;
}

	
