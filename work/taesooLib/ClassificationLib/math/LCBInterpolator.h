
#ifndef __LCB_INTERPOLATOR_H__
#define __LCB_INTERPOLATOR_H__

#include "../../BaseLib/math/mathclass.h"

//	CLASS LCBInterpolator
//	to compute the weights of the samples
//	It approximates the weights by linear hyperplane,
//	and fits the residuals by radial basis functions.

//	Here, I would like to give the weight 1 of a sample
//	when the given parameter is same as the sample.

__inline float CBasis( float r, float d )
{
	float x = d/r;
	if( x<0.0f ) x = -x;
	if( x>1.0f ) return 0.0f;

#ifdef CUBIC_BSPLINE
	x*=2.0f;
	if( x<1.0f ) return
		(4.0f         -6.0f*x*x +3.0f*x*x*x)/4.0f;
	else x-=1.0f;
		(1.0f -3.0f*x +3.0f*x*x -1.0f*x*x*x)/4.0f;
#elif defined AS_IN_THIN_PLATE
	return x*x*logf(x);
#else
	return 1.0f       -6.0f*x*x +8.0f*x*x*x -3.0f*x*x*x*x;
#endif
}

typedef vectorn*	ptrV;

class LCBInterpolator
{
public:

// Status variable
	int			locked;

//  Data...

	int			d;		// Dimensionality of the parameter space
	int			n;		// Number of the samples
	int			count;

	matrixn matS;		// Parameter value of each sample

	matrixn		matA;		// Linear approximation weights
	matrixn		matR;		// Radial basis weights

	float		r;		// Radius of radial basis function
						// that is twice the distance to the nearest
						// pair of samples

//	Basic functions
				LCBInterpolator( int dim=0, int num=0 );
				~LCBInterpolator( void );

	void		reset( int dimension, int number );
	int			addSample( const vectorn& v );

	float		eval( const vectorn& x, int k );
	vectorn&	eval( const vectorn& x );

private:
	void		clear( void );
	void		alloc( void );
	void		compute( void );
};

#endif

