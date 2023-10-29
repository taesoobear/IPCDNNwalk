
#include <qpOASES.hpp>
#include "stdafx.h"
double solve_quadprog_using_qpOASES(const matrixn & G,  const vectorn & g0,  
                      const matrixn & CE, const vectorn & ce0,  
                      const matrixn & CI, const vectorn & ci0, 
                      vectorn & x, bool g0_negative)
{

	USING_NAMESPACE_QPOASES
	
	/* Setup data of QP. */
	real_t *H=new real_t[G.rows()*G.cols()];
	int ii=0;
	for(int i=0; i<G.rows(); i++)
		for(int j=0; j<G.cols(); j++)
			H[ii++]=G[i][j];

	//real_t *g=&g0(0);
	real_t *g=new real_t[g0.size()];
	for (int i=0; i<g0.size(); i++) g[i]=g0[i]*-1.0;
	int ndim=G.rows();
	int numCon=CE.rows()+CI.rows();

	real_t *A=new real_t[ndim*numCon];
	real_t *lbA= new real_t[numCon];
	real_t *ubA= new real_t[numCon];
	real_t *lb= new real_t[ndim];
	real_t *ub= new real_t[ndim];
	x.setSize(ndim);
	for(int i=0; i<CE.rows(); i++)
	{
		for (int j=0; j<ndim; j++)
		{
			A[i*ndim+j]=CE(i,j);
			lbA[i]=ubA[i]=ce0[i]*-1;
		}
	}
	int offset=CE.rows();
	const double dbl_max=1e10;
	for(int i=0; i<CI.rows(); i++)
	{
		for (int j=0; j<ndim; j++)
		{
			A[(i+offset)*ndim+j]=CI(i,j);
			lbA[(i+offset)]=ci0[i]*-1;
			ubA[(i+offset)]=dbl_max;
		}
	}
	for (int j=0; j<ndim; j++)
	{
		lb[j]=-dbl_max;
		ub[j]=dbl_max;
	}
	SQProblem example(G.rows(), numCon);
	Options myOptions;
	myOptions.setToReliable( );
	myOptions.printLevel = PL_LOW;
	example.setOptions(myOptions);

	/* Solve first QP ... */
	int nWSR = 10000;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );

#if 0
	if (false){
		SolutionAnalysis analyser;
		/* ... and analyse it. */
		real_t maxKKTviolation;
		analyser.getMaxKKTviolation( &example, maxKKTviolation );
		printf( "maxKKTviolation: %e\n", maxKKTviolation );
	}
#endif
	example.getPrimalSolution(&x(0));
	delete [] H;
	delete [] A;
	delete [] lbA;
	delete [] ubA;
	delete [] lb;
	delete [] ub;

	return 0;
}
