/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2012 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file interfaces/matlab/qpOASES_sequence.cpp
 *	\author Hans Joachim Ferreau, Aude Perrin
 *	\version 3.0beta
 *	\date 2007-2012
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function
 *  (variant for QPs with fixed matrices).
 *
 */



#include <qpOASES/SQProblem.hpp>


using namespace qpOASES;

#include <qpOASES_matlab_utils.cpp>


/* global pointer to QP object */
static SQProblem* globalQP = 0;
static SymmetricMatrix* globalQP_H = 0;
static Matrix* globalQP_A = 0;
sparse_int_t *globalQP_Hdiag = 0; 
sparse_int_t *globalQP_Hir = 0; 
sparse_int_t *globalQP_Hjc = 0; 
sparse_int_t *globalQP_Air = 0; 
sparse_int_t *globalQP_Ajc = 0;
real_t *globalQP_Hv = 0;
real_t *globalQP_Av = 0;


/*
 *	a l l o c a t e G l o b a l Q P r o b l e m I n s t a n c e
 */
void allocateGlobalQProblemInstance(	int nV, int nC, Options* options
										)
{
	globalQP = new SQProblem( nV,nC );
	globalQP->setOptions( *options );

	return;
}


/*
 *	d e l e t e G l o b a l Q P r o b l e m I n s t a n c e
 */
void deleteGlobalQProblemInstance( )
{
	if ( globalQP != 0 )
	{
		delete globalQP;
		globalQP = 0;
	}

	return;
}


/*
 *	d e l e t e G l o b a l Q P r o b l e m M a t r i c e s
 */
void deleteGlobalQProblemMatrices( )
{
	if ( globalQP_H != 0 )
	{
		delete globalQP_H;
		globalQP_H = 0;
	}

	if (globalQP_Hv)
	{
		delete[] globalQP_Hv;
		globalQP_Hv = 0;
	}
	
	if (globalQP_Hdiag)
	{
		delete[] globalQP_Hdiag;
		globalQP_Hdiag = 0;
	}
	
	if (globalQP_Hjc)
	{
		delete[] globalQP_Hjc;
		globalQP_Hjc = 0;
	}
	
	if (globalQP_Hir)
	{
		delete[] globalQP_Hir;
		globalQP_Hir = 0;
	}
	
	if ( globalQP_A != 0 )
	{
		delete globalQP_A;
		globalQP_A = 0;
	}

	if (globalQP_Av)
	{
		delete[] globalQP_Av;
		globalQP_Av = 0;
	}
	
	if (globalQP_Ajc)
	{
		delete[] globalQP_Ajc;
		globalQP_Ajc = 0;
	}
	
	if (globalQP_Air)
	{
		delete[] globalQP_Air;
		globalQP_Air = 0;
	}

	return;
}


/*
 *	i n i t
 */
void init(	int nV, int nC,
			SymmetricMatrix *H, real_t* g, Matrix *A,
			const real_t* const lb, const real_t* const ub, const real_t* const lbA, const real_t* const ubA,
			int nWSR, const real_t* const x0, Options* options,
			int nOutputs, mxArray* plhs[]
			)
{
	/* 1) Setup initial QP. */
	allocateGlobalQProblemInstance( nV,nC,options );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	if ( x0 == 0 )
		returnvalue = globalQP->init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
	else
		returnvalue = globalQP->init( H,g,A,lb,ub,lbA,ubA, nWSR,0, x0,0,0,0 );

	/* 3) Assign lhs arguments. */
	obtainOutputs(	0,globalQP,returnvalue,nWSR,
					nOutputs,plhs,0,0 );

	return;
}


/*
 *	h o t s t a r t
 */
void hotstart(	const real_t* const g,
				const real_t* const lb, const real_t* const ub,
				const real_t* const lbA, const real_t* const ubA,
				int nWSR, Options* options,
				int nOutputs, mxArray* plhs[]
				)
{
	/* 1) Solve QP with given options. */
	globalQP->setOptions( *options );
	returnValue returnvalue = globalQP->hotstart( g,lb,ub,lbA,ubA, nWSR,0 );

	/* 2) Assign lhs arguments. */
	obtainOutputs(	0,globalQP,returnvalue,nWSR,
					nOutputs,plhs,0,0 );

	return;
}


/*
 *	h o t s t a r t V M
 */
void hotstartVM(	SymmetricMatrix *H, real_t* g, Matrix *A,
					const real_t* const lb, const real_t* const ub, const real_t* const lbA, const real_t* const ubA,
					int nWSR, Options* options,
					int nOutputs, mxArray* plhs[]
					)
{
	/* 1) Solve QP. */
	globalQP->setOptions( *options );
	returnValue returnvalue = globalQP->hotstart( H,g,A,lb,ub,lbA,ubA, nWSR,0 );

	if (returnvalue != SUCCESSFUL_RETURN)
		mexErrMsgTxt("Hotstart failed.");

	/* 2) Assign lhs arguments. */
	obtainOutputs(	0,globalQP,returnvalue,nWSR,
					nOutputs,plhs,0,0 );

	return;
}


/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	unsigned int i,j;

	/* inputs */
	char* typeString;
	real_t *H_for=0, *H_mem=0, *g=0, *A_for=0, *A_mem=0, *lb=0, *ub=0, *lbA=0, *ubA=0, *x0=0;

	Options options;
	options.printLevel = PL_LOW;
	#ifdef __DEBUG__
	options.printLevel = PL_HIGH;
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	options.printLevel = PL_NONE;
	#endif

	/* dimensions */
	unsigned int nV=0, nC=0;


	/* I) CONSISTENCY CHECKS: */
	/* 1) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs < 6 ) || ( nrhs > 10 ) )
		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

	/* 2) Ensure that first input is a string ... */
	if ( mxIsChar( prhs[0] ) != 1 )
		mexErrMsgTxt( "ERROR (qpOASES): First input argument must be a string!" );

	typeString = (char*) mxGetPr( prhs[0] );

	/*    ... and if so, check if it is an allowed one. */
	if ( ( strcmp( typeString,"i" ) != 0 ) && ( strcmp( typeString,"I" ) != 0 ) &&
		 ( strcmp( typeString,"h" ) != 0 ) && ( strcmp( typeString,"H" ) != 0 ) &&
		 ( strcmp( typeString,"m" ) != 0 ) && ( strcmp( typeString,"M" ) != 0 ) &&
		 ( strcmp( typeString,"e" ) != 0 ) && ( strcmp( typeString,"E" ) != 0 ) &&
		 ( strcmp( typeString,"c" ) != 0 ) && ( strcmp( typeString,"C" ) != 0 ) )
	{
		mexErrMsgTxt( "ERROR (qpOASES): Undefined first input argument!\nType 'help qpOASES_sequence' for further information." );
	}


	/* II) SELECT RESPECTIVE QPOASES FUNCTION CALL: */
	/* 1) Init (without or with initial guess for primal solution). */
	if ( ( strcmp( typeString,"i" ) == 0 ) || ( strcmp( typeString,"I" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );

		if ( ( nrhs < 8 ) || ( nrhs > 10 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

		/* ensure that data is given in real_t precision */
		if ( ( mxIsDouble( prhs[1] ) == 0 ) ||
			 ( mxIsDouble( prhs[2] ) == 0 ) ||
			 ( mxIsDouble( prhs[3] ) == 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in real_t precision!" );


		/* Check inputs dimensions and assign pointers to inputs. */
		nV = mxGetM( prhs[1] ); /* row number of Hessian matrix */
		nC = mxGetM( prhs[3] ); /* row number of constraint matrix */

		if ( ( mxGetN( prhs[1] ) != nV ) || ( ( mxGetN( prhs[3] ) != 0 ) && ( mxGetN( prhs[3] ) != nV ) ) )
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );


		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,5 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,6 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,7 ) != SUCCESSFUL_RETURN )
			return;

		/* default value for nWSR */
		int nWSRin = 5*(nV+nC);

		/* Check whether x0 and options are specified .*/
		if ( nrhs > 8 )
		{
			if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,8 ) != SUCCESSFUL_RETURN )
				return;

			if ( nrhs > 9 )
				if ( ( !mxIsEmpty( prhs[9] ) ) && ( mxIsStruct( prhs[9] ) ) )
					setupOptions( &options,prhs[9],nWSRin );
		}

		deleteGlobalQProblemInstance( );
		deleteGlobalQProblemMatrices( );

		/* check for sparsity */
		if ( mxIsSparse( prhs[1] ) != 0 )
		{
			mwIndex *mat_ir = mxGetIr(prhs[1]);
			mwIndex *mat_jc = mxGetJc(prhs[1]);
			double *v = (double*)mxGetPr(prhs[1]);
			long nfill = 0;
			long i, j;

			/* copy indices to avoid 64/32-bit integer confusion */
			/* also add explicit zeros on diagonal for regularization strategy */
			/* copy values, too */
			globalQP_Hir = new sparse_int_t[mat_jc[nV] + nV];
			globalQP_Hjc = new sparse_int_t[nV+1];
			globalQP_Hv = new real_t[mat_jc[nV] + nV];
			for (j = 0; j < nV; j++) 
			{
				globalQP_Hjc[j] = mat_jc[j] + nfill;
				/* fill up to diagonal */
				for (i = mat_jc[j]; i < mat_jc[j+1] && mat_ir[i] <= j; i++) 
				{
					globalQP_Hir[i + nfill] = mat_ir[i];
					globalQP_Hv[i + nfill] = v[i];
				}
				/* possibly add zero diagonal element */
				if (i >= mat_jc[j+1] || mat_ir[i] > j)
				{
					globalQP_Hir[i + nfill] = j;
					globalQP_Hv[i + nfill] = 0.0;
					nfill++;
				}
				/* fill up to diagonal */
				for (; i < mat_jc[j+1]; i++) 
				{
					globalQP_Hir[i + nfill] = mat_ir[i];
					globalQP_Hv[i + nfill] = v[i];
				}
			}
			globalQP_Hjc[nV] = mat_jc[nV] + nfill;

			SymSparseMat *sH;
			globalQP_H = sH = new SymSparseMat(nV, nV, globalQP_Hir, globalQP_Hjc, globalQP_Hv);
			globalQP_Hdiag = sH->createDiagInfo();
		}
		else
		{
			H_for = (real_t*) mxGetPr( prhs[1] );
			H_mem = new real_t[nV*nV];
			for( uint i=0; i<nV*nV; ++i )
				H_mem[i] = H_for[i];
			globalQP_H = new SymDenseMat( nV, nV, nV, H_mem );
			globalQP_H->doFreeMemory();
		}

		/* Convert constraint matrix A from FORTRAN to C style
		 * (not necessary for H as it should be symmetric!). */
		if ( nC > 0 )
		{
			/* Check for sparsity. */
			if ( mxIsSparse( prhs[3] ) != 0 )
			{
				mwIndex *mat_ir = mxGetIr(prhs[3]);
				mwIndex *mat_jc = mxGetJc(prhs[3]);
				double *v = (double*)mxGetPr(prhs[3]);

				/* copy indices to avoid 64/32-bit integer confusion */
				globalQP_Air = new sparse_int_t[mat_jc[nV]];
				globalQP_Ajc = new sparse_int_t[nV+1];
				for (long i = 0; i < mat_jc[nV]; i++) globalQP_Air[i] = mat_ir[i];
				for (long i = 0; i < nV + 1; i++) globalQP_Ajc[i] = mat_jc[i];

				/* copy values, too */
				globalQP_Av = new real_t[globalQP_Ajc[nV]];
				for (long i = 0; i < globalQP_Ajc[nV]; i++) globalQP_Av[i] = v[i];

				globalQP_A = new SparseMatrix(nC, nV, globalQP_Air, globalQP_Ajc, globalQP_Av);
			}
			else
			{
				/* Convert constraint matrix A from FORTRAN to C style
				* (not necessary for H as it should be symmetric!). */
				A_for = (real_t*) mxGetPr( prhs[3] );
				A_mem = new real_t[nC*nV];
				convertFortranToC( A_for,nV,nC, A_mem );
				globalQP_A = new DenseMatrix(nC, nV, nV, A_mem );
				globalQP_A->doFreeMemory();
			}
		}

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC );

		/* Call qpOASES. */
		init(	nV,nC,
				globalQP_H,g,globalQP_A,
				lb,ub,lbA,ubA,
				nWSRin,x0,&options,
				nlhs,plhs
				);

		return;
	}

	/* 2) Hotstart. */
	if ( ( strcmp( typeString,"h" ) == 0 ) || ( strcmp( typeString,"H" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );

		if ( ( nrhs < 6 ) || ( nrhs > 7 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

		/* has QP been initialised? */
		if ( globalQP == 0 )
			mexErrMsgTxt( "ERROR (qpOASES): QP sequence needs to be initialised first!" );


		/* Check inputs dimensions and assign pointers to inputs. */
		nV = globalQP->getNV( );
		nC = globalQP->getNC( );

		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,1 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,3 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,5 ) != SUCCESSFUL_RETURN )
			return;

		/* default value for nWSR */
		int nWSRin = 5*(nV+nC);

		/* Check whether options are specified .*/
		if ( nrhs == 7 )
			if ( ( !mxIsEmpty( prhs[6] ) ) && ( mxIsStruct( prhs[6] ) ) )
				setupOptions( &options,prhs[6],nWSRin );

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC );

		/* call qpOASES */
		hotstart(	g,
					lb,ub,lbA,ubA,
					nWSRin,&options,
					nlhs,plhs
					);

		return;
	}

	/* 3) Cleanup. */
	if ( ( strcmp( typeString,"c" ) == 0 ) || ( strcmp( typeString,"C" ) == 0 ) )
	{
		/* consistency checks */
		if ( nlhs != 0 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );

		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

		/* Cleanup global SQProblem instance. */
		deleteGlobalQProblemInstance( );
		deleteGlobalQProblemMatrices( );
		
		return;
	}

	/* 4) Solve current equality constrained QP. */
	if ( ( strcmp( typeString,"e" ) == 0 ) || ( strcmp( typeString,"E" ) == 0 ) )
	{
		/* consistency checks */
		if (nlhs != 1 && nlhs != 2)
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );

		if (nrhs != 6 && nrhs != 7)
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );

		/* has QP been initialised? */
		if ( globalQP == 0 )
			mexErrMsgTxt( "ERROR (qpOASES): QP sequence needs to be initialised first!" );


		/* Check inputs dimensions and assign pointers to inputs. */
		int nrhs = mxGetN(prhs[1]);
		nV = globalQP->getNV( );
		nC = globalQP->getNC( );
		real_t *x_out, *y_out;

		if ( smartDimensionCheck( &g,nV,nrhs, BT_FALSE,prhs,1 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,nrhs, BT_TRUE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,nrhs, BT_TRUE,prhs,3 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,nrhs, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,nrhs, BT_TRUE,prhs,5 ) != SUCCESSFUL_RETURN )
			return;

		/* Check whether options are specified .*/
		if ( (  nrhs == 7 ) && ( !mxIsEmpty( prhs[6] ) ) && ( mxIsStruct( prhs[6] ) ) )
		{
			int nWSRin = 5*(nV+nC);
			setupOptions( &options,prhs[6],nWSRin );
			globalQP->setOptions( options );
		}

		/* Create output vectors and assign pointers to them. */
		plhs[0] = mxCreateDoubleMatrix( nV, nrhs, mxREAL );
		x_out = mxGetPr(plhs[0]);
		if (nlhs == 2)
		{
			plhs[1] = mxCreateDoubleMatrix( nV + nC, nrhs, mxREAL );
			y_out = mxGetPr(plhs[1]);
		}
		else
			y_out = new real_t[nV+nC];

		/* Solve equality constrained QP */
		returnValue returnvalue = globalQP->solveCurrentEQP( nrhs,g,lb,ub,lbA,ubA, x_out,y_out );

		if (nlhs < 2)
			delete[] y_out;

		if (returnvalue != SUCCESSFUL_RETURN)
		{
			char msg[200];
			msg[199] = 0;
			snprintf(msg, 199, "ERROR (qpOASES): Couldn't solve current EQP (code %d)!", returnvalue);
			mexErrMsgTxt(msg);
		}

		return;
	}

	/* 5) Modify matrices. */
	if ( ( strcmp( typeString,"m" ) == 0 ) || ( strcmp( typeString,"M" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		if ( ( nrhs < 8 ) || ( nrhs > 10 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		/* ensure that data is given in real_t precision */
		if ( ( mxIsDouble( prhs[1] ) == 0 ) ||
			 ( mxIsDouble( prhs[2] ) == 0 ) ||
			 ( mxIsDouble( prhs[3] ) == 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in real_t precision!" );

		/* Check inputs dimensions and assign pointers to inputs. */
		nV = mxGetM( prhs[1] ); /* row number of Hessian matrix */
		nC = mxGetM( prhs[3] ); /* row number of constraint matrix */

		if ( ( mxGetN( prhs[1] ) != nV ) || ( ( mxGetN( prhs[3] ) != 0 ) && ( mxGetN( prhs[3] ) != nV ) ) )
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );

		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,5 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,6 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,7 ) != SUCCESSFUL_RETURN )
			return;

		/* default value for nWSR */
		int nWSRin = 5*(nV+nC);	

		/* Check whether x0 and options are specified .*/
		if ( nrhs > 9 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		if ( nrhs > 8 )
			if ( ( !mxIsEmpty( prhs[8] ) ) && ( mxIsStruct( prhs[8] ) ) )
				setupOptions( &options,prhs[8],nWSRin );

		deleteGlobalQProblemMatrices( );

		/* check for sparsity */
		if ( mxIsSparse( prhs[1] ) != 0 )
		{
			mwIndex *mat_ir = mxGetIr(prhs[1]);
			mwIndex *mat_jc = mxGetJc(prhs[1]);
			double *v = (double*)mxGetPr(prhs[1]);
			long nfill = 0;
			long i, j;

			/* copy indices to avoid 64/32-bit integer confusion */
			/* also add explicit zeros on diagonal for regularization strategy */
			/* copy values, too */
			globalQP_Hir = new sparse_int_t[mat_jc[nV] + nV];
			globalQP_Hjc = new sparse_int_t[nV+1];
			globalQP_Hv = new real_t[mat_jc[nV] + nV];
			for (j = 0; j < nV; j++) 
			{
				globalQP_Hjc[j] = mat_jc[j] + nfill;
				/* fill up to diagonal */
				for (i = mat_jc[j]; i < mat_jc[j+1] && mat_ir[i] <= j; i++) 
				{
					globalQP_Hir[i + nfill] = mat_ir[i];
					globalQP_Hv[i + nfill] = v[i];
				}
				/* possibly add zero diagonal element */
				if (i >= mat_jc[j+1] || mat_ir[i] > j)
				{
					globalQP_Hir[i + nfill] = j;
					globalQP_Hv[i + nfill] = 0.0;
					nfill++;
				}
				/* fill up to diagonal */
				for (; i < mat_jc[j+1]; i++) 
				{
					globalQP_Hir[i + nfill] = mat_ir[i];
					globalQP_Hv[i + nfill] = v[i];
				}
			}
			globalQP_Hjc[nV] = mat_jc[nV] + nfill;

			SymSparseMat *sH;
			globalQP_H = sH = new SymSparseMat(nV, nV, globalQP_Hir, globalQP_Hjc, globalQP_Hv);
			globalQP_Hdiag = sH->createDiagInfo();
		}
		else
		{
			H_for = (real_t*) mxGetPr( prhs[1] );
			H_mem = new real_t[nV*nV];
			for( uint i=0; i<nV*nV; ++i )
				H_mem[i] = H_for[i];
			globalQP_H = new SymDenseMat( nV, nV, nV, H_mem );
			globalQP_H->doFreeMemory();
		}

		/* Convert constraint matrix A from FORTRAN to C style
		 * (not necessary for H as it should be symmetric!). */
		if ( nC > 0 )
		{
			/* Check for sparsity. */
			if ( mxIsSparse( prhs[3] ) != 0 )
			{
				mwIndex *mat_ir = mxGetIr(prhs[3]);
				mwIndex *mat_jc = mxGetJc(prhs[3]);
				double *v = (double*)mxGetPr(prhs[3]);

				/* copy indices to avoid 64/32-bit integer confusion */
				globalQP_Air = new sparse_int_t[mat_jc[nV]];
				globalQP_Ajc = new sparse_int_t[nV+1];
				for (long i = 0; i < mat_jc[nV]; i++) globalQP_Air[i] = mat_ir[i];
				for (long i = 0; i < nV + 1; i++) globalQP_Ajc[i] = mat_jc[i];

				/* copy values, too */
				globalQP_Av = new real_t[globalQP_Ajc[nV]];
				for (long i = 0; i < globalQP_Ajc[nV]; i++) globalQP_Av[i] = v[i];

				globalQP_A = new SparseMatrix(nC, nV, globalQP_Air, globalQP_Ajc, globalQP_Av);
			}
			else
			{
				/* Convert constraint matrix A from FORTRAN to C style
				* (not necessary for H as it should be symmetric!). */
				A_for = (real_t*) mxGetPr( prhs[3] );
				A_mem = new real_t[nC*nV];
				convertFortranToC( A_for,nV,nC, A_mem );
				globalQP_A = new DenseMatrix(nC, nV, nV, A_mem );
				globalQP_A->doFreeMemory();
			}
		}

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC );

		/* Call qpOASES */
		if ( globalQP == 0 )
			mexErrMsgTxt( "ERROR (qpOASES): QP needs to be initialised first!" );

		if ( ( (int)nV != globalQP->getNV( ) ) || ( (int)nC != globalQP->getNC( ) ) )
			mexErrMsgTxt( "ERROR (qpOASES): QP dimensions must be constant during a sequence!" );

		hotstartVM(	globalQP_H,g,globalQP_A,
					lb,ub,lbA,ubA,
					nWSRin,&options,
					nlhs,plhs
					);

		return;
	}
}

/*
 *	end of file
 */
