#include "stdafx.h"
#include <stdlib.h>   
#include <math.h>
#include "tpros_nrutil.h"
#include "rand2.h"
#include "mynr.h"
#include "macopt.h"
#include "../BaseLib/math/mathclass.h"
#include "tpros.h"
#include "../BaseLib/utility/util.h"
#include "../BaseLib/utility/tfile.h"

/* ========================= Subroutines ========================= */

using namespace tpros_nrutil;

#define VERSION_NO "6.3"

tpros::tpros(const char* option, int dim, int ntdata)
{
	int argc;
	char** argv;
	ParseCommandLineString(option,argc, argv);

	_init(argc, argv, dim, ntdata);
}

tpros::tpros(int argc, char** argv, int dim, int ntdata)
{

	_init(argc, argv,dim, ntdata);
}
void tpros::_init(int argc, char** argv, int dim, int ntdata)
{
	mInputDimension=dim;
	mNumTrainingData=ntdata;

	prior_strength_default = 0.05 ;
	prior_length_default =   1.0 ;
	prior_vertical_default = 1.0 ; /* s.d. */
	prior_fractional_noise_default = 0.1 ; /* s.d. */
	prior_fractional_theta2_default = 0.1 ;
	prior_linear_default = 1.0 ;

	C_periodic = 0 ; /* [10] */  /*  Periodic covariance function */
	C_linear= 0 ; /* 24 */  /*  Use linear term in covariance function */

	noiseprior = 0 ;  /* was flag[19] */  /*  (=1) Use (Inverse Gamma/Gaussian) prior on noise m,sd (=2) m,N */
	lengthprior = 0 ; /* was flag[18] */  /*  (=1) Use (Gamma/Gaussian) prior on length scales m,sd (=2) m,N */
	theta1prior = 0 ; /* was flag[21] */  /*  (=1) Use Inverse Gamma prior on theta1 m,sd (=2) m,N  */
	theta2prior = 0 ; /* was flag[22] */  /*  (=1) Use Inverse Gamma prior on theta2 m,sd (=2) m,N */
	theta0prior = 0 ; /* was 16 */  /*  (=1) Use Gaussian prior on variable mean m,sd */
	linearprior = 0 ; /* 26 */  /*  (=1) Use Gaussian prior on linear term default  */

	optimize_lengthscales = 0 ; /* was [1] */  /*  Optimize sig_k */
	optimize_theta0 = 0 ; /* was [15] */  /*  optimize theta0 */
	optimize_theta1 = 0 ; /* was [2] */  /*  Optimize theta1 */
	optimize_theta2 = 0 ; /* was [3] */   /*  Optimize theta2 */
	optimize_noise = 0 ;  /* was [4] */  /*  Optimize noise model */
	optimize_linear = 0 ; /* 25 */  /*  Optimize linear term hyperparameters  */
	use_poly_length_scales = 0 ; /* was [30] */  /*  Use polynomial length scales  */
	OPT_checkgrad = 0 ; /* 20 */  /*  Perform maccheckgrad */
	OPT_use_cg = 0 ; /* was [5] */  /*  Use CG inversion for OPT */
	OPT_tridiag_CG = 0 ; /* 33 */ /*  (=0) double-CG method (=1) Tridiag CG routine (only works for OPT_use_cg == 1) */
	OPT_say_evidence = 0 ; /* 28 */   /*  Calculate evidence after optimization */
	OPT_say_train_e = 0 ; /* 29 */  /*  Calculate training error  */

	INT_load_targets = 0 ; /* 23 */  /*  load in interpolation targets */
	INT_use_cg = 0 ; /* was [8] */  /*  Use CG inversion for INT */
	INT_LUi = 0 ; /* was [9] */  /*  (=1) Use LUi for INT (=0) Use LUd for INT */
	INT_find_eb = 0 ; /* was [6] */  /*  Calculate error bars for INT */
	INT_find_sigma_nu = 0 ; /* [7] */  /*  Calculate noise level for INT */
	INT_include_inputs = 1 ; /* include input values in output file */

	hyp_out = 0 ;    /* [11] don't confuse with hypoutfile */  /*  output hyperparameters */
	hyp_in = 0 ;    /* [13] */  /*  load in hyperparameters */
	write_Cinv_t = 0 ; /* 27 */  /*  Output C_inv[][]*t[] vector to file */
	write_hdump  = 0 ; /* 34 */ /*  save hyperparameters every iteration (to hypoutfile.dump) */
	do_samples = 0 ; /* 35 */ /*  generate random samples from GP (only a c_line option) */
	do_INT = 0 ; /* [12] */  /*  perform interpolation */
	do_OPT = 0 ; /* 14 */  /*  perform optimisation (=0) */
	do_SEARCH = 0 ; /* 36 */ /*  perform k_x optimization */
	SEARCH_max = 0 ; /* 37*/ /*  (=0) minimization (=1) maximization */
	calc_Hessian = 0 ; /* 31 */  /*  Calculate Hessian of coefficients for the length scales */
	training_noise_in_datafile = 0 ; /* 32 */  /*  Load in known training noise levels (in datafile) */

	noise_free_predictions = 0 ;
	rescaling_permitted = 0 ;

	opt_itmax = 0 ;

	max_beta = 1.0 ;
	max_gam  = 1.0 ;

	max_itmax = 0 ; /* maximum number of iterations to do */


	{
		int    i,j,k,l ;
		int    tint;



		//double d;

		FILE   *fp1;

		/* Print out options if argc is insufficient */

		if (argc < 2){
			print_usage () ; 
			Msg::error("tpros error"); 
		}

		strcpy(specfile,argv[1]);


		/* ======================= Set up Section ======================= */

		/* Read in spec file */

		c.D    = dvector(1,4);

		for(i=1; i <= 4; ++i){
			c.D[i] = 0.0;
		}

		max_kx = NULL;
		max_include = NULL;

		/* Set parameters to defaults */
		defaults(c.D);
		/* Read in spec file */
		readInSpecfile(specfile,c.D);

		/* Read command line arguments */
		c_line(argc,argv,c.D);


	}

}


void tpros::C_hyper( void ) /* this master optimization subroutine 
							optimizes the hyperparameters */
{
	int         i,j,k;
	int         tint,tint2;
	int         nlengthscales = mInputDimension*ngauss*npoly;
	double      temp1 , temp2 ;
	double      *w;
	macopt_args macp;
	gq_args     param;
	param.all=this;

	nw = 0;
	if( optimize_lengthscales == 1 ) nw += nlengthscales;
	if( optimize_theta1 == 1 ) nw += ngauss;
	if( optimize_theta2 == 1 ) nw += 1;
	if( optimize_noise == 1 ) nw += nwb;
	if( optimize_linear == 1 ) nw += mInputDimension;
	if( optimize_theta0 == 1 ) nw += 1;

	kth = imatrix(1,nw,1,5);
	w   = dvector(1,nw);

	/* Set up kth[][] matrix (for explanation of kth, search backwards for kth) */

	tint = 0;
	if( optimize_lengthscales == 1 ){
		for(i=1; i <= ngauss; ++i){
			for(j=1; j <= mInputDimension; ++j){
				for(k=1; k <= npoly; ++k){
					tint2 = (i-1)*mInputDimension*npoly + (j-1)*npoly + k;
					kth[tint2+tint][1] = 1;
					kth[tint2+tint][2] = tint2;
					kth[tint2+tint][3] = i;
					kth[tint2+tint][4] = j;
					kth[tint2+tint][5] = k;
				}
			}
		}
		tint += nlengthscales;
	}
	if( optimize_theta1 == 1 ){
		for(i=1; i <= ngauss; ++i){
			kth[tint+i][1] = 2;
			kth[tint+i][2] = i+( nlengthscales ) ;
			kth[tint+i][3] = i;
		}
		tint += ngauss;
	}
	if( optimize_theta2 == 1 ){
		kth[tint+1][1] = 3;
		kth[tint+1][2] = (nlengthscales )+ngauss+1;
		tint += 1;
	}
	if( optimize_noise == 1 ){
		if( nhidden > 1 ){
			for(i=1; i <= nhidden; ++i){
				for(j=1; j <= mInputDimension+1; ++j){
					tint2 = (i-1)*(mInputDimension+1) + j;
					kth[tint+tint2][1] = 4;
					kth[tint+tint2][2] = tint2+(nlengthscales )+ngauss+1;
					kth[tint+tint2][3] = 1;
					kth[tint+tint2][4] = j;
					kth[tint+tint2][5] = i;
				}
			}
			for(i=1; i <= nhidden+1; ++i){
				kth[tint+((mInputDimension+1)*nhidden)+i][1] = 4;
				kth[tint+((mInputDimension+1)*nhidden)+i][2] = i+(nlengthscales )+ngauss+1+((mInputDimension+1)*nhidden);
				kth[tint+((mInputDimension+1)*nhidden)+i][3] = 2;
				kth[tint+((mInputDimension+1)*nhidden)+i][4] = i;
			}
		}
		if( nhidden == 1 ){
			kth[tint+1][1] = 4;
			kth[tint+1][2] = 1+(nlengthscales )+ngauss+1;
			kth[tint+1][3] = 1;
			kth[tint+1][4] = 1;      
		}
		tint += nwb;
	}
	if( optimize_linear == 1 ){
		for(i=1; i <= nlin; ++i){
			kth[tint+i][1] = 5;
			kth[tint+i][2] = i+(nlengthscales )+ngauss+1+nwb;
			kth[tint+i][3] = i;
		}
		tint += nlin;
	}
	if( optimize_theta0 == 1 ){
		kth[tint+1][1] = 6;
		kth[tint+1][2] = (nlengthscales )+ngauss+1+nwb+nlin+1;
		tint += 1;
	}

	w_con_in(w); /* load initial conditions into w */

	macopt_defaults( &macp );

	macp.tol     = mac_h_tol;
	if ( opt_itmax > 0 ) macp.itmax   = opt_itmax;
	macp.rich    = 2;
	macp.verbose = verbose;
	macp.end_if_small_step = mac_h_style ;

	macp.do_newitfunc = 1 ;      /* arrange for a subroutine to happen each itn*/
	macp.newitfunc = &T_return ; /* setting the function to be performed to
								 T_return */
	macp.newitfuncarg = (void *)(&param) ; 


	/* set up random vectors if necessary */

	if( OPT_use_cg == 1 ){

		for(k=1; k <= ntrace; ++k){
			temp1 = 0.0;
			for(i=1; i <= mNumTrainingData; ++i){
				Mran[k][i] = rand_gaussian( );
				temp1 += pow(Mran[k][i],2.0);
			}    
			temp2 = sqrt((double)(mNumTrainingData)/temp1);
			for(i=1; i <= mNumTrainingData; ++i){
				Mran[k][i] = Mran[k][i]*temp2;
			}    
		}

	}

	if( OPT_checkgrad == 1 ){
		maccheckgrad( w , nw , mac_ep , mac_func , (void *)(&param) ,
			mac_dif , (void *)(&param) , 0 );
	}

	macoptII( w , nw , mac_dif , (void *)(&param) , &macp );

	if( OPT_checkgrad == 1 ){
		maccheckgrad( w , nw , mac_ep , mac_func , (void *)(&param) ,
			mac_dif , (void *)(&param) , 0 );
	}

	w_con_out(w);

	free_imatrix(kth,1,nw,1,5);
	free_dvector(w,1,nw);
}

double tpros::mac_func(double *w,void *arg) /* the (-) log posterior
											which we wish to minimize */
{
	return ((gq_args*)arg)->all->mac_func_member(w);
}

double tpros::mac_func_member(double* w)
{
	int i;
	double tot,temp1;
	double *tempa,*tempb;

	tempa = dvector(1,mNumTrainingData);
	tempb = dvector(1,mNumTrainingData);

	w_con_out(w);

	/* Set up the noise polynomials again */
	if( nhidden > 1 ){
		for(i=1; i <= mNumTrainingData; ++i){
			L_calc(i);
		}
	}

	mac_inter_C( ); /* computes the covariance matrix in A */
	tot = prior(w);

	/* We only use LU_apply routine here. This is because I can't be
	bothered to code up the Skilling log_det stuff */

	for(i=1; i <= mNumTrainingData; ++i){
		tempb[i] = t[i] - HYP_THETA_0;
	}

	LU_apply(C,tempb,tempa,mNumTrainingData); /* compute C^{-1}t */

	temp1 = 0.0;
	for(i=1; i <= mNumTrainingData; ++i){
		temp1 += tempa[i]*tempb[i];  /* t C^{-1} t */
	}
	temp1 = temp1 * 0.5;

	tot += temp1 + 0.5*log_det; 

	/* add in jacobian element for the transformation of the variables */

	for(i=1; i <= nw; ++i){
		if( kth[i][1] == 1 && npoly == 1 ) tot += -w[i];
		else if( kth[i][1] == 2 ) tot += -w[i];
		else if( kth[i][1] == 3 ) tot += -w[i];
		else if( kth[i][1] == 4 && nhidden == 1 ) tot += -w[i];
	}

	free_dvector(tempa,1,mNumTrainingData);

	return tot;
}

void tpros::mac_dif(double *w,double *dM,void *arg) /* crucial routine: 
													evaluates the derivative 
													wrt hyp */
{
	return ((gq_args*)arg)->all->mac_dif_member(w, dM);
}

void tpros::mac_dif_member(double *w,double *dM)
{
	/* crucial routine: */
	int    i,j,k,l;
	double tr,log_data;
	double temp1;
	double *tempa,*tempb,*tempc;

	tempa  = dvector(1,mNumTrainingData);
	tempb  = dvector(1,mNumTrainingData);
	tempc  = dvector(1,mNumTrainingData);

	w_con_out(w);

	/* Set up the noise polynomials again */
	if( nhidden > 1 ){
		for(i=1; i <= mNumTrainingData; ++i){
			L_calc(i);
		}
	}

	for(i=1; i <= mNumTrainingData; ++i){
		tempa[i] = t[i] - HYP_THETA_0;
	}

	if( verbose == 2 ){
		printf("hyp : ");
		for(i=1; i <= N_including_NOISE+nlin; ++i){
			printf("%f ",hyp[i]);
		}
		printf("\n");
	}

	mac_inter_C( ); /* computes the covariance matrix in A */

	dif_prior(dM,w);

	/* jacobian elements to deal with the exponential representation 
	of some of the hyperparameters */

	for(i=1; i <= nw; ++i){
		if( kth[i][1] == 1 && npoly == 1 ) dM[i] += -1.0;
		else if( kth[i][1] == 2 ) dM[i] += -1.0;
		else if( kth[i][1] == 3 ) dM[i] += -1.0;
		else if( kth[i][1] == 4 && nhidden == 1 ) dM[i] += -1.0;
	}

	if( OPT_use_cg == 1 ){

		/* CG version - first we need to loop over a few random vectors */

		for(k=1; k <= ntrace; ++k){
			CG_invert(tempb,Mran[k],CG_opt); 
			for(i=1; i <= mNumTrainingData; ++i){
				Mtemp[k][i] = tempb[i];
			}
		}

		/* calculate C_inv * (t-t0) */         

		CG_invert(tempb,tempa,CG_opt);

		/* Loop over all hyperparameters */

		for(l=1; l <= nw; ++l){

			if( kth[l][1] == 6 ){

				/* Calculate n * C_inv * (t-t0) */

				for(i=1; i <= mNumTrainingData; ++i){
					dM[l] -= tempb[i];
				}

			} else {

				mac_inter_dif(w,l);

				/* Now calculate t-t0 * C_inv * dC/d(param) * C_inv * t-t0 */

				log_data = 0.0;
				temp1 = 0.0;
				for(i=1; i <= mNumTrainingData; ++i){
					for(j=1; j <= mNumTrainingData; ++j){
						log_data += tempb[i] * tempb[j] * dif_C[i][j];
					}
				}

				/* Now calculate the trace bit */

				/* first we need to loop over the random vectors */

				tr    = 0.0;
				for(k=1; k <= ntrace; ++k){

					/* calculate r * C_inv * dC/d(param) * r */

					/* We must introduce a minus sign to take into account
					that this is a minimisation not maximisation process.
					We must also introduce a factor of 2.0 to distinguish
					between the required rC_invr and the bounds */

					tr  += trace(Mtemp[k],Mran[k]);

				}

				tr = tr/(double)(ntrace);
				dM[l] += -0.5 * (log_data - tr);

			}

		}  

	}
	else if( OPT_use_cg == 0 ){    

		LU_invert(C,C_inv,mNumTrainingData);

		/* LU version - Loop over all hyperparameters */

		for(l=1; l <= nw; ++l){

			if( kth[l][1] == 6 ){

				/* Calculate n * C_inv * (t-t0) */

				for(i=1; i <= mNumTrainingData; ++i){
					tempb[i] = 0.0;
					for(j=1; j <= mNumTrainingData; ++j){
						tempb[i] += C_inv[i][j]*tempa[j];
					}
				}

				for(i=1; i <= mNumTrainingData; ++i){
					dM[l] -= tempb[i];
				}

			}	else {

				mac_inter_dif(w,l);

				/* Calculate t * C_inv * dC/d(param) * C_inv * t */

				log_data = 0.0;
				for(i=1; i <= mNumTrainingData; ++i){
					tempb[i] = 0.0;
					for(j=1; j <= mNumTrainingData; ++j){
						tempb[i] += C_inv[i][j]*tempa[j];
					}
				}
				for(i=1; i <= mNumTrainingData; ++i){
					tempc[i] = 0.0;
					for(j=1; j <= mNumTrainingData; ++j){
						tempc[i] += dif_C[i][j]*tempb[j];
					}
				}
				for(i=1; i <= mNumTrainingData; ++i){
					tempb[i] = 0.0;
					for(j=1; j <= mNumTrainingData; ++j){
						tempb[i] += C_inv[i][j]*tempc[j];
					}
				}
				for(i=1; i <= mNumTrainingData; ++i){	
					log_data += tempb[i]*tempa[i];
				}

				/* Now calculate the trace bit */

				tr = 0.0;
				for(i=1; i <= mNumTrainingData; ++i){
					for(j=1; j <= mNumTrainingData; ++j){
						tr += C_inv[i][j] * dif_C[j][i];
					}
				}

				dM[l] += -0.5 * (log_data - tr);   

			}  
		}

	}

	free_dvector(tempa,1,mNumTrainingData);
	free_dvector(tempb,1,mNumTrainingData);
	free_dvector(tempc,1,mNumTrainingData);
}

void tpros::load_hyp(char *file)
{
	int    i;
	char   junk[200];
	FILE   *fp1;

	fp1 = fopen( file , "r" );
	if( !fp1 ) fprintf( stderr, "No such file : %s\n",file ), Msg::error("tpros error");

	for(i=1; i <= N_HYPS; ++i){
		fscanf(fp1,"%s",junk);
		if( strcmp(junk,"#") == 0 ){
			fgets(junk,50,fp1);
			i -= 1;
		} else {
			hyp[i] = atof(junk);
		}
	}

	/* convert length scales to exponential representation */

	for(i=1; i <= N_LENGTH_SCALES; ++i){
		hyp[i] = log(hyp[i]);
	}

	fclose( fp1 );
}


void tpros::save_hyp(char *file)
{
	int    i,j,k;
	int    tint;
	FILE   *fp1;

	fp1 = fopen( file , "w" );
	if( !fp1 ) fprintf( stderr, "No such file : %s\n",file ), Msg::error("tpros error");

	for(i=1; i <= ngauss; ++i){
		fprintf(fp1,"# Length Scales for Gaussian no.%d\n",i);
		for(j=1; j <= mInputDimension; ++j){
			if( use_poly_length_scales == 1 ) fprintf(fp1,"# Polynomial coefficients for input no.%d\n",j);
			for(k=1; k <= npoly; ++k){
				tint = (i-1)*mInputDimension*npoly + (j-1)*npoly + k;
				fprintf(fp1,"%f\n",exp(hyp[tint]));
			}
		}
	}

	fprintf(fp1,"# Theta_1 parameter(s)\n");
	for(i=1; i <= ngauss; ++i){
		tint = N_LENGTH_SCALES+i;
		fprintf(fp1,"%f\n",hyp[tint]);
	}

	fprintf(fp1,"# Theta_2 parameter\n");
	fprintf(fp1,"%f\n",HYP_THETA_2);

	if( nhidden == 1 ){
		fprintf(fp1,"# log(noise variance) \n");
		fprintf(fp1,"%f\n",HYP_LOG_NOISE_VAR);
	}    
	if( nhidden > 1 ){
		fprintf(fp1,"# Noise model parameters \n");
		for(i=1; i <= nhidden; ++i){
			for(j=1; j <= mInputDimension+1; ++j){
				tint = N_before_NOISE+(i-1)*(mInputDimension+1)+j;
				fprintf(fp1,"%f\n",hyp[tint]);
			}
		}
		for(i=1; i <= nhidden+1; ++i){
			tint = N_before_NOISE+((mInputDimension+1)*nhidden)+i;
			fprintf(fp1,"%f\n",hyp[tint]);
		}
	}
	if( C_linear == 1 ) fprintf(fp1,"# Linear Term co-efficients\n");
	for(i=1; i <= nlin; ++i){
		tint = N_including_NOISE+i;
		fprintf(fp1,"%f\n",hyp[tint]);    
	}
	fprintf(fp1,"# Theta_0 parameter\n");
	fprintf(fp1,"%f\n",HYP_THETA_0);

	fclose( fp1 );
}

void tpros::mac_inter_C( void )
{
	int    i,j,k,l;
	int    tint;
	double temp1,temp2,temp3,temp4,temp5;

	if( training_noise_in_datafile == 0 ){
		for(i=1; i <= mNumTrainingData; ++i){
			n_vec[i] = noise(i);
		}
	}

	if( use_poly_length_scales == 0 ){ 
		ls_calc( );
	}
	for(i=1; i <= mNumTrainingData; ++i){
		for(j=1; j <= i; ++j){
			A[i][j] = 0.0;
			if( use_poly_length_scales == 0 ){ 
				for(l=1; l <= ngauss; ++l){
					temp1 = 0.0;
					tint = (l-1)*mInputDimension;
					for(k=1; k <= mInputDimension; ++k){
						if( C_periodic == 0 ){
							temp1 += (k_x[i][k] - k_x[j][k])*(k_x[i][k] - k_x[j][k]) / ls[tint+k];
						} else if( C_periodic == 1 ){
							temp5 = (k_x[i][k] - k_x[j][k]) * wavel[k];
							temp1 += pow( sin(temp5) , 2.0 ) / ls[tint+k];
						}
					}
					A[i][j] += hyp[N_LENGTH_SCALES+l] * exp( - 0.5 * temp1 );
				}
			} else {
				for(l=1; l <= ngauss; ++l){
					temp1 = 0.0;
					temp2 = 1.0;
					tint = (l-1)*mInputDimension;
					for(k=1; k <= mInputDimension; ++k){
						temp3 = PL_poly(i,l,k);
						temp4 = PL_poly(j,l,k);	    
						temp2 *= (temp3*temp4)/temp5;
						if( C_periodic == 0 ){
							temp5 = (temp3*temp3 + temp4*temp4);
							temp1 += pow( (k_x[i][k] - k_x[j][k]) , 2.0 ) / temp5;
						}
						if( C_periodic == 1 ){
							temp5 = (k_x[i][k] - k_x[j][k]) * wavel[k];
							temp1 += pow( sin(temp5) , 2.0 ) / temp5;
						}
					}
					temp2 = pow( temp2 , 0.5 ) * root2tonh;
					/* note no factor of 0.5 in exponential */
					A[i][j] += temp2 * ( hyp[N_LENGTH_SCALES+l] * exp( - temp1 ) );
				}
			}
			A[i][j] += HYP_THETA_2 ; 
			if( C_linear == 1 ){
				for(k=1; k <= nlin; ++k){
					A[i][j] += hyp[N_including_NOISE+k] * (k_x[i][k]*k_x[j][k]);
				}
			}
			C[i][j] = A[i][j];  C[j][i] = C[i][j]; A[j][i] = A[i][j];
		} 
		C[i][i] += n_vec[i];
	}   

}

void tpros::mac_inter_dif(double *w,int k)
{
	int    i,j,l;
	int    tint;
	double temp1,temp2,temp3,temp4;
	double expon,difexp;
	double prod,difprod;

	/* NOTE - we do not have a w_con_out(w) statement here as it is
	assumed that one will have gone before in mac_inter_C
	to set this up.
	*/

	if( use_poly_length_scales == 0 ){

		ls_calc( );

		if( C_periodic == 0 ){

			for(i=1; i <= mNumTrainingData; ++i){
				for(j=1; j <= i; ++j){

					dif_C[i][j] = 0.0;

					tint  = 0;
					expon = 0.0;
					prod  = 1.0;

					if( kth[k][1] == 1 || kth[k][1] == 2 ){

						for(l=1; l <= mInputDimension; ++l){

							temp1 = (k_x[i][l] - k_x[j][l])*(k_x[i][l] - k_x[j][l])
								/ ls[l+((kth[k][3]-1)*mInputDimension)];

							expon += temp1;
							if( kth[k][1] == 1 && l == kth[k][4] ){
								difexp = temp1;
							}

						}            

						prod = pow( prod, -0.5 );

					}

					/* calculate the differentials for macoptII */

					if( i != j ){
						if( kth[k][1] == 1 ){
							dif_C[i][j] = difexp*exp( -0.5*expon )*hyp[N_LENGTH_SCALES+kth[k][3]];  
						}
						else if( kth[k][1] == 2 ){
							dif_C[i][j] = hyp[kth[k][2]] * exp( -0.5*expon ) * prod; 
						}
						else if( kth[k][1] == 3 ){
							dif_C[i][j] = hyp[kth[k][2]];
						}
						else if( kth[k][1] == 5 ){
							dif_C[i][j] = hyp[kth[k][2]] * (k_x[i][kth[k][3]]*k_x[j][kth[k][3]]);
						}
					}
					else {
						if( kth[k][1] == 1 ){
							dif_C[i][j] = 0;
						}
						else if( kth[k][1] == 2 ){
							dif_C[i][j] = hyp[kth[k][2]] * exp( -0.5*expon ) * prod; 
						}
						else if( kth[k][1] == 3 ){
							dif_C[i][j] = hyp[kth[k][2]];
						}
						else if( kth[k][1] == 4 ){
							if( nhidden == 1 ){
								dif_C[i][j] = (C[i][i] - A[i][i]);
							}
							else if( nhidden > 1 ){
								if( kth[k][3] == 1 ){
									tint = N_before_NOISE+((mInputDimension+1)*nhidden)+kth[k][5];
									dif_C[i][j] = hyp[tint]*difsig(Lhidden[kth[k][5]])*k_x[i][kth[k][4]]; 
								}
								else if( kth[k][3] == 2 ){
									if( kth[k][4] <= nhidden ){
										dif_C[i][j] = (C[i][i] - A[i][i]) * sigmoid(Lhidden[kth[k][4]]); 
									} else {
										dif_C[i][j] = (C[i][i] - A[i][i]);
									}
								}
							}
						}
						else if( kth[k][1] == 5 ){
							dif_C[i][j] = hyp[kth[k][2]] * (k_x[i][kth[k][3]]*k_x[j][kth[k][3]]);
						}

					}

					/* Deal with symmetry of matrix */

					dif_C[j][i] = dif_C[i][j];

				}
			}    

		} else if ( C_periodic == 1 ){

			for(i=1; i <= mNumTrainingData; ++i){
				for(j=1; j <= i; ++j){

					dif_C[i][j] = 0.0;

					tint  = 0;
					expon = 0.0;
					prod  = 1.0;

					if( kth[k][1] == 1 || kth[k][1] == 2 ){

						for(l=1; l <= mInputDimension; ++l){

							temp1 = (k_x[i][l] - k_x[j][l]) * wavel[l];
							temp1 = pow( sin(temp1) , 2.0 ) / ls[l+((kth[k][3]-1)*mInputDimension)];

							expon += temp1;
							if( kth[k][1] == 1 && l == kth[k][4] ){
								difexp = temp1;
							}

						}            

						prod = pow( prod, -0.5 );

					}

					/* calculate the differentials for macoptII */

					if( i != j ){
						if( kth[k][1] == 1 ){
							dif_C[i][j] = difexp*exp( -0.5*expon )*hyp[N_LENGTH_SCALES+kth[k][3]];  
						}
						else if( kth[k][1] == 2 ){
							dif_C[i][j] = hyp[kth[k][2]] * exp( -0.5*expon ) * prod; 
						}
						else if( kth[k][1] == 3 ){
							dif_C[i][j] = hyp[kth[k][2]];
						}
						else if( kth[k][1] == 5 ){
							dif_C[i][j] = hyp[kth[k][2]] * (k_x[i][kth[k][3]]*k_x[j][kth[k][3]]);
						}
					}
					else {
						if( kth[k][1] == 1 ){
							dif_C[i][j] = 0;
						}
						else if( kth[k][1] == 2 ){
							dif_C[i][j] = hyp[kth[k][2]] * exp( -0.5*expon ) * prod; 
						}
						else if( kth[k][1] == 3 ){
							dif_C[i][j] = hyp[kth[k][2]];
						}
						else if( kth[k][1] == 4 ){
							if( nhidden == 1 ){
								dif_C[i][j] = (C[i][i] - A[i][i]);
							}
							else if( nhidden > 1 ){
								if( kth[k][3] == 1 ){
									tint = N_before_NOISE+((mInputDimension+1)*nhidden)+kth[k][5];
									dif_C[i][j] = hyp[tint]*difsig(Lhidden[kth[k][5]])*k_x[i][kth[k][4]]; 
								}
								else if( kth[k][3] == 2 ){
									if( kth[k][4] <= nhidden ){
										dif_C[i][j] = (C[i][i] - A[i][i]) * sigmoid(Lhidden[kth[k][4]]); 
									} else {
										dif_C[i][j] = (C[i][i] - A[i][i]);
									}
								}
							}
						}
						else if( kth[k][1] == 5 ){
							dif_C[i][j] = hyp[kth[k][2]] * (k_x[i][kth[k][3]]*k_x[j][kth[k][3]]);
						}

					}

					/* Deal with symmetry of matrix */

					dif_C[j][i] = dif_C[i][j];

				}
			}    

		}

	} else if( use_poly_length_scales == 1 ){

		for(i=1; i <= mNumTrainingData; ++i){
			for(j=1; j <= i; ++j){

				dif_C[i][j] = 0.0;

				tint  = 0;
				expon = 0.0;
				prod  = 1.0;

				if( kth[k][1] == 1 || kth[k][1] == 2 ){

					for(l=1; l <= mInputDimension; ++l){

						temp3 = PL_poly(i,kth[k][3],l);
						temp4 = PL_poly(j,kth[k][3],l);
						temp2 = temp3*temp3 + temp4*temp4;
						prod  *= ( temp3*temp4/temp2 );

						if( C_periodic == 0 ){
							temp1 = pow( (k_x[i][l] - k_x[j][l]) , 2.0 ) / temp2;
						}
						else if( C_periodic == 1 ){
							temp1 = (k_x[i][l] - k_x[j][l]) * wavel[l];
							temp1 = pow( sin(temp1) , 2.0 ) / temp2;
						}

						expon += temp1;
						if( kth[k][1] == 1 && l == kth[k][4] ){
							difexp = (temp3*temp3*PL[i][kth[k][5]]+temp4*temp4*PL[j][kth[k][5]]) * (2.0*temp1/temp2);
							difprod = 0.5*(temp3*temp3 - temp4*temp4)*(PL[j][kth[k][5]] - PL[i][kth[k][5]])/temp2;
						}	  
					}            

					prod = pow( prod, 0.5 ) * root2tonh;

				}

				/* calculate the differentials for macoptII */

				if( i != j ){
					if( kth[k][1] == 1 ){
						dif_C[i][j] = (difexp + difprod)*prod*exp( -expon )*hyp[N_LENGTH_SCALES+kth[k][3]];  
					}
					else if( kth[k][1] == 2 ){
						dif_C[i][j] = hyp[kth[k][2]] * exp( -expon ) * prod; 
					}
					else if( kth[k][1] == 3 ){
						dif_C[i][j] = hyp[kth[k][2]];
					}
					else if( kth[k][1] == 5 ){
						dif_C[i][j] = hyp[kth[k][2]] * (k_x[i][kth[k][3]]*k_x[j][kth[k][3]]);
					}
				}
				else {
					if( kth[k][1] == 1 ){
						dif_C[i][j] = difprod*prod*hyp[N_LENGTH_SCALES+kth[k][3]];  
					}
					else if( kth[k][1] == 2 ){
						dif_C[i][j] = hyp[kth[k][2]] * exp( -0.5*expon ) * prod; 
					}
					else if( kth[k][1] == 3 ){
						dif_C[i][j] = hyp[kth[k][2]];
					}
					else if( kth[k][1] == 4 ){
						if( nhidden == 1 ){
							dif_C[i][j] = (C[i][i] - A[i][i]);
						}
						else if( nhidden > 1 ){
							if( kth[k][3] == 1 ){
								tint = N_before_NOISE+((mInputDimension+1)*nhidden)+kth[k][5];
								dif_C[i][j] = hyp[tint]*difsig(Lhidden[kth[k][5]])*k_x[i][kth[k][4]]; 
							}
							else if( kth[k][3] == 2 ){
								if( kth[k][4] <= nhidden ){
									dif_C[i][j] = (C[i][i] - A[i][i]) * sigmoid(Lhidden[kth[k][4]]); 
								} else {
									dif_C[i][j] = (C[i][i] - A[i][i]);
								}
							}
						}
					}
					else if( kth[k][1] == 5 ){
						dif_C[i][j] = hyp[kth[k][2]] * (k_x[i][kth[k][3]]*k_x[j][kth[k][3]]);
					}

				}

				/* Deal with symmetry of matrix */

				dif_C[j][i] = dif_C[i][j];

			}
		}    

	}

}

void tpros::LU_invert(double **E,double **F,int n)
{
	double d;
	int    i,j;

	/* transfer matrix for inversion into F */

	for(i=1; i <= n; ++i){
		for(j=1; j <= n; ++j){
			C_temp[i][j] = E[i][j];
		}
	}

	/* Use LU decomposition to invert E[][] and find det(E) */

	ludcmp(C_temp,n,indx,&d);

	log_det = 0.0;
	for(i=1; i <= n; ++i){
		log_det += log(fabs(C_temp[i][i]));
		d       *= ( C_temp[i][i] > 0.0 ) ? 1.0 : - 1.0 ; 
	}
	if( d < 0.0 ) printf("WARNING - negative eigenvalues\n");

	for(j=1;j<=n;j++){
		for(i=1;i<=n;i++) lu_col[i] = 0.0;
		lu_col[j]=1.0;
		lubksb(C_temp,n,indx,lu_col);
		for(i=1;i<=n;i++) F[i][j] = lu_col[i];
	}

}

void tpros::LU_apply(double **E,double *a,double *b,int n) 
/* inverts E using C_temp. Makes copy of a and puts C^{-1}a into b */
{
	double d;
	int    i,j;

	/* transfer matrix for inversion into F */

	for(i=1; i <= n; ++i){
		for(j=1; j <= n; ++j){
			C_temp[i][j] = E[i][j];
		}
	}

	/* Use LU decomposition to invert E[][] and find det(E) */

	ludcmp(C_temp,n,indx,&d);

	log_det = 0.0;
	for(i=1; i <= n; ++i){
		log_det += log(fabs(C_temp[i][i]));
		d       *= C_temp[i][i]/fabs(C_temp[i][i]);
	}
	if( d < 0.0 ) printf("WARNING - negative eigenvalues\n");

	/* copy a into b */

	for(i=1; i <= n; ++i){
		b[i] = a[i];
	}

	/* calculate Cinv b and dump result in b */

	lubksb(C_temp,n,indx,b);

}

void tpros::LU_stage1(double **a,int n,int *inx,double c[],double v[])
{
	int i,ii=0,ip,j;
	double sum;

	/* 
	Inversion finds LU decomposition

	(C)^{-1} = (LU)^{-1}

	THIS routine takes vector c[] and calculates

	v[] = L^{-1} c[]

	*/

	for(i=1; i <= n; i++){
		v[i] = c[i];
	}

	for(i=1; i <= n; i++){
		ip = inx[i];
		sum = v[ip];
		v[ip] = v[i];
		if( ii )
			for(j=ii; j<=i-1 ;j++) sum -= a[i][j] * v[j];
		else if (sum) ii = i;
		v[i] = sum;
	}

}

void tpros::LU_stage2(double **a,int n,int *inx,double b[],double v[])
{
	int i,ii=0,j;
	double sum;

	/* 
	Inversion finds LU decomposition

	(C)^{-1} = (LU)^{-1}

	THIS routine takes vector b[] and calculates

	v = (U^{T})^{-1} b

	*/

	for(i=1; i <= n; i++){
		v[i] = b[i];
	}

	for(i=1; i <= n; i++){
		sum = v[i];
		if(ii)
			for(j=ii; j<=i-1 ;j++) sum -= a[j][i] * v[j];
		else if (sum) ii=i;
		v[i] = sum / a[i][i];
	}

}

void tpros::CG_invert(double *w,double *u,int itmax)
{
	int    i,j,l;
	int    N;
	double temp1,temp2,temp3;
	double *tempa,*tempb,*tempc,*tempd;
	double lbound;
	double ubound;
	double uu;

	if( OPT_tridiag_CG == 0 ){

		/* ====== double-CG Implementation ====== */

		for(i=1; i <= mNumTrainingData; ++i){
			w[i] = 0.0;
			ws[i] = 0.0;
		}

		N = itmax;

		tempa   = dvector(1,mNumTrainingData); /* C*h      */
		tempb   = dvector(1,mNumTrainingData); /* C*hs     */
		tempc   = dvector(1,mNumTrainingData); /* A*gs     */
		tempd   = dvector(1,mNumTrainingData); /* old A*gs */

		for(i=1; i <= mNumTrainingData; ++i){
			g[1][i] = u[i];
			gs[1][i] = u[i];
		}

		for(l=1; l <= N; ++l){

			if(l == 1){

				temp1 = 0.0;
				for(i=1;i<=mNumTrainingData;++i){
					h[l][i]=g[l][i];
					temp1 +=g[l][i]*g[l][i];
				}	
				mod_g[l] = sqrt(temp1);
				for(i=1;i<=mNumTrainingData;++i){
					hs[l][i]=gs[l][i];
				}	

				for(i=1; i <= mNumTrainingData; ++i){
					tempc[i] = 0.0;
					for(j=1; j <= mNumTrainingData; ++j){
						tempc[i] += A[i][j]*gs[l][j];
					}
				}

			}

			if(l != 1){

				for(i=1; i <= mNumTrainingData; ++i){
					g[l][i] = g[l-1][i] - lambda[l-1]*tempa[i];
				}
				temp1 = 0.0;
				for(i=1;i<=mNumTrainingData;++i){
					temp1 += g[l][i]*g[l][i];
				}
				gam[l-1] = temp1/(mod_g[l-1]*mod_g[l-1]);
				mod_g[l] = sqrt(temp1);
				for(i=1;i<=mNumTrainingData;++i) h[l][i]=g[l][i] + (gam[l-1]*h[l-1][i]);

				for(i=1; i <= mNumTrainingData; ++i){
					gs[l][i] = gs[l-1][i] - lambdas[l-1]*tempb[i];
				}
				for(i=1; i <= mNumTrainingData; ++i){
					tempd[i] = tempc[i];
					tempc[i] = 0.0;
					for(j=1; j <= mNumTrainingData; ++j){
						tempc[i] += A[i][j]*gs[l][j];
					}
				}
				temp1 = 0.0;
				temp2 = 0.0;
				for(i=1;i<=mNumTrainingData;++i){
					temp1 += tempc[i]*gs[l][i];
					temp2 += tempd[i]*gs[l-1][i];
				}
				gams[l-1] = temp1/temp2;
				for(i=1;i<=mNumTrainingData;++i) hs[l][i]=gs[l][i] + (gams[l-1]*hs[l-1][i]);

			}

			for(i=1; i <= mNumTrainingData; ++i){
				tempa[i] = 0.0;
				tempb[i] = 0.0;
				for(j=1; j <= mNumTrainingData; ++j){
					tempa[i] += C[i][j]*h[l][j];
					tempb[i] += C[i][j]*hs[l][j];       
				}
			}
			temp1 = 0.0;
			temp2 = 0.0;
			temp3 = 0.0;
			for(i=1; i <= mNumTrainingData; ++i){
				temp1 += g[l][i]*tempa[i];
				temp2 += gs[l][i]*tempc[i];
				temp3 += tempc[i]*tempb[i];
			}

			lambda[l] = mod_g[l]*mod_g[l]/temp1;
			lambdas[l] = temp2/temp3; 

			for(i=1; i <= mNumTrainingData; ++i){
				w[i] += lambda[l] * h[l][i];
				ws[i] += lambdas[l] * hs[l][i];
			} 

			if( l == N ){
				bounds(w,ws,u,&lbound,&ubound); 
				if( fabs((ubound-lbound)*2.0/(ubound+lbound)) > bnd_percent ){
					if( N < N_max ){
						N += itmax;
					}
					if( N >= N_max ){
						printf("ERROR : Inexact matrix inversion\n");
					}
				}
			}

		}

		free_dvector(tempa,1,mNumTrainingData);
		free_dvector(tempb,1,mNumTrainingData);
		free_dvector(tempc,1,mNumTrainingData);
		free_dvector(tempd,1,mNumTrainingData);

	}
	if( OPT_tridiag_CG == 1 ){

		/* ====== Tridiagonal-CG Implementation ====== */

		N = 1;

		tempa   = dvector(1,mNumTrainingData);
		tempb   = dvector(1,N_max);
		tempc   = dvector(1,N_max);
		tempd   = dvector(1,N_max);

		uu = 0.0;
		for(i=1; i <= mNumTrainingData; ++i){
			uu += u[i]*u[i];
		}

		for(i=1; i <= mNumTrainingData; ++i){
			g[1][i] = u[i];
		}

		for(l=1; l <= N; ++l){

			if(l == 1){

				temp1 = 0.0;
				for(i=1;i<=mNumTrainingData;++i){
					h[l][i]=g[l][i];
					temp1 +=g[l][i]*g[l][i];
				}	
				mod_g[l] = sqrt(temp1);

				hAh[l] = 0.0;
				for(i=1; i <= mNumTrainingData; ++i){
					tempa[i] = 0.0;
					for(j=1; j <= mNumTrainingData; ++j){
						tempa[i] += A[i][j]*h[l][j];
					}
				}    

			}

			if(l != 1){

				gam[l-1] = mod_g[l]*mod_g[l]/(mod_g[l-1]*mod_g[l-1]);
				for(i=1;i<=mNumTrainingData;++i) h[l][i]=g[l][i] + (gam[l-1]*h[l-1][i]);

				hAh[l] = 0.0;
				for(i=1; i <= mNumTrainingData; ++i){
					tempa[i] = 0.0;
					for(j=1; j <= mNumTrainingData; ++j){
						tempa[i] += A[i][j]*h[l][j];
					}
				}    

			}

			temp1 = 0.0;
			for(i=1; i <= mNumTrainingData; ++i){
				temp1 += g[l][i]*h[l][i];
				hAh[l] += h[l][i]*tempa[i];
			}
			lambda[l] = temp1/hAh[l];

			for(i=1; i <= mNumTrainingData; ++i){
				g[l+1][i] = g[l][i];
				g[l+1][i] -= lambda[l]*tempa[i];
			}
			temp1 = 0.0;
			for(i=1;i<=mNumTrainingData;++i){
				temp1 += g[l+1][i]*g[l+1][i];
			}
			mod_g[l+1] = sqrt(temp1);

			/* Check the bounds */

			/* Calculate all tridiagonal matrices */

			if( l == 1 ){

				Td[1] = 1.0 / lambda[1] + n_vec[1];
				Tl[1] = 0.0;
				Tu[N] = 0.0; 

				Td2[1] = hAh[1]*hAh[1]*(1.0+(mod_g[2]*mod_g[2]/(mod_g[1]*mod_g[1]))) / 
					(mod_g[1]*mod_g[1]) + n_vec[1]*hAh[1];
				Tu2[N] = 0.0;
				Tl2[1] = 0.0;

			}
			if( l > 1 ){

				Td[l] = (1.0/lambda[l]) + (gam[l-1]/lambda[l-1]) + n_vec[1];
				Tl[l] = -1.0 * mod_g[l] / (lambda[l-1] * mod_g[l-1]);
				Tu[l-1] = -1.0 * mod_g[l] / (lambda[l-1] * mod_g[l-1]);

				Td2[l] = hAh[l]*hAh[l]*(1.0+(mod_g[l+1]*mod_g[l+1]/(mod_g[l]*mod_g[l]))) / 
					(mod_g[l]*mod_g[l]) + (n_vec[1])*hAh[l];
				Tu2[l-1] = -1.0*hAh[l-1]*hAh[l]/(mod_g[l-1]*mod_g[l-1]);
				Tl2[l] = Tu2[l-1];

			}

			/* Calculate Q */

			if( l == 1 ){
				tempb[1] = mod_g[1];
			}
			if( l > 1 ){
				tempb[l] = 0.0;
			}

			tridiag(Tl,Td,Tu,tempb,tempd,l);   

			lbound = 0.0;
			for(i=1; i <= l; ++i){
				lbound += tempb[i]*tempd[i];
			}

			/* Calculate y_bar */

			if( l == 1 ){
				tempc[l] = (mod_g[l]*mod_g[l])*( (1.0/lambda[1]));
			}
			if( l > 1 ){
				tempc[l] = 0.0;
			}

			/* Calculate Q* */

			tridiag(Tl2,Td2,Tu2,tempc,tempd,l);

			ubound = 0.0;
			for(i=1; i <= l; ++i){
				ubound += tempc[i]*tempd[i];
			}

			ubound = ( (0.5*uu) - 0.5*ubound ) / n_vec[1];      
			lbound = 0.5*lbound;

			if( fabs((ubound-lbound)*2.0/(ubound+lbound)) > bnd_percent ){
				if( N < N_max ){
					N += 1;
				}
				if( N >= N_max ){
					printf("ERROR : Inexact matrix inversion\n");
				}
			} 

		}

		/* Find weight vector */

		l -= 1;

		tridiag(Tl,Td,Tu,tempb,tempc,l);   

		for(i=1; i <= mNumTrainingData; ++i){
			w[i] = 0.0;
			for(j=1; j <= l; ++j){
				w[i] += g[j][i]*tempc[j]/(mod_g[j]);
			}
		}    

		free_dvector(tempa,1,mNumTrainingData);
		free_dvector(tempb,1,N_max);
		free_dvector(tempc,1,N_max);
		free_dvector(tempd,1,N_max);
	}
}

void tpros::bounds(double *w,double *w2,double *u,double *lbound,double *ubound)
{
	int    i;
	double temp1;

	*ubound = -1.0 * CG_func2(w,u);

	/* Here we only use n_vec[1] because we must only have input
	independent noise. */

	temp1 = 0.0;
	for(i=1; i <= mNumTrainingData; ++i){
		temp1 += u[i]*u[i];
	}
	*lbound = -1.0 * ( (0.5*temp1) - CG_func1(w2,u) ) / n_vec[1];

}

double tpros::CG_func1(double *w,double *u)
{
	int    i,j;
	double tot,tot1,tot2;
	double *tempd,*tempc;

	tempc = dvector(1,mNumTrainingData);
	tempd = dvector(1,mNumTrainingData);

	/* Multiply w[] by A[][] - store in tempd */
	tot1 = 0.0;
	for(i=1; i <= mNumTrainingData; ++i){
		tempd[i] = 0.0;
		for(j=1; j <= mNumTrainingData; ++j){
			tempd[i] += A[i][j]*w[j];
		}
	}

	/* Multiply A[][]w[] by u[] and calculate tot1 */
	for(i=1; i <= mNumTrainingData; ++i){
		tot1 += tempd[i] * u[i]; 
	}

	/* Multiply A[][]w[] by w[]*C[][] */
	tot2 = 0.0;
	for(i=1; i <= mNumTrainingData; ++i){
		for(j=1; j <= mNumTrainingData; ++j){
			tot2 -= 0.5 * w[i] * C[i][j] * tempd[j];
		}
	}

	tot = (tot1+tot2);

	free_dvector(tempc,1,mNumTrainingData);
	free_dvector(tempd,1,mNumTrainingData);

	return tot;
}

double tpros::CG_func2(double *w,double *u)
{
	int    i,j;
	double tot,tot1,tot2;

	tot1 = 0.0;
	for(i=1; i <= mNumTrainingData; ++i){
		tot1 += w[i] * u[i];
	}

	tot2 = 0.0;
	for(i=1; i <= mNumTrainingData; ++i){
		for(j=1; j <= mNumTrainingData; ++j){
			tot2 += C[i][j] * w[i] * w[j];
		}
	}
	tot2 = -0.5 * tot2;

	tot = (tot1+tot2);

	return tot;
}

double tpros::trace(double *w,double *u)
{
	int    i,j;
	double temp1;

	temp1 = 0.0;
	for(i=1; i <= mNumTrainingData; ++i){
		for(j=1; j <= mNumTrainingData; ++j){
			temp1 += w[i] * dif_C[i][j] * u[j]; 
		}
	} 

	return temp1;
}

double tpros::noise(int a) 
{
	double temp1;

	if( nhidden == 1 ){
		temp1 = exp( HYP_LOG_NOISE_VAR ) ;
	} else {
		temp1 = exp( L[a] );
	}

	return temp1;
}

double tpros::prior(double *w)
{
	int    i,j,k;
	int    tint,tint2;
	double temp1;

	temp1 = 0.0;
	tint  = 0;

	if( optimize_lengthscales == 1 ){
		if( lengthprior >= 1 ){
			for(j=1; j <= ngauss; ++j){
				for(i=1; i <= mInputDimension; ++i){
					if( use_poly_length_scales == 0 ){
						/* Gamma Prior */
						tint2 = (j-1)*mInputDimension + (i-1) + 1;
						temp1 -= ( (gamr_a[j][i]-1.0) * w[tint2]) - (gamr_b[j][i] * exp( w[tint2])); 
					} else if( use_poly_length_scales == 1 ){
						for(k=1; k <= npoly-1; ++k){
							/* Gaussian Prior on each term */
							tint2 = (j-1)*mInputDimension*npoly + (i-1)*npoly + k;
							temp1 += 0.5 * pow( w[tint2]/gamr_b[j][i] , 2.0 );
						}
						tint2 = (j-1)*mInputDimension*npoly + i*npoly;
						temp1 += 0.5 * pow( (w[tint2]-gamr2_a[j][i])/gamr2_b[j][i] , 2.0 );
					}
				}
			}
		}
		tint +=  mInputDimension*ngauss*npoly ;
	}
	if( optimize_theta1 == 1 ){
		if( theta1prior >= 1 ){
			for(j=1; j <= ngauss; ++j){
				temp1 -= ( - (gamt1_a+1.0) * w[tint+j]) - (gamt1_b * exp(- w[tint+j]));
			}
		}
		tint += ngauss;
	}
	if( optimize_theta2 == 1 ){  
		if( theta2prior >= 1 ){
			temp1 -= ( - (gamt2_a+1.0) * w[tint+1]) - (gamt2_b * exp(- w[tint+1]));
		}
		tint += 1;
	}
	if( optimize_noise == 1 ){
		if( noiseprior >= 1 ){
			/* Inverse Gamma Prior */
			if( nhidden == 1 ){
				temp1 -= ( - (gamn_a+1.0) * w[tint+1]) - (gamn_b * exp(- w[tint+1]));
			} else {
				/* Gaussian Prior on weights */
				for(i=1; i <= nhidden; ++i){
					for(j=1; j <= mInputDimension+1; ++j){
						temp1 += 0.5 * pow( w[tint+(mInputDimension+1)*(i-1)+j]/gamn_b , 2.0 );
					}
				}
				for(i=1; i <= nhidden; ++i){
					temp1 += 0.5 * pow( w[tint+((mInputDimension+1)*nhidden)+i]/gamn_b , 2.0 );
				}
			}
		}
		tint += nwb;
	}
	if( optimize_linear == 1 ){
		if( linearprior >= 1 ){
			for(i=1; i <= nlin; ++i){
				temp1 += 0.5 * pow( (w[tint+i] - gaussl_m)/gaussl_sd , 2.0 );
			}
			tint += nlin;
		}
	}
	if( optimize_theta0 == 1 ){
		if( theta0prior >= 1 ){
			temp1 += 0.5 * pow( (w[tint+1] - gausst0_m)/gausst0_sd , 2.0 );
		}
		tint += 1;
	}

	return temp1;
}

void tpros::dif_prior(double *dM,double *w)
{
	int i,j,k;
	int tint,tint2;

	for(i=1; i <= nw; ++i) dM[i] = 0.0;

	/* Gamma priors */

	tint = 0;
	if( optimize_lengthscales == 1 ){
		if( lengthprior >= 1 ){
			for(j=1; j <= ngauss; ++j){
				for(i=1; i <= mInputDimension; ++i){
					if( use_poly_length_scales == 0 ){
						/* Gamma Prior */
						tint2 = (j-1)*mInputDimension + i;	
						dM[tint2] = -1.0 * ((gamr_a[j][i]-1.0) - ( gamr_b[j][i] * exp(w[tint2]) ) ); 
					} else if( use_poly_length_scales == 1 ){
						for(k=1; k <= npoly-1; ++k){
							/* Gaussian prior on each term */
							tint2 = (j-1)*mInputDimension*npoly + (i-1)*npoly + k;
							dM[tint2] = w[tint2]/(gamr_b[j][i]*gamr_b[j][i]);
						}
						tint2 = (j-1)*mInputDimension*npoly + i*npoly;
						dM[tint2] = (w[tint2]-gamr2_a[j][i])/(gamr2_b[j][i]*gamr2_b[j][i]);
					}
				}
			}
		}
		tint +=  mInputDimension*ngauss*npoly ;
	}
	if( optimize_theta1 == 1 ){
		if( theta1prior >= 1 ){
			for(j=1; j <= ngauss; ++j){
				/* Inverse Gamma Prior */
				dM[tint+j] = -1.0 * (- (gamt1_a+1.0) - ( - gamt1_b * exp(- w[tint+j]) ) );
			}
		}
		tint += ngauss;
	}
	if( optimize_theta2 == 1 ){  
		tint += 1;
		if( theta2prior >= 1 ){
			/* Inverse Gamma Prior */
			dM[tint] = -1.0 * (- (gamt2_a+1.0) - ( - gamt2_b * exp(- w[tint]) ) );
		}
	}
	if( optimize_noise == 1 ){
		if( noiseprior >= 1 ){
			if( nhidden == 1 ){
				/* Inverse Gamma Prior */
				dM[tint+1] = -1.0 * ( - (gamn_a+1.0) - ( - gamn_b * exp(- w[tint+1]) ) );
			}
			if( nhidden > 1 ){
				/* Gaussian Prior on weights */
				for(i=1; i <= nhidden; ++i){
					for(j=1; j <= mInputDimension+1; ++j){
						dM[tint+i] = w[tint+(mInputDimension+1)*(i-1)+j]/(gamn_b*gamn_b);
					}
				}
				for(i=1; i <= nhidden; ++i){
					dM[tint+i] = w[tint+((mInputDimension+1)*nhidden)+i]/(gamn_b*gamn_b);
				}
			}
		}
		tint += nwb;
	}
	if( optimize_linear == 1 ){
		if( linearprior >= 1 ){
			for(i=1; i <= nlin; ++i){
				/* Gaussian Prior */
				dM[tint+i] = (w[tint+i] - gaussl_m)/(gaussl_sd*gaussl_sd);
			}
			tint += nlin;
		}
	}
	if( optimize_theta0 == 1 ){  
		tint += 1;
		if( theta0prior >= 1 ){
			/* Gaussian Prior */
			dM[tint] = (w[tint] - gausst0_m)/(gausst0_sd*gausst0_sd);
		}
	}
}

void tpros::w_con_in(double *w)
{
	int i;

	for(i=1; i <= nw; ++i){

		if( kth[i][1] == 1 ){
			w[i] = hyp[kth[i][2]];
		}
		if( kth[i][1] == 2 ){
			if( hyp[kth[i][2]] > 1.0e-10 ){
				w[i] = log(hyp[kth[i][2]]);
			} else {
				w[i] = log(1.0e-10);
			}
		}
		if( kth[i][1] == 3 ){
			if( hyp[kth[i][2]] > 1.0e-10 ){
				w[i] = log(hyp[kth[i][2]]);
			} else {
				w[i] = log(1.0e-10);
			}
		}
		if( kth[i][1] == 4 ){
			w[i] = hyp[kth[i][2]];
		}
		if( kth[i][1] == 5 ){
			if( hyp[kth[i][2]] > 1.0e-10 ){
				w[i] = log(hyp[kth[i][2]]);
			} else {
				w[i] = log(1.0e-10);
			}
		}
		if( kth[i][1] == 6 ){
			w[i] = hyp[kth[i][2]];
		}

	}

}

void tpros::w_con_out(double *w)
{
	int i;

	for(i=1; i <= nw; ++i){

		if( kth[i][1] == 1 ){
			hyp[kth[i][2]] = w[i];
		}
		if( kth[i][1] == 2 ){
			hyp[kth[i][2]] = exp(w[i]);
		}
		if( kth[i][1] == 3 ){
			hyp[kth[i][2]] = exp(w[i]);
		}
		if( kth[i][1] == 4 ){
			hyp[kth[i][2]] = w[i];
		}
		if( kth[i][1] == 5 ){
			hyp[kth[i][2]] = exp(w[i]);
		}
		if( kth[i][1] == 6 ){
			hyp[kth[i][2]] = w[i];
		}

	}

}

void tpros::PL_setup( void )
{
	int    i,j;
	FILE   *fp1;

	/* we are presently using radial basis functions so lets load in
	the centres from the file rbffile */

	fp1 = fopen( rbffile , "r" );
	if( !fp1 ) fprintf( stderr, "No such file : %s\n",rbffile ), Msg::error("tpros error");
	for(i=1; i <= npoly-1; ++i){
		for(j=1; j <= mInputDimension; ++j){
			fscanf(fp1,"%lf",&pk_x[i][j]);
		}
	}
	fclose( fp1 );

	for(i=1; i <= mNumTrainingData; ++i){
		PL_calc(i);
	}

	/* DEBUG 
	fp1 = fopen( "blob", "w" );
	for(i=1; i <= ninter; ++i){
	for(j=1; j <= mInputDimension; ++j){
	k_x[1][j] = ok_x[i][j];
	fprintf(fp1,"%f ",k_x[1][j]);
	}
	PL_calc(1);
	for(j=1; j <= npoly-1; ++j){
	fprintf(fp1,"%f ",PL[1][j]);
	}
	fprintf(fp1,"\n");
	}
	fclose(fp1); */

}

void tpros::PL_calc(int a)
{
	int    i,j;
	double temp1;

	for(i=1; i <= npoly-1; ++i){

		temp1 = 0.0;
		for(j=1; j <= mInputDimension; ++j){  
			temp1 += pow( (k_x[a][j] - pk_x[i][j])*(double)(npoly) / Lscale[j] , 2.0 );
		}

		PL[a][i] = exp(-1.0*temp1);

	}
	PL[a][npoly] = 1.0;

} 

double tpros::PL_poly(int i,int j,int k)
{
	int    m;
	int    tint;
	double temp1;

	temp1 = 0.0;
	for(m=1; m <= npoly; ++m){
		tint = (j-1)*mInputDimension*npoly + (k-1)*npoly + m;
		temp1 += hyp[tint]*PL[i][m];
	}

	/* return the length scale */

	return exp( temp1 );
}

void tpros::L_setup( void )
{
	int    i,j;
	int    tint;

	/* We shall allow adaption of centre of basis functions for 
	the noise level. Hence We pick the centres randomly to 
	begin with */

	if( hyp_in == 0 ){ /* if not reading in hyp from file */
		for(i=1; i <= nhidden; ++i){
			for(j=1; j <= mInputDimension+1; ++j){ 
				tint = N_before_NOISE+(i-1)*mInputDimension+j;
				hyp[tint] = 0.01*rand_uniform( );
			}
		}
		for(i=1; i <= nhidden; ++i){
			tint = N_before_NOISE+((mInputDimension+1)*nhidden)+i;
			hyp[tint] = 0.01*rand_uniform( );
		}
	}

	for(i=1; i <= mNumTrainingData; ++i){
		L_calc(i);
	}

}

void tpros::L_calc(int a)
{
	int    i,j;
	int    tint;

	for(i=1; i <= nhidden; ++i){
		Lhidden[i] = 0.0;
		for(j=1; j <= mInputDimension; ++j){  
			tint = N_before_NOISE+(i-1)*(mInputDimension+1)+j;
			Lhidden[i] += hyp[tint]*k_x[a][j];
		}
		Lhidden[i] += hyp[tint+1]; /* bias unit */
	}
	L[a] = 0.0;
	for(i=1; i <= nhidden; ++i){
		tint = N_before_NOISE+((mInputDimension+1)*nhidden)+i;
		L[a] += hyp[tint]*sigmoid(Lhidden[i]);
	}
	L[a] += hyp[tint+1]; /* bias unit */

} 

void tpros::readInSpecfile(char *file,double *D)
{
	int   i,j,c;
	char  junk[200];
	FILE  *fpp;
	double temp ;

	fpp = fopen( file, "r" );

	if( !fpp ) fprintf( stderr, "No such file : %s\n",file ), Msg::error("tpros error");

	while( fscanf(fpp,"%s",junk) != EOF){

		/* recursive file feature */

		if(strcmp(junk,"#include") == 0){
			fscanf(fpp,"%s",junk);
			readInSpecfile(junk,D);
		}

		/* Input file names */

		else if(strcmp(junk,"target_file") == 0) fscanf(fpp,"%s",datafile);
		else if(strcmp(junk,"grid_file") == 0) fscanf(fpp,"%s",gridfile);
		else if(strcmp(junk,"rbf_file") == 0) fscanf(fpp,"%s",rbffile);
		else if(strcmp(junk,"hypout") == 0){
			hyp_out = 1;
			fscanf(fpp,"%s",hypoutfile);
		}
		else if(strcmp(junk,"hypin") == 0){ /* changed DJCM 9804 */
			hyp_in = 1;
			fscanf(fpp,"%s",hypinfile);
		}
		else if(strcmp(junk,"Load_known_training_noise_levels_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				training_noise_in_datafile = 1;
			}
		}

		/* Data */

		else if(strcmp(junk,"Seed_for_random_numbers") == 0) fscanf(fpp,"%d",&SEED);
		else if(strcmp(junk,"Dimension_of_input_space") == 0)
		{
			int temp;
			if(mInputDimension==0)
				fscanf(fpp,"%d",&mInputDimension);
			else
				fscanf(fpp,"%d",&temp);
		}
		else if(strcmp(junk,"No._of_training_data_points") == 0)
		{
			int temp;
			if(mNumTrainingData==0)
				fscanf(fpp,"%d",&mNumTrainingData);
			else
				fscanf(fpp,"%d",&temp);
		}
		else if(strcmp(junk,"INT_No._of_points_per_line") == 0) fscanf(fpp,"%d",&gridx1);

		/* Covariance Function */

		else if(strcmp(junk,"Number_of_gaussians_in_cov_fn") == 0){
			fscanf(fpp,"%d",&ngauss);
		}
		else if(strcmp(junk,"Order_of_polynomial_for_length_scales") == 0 ){
			fscanf(fpp,"%d",&npoly);
			if( npoly > 1 ) use_poly_length_scales = 1;
		}
		else if(strcmp(junk,"Linear_term_in_covariance_function_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				C_linear = 1;
			}
		}
		else if(strcmp(junk,"Periodic_covariance_function_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				C_periodic = 1;
				wavel = dvector(1,mInputDimension);
			}
		}
		else if(strcmp(junk,"Wavelengths") == 0){
			for(i=1; i <= mInputDimension; ++i){
				if ( C_periodic == 1){
					fscanf(fpp,"%lf",&wavel[i]);
					printf("%f ",wavel[i]);
				}	  else {
					fscanf(fpp,"%lf",&temp);
				}
			}
		}

		/* Noise Model */

		else if(strcmp(junk,"No._of_hidden_neurons_used_for_noise_model") == 0){
			fscanf(fpp,"%d",&nhidden);
		}

		/* Optimisation */

		else if(strcmp(junk,"Perform_Optimisation_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				do_OPT = 1;
			}
		}
		else if(strcmp(junk,"OPT_dump_hyperparameters_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				write_hdump = 1;
			}
		}
		else if(strcmp(junk,"OPT_Inversion_Method_(CG/LU)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"CG") == 0 ){
				OPT_use_cg = 1;
			}
		}
		else if(strcmp(junk,"OPT_CG_Method_(TCG/DCG)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"DCG") == 0 ){
				OPT_tridiag_CG = 0;
			}
			if( strcmp(junk,"TCG") == 0 ){
				OPT_tridiag_CG = 1;
			}
		}
		else if(strcmp(junk,"OPT_No._of_mac_its_for_CG_inversion") == 0) fscanf(fpp,"%d",&CG_opt);
		else if(strcmp(junk,"OPT_No._of_elements_used_for_trace") == 0) fscanf(fpp,"%d",&ntrace);
		else if(strcmp(junk,"OPT_No._of_mac_its") == 0) fscanf(fpp,"%d",&opt_itmax);
		else if(strcmp(junk,"OPT_accuracy_of_CG_inversion") == 0){
			fscanf(fpp,"%s",junk);
			bnd_percent = atof(junk);
		} 
		else if( strcmp(junk,"OPT_macopt_tolerance") == 0 ){
			fscanf(fpp,"%s",junk);
			mac_h_tol = atof(junk);
		}
		else if( strcmp(junk,"KOPT_macopt_tolerance") == 0 ){
			fscanf(fpp,"%s",junk);
			mac_k_tol = atof(junk);
		}
		else if( strcmp(junk,"OPT_macopt_termination_style") == 0 ){
			fscanf(fpp,"%s",junk);
			mac_h_style = atoi(junk);
		}
		else if( strcmp(junk,"KOPT_macopt_termination_style") == 0 ){
			fscanf(fpp,"%s",junk);
			mac_k_style = atoi(junk);
		}
		else if(strcmp(junk,"OPT_Cinv[][]t[]_output_file") == 0){
			fscanf(fpp,"%s",Cinvtfile);
			write_Cinv_t = 1;
		} 
		else if(strcmp(junk,"OPT_calculate_evidence_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) OPT_say_evidence = 1;
		}
		else if(strcmp(junk,"Optimize_length_scales_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) optimize_lengthscales = 1;
		}
		else if(strcmp(junk,"Optimize_theta1_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) optimize_theta1 = 1;
		}
		else if(strcmp(junk,"Optimize_theta0_(yes/no)") == 0){ 
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) optimize_theta0 = 1;
		}
		else if(strcmp(junk,"Optimize_theta2_(yes/no)") == 0){ 
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) optimize_theta2 = 1;
		}
		else if(strcmp(junk,"Optimize_noise_model_(yes/no)") == 0){ 
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) optimize_noise = 1;
		}
		else if(strcmp(junk,"Optimize_linear_term_(yes/no)") == 0){ 
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) optimize_linear = 1;
		}

		/* Interpolation */

		else if(strcmp(junk,"Perform_Interpolation_(yes/no)") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				do_INT = 1;
			}
		}
		else if(strcmp(junk,"INT_No._of_points_to_be_interpolated") == 0) fscanf(fpp,"%d",&ninter);
		else if(strcmp(junk,"INT_No._of_points_per_line") == 0) fscanf(fpp,"%d",&gridx1);
		else if(strcmp(junk,"INT_target_file") == 0){
			fscanf(fpp,"%s",idatafile);
			INT_load_targets = 1;
		}
		else if(strcmp(junk,"INT_grid_file") == 0) fscanf(fpp,"%s",igridfile);
		else if(strcmp(junk,"INT_output_file") == 0) fscanf(fpp,"%s",outfile);
		else if(strcmp(junk,"INT_include_inputs_(yes/no)") == 0) {
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				INT_include_inputs = 1 ;
			} else {
				INT_include_inputs = 0 ;
			}
		} 
		else if(strcmp(junk,"INT_ls_error_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				calc_Hessian = 1;
			}
		}
		else if(strcmp(junk,"INT_Inversion_Method_(CG/LUd/LUi)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"CG") == 0 ){
				INT_use_cg = 1;
			}
			if( strcmp(junk,"LUi") == 0 ){
				INT_LUi = 1;
				INT_use_cg = 0;
			}
			if( strcmp(junk,"LUd") == 0 ){
				INT_LUi = 0;
				INT_use_cg = 0;
			}
		}     
		else if(strcmp(junk,"INT_CG_Method_(TCG/DCG)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"TCG") == 0 ){
				OPT_tridiag_CG = 1;
			}
			if( strcmp(junk,"DCG") == 0 ){
				OPT_tridiag_CG = 0;
			}
		}     
		else if(strcmp(junk,"INT_No._of_mac_its_for_CG_invert") == 0) fscanf(fpp,"%d",&CG_int);
		else if(strcmp(junk,"INT_Calculate_error_bars_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				INT_find_eb = 1;
			}
		}
		else if(strcmp(junk,"INT_No._of_s.d._for_error_bars") == 0) fscanf(fpp,"%d",&nsig);
		else if(strcmp(junk,"INT_Calculate_training_error_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				OPT_say_train_e = 1;
			}
		}
		else if(strcmp(junk,"INT_Calculate_noise_level_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) INT_find_sigma_nu = 1;
		}
		else if(strcmp(junk,"INT_noise_free_predictions_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ) noise_free_predictions = 1;
		}

		/* Program Output to screen */

		else if( strcmp(junk,"Verbose_(0/1/2)") == 0 ){
			fscanf(fpp,"%s",junk);
			verbose = atoi(junk);
		}

		/* Priors */

		else if(strcmp(junk,"prior_on_length_scales_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				lengthprior = 1;
				gamr_a = dmatrix(1,ngauss,1,mInputDimension);
				gamr_b = dmatrix(1,ngauss,1,mInputDimension);
				if( use_poly_length_scales == 1 ){
					gamr2_a = dmatrix(1,ngauss,1,mInputDimension);
					gamr2_b = dmatrix(1,ngauss,1,mInputDimension);
				}
			}
		}
		else if(strcmp(junk,"prior_on_noise_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				noiseprior = 1;
			}
		}
		else if(strcmp(junk,"prior_on_theta1_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				theta1prior = 1;
			}
		}
		else if(strcmp(junk,"prior_on_theta2_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				theta2prior = 1;
			}
		}
		else if(strcmp(junk,"prior_on_linear_term_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				linearprior = 1;
			}
		}
		else if(strcmp(junk,"prior_on_theta0_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				theta0prior = 1;
			}
		}
		else if( strcmp(junk,"noise_prior_mean") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamn_a = -1.0; /* the a value keeps track of whether default vals
							   are to be used */
			} else {
				gamn_a = atof(junk);
			}
		}
		else if( strcmp(junk,"noise_prior_sd") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamn_a = -1.0;
			} else {
				gamn_b = atof(junk);
			}
		}
		else if( strcmp(junk,"noise_prior_dof") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamn_a = -1.0;
			} else {
				gamn_b = atof(junk);
			}
			if( noiseprior == 1 ) noiseprior = 2;
		}
		else if( (strcmp(junk,"noise_prior_cbja")==0) && (noiseprior>=1) ){
			fscanf(fpp,"%s",junk);
			gamn_a = atof(junk);
			noiseprior = 3;
		}/* CBJ */
		else if( (strcmp(junk,"noise_prior_cbjb")==0) && (noiseprior>=1) ){
			fscanf(fpp,"%s",junk);
			gamn_b = atof(junk);
			noiseprior = 3;
		}/* CBJ */
		else if( strcmp(junk,"theta1_prior_mean") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamt1_a = -1.0;
			} else {
				gamt1_a = atof(junk);
			}
		}
		else if( strcmp(junk,"theta1_prior_sd") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamt1_a = -1.0;
			} else {
				gamt1_b = atof(junk);
			}
		}
		else if( strcmp(junk,"theta1_prior_dof") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamt1_a = -1.0;
			} else {
				gamt1_b = atof(junk);
			}
			if( theta1prior == 1 ) theta1prior = 2;
		}
		else if( strcmp(junk,"theta2_prior_mean") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamt2_a = -1.0;
			} else {
				gamt2_a = atof(junk);
			}
		}
		else if( strcmp(junk,"theta2_prior_sd") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamt2_a = -1.0;
			} else {
				gamt2_b = atof(junk);
			}
		}
		else if( strcmp(junk,"theta2_prior_dof") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gamt2_a = -1.0;
			} else {
				gamt2_b = atof(junk);
			}
			if( theta2prior == 1 ) theta2prior = 2;
		}
		else if( strcmp(junk,"length_prior_mean") == 0 && lengthprior == 1){
			for(i=1; i <= ngauss; ++i){
				fscanf(fpp,"%s",junk);
				for(j=1; j <= mInputDimension; ++j){
					if( strcmp(junk,"d") == 0 ){
						gamr_a[i][j] = -1.0;
						gamr_b[i][j] = -1.0;
					} else {
						gamr_a[i][j] = atof(junk);
					}
				}
			}
		}
		else if( strcmp(junk,"length_prior_sd") == 0 && lengthprior == 1){
			for(i=1; i <= ngauss; ++i){
				fscanf(fpp,"%s",junk);
				for(j=1; j <= mInputDimension; ++j){
					if( strcmp(junk,"d") == 0 ){
						gamr_a[i][j] = -1.0;
						gamr_b[i][j] = -1.0;
					} else {
						gamr_b[i][j] = atof(junk);
					}
				}
			}
		}
		else if( strcmp(junk,"length_prior_mean2") == 0 && lengthprior == 1){
			for(i=1; i <= ngauss; ++i){
				fscanf(fpp,"%s",junk);
				for(j=1; j <= mInputDimension; ++j){
					if( strcmp(junk,"d") == 0 ){
						gamr2_a[i][j] = -1.0;
						gamr2_b[i][j] = -1.0;
					} else {
						gamr2_a[i][j] = atof(junk);
					}
				}
			}
		}
		else if( strcmp(junk,"length_prior_sd2") == 0 && lengthprior == 1){
			for(i=1; i <= ngauss; ++i){
				fscanf(fpp,"%s",junk);
				for(j=1; j <= mInputDimension; ++j){
					if( strcmp(junk,"d") == 0 ){
						gamr2_a[i][j] = -1.0;
						gamr2_b[i][j] = -1.0;
					} else {
						gamr2_b[i][j] = atof(junk);
					}
				}
			}
		}
		else if( strcmp(junk,"length_prior_dof") == 0 && lengthprior == 1){
			for(i=1; i <= ngauss; ++i){
				fscanf(fpp,"%s",junk);
				for(j=1; j <= mInputDimension; ++j){
					if( strcmp(junk,"d") == 0 ){
						gamr_a[i][j] = -1.0;
						gamr_b[i][j] = -1.0;
					} else {
						gamr_b[i][j] = atof(junk);
					}
				}
			}
			if( lengthprior == 1 ) lengthprior = 2; 
		}
		else if( strcmp(junk,"specific_length_prior_mean") == 0 && lengthprior == 1){
			fscanf(fpp,"%s",junk);
			i = atoi(junk);
			for(j=1; j <= ngauss; ++j){
				fscanf(fpp,"%s",junk);
				if( strcmp(junk,"d") == 0 ){
					gamr_a[j][i] = -1.0;
					gamr_b[j][i] = -1.0;
				} else {
					gamr_a[j][i] = atof(junk);
				}
			}
		}
		else if( strcmp(junk,"specific_length_prior_sd") == 0 && lengthprior == 1){
			fscanf(fpp,"%s",junk);
			i = atoi(junk);
			for(j=1; j <= ngauss; ++j){
				fscanf(fpp,"%s",junk);
				if( strcmp(junk,"d") == 0 ){
					gamr_a[j][i] = -1.0;
					gamr_b[j][i] = -1.0;
				} else {
					gamr_b[j][i] = atof(junk);
				}
			}
		}
		else if( strcmp(junk,"specific_length_prior_dof") == 0 && lengthprior == 1){
			fscanf(fpp,"%s",junk);
			i = atoi(junk);
			for(j=1; j <= ngauss; ++j){
				fscanf(fpp,"%s",junk);
				if( strcmp(junk,"d") == 0 ){
					gamr_a[j][i] = -1.0;
					gamr_b[j][i] = -1.0;
				} else {
					gamr_b[j][i] = atof(junk);
				}
			}
			if( lengthprior == 1 ) lengthprior = 2;
		}
		else if( strcmp(junk,"specific_length_prior_mean2") == 0 && lengthprior == 1){
			fscanf(fpp,"%s",junk);
			i = atoi(junk);
			for(j=1; j <= ngauss; ++j){
				fscanf(fpp,"%s",junk);
				if( strcmp(junk,"d") == 0 ){
					gamr2_a[j][i] = -1.0;
					gamr2_b[j][i] = -1.0;
				} else {
					gamr2_a[j][i] = atof(junk);
				}
			}
		}
		else if( strcmp(junk,"specific_length_prior_sd2") == 0 && lengthprior == 1){
			fscanf(fpp,"%s",junk);
			i = atoi(junk);
			for(j=1; j <= ngauss; ++j){
				fscanf(fpp,"%s",junk);
				if( strcmp(junk,"d") == 0 ){
					gamr2_a[j][i] = -1.0;
					gamr2_b[j][i] = -1.0;
				} else {
					gamr2_b[j][i] = atof(junk);
				}
			}
		}
		else if( strcmp(junk,"linear_prior_mean") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gaussl_m = -1.0;
			} else {
				gaussl_m = atof(junk);
			}
		}
		else if( strcmp(junk,"linear_prior_sd") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gaussl_m = -1.0;
			} else {
				gaussl_sd = atof(junk);
			}
		}
		else if( strcmp(junk,"theta0_prior_mean") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gausst0_m = -1.0;
			} else {     
				gausst0_m = atof(junk);
			}
		}
		else if( strcmp(junk,"theta0_prior_sd") == 0 ){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"d") == 0 ){
				gausst0_m = -1.0;
			} else {
				gausst0_sd = atof(junk);
			}
		}

		/* Defaults */

		else if( strcmp(junk,"length_scale_compression") == 0 ){
			fscanf(fpp,"%s",junk);
			stuff = atof(junk);
		}
		else if( strcmp(junk,"initial_theta2") == 0 ){
			fscanf(fpp,"%s",junk);
			D[1] = atof(junk);
		}
		else if( strcmp(junk,"initial_noise_variance") == 0 ){
			fscanf(fpp,"%s",junk);
			D[2] = atof(junk);
		}
		else if( strcmp(junk,"initial_theta0") == 0 ){
			fscanf(fpp,"%s",junk);
			D[3] = atof(junk);
		}
		else if(strcmp(junk,"maccheckgrad_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				OPT_checkgrad = 1;
			}
		}
		else if( strcmp(junk,"maccheckgrad_step") == 0 ){
			fscanf(fpp,"%s",junk);
			mac_ep = atof(junk);
		}

		else if(strcmp(junk,"Perform_k_x_Optimization_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				do_SEARCH = 1; 
				/* koptONE - see also koptTWO */
				if( max_kx == NULL ) {
					max_kx = dvector(1,mInputDimension);
					for (i=1; i <= mInputDimension; ++i){
						max_kx[i] = 0.0 ; /* start from origin by default */
					}
				}
				if( max_include == NULL ) {
					max_include = ivector(1,mInputDimension);
					for (i=1; i <= mInputDimension; ++i){
						max_include[i] = 1 ; /* all are included by default */
					}
				}
			}
		}
		else if(strcmp(junk,"KOPT_objective_function_(1/2/3/4)") == 0){
			fscanf(fpp,"%s",junk);
			if( do_SEARCH > 0 ){
				do_SEARCH = atoi(junk);
			} 
		}
		else if(strcmp(junk,"KOPT_minimize_or_maximize_(min/max)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"min") == 0 ){
				SEARCH_max = 0;
			} else SEARCH_max = 1;
		} 
		else if( strcmp(junk,"KOPT_No._of_mac_its") == 0){
			fscanf(fpp,"%s",junk);
			max_itmax = atoi(junk);
		}
		else if( strcmp(junk,"KOPT_beta") == 0){
			fscanf(fpp,"%s",junk);
			max_beta = atof(junk);
		}
		else if( strcmp(junk,"KOPT_gamma") == 0 ){
			fscanf(fpp,"%s",junk);
			max_gam = atof(junk);
		}
		else if( strcmp(junk,"KOPT_starting_point") == 0 ) {
			for(i=1; i <= mInputDimension; ++i){
				fscanf(fpp,"%s",junk);
				if ( max_kx != NULL )        max_kx[i] = atof(junk);
			}
		}
		else if( strcmp(junk,"KOPT_include") == 0 ) {
			for(i=1; i <= mInputDimension; ++i){
				fscanf(fpp,"%s",junk);
				if ( max_include != NULL )
					max_include[i] = atoi(junk);
			}
		}
		else if(strcmp(junk,"KOPT_logfile") == 0) fscanf(fpp,"%s",max_logfile);
		else if(strcmp(junk,"KOPT_make_log_(yes/no)") == 0){
			fscanf(fpp,"%s",junk);
			if( strcmp(junk,"yes") == 0 ){
				max_log = 1;
			}
		}
		else if ( 
			strcmp(junk,"--------------------------------------------------------------") == 0 ||
			strcmp(junk,"---------------------------------------------------------------") == 0 ||
			strcmp(junk,"-----------------") == 0 ||
			strcmp(junk,"-------------") == 0 ||
			strcmp(junk,"--------------") == 0 ||
			strcmp(junk,"Specification") == 0 ||
			strcmp(junk,"File") == 0 ||
			strcmp(junk,"Tpros") == 0 ||
			strcmp(junk,"Noise_Model") == 0 ||
			strcmp(junk,"Model") == 0 ||
			strcmp(junk,"Files") == 0 ||
			strcmp(junk,"Data") == 0 ||
			strcmp(junk,"Covariance_Function") == 0 ||
			strcmp(junk,"Optimisation") == 0 ||
			strcmp(junk,"Optimization") == 0 ||
			strcmp(junk,"Interpolation") == 0 ||
			strcmp(junk,"Input_Optimization") == 0 ||
			strcmp(junk,"\n") == 0 ||
			strcmp(junk,"Priors") == 0
			){
				/* ignore */
		}
		else {
			//printf ("Ignored entry in %s: >%s<\n", file ,junk ) ;
		}

	}

	fclose(fpp);

}

void tpros::c_line(int argc,char *argv[],double *D)
{
	int   i,j,k;

	/* We skip the first one as this is the spec file */

	for(i=2; i <= argc-1; ++i){

		/* Input file names */

		if(strcmp(argv[i],"-tf") == 0){
			sprintf(datafile,"%s",argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-gf") == 0){ 
			sprintf(gridfile,"%s",argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-rbf") == 0){
			sprintf(rbffile,"%s",argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-ho") == 0){
			hyp_out = 1;
			sprintf(hypoutfile,"%s",argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-int_of") == 0){
			sprintf(outfile,"%s",argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-int_gf") == 0){
			sprintf(igridfile,"%s",argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-int_tf") == 0){
			sprintf(idatafile,"%s",argv[i+1]);
			INT_load_targets = 1;
			i += 1;
		} 
		else if(strcmp(argv[i],"-hi") == 0){
			sprintf(hypinfile,"%s",argv[i+1]);
			hyp_in = 1;
			i += 1;
		} 

		/* Data */

		else if(strcmp(argv[i],"-seed") == 0){
			SEED = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-nh") == 0){
			if(mInputDimension==0)
				mInputDimension = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-ntdat") == 0){
			if(mNumTrainingData ==0)
				mNumTrainingData = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-ninter") == 0){
			ninter = atoi(argv[i+1]);
			i += 1;
		} 

		/* Covariance Function */

		else if(strcmp(argv[i],"-ngauss") == 0){
			ngauss = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-npoly") == 0){
			npoly = atoi(argv[i+1]);
			if( npoly > 1 ) use_poly_length_scales = 1;
			i += 1;
		} 
		else if(strcmp(argv[i],"-lin") == 0){
			C_linear = 1;
		} 
		else if(strcmp(argv[i],"-Nlin") == 0){
			C_linear = 0;
		} 
		else if(strcmp(argv[i],"-NOperiod") == 0){
			C_periodic = 0;
		} 
		else if(strcmp(argv[i],"-wavel") == 0){
			if( C_periodic == 1 ){
				for(j=1; j <= mInputDimension; ++j){
					wavel[j] = atof(argv[i+j]);
				}
				i += mInputDimension;
			}
			if( C_periodic != 1 ){
				printf("ERROR : It is not possible to respecify wavelengths\n on the command line without turning on the period\n option within the SPECFILE. Exiting Code.\n");
				Msg::error("tpros error");
			}
		} 

		/* Noise Model */

		else if(strcmp(argv[i],"-nhidden") == 0){
			nhidden = atoi(argv[i+1]);
			i += 1;
		} 

		/* Optimisation */

		else if(strcmp(argv[i],"-opt") == 0){
			do_OPT = 1;
		} 
		else if(strcmp(argv[i],"-NOopt") == 0){
			do_OPT = 0;
		} 
		else if(strcmp(argv[i],"-opt_inv") == 0){
			if( strcmp(argv[i+1],"CG") == 0) OPT_use_cg = 1; 
			if( strcmp(argv[i+1],"LU") == 0) OPT_use_cg = 0; 
			i += 1;
		} 
		else if(strcmp(argv[i],"-opt_CGits") == 0){
			CG_opt = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-opt_CGtr") == 0){
			ntrace = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-opt_CGbnd") == 0){
			bnd_percent = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-opt_ls") == 0){
			optimize_lengthscales = 1;
		} 
		else if(strcmp(argv[i],"-opt_Cinvt") == 0){
			write_Cinv_t = 1;
			sprintf(Cinvtfile,"%s",argv[i+1]);
			i += 1;
		}
		else if(strcmp(argv[i],"-opt_evid") == 0){
			OPT_say_evidence = 1;
		} 
		else if(strcmp(argv[i],"-NOopt_ls") == 0){
			optimize_lengthscales = 0;
		} 
		else if(strcmp(argv[i],"-opt_t0") == 0){
			optimize_theta0 = 1;
		} 
		else if(strcmp(argv[i],"-NOopt_t0") == 0){
			optimize_theta0 = 0;
		} 
		else if(strcmp(argv[i],"-opt_t1") == 0){
			optimize_theta1 = 1;
		} 
		else if(strcmp(argv[i],"-NOopt_t1") == 0){
			optimize_theta1 = 0;
		} 
		else if(strcmp(argv[i],"-opt_t2") == 0){
			optimize_theta2 = 1;
		} 
		else if(strcmp(argv[i],"-NOopt_t2") == 0){
			optimize_theta2 = 0;
		} 
		else if(strcmp(argv[i],"-opt_n") == 0){
			optimize_noise = 1;
		} 
		else if(strcmp(argv[i],"-NOopt_n") == 0){
			optimize_noise = 0;
		} 
		else if(strcmp(argv[i],"-opt_lin") == 0){
			optimize_linear = 1;
		} 
		else if(strcmp(argv[i],"-NOopt_lin") == 0){
			optimize_linear = 0;
		} 
		else if(strcmp(argv[i],"-opt_its") == 0){
			opt_itmax = atoi(argv[i+1]);
			i += 1;
		} 

		/* Interpolation */

		else if(strcmp(argv[i],"-int") == 0){
			do_INT = 1;
		} 
		else if(strcmp(argv[i],"-NOint") == 0){
			do_INT = 0;
		} 
		else if(strcmp(argv[i],"-int_inv") == 0){
			if( strcmp(argv[i+1],"CG") == 0) INT_use_cg = 1; 
			if( strcmp(argv[i+1],"LUi") == 0){
				INT_LUi = 1;
				INT_use_cg = 0; 
			}
			if( strcmp(argv[i+1],"LUd") == 0){
				INT_LUi = 0;
				INT_use_cg = 0; 
			}
			i += 1;
		} 
		else if(strcmp(argv[i],"-int_CGits") == 0){
			CG_int = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-int_eb") == 0){
			INT_find_eb = 1;
		} 
		else if(strcmp(argv[i],"-NOint_eb") == 0){
			INT_find_eb = 0;
		} 
		else if(strcmp(argv[i],"-int_te") == 0){
			OPT_say_train_e = 1;
		} 
		else if(strcmp(argv[i],"-NOint_te") == 0){
			OPT_say_train_e = 0;
		} 
		else if(strcmp(argv[i],"-ebsd") == 0){
			nsig = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-int_n") == 0){
			INT_find_sigma_nu = 1;
		} 
		else if(strcmp(argv[i],"-NOint_n") == 0){
			INT_find_sigma_nu = 0;
		} 
		else if(strcmp(argv[i],"-int_nf") == 0){
			noise_free_predictions = 1;
		} 

		/* Program output to screen */

		else if(strcmp(argv[i],"-verb") == 0){
			verbose = atoi(argv[i+1]);
			i += 1;
		}

		/* Priors */

		else if(strcmp(argv[i],"-plen") == 0){
			lengthprior = 1;
			gamr_a = dmatrix(1,ngauss,1,mInputDimension);
			gamr_b = dmatrix(1,ngauss,1,mInputDimension);
		} 
		else if(strcmp(argv[i],"-NOplen") == 0){
			lengthprior = 0;
		} 
		else if(strcmp(argv[i],"-pn") == 0){
			noiseprior = 1;
		} 
		else if(strcmp(argv[i],"-NOpn") == 0){
			noiseprior = 0;
		} 
		else if(strcmp(argv[i],"-plin") == 0){
			linearprior = 1;
		} 
		else if(strcmp(argv[i],"-NOplin") == 0){
			linearprior = 0;
		} 
		else if(strcmp(argv[i],"-pt0") == 0){
			theta0prior = 1;
		} 
		else if(strcmp(argv[i],"-NOpt0") == 0){
			theta0prior = 0;
		} 
		else if(strcmp(argv[i],"-pt1") == 0){
			theta1prior = 1;
		} 
		else if(strcmp(argv[i],"-NOpt1") == 0){
			theta1prior = 0;
		} 
		else if(strcmp(argv[i],"-pt2") == 0){
			theta2prior = 1;
		} 
		else if(strcmp(argv[i],"-NOpt2") == 0){
			theta2prior = 0;
		} 
		else if(strcmp(argv[i],"-plm") == 0 && lengthprior == 1){
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1],"d") == 0){
					for(k=1; k <= mInputDimension; ++k){
						gamr_a[j][k] = -1.0;
					}
				} else {
					for(k=1; k <= mInputDimension; ++k){
						gamr_a[j][k] = atof(argv[i+1]);
					}
				}
				i += 1;
			}
		} 
		else if(strcmp(argv[i],"-plsd") == 0 && lengthprior == 1){
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1],"d") == 0){
					for(k=1; k <= mInputDimension; ++k){
						gamr_a[j][k] = -1.0;
					}
				} else {
					for(k=1; k <= mInputDimension; ++k){
						gamr_b[j][k] = atof(argv[i+1]);
					}
				}
				i += 1;
			}
		} 
		else if(strcmp(argv[i],"-plm2") == 0 && lengthprior == 1){
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1],"d") == 0){
					for(k=1; k <= mInputDimension; ++k){
						gamr2_a[j][k] = -1.0;
					}
				} else {
					for(k=1; k <= mInputDimension; ++k){
						gamr2_a[j][k] = atof(argv[i+1]);
					}
				}
				i += 1;
			}
		} 
		else if(strcmp(argv[i],"-plsd2") == 0 && lengthprior == 1){
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1],"d") == 0){
					for(k=1; k <= mInputDimension; ++k){
						gamr2_a[j][k] = -1.0;
					}
				} else {
					for(k=1; k <= mInputDimension; ++k){
						gamr2_b[j][k] = atof(argv[i+1]);
					}
				}
				i += 1;
			}
		} 
		else if(strcmp(argv[i],"-pldof") == 0 && lengthprior == 1){
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1],"d") == 0){
					for(k=1; k <= mInputDimension; ++k){
						gamr_a[j][k] = -1.0;
					}
				} else {
					for(k=1; k <= mInputDimension; ++k){
						gamr_b[j][k] = atof(argv[i+1]);
					}
				}
				i += 1;
			}
			if( lengthprior == 1 ) lengthprior = 2;
		} 
		else if(strcmp(argv[i],"-splm") == 0 && lengthprior == 1){
			k = atoi(argv[i+1]);
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1+j],"d") == 0){
					gamr_a[j][k] = -1.0;
				} else {
					gamr_a[j][k] = atof(argv[i+1+j]);
				}
			}
			i += 1+ngauss;
		}		 
		else if(strcmp(argv[i],"-splsd") == 0 && lengthprior == 1){
			k = atoi(argv[i+1]);
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1+j],"d") == 0){
					gamr_a[j][k] = -1.0;
				} else {
					gamr_b[j][k] = atof(argv[i+1+j]);
				}
			}
			i += 1+ngauss;
		}
		else if(strcmp(argv[i],"-splm2") == 0 && lengthprior == 1){
			k = atoi(argv[i+1]);
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1+j],"d") == 0){
					gamr2_a[j][k] = -1.0;
				} else {
					gamr2_a[j][k] = atof(argv[i+1+j]);
				}
			}
			i += 1+ngauss;
		}		 
		else if(strcmp(argv[i],"-splsd2") == 0 && lengthprior == 1){
			k = atoi(argv[i+1]);
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1+j],"d") == 0){
					gamr2_a[j][k] = -1.0;
				} else {
					gamr2_b[j][k] = atof(argv[i+1+j]);
				}
			}
			i += 1+ngauss;
		}
		else if(strcmp(argv[i],"-spldof") == 0 && lengthprior == 1){
			k = atoi(argv[i+1]);
			for(j=1; j <= ngauss; ++j){
				if(strcmp(argv[i+1+j],"d") == 0){
					gamr_a[j][k] = -1.0;
				} else {
					gamr_b[j][k] = atof(argv[i+1+j]);
				}
			}
			i += 1+ngauss;
			if( lengthprior == 1 ) lengthprior = 2;
		}		 
		else if(strcmp(argv[i],"-pnm") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamn_a = -1.0;
			else gamn_a = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-pnsd") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamn_a = -1.0;
			else gamn_b = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-pndof") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamn_a = -1.0;
			else gamn_b = atof(argv[i+1]);
			i += 1;
			if( noiseprior == 1 ) noiseprior = 2;
		} 
		else if(strcmp(argv[i],"-pt0m") == 0){
			if(strcmp(argv[i+1],"d") == 0) gausst0_m = -1.0;
			else gausst0_m = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-pt0sd") == 0){
			if(strcmp(argv[i+1],"d") == 0) gausst0_m = -1.0;
			else gausst0_sd = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-pt1m") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamt1_a = -1.0;
			else gamt1_a = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-pt1sd") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamt1_a = -1.0;
			else gamt1_b = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-pt1dof") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamt1_a = -1.0;
			else gamt1_b = atof(argv[i+1]);
			i += 1;
			if( theta1prior == 1 ) theta1prior = 2;
		} 
		else if(strcmp(argv[i],"-pt2m") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamt2_a = -1.0;
			else gamt2_a = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-pt2sd") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamt2_a = -1.0;
			else gamt2_b = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-pt2dof") == 0){
			if(strcmp(argv[i+1],"d") == 0) gamt2_a = -1.0;
			else gamt2_b = atof(argv[i+1]);
			i += 1;
			if( theta2prior == 1 ) theta2prior = 2;
		} 
		else if(strcmp(argv[i],"-plinm") == 0){
			if(strcmp(argv[i+1],"d") == 0) gaussl_m = -1.0;
			else gaussl_m = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-plinsd") == 0){
			if(strcmp(argv[i+1],"d") == 0) gaussl_m = -1.0;
			else gaussl_sd = atof(argv[i+1]);
			i += 1;
		} 

		/* Defaults */

		else if(strcmp(argv[i],"-stuff") == 0){
			stuff = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-t2") == 0){
			D[1] = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-t0") == 0){
			D[3] = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-nv") == 0){
			D[2] = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-tol") == 0){ /* included for reverse compatibility */
			mac_h_tol = atof(argv[i+1]);
			mac_k_tol = mac_h_tol ; 
			i += 1;
		} 
		else if(strcmp(argv[i],"-opt_tol") == 0){
			mac_h_tol = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-kopt_tol") == 0){
			mac_k_tol = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-opt_style") == 0){
			mac_h_style = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-kopt_style") == 0){
			mac_k_style = atoi(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-epsilon") == 0){
			mac_ep = atof(argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-check") == 0){
			OPT_checkgrad = 1;
		} 
		else if(strcmp(argv[i],"-NOcheck") == 0){
			OPT_checkgrad = 0;
		} 
		else if(strcmp(argv[i],"-known_n") == 0){
			training_noise_in_datafile = 1;
		} 
		else if(strcmp(argv[i],"-NOknown_n") == 0){
			training_noise_in_datafile = 0;
		} 
		else if(strcmp(argv[i],"-TCG") == 0){
			OPT_tridiag_CG = 1;
		} 
		else if(strcmp(argv[i],"-DCG") == 0){
			OPT_tridiag_CG = 0;
		} 
		else if(strcmp(argv[i],"-NOdump") == 0){
			write_hdump = 0;
		} 
		else if(strcmp(argv[i],"-dump") == 0){
			write_hdump = 1;
		} 
		else if(strcmp(argv[i],"-samp") == 0){
			do_samples = 1;
		} 
		else if(strcmp(argv[i],"-kopt") == 0){
			/* koptTWO - see also koptONE */
			if( max_kx == NULL ) {
				max_kx = dvector(1,mInputDimension);
				for (j=1; j <= mInputDimension; ++j){
					max_kx[j] = 0.0 ; /* start from origin by default */
				}
			}
			if( max_include == NULL ) {
				max_include = ivector(1,mInputDimension);
				for (j=1; j <= mInputDimension; ++j){
					max_include[j] = 1 ; /* all are included by default */
				}
			}
			do_SEARCH = atoi(argv[i+1]);
			i += 1;
		}
		else if(strcmp(argv[i],"-NOkopt") == 0){
			do_SEARCH = 0;
		}
		else if(strcmp(argv[i],"-kopt_min") == 0){
			SEARCH_max = 0;
		}
		else if(strcmp(argv[i],"-kopt_max") == 0){
			SEARCH_max = 1;
		}
		else if(strcmp(argv[i],"-kopt_log") == 0){
			max_log = 1;
		}
		else if(strcmp(argv[i],"-kopt_logfile") == 0){
			max_log = 1;
			sprintf(max_logfile,"%s",argv[i+1]);
			i += 1;
		} 
		else if(strcmp(argv[i],"-kopt_itmax") == 0){
			max_itmax = atoi(argv[i+1]);
			i += 1;
		}
		else if(strcmp(argv[i],"-kopt_beta") == 0){
			max_beta = atof(argv[i+1]);
			i += 1;
		}
		else if(strcmp(argv[i],"-kopt_gam") == 0){
			max_gam = atof(argv[i+1]);
			i += 1;
		}
		else if(strcmp(argv[i],"-kopt_start") == 0){
			if( max_kx == NULL ) max_kx = dvector(1,mInputDimension);
			for(j=1; j <= mInputDimension; ++j){
				max_kx[j] = atof(argv[i+j]);
			}
			i += mInputDimension;
		}
		else if(strcmp(argv[i],"-kopt_include") == 0){
			if( max_include == NULL ) max_include = ivector(1,mInputDimension);
			for(j=1; j <= mInputDimension; ++j){
				max_include[j] = atoi(argv[i+j]);
			}
			i += mInputDimension;
		}

		/* Catch unknown command line arguments */

		else {
			printf("\nERROR : Unknown command line argument %s - Exit to System\n\n",argv[i]);
			Msg::error("tpros error");
		}

	}

}

void tpros::defaults(double *D)
{
	mac_k_tol = 1.0e-3;
	mac_k_style = 1 ; /* 1 = tolerance defined by small step in parameter space */
	mac_h_style = 0 ; /* 0 = tolerance defined by small gradient size */
	mac_h_tol = 1.0e-4;
	mac_ep  = 0.001;

	ngauss = 1;  /* Default of 1 gaussian in covariance function */
	npoly  = 1;  /* Default of 1 basis function in length scale */
	nhidden = 1;  /* Default of spatial invariant noise model */

	bnd_percent = 0.05;

	stuff = 4.0; /* default compression */

	OPT_say_train_e = 1; /* calculate training error as standard */
	OPT_tridiag_CG = 1; /* Use TCG inversion as standard */

	/*
	we set these three elements to particular values to check whether the 
	user has specified them in the readInSpecfile or c_line routines. 
	*/
	D[1] = -1.0; 
	D[2] = -1.0;
	D[3] = -0.123456789;
}

void tpros::err_catch( void )
{

	if( (OPT_use_cg == 1 || INT_use_cg == 1) && nhidden > 1 ){
		printf("\nERROR : Cannot have advanced noise model with CGinversion\n\n");
		Msg::error("tpros error");
	}

	if( training_noise_in_datafile == 1 && optimize_noise == 1 ){
		printf("\nERROR : Known training noise levels can't be optimized\n\n");
		Msg::error("tpros error");
	}

}

void tpros::ls_error( void )
{
	int i,j,k,l,m;
	int tint2; /* djcm removed "nr" variable 99 11 30 */
	double *dM1,*dM2;
	double *w,*gr;
	double *d,*e;
	double **H,**H_inv,**H_temp;
	double temp1;
	gq_args param;
	param.all=this;

	nw = mInputDimension*ngauss*npoly;

	/* Set up kth[][] matrix */

	kth = imatrix(1,nw,1,5);

	if( optimize_lengthscales == 1 ){
		for(i=1; i <= ngauss; ++i){
			for(j=1; j <= mInputDimension; ++j){
				for(k=1; k <= npoly; ++k){
					tint2 = (i-1)*mInputDimension*npoly + (j-1)*npoly + k;
					kth[tint2][1] = 1;
					kth[tint2][2] = tint2;
					kth[tint2][3] = i;
					kth[tint2][4] = j;
					kth[tint2][5] = k;
				}
			}
		}
	}

	dM1   = dvector(1,nw);
	dM2   = dvector(1,nw);
	w     = dvector(1,nw);
	gr    = dvector(1,nw);
	d     = dvector(1,npoly);
	e     = dvector(1,npoly);
	H     = dmatrix(1,npoly,1,npoly);
	H_temp = dmatrix(1,npoly,1,npoly);
	H_inv = dmatrix(1,npoly,1,npoly);

	for(i=1; i <= ngauss; ++i){
		for(j=1; j <= mInputDimension; ++j){

			/* 
			This is set up to calculate the ls_Hessian with respect to
			the parameters of the polynomial length scales.
			*/

			for(k=1; k <= npoly; ++k){

				/* we will use mac_ep here */

				tint2 = k + ((j-1)*npoly) + ((i-1)*mInputDimension*npoly);

				hyp[tint2] += mac_ep;
				w_con_in(w);	      
				mac_dif(w,dM1,(void *)(&param));
				hyp[tint2] -= mac_ep;
				hyp[tint2] -= mac_ep;
				w_con_in(w);	      
				mac_dif(w,dM2,(void *)(&param));
				hyp[tint2] += mac_ep;

				for(l=1; l <= npoly; ++l){
					H[l][tint2] = (dM1[l]-dM2[l])/(mac_ep*2.0);
					H_temp[l][tint2] = H[l][tint2];
				}

			}

			/* 
			Now we calculate the gradient of the length scales at the most 
			probable pararmeter vector gr[] and calculate the error bars
			on the length scales 
			*/

			tred2(H_temp,npoly,d,e);
			tqli(d,e,npoly,H_temp);

			LU_invert(H,H_inv,npoly);
			w_con_in(w);

			tint2 = j + ((i-1)*mInputDimension);

			for(k=1; k <= ninter; ++k){

				for(l=1; l <= mInputDimension; ++l){
					k_x[mNumTrainingData+1][l] = ok_x[k][l];
				}
				PL_calc(mNumTrainingData+1); 

				for(l=1; l <= npoly; ++l){
					/* We are calculating the derivative of the exponent because
					we want to ensure that the error bars stay positive too */
					gr[l] = PL[mNumTrainingData+1][l];
				}

				temp1 = 0.0;
				for(l=1; l <= npoly; ++l){
					for(m=1; m <= npoly; ++m){
						temp1 += gr[l]*gr[m]*H_inv[l][m];
					}
				}
				if( temp1 < 0.0 ){
					temp1 = 0.0;
					printf("WARNING : suspected precision limited reached\n");
				}
				temp1 = pow( temp1, 0.5 );

				ls_e[tint2][k] = temp1;

			}

		}
	}

	free_dvector(dM1,1,nw);
	free_dvector(dM2,1,nw);
	free_dvector(w,1,nw);
	free_dvector(gr,1,nw);
	free_dmatrix(H,1,npoly,1,npoly);
	free_dmatrix(H_inv,1,npoly,1,npoly);
	free_imatrix(kth,1,nw,1,5);
}

double tpros::sigmoid(double x)
{
	double sig;

	sig = 1.0 / ( 1.0 + exp(-x));  

	return sig;
}

double tpros::difsig(double x)
{
	double dif;

	dif = exp(-x) / pow(( 1.0 + exp(-x) ) , 2.0);

	return dif;
}

void tpros::T_return(double *w, int n, void *arg)
{
	return ((gq_args*)arg)->all->T_return_member(w,n);
}
void tpros::T_return_member(double *w, int n)
/* this routine is called by the
macopt optimizer ; it reports parameter
values during the optimization */
{
	int i,j,k;
	int tint;
	FILE *fp1;

	if( write_hdump == 1 ){ /* record the hyperparameters  */

		w_con_out(w);

		fp1 = fopen( hypdump, "a" );

		fprintf(fp1,"%d ",n);    

		for(i=1; i <= ngauss; ++i){
			for(j=1; j <= mInputDimension; ++j){
				for(k=1; k <= npoly; ++k){
					tint = (i-1)*mInputDimension*npoly + (j-1)*npoly + k;
					fprintf(fp1,"%f ",exp(hyp[tint]));
				}
			}
		}
		for(i=1; i <= ngauss; ++i){
			tint = N_LENGTH_SCALES+i;
			fprintf(fp1,"%f ",hyp[tint]);
		}
		fprintf(fp1,"%f ",HYP_THETA_2);
		if( nhidden == 1 ){
			fprintf(fp1,"%f ",HYP_LOG_NOISE_VAR);
		}    
		if( nhidden > 1 ){
			for(i=1; i <= nhidden; ++i){
				for(j=1; j <= mInputDimension+1; ++j){
					tint = N_before_NOISE+(i-1)*(mInputDimension+1)+j;
					fprintf(fp1,"%f ",hyp[tint]);
				}
			}
			for(i=1; i <= nhidden+1; ++i){
				tint = N_before_NOISE+((mInputDimension+1)*nhidden)+i;
				fprintf(fp1,"%f ",hyp[tint]);
			}
		}
		for(i=1; i <= nlin; ++i){
			tint = N_including_NOISE+i;
			fprintf(fp1,"%f ",hyp[tint]);    
		}
		fprintf(fp1,"%f ",HYP_THETA_0);
		fprintf(fp1,"\n");    

		fflush( fp1 );
		fclose( fp1 );

	}

}

void tpros::ls_calc( void )
{
	int l,k;
	int tint;

	for(l=1; l <= ngauss; ++l){	    
		tint = (l-1)*mInputDimension;
		for(k=1; k <= mInputDimension; ++k){	      
			ls[tint+k] = exp(2.0*hyp[tint+k]);	  
		}
	}

}

/* 
Routine to generate random samples from a GP 

Produces two samples from present hyp[] settings
and then dumps that into file "blob"
*/
void tpros::sample( void )
{
	int    i,j,k;
	double *EIG,*e;
	FILE   *fp1;
	mac_inter_C( );

	LU_invert(C,C_inv,mNumTrainingData);

	for(i=1; i <= mNumTrainingData; ++i){
		for(j=1; j <= mNumTrainingData; ++j){
			C_temp[i][j] = C_inv[i][j];
		}
	}

	e = dvector(1,mNumTrainingData);
	EIG = dvector(1,mNumTrainingData);

	tred2(C_temp,mNumTrainingData,EIG,e);
	tqli(EIG,e,mNumTrainingData,C_temp);

	fp1 = fopen( outfile, "w" );
	for(k=1; k <= 2; ++k) {

		for(i=1; i <= mNumTrainingData; ++i){
			e[i] = rand_gaussian( ) * pow( EIG[i], -0.5 );
		}

		for(i=1; i <= mNumTrainingData; ++i){
			t[i] = 0.0;
			for(j=1; j <= mNumTrainingData; ++j){
				t[i] += C_temp[i][j]*e[j];
			}
			fprintf(fp1,"%f %f\n",k_x[i][1],t[i]);
		}
		fprintf(fp1,"\n");

	}
	fclose( fp1 );

	Msg::error("tpros error");
}

/* routines for maximizing output function 
w.r.t. inputs */

void  tpros::find_row_of_covariance_matrix ( double *kappa ) {
	int k,j,m , tint ;
	double temp1 , temp2 , temp3 , temp4 , temp5 , factor ; 

	for(j=1; j <= mNumTrainingData; ++j){
		kappa[j] = 0.0;
		for(m=1; m <= ngauss; ++m){/* compute exp(...) terms in kappa */
			tint = (m-1)*mInputDimension;
			temp1 = 0.0;
			temp2 = 1.0;

			for(k=1; k <= mInputDimension; ++k){
				if( use_poly_length_scales == 0 ){
					temp5 = exp(2.0*hyp[tint+k]);
				}
				if( use_poly_length_scales == 1 ){ /* Polynomial length scales */
					temp3 = PL_poly(mNumTrainingData+1,m,k);
					temp4 = PL_poly(j,m,k);
					temp5 = (temp3*temp3 + temp4*temp4);
					temp2 *= ( temp3*temp4/temp5 );
				}
				if( C_periodic == 0 ){
					temp1 += pow( (k_x[mNumTrainingData+1][k] - k_x[j][k]) , 2.0 ) / temp5;
				}
				if( C_periodic == 1 ){
					temp3 = (k_x[mNumTrainingData+1][k] - k_x[j][k]) * wavel[k];
					temp1 += pow( sin(temp3) , 2.0 ) / temp5;
				}
				/*	mls[k] = temp5;	 */
			}

			if( use_poly_length_scales == 1 ){
				temp2 = pow(temp2 , 0.5) * root2tonh;
				factor = temp2 * exp( - temp1 ) ;
			} else if( use_poly_length_scales == 0 ){
				factor =  temp2 * exp( - 0.5 * temp1 ) ; 
			}
			kappa[j] += hyp[N_LENGTH_SCALES+m] *  factor ; 

		}

		kappa[j] += HYP_THETA_2;

		if( C_linear == 1 ){
			for(k=1; k <= nlin; ++k){
				kappa[j] += hyp[N_including_NOISE+k] * k_x[j][k] * k_x[mNumTrainingData+1][k];
			}
		}

	} /* the k vector for this input is now computed */

}

void  tpros::find_predictive_mean_and_var ( double *Cinv_t ,
										   double *kappa , double *mean , 
										   double *variance , double *scratch ) {
											   /* at present this subroutine assumes that the input vector
											   we are interested in is in k_x[mNumTrainingData+1] ; this should be 
											   made more general */
											   int i ; 
											   /* Calculation of function value and variance */

											   *mean = 0.0; 
											   *variance = 0.0; 

											   /* mean */
											   for(i=1; i <= mNumTrainingData; ++i){
												   *mean += kappa[i]*Cinv_t[i];
											   }
											   *mean += HYP_THETA_0;

											   /* standard deviation */
											   /* first calculate the on-diagonal element kappa */

											   if ( noise_free_predictions ) {
												   *variance = 0.0 ;
											   } else {
												   *variance = noise(mNumTrainingData+1); 
											   }
											   for(i=1; i <= ngauss; ++i){
												   *variance += hyp[ N_LENGTH_SCALES + i ]; /* each Gaussian contributes 
																							THETA_1 */
											   }
											   *variance += HYP_THETA_2;
											   if( C_linear == 1 ){
												   for(i=1; i <= nlin; ++i){
													   *variance += hyp[N_including_NOISE+i]*k_x[mNumTrainingData+1][i]*k_x[mNumTrainingData+1][i];
												   }
											   }
											   /* was  LU_apply(C,kappa,mv2,mNumTrainingData); */

											   for(i=1; i <= mNumTrainingData; ++i){
												   scratch[i] = kappa[i];
											   }

											   lubksb( C_temp , mNumTrainingData , indx , scratch ) ;
											   for(i=1; i <= mNumTrainingData; ++i){
												   *variance -= scratch[i] * kappa[i];
											   }
}


void tpros::state_printer ( double *w, int its, void *arg )
{
	return ((gq_args*)arg)->all->state_printer_member(w, its);
}
void tpros::state_printer_member ( double *w, int its)
{
	/* this routine is called by the
	macopt optimizer ; it reports parameter
	values during the optimization */

	int i ; 
	FILE *fp1 ;
	int n = Ivary;

	printf ( "\n# Line minimization %d, from (only adjustable params are given)\n" , its );
	for ( i = 1 ; i <= n ; i++ ) printf ( "%8.5g " , w[i] ) ;
	printf ( "\n" ) ;

	if ( max_log ) {
		fp1=fopen(  max_logfile , "a" ) ;
		fprintf ( fp1 ,  "%2d " , its );
		for ( i = 1 ; i <= n ; i++ ) fprintf ( fp1 , "%9.5g " , w[i] ) ;
		fprintf ( fp1 , "\n" ) ;

		fclose ( fp1 ) ;
	} 

}

void tpros::maxim( void ) /* optimizes the input variables to maximize or
						  minimize expectn of function of the output */
{
	int i, ivary  ; /* global Ivary is the total number of things varying */
	double *w , temp1 ;
	macopt_args macp;
	gq_args     param;
	param.all=this;
	FILE *fp1 ;

	param.verbosity = 0 ; 
	if ( max_log ) {
		fp1=fopen(  max_logfile , "w" ) ;
		fclose (fp1) ;
	} 


	mk_p  = dvector(1,mNumTrainingData);
	mv    = dvector(1,mNumTrainingData);
	mv2   = dvector(1,mNumTrainingData);

	w     = dvector(1,mInputDimension);

	macopt_defaults( &macp );

	macp.tol     = mac_k_tol;
	if ( max_itmax > 0 ) macp.itmax   = max_itmax;
	macp.rich    = 2;
	macp.verbose = verbose;
	macp.end_if_small_step = mac_k_style ;

	macp.do_newitfunc = 1 ;  /* arrange for a subroutine to happen each itn */
	macp.newitfunc = &state_printer ;
	macp.newitfuncarg = (void *)(&param) ; 

	for(i=1 , ivary=0 ; i <= mInputDimension; ++i) {/* set initial condition in 
													input space */
		if ( max_include[i] ) {
			ivary ++ ;
			w[ivary] = max_kx[i]; 
		}
	}
	Ivary = ivary ; 

	/* we can now allocate memory for the derivative vector */

	mdk_p = dmatrix(1,Ivary,1,mNumTrainingData);

	/* once only, compute the      LU decomposition needed for     C     */

	/* LU inversion with direct method */
	for(i=1; i <= mNumTrainingData; ++i){
		mv2[i] = t[i] - HYP_THETA_0;
	}
	LU_apply(C,mv2,mv,mNumTrainingData); /* this sets mv to C^{-1}t */

	if( OPT_checkgrad == 1 ){ 
		maccheckgrad( w , Ivary , mac_ep , max_func , (void *)(&param) ,
			&max_dif , (void *)(&param) , 0 );
	}
	/* optimize the inputs! */
	macoptII( w , Ivary , max_dif , (void *)(&param) , &macp  ); 

	if( OPT_checkgrad == 1 ){
		maccheckgrad( w , Ivary , mac_ep , &max_func , (void *)(&param) ,
			&max_dif , (void *)(&param) , 0 );
	}
	report_input_vector ( stdout , w , 0 ) ;
	param.verbosity = 1 ; 

	temp1 = max_func(w,(void *)(&param)) ;
	if( SEARCH_max == 0 ) printf("\nminimized value : ");
	if( SEARCH_max == 1 ) printf("\nmaximized value : ");
	if ( SEARCH_max ) temp1 = - temp1 ;
	printf("%11.5g\n", temp1 ); 

	if ( max_log ) {
		fp1=fopen(  max_logfile , "a" ) ;
		report_input_vector ( fp1 , w , 1 ) ;
		fprintf(fp1 , "# %11.5g\n", temp1 ); 

		fclose (fp1) ;
	} 

}

void tpros::report_input_vector ( FILE *fp , double *w , int comments ) {
	int i , ivary ;

	if ( comments ) fprintf(fp,"\n#" ) ;
	else fprintf(fp,"\n" ) ;
	fprintf (fp,"Optimized inputs : \n");
	if ( comments ) fprintf(fp,"#" ) ;
	for(ivary=0 , i=1; i <= mInputDimension; ++i) {
		if ( max_include[i] ) {
			ivary ++ ;
			fprintf(fp,"%11.5g ",w[ivary]);
		}
		else {
			fprintf(fp,"%11.5g ", max_kx[i] ); /* unchanged value */
		}
	}
	fprintf(fp,"\n"); 
	if ( comments ) fprintf(fp,"#" ) ;
	for(ivary=0 , i=1; i <= mInputDimension; ++i) {
		if ( max_include[i] ) {
			ivary ++ ;
			fprintf(fp,"%11s ", " ");
		}
		else {
			fprintf(fp,"%11s ", " > fixed < " ); /* unchanged value */
		}
	}
	fprintf(fp,"\n"); 
}

void tpros::max_dif(double *w,double *dM,void *arg) 
{
	return ((gq_args*)arg)->all->mac_dif_member(w, dM);
}

void tpros::max_dif_member(double *w,double *dM) /* computes derivative
												 of utility with respect
												 to the varying inputs */

{
	int i,j,k,m,n, ivary ;
	int tint;
	double temp1,temp2,temp3,temp4,temp5 , factor ;

	/* globals defined for this routine 

	mk_p
	mv    = Cinv t
	mv2   = Cinv k_p
	mdk_p = differential of k_p
	mls   = square of length scale
	*/

	for( ivary=0 , i=1; i <= mInputDimension; ++i) { /* extract the varying parameters */
		/* DJCM 97/11/22 */
		if ( max_include[i] ) {
			ivary ++ ;
			k_x[mNumTrainingData+1][i] = w[ivary];
		} else {
			k_x[mNumTrainingData+1][i] = max_kx[i]; /* fixed initial value */
		}
	}

	/* was:   for(i=1; i <= mInputDimension; ++i) k_x[mNumTrainingData+1][i] = w[i]; */

	for(i=1; i <= Ivary; ++i){
		for(j=1; j <= mNumTrainingData; ++j){
			mdk_p[i][j] = 0.0;       /* initialize derivatives to zero */
		}
	}

	/* derivative evaluation */

	if( use_poly_length_scales == 1 ) PL_calc(mNumTrainingData+1); /* Polynomial length scales */

	find_row_of_covariance_matrix ( mk_p ) ;  
	for( ivary=0 , n=1; n <= mInputDimension; ++n){
		if ( max_include[n] ) {
			ivary ++ ;
		} else {
			continue ;
		}
		for(j=1; j <= mNumTrainingData; ++j){
			for(m=1; m <= ngauss; ++m){ /* compute exp(...) terms in k */
				tint = (m-1)*mInputDimension;
				temp1 = 0.0;
				temp2 = 1.0;

				for(k=1; k <= mInputDimension; ++k){
					if( use_poly_length_scales == 0 ){
						temp5 = exp(2.0*hyp[tint+k]);
					}
					if( use_poly_length_scales == 1 ){ /* Polynomial length scales */
						temp3 = PL_poly(mNumTrainingData+1,m,k);
						temp4 = PL_poly(j,m,k);
						temp5 = (temp3*temp3 + temp4*temp4);
						temp2 *= ( temp3*temp4/temp5 );
					}
					if( C_periodic == 0 ){
						temp1 += pow( (k_x[mNumTrainingData+1][k] - k_x[j][k]) , 2.0 ) / temp5;
					}
					if( C_periodic == 1 ){
						temp3 = (k_x[mNumTrainingData+1][k] - k_x[j][k]) * wavel[k];
						temp1 += pow( sin(temp3) , 2.0 ) / temp5;
					}
					mls[k] = temp5;	
				}
				if( use_poly_length_scales == 1 ){
					temp2 = pow(temp2 , 0.5) * root2tonh;
					factor = hyp[N_LENGTH_SCALES+m] * temp2 * exp( - temp1 ) ;

					/* one of the few n-dependent lines : */
					mdk_p[ivary][j] += 2.0*(k_x[j][n]-k_x[mNumTrainingData+1][n])*factor /mls[n];	

				} else if( use_poly_length_scales == 0 ){
					factor =  hyp[N_LENGTH_SCALES+m] * temp2 * exp( - 0.5 * temp1 ) ;
					if( C_periodic == 0 ){
						mdk_p[ivary][j] += (k_x[j][n]-k_x[mNumTrainingData+1][n])*(factor)/mls[n];	
					} else {
						temp5 = (k_x[j][n]-k_x[mNumTrainingData+1][n])*wavel[n];
						mdk_p[ivary][j] += wavel[n]*sin(temp5)*cos(temp5)*(factor )/mls[n];	
					}
				}
			}

			if( C_linear == 1 ){
				mdk_p[ivary][j] += hyp[N_including_NOISE+n] * k_x[j][n];      
			}
		}
	}

	/* Calculation of gradients */

	find_predictive_mean_and_var ( mv , mk_p , &temp1 , &temp3 , mv2  ) ; 
	if ( temp3 > 0.0 ) {
		temp3 = pow( temp3 , 0.5 );
	} else {
		temp3 = 0.0 ; 
		fprintf ( stderr , "WARNING: negative predictive variance %g\n" , temp3 ) ;
	}

	for(ivary=0 , n=1; n <= mInputDimension; ++n){
		/* only do the varying parameters DJCM 97/11/22 */
		if ( max_include[n] ) {
			ivary ++ ;
		} else {
			continue ; /* skip over the rest of this n loop, cos we ain't interested */
		}

		temp2 = 0.0; /* mean derivative */
		temp4 = 0.0; /* s.d. derivative */

		/* mean derivative */
		for(i=1; i <= mNumTrainingData; ++i){
			temp2 += mdk_p[ivary][i]*mv[i];
		}

		/* standard deviation derivative */
		if( C_linear == 1 ){
			temp4 += 2.0*hyp[N_including_NOISE+n] * k_x[mNumTrainingData+1][n];
		} 
		for(i=1; i <= mNumTrainingData; ++i){
			temp4 -= 2.0 * mv2[i] * mdk_p[ivary][i];
		}
		temp4 = temp4/(2.0*temp3);

		/* The different maximization options */

		if( do_SEARCH == 1 && SEARCH_max == 0 ){
			dM[ivary] = temp2 + max_beta * temp4;   
		} else if( do_SEARCH == 1 && SEARCH_max == 1 ){
			dM[ivary] = -temp2 + max_beta * temp4;
		} else if( do_SEARCH == 2 && SEARCH_max == 0 ){
			dM[ivary] = temp2 + max_beta * temp3 * temp4;
		} else if( do_SEARCH == 2 && SEARCH_max == 1 ){
			dM[ivary] = -temp2 + max_beta* temp3 * temp4;
		} else if( do_SEARCH == 3 && SEARCH_max == 0 ){
			dM[ivary] = 2.0*(temp1-max_gam)*temp2 + 2.0 * max_beta * temp3 * temp4;
		} else if( do_SEARCH == 3 && SEARCH_max == 1 ){
			dM[ivary] = -2.0*(temp1-max_gam)*temp2 + 2.0 * max_beta * temp3 * temp4;
		} else if( do_SEARCH == 4 && SEARCH_max == 0 ){
			dM[ivary] = temp2/temp1 + (temp3/(temp1*temp1))*(temp4 - (temp3*temp2/temp1));      
		} else if( do_SEARCH == 4 && SEARCH_max == 1 ){
			dM[ivary] = -1.0*temp2/temp1 + (temp3/(temp1*temp1))*(temp4 - (temp3*temp2/temp1));            
		}

	}

	if( verbose == 2 ){
		for(i=1; i <= Ivary; ++i) printf("%f ",w[i]);
		printf(": %f\n",temp1);
	}

}


double tpros::max_func(double *w,void *arg)
{
	gq_args *c = (gq_args *) arg ;
	return c->all->max_func_member(w, c->verbosity);
}

double tpros::max_func_member(double *w, int verbosity) /* computes the function which we
														want to maximize w.r.t. the inputs */
{
	int i, ivary ;
	double temp1,temp3,temp5;


	/* globals defined for this routine 

	mk_p
	mv    = Cinv t
	mv2   = Cinv k_p

	*/

	for(ivary=0 , i=1; i <= mInputDimension; ++i) { /* extract the varying parameters DJCM 97/11/22 */
		if ( max_include[i] ) {
			ivary ++ ;
			k_x[mNumTrainingData+1][i] = w[ivary];
		} else {
			k_x[mNumTrainingData+1][i] = max_kx[i]; /* fixed initial value */
		}
	}

	if( use_poly_length_scales == 1 ) PL_calc(mNumTrainingData+1); /* Polynomial length scales */

	find_row_of_covariance_matrix ( mk_p ) ; 
	find_predictive_mean_and_var ( mv , mk_p , &temp1 , &temp3 , mv2 ) ; 

	/* turn into s.d. */
	if ( temp3 > 0.0 ) {
		temp3 = pow( temp3 , 0.5 );
	} else {
		temp3 = 0.0 ; 
		fprintf ( stderr , "WARNING: negative predictive variance %g\n" , temp3 ) ;
	}

	/* The different maximization options */

	if( do_SEARCH == 1 && SEARCH_max == 0 ){
		temp5 = temp1 + max_beta*temp3;
	} else if( do_SEARCH == 1 && SEARCH_max == 1 ){
		temp5 = -temp1 + max_beta*temp3;
	} else if( do_SEARCH == 2 && SEARCH_max == 0 ){
		temp5 = temp1 + 0.5*max_beta*temp3*temp3;
	} else if( do_SEARCH == 2 && SEARCH_max == 1 ){
		temp5 = -temp1 + 0.5*max_beta*temp3*temp3;
	} else if( do_SEARCH == 3 && SEARCH_max == 0 ){
		temp5 = pow( temp1 - max_gam, 2.0 ) + max_beta*temp3*temp3;
	} else if( do_SEARCH == 3 && SEARCH_max == 1 ){
		temp5 = -pow( temp1 - max_gam, 2.0 ) + max_beta*temp3*temp3;
	} else if( do_SEARCH == 4 && SEARCH_max == 0 ){
		temp5 = log(temp1) + 0.5*(temp3*temp3)/(temp1*temp1);
	} else if( do_SEARCH == 4 && SEARCH_max == 1 ){
		temp5 = -log(temp1) + 0.5*(temp3*temp3)/(temp1*temp1);
	} else if( do_SEARCH == 5 ){
		temp5 = temp1;
	} else if( do_SEARCH == 6 ){
		temp5 = temp3;
	} 
	if ( verbosity >= 1 ) {
		printf ("y = %g, sigma = %g; objective = %g\n" , temp1 , temp3 , temp5 ) ;
	}

	return temp5;
}

void tpros::print_usage ( void ) {
	printf("\nTpros {spec} [optional arguments]\n\n");
	printf("Version %s\n", VERSION_NO );
	printf("Optional Arguments for Tpros\n\n");
	printf("-tf     f   target file\n");
	printf("-gf     f   grid file\n");
	printf("-ho     f   hyperparameter output file\n");
	printf("-hi     f   hyperparameter input file\n");
	printf("-rbf    r   rbf for polynomial length scales\n\n");
	printf("-seed   s   seed\n");
	printf("-nh x   dimension of input space\n");
	printf("-ntdat x   number of data points\n");
	printf("-ninter x   number of interpolation points\n\n");
	printf("-ngauss x   number of gaussian terms in cov fn\n");
	printf("-npoly  x   number of polynomials for length scales\n");
	printf("-lin        linear term in covariance function\n");
	printf("-NOperiod   do not use periodic covariance function\n");
	printf("-wavel   l  wavelengths for periodic function\n\n");
	printf("-nhidden x  number of hidden neurons for noise model\n\n");
	printf("-opt        optimise hyperparameters\n");
	printf("-opt_its x  no. of iterations for OPT\n");
	printf("-opt_inv x  inversion method for OPT\n");
	printf("-opt_CGits x  no. of its for CG inversion for OPT\n");
	printf("-opt_CGtr x no. of elements for trace\n");
	printf("-opt_CGbnd x  accuracy of CG convergence\n");
	printf("-opt_Cinvt x  output file for Cinvt calculation\n");
	printf("-opt_evid   calculate evidence\n");
	printf("-opt_ls     optimise length scales\n");
	printf("-opt_t0     optimise theta0\n");
	printf("-opt_t1     optimise theta1\n");
	printf("-opt_t2     optimise theta2\n");
	printf("-opt_n      optimise noise model\n");
	printf("-opt_lin    optimise linear term\n");
	printf("-opt_style 0/1 optimization termination rule, 0=gradient, 1=step size\n\n");
	printf("-int        do interpolation\n");
	printf("-int_tf f   interpolation target file\n");
	printf("-int_gf f   interpolation grid file\n");
	printf("-int_of f   interpolation output file\n");
	printf("-int_inv m  inversion method for INT\n");
	printf("-int_CGits x  no. of its for CG inversion for INT\n");
	printf("-int_eb     calculate error bars\n");
	printf("-int_te     calculate training error\n");
	printf("-ebsd       specify no. of s.d. for error bars\n");
	printf("-int_n      calculate noise level\n");
	printf("-int_nf     make `noise-free' predictions\n\n");
	printf("-verb x     verbosity of output\n\n");
	printf("-plen       prior on the length scales\n");
	printf("-pn         prior on the noise\n");
	printf("-plin       prior on the linear term\n");
	printf("-pt0        prior on theta0\n");
	printf("-pt1        prior on theta1\n");
	printf("-pt2        prior on theta2\n");
	printf("-plm    x   mean of Gamma priors on length scales\n");
	printf("-plsd   x   s.d. of Gamma/Gaussian priors on length scales\n");
	printf("-pldof  x   dof of Gamma priors on length scales\n");
	printf("-plm2   x   mean2 of Gamma priors on length scales\n");
	printf("-plsd2  x   s.d.2 of Gamma/Gaussian priors on length scales\n");
	printf("-splm   x   specific mean of Gamma priors on length scales\n");
	printf("-splsd  x   specific s.d. of Gamma/Gaussian priors on length scales\n");
	printf("-spldof x   specific dof of Gamma priors on length scales\n");
	printf("-splm2  x   specific mean2 of Gamma priors on length scales\n");
	printf("-splsd2 x   specific s.d.2 of Gamma/Gaussian priors on length scales\n");
	printf("-pnm    x   mean of inverse Gamma prior on noise\n");
	printf("-pnsd   x   s.d. of Inverse Gamma/Gaussian prior on noise\n");
	printf("-pndof  x   dof of Inverse Gamma prior on noise\n");
	printf("-pt0m   x   mean of Gaussian prior on theta0\n");
	printf("-pt0sd  x   s.d. of Gaussian prior on theta0\n");
	printf("-pt1m   x   mean of inverse Gamma prior on theta1\n");
	printf("-pt1sd  x   s.d. of inverse Gamma prior on theta1\n");
	printf("-pt1dof x   dof of inverse Gamma prior on theta1\n");
	printf("-pt2m   x   mean of inverse Gamma prior on theta2\n");
	printf("-pt2sd  x   s.d. of inverse Gamma prior on theta2\n");
	printf("-pt2dof x   dof of inverse Gamma prior on theta2\n");
	printf("-plinm  x   mean of Gaussian prior on linear\n");
	printf("-plinsd x   s.d. of Gaussian prior on linear\n\n");
	printf("-t2     x   initial value of theta2\n");
	printf("-t0     x   initial value of theta0\n");
	printf("-nv     x   initial noise variance\n");
	printf("-check      do maccheckgrad before and after macopt\n");
	printf("-tol    x   macopt tolerance\n");
	printf("-opt_tol x  macopt tolerance (hyperparameter optimization only)\n");
	printf("-epsilon x  maccheckgrad step\n");
	printf("-stuff   x  rescaling for length scale initialisation\n");
	printf("-known_n    load in known training noise levels from target file\n");
	printf("-TCG        use tridiagonal-CG inversion method (default)\n");
	printf("-DCG        use double-CG inversion method\n");
	printf("-dump       dump hyperparameters to file every iteration\n");
	printf("\n");
	printf("-kopt type  perform k_x optimization\n");
	printf("-kopt_max   perform k_x maximization\n");
	printf("-kopt_min   perform k_x minimization\n");
	printf("-kopt_its x no. of its for k_x optimization\n");
	printf("-kopt_style 0/1 k_x optimization termination rule, 0=gradient, 1=step size\n");
	printf("-kopt_tol x macopt tolerance\n");
	printf("-kopt_beta x beta\n");
	printf("-kopt_gam x gamma\n");
	printf("-kopt_start x x x   input coordinates for start of search\n");
	printf("-kopt_include x x   which inputs to include in optimization (0/1)\n");
	printf("-kopt_log         keep log in file\n");
	printf("-kopt_logfile f   log file for search\n");

}
void tpros::welcome_banner(void) {
	int i ; 

	printf("\nTpros : GP training program\n");
	printf("===========================\n");
	printf("\n(c)1996,1997 M N Gibbs and D J C MacKay\n");

	printf("\nVersion %s\n", VERSION_NO );

	printf("\n\nspecfile    : %s\n",specfile);
	printf("datafile    : %s\n",datafile);
	printf("gridfile    : %s\n",gridfile);
	if( hyp_out == 1 ) printf("hypoutfile      : %s\n",hypoutfile);
	if( do_INT == 1 ) printf("outfile     : %s\n",outfile);
	if( hyp_in == 1 ) printf("hypinfile       : %s\n",hypinfile);

	printf("\nParameter values\n\n");
	printf("SEED                             : %d\n",SEED);
	printf("no. of initial data points       : %d\n",mNumTrainingData);
	printf("no. of dimensions in input space : %d\n",mInputDimension);
	printf("no. of gaussians in cov fn       : %d\n",ngauss);
	printf("order of polynomials for l.s.    : %d\n",npoly);

	printf("\n");

	if( C_periodic == 1 ){
		printf("Wavelengths for periodic fn      : ");
		for(i=1; i <= mInputDimension; ++i){
			printf("%f ",wavel[i]);
		}
		printf("\n");
	}

	printf("\n");

	if( do_INT == 1 ){
		printf("igridfile                        : %s\n",igridfile);
		printf("Method used for inversion (INT)  : ");
		if( INT_use_cg == 1){
			printf("CG\n");
			printf("no. of its for CG_invert (INT)   : %d\n",CG_int);
		}
		if( INT_use_cg == 0 && INT_LUi == 1 ) printf("LUi\n");
		if( INT_use_cg == 0 && INT_LUi == 0 ) printf("LUd\n");
		printf("no. of interpolation points      : %d\n",ninter);
		printf("no. of s.d. for error bars       : %d\n",nsig);
		printf("Calculate Error Bars             : ");
		if(INT_find_eb==1) printf("yes\n"); else printf("no\n");
		printf("Calculate noise                  : ");
		if(INT_find_sigma_nu==1) printf("yes\n"); else printf("no\n");
	}

	printf("\n");

	if( do_OPT == 1 ){ /* perform OPT */
		printf("Method used for inversion (OPT)  : ");
		if( OPT_use_cg == 1 ){
			printf("CG\n");
			printf("no. of elements for trace (OPT)  : %d\n",ntrace);
			printf("no. of its for CG_invert (OPT)   : %d\n",CG_opt);
		}
		if( OPT_use_cg == 0 ) printf("LUi\n");
		printf("no. of iterations (OPT)          : %d\n",opt_itmax);
		printf("Optimize sig_k                   : ");
		if(optimize_lengthscales==1) printf("yes\n"); else printf("no\n");
		printf("Optimize theta                   : ");
		if(optimize_theta1==1) printf("yes\n"); else printf("no\n");
		printf("Optimize theta2                  : ");
		if(optimize_theta2==1) printf("yes\n"); else printf("no\n");
		printf("Optimize noise model             : ");
		if(optimize_noise==1) printf("yes\n"); else printf("no\n");
		printf("Optimize linear term             : ");
		if(optimize_linear==1) printf("yes\n"); else printf("no\n");
		printf("Optimize variable mean           : ");
		if(optimize_theta0==1) printf("yes\n"); else printf("no\n");
	}

	printf("\n");

	printf("no. of hidden neurons noise : %d\n",nhidden);

	printf("\n");

}
void tpros::allocate_memory ( working_memory *wm ) {
	int tint ;


	tint = N_HYPS;

	dif_C   = dmatrix(1,mNumTrainingData,1,mNumTrainingData);
	if( OPT_use_cg == 1 || INT_use_cg == 1 ){
		Mtemp   = dmatrix(1,2*ntrace,1,mNumTrainingData);
		Mran    = dmatrix(1,2*ntrace,1,mNumTrainingData);
		g       = dmatrix(1,N_max+1,1,mNumTrainingData); 
		h       = dmatrix(1,N_max+1,1,mNumTrainingData);
		gam     = dvector(1,N_max+1);
		lambda  = dvector(1,N_max+1);
		mod_g   = dvector(1,N_max+1);
		if( OPT_tridiag_CG == 0 ){
			ws      = dvector(1,mNumTrainingData);
			gs      = dmatrix(1,N_max+1,1,mNumTrainingData); 
			hs      = dmatrix(1,N_max+1,1,mNumTrainingData);
			gams    = dvector(1,N_max+1);
			lambdas = dvector(1,N_max+1);
		}
		if( OPT_tridiag_CG == 1 ){
			Td      = dvector(1,N_max);
			Tl      = dvector(1,N_max);
			Tu      = dvector(1,N_max);    
			Td2     = dvector(1,N_max);
			Tl2     = dvector(1,N_max);
			Tu2     = dvector(1,N_max);    
			hAh     = dvector(1,N_max);
		} 
	}
	if( use_poly_length_scales == 1 ){
		PL      = dmatrix(1,mNumTrainingData+1,1,npoly);
		pk_x    = dmatrix(1,npoly,1,mInputDimension); 
		root2tonh = pow( 2.0 , (double)(mInputDimension)/2.0 );
	}
	if( calc_Hessian == 1 ){
		ls_e    = dmatrix(1,N_LENGTH_SCALES,1,mNumTrainingData);
	}
	k_x     = dmatrix(1,mNumTrainingData+1,1,mInputDimension+1);
	A       = dmatrix(1,mNumTrainingData,1,mNumTrainingData);
	C       = dmatrix(1,mNumTrainingData,1,mNumTrainingData);
	C_temp  = dmatrix(1,mNumTrainingData,1,mNumTrainingData);

	mls   = dvector(1,mInputDimension);    /*  square of length scale */ /* DJCM */

	if( OPT_use_cg == 0 ){
		C_inv = dmatrix(1,mNumTrainingData,1,mNumTrainingData);
	}
	if( do_INT == 1 ){
		ok_x  = dmatrix(1,ninter,1,mInputDimension);
		wm->v     = dvector(1,mNumTrainingData);
		wm->v2    = dvector(1,mNumTrainingData);
		wm->k_p   = dvector(1,mNumTrainingData);
	}
	if( do_INT == 0 && write_Cinv_t == 1 ){
		wm->v     = dvector(1,mNumTrainingData);
	}    
	if( INT_use_cg == 0 && INT_LUi == 1 && do_INT == 1 ){
		C_LU  = dmatrix(1,mNumTrainingData,1,mNumTrainingData);
	}
	t       = dvector(1,mNumTrainingData);
	if( INT_load_targets == 1 ) ot = dvector(1,ninter);
	lu_col  = dvector(1,mNumTrainingData);
	wm->tempc   = dvector(1,mNumTrainingData);
	n_vec   = dvector(1,mNumTrainingData);
	hyp     = dvector(1,tint);
	Lscale  = dvector(1,mInputDimension);
	Loff    = dvector(1,mInputDimension);
	if( nhidden > 1 ){
		L       = dvector(1,mNumTrainingData);
		Lhidden = dvector(1,nhidden+1);
		Lhidden[nhidden+1] = 1.0;
	}

	if( use_poly_length_scales == 0 ) ls = dvector(1,(ngauss*mInputDimension*npoly));

	indx    = ivector(1,mNumTrainingData);
}

void tpros::loadGridAndTarget(int numTrainingData, int inputDimension, matrixn& grid, vectorn& target, char* datafile, char* gridfile)
{
	int i,j;
	FILE* fp1;
	/* Targets and noise levels */
	fp1 = fopen( datafile, "r" );
	if (!fp1) fprintf ( stderr,"Problems reading from %s\n",datafile ), Msg::error("tpros error");

	target.setSize(numTrainingData);
	for(j=1; j <= numTrainingData; ++j){
		fscanf( fp1,"%lf",&target[j-1]);
	}
	fclose( fp1 );

	/* Load in training k space grid */

	grid.setSize(numTrainingData, inputDimension);
	fp1 = fopen( gridfile, "r" );
	if (!fp1) fprintf ( stderr,"Problems reading from %s\n",gridfile ), Msg::error("tpros error");
	for(i=1; i <= numTrainingData; ++i){
		for(j=1; j <= inputDimension; ++j){
			fscanf( fp1,"%lf",&grid[i-1][j-1]);
		}
	}
	fclose( fp1 );
}

void tpros::load_data ( void ) {
	FILE   *fp1;
	int i , j ; 
	int tint ;
	double temp1 , temp2 ; 

	/* Targets and noise levels */
	fp1 = fopen( datafile, "r" );
	if (!fp1) fprintf ( stderr,"Problems reading from %s\n",datafile ), Msg::error("tpros error");
	for(j=1; j <= mNumTrainingData; ++j){
		fscanf( fp1,"%lf",&t[j]);
		if( training_noise_in_datafile == 1 ){
			fscanf( fp1,"%lf",&n_vec[j]);  /* loading in known training noise levels */
			n_vec[j] = n_vec[j]*n_vec[j]; /* convert noise levels to variances */
		}
	}
	fclose( fp1 );

	/* Load in interpolation targets if needed */

	if( INT_load_targets == 1 ){
		fp1 = fopen( idatafile, "r" );
		if (!fp1) fprintf ( stderr,"Problems reading from %s\n",idatafile ), Msg::error("tpros error");
		for(j=1; j <= ninter; ++j){
			fscanf( fp1,"%lf",&ot[j]);
		}
		fclose( fp1 );
	}

	/* Load in training k space grid */

	fp1 = fopen( gridfile, "r" );
	if (!fp1) fprintf ( stderr,"Problems reading from %s\n",gridfile ), Msg::error("tpros error");

	for(i=1; i <= mInputDimension; ++i){ /* use these two arrays to find the min and max */
		Lscale[i] = -1.0e50;
		Loff[i] = 1.0e50;
	}
	for(i=1; i <= mNumTrainingData; ++i){
		for(j=1; j <= mInputDimension; ++j){
			fscanf( fp1,"%lf",&k_x[i][j]);
			if( k_x[i][j] > Lscale[j] ) Lscale[j] = k_x[i][j];
			if( k_x[i][j] < Loff[j] ) Loff[j] = k_x[i][j];
		}
		k_x[i][mInputDimension+1] = 1.0;
	}
	fclose( fp1 );


	_load_others();

}

void tpros::_load_others()
{
	FILE   *fp1;
	int i , j ; 
	int tint ;
	double temp1 , temp2 ; 

	/* set up scaling for polynomials */

	for(i=1; i <= mInputDimension; ++i){
		temp1 = (Lscale[i] - Loff[i]);
		if( temp1 <= 1.0e-10 ){
			printf("WARNING : nrying input (%d)\n",i);
			temp1 = 1.0;
		}
		temp2 = (Lscale[i] + Loff[i])/2.0;
		Lscale[i] = temp1;
		Loff[i] = temp2;
	}

}
void tpros::initialize_hyperparameters ( control *c ) {
	/* set up hyperparameters ourselves */
	int i , j , k , tint ; 
	double temp1 ;

	/* Initialise length scales using polynomial information 

	Set up so that if we are using multiple gaussians then 
	the initial length scales get shorter and shorter for each 
	new gaussian 
	*/

	for(j=1; j <= ngauss; ++j){
		for(i=1; i <= mInputDimension; ++i){
			for(k=1; k <= npoly; ++k){
				tint = (j-1)*mInputDimension*npoly + (i-1)*npoly + k;
				/* if no prior on lengthscale desired */
				if( lengthprior == 0 ) hyp[tint] = log(Lscale[i] / (stuff*(double)(j*npoly)) );
				if( lengthprior > 0 ){ /* using mean and s.d./dof notation */
					if( gamr_a[j][i] <= 0.0 && npoly == 1 ){
						/* if default is requested for non-poly case prior */
						hyp[tint] = log(Lscale[i] / (stuff*(double)(j)) );
					}
					else if( gamr_a[j][i] > 0.0 && npoly == 1 ){
						/* if specific prior is given for non-poly case */
						hyp[tint] = log(gamr_a[j][i]) ; 
					}
					else if( gamr2_b[j][i] <= 0.0 && npoly > 1 ){
						/* if default is requested for poly case prior on constant term */
						if( k < npoly ) hyp[tint] = 0.0;
						if( k == npoly ) hyp[tint] = log(Lscale[i] / (stuff*(double)(j*npoly)) );
					}
					else if( gamr2_b[j][i] > 0.0 && npoly > 1 ){
						/* if specific prior is given for poly case */
						if( k < npoly ) hyp[tint] = 0.0;
						if( k == npoly ) hyp[tint] = gamr2_a[j][i];
					}
				}
			}
		}
	} 

	/* Initialize variable mean */

	if( c->D[3] != -0.123456789 ){
		HYP_THETA_0 = c->D[3];
	} else { /* set initial mean to data mean */
		temp1 = 0.0;
		for(i=1; i <= mNumTrainingData; ++i){
			temp1 += t[i];
		}
		HYP_THETA_0 = temp1 / (double)(mNumTrainingData);
	}

	/* Initialise vertical scale using the sum square of the data */

	temp1 = 0.0;
	for(i=1; i <= mNumTrainingData; ++i){
		temp1 += pow( t[i]-HYP_THETA_0 , 2.0 );
	}
	for(i=1; i <= ngauss; ++i){
		hyp[N_LENGTH_SCALES+i] = temp1 / (double)(mNumTrainingData);
	}

	/* Initialise the vertical uncertainty */

	if( c->D[1] > 1.0e-10 ){
		HYP_THETA_2 = c->D[1];
	} else if( c->D[1] >= 0 ){
		HYP_THETA_2 = 1.0e-10;
	} else if( c->D[1] < 0 ){
		HYP_THETA_2 =  prior_fractional_theta2_default * HYP_THETA_1;
	}

	/* Initialisation of the noise model */

	if( nhidden == 1 ){
		if( c->D[2] > 1.0e-10 ){
			HYP_LOG_NOISE_VAR2 = log(c->D[2]);
		} else if( c->D[2] >= 0 ){
			HYP_LOG_NOISE_VAR2 = log(1.0e-10);
		} else if( c->D[2] < 0 ){
			HYP_LOG_NOISE_VAR2 = log(prior_fractional_noise_default * HYP_THETA_1);
		}
	}
	if( nhidden > 1 ){
		if( c->D[2] > 1.0e-10 ){
			hyp[N_before_NOISE+((mInputDimension+1)*nhidden)+nhidden+1] = log(c->D[2]);
		} else if( c->D[2] >= 0 ){
			hyp[N_before_NOISE+((mInputDimension+1)*nhidden)+nhidden+1] = log(1.0e-10);
		} else if( c->D[2] < 0 ){
			hyp[N_before_NOISE+((mInputDimension+1)*nhidden)+nhidden+1] = log(0.1 * HYP_THETA_1);
		}
	}

	/* Initialize linear term 

	The factor of 0.01 just means that the linear term starts off
	with little significance relative to the others
	*/
	if( C_linear == 1 ){
		for(i=1; i <= nlin; ++i){
			/* was
			hyp[N_including_NOISE+i] = rand_gaussian( )*0.01*HYP_THETA_1 * exp(-2.0*hyp[i]);
			*/
			hyp[N_including_NOISE+i] = 0.01*HYP_THETA_1 * exp(-2.0*hyp[i]);
		}
	}
}

/* Initialise the default priors (see Bernardo+Smith for more)

Note on Gamma distribution 

p(x) \propto x^(a-1) exp ( -bx )

Note on Inverse Gamma distribution

p(x) \propto (1/x)^(a+1) exp ( -b/x )

As densities over the log, remove the x^-1 factors.
*/
void tpros::set_up_priors ( void ){
	int i , j , tint ;
	double  temp1 , temp2 ;
	if( lengthprior >= 1 ){
		for(i=1; i <= ngauss; ++i){
			for(j=1; j <= mInputDimension; ++j){
				tint = (i-1)*mInputDimension*npoly + (j-1)*npoly + 1;
				if( npoly > 1 ){	  /* For polynomial case */
					if( gamr_b[i][j] <= 0.0 ){
						printf("\nERROR - default prior not available for polynomial length scales\n");
						Msg::error("tpros error");
					} 
					gamr_a[i][j] = 0.0;

					printf("Length scale prior (%d,%d)(mean,sd) : %-6.2g %-6.2g\n",i,j,gamr_a[i][j],gamr_b[i][j]);
					printf("Length scale offset prior (%d,%d)(mean2,sd2) : %-6.2g %-6.2g\n",i,j,gamr2_a[i][j],gamr2_b[i][j]);

				} else {	  /* For non-polynomial case */
					if( gamr_a[i][j] <= 0.0 ){
						temp1 = prior_length_default  ;            /* mean */
						temp2 = prior_strength_default;            /* d.o.f. */
					} else {
						/* Scaling for Gamma prior */
						if( lengthprior == 1 ){
							printf("Length scale prior (mean,sd) option discontinued\n");
							Msg::error("tpros error");
						}
						else { /* if( lengthprior == 2 ) */
							/* mean and degrees of freedom representation */
							temp1 = gamr_a[i][j]; /* mean   */
							temp2 = gamr_b[i][j]; /* d.o.f. */
						}
					}
					gamr_a[i][j] = temp2;
					gamr_b[i][j] = temp2/temp1;
					if( verbose != 0 ) printf("Length scale prior  (%d,%d)(mean,strength) : %-6.2g %-6.2g a=%-6.2g, b=%-6.2g\n",i,j,temp1,temp2,gamr_a[i][j]  ,gamr_b[i][j] );
				}
			}
		}
	}
	if( noiseprior >= 1 ){
		if( nhidden > 1 ){      /* For polynomial case */
			if( gamn_b <= 0.0 ){
				printf("\nERROR - default prior not available for polynomial noise model\n");
				Msg::error("tpros error");
			} 
			gamn_a = 0.0; /* zero mean Gaussian */
		} else {      /* For non-polynomial case */	/* Inverse Gamma prior */
			if( gamn_a <= 0.0 ){
				temp1 = prior_vertical_default * prior_fractional_noise_default ;
				temp1 = temp1 * temp1 ; 
				temp2 = prior_strength_default;                       /* d.o.f. */
			} else {
				if( noiseprior == 1 ){
					if( 1 ) printf("Noise level prior (mean,sd) option discontinued\n");
					Msg::error("tpros error") ; 
				} else { /* if( noiseprior == 2 ) */
					temp1 = gamn_a; /* centre of noise variance's prior  */
					temp2 = gamn_b; /* d.o.f. */
					if( noiseprior == 3 ){
						printf("Noise level prior (CBJ) option discontinued\n");
					}
				}

			}
			gamn_a = temp2/2.0 ;
			gamn_b = (temp2/2.0) * temp1;
			if( verbose != 0 ) printf("Noise level prior        (mean,strength) : %-6.2g %-6.2g a=%-6.2g, b=%-6.2g\n",temp1,temp2 , gamn_a , gamn_b );
		}
	}
	if( theta1prior >= 1 ){            /*  Inverse Gamma prior */
		if( gamt1_a <= 0.0 ){
			temp1 = prior_vertical_default * prior_vertical_default ;
			temp2 = prior_strength_default;                       /* d.o.f. */
		} else {
			if( theta1prior == 1 ){
				if( 1 ) printf("Theta1 prior (mean,sd) option not available\n");
				Msg::error("tpros error");
			} else { /*      if( theta1prior == 2 ) */
				temp1 = gamt1_a; /* mean vertical variance  */
				temp2 = gamt1_b; /* d.o.f. */
			}
		}
		gamt1_a = temp2/2.0 ;
		gamt1_b = (temp2/2.0 ) * temp1;
		if( verbose != 0 ) printf("Theta1 prior             (mean,strength) : %-6.2g %-6.2g a=%-6.2g, b=%-6.2g\n",temp1,temp2 , gamt1_a , gamt1_b );
	}
	if( theta2prior >= 1 ){
		if( gamt2_a <= 0.0 ){
			temp1 = prior_fractional_theta2_default * prior_vertical_default ;
			/* mean */
			temp2 = prior_strength_default;                       /* d.o.f. */
		} else {
			if( theta2prior == 1 ){
				if( 1 ) printf("Theta2 prior (mean,sd) option not available\n");
				Msg::error("tpros error");
			} else { /*      if( theta2prior == 2 ) */
				temp1 = gamt2_a; /* mean   */
				temp2 = gamt2_b; /* d.o.f. */
			}
		}
		gamt2_a = temp2/2.0 ;
		gamt2_b = (temp2/2.0) * temp1;;
		if( verbose != 0 ) printf("Theta2 prior             (mean,strength) : %-6.2g %-6.2g a=%-6.2g, b=%-6.2g\n",temp1,temp2 , gamt2_a , gamt2_b );
	}
	if( linearprior >= 1 ){
		if( gaussl_m <= 0.0 ){
			/* if priors not set then program does it automatically */
			gaussl_m = prior_linear_default ; 
			temp1 = 0.0;
			for(i=1; i <= mNumTrainingData; ++i){
				temp1 += pow( t[i], 2.0 ); /* was  t[i]-HYP_THETA_0  
										   but I want independence of hyp[] */
			}
			gaussl_sd = pow( temp1 / (double)(mNumTrainingData) , 0.5 );
		}      
		if( verbose != 0 ) printf("Linear term prior        (mean,sd)  : %-6.2g %-6.2g\n",gaussl_m,gaussl_sd);
	}
	if( theta0prior >= 1 ){
		if( gausst0_m <= 0.0 ){
			/* if priors not set then program does it automatically */
			temp1 = 0.0;
			for(i=1; i <= mNumTrainingData; ++i){
				temp1 += t[i];
			}
			gausst0_m  = temp1/(double)(mNumTrainingData); /* data mean */
			temp1 = 0.0;
			for(i=1; i <= mNumTrainingData; ++i){
				temp1 += pow( t[i]-gausst0_m , 2.0 );/* was  t[i]-HYP_THETA_0  */
			}
			gausst0_sd = pow( temp1 / (double)(mNumTrainingData) , 0.5 ); /* vertical s.d. of target */
		} 
		if( verbose != 0 ) printf("Variable Mean prior      (mean,sd)  : %-6.2g %-6.2g\n",gausst0_m,gausst0_sd);
	}

	printf("\n");
}




int tpros::test(vectorn const &input, m_real &mean, m_real &sig)
{
	for(int i=1; i <= mInputDimension; ++i)
	{
		k_x[mNumTrainingData+1][i] = input[i-1];
	}
	k_x[mNumTrainingData+1][mInputDimension+1] = 1.0;

	_test(mean, sig);

	return true;
}


void tpros::generateOutput()
{
	int i,j,l,k,tint,tint2;

	double pred_err,test_err,train_err;
	double temp1,temp2,temp3,temp4,temp5;

	/* Load in interpolation k space grid */

	tint = 0 ;
	if( do_INT == 1 ){
		FILE* fp1;
		fp1 = fopen( igridfile, "r" );
		if (!fp1) fprintf ( stderr,"Problems reading from %s\n",igridfile ), Msg::error("tpros error");
		for(i=1; i <= ninter; ++i){
			for(j=1; j <= mInputDimension; ++j){
				fscanf( fp1,"%lf",&ok_x[i][j]);
				if( fabs(ok_x[i][j]-Loff[j]) >= Lscale[j] ){
					tint += 1;
					if ( rescaling_permitted ) 
						Lscale[j] = fabs(ok_x[i][j]-Loff[j]);
				}
			}
		} 
		fclose( fp1 );
	}
	if( tint > 0 && verbose > 0 ){
		printf("\nWARNING : Interpolation grid overflow (%d points lie outside range of training set)\n",tint);
		if  ( rescaling_permitted ) 
			printf("          Rescaling has been performed\n\n" ) ;
		else 
			printf("          Rescaling has not been performed\n\n" ) ;
	}

	if ( verbose >= 2 ) { /* change this back to 2 */
		printf ("Lengthscales\n" ) ; 
		printf ("%9s %9s \n","range","centre") ;
		for(i=1; i <= mInputDimension; ++i){
			printf("%9.4g %9.4g %d\n", Lscale[i] , Loff[i] , i);
		}
	}

	/* Compute the hessian of the length scales */

	if( calc_Hessian == 1 ) ls_error( );

	/* ===================== Generate Output ==================== */

	if( do_INT == 1 ){

		/* === Calculate interpolatant */

		tint2 = ninter;
		if( OPT_say_train_e == 1 ){
			train_err = 0.0;
			tint2 += mNumTrainingData; 
		} 

		FILE* fp1;
		fp1 = fopen( outfile , "w" );
		if( !fp1 ) fprintf( stderr, "Problems opening : %s\n",outfile ), Msg::error("tpros error");

		m_real eb1, eb2, eb3, mean, sigma_nu;

		for(l=1; l <= tint2; ++l){

			if( INT_load_targets == 1 ){
				pred_err = 0.0;
				test_err = 0.0;
			}

			if( l <= ninter ){
				for(i=1; i <= mInputDimension; ++i){
					k_x[mNumTrainingData+1][i] = ok_x[l][i];
				}
				k_x[mNumTrainingData+1][mInputDimension+1] = 1.0;
			} else {
				for(i=1; i <= mInputDimension; ++i){
					k_x[mNumTrainingData+1][i] = k_x[l-ninter][i];
				}
				k_x[mNumTrainingData+1][mInputDimension+1] = 1.0;
			}

			_test(mean, sigma_nu);

			/* Calculate error bars */      

			if( INT_find_eb == 0 && INT_load_targets == 1 ){
				printf("WARNING : predictive error requested without error bars.\n");
			}
			if( INT_find_eb == 1 ){

				double temp2;
				double temp3,temp4,temp5;
				if( nhidden > 1 ) L_calc(mNumTrainingData+1);
				if( training_noise_in_datafile == 0 && (!noise_free_predictions) ) temp2 = noise(mNumTrainingData+1);
				else temp2 = 0.0;

				for(i=1; i <= ngauss; ++i){
					temp2 += hyp[ N_LENGTH_SCALES + i ];
				}
				temp2 += HYP_THETA_2;

				if( C_linear == 1 ){
					for(i=1; i <= nlin; ++i){
						temp2 += hyp[N_including_NOISE+i]*k_x[mNumTrainingData+1][i]*k_x[mNumTrainingData+1][i];
					}
				}

				/* LU direct or LU indirect */
				if( INT_use_cg == 0 ){
					LU_apply(C,wm.k_p,wm.v2,mNumTrainingData);
					for(i=1; i <= mNumTrainingData; ++i){
						temp2 -= wm.k_p[i] * wm.v2[i];
					}
				}
				/* CG inversion */
				else if( INT_use_cg == 1 ){
					CG_invert(wm.v2,wm.k_p,CG_int);
					for(i=1; i <= mNumTrainingData; ++i){
						temp2 -= wm.k_p[i] * wm.v2[i];
					}
				}

				if ( temp2 >= 0.0 ) {
					temp2 = pow( temp2 , 0.5 );
				} else {
					fprintf ( stderr , " WARNING : negative predictive variance %g\n" , temp2 ) ; 
					temp2 = 0.0 ; 
				}

				temp3 = mean - ((double)(nsig) * temp2);
				temp4 = mean + ((double)(nsig) * temp2);


				eb1=(double)(nsig)*temp2;
				eb2=temp3;
				eb3=temp4;

				/* Calculate Error Statistics */      

				if( INT_load_targets == 1 && l <= ninter ){
					pred_err += 0.5*pow( (ot[l] - mean)/temp2 , 2.0 ) + log(temp2);
					test_err += pow( ot[l] - mean, 2.0 );
				}
				if( OPT_say_train_e == 1 && l > ninter ){
					train_err += pow( t[l-ninter] - mean, 2.0 );
				}
			}

			if( gridx1 > 0 && l%gridx1 == 1 ) fprintf(fp1,"\n");

			/* Save point to file */

			if( l <= ninter ){
				if ( INT_include_inputs ) {
					for(k=1; k <= mInputDimension; ++k){
						fprintf(fp1,"%f ",k_x[mNumTrainingData+1][k]);
					}
				}
				fprintf(fp1,"%f ",mean);
				if( INT_find_eb == 1 ) 
				{
					fprintf(fp1,"%f %f %f ",eb1, eb2, eb3);					
				}

				if( INT_find_sigma_nu == 1 ) fprintf(fp1,"%f ",sigma_nu);
				if( INT_load_targets == 1 ) fprintf(fp1,"%f ",ot[l]);

				if( use_poly_length_scales == 1 ){ /* using fancy input polynomials */
					/* print out value of length scales at each point */
					for(j=1; j <= ngauss; ++j){
						for(i=1; i <= mInputDimension; ++i){
							tint = i + ((j-1)*mInputDimension);
							temp1 = PL_poly(mNumTrainingData+1,j,i);
							fprintf(fp1,"%f ",temp1);
							if( calc_Hessian == 1 ){
								temp2 = exp( log(temp1) - ls_e[tint][l] );
								temp3 = exp( log(temp1) + ls_e[tint][l] );
								fprintf(fp1,"%f %f",temp2,temp3);
							}
						}
					}
				}
				fprintf(fp1,"\n"); 
			}

		}

		fclose( fp1 );

		/* Print out error statistics */

		if( OPT_say_train_e == 1 ) printf("Train Error  : %f\n",train_err);
		if( INT_load_targets == 1 ){
			printf("Test Error   : %f\n",test_err);
			printf("Pred Error   : %f\n",pred_err);
		}

	}

	/* Optimize input variables */

	if( do_SEARCH > 0 ) maxim( );		

}


void tpros::_printMessage()
{
	/* init */
	rand_seed(SEED);

	FILE* fp1;
	N_max = CG_opt*5;
	if( nhidden == 1 ) nwb = 1;
	if( nhidden != 1 ) nwb = ((mInputDimension+1) * nhidden)+(nhidden+1);
	if( npoly > 1 ) npoly += 1; /* adding in constant term */
	if( C_linear == 0 ) nlin = 0;
	else   nlin = mInputDimension;

	if( write_hdump == 1 ){   /* prepare file for hdump */
		sprintf(hypdump,"%s.dump",hypoutfile);
		fp1 = fopen( hypdump, "w" );
		if ( !fp1 ) fprintf ( stderr , " can't open %s\n" ,  hypdump ) , Msg::error("tpros error") ; 
		fclose( fp1 );
	}

	/* Output section */
	if( verbose != 0 ) welcome_banner() ; 

	/* Catch any inconsistent parameter or flag settings */
	err_catch( );

	/* Rescale wavelengths if necessary */

	if( C_periodic == 1 ){
		for(int i=1; i <= mInputDimension; ++i){
			wavel[i] = 3.1415927 / wavel[i];
		}
	}

}

void tpros::loadAndLearn()
{
	_printMessage();

	/* defining primary arrays */

	allocate_memory ( &wm ) ;

	load_data () ; /* also finds Lscales and checks for overflow */

	_learn();
}

void tpros::learn(matrixn const& grid, vectorn const &target)
{
	ASSERT(mNumTrainingData==target.size());
	ASSERT(grid.rows()==mNumTrainingData);
	ASSERT(mInputDimension==grid.cols());

	_printMessage();

	/* defining primary arrays */
	allocate_memory ( &wm ) ;

	int i , j ; 
	int tint ;
	double temp1 , temp2 ; 

	/* Targets and noise levels */
	ASSERT( training_noise_in_datafile == 0);
	ASSERT( INT_load_targets == 0 );

	for(j=1; j <= mNumTrainingData; ++j)
		t[j]=target[j-1];
	
	/* Load in training k space grid */

	for(i=1; i <= mInputDimension; ++i){ /* use these two arrays to find the min and max */
		Lscale[i] = -1.0e50;
		Loff[i] = 1.0e50;
	}
	
	for(i=1; i <= mNumTrainingData; ++i){
		for(j=1; j <= mInputDimension; ++j){
			k_x[i][j]=grid[i-1][j-1];
			if( k_x[i][j] > Lscale[j] ) Lscale[j] = k_x[i][j];
			if( k_x[i][j] < Loff[j] ) Loff[j] = k_x[i][j];
		}
		k_x[i][mInputDimension+1] = 1.0;
	}

	_load_others();	
	_learn();
}

void tpros::_learn()
{
	int i, j;
	double      temp1 ;
	FILE* fp1;


	if( hyp_in == 0 )    /* set up hyperparameters ourselves */
		initialize_hyperparameters( &c ) ;
	else /* if( hyp_in == 1 ) */
		load_hyp(hypinfile);

	/* Generate random samples if required */

	if( do_samples == 1 ) sample( );

	if ( do_OPT ) set_up_priors ( ) ; 

	/* set-up polynomials for length scales */

	if( use_poly_length_scales == 1 ) PL_setup( );

	/* set-up noise model */

	if( nhidden > 1 ) L_setup( );

	/* ================ Optimize hyperparameters ================ */

	if( do_OPT == 1 ) C_hyper( );

	if( do_OPT == 0 ) mac_inter_C( ); /* computes the covariance matrix in A */

	if( OPT_say_evidence == 1 ){
		if (do_INT == 1){
			for(i=1; i <= mNumTrainingData; ++i){
				wm.tempc[i] = t[i] - HYP_THETA_0;
			}
			LU_apply(C,wm.tempc,wm.v,mNumTrainingData);
			temp1 = 0.0 ; 
			for(i=1; i <= mNumTrainingData; ++i){
				temp1 += wm.v[i]*wm.tempc[i];
			}
			temp1 = -0.5*(double)(mNumTrainingData)*log(6.2831853) - 0.5*temp1 - 0.5*log_det;
			printf("\nLog Evidence : %f\n",temp1);
		}
		else
			printf("WARNING: Unable to calculate evidence with interpolation option off.\n");
	}
	if( write_Cinv_t == 1 ){
		for(i=1; i <= mNumTrainingData; ++i){
			wm.tempc[i] = t[i] - HYP_THETA_0;
		}
		LU_apply(C,wm.tempc,wm.v,mNumTrainingData);
		fp1 = fopen( Cinvtfile, "w" );
		if( !fp1 ) fprintf( stderr, "Problems opening : %s\n",Cinvtfile ), Msg::error("tpros error");
		for(i=1; i <= mNumTrainingData; ++i){
			fprintf(fp1,"%f\n",wm.v[i]);
		}
		fclose( fp1 );
	}

	/* Save hyperparameters to file */

	if( hyp_out == 1 ) save_hyp(hypoutfile);

	

	/* set up mean subtracted vector */
	for(i=1; i <= mNumTrainingData; ++i){
		wm.tempc[i] = t[i] - HYP_THETA_0;
	}

	if( do_INT == 1 ){
		double d;
		/* LU inversion with indirect method */
		if( INT_use_cg == 0 && INT_LUi == 1 ){
			for(i=1; i <= mNumTrainingData; ++i){
				for(j=1; j <= mNumTrainingData; ++j){
					C_LU[i][j] = C[i][j];
				}
			}
			ludcmp(C_LU,mNumTrainingData,indx,&d);
			LU_stage1(C_LU,mNumTrainingData,indx,wm.tempc,wm.v);
		}

		/* LU inversion with direct method */
		if( INT_use_cg == 0 && INT_LUi == 0 ){
			LU_apply(C,wm.tempc,wm.v,mNumTrainingData);
		}

		/* CG inversion */
		if( INT_use_cg == 1 ){
			CG_invert(wm.v,wm.tempc,CG_int);
		}
	}
}


void _saveDV(BinaryFile& binaryFile, double* v, int n)
{
	vectorn t;
	t.setSize(n);
	for(int i=1; i<=n; i++)
		t[i-1]=v[i];

	binaryFile.pack(t);
}

void _loadDV(BinaryFile& binaryFile, double*& v, int n)
{
	v=dvector(1, n);

	vectorn t;
	binaryFile.unpack(t);

	for(int i=1; i<=n; i++)
		v[i]=t[i-1];	
}

void _saveIV(BinaryFile& binaryFile, int* v, int n)
{
	intvectorn t;
	t.setSize(n);
	for(int i=1; i<=n; i++)
		t[i-1]=v[i];

	binaryFile.pack(t);
}

void _loadIV(BinaryFile& binaryFile, int*& v, int n)
{
	v=ivector(1, n);

	intvectorn t;
	binaryFile.unpack(t);

	for(int i=1; i<=n; i++)
		v[i]=t[i-1];	
}
void _saveDM(BinaryFile& binaryFile, double** mat, int m, int n)
{
	matrixn temp(m, n);
	for(int i=1; i<=m; i++)
		for(int j=1; j<=n; j++)
			temp[i-1][j-1]=mat[i][j];

	binaryFile.pack(temp);
}

void _loadDM(BinaryFile& binaryFile, double**& mat, int m, int n)
{
	mat = dmatrix(1,m,1,n);

	matrixn temp;
	binaryFile.unpack(temp);

	for(int i=1; i<=m; i++)
		for(int j=1; j<=n; j++)
			mat[i][j]=temp[i-1][j-1];
}

void tpros::load(BinaryFile& binaryFile)
{
	mInputDimension=binaryFile.unpackInt();
	mNumTrainingData=binaryFile.unpackInt();
	ngauss=binaryFile.unpackInt();
	npoly=binaryFile.unpackInt();
	nwb=binaryFile.unpackInt();
	nlin=binaryFile.unpackInt();
	

	use_poly_length_scales=0;
	use_poly_length_scales=0;
	C_periodic = 0;
	C_linear =0;
	INT_use_cg = 0;
	INT_LUi = 1;
	INT_find_eb=1;
	nhidden=1;

	_loadDM(binaryFile, k_x, mNumTrainingData+1, mInputDimension+1);
	_loadDV(binaryFile, wm.k_p, mNumTrainingData);
	_loadDM(binaryFile, C_LU ,mNumTrainingData,mNumTrainingData);
	_loadDV(binaryFile,wm.tempc ,mNumTrainingData);	
	_loadDV(binaryFile,wm.v ,mNumTrainingData);
	_loadIV(binaryFile,indx,mNumTrainingData);
	_loadDV(binaryFile, hyp,N_HYPS);	

}


void tpros::save(BinaryFile& binaryFile)
{	
	binaryFile.packInt(mInputDimension);
	binaryFile.packInt(mNumTrainingData);
	binaryFile.packInt(ngauss);
	binaryFile.packInt(npoly);
	binaryFile.packInt(nwb);
	binaryFile.packInt(nlin);

	Msg::verify(use_poly_length_scales==0 &&
				C_periodic == 0 	&&
				C_linear ==0 &&
				INT_use_cg == 0 && INT_LUi == 1 &&
				INT_find_eb==1 &&
				nhidden==1		
		, "Saving other options is not implemented.");
	
	_saveDM(binaryFile, k_x, mNumTrainingData+1, mInputDimension+1);
	_saveDV(binaryFile, wm.k_p, mNumTrainingData);
	_saveDM(binaryFile, C_LU ,mNumTrainingData,mNumTrainingData);
	_saveDV(binaryFile,wm.tempc ,mNumTrainingData);	
	_saveDV(binaryFile,wm.v ,mNumTrainingData);
	_saveIV(binaryFile,indx,mNumTrainingData);
	_saveDV(binaryFile, hyp,N_HYPS);
	
}

void tpros::_test(m_real& mean, m_real& sigma_nu)
{
	int i,j,k,tint,tint2;
	
	if( use_poly_length_scales == 1 ) PL_calc(mNumTrainingData+1); /* Polynomial length scales */

	find_row_of_covariance_matrix ( wm.k_p ) ;

	/* Calculation of new point */

	mean = 0.0;

	/* LU indirect */
	if( INT_use_cg == 0 && INT_LUi == 1 ){
		LU_stage2(C_LU,mNumTrainingData,indx,wm.k_p,wm.tempc);
		for(i=1; i <= mNumTrainingData; ++i){
			mean += wm.tempc[i]*wm.v[i];
		}
	} 
	/* CG inversion or LU direct */
	else {
		for(i=1; i <= mNumTrainingData; ++i){
			mean += wm.k_p[i]*wm.v[i];
		}
	}

	/* Add on mean */

	mean += HYP_THETA_0;

	

	/* Calculate noise level as function of k_x */

	if( INT_find_sigma_nu == 1 ){

		if( INT_find_eb == 0 && nhidden > 1 ){
			L_calc(mNumTrainingData+1);
		}

		/* We output the square root of the noise function as this is
		the standard deviation associated with the noise */

		sigma_nu = pow(noise(mNumTrainingData+1),0.5);
		
	}

	/* Introduce space for grided ogrid if required */


}
