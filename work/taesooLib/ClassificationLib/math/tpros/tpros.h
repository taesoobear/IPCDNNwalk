#pragma once


class tpros
{
public:
	tpros(int argc, char** argv, int dim=0, int ntdata=0);
	tpros(const char* option, int dim=0, int ntdata=0);

	/* Global file names */
	char specfile[200];
	char gridfile[200];
	char igridfile[200];
	char datafile[200];
	char idatafile[200];
	char rbffile[200];
	char outfile[200];
	char hypoutfile[200];
	char hypdump[200];
	char hypinfile[200];
	char maxout[200];
	char Cinvtfile[200];

	int numTrainingData() const	{ return mNumTrainingData;}
	int inputDimension() const	{ return mInputDimension;}

	static void loadGridAndTarget(int numTrainingData, int inputDimension, matrixn& grid, vectorn& target, char* datafile, char* gridfile);
	void loadAndLearn();
	void learn(matrixn const& grid, vectorn const &target);
	int test(vectorn const &input, m_real &mean, m_real &sig);
	void generateOutput();

	void load(BinaryFile& binaryFile);
	void save(BinaryFile& binaryFile);

	struct working_memory{
		double *k_p ;
		double *v ;
		double *v2 ;
		double *tempc;
	};

	working_memory wm;

private:
	// _로 시작함수는 refactoring과정에서 생긴 함수들.
	void _init(int argc, char** argv, int dim=0, int ntdata=0);
	void _printMessage();
	void _learn();
	void _load_others();
	void _test(m_real& mean, m_real& sigma_nu);
	int mNumTrainingData;
	int mInputDimension; /* number if input dimensions */

	struct control{
		double *D ;
	} ;
	control c;
	/* Specfile subroutines */
	void initialize_hyperparameters ( control * ) ;
	void load_data ( void ) ;	
	void allocate_memory ( working_memory * ) ;
	void welcome_banner(void) ;
	void print_usage ( void ) ;
	void readInSpecfile(char *,double *);
	void c_line(int,char *[],double *);
	void defaults(double *);
	void err_catch( void );

	/* Standard subroutines */

	void set_up_priors ( void ) ;
	void C_hyper( void );

	void mac_inter_C( void );
	void mac_inter_dif(double *,int);

	static void mac_dif(double *,double *,void *);
	void mac_dif_member(double *w,double *dM);
	static double mac_func(double *,void *);
	double mac_func_member(double* w);
	static void T_return(double *, int, void * );
	void T_return_member(double *w, int n);

	void load_hyp(char *);
	void save_hyp(char *);

	void w_con_in(double *);
	void w_con_out(double *);


	/* noise model routine */

	void L_setup( void );
	void L_calc(int);
	void dif_prior(double *,double *);
	void report_input_vector ( FILE *fp , double *w , int ) ;

	double sigmoid(double);
	double difsig(double);

	double prior(double *);
	double noise(int);

	/* LU inversion subroutine */

	void LU_invert(double **,double **,int n);
	void LU_apply(double **,double *,double *,int n);
	void LU_stage1(double **,int,int *,double *,double *);
	void LU_stage2(double **,int,int *,double *,double *);

	void find_row_of_covariance_matrix ( double * ) ;
	void find_predictive_mean_and_var ( double * ,double * , double * , double * , double * ) ;

	/* CG inversion subroutines */

	void CG_invert(double *,double *,int);
	void bounds(double *,double *,double *,double *,double *);

	double CG_func1(double *,double *);
	double CG_func2(double *,double *);

	/* trace routines */

	double trace(double *,double *);

	/* polynomial length scale routines */

	void PL_setup( void );
	void PL_calc( int );

	double PL_poly( int,int,int );

	/* routines for calculating the Ls_Hessian of P(Theta|D) */

	void ls_error( void );

	/* routines for speeding up mac_inter_C and mac_inter_dif */

	void ls_calc( void );

	/* sampling routine */

	void sample( void );

	/* maximization routines */

	void maxim( void );
	static void max_dif(double *,double *,void *);
	void max_dif_member(double *,double *);
	static double max_func(double *,void *);
	double max_func_member(double *w, int verbosity);

	static void state_printer ( double *w, int its, void *arg ) ;
	void state_printer_member ( double *w, int its);

	/* Definitions */
#define N_LENGTH_SCALES (mInputDimension*ngauss*npoly)
#define N_before_NOISE N_LENGTH_SCALES+ngauss+1
#define N_including_NOISE N_LENGTH_SCALES+ngauss+1+nwb
#define N_HYPS N_including_NOISE+nlin+1
#define HYP_LOG_NOISE_VAR hyp[N_LENGTH_SCALES+ngauss+1+nwb]
#define HYP_LOG_NOISE_VAR2 hyp[N_LENGTH_SCALES+ngauss+2]
#define HYP_THETA_1 hyp[(mInputDimension*ngauss*npoly)+1]
#define HYP_THETA_2 hyp[(mInputDimension*ngauss*npoly)+ngauss+1]
#define HYP_THETA_0 hyp[N_LENGTH_SCALES+ngauss+1+nwb+nlin+1]

	/* Declare global variables */

	double **k_x;
	double *t;
	double **ok_x;
	double *ot;

	double **C;
	double **C_inv;
	double **C_temp; /* contains the LU decomposition of C */
	double **C_LU;
	double **dif_C;

	double log_det;
	double *lu_col;
	double *hyp;
	double *n_vec;

	/* Declare integer global variables */

	double prior_strength_default ;
	double prior_length_default ;
	double prior_vertical_default ; /* s.d. */
	double prior_fractional_noise_default ; /* s.d. */
	double prior_fractional_theta2_default ;
	double prior_linear_default ;

	int C_periodic ; /* [10] */ /* Periodic covariance function */
	int C_linear ; /* 24 */ /* Use linear term in covariance function */

	int noiseprior ; /* was flag[19] */ /* (=1) Use (Inverse Gamma/Gaussian) prior on noise m,sd (=2) m,N */
	int lengthprior ; /* was flag[18] */ /* (=1) Use (Gamma/Gaussian) prior on length scales m,sd (=2) m,N */
	int theta1prior ; /* was flag[21] */ /* (=1) Use Inverse Gamma prior on theta1 m,sd (=2) m,N */
	int theta2prior ; /* was flag[22] */ /* (=1) Use Inverse Gamma prior on theta2 m,sd (=2) m,N */
	int theta0prior ; /* was 16 */ /* (=1) Use Gaussian prior on variable mean m,sd */
	int linearprior ; /* 26 */ /* (=1) Use Gaussian prior on linear term default */

	int optimize_lengthscales ; /* was [1] */ /* Optimize sig_k */
	int optimize_theta0 ; /* was [15] */ /* optimize theta0 */
	int optimize_theta1 ; /* was [2] */ /* Optimize theta1 */
	int optimize_theta2 ; /* was [3] */ /* Optimize theta2 */
	int optimize_noise ; /* was [4] */ /* Optimize noise model */
	int optimize_linear ; /* 25 */ /* Optimize linear term hyperparameters */
	int use_poly_length_scales ; /* was [30] */ /* Use polynomial length scales */
	int OPT_checkgrad ; /* 20 */ /* Perform maccheckgrad */
	int OPT_use_cg ; /* was [5] */ /* Use CG inversion for OPT */
	int OPT_tridiag_CG ; /* 33 */ /* (=0) double-CG method (=1) Tridiag CG routine (only works for OPT_use_cg == 1) */
	int OPT_say_evidence ; /* 28 */ /* Calculate evidence after optimization */
	int OPT_say_train_e ; /* 29 */ /* Calculate training error */

	int INT_load_targets ; /* 23 */ /* load in interpolation targets */
	int INT_use_cg ; /* was [8] */ /* Use CG inversion for INT */
	int INT_LUi ; /* was [9] */ /* (=1) Use LUi for INT (=0) Use LUd for INT */
	int INT_find_eb ; /* was [6] */ /* Calculate error bars for INT */
	int INT_find_sigma_nu ; /* [7] */ /* Calculate noise level for INT */
	int INT_include_inputs ; /* include input values in output file */

	int hyp_out ; /* [11] don't confuse with hypoutfile */ /* output hyperparameters */
	int hyp_in ; /* [13] */ /* load in hyperparameters */
	int write_Cinv_t ; /* 27 */ /* Output C_inv[][]*t[] vector to file */
	int write_hdump ; /* 34 */ /* save hyperparameters every iteration (to hypoutfile.dump) */
	int do_samples ; /* 35 */ /* generate random samples from GP (only a c_line option) */
	int do_INT ; /* [12] */ /* perform interpolation */
	int do_OPT ; /* 14 */ /* perform optimisation (=0) */
	int do_SEARCH ; /* 36 */ /* perform k_x optimization */
	int SEARCH_max ; /* 37*/ /* (=0) minimization (=1) maximization */
	int calc_Hessian ; /* 31 */ /* Calculate Hessian of coefficients for the length scales */
	int training_noise_in_datafile ; /* 32 */ /* Load in known training noise levels (in datafile) */

	int noise_free_predictions ;
	int rescaling_permitted ;
	int ninter;
	int nsig;
	int nhidden; /* I think this is the n basis functions for the noise model (1) */
	int ntrace;
	int ngauss;
	int nlin;
	int npoly; /* number of parameters used to parameterize some functino of the inputs (not the noise level). normally npoly = 0 ? */
	int SEED;
	int CG_opt;
	int opt_itmax ;
	int CG_int;
	int CG_max;
	int nw,nwb;
	int N_max; /* maximum number of vectors in CG ? */
	int gridx1;/* assuming xy grid for output, this is no. of points in the most
			   rapidly varying direction */
	int *indx; /* vector used by ludcmp and lubksb */

	/* globals required for polynomials for length scales */

	double **pk_x;
	double **PL;
	double root2tonh;

	/* number we divide dynamic range by to get initial length scales */

	double stuff;

	/* temporary global variables DJCM */

	/* globals required for macoptII */

	struct gq_args{
		double *u;
		int verbosity ;
		tpros* all;
	} ;

	/* Description : matrix to store mapping from integer 'k' to hyperparameter

	kth[i][1] - type of hyperparameter 1 - length scale polynomial co-eff.
	2 - theta1
	3 - theta2
	4 - noise neural network weights
	5 - linear term
	6 - theta0

	kth[i][2] - number of corresponding hyperparameter
	(this may equal k, if all hyperparameters are being
	optimized, but in general it will not. k runs over the
	hyperparameters that are being optimized)

	kth[i][3->] - details of corresponding hyperparameter

	eg. kth[i][1] = 1 kth[i][3->] ranges -> ngauss mInputDimension
	kth[i][1] = 2 kth[i][3->] ranges -> ngauss
	kth[i][1] = 3 kth[i][3->] ranges -> 1
	kth[i][1] = 4 kth[i][3->] ranges -> 2 (mInputDimension+1 nhidden)/(nhidden+1 1)
	kth[i][1] = 5 kth[i][3->] ranges -> mInputDimension
	*/
	int **kth;

	double mac_h_tol; /* was mac_tol */ /* tol during hyperparameter optimization */
	double mac_k_tol; /* tolerance during optimization of inputs */
	int mac_h_style ;/* whether to define tolerance using small
					 gradient (0) or small step size (1) */
	int mac_k_style ;/* whether to define tolerance using small
					 gradient (0) or small step size (1) */
	double mac_ep;
	int verbose;

	/* Global variables required for CG inversion */

	double **Mran;
	double **Mtemp;
	double **A; /* the covariance matrix */
	double bnd_percent;

	double *ws;
	double *mod_g;
	double **g;
	double **h;
	double **gs;
	double **hs;
	double *gam;
	double *lambda;
	double *gams;
	double *lambdas;

	double *hAh;
	double *Td,*Tl,*Tu;
	double *Td2,*Tl2,*Tu2;

	/* Legendre polynomials */

	double *L;
	double *Lscale;/* records full range of the input data */
	/* is used to set initial value of hyperparams */
	/* is used in PL_calc, nowhere else */
	double *Loff;
	double *Lhidden;
	double Lfact;


	/* globals required for periodic covariance function */

	double *wavel;

	/* globals required for Gamma priors */

	double **gamr_a;
	double **gamr_b;
	double **gamr2_a;
	double **gamr2_b;
	double gamn_a;
	double gamn_b;
	double gamt1_a;
	double gamt1_b;
	double gamt2_a;
	double gamt2_b;
	double gaussl_m;
	double gaussl_sd;
	double gausst0_m;
	double gausst0_sd;

	/* globals required for ls_error */

	double **ls_e;

	/* global variables for speeding up mac_inter_C and mac_inter_dif */

	double *ls;

	/* global variables for maxim routine */

	double *mk_p;
	double *mv;
	double *mv2;
	double *mls;
	double *max_kx;
	int *max_include; /* whether to include a given input in the adjusted ones */
	double **mdk_p;
	int max_log ;
	char max_logfile[200] ;
	int Ivary ; /* number of varying things */

	double max_beta ;
	double max_gam ;

	int max_itmax ; /* maximum number of iterations to do */

};
