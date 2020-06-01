#pragma once 
namespace tpros_nrutil
{

static double dsqrarg; 
#define DSQR(a)   ((dsqrarg=(a)) == 0.0 ? 0.0 : dsqrarg*dsqrarg)

#define SIGN(a,b) ((b) > 0.0 ? fabs(a) : -fabs(a))

static double dmaxarg1,dmaxarg2; 
#define DMAX(a,b) (dmaxarg1=(a),dmaxarg2=(b),(dmaxarg1) > (dmaxarg2) ? (dmaxarg1) : (dmaxarg2))

float *vector(int,int);
float **matrix(int,int,int,int);
float **convert_matrix(float *,int,int,int,int);
double *dvector(int,int);
double **dmatrix(int,int,int,int);
int *ivector(int,int);
int **imatrix(int,int,int,int);
unsigned char *cvector(int,int);
unsigned char **cmatrix(int,int,int,int);
float **submatrix(float **,int,int,int,int,int,int);
/*
void free_vector(float *,int,int);
*/
void free_dvector(double *,int,int);
void free_cvector(unsigned char *,int,int);
void free_cmatrix(unsigned char **,int,int,int,int);
void free_ivector(int *,int,int);
void free_matrix(float **,int,int,int,int);
void free_dmatrix(double **, int,int,int,int);
void free_imatrix(int **,int,int,int,int);
void free_submatrix(float **,int,int,int,int);
void free_convert_matrix(float **,int,int,int,int);
void nrerror( const char * );

/* Extra routines added by MNG from NR book */

void ludcmp(double **,int,int *,double *);
void lubksb(double **,int,int *,double *);
void tred2(double **,int,double *,double *);
void tqli(double *,double *,int,double **);

double pythag(double,double);

void newt(double [],int,int *,
	  void (*vecfunc)(int,double [],double []),
	  void (*fdjac)(int,double [],double [],double **,
			void (*)(int,double [],double [])) );
void lnsrch(int,double [],double ,double [],double [],
	    double [],double *,double,int *,
	    double (*func)(double []));
double fmin(double []);

void tridiag(double *,double *,double *,double *,double *,int);

double ***dmatrix3( int   , int , int , int , int , int ) ;
}