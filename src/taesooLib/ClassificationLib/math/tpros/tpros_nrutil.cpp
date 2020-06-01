
#include "stdafx.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "tpros_nrutil.h"


void tpros_nrutil::nrerror(const char error_text[])
{
	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	ASSERT(0);
	exit(1);
}



float *tpros_nrutil::vector(int nl,int mInputDimension)
{
	float *v;

	v=(float *)malloc((unsigned) (mInputDimension-nl+1)*sizeof(float));
	if (!v) nrerror("allocation failure in vector()");
	return v-nl;
}

int *tpros_nrutil::ivector(int nl,int mInputDimension)
{
	int *v;

	v=(int *)malloc((unsigned) (mInputDimension-nl+1)*sizeof(int));
	if (!v) nrerror("allocation failure in ivector()");
	return v-nl;
}

unsigned char *tpros_nrutil::cvector(int nl,int mInputDimension)
{
	unsigned char *v;

	v=(unsigned char *)malloc((unsigned) (mInputDimension-nl+1)*sizeof(unsigned char));
	if (!v) nrerror("allocation failure in ivector()");
	return v-nl;
}

double *tpros_nrutil::dvector(int nl,int mInputDimension)
{
	double *v;

	v=(double *)malloc((unsigned) (mInputDimension-nl+1)*sizeof(double));
	if (!v) nrerror("allocation failure in dvector()");
	return v-nl;
}



float **tpros_nrutil::matrix(int nrl,int nrh,int ncl,int nch)
{
	int i;
	float **m;

	m=(float **) malloc((unsigned) (nrh-nrl+1)*sizeof(float*));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i]=(float *) malloc((unsigned) (nch-ncl+1)*sizeof(float));
		if (!m[i]) nrerror("allocation failure 2 in matrix()");
		m[i] -= ncl;
	}
	return m;
}

double **tpros_nrutil::dmatrix(int nrl,int nrh,int ncl,int nch)
{
	int i;
	double **m;

	m=(double **) malloc((unsigned) (nrh-nrl+1)*sizeof(double*));
	if (!m) nrerror("allocation failure 1 in dmatrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i]=(double *) malloc((unsigned) (nch-ncl+1)*sizeof(double));
		if (!m[i]) nrerror("allocation failure 2 in dmatrix()");
		m[i] -= ncl;
	}
	return m;
}

int **tpros_nrutil::imatrix(int nrl,int nrh,int ncl,int nch)
{
	int i,**m;

	m=(int **)malloc((unsigned) (nrh-nrl+1)*sizeof(int*));
	if (!m) nrerror("allocation failure 1 in imatrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i]=(int *)malloc((unsigned) (nch-ncl+1)*sizeof(int));
		if (!m[i]) nrerror("allocation failure 2 in imatrix()");
		m[i] -= ncl;
	}
	return m;
}

unsigned char **tpros_nrutil::cmatrix(int nrl,int nrh,int ncl,int nch)
{
	int i;
	unsigned char **m;

	m=(unsigned char **)malloc((unsigned) (nrh-nrl+1)*sizeof(unsigned char*));
	if (!m) nrerror("allocation failure 1 in imatrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i]=(unsigned char *)malloc((unsigned) (nch-ncl+1)*sizeof(unsigned char));
		if (!m[i]) nrerror("allocation failure 2 in imatrix()");
		m[i] -= ncl;
	}
	return m;
}



float **	tpros_nrutil:: submatrix(float **a,
		  int oldrl,int oldrh,int oldcl,int oldch,
		  int newrl,int newcl)
{
	int i,j;
	float **m;

	m=(float **) malloc((unsigned) (oldrh-oldrl+1)*sizeof(float*));
	if (!m) nrerror("allocation failure in submatrix()");
	m -= newrl;

	for(i=oldrl,j=newrl;i<=oldrh;i++,j++) m[j]=a[i]+oldcl-newcl;

	return m;
}


/*
void free_vector(float *v,int nl,int mInputDimension)
{
	free((char*) (v+nl));
}
*/

void tpros_nrutil::free_cvector(unsigned char *v,int nl,int mInputDimension)
{
	free((char*) (v+nl));
}

void tpros_nrutil::free_ivector(int *v,int nl,int mInputDimension)
{
	free((char*) (v+nl));
}

void tpros_nrutil::free_dvector(double *v,int nl,int mInputDimension)
{
	free((char*) (v+nl));
}


void tpros_nrutil::free_matrix(float **m,int nrl,int nrh,int ncl,int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}

void tpros_nrutil::free_dmatrix(double **m,int nrl,int nrh,int ncl,int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}

void tpros_nrutil::free_cmatrix(unsigned char **m,int nrl,int nrh,int ncl,int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}

void tpros_nrutil::free_imatrix(int **m,int nrl,int nrh,int ncl,int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}

namespace tpros_nrutil
{



void free_submatrix(float **b,int nrl,int nrh,int ncl,int nch)
{
	free((char*) (b+nrl));
}



float **convert_matrix(float *a,int nrl,int nrh,int ncl,int nch)
{
	int i,j,nrow,ncol;
	float **m;

	nrow=nrh-nrl+1;
	ncol=nch-ncl+1;
	m = (float **) malloc((unsigned) (nrow)*sizeof(float*));
	if (!m) nrerror("allocation failure in convert_matrix()");
	m -= nrl;
	for(i=0,j=nrl;i<=nrow-1;i++,j++) m[j]=a+ncol*i-ncl;
	return m;
}



void free_convert_matrix(float **b,int nrl,int nrh,int ncl,int nch)
{
	free((char*) (b+nrl));
}

void ludcmp( double **a, int n, int *indx, double *d)
{
  int i,imax=-1 ,j,k;
  double big,dum,sum,temp;
  double *vv;

  vv=dvector(1,n);
  *d=1.0;
  for(i=1; i <= n; i++){
    big = 0.0;
    for(j=1; j <= n; j++)
      if((temp=fabs(a[i][j])) > big) big = temp;
    if(big == 0.0) nrerror("Singular matrix in routine ludcmp");
    vv[i] = 1.0/big;
  }
  for(j=1; j <= n; j++){
    for(i=1; i < j; i++){
      sum = a[i][j];
      for(k=1; k < i; k++) sum -= a[i][k] * a[k][j];
      a[i][j] = sum;
    }
    big = 0.0;
    for(i=j; i <= n ; i++){
      sum = a[i][j];
      for(k=1; k < j ; k++)
	sum -= a[i][k] * a[k][j];
      a[i][j] = sum;
      if ( (dum=vv[i]*fabs(sum)) >= big ){
	big = dum;
	imax = i;
      }
    }
    if ( imax == -1 ) {
      nrerror("LUDCMP has impossible value of imax. Conjecture that matrix a is Infinite");
    }
    if (j != imax) { /* NOTE, if a[][]= Infinity then
				    you can get to this point with imax not set */    
      for(k=1; k <= n; k++){
	dum = a[imax][k];
	a[imax][k] = a[j][k];
	a[j][k] = dum;
      }
      *d = -(*d);
      vv[imax] = vv[j];
    }
    indx[j] = imax;
    if( a[j][j] == 0.0 ) a[j][j] = 1.0e-50;
    if( j != n ) {
      dum = 1.0/(a[j][j]);
      for(i=j+1; i <= n; i++) a[i][j] *= dum;
    }
  }
  free_dvector(vv,1,n);

}

void lubksb(double **a,int n,int *indx,double *b)
{
  int    i,ii=0,ip,j;
  double sum;

  for(i=1; i <= n; i++){
    ip=indx[i];
    sum=b[ip];
    b[ip]=b[i];
    if(ii)
      for(j=ii;j<=i-1;j++) sum -= a[i][j] * b[j];
    else if (sum) ii=i;
    b[i] = sum;
  }
  for(i=n;i>=1;i--){
    sum=b[i];
    for(j=i+1;j<=n;j++) sum -= a[i][j]*b[j];
    b[i] = sum/a[i][i];
  }
}

void tqli(double d[],double e[],int n,double **z)
{
  double pythag(double a,double b);
  int    m,l,iter,i,k;
  double s,r,p,g,f,dd,c,b;

  for(i=2;i<=n;i++) e[i-1]=e[i];
  e[n]=0.0;
  for(l=1;l<=n;l++){
    iter=0;
    do {
      for(m=l;m<=n-1;m++){
	dd=fabs(d[m])+fabs(d[m+1]);
	if ((double)(fabs(e[m])+dd) == dd) break;
      }
      if(m != l){
	if(iter++ == 30) nrerror("Too many iterations in tqli");
	g=(d[l+1]-d[l])/(2.0*e[l]);
	r=pythag(g,1.0);
	g=d[m]-d[l]+e[l]/(g+SIGN(r,g));
	s=c=1.0;
	p=0.0;
	for(i=m-1;i>=l;i--){
	  f=s*e[i];
	  b=c*e[i];
	  e[i+1]=(r=pythag(f,g));
	  if(r == 0.0){
	    d[i+1] -= p;
	    e[m] = 0.0;
	    break;
	  }
	  s=f/r;
	  c=g/r;
	  g=d[i+1]-p;
	  r=(d[i]-g)*s+2.0*c*b;
	  d[i+1]=g+(p=s*r);
	  g=c*r-b;
	  for(k=1;k<=n;++k){
	    f=z[k][i+1];
	    z[k][i+1]=s*z[k][i]+c*f;
	    z[k][i]=c*z[k][i]-s*f;
	  }
	}
	if(r == 0.0 && i) continue;
	d[l] -= p;
	e[l] = g;
	e[m] = 0.0;
      }
    } while (m != l);
  }

}

void tred2(double **a,int n,double d[],double e[])
{
  int    l,k,j,i;
  double scale,hh,h,g,f;
  
  for(i=n;i>=2;i--){
    l=i-1;
    h=scale=0.0;
    if(l > 1){
      for(k=1;k<=l;k++) scale += fabs(a[i][k]);
      if(scale == 0.0) e[i]=a[i][l];
      else {
	for(k=1;k<=l;++k){
	  a[i][k] /= scale;
	  h += a[i][k]*a[i][k];
	}
	f=a[i][l];
	g=(f > 0.0 ? -sqrt(h) : sqrt(h) );
	e[i]=scale*g;
	h -= f*g;
	a[i][l]=f-g;
	f=0.0;
	for(j=1;j<=l;j++){
	  a[j][i]=a[i][j]/h;
	  g=0.0;
	  for(k=1;k<=j;k++) g += a[j][k]*a[i][k];
	  for(k=j+1;k<=l;k++) g += a[k][j]*a[i][k];
	  e[j]=g/h;
	  f += e[j]*a[i][j];
	}
	hh=f/(h+h);
	for(j=1;j<=l;j++){
	  f=a[i][j];
	  e[j]=g=e[j]-hh*f;
	  for(k=1;k<=j;k++) a[j][k] -= (f*e[k]+g*a[i][k]);
	}
      }
    } else e[i]=a[i][l];
    d[i]=h;
  }
  d[1]=0.0;
  e[1]=0.0;
  for(i=1;i<=n;i++){
    l=i-1;
    if(d[i]){
      for(j=1;j<=l;j++){
	g=0.0;
	for(k=1;k<=l;k++)
	  g += a[i][k]*a[k][j];
	for(k=1;k<=l;k++)
	  a[k][j] -= g*a[k][i];
      }
    }
    d[i]=a[i][i];
    a[i][i]=1.0;
    for(j=1;j<=l;j++) a[j][i]=a[i][j]=0.0;
  }

}

double pythag(double a,double b)
{
  double absa,absb;
  absa=fabs(a);
  absb=fabs(b);
  if(absa > absb) return absa*sqrt(1.0+DSQR(absb/absa));
  else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+DSQR(absa/absb)));
}

#define TOLX    1.0e-7
#define MAXITS  200
#define TOLF    1.0e-4
#define TOLMIN  1.0e-6
#define STPMX   100.0
#define FREERETURN {free_dvector(fvec,1,n);free_dvector(xold,1,n);\
		    free_dvector(p,1,n);free_dvector(g,1,n);\
		    free_dmatrix(fjac,1,n,1,n);free_ivector(indx,1,n);\
		    return;}

int     nn;
double   *fvec;
void    (*nrfuncv)(int n,double v[],double f[]);

void newt(double x[],int n,int *check,
	  void (*vecfunc)(int,double [],double []),
	  void (*fdjac)(int,double [],double [],double **,
			void (*)(int,double [],double [])) )
{
  int i,its,j,*indx;
  double d,den,f,fold,stpmax,sum,temp,test,**fjac,*g,*p,*xold;

  indx = ivector(1,n);
  fjac = dmatrix(1,n,1,n);
  g    = dvector(1,n);
  p    = dvector(1,n);
  xold = dvector(1,n);
  fvec = dvector(1,n);

  nn      = n;
  nrfuncv = vecfunc;
  f       = fmin(x);
  test    = 0.0;

  for(i=1;i<=n;i++)
    if( fabs(fvec[i]) > test) test = fabs(fvec[i]);
  if(test < 0.01*TOLF) FREERETURN
  for(sum=0.0,i=1;i<=n;i++) sum += DSQR(x[i]);
  stpmax = STPMX*DMAX(sqrt(sum),(double)n);

  for(its=1; its<=MAXITS;its++){
    (*fdjac)(n,x,fvec,fjac,vecfunc);
    for(i=1;i<=n;i++){
      for(sum=0.0,j=1;j<=n;j++) sum += fjac[j][i]*fvec[j];
      g[i] = sum;
    }
    for(i=1;i<=n;i++) xold[i]=x[i];
    fold = f;
    for(i=1;i<=n;i++) p[i] = -fvec[i];
    ludcmp(fjac,n,indx,&d);
    lubksb(fjac,n,indx,p);
    lnsrch(n,xold,fold,g,p,x,&f,stpmax,check,fmin);
    test = 0.0;
    for(i=1;i<=n;i++)
      if(fabs(fvec[i]) > test) test = fabs(fvec[i]);
    if(test < TOLF){
      *check = 0;
      FREERETURN
    }
    if(*check){
      test = 0.0;
      den = DMAX(f,0.5*n);
      for(i=1;i<=n;i++){
	temp=fabs(g[i])*DMAX(fabs(x[i]),1.0)/den;
	if( temp > test ) test = temp;
      }
      *check = (test < TOLMIN ? 1 : 0);
      FREERETURN
    }
    test = 0.0;
    for(i=1;i<=n;i++){
      temp = (fabs(x[i]-xold[i]))/DMAX(fabs(x[i]),1.0);
      if( temp > test ) test = temp;
    }
    if( test < TOLX ) FREERETURN
  }
  nrerror("MAXITS exceeded in newt");

}

#define ALF     1.0e-4
#define TOLX    1.0e-7

void lnsrch(int n,double xold[],double fold,double g[],double p[],
	    double x[],double *f,double stpmax,int *check,
	    double (*func)(double []))
{
  int   i;
  double a,alam,alam2,alamin,b,disc,f2,fold2;
  double rhs1,rhs2,slope,sum,temp,test,tmplam;

  *check = 0;
  for(sum=0.0,i=1;i<=n;i++) sum += p[i]*p[i];
  sum=sqrt(sum);
  if(sum > stpmax)
    for(i=1;i<=n;i++) p[i] *= stpmax/sum;
  for(slope=0.0,i=1;i<=n;i++)
    slope += g[i]*p[i];
  test=0.0;
  for(i=1;i<=n;i++){
    temp=fabs(p[i])/DMAX(fabs(xold[i]),1.0);
    if(temp > test) test=temp;
  }
  alamin = TOLX/test;
  alam   = 1.0;
  for(;;){
    for(i=1;i<=n;i++) x[i] = xold[i]+alam*p[i];
    *f=(*func)(x);
    if(alam < alamin){
      for(i=1;i<=n;i++) x[i] = xold[i];
      *check=1;
      return;
    } else if(*f <= fold+ALF*alam*slope) return;
    else {
      if( alam == 1.0 )
	tmplam = -slope/(2.0*(*f-fold-slope));
      else {
	rhs1 = *f-fold-alam*slope;
	rhs2 = f2-fold2-alam2*slope;
	a = (rhs1/(alam*alam) - rhs2/(alam2*alam2))/(alam-alam2);
	b = (-alam2*rhs1/(alam*alam) + alam*rhs2/(alam2*alam2))/(alam-alam2);
	if(a == 0.0) tmplam = -slope/(2.0*b);
	else {
	  disc = b*b-3.0*a*slope;
	  if(disc < 0.0) nrerror("Roundoff problem in lnsrch.");
	  else tmplam = (-b+sqrt(disc))/(3.0*a);
	}
	if(tmplam > 0.5*alam)
	  tmplam = 0.5*alam;
      }
    }
    alam2 = alam;
    f2    = *f;
    fold2 = fold;
    alam  = DMAX(tmplam,0.1*alam);
  }

}

/* extern int nn;
   extern double *fvec;
   extern void (*nrfuncv)(int n,double v[],double f[]); */

double fmin(double x[])
{
  int i;
  double sum;

  (*nrfuncv)(nn,x,fvec);
  for(sum = 0.0,i=1;i <= nn;i++) sum += DSQR(fvec[i]);
  return 0.5*sum;
}

void tridiag(double *a,double *b,double *c,double *r,double *u,int n)
{
  int j;
  double bet,*gam;

  gam = dvector(1,n);

  if( b[1] == 0.0 ) nrerror("Error 1 in TRIDIAG");
  u[1] = r[1]/(bet=b[1]);
  for(j=2; j <= n; ++j){
    gam[j] = c[j-1]/bet;
    bet = b[j]-a[j]*gam[j];
    if( bet == 0.0 ) nrerror("Error 2 in TRIDIAG");
    u[j] =(r[j]-a[j]*u[j-1])/bet;
  }
  for(j=(n-1); j >= 1; j--) u[j] -= gam[j+1]*u[j+1];
  
  free_dvector(gam,1,n);
}

double ***dmatrix3( int l1 , int h1 , int l2 , int h2 , int l3 , int h3 )
{
  double ***c;
  int s1,i;
  
  s1=h1-l1+1;
  
  c=(double ***) malloc((unsigned) s1*sizeof(double **)) - l1;
  for (i=l1;i<=h1;i++)
    c[i]=dmatrix(l2,h2,l3,h3);
  return c;
}

}