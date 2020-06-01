#include "stdafx.h"
/*
* All questions regarding the software should be addressed to
* 
*       Prof. Charles A. Bouman
*       Purdue University
*       School of Electrical and Computer Engineering
*       1285 Electrical Engineering Building
*       West Lafayette, IN 47907-1285
*       USA
*       +1 765 494 0340
*       +1 765 494 3358 (fax)
*       email:  bouman@ecn.purdue.edu
*       http://www.ece.purdue.edu/~bouman
* 
* Copyright (c) 1995 The Board of Trustees of Purdue University.
*
* Permission to use, copy, modify, and distribute this software and its
* documentation for any purpose, without fee, and without written agreement is
* hereby granted, provided that the above copyright notice and the following
* two paragraphs appear in all copies of this software.
*
* IN NO EVENT SHALL PURDUE UNIVERSITY BE LIABLE TO ANY PARTY FOR DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE
* USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF PURDUE UNIVERSITY HAS
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* PURDUE UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS,
* AND PURDUE UNIVERSITY HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT,
* UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
*/


#include<stdio.h>
#include<stdlib.h>
#include <math.h>
#include <float.h>

#include "alloc_util.h"


static double double_abs(double x);

static int 
ludcmp(double **a,int n,int *indx,double *d);

static void 
lubksb(double **a,int n,int *indx,double *b);




/***********************************************************/
/* inverts a matrix of arbitrary size input as a 2D array. */ 
/***********************************************************/
int clust_invert(
  double **a,      /* input/output matrix */
  int    n,        /* dimension */
  double *det_man, /* determinant mantisa */
  int    *det_exp, /* determinant exponent */
  /* scratch space */
  int    *indx,    /* indx = G_alloc_ivector(n);  */
  double **y,      /* y = G_alloc_matrix(n,n); */
  double *col      /* col = G_alloc_vector(n); */
)
{
	int  i,j;
	double  d_man;
        int d_exp;

        d_exp = 0;
        if(ludcmp(a,n,indx,&d_man)) {
          for(j=0; j<n; j++) {
            d_man *= a[j][j];
            while( double_abs(d_man)>10 ) {
              d_man = d_man/10;
              d_exp++;
            }
            while( (double_abs(d_man)<0.1)&&(double_abs(d_man)>0) ) {
              d_man = d_man*10;
              d_exp--;
            }
          }
          *det_man = d_man;
          *det_exp = d_exp;
	  for(j=0; j<n; j++) {
	    for(i=0; i<n; i++) col[i]=0.0;
	    col[j]=1.0;
	    lubksb(a,n,indx,col);
	    for(i=0; i<n; i++) y[i][j]=col[i];
	  } 

	  for(i=0; i<n; i++)
	  for(j=0; j<n; j++) a[i][j]=y[i][j];
          return(1);
        }
        else {
          *det_man = 0.0;
          *det_exp = 0;
          return(0);
        }
}


static double double_abs(double x)
{
       if(x<0) x = -x;
       return(x);
}





#define TINY 1.0e-20


static int 
ludcmp(double **a,int n,int *indx,double *d)
{
    int i,imax,j,k;
    double big,dum,sum,temp;
    double *vv;

    vv=G_alloc_vector(n);
    *d=1.0;
    for (i=0;i<n;i++) 
    {
        big=0.0;
        for (j=0;j<n;j++)
            if ((temp=fabs(a[i][j])) > big)
		big=temp;
        if (big == 0.0)
	    return 0; /* Singular matrix  */
        vv[i]=1.0/big;
    }
    for (j=0;j<n;j++) 
    {
        for (i=0;i<j;i++) 
	{
            sum=a[i][j];
            for (k=0;k<i;k++)
		sum -= a[i][k]*a[k][j];
            a[i][j]=sum;
        }
        big=0.0;
        for (i=j;i<n;i++) 
	{
            sum=a[i][j];
            for (k=0;k<j;k++)
                sum -= a[i][k]*a[k][j];
            a[i][j]=sum;
            if ( (dum=vv[i]*fabs(sum)) >= big) 
	    {
                big=dum;
                imax=i;
            }
        }
        if (j != imax) 
	{
            for (k=0;k<n;k++) 
	    {
                dum=a[imax][k];
                a[imax][k]=a[j][k];
                a[j][k]=dum;
            }
            *d = -(*d);
            vv[imax]=vv[j];
        }
        indx[j]=imax;

 	/* Change made 3/27/98 for robustness */
        if ( (a[j][j]>=0)&&(a[j][j]<TINY) ) a[j][j]= TINY;
        if ( (a[j][j]<0)&&(a[j][j]>-TINY) ) a[j][j]= -TINY;

        if (j != n-1) 
	{
            dum=1.0/(a[j][j]);
            for (i=j+1;i<n;i++)
		a[i][j] *= dum;
        }
    }
    G_free_vector (vv);
    return(1);
}

#undef TINY

static void 
lubksb(double **a,int n,int *indx,double *b)
{
    int i,ii,ip,j;
    double sum;

    ii = -1;
    for (i=0;i<n;i++)
    {
        ip=indx[i];
        sum=b[ip];
        b[ip]=b[i];
        if (ii >= 0)
            for (j=ii;j<i;j++)
		sum -= a[i][j]*b[j];
        else if (sum)
	    ii=i;
        b[i]=sum;
    }
    for (i=n-1;i>=0;i--)
    {
        sum=b[i];
        for (j=i+1;j<n;j++)
	    sum -= a[i][j]*b[j];
        b[i]=sum/a[i][i];
    }
}

