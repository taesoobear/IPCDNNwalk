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


#ifndef CLUST_DEFS_H
#define CLUST_DEFS_H

/*****************************************************/
/* This constant determines the ratio of the average */
/* covariance to the minimum allowed covariance.     */
/* It is used to insure that the measured covariance */
/* is not singular. It may need to be adjusted for   */
/* different applications.                           */
/*****************************************************/
#define COVAR_DYNAMIC_RANGE 1E5

#define CLUSTER_FULL 1 /* Use full covariance matrix in clustering */
#define CLUSTER_DIAG 0 /* Use diagonal covariance matrix in clustering */
#define CLUSTER_SPHR 2

#define PI 3.141592654

#include<stdio.h>
#include<stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>


/* SigSet (Signature Set) data stucture used throughout package.         */
/*   ClassSig (Class Signature) data stucture holds the parameters       */
/*       of a single Gaussian mixture model. SigSet.nclasses is the      */
/*       number of ClassSig's in a SigSet.                               */
/*     SubSig (Subsignature) data stucture holds each component of a     */
/*         Gaussian mixture model. SigSet.classSig[k].nsubclasses is the */
/*         number of SubSig's in a ClassSig.                             */

struct ClassData
{
    int npixels;
    double SummedWeights;
    double **x; /* list of pixel vectors:     x[npixels][nbands] */
    double **p; /* prob pixel is in subclass: p[npixels][subclasses] */
    double  *w; /* weight of pixel:           w[npixels] */
};

struct SubSig
{
    double N;       /* expected number of pixels in subcluster */
    double pi;      /* probability of component in GMM */
    double *means;  /* mean of component in GMM */
    double **R;     /* convarance of component in GMM */
    double **Rinv;  /* inverse of R */
    double cnst;    /* normalizing constant for multivariate Gaussian */
    int used;
};

struct ClassSig
{
    long classnum;
    char *title;
    int used;
    int type;
    int nsubclasses;
	SubSig *subSig;
	ClassData classData;
};

struct SigSet
{
    int nbands;
    int nclasses;
    char *title;
	
	ClassSig *classSig;
};





#define SIGNATURE_TYPE_MIXED 1


#endif /* CLUST_DEFS_H */
