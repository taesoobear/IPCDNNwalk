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


#include "clust_defs.h"
#include "alloc_util.h"
#include "classify_util.h"


void ClassLogLikelihood(
  double *vector, 
  double *ll,        /* log likelihood, ll[class] */
  struct SigSet *S   /* class signatures */
)
{
    int  m;               /* class index */
    int  k;               /* subclass index */
    int  b1,b2;           /* spectral index */
    int  max_nsubclasses; /* maximum number of subclasses */
    int  nbands;          /* number of spectral bands */
    double *subll;        /* log likelihood of subclasses */
    double *diff; 
    double maxlike;
    double subsum;
    struct ClassSig *C;
    struct SubSig *SubS;

    nbands = S->nbands;

    /* determine the maximum number of subclasses */
    max_nsubclasses = 0;
    for(m=0; m<S->nclasses; m++ )
      if(S->classSig[m].nsubclasses>max_nsubclasses)
        max_nsubclasses = S->classSig[m].nsubclasses;

    /* allocate memory */
    diff = (double *)G_malloc(nbands*sizeof(double));
    subll = (double *)G_malloc(max_nsubclasses*sizeof(double));

    /* Compute log likelihood for each class */

    /* for each class */
    for(m=0; m<S->nclasses; m++ ) {
      C = &(S->classSig[m]);

      /* compute log likelihood for each subclass */
      for(k=0; k<C->nsubclasses; k++) {
        SubS = &(C->subSig[k]);
        subll[k] = SubS->cnst;
        for(b1=0; b1<nbands; b1++) {
          diff[b1] = vector[b1] - SubS->means[b1];
          subll[k] -= 0.5*diff[b1]*diff[b1]*SubS->Rinv[b1][b1];
        }
        for(b1=0; b1<nbands; b1++) 
        for(b2=b1+1; b2<nbands; b2++)
          subll[k] -= diff[b1]*diff[b2]*SubS->Rinv[b1][b2];
      }

      /* shortcut for one subclass */
      if(C->nsubclasses==1) {
        ll[m] = subll[0];
      }
      /* compute mixture likelihood */
      else {
        /* find the most likely subclass */
        for(k=0; k<C->nsubclasses; k++)
        {
          if(k==0) maxlike = subll[k];
          if(subll[k]>maxlike) maxlike = subll[k];
        }

        /* Sum weighted subclass likelihoods */
        subsum = 0;
        for(k=0; k<C->nsubclasses; k++)
          subsum += exp( subll[k]-maxlike )*C->subSig[k].pi;

        ll[m] = log(subsum) + maxlike;
      }
    }
    free((char *)diff);
    free((char *)subll);
}




void ClassLogLikelihood_init(struct SigSet *S)
{
   int m; 
   int i;
   int b1,b2;
   int nbands;
   double *lambda;
   struct ClassSig *C;
   struct SubSig *SubS;

   nbands = S->nbands;
   /* allocate scratch memory */
   lambda = (double *)G_malloc(nbands*sizeof(double));

   /* invert matrix and compute constant for each subclass */

   /* for each class */
   for(m=0; m<S->nclasses; m++ ) {
     C = &(S->classSig[m]);

     /* for each subclass */
     for(i=0; i<C->nsubclasses; i++) {
        SubS = &(C->subSig[i]);

        /* Test for symetric  matrix */
        for(b1=0; b1<nbands; b1++)
        for(b2=0; b2<nbands; b2++) {
          if(SubS->R[b1][b2]!=SubS->R[b2][b1]) {
            fprintf(stderr,"\nWarning: nonsymetric covariance for class %d ",m+1);
            fprintf(stderr,"Subclass %d\n",i+1);
          }
          SubS->Rinv[b1][b2] = SubS->R[b1][b2];
        }

        /* Test for positive definite matrix */
        eigen(SubS->Rinv,lambda,nbands);
        for(b1=0; b1<nbands; b1++) {
          if(lambda[b1]<=0.0) {
            fprintf(stderr,"Warning: nonpositive eigenvalues for class %d",m+1);
            fprintf(stderr,"Subclass %d\n",i+1);
          }
        }

        /* Precomputes the cnst */
        SubS->cnst = (-nbands/2.0)*log(2*PI);
        for(b1=0; b1<nbands; b1++) {
            SubS->cnst += - 0.5*log(lambda[b1]);
        }

        /* Precomputes the inverse of tex->R */
        invert(SubS->Rinv,nbands);
      }
    }
    free((char *)lambda);
}

