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
#include "clust_io.h"
#include "classify_util.h"


int classify(int  argc,char *argv[])
{
    FILE *fp;
    int i,j;
    struct SigSet S;
    char *paramfname,*datafname;
    int NDataVectors,NRead;
    double **data,tmp,*ll;
    int maxindex;
    double maxval;

    /* Read Command Line */
    paramfname = argv[1];
    datafname = argv[2];


    /* Read SigSet from parameter file */
    if((fp=fopen(paramfname,"r"))==NULL) {
      fprintf(stderr, "\nError: Can't open parameter file %s",paramfname); 
      exit(-1);
    }
    I_ReadSigSet (fp,&S);
    fclose(fp);


    /* Determine number of lines in file */
    if((fp=fopen(datafname,"r"))==NULL) {
      fprintf(stderr, "\nError: Can't open data file %s",datafname); 
      exit(-1);
    }
    NRead = 1;
    NDataVectors = -1;
    while( NRead>0 ) {
      for(j=0; j<S.nbands; j++) {
        NRead = fscanf(fp,"%lf",&tmp );
      }
      fscanf(fp,"\n");
      NDataVectors++;
    }
    fclose(fp);


    /* Read lines from file */
    if((fp=fopen(datafname,"r"))==NULL) {
      fprintf(stderr, "\nError: Can't open data file %s",datafname); 
      exit(-1);
    }
    data = G_alloc_matrix(NDataVectors,S.nbands);
    for(i=0; i<NDataVectors; i++) {
      for(j=0; j<S.nbands; j++) {
        fscanf(fp,"%lf",&(data[i][j]));
      }
      fscanf(fp,"\n");
    }
    fclose(fp);

    /* Initialize constants for Log likelihood calculations */
    ClassLogLikelihood_init(&S);

    /* Compute Log likelihood for each class*/
    ll = G_alloc_vector(S.nclasses);

    for(i=0; i<NDataVectors; i++) {
      ClassLogLikelihood(data[i],ll,&S); 

      maxval = ll[0];
      maxindex = 0;
      for(j=0; j<S.nclasses; j++) {
        if( ll[j] > maxval ) {
          maxval = ll[j];
          maxindex = j;
        }
      }

      for(j=0; j<S.nclasses; j++) printf("Loglike = %g ",ll[j]); 
      printf("ML Class = %d\n",maxindex); 
    }

    G_free_vector(ll);
    G_free_matrix(data);
    return(0);
}

