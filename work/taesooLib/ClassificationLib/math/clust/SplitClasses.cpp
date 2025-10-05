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
#include "clust_util.h"
#include "clust_io.h"


int splitClasses(int  argc, char *argv[])
{
    FILE *fp;
    int i,j,k,l;
    struct SigSet Sin,Sout;
    struct ClassSig *Sig;
    char *paramfnameIn,*paramfnameOut;

    if( argc!=3 ) {
      fprintf(stderr,"\nUsage: %s InputParameters OutputParameters",argv[0]); 
      fprintf(stderr,"\n\n%s separates GMM components into individual classes.",argv[0]); 
      fprintf(stderr,"\nThis can be useful for unsupervised segmentation applications.\n\n"); 
      exit(-1);
    }

    /* Read Command Line */
    paramfnameIn = argv[1];
    paramfnameOut = argv[2];


    /* Read SigSet from parameter file */
    if((fp=fopen(paramfnameIn,"r"))==NULL) {
      fprintf(stderr, "\nError: Can't open parameter file %s",paramfnameIn); 
      exit(-1);
    }
    I_ReadSigSet (fp,&Sin);
    fclose(fp);

    /* Initialize SigSet data structure */
    I_InitSigSet (&Sout);
    I_SigSetNBands (&Sout, Sin.nbands);
    I_SetSigTitle (&Sout, "signature set for unsupervised clustering");

    /* Copy each subcluster (subsignature) from input to cluster (class signature) of output */
    for(k=0; k<Sin.nclasses; k++) {
      for(l=0; l<(Sin.classSig[k].nsubclasses); l++) {
        Sig = I_NewClassSig(&Sout);
        I_SetClassTitle (Sig, "Single Model Class");
        I_NewSubSig (&Sout, Sig);
        Sig->subSig[0].pi = 1.0;
        for(i=0; i<Sin.nbands; i++) {
          Sig->subSig[0].means[i] = Sin.classSig[k].subSig[l].means[i];
        }
        for(i=0; i<Sin.nbands; i++)
        for(j=0; j<Sin.nbands; j++) {
          Sig->subSig[0].R[i][j] = Sin.classSig[k].subSig[l].R[i][j];
        }
      }
    }

    /* Write out result to output parameter file */
    if((fp=fopen(paramfnameOut,"w"))==NULL) {
      fprintf(stderr, "\nError: Can't open parameter file %s",paramfnameOut); 
    }
    I_WriteSigSet(fp, &Sout);
    fclose(fp);
                                                                                
    /* De-allocate cluster signature memory */
    I_DeallocSigSet (&Sin);
    I_DeallocSigSet (&Sout);

    return(0);
}


