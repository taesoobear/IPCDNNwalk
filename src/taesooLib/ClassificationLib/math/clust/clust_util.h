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


#ifndef CLUST_UTIL_H
#define CLUST_UTIL_H

int I_SigSetNClasses(struct SigSet *S);
struct ClassData *I_AllocClassData(struct SigSet *S, struct ClassSig *C, int npixels);
void I_InitSigSet(struct SigSet *S);
void I_SigSetNBands(struct SigSet *S, int nbands);
struct ClassSig *I_NewClassSig(struct SigSet *S);
struct SubSig *I_NewSubSig (struct SigSet *S, struct ClassSig *C);
void I_SetSigTitle(struct SigSet *S, char *title);
char *I_GetSigTitle(struct SigSet *S);
void I_SetClassTitle(struct ClassSig *C, char *title);
char *I_GetClassTitle(struct ClassSig *C);
void I_DeallocClassData(struct SigSet *S, struct ClassSig *C);
void I_DeallocSubSig(struct ClassSig *C);
void I_DeallocClassSig(struct SigSet *S);
void I_DeallocSigSet(struct SigSet *S);


#endif /* CLUST_UTIL_H */

