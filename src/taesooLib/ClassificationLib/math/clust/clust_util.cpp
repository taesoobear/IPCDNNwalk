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


int
I_SigSetNClasses(struct SigSet *S)
{
  int i, count;

  for (i = 0, count = 0; i < S->nclasses; i++)
    if (S->classSig[i].used) count++;

  return count;
}


struct ClassData *
I_AllocClassData(struct SigSet *S, struct ClassSig *C, int npixels)
{
  struct ClassData *Data;

  Data = &(C->classData);
  Data->npixels = npixels;
  Data->x = G_alloc_matrix (npixels, S->nbands);
  Data->p = G_alloc_matrix (npixels, C->nsubclasses);
  Data->w = G_alloc_vector (npixels*sizeof(double));
  return Data;
}


void
I_InitSigSet(struct SigSet *S)
{
  S->nbands = 0;
  S->nclasses = 0;
  S->classSig = NULL;
  S->title = NULL;
}


void
I_SigSetNBands(struct SigSet *S, int nbands)
{
  S->nbands = nbands;
}


struct ClassSig *
I_NewClassSig(struct SigSet *S)
{
  struct ClassSig *Sp;
  if (S->nclasses == 0)
    S->classSig  = (struct ClassSig *) G_malloc (sizeof(struct ClassSig));
  else
    S->classSig  = (struct ClassSig *) G_realloc((char *)S->classSig ,
                  sizeof(struct ClassSig)*(S->nclasses+1));

  Sp = &S->classSig [S->nclasses++];
  Sp->classnum = 0;
  Sp->nsubclasses = 0;
  Sp->used = 1;
  Sp->type = SIGNATURE_TYPE_MIXED;
  Sp->title = NULL;
  Sp->classData.npixels = 0;
  Sp->classData.SummedWeights = 0.0;
  Sp->classData.x = NULL;
  Sp->classData.p = NULL;
  Sp->classData.w = NULL;
    
  return Sp;
}


struct SubSig *
I_NewSubSig(struct SigSet *S, struct ClassSig *C)
{
  struct SubSig *Sp;
  int i;

  if (C->nsubclasses == 0)
    C->subSig = (struct SubSig *) G_malloc (sizeof(struct SubSig));
  else
    C->subSig = (struct SubSig *) G_realloc ((char *)C->subSig,
		sizeof(struct SubSig) * (C->nsubclasses+1));

  Sp = &C->subSig[C->nsubclasses++];
  Sp->used = 1;
  Sp->R = (double **) G_calloc (S->nbands, sizeof(double *));
  Sp->R[0] = (double *) G_calloc (S->nbands * S->nbands, sizeof(double));
  for (i = 1; i < S->nbands; i++)
    Sp->R[i] = Sp->R[i-1] + S->nbands;
  Sp->Rinv = (double **) G_calloc (S->nbands, sizeof(double *));
  Sp->Rinv[0] = (double *) G_calloc (S->nbands * S->nbands, sizeof(double));
  for (i = 1; i < S->nbands; i++)
    Sp->Rinv[i] = Sp->Rinv[i-1] + S->nbands;
  Sp->means = (double *) G_calloc (S->nbands, sizeof(double));
  Sp->N = 0;
  Sp->pi = 0;
  Sp->cnst = 0;
  return Sp;
}


void
I_SetSigTitle(struct SigSet *S, char *title)
{
  if (title == NULL) title = "";
  if (S->title) free (S->title);
  S->title = G_malloc (strlen (title)+1);
  strcpy(S->title, title);
}


char *
I_GetSigTitle(struct SigSet *S)
{
  if (S->title) return S->title;
  else return "";
}


void
I_SetClassTitle(struct ClassSig *C, char *title)
{
  if (title == NULL) title = "";
  if (C->title) free (C->title);
  C->title = G_malloc (strlen (title)+1);
  strcpy(C->title, title);
}

char *
I_GetClassTitle(struct ClassSig *C)
{
  if (C->title) return C->title;
  else return "";
}


/* Deallocators */

void
I_DeallocClassData(struct SigSet *S, struct ClassSig *C)
{
  struct ClassData * Data;

  Data = &(C->classData);
  G_free_matrix(Data->x);
  Data->x = NULL;
  G_free_matrix(Data->p);
  Data->p = NULL;
  G_free_vector(Data->w);
  Data->w = NULL;
  Data->npixels = 0;
  Data->SummedWeights = 0.0;
}

    
void
I_DeallocSubSig(struct ClassSig *C)
{
  struct SubSig *Sp;

  Sp = &C->subSig[--C->nsubclasses];

  G_dealloc( (char *) Sp->R[0] );
  G_dealloc( (char *) Sp->R );
  G_dealloc( (char *) Sp->Rinv[0] );
  G_dealloc( (char *) Sp->Rinv );
  G_dealloc( (char *) Sp->means );

  C->subSig = (struct SubSig *) G_realloc((char *)C->subSig,
		    sizeof(struct SubSig) * (C->nsubclasses));
}


void
I_DeallocClassSig(struct SigSet *S)
{
  struct ClassSig *Sp;
    
  Sp = &(S->classSig [--S->nclasses]);
    
  I_DeallocClassData( S, Sp ); 
    
  while(Sp->nsubclasses>0) {
    I_DeallocSubSig(Sp);
  }
  S->classSig  = (struct ClassSig *) G_realloc ((char *)S->classSig ,
                sizeof(struct ClassSig) * (S->nclasses));
}     
    

void
I_DeallocSigSet(struct SigSet *S)
{
  while(S->nclasses>0) {
    I_DeallocClassSig( S );
  }
}



