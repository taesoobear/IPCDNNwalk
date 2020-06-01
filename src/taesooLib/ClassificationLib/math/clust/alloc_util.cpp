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


char *G_malloc(int n)
{
    char *b;

    b = (char*)malloc((unsigned)n);
    if (b || !n) return(b);

    fprintf (stderr, "Out Of Memory\n");
    exit(1);
}


char *G_calloc(int n,int m)
{
    char *b;

    b = (char*)calloc((unsigned)n,(unsigned)m);
    if (b || !n || !m) return(b);

    fprintf (stderr, "Out Of Memory\n");
    exit(1);
}


char *G_realloc(char *b,int n)
{
    if (b == NULL) b = (char*)malloc ((unsigned)n);
    else b = (char*)realloc(b, (unsigned)n);
    if (b || !n) return(b);

    fprintf (stderr, "Out Of Memory\n");
    exit(1);
}

void G_dealloc(char *b)
{
    free( b );
} 





double *G_alloc_vector(int n)
{
    return (double *) G_calloc (n, sizeof(double));
}


double **G_alloc_matrix(int rows,int cols)
{
    double **m;
    int i;

    m = (double **) G_calloc (rows, sizeof(double *));
    m[0] = (double *) G_calloc (rows*cols, sizeof(double));
    for (i = 1; i < rows; i++)
	m[i] = m[i-1] + cols;
    return m;
}


void G_free_vector(double *v)
{
    if(v!=NULL) free ((char *)v);
}


void G_free_matrix(double **m)
{
    if(m!=NULL) {
      free ((char *)(m[0]));
      free ((char *)m);
    }
}


int *G_alloc_ivector(int n)
{
    return (int *) G_calloc (n, sizeof(int));
}


int **G_alloc_imatrix(int rows,int cols)
{
    int **m;
    int i;

    m = (int **) G_calloc (rows, sizeof(int *));
    m[0] = (int *) G_calloc (rows*cols, sizeof(int));
    for (i = 1; i < rows; i++)
	m[i] = m[i-1] + cols;
    return m;
}


void G_free_ivector(int *v)
{
    if(v!=NULL) {
      free ((char *)v);
    }
}


void G_free_imatrix(int **m)
{
    if(m!=NULL) {
      free ((char *)(m[0]));
      free ((char *)m);
    }
}
