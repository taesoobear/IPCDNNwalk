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
#include "subcluster.h"
#include "alloc_util.h"
#include "clust_util.h"
//#include "../../utility/checkPoints.h"
//extern Stopwatch gStopwatch2;
int clusterMessageVerboseLevel;

static void seed(ClassSig *Sig, int nbands, double Rmin, int option);

static double refine_clusters(
    ClassSig *Sig, 
    int nbands, 
    double Rmin, 
    int option);

static void reestimate(ClassSig *Sig, int nbands, double Rmin, int option);

static double regroup(ClassSig *Sig, int nbands);

static void reduce_order(
    ClassSig *Sig,
    int nbands,
    int *min_ii,
    int *min_jj);

static double loglike(
    double *x, 
    SubSig *SubSig, 
    int nbands);

static double distance(
    SubSig *SubSig1,
    SubSig *SubSig2,
    int nbands);

static void compute_constants(ClassSig *Sig, int nbands);

static void normalize_pi(ClassSig *Sig);

static void add_SubSigs(
    SubSig *SubSig1,
    SubSig *SubSig2,
    SubSig *SubSig3,
    int nbands);

static void save_ClassSig(
    ClassSig *Sig1,
    SigSet *S,
    int nbands);

static void copy_ClassSig(
    ClassSig *Sig1,
    ClassSig *Sig2,
    int nbands);

static void copy_SubSig(
    SubSig *SubSig1,
    SubSig *SubSig2,
    int nbands);

static void DiagonalizeMatrix(double **R, int nbands);



int subcluster(
    SigSet *S, /* Input: structure contataining input data */ 
    int Class_Index,  /* Input: index corresponding to class to be processed */
    int desired_num,  /* Input: desired number of subclusters. */
                      /*      0=>ignore this input. */
    int option,       /* Input: type of clustering to use */
                      /*      option=1=CLUSTER_FULL=>full covariance matrix */
                      /*      option=0=CLUSTER_DIAG=>diagonal covariance matrix */
    double Rmin,      /* Minimum value for diagonal elements of convariance */
    int *Max_num)     /* Output: maximum number of allowed subclusters */
{
    int nparams_clust;
    int ndata_points;
    int min_i,min_j;
    int status;
    int nbands;
    double rissanen;
    double min_riss;
    struct ClassSig *Sig;
    static struct SigSet Smin;

    status = 0;

    /* set class pointer */
    Sig = &(S->classSig[Class_Index]);

    /* set number of bands */
    nbands = S->nbands;

    /* compute number of parameters per cluster */
    nparams_clust = 1+nbands+0.5*(nbands+1)*nbands;
    if(option==CLUSTER_DIAG) nparams_clust = 1+nbands+nbands;

    /* compute number of data points */
    ndata_points = Sig->classData.npixels*nbands;

    /* compute maximum number of subclasses */
    ndata_points = Sig->classData.npixels*nbands;
    *Max_num = (ndata_points+1)/nparams_clust - 1;

    /* check for too many subclasses */
    if(Sig->nsubclasses > (*Max_num/2) )
    {
      Sig->nsubclasses = *Max_num/2;
      fprintf(stderr,"Too many subclasses for class index %d\n",Class_Index);
      fprintf(stderr,"         number of subclasses set to %d\n\n",Sig->nsubclasses);
      status = -2;
    }

	
    /* initialize clustering */
    seed(Sig,nbands,Rmin,option);



	/* EM algorithm */
    min_riss = refine_clusters(Sig,nbands,Rmin,option);


    if(2<=clusterMessageVerboseLevel) {
      fprintf(stdout,"Subclasses = %d; Rissanen = %f; \n",Sig->nsubclasses,min_riss);
    }

    /* Save contents of Class Signature to Smin */
    save_ClassSig(Sig,&Smin,nbands);

    if(desired_num==0) {
      while(Sig->nsubclasses>1) {
        reduce_order(Sig,nbands,&min_i,&min_j);

        if(2<=clusterMessageVerboseLevel) {
          fprintf(stdout,"Combining Subclasses (%d,%d)\n",min_i,min_j);
        }

        rissanen = refine_clusters(Sig,nbands,Rmin,option);

        if(2<=clusterMessageVerboseLevel) {
          fprintf(stdout,"Subclasses = %d; Rissanen = %f; \n",
                       Sig->nsubclasses, rissanen);
        }

        if(rissanen<min_riss)
        {
          min_riss = rissanen;

          /* Delete old Smin, and save new Smin */
          I_DeallocSigSet(&Smin);
          save_ClassSig(Sig,&Smin,nbands);
        }
      }
    }
    else {
      while( (Sig->nsubclasses>desired_num)&&(Sig->nsubclasses>0) ) {
        reduce_order(Sig,nbands,&min_i,&min_j);

        if(2<=clusterMessageVerboseLevel) {
          fprintf(stdout,"Combining Subclasses (%d,%d)\n",min_i,min_j);
        }
 
        rissanen = refine_clusters(Sig,nbands,Rmin,option);

        if(2<=clusterMessageVerboseLevel) {
          fprintf(stdout,"Subclasses = %d; Rissanen = %f; \n",
                       Sig->nsubclasses,rissanen);
        }

        /* Delete old Smin, and save new Smin */
        I_DeallocSigSet(&Smin);
        save_ClassSig(Sig,&Smin,nbands);
      }  
    }

    /* Deallocate memory for class, and replace with solution */
    while(Sig->nsubclasses>0) I_DeallocSubSig(Sig);
    Sig->subSig = Smin.classSig[0].subSig;
    Sig->nsubclasses = Smin.classSig[0].nsubclasses;

    /* return warning status */
    return(status);
}


/******************************************************************/
/* Computes initial values for parameters of Gaussian Mixture     */
/* model. The subroutine returns the minimum allowed value for    */
/* the diagonal entries of the convariance matrix of each class.  */
/*****************************************************************/
static void seed(struct ClassSig *Sig, int nbands, double Rmin, int option)
{
     int     i,b1,b2;
     double  period;
     double  *mean,**R;

     /* Compute the mean of variance for each band */
     mean = G_alloc_vector(nbands);
     R = G_alloc_matrix(nbands,nbands);

     for(b1=0; b1<nbands; b1++) {
       mean[b1] = 0.0;
       for(i=0; i<Sig->classData.npixels; i++) {
         mean[b1] += (Sig->classData.x[i][b1])*(Sig->classData.w[i]);
       }
       mean[b1] /= Sig->classData.SummedWeights;
     }

     for(b1=0; b1<nbands; b1++) 
     for(b2=0; b2<nbands; b2++) {
       R[b1][b2] = 0.0;
       for(i=0; i<Sig->classData.npixels; i++) {
         R[b1][b2] += (Sig->classData.x[i][b1])*(Sig->classData.x[i][b2])*(Sig->classData.w[i]);
       }
       R[b1][b2] /= Sig->classData.SummedWeights;
       R[b1][b2] -= mean[b1]*mean[b2];
     }

     /* If diagonal clustering is desired, then diagonalize matrix */
     if(option==CLUSTER_DIAG) DiagonalizeMatrix(R,nbands);

     /* Compute the sampling period for seeding */
     if(Sig->nsubclasses>1) {
       period = (Sig->classData.npixels-1)/(Sig->nsubclasses-1.0);
     }
     else period =0;


     /* Seed the means and set the covarience components */
     for(i=0; i<Sig->nsubclasses; i++) {
       for(b1=0; b1<nbands; b1++) {
         Sig->subSig[i].means[b1] = Sig->classData.x[(int)(i*period)][b1];
       }

       for(b1=0; b1<nbands; b1++)
       for(b2=0; b2<nbands; b2++) {
         Sig->subSig[i].R[b1][b2] = R[b1][b2];
       }
       for(b1=0; b1<nbands; b1++) {
         Sig->subSig[i].R[b1][b1] += Rmin;
       }
       Sig->subSig[i].pi = 1.0/Sig->nsubclasses;
     }

     G_free_vector(mean);
     G_free_matrix(R);

     compute_constants(Sig,nbands);
     normalize_pi(Sig);
}


/*****************************************************************/
/* Computes ML clustering of data using Gaussian Mixture model.  */
/* Returns the values of the Rissen constant for the clustering. */
/*****************************************************************/
static double refine_clusters(
    struct ClassSig *Sig, 
    int nbands, 
    double Rmin, 
    int option)
{
     int nparams_clust;
     int num_params;
     int ndata_points;
     int repeat;
     double rissanen_const;
     double change,ll_new,ll_old;
     double epsilon;

     /* compute number of parameters per cluster */
     nparams_clust = 1+nbands+0.5*(nbands+1)*nbands;
     if(option==CLUSTER_DIAG) nparams_clust = 1+nbands+nbands;

     /* compute number of data points */
     ndata_points = Sig->classData.npixels*nbands;

     /* compute epsilon */
     epsilon = nparams_clust*log((double)ndata_points);
     epsilon *= 0.01;

     /* Perform initial regrouping */
     ll_new = regroup(Sig,nbands);

     /* Perform EM algorithm */
     change = 2*epsilon;
     do {
       ll_old = ll_new;
       reestimate(Sig,nbands,Rmin,option);

       ll_new = regroup(Sig,nbands);
       change = ll_new-ll_old;
       repeat = change>epsilon;
     } while(repeat);

     /* compute Rissanens expression */
     if(Sig->nsubclasses>0) {
       num_params = Sig->nsubclasses*nparams_clust - 1;
       rissanen_const = -ll_new + 0.5*num_params*log((double)ndata_points);
       return(rissanen_const);
     }
     else {
       return((double)0);
     }
}


static void reestimate(struct ClassSig *Sig, int nbands, double Rmin, int option)
{
     int i;
     int s;
     int b1,b2;
     double diff1,diff2;
     struct ClassData *Data;

     /* set data pointer */
     Data = &(Sig->classData);
     
     /* Compute N */
     for(i=0; i<Sig->nsubclasses; i++) 
     {
       Sig->subSig[i].N = 0;
       for(s=0; s<Data->npixels; s++)
         Sig->subSig[i].N += (Data->p[s][i])*(Data->w[s]);
       Sig->subSig[i].pi = Sig->subSig[i].N;
     }

     /* Compute means and variances for each subcluster */
     for(i=0; i<Sig->nsubclasses; i++) 
     {
       /* Compute mean */
       for(b1=0; b1<nbands; b1++) 
       {
         Sig->subSig[i].means[b1] = 0;
         for(s=0; s<Data->npixels; s++)
           Sig->subSig[i].means[b1] += Data->p[s][i]*Data->x[s][b1]*Data->w[s];
         Sig->subSig[i].means[b1] /= Sig->subSig[i].N;
       }
	
       /* Compute R */
       for(b1=0; b1<nbands; b1++) 
       for(b2=b1; b2<nbands; b2++)
       {
         Sig->subSig[i].R[b1][b2] = 0;
         for(s=0; s<Data->npixels; s++)
         {
           diff1 = Data->x[s][b1] - Sig->subSig[i].means[b1];
           diff2 = Data->x[s][b2] - Sig->subSig[i].means[b2];
           Sig->subSig[i].R[b1][b2] += Data->p[s][i]*diff1*diff2*Data->w[s];
         }
         Sig->subSig[i].R[b1][b2] /= Sig->subSig[i].N;
         Sig->subSig[i].R[b2][b1] = Sig->subSig[i].R[b1][b2];
       }

       /* Regularize matrix */
       for(b1=0; b1<nbands; b1++) {
         Sig->subSig[i].R[b1][b1] += Rmin;
       }

       if(option==CLUSTER_DIAG) DiagonalizeMatrix(Sig->subSig[i].R,nbands);
     }

     /* Normalize probabilities for subclusters */
     normalize_pi(Sig);

     /* Compute constants */
     compute_constants(Sig,nbands);
     normalize_pi(Sig);
}


static double regroup(struct ClassSig *Sig, int nbands)
{
   int s;
   int i;
   double tmp;
   double maxlike;
   double likelihood;
   double subsum;
   struct ClassData *Data;

   /* set data pointer */
   Data = &(Sig->classData);

   /* compute likelihoods */
   likelihood = 0;
   for(s=0; s<Data->npixels; s++)
   {

     for(i=0; i<Sig->nsubclasses; i++)
     {

		 tmp = loglike(Data->x[s],&(Sig->subSig[i]),nbands);

       Data->p[s][i] = tmp;
       if(i==0) maxlike = tmp;
       if(tmp>maxlike) maxlike = tmp;
     }

     subsum = 0;
     for(i=0; i<Sig->nsubclasses; i++)
     {
       tmp = exp( Data->p[s][i]-maxlike )*Sig->subSig[i].pi;
       subsum += tmp;
       Data->p[s][i] = tmp;
     }
     likelihood += log(subsum) + maxlike;

     for(i=0; i<Sig->nsubclasses; i++)
       Data->p[s][i] /= subsum;
   }

   return(likelihood);
}


static void reduce_order(
    struct ClassSig *Sig,
    int nbands,
    int *min_ii,
    int *min_jj)
{
    int i,j;
    int min_i,min_j;
    double dist;
    double min_dist;
    struct SubSig *SubSig1,*SubSig2;

    static int first=1;
    struct SigSet S;
    static struct ClassSig *Sig3;
    static struct SubSig *SubSig3;

    /* allocate scratch space first time subroutine is called */
    if(first)
    {
      I_InitSigSet (&S);
      I_SigSetNBands (&S, nbands);
      Sig3 = I_NewClassSig(&S);
      I_NewSubSig (&S, Sig3);
      SubSig3 = Sig3->subSig;
      first = 0;
    }

    if(Sig->nsubclasses>1)
    {
      /* find the closest subclasses */
      for(i=0; i<Sig->nsubclasses-1; i++)
      for(j=i+1; j<Sig->nsubclasses; j++)
      {
        dist = distance(&(Sig->subSig[i]),&(Sig->subSig[j]),nbands);
        if((i==0)&&(j==1))
        {
          min_dist = dist;
          min_i = i;
          min_j = j;
        }
        if(dist<min_dist)
        {
          min_dist = dist;
          min_i = i;
          min_j = j;
        }
      }

      /* Save result for output */
      *min_ii = min_i; *min_jj = min_j;

      /* Combine Subclasses */
      SubSig1 = &(Sig->subSig[min_i]);
      SubSig2 = &(Sig->subSig[min_j]);
      add_SubSigs(SubSig1,SubSig2,SubSig3,nbands);
      copy_SubSig(SubSig3,SubSig1,nbands);

      /* remove extra subclass */
      for(i=min_j; i<Sig->nsubclasses-1; i++)
        copy_SubSig(&(Sig->subSig[i+1]),&(Sig->subSig[i]),nbands);

      /* Remove last Subclass */
      /* (Sig->nsubclasses)--; */
      I_DeallocSubSig(Sig); 

      /* Rerun compute_constants */
      compute_constants(Sig,nbands);
      normalize_pi(Sig);
    }
}


static double loglike(double *x, struct SubSig *SubSig, int nbands)
{

    int b1,b2;
    double diff1,diff2;
    double sum;

    sum = 0;
    for(b1=0; b1<nbands; b1++) 
    for(b2=0; b2<nbands; b2++)
    {
      diff1 = x[b1]-SubSig->means[b1];
      diff2 = x[b2]-SubSig->means[b2];
      sum += diff1*diff2*SubSig->Rinv[b1][b2];
    }

    sum = -0.5*sum + SubSig->cnst;

    return(sum);
}


static double distance(
    struct SubSig *SubSig1,
    struct SubSig *SubSig2,
    int nbands)
{
    double dist;

    static int first=1;
    struct SigSet S;
    static struct ClassSig *Sig3;
    static struct SubSig *SubSig3;


    /* allocate scratch space first time subroutine is called */
    if(first)
    {
      I_InitSigSet (&S);
      I_SigSetNBands (&S, nbands);
      Sig3 = I_NewClassSig(&S);
      I_NewSubSig (&S, Sig3);
      SubSig3 = Sig3->subSig;
      first = 0;
    }

    /* form SubSig3 by adding SubSig1 and SubSig2 */
    add_SubSigs(SubSig1,SubSig2,SubSig3,nbands);

    /* compute constant for SubSig3 */
    compute_constants(Sig3,nbands);

    /* compute distance */
    dist = SubSig1->N*SubSig1->cnst + SubSig2->N*SubSig2->cnst
           - SubSig3->N*SubSig3->cnst;

    return(dist);
}


/**********************************************************/
/* invert matrix and compute Sig->subSig[i].cnst          */
/**********************************************************/
static void compute_constants(struct ClassSig *Sig, int nbands)
{
   int i;
   int b1,b2;
   double det_man;
   int    det_exp;

   static int first=1;
   static int *indx;
   static double **y;
   static double *col;


   /* allocate memory first time subroutine is called */
   if(first)
   {
     indx = G_alloc_ivector(nbands);
     y = G_alloc_matrix(nbands,nbands); 
     col = G_alloc_vector(nbands);
     first = 0;
   }

   /* invert matrix and compute constant for each subclass */
   for(i=0; i<Sig->nsubclasses; i++) {
     for(b1=0; b1<nbands; b1++)
     for(b2=0; b2<nbands; b2++)
       Sig->subSig[i].Rinv[b1][b2] = Sig->subSig[i].R[b1][b2];

     clust_invert(Sig->subSig[i].Rinv,nbands,&det_man,&det_exp,indx,y,col);

     Sig->subSig[i].cnst = (-nbands/2.0)*log(2*PI) 
                            - 0.5*log(det_man) - 0.5*det_exp*log(10.0);
   } 
}


static void normalize_pi(struct ClassSig *Sig)
{
    int i;
    double sum;

    sum = 0.0;
    for(i=0; i<Sig->nsubclasses; i++) sum += Sig->subSig[i].pi;

    if(sum>0) {
      for(i=0; i<Sig->nsubclasses; i++) Sig->subSig[i].pi /= sum;
    }
    else {
      for(i=0; i<Sig->nsubclasses; i++) Sig->subSig[i].pi = 0.0;
    }
}


/*******************************************/
/* add SubSig1 and SubSig2 to form SubSig3 */
/*******************************************/
static void add_SubSigs(
    struct SubSig *SubSig1,
    struct SubSig *SubSig2,
    struct SubSig *SubSig3,
    int nbands)
{
    int b1,b2;
    double wt1,wt2;
    double tmp;

    wt1 = SubSig1->N/(SubSig1->N + SubSig2->N);
    wt2 = 1 - wt1;

    /* compute means */
    for(b1=0; b1<nbands; b1++)
      SubSig3->means[b1] = wt1*SubSig1->means[b1] + wt2*SubSig2->means[b1];

    /* compute covariance */
    for(b1=0; b1<nbands; b1++)
    for(b2=b1; b2<nbands; b2++)
    {
      tmp = (SubSig3->means[b1]-SubSig1->means[b1])
            *(SubSig3->means[b2]-SubSig1->means[b2]);
      SubSig3->R[b1][b2] = wt1*(SubSig1->R[b1][b2] + tmp);
      tmp = (SubSig3->means[b1]-SubSig2->means[b1])
            *(SubSig3->means[b2]-SubSig2->means[b2]);
      SubSig3->R[b1][b2] += wt2*(SubSig2->R[b1][b2] + tmp);
      SubSig3->R[b2][b1] = SubSig3->R[b1][b2];
    }

    /* compute pi and N */
    SubSig3->pi = SubSig1->pi + SubSig2->pi;
    SubSig3->N = SubSig1->N + SubSig2->N;
}


/**********************/
/* saves Sig1 to Sig2 */
/**********************/
static void save_ClassSig(
    struct ClassSig *Sig1,
    struct SigSet *S,
    int nbands)
{
    struct ClassSig *Sig2;

    I_InitSigSet (S);
    I_SigSetNBands (S, nbands);
    Sig2 = I_NewClassSig(S);
    while(Sig2->nsubclasses<Sig1->nsubclasses) I_NewSubSig(S, Sig2);
    copy_ClassSig(Sig1,Sig2,nbands);
}


/*********************/
/* copy Sig1 to Sig2 */
/*********************/
static void copy_ClassSig(
    struct ClassSig *Sig1,
    struct ClassSig *Sig2,
    int nbands)
{
    int i;

    Sig2->classnum = Sig1->classnum;
    Sig2->title = Sig1->title;
    Sig2->used = Sig1->used;
    Sig2->type = Sig1->type;
    Sig2->nsubclasses = Sig1->nsubclasses;
    for(i=0; i<Sig1->nsubclasses; i++)
      copy_SubSig(&(Sig1->subSig[i]),&(Sig2->subSig[i]),nbands);
}


/***************************/
/* copy SubSig1 to SubSig2 */
/***************************/
static void copy_SubSig(
    struct SubSig *SubSig1,
    struct SubSig *SubSig2,
    int nbands)
{
    int b1,b2;

    SubSig2->N = SubSig1->N;
    SubSig2->pi = SubSig1->pi;
    SubSig2->cnst = SubSig1->cnst;
    SubSig2->used = SubSig1->used;

    for(b1=0; b1<nbands; b1++)
      SubSig2->means[b1] = SubSig1->means[b1];

    for(b1=0; b1<nbands; b1++)
    for(b2=0; b2<nbands; b2++)
    {
      SubSig2->R[b1][b2] = SubSig1->R[b1][b2];
      SubSig2->Rinv[b1][b2] = SubSig1->Rinv[b1][b2];
    }
}

static void DiagonalizeMatrix(double **R, int nbands)
{
    int b1,b2;

    for(b1=0; b1<nbands; b1++)
    for(b2=0; b2<nbands; b2++)
      if(b1!=b2) R[b1][b2] = 0;
}

#if 0
static void list_Sig(struct ClassSig *Sig, int nbands)
{
    int i,j,k;

    for(i=0; i<Sig->nsubclasses; i++)
    {
      print

f("Subclass %d: pi = %f, ",i,Sig->subSig[i].pi);
      printf("cnst = %f\n",Sig->subSig[i].cnst);
      for(j=0; j<nbands; j++)
      {
        printf("%f;    ",Sig->subSig[i].means[j]);
        for(k=0; k<nbands; k++)
          printf("%f ",Sig->subSig[i].R[j][k]);
        printf("\n");
      }
      printf("\n");
    }
}

static void print_class(struct ClassSig *Sig, char *fname)
{
    FILE *fp;
    int s,i;


    if((fp=fopen(fname,"w"))==NULL) {
      fprintf(stderr,"can't open data file\n"); exit(1);}

    for(s=0; s<Sig->classData.npixels; s++)
    {
      /* fprintf(fp,"Pixel number %d:  ", s); */
      for(i=0; i<Sig->nsubclasses; i++)
        fprintf(fp,"%f  ",Sig->classData.p[s][i]);
      fprintf(fp,"\n");
    }

    fclose(fp);

}
#endif
	
