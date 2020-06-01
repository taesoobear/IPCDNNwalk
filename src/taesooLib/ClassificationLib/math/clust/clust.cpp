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
#include "clust_util.h"
#include "subcluster.h"
#include "clust.h"
#include "../../../BaseLib/math/Operator.h"
#include "../../../BaseLib/math/Operator_NR.h"
//#include "../../utility/checkPoints.h"
double AverageVariance(struct ClassSig *Sig, int nbands);


void readData(const char* fname, matrixn & out)
{
	FILE* fp;
      /* Read Data */
      if((fp=fopen(fname,"r"))==NULL) {
        fprintf(stderr,"can't open data file %s", fname);
        exit(1);
      }

      for(int i=0; i<out.rows(); i++) {
		  for(int j=0; j<out.cols(); j++) {
          fscanf(fp,"%lf",&out[i][j]);
        }
        fscanf(fp,"\n");
      }
      fclose(fp);

}

//Stopwatch gStopwatch2;
void demo1()
{

	//////////////////////////////////
	// Full covariance
	//////////////////////////////////
	// original code
  //	gStopwatch2.start();
	TStrings argv1;
	argv1.setStrings(4, "clust", "20", "../../BaseLib/math/clust/example1/info_file", "../../BaseLib/math/clust/example1/params");
	clust(4, argv1);

	//	gStopwatch2.end("1");

	// simplified code
	//	gStopwatch2.start();
	matrixn samples(500,2);
	readData("../../BaseLib/math/clust/example1/data", samples);
	Clust::ClassSig clustSimplified(20, samples, "full", 0, "../../BaseLib/math/clust/example1/paramsSimplified");

	//	gStopwatch2.end("2");


	//////////////////////////////////
	// Diagonal covariance
	//////////////////////////////////
	//	gStopwatch2.start();
	TStrings argv2;
	argv2.setStrings(5, "clust", "20", "../../BaseLib/math/clust/example1/info_file", "../../BaseLib/math/clust/example1/params_diag", "diag");
	clust(5, argv2);
	//	gStopwatch2.end("3");

	// simplified code
	//	gStopwatch2.start();
	Clust::ClassSig clustSimplifiedSphr(20, samples, "diag", 0, "../../BaseLib/math/clust/example1/paramsSimplifiedDiag");

	//	gStopwatch2.end("2");

	//////////////////////////////////
	// Spherical covariance
	//////////////////////////////////

	Clust::ClassSig clustSimplifiedDiag(20, samples, "sphr", 0, "../../BaseLib/math/clust/example1/paramsSimplifiedSphr");


	/*
	TStrings argv3;
	argv3.setStrings(6, "clust", "20", "../../BaseLib/math/clust/example1/info_file", "../../BaseLib/math/clust/example1/params_full5", "full", "5");
	clust(6, argv3);

	TStrings argv4;
	argv4.setStrings(6, "clust", "20", "../../BaseLib/math/clust/example1/info_file", "../../BaseLib/math/clust/example1/params_diag5", "diag", "5");
	clust(6, argv4);
*/
}

int clust(int  argc,TStrings const& argv)
{
    FILE *fp,*info_fp;

    int i,j,k;
    int init_num_of_subclasses,max_num;
    int vector_dimension;
    int nclasses;
    int num_of_samples;
    char option1[16];
    int  option2;
    double Rmin;
    char fname[512];

    /* set level of diagnostic printing */
    clusterMessageVerboseLevel = 2;

    /* print usage message if arguments are not valid */
    if((argc!=4) && (argc!=5) && (argc!=6)) {
		fprintf(stderr,"\n\nUsage: %s #_subclasses info_file output_params [option1 option2]\n\n",argv[0].ptr() );
      fprintf(stderr,"    #_subclasses - initial number of clusters for each class\n\n");
      fprintf(stderr,"    info_file - name of file which contains the following information:\n");
      fprintf(stderr,"      <# of classes>\n");
      fprintf(stderr,"      <data vector length>\n");
      fprintf(stderr,"      <class 1 data file name> <# of data vectors in class 1>\n");
      fprintf(stderr,"      <class 2 data file name> <# of data vectors in class 2>\n");
      fprintf(stderr,"                     .                        .\n");
      fprintf(stderr,"                     .                        .\n");
      fprintf(stderr,"                     .                        .\n");
      fprintf(stderr,"      <last class data file name> <# of data vectors in last class>\n\n");
      fprintf(stderr,"    output_params - name of file containing output clustering");
      fprintf(stderr," parameters\n\n");
      fprintf(stderr,"    option1 - (optional) controls clustering model\n");
      fprintf(stderr,"      full - (default) use full convariance matrices\n");
      fprintf(stderr,"      diag - use diagonal convariance matrices\n\n");
      fprintf(stderr,"    option2 - (optional) controls number of clusters\n");
      fprintf(stderr,"      0 - (default) estimate number of clusters\n");
      fprintf(stderr,"      n - use n clusters in mixture model with n<#_subclasses");
      exit(1);
    }

    /* read number of initial subclasses to use */
    sscanf(argv[1],"%d",&init_num_of_subclasses);

/* Set option 1 */
    if(argc==4) {
      sprintf(option1,"full") ;
    }

    if((argc==5) || (argc==6)) {
      /* set default option 1 */
      sscanf(argv[4], "%s", option1) ;
      if((strcmp(option1, "full")!=0) && (strcmp(option1, "diag")!=0)) {
        fprintf(stderr,"\nInvalid option1: %s\n\n",option1);
        fprintf(stderr,"There are 2 valid assumptions:\n");
        fprintf(stderr,"    full - default option which allows full convariance matrices\n");
        fprintf(stderr,"    diag - use diagonal convariance matrices\n\n");
        exit(1);
      }
    }

/* Set option 2 */
    if((argc==4) || (argc==5)) {
      option2 = 0;
    }

    if(argc==6) {
      /* set default option 2 */
      sscanf(argv[5], "%d", &option2) ;
      if( (option2<0) || (option2>init_num_of_subclasses) ) {
        fprintf(stderr,"\nInvalid option2: %d \n\n",option2);
        fprintf(stderr,"There are 2 valid assumptions:\n");
        fprintf(stderr,"      0 - (default) estimate number of clusters\n");
        fprintf(stderr,"      n - use n clusters in mixture model with n<#_subclasses\n\n");
        exit(1);
      }
    }


    /* open information file */
    if((info_fp=fopen(argv[2],"r"))==NULL) {
      fprintf(stderr,"can't open information file");
      exit(1);
    }

    /* read number of classes from info file */
    fscanf(info_fp,"%d\n",&nclasses);

    /* read vector dimension from info file */
    fscanf(info_fp,"%d\n",&vector_dimension);



    struct SigSet S;
    struct ClassSig *Sig;


    /* Initialize SigSet data structure */
    I_InitSigSet (&S);
    I_SigSetNBands (&S, vector_dimension);
    I_SetSigTitle (&S, "test signature set");


    /* Allocate memory for cluster signatures */
    for(k=0; k<nclasses; k++) {
      Sig = I_NewClassSig(&S);
      I_SetClassTitle (Sig, "test class signature");
      for(i=0; i<init_num_of_subclasses; i++)
        I_NewSubSig (&S, Sig);
    }

    /* Read data for each class */
    for(k=0; k<nclasses; k++) {


	  /* read class k data file name */
	  fscanf(info_fp,"%s",fname);

	  /* read number of samples for class k */
	  fscanf(info_fp,"%d\n",&num_of_samples);

      Sig = &(S.classSig[k]);

      I_AllocClassData (&S, Sig, num_of_samples);

      /* Read Data */
      if((fp=fopen(fname,"r"))==NULL) {
        fprintf(stderr,"can't open data file %s", fname);
        exit(1);
      }

      for(i=0; i<Sig->classData.npixels; i++) {
        for(j=0; j<vector_dimension; j++) {
          fscanf(fp,"%lf",&(Sig->classData.x[i][j]) );
        }
        fscanf(fp,"\n");
      }
      fclose(fp);

      /* Set unity weights and compute SummedWeights */
      Sig->classData.SummedWeights = 0.0;
      for(i=0; i<Sig->classData.npixels; i++) {
        Sig->classData.w[i] = 1.0;
        Sig->classData.SummedWeights += Sig->classData.w[i];
      }
    }
    fclose(info_fp);


    /* Compute the average variance over all classes */
    Rmin = 0;
    for(k=0; k<nclasses; k++) {
      Sig = &(S.classSig[k]);
      Rmin += AverageVariance(Sig, vector_dimension);
    }
    Rmin = Rmin/(COVAR_DYNAMIC_RANGE*nclasses);

    /* Perform clustering for each class */
    for(k=0; k<nclasses; k++) {

      Sig = &(S.classSig[k]);

      if(1<=clusterMessageVerboseLevel) {
        fprintf(stdout,"Start clustering class %d\n\n",k);
      }


      if(strcmp(option1, "diag")==0) {
        /* assume covariance matrices to be diagonal */
        subcluster(&S,k,option2,(int)CLUSTER_DIAG,Rmin,&max_num);

      } else {
        /* no assumption for covariance matrices */
        subcluster(&S,k,option2,(int)CLUSTER_FULL,Rmin,&max_num);
      }


      if(2<=clusterMessageVerboseLevel) {
        fprintf(stdout,"Maximum number of subclasses = %d\n",max_num);
      }

      I_DeallocClassData(&S, Sig);
    }

    /* Write out result to output parameter file */
    if((fp=fopen(argv[3],"w"))==NULL) {
      fprintf(stderr,"can't open parameter output file");
      exit(1);
    }
    I_WriteSigSet(fp, &S);
    fclose(fp);

    /* De-allocate cluster signature memory */
    I_DeallocSigSet(&S);

	return 0;

}

Clust::ClassSig::ClassSig(int init_num_of_subclasses, matrixn const& samples, const char* option1, int  option2, const char* outfile)
{
	int i,j,k=0;
    int max_num;
    int nclasses=1;

    double Rmin;

	int num_of_samples=samples.rows();
	int vector_dimension=samples.cols();

    /* set level of diagnostic printing */
    clusterMessageVerboseLevel = 2;


	Clust::ClassSig &Sig=*this;

	int eCovMat;


	if(strcmp(option1, "diag")==0)
		eCovMat=CLUSTER_DIAG;
	else if(strcmp(option1, "full")==0)
		eCovMat=CLUSTER_FULL;
	else
		eCovMat=CLUSTER_SPHR;

	switch(eCovMat)
	{
	case CLUSTER_FULL:
		{
			mSubSig.changeFactory(new TDefaultFactoryDerived <Clust::SubSig, Clust::SubSigFull>());
			mpSmin=new Clust::ClassSig::SubSigs ();
			mpSmin->changeFactory(new TDefaultFactoryDerived <Clust::SubSig, Clust::SubSigFull>());
			mpSubSig3=new Clust::SubSigFull();
		}
		break;
	case CLUSTER_DIAG:
		{
			mSubSig.changeFactory(new TDefaultFactoryDerived <Clust::SubSig, Clust::SubSigDiag>());
			mpSmin=new Clust::ClassSig::SubSigs ();
			mpSmin->changeFactory(new TDefaultFactoryDerived <Clust::SubSig, Clust::SubSigDiag>());
			mpSubSig3=new Clust::SubSigDiag();
		}
		break;
	case CLUSTER_SPHR:
		{
			mSubSig.changeFactory(new TDefaultFactoryDerived <Clust::SubSig, Clust::SubSigSphr>());
			mpSmin=new Clust::ClassSig::SubSigs ();
			mpSmin->changeFactory(new TDefaultFactoryDerived <Clust::SubSig, Clust::SubSigSphr>());
			mpSubSig3=new Clust::SubSigSphr();
		}
		break;
	}

	Sig.init(vector_dimension, "test class signature");
	for(i=0; i<init_num_of_subclasses; i++)
		Sig.mSubSig.insertNewSubSig();

	Sig.allocClassData(samples);


    /* Compute the average variance over all classes */
    Rmin = 0;
    Rmin += Sig.averageVariance();
    Rmin = Rmin/(COVAR_DYNAMIC_RANGE);

    /* Perform clustering */
	if(1<=clusterMessageVerboseLevel) {
	fprintf(stdout,"Start clustering class %d\n\n",k);
	}

	Sig.subcluster(option2,eCovMat,Rmin,&max_num);

	if(2<=clusterMessageVerboseLevel) {
	fprintf(stdout,"Maximum number of subclasses = %d\n",max_num);
	}


	/* Write out result to output parameter file */
	if(outfile)
	{
		FILE* fp;
		if((fp=fopen(outfile,"w"))==NULL) {
		  fprintf(stderr,"can't open parameter output file");
		  exit(1);
		}
		Sig.writeToFile(fp);
		fclose(fp);
	}
}

Clust::ClassSig::ClassSig()
{
	used = 1;
	type = SIGNATURE_TYPE_MIXED;
	mSubSig.nbands=0;
	classData.SummedWeights = 0.0;
}

void Clust::ClassSig::init(int _nbands, const char* _title)
{
	title=_title;
    used = 1;
  type = SIGNATURE_TYPE_MIXED;
  mSubSig.nbands=_nbands;
  classData.SummedWeights = 0.0;
}

Clust::ClassSig::~ClassSig()
{
}


void Clust::SubSig::init(int nbands)
{
	SubSig *Sp;
	Sp=this;
	Sp->used = 1;
	Sp->means.setSize(nbands);
	Sp->N = 0;
	Sp->pi = 0;
	Sp->cnst = 0;
}


void Clust::SubSigFull::init(int nbands)
{
	SubSig::init(nbands);
	R .setSize(nbands, nbands);
	Rinv.setSize(nbands, nbands);
}

void Clust::SubSigDiag::init(int nbands)
{
	SubSig::init(nbands);
	diagR .setSize(nbands);
	diagRinv.setSize(nbands);
}

void Clust::SubSigSphr::init(int nbands)
{
	SubSig::init(nbands);
}

void Clust::SubSigFull::writeToFile(FILE *fd)
{
	int b1, b2;
	SubSigFull* Sp=this;
	      fprintf (fd, " subclass:\n");
      fprintf (fd, "  pi: %g\n", Sp->pi);
      fprintf (fd, "  means:");
      for (b1 = 0; b1 < nbands(); b1++)
		fprintf (fd, " %g", Sp->means[b1]);
      fprintf (fd, "\n");
      fprintf (fd, "  covar:\n");
      for (b1 = 0; b1 < nbands(); b1++) {
	fprintf (fd, "   ");
	for (b2 = 0; b2 < nbands(); b2++)
	  fprintf (fd, " %g", ((SubSigFull*) Sp)->R[b1][b2]);
	fprintf (fd, "\n");
      }
      fprintf (fd, " endsubclass:\n");

}

void Clust::SubSigDiag::writeToFile(FILE *fd)
{
	int b1, b2;
	SubSigDiag* Sp=this;
	      fprintf (fd, " subclass:\n");
      fprintf (fd, "  pi: %g\n", Sp->pi);
      fprintf (fd, "  means:");
      for (b1 = 0; b1 < nbands(); b1++)
		fprintf (fd, " %g", Sp->means[b1]);
      fprintf (fd, "\n");
      fprintf (fd, "  covar:\n");
      for (b1 = 0; b1 < nbands(); b1++) {
	fprintf (fd, "   ");
	for (b2 = 0; b2 < nbands(); b2++)
		if(b1==b2)
			fprintf (fd, " %g", ((SubSigDiag*) Sp)->diagR[b1]);
		else
			fprintf (fd, " 0");
	fprintf (fd, "\n");
      }
      fprintf (fd, " endsubclass:\n");

}

void Clust::SubSigSphr::writeToFile(FILE *fd)
{
	int b1, b2;
	SubSigSphr* Sp=this;
	      fprintf (fd, " subclass:\n");
      fprintf (fd, "  pi: %g\n", Sp->pi);
      fprintf (fd, "  means:");
      for (b1 = 0; b1 < nbands(); b1++)
		fprintf (fd, " %g", Sp->means[b1]);
      fprintf (fd, "\n");
      fprintf (fd, "  covar:\n");
      for (b1 = 0; b1 < nbands(); b1++) {
	fprintf (fd, "   ");
	for (b2 = 0; b2 < nbands(); b2++)
		if(b1==b2)
			fprintf (fd, " %g", ((SubSigSphr*) Sp)->var);
		else
			fprintf (fd, " 0");
	fprintf (fd, "\n");
      }
      fprintf (fd, " endsubclass:\n");

}


void Clust::SubSigFull ::setCov(matrixn const& R, double Rmin)
{
	this->R=R;
	for(int b1=0; b1<nbands(); b1++)
	{
         this->R[b1][b1] += Rmin;
    }
}

void Clust::SubSigDiag ::setCov(matrixn const& R, double Rmin)
{
	for(int b1=0; b1<nbands(); b1++)
	{
		this->diagR[b1]=R[b1][b1];
        this->diagR[b1] += Rmin;
    }
}

void Clust::SubSigSphr::setCov(matrixn const& R, double Rmin)
{
	var=R[0][0];
	var+=Rmin;
}

void Clust::SubSigFull::compute_constants()
{
   m_real det_man;
   int    det_exp;

   m::LUinvert(Rinv, R, det_man, det_exp);
   cnst = (-1*nbands()/2.0)*log(2*PI) - 0.5*log(det_man) - 0.5*det_exp*log(10.0);
}

void Clust::SubSigDiag::compute_constants()
{
	m_real log_det;
	m::Diaginvert(diagRinv, diagR, log_det);
	cnst = (-1*nbands()/2.0)*log(2*PI) - 0.5*log_det;
}

void Clust::SubSigSphr::compute_constants()
{
	m_real log_det=nbands()*log(var);
	cnst = (-1*nbands()/2.0)*log(2*PI) - 0.5*log_det;
}


double Clust::SubSigFull::loglike(vectorn const& x)
{
	return -0.5*m::sMs(x, means, Rinv)+cnst;
}

double Clust::SubSigDiag::loglike(vectorn const& x)
{
	return -0.5*m::sDs(x, means, diagRinv)+cnst;
}

double Clust::SubSigSphr::loglike(vectorn const& x)
{

	return -0.5*m::ss(x, means)/var+cnst;
}

Clust::SubSigFull& Clust::SubSigFull::operator=(const SubSig& other)
{
	N=other.N;
	pi=other.pi;
	means=other.means;
	cnst=other.cnst;
	used=other.used;
	R=((SubSigFull&)other).R;     /* convarance of component in GMM */
	Rinv=((SubSigFull&)other).Rinv;     /* convarance of component in GMM */;  /* inverse of R */

	return *this;
}

Clust::SubSigDiag& Clust::SubSigDiag::operator=(const SubSig& other)
{
	N=other.N;
	pi=other.pi;
	means=other.means;
	cnst=other.cnst;
	used=other.used;
	diagR=((SubSigDiag&)other).diagR;     /* convarance of component in GMM */
	diagRinv=((SubSigDiag&)other).diagRinv;     /* convarance of component in GMM */;  /* inverse of R */

	return *this;
}

Clust::SubSigSphr& Clust::SubSigSphr::operator=(const SubSig& other)
{
	N=other.N;
	pi=other.pi;
	means=other.means;
	cnst=other.cnst;
	used=other.used;
	var=((SubSigSphr&)other).var;     /* convarance of component in GMM */

	return *this;
}

void Clust::SubSigFull::estimateCov(ClassData* Data, int i, double Rmin)
{
	int b1,b2;
     double diff1,diff2;


   for(b1=0; b1<nbands(); b1++)
   for(b2=b1; b2<nbands(); b2++)
   {
     R[b1][b2] = 0;
     for(int s=0; s<Data->npixels(); s++)
     {
       diff1 = Data->x[s][b1] - means[b1];
       diff2 = Data->x[s][b2] - means[b2];
       R[b1][b2] += Data->p[s][i]*diff1*diff2*Data->w[s];
     }
     R[b1][b2] /= N;
     R[b2][b1] = R[b1][b2];
   }

      /* Regularize matrix */
   for(b1=0; b1<nbands(); b1++) {
     R[b1][b1] += Rmin;
   }

}

// M-step에서 covariance업데이트.
void Clust::SubSigDiag::estimateCov(ClassData* Data, int i, double Rmin)
{
	int b1;
    double diff1,diff2;

	// diagR[i]=sum_n Pk_n(x_in-mu_ik)^2/ N
	// where Pk_n=Data->p[s][i]*Data->w[s],
	//	, and N=sum_n Pk_n

	for(b1=0; b1<nbands(); b1++)
	{
		diagR[b1] = 0;
		for(int s=0; s<Data->npixels(); s++)
		{
		diff1 = Data->x[s][b1] - means[b1];
		diagR[b1] += Data->p[s][i]*diff1*diff1*Data->w[s];
		}
		diagR[b1] /= N;
   }

	/* Regularize matrix */
	for(b1=0; b1<nbands(); b1++) {
		 diagR[b1] += Rmin;
	}

}

void Clust::SubSigSphr::estimateCov(ClassData* Data, int i, double Rmin)
{
    double diff1;

	// diagR[i]=sum_n Pk_n ||x_n-mu_k||^2/ N
	// where Pk_n=Data->p[s][i]*Data->w[s],
	//	, and N=sum_n Pk_n



	var=0;

	for(int s=0; s<Data->npixels(); s++)
	{
		diff1=m::ss(Data->x.row(s),means);
		var+=Data->p[s][i]*diff1*Data->w[s];
	}
	var/=(N*nbands());

	/* Regularize variance*/
	var+=Rmin;
}

void Clust::SubSigFull::add(SubSig const& SubSig1,SubSig const& SubSig2)
{
	int b1,b2;
    double wt1,wt2;
    double tmp;

    wt1 = SubSig1.N/(SubSig1.N + SubSig2.N);
    wt2 = 1 - wt1;

    /* compute means */
    for(b1=0; b1<nbands(); b1++)
      means[b1] = wt1*SubSig1.means[b1] + wt2*SubSig2.means[b1];

    /* compute covariance */
    for(b1=0; b1<nbands(); b1++)
    for(b2=b1; b2<nbands(); b2++)
    {
      tmp = (means[b1]-SubSig1.means[b1])
            *(means[b2]-SubSig1.means[b2]);
      R[b1][b2] = wt1*(((SubSigFull&)SubSig1).R[b1][b2] + tmp);
      tmp = (means[b1]-SubSig2.means[b1])
            *(means[b2]-SubSig2.means[b2]);
      R[b1][b2] += wt2*(((SubSigFull&)SubSig2).R[b1][b2] + tmp);
      R[b2][b1] = R[b1][b2];
    }

    /* compute pi and N */
    pi = SubSig1.pi + SubSig2.pi;
    N = SubSig1.N + SubSig2.N;

}

void Clust::SubSigDiag::add(SubSig const& SubSig1,SubSig const& SubSig2)
{
	int b1;
    double wt1,wt2;
    double tmp;

    wt1 = SubSig1.N/(SubSig1.N + SubSig2.N);
    wt2 = 1 - wt1;

    /* compute means */
    for(b1=0; b1<nbands(); b1++)
      means[b1] = wt1*SubSig1.means[b1] + wt2*SubSig2.means[b1];

    /* compute covariance */
    for(b1=0; b1<nbands(); b1++)
    {
      tmp = (means[b1]-SubSig1.means[b1])*(means[b1]-SubSig1.means[b1]);
      diagR[b1] = wt1*(((SubSigDiag&)SubSig1).diagR[b1]  + tmp);
      tmp = (means[b1]-SubSig2.means[b1])*(means[b1]-SubSig2.means[b1]);
      diagR[b1]  += wt2*(((SubSigDiag&)SubSig2).diagR[b1]  + tmp);

//	  printf("Diag add %f %f -> %f\n", ((SubSigDiag&)SubSig1).diagR[b1]  , ((SubSigDiag&)SubSig2).diagR[b1], diagR[b1]);
    }

    /* compute pi and N */
    pi = SubSig1.pi + SubSig2.pi;
    N = SubSig1.N + SubSig2.N;

}


void Clust::SubSigSphr::add(SubSig const& SubSig1,SubSig const& SubSig2)
{
	int b1;
    double wt1,wt2;
    double tmp;

    wt1 = SubSig1.N/(SubSig1.N + SubSig2.N);
    wt2 = 1 - wt1;

    /* compute means */
    for(b1=0; b1<nbands(); b1++)
      means[b1] = wt1*SubSig1.means[b1] + wt2*SubSig2.means[b1];


	/* compute covariance */
	tmp = m::ss(means, SubSig1.means);
	var = wt1*(((SubSigSphr&)SubSig1).var  + tmp);
	tmp = m::ss(means, SubSig2.means);
	var += wt2*(((SubSigSphr&)SubSig2).var  + tmp);

	//printf("Sphr add %f %f -> %f\n", ((SubSigSphr&)SubSig1).var  , ((SubSigSphr&)SubSig2).var, var);

    /* compute pi and N */
    pi = SubSig1.pi + SubSig2.pi;
    N = SubSig1.N + SubSig2.N;

}


double Clust::SubSigFull::distance(SubSig const& SubSig2)
{
   Clust::SubSig &SubSig1=*this;
    double dist;

    static int first=1;
	static Clust::SubSigFull SubSig3;

	SubSig3.init(nbands());

    /* form SubSig3 by adding SubSig1 and SubSig2 */
    SubSig3.add(SubSig1,SubSig2);

    /* compute constant for SubSig3 */
	SubSig3.compute_constants();

    /* compute distance */
    dist = SubSig1.N*cnst + SubSig2.N*SubSig2.cnst - SubSig3.N*SubSig3.cnst;

    return(dist);
}


double Clust::SubSigDiag::distance(SubSig const& SubSig2)
{
   Clust::SubSig &SubSig1=*this;
    double dist;

    static int first=1;
	static Clust::SubSigDiag SubSig3;

	SubSig3.init(nbands());

    /* form SubSig3 by adding SubSig1 and SubSig2 */
    SubSig3.add(SubSig1,SubSig2);

    /* compute constant for SubSig3 */
	SubSig3.compute_constants();

    /* compute distance */
    dist = SubSig1.N*cnst + SubSig2.N*SubSig2.cnst - SubSig3.N*SubSig3.cnst;

    return(dist);
}

double Clust::SubSigSphr::distance(SubSig const& SubSig2)
{
   Clust::SubSig &SubSig1=*this;
    double dist;

    static int first=1;
	static Clust::SubSigSphr SubSig3;

	SubSig3.init(nbands());

    /* form SubSig3 by adding SubSig1 and SubSig2 */
    SubSig3.add(SubSig1,SubSig2);

    /* compute constant for SubSig3 */
	SubSig3.compute_constants();

    /* compute distance */
    dist = SubSig1.N*cnst + SubSig2.N*SubSig2.cnst - SubSig3.N*SubSig3.cnst;

    return(dist);
}


void Clust::ClassSig::SubSigs::insertNewSubSig()
{
  resize(size()+1);
  data(size()-1).init(nbands);
}

void Clust::ClassSig::allocClassData(matrixn const& samples)
{
	ClassData *Data;

	Data = &(classData);

	int npixels=samples.rows();
	Data->x =samples;
	Data->p.setSize(npixels, nsubclasses());
	/* Set unity weights and compute SummedWeights */
	Data->w.setAllValue(npixels,1.0);
	classData.SummedWeights = Data->w.sum();
}

double Clust::ClassSig::averageVariance() const
{
	const Clust::ClassSig *Sig=this;
     int     i,b1;

	 static vectorn mean;
	 static matrixn R;
	 double Rmin;

     /* Compute the mean of variance for each band */
	 mean .setSize(nbands());
	 R .setSize(nbands(),nbands());

     for(b1=0; b1<nbands(); b1++) {
       mean[b1] = 0.0;
       for(i=0; i<Sig->classData.npixels(); i++) {
         mean[b1] += (Sig->classData.x[i][b1])*(Sig->classData.w[i]);
       }
       mean[b1] /= Sig->classData.SummedWeights;
     }

     for(b1=0; b1<nbands(); b1++) {
       R[b1][b1] = 0.0;
       for(i=0; i<Sig->classData.npixels(); i++) {
         R[b1][b1] += (Sig->classData.x[i][b1])*(Sig->classData.x[i][b1])*(Sig->classData.w[i]);
       }
       R[b1][b1] /= Sig->classData.SummedWeights;
       R[b1][b1] -= mean[b1]*mean[b1];
     }

     /* Compute average of diagonal entries */
     Rmin = 0.0;
     for(b1=0; b1<nbands(); b1++)
       Rmin += R[b1][b1];

     Rmin = Rmin/(nbands());


     return(Rmin);
}

/******************************************************************/
/* Computes initial values for parameters of Gaussian Mixture     */
/* model. The subroutine returns the minimum allowed value for    */
/* the diagonal entries of the convariance matrix of each class.  */
/*****************************************************************/
void Clust::ClassSig::seed(double Rmin, int option)
{
	struct ClassSig *Sig=this;
     int     i,b1,b2;
     double  period;

 	 static vectorn mean;
	 static matrixn R;
	 R.setSize(nbands(), nbands());

     /* Compute the mean of variance for each band */
	 mean .setSize(nbands());

     for(b1=0; b1<nbands(); b1++) {
       mean[b1] = 0.0;
       for(i=0; i<Sig->classData.npixels(); i++) {
         mean[b1] += (Sig->classData.x[i][b1])*(Sig->classData.w[i]);
       }
       mean[b1] /= Sig->classData.SummedWeights;
     }

     if(option==CLUSTER_FULL)
	 {
		 for(b1=0; b1<nbands(); b1++)
		 for(b2=0; b2<nbands(); b2++) {
		   R[b1][b2] = 0.0;
		   for(i=0; i<Sig->classData.npixels(); i++) {
			 R[b1][b2] += (Sig->classData.x[i][b1])*(Sig->classData.x[i][b2])*(Sig->classData.w[i]);
		   }
		   R[b1][b2] /= Sig->classData.SummedWeights;
		   R[b1][b2] -= mean[b1]*mean[b2];
		 }
	 }
	 else if(option==CLUSTER_DIAG)
	 {
		 // non-diagonal elements are assumed to be zero.
		 for(b1=0; b1<nbands(); b1++)
		{
		   R[b1][b1] = 0.0;
		   for(i=0; i<Sig->classData.npixels(); i++) {
			 R[b1][b1] += (Sig->classData.x[i][b1])*(Sig->classData.x[i][b1])*(Sig->classData.w[i]);
		   }
		   R[b1][b1] /= Sig->classData.SummedWeights;
		   R[b1][b1] -= mean[b1]*mean[b1];
		 }
	 }
	 else
	 {
		 m_real var=0;
		// non-diagonal elements are assumed to be zero.
		 for(i=0; i<Sig->classData.npixels(); i++)
		 {
			 var+=m::ss(Sig->classData.x.row(i), mean)*Sig->classData.w[i];
		 }
		 var/=Sig->classData.SummedWeights;

		 R[0][0]=var;
	 }

     /* Compute the sampling period for seeding */
     if(Sig->nsubclasses()>1) {
       period = (Sig->classData.npixels()-1)/(Sig->nsubclasses()-1.0);
     }
     else period =0;


     /* Seed the means and set the covarience components */
     for(i=0; i<Sig->nsubclasses(); i++) {
       for(b1=0; b1<nbands(); b1++) {
         Sig->mSubSig[i].means[b1] = Sig->classData.x[(int)(i*period)][b1];
       }

	   Sig->mSubSig[i].setCov(R, Rmin);

       Sig->mSubSig[i].pi = 1.0/Sig->nsubclasses();
     }


     compute_constants();

	normalize_pi();


}


void Clust::ClassSig::normalize_pi()
{
	struct ClassSig *Sig=this;
	int i;
	double sum;

	sum = 0.0;
	for(i=0; i<Sig->nsubclasses(); i++) sum += Sig->mSubSig[i].pi;

	if(sum>0) {
	  for(i=0; i<Sig->nsubclasses(); i++) Sig->mSubSig[i].pi /= sum;
	}
	else {
	  for(i=0; i<Sig->nsubclasses(); i++) Sig->mSubSig[i].pi = 0.0;
	}
}
/**********************************************************/
/* invert matrix and compute Sig->mSubSig[i].cnst          */
/**********************************************************/
void Clust::ClassSig::compute_constants()
{
	ClassSig *Sig=this;

   /* invert matrix and compute constant for each subclass */
   for(int i=0; i<Sig->nsubclasses(); i++)
	   Sig->mSubSig[i].compute_constants();
}


Clust::ClassSig::SubSigs& Clust::ClassSig::SubSigs::operator=(const ClassSig::SubSigs& other)
{
	nbands=other.nbands;
	while(size()<other.size())
		insertNewSubSig();

	resize(other.size());

    for(int i=0; i<other.size(); i++)
      data(i)=other[i];
	return *this;
}


void Clust::ClassSig::reduce_order(int *min_ii,  int *min_jj)
{
	ClassSig *Sig=this;
    int i,j;
    int min_i,min_j;
    double dist;
    double min_dist;

	Clust::SubSig& SubSig3=*mpSubSig3;
	SubSig3.init(nbands());

    if(Sig->nsubclasses()>1)
    {
      /* find the closest subclasses */
      for(i=0; i<Sig->nsubclasses()-1; i++)
      for(j=i+1; j<Sig->nsubclasses(); j++)
      {
		  dist=Sig->mSubSig[i].distance(Sig->mSubSig[j]);
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
      SubSig &SubSig1= (Sig->mSubSig[min_i]);
      SubSig &SubSig2 =(Sig->mSubSig[min_j]);
      SubSig3.add(SubSig1,SubSig2);
      SubSig1=SubSig3;

      /* remove extra subclass */
	  Sig->mSubSig.remove(min_j, min_j+1);

      /* Rerun compute_constants */
      compute_constants();
      normalize_pi();
    }
}

void Clust::ClassSig::SubSigs::remove(int start, int end)
{
	int numCols=end-start;

	// 포인터 바꿔치기 한다.
	for(int i=end; i<size(); i++)
	{
		swap(i, i-end+start);
	}

	int newSize=size()-numCols;

	mSize=newSize;
}

void Clust::ClassSig::SubSigs::resize(int nsize)
{
	// 늘리기만 한다.
	if(TArray<SubSig>::size()<nsize)
		TArray<SubSig>::resize(nsize);
	mSize=nsize;
}


int Clust::ClassSig::subcluster(
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
	int nbands=mSubSig.nbands;
    double rissanen;
    double min_riss;
	Clust::ClassSig *Sig;
	Clust::ClassSig::SubSigs & Smin=*mpSmin;

    status = 0;

    /* set class pointer */
    Sig =this;

    /* compute number of parameters per cluster */
    nparams_clust = 1+nbands+0.5*(nbands+1)*nbands;
    if(option==CLUSTER_DIAG) nparams_clust = 1+nbands+nbands;
	else if(option==CLUSTER_SPHR) nparams_clust= 1+nbands+1;

    /* compute number of data points */
    ndata_points = Sig->classData.npixels()*nbands;

    /* compute maximum number of subclasses */
    ndata_points = Sig->classData.npixels()*nbands;
    *Max_num = (ndata_points+1)/nparams_clust - 1;

    /* check for too many subclasses */
    if(Sig->nsubclasses()> (*Max_num/2) )
    {
		Sig->mSubSig.resize(*Max_num/2);
      fprintf(stderr,"Too many subclasses \n");
      fprintf(stderr,"         number of subclasses set to %d\n\n",Sig->nsubclasses());
      status = -2;
    }


    /* initialize clustering */
    seed(Rmin,option);



    /* EM algorithm */
    min_riss = refine_clusters(Rmin,option);


    if(2<=clusterMessageVerboseLevel) {
      fprintf(stdout,"Subclasses = %d; Rissanen = %f; \n",Sig->nsubclasses(),min_riss);
    }

    /* Save contents of Class Signature to Smin */
	Smin=Sig->mSubSig;

    if(desired_num==0) {
      while(Sig->nsubclasses()>1) {
        reduce_order(&min_i,&min_j);

        if(2<=clusterMessageVerboseLevel) {
          fprintf(stdout,"Combining Subclasses (%d,%d)\n",min_i,min_j);
        }

        rissanen = refine_clusters(Rmin,option);

        if(2<=clusterMessageVerboseLevel) {
          fprintf(stdout,"Subclasses = %d; Rissanen = %f; \n",
                       Sig->nsubclasses(), rissanen);
        }

        if(rissanen<min_riss)
        {
          min_riss = rissanen;

          /* Delete old Smin, and save new Smin */
		  Smin=Sig->mSubSig;
        }
      }
    }
    else {
      while( (Sig->nsubclasses()>desired_num)&&(Sig->nsubclasses()>0) ) {
        reduce_order(&min_i,&min_j);

        if(2<=clusterMessageVerboseLevel) {
          fprintf(stdout,"Combining Subclasses (%d,%d)\n",min_i,min_j);
        }

        rissanen = refine_clusters(Rmin,option);

        if(2<=clusterMessageVerboseLevel) {
          fprintf(stdout,"Subclasses = %d; Rissanen = %f; \n",
                       Sig->nsubclasses(),rissanen);
        }

        /* Delete old Smin, and save new Smin */
		Smin=Sig->mSubSig;
      }
    }

    /* Deallocate memory for class, and replace with solution */
	Sig->mSubSig=Smin;

    /* return warning status */
    return(status);
}

m_real Clust::ClassSig::logLikelihood(vectorn const& sample) const
{
	const ClassSig *Sig=this;
	int s;
	int i;
	double tmp;
	double maxlike;
	double likelihood;
	double subsum;

	/* compute likelihoods */
   likelihood = 0;

	static vectorn temp;
	temp.setSize(Sig->nsubclasses());
	for(i=0; i<Sig->nsubclasses(); i++)
	{
		tmp = Sig->mSubSig[i].loglike(sample);
		temp[i] = tmp;
		if(i==0) maxlike = tmp;
		if(tmp>maxlike) maxlike = tmp;
	}

	subsum = 0;
	for(i=0; i<Sig->nsubclasses(); i++)
	{
		tmp = exp( temp[i]-maxlike )*Sig->mSubSig[i].pi;
		subsum += tmp;
	}
	likelihood += log(subsum) + maxlike;

   return(likelihood);
}

void Clust::ClassSig::logLikelihoods(vectorn const&sample, vectorn& loglikelihoods) const
{
	const ClassSig *Sig=this;
	int s;
	int i;
	double tmp;
	double maxlike;

	/* compute likelihoods */

	static vectorn temp;
	temp.setSize(Sig->nsubclasses());
	loglikelihoods.setSize(Sig->nsubclasses());

	for(i=0; i<Sig->nsubclasses(); i++)
	{
		tmp = Sig->mSubSig[i].loglike(sample);
		temp[i] = tmp;
		if(i==0) maxlike = tmp;
		if(tmp>maxlike) maxlike = tmp;
	}

	for(i=0; i<Sig->nsubclasses(); i++)
	{
		tmp = exp( temp[i]-maxlike )*Sig->mSubSig[i].pi;
		loglikelihoods[i]=log(tmp)+maxlike;
	}
}

double Clust::ClassSig::regroup()
{
	struct ClassSig *Sig=this;
   int s;
   int i;
   double tmp;
   double maxlike;
   double likelihood;
   double subsum;
   struct Clust::ClassData *Data;

   /* set data pointer */
   Data = &(Sig->classData);

   /* compute likelihoods */
   likelihood = 0;
   for(s=0; s<Data->npixels(); s++)
   {

     for(i=0; i<Sig->nsubclasses(); i++)
     {
       tmp = Sig->mSubSig[i].loglike(Data->x.row(s));
       Data->p[s][i] = tmp;
       if(i==0) maxlike = tmp;
       if(tmp>maxlike) maxlike = tmp;
     }

     subsum = 0;
     for(i=0; i<Sig->nsubclasses(); i++)
     {
       tmp = exp( Data->p[s][i]-maxlike )*Sig->mSubSig[i].pi;
       subsum += tmp;
       Data->p[s][i] = tmp;
     }
     likelihood += log(subsum) + maxlike;

     for(i=0; i<Sig->nsubclasses(); i++)
       Data->p[s][i] /= subsum;
   }

   return(likelihood);
}


void Clust::ClassSig::writeToFile(FILE *fd)
{
  ClassSig *Cp=this;
  struct SubSig *Sp;
  int j;

  fprintf (fd, "title: %s\n",title.ptr());
  fprintf (fd, "nbands: %d\n", nbands());

  {
    if (!Cp->used) return ;
    fprintf (fd, "class:\n");
    fprintf (fd, " classnum: %ld\n", 0);
    fprintf (fd, " classtitle: %s\n",title.ptr());
    fprintf (fd, " classtype: %d\n", Cp->type);

    for (j = 0; j < Cp->nsubclasses(); j++) {
      Sp = &Cp->mSubSig[j];
	  Sp->writeToFile(fd);
    }
    fprintf (fd, "endclass:\n");
  }
}


void Clust::ClassSig::reestimate(double Rmin, int option)
{
	ClassSig *Sig=this;
     int i;
     int s;
     int b1,b2;
     struct ClassData *Data;

     /* set data pointer */
     Data = &(Sig->classData);

     /* Compute N */
     for(i=0; i<Sig->nsubclasses(); i++)
     {
       Sig->mSubSig[i].N = 0;
       for(s=0; s<Data->npixels(); s++)
         Sig->mSubSig[i].N += (Data->p[s][i])*(Data->w[s]);
       Sig->mSubSig[i].pi = Sig->mSubSig[i].N;
     }

     /* Compute means and variances for each subcluster */
     for(i=0; i<Sig->nsubclasses(); i++)
     {
       /* Compute mean */
       for(b1=0; b1<nbands(); b1++)
       {
         Sig->mSubSig[i].means[b1] = 0;
         for(s=0; s<Data->npixels(); s++)
           Sig->mSubSig[i].means[b1] += Data->p[s][i]*Data->x[s][b1]*Data->w[s];
         Sig->mSubSig[i].means[b1] /= Sig->mSubSig[i].N;
       }

       /* Compute R */
	   Sig->mSubSig[i].estimateCov(Data, i, Rmin);
     }

     /* Normalize probabilities for subclusters */
     normalize_pi();

     /* Compute constants */
     compute_constants();
     normalize_pi();
}

double Clust::ClassSig::refine_clusters(
    double Rmin,
    int option)
{
    ClassSig *Sig=this;

     int nparams_clust;
     int num_params;
     int ndata_points;
     int repeat;
     double rissanen_const;
     double change,ll_new,ll_old;
     double epsilon;

     /* compute number of parameters per cluster */
     nparams_clust = 1+nbands()+0.5*(nbands()+1)*nbands();
     if(option==CLUSTER_DIAG) nparams_clust = 1+nbands()+nbands();

     /* compute number of data points */
     ndata_points = Sig->classData.npixels()*nbands();

     /* compute epsilon */
     epsilon = nparams_clust*log((double)ndata_points);
     epsilon *= 0.01;

     /* Perform initial regrouping */
     ll_new = regroup();

     /* Perform EM algorithm */
     change = 2*epsilon;
     do {
       ll_old = ll_new;
       reestimate(Rmin,option);

       ll_new = regroup();
       change = ll_new-ll_old;
       repeat = change>epsilon;
     } while(repeat);

     /* compute Rissanens expression */
     if(Sig->nsubclasses()>0) {
       num_params = Sig->nsubclasses()*nparams_clust - 1;
       rissanen_const = -ll_new + 0.5*num_params*log((double)ndata_points);
       return(rissanen_const);
     }
     else {
       return((double)0);
     }
}




double AverageVariance(struct ClassSig *Sig, int nbands)
{
     int     i,b1;
     double  *mean,**R,Rmin;

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

     for(b1=0; b1<nbands; b1++) {
       R[b1][b1] = 0.0;
       for(i=0; i<Sig->classData.npixels; i++) {
         R[b1][b1] += (Sig->classData.x[i][b1])*(Sig->classData.x[i][b1])*(Sig->classData.w[i]);
       }
       R[b1][b1] /= Sig->classData.SummedWeights;
       R[b1][b1] -= mean[b1]*mean[b1];
     }

     /* Compute average of diagonal entries */
     Rmin = 0.0;
     for(b1=0; b1<nbands; b1++)
       Rmin += R[b1][b1];

     Rmin = Rmin/(nbands);

     G_free_vector(mean);
     G_free_matrix(R);

     return(Rmin);
}

